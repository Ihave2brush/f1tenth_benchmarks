"""The published candidate set must reflect body checks, not centre flags."""
import pytest

from test_robust_adaptive_paths import make_straight
from f1tenth_benchmarks.research.overtaking.session import SessionConfig, make_plan


def plan(geometry, mutate=None, **settings):
    def generator(*args, **kwargs):
        paths = [dict(candidate_id=i, sampled_valid=True, experimental_valid=True,
                      status='sampled_valid', reason=None, points=[dict(x=1.5, y=3., yaw=0.)],
                      spline=dict(knots=[0.]*4+[3.]*4, coefficients=[0.]*4, degree=3))
                 for i in range(1, 6)]
        if mutate:
            mutate(paths)
        return dict(start=dict(s=.5), paths=paths, valid_ids=[1,2,3,4,5],
                    fallback_candidate_id=None, requires_stop=False)
    state = dict(control_step=0, sim_time_s=0., pose=[1.5,3.,0.], valid=True, frenet=dict(valid=True))
    return make_plan(geometry, state, SessionConfig(**settings), 0, generator=generator)


def test_clear_curve_gets_sampled_body_result():
    event = plan(make_straight())
    assert event['sampled_valid_count'] == 5
    for path in event['paths']:
        assert path['footprint_sampled_valid']
        assert path['footprint_check']['sample_count'] == 301
        assert not path['footprint_verified']  # no continuous claim
    assert not event['planning_allowed']


@pytest.mark.parametrize('occupancy', [1, -1])
def test_wall_or_unknown_beside_centre_rejects_all_published_candidates(occupancy):
    geometry = make_straight()
    # Cell x=[3,3.1], y=[3.2,3.3]: centreline is free, inflated body hits.
    geometry.raster.occupancy[27, 30] = occupancy
    event = plan(geometry)
    assert event['sampled_valid_count'] == 0
    assert event['result']['valid_ids'] == []
    assert event['result']['requires_stop']
    for path in event['paths']:
        assert path['reason'] == 'three_disc_clearance'
        assert not path['experimental_valid']
        assert path['footprint_check']['first_invalid_sample']['distance_s_m'] > 0
    assert plan(geometry, footprint_clearance_m=0.)['sampled_valid_count'] == 5


def test_current_source_pose_must_also_be_safe():
    geometry = make_straight()
    geometry.raster.occupancy[29, 15] = 1
    event = plan(geometry)
    assert event['sampled_valid_count'] == 0
    assert all(p['reason'] == 'source_three_disc_clearance' for p in event['paths'])


def test_missing_curve_cannot_pass_using_only_stored_start_point():
    event = plan(make_straight(), lambda paths: paths[0].pop('spline'))
    assert event['sampled_valid_count'] == 4
    assert event['paths'][0]['reason'] == 'footprint_curve_unavailable'


def test_repaired_curve_is_checked_instead_of_saved_sparse_points():
    def repaired(paths):
        paths[0]['generation_route'] = 'budgeted_bspline_repair'
        paths[0]['spline']['coefficients'] = [0., 0., 2., 2.]
    event = plan(make_straight(), repaired)
    assert event['sampled_valid_count'] == 4
    assert event['paths'][0]['reason'] == 'three_disc_clearance'


@pytest.mark.parametrize('setting', [dict(footprint_step_m=0), dict(footprint_clearance_m=-.1),
                                     dict(footprint_width_m=float('nan'))])
def test_invalid_settings(setting):
    with pytest.raises(ValueError):
        SessionConfig(**setting)
