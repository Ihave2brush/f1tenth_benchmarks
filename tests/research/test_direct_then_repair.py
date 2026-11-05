"""Direct Hermite preserves pose/limits; only failed IDs enter legacy repair."""
import math

import numpy as np
import pytest

from test_robust_adaptive_paths import make_straight
from f1tenth_benchmarks.research.overtaking import robust_adaptive_paths as planner
from f1tenth_benchmarks.research.overtaking.session import SessionConfig


def test_direct_coefficients_match_actual_heading_and_end_position():
    length, d0, slope, target = 8., .12, .15, -.5
    basis = planner._basis(length, 6)
    coefficients = planner.direct_hermite_coefficients(length, d0, slope, target)
    np.testing.assert_allclose([basis(0.)@coefficients, basis(0., nu=1)@coefficients,
                               basis(length)@coefficients, basis(length, nu=1)@coefficients],
                              [d0, slope, target, 0.], rtol=0, atol=1e-12)


def test_all_direct_valid_skips_every_optimizer(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('valid direct candidates must not enter optimizer')
    monkeypatch.setattr(planner, 'fit_multistart', forbidden)
    result = planner.generate_robust_paths(make_straight(), 1.5, 3.1, yaw=.05,
        lookahead_m=4., query_mode='fast_with_fallback', generation_mode='direct_then_repair')
    assert result['sampled_valid_count'] == result['direct_valid_count'] == 5
    assert result['repaired_valid_count'] == 0
    for path in result['paths']:
        assert path['generation_route'] == 'direct' and not path['attempts']
        assert not path['repair_attempted']
        assert path['start_position_error_m'] < 1e-6
        assert abs(path['start_heading_error_rad']) < 1e-6
        assert path['polyline_collision_verified']


def test_only_failed_direct_ids_use_legacy_repair(monkeypatch):
    validate = planner._validate
    def injected(geometry, basis, c, dense, sections, k, kp, target, *args):
        points, invalid, segment = validate(geometry, basis, c, dense, sections, k, kp, target, *args)
        if len(c) == 6 and target > .1:
            points[5].update(valid=False, reason='injected_direct_failure')
            return points, 5, segment
        return points, invalid, segment
    monkeypatch.setattr(planner, '_validate', injected)
    result = planner.generate_robust_paths(make_straight(), 1.5, 3., lookahead_m=3.,
        query_mode='fast_with_fallback', generation_mode='direct_then_repair')
    assert result['direct_valid_count'] == 3
    assert result['repaired_valid_count'] == 2
    for path in result['paths']:
        if path['candidate_id'] <= 3:
            assert not path['attempts']
        else:
            assert path['repair_attempted'] and path['attempts']
            assert path['direct_attempt']['reason'] == 'injected_direct_failure'
            assert path['spline']['degree'] == 5


def test_direct_only_does_not_relax_limits_or_call_repair(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('direct_only cannot call repair')
    monkeypatch.setattr(planner, 'fit_multistart', forbidden)
    result = planner.generate_robust_paths(make_straight(), 1.5, 3., lookahead_m=3.,
        max_curvature=.01, generation_mode='direct_only', query_mode='fast_with_fallback')
    assert result['sampled_valid_count'] < 5
    for path in result['paths']:
        assert not path['attempts']
        if path['sampled_valid']:
            assert all(abs(point['curvature_inv_m']) <= .01+1e-7 for point in path['points'])
        else:
            assert path['reason']


def test_dense_screen_rejects_without_running_expensive_validator(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('known slope/curvature failures do not need geometry validation')
    monkeypatch.setattr(planner, '_validate', forbidden)
    result = planner.generate_robust_paths(make_straight(), 1.5, 3.123, lookahead_m=3.,
        max_slope=1., max_curvature=1e-8, generation_mode='direct_only',
        query_mode='fast_with_fallback', yaw=.01)
    assert result['sampled_valid_count'] == 0
    assert all(p['direct_attempt']['stage'] == 'dense_constraint_screen' for p in result['paths'])


def test_grid_snapshot_reuse_and_close_preserve_shared_table():
    from f1tenth_benchmarks.research.overtaking.geometry_queries import FastQueryContext
    from f1tenth_benchmarks.research.overtaking.road_table import get_road_table
    geometry = make_straight()
    context = FastQueryContext(geometry)
    first = context.boundaries_many([1., 2.])
    assert context.boundaries_many([1., 2.]) is first
    table = context.table
    context.close()
    assert context._candidate_grid is None
    assert get_road_table(geometry) is table
    assert 'candidate_cartesian_many' not in context.geometry.__dict__


def test_late_geometric_results_are_not_counted_as_timely(monkeypatch):
    from types import SimpleNamespace
    from f1tenth_benchmarks.research.overtaking import session
    times = iter([0., .2])
    monkeypatch.setattr(session, 'perf_counter', lambda: next(times))
    def fake(geometry, x, y, yaw, **settings):
        return dict(paths=[dict(candidate_id=i, sampled_valid=True, reason=None,
            status='sampled_valid', points=[dict(x=x, y=y, yaw=yaw)]) for i in range(1, 6)])
    state = dict(control_step=0, sim_time_s=0., valid=True, pose=[0., 0., 0.],
                 frenet=dict(valid=True))
    event = session.make_plan(SimpleNamespace(geometry_id='test'), state,
        SessionConfig(generation_mode='direct_then_repair'), 0, generator=fake)
    assert event['sampled_valid_count'] == 5
    assert not event['cycle_budget_observation']['completed_within_target']
    assert event['cycle_budget_observation']['timely_valid_count'] == 0
    assert not event['cycle_budget_observation']['deadline_enforced']


@pytest.mark.parametrize('kwargs', [dict(generation_mode='bad'), dict(generation_mode=None)])
def test_invalid_mode(kwargs):
    with pytest.raises(ValueError):
        planner.generate_robust_paths(make_straight(), 1.5, 3., lookahead_m=3., **kwargs)
    with pytest.raises(ValueError):
        SessionConfig(**kwargs)
