"""Reference offset is allowed; forged reference labels cannot bypass gates."""
from copy import deepcopy
import numpy as np
import pytest

from test_robust_adaptive_paths import make_straight
from f1tenth_benchmarks.research.overtaking.reference_path import check_reference_path
from f1tenth_benchmarks.research.overtaking.session import SessionConfig, make_plan, reject_paths
from f1tenth_benchmarks.research.run_fixed_grid_benchmark import audit_legacy_grid


def reference_fixture():
    geometry = make_straight()
    source = [1.5, 3.1, .05]
    projection = geometry.to_frenet(source[:2], yaw=source[2])
    origin = projection['s_wrapped']
    reference = geometry.reference(origin)
    path = dict(candidate_id=3, geometry_id=geometry.geometry_id,
        path_type='reference_centerline', sampled_valid=True, status='sampled_valid', reason=None,
        target_d_m=0., reference_d=0., reference_s_origin=origin, requested_target_d=0., endpoint_mode='fixed',
        points=[dict(x=reference['xy'][0], y=reference['xy'][1], yaw=reference['psi'])],
        spline=dict(knots=[0.]*6+[3.]*6, coefficients=[0.]*6, degree=5))
    return geometry, source, projection, path


def event_for(path, geometry, source, projection):
    def generator(*args, **kwargs):
        paths = reject_paths('other_rejected')
        paths[2] = deepcopy(path)
        return dict(paths=paths, start=dict(s=projection['s_wrapped'], d=projection['d'],
            d_prime=(1-projection['kappa']*projection['d'])*np.tan(projection['e_psi'])),
            valid_ids=[3], fallback_candidate_id=3, requires_stop=False)
    state = dict(control_step=0, sim_time_s=0., pose=source, valid=True, frenet=dict(valid=True))
    return make_plan(geometry, state, SessionConfig(safety_margin_m=.15), 0, generator=generator)


def test_reference_offset_passes_reference_gate_not_trajectory_gate():
    geometry, source, projection, path = reference_fixture()
    assert check_reference_path(geometry, path, source, 3.)['valid']
    event = event_for(path, geometry, source, projection)
    assert event['sampled_valid_count'] == 1
    assert event['paths'][2]['start_position_error_m'] > .09
    assert event['paths'][2]['reference_contract_check']['valid']
    assert event['result']['fallback_candidate_id'] == 3
    assert audit_legacy_grid(geometry, event, SessionConfig(safety_margin_m=.15))['passed']


@pytest.mark.parametrize('mutation', ['nonzero_spline', 'wrong_origin', 'fake_first_point', 'wrong_id'])
def test_forged_reference_label_is_rejected(mutation):
    geometry, source, projection, path = reference_fixture()
    if mutation == 'nonzero_spline':
        path['spline']['coefficients'][1] = .01
    elif mutation == 'wrong_origin':
        path['reference_s_origin'] += .2
    elif mutation == 'fake_first_point':
        path['points'][0]['y'] += .01
    else:
        path['candidate_id'] = 2
    assert not check_reference_path(geometry, path, source, 3.)['valid']
    if mutation != 'wrong_id':
        event = event_for(path, geometry, source, projection)
        assert event['sampled_valid_count'] == 0
        assert event['result']['fallback_candidate_id'] is None
        assert event['result']['requires_stop']


def test_state_trajectory_cannot_use_reference_offset_exception():
    geometry, source, projection, path = reference_fixture()
    path['path_type'] = 'state_matched_trajectory'
    event = event_for(path, geometry, source, projection)
    assert event['sampled_valid_count'] == 0
    assert event['paths'][2]['reason'] == 'start_pose_mismatch'


def test_reference_beyond_tracking_envelope_is_not_a_fallback():
    geometry, source, _, path = reference_fixture()
    check = check_reference_path(geometry, path, source, 3., max_lateral_error=.05, max_heading_error=.5)
    assert not check['valid']
    assert check['reason'] == 'reference_tracking_envelope_exceeded'
