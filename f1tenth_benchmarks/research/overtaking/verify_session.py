"""Independent physical replay and recorded-plan time/source checks."""
import argparse
from argparse import Namespace
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from ..core.geometry import load_geometry
from ..core.frenet_lap_tracking import CoreLapTracker
from ..core.centreline_control import CentrelinePurePursuit
from .session import latest_plan


def verify_session(directory, require_complete=True):
    root = Path(directory)
    metadata = json.loads((root/'metadata.json').read_text())
    summary = json.loads((root/'summary.json').read_text())
    index = json.loads((root/'playback_index.json').read_text())
    streams = {}
    hashes = {}
    for name in ('states', 'actions', 'plans'):
        raw = (root/(name+'.jsonl')).read_bytes()
        hashes[name+'.jsonl'] = hashlib.sha256(raw).hexdigest()
        records = [json.loads(line) for line in raw.splitlines()]
        assert len(records) == len(index['streams'][name])
        for row, item in zip(records, index['streams'][name]):
            assert json.loads(raw[item['byte_offset']:item['byte_offset']+item['byte_length']]) == row
        streams[name] = records
    states, actions, plans = (streams[n] for n in ('states', 'actions', 'plans'))
    assert states and plans
    assert [s['control_step'] for s in states] == list(range(len(states)))
    assert len(actions) == len(states)-1
    assert len(states) == summary['state_count'] and len(plans) == summary['plan_count']
    assert len(actions) == summary['action_count'] == summary['control_steps']
    assert metadata['playback_origin_s'] == states[0]['sim_time_s']
    assert not metadata['candidate_control_enabled'] and not metadata['planning_allowed']
    physics = metadata['simulator_params']
    dynamics = DynamicsSimulator(Namespace(**physics))
    dynamics.reset(metadata['initial_pose'])
    dynamics.state[3] = metadata['initial_speed_mps']
    assert metadata['map'] == 'esp'
    geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
    assert geometry.geometry_id == metadata['geometry_id']
    assert geometry.raster.source_hashes == metadata['track']['source_hashes']
    sampled = np.loadtxt('maps/frenet/esp/geometry/centerline.csv', delimiter=',', skiprows=1)
    tracker = CoreLapTracker(geometry, sampled[:, :2])
    controller = CentrelinePurePursuit(sampled[:, :2], metadata['config']['speed_mps'],
        metadata['config']['pp_lookahead_m'], metadata['vehicle_params']['wheelbase'],
        metadata['vehicle_params']['max_steer'])
    for i, state in enumerate(states):
        assert state['reset_count'] == 1
        assert state['pose'] == [state['physical_state'][j] for j in (0, 1, 4)]
        assert state['speed_mps'] == state['physical_state'][3]
        np.testing.assert_allclose(dynamics.state, state['pre_step_state'], rtol=0, atol=1e-12)
        np.testing.assert_array_equal(dynamics.steer_buffer, state['pre_step_steering_buffer'])
        action = metadata['initialization_action'] if i == 0 else actions[i-1]['action']
        if i:
            preceding = actions[i-1]
            assert preceding['source_control_step'] == i-1 and preceding['target_control_step'] == i
            assert preceding['source_sim_time_s'] == states[i-1]['sim_time_s']
            assert preceding['control_source'] == 'centreline_pure_pursuit'
            expected_action = controller.plan(dict(pose=np.asarray(states[i-1]['pose'])))
            np.testing.assert_allclose(action, expected_action, rtol=0, atol=1e-12)
            np.testing.assert_allclose(state['sim_time_s']-states[i-1]['sim_time_s'],
                                       physics['timestep']*physics['n_sim_steps'], rtol=0, atol=1e-10)
        for _ in range(physics['n_sim_steps']):
            dynamics.update_pose(*action)
        np.testing.assert_allclose(dynamics.state, state['physical_state'], rtol=0, atol=1e-12)
        np.testing.assert_array_equal(dynamics.steer_buffer, state['steering_buffer'])
        if state['valid']:
            rebuilt = tracker.update(np.asarray(state['pose']), state['sim_time_s'], state['speed_mps'])
            assert rebuilt.valid
            for key in ('s_wrapped', 's_unwrapped', 'd', 'e_psi'):
                np.testing.assert_allclose(state['frenet'][key], tracker.frenet[key], rtol=0, atol=1e-9)
            assert state['completed_full_laps'] == tracker.completed_full_laps

    seam_steps = []
    previous = None
    for state in states:
        if state['valid']:
            f = state['frenet']
            assert f['valid'] and f['geometry_id'] == metadata['geometry_id']
            assert f['timestamp'] == state['sim_time_s']
            if previous is not None:
                df = f['s_unwrapped']-previous['frenet']['s_unwrapped']
                # This accepted session drives forward; do not manufacture
                # monotonic progress in localization or in playback.
                assert df >= -1e-8
                # A tiny negative initial projection may wrap from L to zero
                # on the first moving sample; it is not a completed lap.
                if math.floor(f['s_unwrapped']/geometry.curve.L) > max(
                        0, math.floor(previous['frenet']['s_unwrapped']/geometry.curve.L)):
                    assert f['s_wrapped'] < previous['frenet']['s_wrapped']-1.
                    seam_steps.append(state['control_step'])
            previous = state
    for i, plan in enumerate(plans):
        assert plan['plan_id'] == i
        source = states[plan['source_control_step']]
        assert plan['source_pose'] == source['pose']
        assert plan['source_sim_time_s'] == source['sim_time_s']
        assert plan['available_control_step'] == source['control_step']
        assert plan['available_sim_time_s'] == source['sim_time_s']
        assert plan['geometry_id'] == metadata['geometry_id']
        assert not plan['planning_allowed'] and not plan['footprint_verified']
        assert [p['candidate_id'] for p in plan['paths']] == [1, 2, 3, 4, 5]
        assert plan['sampled_valid_count'] == sum(p['sampled_valid'] for p in plan['paths'])
        if not source['valid']:
            assert plan['status'] == 'skipped'
        for p in plan['paths']:
            if not p['sampled_valid']:
                assert p['reason']
                continue
            assert source['valid'] and p['points'] and p['geometry_id'] == metadata['geometry_id']
            a = p['points'][0]
            error = math.hypot(a['x']-source['pose'][0], a['y']-source['pose'][1])
            yaw_error = (a['yaw']-source['pose'][2]+math.pi) % (2*math.pi)-math.pi
            assert error <= metadata['config']['max_start_position_error_m']
            assert abs(yaw_error) <= metadata['config']['max_start_heading_error_rad']
            np.testing.assert_allclose([error, yaw_error],
                [p['start_position_error_m'], p['start_heading_error_rad']], rtol=0, atol=1e-12)
    if math.isclose(metadata['config']['plan_period_s'], metadata['config']['control_dt_s'], abs_tol=1e-12):
        assert len(plans) == len(states)
        assert [p['source_control_step'] for p in plans] == list(range(len(states)))
    # Check every cursor in reverse order as well as before the first plan.
    assert latest_plan(plans, -1) is None
    for state in reversed(states):
        selected = latest_plan(plans, state['control_step'])
        assert selected is not None
        assert selected['source_control_step'] <= state['control_step']
        expected = max(p['plan_id'] for p in plans if p['source_control_step'] <= state['control_step'])
        assert selected['plan_id'] == expected
    if require_complete:
        assert summary['recording_complete'] and summary['end_reason'] == 'requested_laps_reached'
        assert summary['error'] is None and not summary['collision']
        assert summary['completed_full_laps'] == metadata['config']['laps']
        assert len(seam_steps) == metadata['config']['laps']
        assert plans[-1]['source_control_step'] == states[-1]['control_step']
        assert plans[-1]['trigger'] == 'terminal'
    return dict(schema_version=1, passed=True, session=str(root),
        geometry_id=metadata['geometry_id'], program=metadata['program'], config=metadata['config'],
        checks=dict(physical_replay=True, byte_index=True, source_pose_and_time=True,
                    controller_commands=True, independent_frenet_and_laps=True,
                    five_candidate_ids=True, start_pose_gate=True, backward_lookup_no_future=True),
        source_sha256=hashes, summary=summary, seam_control_steps=seam_steps,
        candidate_control_enabled=False, footprint_verified=False, planning_allowed=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    parser.add_argument('--report')
    parser.add_argument('--allow-incomplete', action='store_true')
    args = parser.parse_args()
    report = verify_session(args.directory, not args.allow_incomplete)
    if args.report:
        with Path(args.report).open('x') as stream:
            json.dump(report, stream, indent=2, allow_nan=False)
    print(json.dumps(dict(passed=report['passed'], states=report['summary']['state_count'],
                          plans=report['summary']['plan_count'], seam_control_steps=report['seam_control_steps'])))


if __name__ == '__main__':
    main()
