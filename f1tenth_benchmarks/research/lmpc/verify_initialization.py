"""Independently reconstruct B1 events, observations and physical replay."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from f1tenth_benchmarks.research.core.geometry import load_geometry
from .lap_adapter import CoreLapTracker
from .state import paper_state
from .validation import FootprintChecker
from .run_smoke import replay
from .initialization import ControlledExcitation


def verify_initialization(directory):
    root = Path(directory)
    report = json.loads((root/'session.json').read_text())
    if not report['passed'] or report['initialization_laps_accepted'] != 2:
        raise ValueError('not an accepted B1 session')
    for name, digest in report['input_hashes'].items():
        path = Path(name)
        if not path.exists():
            path = Path(name.removeprefix('/workspace/'))
        if hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise ValueError('input hash mismatch: '+name)
    rows = [json.loads(line) for line in (root/'samples.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
    if len(rows) != report['sample_count']:
        raise ValueError('sample count mismatch')
    config, physics = report['config'], report['simulator_params']
    reference_speed=config.get('experiment',{}).get('initialization_speed_mps',config['paper']['initialization_speed'])
    settings = config['initialization']
    excitation_settings=config.get('experiment',{}).get('excitation')
    excitation=ControlledExcitation(excitation_settings) if excitation_settings else None
    geometry = load_geometry(config['adaptation']['geometry_path'], config['adaptation']['map_yaml'], inspect=True)
    if geometry.geometry_id != report['geometry_id']:
        raise ValueError('geometry mismatch')
    sampled = np.loadtxt(Path(config['adaptation']['geometry_path'])/'centerline.csv', delimiter=',', skiprows=1)
    tracker = CoreLapTracker(geometry, sampled[:, :2])
    checker = FootprintChecker(geometry.raster, physics['vehicle_length'],
                               physics['vehicle_width'], settings['footprint_margin_m'])
    replay_result = replay(rows, physics, report['start_pose'])
    reconstructed = []
    footprint_checks = 0
    for i, row in enumerate(rows):
        physical = np.asarray(row['physical_state'])
        expected_mode='acceleration' if report.get('acceleration_interface') and i else 'target_speed'
        if row['reset_count'] != 1 or row['input_mode'] != expected_mode:
            raise ValueError('reset or input semantics changed')
        if len(row['physics_ticks']) != physics['n_sim_steps']:
            raise ValueError('incorrect physics substep count')
        if i:
            if excitation is not None:
                diagnostic=row['excitation_diagnostic']
                previous=rows[i-1]
                baseline=[float(np.clip(row['controller_diagnostic']['unconstrained_steering'],
                                       -report['vehicle_params']['max_steer'],report['vehicle_params']['max_steer'])),
                          float(np.clip((reference_speed-previous['physical_state'][3])/config['paper']['control_dt'],
                                        -physics['a_max'],physics['a_max']))]
                expected=excitation.apply(baseline,report['vehicle_params']['max_steer'],physics['a_max'])
                np.testing.assert_allclose(expected,row['command'],rtol=0,atol=1e-12)
                np.testing.assert_allclose(diagnostic['offset'],excitation.offset,rtol=0,atol=1e-12)
            np.testing.assert_allclose(row['time']-rows[i-1]['time'], config['paper']['control_dt'], rtol=0, atol=1e-10)
            np.testing.assert_array_equal(row['physics_ticks'][0]['pre_state'], rows[i-1]['physical_state'])
            speed_reference=row.get('target_speed_reference') if report.get('acceleration_interface') else row['command'][1]
            if speed_reference != reference_speed:
                raise ValueError('initialization target speed changed')
            if abs(row['command'][0]) > report['vehicle_params']['max_steer']:
                raise ValueError('steering command limit exceeded')
        for j, tick in enumerate(row['physics_ticks']):
            if tick['dt'] != physics['timestep'] or tick['input_mode'] != expected_mode:
                raise ValueError('incorrect physics timestep or input mode')
            np.testing.assert_array_equal(tick['command'], row['command'])
            if j:
                np.testing.assert_array_equal(tick['pre_state'], row['physics_ticks'][j-1]['post_state'])
            pre, post = np.asarray(tick['pre_state']), np.asarray(tick['post_state'])
            np.testing.assert_allclose((post[[3, 2]]-pre[[3, 2]])/tick['dt'],
                                      [tick['applied_acceleration'], tick['applied_steering_rate']],
                                      rtol=0, atol=1e-10)
            if not physics['s_min']-1e-12 <= post[2] <= physics['s_max']+1e-12:
                raise ValueError('actual steering limit exceeded')
            if expected_mode == 'acceleration':
                np.testing.assert_allclose(tick['requested_acceleration'],row['command'][1],rtol=0,atol=1e-10)
            for state in (pre, post):
                check = checker.check(state[[0, 1, 4]])
                footprint_checks += 1
                if not check['valid']:
                    raise ValueError('footprint verification failed: '+str(check))
        result = tracker.update(physical[[0, 1, 4]], row['time'], float(physical[3]))
        if not result.valid:
            raise ValueError('Frenet replay rejected: '+str(result.reason))
        reconstructed_state = paper_state(physical, tracker.frenet, row['time'], geometry.geometry_id)
        np.testing.assert_allclose(reconstructed_state, row['paper_state'], rtol=0, atol=1e-9)
        saved_frenet = row['frenet']
        if (not saved_frenet['valid'] or saved_frenet['geometry_id'] != geometry.geometry_id
                or saved_frenet['frame_id'] != 'map' or saved_frenet['timestamp'] != row['time']
                or not saved_frenet['freshness_verified'] or not saved_frenet['version_verified']):
            raise ValueError('saved Frenet metadata mismatch')
        for field in ('s_wrapped', 's_unwrapped', 'd', 'e_psi', 'jacobian'):
            np.testing.assert_allclose(saved_frenet[field], tracker.frenet[field], rtol=0, atol=1e-9)
        if (abs(reconstructed_state[5]) > settings['max_abs_e_y_m']
                or abs(reconstructed_state[3]) > settings['max_abs_e_psi_rad']):
            raise ValueError('tracking tolerance exceeded')
        if row['elapsed_time'] != row['time']-rows[0]['time']:
            raise ValueError('elapsed timestamp mismatch')
        if row['elapsed_time'] >= settings['settled_after_s'] and abs(
                physical[3]-reference_speed) > settings['max_settled_speed_error_mps']:
            raise ValueError('settled speed tolerance exceeded')
        if row['completed_full_laps'] != tracker.completed_full_laps:
            raise ValueError('lap count mismatch')
        if result.event:
            event = result.event
            reconstructed.append(event)
            saved = events[len(reconstructed)-1]
            if saved['row_index'] != i or saved['kind'] != event.kind:
                raise ValueError('event association mismatch')
            for field in ('finish_s', 'alpha', 'duration', 'crossing_time_estimate', 'lap_start_time'):
                np.testing.assert_allclose(saved[field], getattr(event, field), rtol=0, atol=1e-10)
    if len(reconstructed) != len(events) or tracker.completed_full_laps != report.get('collection_laps', 2):
        raise ValueError('two-lap reconstruction mismatch')
    if events[-1]['row_index'] != len(rows)-1:
        raise ValueError('session did not stop on final crossing step')
    return dict(passed=True, physical_replay=replay_result, independent_frenet_and_events=True,
                verifier_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                sampled_footprint_checks=footprint_checks, sample_count=len(rows),
                lap_times=[event.duration for event in reconstructed], planning_allowed=False,
                continuous_verified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    parser.add_argument('--output')
    args = parser.parse_args()
    report = verify_initialization(args.directory)
    if args.output:
        with Path(args.output).open('x') as stream:
            json.dump(report, stream, indent=2)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
