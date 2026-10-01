"""Continuous research runner; planner.plan(observation) stays generic."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import numpy as np
from .continuous_logging import ContinuousLapLogger
from .continuous_sim import F1TenthSim_Continuous
from .closed_velocity_profile import protected_hashes


def run_continuous_laps(sim, planner, n_laps, output_dir, start_pose=None,
                        initial_speed=0., save_scans=False, metadata=None):
    if not isinstance(n_laps, int) or isinstance(n_laps, bool) or n_laps <= 0:
        raise ValueError('n_laps must be a positive integer')
    info = dict(map=sim.map_name, requested_full_laps=n_laps,
                planner_name=getattr(planner, 'name', type(planner).__name__),
                simulator_params=vars(sim.params), tracker_config=asdict(sim.tracker.config),
                centreline_length=sim.tracker.L,
                centreline_sha256=hashlib.sha256(sim.tracker.points.tobytes()).hexdigest(),
                finish_origin=sim.tracker.finish_origin, finish_tangent=sim.tracker.finish_tangent,
                timeout=sim.timeout, start_pose=start_pose, initial_speed=initial_speed,
                python_version=platform.python_version(), numpy_version=np.__version__)
    info.update(metadata or {})
    logger = ContinuousLapLogger(output_dir, info, save_scans)
    reason, error = 'error', None
    try:
        obs, done, actual_start = sim.reset(start_pose, initial_speed)
        logger.metadata['actual_start_pose'] = actual_start.tolist()
        logger.record(sim, [0., initial_speed], initializing=True)
        while not done:
            action = planner.plan(obs)
            obs, done = sim.step(action)
            logger.record(sim, action)
            if done:
                break
            if sim.tracker.completed_full_laps >= n_laps:
                reason = 'requested_laps_reached'
                break
        if done:
            reason = sim.terminal_reason
    except BaseException as exc:
        error = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        summary = logger.finalize(reason, sim, error)
    return summary


def run_esp_continuous_pp(n_laps=3, output_dir=None, racetrack_set='mu60_closed',
                          timeout=300., start_pose=None, initial_speed=0., save_scans=False):
    from f1tenth_benchmarks.classic_racing.GlobalPurePursuit import GlobalPurePursuit
    if output_dir is None:
        session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        output_dir = Path('Data/research/continuous_laps/stage5')/session
    before = protected_hashes()
    planner = GlobalPurePursuit('continuous_research', planner_name='ResearchContinuousPP',
                               init_folder=False, extra_params={'racetrack_set': racetrack_set})
    planner.set_map('esp')
    sim = F1TenthSim_Continuous('esp', timeout=timeout)
    summary = run_continuous_laps(sim, planner, n_laps, output_dir, start_pose,
                                  initial_speed, save_scans,
                                  metadata={'racetrack_set': racetrack_set,
                                            'planner_params': vars(planner.planner_params),
                                            'vehicle_params': vars(planner.vehicle_params),
                                            'raceline_sha256': hashlib.sha256(Path(
                                                f'Data/racelines/{racetrack_set}/esp_raceline.csv').read_bytes()).hexdigest()})
    after = protected_hashes()
    changed = [name for name, digest in before.items() if after.get(name) != digest]
    with (Path(output_dir)/'protected_hashes.json').open('x') as f:
        json.dump(dict(before=before, changed=changed, passed=not changed), f, indent=2)
    if changed:
        raise RuntimeError(f'protected files changed: {changed}')
    return summary



def verify_session(directory, require_complete=True):
    """Read saved artifacts and replay physical dynamics from the initial pose.

    Replay never applies reset at finish and checks every physical state/buffer,
    including the actual update after each non-final crossing.
    """
    from argparse import Namespace
    from .continuous_sim import DynamicsSimulator
    root = Path(directory)
    summary = json.loads((root/'session.json').read_text())
    samples = [json.loads(line) for line in (root/'samples.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
    raw = np.load(root/f"SimLog_{summary['metadata']['map']}_session.npy")
    checks = {}
    full = [e for e in events if e['kind'] == 'full']
    checks['full_count'] = len(full) == summary['completed_full_laps'] == summary['full_event_count']
    checks['partial_count'] = len(events)-len(full) == summary['partial_event_count']
    checks['one_reset'] = summary['reset_count'] == 1 and all(r['reset_count'] == 1 for r in samples)
    checks['sample_count'] = len(samples) == summary['sample_count'] == summary['total_steps']
    checks['ten_column_physical_samples'] = raw.shape == (len(samples), 10)
    np.testing.assert_array_equal(raw[:, :7], [r['physical_state'] for r in samples])
    checks['control_steps'] = summary['control_steps']+1 == summary['total_steps']
    if require_complete:
        checks['requested_laps_reached'] = (summary['end_reason'] == 'requested_laps_reached'
            and len(full) == summary['metadata']['requested_full_laps']
            and not summary['collision'] and summary['tracker_invalid_reason'] is None)
    for row_index, row in enumerate(samples):
        if row_index:
            np.testing.assert_array_equal(row['pre_step_state'], samples[row_index-1]['physical_state'])
            np.testing.assert_array_equal(row['pre_step_steering_buffer'], samples[row_index-1]['post_step_steering_buffer'])
        projection = row['tracker_sample']
        if projection:
            assert projection['index'] == row_index
            assert projection['time'] == row['simulation_time']
    checks['physical_and_buffer_continuity'] = True
    for index, e in enumerate(events):
        before, after = e['previous_sample'], e['current_sample']
        ds = after['s_unwrapped']-before['s_unwrapped']
        assert ds > 0 and before['s_unwrapped'] < e['finish_s'] <= after['s_unwrapped']
        alpha = (e['finish_s']-before['s_unwrapped'])/ds
        assert 0 <= alpha <= 1
        np.testing.assert_allclose(e['alpha'], alpha, rtol=0, atol=1e-12)
        np.testing.assert_allclose(e['crossing_time_estimate'],
                                  before['time']+alpha*(after['time']-before['time']), rtol=0, atol=1e-12)
        np.testing.assert_allclose(e['duration'], e['crossing_time_estimate']-e['lap_start_time'], rtol=0, atol=1e-12)
        if index:
            assert e['lap_start_time'] == events[index-1]['crossing_time_estimate']
        j = e['post_step_row_index']
        assert before['index'] == j-1 and after['index'] == j
        np.testing.assert_array_equal(e['post_step_physical_state'], raw[j, :7])
        assert e['post_step_sample_time'] == samples[j]['simulation_time']
    checks['event_interpolation_and_sample_boundaries'] = True
    if summary['end_reason'] == 'requested_laps_reached':
        checks['stop_on_final_crossing_step'] = events[-1]['post_step_row_index'] == len(samples)-1
    dynamics = DynamicsSimulator(Namespace(**summary['metadata']['simulator_params']))
    dynamics.reset(summary['metadata']['actual_start_pose'])
    dynamics.state[3] = summary['metadata']['initial_speed']
    for row in samples:
        np.testing.assert_allclose(dynamics.state, row['pre_step_state'], rtol=0, atol=1e-12)
        np.testing.assert_array_equal(dynamics.steer_buffer, row['pre_step_steering_buffer'])
        action = row['action_for_preceding_interval']
        for _ in range(summary['metadata']['simulator_params']['n_sim_steps']):
            dynamics.update_pose(*action)
        np.testing.assert_allclose(dynamics.state, row['physical_state'], rtol=0, atol=1e-12)
        np.testing.assert_array_equal(dynamics.steer_buffer, row['post_step_steering_buffer'])
    checks['independent_dynamics_replay_without_finish_reset'] = True
    scan_path = root/f"ScanLog_{summary['metadata']['map']}_session.npy"
    if scan_path.exists():
        scans = np.load(scan_path)
        checks['scan_samples'] = (scans.shape == (len(samples), summary['metadata']['simulator_params']['num_beams'])
                                  and bool(np.isfinite(scans).all()))
    report = dict(passed=all(checks.values()), checks=checks,
                  full_lap_times=summary['full_lap_times'], samples=len(samples))
    if not report['passed']:
        raise RuntimeError(f'session verification failed: {report}')
    return report


def run_baseline_regression(output_dir):
    """Run original simulate_laps in a temporary cwd with separate Logs.

    Copy only new regression outputs into the research directory. Original
    params/maps/Data are symlinked read-only by convention and hash checked.
    """
    import csv
    import gc
    import os
    import shutil
    import tempfile
    from f1tenth_benchmarks.classic_racing.GlobalPurePursuit import GlobalPurePursuit
    from f1tenth_benchmarks.simulator.f1tenth_sim import F1TenthSim_TrueLocation
    from f1tenth_benchmarks.run_scripts.run_functions import simulate_laps
    root = Path.cwd()
    output = Path(output_dir).resolve()
    output.mkdir(parents=True, exist_ok=False)
    before = protected_hashes()
    expected = list(csv.DictReader((root/'Logs/ESP_GlobalPP/Results_ESP_GlobalPP.csv').open()))
    expected = next(row for row in expected if row['TestID'] == 'esp_pp_test' and row['Lap'] == '0')
    try:
        with tempfile.TemporaryDirectory(prefix='continuous_baseline_') as tmp:
            tmp_path = Path(tmp)
            for name in ['params', 'maps', 'Data']:
                (tmp_path/name).symlink_to(root/name, target_is_directory=True)
            os.chdir(tmp)
            planner = GlobalPurePursuit('esp_pp_test', planner_name='ESP_GlobalPP')
            planner.set_map('esp')
            sim = F1TenthSim_TrueLocation('esp', planner.name, planner.test_id)
            simulate_laps(sim, planner, 1)
            # Invoke inherited automatic finalization only inside isolated cwd.
            del sim
            gc.collect()
            shutil.copytree(tmp_path/'Logs', output/'Logs')
    finally:
        os.chdir(root)
    actual = list(csv.DictReader((output/'Logs/ESP_GlobalPP/Results_ESP_GlobalPP.csv').open()))[0]
    after = protected_hashes()
    changed = [name for name, digest in before.items() if after.get(name) != digest]
    keys = ['Time', 'Steps', 'Progress', 'LapComplete', 'Collision']
    report = dict(passed=all(actual[k] == expected[k] for k in keys) and not changed,
                  expected={k: expected[k] for k in keys}, actual={k: actual[k] for k in keys},
                  protected_files_changed=changed,
                  runner='original simulate_laps, original mu60, isolated temporary cwd')
    with (output/'regression.json').open('x') as f:
        json.dump(report, f, indent=2)
    if not report['passed']:
        raise RuntimeError(f'baseline regression failed: {report}')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument('--verify-only', metavar='SESSION_DIR')
    modes.add_argument('--baseline-regression', action='store_true')
    parser.add_argument('--laps', type=int, default=3)
    parser.add_argument('--output-dir')
    parser.add_argument('--racetrack-set', default='mu60_closed')
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--start-pose', type=float, nargs=3)
    parser.add_argument('--initial-speed', type=float, default=0.)
    parser.add_argument('--save-scans', action='store_true')
    args = parser.parse_args()
    if args.verify_only:
        print(json.dumps(verify_session(args.verify_only), indent=2))
        return
    if args.baseline_regression:
        if not args.output_dir:
            parser.error('--baseline-regression requires --output-dir')
        print(json.dumps(run_baseline_regression(args.output_dir), indent=2))
        return
    result = run_esp_continuous_pp(args.laps, args.output_dir, args.racetrack_set,
                                  args.timeout, args.start_pose, args.initial_speed, args.save_scans)
    print(json.dumps(result, indent=2))
    if result['end_reason'] != 'requested_laps_reached':
        raise SystemExit(2)


if __name__ == '__main__':
    main()
