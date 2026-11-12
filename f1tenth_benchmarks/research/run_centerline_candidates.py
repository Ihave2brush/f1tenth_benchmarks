"""Centreline PP plus synchronous V5.2 recording, independent of LMPC."""
import argparse
from collections import Counter
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import subprocess
from time import perf_counter

import numpy as np

from .continuous_sim import F1TenthSim_Continuous
from .core.centreline_control import CentrelinePurePursuit
from .core.frenet_lap_tracking import CoreLapTracker
from .core.geometry.tracker import TrackingOptions
from .continuous_logging import _json_safe
from .overtaking.session import SessionConfig, PlanScheduler, SessionWriter, make_plan, reject_paths
from .overtaking.track_data import TrackData
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file


def source_version():
    def git(*args):
        return subprocess.check_output(['git', *args], stderr=subprocess.DEVNULL)
    try:
        version = dict(commit=git('rev-parse', 'HEAD').decode().strip(),
                       branch=git('branch', '--show-current').decode().strip() or None,
                       tracked_diff_sha256=hashlib.sha256(git('diff', 'HEAD', '--binary')).hexdigest())
    except (OSError, subprocess.CalledProcessError):
        version = dict(commit=None, branch=None, tracked_diff_sha256=None)
        # The simulation image has no Git executable. Read the checkout's
        # public ref files while source hashes still identify every code file.
        git_dir = Path('.git')
        try:
            if git_dir.is_file():
                git_dir = git_dir.parent/git_dir.read_text().strip().removeprefix('gitdir: ')
            head = (git_dir/'HEAD').read_text().strip()
            if head.startswith('ref: '):
                ref = head[5:]
                version['branch'] = ref.removeprefix('refs/heads/')
                if (git_dir/ref).exists():
                    version['commit'] = (git_dir/ref).read_text().strip()
                else:
                    for line in (git_dir/'packed-refs').read_text().splitlines():
                        if line.endswith(' '+ref):
                            version['commit'] = line.split()[0]
            else:
                version['commit'] = head
        except OSError:
            pass
    # Include new, untracked code too; Git diff alone cannot identify it.
    paths = sorted(Path('f1tenth_benchmarks/research').rglob('*.py'))
    paths += sorted(Path('f1tenth_benchmarks/simulator').glob('*.py'))
    paths += [Path('f1tenth_benchmarks/classic_racing/GlobalPurePursuit.py'),
              Path('f1tenth_benchmarks/utils/BasePlanner.py')]
    version['code_sha256'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
    return version


def snapshot(sim):
    valid = bool(sim.last_tracking_result.valid and np.isfinite(sim.current_state).all())
    frenet = sim.tracker.frenet if valid else dict(valid=False, reason=sim.terminal_reason)
    return _json_safe(dict(schema_version=1, control_step=sim.control_steps, sim_time_s=sim.current_time,
        elapsed_time_s=sim.elapsed_time, pose=sim.current_state[[0, 1, 4]].tolist(),
        speed_mps=float(sim.current_state[3]), physical_state=sim.current_state.tolist(),
        pre_step_state=sim.last_pre_step_state.tolist(),
        pre_step_steering_buffer=sim.last_pre_step_buffer.tolist(),
        steering_buffer=sim.dynamics_simulator.steer_buffer.tolist(),
        valid=valid, reason=None if valid else sim.terminal_reason,
        frenet=frenet, completed_full_laps=sim.tracker.completed_full_laps,
        reset_count=sim.reset_count, collision=bool(sim.collision), terminal_reason=sim.terminal_reason))


def run_session(output_dir, config=None, geometry_dir='maps/frenet/esp/geometry', map_yaml='maps/esp.yaml',
                generator=None, experimental_metadata=None):
    config = config or SessionConfig()
    if Path(output_dir).exists():
        raise FileExistsError('choose a new session directory')
    track = TrackData(geometry_dir, map_yaml)
    if config.query_mode in ('fast_with_fallback', 'fixed_grid_with_fallback', 'fixed_grid_lookup'):
        from .overtaking.road_table import get_road_table
        get_road_table(track.geometry, geometry_dir)
    if not track.geometry.curve.closed:
        raise ValueError('continuous laps require a closed reference')
    map_id = track.manifest['map_id']
    if map_id not in ('esp', 'my_map') or Path(map_yaml).resolve() != Path('maps', map_id+'.yaml').resolve():
        raise ValueError('runner requires a matching ESP or MY map bundle')
    config = config.for_map(map_id)
    physics = load_parameter_file('simulator_params')
    vehicle = load_parameter_file('vehicle_params')
    substeps = round(config.control_dt_s/physics.timestep)
    if substeps < 1 or abs(substeps*physics.timestep-config.control_dt_s) > 1e-10:
        raise ValueError('control_dt_s must be an integer number of physics ticks')
    if config.speed_mps > vehicle.max_speed:
        raise ValueError('speed_mps exceeds vehicle limit')
    if config.lookahead_m >= track.geometry.curve.L:
        raise ValueError('candidate lookahead must be shorter than lap length')
    controller = CentrelinePurePursuit(track.control_path, config.speed_mps,
                                      config.pp_lookahead_m, vehicle.wheelbase, vehicle.max_steer)
    sim = F1TenthSim_Continuous(map_id, timeout=config.timeout_s, centreline_xy=track.control_path,
                               extra_params={'n_sim_steps': substeps})
    tracking_options = TrackingOptions(max_speed_mps=float(vehicle.max_speed))
    sim.tracker = CoreLapTracker(track.geometry, track.control_path, frenet_options=tracking_options)
    reference = track.geometry.reference(0.)
    start = [*reference['xy'], reference['psi']]
    obs, done, actual_start = sim.reset(start)
    metadata = _json_safe(dict(schema_version=1, kind='centerline_candidates_recording',
        map=map_id, frame_id='map', geometry_id=track.geometry.geometry_id,
        track=track.provenance, track_audit=track.audit, config=asdict(config),
        tracking_options=asdict(tracking_options),
        simulator_params=vars(sim.params), vehicle_params=vars(vehicle),
        program=source_version(), python_version=platform.python_version(), numpy_version=np.__version__,
        created_at_utc=datetime.now(timezone.utc).isoformat(), initial_pose=actual_start.tolist(),
        initial_speed_mps=0., initialization_action=[0., 0.], playback_origin_s=sim.current_time,
        planning_mode='synchronous_inspection_only', control_source='shared_centreline_pure_pursuit',
        candidate_control_enabled=False, footprint_verified=False, planning_allowed=False))
    if experimental_metadata is not None:
        metadata['experimental_planner'] = experimental_metadata
    writer = SessionWriter(output_dir, metadata)
    schedule = PlanScheduler(config.plan_period_s, sim.current_time)
    wall_start = perf_counter()
    durations, valid_counts, errors_d, errors_yaw = [], [], [], []
    rejects = Counter()
    events = []
    plan_id = 0
    cycle_timings = []
    reason, error = 'error', None

    def record_plan(state, trigger):
        nonlocal plan_id
        try:
            event = make_plan(track.geometry, state, config, plan_id, trigger, generator=generator)
            if experimental_metadata is not None:
                event['validation_scope'] = experimental_metadata['validation_scope']
        except BaseException as exc:
            # Persist source and all IDs even when a programming/runtime error
            # interrupts generation. Fail the session instead of swallowing it.
            failed = dict(schema_version=1, plan_id=plan_id, trigger=trigger,
                source_control_step=state['control_step'], source_sim_time_s=state['sim_time_s'],
                available_control_step=state['control_step'], available_sim_time_s=state['sim_time_s'],
                source_pose=state['pose'], geometry_id=track.geometry.geometry_id,
                status='error', reason=f'{type(exc).__name__}: {exc}',
                paths=reject_paths('generation_interrupted'), sampled_valid_count=0, wall_time_s=None,
                planning_allowed=False, footprint_verified=False)
            writer.append('plans', failed)
            plan_id += 1
            raise
        writer.append('plans', event)
        plan_id += 1
        durations.append(event['wall_time_s'])
        valid_counts.append(event['sampled_valid_count'])
        rejects.update(p['reason'] for p in event['paths'] if not p['sampled_valid'])
        progress = dict(status='running', control_step=state['control_step'],
            sim_time_s=state['sim_time_s'], plan_count=plan_id,
            sampled_valid_count=event['sampled_valid_count'], lookahead_m=config.lookahead_m,
            plan_period_s=config.plan_period_s, latest_plan_wall_time_s=event['wall_time_s'],
            elapsed_wall_time_s=perf_counter()-wall_start)
        progress_tmp = writer.root/'progress.tmp'
        progress_tmp.write_text(json.dumps(progress, allow_nan=False))
        progress_tmp.replace(writer.root/'progress.json')
        print(json.dumps(progress), flush=True)

    try:
        while True:
            cycle_start = perf_counter()
            previous_plan_count = plan_id
            state = snapshot(sim)
            writer.append('states', state)
            if state['valid']:
                errors_d.append(abs(state['frenet']['d']))
                errors_yaw.append(abs(state['frenet']['e_psi']))
            if sim.last_tracking_result.event:
                events.append(asdict(sim.last_tracking_result.event))
            completed = sim.tracker.completed_full_laps >= config.laps
            preview_limit = bool(config.max_control_steps and sim.control_steps >= config.max_control_steps)
            exceeded = bool(state['valid'] and (errors_d[-1] > config.max_lateral_error_m or
                                               errors_yaw[-1] > config.max_heading_error_rad))
            terminal = done or completed or exceeded or preview_limit
            if terminal or schedule.due(sim.current_time):
                record_plan(state, 'terminal' if terminal else 'periodic')
            if terminal:
                reason = (sim.terminal_reason if done else 'tracking_limit_exceeded' if exceeded
                          else 'requested_laps_reached' if completed else 'control_step_limit')
                cycle_timings.append(dict(source_control_step=state['control_step'], terminal=True,
                    plan_generated=plan_id != previous_plan_count, wall_time_s=perf_counter()-cycle_start))
                break
            action = controller.plan(obs)
            writer.append('actions', dict(schema_version=1, source_control_step=sim.control_steps,
                target_control_step=sim.control_steps+1, source_sim_time_s=sim.current_time,
                action=action.tolist(), control_source='centreline_pure_pursuit',
                controller_diagnostic=controller.diagnostic))
            obs, done = sim.step(action)
            cycle_timings.append(dict(source_control_step=state['control_step'], terminal=False,
                plan_generated=plan_id != previous_plan_count, wall_time_s=perf_counter()-cycle_start))
    except BaseException as exc:
        error = f'{type(exc).__name__}: {exc}'
        raise
    finally:
        wall_time = perf_counter()-wall_start
        summary = dict(schema_version=1, end_reason=reason, error=error,
            completed_full_laps=sim.tracker.completed_full_laps, reset_count=sim.reset_count,
            collision=bool(sim.collision), control_steps=sim.control_steps,
            state_count=len(writer.index['states']), action_count=len(writer.index['actions']),
            plan_count=plan_id, sim_elapsed_s=sim.elapsed_time, wall_time_s=wall_time,
            sim_seconds_per_wall_second=sim.elapsed_time/wall_time,
            planning_wall_time_s=dict(total=sum(durations), median=float(np.median(durations)) if durations else None,
                p95=float(np.percentile(durations, 95)) if durations else None, max=max(durations, default=None)),
            valid_candidate_count_min=min(valid_counts, default=0),
            valid_candidate_count_max=max(valid_counts, default=0), rejected_reasons=dict(rejects),
            max_abs_d_m=max(errors_d, default=None), max_abs_e_psi_rad=max(errors_yaw, default=None),
            lap_events=events, candidate_control_enabled=False, planning_allowed=False,
            footprint_verified=False, continuous_verified=False,
            recording_complete=reason == 'requested_laps_reached' and error is None)
        summary['cycle_timings'] = cycle_timings
        summary['cycle_timing_scope'] = ('snapshot, state/plan/action JSON and writes, progress, planning, '
            'PP and physics/scan/tracker; excludes initialization, finalization and timing-list append')
        writer.finalize(summary)
        with (writer.root/'progress.json').open('w') as stream:
            json.dump(dict(status='complete' if summary['recording_complete'] else 'stopped',
                           end_reason=reason, error=error, control_step=sim.control_steps,
                           plan_count=plan_id, lookahead_m=config.lookahead_m,
                           plan_period_s=config.plan_period_s), stream, allow_nan=False)
    return summary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--map', choices=('esp', 'my_map'), default='esp')
    parser.add_argument('--laps', type=int, default=1)
    parser.add_argument('--plan-period', type=float, default=.1)
    parser.add_argument('--lookahead', type=float, default=3.)
    parser.add_argument('--pp-lookahead', type=float, default=None,
                        help='PP distance in metres; default my_map=0.3, esp=0.6')
    parser.add_argument('--query-mode', choices=('exact', 'reused_exact', 'fast_with_fallback', 'fixed_grid_with_fallback', 'fixed_grid_lookup'), default='exact')
    parser.add_argument('--generation-mode', choices=('optimized', 'direct_only', 'direct_then_repair', 'width_fraction_only', 'width_fraction_then_repair'), default='optimized')
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--footprint-clearance', type=float, default=None,
                        help='three-disc wall clearance in metres; default simulator YAML')
    parser.add_argument('--footprint-step', type=float, default=.01,
                        help='three-disc validation spacing along reference s, metres')
    parser.add_argument('--max-control-steps', type=int, default=0, help='short preview only; zero runs to lap/timeout')
    args = parser.parse_args()
    result = run_session(args.output_dir, SessionConfig(laps=args.laps, plan_period_s=args.plan_period,
                                                       pp_lookahead_m=args.pp_lookahead,
                                                       lookahead_m=args.lookahead, timeout_s=args.timeout,
                                                       query_mode=args.query_mode,
                                                       generation_mode=args.generation_mode,
                                                       max_control_steps=args.max_control_steps,
                                                       footprint_step_m=args.footprint_step,
                                                       **({} if args.footprint_clearance is None else
                                                          dict(footprint_clearance_m=args.footprint_clearance))),
                         geometry_dir='maps/frenet/'+args.map+'/geometry',
                         map_yaml='maps/'+args.map+'.yaml')
    print(json.dumps(result, indent=2, allow_nan=False))
    if not result['recording_complete']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
