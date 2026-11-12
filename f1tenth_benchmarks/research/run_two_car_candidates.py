"""Run two-car overtaking with no LiDAR, a centreline lead, and a candidate-controlled ego car."""

import argparse
from collections import Counter
from dataclasses import asdict
from functools import partial
import json
import math
from pathlib import Path

import numpy as np

from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
from .continuous_logging import _json_safe
from .overtaking.millimetre_domain import MillimetreDomain
from .overtaking.track_data import TrackData
from .overtaking.two_car_env import TwoCarOvertakingEnv, sample_candidate_xy, select_candidate_default
from .run_two_car import plot


def _load_domain(track, map_name, domain_dir, output, margin):
    if domain_dir is not None:
        directory = Path(domain_dir)
        return MillimetreDomain.load(directory, track.geometry, expected_margin=margin), directory
    historical = {'my_map': 'mm_domain_width27_length51_20261030_001',
                  'esp': 'esp_mm_20ms_20261030_001'}
    directory = Path('Data/research/overtaking') / historical[map_name] / 'domain'
    try:
        return MillimetreDomain.load(directory, track.geometry, expected_margin=margin), directory
    except (FileNotFoundError, ValueError) as exc:
        print(f'Building a 1 mm domain for {map_name}: {exc}', flush=True)
    return MillimetreDomain.build(track.geometry, margin=margin), output / 'domain'


def _timing_ms(seconds):
    if not seconds:
        return None
    ms = np.asarray(seconds) * 1000
    p50, p95 = np.percentile(ms, [50, 95])
    return dict(p50=float(p50), p95=float(p95), max=float(ms.max()))


def _preview(env, frames, output):
    plot_frames = [dict(frame, states=[frame[key]['physical_state'] for key in ('ego_state', 'lead_state')])
                   for frame in frames]
    plan_frame = next((frame for frame in reversed(plot_frames) if frame.get('plan')), None)
    if plan_frame is not None:
        candidates = []
        for path in plan_frame['plan']['paths']:
            try:
                xy = sample_candidate_xy(env.geometry, path, env.config.lookahead_m)
            except ValueError:
                if path['sampled_valid']:
                    raise
                continue
            candidates.append(dict(id=path['candidate_id'], valid=path['sampled_valid'], xy=xy))
        plan_frame['candidates'] = candidates
    plot(env.sim, plot_frames, env.track.control_path, output, candidate_frame=plan_frame)


def run(output, map_name='my_map', domain_dir=None, seconds=12., gap=2., ego_speed=1.2,
        lead_speed=.9, budget_ms=40., preferred_candidate=3, wall_clearance=.05,
        pair_clearance=.05, ego_start_s=0., phase=1, original_esp_control=False, path_lookahead_m=3.,
        rolling_start=False, laps=None, curve_speed_control=False, measured_bend_caps=False,
        raceline_path=None, raceline_speed=None, skip_raceline_candidates=False):
    if map_name not in ('my_map', 'esp') or preferred_candidate not in range(1, 6):
        raise ValueError('invalid map or preferred candidate')
    if not math.isfinite(seconds) or seconds <= 0 or not math.isfinite(wall_clearance) or wall_clearance < 0:
        raise ValueError('invalid duration or wall clearance')
    if laps is not None and (type(laps) is not int or laps < 1):
        raise ValueError('laps must be a positive integer')
    out = Path(output)
    if out.exists():
        raise FileExistsError('choose a new output directory')
    track = TrackData(f'maps/frenet/{map_name}/geometry', f'maps/{map_name}.yaml')
    physics = load_parameter_file('simulator_params')
    margin = physics.vehicle_width / 2 + wall_clearance
    domain, directory = _load_domain(track, map_name, domain_dir, out, margin)
    order = (preferred_candidate,) + tuple(i for i in (3, 2, 4, 1, 5) if i != preferred_candidate)
    env = TwoCarOvertakingEnv(track, domain, ego_speed, lead_speed, gap, ego_start_s, budget_ms/1000,
                              wall_clearance, pair_clearance,
                              selector=partial(select_candidate_default, preferred_order=order), phase=phase,
                              original_esp_control=original_esp_control, path_lookahead_m=path_lookahead_m,
                              rolling_start=rolling_start, curve_speed_control=curve_speed_control,
                              measured_bend_caps=((151., 166., 3.9), (184., 202., 3.8)) if measured_bend_caps else (),
                              raceline_path=raceline_path, raceline_speed=raceline_speed,
                              raceline_candidates=not skip_raceline_candidates)
    frames = [env.reset()]
    cars = ('ego_state', 'lead_state')
    origins = {car: frames[0][car]['frenet'].get('s_unwrapped') for car in cars}
    laps_at = {car: [] for car in cars}
    pass_times = []
    selected, valid = Counter(), Counter()
    plan_times, cycle_times = [], []
    for _ in range(math.ceil(seconds/env.config.control_dt_s)):
        if frames[-1]['terminal_reason'] is not None:
            break
        transition, frame = env.step()
        plan = transition['plan']
        cycle_times.append(transition['cycle_wall_time_s'])
        if plan is not None:
            plan_times.append(plan['wall_time_s'])
            valid[plan['sampled_valid_count']] += 1
            if transition['selection_reason'] != 'raceline':
                selected[transition['selected_candidate_id']] += 1
        frames[-1] = dict(frames[-1], **transition)
        frames.append(frame)
        if laps is not None:
            for car in cars:
                progress = frame[car]['frenet'].get('s_unwrapped')
                if progress is None:
                    continue
                times = laps_at[car]
                while len(times) < laps and progress-origins[car] >= (len(times)+1)*env.length:
                    times.append(frame['time_s'])
                    print(f"{car}: lap {len(times)}/{laps} at {frame['time_s']:.2f} s", flush=True)
            if phase == 2 and transition['decision']['completed_passes'] > len(pass_times):
                pass_times.append(frames[-2]['time_s'])
                print(f'Pass {len(pass_times)} completed at {pass_times[-1]:.2f} s', flush=True)
            if len(cycle_times) % 300 == 0:
                print(f"Progress: {frame['time_s']:.1f} s · {len(pass_times)} completed passes", flush=True)
            if all(len(times) == laps for times in laps_at.values()) and not frame['terminal_reason']:
                frame['terminal_reason'] = 'lap_target_reached'
                break
    end = frames[-1]
    report = dict(map=map_name, lidar='disabled', candidate_control_enabled=True,
                  domain_dir=str(directory.resolve()), domain_fingerprint=domain.metadata['source_fingerprint'],
                  config=asdict(env.config), target_speeds_mps=[ego_speed, lead_speed], ego_start_s=ego_start_s,
                  spawn_gap_m=gap, preferred_order=order, budget_ms=budget_ms, pair_clearance_m=pair_clearance,
                  duration_requested_s=seconds, duration_s=end['time_s'], steps=len(cycle_times),
                  terminal_reason=end['terminal_reason'] or 'duration_reached', events=end['events'],
                  plan_ms=_timing_ms(plan_times), cycle_ms=_timing_ms(cycle_times),
                  selected_counts={str(i): selected[i] for i in range(1, 6)},
                  no_selection_steps=selected[None], valid_count_histogram=dict(valid),
                  fallback_steps=sum(f['ego_control']['fallback_used'] for f in frames[:-1]))
    if rolling_start:
        report['initial_speeds_mps'] = list(env.initial_speeds)
    if original_esp_control:
        report['steering_dt_s'] = env.steer_ticks * env.sim.params.timestep
        report['lookahead_policy'] = 'original_esp_speed_linked'
    if phase == 2:
        report['phase'] = phase
        report['completed_passes'] = env.selector.completed_passes
        report['lateral_target_inset_m'] = env.generator.lateral_inset_m
        report['decision_counts'] = dict(Counter(f['decision']['mode'] for f in frames[:-1]))
        report['decision_ms'] = _timing_ms([f['decision']['wall_time_s'] for f in frames[:-1]])
    if curve_speed_control:
        report['curve_speed_control'] = dict(
            lateral_accel_mps2=env.selector.speed_limit.lateral_accel_mps2,
            braking_mps2=env.selector.speed_limit.braking_mps2,
            preview_m=env.selector.speed_limit.preview_m,
            measured_bend_caps=env.selector.speed_limit.measured_bend_caps)
    if env.raceline is not None:
        report['raceline'] = dict(source=str(env.raceline.source), max_speed_mps=env.raceline.max_speed,
                                  candidate_generation_continuous=len(plan_times) == len(cycle_times),
                                  pass_wait_zone_m=list(env.race_pass_wait_zone),
                                  encounter_m=env.raceline.encounter_m,
                                  encounter_times_s=[f['time_s'] for f in frames[:-1]
                                                     if f['decision'].get('encounter_started')])
    if laps is not None:
        report['laps_requested_per_car'] = laps
        report['lap_completion_times_s'] = laps_at
        report['pass_completion_times_s'] = pass_times
    out.mkdir(parents=True, exist_ok=False)
    if directory == out / 'domain':
        domain.save(directory)
    for name, data in [('frames.json', frames), ('report.json', report)]:
        with (out / name).open('x') as stream:
            json.dump(_json_safe(data), stream, allow_nan=False)
            stream.write('\n')
    _preview(env, frames, out / 'preview.png')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--phase', type=int, choices=(1, 2), default=1)
    parser.add_argument('--map', choices=('my_map', 'esp'), default='my_map')
    parser.add_argument('--domain-dir')
    parser.add_argument('--original-esp-control', action='store_true')
    parser.add_argument('--rolling-start', action='store_true')
    parser.add_argument('--curve-speed-control', action='store_true')
    parser.add_argument('--measured-bend-caps', action='store_true')
    parser.add_argument('--raceline-path', help='ESP optimized raceline CSV to track between passes')
    parser.add_argument('--raceline-speed', type=float, help='speed ceiling on the optimized line')
    parser.add_argument('--skip-raceline-candidates', action='store_true',
                        help='skip unused candidate generation on the optimized line for diagnostics')
    parser.add_argument('--path-lookahead-m', type=float, default=3.)
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--laps', type=int, help='laps per car measured from their spawn positions; --seconds caps the run')
    parser.add_argument('--gap', type=float, default=2.)
    parser.add_argument('--ego-start-s', type=float, default=0.)
    parser.add_argument('--ego-speed', type=float, default=1.2)
    parser.add_argument('--lead-speed', type=float, default=.9)
    parser.add_argument('--budget-ms', type=float, default=40.)
    parser.add_argument('--preferred-candidate', type=int, choices=range(1, 6), default=3,
                        help='candidate preference for Phase 1')
    parser.add_argument('--wall-clearance', type=float, default=.05)
    parser.add_argument('--pair-clearance', type=float, default=.05)
    args = vars(parser.parse_args())
    args['map_name'] = args.pop('map')
    print(json.dumps(_json_safe(run(**args)), indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
