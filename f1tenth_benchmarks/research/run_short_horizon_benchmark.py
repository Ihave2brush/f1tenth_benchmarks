"""Compare 5/6/7 m planning on frozen poses and identical recorded motion.

Offline replay of inputs, not candidate control or an enforced deadline.
Fixed cases share a table within each mode/horizon group. Every moving
segment starts with a fresh geometry/table, independently of fixed cases.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import platform
from time import perf_counter

import numpy as np
import scipy

from .core.geometry import load_geometry
from .overtaking.road_table import get_road_table
from .overtaking.session import SessionConfig, make_plan


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def freeze_inputs(cases_path, steps=8):
    original = json.loads(Path(cases_path).read_text())
    session = Path(original['source_session'])
    lines = (session/'states.jsonl').read_bytes().splitlines(keepends=True)
    assert sha(session/'states.jsonl') == original['source_sha256']['states.jsonl']
    states = [json.loads(line) for line in lines]
    fixed, segments, seen = [], [], set()
    for case in original['cases']:
        if case['state_sha256'] in seen and case['state_sha256'] is not None:
            continue
        seen.add(case['state_sha256'])
        name = case['name'].rsplit('_', 1)[0]
        fixed.append(dict(name=name, pose=case['pose'], control_step=case['control_step'],
                          source_state_sha256=case['state_sha256'],
                          expected_useful=case['state_sha256'] is not None))
        if case['state_sha256'] is None or name == 'step317':
            continue
        end = next(i for i, s in enumerate(states) if s['control_step'] == case['control_step'])
        assert hashlib.sha256(lines[end]).hexdigest() == case['state_sha256']
        start = min(end, len(states)-steps)
        selected = []
        for index in range(start, start+steps):
            state = states[index]
            selected.append(dict(state={k: state[k] for k in
                ('control_step', 'sim_time_s', 'pose', 'speed_mps', 'valid', 'reason', 'frenet')},
                source_state_sha256=hashlib.sha256(lines[index]).hexdigest()))
        segments.append(dict(name=name, states=selected))
    settings = dict(original['cases'][0]['settings'])
    settings.pop('lookahead_m')
    settings['max_curvature_inv_m'] = settings.pop('max_curvature')
    return dict(schema_version=1, geometry_id=original['geometry_id'],
                source_session=str(session), source_sha256=original['source_sha256'],
                parent_cases_sha256=sha(cases_path), horizons_m=[5., 6., 7.],
                settings=settings, fixed_cases=fixed, moving_segments=segments,
                scope='recorded_centerline_PP_inputs_no_candidate_control')


def measure(geometry, state, config, plan_id, return_event=False):
    started = perf_counter()
    event = make_plan(geometry, state, config, plan_id)
    planned = perf_counter()
    raw = json.dumps(event, separators=(',', ':'), allow_nan=False).encode()
    ended = perf_counter()
    paths = event['paths']
    assert [p['candidate_id'] for p in paths] == [1, 2, 3, 4, 5]
    for path in paths:
        if path['sampled_valid']:
            assert path['start_position_error_m'] <= config.max_start_position_error_m
            assert abs(path['start_heading_error_rad']) <= config.max_start_heading_error_rad
            assert path['polyline_collision_verified']
    result = event.get('result', {})
    valid_ids = [p['candidate_id'] for p in paths if p['sampled_valid']]
    total = ended-started
    measurement = dict(source_control_step=state['control_step'], source_sim_time_s=state['sim_time_s'],
        speed_mps=state.get('speed_mps'), make_plan_s=planned-started,
        encoding_s=ended-planned, total_s=total, output_bytes=len(raw),
        valid_ids=valid_ids, valid_count=len(valid_ids),
        timely_valid_count=len(valid_ids) if total <= config.control_dt_s else 0,
        over_budget=total > config.control_dt_s,
        direct_valid_count=result.get('direct_valid_count'),
        repaired_valid_count=result.get('repaired_valid_count'),
        repair_attempted_count=(sum(bool(p.get('repair_attempted')) for p in paths)
                                if config.generation_mode != 'optimized' else None),
        phases=result.get('timing'), query_reuse=result.get('query_reuse'),
        validation_grid=result.get('validation_grid'),
        paths=[dict(candidate_id=p['candidate_id'], valid=p['sampled_valid'], reason=p['reason'],
            direct_attempt=p.get('direct_attempt'), generation_route=p.get('generation_route'),
            repair_attempted=p.get('repair_attempted'), timing=p.get('timing'),
            optimizer_attempt_count=len(p.get('attempts', [])), point_count=len(p['points']))
            for p in paths])
    return (measurement, event) if return_event else measurement


def summarize(rows):
    if not rows:
        raise ValueError('cannot summarize empty measurements')
    result = dict(count=len(rows), over_budget_count=sum(r['over_budget'] for r in rows),
        useful_within_budget_count=sum(r['timely_valid_count'] > 0 for r in rows),
        zero_valid_count=sum(r['valid_count'] == 0 for r in rows),
        valid_count_min=min(r['valid_count'] for r in rows),
        valid_count_max=max(r['valid_count'] for r in rows),
        valid_count_mean=float(np.mean([r['valid_count'] for r in rows])),
        valid_id_counts={str(i): sum(i in r['valid_ids'] for r in rows) for i in range(1, 6)})
    for key in ('make_plan_s', 'encoding_s', 'total_s'):
        a = np.array([r[key] for r in rows])
        result[key] = dict(median=float(np.median(a)), p95=float(np.percentile(a, 95)),
                           p99=float(np.percentile(a, 99)), max=float(np.max(a)))
    return result


def fresh_geometry(geometry_id):
    started = perf_counter()
    geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
    assert geometry.geometry_id == geometry_id
    get_road_table(geometry, 'maps/frenet/esp/geometry')
    return geometry, perf_counter()-started


def run(manifest, output, repeats=3):
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    (root/'inputs.json').write_text(json.dumps(manifest, indent=2, allow_nan=False)+'\n')
    report = dict(schema_version=1, target_s=.1, deadline_enforced=False,
        scope='make_plan_plus_JSON_excludes_control_disk_and_separate_geometry_table_initialization',
        input_sha256=sha(root/'inputs.json'), groups=[],
        environment=dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
            openblas_num_threads=os.environ.get('OPENBLAS_NUM_THREADS')),
        code_sha256={str(p): sha(p) for p in [Path(__file__),
            Path(__file__).with_name('overtaking')/'robust_adaptive_paths.py',
            Path(__file__).with_name('overtaking')/'session.py',
            Path(__file__).with_name('overtaking')/'geometry_queries.py',
            Path(__file__).with_name('overtaking')/'road_table.py']})
    for horizon in manifest['horizons_m']:
        for mode in ('optimized', 'direct_then_repair'):
            config = SessionConfig(**manifest['settings'], lookahead_m=horizon,
                query_mode='fast_with_fallback', generation_mode=mode)
            group = dict(horizon_m=horizon, mode=mode, config=asdict(config), fixed=[], moving=[])
            report['groups'].append(group)
            geometry, initialization = fresh_geometry(manifest['geometry_id'])
            group['fixed_initialization_s'] = initialization
            for case in manifest['fixed_cases']:
                state = dict(control_step=case['control_step'] or 0, sim_time_s=0.,
                             pose=case['pose'], valid=True, frenet=dict(valid=True))
                runs = [measure(geometry, state, config, i) for i in range(repeats+1)]
                row = dict(name=case['name'], expected_useful=case['expected_useful'],
                    cache_policy='first_encounter_in_shared_group_then_identical_repeats', runs=runs,
                    warm_summary=summarize(runs[1:]))
                group['fixed'].append(row)
                print(json.dumps(dict(horizon=horizon, mode=mode, fixed=case['name'],
                    warm_median_s=row['warm_summary']['total_s']['median'],
                    valid=runs[-1]['valid_count'])), flush=True)
            del geometry
            for segment in manifest['moving_segments']:
                geometry, initialization = fresh_geometry(manifest['geometry_id'])
                rows = [measure(geometry, item['state'], config, i)
                        for i, item in enumerate(segment['states'])]
                row = dict(name=segment['name'], initialization_s=initialization,
                           runs=rows, summary=summarize(rows))
                group['moving'].append(row)
                del geometry
                print(json.dumps(dict(horizon=horizon, mode=mode, moving=segment['name'],
                    summary=row['summary'])), flush=True)
                (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
            moving = [r for s in group['moving'] for r in s['runs']]
            group['moving_summary'] = summarize(moving)
            group['moving_after_first_in_each_segment_summary'] = summarize(
                [r for s in group['moving'] for r in s['runs'][1:]])
            (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    report['completed'] = True
    (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', help='reuse the emitted frozen inputs.json')
    parser.add_argument('--cases', default='docs/project/reports/algorithm_p01_cases_20261027.json')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--repeats', type=int, default=3)
    args = parser.parse_args()
    if not 1 <= args.repeats <= 100:
        parser.error('repeats must be in [1, 100]')
    manifest = json.loads(Path(args.manifest).read_text()) if args.manifest else freeze_inputs(args.cases)
    run(manifest, args.output_dir, args.repeats)


if __name__ == '__main__':
    main()
