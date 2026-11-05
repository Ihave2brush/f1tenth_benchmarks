"""Freeze recorded inputs and compare original versus reused exact generation.

This is an offline diagnostic, not a real-time or continuous-domain acceptance.
Outputs use a new directory and include portable inputs, source hashes, complete
result digests, exact structural equality, shape metrics and phase timings.
"""
import argparse
from contextlib import contextmanager
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import subprocess
from time import perf_counter

import numpy as np
import scipy

from .core.geometry import load_geometry
from .overtaking import robust_adaptive_paths as current


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()


def sha_file(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def git_provenance():
    try:
        commit = subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True, stderr=subprocess.PIPE).strip()
        diff = subprocess.check_output(['git', 'diff', 'HEAD', '--binary'], stderr=subprocess.PIPE)
    except (OSError, subprocess.CalledProcessError):
        return dict(commit=None, tracked_diff_sha256=None, git_provenance_status='unavailable')
    return dict(commit=commit, tracked_diff_sha256=hashlib.sha256(diff).hexdigest(),
                git_provenance_status='available')


def freeze_cases(session, geometry):
    root = Path(session)
    metadata = json.loads((root/'metadata.json').read_text())
    assert metadata['geometry_id'] == geometry.geometry_id
    raw_states = (root/'states.jsonl').read_bytes()
    lines = raw_states.splitlines(keepends=True)
    states = [json.loads(line) for line in lines]
    valid = [i for i, state in enumerate(states) if state['valid']]
    # Freeze selection in the emitted manifest; future runs use those same poses.
    references = {i: geometry.reference(states[i]['frenet']['s_wrapped']) for i in valid}
    straight = min(valid, key=lambda i: abs(references[i]['kappa']))
    sharp = max(valid, key=lambda i: abs(references[i]['kappa']))
    widths = {}
    for i in valid:
        boundary = geometry.boundaries(states[i]['frenet']['s_wrapped'])
        if boundary['valid']:
            widths[i] = boundary['d_left']-boundary['d_right']
    narrow = min(widths, key=widths.get)
    selections = [('step310_40m', 310, 40.), ('step317_40m', 317, 40.),
                  ('step807_10m', 807, 10.), ('step807_40m', 807, 40.),
                  ('straight_40m', straight, 40.), ('sharp_40m', sharp, 40.),
                  ('narrow_40m', narrow, 40.), ('seam_3m', len(states)-2, 3.)]
    config = metadata['config']
    settings = dict(safety_margin_m=config['safety_margin_m'],
                    sample_step_m=config['sample_step_m'], max_slope=config['max_slope'],
                    max_curvature=config['max_curvature_inv_m'],
                    endpoint_tolerance_m=config['endpoint_tolerance_m'],
                    max_refinements=config['max_refinements'], max_iterations=config['max_iterations'])
    cases = []
    for name, index, length in selections:
        state = states[index]
        cases.append(dict(name=name, control_step=state['control_step'], pose=state['pose'],
                          state_sha256=hashlib.sha256(lines[index]).hexdigest(),
                          settings=dict(settings, lookahead_m=length)))
    cases.append(dict(name='outside_map_3m', control_step=None, pose=[1e6, 1e6, 0.],
                      state_sha256=None, settings=dict(settings, lookahead_m=3.)))
    return dict(schema_version=1, geometry_id=geometry.geometry_id,
                source_session=str(root), source_sha256={name: sha_file(root/name) for name in
                    ('states.jsonl', 'metadata.json', 'playback_index.json')}, cases=cases,
                acceptance=dict(complete_result_equal=True, valid_candidate_count_no_regression=True,
                                shape_change_allowed=False, real_time_claim=False))


@contextmanager
def measure_phases(module):
    original = {}
    times = {}
    for name in ('fit_multistart', '_validate', 'query_road_sections', '_reference_data'):
        original[name] = getattr(module, name)
        times[name] = dict(calls=0, wall_time_s=0.)
        def wrapped(*args, _name=name, **kwargs):
            start = perf_counter()
            try:
                return original[_name](*args, **kwargs)
            finally:
                times[_name]['calls'] += 1
                times[_name]['wall_time_s'] += perf_counter()-start
        setattr(module, name, wrapped)
    try:
        yield times
    finally:
        for name, function in original.items():
            setattr(module, name, function)


def evaluate(module, geometry, case, mode=None):
    settings = dict(case['settings'])
    if mode is not None:
        settings['query_mode'] = mode
    started = perf_counter()
    with measure_phases(module) as phases:
        try:
            x, y, yaw = case['pose']
            result = module.generate_robust_paths(geometry, x, y, yaw=yaw, **settings)
        except ValueError as exc:
            result = dict(error_type='ValueError', reason=str(exc))
    elapsed = perf_counter()-started
    reuse = result.pop('query_reuse', None)
    level = result.pop('verification_level', None)
    encode_start = perf_counter()
    raw = encoded(result)
    encoding_s = perf_counter()-encode_start
    metrics = []
    for path in result.get('paths', []):
        points = path['points']
        lateral = np.array([p['d'] for p in points])
        extra_tv = None if len(lateral) < 2 else float(
            np.abs(np.diff(lateral)).sum()-abs(lateral[-1]-lateral[0]))
        metrics.append(dict(candidate_id=path['candidate_id'], sampled_valid=path['sampled_valid'],
                            reason=path['reason'], point_count=len(points), excess_lateral_tv_m=extra_tv,
                            start_position_error_m=path['start_position_error_m'],
                            start_heading_error_rad=path['start_heading_error_rad']))
    measurement = dict(generation_wall_time_s=elapsed, phases=phases, encoding_wall_time_s=encoding_s,
                       verification_level=level,
                       encoded_bytes=len(raw), result_sha256=hashlib.sha256(raw).hexdigest(),
                       query_reuse=reuse, candidates=metrics,
                       sampled_valid_count=result.get('sampled_valid_count'), error=result.get('reason'))
    return result, measurement


def run(session, output, baseline_source=None, cases_path=None, query_mode='reused_exact',
        baseline_mode=None, warm_repeat=False, seed_csv=None, isolate_case_cache=False,
        warm_repetitions=1):
    out = Path(output)
    out.mkdir(parents=True, exist_ok=False)
    geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
    initialization_s = 0.
    if seed_csv:
        from .overtaking.road_table import get_road_table
        start = perf_counter()
        get_road_table(geometry, seed_csv)
        initialization_s = perf_counter()-start
    manifest = json.loads(Path(cases_path).read_text()) if cases_path else freeze_cases(session, geometry)
    assert manifest['geometry_id'] == geometry.geometry_id
    (out/'cases.json').write_bytes(encoded(manifest))
    baseline = current
    if baseline_source:
        spec = importlib.util.spec_from_file_location(
            'f1tenth_benchmarks.research.overtaking._frozen_p01_baseline', baseline_source)
        baseline = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(baseline)
    code_paths = [Path(current.__file__), Path(__file__),
                  Path(current.__file__).with_name('geometry_queries.py'),
                  Path(current.__file__).with_name('road_table.py'),
                  Path('f1tenth_benchmarks/research/core/geometry/frenet.py'),
                  Path('f1tenth_benchmarks/research/core/geometry/extraction.py'),
                  Path(current.__file__).with_name('constrained_adaptive_paths.py')]
    report = dict(schema_version=1, passed=False, cases=[], case_manifest_sha256=sha_file(out/'cases.json'),
                  geometry_id=geometry.geometry_id, map_source_hashes=geometry.raster.source_hashes,
                  geometry_manifest_sha256=sha_file('maps/frenet/esp/geometry/manifest.json'),
                  code_sha256={str(path): sha_file(path) for path in code_paths},
                  baseline_source_sha256=sha_file(baseline.__file__),
                  environment=dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__),
                  **git_provenance(),
                  repetitions_per_case=dict(baseline=1, first_generation=1,
                                             warm=warm_repetitions if warm_repeat else 0),
                  comparison_excluded_fields=['query_reuse', 'verification_level'],
                  timing_scope='offline_generation_with_phase_instrumentation',
                  query_mode=query_mode, baseline_mode=baseline_mode or 'exact',
                  csv_initialization_s=initialization_s,
                  limitation='few repeats on shared host, no tail-latency or real-time claim; numerical result comparison excludes method diagnostics')
    for case in manifest['cases']:
        case_initialization_s = 0.
        if isolate_case_cache and query_mode == 'fast_with_fallback':
            from .overtaking.road_table import get_road_table
            if hasattr(geometry, '_candidate_road_table'):
                del geometry._candidate_road_table
            start = perf_counter()
            get_road_table(geometry, seed_csv)
            case_initialization_s = perf_counter()-start
        print(json.dumps(dict(case=case['name'], status='baseline_started')), flush=True)
        exact, before = evaluate(baseline, geometry, case, baseline_mode)
        print(json.dumps(dict(case=case['name'], status='reused_started', baseline_s=before['generation_wall_time_s'])), flush=True)
        reused, after = evaluate(current, geometry, case, query_mode)
        equal = exact == reused
        row = dict(name=case['name'], complete_result_equal=equal, baseline=before,
                   speedup=before['generation_wall_time_s']/after['generation_wall_time_s'])
        row[query_mode] = after
        row['case_initialization_s'] = case_initialization_s
        row['isolated_case_cache'] = isolate_case_cache
        if warm_repeat:
            warm_measures = []
            warm_equal = True
            for _ in range(warm_repetitions):
                warm_result, warm_measure = evaluate(current, geometry, case, query_mode)
                warm_measures.append(warm_measure)
                warm_equal = warm_equal and exact == warm_result
            row['warm_repeat'] = warm_measures[0]
            row['warm_repetitions'] = warm_measures
            row['warm_result_equal'] = warm_equal
        report['cases'].append(row)
        (out/'report.json').write_bytes(encoded(report))
        print(json.dumps(dict(case=case['name'], complete_result_equal=equal,
                              exact_s=before['generation_wall_time_s'], reused_s=after['generation_wall_time_s'])), flush=True)
    report['passed'] = all(row['complete_result_equal'] and row.get('warm_result_equal', True)
                           for row in report['cases'])
    (out/'report.json').write_bytes(encoded(report))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', default='Data/research/overtaking/40m_every_step_20261021_001')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--baseline-source', help='optional frozen prechange robust_adaptive_paths.py')
    parser.add_argument('--cases', help='reuse a previously frozen cases.json without reading session')
    parser.add_argument('--query-mode', choices=('reused_exact', 'fast_with_fallback'), default='reused_exact')
    parser.add_argument('--baseline-mode', choices=('exact', 'reused_exact'))
    parser.add_argument('--warm-repeat', action='store_true')
    parser.add_argument('--seed-csv', help='optional audited geometry bundle to seed before timing generation')
    parser.add_argument('--isolate-case-cache', action='store_true', help='rebuild table before each case; time separately')
    parser.add_argument('--warm-repetitions', type=int, default=1)
    args = parser.parse_args()
    if not 1 <= args.warm_repetitions <= 100:
        parser.error('--warm-repetitions must be in [1, 100]')
    result = run(args.session, args.output_dir, args.baseline_source, args.cases,
                 args.query_mode, args.baseline_mode, args.warm_repeat, args.seed_csv,
                 args.isolate_case_cache, args.warm_repetitions)
    print(json.dumps(dict(passed=result['passed'], cases=len(result['cases']))))
    if not result['passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
