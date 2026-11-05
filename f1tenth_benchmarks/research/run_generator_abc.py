"""Compare three proposal strategies with the same queries and hard gates.

A: Hermite then legacy repair. B: pose-corrected V5 width fraction then the
same repair. C: legacy optimization. Fixed-grid acceleration is a separate
experiment; it is not mixed into this generation-strategy comparison.
"""
import argparse
from contextlib import contextmanager
from dataclasses import asdict
import json
import os
from pathlib import Path
import platform
from time import perf_counter

import numpy as np
import scipy

from .core.geometry import load_geometry
from .overtaking import robust_adaptive_paths as planner
from .overtaking import session
from .run_fixed_grid_benchmark import audit_legacy_grid
from .run_short_horizon_benchmark import fresh_geometry, measure, sha, summarize

ARMS = {'A': 'direct_then_repair', 'B': 'width_fraction_then_repair', 'C': 'optimized'}


@contextmanager
def stage_profile():
    """Few function hooks, applied equally to all arms; exclude nested doubles."""
    targets = [(session, 'generate_robust_paths', 'generator'),
        *[(planner, name, label) for name, label in (
            ('query_road_sections', 'road_sections'), ('_reference_data', 'curvature_references'),
            ('direct_hermite_coefficients', 'direct_build'), ('width_fraction_spline', 'direct_build'),
            ('direct_constraint_screen', 'direct_screen'), ('fit_multistart', 'solver'),
            ('_validate', 'validation'), ('validate_path_segments', 'polyline'))]]
    saved, stack, timers = [], [], {}
    clocks = dict(generator_start=None, generator_end=None, first_candidate=None)
    def wrap(function, label):
        def measured(*args, **kwargs):
            started = perf_counter()
            if label == 'generator':
                clocks['generator_start'] = started
            if label in ('direct_build', 'solver') and clocks['first_candidate'] is None:
                clocks['first_candidate'] = started
            frame = [0.]
            stack.append(frame)
            try:
                return function(*args, **kwargs)
            finally:
                ended = perf_counter()
                elapsed = ended-started
                stack.pop()
                if stack:
                    stack[-1][0] += elapsed
                row = timers.setdefault(label, dict(calls=0, inclusive_s=0., exclusive_s=0.))
                row['calls'] += 1
                row['inclusive_s'] += elapsed
                row['exclusive_s'] += max(0., elapsed-frame[0])
                if label == 'generator':
                    clocks['generator_end'] = ended
        return measured
    result = dict(timers=timers)
    try:
        for module, name, label in targets:
            original = getattr(module, name)
            saved.append((module, name, original))
            setattr(module, name, wrap(original, label))
        yield result
    finally:
        for module, name, original in reversed(saved):
            setattr(module, name, original)
        start, first = clocks['generator_start'], clocks['first_candidate']
        result['preparation_to_first_candidate_s'] = None if first is None or start is None else first-start
        result['generator_total_s'] = timers.get('generator', {}).get('inclusive_s')


def path_quality(path):
    """Sampled quality only; thresholds are frozen in report metadata."""
    if not path['sampled_valid']:
        return None
    points = path['points']
    d = np.asarray([p['d'] for p in points])
    delta = np.diff(d)
    signs = np.sign(delta[np.abs(delta) > 1e-5])
    excess = max(0., float(np.abs(delta).sum()-abs(d[-1]-d[0])))
    curvature = np.asarray([p['curvature_inv_m'] for p in points])
    gaps = np.diff([p['s_progress'] for p in points])
    return dict(excess_lateral_tv_m=excess,
        increment_sign_reversals=int(np.count_nonzero(signs[1:] != signs[:-1])),
        max_abs_slope=float(max(abs(p['d_prime']) for p in points)),
        max_abs_curvature_inv_m=float(np.max(np.abs(curvature))),
        max_sample_curvature_change_per_m=float(np.max(np.abs(np.diff(curvature))/gaps)),
        min_lateral_margin_m=float(min(p['lateral_margin_m'] for p in points)),
        start_position_error_m=path['start_position_error_m'],
        start_heading_error_rad=path['start_heading_error_rad'])


def measured_plan(geometry, state, config, index):
    with stage_profile() as profile:
        row, event = measure(geometry, state, config, index, return_event=True)
    row['stage_profile'] = profile
    row['optimizer_candidate_count'] = sum(bool(p.get('attempts')) for p in event['paths'])
    row['optimizer_iterations'] = sum(a['iterations'] for p in event['paths'] for a in p.get('attempts', []))
    for summary, path in zip(row['paths'], event['paths']):
        summary['quality'] = path_quality(path)
    return row, event


def run(inputs, output, repeats=3, arms=('A', 'B', 'C')):
    manifest = json.loads(Path(inputs).read_text())
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    (root/'inputs.json').write_bytes(Path(inputs).read_bytes())
    source_paths = [Path(__file__), Path(__file__).with_name('run_short_horizon_benchmark.py'),
        Path(__file__).with_name('run_fixed_grid_benchmark.py'),
        *[Path(__file__).with_name('overtaking')/name for name in ('robust_adaptive_paths.py',
            'session.py', 'width_fraction_paths.py', 'geometry_queries.py', 'road_table.py')]]
    report = dict(schema_version=1, input_sha256=sha(inputs), target_s=.1, deadline_enforced=False,
        scope='make_plan_plus_JSON_with_equal_stage_hooks_excludes_control_disk_initialization_and_B_exact_audit',
        quality_policy=dict(sign_increment_threshold_m=1e-5, sampled_only=True,
            hard_limit_changes=False, acceptance_based_on_quality_metrics=False),
        B_definition='width_fraction_c2_hermite_pose_correction_v1_not_unmodified_PR5_V5',
        environment=dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__,
            openblas_num_threads=os.environ.get('OPENBLAS_NUM_THREADS')),
        code_sha256={str(p): sha(p) for p in source_paths}, groups=[])
    def save():
        (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    for horizon in manifest['horizons_m']:
        for arm in arms:
            config = session.SessionConfig(**manifest['settings'], lookahead_m=horizon,
                query_mode='fast_with_fallback', generation_mode=ARMS[arm])
            group = dict(arm=arm, horizon_m=horizon, config=asdict(config), fixed=[], moving=[])
            report['groups'].append(group)
            geometry, initialization = fresh_geometry(manifest['geometry_id'])
            group['fixed_initialization_s'] = initialization
            pending_audits = []
            for case in manifest['fixed_cases']:
                state = dict(control_step=case['control_step'] or 0, sim_time_s=0.,
                             pose=case['pose'], valid=True, frenet=dict(valid=True))
                pairs = [measured_plan(geometry, state, config, i) for i in range(repeats+1)]
                rows = [r for r, _ in pairs]
                group['fixed'].append(dict(name=case['name'], runs=rows, warm_summary=summarize(rows[1:])))
                if arm == 'B':
                    pending_audits.extend(pairs)
                print(json.dumps(dict(arm=arm, horizon=horizon, fixed=case['name'],
                    warm_median_s=group['fixed'][-1]['warm_summary']['total_s']['median'],
                    valid=rows[-1]['valid_count'], optimizer_candidates=rows[-1]['optimizer_candidate_count'])), flush=True)
            del geometry
            if arm == 'B':
                audit_geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
                for row, event in pending_audits:
                    row['independent_exact_audit'] = audit_legacy_grid(audit_geometry, event, config)
                del audit_geometry, pending_audits
            save()
            for segment in manifest['moving_segments']:
                geometry, initialization = fresh_geometry(manifest['geometry_id'])
                pairs = [measured_plan(geometry, item['state'], config, i)
                         for i, item in enumerate(segment['states'])]
                rows = [r for r, _ in pairs]
                del geometry
                if arm == 'B':
                    audit_geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
                    for row, event in pairs:
                        row['independent_exact_audit'] = audit_legacy_grid(audit_geometry, event, config)
                    del audit_geometry
                group['moving'].append(dict(name=segment['name'], initialization_s=initialization,
                    runs=rows, summary=summarize(rows)))
                del pairs
                print(json.dumps(dict(arm=arm, horizon=horizon, moving=segment['name'],
                    summary=group['moving'][-1]['summary'])), flush=True)
                save()
            group['moving_summary'] = summarize([r for s in group['moving'] for r in s['runs']])
            group['moving_after_first_summary'] = summarize([r for s in group['moving'] for r in s['runs'][1:]])
            save()
    audits = [r['independent_exact_audit'] for g in report['groups'] if g['arm'] == 'B'
              for section in ('fixed', 'moving') for s in g[section] for r in s['runs']]
    report.update(completed=True, B_independent_audit_passed=all(a['passed'] for a in audits),
                  B_audited_candidate_instances=sum(a['checked_count'] for a in audits))
    save()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', default='docs/project/reports/short_horizon_inputs_20261028.json')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--arm', choices=('all', 'A', 'B', 'C'), default='all')
    args = parser.parse_args()
    if not 1 <= args.repeats <= 100:
        parser.error('repeats must be in [1,100]')
    run(args.inputs, args.output_dir, args.repeats, tuple(ARMS) if args.arm == 'all' else (args.arm,))


if __name__ == '__main__':
    main()
