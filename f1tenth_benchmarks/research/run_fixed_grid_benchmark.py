"""Fixed-grid experiment with separate legacy-grid exact audits.

Normal timing excludes all audits, geometry initialization, control and disk.
Audits use an independent geometry and never warm the measured road table.
"""
import argparse
import json
import math
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.interpolate import BSpline

from .core.geometry import load_geometry
from .overtaking.geometry_queries import ExactQueryContext
from .overtaking.robust_adaptive_paths import _validate, _reference_data, query_road_sections
from .overtaking.session import SessionConfig
from .overtaking.reference_path import REFERENCE_PATH, reference_spline_valid
from .run_short_horizon_benchmark import fresh_geometry, measure, sha, summarize


def audit_legacy_grid(geometry, event, config):
    started = perf_counter()
    if not any(p['sampled_valid'] for p in event['paths']):
        return dict(checked_count=0, passed=True, paths=[], wall_time_s=perf_counter()-started)
    context = ExactQueryContext(geometry)
    try:
        query = context.geometry
        start = event['result']['start']
        step = min(config.sample_step_m, geometry.raster.resolution/2, .05)
        dense = np.linspace(0, config.lookahead_m, math.ceil(config.lookahead_m/step)+1)
        source = start['s']+dense
        wrapped = source % geometry.curve.L if geometry.curve.closed else source
        low, high = source-.001, source+.001
        if not geometry.curve.closed:
            low, high = np.maximum(0, low), np.minimum(geometry.curve.L, high)
        context.prefetch_reference(np.r_[source, wrapped, low, high])
        sections = query_road_sections(query, source, config.safety_margin_m)
        for section in sections:
            section['margin'] = config.safety_margin_m
        k, kp = _reference_data(query, source)
        rows = []
        for path in event['paths']:
            if not path['sampled_valid']:
                continue
            spline = path['spline']
            coefficients = np.asarray(spline['coefficients'])
            basis = BSpline(spline['knots'], np.eye(len(coefficients)), spline['degree'])
            is_reference = path.get('path_type') == REFERENCE_PATH
            if is_reference:
                if (not reference_spline_valid(path, config.lookahead_m) or
                        geometry.distance_s(path['reference_s_origin'], start['s']) > 1e-6):
                    rows.append(dict(candidate_id=path['candidate_id'], passed=False,
                                     reason='invalid_reference_path_contract', first_invalid_sample=0))
                    continue
            tolerance = 0. if path['endpoint_mode'] == 'fixed' else config.endpoint_tolerance_m
            points, invalid, segment = _validate(query, basis, coefficients, dense, sections, k, kp,
                path['requested_target_d'], tolerance, 0. if is_reference else start['d'],
                0. if is_reference else start['d_prime'],
                config.max_slope, config.max_curvature_inv_m)
            rows.append(dict(candidate_id=path['candidate_id'], passed=invalid is None,
                path_type=path.get('path_type', 'state_matched_trajectory'),
                first_invalid_sample=invalid, reason=None if invalid is None else points[invalid]['reason'],
                polyline_check=segment, point_count=len(points)))
        return dict(checked_count=len(rows), passed=all(r['passed'] for r in rows), paths=rows,
                    wall_time_s=perf_counter()-started, scope='legacy_dense_grid_exact_geometry_and_polyline')
    finally:
        context.close()


def run(inputs, output, repeats=3, query_modes=('fast_with_fallback', 'fixed_grid_with_fallback')):
    manifest = json.loads(Path(inputs).read_text())
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    (root/'inputs.json').write_bytes(Path(inputs).read_bytes())
    report = dict(schema_version=1, input_sha256=sha(inputs), target_s=.1, deadline_enforced=False,
        generation_mode='direct_then_repair',
        scope='make_plan_plus_JSON_excludes_initialization_control_disk_and_separate_exact_audit',
        code_sha256={str(p): sha(p) for p in [Path(__file__),
            Path(__file__).with_name('run_short_horizon_benchmark.py'),
            *[Path(__file__).with_name('overtaking')/n for n in
              ('fixed_grid.py', 'robust_adaptive_paths.py', 'session.py', 'geometry_queries.py', 'road_table.py')]]},
        groups=[])
    def save():
        (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    for horizon in manifest['horizons_m']:
        for mode in query_modes:
            config = SessionConfig(**manifest['settings'], lookahead_m=horizon,
                query_mode=mode, generation_mode='direct_then_repair')
            group = dict(horizon_m=horizon, query_mode=mode, fixed=[], moving=[])
            report['groups'].append(group)
            geometry, initialization = fresh_geometry(manifest['geometry_id'])
            group['fixed_initialization_s'] = initialization
            captured = []
            for case in manifest['fixed_cases']:
                state = dict(control_step=case['control_step'] or 0, sim_time_s=0.,
                             pose=case['pose'], valid=True, frenet=dict(valid=True))
                pairs = [measure(geometry, state, config, i, return_event=True) for i in range(repeats+1)]
                runs = [r for r, _ in pairs]
                group['fixed'].append(dict(name=case['name'], runs=runs, warm_summary=summarize(runs[1:])))
                captured.extend((case['name'], i, event) for i, (_, event) in enumerate(pairs))
                print(json.dumps(dict(horizon=horizon, mode=mode, fixed=case['name'],
                    warm_median_s=group['fixed'][-1]['warm_summary']['total_s']['median'])), flush=True)
            del geometry
            if mode == 'fixed_grid_with_fallback':
                audit_geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
                for name, i, event in captured:
                    row = next(c for c in group['fixed'] if c['name'] == name)['runs'][i]
                    row['legacy_grid_audit'] = audit_legacy_grid(audit_geometry, event, config)
                del audit_geometry
            del captured
            save()
            for segment in manifest['moving_segments']:
                geometry, initialization = fresh_geometry(manifest['geometry_id'])
                pairs = [measure(geometry, item['state'], config, i, return_event=True)
                         for i, item in enumerate(segment['states'])]
                rows = [r for r, _ in pairs]
                del geometry
                if mode == 'fixed_grid_with_fallback':
                    audit_geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
                    for row, event in pairs:
                        row['legacy_grid_audit'] = audit_legacy_grid(audit_geometry, event, config)
                    del audit_geometry
                group['moving'].append(dict(name=segment['name'], initialization_s=initialization,
                    runs=rows, summary=summarize(rows)))
                del pairs
                print(json.dumps(dict(horizon=horizon, mode=mode, moving=segment['name'],
                    summary=group['moving'][-1]['summary'])), flush=True)
                save()
            group['moving_summary'] = summarize([r for s in group['moving'] for r in s['runs']])
            group['moving_after_first_summary'] = summarize([r for s in group['moving'] for r in s['runs'][1:]])
            save()
    audits = [r['legacy_grid_audit'] for g in report['groups']
              if g['query_mode'] == 'fixed_grid_with_fallback'
              for section in ('fixed', 'moving') for s in g[section] for r in s['runs']]
    report.update(completed=True, legacy_grid_audit_passed=all(a['passed'] for a in audits),
                  legacy_grid_audited_candidates=sum(a['checked_count'] for a in audits))
    save()
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs', default='docs/project/reports/short_horizon_inputs_20261028.json')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--query-mode', choices=('both', 'fast_with_fallback', 'fixed_grid_with_fallback'),
                        default='both')
    args = parser.parse_args()
    if not 1 <= args.repeats <= 100:
        parser.error('repeats must be in [1,100]')
    modes = ('fast_with_fallback', 'fixed_grid_with_fallback') if args.query_mode == 'both' else (args.query_mode,)
    run(args.inputs, args.output_dir, args.repeats, modes)


if __name__ == '__main__':
    main()
