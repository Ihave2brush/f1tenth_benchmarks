"""Reproducible MY map offline-domain experiment; never controls a vehicle."""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import resource
from time import perf_counter, process_time

import numpy as np
import scipy

from .core.geometry import load_geometry
from .overtaking.millimetre_domain import MillimetreDomain, hermite_values, vehicle_clearance_profile
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
from .overtaking.robust_adaptive_paths import direct_hermite_coefficients
from .overtaking.session import SessionConfig
from .overtaking.reference_path import REFERENCE_PATH, STATE_PATH
from .run_fixed_grid_benchmark import audit_legacy_grid

GEOMETRY = 'maps/frenet/my_map/geometry'
MAP = 'maps/my_map.yaml'
INPUTS = 'docs/project/reports/my_map_3m_inputs_20261029.json'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def summary(values):
    return dict(median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                p99=float(np.percentile(values, 99)), max=float(np.max(values)))


def audit_event(plan, start):
    paths = []
    for path in plan['paths']:
        target = path['target_d_m']
        is_reference = path.get('path_type') == REFERENCE_PATH
        coefficients = (np.zeros(6) if is_reference else
                        direct_hermite_coefficients(3., start['d'], start['d_prime'], target))
        paths.append(dict(candidate_id=path['candidate_id'], sampled_valid=path['experimental_valid'],
            path_type=path.get('path_type', STATE_PATH), reference_s_origin=start['s'],
            reference_d=0. if is_reference else None,
            endpoint_mode='fixed', requested_target_d=target,
            spline=dict(coefficients=coefficients.tolist(), knots=[0.]*6+[3.]*6, degree=5)))
    return dict(paths=paths, result=dict(start=start))


def exact_start(geometry, pose):
    projection = geometry.to_frenet(pose[:2], yaw=pose[2])
    if not projection['valid']:
        return None, projection['reason']
    s, d = projection['s_wrapped'], projection['d']
    slope = (1-projection['kappa']*d)*math.tan(projection['e_psi'])
    if abs(projection['e_psi']) >= math.pi/2-1e-3:
        return None, 'heading_not_forward'
    # Keep measured pose exact; snapping the start would introduce a jump.
    return dict(s=float(s), d=float(d), d_prime=float(slope)), None


def plot_result(geometry, domain, rows, output):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 2, figsize=(13, 5.5))
    raster = geometry.raster
    extent = [raster.origin[0], raster.origin[0]+raster.width*raster.resolution,
              raster.origin[1], raster.origin[1]+raster.height*raster.resolution]
    axes[0].imshow(raster.gray, cmap='gray', extent=extent, origin='upper', vmin=0, vmax=255)
    arrays = domain.arrays
    for name in ('lower', 'upper'):
        points = arrays['xy']+arrays[name][:, None]*arrays['normal']
        axes[0].plot(points[:, 0], points[:, 1], color='#12a894', linewidth=1)
    axes[0].plot(arrays['xy'][:, 0], arrays['xy'][:, 1], '--', color='#748194', linewidth=.8)
    chosen = next((r for r in rows if r.get('start') and r['plan']['valid_ids']), None)
    if chosen:
        start = chosen['start']
        distances = np.linspace(0, 3, 301)
        reference, tangent, _, _ = geometry.curve.sample(start['s']+distances)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        for path in chosen['plan']['paths']:
            d = (np.zeros_like(distances) if path.get('path_type') == REFERENCE_PATH else
                 hermite_values(distances, 3, start['d'], start['d_prime'], path['target_d_m'])[0])
            points = reference+d[:, None]*normal
            axes[0].plot(points[:, 0], points[:, 1], '-' if path['experimental_valid'] else ':',
                         label=str(path['candidate_id']), alpha=.9 if path['experimental_valid'] else .4)
        axes[0].legend(title='Candidate', loc='upper right')
    margin = domain.metadata['margin_m']
    axes[0].set(title=f'MY map: {margin*100:g} cm centre-path prefilter (body corners unverified)', xlabel='x (m)', ylabel='y (m)')
    axes[0].set_aspect('equal')
    axes[1].plot(arrays['sites'], arrays['raw_lower']+margin, color='#a3aab4', linewidth=.6, label='Normal-ray + centre prefilter')
    axes[1].plot(arrays['sites'], arrays['raw_upper']-margin, color='#a3aab4', linewidth=.6)
    axes[1].fill_between(arrays['sites'], arrays['lower'], arrays['upper'], color='#12a894', alpha=.4,
                         label='Offline safe domain')
    axes[1].set(title='Conservative domain versus original normal-ray limits', xlabel='s (m)', ylabel='d (m)')
    axes[1].legend()
    fig.tight_layout()
    fig.savefig(output, dpi=160)
    plt.close(fig)


def run(output, inputs=INPUTS, domain_dir=None):
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    report = dict(schema_version=1, completed=False, candidate_control_enabled=False,
        input_sha256=sha(inputs), environment=dict(python=platform.python_version(),
        numpy=np.__version__, scipy=scipy.__version__, openblas_threads=os.environ.get('OPENBLAS_NUM_THREADS')),
        scope='table-only planner and separately measured pose projection; no disk/control/audits in timing',
        code_sha256={str(p): sha(p) for p in [Path(__file__), Path(__file__).with_name('overtaking')/'millimetre_domain.py']})
    def save():
        (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    manifest = json.loads(Path(inputs).read_text())
    (root/'inputs.json').write_bytes(Path(inputs).read_bytes())
    geometry = load_geometry(GEOMETRY, MAP, inspect=True)
    if geometry.geometry_id != manifest['geometry_id']:
        raise ValueError('input geometry mismatch')
    profile = vehicle_clearance_profile(load_parameter_file('simulator_params'))
    margin = profile['centre_prefilter_margin_m']
    report['vehicle_clearance_profile'] = profile
    report['simulator_params_sha256'] = sha('params/simulator_params.yaml')
    report['source_states_note'] = 'replay of archived poses; not a new lap with the resized vehicle'
    domain = (MillimetreDomain.build(geometry, margin=margin, progress=lambda row: print(json.dumps(row), flush=True))
              if domain_dir is None else MillimetreDomain.load(domain_dir, geometry, expected_margin=margin))
    report['reused_domain_directory'] = domain_dir
    domain.save(root/'domain')
    report['domain'] = domain.metadata
    started = perf_counter()
    domain = MillimetreDomain.load(root/'domain', geometry, expected_margin=margin)
    report['artifact_reload_wall_time_s'] = perf_counter()-started
    print(json.dumps(dict(stage='domain_complete', metadata=domain.metadata)), flush=True)
    rows = []
    source_rows = [(case['name'], case['pose']) for case in manifest['fixed_cases']]
    source_rows += [('moving_'+str(i), state['pose']) for i, state in enumerate(manifest['states'])]
    cpu_start = process_time()
    for name, pose in source_rows:
        started = perf_counter()
        start, reason = exact_start(geometry, pose)
        projection_time = perf_counter()-started
        if start is None:
            rows.append(dict(name=name, start=None, reason=reason, projection_s=projection_time))
            continue
        samples, explicit_samples = [], []
        for repeat in range(3):
            for inherit in ((True, False) if repeat % 2 == 0 else (False, True)):
                started = perf_counter()
                result = domain.plan(start['s'], start['d'], start['d_prime'], inherit=inherit)
                elapsed = perf_counter()-started
                if inherit:
                    plan = result
                    samples.append(elapsed)
                else:
                    explicit = result
                    explicit_samples.append(elapsed)
        explicit_time = float(np.median(explicit_samples))
        if explicit['valid_ids'] != plan['valid_ids'] or [p['boundary_valid'] for p in explicit['paths']] != [p['boundary_valid'] for p in plan['paths']]:
            raise AssertionError('outside-in and independent boundary checks disagree')
        rows.append(dict(name=name, pose=pose, start=start, projection_s=projection_time,
            planning_s=float(np.median(samples)), planning_runs_s=samples,
            explicit_planning_s=explicit_time, explicit_planning_runs_s=explicit_samples, plan=plan))
    report['normal_comparison_cpu_time_s'] = process_time()-cpu_start
    # Independent off-grid interpolation error check, including the seam.
    check_s = np.r_[np.linspace(0, domain.length, 1001), .000001, domain.length-.000001]
    exact_k = geometry.curve.sample(check_s)[3]
    exact_kp = (geometry.curve.sample((check_s+.001) % domain.length)[3]-
                geometry.curve.sample((check_s-.001) % domain.length)[3])/.002
    approximate_k, approximate_kp = domain.reference(check_s)
    report['reference_interpolation_audit'] = dict(samples=len(check_s),
        max_kappa_error_inv_m=float(np.max(abs(exact_k-approximate_k))),
        max_kappa_prime_error_inv_m2=float(np.max(abs(exact_kp-approximate_kp))))
    report['rows'] = rows
    moving = [r for r in rows if r['name'].startswith('moving_') and r['start']]
    report['moving_summary'] = dict(count=len(moving),
        planning_s=summary([r['planning_s'] for r in moving]),
        projection_s=summary([r['projection_s'] for r in moving]),
        projection_plus_planning_s=summary([r['projection_s']+r['planning_s'] for r in moving]),
        mean_valid_candidates=float(np.mean([len(r['plan']['valid_ids']) for r in moving])),
        min_valid_candidates=min(len(r['plan']['valid_ids']) for r in moving),
        zero_valid_steps=sum(not r['plan']['valid_ids'] for r in moving),
        inherited_boundary_checks=sum(r['plan']['inherited_boundary_checks'] for r in moving),
        explicit_boundary_checks=sum(r['plan']['explicit_boundary_checks'] for r in moving),
        explicit_comparison_planning_s=summary([r['explicit_planning_s'] for r in moving]),
        outside_in_matches_independent_checks=True,
        over_100ms=sum(r['projection_s']+r['planning_s'] > .1 for r in moving))
    # Current-code references, same poses. Policies differ (normal vs Euclidean
    # margin, solver repair and XY output), so this is NOT an equivalence speedup.
    from .overtaking.robust_adaptive_paths import generate_robust_paths
    from .overtaking.road_table import get_road_table
    report['current_code_references'] = []
    for generation in ('direct_only', 'direct_then_repair'):
        baseline_geometry = load_geometry(GEOMETRY, MAP, inspect=True)
        get_road_table(baseline_geometry, GEOMETRY)
        baseline_rows = []
        for state in manifest['states']:
            started = perf_counter()
            result = generate_robust_paths(baseline_geometry, *state['pose'], lookahead_m=3.,
                safety_margin_m=margin, query_mode='fixed_grid_lookup', generation_mode=generation)
            elapsed = perf_counter()-started
            baseline_rows.append(dict(wall_time_s=elapsed, valid_count=result['sampled_valid_count']))
        baseline_summary = dict(generation_mode=generation, query_mode='fixed_grid_lookup',
            scope='generate_robust_paths includes source projection, checks and XY points; excludes JSON',
            normal_ray_margin_m=margin, rows=baseline_rows,
            wall_time_s=summary([r['wall_time_s'] for r in baseline_rows]),
            mean_valid_candidates=float(np.mean([r['valid_count'] for r in baseline_rows])),
            zero_valid_steps=sum(r['valid_count'] == 0 for r in baseline_rows))
        report['current_code_references'].append(baseline_summary)
        print(json.dumps(dict(stage='current_code_reference', generation=generation,
                             summary={k: v for k, v in baseline_summary.items() if k != 'rows'})), flush=True)
    save()
    print(json.dumps(dict(stage='normal_complete', summary=report['moving_summary'])), flush=True)
    # Completely separate exact geometry, after timing. No table-only acceptance
    # is labelled production-safe based on this finite audit.
    audit_geometry = load_geometry(GEOMETRY, MAP, inspect=True)
    config = SessionConfig(generation_mode='direct_only', safety_margin_m=margin)
    for index, row in enumerate(rows):
        if not row['start']:
            continue
        event = audit_event(row['plan'], row['start'])
        row['original_density_audit'] = audit_legacy_grid(audit_geometry, event, config)
        if index % 10 == 0:
            print(json.dumps(dict(stage='exact_audit', row=index, total=len(rows))), flush=True)
    # A small high-density cross-check adds independent continuous-reference
    # occupancy and smoothness samples every <=1 mm, not only the old 2.5 cm.
    from .overtaking.geometry_queries import ExactQueryContext
    from .overtaking.robust_adaptive_paths import _validate, _reference_data, query_road_sections, _basis
    dense_audits = []
    for row in [r for r in rows if r['start'] and r['plan']['valid_ids']][:3]:
        context = ExactQueryContext(audit_geometry)
        try:
            start = row['start']
            distance = np.linspace(0, 3, 3001)
            source = start['s']+distance
            sections = query_road_sections(context.geometry, source, margin)
            for section in sections:
                section['margin'] = margin
            k, kp = _reference_data(context.geometry, source)
            for path in row['plan']['paths']:
                if not path['experimental_valid']:
                    continue
                is_reference = path.get('path_type') == REFERENCE_PATH
                coefficients = (np.zeros(6) if is_reference else
                                direct_hermite_coefficients(3., start['d'], start['d_prime'], path['target_d_m']))
                points, invalid, segment = _validate(context.geometry, _basis(3., 6), coefficients,
                    distance, sections, k, kp, path['target_d_m'], 0.,
                    0. if is_reference else start['d'], 0. if is_reference else start['d_prime'],
                    1., config.max_curvature_inv_m)
                dense_audits.append(dict(name=row['name'], candidate_id=path['candidate_id'],
                    passed=invalid is None, reason=None if invalid is None else points[invalid]['reason'],
                    point_count=len(points), polyline=segment))
        finally:
            context.close()
    report['one_mm_audits'] = dense_audits
    audits = [r['original_density_audit'] for r in rows if 'original_density_audit' in r]
    report['audit_summary'] = dict(original_density_candidate_instances=sum(a['checked_count'] for a in audits),
        original_density_failures=sum(not p['passed'] for a in audits for p in a['paths']),
        one_mm_candidate_instances=len(dense_audits), one_mm_failures=sum(not a['passed'] for a in dense_audits))
    report['peak_rss_kib_including_build_and_audits'] = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    report['completed'] = True
    save()
    plot_result(geometry, domain, rows, root/'overview.png')
    print(json.dumps(dict(stage='complete', summary=report['moving_summary'], audits=report['audit_summary'])), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--inputs', default=INPUTS)
    parser.add_argument('--domain-dir', help='reload a source/hash-checked offline artifact instead of rebuilding')
    args = parser.parse_args()
    run(args.output_dir, args.inputs, args.domain_dir)
