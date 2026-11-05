"""Plot recorded mm-domain failures with a matched original or reference line.

Read-only with respect to plans, map and parameters. Curvature in the figures
uses independently sampled continuous reference geometry at 1 mm spacing.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline, PPoly

from .core.geometry import load_geometry
from .overtaking.millimetre_domain import MillimetreDomain, hermite_values, hermite_segment_ranges
from .overtaking.constrained_adaptive_paths import evaluate_path_smoothness
from .run_millimetre_domain import GEOMETRY, MAP, sha
from .overtaking.reference_path import REFERENCE_PATH


def run(source, output, steps=None, session=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    root, out = Path(source), Path(output)
    out.mkdir(parents=True, exist_ok=False)
    report = json.loads((root/'report.json').read_text())
    map_id = report.get('map', 'my_map')
    geometry = load_geometry(f'maps/frenet/{map_id}/geometry', f'maps/{map_id}.yaml', inspect=True)
    session_dir = root/(session or ('session' if (root/'session'/'metadata.json').exists() else 'mm'))
    metadata = json.loads((session_dir/'metadata.json').read_text())
    domain_dir = metadata['experimental_planner']['domain_dir']
    domain = MillimetreDomain.load(domain_dir, geometry, expected_margin=report['profile']['centre_prefilter_margin_m'])
    original_path = root/'original_exact'/'plans.jsonl'
    original = ({p['source_control_step']: p for p in map(json.loads, original_path.read_text().splitlines())}
                if original_path.exists() else {})
    if steps is None and 'mm_zero_valid_details' in report:
        failures = report['mm_zero_valid_details']
    else:
        events = map(json.loads, (session_dir/'plans.jsonl').read_text().splitlines())
        failures = [dict(step=e['source_control_step'], pose=e['source_pose'],
                         start=e['result']['start'], paths=e['paths'])
                    for e in events if (e['source_control_step'] in steps if steps is not None
                                        else e['sampled_valid_count'] == 0)]
    comparison_label = 'V5.2 C3' if original else 'Reference d=0'
    pp_lookahead = metadata['config']['pp_lookahead_m']
    cases = []
    palette = ['#2166ac', '#d08a00', '#008b79', '#a64ac9', '#d6604d']
    limit = metadata['config']['max_curvature_inv_m']
    for failure in failures:
        step, start = failure['step'], failure['start']
        distance = np.linspace(0., 3., 3001)
        source_s = start['s']+distance
        reference, tangent, _, k = geometry.curve.sample(source_s)
        kp = (geometry.curve.sample(source_s+.001)[3]-geometry.curve.sample(source_s-.001)[3])/.002
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        indices = domain.indices(source_s)
        lower, upper = domain.arrays['lower'][indices], domain.arrays['upper'][indices]
        interval_edges, interval_rows = domain.segment_grid(start['s'], 3.)
        paths, metrics = [], []
        for candidate in failure['paths']:
            target = candidate['target_d_m']
            is_reference = candidate.get('path_type') == REFERENCE_PATH
            if candidate.get('generation_route') == 'budgeted_bspline_repair':
                spec = candidate['spline']
                curve = BSpline(spec['knots'], spec['coefficients'], spec['degree'])
                d, p, q = [curve(distance, nu=n) for n in range(3)]
                ends = curve(interval_edges)
                low, high = np.minimum(ends[:-1], ends[1:]), np.maximum(ends[:-1], ends[1:])
                roots = PPoly.from_spline(curve).derivative().roots(extrapolate=False)
                for x in roots[np.isfinite(roots) & (roots > 0) & (roots < 3.)]:
                    i = min(int(np.searchsorted(interval_edges, x, side='right')-1), len(low)-1)
                    low[i], high[i] = min(low[i], curve(x)), max(high[i], curve(x))
            else:
                d, p, q = ((np.zeros_like(distance),)*3 if is_reference else
                           hermite_values(distance, 3., start['d'], start['d_prime'], target))
                low, high = ((np.zeros(len(interval_rows)),)*2 if is_reference else
                             hermite_segment_ranges(interval_edges, 3., start['d'], start['d_prime'], target))
            _, curvature = evaluate_path_smoothness(d, p, q, k, kp)
            slack = np.minimum(low-domain.arrays['lower'][interval_rows], domain.arrays['upper'][interval_rows]-high)
            # Match the validator's numerical tolerance at touching boundaries.
            bad = np.flatnonzero(slack < -1e-9)
            first_bad = None if not len(bad) else float(interval_edges[int(bad[0])])
            peak = int(np.argmax(abs(curvature)))
            paths.append(dict(d=d, xy=reference+d[:, None]*normal, curvature=curvature))
            metrics.append(dict(candidate_id=candidate['candidate_id'], target_d_m=target,
                boundary_valid=candidate['boundary_valid'],
                min_safe_band_slack_m=float(slack.min()), first_bad_distance_m=first_bad,
                exact_max_abs_curvature_inv_m=float(abs(curvature[peak])),
                peak_distance_m=float(distance[peak]),
                exact_sampled_curvature_valid=bool(abs(curvature[peak]) <= limit+1e-7),
                recorded_valid=bool(candidate.get('sampled_valid', candidate.get('experimental_valid', False))),
                recorded_reason=candidate['reason']))
        old = next((p for p in original[step]['paths'] if p['candidate_id'] == 3), None) if original else None
        original_audits = []
        for original_path in original[step]['paths'] if original else []:
            if not original_path['sampled_valid']:
                continue
            old_spline = original_path['spline']
            original_curve = BSpline(old_spline['knots'], old_spline['coefficients'], old_spline['degree'])
            dd, pp, qq = [original_curve(distance, nu=n) for n in range(3)]
            _, kk = evaluate_path_smoothness(dd, pp, qq, k, kp)
            peak_k = float(abs(kk).max())
            original_audits.append(dict(candidate_id=original_path['candidate_id'],
                exact_max_abs_curvature_at_1mm_inv_m=peak_k,
                one_mm_sampled_curvature_pass=peak_k <= limit+1e-7))
        if old is not None:
            spline = old['spline']
            curve = BSpline(spline['knots'], spline['coefficients'], spline['degree'])
            old_d, old_p, old_q = [curve(distance, nu=n) for n in range(3)]
        else:
            # Never overlay an old lap from a different PP distance as though
            # it had the same source pose. Show the actual reference instead.
            old_d = old_p = old_q = np.zeros_like(distance)
        _, old_k = evaluate_path_smoothness(old_d, old_p, old_q, k, kp)
        old_xy = reference+old_d[:, None]*normal
        fig, axes = plt.subplots(2, 2, figsize=(14, 10))
        world, frenet, curvature_ax, table_ax = axes.ravel()
        raster = geometry.raster
        world.imshow(raster.gray, cmap='gray', extent=[-4.8, 3.9, -4.1, 2.3],
                     origin='upper', vmin=0, vmax=255, alpha=.85)
        for boundary in (lower, upper):
            xy = reference+boundary[:, None]*normal
            world.plot(xy[:, 0], xy[:, 1], color='#687e79', linewidth=1.1, alpha=.8)
        all_xy = []
        for index, (path, metric) in enumerate(zip(paths, metrics)):
            color = palette[index]
            xy = path['xy']
            all_xy.append(xy)
            world.plot(xy[:, 0], xy[:, 1], color=color, linewidth=1.5, label='New C'+str(index+1))
            frenet.plot(distance, path['d'], color=color, linewidth=1.5, label='C'+str(index+1))
            curvature_ax.plot(distance, abs(path['curvature']), color=color, linewidth=1.2,
                              label='C'+str(index+1))
            if metric['first_bad_distance_m'] is not None:
                x = metric['first_bad_distance_m']
                j = min(len(distance)-1, int(round(x/.001)))
                world.scatter([xy[j, 0]], [xy[j, 1]], marker='x', color='#b30000', s=42, zorder=6)
                frenet.scatter([x], [path['d'][j]], marker='x', color='#b30000', s=42, zorder=6)
        world.plot(old_xy[:, 0], old_xy[:, 1], '--', color='#111111', linewidth=1.7, label=comparison_label)
        frenet.plot(distance, old_d, '--', color='#111111', linewidth=1.6, label=comparison_label)
        curvature_ax.plot(distance, abs(old_k), '--', color='#111111', linewidth=1.5, label=comparison_label)
        frenet.fill_between(distance, lower, upper, color='#c8e8dc', alpha=.7, label='Offline safe band', zorder=0)
        frenet.set(title='B. Frenet domain: red crosses mark first boundary failure',
                   xlabel='Forward reference distance (m)', ylabel='d (m)', xlim=(0, 3.12))
        frenet.grid(alpha=.2)
        frenet.legend(fontsize=8, ncol=4, loc='lower left')
        curvature_ax.axhline(limit, color='#b30000', linestyle='--', linewidth=1.3,
                            label=f'Limit {limit:.6f} /m')
        ymax = max(max(m['exact_max_abs_curvature_inv_m'] for m in metrics), float(abs(old_k).max()), limit)*1.07
        curvature_ax.axhspan(limit, ymax, color='#fbe2df', alpha=.7)
        curvature_ax.set(title='C. Exact-reference curvature sampled every 1 mm',
                         xlabel='Forward reference distance (m)', ylabel='Absolute curvature (1/m)',
                         xlim=(0, 3), ylim=(0, ymax))
        c3 = metrics[2]
        curvature_ax.scatter([c3['peak_distance_m']], [c3['exact_max_abs_curvature_inv_m']],
                             color=palette[2], s=36, zorder=7)
        curvature_ax.annotate(f"C3 peak = {c3['exact_max_abs_curvature_inv_m']:.6f}",
            xy=(c3['peak_distance_m'], c3['exact_max_abs_curvature_inv_m']), xytext=(.15, ymax*.92),
            fontsize=9, arrowprops=dict(arrowstyle='->', color=palette[2]), color='#006f60')
        curvature_ax.grid(alpha=.2)
        curvature_ax.legend(fontsize=8, ncol=4, loc='lower left')
        pose = failure['pose']
        c, s = np.cos(pose[2]), np.sin(pose[2])
        half_length = report['profile']['vehicle_length_m']/2
        half_width = report['profile']['vehicle_width_m']/2
        corners = np.array([[half_length, half_width], [half_length, -half_width],
                            [-half_length, -half_width], [-half_length, half_width]])
        corners = corners @ np.array([[c, s], [-s, c]])+pose[:2]
        world.add_patch(Polygon(corners, closed=True, facecolor='#e9b4d5', edgecolor='#7b245a', alpha=.9, zorder=8))
        extent_xy = np.vstack([*all_xy, old_xy, corners])
        world.set(xlim=(extent_xy[:, 0].min()-.25, extent_xy[:, 0].max()+.25),
                  ylim=(extent_xy[:, 1].min()-.3, extent_xy[:, 1].max()+.3),
                  title='A. Map, proposals and '+('original optimized C3' if original else 'reference centreline'),
                  xlabel='x (m)', ylabel='y (m)')
        world.set_aspect('equal')
        world.legend(fontsize=8, ncol=2, loc='lower left')
        table_ax.axis('off')
        table_ax.set_title('D. New proposals: independent boundary / curvature verdicts', pad=18)
        cells = [[f"C{m['candidate_id']}", f"{m['target_d_m']:.3f}",
                  'PASS' if m['boundary_valid'] else 'FAIL',
                  'PASS' if m['exact_sampled_curvature_valid'] else 'FAIL',
                  f"{m['exact_max_abs_curvature_inv_m']:.6f}"] for m in metrics]
        table = table_ax.table(cellText=cells,
            colLabels=['Path', 'Target d (m)', 'Boundary', 'Curvature', 'Max |k| (/m)'],
            cellLoc='center', loc='upper center', bbox=[0, .50, 1, .44],
            colWidths=[.10, .22, .2, .2, .28])
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        for row, metric in enumerate(metrics, 1):
            table[row, 0].set_facecolor(palette[row-1])
            table[row, 0].get_text().set_color('white')
            for col, flag in [(2, metric['boundary_valid']), (3, metric['exact_sampled_curvature_valid'])]:
                table[row, col].set_facecolor('#d8eee4' if flag else '#f5d7d2')
        old_max = float(abs(old_k).max())
        c3_is_reference = failure['paths'][2].get('path_type') == REFERENCE_PATH
        comparison_note = (f'V5.2 C3 was accepted at the original 2.5 cm check spacing.\n'
            f'Its 1 mm sampled max |k| shown here: {old_max:.6f} /m.\n'
            f'New and original C3 have different terminal d and curve shape.' if original else
            f'C3 is identically d=0: the reference centreline itself.\n'
            f'Actual vehicle offset = {start["d"]:.6f} m; PP handles tracking.\n'
            f'PP lookahead = {pp_lookahead:g} m; no forced Hermite connection.' if c3_is_reference else
            f'C3 starts at actual d={start["d"]:.6f} m and returns to d=0.\n'
            f'The dashed d=0 reference is a guide, not an actual-start proposal.\n'
            f'PP lookahead = {pp_lookahead:g} m; all five proposals are rejected.')
        accepted = [m['candidate_id'] for m in metrics if m['recorded_valid']]
        result_note = ('Recorded accepted candidates: '+', '.join('C'+str(i) for i in accepted)+'.'
                       if accepted else 'No new candidate passes both columns.')
        table_ax.text(0, .43, result_note+'\n'+comparison_note,
            transform=table_ax.transAxes, fontsize=9, va='top', linespacing=1.7)
        table_ax.text(0, .04, f'Body rectangle: width {2*half_width*100:g} cm / length {2*half_length*100:g} cm.\n'
                      f"Safe band: {report['profile']['centre_prefilter_margin_m']*100:g} cm CENTRE prefilter.\n"
                      f"Full body-edge {report['profile']['requested_body_edge_clearance_m']*100:g} cm clearance is not verified.",
                      transform=table_ax.transAxes, fontsize=9, color='#7b245a', va='bottom')
        fig.suptitle(f'{map_id} | PP={pp_lookahead:g} m | step {step} | s0={start["s"]:.3f} m', fontsize=14)
        fig.tight_layout(rect=[0, 0, 1, .96])
        for extension in ('png', 'svg'):
            fig.savefig(out/f'step_{step}.{extension}', dpi=170)
        plt.close(fig)
        cases.append(dict(step=step, source_pose=pose, start=start, proposals=metrics,
            comparison_kind='original_c3' if original else 'reference_centerline',
            comparison_max_abs_curvature_at_1mm_inv_m=old_max,
            original_c3_recorded_accepted=old['sampled_valid'] if old else None,
            original_c3_target_d_m=old['requested_target_d'] if old else None,
            original_c3_max_abs_curvature_at_1mm_inv_m=old_max if old else None,
            original_c3_1mm_curvature_pass=old_max <= limit+1e-7 if old else None,
            original_accepted_paths_one_mm_curvature_audit=original_audits))
        print(json.dumps(dict(step=step, comparison=comparison_label,
                             c3_max_at_1mm=metrics[2]['exact_max_abs_curvature_inv_m'])), flush=True)
    evidence = dict(source_report_sha256=sha(root/'report.json'), plot_program_sha256=sha(__file__),
                    domain_npz_sha256=sha(Path(domain_dir)/'domain.npz'), cases=cases,
                    source_plans_sha256=sha(session_dir/'plans.jsonl'),
                    pp_lookahead_m=pp_lookahead,
                    scope='read-only visualization; does not change recorded acceptance results')
    (out/'diagnostics.json').write_text(json.dumps(evidence, indent=2, allow_nan=False)+'\n')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--steps', type=int, nargs='+', help='explicit steps, including accepted cases')
    parser.add_argument('--session', help='session subdirectory for a comparison arm')
    args = parser.parse_args()
    run(args.source, args.output_dir, args.steps, args.session)
