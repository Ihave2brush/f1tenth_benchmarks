"""Same-control actual laps comparing cooperative C2/C4 B-spline budgets."""
import argparse
from contextlib import redirect_stdout
from copy import deepcopy
from dataclasses import asdict
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline

from .core.geometry import load_geometry
from .overtaking.millimetre_domain import MillimetreDomain, vehicle_clearance_profile
from .overtaking.mm_bspline_repair import BudgetedDomainGenerator
from .overtaking.session import SessionConfig
from .overtaking.verify_session import verify_session
from .overtaking.geometry_queries import ExactQueryContext
from .overtaking.robust_adaptive_paths import _validate, _reference_data, query_road_sections
from .run_mm_lap_comparison import summarize_session
from .run_centerline_candidates import run_session
from .run_fixed_grid_benchmark import audit_legacy_grid
from .run_millimetre_domain import sha
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file


def run(output, domain_dir, budgets, map_id="my_map", build_domain=False, repair_candidates=(2,4), compare_outer=False):
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    geometry_dir, map_yaml = f"maps/frenet/{map_id}/geometry", f"maps/{map_id}.yaml"
    geometry = load_geometry(geometry_dir, map_yaml, inspect=True)
    profile = vehicle_clearance_profile(load_parameter_file('simulator_params'))
    config = SessionConfig(safety_margin_m=profile['centre_prefilter_margin_m'],
                           query_mode='exact', generation_mode='direct_only').for_map(map_id)
    report = dict(completed=False, map=map_id, config=asdict(config), profile=profile,
        code_sha256={str(p):sha(p) for p in [Path(__file__),
            Path(__file__).with_name('overtaking')/'mm_bspline_repair.py']}, arms=[],
        deadline_policy='cooperative callback checks, no hard native interruption or end-to-end deadline',
        candidate_control_enabled=False, footprint_verified=False)
    def save():
        (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    save()
    if build_domain:
        def progress(row):
            print(json.dumps(row), flush=True)
        domain = MillimetreDomain.build(geometry, margin=profile['centre_prefilter_margin_m'], progress=progress)
        domain.save(domain_dir)
    domain = MillimetreDomain.load(domain_dir, geometry, expected_margin=profile['centre_prefilter_margin_m'])
    report['domain'] = domain.metadata
    report['domain_dir'] = str(domain_dir)
    report['domain_npz_sha256'] = sha(Path(domain_dir)/'domain.npz')
    report['domain_file_bytes'] = (Path(domain_dir)/'domain.npz').stat().st_size
    save()
    print(json.dumps(dict(stage='domain_ready', metadata=domain.metadata)), flush=True)
    all_events = {}
    policies = [(2,4),(1,2,4,5)] if compare_outer else [tuple(repair_candidates)]
    report['repair_policies'] = [list(ids) for ids in policies]
    runs = [(b, ids) for b in budgets for ids in policies]
    for budget_ms, ids in runs:
        name = 'budget_'+str(int(budget_ms))+'ms'+('_c'+''.join(map(str,ids)) if compare_outer else '')
        print(json.dumps(dict(stage='lap_start', arm=name)), flush=True)
        generator = BudgetedDomainGenerator(domain, budget_s=budget_ms/1000, repair_candidates=ids)
        meta = dict(policy=domain.metadata['policy'], domain_dir=domain_dir,
            target_policy='reference_centerline', repair_budget_ms=budget_ms, repair_candidates=list(ids),
            validation_scope='reference_C3_plus_optional_budgeted_bspline_C2_C4_no_body_footprint')
        with (root/(name+'.log')).open('x') as log, redirect_stdout(log):
            result = run_session(root/name, config, geometry_dir, map_yaml, generator=generator, experimental_metadata=meta)
        verification = verify_session(root/name, require_complete=result['recording_complete'])
        (root/(name+'_verification.json')).write_text(json.dumps(verification, indent=2, allow_nan=False)+'\n')
        events = [json.loads(s) for s in (root/name/'plans.jsonl').read_text().splitlines()]
        all_events[name] = events
        repairs = [e['result']['bspline_repair'] for e in events if e.get('result',{}).get('bspline_repair')]
        arm = dict(name=name, budget_ms=budget_ms, repair_candidates=list(ids), verification_passed=verification['passed'],
            repaired_instances=sum(len(r['repaired_ids']) for r in repairs),
            requested_instances=sum(len(r['requested_ids']) for r in repairs),
            timed_out_instances=sum(len(r['timed_out_ids']) for r in repairs),
            reference_valid_count=sum(e['paths'][2]['sampled_valid'] for e in events),
            valid_id_counts={str(cid):sum(e['paths'][cid-1]['sampled_valid'] for e in events) for cid in range(1,6)},
            steps_34_35=[dict(step=e['source_control_step'], valid_ids=[p['candidate_id'] for p in e['paths'] if p['sampled_valid']],
                repaired=e['result']['bspline_repair']['repaired_ids'], wall_time_s=e['wall_time_s'])
                for e in events if e['source_control_step'] in (34,35) and e.get('result',{}).get('bspline_repair')], **summarize_session(root/name))
        report['arms'].append(arm)
        save()
        print(json.dumps({k:v for k,v in arm.items() if k not in ('session_summary','rejected_reasons')}), flush=True)
    report['same_control_comparison_performed'] = len(report['arms']) > 1
    report['identical_states'] = (len({sha(root/a['name']/'states.jsonl') for a in report['arms']}) == 1
                                   if len(report['arms']) > 1 else None)
    report['identical_actions'] = (len({sha(root/a['name']/'actions.jsonl') for a in report['arms']}) == 1
                                    if len(report['arms']) > 1 else None)
    if len(report['arms']) > 1:
        assert report['identical_states'] and report['identical_actions']
    baseline = all_events[report['arms'][0]['name']]
    # Every unrepaired path, including C3, must remain byte-equivalent as data.
    for name, events in all_events.items():
        for original, event in zip(baseline, events):
            for previous, path in zip(original['paths'], event['paths']):
                if (path.get('generation_route') != 'budgeted_bspline_repair'
                        and previous.get('generation_route') != 'budgeted_bspline_repair'):
                    assert path == previous
    report['unrepaired_paths_unchanged'] = True if len(report['arms']) > 1 else None
    report['unchanged_check_scope'] = 'paths unrepaired in both compared arms'
    audit_geometry = load_geometry(geometry_dir, map_yaml, inspect=True)
    zero_baseline_present = any(a['budget_ms'] == 0 for a in report['arms'])
    audit_failures = []
    for arm in report['arms']:
        rows = []
        for event in all_events[arm['name']]:
            check_event = deepcopy(event)
            for path in check_event['paths']:
                # Audit the baseline once, and all added curves in every arm.
                if zero_baseline_present and arm['budget_ms'] != 0 and path.get('generation_route') != 'budgeted_bspline_repair':
                    path['sampled_valid'] = False
            row = audit_legacy_grid(audit_geometry, check_event, config)
            rows.append(row)
            if not row['passed']:
                audit_failures.append(dict(arm=arm['name'], step=event['source_control_step'], **row))
            if len(rows) % 100 == 0:
                print(json.dumps(dict(stage='audit_progress', arm=arm['name'], done=len(rows), total=len(all_events[arm['name']]), failed=len(audit_failures))), flush=True)
        arm['original_density_audit'] = dict(checked=sum(r['checked_count'] for r in rows),
            failed=sum(not p['passed'] for r in rows for p in r['paths']))
        print(json.dumps(dict(stage='audit', arm=arm['name'], result=arm['original_density_audit'])), flush=True)
    report['original_density_failures'] = audit_failures
    save()
    # Matched input at the two previously problematic steps, exact reference
    # every <=1 mm for every repaired curve, beyond the timed normal work.
    dense_rows = []
    eligible = [e for e in baseline if e.get('result',{}).get('start')]
    if not eligible:
        dense_steps = []
    elif map_id == 'my_map':
        dense_steps = [step for step in (34,35) if any(e['source_control_step']==step for e in baseline)]
    else:
        # Cover sparse availability, peak accepted curvature, solver work,
        # worst planning time and the wrap seam, plus any exact-audit failures.
        def peak(event):
            return max((p.get('max_abs_curvature_inv_m',0.) for p in event['paths'] if p['sampled_valid']), default=0.)
        dense_steps = sorted(set([min(eligible,key=lambda e:e['sampled_valid_count'])['source_control_step'],
            max(eligible,key=peak)['source_control_step'],
            max(eligible,key=lambda e:e['wall_time_s'])['source_control_step'],
            max(eligible,key=lambda e:len(e.get('result',{}).get('bspline_repair',{}).get('repaired_ids',[])))['source_control_step'],
            eligible[-1]['source_control_step'],
            *[r['step'] for r in audit_failures],
            *[max([e for e in events if e.get('result',{}).get('start')], key=peak)['source_control_step'] for events in all_events.values()],
            *[max(events,key=lambda e:len(e.get('result',{}).get('bspline_repair',{}).get('repaired_ids',[])))['source_control_step'] for events in all_events.values()]]))
    report['dense_audit_steps'] = dense_steps
    for step in dense_steps:
        event = next(e for e in baseline if e['source_control_step'] == step)
        start = event['result']['start']
        context = ExactQueryContext(audit_geometry)
        try:
            distance = np.linspace(0., config.lookahead_m, int(np.ceil(config.lookahead_m/.001))+1)
            source = start['s']+distance
            sections = query_road_sections(context.geometry, source, config.safety_margin_m)
            for section in sections:
                section['margin'] = config.safety_margin_m
            k, kp = _reference_data(context.geometry, source)
            for name, events in all_events.items():
                current = next(e for e in events if e['source_control_step'] == step)
                for path in current['paths']:
                    if not path['sampled_valid'] or (map_id=='my_map' and path.get('generation_route') != 'budgeted_bspline_repair'):
                        continue
                    spec = path['spline']
                    coefficients = np.array(spec['coefficients'])
                    basis = BSpline(spec['knots'], np.eye(len(coefficients)), spec['degree'])
                    is_reference = path.get('path_type') == 'reference_centerline'
                    points, invalid, segment = _validate(context.geometry, basis, coefficients, distance,
                        sections, k, kp, path['requested_target_d'], 0., 0. if is_reference else start['d'], 0. if is_reference else start['d_prime'],
                        config.max_slope, config.max_curvature_inv_m)
                    dense_rows.append(dict(arm=name, step=step, candidate_id=path['candidate_id'],
                        passed=invalid is None, reason=None if invalid is None else points[invalid]['reason'],
                        max_abs_curvature_inv_m=max(abs(p['curvature_inv_m']) for p in points), polyline=segment))
        finally:
            context.close()
    report['one_mm_repair_audits'] = dense_rows
    report['dense_audit_scope'] = ('repaired_curves_at_34_35' if map_id=='my_map' else 'all_accepted_curves_at_selected_diagnostic_steps')
    report['all_audits_passed'] = (all(a['original_density_audit']['failed'] == 0 for a in report['arms'])
                                  and all(r['passed'] for r in dense_rows))
    report['all_requested_laps_completed'] = all(a['session_summary']['recording_complete'] for a in report['arms'])
    report['completed'] = True
    save()
    print(json.dumps(dict(stage='complete', all_audits_passed=report['all_audits_passed'])), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', choices=['my_map','esp'], default='my_map')
    parser.add_argument('--repair-candidates', type=int, nargs='+', default=[2,4], choices=[1,2,4,5])
    parser.add_argument('--compare-outer', action='store_true', help='compare inner-only and all four non-reference slots at the same budget')
    parser.add_argument('--build-domain', action='store_true', help='build a new 1 mm domain before the lap')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--domain-dir', required=True)
    parser.add_argument('--budgets-ms', type=float, nargs='+', default=[0.,20.,40.,80.])
    args = parser.parse_args()
    if len(set(args.budgets_ms)) != len(args.budgets_ms) or (len(args.budgets_ms)>1 and args.budgets_ms[0]!=0):
        parser.error('budgets must be unique; multi-arm comparison must start with zero')
    if any(not np.isfinite(b) or b < 0 or int(b) != b for b in args.budgets_ms):
        parser.error('budgets must be nonnegative integer milliseconds')
    run(args.output_dir, args.domain_dir, args.budgets_ms, args.map, args.build_domain, args.repair_candidates, args.compare_outer)
