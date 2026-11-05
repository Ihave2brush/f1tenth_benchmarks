"""Actual MY-map laps with current dimensions, followed by independent audits.

All arms drive the same centreline PP; candidates are inspection-only. This is
synchronous simulation, not a real-time worker/deadline demonstration.
"""
import argparse
from collections import Counter
from contextlib import redirect_stdout
from dataclasses import asdict
import json
import math
from pathlib import Path
from time import perf_counter

import numpy as np

from .core.geometry import load_geometry
from .overtaking.millimetre_domain import MillimetreDomain, vehicle_clearance_profile
from .overtaking.robust_adaptive_paths import direct_hermite_coefficients
from .overtaking.session import SessionConfig, reject_paths
from .overtaking.reference_path import REFERENCE_PATH
from .overtaking.verify_session import verify_session
from .run_centerline_candidates import run_session
from .run_millimetre_domain import GEOMETRY, MAP, exact_start, summary, sha
from .run_fixed_grid_benchmark import audit_legacy_grid
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file


class DomainGenerator:
    def __init__(self, domain, target_policy='reference_centerline'):
        self.domain = domain
        self.target_policy = target_policy

    def __call__(self, geometry, x, y, yaw, **settings):
        start, reason = exact_start(geometry, [x, y, yaw])
        if start is None:
            raise ValueError('Invalid start: '+str(reason))
        length = settings['lookahead_m']
        plan = self.domain.plan(start['s'], start['d'], start['d_prime'], length=length,
            max_slope=settings['max_slope'], max_curvature=settings['max_curvature'],
            target_policy=self.target_policy)
        if not plan['paths']:
            return dict(start=start, paths=reject_paths(plan['reason']), **{k: v for k, v in plan.items() if k != 'paths'})
        # Reconstruct the mathematical start independently, so make_plan's
        # pose gate checks the curve rather than a copied source x/y/yaw.
        reference = geometry.reference(start['s'])
        nx, ny = -reference['tangent'][1], reference['tangent'][0]
        first = dict(x=reference['xy'][0]+start['d']*nx,
            y=reference['xy'][1]+start['d']*ny,
            yaw=reference['psi']+math.atan2(start['d_prime'], 1-reference['kappa']*start['d']))
        paths = []
        for proposal in plan['paths']:
            valid = proposal['experimental_valid']
            target = proposal['target_d_m']
            is_reference = proposal['path_type'] == REFERENCE_PATH
            coefficients = (np.zeros(6) if is_reference else
                            direct_hermite_coefficients(length, start['d'], start['d_prime'], target))
            path_first = dict(x=reference['xy'][0], y=reference['xy'][1], yaw=reference['psi']) if is_reference else dict(first)
            paths.append(dict(proposal, geometry_id=geometry.geometry_id,
                sampled_valid=valid, status='sampled_valid' if valid else 'rejected',
                points=[path_first], requested_target_d=target, endpoint_mode='fixed',
                reference_s_origin=start['s'], reference_d=0. if is_reference else None,
                spline=dict(knots=[0.]*6+[length]*6, degree=5, coefficients=coefficients.tolist()),
                output_representation='analytic_spline_and_single_start_point',
                footprint_verified=False))
        return dict(start=start, paths=paths, **{k: v for k, v in plan.items() if k != 'paths'})


def summarize_session(root):
    saved = json.loads((root/'summary.json').read_text())
    events = [json.loads(line) for line in (root/'plans.jsonl').read_text().splitlines()]
    moving = [r for r in saved['cycle_timings'] if not r['terminal']]
    valid = [e['sampled_valid_count'] for e in events]
    return dict(session_summary=saved, plan_s=summary([e['wall_time_s'] for e in events]),
        complete_cycle_s=summary([r['wall_time_s'] for r in moving]),
        plan_over_100ms=sum(e['wall_time_s'] > .1 for e in events),
        cycle_over_100ms=sum(r['wall_time_s'] > .1 for r in moving),
        plan_count=len(events), nonterminal_cycle_count=len(moving),
        mean_valid_candidates=float(np.mean(valid)), zero_valid_plan_count=sum(n == 0 for n in valid),
        zero_valid_steps=[e['source_control_step'] for e in events if e['sampled_valid_count'] == 0],
        timely_nonzero_plans=sum(e['wall_time_s'] <= .1 and e['sampled_valid_count'] > 0 for e in events),
        rejected_reasons=dict(Counter(p['reason'] for e in events for p in e['paths'] if not p['sampled_valid'])))


def run(output, domain_dir):
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    profile = vehicle_clearance_profile(load_parameter_file('simulator_params'))
    geometry = load_geometry(GEOMETRY, MAP, inspect=True)
    domain = MillimetreDomain.load(domain_dir, geometry, expected_margin=profile['centre_prefilter_margin_m'])
    experimental = dict(policy=domain.metadata['policy'], vehicle_clearance_profile=profile,
        validation_scope='offline_clearance_rectangles_and_sampled_curvature_no_body_footprint',
        domain_dir=domain_dir, domain_npz_sha256=sha(Path(domain_dir)/'domain.npz'),
        output_representation='analytic_spline_and_single_start_point')
    report = dict(completed=False, map='my_map', profile=profile, arms=[],
        params_sha256=sha('params/simulator_params.yaml'),
        code_sha256={str(p): sha(p) for p in (Path(__file__),
            Path(__file__).with_name('run_centerline_candidates.py'),
            Path(__file__).with_name('overtaking')/'millimetre_domain.py')},
        scope='three actual synchronous laps, same centreline PP, candidates never control',
        original_definition='current-code exact + optimized V5.2-style multistart, not historical checkout',
        deadline_enforced=False, candidate_control_enabled=False, footprint_verified=False)
    def save():
        (root/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    for name, query, generation in [('mm', 'exact', 'direct_only'),
                                     ('latest_repair', 'fixed_grid_lookup', 'direct_then_repair'),
                                     ('original_exact', 'exact', 'optimized')]:
        config = SessionConfig(laps=1, safety_margin_m=profile['centre_prefilter_margin_m'],
                               query_mode=query, generation_mode=generation).for_map('my_map')
        print(json.dumps(dict(stage='lap_start', arm=name)), flush=True)
        started = perf_counter()
        with (root/(name+'_progress.log')).open('x') as log, redirect_stdout(log):
            saved = run_session(root/name, config, GEOMETRY, MAP,
                generator=DomainGenerator(domain) if name == 'mm' else None,
                experimental_metadata=experimental if name == 'mm' else None)
        if not saved['recording_complete']:
            raise RuntimeError(name+' did not complete lap: '+saved['end_reason'])
        verification = verify_session(root/name)
        (root/(name+'_verification.json')).write_text(json.dumps(verification, indent=2, allow_nan=False)+'\n')
        arm = dict(name=name, config=asdict(config), initialization_and_lap_and_replay_s=perf_counter()-started,
                   verification_passed=verification['passed'], **summarize_session(root/name))
        report['arms'].append(arm)
        save()
        print(json.dumps(dict(stage='lap_complete', arm=name, plan=arm['plan_s'],
            cycle=arm['complete_cycle_s'], zero_valid=arm['zero_valid_steps'])), flush=True)
    # Deterministic equality is essential: independently run laps, same poses,
    # actions and physical states, not merely the same nominal speed.
    report['identical_states'] = len({sha(root/a['name']/'states.jsonl') for a in report['arms']}) == 1
    report['identical_actions'] = len({sha(root/a['name']/'actions.jsonl') for a in report['arms']}) == 1
    if not report['identical_states'] or not report['identical_actions']:
        raise AssertionError('comparison laps do not have identical source states/actions')
    audit_geometry = load_geometry(GEOMETRY, MAP, inspect=True)
    audits = []
    failures = []
    with (root/'mm'/'plans.jsonl').open() as stream:
        for index, line in enumerate(stream):
            event = json.loads(line)
            check = audit_legacy_grid(audit_geometry, event, SessionConfig(safety_margin_m=profile['centre_prefilter_margin_m']))
            audits.append(dict(step=event['source_control_step'], **check))
            if event['sampled_valid_count'] == 0:
                failures.append(dict(step=event['source_control_step'], pose=event['source_pose'],
                    start=event.get('result', {}).get('start'), paths=event['paths']))
            if index % 20 == 0:
                print(json.dumps(dict(stage='independent_mm_audit', plan=index)), flush=True)
    report['mm_original_density_audits'] = audits
    report['mm_zero_valid_details'] = failures
    report['audit_summary'] = dict(checked=sum(a['checked_count'] for a in audits),
        failures=sum(not p['passed'] for a in audits for p in a['paths']))
    report['mm_boundary_reuse'] = dict(explicit=0, inherited=0)
    with (root/'mm'/'plans.jsonl').open() as stream:
        for line in stream:
            event = json.loads(line)
            for key, source in [('explicit', 'explicit_boundary_checks'), ('inherited', 'inherited_boundary_checks')]:
                report['mm_boundary_reuse'][key] += event.get('result', {}).get(source, 0)
    report['completed'] = True
    save()
    print(json.dumps(dict(stage='complete', audits=report['audit_summary'], identical_states=report['identical_states'])), flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--domain-dir', required=True)
    args = parser.parse_args()
    run(args.output_dir, args.domain_dir)
