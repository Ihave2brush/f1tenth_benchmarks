"""Read saved A/B curves, reconstruct their initial guesses, and profile costs."""
from collections import Counter, defaultdict
import json
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy.interpolate import BSpline

from .core.geometry import load_geometry
from .overtaking.millimetre_domain import MillimetreDomain
from .overtaking import mm_bspline_repair as repair


def summary(values):
    return dict(n=len(values), median=float(np.median(values)), p95=float(np.percentile(values, 95)),
                max=float(np.max(values))) if values else dict(n=0)


def curve(path):
    s = path['spline']
    return BSpline(s['knots'], s['coefficients'], s['degree'])


def run():
    root = Path('Data/research/overtaking/aligned_warm_start_20261105_001')
    saved = json.loads((root/'report.json').read_text())
    assert saved['completed'] and saved['all_audits_passed']
    events = {m: [json.loads(s) for s in (root/(m+'_1.jsonl')).read_text().splitlines()]
              for m in ('coefficients', 'aligned')}
    source = Path('Data/research/overtaking/my_map_mm40_outer_20261030_001/validation/report.json')
    original = json.loads(source.read_text())
    geometry = load_geometry('maps/frenet/my_map/geometry', 'maps/my_map.yaml', inspect=True)
    domain = MillimetreDomain.load(original['domain_dir'], geometry, expected_margin=.185)
    length = 3.
    sites = np.linspace(0., length, 301)
    basis = repair._basis(length, 8)
    E = np.stack([basis(0.), basis(0., nu=1), basis(length), basis(length, nu=1)])
    cache = {m: {} for m in events}
    rows, output_rms, iteration_groups = [], [], {}
    for mode, es in events.items():
        groups = defaultdict(lambda: dict(attempts=0, iterations=0))
        for e in es:
            for a in e['result']['bspline_repair']['attempts']:
                group = 'refinement' if a['refinement'] else 'previous_warm' if a['warm_start'] else 'cold'
                groups[group]['attempts'] += 1
                groups[group]['iterations'] += a['iterations']
        iteration_groups[mode] = dict(groups)
    for a, b in zip(events['coefficients'], events['aligned']):
        start = a['result']['start']
        for pa, pb in zip(a['paths'], b['paths']):
            cid = pa['candidate_id']
            if all(p.get('generation_route') == 'budgeted_bspline_repair' for p in (pa, pb)):
                output_rms.append(float(np.sqrt(np.mean((curve(pa)(sites)-curve(pb)(sites))**2))))
            old_a, old_b = cache['coefficients'].get(cid), cache['aligned'].get(cid)
            if cid not in a['result']['bspline_repair']['requested_ids'] or old_a is None or old_b is None:
                continue
            delta = (start['s']-old_a['s']) % domain.length
            if delta >= .5:
                continue
            target = pa['requested_target_d']
            rhs = np.array([start['d'], start['d_prime'], target, 0.])
            ca = np.asarray(old_a['path']['spline']['coefficients'])
            seed_a = ca+E.T@np.linalg.solve(E@E.T, rhs-E@ca)
            seed_b = repair.aligned_previous_seed(
                dict(coefficients=np.asarray(old_b['path']['spline']['coefficients'])),
                start, target, basis, length, delta)
            initial_a, initial_b = basis(sites)@seed_a, basis(sites)@seed_b
            row = dict(step=a['source_control_step'], candidate=cid, delta_s_m=delta,
                       cache_age_steps=a['source_control_step']-old_a['step'],
                       target_change_m=abs(target-old_a['path']['requested_target_d']),
                       seeds_rms_m=float(np.sqrt(np.mean((initial_a-initial_b)**2))))
            if pa.get('generation_route') == 'budgeted_bspline_repair':
                final = curve(pa)(sites)
                previous = curve(old_a['path'])
                overlap = sites <= length-delta
                row.update(previous_overlap_rms_m=float(np.sqrt(np.mean(
                    (previous(sites[overlap]+delta)-final[overlap])**2))),
                    seed_a_to_final_rms_m=float(np.sqrt(np.mean((initial_a-final)**2))),
                    seed_b_to_final_rms_m=float(np.sqrt(np.mean((initial_b-final)**2))))
            rows.append(row)
        for mode, event in [('coefficients', a), ('aligned', b)]:
            for cid in event['result']['bspline_repair']['repaired_ids']:
                cache[mode][cid] = dict(s=event['result']['start']['s'],
                                       step=event['source_control_step'], path=event['paths'][cid-1])
    successes = [r for r in rows if 'seed_a_to_final_rms_m' in r]
    report = dict(scope='saved_first_paired_trial_plus_separate_untimed_deadline_diagnostic',
        iterations=iteration_groups, eligible_previous_seeds=len(rows),
        requested=sum(len(e['result']['bspline_repair']['requested_ids']) for e in events['coefficients']),
        repaired_output_ab_rms_m=summary(output_rms),
        previous_overlap_rms_m=summary([r['previous_overlap_rms_m'] for r in successes]),
        seed_a_to_final_rms_m=summary([r['seed_a_to_final_rms_m'] for r in successes]),
        seed_b_to_final_rms_m=summary([r['seed_b_to_final_rms_m'] for r in successes]),
        aligned_seed_closer_count=sum(r['seed_b_to_final_rms_m'] < r['seed_a_to_final_rms_m'] for r in successes),
        successful_warm_pairs=len(successes),
        cache_age_counts=dict(Counter(r['cache_age_steps'] for r in rows)),
        target_change_m=summary([r['target_change_m'] for r in rows]), rows=rows,
        geometry_metric='Frenet d RMS at same global s; overlap only excludes unknown tail',
        seed_metric='endpoint-projected initial guess versus accepted A spline; conditional on A repair success',
        phase_profiles={})
    # Separate diagnostic: generous deadline, wrappers only collect inclusive times
    # of non-nested stages; these do not replace the recorded 40 ms experiment.
    functions = [(repair, 'fit_once', 'solver'), (repair, 'aligned_previous_seed', 'align_seed'),
                 (repair.RequestValidator, '__init__', 'validator_setup'),
                 (repair.RequestValidator, 'check', 'validation'),
                 (repair.DomainGenerator, '__call__', 'direct_and_projection')]
    originals = []
    active = defaultdict(float)
    def wrapped(fn, key):
        def call(*args, **kwargs):
            started = perf_counter()
            try:
                return fn(*args, **kwargs)
            finally:
                active[key] += perf_counter()-started
        return call
    try:
        for owner, name, key in functions:
            fn = getattr(owner, name)
            originals.append((owner, name, fn))
            setattr(owner, name, wrapped(fn, key))
        for mode in events:
            warm = repair.BudgetedDomainGenerator(domain, budget_s=60., repair_candidates=(1, 2, 4, 5), warm_start_mode=mode)
            for e in events[mode][:5]:
                warm(geometry, *e['source_pose'], lookahead_m=3., max_slope=1.,
                     max_curvature=saved['config']['max_curvature_inv_m'])
            active.clear()
            generator = repair.BudgetedDomainGenerator(domain, budget_s=60., repair_candidates=(1, 2, 4, 5), warm_start_mode=mode)
            started = perf_counter()
            for e in events[mode]:
                generator(geometry, *e['source_pose'], lookahead_m=3., max_slope=1.,
                          max_curvature=saved['config']['max_curvature_inv_m'])
            total = perf_counter()-started
            report['phase_profiles'][mode] = dict(total_ms=total*1000,
                phases_ms={k: v*1000 for k, v in active.items()},
                residual_ms=(total-sum(active.values()))*1000)
    finally:
        for owner, name, fn in originals:
            setattr(owner, name, fn)
    (root/'diagnosis.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({k: v for k, v in report.items() if k != 'rows'}), flush=True)


if __name__ == '__main__':
    run()
