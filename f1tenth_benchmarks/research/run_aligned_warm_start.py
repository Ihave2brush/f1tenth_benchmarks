"""Paired MY-map replay: existing coefficient seed versus shifted seed only."""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import time

import numpy as np

from .core.geometry import load_geometry
from .overtaking.millimetre_domain import MillimetreDomain
from .overtaking.mm_bspline_repair import BudgetedDomainGenerator
from .overtaking.session import SessionConfig, make_plan
from .run_fixed_grid_benchmark import audit_legacy_grid


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def stats(values):
    return {name: float(np.percentile(values, q)) for name, q in
            [('median', 50), ('p95', 95), ('max', 100)]}


def run(output):
    root = Path(output)
    root.mkdir(parents=True, exist_ok=False)
    source = Path('Data/research/overtaking/my_map_mm40_outer_20261030_001/validation')
    session = source/'budget_40ms_c1245'
    original = json.loads((source/'report.json').read_text())
    states = [json.loads(s) for s in (session/'states.jsonl').read_text().splitlines()]
    config = SessionConfig(**original['config'])
    geometry = load_geometry('maps/frenet/my_map/geometry', 'maps/my_map.yaml', inspect=True)
    domain = MillimetreDomain.load(original['domain_dir'], geometry,
                                  expected_margin=config.safety_margin_m)
    report = dict(completed=False, map='my_map', config=asdict(config),
        experiment='only_previous_curve_seed_alignment_changes',
        scope='fixed_recorded_states_with_session_gate_not_new_driving_or_full_cycle',
        baseline='existing coefficient warm start, not cold start', budget_ms=40,
        repair_candidates=[1, 2, 4, 5], repeats=3,
        order=[['coefficients', 'aligned'], ['aligned', 'coefficients'], ['coefficients', 'aligned']],
        source_sha256={str(p): sha(p) for p in [session/'states.jsonl', session/'actions.jsonl',
            source/'report.json', Path(original['domain_dir'])/'domain.npz', Path(__file__),
            Path(__file__).with_name('overtaking')/'mm_bspline_repair.py']}, trials=[])
    def save():
        (root/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    save()
    all_events = {}
    # Untimed import/cache warmup; both arms start measured runs with empty history.
    for mode in ('coefficients', 'aligned'):
        warm = BudgetedDomainGenerator(domain, repair_candidates=(1, 2, 4, 5), warm_start_mode=mode)
        for i, state in enumerate(states[:5]):
            make_plan(geometry, state, config, i, generator=warm)
    for repeat, order in enumerate(report['order']):
        for mode in order:
            name = f'{mode}_{repeat+1}'
            generator = BudgetedDomainGenerator(domain, budget_s=.04,
                repair_candidates=(1, 2, 4, 5), warm_start_mode=mode)
            events = []
            cpu, wall = time.process_time(), time.perf_counter()
            for i, state in enumerate(states):
                events.append(make_plan(geometry, state, config, i,
                    trigger='terminal' if i == len(states)-1 else 'periodic', generator=generator))
            cpu_s, wall_s = time.process_time()-cpu, time.perf_counter()-wall
            (root/(name+'.jsonl')).write_text(''.join(json.dumps(e)+'\n' for e in events))
            all_events[name] = events
            repairs = [e['result']['bspline_repair'] for e in events]
            attempts = [a for r in repairs for a in r['attempts']]
            trial = dict(name=name, mode=mode, repeat=repeat+1, calls=len(events),
                plan_ms=stats([e['wall_time_s']*1000 for e in events]),
                generator_ms=stats([r['elapsed_s']*1000 for r in repairs]),
                plan_over_40ms=sum(e['wall_time_s'] > .04 for e in events),
                generator_over_40ms=sum(r['elapsed_s'] > .04 for r in repairs),
                cpu_s=cpu_s, batch_wall_s=wall_s,
                valid_id_counts={str(c): sum(e['paths'][c-1]['sampled_valid'] for e in events)
                                 for c in range(1, 6)},
                mean_valid_candidates=float(np.mean([e['sampled_valid_count'] for e in events])),
                zero_valid=sum(e['sampled_valid_count'] == 0 for e in events),
                requested=sum(len(r['requested_ids']) for r in repairs),
                repaired=sum(len(r['repaired_ids']) for r in repairs),
                timed_out=sum(len(r['timed_out_ids']) for r in repairs),
                logged_attempts=len(attempts),
                logged_iterations=sum(a['iterations'] for a in attempts),
                warm_initial_attempts=sum(a['warm_start'] and a['refinement'] == 0 for a in attempts),
                iteration_note='completed/logged attempts only; timeout-interrupted attempts omitted')
            report['trials'].append(trial)
            save()
            print(json.dumps(trial), flush=True)
    # C3 and unrepaired direct curves must stay equal under the seed-only change.
    for repeat in range(3):
        baseline = all_events[f'coefficients_{repeat+1}']
        aligned = all_events[f'aligned_{repeat+1}']
        for a, b in zip(baseline, aligned):
            assert a['source_pose'] == b['source_pose'] and a['paths'][2] == b['paths'][2]
            for pa, pb in zip(a['paths'], b['paths']):
                if all(p.get('generation_route') != 'budgeted_bspline_repair' for p in (pa, pb)):
                    assert pa == pb
    report['reference_and_unrepaired_paths_unchanged'] = True
    audit_geometry = load_geometry('maps/frenet/my_map/geometry', 'maps/my_map.yaml', inspect=True)
    for trial in report['trials']:
        checked, failures = 0, []
        for event in all_events[trial['name']]:
            audit = audit_legacy_grid(audit_geometry, event, config)
            checked += audit['checked_count']
            if not audit['passed']:
                failures.append(dict(step=event['source_control_step'], audit=audit))
        trial['independent_audit'] = dict(checked=checked, failures=failures)
        save()
        print(json.dumps(dict(stage='audit', trial=trial['name'], checked=checked,
                              failures=len(failures))), flush=True)
    report['all_audits_passed'] = all(not t['independent_audit']['failures'] for t in report['trials'])
    report['completed'] = True
    save()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    run(parser.parse_args().output)
