"""Revised B2: reuse two initialization laps with an audited periodic terminal tail."""
import argparse
import json
from pathlib import Path

import numpy as np

from .database import LapDatabase, sha256
from .periodic import attach_periodic_tail, check_seam_bounds
from .safe_set import select_local_safe_set, TerminalSupportError, initial_terminal_candidate
from .verify_initialization import verify_initialization


def validate_periodic_database(database):
    lap = database.laps[-1]
    tail = lap.periodic_tail
    if tail is None or not check_seam_bounds(tail.audit, tail.audit['policy'])['passed']:
        raise ValueError('audited periodic tail required')
    overhang = database.metadata['max_candidate_overhang_m']
    end = database.length+overhang
    # Verify the entire stated s interval on a fine grid plus exact seam samples.
    queries = np.unique(np.r_[np.linspace(0., end, 401), database.length,
                              lap.states[-1, 4], tail.states[:20, 4]])
    kinds, q_results = set(), []
    for s in queries:
        candidate = lap.states[12].copy(); candidate[4] = s
        local = select_local_safe_set(database, candidate)
        if local.states.shape != (6, 40):
            raise AssertionError('terminal matrix must have 40 columns')
        for source_lap in database.laps[-2:]:
            pool = source_lap.terminal_pool()
            oracle = sorted(range(len(pool['states'])),
                            key=lambda i:((pool['states'][i, 4]-s)**2, i))[:20]
            actual = [p['local_index'] for p in local.provenance if p['lap_id']==source_lap.lap_id]
            if sorted(actual) != sorted(oracle):
                raise AssertionError('periodic nearest-neighbor oracle mismatch')
        for p in local.provenance:
            if max(p['sample_id'], p['successor_sample_id']) > database.metadata['available_through_row']:
                raise AssertionError('future source reference')
            if p['transition_kind'] != 'observed' and (p['measured_transition'] or p['identification_eligible']):
                raise AssertionError('periodic template misclassified as a new measurement')
            kinds.add(p['transition_kind'])
        np.testing.assert_array_equal(local.costs-local.successor_costs, np.ones(40))
    seed = initial_terminal_candidate(database, cycle=2)
    for s in [database.length-.5, database.length, database.length+.5, end]:
        candidate = lap.states[12].copy(); candidate[4] = s
        local = select_local_safe_set(database, candidate)
        lam = np.zeros(40); lam[9], lam[30] = .35, .65
        q = local.q_function(local.combination(lam))
        if q['value'] > local.cost(lam)+1e-7:
            raise AssertionError('Q-function objective exceeds known feasible cost')
        known = local.q_function(local.states[:, 9])
        if known['value'] > local.costs[9]+1e-7:
            raise AssertionError('known-state Q inconsistent')
        next_candidate = local.advance(q['lambda_weights'])
        np.testing.assert_allclose(next_candidate['previous_cost']-next_candidate['continuation_cost'], 1., atol=1e-8)
        translated = candidate.copy(); translated[4] += 2*database.length
        shifted = select_local_safe_set(database, translated, cycle=2)
        np.testing.assert_allclose(shifted.states[4]-local.states[4], 2*database.length, atol=1e-10)
        np.testing.assert_array_equal(shifted.costs, local.costs)
        q_results.append(dict(candidate_s=float(s), q_value=q['value'],
                              convex_cost=local.cost(lam), equality_residual=q['equality_residual'],
                              known_state_residual=known['equality_residual'],
                              successor_cost_drop=next_candidate['previous_cost']-next_candidate['continuation_cost']))
    # Audited tail is finite: its last node must still fail closed without a successor.
    beyond = lap.states[12].copy(); beyond[4] = lap.terminal_pool()['states'][-1, 4]
    try:
        select_local_safe_set(database, beyond)
    except TerminalSupportError:
        finite_limit_rejected = True
    else:
        raise AssertionError('missing finite-tail successor was silently synthesized')
    return dict(passed=True, stage='B2_periodic_terminal_interfaces',
                historical_source_laps=2, new_observed_laps=0,
                periodic_prefix_steps=len(tail.states), source_prefix_sample_ids=[int(tail.source_sample_ids[0]), int(tail.source_sample_ids[-1])],
                measured_seam=False, exact_dynamics_successor=False,
                seam_audit=tail.audit, terminal_columns=40,
                query_scan=dict(checked=len(queries), supported=len(queries), refused=0,
                                from_s=0., through_s=end, max_candidate_overhang_m=overhang),
                transition_kinds=sorted(kinds), q_function_checks=q_results,
                finite_tail_limit_rejected=finite_limit_rejected,
                full_lap_terminal_support=True,
                full_lap_terminal_support_scope='s-neighbor availability with bounded approximate periodic seam; not MPC feasibility',
                third_lap_initial_terminal_candidate=seed.tolist(),
                third_lap_history_ready=True, b3_controller_ready=False,
                learning_started=False, planning_allowed=False, continuous_verified=False)


def build_periodic(source, output, policy='params/research/lmpc_periodic.yaml'):
    verification = verify_initialization(source)
    observed = LapDatabase.from_b1(source)
    periodic = attach_periodic_tail(observed, source, policy)
    report = validate_periodic_database(periodic)
    for old, new in zip(observed.laps, periodic.laps):
        for key in ['states', 'physical_states', 'commands', 'effective_inputs', 'times', 'sample_ids', 'costs']:
            np.testing.assert_array_equal(getattr(old, key), getattr(new, key))
    periodic.save(output)
    restored = LapDatabase.load(output)
    for original, loaded in zip(periodic.laps, restored.laps):
        a, b = original.terminal_pool(), loaded.terminal_pool()
        for key in ['states', 'costs', 'sample_ids', 'commands', 'effective_inputs']:
            np.testing.assert_array_equal(a[key], b[key])
        if a['transition_kinds'] != b['transition_kinds']:
            raise AssertionError('transition classification lost on load')
    rows = [json.loads(line) for line in (Path(source)/'samples.jsonl').read_text().splitlines()]
    snapshot = dict(stage='B3_resume_seed', source_sample_id=periodic.laps[-1].finish_row,
                    source_geometry_id=periodic.geometry_id, snapshot=rows[periodic.laps[-1].finish_row],
                    restore_requirement='restore original state, steering memory, clocks and replay core lap tracker; no physical reset')
    (Path(output)/'resume_snapshot.json').write_text(json.dumps(snapshot, indent=2, allow_nan=False))
    report.update(source_verification=verification, artifact_roundtrip=True,
                  observed_arrays_unchanged=True, source_hashes=periodic.metadata['source_hashes'],
                  policy_sha256=sha256(policy),
                  code_hashes={name:sha256(Path(__file__).with_name(name)) for name in
                               ['build_periodic.py', 'periodic.py', 'database.py', 'safe_set.py']})
    (Path(output)/'acceptance.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--policy', default='params/research/lmpc_periodic.yaml')
    args = parser.parse_args()
    report = build_periodic(args.source, args.output_dir, args.policy)
    print(json.dumps({key:report[key] for key in
                      ['passed', 'stage', 'query_scan', 'full_lap_terminal_support',
                       'third_lap_history_ready', 'b3_controller_ready', 'q_function_checks']}, indent=2))
