"""Build and validate B2 artifacts from independently verified B1 evidence."""
import argparse
import json
from pathlib import Path

import numpy as np

from .database import LapDatabase, sha256
from .safe_set import (TerminalSupportError, initial_terminal_candidate,
                       select_local_safe_set)
from .verify_initialization import verify_initialization


def validate_database(database):
    results = []
    latest = database.laps[-1]
    continuous = database.metadata.get('terminal_extension_mode') == 'observed_continuous_laps'
    overhang = 12*database.dt*8 if continuous else 0.
    queries = np.linspace(latest.states[12, 4], database.length+overhang, 423 if continuous else 101)
    supported, refused = [], []
    for s in queries:
        candidate = latest.states[12].copy()
        candidate[4] = s
        try:
            local = select_local_safe_set(database, candidate)
        except TerminalSupportError as exc:
            refused.append(dict(s=float(s), reason=str(exc)))
            continue
        supported.append(float(s))
        for lap in database.laps[-2:]:
            chosen = [p['local_index'] for p in local.provenance if p['lap_id']==lap.lap_id]
            oracle = sorted(range(len(lap.states)),
                            key=lambda i:((lap.states[i, 4]-s)**2, int(lap.sample_ids[i])))[:20]
            if sorted(chosen) != sorted(oracle):
                raise AssertionError('nearest-neighbor selection disagrees with independent oracle')
        if local.states.shape != (6, 40):
            raise AssertionError('expected 20 columns from each of two laps')
        np.testing.assert_array_equal(local.costs-local.successor_costs, np.ones(40))
    seed = initial_terminal_candidate(database)
    for s in [seed[4], database.length*.25, database.length*.5, database.length*.75]:
        candidate = seed.copy(); candidate[4] = s
        local = select_local_safe_set(database, candidate)
        lam = np.zeros(40); lam[9], lam[30] = .35, .65
        terminal = local.combination(lam)
        q = local.q_function(terminal)
        known = local.q_function(local.states[:, 9])
        if known['value'] > local.costs[9]+1e-7:
            raise AssertionError('known successful state has inconsistent Q-function cost')
        if q['value'] > local.cost(lam)+1e-7:
            raise AssertionError('Q-function did not minimize feasible stored cost')
        advance = local.advance(q['lambda_weights'])
        np.testing.assert_allclose(advance['previous_cost']-advance['continuation_cost'], 1., atol=1e-9)
        shifted = select_local_safe_set(database, candidate+np.array([0, 0, 0, 0, database.length, 0]), cycle=1)
        np.testing.assert_allclose(shifted.states[4]-local.states[4], database.length, atol=1e-10)
        np.testing.assert_array_equal(shifted.costs, local.costs)
        results.append(dict(candidate_s=float(s), q_value=q['value'],
                            convex_cost=local.cost(lam), equality_residual=q['equality_residual'],
                            equality_rank=q['equality_rank'],
                            known_state_q_value=known['value'], known_state_stored_cost=float(local.costs[9]),
                            known_state_residual=known['equality_residual'],
                            advanced_s=float(advance['candidate'][4])))
    if not supported or (continuous and refused) or (not continuous and not refused):
        raise AssertionError('terminal support disagrees with observed continuation mode')
    laps = [dict(lap_id=lap.lap_id, start_row=lap.start_row, finish_row=lap.finish_row,
                 terminal_index=lap.terminal_index, stored_states=len(lap.states),
                 observed_extension_states=len(lap.states)-lap.terminal_index-1,
                 discrete_duration_s=lap.terminal_index*lap.dt,
                 interpolated_lap_time_s=lap.event['duration'],
                 min_cost_steps=float(lap.costs[-1]), final_successor_available=False)
            for lap in database.laps]
    return dict(passed=True, stage='B2_data_and_terminal_interfaces', laps=laps,
                initial_terminal_candidate=seed.tolist(),
                terminal_columns=40, points_per_lap=20, history_laps=2,
                query_scan=dict(checked=len(queries), supported=len(supported), refused=len(refused),
                                last_supported_s=max(supported), refused_queries=refused),
                latest_lap_final_window_boundary_s=float((latest.states[-1, 4]+latest.states[-21, 4])/2),
                q_function_checks=results,
                full_lap_terminal_support=continuous,
                max_candidate_overhang_m=overhang,
                next_required='paper six-state prediction validation' if continuous else 'observe actual successful continuation for lap 2 before its finish window is used',
                learning_started=False, planning_allowed=False, continuous_verified=False)


def build_database(source, output):
    verification = verify_initialization(source)
    database = LapDatabase.from_b1(source)
    report = validate_database(database)
    # Check the availability frontier independently: before lap 2 finishes it is not eligible.
    prefix = LapDatabase.from_b1(source, database.laps[-1].finish_row-1)
    if len(prefix.laps) != 1 or prefix.laps[0].sample_ids[-1] != prefix.laps[0].finish_row:
        raise AssertionError('future successful lap leaked into prefix database')
    try:
        select_local_safe_set(prefix, prefix.laps[0].states[12])
    except TerminalSupportError:
        report['availability_cutoff_verified'] = True
    else:
        raise AssertionError('uncompleted second lap became eligible')
    database.save(output)
    loaded = LapDatabase.load(output)
    for original, restored in zip(database.laps, loaded.laps):
        for key in ['states', 'physical_states', 'commands', 'effective_inputs', 'times', 'sample_ids', 'costs']:
            np.testing.assert_array_equal(getattr(original, key), getattr(restored, key))
    report.update(source_verification=verification, artifact_roundtrip=True,
                  source_hashes=database.metadata['source_hashes'],
                  code_hashes={p.name:sha256(p) for p in [Path(__file__),
                               Path(__file__).with_name('database.py'), Path(__file__).with_name('safe_set.py')]})
    (Path(output)/'acceptance.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source')
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    report = build_database(args.source, args.output_dir)
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
