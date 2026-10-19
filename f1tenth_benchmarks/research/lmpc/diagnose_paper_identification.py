"""Measured-data identifiability audit and isolated original-plant sensitivity.

Counterfactual dynamics copies are diagnostics, never stored successful laps.
No simulator session, planner action, ridge, prior, or sampling-policy change.
"""
import argparse
import json
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from f1tenth_benchmarks.research.core.geometry import load_geometry
from .config import load_config
from .database import LapDatabase, sha256
from .dynamics import TracedDynamics
from .paper_model import PaperLocalModel
from .run_lmpc import load_b3


def regression_information(states, controls, ids, query, count=80):
    distance = np.sum(((states[:, :3]-query[:3])*[.1, 1., 1.])**2, axis=1)
    idx = np.lexsort((ids, distance))[:min(count, len(ids))]
    argument = distance[idx]/10.
    weights = .75*np.maximum(1.-argument**2, 0.)
    design = np.column_stack([states[idx, :3], controls[idx, 1], np.ones(len(idx))])
    weighted = design*np.sqrt(weights[:, None])
    singular = np.linalg.svd(weighted, compute_uv=False)
    threshold = singular[0]*1e-8
    rest = weighted[:, [0, 1, 2, 4]]
    base_singular = np.linalg.svd(rest, compute_uv=False)
    rank = int(np.sum(singular > threshold))
    base_rank = int(np.sum(base_singular > threshold))
    residual = weighted[:, 3]-rest@np.linalg.lstsq(rest, weighted[:, 3], rcond=1e-8)[0]
    return dict(count=len(idx), positive_weights=int(np.count_nonzero(weights)),
                unique_raw_ids=len(np.unique(ids[idx])), source_ids=ids[idx].tolist(),
                singular_values=singular.tolist(), rank=rank, rank_without_acceleration=base_rank,
                acceleration_identifiable=rank > base_rank,
                acceleration_information_norm=float(np.linalg.norm(residual)),
                a_min=float(controls[idx, 1].min()), a_max=float(controls[idx, 1].max()),
                state_ranges=np.ptp(states[idx, :3], axis=0).tolist(),
                unique_feature_rows_rounded_8=len(np.unique(np.round(design, 8), axis=0)))


def velocities(physical):
    return np.array([physical[3]*np.cos(physical[6]), physical[3]*np.sin(physical[6]), physical[5]])


def plant_copy(row, command, params):
    plant = TracedDynamics(SimpleNamespace(**params))
    plant.state = np.asarray(row['physical_state']).copy()
    plant.steer_buffer = np.asarray(row['steering_buffer']).copy()
    plant.input_mode = 'acceleration'
    for _ in range(round(.1/params['timestep'])):
        plant.update_pose(*command)
    return velocities(plant.state), plant.trace


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='params/research/lmpc_paper.yaml')
    parser.add_argument('--output-dir', required=True)
    args = parser.parse_args()
    out = Path(args.output_dir); out.mkdir(parents=True, exist_ok=False)
    cfg = load_b3(args.config); base = load_config(cfg['base_config'])
    db = LapDatabase.load(cfg['database'])
    source = Path(db.metadata['source_directory'])
    rows = [json.loads(line) for line in (source/'samples.jsonl').read_text().splitlines()]
    report = json.loads((source/'session.json').read_text())
    geometry = load_geometry(base.adaptation['geometry_path'], base.adaptation['map_yaml'], inspect=True)
    all_model = PaperLocalModel(db, geometry, diagnostic_only=True)
    train_ids = np.arange(db.laps[0].finish_row-1, dtype=int)
    split_model = PaperLocalModel(db, geometry, train_ids, diagnostic_only=True)
    # Same independently held-out second-lap anchors as prior model validation.
    lap = db.laps[1]
    queries = np.unique(np.r_[np.arange(lap.start_row, lap.finish_row-11, 20), lap.finish_row-12])
    matrix_cases = []
    for model_label, model in [('available_initialization', all_model), ('first_lap_only', split_model)]:
        ranks, unidentified = [], 0
        for i in queries:
            result = regression_information(model.states, model.controls, model.source_ids,
                                             np.array(rows[int(i)]['paper_state']))
            result.update(query_raw_id=int(i), pool=model_label)
            matrix_cases.append(result); ranks.append(result['rank'])
            unidentified += int(not result['acceleration_identifiable'])
        print(model_label, 'queries', len(queries), 'unidentified a', unidentified,
              'rank histogram', {r:ranks.count(r) for r in sorted(set(ranks))}, flush=True)

    errors = []
    for i in all_model.source_ids:
        before, after = rows[int(i)], rows[int(i)+1]
        ticks = after['physics_ticks']
        np.testing.assert_array_equal(ticks[0]['pre_state'], before['physical_state'])
        np.testing.assert_array_equal(ticks[-1]['post_state'], after['physical_state'])
        for tick in ticks:
            np.testing.assert_array_equal(tick['command'], after['command'])
        averaged = sum(t['applied_acceleration']*t['dt'] for t in ticks)/.1
        errors.append(abs(averaged-after['command'][1]))
    informative = all_model.source_ids[np.abs(all_model.controls[:, 1]) > 1e-6]
    startup = [dict(raw_id=int(i), before_v=float(rows[int(i)]['physical_state'][3]),
                    after_v=float(rows[int(i)+1]['physical_state'][3]),
                    command_a=float(rows[int(i)+1]['command'][1])) for i in informative]

    details = []
    for i in [lap.finish_row-20, lap.finish_row-14, lap.finish_row-8, lap.finish_row-2]:
        row = rows[i]; z = np.array(row['paper_state']); u = np.array(rows[i+1]['command'])
        theta, fit = all_model.identify(z, u)
        # Constant-a null direction: infinitely many gain/intercept pairs fit it.
        chosen = np.array(fit['source_ids'], dtype=int)
        features = np.array([rows[j]['paper_state'][:3]+[rows[j+1]['command'][1], 1.] for j in chosen])
        alternative = theta[0].copy(); alternative[3] += .1
        alternative[4] -= .1*np.mean(features[:, 3])
        prediction_difference = float(np.max(abs(features@(alternative-theta[0]))))
        base_velocity, ticks = plant_copy(row, u, report['simulator_params'])
        np.testing.assert_allclose(ticks[-1]['post_state'], rows[i+1]['physical_state'], rtol=0, atol=1e-12)
        plus = u.copy(); plus[1] += .1
        minus = u.copy(); minus[1] -= .1
        vplus, tplus = plant_copy(row, plus, report['simulator_params'])
        vminus, tminus = plant_copy(row, minus, report['simulator_params'])
        gain = (vplus-vminus)/.2
        sizes = [80, 160, 320, 640, len(all_model.source_ids)]
        details.append(dict(raw_id=i, input_columns=['delta','a'], measured_fit=fit,
            counts_counterfactual=[regression_information(all_model.states, all_model.controls,
                                  all_model.source_ids, z, count) for count in sizes],
            indistinguishable_fit=dict(original=theta[0].tolist(), alternative=alternative.tolist(),
                max_measured_prediction_difference=prediction_difference,
                difference_when_command_a_increases_point1=float(.1*(alternative[3]-theta[0,3]))),
            original_plant_counterfactual=dict(scope='isolated ten-physics-tick copies; never inserted into database',
                base_command=u.tolist(), plus_command=plus.tolist(), minus_command=minus.tolist(),
                measured_next_velocities=velocities(np.array(rows[i+1]['physical_state'])).tolist(),
                base_next_velocities=base_velocity.tolist(), plus_next_velocities=vplus.tolist(),
                minus_next_velocities=vminus.tolist(), central_a_gain=gain.tolist(),
                learned_a_gain=[float(theta[0,3]),0.,0.],
                learned_input_gain_coefficients=theta[:,3].tolist(), replay_match=True,
                max_plus_requested_a_error=float(max(abs(t['requested_acceleration']-plus[1]) for t in tplus)),
                max_minus_requested_a_error=float(max(abs(t['requested_acceleration']-minus[1]) for t in tminus)))))

    result = dict(stage='paper_identifiability_diagnosis', scope='measured data audit and isolated plant copies; not closed loop',
        source_hashes=db.metadata['source_hashes'], source_count=len(all_model.source_ids),
        config_hash=sha256(args.config), script_hash=sha256(__file__),
        outgoing_input_alignment_passed=True, actual_a_mean_command_error_max=float(max(errors)),
        nonzero_acceleration_threshold=1e-6, nonzero_acceleration_transitions=startup,
        numerical_rank_cutoff=1e-8, query_count=len(queries), matrix_cases=matrix_cases, detail_cases=details,
        fixed_paper_settings=True, independent_prediction_acceptance_passed=False,
        no_counterfactual_added_to_history=True)
    (out/'diagnosis.json').write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(dict(output=str(out), source_count=len(all_model.source_ids),
        nonzero_a_count=len(informative), informative_transitions=startup,
        measured_input_alignment_passed=True,
        learned_vs_original_plant_a_gain=[dict(raw_id=d['raw_id'], learned=d['original_plant_counterfactual']['learned_a_gain'][0],
                                              actual_copy=d['original_plant_counterfactual']['central_a_gain'][0]) for d in details]), indent=2), flush=True)


if __name__ == '__main__':
    main()
