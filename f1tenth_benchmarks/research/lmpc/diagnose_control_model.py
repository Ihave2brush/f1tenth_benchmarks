"""B2.5 offline input contract, independent prediction and frozen QP diagnosis.

Does not authorize a control action or add successful terminal history.
"""
import argparse
import json
from pathlib import Path

import numpy as np

from f1tenth_benchmarks.research.core.geometry import load_geometry
from .config import load_config
from .database import LapDatabase, sha256
from .local_model import LocalModel, augmented, physical_step
from .run_lmpc import load_b3
from .run_smoke import replay


def metrics(errors):
    a = np.asarray(errors)
    return dict(count=len(a), bias=np.mean(a, axis=0).tolist(),
                rmse=np.sqrt(np.mean(a*a, axis=0)).tolist(),
                p95_abs=np.percentile(abs(a), 95, axis=0).tolist(),
                max_abs=np.max(abs(a), axis=0).tolist())


def audit(config_path, output):
    root = Path(output); root.mkdir(parents=True, exist_ok=False)
    config = load_b3(config_path); base = load_config(config['base_config'])
    db = LapDatabase.load(config['database'])
    source = Path(db.metadata['source_directory'])
    rows = [json.loads(s) for s in (source/'samples.jsonl').read_text().splitlines()]
    session = json.loads((source/'session.json').read_text())
    params = session['simulator_params']
    geom = load_geometry(base.adaptation['geometry_path'], base.adaptation['map_yaml'], inspect=True)
    # Fixed engineering budgets before any evaluation: reserve .05m lateral,
    # .06rad heading and .15m progress over 1.2s; not a feasibility theorem.
    policy = dict(stage='B2.5_offline_diagnostic', model_acceptance_scope='PP flying heldout only',
                  columns=['vx','vy','r','e_psi','s','e_y','actual_steering','buffer_0','buffer_1'],
                  one_step_max=[.05,.03,.2,.03,.03,.01,.03200001,1e-12,1e-12],
                  twelve_step_max=[.1,.06,.3,.06,.15,.05,.03200001,1e-12,1e-12],
                  rationale='Reserve .05m lateral and .06rad heading budgets below .25m/.5rad limits; steering budget is one original .01s rate quantum. Local engineering gates, not paper values or nonlinear recursive-feasibility proof.',
                  sampling='second lap every 20th raw transition plus 16 highest absolute core curvature windows and final true crossing; 12 measured inputs, no periodic seam',
                  startup_scope='lap1 is training; startup evaluation is descriptive, not independent acceptance',
                  source_hashes=db.metadata['source_hashes'],
                  config_hash=sha256(config_path), code_hashes={p.name:sha256(p) for p in Path(__file__).parent.glob('*.py')})
    (root/'preregistered_policy.json').write_text(json.dumps(policy, indent=2))
    # Endpoints of train transitions must not include the heldout start state.
    split = db.laps[0].finish_row
    train = np.arange(split-1, dtype=int)
    test = np.arange(split, len(rows)-12, dtype=int)
    assert not set(train).intersection(test)
    assert not set(np.r_[train, train+1]).intersection(np.r_[test, test+12])
    (root/'source_split.json').write_text(json.dumps(dict(training_transition_ids=train.tolist(),
                validation_window_candidates=test.tolist(), shared_boundary_excluded_transition=split-1,
                periodic_identification=False), indent=2))
    full_replay = replay(rows, params, session['start_pose'])
    acceleration_errors=[]; original_errors=[]; rate_values=[]
    for i in range(len(rows)-1):
        before, after = rows[i], rows[i+1]
        assert len(after['physics_ticks']) == 10
        assert abs(after['time']-before['time']-.1)<1e-9
        z=augmented(before['paper_state'], before['physical_state'], before['steering_buffer'])
        ticks=after['physics_ticks']
        avg=sum(t['dt']*t['applied_acceleration'] for t in ticks)/.1
        u=np.array([after['command'][0], avg])
        actual=augmented(after['paper_state'], after['physical_state'], after['steering_buffer'])
        exact=physical_step(z,u,params,.1)
        original_errors.append(exact-actual[[0,1,2,6,7,8]])
        acceleration_errors.append(max(t['applied_acceleration'] for t in ticks)-min(t['applied_acceleration'] for t in ticks))
        rate_values.extend(t['applied_steering_rate'] for t in ticks)
    curves=np.array([abs(geom.reference(rows[i]['paper_state'][4])['kappa']) for i in test])
    chosen=np.unique(np.r_[test[::20], test[np.argsort(curves)[-16:]],test[-1]])
    (root/'validation_windows.json').write_text(json.dumps(chosen.tolist()))
    result=dict(stage='B2.5_offline_diagnostic', passed=False, original_physical_replay=full_replay,
                input_contract=dict(incoming_command_row='i+1', dt=.1, substeps=10, steering_buffer_entries=2,
                    acceleration_average_variation_max=float(max(acceleration_errors)),
                    acceleration_average_is_exact_constant=False,
                    original_constant_average_prior_error=metrics(original_errors),
                    steering_rate_values=np.unique(rate_values).tolist()),
                models={}, startup_independently_validated=False,
                b2_5_complete=False, policy=policy)
    detailed=[]
    for mode in ['smoothed_servo','original_pid','original_pid_geometric']:
        model=LocalModel(db,geom,params,config['settings']['residual_ridge'],mode,train)
        groups={name:{'one':[], 'twelve':[]} for name in ['all','straight','curve','true_finish']}
        condition=[]
        for i in chosen:
            initial=augmented(rows[i]['paper_state'],rows[i]['physical_state'],rows[i]['steering_buffer'])
            value=initial.copy(); first=None; max_error=None
            for k in range(12):
                after=rows[i+k+1]; ticks=after['physics_ticks']
                u=np.array([after['command'][0], sum(t['dt']*t['applied_acceleration'] for t in ticks)/.1])
                coeff,origin,diag=model.identify(value,u)
                assert set(diag['source_ids'])<=set(train)
                value=model.evaluate(value,u,(coeff,origin))
                measured=augmented(after['paper_state'],after['physical_state'],after['steering_buffer'])
                error=value-measured
                if k==0: first=error.copy()
                max_error=abs(error) if max_error is None else np.maximum(max_error,abs(error))
                condition.extend(d['condition_number'] for d in diag['paper_regressors'])
            category='curve' if abs(geom.reference(initial[4])['kappa'])>=.05 else 'straight'
            classes=['all',category]
            if rows[i+12]['paper_state'][4]>=2*db.length: classes.append('true_finish')
            for name in classes:
                groups[name]['one'].append(first); groups[name]['twelve'].append(max_error)
            detailed.append(dict(mode=mode,source_id=int(i),category=category,one_error=first.tolist(),
                                 twelve_max_abs=max_error.tolist(),twelve_final_error=error.tolist()))
        summary={name:{key:metrics(errors) for key,errors in group.items()} for name,group in groups.items() if group['one']}
        one=np.asarray(summary['all']['one']['max_abs']); twelve=np.asarray(summary['all']['twelve']['max_abs'])
        result['models'][mode]=dict(metrics=summary, heldout_flying_passed=bool(np.all(one<=policy['one_step_max']) and np.all(twelve<=policy['twelve_step_max'])),
                                  paper_regressor_condition_max=float(max(condition)),training_count=len(train),validation_window_count=len(chosen))
    (root/'prediction_windows.json').write_text(json.dumps(detailed, indent=2))
    (root/'offline_audit.json').write_text(json.dumps(result, indent=2,allow_nan=False))
    print(json.dumps(dict(original_replay=full_replay,models={k:dict(passed=v['heldout_flying_passed'],one_max=v['metrics']['all']['one']['max_abs'],twelve_max=v['metrics']['all']['twelve']['max_abs']) for k,v in result['models'].items()}),indent=2),flush=True)
    return result


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config',default='params/research/lmpc_b3.yaml'); p.add_argument('--output-dir',required=True)
    a=p.parse_args(); audit(a.config,a.output_dir)


if __name__=='__main__': main()
