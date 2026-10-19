"""Independent fixed-ATV validation with training-history nominal seeds.

Uses IV-C model construction, not recursive re-identification at predictions.
Historical seeds are an offline test choice, not previous QP optimal solutions.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from f1tenth_benchmarks.research.core.geometry import load_geometry
from .paper_model import PaperLocalModel, PaperIdentificationError, build_atv_horizon, rollout_atv
from .database import LapDatabase,sha256
from .run_lmpc import load_b3
from .config import load_config


def summary(errors):
    if not errors:
        return dict(count=0,bias=None,rmse=None,p95_abs=None,max_abs=None)
    a=np.asarray(errors)
    return dict(count=len(a),bias=np.mean(a,axis=0).tolist(),rmse=np.sqrt(np.mean(a*a,axis=0)).tolist(),
                p95_abs=np.percentile(abs(a),95,axis=0).tolist(),max_abs=np.max(abs(a),axis=0).tolist())


def verify(config_path,output):
    root=Path(output);root.mkdir(parents=True,exist_ok=False)
    config=load_b3(config_path);base=load_config(config['base_config']);db=LapDatabase.load(config['database'])
    rows=[json.loads(s) for s in (Path(db.metadata['source_directory'])/'samples.jsonl').read_text().splitlines()]
    geom=load_geometry(base.adaptation['geometry_path'],base.adaptation['map_yaml'],inspect=True)
    train=np.arange(db.laps[0].finish_row-1,dtype=int)
    test=np.arange(db.laps[1].start_row,db.laps[1].finish_row-11,20,dtype=int)
    test=np.unique(np.r_[test,db.laps[1].finish_row-12])
    assert not set(np.r_[train,train+1]).intersection(np.concatenate([np.arange(i,i+13) for i in test]))
    policy=dict(training_transition_ids=train.tolist(),validation_window_ids=test.tolist(),
                one_max=[.05,.03,.2,.03,.03,.01],twelve_max=[.1,.06,.3,.06,.15,.05],
                rationale='Same predeclared B2.5 budgets: reserve .05m lateral/.06rad heading over1.2s; engineering gates, not nonlinear feasibility proof',
                validation_scope='second flying PP lap only; first standing lap is train; third continuation excluded from independent validation and training',
                config_sha256=sha256(config_path),source_hashes=db.metadata['source_hashes'])
    policy['acceleration_gain_policy']=config['settings'].get('acceleration_gain_policy','strict')
    policy['identification_gate']=config['settings'].get('identification_gate','strict')
    policy.update(prediction_method='fixed_nominal_ATV_equation15',
        nominal_source='first training lap only: nearest progress, clamped to a complete 12-step window',
        nominal_limitation='Offline historical seed, not shifted previous QP optimum; no closed-loop acceptance',
        test_inputs='recorded validation commands: rollout input only, never nominal selection or regression')
    (root/'preregistered_policy.json').write_text(json.dumps(policy,indent=2))
    model=PaperLocalModel(db,geom,train,
        acceleration_gain_policy=config['settings'].get('acceleration_gain_policy','strict'),
        identification_gate=config['settings'].get('identification_gate','strict'))
    first=[];free=[];records=[];ranks=[];failures=[]
    seed_ids=train[train<=train[-1]-11]
    seed_progress=np.array([rows[i]['paper_state'][4] for i in seed_ids])
    for i in test:
        z=np.array(rows[i]['paper_state']);window=[];models=[]
        progress=seed_progress[0]+(z[4]-seed_progress[0])%db.length
        seed=int(seed_ids[np.argmin(abs(seed_progress-progress))])
        nominal=np.array([rows[j]['paper_state'] for j in range(seed,seed+13)])
        nominal[:,4]+=np.round((z[4]-nominal[0,4])/db.length)*db.length
        nominal_inputs=np.array([rows[j+1]['command'] for j in range(seed,seed+12)])
        assert set(range(seed,seed+12))<=set(train)
        try:
            models=build_atv_horizon(model,nominal,nominal_inputs)
            for _,_,_,diag in models:
                assert set(diag['source_ids'])<=set(train)
                ranks.extend(x['rank'] for x in diag['paper_regressors'])
            inputs=np.array([rows[i+k+1]['command'] for k in range(12)])
            predicted=rollout_atv(z,inputs,models)
            actual=np.array([rows[i+k+1]['paper_state'] for k in range(12)])
            window=(predicted-actual).tolist()
        except (ValueError,np.linalg.LinAlgError) as exc:
            failures.append(dict(raw_source_id=int(i),error=str(exc),
                horizon_step=exc.diagnostic.get('horizon_step') if isinstance(exc,PaperIdentificationError) else None,
                failure_kind='unidentifiable_input' if isinstance(exc,PaperIdentificationError) else 'prediction_error',
                identification=exc.diagnostic if isinstance(exc,PaperIdentificationError) else None))
        if window:first.append(window[0]);free.extend(window)
        records.append(dict(raw_source_id=int(i),errors=np.asarray(window).tolist(),
                            nominal_source_ids=list(range(seed,seed+13)),
                            nominal_states=nominal.tolist(),nominal_inputs=nominal_inputs.tolist(),
                            prediction_models=[dict(A=a.tolist(),B=b.tolist(),C=c.tolist(),identification=d)
                                for a,b,c,d in models],
                            kappa=geom.reference(rows[i]['paper_state'][4])['kappa']))
    one=summary(first);twelve=summary(free)
    result=dict(stage='paper_fixed_atv_independent_validation',passed=bool(not failures and len(first)==len(test) and np.all(np.array(one['max_abs'])<=policy['one_max']) and np.all(np.array(twelve['max_abs'])<=policy['twelve_max'])),
                one_step=one,twelve_free=twelve,regression_rank_histogram={str(r):ranks.count(r) for r in sorted(set(ranks))},
                failed_windows=failures,requested_windows=len(test),complete_windows=len(test)-len(failures),
                twelve_metrics_scope='all evaluated steps; incomplete windows explicitly fail acceptance',
                startup_independently_validated=False,full_lap_controller_passed=False,policy=policy,records=records)
    (root/'validation.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    console={k:v for k,v in result.items() if k not in ('records','policy','failed_windows')}
    console['failed_window_count']=len(failures)
    print(json.dumps(console,indent=2),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--config',default='params/research/lmpc_paper.yaml');p.add_argument('--output-dir',required=True)
    a=p.parse_args();verify(a.config,a.output_dir)


if __name__=='__main__':main()
