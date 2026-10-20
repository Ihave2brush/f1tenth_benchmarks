"""Offline alternative minimizers of the same measured local WLS problem.

P80/Q/h unchanged. First-lap-only training; no held-out states/targets in fitting.
Nullspace reference coefficients come from identifiable historical local fits,
not a physical prior or ridge. Never used by the production controller.
"""
import json
from pathlib import Path

import numpy as np

from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.lmpc.config import load_config
from f1tenth_benchmarks.research.lmpc.database import LapDatabase, sha256
from f1tenth_benchmarks.research.lmpc.paper_model import PaperLocalModel
from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3


def main():
    cfg=load_b3('params/research/lmpc_paper.yaml');base=load_config(cfg['base_config'])
    db=LapDatabase.load(cfg['database'])
    rows=[json.loads(s) for s in (Path(db.metadata['source_directory'])/'samples.jsonl').read_text().splitlines()]
    geom=load_geometry(base.adaptation['geometry_path'],base.adaptation['map_yaml'],inspect=True)
    train=np.arange(db.laps[0].finish_row-1,dtype=int)
    model=PaperLocalModel(db,geom,train,diagnostic_only=True)
    # Earliest source state with identifiable input supplies each channel seed.
    seed=np.zeros((3,5)); seed_details={}
    for raw_id in np.unique(np.r_[np.arange(20),np.arange(20,len(train),20)]):
        theta,diag=model.identify(np.array(rows[int(raw_id)]['paper_state']))
        for channel,reg in enumerate(diag['paper_regressors']):
            if channel not in seed_details and reg['input_identifiable']:
                seed[channel]=theta[channel]
                seed_details[channel]=dict(query_raw_id=int(raw_id),rank=reg['rank'],
                                           coefficients=theta[channel].tolist(), source_ids=diag['source_ids'])
        if len(seed_details)==3:break
    out=Path('Data/research/lmpc/r3_wls_tie_selection_20261014_001');out.mkdir(parents=True,exist_ok=False)
    if len(seed_details)!=3:
        report=dict(scope='offline only',passed=False,reason='not all input channels have identifiable historical local seeds',
                    seed_details=seed_details,training_ids=train.tolist())
        (out/'comparison.json').write_text(json.dumps(report,indent=2));print(json.dumps(report),flush=True);return
    lap=db.laps[1];query=np.unique(np.r_[np.arange(lap.start_row,lap.finish_row-11,20),lap.finish_row-12])
    assert not set(np.r_[train,train+1]).intersection(np.concatenate([np.arange(i,i+13) for i in query]))
    results=[]
    for mode in ['minimum_norm','historical_seed_nullspace','previous_fit_nullspace']:
        records=[];errors=[];first=[];failures=[];sse_differences=[];relative_design_changes=[]
        for i in query:
            z=np.array(rows[int(i)]['paper_state']);reference=seed.copy();window=[]
            for k in range(12):
                try:
                    theta,diag=model.identify(z)
                    ids=np.asarray(diag['source_ids']);idx=np.searchsorted(model.source_ids,ids)
                    distances=np.sum(((model.states[idx,:3]-z[:3])*[.1,1,1])**2,axis=1)
                    weights=.75*np.maximum(1-(distances/10)**2,0.)
                    for channel,command_index in enumerate([1,0,0]):
                        X=np.column_stack([model.states[idx,:3],model.controls[idx,command_index],np.ones(80)])
                        M=X*np.sqrt(weights[:,None]);y=model.targets[idx,channel]*np.sqrt(weights)
                        old=theta[channel].copy()
                        if mode!='minimum_norm':
                            _,s,V=np.linalg.svd(M,full_matrices=False);rank=int(np.sum(s>s[0]*1e-8))
                            null=V[rank:].T
                            prior=seed[channel] if mode=='historical_seed_nullspace' else reference[channel]
                            theta[channel]+=null@(null.T@prior)
                        old_error=M@old-y;new_error=M@theta[channel]-y
                        sse_differences.append(float(abs(new_error@new_error-old_error@old_error)))
                        relative_design_changes.append(float(np.linalg.norm(M@(theta[channel]-old))/max(np.linalg.norm(y),1e-12)))
                    reference=theta.copy()
                    u=np.array(rows[int(i)+k+1]['command'])
                    z=model.evaluate(z,u,theta)
                    if not np.isfinite(z).all():raise ValueError('nonfinite predicted state')
                    err=z-np.array(rows[int(i)+k+1]['paper_state']);window.append(err)
                except (ValueError,np.linalg.LinAlgError) as exc:
                    failures.append(dict(raw_id=int(i),step=k,reason=str(exc)));break
            if window:first.append(window[0]);errors.extend(window)
            records.append(dict(raw_id=int(i),errors=np.asarray(window).tolist()))
        maximum=np.max(np.abs(errors),axis=0) if errors else np.full(6,np.inf)
        first_max=np.max(np.abs(first),axis=0) if first else np.full(6,np.inf)
        limits=np.array([.1,.06,.3,.06,.15,.05]);one_limits=np.array([.05,.03,.2,.03,.03,.01])
        report=dict(mode=mode,complete_windows=len(query)-len(failures),requested_windows=len(query),
            max_abs_error=maximum.tolist(),one_step_max_abs=first_max.tolist(), failures=failures,
            max_wls_sse_change=max(sse_differences,default=0.),
            max_relative_weighted_prediction_change=max(relative_design_changes,default=0.),
            independent_prediction_passed=bool(not failures and np.all(maximum<=limits) and np.all(first_max<=one_limits)),
            records=records)
        results.append(report)
        print(json.dumps({k:v for k,v in report.items() if k not in ('records','failures')}),flush=True)
    report=dict(scope='offline WLS minimizer selection experiment, not production or closed-loop acceptance',
                source_hashes=db.metadata['source_hashes'],script_hash=sha256(__file__),
                training_ids=train.tolist(),validation_queries=query.tolist(),seed_details=seed_details,
                model_settings='P80, weighted squared state distance, Epanechnikov h10, rcond1e-8',
                numerical_note='discarded directions are numerical nullspace; nonzero design/SSE change must be reported, not claimed exactly zero',
                physical_prior=False,ridge=False,results=results)
    (out/'comparison.json').write_text(json.dumps(report,indent=2,allow_nan=False))


if __name__=='__main__':main()
