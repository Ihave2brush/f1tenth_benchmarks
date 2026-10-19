"""Frozen A3 command comparison for A7; no new QP or future state feedback.

Run from the repository root. Uses the M1 recorded original-plant truth and
independently checks the original six-state predictions. This is a diagnostic
comparison; commands outside A7 reachability are explicitly counted.
"""
import argparse
import json
from pathlib import Path
import numpy as np
from f1tenth_benchmarks.research.core.geometry import load_geometry
from .database import LapDatabase,sha256
from .paper_model import PaperLocalModel,build_atv_horizon as baseline_horizon,rollout_atv
from .actuator7_model import Actuator7Model,build_atv_horizon,replay_actuator


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True);args=parser.parse_args()
    out=Path(args.output_dir);out.mkdir(parents=True,exist_ok=False)
    run=Path('Data/research/lmpc/experiments/A3_lmpc_20261015_001')
    m1=Path('Data/research/lmpc/actuator_wls_m1_20261015_001/a3_shadow.json')
    rows=[json.loads(s) for s in (run/'samples.jsonl').read_text().splitlines()]
    records=[json.loads(s) for s in (run/'controls.jsonl').read_text().splitlines()]
    session=json.loads((run/'session.json').read_text());handoff=session['handoff']['raw_sample_id']
    shadows=json.loads(m1.read_text());db=LapDatabase.load(run/'initial_database')
    geom=load_geometry('maps/frenet/esp/geometry','maps/esp.yaml',inspect=True)
    kw=dict(acceleration_gain_policy='dt_cos_beta_when_unidentified',identification_gate='report_only')
    base=PaperLocalModel(db,geom,**kw);model=Actuator7Model(db,geom,**kw)
    source_files=[run/'samples.jsonl',run/'controls.jsonl',m1]
    policy=dict(scope='fixed A3 commands, same nominal six-state query and WLS neighbors; no closed-loop inference',
        sources={str(p):sha256(p) for p in source_files},
        code_hashes={str(p):sha256(p) for p in Path(__file__).parent.glob('*7*.py')},
        limitation='A7 ideal endpoint and continuous mean sensitivities are approximate. Frozen commands may violate .256 reachability; report count.')
    (out/'policy.json').write_text(json.dumps(policy,indent=2))
    results=[]
    for i,(record,truth_record) in enumerate(zip(records,shadows)):
        if 'prediction_models' not in record:continue
        row=rows[handoff+i];initial=np.array(record['actual_state']);commands=np.array(record['controls'])
        nominal=np.array([d['query_state'] for d in record['atv_kernel_support']]+[record['states'][-1]])
        old=baseline_horizon(base,nominal,commands)
        old_pred=rollout_atv(initial,commands,old)
        np.testing.assert_allclose(old_pred,np.array(truth_record['predictions']['command']),atol=2e-5,rtol=0)
        delta=row['physical_state'][2];buffer=np.array(row['steering_buffer']);angles=[delta]
        exact_means=[];exact_ends=[]
        for u in commands:
            delta,buffer,mean=replay_actuator(delta,buffer,u[0]);angles.append(delta)
            exact_means.append(mean);exact_ends.append(delta)
        z=np.column_stack([nominal,angles])
        new=build_atv_horizon(model,z,commands,np.array(row['steering_buffer']))
        for a,b in zip(old,new):assert a[3]['source_ids']==b[3]['source_ids']
        pred=rollout_atv(np.r_[initial,angles[0]],commands,new)
        truth=np.array(truth_record['truth'],dtype=float)
        rmse=lambda p:np.sqrt(np.mean((p[:,:3]-truth[:,:3])**2,axis=0)).tolist()
        result=dict(snapshot=i+1,baseline_velocity_rmse=rmse(old_pred),A7_velocity_rmse=rmse(pred),
            A7_endpoint_angle_rmse_rad=float(np.sqrt(np.mean((pred[:,6]-exact_ends)**2))),
            fixed_commands_outside_reachability=int(np.sum(abs(commands[:,0]-np.array(angles[:-1]))>.256+1e-12)),
            predictions=pred.tolist(),models=[dict(A=a.tolist(),B=b.tolist(),C=c.tolist(),diagnostic=d) for a,b,c,d in new])
        results.append(result)
    (out/'comparison.json').write_text(json.dumps(results,indent=2,allow_nan=False))
    print(json.dumps([{k:v for k,v in r.items() if k not in ('predictions','models')} for r in results],indent=2))

if __name__=='__main__':main()
