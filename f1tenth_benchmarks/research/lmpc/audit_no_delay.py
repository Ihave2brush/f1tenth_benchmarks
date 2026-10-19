"""Audit a completed A8 run: no FIFO delay, six states, fresh data and shadow errors."""
import argparse
from argparse import Namespace
import json
from pathlib import Path
import numpy as np
from .no_delay import NoDelayTracedDynamics
from .database import LapDatabase,sha256
from .paper_model import rollout_atv


def audit(directory):
    root=Path(directory);session=json.loads((root/'session.json').read_text())
    if session.get('dynamics_variant')!='A8_no_delay_six_state':raise ValueError('not A8')
    rows=[json.loads(x) for x in (root/'samples.jsonl').read_text().splitlines()]
    controls=[json.loads(x) for x in (root/'controls.jsonl').read_text().splitlines()]
    db=LapDatabase.load(root/'initial_database')
    init=root/'initialization';source=Path(db.metadata['source_directory'])
    if source.resolve()!=init.resolve():raise ValueError('WLS database is not this fresh initialization')
    for name,digest in db.metadata['source_hashes'].items():
        if sha256(init/name)!=digest:raise ValueError('fresh WLS source hash mismatch')
    count=0;max_rate=0.
    for row in rows:
        for tick in row['physics_ticks']:
            if tick['delayed_target_steering']!=tick['command'][0]:raise ValueError('steering delay detected')
            rate=abs(tick['applied_steering_rate']);max_rate=max(max_rate,rate)
            if rate>3.2+1e-12:raise ValueError('steering rate limit changed')
            count+=1
    horizons=[];handoff=session['handoff']['raw_sample_id']
    for i,control in enumerate(controls):
        if 'prediction_models' not in control:continue
        state=np.array(control['actual_state']);commands=np.array(control['controls'])
        if state.shape!=(6,) or np.asarray(control['states']).shape!=(13,6):raise ValueError('state dimension changed')
        models=[(np.array(m['A']),np.array(m['B']),np.array(m['C']),{}) for m in control['prediction_models']]
        if any(a.shape!=(6,6) or b.shape!=(6,2) for a,b,_,_ in models):raise ValueError('matrix dimension changed')
        for fit in control['identification']:
            if np.asarray(fit['coefficients']).shape!=(3,5):raise ValueError('WLS columns changed')
        plant=NoDelayTracedDynamics(Namespace(**session['simulator_params']))
        plant.state=np.array(rows[handoff+i]['physical_state']);plant.input_mode='acceleration'
        truth=[]
        for command in commands:
            for _ in range(10):plant.update_pose(*command)
            p=plant.state;truth.append([p[3]*np.cos(p[6]),p[3]*np.sin(p[6]),p[5]])
        pred=rollout_atv(state,commands,models)
        horizons.append(dict(planning_index=i+1,velocity_rmse=np.sqrt(np.mean((pred[:,:3]-truth)**2,axis=0)).tolist(),
            prediction=pred[:,:3].tolist(),plant_velocity=truth,
            scope='fixed planned commands, free 12-step ATV versus no-delay original vehicle; velocity only; no geometry/collision acceptance'))
    result=dict(steering_delay_s=0.,all_ticks_current_command=True,physics_ticks=count,max_abs_steering_rate=max_rate,
        prediction_state_dimension=6,regression_columns=5,regression_steering='delta_cmd',fresh_WLS_source=str(source),
        source_hashes_verified=True,executed_actions=sum('actual_one_step_prediction_error' in c for c in controls),
        full_lap_passed=session['passed'],error=session['error'],one_step_rmse=session['prediction_error_rmse'],
        frozen_horizon_comparisons=horizons,
        source_session_hash=sha256(root/'session.json'),audit_code_hash=sha256(__file__))
    (root/'no_delay_audit.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in result.items() if k!='frozen_horizon_comparisons'},indent=2))
    return result

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory');args=p.parse_args();audit(args.directory)
