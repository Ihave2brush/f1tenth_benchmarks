"""Frozen B3 QP diagnosis: offline relaxations never authorize plant actions."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
from unittest.mock import patch

import numpy as np
import osqp
from scipy.optimize import linprog

from f1tenth_benchmarks.research.core.geometry import load_geometry
from .config import load_config
from .controller import LocalLMPC, MPCFailure
from .database import LapDatabase, sha256
from .local_model import LocalModel


def diagnose(source, output):
    old=Path(source); root=Path(output); root.mkdir(parents=True,exist_ok=False)
    session=json.loads((old/'session.json').read_text()); failure=json.loads((old/'failure.json').read_text())
    records=[json.loads(s) for s in (old/'controls.jsonl').read_text().splitlines()]
    previous=next(c for c in reversed(records) if 'failure' not in c)
    base=load_config(session['config']['base_config']); db=LapDatabase.load(session['source_database'])
    geom=load_geometry(base.adaptation['geometry_path'],base.adaptation['map_yaml'],inspect=True)
    # The archived pilot predates the explicit mode field. Its source label
    # documents the smooth servo; do not treat this as an exact-version replay.
    mode=session['config'].get('prediction_value_mode', 'smoothed_servo')
    model=LocalModel(db,geom,session['simulator_params'],session['config']['settings']['residual_ridge'],mode)
    original_solver=osqp.OSQP
    result=dict(scope='offline_only_no_plant_actions_no_terminal_history_update',
                historical_code_hashes_match=all(Path(p).exists() and sha256(p)==h for p,h in session['code_hashes'].items()),
                source_session_sha256=sha256(old/'session.json'),source_failure_sha256=sha256(old/'failure.json'),
                actual_state=failure['actual_state'], terminal_candidate=failure['failure_detail']['candidate'], variants={})
    result['prediction_value_mode']=mode
    result['mode_inferred_from_archived_smooth_servo_label']='prediction_value_mode' not in session['config']
    for mode in ['shifted','reroll_measured']:
        captured=[]
        class Capture:
            def __init__(self): self.solver=original_solver()
            def setup(self,**kw):
                captured.append({k:kw[k] for k in ['P','q','A','l','u']})
                return self.solver.setup(**kw)
            def solve(self): return self.solver.solve()
        settings=deepcopy(session['config']['settings']); settings['warm_start_mode']=mode
        controller=LocalLMPC(db,model,settings)
        controller.previous=(np.array(previous['states']),np.array(previous['controls']))
        controller.previous_input=np.array(previous['controls'][0])
        controller.candidate=np.array(failure['failure_detail']['candidate'])
        with patch('f1tenth_benchmarks.research.lmpc.controller.osqp.OSQP',Capture):
            try:
                action,report=controller.plan(np.array(failure['actual_state']))
                detail=dict(solved=True,action=action.tolist(),qp=report['qp'])
            except MPCFailure as e:
                detail=dict(solved=False,error=str(e),diagnostic=e.diagnostic)
        probes=[]
        for iteration,data in enumerate(captured):
            # Rows: initial9, dynamics108, terminal6, simplex1, lambda40,
            # future-state108, input24. Keep simplex and lambda in every probe.
            a=data['A'].toarray(); l=data['l']; u=data['u']; m=len(l)
            slices={'initial':slice(0,9),'dynamics':slice(9,117),'terminal':slice(117,123),
                    'state_and_trust':slice(164,272),'input':slice(272,296)}
            assert m==296
            statuses={}
            groups=['none',*slices]+['terminal_'+str(c) for c in range(6)]+['input_and_state']
            for group in groups:
                keep=np.ones(m,dtype=bool)
                if group.startswith('terminal_'):
                    keep[117+int(group.split('_')[1])]=False
                elif group=='input_and_state':
                    keep[slices['input']]=False; keep[slices['state_and_trust']]=False
                elif group!='none': keep[slices[group]]=False
                equal=(l==u)&keep; bounded=(l!=u)&keep
                lp=linprog(np.zeros(a.shape[1]),A_ub=np.vstack([a[bounded],-a[bounded]]),
                           b_ub=np.r_[u[bounded],-l[bounded]], A_eq=a[equal],b_eq=l[equal],
                           bounds=[(None,None)]*a.shape[1],method='highs',
                           options={'primal_feasibility_tolerance':1e-8,'dual_feasibility_tolerance':1e-8})
                violation=None if not lp.success else float(np.max(np.maximum(l[keep]-a[keep]@lp.x,a[keep]@lp.x-u[keep])))
                statuses[group]=dict(status=int(lp.status),feasible=bool(lp.success),message=lp.message,
                                     maximum_retained_row_violation=violation)
            probes.append(dict(iteration=iteration,relaxations=statuses))
            np.savez_compressed(root/(mode+'_qp_'+str(iteration)+'.npz'),P=data['P'].toarray(),q=data['q'],A=a,l=l,u=u)
        detail['offline_constraint_removal']=probes
        result['variants'][mode]=detail
    (root/'frozen_qp_diagnosis.json').write_text(json.dumps(result,indent=2,allow_nan=False))
    print(json.dumps({k:dict(solved=v['solved'],error=v.get('error'),relaxations=v['offline_constraint_removal']) for k,v in result['variants'].items()},indent=2),flush=True)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--source',default='Data/research/lmpc/b3_esp_pilot_20261012_002')
    p.add_argument('--output-dir',required=True)
    a=p.parse_args(); diagnose(a.source,a.output_dir)


if __name__=='__main__':main()
