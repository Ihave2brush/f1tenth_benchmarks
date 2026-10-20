"""Same QP matrices; solver-parameter-only offline comparison."""
import json
from pathlib import Path
from unittest.mock import patch
import numpy as np
import osqp
from f1tenth_benchmarks.research.lmpc.controller import solve_qp
from f1tenth_benchmarks.research.lmpc.safe_set import select_local_safe_set
from f1tenth_benchmarks.research.lmpc.database import LapDatabase

d=json.loads(Path('Data/research/lmpc/r4_qp_setup_20261014_002/qp_setup.json').read_text())
cfg=json.loads(Path('Data/research/lmpc/b2_esp_observed_20261012_001/database.json').read_text())
db=LapDatabase.load('Data/research/lmpc/b2_esp_observed_20261012_001')
detail=d['attempts'][0]['diagnostic'];nominal=db.laps[-1].states[:13].copy();nominal[0]=d['initial']
inputs=db.laps[-1].commands[:12].copy()
models=[]
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.lmpc.paper_model import PaperLocalModel
geom=load_geometry('maps/frenet/esp/geometry','maps/esp.yaml',inspect=True)
m=PaperLocalModel(db,geom,acceleration_gain_policy='dt_cos_beta_when_unidentified',identification_gate='report_only')
models=[m.linearize(nominal[k],inputs[k]) for k in range(12)]
terminal=select_local_safe_set(db,np.array(detail['candidate']))
captured={}
class Captured(Exception):pass
class Capture:
    def setup(self,**kw):captured.update(kw);raise Captured()
with patch('f1tenth_benchmarks.research.lmpc.controller.osqp.OSQP',Capture):
    try:solve_qp(np.array(d['initial']),nominal,inputs,models,terminal,np.array(d['previous_input']),d['settings'],[1]*12,db.length)
    except Captured:pass
from scipy.optimize import linprog
A=captured['A'].toarray();lo=captured['l'];hi=captured['u'];eq=lo==hi
upper=(~eq)&np.isfinite(hi);lower=(~eq)&np.isfinite(lo)
lp=linprog(np.zeros(A.shape[1]),A_eq=A[eq],b_eq=lo[eq],
    A_ub=np.vstack([A[upper],-A[lower]]),b_ub=np.r_[hi[upper],-lo[lower]],
    bounds=[(None,None)]*A.shape[1],method='highs',
    options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
print('LP feasibility:',lp.message,flush=True)
if lp.success:print('LP residual',float(np.max(np.maximum(lo-A@lp.x,A@lp.x-hi))),flush=True)
result=[]
for rho,adaptive,scaled in [(r,a,s) for r in (.001,.01,.1,1.,10.) for a in (False,True) for s in (False,True)]:
    solver=osqp.OSQP();kw=dict(captured);kw.update(rho=rho,adaptive_rho=adaptive,scaled_termination=scaled)
    solver.setup(**kw);res=solver.solve()
    item=dict(rho=rho,adaptive=adaptive,scaled_termination=scaled,status=res.info.status,
        iterations=res.info.iter,primal=res.info.pri_res,dual=res.info.dua_res,time=res.info.run_time)
    result.append(item);print(item,flush=True)
out=Path('Data/research/lmpc/r4_qp_solver_tuning_20261014_001');out.mkdir(exist_ok=False)
(out/'tuning.json').write_text(json.dumps(result,indent=2))
