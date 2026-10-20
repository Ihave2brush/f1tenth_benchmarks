"""Offline single-QP moving handoff; no plant actions or constraint relaxation."""
import json
from pathlib import Path
import numpy as np
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.lmpc.config import load_config
from f1tenth_benchmarks.research.lmpc.database import LapDatabase,sha256
from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3
from f1tenth_benchmarks.research.lmpc.paper_model import PaperLocalModel
from f1tenth_benchmarks.research.lmpc.paper_controller import PaperLocalLMPC
from f1tenth_benchmarks.research.lmpc.controller import MPCFailure

config_path='params/research/lmpc_paper.yaml'
cfg=load_b3(config_path);base=load_config(cfg['base_config']);db=LapDatabase.load(cfg['database'])
geom=load_geometry(base.adaptation['geometry_path'],base.adaptation['map_yaml'],inspect=True)
rows=[json.loads(s) for s in (Path(db.metadata['source_directory'])/'samples.jsonl').read_text().splitlines()]
model=PaperLocalModel(db,geom,acceleration_gain_policy=cfg['settings']['acceleration_gain_policy'],
    identification_gate=cfg['settings']['identification_gate'])
controller=PaperLocalLMPC(db,model,cfg['settings'])
# Complete observed third PP lap, mapped to the next experiment's s origin.
initial=np.array(rows[-1]['paper_state']);initial[4]-=3*db.length
nominal=db.laps[-1].states[:13].copy();inputs=db.laps[-1].commands[:12].copy()
nominal[0]=initial
controller.set_first_horizon(nominal,inputs,np.array(rows[-1]['command']))
out=Path('Data/research/lmpc/r4_qp_setup_20261014_001');out.mkdir(exist_ok=False)
result=dict(scope='offline_QP_only_no_plant_actions',config_sha256=sha256(config_path),
    settings=cfg['settings'],initial=initial.tolist(),handoff_raw_source_id=len(rows)-1,
    nominal_seed='latest flying history, actual measured state at first point',
    previous_input=controller.previous_input.tolist(),attempts=[])
try:
    action,report=controller.plan(initial)
    result['attempts'].append(dict(solved=True,action=action.tolist(),report=report))
    # Second solve checks IV-C shifting of a real previous QP solution. The
    # initial state is its model prediction, NOT a new physical measurement.
    next_state=np.array(report['states'][1])
    action2,report2=controller.plan(next_state)
    result['attempts'].append(dict(solved=True,input_state_kind='previous_QP_prediction',
        action=action2.tolist(),report=report2))
except MPCFailure as exc:
    result['attempts'].append(dict(solved=False,error=str(exc),diagnostic=exc.diagnostic))
except ValueError as exc:
    result['attempts'].append(dict(solved=False,error=str(exc)))
(out/'qp_setup.json').write_text(json.dumps(result,indent=2,allow_nan=False))
for a in result['attempts']:
    print(json.dumps({k:v for k,v in a.items() if k not in ('report','diagnostic')},indent=2))
    print(json.dumps(a.get('report',{}).get('qp',a.get('diagnostic',{})),default=str)[:2500])
