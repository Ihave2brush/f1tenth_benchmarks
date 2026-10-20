"""Four observed finish windows with the literal indicator implementation."""
import json
from pathlib import Path
import numpy as np
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.lmpc.config import load_config
from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3
from f1tenth_benchmarks.research.lmpc.database import LapDatabase
from f1tenth_benchmarks.research.lmpc.paper_model import PaperLocalModel
from f1tenth_benchmarks.research.lmpc.paper_controller import PaperLocalLMPC
from f1tenth_benchmarks.research.lmpc.controller import MPCFailure

cfg=load_b3('params/research/lmpc_paper.yaml');base=load_config(cfg['base_config'])
db=LapDatabase.load(cfg['database'])
geom=load_geometry(base.adaptation['geometry_path'],base.adaptation['map_yaml'],inspect=True)
rows=[json.loads(s) for s in (Path(db.metadata['source_directory'])/'samples.jsonl').read_text().splitlines()]
model=PaperLocalModel(db,geom,acceleration_gain_policy=cfg['settings']['acceleration_gain_policy'],
    identification_gate=cfg['settings']['identification_gate'])
records=[];lap=db.laps[-1]
for raw_id in (3916,3922,3928,3934):
    index=int(np.where(lap.sample_ids==raw_id)[0][0])
    nominal=lap.states[index:index+13].copy();inputs=lap.commands[index:index+12].copy()
    controller=PaperLocalLMPC(db,model,cfg['settings'])
    controller.set_first_horizon(nominal,inputs,np.asarray(rows[raw_id]['command']))
    try:
        action,report=controller.plan(nominal[0])
        record=dict(raw_id=raw_id,solved=True,action=action.tolist(),report=report)
    except MPCFailure as exc:
        record=dict(raw_id=raw_id,solved=False,error=str(exc),diagnostic=exc.diagnostic)
    records.append(record)
    print(json.dumps({k:v for k,v in record.items() if k not in ('report','diagnostic')}),flush=True)
    if record['solved']:print(json.dumps(report['qp']),flush=True)
out=Path('Data/research/lmpc/r4_literal_finish_qp_20261014_001');out.mkdir(exist_ok=False)
(out/'validation.json').write_text(json.dumps(dict(scope='offline_only',records=records),indent=2,allow_nan=False))
