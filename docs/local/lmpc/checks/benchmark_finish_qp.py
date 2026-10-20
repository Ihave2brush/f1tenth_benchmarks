"""Repeat fixed observed windows; compare correctness and separate timings.

Plant shadow uses a copy only; no live simulator action or learning update.
"""
import json
from pathlib import Path
from time import perf_counter
import numpy as np
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.lmpc.config import load_config
from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3
from f1tenth_benchmarks.research.lmpc.database import LapDatabase,sha256
from f1tenth_benchmarks.research.lmpc.paper_model import PaperLocalModel
from f1tenth_benchmarks.research.lmpc.paper_controller import PaperLocalLMPC
from f1tenth_benchmarks.research.lmpc.planning_scope import ScopedPlanningGate
from f1tenth_benchmarks.research.lmpc.validation import FootprintChecker,CandidateValidator

cfg=load_b3('params/research/lmpc_paper.yaml');base=load_config(cfg['base_config']);db=LapDatabase.load(cfg['database'])
geom=load_geometry(base.adaptation['geometry_path'],base.adaptation['map_yaml'],inspect=True)
source=Path(db.metadata['source_directory'])
rows=[json.loads(s) for s in (source/'samples.jsonl').read_text().splitlines()]
params=json.loads((source/'session.json').read_text())['simulator_params']
footprint=FootprintChecker(geom.raster,params['vehicle_length'],params['vehicle_width'],cfg['settings']['footprint_margin_m'])
validator=CandidateValidator(geom,footprint,8.,.4189,9.51)
gate=ScopedPlanningGate(geom,validator,params,cfg['settings'])
model=PaperLocalModel(db,geom,acceleration_gain_policy=cfg['settings']['acceleration_gain_policy'],
    identification_gate=cfg['settings']['identification_gate'])
baseline_path=Path('Data/research/lmpc/r4_literal_finish_qp_20261014_002/validation.json')
controller_hash=sha256('f1tenth_benchmarks/research/lmpc/controller.py')
baseline={r['raw_id']:r for r in json.loads(baseline_path.read_text())['records']}
out=Path('Data/research/lmpc/r4_qp_speed_benchmark_20261014_001');out.mkdir(exist_ok=False)
records=[];lap=db.laps[-1]
for raw_id in (3916,3922,3928,3934):
    index=int(np.where(lap.sample_ids==raw_id)[0][0])
    nominal=lap.states[index:index+13].copy();inputs=lap.commands[index:index+12].copy()
    runs=[]
    for repeat in range(5):
        controller=PaperLocalLMPC(db,model,cfg['settings'])
        controller.set_first_horizon(nominal,inputs,np.asarray(rows[raw_id]['command']))
        started=perf_counter();action,report=controller.plan(nominal[0]);model_qp=perf_counter()-started
        permit=gate.check(np.asarray(rows[raw_id]['physical_state']),np.asarray(rows[raw_id]['steering_buffer']),
            np.asarray(report['states']),np.asarray(report['controls']))
        total=perf_counter()-started;qp=report['qp'][0]
        runs.append(dict(repeat=repeat,qp=qp,model_qp_s=model_qp,
            model_qp_first_action_validation_s=total,planning_gate=permit,
            first_input_max_difference=float(np.max(abs(action-np.asarray(baseline[raw_id]['action'])))),
            objective_difference=float(qp['objective_with_stage_and_rate']-
                baseline[raw_id]['report']['qp'][0]['objective_with_stage_and_rate'])))
    record=dict(raw_id=raw_id,baseline_qp_s=baseline[raw_id]['report']['qp'][0]['finish_total_s'],runs=runs)
    for label,values in [('qp',[r['qp']['finish_total_s'] for r in runs]),
                         ('model_qp',[r['model_qp_s'] for r in runs]),
                         ('full_decision',[r['model_qp_first_action_validation_s'] for r in runs])]:
        record[label+'_timing']=dict(median_s=float(np.median(values)),max_s=float(max(values)),
                                    all_under_100ms=bool(max(values)<=.1))
    record['equivalent_within_gates']=all(abs(r['objective_difference'])<1e-5 and r['first_input_max_difference']<1e-5
        and max(r['qp']['constraint_violation'],r['qp']['terminal_residual'])<=cfg['settings']['equality_gate'] for r in runs)
    record['planning_gate_pass_count']=sum(r['planning_gate']['valid'] for r in runs)
    records.append(record)
    print(json.dumps({k:v for k,v in record.items() if k!='runs'},indent=2),flush=True)
result=dict(scope='four recorded windows, five repeats each, offline only',records=records,
    baseline_sha256=sha256(baseline_path),config_sha256=sha256('params/research/lmpc_paper.yaml'),
    controller_sha256=controller_hash,
    controller_unchanged_during_benchmark=controller_hash==sha256('f1tenth_benchmarks/research/lmpc/controller.py'),
    full_lap_verified=False,code_change_scope='QP scheduling and equivalent constraint handling only')
(out/'benchmark.json').write_text(json.dumps(result,indent=2,allow_nan=False))
