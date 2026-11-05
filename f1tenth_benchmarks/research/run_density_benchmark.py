"""Compare original 2.5 cm and nominal control-centerline 5 cm sampling.

The override is experimental: all coarse accepted curves are separately checked
on the unchanged legacy dense grid. Audits never warm measured geometry.
"""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
import platform
import os
import numpy as np
import scipy
from .core.geometry import load_geometry
from .overtaking.session import SessionConfig
from .run_generator_abc import measured_plan
from .run_fixed_grid_benchmark import audit_legacy_grid
from .run_short_horizon_benchmark import fresh_geometry, sha, summarize


def run(inputs, output):
    manifest=json.loads(Path(inputs).read_text())
    root=Path(output);root.mkdir(parents=True,exist_ok=False)
    (root/'inputs.json').write_bytes(Path(inputs).read_bytes())
    paths=[Path(__file__),Path(__file__).with_name('run_generator_abc.py'),
        Path(__file__).with_name('run_fixed_grid_benchmark.py'),
        Path(__file__).with_name('run_short_horizon_benchmark.py'),
        *[Path(__file__).with_name('overtaking')/n for n in
          ('session.py','robust_adaptive_paths.py','geometry_queries.py','road_table.py')]]
    report=dict(schema_version=1,completed=False,input_sha256=sha(inputs),
        code_sha256={str(p):sha(p) for p in paths},geometry_id=manifest['geometry_id'],
        environment=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,
            openblas_threads=os.environ.get('OPENBLAS_NUM_THREADS')),
        scope='make_plan_plus_JSON_equal_stage_hooks_excludes_init_control_disk_quality_and_audit',
        generation_mode='direct_then_repair',deadline_enforced=False,target_s=.1,
        coarse_policy='nominal_5cm_control_centerline_not_original_20cm_centerline',groups=[])
    pending=[]
    def save():
        (root/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    for h in manifest['horizons_m']:
        for step in (.025,.05):
            # Default arm deliberately keeps the old implicit raster-half policy.
            config=SessionConfig(**manifest['settings'],lookahead_m=h,
                query_mode='fast_with_fallback',generation_mode='direct_then_repair',
                validation_step_m=None if step==.025 else step)
            group=dict(horizon_m=h,nominal_step_m=step,config=asdict(config),fixed=[],moving=[])
            report['groups'].append(group)
            geometry,init=fresh_geometry(manifest['geometry_id']);group['fixed_initialization_s']=init
            for case in manifest['fixed_cases']:
                state=dict(control_step=case['control_step'] or 0,sim_time_s=0.,
                    pose=case['pose'],valid=True,frenet=dict(valid=True))
                pairs=[measured_plan(geometry,state,config,i) for i in range(2)]
                rows=[x for x,_ in pairs]
                group['fixed'].append(dict(name=case['name'],runs=rows))
                if step==.05:pending.extend((row,event,config) for row,event in pairs)
            del geometry
            for segment in manifest['moving_segments']:
                geometry,init=fresh_geometry(manifest['geometry_id'])
                pairs=[measured_plan(geometry,item['state'],config,i)
                    for i,item in enumerate(segment['states'])]
                rows=[x for x,_ in pairs]
                group['moving'].append(dict(name=segment['name'],initialization_s=init,runs=rows))
                if step==.05:pending.extend((row,event,config) for row,event in pairs)
                del geometry
                print(json.dumps(dict(horizon_m=h,step_m=step,segment=segment['name'],
                    summary=summarize(rows))),flush=True)
            group['moving_summary']=summarize([x for seg in group['moving'] for x in seg['runs']])
            save()
    # Run all normal timing first. Full exported coarse splines stay available for diagnosis.
    audit_geometry=load_geometry('maps/frenet/esp/geometry','maps/esp.yaml',inspect=True)
    with (root/'coarse_events.jsonl').open('w') as stream:
        for row,event,config in pending:
            row['legacy_grid_audit']=audit_legacy_grid(audit_geometry,event,config)
            stream.write(json.dumps(event,allow_nan=False)+'\n')
    audits=[row['legacy_grid_audit'] for row,_,_ in pending]
    report.update(completed=True,audited_candidate_instances=sum(a['checked_count'] for a in audits),
        audit_failed_candidate_instances=sum(not p['passed'] for a in audits for p in a['paths']),
        all_coarse_legacy_audits_passed=all(a['passed'] for a in audits))
    save()
    print(json.dumps(dict(completed=True,audit_failed_candidate_instances=report['audit_failed_candidate_instances'])),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',default='docs/project/reports/short_horizon_inputs_20261028.json')
    parser.add_argument('--output-dir',required=True)
    args=parser.parse_args();run(args.inputs,args.output_dir)
