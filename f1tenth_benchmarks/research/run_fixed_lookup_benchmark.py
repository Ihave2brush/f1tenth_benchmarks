"""Fixed-road inverse reuse without shifting candidate validation sites."""
import argparse
from dataclasses import asdict
import json
import os
from pathlib import Path
import platform
from time import perf_counter
import numpy as np
import scipy
from .core.geometry import load_geometry
from .overtaking.road_table import get_road_table
from .overtaking.session import SessionConfig
from .overtaking.verify_session import verify_session
from .run_centerline_candidates import run_session
from .run_fixed_grid_benchmark import audit_legacy_grid
from .run_generator_abc import measured_plan
from .run_short_horizon_benchmark import sha,summarize

GEOMETRY='maps/frenet/my_map/geometry'
MAP_YAML='maps/my_map.yaml'
MODES=('fast_with_fallback','fixed_grid_with_fallback','fixed_grid_lookup')


def fresh():
    started=perf_counter();g=load_geometry(GEOMETRY,MAP_YAML,inspect=True)
    get_road_table(g,GEOMETRY)
    return g,perf_counter()-started


def run(inputs,output):
    root=Path(output);root.mkdir(parents=True,exist_ok=False)
    manifest=json.loads(Path(inputs).read_text())
    assert manifest['map']=='my_map' and manifest['lookahead_m']==3.
    (root/'inputs.json').write_bytes(Path(inputs).read_bytes())
    sources=[Path(__file__),Path(__file__).with_name('run_generator_abc.py'),
        Path(__file__).with_name('run_short_horizon_benchmark.py'),
        Path(__file__).with_name('run_fixed_grid_benchmark.py'),
        Path(__file__).with_name('run_centerline_candidates.py'),
        *[Path(__file__).with_name('overtaking')/n for n in ('robust_adaptive_paths.py',
            'geometry_queries.py','road_table.py','fixed_grid.py','fixed_arc_lookup.py',
            'session.py','verify_session.py')]]
    report=dict(schema_version=1,completed=False,input_sha256=sha(inputs),
        map='my_map',lookahead_m=3.,geometry_id=manifest['geometry_id'],deadline_enforced=False,target_s=.1,
        scope='make_plan_plus_JSON_equal_hooks_excludes_init_disk_quality_audit_and_two_lap_session',
        code_sha256={str(p):sha(p) for p in sources},groups=[],
        environment=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,
            openblas_threads=os.environ.get('OPENBLAS_NUM_THREADS')))
    def save():(root/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    pending=[]
    for mode in MODES:
        config=SessionConfig(query_mode=mode,generation_mode='direct_then_repair')
        assert config.lookahead_m==3.
        group=dict(query_mode=mode,config=asdict(config),fixed=[],moving=[]);report['groups'].append(group)
        geometry,init=fresh();assert geometry.geometry_id==manifest['geometry_id']
        group['fixed_initialization_s']=init
        for case in manifest['fixed_cases']:
            state=dict(control_step=0,sim_time_s=0.,pose=case['pose'],valid=True,frenet=dict(valid=True))
            pairs=[measured_plan(geometry,state,config,i) for i in range(2)]
            group['fixed'].append(dict(name=case['name'],runs=[r for r,_ in pairs]))
            pending.extend((r,e,config) for r,e in pairs)
        del geometry
        geometry,init=fresh();group['moving_initialization_s']=init
        for i,state in enumerate(manifest['states']):
            row,event=measured_plan(geometry,state,config,i)
            group['moving'].append(row);pending.append((row,event,config))
        del geometry
        group['moving_summary']=summarize(group['moving']);save()
        print(json.dumps(dict(mode=mode,summary=group['moving_summary'])),flush=True)
    audit_geometry=load_geometry(GEOMETRY,MAP_YAML,inspect=True)
    with (root/'events.jsonl').open('w') as stream:
        for row,event,config in pending:
            row['independent_exact_audit']=audit_legacy_grid(audit_geometry,event,config)
            stream.write(json.dumps(event,allow_nan=False)+'\n')
    report['normal_plan_count']=len(pending)
    report['audited_candidate_instances']=sum(r['independent_exact_audit']['checked_count'] for r,_,_ in pending)
    report['audit_failed_candidate_instances']=sum(not p['passed'] for r,_,_ in pending
        for p in r['independent_exact_audit']['paths'])
    save()
    # Separate integration run: two uninterrupted laps, no candidates control the car.
    preview=root/'two_laps'
    config=SessionConfig(laps=2,query_mode='fixed_grid_lookup',generation_mode='direct_then_repair')
    summary=run_session(preview,config,geometry_dir=GEOMETRY,map_yaml=MAP_YAML)
    verification=verify_session(preview,require_complete=summary['recording_complete'])
    (root/'two_lap_verification.json').write_text(json.dumps(verification,indent=2,allow_nan=False)+'\n')
    lap_audits=[];lookup_rows=[]
    with (preview/'plans.jsonl').open() as stream:
        for line in stream:
            event=json.loads(line)
            audit=audit_legacy_grid(audit_geometry,event,config)
            lap_audits.append(audit)
            result=event.get('result',{});query=result.get('query_reuse',{})
            lookup_rows.append(dict(step=event['source_control_step'],
                request=query.get('fixed_arc_lookup_request'),lookup=query.get('fixed_arc_lookup'),
                road_table=query.get('road_table'),sampled_valid_count=event['sampled_valid_count']))
    report.update(two_laps=dict(summary=summary,verification=verification,
        audits=lap_audits,lookup_rows=lookup_rows,
        audited_instances=sum(a['checked_count'] for a in lap_audits),
        failed_instances=sum(not p['passed'] for a in lap_audits for p in a['paths'])),completed=True)
    save();print(json.dumps(dict(completed=True,normal_plans=len(pending),
        two_lap_complete=summary['recording_complete'])),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--inputs',default='docs/project/reports/my_map_3m_inputs_20261029.json')
    parser.add_argument('--output-dir',required=True)
    args=parser.parse_args();run(args.inputs,args.output_dir)
