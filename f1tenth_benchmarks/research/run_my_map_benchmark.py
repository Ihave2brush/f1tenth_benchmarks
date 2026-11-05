"""MY map 3 m standard: identical recorded PP poses across all research modes."""
import argparse
from dataclasses import asdict
import json
import math
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
from .run_generator_abc import measured_plan
from .run_fixed_grid_benchmark import audit_legacy_grid
from .run_short_horizon_benchmark import sha, summarize

GEOMETRY = 'maps/frenet/my_map/geometry'
MAP_YAML = 'maps/my_map.yaml'
ARMS = {
    'original_exact': ('exact', 'optimized', None),
    'P1_reused_exact': ('reused_exact', 'optimized', None),
    'C_fast_optimized': ('fast_with_fallback', 'optimized', None),
    'A_hermite_repair': ('fast_with_fallback', 'direct_then_repair', None),
    'B_width_fraction': ('fast_with_fallback', 'width_fraction_then_repair', None),
    'F_fixed_grid_guard': ('fixed_grid_with_fallback', 'direct_then_repair', None),
    'A_5cm': ('fast_with_fallback', 'direct_then_repair', .05),
    'B_5cm': ('fast_with_fallback', 'width_fraction_then_repair', .05),
    'C_5cm': ('fast_with_fallback', 'optimized', .05),
}


def config_for(arm):
    query, generation, step = ARMS[arm]
    return SessionConfig(lookahead_m=3., pp_lookahead_m=.3, speed_mps=1.2, query_mode=query,
                         generation_mode=generation, validation_step_m=step)


def fresh(query):
    started=perf_counter()
    geometry=load_geometry(GEOMETRY, MAP_YAML, inspect=True)
    if query in ('fast_with_fallback','fixed_grid_with_fallback'):
        get_road_table(geometry, GEOMETRY)
    return geometry,perf_counter()-started


def freeze_inputs(source):
    root=Path(source)
    metadata=json.loads((root/'metadata.json').read_text())
    assert metadata['map']=='my_map'
    geometry,_=fresh('exact')
    assert metadata['geometry_id']==geometry.geometry_id
    rows=np.loadtxt(Path(GEOMETRY)/'centerline.csv',delimiter=',',skiprows=1)
    boundaries=np.loadtxt(Path(GEOMETRY)/'boundaries.csv',delimiter=',',skiprows=1,usecols=(0,3,6))
    anchors=[('start',0.),('quarter',geometry.curve.L/4),
             ('max_curvature',float(rows[np.argmax(abs(rows[:,4])),2])),
             ('narrowest',float(boundaries[np.argmin(boundaries[:,1]-boundaries[:,2]),0])),
             ('before_seam',geometry.curve.L-.12)]
    cases=[]
    for name,s in anchors:
        ref=geometry.reference(s)
        cases.append(dict(name=name,pose=[*ref['xy'],ref['psi']],s_m=s))
    cases.append(dict(name='heading_offset',pose=[*cases[1]['pose'][:2],cases[1]['pose'][2]+.15]))
    cases.append(dict(name='outside',pose=[20.,20.,0.]))
    states=[json.loads(line) for line in (root/'states.jsonl').read_text().splitlines()]
    return dict(schema_version=1,map='my_map',geometry_id=geometry.geometry_id,
        length_m=geometry.curve.L,lookahead_m=3.,source_session=str(source),
        source_sha256={n:sha(root/n) for n in ('metadata.json','states.jsonl','actions.jsonl','summary.json')},
        fixed_cases=cases,states=states,
        scope='actual_centerline_PP_recorded_states_same_for_every_arm_no_candidate_control')


def run(output, source=None):
    root=Path(output);root.mkdir(parents=True,exist_ok=False)
    if source is None:
        source=root/'source_PP'
        run_session(source,config_for('A_hermite_repair'),geometry_dir=GEOMETRY,map_yaml=MAP_YAML)
    summary=json.loads((Path(source)/'summary.json').read_text())
    verification=verify_session(source,require_complete=summary['recording_complete'])
    (root/'source_verification.json').write_text(json.dumps(verification,indent=2,allow_nan=False)+'\n')
    manifest=freeze_inputs(source)
    (root/'inputs.json').write_text(json.dumps(manifest,indent=2,allow_nan=False)+'\n')
    source_paths=[Path(__file__),Path(__file__).with_name('run_generator_abc.py'),
        Path(__file__).with_name('run_fixed_grid_benchmark.py'),
        Path(__file__).with_name('run_short_horizon_benchmark.py'),
        Path(__file__).with_name('run_centerline_candidates.py'),
        Path(__file__).with_name('core')/'frenet_lap_tracking.py',
        Path(__file__).with_name('core')/'geometry'/'tracker.py',
        *[Path(__file__).with_name('overtaking')/n for n in ('session.py','robust_adaptive_paths.py',
            'width_fraction_paths.py','fixed_grid.py','geometry_queries.py','road_table.py','verify_session.py')]]
    report=dict(schema_version=1,completed=False,map='my_map',length_m=manifest['length_m'],
        lookahead_m=3.,input_sha256=sha(root/'inputs.json'),geometry_id=manifest['geometry_id'],
        source_summary=summary,source_verification=verification,
        scope='make_plan_plus_JSON_equal_hooks_excludes_init_disk_control_quality_and_independent_audit',
        deadline_enforced=False,target_s=.1,
        environment=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,
            openblas_threads=os.environ.get('OPENBLAS_NUM_THREADS')),
        code_sha256={str(p):sha(p) for p in source_paths},groups=[])
    def save():
        (root/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    pending=[]
    for arm in ARMS:
        config=config_for(arm);group=dict(arm=arm,config=asdict(config),fixed=[],moving=[])
        report['groups'].append(group)
        geometry,init=fresh(config.query_mode);group['fixed_initialization_s']=init
        for case in manifest['fixed_cases']:
            state=dict(control_step=0,sim_time_s=0.,pose=case['pose'],valid=True,frenet=dict(valid=True))
            pairs=[measured_plan(geometry,state,config,i) for i in range(2)]
            group['fixed'].append(dict(name=case['name'],runs=[r for r,_ in pairs]))
            pending.extend((r,e,config) for r,e in pairs)
        del geometry
        geometry,init=fresh(config.query_mode);group['moving_initialization_s']=init
        for i,state in enumerate(manifest['states']):
            row,event=measured_plan(geometry,state,config,i)
            group['moving'].append(row);pending.append((row,event,config))
            if i%25==0:
                print(json.dumps(dict(arm=arm,step=i,of=len(manifest['states']),valid=row['valid_ids'],
                    total_s=row['total_s'])),flush=True)
        del geometry
        group['moving_summary']=summarize(group['moving']);save()
        print(json.dumps(dict(arm=arm,summary=group['moving_summary'])),flush=True)
    # Separate original-grid exact audit for every arm, after ALL normal timing.
    audit_geometry=load_geometry(GEOMETRY,MAP_YAML,inspect=True)
    with (root/'events.jsonl').open('w') as stream:
        for i,(row,event,config) in enumerate(pending):
            row['independent_exact_audit']=audit_legacy_grid(audit_geometry,event,config)
            stream.write(json.dumps(event,allow_nan=False)+'\n')
            if i%100==0:print(json.dumps(dict(audit_completed=i,of=len(pending))),flush=True)
    report.update(completed=True,normal_plan_count=len(pending),
        audited_candidate_instances=sum(r['independent_exact_audit']['checked_count'] for r,_,_ in pending),
        audit_failed_candidate_instances=sum(not p['passed'] for r,_,_ in pending
            for p in r['independent_exact_audit']['paths']))
    save();print(json.dumps(dict(completed=True,normal_plan_count=len(pending))),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True)
    parser.add_argument('--source-session')
    args=parser.parse_args();run(args.output_dir,args.source_session)
