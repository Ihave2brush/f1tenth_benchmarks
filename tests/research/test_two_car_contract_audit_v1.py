"""Synthetic saved-format audit; no real ESP run or candidate generation."""
from copy import deepcopy
import csv
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest
from binding_contract_fixtures import analytic_input, artifact

ROOT=Path(__file__).resolve().parents[2]
spec=importlib.util.spec_from_file_location('two_car_contract_audit',ROOT/'tools/audit_two_car_contract_v1.py')
tool=importlib.util.module_from_spec(spec);spec.loader.exec_module(tool)


def saved_case():
    p,m=analytic_input();p['paths'][2].update(path_type='reference_centerline',requested_target_d=0.,reference_d=0.,
        spline=dict(knots=[0.]*6+[3.]*6,coefficients=[0.]*6,degree=5))
    for c in p['paths']:c['footprint_sampled_valid']=True
    ego=dict(valid=True,pose=p['source_pose'][:],speed_mps=1.,sim_time_s=10.,control_step=4,
             frenet=dict(s_unwrapped=10.,d=0.))
    lead=dict(valid=True,pose=[3.,0.,0.],speed_mps=.8,sim_time_s=10.,control_step=4,
              frenet=dict(s_unwrapped=13.,d=0.),delta_s_unwrapped_m=3.)
    def opt(cid,speed,**kw):
        return dict(candidate_id=cid,target_speed_mps=speed,safe=True,reason=None,
                    first_rejection_s=None,min_pair_clearance_m=.3,relative_s_progress_m=.1,**kw)
    f=dict(time_s=10.,ego_state=ego,lead_state=lead,plan=p,selected_candidate_id=3,selection_reason='follow',
        costs=dict(options=[opt(2,1.),opt(2,.8),opt(None,1.,reference='raceline'),opt(3,.8)]),
        raceline_xy=[[0.,0.],[3.,0.]],decision=dict(horizon_s=.1,sample_dt_s=.05,
        pair_threshold_m=.09,wall_threshold_m=.07,lead_prediction_poses=[[3.,0.,0.],[3.04,0.,0.],[3.08,0.,0.]]))
    report=dict(map='synthetic',phase=2,pair_clearance_m=.05,config=dict(lookahead_m=3.,control_dt_s=.1,
        footprint_length_m=.51,footprint_width_m=.27,footprint_clearance_m=.05))
    m['producer_commit']=tool.PRODUCER
    return [f],report,m


def audit(frames=None,report=None,metadata=None,**kw):
    f,r,m=saved_case()
    return tool.audit_two_car_frames(f if frames is None else frames,r if report is None else report,
        session_metadata=m if metadata is None else metadata,source_classification='synthetic',**kw)


def test_real_field_layout_order_speed_options_reference_and_no_mutation():
    f,r,m=saved_case();before=deepcopy((f,r,m));out=audit(f,r,m)
    assert out['contract_valid'] and out['source_classification']=='synthetic'
    assert (f,r,m)==before
    assert [c['candidate_id'] for c in out['frames'][0]['candidates']]==[1,2,3,4,5]
    opts=out['options'];assert len(opts)==4
    assert [o['target_speed_mps'] for o in opts[:2]]==[1.,.8]
    assert opts[0]['option_key']!=opts[1]['option_key']
    assert opts[2]['candidate_id'] is None and opts[2]['reference']=='raceline'
    assert opts[2]['raceline_xy_sha256'] and opts[3]['reference_only']
    assert all(o['coverage_status']=='unknown' and o['evaluation_complete'] is None for o in opts)
    assert all(not o['eligible_for_ego_trajectory_comparison'] for o in opts)
    assert opts[0]['model']=='three_discs' and opts[0]['configured_pair_clearance_m']==.05
    assert opts[0]['pair_threshold_m']==.09
    assert out['inspection_only'] and not out['planning_allowed']
    json.dumps(out,allow_nan=False)


@pytest.mark.parametrize('change',['lead_time','lead_step','frame_time','plan_time','plan_step','plan_pose',
    'state_frame','state_geometry','report_frame','metadata_geometry','invalid_source_time','invalid_pose','missing_speed',
    'missing_state','duplicate_ids','wrong_count','reference_not_c3','bad_native','bad_footprint'])
def test_source_or_candidate_contract_mismatch(change):
    f,r,m=saved_case();x=f[0]
    if change=='lead_time':x['lead_state']['sim_time_s']=11.
    elif change=='lead_step':x['lead_state']['control_step']=5
    elif change=='frame_time':x['time_s']=11.
    elif change=='plan_time':x['plan']['source_sim_time_s']=11.
    elif change=='plan_step':x['plan']['source_control_step']=5
    elif change=='plan_pose':x['plan']['source_pose']=[1.,0.,0.]
    elif change=='state_frame':x['lead_state']['frame_id']='odom'
    elif change=='state_geometry':x['ego_state']['geometry_id']='other'
    elif change=='report_frame':r['frame_id']='odom'
    elif change=='metadata_geometry':m['geometry_id']='other'
    elif change=='invalid_source_time':x['ego_state']['sim_time_s']=True
    elif change=='invalid_pose':x['ego_state']['pose']=[0.,0.]
    elif change=='missing_speed':del x['lead_state']['speed_mps']
    elif change=='missing_state':del x['lead_state']
    elif change=='duplicate_ids':x['plan']['paths'][1]['candidate_id']=1
    elif change=='wrong_count':x['plan']['paths'].pop()
    elif change=='reference_not_c3':x['plan']['paths'][0]['path_type']='reference_centerline'
    elif change=='bad_native':x['plan']['paths'][0]['sampled_valid']=None
    else:x['plan']['paths'][0]['footprint_sampled_valid']='yes'
    out=audit(f,r,m)
    assert not out['contract_valid']
    assert all(not o['eligible_for_ego_trajectory_comparison'] for o in out['options'])


def test_rejected_source_and_reference_remain_noncomparable():
    f,r,m=saved_case();x=f[0];x['plan']['paths'][1].update(sampled_valid=False,reason='static_rejection')
    x['costs']['options'][0].update(safe=False,reason='static_rejection',min_pair_clearance_m=None)
    x['costs']['options'][1].update(safe=False,reason='static_rejection',min_pair_clearance_m=None)
    x['ego_state']['valid']=False
    out=audit(f,r,m)
    assert 'native_rejected' in out['options'][0]['incomparability_reasons']
    assert 'invalid_source_state' in out['options'][0]['incomparability_reasons']
    assert out['options'][3]['reference_only']


@pytest.mark.parametrize('change',['missing_distance','early_stop','negative_distance','unknown_producer'])
def test_diagnostics_not_fabricated(change):
    f,r,m=saved_case();o=f[0]['costs']['options'][0]
    if change=='missing_distance':del o['min_pair_clearance_m']
    elif change=='early_stop':o.update(safe=False,reason='lookahead_unavailable',first_rejection_s=.05)
    elif change=='negative_distance':o.update(safe=False,reason='predicted_pair_clearance',min_pair_clearance_m=-.03)
    else:m.pop('producer_commit')
    result=audit(f,r,m)['options'][0]
    assert result['coverage_status']=='unknown' and result['checked_range_s'] is None
    assert result['evaluation_complete'] is None
    if change=='missing_distance':assert result['min_pair_clearance_m'] is None
    if change=='negative_distance':assert result['min_pair_clearance_m']==-.03
    if change=='unknown_producer':assert result['model'] is None


@pytest.mark.parametrize('checked,complete,valid,coverage',[
    ([0.,.1],True,True,'reported_complete'),([0.,.05],False,True,'reported_partial'),
    ([0.,.05],True,False,'unknown'),([0.,.2],True,False,'unknown'),
    ([0.,.1],False,False,'unknown'),([0.,.07],False,False,'unknown'),
    (None,True,False,'unknown'),([0.,.05],None,False,'unknown')])
def test_explicit_checked_horizon_contract(checked,complete,valid,coverage):
    f,r,m=saved_case();f[0]['costs']['options'][0].update(checked_range_s=checked,evaluation_complete=complete)
    out=audit(f,r,m);o=out['options'][0]
    assert out['contract_valid'] is valid
    assert o['coverage_status']==coverage
    assert not o['eligible_for_ego_trajectory_comparison']


@pytest.mark.parametrize('change',['horizon','dt','lead_count','lead_origin','first_time','bad_speed','unknown_id','null_without_reference','bad_safe','missing_options'])
def test_bad_option_or_grid(change):
    f,r,m=saved_case();d=f[0]['decision'];o=f[0]['costs']['options'][0]
    if change=='horizon':d['horizon_s']=.12
    elif change=='dt':d['sample_dt_s']=0.
    elif change=='lead_count':d['lead_prediction_poses'].pop()
    elif change=='lead_origin':d['lead_prediction_poses'][0]=[2.,0.,0.]
    elif change=='first_time':o['first_rejection_s']=.2
    elif change=='bad_speed':o['target_speed_mps']=-1.
    elif change=='unknown_id':o['candidate_id']=6
    elif change=='null_without_reference':o['candidate_id']=None
    elif change=='bad_safe':o['safe']=None
    else:f[0]['costs']={'other':1}
    assert not audit(f,r,m)['contract_valid']


def test_raceline_without_plan_and_terminal_snapshot():
    f,r,m=saved_case();x=f[0];x.pop('plan');x['costs']['options']=[x['costs']['options'][2]]
    last=deepcopy(x);last.pop('costs');last.pop('decision');last.pop('raceline_xy');last['time_s']=10.1
    for k in ('ego_state','lead_state'):last[k].update(control_step=5,sim_time_s=10.1)
    f.append(last);out=audit(f,r,m)
    assert out['contract_valid'] and out['frames'][0]['binding_status']=='no_plan'
    assert out['options'][0]['plan_v2_sha256'] is None
    assert out['frames'][1]['audit_status']=='snapshot_only'
    f.append(deepcopy(last));assert not audit(f,r,m)['contract_valid']


def test_polyline_v1_and_json_roundtrip():
    f,r,m=saved_case()
    for c in f[0]['plan']['paths']:
        c.pop('spline');c.pop('output_representation');c['points']=[dict(x=0.,y=0.,valid=True),dict(x=3.,y=0.,valid=True)]
    out=audit(f,r,m)
    assert out['contract_valid'] and out==audit(json.loads(json.dumps(f)),json.loads(json.dumps(r)),json.loads(json.dumps(m)))


def test_geometry_and_parameter_metadata_verification(tmp_path):
    f,r,m=saved_case();path,pin=artifact(tmp_path)
    out=audit(f,r,m,geometry_manifest_path=path,expected_manifest_sha256=pin,
              parameter_metadata=dict(vehicle_length=.51,vehicle_width=.27))
    assert out['contract_valid'] and out['frames'][0]['plan_binding']['geometry_artifacts_verified']
    assert not audit(f,r,m,geometry_manifest_path=path,expected_manifest_sha256='0'*64)['contract_valid']
    assert not audit(f,r,m,parameter_metadata=dict(vehicle_length=.6))['contract_valid']
    with pytest.raises(ValueError):audit(f,r,dict(geometry_id='g'))


def test_no_generation_decision_control_or_prediction(monkeypatch):
    from f1tenth_benchmarks.research.overtaking import session,mm_bspline_repair,two_car_env,two_car_decision
    from f1tenth_benchmarks.research.collision.multi_car import MultiCarSim
    def forbidden(*a,**kw):pytest.fail('audit invoked planner/decision/control')
    for obj,name in [(session,'make_plan'),(mm_bspline_repair.BudgetedDomainGenerator,'__call__'),
        (two_car_decision.OvertakingSelector,'__call__'),(two_car_env.EgoCandidateController,'plan'),(MultiCarSim,'step')]:
        monkeypatch.setattr(obj,name,forbidden)
    assert audit()['contract_valid']
    assert 'two_car_decision' not in Path(tool.__file__).read_text()
    assert 'two_car_env' not in Path(tool.__file__).read_text()


def test_cli_json_csv_source_hashes_no_overwrite_or_source_mutation(tmp_path):
    f,r,m=saved_case();source=tmp_path/'source';source.mkdir()
    for name,obj in [('frames',f),('report',r),('metadata',m)]:
        (source/(name+'.json')).write_text(json.dumps(obj))
    before={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()}
    out=tmp_path/'output'
    cmd=[sys.executable,str(ROOT/'tools/audit_two_car_contract_v1.py'),'--frames',str(source/'frames.json'),
        '--report',str(source/'report.json'),'--metadata',str(source/'metadata.json'),
        '--classification','synthetic','--output',str(out)]
    env=dict(os.environ,PYTHONPATH=str(ROOT),PYTHONDONTWRITEBYTECODE='1')
    subprocess.run(cmd,check=True,capture_output=True,text=True,env=env)
    result=json.loads((out/'audit.json').read_text())
    assert result['source_file_sha256']['frames']==before['frames.json']
    rows=list(csv.DictReader((out/'options.csv').open()))
    assert [{k:json.loads(v) for k,v in row.items()} for row in rows]==result['options']
    assert {p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in source.iterdir()}==before
    assert subprocess.run(cmd,capture_output=True,env=env).returncode!=0
    cmd[-1]=str(source/'output')
    assert subprocess.run(cmd,capture_output=True,env=env).returncode!=0
    assert not (source/'output').exists()


@pytest.mark.parametrize('change',['pair_margin','wall_margin','decision_threshold','safe_reason','safe_distance'])
def test_native_diagnostic_consistency(change):
    f,r,m=saved_case()
    if change=='pair_margin':r['pair_clearance_m']=-.1
    elif change=='wall_margin':r['config']['footprint_clearance_m']=-.1
    elif change=='decision_threshold':f[0]['decision']['pair_threshold_m']=-.1
    elif change=='safe_reason':f[0]['costs']['options'][0]['reason']='lookahead_unavailable'
    else:f[0]['costs']['options'][0]['min_pair_clearance_m']=0.
    assert not audit(f,r,m)['contract_valid']


def test_geometry_evidence_even_on_raceline_only_frame(tmp_path):
    f,r,m=saved_case();f[0].pop('plan');f[0]['costs']['options']=[f[0]['costs']['options'][2]]
    path,pin=artifact(tmp_path)
    out=audit(f,r,m,geometry_manifest_path=path,expected_manifest_sha256=pin)
    assert out['contract_valid'] and out['geometry_evidence']['geometry_artifacts_verified']
    assert out['options'][0]['reference_only']
    assert not audit(f,r,m,geometry_manifest_path=path,expected_manifest_sha256='bad')['contract_valid']


def test_missing_body_and_margin_metadata_not_defaulted():
    f,r,m=saved_case();r['config'].pop('footprint_length_m');r['config'].pop('footprint_width_m')
    f[0]['decision'].pop('pair_threshold_m');f[0]['decision'].pop('wall_threshold_m')
    o=audit(f,r,m)['options'][0]
    assert o['body_length_m'] is None and o['body_width_m'] is None and o['pair_threshold_m'] is None
    assert 'body_dimensions_not_recorded' in o['incomparability_reasons']
    assert 'margin_thresholds_not_recorded' in o['incomparability_reasons']


@pytest.mark.parametrize('field',['geometry_id','frame_id'])
def test_candidate_metadata_mixing_rejected(field):
    f,r,m=saved_case();f[0]['plan']['paths'][0][field]='different'
    out=audit(f,r,m)
    assert not out['contract_valid']
    assert 'candidate_metadata_mismatch:'+field in out['frames'][0]['errors']


def test_opponent_change_changes_audit_context_but_not_plan_identity():
    f,r,m=saved_case();a=audit(f,r,m)
    f[0]['lead_state']['speed_mps']=.7
    b=audit(f,r,m)
    assert a['frames'][0]['plan_binding']==b['frames'][0]['plan_binding']
    assert a['audit_context_sha256']!=b['audit_context_sha256']
    assert a['options'][0]['audit_context_sha256']==a['audit_context_sha256']


def test_missing_options_differs_from_recorded_empty_options():
    f,r,m=saved_case();f[0]['costs']={}
    a=audit(f,r,m)
    assert not a['frames'][0]['options_recorded'] and a['frames'][0]['recorded_option_count'] is None
    f[0]['costs']={'options':[]}
    b=audit(f,r,m)
    assert b['frames'][0]['options_recorded'] and b['frames'][0]['recorded_option_count']==0


def test_safe_cannot_have_a_recorded_rejection_time():
    f,r,m=saved_case();f[0]['costs']['options'][0]['first_rejection_s']=.05
    out=audit(f,r,m)
    assert not out['contract_valid']
    assert 'safe_with_rejection_time' in out['options'][0]['option_errors']


def test_invalid_audit_cli_saves_diagnostics_but_returns_failure(tmp_path):
    f,r,m=saved_case();f[0]['lead_state']['sim_time_s']=11.
    source=tmp_path/'source';source.mkdir();out=tmp_path/'output'
    for name,obj in [('frames',f),('report',r),('metadata',m)]:
        (source/(name+'.json')).write_text(json.dumps(obj))
    cmd=[sys.executable,'-m','tools.audit_two_car_contract_v1','--frames',str(source/'frames.json'),
         '--report',str(source/'report.json'),'--metadata',str(source/'metadata.json'),
         '--classification','synthetic','--output',str(out)]
    completed=subprocess.run(cmd,cwd=ROOT,capture_output=True,text=True,env=dict(os.environ,PYTHONPATH=str(ROOT),PYTHONDONTWRITEBYTECODE='1'))
    assert completed.returncode==2
    assert not json.loads((out/'audit.json').read_text())['contract_valid']


@pytest.fixture(scope='module')
def upstream_two_car_track_domain():
    from f1tenth_benchmarks.research.overtaking.track_data import TrackData
    from f1tenth_benchmarks.research.overtaking.millimetre_domain import MillimetreDomain
    track=TrackData('maps/frenet/my_map/geometry','maps/my_map.yaml')
    return track,MillimetreDomain.build(track.geometry,margin=.185)


@pytest.mark.parametrize('lead_speed,gap',[(1.2,2.),(.9,1.2)])
def test_selector_and_control_unchanged_with_readonly_audit(upstream_two_car_track_domain,lead_speed,gap):
    from dataclasses import asdict
    from f1tenth_benchmarks.research.overtaking.two_car_env import TwoCarOvertakingEnv
    track,domain=upstream_two_car_track_domain
    def collect(with_audit):
        env=TwoCarOvertakingEnv(track,domain,lead_speed=lead_speed,gap=gap,phase=2,budget_s=0.,rolling_start=True)
        frame=env.reset();trace=[]
        report=dict(map='my_map',config=asdict(env.config),pair_clearance_m=env.sim.pair_clearance)
        metadata=dict(geometry_id=track.geometry.geometry_id,frame_id='map',producer_commit=tool.PRODUCER)
        for _ in range(3):
            source=deepcopy(frame);transition,frame=env.step()
            recorded=dict(source,**transition)
            before=deepcopy(recorded)
            if with_audit:
                audited=tool.audit_two_car_frames([recorded],report,session_metadata=metadata,
                    source_classification='saved_two_car_simulation')
                assert audited['contract_valid']
                assert recorded==before
            decision={k:v for k,v in transition['decision'].items() if k!='wall_time_s'}
            trace.append(dict(selected=transition['selected_candidate_id'],reason=transition['selection_reason'],
                actions=transition['actions'],decision=decision,costs=transition['costs'],events=frame['events'],
                ego_pose=frame['ego_state']['pose'],lead_pose=frame['lead_state']['pose']))
        return trace
    assert collect(False)==collect(True)


def test_configured_margins_are_not_measured_clearances():
    f,r,m=saved_case();before=deepcopy((f,r,m));out=audit(f,r,m);o=out['options'][0]
    assert o['configured_pair_clearance_m']==.05
    assert o['configured_wall_clearance_m']==.05
    assert o['physical_pair_clearance_m'] is None and o['physical_wall_clearance_m'] is None
    assert o['min_pair_clearance_m']==.3
    assert o['pair_threshold_m']==.09 and o['wall_threshold_m']==.07
    assert (f,r,m)==before
    assert o['inspection_only'] is True and o['planning_allowed'] is False


@pytest.mark.parametrize('with_plan',[False,True])
@pytest.mark.parametrize('terminal',[False,True])
@pytest.mark.parametrize('costs,valid,reason',[
    (None,True,None),({},True,None),({'options':[]},True,None),
    ([],False,'invalid_costs'),([1],False,'invalid_costs'),(False,False,'invalid_costs'),
    (0,False,'invalid_costs'),('',False,'invalid_costs'),('bad',False,'invalid_costs'),
    ({'other':1},False,'missing_options'),({'options':None},False,'invalid_options'),
    ({'options':{}},False,'invalid_options'),({'options':'bad'},False,'invalid_options'),
    ({'options':False},False,'invalid_options'),({'options':[None]},False,'invalid_option')])
def test_costs_schema_independent_of_plan_and_terminal(with_plan,terminal,costs,valid,reason):
    f,r,m=saved_case();x=f[0];x['costs']=deepcopy(costs)
    if not with_plan:x.pop('plan')
    if terminal:x['terminal_reason']='duration_reached'
    before=deepcopy((f,r,m));out=audit(f,r,m)
    assert out['contract_valid'] is valid
    if reason:
        errors=out['frames'][0]['errors']+[e for o in out['options'] for e in o['option_errors']]
        assert reason in errors
    elif not with_plan:
        assert out['frames'][0]['audit_status']=='snapshot_only'
        assert out['frames'][0]['binding_status']=='no_plan'
        assert out['frames'][0]['plan_binding'] is None
    assert (f,r,m)==before
    if valid:
        assert out['options']==[]
        assert out['frames'][0]['options_recorded'] is (costs=={'options':[]})
        assert out['frames'][0]['recorded_option_count']==(0 if costs=={'options':[]} else None)


@pytest.mark.parametrize('terminal',[False,True])
def test_raceline_only_native_diagnostics_without_plan_remain_readable(terminal):
    f,r,m=saved_case();x=f[0];x.pop('plan');native=deepcopy(x['costs']['options'][2])
    native.update(safe=False,reason='predicted_pair_clearance',first_rejection_s=.05,min_pair_clearance_m=-.03)
    x['costs']={'options':[native]}
    if terminal:x['terminal_reason']='envelope_event'
    out=audit(f,r,m);o=out['options'][0]
    assert out['contract_valid']
    assert o['candidate_id'] is None and o['reference']=='raceline'
    assert o['plan_id'] is None and o['plan_v2_sha256'] is None
    assert o['binding_status']=='no_plan' and o['reference_only']
    assert o['original_diagnostics']==native
    assert o['safe'] is False and o['reason']=='predicted_pair_clearance'
    assert o['min_pair_clearance_m']==-.03
