"""Artificial dynamics plus fixed-plan on/off replay. No formal ESP evidence."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS

import numpy as np
import pytest

from f1tenth_benchmarks.research.collision.discs import ThreeDiscBody
from f1tenth_benchmarks.research.overtaking.two_car_decision import OvertakingSelector
from f1tenth_benchmarks.research.overtaking.rollout_diagnostics_v1 import RolloutDiagnostics,validate_record,write_sidecar


class Car:
    def __init__(self, fail_after=None):
        self.state=np.array([0.,0.,0.,1.,0.,0.,0.]);self.steer_buffer=np.zeros(2)
        self.calls=0;self.fail_after=fail_after
    def update_pose(self,steer,speed):
        self.calls+=1
        if self.fail_after is not None and self.calls>self.fail_after:raise ValueError('artificial dynamics failure')
        self.state[0]+=speed*.01;self.state[3]=speed


class PP:
    def __init__(self,fail_after=None):self.path=np.array([[0.,0.],[5.,0.],[10.,0.]]);self.calls=0;self.fail_after=fail_after;self.l2s=np.array([25.,25.])
    def nearest(self,xy):return None,0.,xy[0]/5,0
    def plan(self,pose,lookahead):
        self.calls+=1
        if self.fail_after is not None and self.calls>self.fail_after:raise ValueError('artificial PP failure')
        return [0.,1.],{}


class Map:
    def __init__(self,hit=False):self.hit=hit;self.calls=0
    def intersects(self,*args):self.calls+=1;return self.hit


class Curve:
    L=100.
    def sample(self,s):
        s=np.asarray(s);return np.column_stack([s,0*s]),np.column_stack([np.ones_like(s),0*s]),0*s,0*s


def fixture(*,enabled=True,lead_x=5.,wall=False,car_fail=None):
    geom=NS(geometry_id='synthetic',frame_id='map',curve=Curve())
    sim=NS(cars=[Car(car_fail)],bodies=[ThreeDiscBody()],params=NS(timestep=.01),pair_clearance=.05,
           wall_clearance=.05,physics_tick=0,map=Map(wall))
    controller=NS(config=NS(lookahead_m=10.),lookahead=lambda speed:.5,pursuit=lambda p:PP())
    trace=RolloutDiagnostics() if enabled else None
    selector=OvertakingSelector(geom,sim,controller,1.2,1,diagnostics=trace)
    def state(x,speed):return dict(pose=[x,0.,0.],speed_mps=speed,sim_time_s=10.,control_step=2,valid=True,
        frenet=dict(s_wrapped=x,s_unwrapped=x,d=0.,e_psi=0.,jacobian=1.))
    ego=state(0.,1.);lead=state(lead_x,0.);lead['delta_s_unwrapped_m']=lead_x
    paths=[dict(candidate_id=i,path_type='reference_centerline' if i==3 else 'state_matched_trajectory',
            sampled_valid=True,footprint_sampled_valid=True,reason=None,target_d_m=(i-3)*.5) for i in range(1,6)]
    plan=dict(plan_id=2,source_control_step=2,source_sim_time_s=10.,source_pose=ego['pose'][:],geometry_id='synthetic',paths=paths)
    if trace:
        trace.begin_frame(ego,lead);trace.source_runtime=dict(source_physics_tick=0,held_steering=0.)
    return selector,trace,ego,lead,plan


def rollout(*,lead_x=5.,wall=False,pp_fail=None,car_fail=None,post_gate=False,candidate=1):
    selector,trace,ego,lead,plan=fixture(lead_x=lead_x,wall=wall,car_fail=car_fail)
    times=np.arange(5)*.05;poses,rate=selector._predict_lead(lead,times)
    trace.start_option(candidate,2.,path=plan['paths'][candidate-1])
    result=selector._rollout(PP(pp_fail),2.,[selector.body.centres(p) for p in poses],rate,times)
    if post_gate:result.update(safe=False,reason='insufficient_pass_progress')
    option=dict(candidate_id=candidate,target_speed_mps=2.,**result);trace.finish_option(option)
    record=trace.finish_frame(dict(plan=plan,costs=dict(options=[option])),selector,selector.geometry)
    return record,selector,option


def test_complete_roundtrip_and_minimum_and_unmodified_live_car(tmp_path):
    r,s,n=rollout();validate_record(r);o=r['options'][0]
    assert o['evaluation_complete'] and o['checked_sample_count']==5
    assert o['checked_start_s']==0 and o['checked_end_s']==.2
    assert min(x['signed_min_pair_clearance_m'] for x in o['samples'])==n['min_pair_clearance_m']
    assert s.sim.cars[0].state[0]==0 and s.sim.cars[0].calls==0
    assert s.sim.cars[0].steer_buffer.tolist()==[0.,0.]
    assert r['binding_status']=='unverified_snapshot_only' and r['plan_binding_v2'] is None
    assert all(x['lead_pose']==[5.,0.,0.] for x in o['samples'])
    assert r['inspection_only'] and not r['planning_allowed']
    write_sidecar(tmp_path/'trace.jsonl',[r]);back=json.loads((tmp_path/'trace.jsonl').read_text());assert validate_record(back)==r
    with pytest.raises(FileExistsError):write_sidecar(tmp_path/'trace.jsonl',[r])


@pytest.mark.parametrize('lead_x,current',[(.8,False),(.2,True)])
def test_contact_does_not_imply_early_stop(lead_x,current):
    r,s,n=rollout(lead_x=lead_x);validate_record(r);o=r['options'][0]
    assert o['native_safe'] is False and o['native_reason']=='predicted_pair_clearance'
    assert (o['first_rejection_s']==0)==current
    assert o['evaluation_complete'] and o['termination_reason']=='horizon_reached'
    assert any(x['signed_min_pair_clearance_m']<=0 for x in o['samples'])
    assert o['checked_end_s']>o['first_rejection_s']
    assert o['wall_evaluation_complete'] is False


def test_wall_rejection_only_records_actual_checks():
    r,s,n=rollout(wall=True);validate_record(r);o=r['options'][0]
    assert n['reason']=='predicted_wall_clearance' and s.sim.map.calls==1
    assert o['samples'][0]['wall_intersects'] is True
    assert all(x['wall_intersects'] is None for x in o['samples'][1:])
    assert o['evaluation_complete'] and not o['wall_evaluation_complete']


@pytest.mark.parametrize('kind,successful', [('pp',0),('pp',1),('pp',6),('dynamics',0),('dynamics',6)])
def test_partial_failure_never_labels_target_grid_checked(kind,successful):
    r,s,n=rollout(pp_fail=successful if kind=='pp' else None,car_fail=successful if kind=='dynamics' else None)
    validate_record(r);o=r['options'][0]
    assert not o['evaluation_complete'] and o['termination_reason']=='rollout_value_error'
    assert o['error_operation']==('pp_plan' if kind=='pp' else 'update_pose')
    assert o['checked_end_s']<=o['last_integrated_t_s']<o['failed_target_grid_t_s']
    assert len(o['samples'])==1+successful//5
    assert o['checked_end_s']==pytest.approx((successful//5)*.05)


def test_first_rejection_and_later_termination_separate():
    r,s,n=rollout(lead_x=.2,pp_fail=6);validate_record(r);o=r['options'][0]
    assert o['native_reason']=='predicted_pair_clearance' and o['first_rejection_s']==0
    assert o['termination_reason']=='rollout_value_error' and o['checked_end_s']==.05


def test_post_gate_and_c3_reference_not_promoted():
    r,s,n=rollout(post_gate=True,candidate=3);validate_record(r);o=r['options'][0]
    assert o['post_gate_reason']=='insufficient_pass_progress' and o['termination_reason']=='horizon_reached'
    assert o['reference_only'] and o['source_path_type']=='reference_centerline'
    assert o['rollout_trajectory_type']=='modeled_ego_rollout'
    assert r['plan_snapshot']['paths'][2]['path_type']=='reference_centerline'


@pytest.mark.parametrize('lead_x,wall',[(5.,False),(.45,False),(.2,False),(5.,True)])
def test_selector_on_off_fixed_plan_and_source_parity(lead_x,wall):
    outputs=[]
    for enabled in (False,True):
        s,t,ego,lead,plan=fixture(enabled=enabled,lead_x=lead_x,wall=wall);before=deepcopy((ego,lead,plan))
        result=s(plan,ego,lead)
        assert (ego,lead,plan)==before
        if enabled:
            record=t.finish_frame(dict(plan=plan,costs=result['costs']),s,s.geometry);validate_record(record)
        result['decision'].pop('wall_time_s')
        outputs.append((result,s.sim.cars[0].state.tolist(),s.sim.cars[0].steer_buffer.tolist(),s.sim.map.calls))
    assert outputs[0]==outputs[1]


def test_static_rejection_empty_and_unique_repeated_speed_options():
    s,t,e,l,p=fixture(lead_x=1.2)
    for c in p['paths']:c.update(sampled_valid=False,reason='artificial_static_reject')
    result=s(p,e,l);r=t.finish_frame(dict(plan=p,costs=result['costs']),s,s.geometry);validate_record(r)
    assert r['options'] and all(not o['samples'] and o['checked_end_s'] is None and not o['evaluation_complete'] for o in r['options'])
    ids=[o['evaluation_id'] for o in r['options']];assert len(ids)==len(set(ids))
    # Identical candidate-speed calls are separate evaluation identities.
    times=np.arange(3)*.05;s._predict_lead(l,times)
    opts=[]
    for _ in range(2):
        t.start_option(1,1.,path=p['paths'][0]);o=dict(candidate_id=1,target_speed_mps=1.,safe=False,reason='static_rejection')
        t.finish_option(o);opts.append(o)
    r=t.finish_frame(dict(plan=p,costs=dict(options=opts)),s,s.geometry)
    assert len(set(o['evaluation_id'] for o in r['options']))==2


def test_raceline_different_group_and_final_prepend_order():
    s,t,e,l,p=fixture();opts=[]
    for cid,count in [(1,3),(None,17)]:
        times=np.arange(count)*.05;poses,rate=s._predict_lead(l,times)
        t.start_option(cid,1.,path=p['paths'][0] if cid else None,reference_id='raceline' if cid is None else None,reference_xy=PP().path if cid is None else None)
        native=s._rollout(PP(),1.,[s.body.centres(x) for x in poses],rate,times)
        o=dict(candidate_id=cid,target_speed_mps=1.,**native);t.finish_option(o);opts.append(o)
    r=t.finish_frame(dict(plan=p,costs=dict(options=opts[::-1])),s,s.geometry);validate_record(r)
    assert [o['candidate_id'] for o in r['options']]==[None,1]
    assert [o['option_index'] for o in r['options']]==[0,1]
    assert r['options'][0]['requested_horizon_s']==.8 and r['options'][1]['requested_horizon_s']==.1
    assert r['options'][0]['prediction_group_id']!=r['options'][1]['prediction_group_id']


@pytest.mark.parametrize('field',['complete','end','grid','frame','time','pose','nan','inf','binding','hash'])
def test_invalid_record_never_accepted(field):
    r,_,_=rollout(pp_fail=6);o=r['options'][0]
    if field=='complete':o['evaluation_complete']=True
    elif field=='end':o['checked_end_s']=.2
    elif field=='grid':o['requested_sample_times_s'][1]=.04
    elif field=='frame':r['metadata']['frame_id']='odom'
    elif field=='time':r['source']['lead_state']['sim_time_s']=11.
    elif field=='pose':r['source']['ego_state']['pose']=None
    elif field=='nan':o['samples'][0]['ego_pose'][0]=float('nan')
    elif field=='inf':o['samples'][0]['signed_min_pair_clearance_m']=float('inf')
    elif field=='binding':r['binding_status']='verified'
    else:r['plan_snapshot_sha256']='0'*64
    with pytest.raises(ValueError):validate_record(r)


from test_two_car_candidates import track_and_domain


def test_real_physics_environment_fixed_plan_paired_replay(track_and_domain,monkeypatch,tmp_path):
    from f1tenth_benchmarks.research.overtaking.two_car_env import TwoCarOvertakingEnv
    track,domain=track_and_domain
    envs=[TwoCarOvertakingEnv(track,domain,phase=2,rollout_diagnostics=enabled) for enabled in (False,True)]
    frames=[env.reset() for env in envs];records=[]
    for step in range(3):
        ego=frames[0]['ego_state'];q=np.linspace(0.,3.,151)
        xy,_,yaw,_=track.geometry.curve.sample(ego['frenet']['s_wrapped']+q)
        paths=[]
        for cid in range(1,6):
            paths.append(dict(candidate_id=cid,path_type='reference_centerline' if cid==3 else 'state_matched_trajectory',
                sampled_valid=True,footprint_sampled_valid=True,reason=None,target_d_m=0.,
                points=[dict(x=float(x),y=float(y),yaw=float(a),valid=True) for (x,y),a in zip(xy,yaw)]))
        plan=dict(plan_id=step,source_control_step=step,source_sim_time_s=ego['sim_time_s'],source_pose=ego['pose'][:],
                  geometry_id=track.geometry.geometry_id,paths=paths)
        before=deepcopy(plan);results=[]
        for i,env in enumerate(envs):
            monkeypatch.setattr(env,'_make_plan',lambda *args,p=plan:deepcopy(p))
            transition,frames[i]=env.step();assert plan==before
            transition.pop('cycle_wall_time_s');transition['decision'].pop('wall_time_s')
            results.append(transition)
        assert results[0]==results[1] and frames[0]==frames[1]
        assert envs[0].sim.physics_tick==envs[1].sim.physics_tick
        for a,b in zip(envs[0].sim.cars,envs[1].sim.cars):
            np.testing.assert_array_equal(a.state,b.state);np.testing.assert_array_equal(a.steer_buffer,b.steer_buffer)
        record=envs[1].rollout_diagnostics.last_record;validate_record(record);records.append(record)
    write_sidecar(tmp_path/'rollout_diagnostics.jsonl',records)
    assert len((tmp_path/'rollout_diagnostics.jsonl').read_text().splitlines())==3


def test_cli_flag_is_opt_in():
    import inspect
    from f1tenth_benchmarks.research import run_two_car_candidates as runner
    assert inspect.signature(runner.run).parameters['rollout_diagnostics'].default is False


@pytest.mark.parametrize('lead_x,wall',[(5.,False),(.8,False),(.2,False),(5.,True)])
def test_disabled_matches_unmodified_upstream_selector(lead_x,wall,tmp_path):
    import importlib.util,subprocess
    original=tmp_path/'original.py'
    original.write_bytes(subprocess.check_output(['git','show','707d869:f1tenth_benchmarks/research/overtaking/two_car_decision.py']))
    spec=importlib.util.spec_from_file_location('original_selector',original)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    answers=[]
    for old in (True,False):
        s,t,e,l,p=fixture(enabled=False,lead_x=lead_x,wall=wall)
        if old:s=module.OvertakingSelector(s.geometry,s.sim,s.controller,1.2,1)
        r=s(p,e,l);r['decision'].pop('wall_time_s');answers.append((r,s.sim.map.calls))
    assert answers[0]==answers[1]


def test_actual_raceline_then_candidate_fallback_groups():
    s,t,e,l,p=fixture(wall=True)
    race=NS(local_plan=lambda *a:(PP(),1.2))
    pp,speed,race_opts,decision=s.try_raceline(race,e,l)
    assert pp is None and decision is None and len(race_opts)==3
    choice=s(p,e,l);choice['costs']['options'][:0]=race_opts
    r=t.finish_frame(dict(plan=p,costs=choice['costs']),s,s.geometry);validate_record(r)
    assert len(r['lead_predictions'])==2
    assert r['options'][0]['requested_horizon_s']==.8
    assert r['options'][-1]['requested_horizon_s']==2.
    assert all(o['reference_id']=='raceline' for o in r['options'][:3])
    assert len(set(o['evaluation_id'] for o in r['options']))==len(r['options'])


def test_actual_raceline_only_no_plan():
    s,t,e,l,p=fixture()
    pp,speed,opts,decision=s.try_raceline(NS(local_plan=lambda *a:(PP(),1.2)),e,l)
    assert pp is not None
    r=t.finish_frame(dict(plan=None,costs=dict(options=opts)),s,s.geometry);validate_record(r)
    assert r['plan_snapshot_sha256'] is None and r['options'][0]['reference_xy_sha256']
    assert r['options'][0]['reference_id']=='raceline'


@pytest.mark.parametrize('change',['missing_source','bad_clock','frame','geometry','nonfinite','v2_claim','candidate_hash','source_hash'])
def test_invalid_sources_explicitly_rejected(change):
    s,t,e,l,p=fixture()
    result=s(p,e,l)
    if change=='missing_source':t.source.pop('lead_state')
    elif change=='bad_clock':t.source['lead_state']['sim_time_s']=0.
    elif change=='frame':t.source['ego_state']['frame_id']='odom'
    elif change=='geometry':t.source['lead_state']['geometry_id']='other'
    elif change=='nonfinite':t.source['ego_state']['speed_mps']=float('inf')
    r=t.finish_frame(dict(plan=p,costs=result['costs']),s,s.geometry)
    if change=='v2_claim':r['plan_binding_v2']={'verified':True}
    elif change=='candidate_hash':r['options'][0]['candidate_snapshot_sha256']='f'*64
    elif change=='source_hash':r['ego_state_sha256']='f'*64
    if change in ('missing_source','bad_clock','frame','geometry','nonfinite'):assert r['status']=='invalid'
    with pytest.raises(ValueError):validate_record(r)


def test_sidecar_opt_in_runner_keeps_frames_report_schema(track_and_domain,monkeypatch,tmp_path):
    from f1tenth_benchmarks.research import run_two_car_candidates as runner
    from f1tenth_benchmarks.research.overtaking.two_car_env import TwoCarOvertakingEnv
    track,domain=track_and_domain
    monkeypatch.setattr(runner,'_load_domain',lambda *a:(domain,tmp_path/'domain'))
    monkeypatch.setattr(runner,'_preview',lambda *a:None)
    def fixed(env,ego,lead):
        q=np.linspace(0.,3.,151);xy,_,yaw,_=env.geometry.curve.sample(ego['frenet']['s_wrapped']+q)
        return dict(plan_id=env.control_step,source_control_step=env.control_step,source_sim_time_s=ego['sim_time_s'],
            source_pose=ego['pose'],geometry_id=env.geometry.geometry_id,wall_time_s=0.,sampled_valid_count=5,
            paths=[dict(candidate_id=i,path_type='reference_centerline' if i==3 else 'state_matched_trajectory',
                sampled_valid=True,footprint_sampled_valid=True,reason=None,target_d_m=0.,
                points=[dict(x=float(x),y=float(y),yaw=float(a),valid=True) for (x,y),a in zip(xy,yaw)]) for i in range(1,6)])
    monkeypatch.setattr(TwoCarOvertakingEnv,'_make_plan',fixed)
    for enabled in (False,True):runner.run(tmp_path/str(enabled),map_name='my_map',phase=2,seconds=.1,rollout_diagnostics=enabled)
    assert not (tmp_path/'False/rollout_diagnostics.jsonl').exists()
    record=json.loads((tmp_path/'True/rollout_diagnostics.jsonl').read_text());validate_record(record)
    off=json.loads((tmp_path/'False/frames.json').read_text());on=json.loads((tmp_path/'True/frames.json').read_text())
    def timing_removed(value):
        if isinstance(value,dict):return {k:timing_removed(v) for k,v in value.items() if k not in ('wall_time_s','cycle_wall_time_s')}
        if isinstance(value,list):return [timing_removed(v) for v in value]
        return value
    assert timing_removed(off)==timing_removed(on)
    assert json.loads((tmp_path/'False/report.json').read_text()).keys()==json.loads((tmp_path/'True/report.json').read_text()).keys()


def test_missing_runtime_is_explicit_invalid():
    s,t,e,l,p=fixture();result=s(p,e,l);t.source_runtime=None
    r=t.finish_frame(dict(plan=p,costs=result['costs']),s,s.geometry)
    assert r['status']=='invalid' and 'missing_source_runtime' in r['errors']
    with pytest.raises(ValueError):validate_record(r)


def test_invalid_record_can_be_saved_without_safety_upgrade(tmp_path):
    r,_,_=rollout();r.update(status='invalid',errors=['source_unavailable'])
    write_sidecar(tmp_path/'invalid.jsonl',[r])
    with pytest.raises(ValueError):validate_record(json.loads((tmp_path/'invalid.jsonl').read_text()))
    r['planning_allowed']=True
    with pytest.raises(ValueError):write_sidecar(tmp_path/'wrong.jsonl',[r])


def test_configured_clearances_are_not_measured_distances():
    record,_,_=rollout()
    meta=record['metadata']
    assert meta['configured_pair_clearance_m']==.05
    assert meta['configured_wall_clearance_m']==.05
    assert meta['physical_pair_clearance_m'] is None
    assert meta['physical_wall_clearance_m'] is None
    validate_record(record)


def test_selector_hot_path_does_not_hash_serialize_or_write(monkeypatch):
    from f1tenth_benchmarks.research.overtaking import rollout_diagnostics_v1 as diagnostics
    s,t,e,l,p=fixture(lead_x=.8)
    before=deepcopy((e,l,p))
    def forbidden(*args,**kwargs):raise AssertionError('hash/JSON/file IO inside native decision')
    with monkeypatch.context() as patch:
        patch.setattr(diagnostics,'snapshot_sha256',forbidden)
        patch.setattr(diagnostics.json,'dumps',forbidden)
        patch.setattr(diagnostics,'write_sidecar',forbidden)
        result=s(p,e,l)
    assert (e,l,p)==before
    validate_record(t.finish_frame(dict(plan=p,costs=result['costs']),s,s.geometry))


def test_record_finalization_after_step_does_not_mutate_caller_inputs():
    s,t,e,l,p=fixture(lead_x=.8)
    result=s(p,e,l);before=deepcopy((e,l,p,result))
    record=t.finish_frame(dict(plan=p,costs=result['costs']),s,s.geometry)
    assert (e,l,p,result)==before
    validate_record(record)
    assert record['plan_snapshot'] is not p


def test_collector_is_independent_of_cost_and_binding_modules():
    import ast
    from pathlib import Path
    import f1tenth_benchmarks.research.overtaking.rollout_diagnostics_v1 as diagnostics
    tree=ast.parse(Path(diagnostics.__file__).read_text())
    forbidden=('cost_','plan_binding','frenet','prediction','two_car_decision')
    for node in ast.walk(tree):
        if isinstance(node,ast.Import):names=[n.name for n in node.names]
        elif isinstance(node,ast.ImportFrom):names=[node.module or '']
        else:continue
        assert not any(token in name for name in names for token in forbidden)


@pytest.mark.parametrize('version',[None,2,True])
def test_unknown_or_malformed_schema_version_rejected(version):
    record,_,_=rollout();record['schema_version']=version
    with pytest.raises(ValueError,match='unsupported_diagnostics_version'):validate_record(record)


@pytest.mark.parametrize('claim',[True,None,'false'])
def test_snapshot_cannot_claim_verified_historical_producer(claim):
    record,_,_=rollout();record['historical_producer_binding_verified']=claim
    with pytest.raises(ValueError,match='unsupported_historical_binding_claim'):validate_record(record)


@pytest.mark.parametrize('errors',['bad',None,[123]])
def test_malformed_error_list_is_explicitly_rejected(errors):
    record,_,_=rollout();record['errors']=errors
    with pytest.raises(ValueError,match='invalid_diagnostics_errors'):validate_record(record)
