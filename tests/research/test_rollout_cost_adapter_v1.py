"""Synthetic straight trajectories, analytic five-slot plan; no live rollout."""
from copy import deepcopy
import json
import math
from types import SimpleNamespace as NS

import numpy as np
import pytest
from test_path_length_feature_v1 import case,evaluate
from test_two_car_contract_audit_v1 import tool
from test_plan_binding_v2 import artifact
from f1tenth_benchmarks.research.overtaking.rollout_diagnostics_v1 import RolloutDiagnostics,write_sidecar
from f1tenth_benchmarks.research.overtaking.rollout_cost_adapter_v1 import adapt_rollout_cost_inputs,read_rollout_sidecar
from f1tenth_benchmarks.research.overtaking.cost_feature_v1 import build_cost_features
from f1tenth_benchmarks.research.overtaking.cost_evaluation_v1 import evaluate_costs


def fixture(*,lead_x=3.,partial=False,current=False,no_plan=False,options=None,recorded_complete=False,geometry_args=None,frenet_available=True):
    plan,geom=case(reference=True)
    for p in plan['paths']:p['footprint_sampled_valid']=True
    if current:lead_x=.2
    ego=dict(valid=True,pose=[0.,0.,0.],speed_mps=1.,sim_time_s=0.,control_step=0,
             frenet=dict(s_unwrapped=0.,s_wrapped=0.,d=0.,e_psi=0.,jacobian=1.))
    lead=dict(valid=True,pose=[lead_x,0.,0.],speed_mps=0.,sim_time_s=0.,control_step=0,
              frenet=dict(s_unwrapped=lead_x,s_wrapped=lead_x,d=0.,e_psi=0.,jacobian=1.))
    if not frenet_available:ego['frenet']=lead['frenet']=None
    options=options if options is not None else [(1,1.),(1,2.),(2,1.),(3,1.),(None,1.)]
    times=np.arange(5)*.05;poses=np.tile([lead_x,0.,0.],(5,1));radius=math.hypot(.51/6,.27/2)
    trace=RolloutDiagnostics('synthetic_straight_samples_not_ESP')
    trace.begin_frame(ego,lead);trace.source_runtime=dict(source_physics_tick=0,held_steering=0.)
    trace.prediction(times,poses,0.)
    raw=[]
    for cid,speed in options:
        pp=NS(path=np.array([[0.,0.],[1.5,0.],[3.,0.]]),speed=speed,lookahead=.5,wheelbase=.33,max_steer=.42)
        trace.start_option(cid,speed,path=plan['paths'][cid-1] if cid else None,
                           reference_id='raceline' if cid is None else None,reference_xy=pp.path if cid is None else None)
        trace.rollout_start(pp);minimum=math.inf;first=None;stop=2 if partial else len(times)
        for k,t in enumerate(times[:stop]):
            x=speed*t
            # Independent artificial envelope samples; no Selector/collision module call.
            a=np.array([x-.17,x,x+.17]);b=np.array([lead_x-.17,lead_x,lead_x+.17])
            clearance=float(np.abs(a[:,None]-b[None,:]).min()-2*radius);minimum=min(minimum,clearance)
            wall=None if first is not None or clearance<=.09 else False
            if first is None and clearance<=.09:first=float(t)
            state=np.array([x,0.,0.,speed,0.,0.,0.]);trace.sample(k,t,state,clearance,wall);trace.integrated(float(t))
        if partial:trace.integrated(.06);trace.stopped(.1,'pp_plan')
        reason='predicted_pair_clearance' if first is not None else 'lookahead_unavailable' if partial else None
        opt=dict(candidate_id=cid,target_speed_mps=speed,safe=reason is None,reason=reason,
            first_rejection_s=first if first is not None else .1 if partial else None,
            min_pair_clearance_m=minimum,relative_s_progress_m=speed*(.06 if partial else .2))
        if cid is None:opt['reference']='raceline'
        if recorded_complete:opt.update(checked_range_s=[0.,.05 if partial else .2],evaluation_complete=not partial)
        trace.finish_option(opt);raw.append(opt)
    frame=dict(time_s=0.,ego_state=ego,lead_state=lead,plan=None if no_plan else plan,costs=dict(options=raw),
        decision=dict(horizon_s=.2,sample_dt_s=.05,pair_threshold_m=.09,wall_threshold_m=.07,lead_prediction_poses=poses.tolist()),
        raceline_xy=[[0.,0.],[1.5,0.],[3.,0.]])
    config=dict(lookahead_m=3.,footprint_length_m=.51,footprint_width_m=.27,footprint_clearance_m=.05,control_dt_s=.1)
    sim=NS(pair_clearance=.05,wall_clearance=.05,physics_tick=0,params=NS(timestep=.01,vehicle_length=.51,vehicle_width=.27))
    selector=NS(body=NS(length=.51,width=.27,radius=radius),sim=sim,controller=NS(config=NS(**config)),
        pair_margin=.09,wall_margin=.07,sample_dt=.05,dt=.01,sample_steps=5,steer_ticks=4,held_steering=0.)
    record=trace.finish_frame(frame,selector,geom);assert record['status']=='valid',record['errors']
    metadata=dict(geometry_id=geom.geometry_id,frame_id='map',producer_commit=tool.PRODUCER)
    ga=geometry_args or {}
    audit=tool.audit_two_car_frames([frame],dict(config=config,pair_clearance_m=.05),session_metadata=metadata,source_classification='synthetic',**ga)
    assert audit['contract_valid'],audit
    lengths=[] if no_plan else [dict(frame_index=0,result=evaluate(plan,geom,binding=audit['frames'][0]['plan_binding'],session_metadata=metadata))]
    return record,frame,audit,lengths


def adapt(args,**kwargs):return adapt_rollout_cost_inputs(*args,**kwargs)


def risk_policy():
    return dict(schema_version=1,policy_id='synthetic_trace_math_no_weights',experimental=True,
        components=dict(risk=dict(mode='exposure_integral',clearance_ref_m=.2,clearance_scale_m=.1,time_scale_s=.2,
            valid_clearance_m=[-10.,10.],valid_horizon_s=[.2,.2])))


def costs(result):return evaluate_costs(result['cost_features'],policy=risk_policy())


def test_complete_far_file_io_json_and_inputs_unchanged(tmp_path):
    args=fixture();before=deepcopy(args);record,frame,audit,lengths=args
    path=tmp_path/'rollout_diagnostics.jsonl';write_sidecar(path,[record]);raw=path.read_bytes()
    assert read_rollout_sidecar(path)['records']==[record]
    r=adapt_rollout_cost_inputs(path,frame,audit,lengths)
    assert path.read_bytes()==raw and args==before
    assert len(r['options'])==5 and len(r['risk_sample_series'])==5
    assert [o['candidate_id'] for o in r['options']]==[1,1,2,3,None]
    assert all(o['sample_grid_complete'] and not o['formal_comparison_qualified'] for o in r['options'])
    assert all(c['components']['risk']['raw_value']==0 for c in costs(r)['options'])
    assert all(c['experimental_total_cost'] is None for c in costs(r)['options'])
    assert r['producer_binding_verified'] is False
    assert r['geometry_evidence']['geometry_artifacts_verified'] is False
    assert json.loads(json.dumps(r,allow_nan=False))==r
    r['options'][0]['projection_inputs']['ego_pose_samples'][0][0]=999
    assert args==before


@pytest.mark.parametrize('current',[False,True])
def test_complete_contact_native_gate_preserved_and_risk_positive(current):
    args=fixture(lead_x=.8,current=current);r=adapt(args);evaluated=costs(r)
    assert r['options'][0]['native_safe'] is False
    assert (r['options'][0]['first_rejection_s']==0)==current
    assert r['options'][0]['min_pair_clearance_m']<0
    assert r['cost_features']['options'][0]['comparison_status']=='unsafe'
    assert evaluated['options'][0]['components']['risk']['raw_value']>0
    assert evaluated['options'][0]['components']['risk']['normalized_value'] is None
    assert evaluated['options'][0]['experimental_total_cost'] is None


def test_partial_retains_nonzero_partial_integral_not_full_cost():
    r=adapt(fixture(lead_x=.8,partial=True));c=costs(r)['options'][1]['components']['risk']
    assert not r['options'][1]['sample_grid_complete'] and r['options'][1]['checked_range_s']==[0.,.05]
    assert c['raw_value'] is None and c['checked_partial_integral_m2_s']>0
    assert c['normalized_value'] is None


@pytest.mark.parametrize('options', [[(1,1.),(1,1.)],[(1,1.),(1,2.)]])
def test_repeated_ids_speed_identity_and_geometry_cache(options):
    r=adapt(fixture(options=options));assert len(r['options'])==2
    assert len(set(o['evaluation_id'] for o in r['options']))==2
    assert [o['target_speed_mps'] for o in r['options']]==[x[1] for x in options]
    assert r['cost_features']['geometry_recomputation_count']==0


def test_raceline_only_no_fabricated_plan():
    r=adapt(fixture(no_plan=True,options=[(None,1.)]));o=r['options'][0]
    assert o['candidate_id'] is None and o['reference_id']=='raceline'
    assert o['plan_v2_sha256'] is None and o['candidate_v2_sha256'] is None
    assert r['binding_content_verification']=='no_plan'
    assert r['cost_features']['options'][0]['comparison_status']=='reference_only'


def test_c3_stays_reference_and_rollout_stays_separate():
    r=adapt(fixture());o=r['options'][3];cf=r['cost_features']['options'][3]
    assert o['source_path_type']=='reference_centerline' and o['reference_only']
    assert o['rollout_trajectory_type']=='modeled_ego_rollout'
    assert cf['geometry_length_scope']=='reference_geometry_only' and cf['ego_rollout_length_m'] is None
    assert cf['comparison_status']=='reference_only'


@pytest.mark.parametrize('change',['missing_sample','nan','infinity','clock','step','frame','geometry','candidate',
    'speed','evaluation_duplicate','prediction_group','native_safe','minimum','configured_pair','body','radius','spline','candidate_snapshot'])
def test_bad_joins_or_samples_rejected(change):
    args=list(fixture());r,f,a,l=args
    if change=='missing_sample':r['options'][0]['samples'].pop(1)
    elif change=='nan':r['options'][0]['samples'][0]['ego_pose'][0]=float('nan')
    elif change=='infinity':r['options'][0]['samples'][0]['signed_min_pair_clearance_m']=float('inf')
    elif change=='clock':f['ego_state']['sim_time_s']=1.
    elif change=='step':f['lead_state']['control_step']=2
    elif change=='frame':r['metadata']['frame_id']='odom'
    elif change=='geometry':r['metadata']['geometry_id']='other'
    elif change=='candidate':r['options'][0]['candidate_id']=2
    elif change=='speed':r['options'][0]['target_speed_mps']=1.1
    elif change=='evaluation_duplicate':r['options'][1]['evaluation_id']=r['options'][0]['evaluation_id']
    elif change=='prediction_group':r['options'][0]['prediction_group_id']='missing'
    elif change=='native_safe':r['options'][0]['native_safe']=False
    elif change=='minimum':r['options'][0]['signed_min_pair_clearance_m']=-3.
    elif change=='configured_pair':r['metadata']['physical_pair_clearance_m']=.3
    elif change=='body':r['metadata']['body_width_m']=.4
    elif change=='radius':r['metadata']['body_radius_m']=.9
    elif change=='spline':f['plan']['paths'][0]['spline']['coefficients'][1]=.1
    else:r['options'][0]['candidate_snapshot']['spline']['coefficients'][0]=.1
    with pytest.raises(ValueError):adapt(args)


def test_binding_and_pinned_geometry(tmp_path):
    path,pin=artifact(tmp_path)
    # Fixture artifact geometry id is g; match the length fixture identity.
    manifest=json.loads(path.read_text());manifest['geometry_id']='length-synthetic';path.write_text(json.dumps(manifest))
    import hashlib
    pin=hashlib.sha256(path.read_bytes()).hexdigest()
    ga=dict(geometry_manifest_path=path,expected_manifest_sha256=pin)
    args=fixture(geometry_args=ga);bound=args[2]['frames'][0]['plan_binding']
    r=adapt(args,plan_binding=bound,**ga)
    assert r['geometry_evidence']['geometry_artifacts_verified'] and not r['producer_binding_verified']
    assert not r['formal_comparison_qualified']
    bad=deepcopy(bound);bad['plan_sha256']='0'*64
    with pytest.raises(ValueError):adapt(args,plan_binding=bad,**ga)
    with pytest.raises(ValueError):adapt(args,geometry_manifest_path=path,expected_manifest_sha256='0'*64)
    (path.parent/'reference_curve.json').write_text('changed')
    with pytest.raises(ValueError):adapt(args,**ga)


def test_prebuilt_features_and_no_total_even_native_reports_complete():
    args=fixture(recorded_complete=True);old=build_cost_features(args[2],args[3]);before=deepcopy(old);r=adapt(args,existing_cost_features=old)
    assert old==before
    assert r['cost_features']['options'][0]['upstream_comparison_status']=='features_complete_policy_required'
    assert r['cost_features']['options'][0]['comparison_status']=='unknown'
    assert all(not o['formal_comparison_qualified'] for o in r['options'])
    old['input_content_sha256']['audit']='0'*64
    with pytest.raises(ValueError):adapt(args,existing_cost_features=old)


def test_requested_grid_mismatch_not_silently_rewritten():
    args=list(fixture());args[2]['options'][0]['requested_horizon_s']=.1
    with pytest.raises(ValueError,match='native_group_horizon_mismatch'):adapt(args)


def test_projection_data_preserved_and_no_selection_keys():
    args=fixture();r=adapt(args);p=r['options'][0]['projection_inputs']
    assert p['source_ego_frenet']==args[1]['ego_state']['frenet']
    assert p['sample_times_s']==[0.,.05,.1,.15000000000000002,.2]
    assert not p['projection_performed'] and p['projection_origin']['ego_s_unwrapped']==0.
    assert 'selected_candidate_id' not in json.dumps(r)
    assert all(o['inspection_only'] and o['planning_allowed'] is False for o in r['options'])


def test_no_model_or_geometry_execution(monkeypatch):
    args=fixture()
    from f1tenth_benchmarks.research.overtaking import path_length_feature_v1
    import builtins
    def fail(*a,**k):raise AssertionError('forbidden execution')
    monkeypatch.setattr(path_length_feature_v1,'evaluate_path_length_features',fail)
    original_import=builtins.__import__
    def no_prediction_or_collision(name,globals=None,locals=None,fromlist=(),level=0):
        forbidden=('prediction_v1','collision_v1')
        if name.split('.')[-1] in forbidden or any(x in forbidden for x in (fromlist or ())):
            fail()
        return original_import(name,globals,locals,fromlist,level)
    monkeypatch.setattr(builtins,'__import__',no_prediction_or_collision)
    adapt(args)


def test_distinct_prediction_groups_kept_not_merged():
    args=list(fixture());r=args[0];other=deepcopy(r['lead_predictions'][0]);other['prediction_group_id']='second-group'
    r['lead_predictions'].append(other)
    r['options'][1]['prediction_group_id']='second-group'
    result=adapt(args)
    assert result['risk_sample_series'][0]['provenance']['prediction_group_id']!=result['risk_sample_series'][1]['provenance']['prediction_group_id']
    assert result['options'][1]['projection_inputs']['prediction_group_id']=='second-group'


def test_explicit_experimental_policy_cannot_upgrade_unverified_source():
    result=adapt(fixture(recorded_complete=True));p=risk_policy()
    p['weights']={'risk':1.}
    p['comparison']=dict(frame_index=0,option_indices=[0,1],audit_context_sha256=result['cost_features']['audit_context_sha256'],
        goal_mode='common_cartesian_endpoints',endpoint_tolerance_m=1e-6,common_requested_horizon_s=.2,sample_dt_s=.05,
        accept_audit_time_binding=True,allow_synthetic_geometry=True)
    ev=evaluate_costs(result['cost_features'],policy=p)
    assert ev['comparison_errors'] and all(o['experimental_total_cost'] is None for o in ev['options'])


def test_configured_names_do_not_become_measured_values():
    args=list(fixture());md=args[0]['metadata']
    for name in ('pair','wall'):
        assert md['configured_'+name+'_clearance_m']==.05
        assert md['physical_'+name+'_clearance_m'] is None
    result=adapt(args);o=result['options'][0]
    assert o['configured_pair_clearance_m']==.05 and o['physical_pair_clearance_m'] is None
    assert o['min_pair_clearance_m']!=.05


def test_jsonl_source_ambiguity_rejected(tmp_path):
    args=fixture();p=tmp_path/'duplicate.jsonl';write_sidecar(p,[args[0],args[0]])
    with pytest.raises(ValueError,match='ambiguous'):adapt_rollout_cost_inputs(p,*args[1:])


def test_single_source_audit_is_explicit_not_silent_partial_batch():
    args=list(fixture());a=args[2];extra=deepcopy(a['options'][0]);extra['frame_index']=5;a['options'].append(extra)
    with pytest.raises(ValueError,match='single_source_audit'):adapt(args)


def test_missing_audit_binding_rejected():
    args=list(fixture());args[2]['frames'][0]['plan_binding']=None
    with pytest.raises(ValueError,match='missing_audit_binding'):adapt(args)


def test_missing_frenet_anchors_are_null_not_zero():
    r=adapt(fixture(frenet_available=False));p=r['options'][0]['projection_inputs']
    assert p['source_ego_frenet'] is None and p['source_lead_frenet'] is None
    assert p['projection_origin']['ego_s_unwrapped'] is None
    assert p['projection_origin']['lead_s_unwrapped'] is None
    assert not p['projection_performed']


def test_complete_source_frame_content_is_bound_to_audit():
    args=list(fixture());args[1]['unexpected_changed_annotation']='changed after audit'
    with pytest.raises(ValueError,match='source_frame_fingerprint_mismatch'):adapt(args)


@pytest.mark.parametrize('change',['model','lead_pose','physics_dt'])
def test_model_and_native_lead_prediction_must_match(change):
    args=list(fixture());r=args[0]
    if change=='model':r['lead_predictions'][0]['model']='different_prediction_model'
    elif change=='physics_dt':r['metadata']['physics_dt_s']=.02
    else:
        r['lead_predictions'][0]['poses'][1][1]=1.
        for o in r['options']:o['samples'][1]['lead_pose'][1]=1.
    with pytest.raises(ValueError):adapt(args)


def test_legacy_v1_configured_settings_are_not_measurements():
    args=list(fixture());md=args[0]['metadata']
    for name in ('pair','wall'):
        md['physical_'+name+'_clearance_m']=md.pop('configured_'+name+'_clearance_m')
    row=adapt(args)['options'][0]
    assert row['configured_pair_clearance_m']==.05
    assert row['physical_pair_clearance_m'] is None
    assert row['configured_margin_field_origins']['pair'].startswith('legacy_v1_setting:')
