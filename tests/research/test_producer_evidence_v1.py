"""Synthetic data-contract tests; no native Selector, controller or simulation."""
from copy import deepcopy
from types import SimpleNamespace as NS
import json,math
import numpy as np
import pytest
from f1tenth_benchmarks.research.overtaking import producer_evidence_v1 as module
from f1tenth_benchmarks.research.overtaking.rollout_diagnostics_v1 import RolloutDiagnostics,validate_record


def fixture(*,lead_x=3.,partial=False,current=False,no_plan=False,options=None,mixed=False,rejected=False):
    """Create an artificial saved-format record through the actual schema writer.

    Clearances are arbitrary synthetic diagnostics, not outputs of a vehicle model.
    The only 'pp' object is a namespace of data; no PP algorithm is executed.
    """
    if current:lead_x=.2
    ego=dict(pose=[0.,0.,0.],speed_mps=1.,control_step=2,sim_time_s=10.,valid=True)
    lead=dict(pose=[lead_x,0.,0.],speed_mps=0.,control_step=2,sim_time_s=10.,valid=True)
    paths=[dict(candidate_id=cid,path_type='reference_centerline' if cid==3 else 'state_matched_trajectory',
        geometry_id='producer-core-synthetic',sampled_valid=not(rejected and cid==1),
        footprint_sampled_valid=not(rejected and cid==1),reason='native_rejected' if rejected and cid==1 else None,
        points=[dict(x=0.,y=0.,yaw=0.,valid=True)],reference_s_origin=0.,
        output_representation='analytic_spline_and_single_start_point',
        spline=dict(knots=[0.]*6+[3.]*6,coefficients=[0.]*6,degree=5)) for cid in range(1,6)]
    plan=None if no_plan else dict(plan_id=1,source_control_step=2,source_sim_time_s=10.,source_pose=ego['pose'][:],
        geometry_id='producer-core-synthetic',generator_settings={'lookahead_m':3.},paths=paths)
    trace=RolloutDiagnostics(source_classification='synthetic_core_contract_fixture');trace.begin_frame(ego,lead)
    trace.source_runtime=dict(source_physics_tick=0,held_steering=0.,frame_id='map',
        source_physical_state=np.array([0.,0.,0.,1.,0.,0.,0.]),source_steering_buffer=np.zeros(2))
    radius=math.hypot(.51/6,.27/2)
    config=dict(lookahead_m=3.,footprint_length_m=.51,footprint_width_m=.27,footprint_clearance_m=.05,control_dt_s=.1)
    selector=NS(body=NS(length=.51,width=.27,radius=radius),pair_margin=.09,wall_margin=.07,sample_dt=.05,
        dt=.01,sample_steps=5,steer_ticks=4,held_steering=0.,controller=NS(config=NS(**config)),
        sim=NS(params=NS(timestep=.01,vehicle_length=.51,vehicle_width=.27),pair_clearance=.05,wall_clearance=.05,physics_tick=0))
    geometry=NS(geometry_id='producer-core-synthetic',frame_id='map')
    raw=[];times=np.arange(5)*.05
    poses=np.tile(lead['pose'],(len(times),1));trace.prediction(times,poses,0.)
    options=options if options is not None else [(1,1.),(1,2.),(3,1.),(None,1.)]
    for index,(cid,speed) in enumerate(options):
        if mixed and index==1:
            times=np.arange(3)*.05;poses=np.tile(lead['pose'],(len(times),1));trace.prediction(times,poses,0.)
        pp=NS(path=np.array([[0.,0.],[1.5,0.],[3.,0.]]),speed=speed,lookahead=.5,wheelbase=.33,max_steer=.42)
        trace.start_option(cid,speed,path=paths[cid-1] if cid is not None else None,
            reference_id='raceline' if cid is None else None,reference_xy=pp.path if cid is None else None)
        reject=rejected and cid==1
        minimum=None;first=None
        if not reject:
            trace.rollout_start(pp)
            used=times[:2] if partial else times
            for k,t in enumerate(used):
                clearance=-.1 if current or lead_x<1. else .8-speed*float(t)
                if clearance<=.09 and first is None:first=float(t)
                minimum=clearance if minimum is None else min(minimum,clearance)
                trace.sample(k,t,np.array([speed*t,0.,0.,speed,0.,0.,0.]),clearance,False)
                trace.integrated(float(t))
            if partial:trace.stopped(float(times[2]),'declared_synthetic_failure')
        reason='native_rejected' if reject else 'synthetic_partial' if partial else 'synthetic_contact' if first is not None else None
        option=dict(candidate_id=cid,target_speed_mps=speed,safe=reason is None,reason=reason,
            first_rejection_s=first,min_pair_clearance_m=minimum,relative_s_progress_m=0.)
        trace.finish_option(option);raw.append(option)
    transition=dict(plan=plan,costs={'options':raw})
    record=trace.finish_frame(transition,selector,geometry);validate_record(record)
    return record,transition

fixtures=NS(fixture=fixture)


def metadata(record):
    return dict(geometry_id=record['metadata']['geometry_id'],frame_id=record['metadata']['frame_id'],
        producer_commit='a'*40,source_file_sha256={'synthetic_fixture.py':'b'*64},
        physics_parameters={'timestep':.01},vehicle_parameters={'length':.51,'width':.27},source_classification='synthetic')


def make_capture(record,meta=None):
    # Replay artificial protocol data to test schema rules, not historical timing.
    meta=metadata(record) if meta is None else meta
    c=module.ProducerCaptureV1(record['source'],meta,record['metadata'],length_m=3.)
    if record['plan_snapshot'] is not None:c.capture_plan(record['plan_snapshot'])
    for group in record['lead_predictions']:c.capture_group(group)
    paths={p['tracking_path_id']:p['xy'] for p in record['tracking_paths']}
    for o in record['options']:
        c.capture_option(o)
        if o['rollout_attempted']:
            c.capture_rollout_input(o['evaluation_id'],NS(path=np.asarray(paths[o['tracking_path_id']]),**o['pp_settings']))
    return c,meta


def evidence_case(**kwargs):
    args=fixture(**kwargs);capture,meta=make_capture(args[0]);return args,capture.finalize(args[0]),meta

def reseal(e):
    e.pop('evidence_sha256',None);e['evidence_sha256']=module.content_digest(module.SCHEMA,e)

def verify(e,r,m):return module.verify_producer_evidence_v1(e,r,session_metadata=m)


def test_complete_content_roundtrip_and_no_safety_or_authentication_upgrade():
    args,e,m=evidence_case();before=deepcopy((args,e,m));v=verify(e,args[0],m)
    assert v['status']=='locally_verified_content'
    assert v['new_capture_contract_compliant'] and not v['historical_authentication_verified']
    assert not v['capture_timing_independently_authenticated'] and not v['formal_comparison_qualified']
    assert v['code_artifacts_status']=='recorded_hash_claims_not_reverified'
    assert json.loads(json.dumps(e,allow_nan=False))==e
    assert (args,e,m)==before
    assert e['plan_binding_v2']['binding_version']==2


def test_snapshot_is_immutable_and_independent_of_original_arrays():
    args=fixtures.fixture();r=deepcopy(args[0]);c,m=make_capture(r)
    with pytest.raises(TypeError):c._source['source_sim_time_s']=99.
    with pytest.raises(TypeError):c._plan['paths'][0]['candidate_id']=99
    first=next(iter(c._rollouts.values()))
    with pytest.raises(ValueError):first['xy'][0,0]=99.
    r['plan_snapshot']['paths'][0]['spline']['coefficients'][0]=99.
    e=c.finalize(args[0]);assert e['plan_snapshot']==args[0]['plan_snapshot']
    with pytest.raises(ValueError,match='finalized'):c.capture_plan(args[0]['plan_snapshot'])


def test_capture_hot_path_cannot_hash_canonicalize_bind_or_write(monkeypatch):
    r=fixtures.fixture()[0]
    def forbidden(*a,**k):raise AssertionError('expensive hot path operation')
    with monkeypatch.context() as patch:
        for name in ('content_digest','snapshot_sha256','build_plan_binding'):
            patch.setattr(module,name,forbidden)
        patch.setattr(module.json,'dumps',forbidden)
        patch.setattr(module.Path,'open',forbidden)
        c,m=make_capture(r)
    assert c.finalize(r)['digest_phase']=='after_native_step'


@pytest.mark.parametrize('field',['source','geometry','frame','speed','group','candidate','plan','runtime','phase','digest','missing_digest','duplicate','sequence','tracking','binding','code'])
def test_mismatches_rejected_even_when_outer_digest_is_recomputed(field):
    args,e,m=evidence_case();r=args[0]
    if field=='source':e['source']['source_sim_time_s']+=.1
    elif field=='geometry':e['session_metadata']['geometry_id']='wrong'
    elif field=='frame':e['session_metadata']['frame_id']='odom'
    elif field=='speed':e['options'][0]['target_speed_mps']+=1.
    elif field=='group':e['options'][0]['prediction_group_id']='wrong'
    elif field=='candidate':e['options'][0]['candidate_id']=2
    elif field=='plan':e['plan_snapshot']['paths'][0]['spline']['coefficients'][0]+=.1
    elif field=='runtime':e['runtime_metadata']['physics_dt_s']=.02
    elif field=='phase':e['capture_kind']='audit_time_reconstruction'
    elif field=='digest':e['source_snapshot_sha256']='0'*64
    elif field=='missing_digest':e['source_snapshot_sha256']=None
    elif field=='duplicate':e['options'].append(deepcopy(e['options'][0]))
    elif field=='sequence':e['options'][0]['capture_sequence']=0
    elif field=='tracking':next(iter(e['rollout_inputs'].values()))['xy'][0][0]+=1.
    elif field=='binding':e['plan_binding_v2']['plan_sha256']='0'*64
    else:e['session_metadata']['source_file_sha256']['synthetic_fixture.py']='0'*64
    reseal(e)
    with pytest.raises(ValueError):verify(e,r,m)


@pytest.mark.parametrize('field',list(module.CONTEXT_KEYS))
def test_missing_context_is_not_invented(field):
    r=fixtures.fixture()[0];m=metadata(r);m.pop(field);c,_=make_capture(r,m)
    with pytest.raises(ValueError,match='missing_context'):c.finalize(r)


def test_writer_exclusive_and_import_has_no_side_effects(tmp_path):
    args,e,m=evidence_case();p=tmp_path/'producer.jsonl';module.write_producer_evidence(p,[e])
    assert json.loads(p.read_text())==e
    with pytest.raises(FileExistsError):module.write_producer_evidence(p,[e])
    import importlib
    before=set(tmp_path.iterdir());importlib.reload(module);assert set(tmp_path.iterdir())==before


def test_capture_requires_plan_and_rejects_duplicates():
    r=fixtures.fixture()[0];c=module.ProducerCaptureV1(r['source'],metadata(r),r['metadata'])
    c.capture_group(r['lead_predictions'][0])
    with pytest.raises(ValueError,match='missing_pre_evaluation_plan'):c.capture_option(r['options'][0])
    c.capture_plan(r['plan_snapshot']);c.capture_option(r['options'][0])
    with pytest.raises(ValueError,match='duplicate_evaluation_id'):c.capture_option(r['options'][0])


def test_raceline_can_precede_a_later_generated_candidate_plan():
    r=fixtures.fixture(options=[(None,1.),(1,1.)])[0];m=metadata(r)
    c=module.ProducerCaptureV1(r['source'],m,r['metadata'],length_m=3.)
    c.capture_group(r['lead_predictions'][0]);paths={p['tracking_path_id']:p['xy'] for p in r['tracking_paths']}
    for index,option in enumerate(r['options']):
        if index==1:c.capture_plan(r['plan_snapshot'])
        c.capture_option(option)
        c.capture_rollout_input(option['evaluation_id'],NS(path=np.asarray(paths[option['tracking_path_id']]),**option['pp_settings']))
    evidence=c.finalize(r)
    assert evidence['options'][0]['plan_present_at_capture'] is False
    assert evidence['options'][1]['plan_present_at_capture'] is True
    verify(evidence,r,m)


@pytest.mark.parametrize('field',['planning','schema','digest'])
def test_writer_rejects_invalid_evidence_before_creating_output(tmp_path,field):
    args,e,m=evidence_case();p=tmp_path/'invalid.jsonl'
    if field=='planning':e['planning_allowed']=True
    elif field=='schema':e['schema']='unknown'
    else:e['evidence_sha256']='0'*64
    with pytest.raises(ValueError):module.write_producer_evidence(p,[e])
    assert not p.exists()


@pytest.mark.parametrize('key',['options','groups','rollout_inputs','runtime_metadata'])
def test_missing_structure_is_value_error_not_legacy_fallback(key):
    args,e,m=evidence_case();e.pop(key);reseal(e)
    with pytest.raises(ValueError):verify(e,args[0],m)


@pytest.mark.parametrize('kwargs',[{'partial':True},{'current':True},{'rejected':True},
    {'no_plan':True,'options':[(None,1.)]}])
def test_partial_unsafe_rejected_and_reference_are_not_upgraded(kwargs):
    args,e,m=evidence_case(**kwargs);before=deepcopy(args);result=verify(e,args[0],m)
    assert args==before and not result['formal_comparison_qualified']
    assert result['producer_binding_verified'] is False
    if kwargs.get('partial'):assert all(not o['evaluation_complete'] for o in args[0]['options'])
    if kwargs.get('current'):assert all(o['native_safe'] is False for o in args[0]['options'])
    if kwargs.get('rejected'):
        assert args[0]['plan_snapshot']['paths'][0]['sampled_valid'] is False
        assert args[0]['options'][0]['native_reason']=='native_rejected'
        assert args[0]['options'][0]['samples']==[]
    if kwargs.get('no_plan'):assert e['plan_binding_v2'] is None
    for o in args[0]['options']:
        if o['candidate_id']==3:assert o['reference_only'] and o['source_path_type']=='reference_centerline'


def test_each_prediction_group_keeps_its_own_grid():
    args,e,m=evidence_case(mixed=True);r=args[0]
    assert len(r['lead_predictions'])==2
    assert r['options'][0]['requested_horizon_s']==.2 and r['options'][1]['requested_horizon_s']==.1
    assert r['options'][0]['prediction_group_id']!=r['options'][1]['prediction_group_id']
    assert [(o['candidate_id'],o['target_speed_mps']) for o in r['options']][:2]==[(1,1.),(1,2.)]
    assert verify(e,r,m)['status']=='locally_verified_content'


def test_source_archive_no_git_and_no_runtime_imports(tmp_path,monkeypatch):
    import subprocess,builtins,importlib.util
    args,e,m=evidence_case();before=deepcopy((args,e,m));original=builtins.__import__
    denied={'two_car_decision','two_car_env','dynamic_models','discs','GlobalPurePursuit',
        'run_recorded_candidate_pp','cost_feature_v1','cost_evaluation_v1','rollout_cost_adapter_v1',
        'model_provenance_v1'}
    def forbidden(*a,**k):raise AssertionError('core must not execute Git or native models')
    def guarded(name,globals=None,locals=None,fromlist=(),level=0):
        if name.split('.')[-1] in denied or any(n in denied for n in (fromlist or ())):forbidden()
        return original(name,globals,locals,fromlist,level)
    monkeypatch.setattr(subprocess,'check_output',forbidden);monkeypatch.setattr(subprocess,'Popen',forbidden)
    monkeypatch.setattr(builtins,'__import__',guarded);monkeypatch.chdir(tmp_path)
    spec=importlib.util.spec_from_file_location('f1tenth_benchmarks.research.overtaking._producer_core_probe',module.__file__)
    fresh=importlib.util.module_from_spec(spec);spec.loader.exec_module(fresh)
    assert not hasattr(fresh,'ProducerEvidenceDiagnosticsV1') and not hasattr(fresh,'selector_runtime_metadata')
    assert fresh.verify_producer_evidence_v1(e,args[0],session_metadata=m)['status']=='locally_verified_content'
    assert (args,e,m)==before and not list(tmp_path.iterdir())


def test_pp_input_is_data_only_and_sidecar_is_not_modified():
    args=fixture();r=deepcopy(args[0]);c,m=make_capture(r)
    before=module.snapshot_sha256(r);e=c.finalize(r)
    assert module.snapshot_sha256(r)==before and r==args[0]
    assert e['sidecar_record_sha256']==before
    assert e['rollout_inputs'] and not e['formal_comparison_qualified']
    assert all(v['xy'] for v in e['rollout_inputs'].values())


@pytest.mark.parametrize('method,value',[('capture_group',{}),('capture_option',{})])
def test_missing_capture_identity_has_explicit_rejection(method,value):
    r=fixture()[0];c=module.ProducerCaptureV1(r['source'],metadata(r),r['metadata'])
    with pytest.raises(ValueError,match='capture_missing'):getattr(c,method)(value)


def legacy(record,meta):
    # Artificial legacy shape, never historical ESP timing evidence.
    source_hash=module.snapshot_sha256(record['source'])
    bound=module.build_plan_binding(record['plan_snapshot'],session_metadata=meta,length_m=3.)
    saved=dict(schema='esp_producer_evidence_v1',schema_version=1,source=deepcopy(record['source']),
        plan=dict(capture_phase='after_native_generation_before_selector',plan_binding_v2=bound,
            plan_snapshot_sha256=module.snapshot_sha256(record['plan_snapshot']),source_snapshot_sha256=source_hash),
        options=[],errors=[],inspection_only=True,planning_allowed=False,historical_authentication_verified=False)
    for option in record['options']:
        group=next(g for g in record['lead_predictions'] if g['prediction_group_id']==option['prediction_group_id'])
        event={k:option[k] for k in ('evaluation_id','prediction_group_id','candidate_id','reference_id','target_speed_mps')}
        event.update(capture_phase='native_start_option_before_rollout',source_snapshot_sha256=source_hash,
            plan_v2_sha256=bound['plan_sha256'],candidate_snapshot_sha256=option['candidate_snapshot_sha256'],
            lead_group_sha256=module.snapshot_sha256(group))
        saved['options'].append(dict(event=event,event_sha256=module.snapshot_sha256(event)))
    return saved


def test_legacy_is_opt_in_and_never_relabelled_as_new_capture():
    args,_,m=evidence_case();old=legacy(args[0],m);before=deepcopy(old)
    with pytest.raises(ValueError,match='no_legacy_downgrade'):
        module.verify_optional_evidence(old,args[0],session_metadata=m)
    result=module.verify_optional_evidence(old,args[0],session_metadata=m,allow_legacy_esp=True)
    assert result['status']=='legacy_content_verified' and old==before
    assert result['new_capture_contract_compliant'] is False
    assert result['historical_authentication_verified'] is False and not result['formal_comparison_qualified']


@pytest.mark.parametrize('field',['time','plan_hash','source_hash','phase','candidate','speed','group',
    'event_hash','binding','count','duplicate','flags','version','geometry','frame'])
def test_legacy_malformed_data_not_accepted(field):
    args,_,m=evidence_case();r=args[0];e=legacy(r,m);event=e['options'][0]['event']
    if field=='time':e['source']['source_sim_time_s']+=1.
    elif field=='plan_hash':e['plan']['plan_snapshot_sha256']='0'*64
    elif field=='source_hash':e['plan']['source_snapshot_sha256']='0'*64
    elif field=='phase':event['capture_phase']='after_evaluation'
    elif field=='candidate':event['candidate_id']=5
    elif field=='speed':event['target_speed_mps']+=1.
    elif field=='group':event['prediction_group_id']='bad'
    elif field=='event_hash':e['options'][0]['event_sha256']='0'*64
    elif field=='binding':e['plan']['plan_binding_v2']['plan_sha256']='0'*64
    elif field=='count':e['options'].pop()
    elif field=='duplicate':e['options'][1]=deepcopy(e['options'][0])
    elif field=='flags':e['historical_authentication_verified']=True
    elif field=='version':e['schema_version']=True
    elif field=='geometry':m['geometry_id']='bad'
    else:m['frame_id']='odom'
    if field!='event_hash':e['options'][0]['event_sha256']=module.snapshot_sha256(event)
    with pytest.raises(ValueError):module.verify_optional_evidence(e,r,session_metadata=m,allow_legacy_esp=True)


@pytest.mark.parametrize('field',['schema','version','missing_digest'])
def test_malformed_new_evidence_cannot_fall_back_to_legacy(field):
    args,e,m=evidence_case()
    if field=='schema':e['schema']='unknown'
    elif field=='version':e['schema_version']=2
    else:e.pop('evidence_sha256')
    with pytest.raises(ValueError):module.verify_optional_evidence(e,args[0],session_metadata=m,allow_legacy_esp=True)


def test_absent_evidence_retains_missing_unverified_status():
    args,_,m=evidence_case();result=module.verify_optional_evidence(None,args[0],session_metadata=m)
    assert result['status']=='missing' and result['producer_binding_verified'] is False
    assert not result['formal_comparison_qualified']


def test_malformed_legacy_object_rejected_as_value_error():
    args,_,m=evidence_case()
    with pytest.raises(ValueError):module.verify_legacy_esp_evidence(None,args[0],session_metadata=m)


@pytest.mark.parametrize('field',['session','runtime'])
def test_missing_context_container_is_explicitly_rejected(field):
    r=fixture()[0];m=metadata(r)
    c=module.ProducerCaptureV1(r['source'],None if field=='session' else m,None if field=='runtime' else r['metadata'])
    with pytest.raises(ValueError,match='missing_.*metadata'):c.finalize(r)


def test_missing_pp_path_is_explicitly_rejected():
    r=fixture()[0];c=module.ProducerCaptureV1(r['source'],metadata(r),r['metadata'])
    c.capture_plan(r['plan_snapshot']);c.capture_group(r['lead_predictions'][0]);c.capture_option(r['options'][0])
    with pytest.raises(ValueError,match='capture_missing_pp_path'):c.capture_rollout_input(r['options'][0]['evaluation_id'],NS())
