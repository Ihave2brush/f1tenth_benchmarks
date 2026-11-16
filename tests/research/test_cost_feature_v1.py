"""Synthetic contract/math fixtures; no ESP double-car recordings."""
from copy import deepcopy
import json

import pytest
from test_path_length_feature_v1 import case, evaluate
from test_two_car_contract_audit_v1 import tool
from f1tenth_benchmarks.research.overtaking import cost_feature_v1 as feature


def artifacts(*, complete=True, amplitude=0.):
    plan,geometry=case(amplitude=amplitude,reference=True)
    for p in plan['paths']:p['footprint_sampled_valid']=True
    ego=dict(valid=True,pose=plan['source_pose'][:],speed_mps=1.,sim_time_s=0.,control_step=0)
    lead=dict(valid=True,pose=[3.,0.,0.],speed_mps=.8,sim_time_s=0.,control_step=0)
    def option(cid,speed,**extra):
        r=dict(candidate_id=cid,target_speed_mps=speed,safe=True,reason=None,first_rejection_s=None,
               min_pair_clearance_m=.15,relative_s_progress_m=.1)
        if complete:r.update(checked_range_s=[0.,.1],evaluation_complete=True)
        r.update(extra);return r
    frame=dict(time_s=0.,ego_state=ego,lead_state=lead,plan=plan,
        costs=dict(options=[option(1,1.),option(1,.8),option(2,1.),option(3,1.),option(None,1.,reference='raceline')]),
        raceline_xy=[[0.,0.],[3.,0.]],decision=dict(horizon_s=.1,sample_dt_s=.05,pair_threshold_m=.09,
        wall_threshold_m=.07,lead_prediction_poses=[[3.,0.,0.],[3.04,0.,0.],[3.08,0.,0.]]))
    report=dict(map='synthetic',phase=2,pair_clearance_m=.05,config=dict(lookahead_m=3.,control_dt_s=.1,
        footprint_length_m=.51,footprint_width_m=.27,footprint_clearance_m=.05))
    metadata=dict(geometry_id=geometry.geometry_id,frame_id=geometry.frame_id,producer_commit=tool.PRODUCER)
    audit=tool.audit_two_car_frames([frame],report,session_metadata=metadata,source_classification='synthetic')
    assert audit['contract_valid'],audit
    lengths=evaluate(plan,geometry,binding=audit['frames'][0]['plan_binding'],session_metadata=metadata)
    assert lengths['batch_status']=='evaluated',lengths
    return audit,[dict(frame_index=0,result=lengths)]


def risk_series(audit,index=0,*,partial=False):
    o=audit['options'][index]
    keys=('frame_index','option_index','candidate_id','target_speed_mps','audit_context_sha256','plan_v2_sha256',
          'source_sim_time_s','geometry_id','frame_id','sample_dt_s','requested_horizon_s','body_length_m',
          'body_width_m','body_reference','pair_threshold_m','clearance_convention','model')
    r={k:o[k] for k in keys}
    r.update(provenance=dict(kind='synthetic',description='Artificial signed three-disc samples; no ESP rollout'),
        evaluation_complete=not partial,checked_range_s=[0.,.05 if partial else .1],
        samples=[dict(t_s=0.,signed_clearance_m=.2),dict(t_s=.05,signed_clearance_m=.15)])
    if not partial:r['samples'].append(dict(t_s=.1,signed_clearance_m=.2))
    return r


def test_join_speed_options_geometry_cache_json_and_no_mutation():
    a,l=artifacts();before=deepcopy((a,l));r=feature.build_cost_features(a,l)
    assert (a,l)==before
    assert [(o['candidate_id'],o['target_speed_mps']) for o in r['options']]==[(1,1.),(1,.8),(2,1.),(3,1.),(None,1.)]
    assert r['geometry_recomputation_count']==0 and r['unique_geometry_records_reused']==5
    assert r['options'][0]['geometry_length_m']==pytest.approx(3.)
    assert r['options'][1]['geometry_length_m']==r['options'][0]['geometry_length_m']
    assert r['options'][0]['comparison_status']=='features_complete_policy_required'
    assert all(o['inspection_only'] and not o['planning_allowed'] for o in r['options'])
    assert json.loads(json.dumps(r,allow_nan=False))==r
    r['options'][0]['checked_range_s'][0]=99
    assert (a,l)==before


def test_reference_and_raceline_not_ego_travel():
    a,l=artifacts();r=feature.build_cost_features(a,l)['options']
    assert r[3]['comparison_status']==r[4]['comparison_status']=='reference_only'
    assert r[3]['geometry_length_m']==pytest.approx(3.)
    assert r[3]['ego_rollout_length_m'] is None and r[4]['geometry_length_m'] is None


def test_unknown_native_coverage_not_upgraded_by_external_series():
    a,l=artifacts(complete=False);r=feature.build_cost_features(a,l,risk_sample_series=[risk_series(a)])['options'][0]
    assert r['comparison_status']=='unknown' and r['evaluation_complete'] is None
    assert r['risk_series']['evaluation_complete'] is True
    assert r['missing_feature_mask']['complete_checked_horizon']


@pytest.mark.parametrize('change',['safe','native','footprint','missing_minimum','negative_minimum','missing_progress','first_rejection','model','body','margin','incomplete'])
def test_native_gates_and_missing_values(change):
    a,l=artifacts();o=a['options'][0]
    if change=='safe':o.update(safe=False,reason='rejected_by_selector')
    elif change=='native':
        for row in (a['frames'][0]['candidates'][0],l[0]['result']['paths'][0]):row.update(native_sampled_valid=False,native_reason='invalid')
    elif change=='footprint':a['frames'][0]['candidates'][0]['footprint_sampled_valid']=False
    elif change=='missing_minimum':o['min_pair_clearance_m']=None
    elif change=='negative_minimum':o.update(safe=False,reason='predicted_pair_clearance',min_pair_clearance_m=-.03)
    elif change=='missing_progress':o['relative_s_progress_m']=None
    elif change=='first_rejection':o['first_rejection_s']=.05
    elif change=='model':o['model']=None
    elif change=='body':o['body_length_m']=None
    elif change=='margin':o['pair_threshold_m']=None
    else:o.update(evaluation_complete=False,checked_range_s=[0.,.05],coverage_status='reported_partial')
    r=feature.build_cost_features(a,l)['options'][0]
    assert r['comparison_status']!='features_complete_policy_required'
    if change=='negative_minimum':assert r['min_pair_clearance_m']==-.03
    if change=='missing_minimum':assert r['min_pair_clearance_m'] is None
    if change=='first_rejection':assert 'contradictory_native_safe' in r['reasons']


@pytest.mark.parametrize('change',['binding','source_time','frame','plan_digest','pose','candidate_order','duplicate_option','duplicate_length','missing_length'])
def test_binding_and_source_validation(change):
    a,l=artifacts()
    if change=='binding':l[0]['result']['plan_binding']['plan_sha256']='f'*64
    elif change=='source_time':a['options'][0]['source_sim_time_s']=1.
    elif change=='frame':a['options'][0]['frame_id']='odom'
    elif change=='plan_digest':a['options'][0]['plan_v2_sha256']='e'*64
    elif change=='pose':a['frames'][0]['ego_state']['pose'][0]=1.
    elif change=='candidate_order':l[0]['result']['paths'].reverse()
    elif change=='duplicate_option':a['options'].append(deepcopy(a['options'][0]))
    elif change=='duplicate_length':l.append(deepcopy(l[0]))
    else:l=[]
    if change.startswith('duplicate'):
        with pytest.raises(ValueError):feature.build_cost_features(a,l)
    else:
        r=feature.build_cost_features(a,l)['options'][0]
        assert (r['comparison_status']=='features_complete_policy_required')==(change=='candidate_order')


@pytest.mark.parametrize('change',['frame','time','candidate','speed','model','grid','minimum','complete','provenance','duplicate','unknown_option'])
def test_external_samples_strictly_bound(change):
    a,l=artifacts();s=risk_series(a)
    if change=='frame':s['frame_id']='odom'
    elif change=='time':s['source_sim_time_s']=1.
    elif change=='candidate':s['candidate_id']=2
    elif change=='speed':s['target_speed_mps']=.1
    elif change=='model':s['model']='single_circle'
    elif change=='grid':s['samples'][1]['t_s']=.03
    elif change=='minimum':s['samples'][1]['signed_clearance_m']=.14
    elif change=='complete':s['evaluation_complete']=False
    elif change=='provenance':s['provenance']={}
    elif change=='unknown_option':s['option_index']=99
    if change in ('duplicate','unknown_option'):
        with pytest.raises(ValueError):feature.build_cost_features(a,l,risk_sample_series=[s,s] if change=='duplicate' else [s])
    else:
        r=feature.build_cost_features(a,l,risk_sample_series=[s])['options'][0]
        assert r['comparison_status']=='rejected' and r['risk_series']['status']=='invalid'


def test_partial_external_samples_not_full_and_signed_contact_retained():
    a,l=artifacts();r=feature.build_cost_features(a,l,risk_sample_series=[risk_series(a,partial=True)])['options'][0]
    assert r['risk_series']['status']=='reported_partial'
    a['options'][0].update(safe=False,reason='contact',min_pair_clearance_m=-.02)
    s=risk_series(a);s['samples'][1]['signed_clearance_m']=-.02
    r=feature.build_cost_features(a,l,risk_sample_series=[s])['options'][0]
    assert r['comparison_status']=='unsafe' and r['risk_series']['first_envelope_contact_grid_t_s']==.05


def test_no_geometry_or_model_execution(monkeypatch):
    a,l=artifacts()
    def fail(*args,**kwargs):raise AssertionError('forbidden execution')
    from f1tenth_benchmarks.research.overtaking import path_length_feature_v1
    monkeypatch.setattr(path_length_feature_v1,'evaluate_path_length_features',fail)
    # These modules are intentionally outside the standalone Cost PR.
    import builtins
    original_import=builtins.__import__
    def guarded_import(name,globals=None,locals=None,fromlist=(),level=0):
        forbidden={'prediction_v1','collision_v1','prediction_collision_adapter_v1'}
        if name.rsplit('.',1)[-1] in forbidden or forbidden.intersection(fromlist or ()):
            raise AssertionError('cost core must not import prediction/collision')
        return original_import(name,globals,locals,fromlist,level)
    monkeypatch.setattr(builtins,'__import__',guarded_import)
    feature.build_cost_features(a,l)


@pytest.mark.parametrize('value',[{},(),'',0,False])
def test_invalid_empty_risk_container_not_treated_as_missing(value):
    a,l=artifacts()
    with pytest.raises(ValueError):feature.build_cost_features(a,l,risk_sample_series=value)
