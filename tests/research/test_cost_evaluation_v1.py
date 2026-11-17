"""Explicit synthetic policies demonstrate math, not recommended weights."""
from copy import deepcopy
import json

import pytest
from test_cost_feature_v1 import artifacts, risk_series
from f1tenth_benchmarks.research.overtaking.cost_feature_v1 import build_cost_features
from f1tenth_benchmarks.research.overtaking.cost_evaluation_v1 import evaluate_costs


def features(*,complete=True,samples=True):
    a,l=artifacts(complete=complete)
    return build_cost_features(a,l,risk_sample_series=[risk_series(a,i) for i in (0,1)] if samples else None)


def policy(f):
    return dict(schema_version=1,policy_id='synthetic_math_example_not_production',experimental=True,
        components=dict(length=dict(scale_m=3.,valid_length_m=[0.,10.]),
            progress=dict(mode='negative_relative_progress',scale_m=.1,valid_progress_m=[-3.,3.]),
            risk=dict(mode='exposure_integral',clearance_ref_m=.2,clearance_scale_m=.1,time_scale_s=.1,
                      valid_clearance_m=[-1.,10.],valid_horizon_s=[.1,.1])),
        weights=dict(length=1.,risk=2.,progress=.5),
        comparison=dict(frame_index=0,option_indices=[0,1],audit_context_sha256=f['audit_context_sha256'],
            goal_mode='common_cartesian_endpoints',endpoint_tolerance_m=1e-6,
            common_requested_horizon_s=.1,sample_dt_s=.05,accept_audit_time_binding=True,allow_synthetic_geometry=True))


def test_no_policy_no_invented_weights_or_scores():
    f=features();r=evaluate_costs(f)
    assert r['policy'] is None
    assert all(o['experimental_total_cost'] is None and o['components']=={} for o in r['options'])
    assert 'selected_candidate_id' not in r
    assert r['options'][0]['raw_features']['geometry_length_m']==pytest.approx(3.)


def test_explicit_dimensionless_components_and_no_selection():
    f=features();p=policy(f);before=deepcopy((f,p));r=evaluate_costs(f,policy=p);o=r['options'][0]
    assert (f,p)==before
    assert o['components']['length']['raw_value']==pytest.approx(3.)
    assert o['components']['length']['normalized_value']==pytest.approx(1.)
    assert o['components']['progress']['raw_value']==pytest.approx(-.1)
    assert o['components']['progress']['normalized_value']==pytest.approx(-1.)
    assert o['components']['risk']['raw_value']==pytest.approx(.000125)
    assert o['components']['risk']['normalized_value']==pytest.approx(.125)
    assert o['experimental_total_cost']==pytest.approx(.75)
    assert r['options'][1]['experimental_total_cost']==pytest.approx(.75)
    assert r['options'][2]['experimental_total_cost'] is None
    assert not any('selected' in k or 'rank' in k for k in r)
    assert json.loads(json.dumps(r,allow_nan=False))==r
    assert r['inspection_only'] and not r['planning_allowed']


def test_no_weights_only_components():
    f=features();p=policy(f);p.pop('weights');r=evaluate_costs(f,policy=p)
    assert r['options'][0]['components']['risk']['normalized_value']==pytest.approx(.125)
    assert all(o['experimental_total_cost'] is None for o in r['options'])


@pytest.mark.parametrize('change',['no_samples','unknown','partial','unsafe','rejected','reference','null_length',
                                  'null_progress','different_end','different_start','different_interval','different_horizon',
                                  'different_frame','different_model','different_margin','missing_binding','non_synthetic'])
def test_missing_unsafe_or_unfair_group_never_scores(change):
    f=features(samples=change!='no_samples',complete=change!='unknown');o=f['options'][0];p=policy(f)
    if change=='partial':
        o['risk_series'].update(status='reported_partial',evaluation_complete=False,samples=o['risk_series']['samples'][:2])
    elif change in ('unsafe','rejected'):o['comparison_status']=change
    elif change=='reference':p['comparison']['option_indices']=[0,3]
    elif change=='null_length':o['geometry_length_m']=None
    elif change=='null_progress':o['relative_s_progress_m']=None
    elif change=='different_end':o['geometry_end_xy'][1]=.01
    elif change=='different_start':o['geometry_start_xy'][1]=.01
    elif change=='different_interval':o['reference_interval_s_m'][1]=3.1
    elif change=='different_horizon':o['requested_horizon_s']=.2
    elif change=='different_frame':o['frame_id']='odom'
    elif change=='different_model':o['model']='single_circle'
    elif change=='different_margin':o['pair_threshold_m']=.01
    elif change=='missing_binding':o['binding_status']='unknown'
    elif change=='non_synthetic':f['source_classification']='recorded'
    r=evaluate_costs(f,policy=p)
    assert r['comparison_errors']
    assert all(x['experimental_total_cost'] is None for x in r['options'])
    if change=='partial':
        c=r['options'][0]['components']['risk']
        assert c['raw_value'] is None and c['normalized_value'] is None
        assert c['checked_partial_integral_m2_s']==pytest.approx(.0000625)


def test_minimum_diagnostic_is_not_exposure_and_signed_is_preserved():
    f=features(samples=False);f['options'][0].update(comparison_status='unsafe',min_pair_clearance_m=-.03)
    p=policy(f);p['components']['risk']=dict(mode='minimum_shortfall_squared',clearance_ref_m=.2,
        clearance_scale_m=.1,valid_clearance_m=[-1.,10.])
    r=evaluate_costs(f,policy=p);c=r['options'][0]['components']['risk']
    assert c['raw_value']==pytest.approx(.23**2) and c['raw_unit']=='m^2'
    assert c['normalized_value'] is None
    assert r['options'][0]['raw_features']['min_pair_clearance_m']==-.03
    assert 'not exposure' in c['interpretation']


@pytest.mark.parametrize('change',['scale','weights_missing_component','negative_weight','all_zero_weights','nan_weight',
    'mode','risk_reference','range','experimental','version','policy_id','extra_component'])
def test_invalid_policy_is_rejected(change):
    f=features();p=policy(f)
    if change=='scale':p['components']['length']['scale_m']=0
    elif change=='weights_missing_component':p['weights'].pop('risk')
    elif change=='negative_weight':p['weights']['risk']=-1
    elif change=='all_zero_weights':p['weights']={k:0 for k in p['weights']}
    elif change=='nan_weight':p['weights']['risk']=float('nan')
    elif change=='mode':p['components']['progress']['mode']='integrated_s'
    elif change=='risk_reference':p['components']['risk']['clearance_ref_m']=-1
    elif change=='range':p['components']['length']['valid_length_m']=[10,0]
    elif change=='experimental':p['experimental']=False
    elif change=='version':p['schema_version']=2
    elif change=='policy_id':p['policy_id']=''
    else:p['components']['time']={}
    with pytest.raises(ValueError):evaluate_costs(f,policy=p)


@pytest.mark.parametrize('change',['missing_group','duplicate_ids','missing_option','policy_clock','binding_acceptance',
    'synthetic_permission','goal_mode','large_tolerance','context','out_of_range'])
def test_explicit_policy_cannot_assert_fairness_without_evidence(change):
    f=features();p=policy(f);c=p['comparison']
    if change=='missing_group':p.pop('comparison')
    elif change=='duplicate_ids':c['option_indices']=[0,0]
    elif change=='missing_option':c['option_indices']=[0,99]
    elif change=='policy_clock':c['sample_dt_s']=.1
    elif change=='binding_acceptance':c['accept_audit_time_binding']=False
    elif change=='synthetic_permission':c['allow_synthetic_geometry']=False
    elif change=='goal_mode':c['goal_mode']='common_frenet_horizon'
    elif change=='large_tolerance':c['endpoint_tolerance_m']=1.
    elif change=='context':c['audit_context_sha256']='f'*64
    else:p['components']['length']['valid_length_m']=[0.,2.]
    r=evaluate_costs(f,policy=p)
    assert r['comparison_errors'] and all(o['experimental_total_cost'] is None for o in r['options'])


def test_clearance_profile_matters_even_same_minimum():
    a,l=artifacts();s0=risk_series(a,0);s1=risk_series(a,1)
    s1['samples'][0]['signed_clearance_m']=s1['samples'][2]['signed_clearance_m']=.15
    f=build_cost_features(a,l,risk_sample_series=[s0,s1]);r=evaluate_costs(f,policy=policy(f))
    assert f['options'][0]['min_pair_clearance_m']==f['options'][1]['min_pair_clearance_m']
    assert r['options'][1]['components']['risk']['raw_value']==pytest.approx(.00025)
    assert r['options'][0]['components']['risk']['raw_value']==pytest.approx(.000125)


def test_reference_length_never_normalized_and_no_model_calls(monkeypatch):
    f=features();p=policy(f)
    from f1tenth_benchmarks.research.overtaking import path_length_feature_v1
    def fail(*a,**kw):raise AssertionError('forbidden execution')
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
    r=evaluate_costs(f,policy=p)
    assert r['options'][3]['components']['length']['raw_value']==pytest.approx(3.)
    assert r['options'][3]['components']['length']['normalized_value'] is None


@pytest.mark.parametrize('field,value',[('evaluation_complete',False),('native_full_horizon_reported',False),
    ('checked_range_s',[0.,.05]),('native_sampled_valid',False),('footprint_sampled_valid',False),
    ('native_safe',False),('native_option_reason','failure'),('first_rejection_s',.05),
    ('reference_only',True),('model','single_circle'),('body_reference',None),('min_pair_clearance_m',-.1)])
def test_claimed_comparison_status_cannot_bypass_native_gate(field,value):
    f=features();p=policy(f);f['options'][0][field]=value
    r=evaluate_costs(f,policy=p)
    assert r['comparison_errors'] and all(x['experimental_total_cost'] is None for x in r['options'])


@pytest.mark.parametrize('value',[[],0,False,'policy'])
def test_policy_requires_object(value):
    with pytest.raises(ValueError):evaluate_costs(features(),policy=value)
