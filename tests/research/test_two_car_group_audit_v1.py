"""Synthetic saved contracts; no generator, simulation, or live control."""
from copy import deepcopy
import json

import pytest

from test_rollout_cost_adapter_v1 import fixture, risk_policy
from f1tenth_benchmarks.research.overtaking import two_car_group_audit_v1 as groups
from f1tenth_benchmarks.research.overtaking.rollout_diagnostics_v1 import validate_record, snapshot_sha256


def source(*, mixed=False, **kwargs):
    record,frame,audit,lengths=fixture(**kwargs)
    report={'config': {'lookahead_m':3.,'footprint_length_m':.51,'footprint_width_m':.27,
                       'footprint_clearance_m':.05,'control_dt_s':.1},'pair_clearance_m':.05}
    metadata=deepcopy(audit['source_metadata']);metadata['source_classification']='synthetic'
    if mixed:
        index=1;row=record['options'][index]
        old=next(g for g in record['lead_predictions'] if g['prediction_group_id']==row['prediction_group_id'])
        group=deepcopy(old);group['prediction_group_id']='synthetic:short-group'
        group['requested_sample_times_s']=group['requested_sample_times_s'][:3];group['poses']=group['poses'][:3]
        record['lead_predictions'].append(group)
        row['prediction_group_id']=group['prediction_group_id'];row['samples']=row['samples'][:3]
        row.update(requested_horizon_s=.1,checked_end_s=.1,checked_sample_count=3,last_integrated_t_s=.1,
                   requested_sample_times_s=deepcopy(group['requested_sample_times_s']))
        row['signed_min_pair_clearance_m']=min(s['signed_min_pair_clearance_m'] for s in row['samples'])
        row['wall_evaluation_complete']=all(s['wall_intersects'] is not None for s in row['samples'])
        frame['costs']['options'][index]['min_pair_clearance_m']=row['signed_min_pair_clearance_m']
    validate_record(record)
    return frame,report,metadata,record,lengths


def evaluate(args,**kwargs):
    frame,report,metadata,record,lengths=args
    return groups.evaluate_grouped_rollout(frame,report,metadata,record,
        path_length_results=lengths,risk_policy=risk_policy(),source_classification='synthetic',**kwargs)


def test_grouped_identity_native_audit_and_derived_views_are_separate():
    args=source(mixed=True);before=deepcopy(args);result=evaluate(args)
    assert args==before
    assert result['native_audit']['source_classification']=='synthetic'
    assert result['source_frame_sha256']==snapshot_sha256(args[0])
    assert result['native_audit']['input_content_fingerprints']['frames']
    assert [o['native_option_index'] for o in result['options']]==list(range(5))
    assert [(o['candidate_id'],o['target_speed_mps']) for o in result['options']]==[(1,1.),(1,2.),(2,1.),(3,1.),(None,1.)]
    assert len({o['evaluation_id'] for o in result['options']})==5
    for index,row in enumerate(result['options']):
        assert row['derivation']['native_option_index']==index
        assert row['derivation']['view_option_index']==0
        assert row['derivation']['original_source_frame_sha256']==result['source_frame_sha256']
        assert row['derived_source_frame']['costs']['options']==[args[0]['costs']['options'][index]]
        assert row['audit_result']['options'][0]['option_index']==0
        assert not row['formal_comparison_qualified']
        assert 'selected_candidate_id' not in row['derived_source_frame']
        assert all(o['experimental_total_cost'] is None for o in row['cost_components']['options'])
    assert json.loads(json.dumps(result,allow_nan=False))==result


def test_distinct_groups_keep_recorded_grids_not_native_decision_grid():
    result=evaluate(source(mixed=True));rows=result['options']
    assert rows[0]['requested_horizon_s']==.2 and rows[1]['requested_horizon_s']==.1
    assert rows[0]['prediction_group_id']!=rows[1]['prediction_group_id']
    assert rows[0]['checked_range_s']==[0.,.2] and rows[1]['checked_range_s']==[0.,.1]
    assert rows[1]['derived_source_frame']['decision']['horizon_s']==.1
    assert result['native_audit']['frames'][0]['decision_diagnostics']['horizon_s']==.2
    assert rows[1]['derivation']['grid_source']=='recorded_sidecar_not_inferred'


@pytest.mark.parametrize('kwargs',[{}, {'current':True}, {'partial':True}, {'lead_x':.8}, {'current':True,'lead_x':.8}])
def test_complete_partial_unsafe_and_contact_evidence_not_promoted(kwargs):
    args=source(**kwargs);r=evaluate(args)
    for original,row in zip(args[0]['costs']['options'],r['options']):
        assert row['native_safe']==original['safe'] and row['native_reason']==original['reason']
        assert row['min_pair_clearance_m']==original['min_pair_clearance_m']
        feature=row['adapter_result']['cost_features']['options'][0]
        assert feature['comparison_status'] in ('unknown','unsafe','reference_only','rejected')
        assert not row['adapter_result']['producer_binding_verified']
        assert all(o['experimental_total_cost'] is None for o in row['cost_components']['options'])
        if kwargs.get('partial'):
            assert not row['sample_grid_complete'] and row['checked_range_s']==[0.,.05]
    if kwargs.get('lead_x')==.8:
        assert r['options'][0]['min_pair_clearance_m']<0
        assert r['options'][0]['cost_components']['options'][0]['components']['risk']['raw_value']>0


def test_reference_centerline_and_modeled_ego_rollout_are_distinct():
    row=evaluate(source())['options'][3]
    adapted=row['adapter_result']['options'][0]
    feature=row['adapter_result']['cost_features']['options'][0]
    assert adapted['source_path_type']=='reference_centerline' and adapted['reference_only']
    assert adapted['rollout_trajectory_type']=='modeled_ego_rollout'
    assert feature['comparison_status']=='reference_only'
    assert feature['geometry_length_scope']=='reference_geometry_only'
    assert feature['ego_rollout_length_m'] is None


def test_raceline_only_does_not_fabricate_a_candidate_or_plan_binding():
    row=evaluate(source(no_plan=True,options=[(None,1.)]))['options'][0]
    adapted=row['adapter_result']['options'][0]
    assert row['candidate_id'] is None and row['reference_id']=='raceline'
    assert row['audit_result']['frames'][0]['plan_binding'] is None
    assert adapted['plan_v2_sha256'] is None and adapted['candidate_v2_sha256'] is None


def test_configured_margin_and_signed_clearance_remain_different():
    row=evaluate(source(lead_x=.8))['options'][0]['adapter_result']['options'][0]
    assert row['configured_pair_clearance_m']==.05
    assert row['physical_pair_clearance_m'] is None
    assert row['min_pair_clearance_m']<0


@pytest.mark.parametrize('change',['ego','lead','plan','count','order','speed','geometry','frame','binding'])
def test_inconsistent_source_and_option_contracts_rejected(change):
    args=list(source());f,_,m,r,l=args
    if change=='ego':f['ego_state']['sim_time_s']+=.1
    elif change=='lead':f['lead_state']['control_step']+=1
    elif change=='plan':f['plan']['source_sim_time_s']+=.1
    elif change=='count':f['costs']['options'].pop()
    elif change=='order':f['costs']['options'][0],f['costs']['options'][1]=f['costs']['options'][1],f['costs']['options'][0]
    elif change=='speed':f['costs']['options'][0]['target_speed_mps']+=.1
    elif change=='geometry':m['geometry_id']='other-geometry'
    elif change=='frame':m['frame_id']='odom'
    else:r['binding_status']='verified'
    with pytest.raises(ValueError):evaluate(args)


def test_missing_sidecar_is_unknown_not_complete():
    args=list(source());args[3]=None
    r=evaluate(args)
    assert r['status']=='unknown' and r['options']==[]
    assert r['reasons']==['no_saved_sidecar_groups']
    assert not r['formal_comparison_qualified']


def test_reuse_lengths_and_no_model_or_new_risk_integral(monkeypatch):
    args=source()
    def forbidden(*a,**k):raise AssertionError('offline adapter must not execute models or resample')
    monkeypatch.setattr(groups,'evaluate_path_length_features',forbidden)
    from f1tenth_benchmarks.research.overtaking.two_car_env import TwoCarOvertakingEnv
    from f1tenth_benchmarks.research.overtaking.two_car_decision import OvertakingSelector
    monkeypatch.setattr(TwoCarOvertakingEnv,'step',forbidden)
    monkeypatch.setattr(OvertakingSelector,'__call__',forbidden)
    original=groups.evaluate_costs;calls=[]
    def existing_evaluation(*a,**k):calls.append(1);return original(*a,**k)
    monkeypatch.setattr(groups,'evaluate_costs',existing_evaluation)
    r=evaluate(args)
    assert len(calls)==len(r['options'])
    assert r['path_length_results']==args[-1]
    assert all(row['adapter_result']['cost_features']['geometry_recomputation_count']==0 for row in r['options'])
