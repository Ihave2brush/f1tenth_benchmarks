"""Versioned, explicit group projections; original Audit V1 and sources unchanged."""
from copy import deepcopy
from time import perf_counter

from tools.audit_two_car_contract_v1 import audit_two_car_frames
from .cost_feature_v1 import json_copy
from .cost_evaluation_v1 import evaluate_costs
from .path_length_feature_v1 import evaluate_path_length_features
from .rollout_cost_adapter_v1 import adapt_rollout_cost_inputs
from .rollout_diagnostics_v1 import validate_record, snapshot_sha256
from .plan_binding_v1 import _tree


def _same(a,b,reason):
    if _tree(a)!=_tree(b):raise ValueError(reason)


def evaluate_grouped_rollout(source_frame, report, metadata, sidecar_record, *, risk_policy,
                             geometry=None, path_length_results=None, geometry_manifest_path=None,
                             expected_manifest_sha256=None, geometry_curve_json=None,
                             source_classification='saved_two_car_simulation'):
    """Evaluate one saved source using derived single-option views, never replay.

    Each view records the native index plus local index 0. Its grid comes from
    the saved sidecar, NOT from inference about native decision.horizon_s.
    View fingerprints bind views, not the unmodified original frame. Raw source
    hashes and the original V1 audit remain alongside them. No native coverage
    fields or producer bindings are synthesized.
    """
    frame,rep,meta=json_copy([source_frame,report,metadata])
    kwargs=dict(session_metadata=meta,source_classification=source_classification,
                geometry_manifest_path=geometry_manifest_path,expected_manifest_sha256=expected_manifest_sha256)
    timing=dict(native_audit_s=0.,path_length_s=0.,group_audit_s=0.,adapter_s=0.,cost_evaluation_s=0.)
    start=perf_counter();native=audit_two_car_frames([frame],rep,**kwargs);timing['native_audit_s']=perf_counter()-start
    original_hash=snapshot_sha256(frame)
    result=dict(schema='two_car_group_audit_v1',schema_version=1,method='explicit_saved_group_projection',
        source_frame_sha256=original_hash,native_audit=native,source_classification=meta.get('source_classification','saved_two_car_simulation'),
        options=[],timings_s=timing,inspection_only=True,planning_allowed=False,
        producer_binding_verified=False,formal_comparison_qualified=False)
    if sidecar_record is None:
        result.update(status='unknown',reasons=['no_saved_sidecar_groups']);return result
    record=validate_record(json_copy(sidecar_record))
    _same(record['source']['ego_state'],frame.get('ego_state'),'group_ego_source_mismatch')
    _same(record['source']['lead_state'],frame.get('lead_state'),'group_lead_source_mismatch')
    _same(record.get('plan_snapshot'),frame.get('plan'),'group_plan_snapshot_mismatch')
    options=frame.get('costs',{}).get('options',[]) if isinstance(frame.get('costs'),dict) else []
    if len(options)!=len(record['options']):raise ValueError('group_option_count_mismatch')
    original_record_hash=snapshot_sha256(record)
    plan=frame.get('plan');binding=native['frames'][0].get('plan_binding')
    if plan is not None and binding is None:raise ValueError('invalid_original_plan_binding')
    if path_length_results is None:
        if plan is None:path_length_results=[]
        else:
            if geometry is None:raise ValueError('geometry_or_existing_lengths_required')
            start=perf_counter()
            measured=evaluate_path_length_features(plan,geometry,binding=binding,session_metadata=meta,
                sampling_steps_m=(.02,),geometry_curve_json=geometry_curve_json)
            timing['path_length_s']=perf_counter()-start
            if measured['batch_status']!='evaluated':raise ValueError('path_length_batch_rejected:'+str(measured['reason']))
            path_length_results=[dict(frame_index=0,result=measured)]
    result['path_length_results']=json_copy(path_length_results)
    for index,(original,trace) in enumerate(zip(options,record['options'])):
        _same(trace['option_index'],index,'group_option_order_mismatch')
        for key in ('candidate_id','target_speed_mps'):_same(original.get(key),trace.get(key),'group_identity_mismatch:'+key)
        group=next(g for g in record['lead_predictions'] if g['prediction_group_id']==trace['prediction_group_id'])
        # These are labeled derived artifacts. Never modify or relabel the
        # original decision, options, source frame, sidecar, or Audit V1.
        view=deepcopy(frame)
        for field in ('selected_candidate_id','selection_reason','actions'):view.pop(field,None)
        view['costs']=dict(options=[deepcopy(original)])
        view['decision']=dict(horizon_s=group['requested_sample_times_s'][-1],sample_dt_s=record['metadata']['sample_dt_s'],
            lead_prediction_poses=deepcopy(group['poses']),pair_threshold_m=record['metadata']['pair_threshold_m'],
            wall_threshold_m=record['metadata']['wall_threshold_m'],
            projection_scope='saved_sidecar_group_not_native_decision')
        derived=deepcopy(record)
        derived['lead_predictions']=[deepcopy(group)];derived['options']=[deepcopy(trace)]
        derived['options'][0]['option_index']=0
        provenance=dict(kind='explicit_single_option_group_view',native_option_index=index,view_option_index=0,
            evaluation_id=trace['evaluation_id'],prediction_group_id=trace['prediction_group_id'],
            original_source_frame_sha256=original_hash,original_sidecar_record_sha256=original_record_hash,
            grid_source='recorded_sidecar_not_inferred',producer_binding_verified=False)
        derived['group_projection']=deepcopy(provenance)
        validate_record(derived)
        start=perf_counter();audit=audit_two_car_frames([view],rep,**kwargs);timing['group_audit_s']+=perf_counter()-start
        if not audit['contract_valid']:raise ValueError('invalid_group_audit:'+str(audit['frames'][0]['errors']))
        start=perf_counter()
        adapted=adapt_rollout_cost_inputs(derived,view,audit,path_length_results,plan_binding=binding,
            geometry_manifest_path=geometry_manifest_path,expected_manifest_sha256=expected_manifest_sha256)
        timing['adapter_s']+=perf_counter()-start
        start=perf_counter();cost=evaluate_costs(adapted['cost_features'],policy=risk_policy);timing['cost_evaluation_s']+=perf_counter()-start
        result['options'].append(dict(native_option_index=index,evaluation_id=trace['evaluation_id'],
            prediction_group_id=trace['prediction_group_id'],candidate_id=trace['candidate_id'],reference_id=trace.get('reference_id'),
            target_speed_mps=trace['target_speed_mps'],requested_horizon_s=trace['requested_horizon_s'],
            checked_range_s=[trace['checked_start_s'],trace['checked_end_s']] if trace['samples'] else None,
            sample_grid_complete=trace['evaluation_complete'],native_safe=trace['native_safe'],native_reason=trace['native_reason'],
            min_pair_clearance_m=trace['signed_min_pair_clearance_m'],derivation=provenance,
            derived_source_frame=view,derived_sidecar=derived,audit_result=audit,adapter_result=adapted,cost_components=cost,
            formal_comparison_qualified=False,inspection_only=True,planning_allowed=False))
    result.update(status='evaluated_diagnostic_views',reasons=['producer_binding_not_recorded_by_sidecar_v1'])
    _same(source_frame,frame,'source_mutated')
    return json_copy(result)
