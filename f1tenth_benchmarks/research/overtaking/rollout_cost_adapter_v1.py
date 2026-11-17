"""Offline sidecar-to-cost join. Reuses validators; no rollout or cost formula."""
import hashlib
import json
import math
from pathlib import Path

from .cost_feature_v1 import build_cost_features, json_copy, flags, number, CLEARANCE
from .plan_binding_v1 import _tree
from .plan_binding_v2 import verify_plan_binding, geometry_evidence, content_digest
from .path_length_feature_v1 import _verify_v2_in_memory
from .rollout_diagnostics_v1 import validate_record, snapshot_sha256


def _same(a, b, reason):
    if _tree(a) != _tree(b):
        raise ValueError(reason)


def read_rollout_sidecar(path):
    """Read only; every line must pass the existing diagnostics validator."""
    raw=Path(path).read_bytes()
    records=[validate_record(json.loads(line)) for line in raw.decode('utf-8').splitlines() if line.strip()]
    if not records: raise ValueError('empty_sidecar')
    return dict(records=records, file_sha256=hashlib.sha256(raw).hexdigest())


def _configured(meta, name):
    current='configured_'+name+'_clearance_m'; legacy='physical_'+name+'_clearance_m'
    if current in meta:
        if meta.get(legacy) is not None and meta[legacy]!=meta[current]:
            raise ValueError('conflicting_margin_fields:'+name)
        value=meta[current]; origin=current
    else:
        # The V1 writer mislabeled configured settings. Interpret only this
        # documented schema; never export these values as measured clearance.
        value=meta.get(legacy); origin='legacy_v1_setting:'+legacy
    if not number(value) or value<0: raise ValueError('missing_configured_margin:'+name)
    return value,origin


def adapt_rollout_cost_inputs(sidecar, source_frame, audit_result, path_length_results, *,
                              plan_binding=None, geometry_manifest_path=None,
                              expected_manifest_sha256=None, existing_cost_features=None):
    """Return existing Cost Feature output plus trace identity/qualification.

    sidecar is one parsed record or a JSONL path. For a file, source clock must
    select exactly one frame. Geometry pins are caller trust anchors. V1 has no
    authenticated producer binding: complete samples never authorize total cost.
    """
    frame,audit,lengths=json_copy([source_frame,audit_result,path_length_results])
    if not flags(audit) or audit.get('contract_valid') is not True:
        raise ValueError('invalid_audit_contract')
    if not isinstance(audit.get('frames'),list) or not isinstance(audit.get('options'),list) or not isinstance(audit.get('source_metadata'),dict):
        raise ValueError('invalid_audit_schema')
    if not isinstance(frame,dict): raise ValueError('invalid_source_frame')
    if len(audit['frames'])!=1:raise ValueError('single_source_audit_required')
    _same(audit.get('input_content_fingerprints',{}).get('frames'),
          content_digest('saved_frames',[frame]),'source_frame_fingerprint_mismatch')
    ego=frame.get('ego_state');lead=frame.get('lead_state')
    if not isinstance(ego,dict) or not isinstance(lead,dict):raise ValueError('missing_source_states')
    clock=(ego.get('control_step'),ego.get('sim_time_s'));file_sha=None
    if isinstance(sidecar,(str,Path)):
        loaded=read_rollout_sidecar(sidecar);file_sha=loaded['file_sha256']
        matches=[r for r in loaded['records'] if (r['source']['source_control_step'],r['source']['source_sim_time_s'])==clock]
        if len(matches)!=1:raise ValueError('missing_or_ambiguous_sidecar_source')
        record=matches[0]
    else:record=validate_record(json_copy(sidecar))
    source=record['source'];meta=record['metadata']
    _same(source['ego_state'],ego,'ego_source_mismatch')
    _same(source['lead_state'],lead,'lead_source_mismatch')
    _same(frame.get('time_s'),ego.get('sim_time_s'),'frame_time_mismatch')
    if clock!=(source['source_control_step'],source['source_sim_time_s']):raise ValueError('source_clock_mismatch')
    candidates=[f for f in audit['frames'] if (f.get('source_control_step'),f.get('source_sim_time_s'))==clock]
    if len(candidates)!=1:raise ValueError('missing_or_ambiguous_audit_source')
    af=candidates[0];fi=af['frame_index']
    _same(af.get('ego_state'),ego,'audit_ego_mismatch');_same(af.get('lead_state'),lead,'audit_lead_mismatch')
    _same(af.get('decision_diagnostics'),frame.get('decision',{}),'audit_decision_mismatch')
    for k in ('geometry_id','frame_id'):
        _same(meta.get(k),audit['source_metadata'].get(k),'metadata_mismatch:'+k)
    plan=frame.get('plan')
    _same(record.get('plan_snapshot'),plan,'plan_snapshot_mismatch')
    bound=af.get('plan_binding');verification='no_plan'
    if plan is None:
        if plan_binding is not None or bound is not None:raise ValueError('fabricated_plan_binding')
    else:
        if not isinstance(bound,dict):raise ValueError('missing_audit_binding')
        if plan_binding is not None:_same(plan_binding,bound,'supplied_binding_mismatch')
        bound,verification=_verify_v2_in_memory(plan,audit['source_metadata'],bound)
    evidence=geometry_evidence(geometry_manifest_path,expected_manifest_sha256)
    if geometry_manifest_path is not None:
        for k in ('geometry_id','frame_id'):
            if evidence.get(k) is not None:_same(evidence[k],meta[k],'geometry_evidence_mismatch:'+k)
        if bound is not None:
            verify_plan_binding(plan,bound,session_metadata=audit['source_metadata'],
                length_m=bound['lookahead_length_m'],geometry_manifest_path=geometry_manifest_path,
                expected_manifest_sha256=expected_manifest_sha256)
    costs=frame.get('costs')
    native=[] if costs is None or costs=={} else costs.get('options') if isinstance(costs,dict) else None
    if not isinstance(native,list):raise ValueError('missing_native_options')
    opts=[o for o in audit['options'] if o['frame_index']==fi]
    if len(opts)!=len(audit['options']):raise ValueError('single_source_audit_required')
    if len(opts)!=len(native) or len(opts)!=len(record['options']):raise ValueError('option_count_mismatch')
    if [o['option_index'] for o in opts]!=list(range(len(opts))):raise ValueError('audit_option_order_mismatch')
    pair,pair_origin=_configured(meta,'pair');wall,wall_origin=_configured(meta,'wall')
    ego_frenet=ego.get('frenet') if isinstance(ego.get('frenet'),dict) else None
    lead_frenet=lead.get('frenet') if isinstance(lead.get('frenet'),dict) else None
    if existing_cost_features is not None:
        baseline=build_cost_features(audit,lengths)
        old=json_copy(existing_cost_features)
        if not flags(old):raise ValueError('invalid_existing_cost_features')
        for k in ('audit_context_sha256','input_content_sha256'):
            if k=='input_content_sha256':
                if not isinstance(old.get(k),dict):raise ValueError('invalid_existing_cost_features')
                for field in ('audit','length'):_same(old[k].get(field),baseline[k][field],'stale_existing_cost_features')
            else:_same(old.get(k),baseline[k],'stale_existing_cost_features')
    series=[];rows=[];record_hash=snapshot_sha256(record)
    for i,(o,tr,raw) in enumerate(zip(opts,record['options'],native)):
        _same(o['original_diagnostics'],raw,'native_audit_option_mismatch')
        for k in ('candidate_id','target_speed_mps','option_index'):_same(tr.get(k),o.get(k),'option_identity_mismatch:'+k)
        if tr['candidate_id'] is None:
            if tr.get('reference_id')!='raceline' or o.get('reference')!='raceline':raise ValueError('reference_identity_mismatch')
            if tr.get('candidate_snapshot') is not None:raise ValueError('fabricated_reference_candidate')
            if frame.get('raceline_xy') is not None:_same(tr.get('reference_xy'),frame['raceline_xy'],'raceline_xy_mismatch')
        else:
            paths=[p for p in plan['paths'] if p['candidate_id']==tr['candidate_id']] if plan else []
            if len(paths)!=1:raise ValueError('missing_source_candidate')
            _same(tr.get('candidate_snapshot'),paths[0],'candidate_snapshot_mismatch')
            _same(tr.get('source_path_type'),paths[0].get('path_type','state_matched_trajectory'),'path_type_mismatch')
        for tk,ak in [('native_safe','safe'),('native_reason','reason'),('first_rejection_s','first_rejection_s'),
                      ('signed_min_pair_clearance_m','min_pair_clearance_m'),('native_relative_s_progress_m','relative_s_progress_m')]:
            _same(tr.get(tk),o.get(ak),'native_diagnostic_mismatch:'+ak)
        for k in ('model','body_reference','body_length_m','body_width_m','pair_threshold_m','wall_threshold_m','clearance_convention'):
            _same(meta.get(k),o.get(k),'model_parameter_mismatch:'+k)
        _same(pair,o.get('configured_pair_clearance_m'),'configured_pair_margin_mismatch')
        _same(wall,o.get('configured_wall_clearance_m'),'configured_wall_margin_mismatch')
        radius=math.hypot(meta['body_length_m']/6,meta['body_width_m']/2)
        if not number(meta.get('body_radius_m')) or not math.isclose(radius,meta['body_radius_m'],rel_tol=0,abs_tol=1e-12):
            raise ValueError('body_radius_mismatch')
        group=next(g for g in record['lead_predictions'] if g['prediction_group_id']==tr['prediction_group_id'])
        if group.get('model')!='fixed_frenet_d_constant_s_rate_yaw_decay_0.3s':
            raise ValueError('unsupported_lead_prediction_model')
        native_lead=frame.get('decision',{}).get('lead_prediction_poses')
        if native_lead is not None:_same(group['poses'],native_lead,'native_lead_prediction_mismatch')
        if type(meta.get('sample_steps')) is not int or meta['sample_steps']<=0 or not number(meta.get('physics_dt_s')) or not math.isclose(meta['sample_steps']*meta['physics_dt_s'],meta['sample_dt_s'],rel_tol=0,abs_tol=1e-12):
            raise ValueError('physics_sample_grid_mismatch')
        _same(tr['sample_dt_s'],o['sample_dt_s'],'native_sample_dt_mismatch')
        _same(tr['requested_horizon_s'],o['requested_horizon_s'],'native_group_horizon_mismatch')
        if o.get('checked_range_s') is not None:
            _same(o['checked_range_s'],[tr['checked_start_s'],tr['checked_end_s']],'native_checked_range_mismatch')
            _same(o['evaluation_complete'],tr['evaluation_complete'],'native_complete_mismatch')
        sample_rows=[dict(t_s=s['t_s'],signed_clearance_m=s['signed_min_pair_clearance_m']) for s in tr['samples']]
        if sample_rows:
            keys=('frame_index','option_index','candidate_id','target_speed_mps','audit_context_sha256','plan_v2_sha256',
                  'source_sim_time_s','geometry_id','frame_id','sample_dt_s','requested_horizon_s','body_length_m',
                  'body_width_m','body_reference','pair_threshold_m','clearance_convention','model')
            risk={k:o[k] for k in keys}
            risk.update(samples=sample_rows,checked_range_s=[tr['checked_start_s'],tr['checked_end_s']],
                evaluation_complete=tr['evaluation_complete'],provenance=dict(
                    kind='synthetic' if audit['source_classification']=='synthetic' else 'recorded_three_disc_rollout',
                    description='Saved V1 modeled rollout; snapshot/audit join, no authenticated producer binding',
                    evaluation_id=tr['evaluation_id'],prediction_group_id=tr['prediction_group_id'],
                    sidecar_file_sha256=file_sha,source_record_sha256=record_hash))
            series.append(risk)
        reasons=['producer_binding_not_recorded_by_sidecar_v1']
        if not evidence['geometry_artifacts_verified']:reasons.append('geometry_artifacts_not_reverified_against_pin')
        rows.append(dict(frame_index=fi,option_index=i,evaluation_id=tr['evaluation_id'],prediction_group_id=tr['prediction_group_id'],
            candidate_id=tr['candidate_id'],reference_id=tr.get('reference_id'),target_speed_mps=tr['target_speed_mps'],
            source_control_step=clock[0],source_sim_time_s=clock[1],geometry_id=meta['geometry_id'],frame_id=meta['frame_id'],
            plan_v2_sha256=bound['plan_sha256'] if bound else None,
            candidate_v2_sha256=next((c['sha256'] for c in bound['candidate_sha256'] if c['candidate_id']==tr['candidate_id']),None) if bound else None,
            identity_binding_scope='sidecar_internal_identity_plus_snapshot_and_audit_option_join',
            lead_prediction_match='native_decision_poses_matched' if native_lead is not None else 'unverified_native_prediction_missing',
            source_path_type=tr['source_path_type'],reference_only=tr['reference_only'],
            rollout_trajectory_type=tr.get('rollout_trajectory_type'),sample_grid_complete=tr['evaluation_complete'],
            risk_samples_available=bool(sample_rows),formal_comparison_qualified=False,qualification_reasons=reasons,
            configured_pair_clearance_m=pair,configured_wall_clearance_m=wall,physical_pair_clearance_m=None,physical_wall_clearance_m=None,
            configured_margin_field_origins=dict(pair=pair_origin,wall=wall_origin),native_safe=o['safe'],native_reason=o.get('reason'),
            min_pair_clearance_m=o.get('min_pair_clearance_m'),first_rejection_s=o.get('first_rejection_s'),
            requested_horizon_s=tr['requested_horizon_s'],checked_range_s=[tr['checked_start_s'],tr['checked_end_s']] if sample_rows else None,
            termination_reason=tr['termination_reason'],signed_clearance_samples=sample_rows,
            projection_inputs=dict(ego_pose_samples=[s['ego_pose'] for s in tr['samples']],lead_pose_samples=[s['lead_pose'] for s in tr['samples']],
                sample_times_s=[s['t_s'] for s in tr['samples']],source_ego_frenet=ego_frenet,source_lead_frenet=lead_frenet,
                projection_origin=dict(source_sim_time_s=clock[1],ego_s_unwrapped=ego_frenet.get('s_unwrapped') if ego_frenet else None,
                                       lead_s_unwrapped=lead_frenet.get('s_unwrapped') if lead_frenet else None),
                geometry_id=meta['geometry_id'],frame_id=meta['frame_id'],prediction_group_id=tr['prediction_group_id'],projection_performed=False),
            inspection_only=True,planning_allowed=False))
    features=build_cost_features(audit,lengths,risk_sample_series=series)
    # Preserve stronger native gates; completeness alone never provides source trust.
    for row in features['options']:
        row['upstream_comparison_status']=row['comparison_status']
        if row['comparison_status'] not in ('rejected','unsafe','reference_only'):row['comparison_status']='unknown'
        row['reasons'].append('producer_binding_not_recorded_by_sidecar_v1')
        row['formal_comparison_qualified']=False
    return json_copy(dict(schema_version=1,method='offline_rollout_cost_adapter_v1',options=rows,cost_features=features,
        risk_sample_series=series,sidecar_file_sha256=file_sha,record_sha256=record_hash,
        binding_content_verification=verification,producer_binding_verified=False,geometry_evidence=evidence,
        formal_comparison_qualified=False,inspection_only=True,planning_allowed=False,
        limitations=['V1 sidecar identities are internally checked, not authenticated producer evidence.',
                     'Native Audit coverage is not upgraded from sidecar sample completeness.',
                     'Different native/sidecar prediction grids require a future group-aware Audit, not silent rewriting.',
                     'No cost formula, rollout, projection, selection or control is executed.']))
