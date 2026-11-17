"""Pure offline joins of existing audit/length artifacts; no model execution."""
import json
import math
from numbers import Real

from .plan_binding_v1 import _tree
from .plan_binding_v2 import HEADER, content_digest

METHOD = 'offline_cost_feature_v1'
CLEARANCE = 'signed_three_disc_pair_minimum_claim_not_single_circle'


def json_copy(value):
    _tree(value)
    return json.loads(json.dumps(value, allow_nan=False))


def number(value):
    return not isinstance(value,bool) and isinstance(value,Real) and math.isfinite(value)


def flags(value):
    return isinstance(value,dict) and value.get('inspection_only') is True and value.get('planning_allowed') is False


def digest(value):
    return isinstance(value,str) and len(value)==64 and all(c in '0123456789abcdef' for c in value)


def _indexed(rows,key,name):
    if not isinstance(rows,list): raise ValueError('invalid_'+name)
    result={}
    for row in rows:
        if not isinstance(row,dict) or type(row.get(key)) is not int or row[key]<0 or row[key] in result:
            raise ValueError('invalid_or_duplicate_'+name)
        result[row[key]]=row
    return result


def _binding(value):
    return isinstance(value,dict) and all(type(value.get(k)) is type(v) and value[k]==v for k,v in HEADER.items()) and digest(value.get('plan_sha256'))


def _risk_series(series,option):
    """Validate external samples, never upgrade the native audit coverage."""
    if series is None:return dict(status='missing',reason='missing_clearance_samples',samples=None,evaluation_complete=None)
    if not isinstance(series,dict):return dict(status='invalid',reason='invalid_risk_series',samples=None,evaluation_complete=None)
    try:
        for key in ('frame_index','option_index','candidate_id','target_speed_mps','audit_context_sha256',
                    'plan_v2_sha256','source_sim_time_s','geometry_id','frame_id','sample_dt_s','requested_horizon_s',
                    'body_length_m','body_width_m','body_reference','pair_threshold_m','clearance_convention','model'):
            if key not in series or series[key]!=option.get(key):raise ValueError('risk_source_mismatch:'+key)
        if series['model']!='three_discs' or series['clearance_convention']!=CLEARANCE:
            raise ValueError('unsupported_risk_model')
        provenance=series.get('provenance')
        if not isinstance(provenance,dict) or provenance.get('kind') not in ('synthetic','recorded_three_disc_rollout') or not isinstance(provenance.get('description'),str) or not provenance['description']:
            raise ValueError('missing_risk_provenance')
        samples=series.get('samples');dt=series['sample_dt_s'];horizon=series['requested_horizon_s']
        if not number(dt) or dt<=0 or not number(horizon) or horizon<=0 or horizon/dt>100000:
            raise ValueError('invalid_risk_time_grid')
        if not isinstance(samples,list) or not samples or len(samples)>100001:
            raise ValueError('missing_or_excess_risk_samples')
        for k,s in enumerate(samples):
            if not isinstance(s,dict) or not number(s.get('t_s')) or not number(s.get('signed_clearance_m')):
                raise ValueError('invalid_risk_sample')
            if not math.isclose(s['t_s'],k*dt,rel_tol=0,abs_tol=1e-9):raise ValueError('risk_grid_not_aligned')
        end=samples[-1]['t_s'];complete=series.get('evaluation_complete')
        if end>horizon+1e-9 or type(complete) is not bool or complete!=math.isclose(end,horizon,rel_tol=0,abs_tol=1e-9):
            raise ValueError('risk_coverage_mismatch')
        checked=series.get('checked_range_s')
        if not isinstance(checked,list) or len(checked)!=2 or checked[0]!=0 or checked[1]!=end:
            raise ValueError('risk_checked_range_mismatch')
        minimum=min(s['signed_clearance_m'] for s in samples);native=option.get('min_pair_clearance_m')
        if native is not None and (not number(native) or (complete and not math.isclose(minimum,native,rel_tol=0,abs_tol=1e-9)) or (not complete and minimum<native-1e-9)):
            raise ValueError('risk_minimum_summary_mismatch')
        return dict(status='reported_complete' if complete else 'reported_partial',reason=None,
            samples=samples,evaluation_complete=complete,checked_range_s=checked,provenance=provenance,
            sample_minimum_signed_m=minimum,current_envelope_contact=samples[0]['signed_clearance_m']<=0,
            first_envelope_contact_grid_t_s=next((s['t_s'] for s in samples if s['signed_clearance_m']<=0),None),
            series_content_sha256=content_digest('external_risk_series_v1',series))
    except (ValueError,KeyError,TypeError) as exc:
        return dict(status='invalid',reason=str(exc),samples=None,evaluation_complete=None)


def build_cost_features(audit_result, path_length_results, *, risk_sample_series=None):
    """Join options to one precomputed length result per frame/candidate.

    path_length_results: [{frame_index:int, result:PathLength output}, ...].
    risk_sample_series: optional records keyed by frame_index/option_index and
    explicit context, model, time grid and provenance. No geometry is sampled.
    """
    json_copy([audit_result,path_length_results,risk_sample_series])
    if not flags(audit_result) or audit_result.get('method')!='readonly_two_car_contract_audit_v1':
        raise ValueError('invalid_audit_artifact')
    if not digest(audit_result.get('audit_context_sha256')):raise ValueError('missing_audit_context')
    frames=_indexed(audit_result.get('frames'),'frame_index','audit_frames')
    lengths=_indexed(path_length_results,'frame_index','path_length_results')
    extras=[] if risk_sample_series is None else risk_sample_series
    if not isinstance(extras,list):raise ValueError('invalid_risk_series_list')
    risk={}
    for s in extras:
        if not isinstance(s,dict) or type(s.get('frame_index')) is not int or type(s.get('option_index')) is not int:
            raise ValueError('invalid_risk_option_key')
        key=(s['frame_index'],s['option_index'])
        if key in risk:raise ValueError('duplicate_risk_option_key')
        risk[key]=s
    options=audit_result.get('options')
    if not isinstance(options,list):raise ValueError('invalid_audit_options')
    keys=set();cache={};cache_errors={}
    for index,entry in lengths.items():
        try:
            frame=frames[index];value=entry.get('result')
            if not flags(value) or value.get('method')!='offline_path_length_feature_v1' or value.get('batch_status')!='evaluated':
                raise ValueError('invalid_path_length_artifact')
            parent=frame.get('plan_binding');bound=value.get('plan_binding')
            if not _binding(parent) or _tree(parent)!=_tree(bound):raise ValueError('audit_length_binding_mismatch')
            native=_indexed(frame.get('candidates'),'candidate_id','native_candidates')
            paths=_indexed(value.get('paths'),'candidate_id','length_candidates')
            if set(native)!={1,2,3,4,5} or set(paths)!=set(native):raise ValueError('invalid_five_candidate_ids')
            for cid,p in paths.items():
                n=native[cid]
                if not flags(p):raise ValueError('invalid_path_row_flags')
                if p.get('path_type')!=n.get('path_type') or p.get('native_sampled_valid') is not n.get('native_sampled_valid') or p.get('native_reason')!=n.get('native_reason') or p.get('native_footprint_sampled_valid') is not n.get('footprint_sampled_valid'):
                    raise ValueError('length_native_contract_mismatch')
                cache[(index,cid)]=dict(path=p,native=n,parent=parent,
                    geometry_object_curve_verified=value.get('geometry_object_curve_verified') is True,
                    artifact_evidence_claimed=bound.get('geometry_artifacts_verified') is True)
        except (ValueError,KeyError,TypeError) as exc:cache_errors[index]=str(exc)
    rows=[]
    for option in options:
        if not isinstance(option,dict):raise ValueError('invalid_audit_option')
        fi,oi=option.get('frame_index'),option.get('option_index')
        if type(fi) is not int or type(oi) is not int or fi<0 or oi<0 or (fi,oi) in keys:
            raise ValueError('invalid_or_duplicate_option_key')
        keys.add((fi,oi));cid=option.get('candidate_id');frame=frames.get(fi);shared=cache.get((fi,cid)) if type(cid) is int else None
        errors=[];exclusions=[]
        if audit_result.get('contract_valid') is not True or option.get('audit_status')=='invalid_contract':errors.append('audit_contract_invalid')
        if frame is None or frame.get('audit_status')=='invalid_contract':errors.append('invalid_source_frame')
        if not flags(option) or option.get('audit_context_sha256')!=audit_result['audit_context_sha256']:errors.append('option_context_mismatch')
        if fi in cache_errors:errors.append(cache_errors[fi])
        raceline=cid is None and option.get('reference')=='raceline'
        if not raceline and (type(cid) is not int or cid not in (1,2,3,4,5)):errors.append('invalid_candidate_id')
        if not raceline and shared is None:errors.append('missing_bound_path_length')
        p=shared['path'] if shared else {};n=shared['native'] if shared else {};parent=shared['parent'] if shared else None
        if parent:
            source=parent['source']
            for key in ('plan_id','source_sim_time_s','source_control_step','geometry_id','frame_id'):
                if option.get(key)!=source.get(key):errors.append('source_mismatch:'+key)
            if option.get('plan_v2_sha256')!=parent['plan_sha256']:errors.append('option_plan_digest_mismatch')
            if frame['ego_state'].get('pose')!=source['source_pose']:errors.append('source_pose_mismatch')
        reference=bool(raceline or n.get('reference_only') or p.get('reference_only') or n.get('path_type')=='reference_centerline')
        if reference:exclusions.append('reference_not_modeled_ego_trajectory')
        if n and n.get('native_sampled_valid') is not True:exclusions.append('native_rejected')
        if n and n.get('footprint_sampled_valid') is not True:exclusions.append('footprint_not_confirmed')
        if option.get('safe') is not True:exclusions.append('native_unsafe_or_unavailable')
        if option.get('safe') is True and (option.get('reason') is not None or option.get('first_rejection_s') is not None):errors.append('contradictory_native_safe')
        minimum=option.get('min_pair_clearance_m');threshold=option.get('pair_threshold_m')
        if minimum is not None and not number(minimum):errors.append('invalid_signed_clearance')
        if option.get('safe') is True and number(minimum) and number(threshold) and minimum<=threshold:errors.append('safe_conflicts_pair_threshold')
        if not number(option.get('target_speed_mps')) or option['target_speed_mps']<0:errors.append('invalid_target_speed')
        if frame and (frame.get('ego_state',{}).get('valid') is not True or frame.get('lead_state',{}).get('valid') is not True):exclusions.append('invalid_source_state')
        complete=option.get('evaluation_complete')
        checked=option.get('checked_range_s');horizon=option.get('requested_horizon_s');dt=option.get('sample_dt_s')
        time_valid=number(horizon) and horizon>0 and number(dt) and dt>0
        if time_valid and (horizon/dt>100000 or not math.isclose(horizon/dt,round(horizon/dt),rel_tol=0,abs_tol=1e-8)):time_valid=False
        if not time_valid:errors.append('invalid_requested_time_grid')
        full=bool(complete is True and option.get('coverage_status')=='reported_complete' and isinstance(checked,list) and len(checked)==2 and checked[0]==0 and checked[1]==horizon and time_valid)
        if not full:exclusions.append('checked_horizon_unknown_or_incomplete')
        if complete is True and not full:errors.append('inconsistent_complete_coverage')
        geometry_length=p.get('frenet_integral_length_m') if p.get('length_status')=='computed' else None
        if geometry_length is not None and (not number(geometry_length) or geometry_length<=0):errors.append('invalid_geometry_length')
        if p and p.get('length_status')!='computed':exclusions.append('geometry_length_unavailable')
        if option.get('body_reference')!='geometric_body_centre':exclusions.append('body_reference_unconfirmed')
        if option.get('model')!='three_discs' or option.get('clearance_convention')!=CLEARANCE:exclusions.append('model_unconfirmed')
        for key in ('body_length_m','body_width_m'):
            if not number(option.get(key)) or option[key]<=0:exclusions.append('missing_body_dimensions')
        for key in ('pair_threshold_m','wall_threshold_m'):
            if not number(option.get(key)) or option[key]<0:exclusions.append('missing_margin_threshold')
        risk_data=_risk_series(risk.get((fi,oi)),option)
        if risk_data['status']=='invalid':errors.append(risk_data['reason'])
        progress=option.get('relative_s_progress_m')
        if progress is not None and not number(progress):errors.append('invalid_progress')
        missing=dict(geometry_length=geometry_length is None,min_pair_clearance=minimum is None,
            relative_progress=progress is None,complete_checked_horizon=not full,
            clearance_samples=risk_data['samples'] is None,complete_risk_samples=risk_data.get('evaluation_complete') is not True,
            modeled_ego_rollout_length=True)
        if errors:status='rejected'
        elif reference:status='reference_only'
        elif 'native_rejected' in exclusions or 'footprint_not_confirmed' in exclusions:status='rejected'
        elif option.get('safe') is False:status='unsafe'
        elif exclusions:status='unknown'
        elif minimum is None or geometry_length is None or progress is None:status='partial_features'
        else:status='features_complete_policy_required'
        rows.append(dict(frame_index=fi,option_index=oi,candidate_id=cid,reference=option.get('reference'),
            target_speed_mps=option.get('target_speed_mps'),path_type=n.get('path_type'),reference_only=reference,
            native_sampled_valid=n.get('native_sampled_valid'),footprint_sampled_valid=n.get('footprint_sampled_valid'),
            native_reason=n.get('native_reason'),native_safe=option.get('safe'),native_option_reason=option.get('reason'),
            first_rejection_s=option.get('first_rejection_s'),comparison_status=status,
            reasons=errors+list(dict.fromkeys(exclusions)),binding_status='matching_content_records_not_producer_verified' if shared and not cache_errors.get(fi) else 'unavailable_or_mismatched',
            historical_producer_binding_verified=False,plan_v2_sha256=option.get('plan_v2_sha256'),
            audit_context_sha256=option.get('audit_context_sha256'),plan_id=option.get('plan_id'),
            source_control_step=option.get('source_control_step'),source_pose=parent['source']['source_pose'] if parent else None,
            source_sim_time_s=option.get('source_sim_time_s'),
            geometry_id=option.get('geometry_id'),frame_id=option.get('frame_id'),
            geometry_length_m=geometry_length,geometry_length_scope=p.get('geometry_scope'),ego_rollout_length_m=None,
            geometry_start_xy=p.get('start_xy'),geometry_end_xy=p.get('end_xy'),reference_interval_s_m=p.get('reference_interval_s_m'),
            geometry_object_curve_verified=shared['geometry_object_curve_verified'] if shared else False,
            geometry_artifact_evidence_claimed=shared['artifact_evidence_claimed'] if shared else False,
            min_pair_clearance_m=minimum,relative_s_progress_m=progress,
            progress_semantics='native_estimated_path_advance_minus_lead_s_rate_times_last_rollout_time_not_s_time_integral',
            requested_horizon_s=horizon,sample_dt_s=dt,checked_range_s=checked,evaluation_complete=complete,
            native_full_horizon_reported=full,risk_series=risk_data,missing_feature_mask=missing,
            model=option.get('model'),clearance_convention=option.get('clearance_convention'),
            body_reference=option.get('body_reference'),body_length_m=option.get('body_length_m'),body_width_m=option.get('body_width_m'),
            pair_threshold_m=threshold,wall_threshold_m=option.get('wall_threshold_m'),
            inspection_only=True,planning_allowed=False))
    if any(key not in keys for key in risk):raise ValueError('risk_series_for_unknown_option')
    return json_copy(dict(schema_version=1,method=METHOD,source_classification=audit_result.get('source_classification'),
        audit_context_sha256=audit_result['audit_context_sha256'],options=rows,
        input_content_sha256=dict(audit=content_digest('cost_feature_audit_input',audit_result),
            length=content_digest('cost_feature_length_inputs',path_length_results),risk=content_digest('cost_feature_risk_inputs',risk_sample_series)),
        geometry_recomputation_count=0,unique_geometry_records_reused=len(cache),
        inspection_only=True,planning_allowed=False,
        limitations=['No sampling/model/control execution; artifacts are caller assertions.',
            'Audit-time binding is not authenticated producer history.',
            'Reference geometry is not modeled ego travel; minimum is not exposure.']))
