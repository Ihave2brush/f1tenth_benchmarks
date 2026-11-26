"""Offline anchored Frenet projection of saved modeled rollouts; no control."""
from copy import deepcopy
from dataclasses import asdict
import math

from ..core.geometry.tracker import FrenetTracker, TrackingOptions, wrap_progress
from .rollout_diagnostics_v1 import validate_record, snapshot_sha256


def _number(x):
    return isinstance(x,(int,float)) and not isinstance(x,bool) and math.isfinite(x)


def project_pose_series(samples, source_state, geometry, *, geometry_id, frame_id,
                        tracking_options, anchor_tolerance_m=1e-4):
    """Project recorded poses using a new private tracker and explicit s anchor.

    samples: [{t_s, pose:[x_m,y_m,yaw_rad]}]; times relative to source state.
    No interpolation or recovery after a rejected pose. Initial ambiguity is
    rejected conservatively, even when the caller supplies an s anchor.
    """
    if not isinstance(tracking_options,TrackingOptions):raise ValueError('explicit_tracking_options_required')
    tracking_options.validate()
    if not _number(anchor_tolerance_m) or anchor_tolerance_m<=0:raise ValueError('invalid_anchor_tolerance')
    if geometry_id!=geometry.geometry_id:raise ValueError('geometry_mismatch')
    if frame_id!='map':raise ValueError('frame_mismatch')
    source=deepcopy(source_state);poses=deepcopy(samples)
    fr=source.get('frenet') if isinstance(source,dict) else None
    if not isinstance(fr,dict) or fr.get('valid') is not True:raise ValueError('missing_valid_source_frenet')
    if fr.get('geometry_id')!=geometry_id or fr.get('frame_id')!=frame_id:raise ValueError('anchor_context_mismatch')
    if not all(_number(fr.get(k)) for k in ('s_wrapped','s_unwrapped')):raise ValueError('missing_source_s_anchor')
    if not _number(source.get('sim_time_s')):raise ValueError('missing_source_time')
    base=source['sim_time_s'];L=float(geometry.curve.L);closed=bool(geometry.curve.closed)
    if not _number(L) or L<=0:raise ValueError('invalid_geometry_length')
    a,b=fr['s_wrapped'],fr['s_unwrapped']
    # CoreLapTracker may expose a non-integer origin shift. Never infer it.
    shifted=('origin_shift' in fr or 'native_s_unwrapped' in fr)
    origin=fr.get('origin_shift',0.)
    native=fr.get('native_s_unwrapped',b)
    if shifted and (not _number(fr.get('origin_shift')) or not _number(fr.get('native_s_unwrapped'))):
        raise ValueError('incomplete_source_origin_contract')
    if abs(native+origin-b)>anchor_tolerance_m:raise ValueError('inconsistent_source_origin')
    if closed:
        if not 0<=a<L or abs(wrap_progress(native-a,L))>anchor_tolerance_m:raise ValueError('inconsistent_source_anchor')
    elif abs(a-native)>anchor_tolerance_m:raise ValueError('inconsistent_source_anchor')
    tracker=FrenetTracker(geometry,options=deepcopy(tracking_options),source_reference='saved_modeled_pose',offset_calibrated=False)
    pose=source.get('pose')
    if not isinstance(pose,list) or len(pose)!=3 or not all(_number(v) for v in pose):raise ValueError('invalid_source_pose')
    initial=tracker.update(pose[:2],pose[2],base,now=base,frame_id=frame_id,geometry_id=geometry_id)
    error=None;shift=None
    if not initial['valid']:error='source_projection:'+str(initial['reason'])
    else:
        delta=initial['s_wrapped']-a
        if closed:delta=wrap_progress(delta,L)
        if abs(delta)>anchor_tolerance_m:error='source_anchor_projection_mismatch'
        else:shift=b-initial['s_unwrapped']
    rows=[];previous=None;first_failure=error
    for index,item in enumerate(poses):
        t=item.get('t_s') if isinstance(item,dict) else None
        p=item.get('pose') if isinstance(item,dict) else None
        reason=first_failure
        if not _number(t) or t<0:reason='invalid_sample_time'
        elif previous is not None and t<=previous:reason='nonmonotonic_sample_time'
        elif index==0 and t!=0:reason='missing_t0'
        elif not isinstance(p,list) or len(p)!=3 or not all(_number(v) for v in p):reason='invalid_sample_pose'
        elif index==0 and any(abs(x-y)>1e-9 for x,y in zip(p,pose)):reason='sample_t0_source_pose_mismatch'
        result=None
        if reason is None:
            result=initial if index==0 else tracker.update(p[:2],p[2],base+t,now=base+t,frame_id=frame_id,geometry_id=geometry_id)
            if not result['valid']:reason=result['reason']
        if reason is not None and first_failure is None:first_failure=reason
        su=result['s_unwrapped']+shift if reason is None else None
        rows.append(dict(sample_index=index,t_s=t if _number(t) else None,projection_valid=reason is None,reason=reason,
            s_wrapped=result['s_wrapped'] if reason is None else None,s_unwrapped=su,
            d_m=result['d'] if reason is None else None,e_psi_rad=result['e_psi'] if reason is None else None,
            native_s_unwrapped=su-origin if reason is None else None,
            wrap_count=math.floor((su-origin)/L) if reason is None and closed else 0 if reason is None else None,
            selection_mode=result['selection_mode'] if reason is None else None))
        if _number(t):previous=t
    return dict(samples=rows,projection_valid=bool(rows) and all(r['projection_valid'] for r in rows),
        reason=first_failure or ('no_samples' if not rows else None),source_s_wrapped_m=a,source_s_unwrapped_m=b,
        source_origin_shift_m=origin,source_native_s_unwrapped_m=native,
        projection_method='independent_existing_FrenetTracker_plus_verified_source_unwrap_offset',
        inspection_only=True,planning_allowed=False)


def relative_progress_features(ego,lead,*,requested_horizon_s,native_complete):
    """Trapezoidal integral over a contiguous valid prefix only, unit m*s."""
    if not _number(requested_horizon_s) or requested_horizon_s<0:raise ValueError('invalid_requested_horizon')
    er,lr=ego['samples'],lead['samples']
    if len(er)!=len(lr) or [r['t_s'] for r in er]!=[r['t_s'] for r in lr]:raise ValueError('ego_lead_time_grid_mismatch')
    rows=[]
    for e,l in zip(er,lr):
        if not e['projection_valid'] or not l['projection_valid']:break
        if (not _number(e['t_s']) or e['t_s']<0 or e['t_s']>requested_horizon_s+1e-9
            or (not rows and e['t_s']!=0) or (rows and e['t_s']<=rows[-1]['t_s'])):
            raise ValueError('invalid_projected_sample_times')
        if not all(_number(v) for v in (e['s_unwrapped'],l['s_unwrapped'],ego['source_s_unwrapped_m'],lead['source_s_unwrapped_m'])):
            raise ValueError('invalid_projected_s')
        value=(e['s_unwrapped']-ego['source_s_unwrapped_m'])-(l['s_unwrapped']-lead['source_s_unwrapped_m'])
        rows.append(dict(t_s=e['t_s'],delta_s_relative_m=value))
    partial=-sum(.5*(a['delta_s_relative_m']+b['delta_s_relative_m'])*(b['t_s']-a['t_s']) for a,b in zip(rows,rows[1:]))
    complete=bool(rows) and len(rows)==len(er) and native_complete is True and abs(rows[0]['t_s'])<1e-12 and abs(rows[-1]['t_s']-requested_horizon_s)<1e-9
    return dict(samples=rows,evaluation_complete=complete,requested_horizon_s=requested_horizon_s,
        checked_range_s=[rows[0]['t_s'],rows[-1]['t_s']] if rows else None,
        delta_s_relative_final_m=rows[-1]['delta_s_relative_m'] if complete else None,
        progress_integral_m_s=partial if complete else None,
        checked_partial_delta_s_relative_m=rows[-1]['delta_s_relative_m'] if rows else None,
        checked_partial_progress_integral_m_s=partial if rows else None,
        reason=None if complete else 'incomplete_or_invalid_projection',inspection_only=True,planning_allowed=False)


def project_rollout_record(record,geometry,*,tracking_options,frame_id='map',anchor_tolerance_m=1e-4):
    """Read one saved V1 sidecar. Every native option survives in its original order."""
    saved=validate_record(deepcopy(record));meta=saved['metadata'];source=saved['source']
    if meta['geometry_id']!=geometry.geometry_id:raise ValueError('geometry_mismatch')
    if meta['frame_id']!=frame_id or frame_id!='map':raise ValueError('frame_mismatch')
    output=[]
    for option in saved['options']:
        series={}
        for role in ('ego','lead'):
            series[role]=project_pose_series([dict(t_s=s['t_s'],pose=s[role+'_pose']) for s in option['samples']],
                source[role+'_state'],geometry,geometry_id=meta['geometry_id'],frame_id=frame_id,
                tracking_options=tracking_options,anchor_tolerance_m=anchor_tolerance_m)
        progress=relative_progress_features(series['ego'],series['lead'],requested_horizon_s=option['requested_horizon_s'],native_complete=option['evaluation_complete'])
        output.append(dict(option_index=option['option_index'],evaluation_id=option['evaluation_id'],
            prediction_group_id=option['prediction_group_id'],candidate_id=option['candidate_id'],reference_id=option['reference_id'],
            target_speed_mps=option['target_speed_mps'],source_path_type=option['source_path_type'],
            native_reference_only=option['reference_only'],
            reference_only=bool(option['reference_only'] or option['source_path_type']=='reference_centerline' or option['reference_id']=='raceline'),
            reference_classification_source='native_path_type_or_explicit_reference_id',
            rollout_trajectory_type=option['rollout_trajectory_type'],
            native_safe=option['native_safe'],native_reason=option['native_reason'],native_relative_s_progress_m=option['native_relative_s_progress_m'],
            native_signed_min_pair_clearance_m=option['signed_min_pair_clearance_m'],native_evaluation_complete=option['evaluation_complete'],
            ego_projection=series['ego'],lead_projection=series['lead'],progress=progress,
            formal_comparison_qualified=False,inspection_only=True,planning_allowed=False))
    return dict(schema='offline_rollout_frenet_v1',schema_version=1,source_record_sha256=snapshot_sha256(saved),
        source_control_step=source['source_control_step'],source_sim_time_s=source['source_sim_time_s'],
        geometry_id=meta['geometry_id'],frame_id=frame_id,tracking_options=asdict(tracking_options),
        anchor_tolerance_m=anchor_tolerance_m,options=output,producer_binding_verified=False,physical_offset_calibration_verified=False,
        lead_trajectory_provenance='native_modeled_prediction_not_ground_truth',
        limitations=['No live tracker updates, interpolation, selection or control.',
                    'Initial ambiguous projection is rejected conservatively.',
                    'Native progress is an estimate; projected progress is a separate feature.',
                    'A valid projection does not verify collision safety or producer history.'],
        inspection_only=True,planning_allowed=False)
