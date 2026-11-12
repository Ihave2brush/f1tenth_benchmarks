"""Read saved two-car diagnostics; never run a generator, selector or controller."""
import argparse
import csv
import hashlib
import json
import math
from numbers import Real
from pathlib import Path

from f1tenth_benchmarks.research.overtaking.plan_binding_v2 import build_plan_binding, content_digest, geometry_evidence
from f1tenth_benchmarks.research.overtaking.plan_binding_v1 import _tree

PRODUCER = '707d869073b8b1afe3d5a1d671e872581c01d4bb'
OPTION_FIELDS = ('candidate_id','reference','target_speed_mps','safe','reason','static_reason',
                 'first_rejection_s','min_pair_clearance_m','relative_s_progress_m')


def _copy(value):
    _tree(value)
    return json.loads(json.dumps(value, allow_nan=False))


def _number(value):
    return not isinstance(value,bool) and isinstance(value,Real) and math.isfinite(value)


def _step(value):
    return type(value) is int and value>=0


def _pose(value):
    return isinstance(value,list) and len(value)==3 and all(_number(v) for v in value)


def _state_errors(frame, metadata):
    errors=[]; ego=frame.get('ego_state'); lead=frame.get('lead_state')
    if not isinstance(ego,dict) or not isinstance(lead,dict): return ['missing_two_car_states']
    for name,state in [('ego',ego),('lead',lead)]:
        if not _step(state.get('control_step')): errors.append(name+'_invalid_control_step')
        if not _number(state.get('sim_time_s')) or state['sim_time_s']<0: errors.append(name+'_invalid_time')
        if not _pose(state.get('pose')): errors.append(name+'_invalid_pose')
        if not _number(state.get('speed_mps')) or state['speed_mps']<0: errors.append(name+'_invalid_speed')
        if type(state.get('valid')) is not bool: errors.append(name+'_missing_native_validity')
        for k in ('geometry_id','frame_id'):
            if k in state and state[k]!=metadata[k]: errors.append(name+'_'+k+'_mismatch')
    if ego.get('control_step')!=lead.get('control_step'): errors.append('two_car_control_step_mismatch')
    if ego.get('sim_time_s')!=lead.get('sim_time_s'): errors.append('two_car_time_mismatch')
    if not _number(frame.get('time_s')) or frame['time_s']!=ego.get('sim_time_s'): errors.append('frame_time_mismatch')
    return errors


def audit_two_car_frames(frames, report, *, session_metadata, source_classification,
                         geometry_manifest_path=None, expected_manifest_sha256=None,
                         parameter_metadata=None):
    """Audit recorded source joins and options, not physical safety or ranking.

    Metadata/classification are explicit external assertions. Only recorded
    checked_range_s + evaluation_complete establish reported coverage; current
    upstream files omit them, so even safe=True retains unknown completeness.
    """
    if not isinstance(frames,list) or not frames or not isinstance(report,dict):
        raise ValueError('invalid_frames_or_report')
    if not isinstance(session_metadata,dict) or any(not isinstance(session_metadata.get(k),str) or not session_metadata[k] for k in ('geometry_id','frame_id')):
        raise ValueError('missing_explicit_geometry_or_frame')
    if source_classification not in ('synthetic','saved_two_car_simulation'):
        raise ValueError('explicit_source_classification_required')
    _copy([frames,report,session_metadata,parameter_metadata])
    config=report.get('config',{})
    if not isinstance(config,dict): raise ValueError('invalid_report_config')
    global_errors=[]
    for k in ('geometry_id','frame_id'):
        if k in report and report[k]!=session_metadata[k]: global_errors.append('report_'+k+'_mismatch')
    for k in ('lookahead_m','footprint_length_m','footprint_width_m','control_dt_s'):
        if k in config and (not _number(config[k]) or config[k]<=0): global_errors.append('invalid_config_'+k)
    for obj,key in [(config,'footprint_clearance_m'),(report,'pair_clearance_m')]:
        if key in obj and (not _number(obj[key]) or obj[key]<0): global_errors.append('invalid_margin:'+key)
    try:
        evidence=geometry_evidence(geometry_manifest_path,expected_manifest_sha256)
        for key in ('geometry_id','frame_id'):
            if evidence.get(key) is not None and evidence[key]!=session_metadata[key]:
                global_errors.append('geometry_manifest_source_mismatch:'+key)
        if evidence.get('map_id') is not None and report.get('map') is not None and evidence['map_id']!=report['map']:
            global_errors.append('report_geometry_map_mismatch')
    except (ValueError,TypeError,KeyError,OSError) as exc:
        evidence=dict(status='invalid_contract',geometry_artifacts_verified=False,map_sources_verified=False)
        global_errors.append(str(exc))
    if parameter_metadata is not None:
        if not isinstance(parameter_metadata,dict): raise ValueError('invalid_parameter_metadata')
        for config_key,param_key in [('footprint_length_m','vehicle_length'),('footprint_width_m','vehicle_width')]:
            if param_key in parameter_metadata and (not _number(parameter_metadata[param_key]) or parameter_metadata[param_key]<=0 or
                (config_key in config and parameter_metadata[param_key]!=config[config_key])):
                global_errors.append('body_parameter_mismatch:'+param_key)
    result=dict(schema_version=1,method='readonly_two_car_contract_audit_v1',
        source_classification=source_classification,classification_provenance='caller_assertion',
        map_claim=report.get('map'),source_metadata=session_metadata,
        geometry_evidence=evidence,parameter_metadata=parameter_metadata,
        input_content_fingerprints=dict(frames=content_digest('saved_frames',frames),report=content_digest('saved_report',report),
            metadata=content_digest('external_metadata',session_metadata),parameters=content_digest('external_parameters',parameter_metadata)),
        source_file_sha256=None,frames=[],options=[],contract_errors=global_errors,
        inspection_only=True,planning_allowed=False,
        limitations=['Read-only diagnostics, no reconstruction or safety approval.',
            'Requested horizon and safe flag do not prove checked coverage.',
            'Binding made during audit is not historical producer binding.',
            'Signed three-disc clearance differs from clipped single-circle clearance.'])
    result['audit_context_sha256']=content_digest('two_car_audit_context',dict(
        fingerprints=result['input_content_fingerprints'],geometry_evidence=evidence,
        source_classification=source_classification))
    previous=None
    for index, frame in enumerate(frames):
        if not isinstance(frame,dict):
            result['frames'].append(dict(frame_index=index,audit_status='invalid_contract',errors=['invalid_frame'],
                inspection_only=True,planning_allowed=False));continue
        errors=list(global_errors)+_state_errors(frame,session_metadata)
        ego=frame.get('ego_state') if isinstance(frame.get('ego_state'),dict) else {}
        lead=frame.get('lead_state') if isinstance(frame.get('lead_state'),dict) else {}
        if previous is not None and _number(ego.get('sim_time_s')) and _step(ego.get('control_step')):
            if ego['sim_time_s']<=previous[0] or ego['control_step']<=previous[1]: errors.append('nonincreasing_frame_sequence')
        if _number(ego.get('sim_time_s')) and _step(ego.get('control_step')): previous=(ego['sim_time_s'],ego['control_step'])
        item=dict(frame_index=index,source_control_step=ego.get('control_step'),source_sim_time_s=ego.get('sim_time_s'),
            ego_state=ego,lead_state=lead,selected_candidate_id=frame.get('selected_candidate_id'),
            selection_reason=frame.get('selection_reason'),terminal_reason=frame.get('terminal_reason'),
            recorded_actions=frame.get('actions'),actions_scope='saved_only_not_executed',
            plan_binding=None,binding_status='no_plan',candidates=[],errors=errors,
            inspection_only=True,planning_allowed=False)
        plan=frame.get('plan'); candidates={}
        if plan is not None:
            if not isinstance(plan,dict): errors.append('invalid_plan')
            else:
                for p,s in [('source_control_step','control_step'),('source_sim_time_s','sim_time_s'),('source_pose','pose')]:
                    if plan.get(p)!=ego.get(s): errors.append('plan_state_join_mismatch:'+p)
                if not _step(plan.get('source_control_step')): errors.append('invalid_plan_control_step')
                if not _number(plan.get('source_sim_time_s')): errors.append('invalid_plan_source_time')
                paths=plan.get('paths')
                if isinstance(paths,list):
                    for p in paths:
                        if not isinstance(p,dict): errors.append('invalid_candidate');continue
                        row=dict(candidate_id=p.get('candidate_id'),native_sampled_valid=p.get('sampled_valid'),native_reason=p.get('reason'),
                            footprint_sampled_valid=p.get('footprint_sampled_valid'),footprint_check=p.get('footprint_check'),
                            path_type=p.get('path_type','state_matched_trajectory'),
                            path_type_origin='explicit' if 'path_type' in p else 'legacy_default',
                            reference_only=p.get('path_type')=='reference_centerline',reference_contract_revalidated=False)
                        item['candidates'].append(row)
                        for key in ('geometry_id','frame_id'):
                            if key in p and p[key]!=session_metadata[key]: errors.append('candidate_metadata_mismatch:'+key)
                        if type(row['native_sampled_valid']) is not bool: errors.append('invalid_native_sampled_valid')
                        if row['footprint_sampled_valid'] is not None and type(row['footprint_sampled_valid']) is not bool: errors.append('invalid_footprint_flag')
                        if row['reference_only'] and row['candidate_id']!=3: errors.append('reference_path_not_c3')
                        if row['path_type'] not in ('state_matched_trajectory','reference_centerline'): errors.append('unsupported_path_type')
                        cid=row['candidate_id']
                        if type(cid) is int: candidates[cid]=row
                try:
                    item['plan_binding']=build_plan_binding(plan,session_metadata=session_metadata,
                        length_m=config.get('lookahead_m'),geometry_manifest_path=geometry_manifest_path,
                        expected_manifest_sha256=expected_manifest_sha256)
                    evidence=item['plan_binding']['geometry_evidence']
                    if evidence.get('map_id') is not None and report.get('map') is not None and evidence['map_id']!=report['map']:
                        raise ValueError('report_geometry_map_mismatch')
                    item['binding_status']='computed_at_audit_not_producer_verified'
                except (ValueError,TypeError,KeyError,OSError) as exc:
                    item['binding_status']='invalid_contract';errors.append(str(exc))
        decision=frame.get('decision',{})
        if not isinstance(decision,dict): errors.append('invalid_decision');decision={}
        requested=decision.get('horizon_s'); dt=decision.get('sample_dt_s')
        if requested is not None and (not _number(requested) or requested<0): errors.append('invalid_requested_horizon')
        if dt is not None and (not _number(dt) or dt<=0): errors.append('invalid_sample_dt')
        if _number(requested) and _number(dt) and dt>0 and not math.isclose(requested/dt,round(requested/dt),abs_tol=1e-8): errors.append('requested_grid_not_aligned')
        for key in ('pair_threshold_m','wall_threshold_m'):
            if decision.get(key) is not None and (not _number(decision[key]) or decision[key]<0):
                errors.append('invalid_decision_threshold:'+key)
        poses=decision.get('lead_prediction_poses')
        if poses is not None:
            if not isinstance(poses,list) or not poses or any(not _pose(p) for p in poses): errors.append('invalid_lead_prediction_poses')
            elif _number(requested) and _number(dt) and dt>0 and len(poses)!=round(requested/dt)+1: errors.append('lead_prediction_grid_count_mismatch')
            elif poses[0]!=lead.get('pose'): errors.append('lead_prediction_origin_pose_mismatch')
        item['decision_diagnostics']=decision
        item['opponent_prediction_embedded_timestamp_verified']=False
        costs=frame.get('costs')
        options=costs.get('options') if isinstance(costs,dict) else None
        item['options_recorded']=isinstance(options,list)
        item['recorded_option_count']=len(options) if isinstance(options,list) else None
        if options is None:
            if plan is not None and costs not in (None,{}): errors.append('missing_options')
            options=[]
        if not isinstance(options,list): errors.append('invalid_options');options=[]
        for option_index, original in enumerate(options):
            option_errors=[]
            raw=original if isinstance(original,dict) else {}
            if not isinstance(original,dict): option_errors.append('invalid_option')
            cid=raw.get('candidate_id'); ref=raw.get('reference')
            candidate=candidates.get(cid) if type(cid) is int else None
            raceline=cid is None and ref=='raceline'
            if not raceline and (type(cid) is not int or cid not in candidates): option_errors.append('unknown_candidate_id')
            speed=raw.get('target_speed_mps')
            if not _number(speed) or speed<0: option_errors.append('invalid_target_speed')
            if type(raw.get('safe')) is not bool: option_errors.append('missing_native_safe')
            for k in ('first_rejection_s','min_pair_clearance_m','relative_s_progress_m'):
                if raw.get(k) is not None and not _number(raw[k]): option_errors.append('invalid_'+k)
            first=raw.get('first_rejection_s')
            if _number(first) and (first<0 or (_number(requested) and first>requested)): option_errors.append('first_rejection_outside_request')
            checked=raw.get('checked_range_s'); complete=raw.get('evaluation_complete')
            coverage='unknown'; full=False
            if checked is not None:
                if not isinstance(checked,list) or len(checked)!=2 or not all(_number(v) for v in checked) or checked[0]!=0 or checked[1]<0 or not _number(requested) or checked[1]>requested:
                    option_errors.append('invalid_checked_range')
                elif type(complete) is not bool:
                    option_errors.append('missing_recorded_coverage_complete_flag')
                elif complete and checked[1]!=requested:
                    option_errors.append('complete_range_mismatch')
                elif not complete and checked[1]>=requested:
                    option_errors.append('incomplete_range_mismatch')
                else:
                    full=complete;coverage='reported_complete' if complete else 'reported_partial'
            elif complete is not None: option_errors.append('complete_flag_without_checked_range')
            if isinstance(checked,list) and len(checked)==2 and _number(checked[1]) and _number(dt) and dt>0:
                if not math.isclose(checked[1]/dt,round(checked[1]/dt),abs_tol=1e-8):
                    option_errors.append('checked_grid_not_aligned');coverage='unknown';full=False
            if candidate and candidate['native_sampled_valid'] is False and raw.get('safe') is True: option_errors.append('native_rejection_marked_safe')
            reference_only=bool((candidate and candidate['reference_only']) or raceline)
            if raw.get('safe') is True and raw.get('first_rejection_s') is not None:
                option_errors.append('safe_with_rejection_time')
            if raw.get('safe') is True and raw.get('reason') is not None:
                option_errors.append('safe_with_rejection_reason')
            if raw.get('safe') is True and _number(raw.get('min_pair_clearance_m')) and _number(decision.get('pair_threshold_m')) and raw['min_pair_clearance_m']<=decision['pair_threshold_m']:
                option_errors.append('safe_conflicts_recorded_pair_threshold')
            reasons=list(errors)+option_errors
            if reference_only: reasons.append('reference_not_ego_trajectory')
            if candidate and candidate['native_sampled_valid'] is not True: reasons.append('native_rejected')
            if candidate and candidate['footprint_sampled_valid'] is not True: reasons.append('footprint_not_confirmed')
            if ego.get('valid') is not True or lead.get('valid') is not True: reasons.append('invalid_source_state')
            if not full: reasons.append('checked_horizon_unknown' if checked is None else 'incomplete_or_invalid_coverage')
            if any(config.get(k) is None for k in ('footprint_length_m','footprint_width_m')):
                reasons.append('body_dimensions_not_recorded')
            if any(decision.get(k) is None for k in ('pair_threshold_m','wall_threshold_m')):
                reasons.append('margin_thresholds_not_recorded')
            if session_metadata.get('producer_commit')!=PRODUCER:
                reasons.append('model_version_unconfirmed')
            if raw.get('min_pair_clearance_m') is None: reasons.append('missing_min_pair_clearance')
            option=dict(frame_index=index,option_index=option_index,
                option_key='frame:'+str(index)+'/option:'+str(option_index),
                audit_context_sha256=result['audit_context_sha256'],
                plan_id=plan.get('plan_id') if isinstance(plan,dict) else None,
                plan_v2_sha256=item['plan_binding']['plan_sha256'] if item['plan_binding'] else None,
                source_sim_time_s=ego.get('sim_time_s'),source_control_step=ego.get('control_step'),
                geometry_id=session_metadata['geometry_id'],frame_id=session_metadata['frame_id'],
                **{k:raw.get(k) for k in OPTION_FIELDS},original_diagnostics=raw,
                reference_only=reference_only,binding_status=item['binding_status'],
                model='three_discs' if session_metadata.get('producer_commit')==PRODUCER else None,
                body_reference='geometric_body_centre' if session_metadata.get('producer_commit')==PRODUCER else None,
                model_provenance='external_producer_commit_mapping' if session_metadata.get('producer_commit')==PRODUCER else 'unconfirmed',
                clearance_convention='signed_three_disc_pair_minimum_claim_not_single_circle',
                body_length_m=config.get('footprint_length_m'),body_width_m=config.get('footprint_width_m'),
                pair_threshold_m=decision.get('pair_threshold_m'),wall_threshold_m=decision.get('wall_threshold_m'),
                physical_pair_clearance_m=report.get('pair_clearance_m'),physical_wall_clearance_m=config.get('footprint_clearance_m'),
                sample_dt_s=dt,requested_horizon_s=requested,checked_range_s=checked,
                coverage_status=coverage,full_horizon_reported=full,
                evaluation_complete=complete if type(complete) is bool else None,
                eligible_for_ego_trajectory_comparison=False,
                incomparability_reasons=reasons or ['diagnostics_only_not_collision_assessment'],
                audit_status='invalid_contract' if errors or option_errors else 'diagnostics_only',
                option_errors=option_errors,
                raceline_xy_sha256=content_digest('recorded_raceline_xy',frame['raceline_xy']) if raceline and frame.get('raceline_xy') is not None else None,
                inspection_only=True,planning_allowed=False)
            result['options'].append(option)
        item['audit_status']='invalid_contract' if errors else 'joined_diagnostics' if plan is not None or options else 'snapshot_only'
        result['frames'].append(item)
    result['contract_valid']=not global_errors and all(f['audit_status']!='invalid_contract' for f in result['frames']) and all(o['audit_status']!='invalid_contract' for o in result['options'])
    return _copy(result)


def write_audit_outputs(audit, output_dir):
    """New output directory only; CSV cells are JSON to preserve null/types."""
    encoded=json.dumps(audit,indent=2,allow_nan=False)
    out=Path(output_dir);out.mkdir(parents=True,exist_ok=False)
    (out/'audit.json').write_text(encoded,encoding='utf-8')
    fields=list(audit['options'][0]) if audit['options'] else ['frame_index','option_index','candidate_id']
    with (out/'options.csv').open('x',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
        for row in audit['options']:writer.writerow({k:json.dumps(row.get(k),allow_nan=False) for k in fields})


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames',required=True);parser.add_argument('--report',required=True)
    parser.add_argument('--metadata',required=True,help='Explicit geometry_id/frame_id; optional producer_commit')
    parser.add_argument('--classification',required=True,choices=('synthetic','saved_two_car_simulation'))
    parser.add_argument('--parameters');parser.add_argument('--geometry-manifest');parser.add_argument('--expected-manifest-sha256')
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    paths={'frames':Path(args.frames),'report':Path(args.report),'metadata':Path(args.metadata)}
    if args.parameters: paths['parameters']=Path(args.parameters)
    output=Path(args.output).resolve()
    if any(p.resolve().parent==output or p.resolve().parent in output.parents for p in paths.values()):
        raise ValueError('output_must_be_outside_source_directories')
    raw={k:p.read_bytes() for k,p in paths.items()}
    def reject(value): raise ValueError('nonfinite_json:'+value)
    objects={k:json.loads(v,parse_constant=reject) for k,v in raw.items()}
    audit=audit_two_car_frames(objects['frames'],objects['report'],session_metadata=objects['metadata'],
        source_classification=args.classification,parameter_metadata=objects.get('parameters'),
        geometry_manifest_path=args.geometry_manifest,expected_manifest_sha256=args.expected_manifest_sha256)
    audit['source_file_sha256']={k:hashlib.sha256(v).hexdigest() for k,v in raw.items()}
    if any(p.read_bytes()!=raw[k] for k,p in paths.items()):raise RuntimeError('source_files_changed_during_audit')
    write_audit_outputs(audit,args.output)
    if not audit['contract_valid']:
        parser.exit(2, 'Audit contract invalid; diagnostic outputs were saved.\n')


if __name__=='__main__':main()
