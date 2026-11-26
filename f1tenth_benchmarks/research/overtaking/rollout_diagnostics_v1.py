"""Opt-in copied rollout evidence. No collision, projection, cost or control calls."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path

import numpy as np

SCHEMA = 'two_car_rollout_diagnostics_v1'


def _json(value, errors):
    if isinstance(value, np.ndarray):
        return _json(value.tolist(), errors)
    if isinstance(value, np.generic):
        return _json(value.item(), errors)
    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        if not math.isfinite(value):
            errors.append('nonfinite_value'); return None
        return value
    if isinstance(value, (list, tuple)):
        return [_json(v, errors) for v in value]
    if isinstance(value, dict) and all(isinstance(k, str) for k in value):
        return {k: _json(v, errors) for k, v in value.items()}
    errors.append('unsupported_value'); return None


def snapshot_sha256(value):
    """Descriptive JSON snapshot hash, explicitly not a V2 plan binding."""
    errors = []; clean = _json(value, errors)
    if errors: raise ValueError('invalid_snapshot:' + ','.join(errors))
    return hashlib.sha256(json.dumps(clean, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()


def _number(v):
    return type(v) in (int, float) and math.isfinite(v)


def _pose(v):
    return isinstance(v, list) and len(v) == 3 and all(_number(x) for x in v)


def _errors(record):
    errors = []
    source = record.get('source', {}); meta = record.get('metadata', {})
    gid, frame = meta.get('geometry_id'), meta.get('frame_id')
    if not isinstance(gid, str) or not gid or frame != 'map': errors.append('missing_or_invalid_geometry_frame')
    time = source.get('source_sim_time_s'); step = source.get('source_control_step')
    if not _number(time) or time < 0 or type(step) is not int or step < 0: errors.append('invalid_source_clock')
    for key in ('ego_state', 'lead_state'):
        state = source.get(key)
        if not isinstance(state, dict): errors.append('missing_' + key); continue
        if (state.get('sim_time_s') != time or state.get('control_step') != step or
                not _pose(state.get('pose')) or not _number(state.get('speed_mps'))):
            errors.append('invalid_' + key)
        if state.get('frame_id', frame) != frame or state.get('geometry_id', gid) != gid:
            errors.append('source_frame_geometry_mismatch')
    if record.get('plan_binding_v2') is not None: errors.append('unsupported_v2_binding_claim')
    if record.get('binding_status') != 'unverified_snapshot_only': errors.append('unsupported_binding_claim')
    if record.get('historical_producer_binding_verified') is not False:
        errors.append('unsupported_historical_binding_claim')
    plan = record.get('plan_snapshot')
    if plan is not None:
        if (plan.get('source_control_step') != step or plan.get('source_sim_time_s') != time or
                plan.get('source_pose') != source.get('ego_state', {}).get('pose') or plan.get('geometry_id') != gid or
                plan.get('frame_id', frame) != frame): errors.append('plan_source_mismatch')
        if snapshot_sha256(plan) != record.get('plan_snapshot_sha256'): errors.append('plan_snapshot_hash_mismatch')
    for name in ('ego_state', 'lead_state'):
        if snapshot_sha256(source.get(name)) != record.get(name.replace('_state', '_state_sha256')):
            errors.append('source_identity_mismatch')
    if meta.get('model') != 'three_discs' or meta.get('body_reference') != 'geometric_body_centre':
        errors.append('invalid_body_model')
    for name in ('body_length_m','body_width_m','body_radius_m','physics_dt_s','sample_dt_s'):
        if not _number(meta.get(name)) or meta[name] <= 0: errors.append('invalid_body_or_time_parameters')
    for path in record.get('tracking_paths',[]):
        if snapshot_sha256(path.get('xy')) != path.get('xy_sha256'): errors.append('tracking_path_hash_mismatch')
    groups = {}; seen = set()
    for group in record.get('lead_predictions', []):
        key = group.get('prediction_group_id'); times = group.get('requested_sample_times_s', []); poses = group.get('poses', [])
        if key in groups: errors.append('duplicate_prediction_group')
        groups[key] = group
        dt = meta.get('sample_dt_s')
        if (len(times)<2 or not _number(dt) or dt <= 0 or len(poses) != len(times) or
                any(not _number(t) or not math.isclose(t, k*dt, rel_tol=0, abs_tol=1e-9) for k,t in enumerate(times)) or
                any(not _pose(p) for p in poses)): errors.append('invalid_lead_grid')
        if poses and poses[0] != source.get('lead_state', {}).get('pose'): errors.append('lead_origin_mismatch')
    for index, option in enumerate(record.get('options', [])):
        eid = option.get('evaluation_id')
        if not isinstance(eid,str) or eid in seen: errors.append('invalid_evaluation_id')
        seen.add(eid)
        if option.get('option_index') != index: errors.append('option_order_mismatch')
        group = groups.get(option.get('prediction_group_id'))
        if group is None: errors.append('missing_prediction_group'); continue
        times = group['requested_sample_times_s']; samples = option.get('samples', [])
        if (option.get('requested_sample_times_s') != times or option.get('requested_horizon_s') != times[-1] or
                option.get('sample_dt_s') != meta.get('sample_dt_s')): errors.append('option_grid_mismatch')
        if not _number(option.get('target_speed_mps')) or option['target_speed_mps'] < 0: errors.append('invalid_speed')
        cid = option.get('candidate_id')
        if not (type(cid) is int and cid in range(1,6)) and not (cid is None and option.get('reference_id')):
            errors.append('invalid_option_identity')
        for k,s in enumerate(samples):
            if (k >= len(times) or s.get('sample_index') != k or s.get('t_s') != times[k] or
                    not _pose(s.get('ego_pose')) or not _pose(s.get('lead_pose')) or
                    s.get('lead_pose') != group['poses'][k] or not _number(s.get('ego_speed_mps')) or
                    not _number(s.get('signed_min_pair_clearance_m'))): errors.append('invalid_sample')
        expected = bool(samples and len(samples) == len(times) and option.get('termination_reason') == 'horizon_reached')
        if option.get('evaluation_complete') is not expected: errors.append('false_complete_claim')
        start = samples[0]['t_s'] if samples else None; end = samples[-1]['t_s'] if samples else None
        if (option.get('checked_start_s') != start or option.get('checked_end_s') != end or
                option.get('checked_sample_count') != len(samples)): errors.append('checked_range_mismatch')
        if samples and samples[0]['ego_pose'] != source.get('ego_state',{}).get('pose'): errors.append('ego_origin_mismatch')
        for name in ('candidate_snapshot','reference_xy'):
            if option.get(name) is not None and snapshot_sha256(option[name]) != option.get(name+'_sha256'):
                errors.append('option_snapshot_mismatch')
        if samples:
            minimum = min(s['signed_min_pair_clearance_m'] for s in samples if _number(s.get('signed_min_pair_clearance_m')))
            if not _number(option.get('signed_min_pair_clearance_m')) or not math.isclose(minimum, option['signed_min_pair_clearance_m'], rel_tol=0, abs_tol=1e-12):
                errors.append('native_minimum_mismatch')
        if option.get('reference_only') and option.get('source_path_type') != 'reference_centerline':
            errors.append('reference_semantics_mismatch')
        if option.get('inspection_only') is not True or option.get('planning_allowed') is not False: errors.append('control_flag_mismatch')
    return sorted(set(errors))


def validate_record(record):
    """Strict reader validation. Invalid records are never complete evidence."""
    errors = []; clean = _json(record, errors)
    if not isinstance(clean,dict) or clean.get('schema') != SCHEMA or clean.get('inspection_only') is not True or clean.get('planning_allowed') is not False:
        raise ValueError('invalid_diagnostics_schema')
    if type(clean.get('schema_version')) is not int or clean['schema_version'] != 1:
        raise ValueError('unsupported_diagnostics_version')
    if not isinstance(clean.get('errors'), list) or not all(isinstance(e,str) for e in clean['errors']):
        raise ValueError('invalid_diagnostics_errors')
    try: errors += _errors(clean)
    except (KeyError,TypeError,ValueError,IndexError): errors.append('malformed_diagnostics')
    if errors or clean.get('status') != 'valid': raise ValueError(','.join(sorted(set(errors + clean.get('errors', [])))))
    return clean


class RolloutDiagnostics:
    """Memory collector; serialization/hashes run only in finish_frame()."""
    def __init__(self, source_classification='unclassified_runtime_model'):
        self.source_classification = source_classification
        self.frame_counter = 0
        self.last_record = None

    def begin_frame(self, ego, lead):
        self.source = deepcopy(dict(source_control_step=ego.get('control_step'), source_sim_time_s=ego.get('sim_time_s'),
                                    ego_state=ego, lead_state=lead))
        self.source_runtime = None
        self.groups = []; self.options = []; self.group = None; self.active = None
        self.tracking_paths = []; self._tracking_ids = {}; self._tracking_refs = []
        self.frame_counter += 1

    def prediction(self, times, poses, rate):
        self.group = dict(prediction_group_id=f'{self.frame_counter}:lead:{len(self.groups)}',
            requested_sample_times_s=times.copy(), poses=poses.copy(), lead_s_rate_mps=rate,
            model='fixed_frenet_d_constant_s_rate_yaw_decay_0.3s')
        self.groups.append(self.group)

    def start_option(self, candidate_id, speed, path=None, reference_id=None, reference_xy=None):
        self.active = dict(evaluation_id=f'{self.frame_counter}:eval:{len(self.options)}', candidate_id=candidate_id,
            reference_id=reference_id, target_speed_mps=speed, prediction_group_id=self.group['prediction_group_id'],
            source_path_type=path.get('path_type','state_matched_trajectory') if path else None,
            reference_only=bool(path and path.get('path_type')=='reference_centerline'),
            candidate_snapshot=deepcopy(path), reference_xy=reference_xy.copy() if reference_xy is not None else None,
            requested_sample_times_s=self.group['requested_sample_times_s'], samples=[],
            rollout_attempted=False, termination_reason='not_started', error_operation=None,
            last_integrated_t_s=None, failed_target_grid_t_s=None,
            inspection_only=True, planning_allowed=False)
        self.options.append(self.active)

    def rollout_start(self, pp):
        key = id(pp)
        if key not in self._tracking_ids:
            self._tracking_refs.append(pp)
            self._tracking_ids[key] = len(self.tracking_paths)
            self.tracking_paths.append(dict(tracking_path_id=len(self.tracking_paths), xy=pp.path.copy()))
        self.active['tracking_path_id'] = self._tracking_ids[key]
        self.active['pp_settings'] = {k:getattr(pp,k,None) for k in ('speed','lookahead','wheelbase','max_steer')}
        self.active['rollout_attempted'] = True
        self.active['last_integrated_t_s'] = 0.
        self.active['termination_reason'] = 'horizon_reached'

    def sample(self, index, time, state, clearance, wall):
        self.active['samples'].append(dict(sample_index=index, t_s=float(time), ego_pose=state[[0,1,4]].copy(),
            ego_speed_mps=float(state[3]), signed_min_pair_clearance_m=clearance,
            wall_intersects=wall, wall_check_status='not_checked' if wall is None else 'checked'))

    def integrated(self, time):
        self.active['last_integrated_t_s'] = time

    def stopped(self, time, operation):
        self.active.update(termination_reason='rollout_value_error', failed_target_grid_t_s=float(time), error_operation=operation)

    def finish_option(self, result):
        self.active['native'] = dict(result)
        self.active['_result_identity'] = id(result)
        if not self.active['rollout_attempted']:
            self.active['termination_reason'] = 'not_started:' + str(result.get('reason'))

    def finish_frame(self, transition, selector, geometry):
        """Invalid source evidence is marked; it never changes control results."""
        errors = []
        plan = transition.get('plan'); source = self.source
        metadata = dict(geometry_id=getattr(geometry,'geometry_id',None), frame_id=getattr(geometry,'frame_id',None),
            model='three_discs', body_reference='geometric_body_centre', body_length_m=selector.body.length,
            body_width_m=selector.body.width, body_radius_m=selector.body.radius,
            pair_threshold_m=selector.pair_margin, wall_threshold_m=selector.wall_margin,
            configured_pair_clearance_m=selector.sim.pair_clearance, configured_wall_clearance_m=selector.sim.wall_clearance,
            physical_pair_clearance_m=None, physical_wall_clearance_m=None,
            clearance_convention='signed_three_disc_pair_minimum_claim_not_single_circle',
            sample_dt_s=selector.sample_dt, physics_dt_s=selector.dt, sample_steps=selector.sample_steps,
            steer_ticks=selector.steer_ticks, source_physics_tick=selector.sim.physics_tick,
            held_steering=selector.held_steering, physics_parameters=vars(selector.sim.params).copy(),
            controller_config=vars(selector.controller.config).copy(),
            lead_prediction_is_ground_truth=False, projection_performed=False)
        metadata['controller_vehicle_parameters'] = vars(selector.controller.vehicle).copy() if hasattr(selector.controller,'vehicle') else None
        linked = getattr(selector.controller,'speed_linked_pp',None)
        metadata['speed_linked_pp_parameters'] = vars(linked).copy() if linked is not None else None
        # The step wrapper supplies pre-step tick/steering; never label post-step state as source.
        if self.source_runtime is None:
            errors.append('missing_source_runtime')
            metadata.update(source_physics_tick=None, held_steering=None)
        else:
            metadata.update(self.source_runtime)
        metadata.update(units=dict(pose_xy='m',yaw='rad',relative_time='s',speed='m/s',signed_clearance='m'),
                        swept_contact_checked=False, equal_body_radius_assumed=True)
        record = dict(schema=SCHEMA, schema_version=1, source_classification=self.source_classification, source=source, metadata=metadata,
            binding_status='unverified_snapshot_only', plan_binding_v2=None,
            historical_producer_binding_verified=False, plan_snapshot=deepcopy(plan),
            lead_predictions=self.groups, tracking_paths=self.tracking_paths, options=[], inspection_only=True, planning_allowed=False)
        if len(self.options) != len(transition.get('costs',{}).get('options',[])):
            errors.append('unmatched_evaluation_count')
        by_id = {o['_result_identity']:o for o in self.options if '_result_identity' in o}
        for index,native in enumerate(transition.get('costs',{}).get('options',[])):
            raw = by_id.get(id(native))
            if raw is None:
                errors.append('unmatched_option'); continue
            o = {k:v for k,v in raw.items() if not k.startswith('_') and k!='native'}
            group = next(g for g in self.groups if g['prediction_group_id']==o['prediction_group_id'])
            times = group['requested_sample_times_s']; samples=o['samples']
            for s in samples:s['lead_pose']=group['poses'][s['sample_index']].copy()
            o.update(option_index=index, requested_horizon_s=float(times[-1]), sample_dt_s=selector.sample_dt,
                checked_start_s=samples[0]['t_s'] if samples else None, checked_end_s=samples[-1]['t_s'] if samples else None,
                checked_sample_count=len(samples), evaluation_complete=bool(samples and len(samples)==len(times) and o['termination_reason']=='horizon_reached'),
                evaluation_scope='discrete_pair_grid', rollout_trajectory_type='modeled_ego_rollout' if samples else None,
                wall_evaluation_complete=bool(samples and len(samples)==len(times) and all(s['wall_intersects'] is not None for s in samples)),
                native_safe=native.get('safe'), native_reason=native.get('reason'), first_rejection_s=native.get('first_rejection_s'),
                signed_min_pair_clearance_m=native.get('min_pair_clearance_m'), native_relative_s_progress_m=native.get('relative_s_progress_m'),
                post_gate_reason=native.get('reason') if native.get('reason')=='insufficient_pass_progress' else None)
            record['options'].append(o)
        clean = _json(record, errors)
        try:
            clean['plan_snapshot_sha256']=snapshot_sha256(clean['plan_snapshot']) if plan is not None else None
            clean['ego_state_sha256']=snapshot_sha256(clean['source']['ego_state'])
            clean['lead_state_sha256']=snapshot_sha256(clean['source']['lead_state'])
            for path in clean['tracking_paths']:
                path['xy_sha256']=snapshot_sha256(path['xy'])
            for option in clean['options']:
                option['candidate_snapshot_sha256']=snapshot_sha256(option['candidate_snapshot']) if option['candidate_snapshot'] is not None else None
                option['reference_xy_sha256']=snapshot_sha256(option['reference_xy']) if option['reference_xy'] is not None else None
            errors += _errors(clean)
        except (KeyError,TypeError,ValueError,IndexError):errors.append('malformed_diagnostics')
        clean.update(status='invalid' if errors else 'valid', errors=sorted(set(errors)))
        self.last_record = clean
        return clean


def write_sidecar(path, records):
    """Exclusive creation after decisions; keep explicit invalid records too."""
    path=Path(path)
    lines=[]
    for r in records:
        errors=[];clean=_json(r,errors)
        if errors:raise ValueError('nonfinite_sidecar')
        if clean.get('inspection_only') is not True or clean.get('planning_allowed') is not False:
            raise ValueError('invalid_control_flags')
        if clean.get('status')=='valid':validate_record(clean)
        elif clean.get('status')!='invalid':raise ValueError('missing_status')
        lines.append(json.dumps(clean,sort_keys=True,allow_nan=False))
    with path.open('x') as stream:
        for line in lines:stream.write(line+'\n')
