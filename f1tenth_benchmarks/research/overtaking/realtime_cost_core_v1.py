"""Read-only shadow costs. Exact V1 math; no ranking, control, IO or hot-path hashes."""
from collections.abc import Mapping
from dataclasses import dataclass
import math
from time import perf_counter
from types import MappingProxyType

import numpy as np

from .cost_evaluation_v1 import _component
from .offline_rollout_frenet_v1 import project_pose_series, relative_progress_features
from .realtime_cost_inputs_v1 import RuntimeCostContext, _length_key

FLAGS = dict(inspection_only=True, planning_allowed=False, formal_comparison_qualified=False)


def frozen(value):
    if isinstance(value, Mapping):
        return MappingProxyType({k: frozen(v) for k, v in value.items()})
    if isinstance(value, (list, tuple, np.ndarray)):
        return tuple(frozen(v) for v in value)
    return value


def tree(value):
    if isinstance(value, Mapping):
        return tuple((k, tree(v)) for k, v in sorted(value.items()))
    if isinstance(value, (list, tuple, np.ndarray)):
        return tuple(tree(v) for v in value)
    return value


def number(v):
    return isinstance(v, (int, float, np.integer, np.floating)) and not isinstance(v, (bool, np.bool_)) and math.isfinite(v)


def projection_key(record, context):
    # Exact value join, not content authentication. Include anchors, poses, grids,
    # groups and model metadata so an old result cannot silently follow new data.
    return tree((context.geometry_id, vars(context.tracking_options), record.get('source'), record.get('metadata'),
        record.get('lead_predictions'), [(o.get('evaluation_id'), o.get('prediction_group_id'),
        o.get('candidate_id'), o.get('reference_id'), o.get('target_speed_mps'),
        o.get('requested_sample_times_s'), [(s.get('t_s'), s.get('ego_pose'), s.get('lead_pose'))
        for s in o.get('samples', [])]) for o in record.get('options', [])]))


@dataclass(frozen=True)
class ProjectionBundle:
    key: tuple
    entries: object
    method: str = 'exact_existing_frenet_tracker_v1'


def prepare_projection_bundle(record, context):
    """Explicit non-hot preprocessing; shared lead projection, private ego trackers.

    This is expensive and cannot be assumed available in a native Planner.
    A bundle is valid only for the exact sampled rollout and source anchors.
    """
    src, meta = record['source'], record['metadata']
    entries, lead_cache = {}, {}
    groups = {g['prediction_group_id']: g for g in record['lead_predictions']}
    for o in record['options']:
        eid, gid = o['evaluation_id'], o['prediction_group_id']
        if eid in entries:
            raise ValueError('duplicate_evaluation_id')
        try:
            samples = o['samples']
            if not samples:
                entries[eid] = frozen(dict(reason='missing_rollout_samples', ego=None, lead=None))
                continue
            g = groups[gid]
            if gid not in lead_cache:
                if len(g['poses']) != len(g['requested_sample_times_s']):
                    raise ValueError('group_grid_mismatch')
                lead_cache[gid] = project_pose_series([dict(t_s=t, pose=list(p)) for t, p in
                    zip(g['requested_sample_times_s'], g['poses'])], src['lead_state'], context.geometry,
                    geometry_id=meta['geometry_id'], frame_id=meta['frame_id'], tracking_options=context.tracking_options)
            lead = dict(lead_cache[gid], samples=lead_cache[gid]['samples'][:len(samples)])
            ego = project_pose_series([dict(t_s=s['t_s'], pose=list(s['ego_pose'])) for s in samples],
                src['ego_state'], context.geometry, geometry_id=meta['geometry_id'], frame_id=meta['frame_id'],
                tracking_options=context.tracking_options)
            entries[eid] = frozen(dict(ego=ego, lead=lead, reason=None))
        except (ValueError, KeyError, TypeError) as exc:
            entries[eid] = frozen(dict(reason='projection_unavailable:'+str(exc), ego=None, lead=None))
    return ProjectionBundle(projection_key(record, context), MappingProxyType(entries))


@dataclass(frozen=True)
class RuntimeOption:
    identity: tuple
    source_path_type: object
    reference_only: bool
    native_safe: object
    native_reason: object
    native_sampled_valid: object
    requested_horizon_s: object
    complete: bool
    checked_range_s: object
    length_m: object
    length_method: object
    length_reason: object
    risk_samples: tuple
    projected: object
    reasons: tuple


@dataclass(frozen=True)
class RuntimeBatch:
    source: tuple
    options: tuple
    risk_spec: object
    preparation_s: float
    projection_state: str
    model_context: object


def prepare_runtime_inputs(record, context, *, length_cache=None, projection_bundle=None):
    """Memory adapter; no IO, hashes, sampling, projection or native rollout.

    Converts existing samples to privately frozen minimal inputs. The caller
    must provide an owned immutable source snapshot while this function runs.
    """
    start = perf_counter()
    src, meta = record.get('source', {}), record.get('metadata', {})
    plan, options = record.get('plan_snapshot'), record.get('options', [])
    step, stamp = src.get('source_control_step'), src.get('source_sim_time_s')
    base = []
    if type(step) is not int or step < 0 or not number(stamp) or stamp < 0:
        base.append('missing_source_clock')
    if meta.get('geometry_id') != context.geometry_id or context.geometry.geometry_id != context.geometry_id:
        base.append('geometry_mismatch')
    if meta.get('frame_id') != 'map':
        base.append('frame_mismatch')
    if meta.get('model') != 'three_discs':
        base.append('model_mismatch')
    for role in ('ego_state', 'lead_state'):
        actor = src.get(role, {}); fr = actor.get('frenet', {})
        if actor.get('control_step') != step or actor.get('sim_time_s') != stamp:
            base.append('source_state_clock_mismatch')
        if (fr.get('geometry_id'), fr.get('frame_id')) != (meta.get('geometry_id'), meta.get('frame_id')):
            base.append('source_state_context_mismatch')
        pose = actor.get('pose')
        if not isinstance(pose, (list, tuple)) or len(pose) != 3 or not all(number(v) for v in pose):
            base.append('invalid_source_pose')
        if actor.get('valid') is not True:
            base.append('invalid_source_state')
    if plan is not None:
        if (plan.get('source_control_step'), plan.get('source_sim_time_s'), tree(plan.get('source_pose'))) != (step, stamp, tree(src.get('ego_state', {}).get('pose'))):
            base.append('plan_source_mismatch')
        if plan.get('geometry_id') != meta.get('geometry_id') or plan.get('frame_id', 'map') != meta.get('frame_id'):
            base.append('plan_context_mismatch')
    groups = {}; group_duplicates = set()
    for g in record.get('lead_predictions', []):
        gid = g.get('prediction_group_id')
        if gid in groups: group_duplicates.add(gid)
        groups[gid] = g
    ids = [o.get('evaluation_id') for o in options]
    paths = {p.get('candidate_id'): p for p in (plan or {}).get('paths', [])}
    plan_key = _length_key(plan, context.geometry_id)
    length_ok = length_cache is not None and length_cache.key == plan_key
    proj_ok = projection_bundle is not None and projection_bundle.key == projection_key(record, context)
    rows = []
    for o in options:
        reasons = list(base)
        eid, gid, cid, speed = (o.get(k) for k in ('evaluation_id', 'prediction_group_id', 'candidate_id', 'target_speed_mps'))
        if not isinstance(eid, str) or not eid or ids.count(eid) != 1:
            reasons.append('missing_or_duplicate_evaluation_id')
        if not isinstance(gid, str) or gid not in groups or gid in group_duplicates:
            reasons.append('missing_or_duplicate_prediction_group')
        if not number(speed) or speed < 0: reasons.append('invalid_target_speed')
        path = paths.get(cid)
        if cid is None:
            if o.get('reference_id') != 'raceline': reasons.append('missing_reference_identity')
        elif path is None:
            reasons.append('candidate_not_in_plan')
        elif path.get('path_type') != o.get('source_path_type'):
            reasons.append('candidate_path_type_mismatch')
        reference = bool(o.get('reference_only') or o.get('source_path_type') == 'reference_centerline' or o.get('reference_id') == 'raceline')
        times, horizon, samples = o.get('requested_sample_times_s'), o.get('requested_horizon_s'), o.get('samples', [])
        valid_grid = (isinstance(times, (list, tuple, np.ndarray)) and len(times) >= 2 and
            all(number(t) for t in times) and times[0] == 0 and all(b > a for a, b in zip(times, times[1:])) and
            number(horizon) and horizon > 0 and abs(times[-1]-horizon) < 1e-9)
        if not valid_grid: reasons.append('invalid_requested_grid')
        group = groups.get(gid)
        if valid_grid and group is not None:
            if tree(times) != tree(group.get('requested_sample_times_s')) or len(group.get('poses', [])) != len(times):
                reasons.append('group_grid_mismatch')
        risk_samples = []
        if valid_grid:
            if len(samples) > len(times): reasons.append('excess_samples')
            for i, s in enumerate(samples):
                t, c = s.get('t_s'), s.get('signed_min_pair_clearance_m')
                if i >= len(times) or not number(t) or abs(t-times[i]) > 1e-9 or not number(c):
                    reasons.append('invalid_sample_grid_or_clearance'); break
                for role in ('ego_pose', 'lead_pose'):
                    pose = s.get(role)
                    if not isinstance(pose, (list, tuple)) or len(pose) != 3 or not all(number(v) for v in pose):
                        reasons.append('invalid_sample_pose')
                if group is not None and i < len(group.get('poses', [])) and tree(s.get('lead_pose')) != tree(group['poses'][i]):
                    reasons.append('lead_sample_group_mismatch')
                risk_samples.append(frozen(dict(t_s=float(t), signed_clearance_m=float(c))))
        if samples:
            for role in ('ego', 'lead'):
                a, b = samples[0].get(role+'_pose'), src.get(role+'_state', {}).get('pose')
                if a is None or b is None or len(a) != 3 or len(b) != 3 or not all(number(v) for v in a+b) or not np.allclose(a, b, rtol=0, atol=1e-9):
                    reasons.append('sample_t0_source_pose_mismatch')
        checked_start = o.get('checked_start_s', 0.)
        checked_end = o.get('checked_end_s', samples[-1]['t_s'] if samples else None)
        complete = (not reasons and bool(samples) and len(samples) == len(times) and
            o.get('evaluation_complete') is True and number(o.get('last_integrated_t_s')) and
            abs(o['last_integrated_t_s']-horizon) < 1e-9 and checked_start == 0. and
            number(checked_end) and abs(checked_end-horizon) < 1e-9)
        length = length_cache.entries.get(cid) if length_ok else None
        length_reason = None if length is not None else 'missing_length_cache' if length_cache is None else 'length_cache_invalidated_or_missing'
        if reasons: length = None; length_reason = 'invalid_source_contract'
        projected = projection_bundle.entries.get(eid) if proj_ok else None
        rows.append(RuntimeOption((eid, gid, cid, o.get('reference_id'), speed), o.get('source_path_type'), reference,
            o.get('native_safe'), o.get('native_reason'), path.get('sampled_valid') if path else None, horizon, bool(complete),
            (samples[0]['t_s'], samples[-1]['t_s']) if samples else None, length,
            length_cache.method if length is not None else None, length_reason,
            tuple(risk_samples) if not reasons else (), projected, tuple(sorted(set(reasons)))))
    return RuntimeBatch((step, stamp, meta.get('geometry_id'), meta.get('frame_id'), meta.get('model')),
        tuple(rows), context.risk_spec, perf_counter()-start,
        'exact_bundle' if proj_ok else 'unavailable' if projection_bundle is None else 'invalidated',
        frozen({k:meta.get(k) for k in ('body_reference','body_length_m','body_width_m','body_radius_m',
            'pair_threshold_m','wall_threshold_m','configured_pair_clearance_m','configured_wall_clearance_m','sample_dt_s')}))


def evaluate_formula_components(o, risk_spec):
    """Three raw components only, using the original integration functions."""
    raw_risk = partial_risk = progress = partial_progress = None
    reason = None
    if o.risk_samples and not o.reasons:
        component = _component('risk', risk_spec, dict(requested_horizon_s=o.requested_horizon_s,
            comparison_status='reference_only' if o.reference_only else 'unsafe' if o.native_safe is False else 'observation',
            risk_series=dict(status='reported_complete' if o.complete else 'reported_partial',
                evaluation_complete=o.complete, samples=o.risk_samples)))
        raw_risk, partial_risk = component['raw_value'], component['checked_partial_integral_m2_s']
    if o.projected is not None and not o.reasons:
        reason = o.projected['reason']
        if reason is None:
            try:
                p = relative_progress_features(o.projected['ego'], o.projected['lead'],
                    requested_horizon_s=o.requested_horizon_s, native_complete=o.complete)
                progress, partial_progress, reason = p['progress_integral_m_s'], p['checked_partial_progress_integral_m_s'], p['reason']
            except ValueError as exc: reason = str(exc)
    else: reason = 'projection_unavailable_or_invalidated'
    return o.length_m, raw_risk, partial_risk, progress, partial_progress, reason


def evaluate_cost_batch(batch):
    """Exact original Risk + Frenet integrators on prevalidated memory inputs."""
    start = perf_counter(); outputs = []
    for o in batch.options:
        _, raw_risk, partial_risk, progress, partial_progress, reason = evaluate_formula_components(o, batch.risk_spec)
        status = ('rejected' if o.reasons or o.native_sampled_valid is False else 'reference_only' if o.reference_only else
            'unsafe' if o.native_safe is False else 'unknown' if o.native_safe is not True or o.native_reason is not None else
            'unknown' if o.identity[2] is not None and o.native_sampled_valid is not True else
            'incomplete' if not o.complete else 'complete_observation')
        outputs.append(dict(evaluation_id=o.identity[0], prediction_group_id=o.identity[1], candidate_id=o.identity[2],
            reference_id=o.identity[3], target_speed_mps=o.identity[4], source_control_step=batch.source[0],
            source_sim_time_s=batch.source[1], geometry_id=batch.source[2], frame_id=batch.source[3], model=batch.source[4],
            source_path_type=o.source_path_type, reference_only=o.reference_only,
            native_safe=o.native_safe, native_reason=o.native_reason, native_sampled_valid=o.native_sampled_valid,
            requested_horizon_s=o.requested_horizon_s,
            checked_range_s=list(o.checked_range_s) if o.checked_range_s else None, pair_complete=o.complete,
            length_m=o.length_m, length_method=o.length_method, length_reason=o.length_reason,
            length_scope='reference_geometry_only' if o.reference_only else 'candidate_geometry_not_modeled_travel',
            risk_m2_s=raw_risk, partial_risk_m2_s=partial_risk, progress_m_s=progress,
            partial_progress_m_s=partial_progress, progress_reason=reason, status=status,
            native_rejected=bool(o.native_safe is False and not o.risk_samples),
            features_complete=bool(o.complete and o.length_m is not None and raw_risk is not None and progress is not None),
            reasons=list(o.reasons), source_binding='exact_memory_value_join_not_historical_authentication', **FLAGS))
    return dict(schema='runtime_cost_core_v1', schema_version=1, options=outputs,
        model_context=dict(batch.model_context), model_verification_status='not_performed_in_hot_path',
        preparation_s=batch.preparation_s, calculation_s=perf_counter()-start,
        total_cost=None, comparison_status='not_assessed_no_weights_no_ranking', **FLAGS)


def evaluate_runtime_cost_core(record, context, *, length_cache=None, projection_bundle=None):
    """One-call adapter + core, with projection explicitly excluded if precomputed."""
    return evaluate_cost_batch(prepare_runtime_inputs(record, context, length_cache=length_cache,
        projection_bundle=projection_bundle))
