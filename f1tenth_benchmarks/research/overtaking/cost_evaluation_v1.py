"""Versioned offline component costs; explicit experimental policy, no selection."""
import math

from .cost_feature_v1 import METHOD, flags, json_copy, number


def _positive(value, name):
    if not number(value) or value <= 0:
        raise ValueError('invalid_positive_scale:' + name)
    return value


def _range(value, name):
    if not isinstance(value, list) or len(value) != 2 or not all(number(x) for x in value) or value[0] > value[1]:
        raise ValueError('invalid_applicability_range:' + name)
    return value


def _inside(value, interval):
    return number(value) and interval[0] <= value <= interval[1]


def _policy(value):
    if value is None:
        return None
    p = json_copy(value)
    if not isinstance(p, dict):
        raise ValueError('policy_must_be_object')
    if type(p.get('schema_version')) is not int or p['schema_version'] != 1 or p.get('experimental') is not True or not isinstance(p.get('policy_id'), str) or not p['policy_id']:
        raise ValueError('explicit_versioned_experimental_policy_required')
    components = p.get('components')
    if not isinstance(components, dict) or not components or set(components) - {'length', 'risk', 'progress'}:
        raise ValueError('invalid_components')
    for name, spec in components.items():
        if not isinstance(spec, dict):
            raise ValueError('invalid_component_policy')
        if name == 'length':
            _positive(spec.get('scale_m'), name)
            _range(spec.get('valid_length_m'), name)
        elif name == 'progress':
            if spec.get('mode') != 'negative_relative_progress':
                raise ValueError('unsupported_progress_mode')
            _positive(spec.get('scale_m'), name)
            _range(spec.get('valid_progress_m'), name)
        else:
            if spec.get('mode') not in ('exposure_integral', 'minimum_shortfall_squared'):
                raise ValueError('unsupported_risk_mode')
            if not number(spec.get('clearance_ref_m')) or spec['clearance_ref_m'] < 0:
                raise ValueError('invalid_clearance_reference')
            _positive(spec.get('clearance_scale_m'), name)
            _range(spec.get('valid_clearance_m'), name)
            if spec['mode'] == 'exposure_integral':
                _positive(spec.get('time_scale_s'), name)
                _range(spec.get('valid_horizon_s'), name)
    weights = p.get('weights')
    if weights is not None and (not isinstance(weights, dict) or set(weights) != set(components) or
                               not all(number(x) and x >= 0 for x in weights.values()) or not any(x > 0 for x in weights.values())):
        raise ValueError('explicit_nonnegative_weights_for_all_components_required')
    return p


def _component(name, spec, row):
    raw = normalized = checked_integral = None
    reason = None
    unit = 'm'
    if name == 'length':
        raw = row.get('geometry_length_m')
        if row.get('reference_only'):
            reason = 'reference_geometry_not_ego_travel'
        elif not _inside(raw, spec['valid_length_m']):
            reason = 'missing_or_out_of_range_length'
        else:
            normalized = raw / spec['scale_m']
    elif name == 'progress':
        progress = row.get('relative_s_progress_m')
        raw = -progress if number(progress) else None
        if not _inside(progress, spec['valid_progress_m']):
            reason = 'missing_or_out_of_range_progress'
        else:
            normalized = raw / spec['scale_m']
    else:
        mode = spec['mode']; ref = spec['clearance_ref_m']
        if mode == 'minimum_shortfall_squared':
            unit = 'm^2'
            minimum = row.get('min_pair_clearance_m')
            if not _inside(minimum, spec['valid_clearance_m']):
                reason = 'missing_or_out_of_range_signed_minimum'
            else:
                raw = max(0., ref - minimum) ** 2
                normalized = raw / spec['clearance_scale_m'] ** 2
        else:
            unit = 'm^2*s'
            series = row.get('risk_series', {})
            samples = series.get('samples')
            if series.get('status') not in ('reported_complete', 'reported_partial') or not samples:
                reason = 'missing_valid_clearance_samples'
            elif not _inside(row.get('requested_horizon_s'), spec['valid_horizon_s']):
                reason = 'horizon_outside_policy_range'
            elif not all(_inside(s['signed_clearance_m'], spec['valid_clearance_m']) for s in samples):
                reason = 'clearance_outside_policy_range'
            else:
                checked_integral = sum((max(0., ref-a['signed_clearance_m'])**2 +
                                        max(0., ref-b['signed_clearance_m'])**2) *
                                       (b['t_s']-a['t_s']) / 2 for a,b in zip(samples[:-1], samples[1:]))
                if series.get('evaluation_complete') is not True:
                    reason = 'partial_exposure_not_full_horizon_cost'
                else:
                    raw = checked_integral
                    normalized = raw / (spec['clearance_scale_m']**2 * spec['time_scale_s'])
    if row.get('comparison_status') in ('rejected', 'unsafe', 'reference_only'):
        normalized = None
        reason = 'native_or_reference_gate:' + row['comparison_status']
    return dict(raw_value=raw, raw_unit=unit, normalized_value=normalized,
                normalized_unit='dimensionless', status='available' if normalized is not None else 'unavailable',
                reason=reason, checked_partial_integral_m2_s=checked_integral,
                interpretation='minimum diagnostic is not exposure' if name=='risk' and spec['mode']=='minimum_shortfall_squared' else None)


def _comparison(rows, all_rows, policy, features):
    """A policy may request a group, but cannot assert away numeric differences."""
    c = policy.get('comparison')
    if not isinstance(c, dict):
        return set(), ['explicit_comparison_group_required']
    errors = []
    indices = c.get('option_indices')
    if type(c.get('frame_index')) is not int or not isinstance(indices, list) or len(indices)<2 or any(type(i) is not int for i in indices) or len(set(indices)) != len(indices):
        return set(), ['invalid_comparison_group']
    selected = [r for r in all_rows if r['frame_index']==c['frame_index'] and r['option_index'] in indices]
    keys = {(r['frame_index'],r['option_index']) for r in selected}
    if len(selected) != len(indices):errors.append('comparison_option_missing')
    if c.get('audit_context_sha256') != features.get('audit_context_sha256') or c.get('accept_audit_time_binding') is not True:
        errors.append('explicit_audit_time_binding_acceptance_required')
    tolerance=c.get('endpoint_tolerance_m')
    if c.get('goal_mode')!='common_cartesian_endpoints' or not number(tolerance) or not 0<=tolerance<=1e-4:
        errors.append('strict_common_cartesian_endpoints_required');tolerance=0.
    first=selected[0] if selected else {}
    for row in selected:
        if row.get('comparison_status')!='features_complete_policy_required':errors.append('option_not_complete_comparable')
        if (row.get('native_sampled_valid') is not True or row.get('footprint_sampled_valid') is not True or
            row.get('native_safe') is not True or row.get('native_option_reason') is not None or row.get('first_rejection_s') is not None or
            row.get('reference_only') is not False or row.get('evaluation_complete') is not True or row.get('native_full_horizon_reported') is not True or
            row.get('checked_range_s') != [0., row.get('requested_horizon_s')]):
            errors.append('native_gate_or_coverage_inconsistent')
        if (row.get('model') != 'three_discs' or row.get('body_reference') != 'geometric_body_centre' or
            not number(row.get('min_pair_clearance_m')) or not number(row.get('pair_threshold_m')) or
            row['min_pair_clearance_m'] <= row['pair_threshold_m']):
            errors.append('native_three_disc_gate_unconfirmed')
        if row.get('historical_producer_binding_verified') is not False or row.get('binding_status')!='matching_content_records_not_producer_verified':
            errors.append('unsupported_binding_contract')
        if features.get('source_classification')=='synthetic':
            if c.get('allow_synthetic_geometry') is not True:errors.append('synthetic_geometry_not_explicitly_allowed')
        elif not (row.get('geometry_object_curve_verified') and row.get('geometry_artifact_evidence_claimed')):
            errors.append('geometry_evidence_missing')
        for name in ('geometry_start_xy','geometry_end_xy'):
            a=row.get(name);b=first.get(name)
            if not isinstance(a,list) or not isinstance(b,list) or len(a)!=2 or len(b)!=2 or not all(number(v) for v in a+b) or math.hypot(a[0]-b[0],a[1]-b[1])>tolerance:
                errors.append('different_or_missing_cartesian_endpoints')
        for name in ('reference_interval_s_m','source_sim_time_s','plan_id','source_control_step','source_pose','plan_v2_sha256','geometry_id','frame_id','model','clearance_convention',
                     'body_reference','body_length_m','body_width_m','pair_threshold_m','wall_threshold_m','sample_dt_s','requested_horizon_s'):
            if row.get(name) is None or row.get(name)!=first.get(name):errors.append('different_or_missing_comparison_context:'+name)
        if row.get('requested_horizon_s')!=c.get('common_requested_horizon_s') or row.get('sample_dt_s')!=c.get('sample_dt_s'):
            errors.append('policy_time_grid_mismatch')
    by_key={(r['frame_index'],r['option_index']):r for r in rows}
    if any(any(v['normalized_value'] is None for v in by_key[k]['components'].values()) for k in keys):
        errors.append('required_component_unavailable')
    return keys, sorted(set(errors))


def evaluate_costs(features, *, policy=None):
    """Return component diagnostics, and optionally explicit experimental scores.

    No policy means no normalized costs or weights. Policy scales are physical
    reference scales, never inferred from this batch's extrema. Scores require
    complete native coverage plus an explicit, numerically verified comparison
    group; no ranking or candidate selection is produced.
    """
    f=json_copy(features)
    if not flags(f) or f.get('method')!=METHOD or type(f.get('schema_version')) is not int or f['schema_version']!=1:
        raise ValueError('invalid_cost_feature_artifact')
    p=_policy(policy);rows=[];keys=set()
    if not isinstance(f.get('options'),list):raise ValueError('invalid_feature_options')
    for row in f['options']:
        key=(row.get('frame_index'),row.get('option_index'))
        if not flags(row) or any(type(v) is not int for v in key) or key in keys:
            raise ValueError('invalid_or_duplicate_feature_option')
        keys.add(key)
        components={name:_component(name,spec,row) for name,spec in p['components'].items()} if p else {}
        rows.append(dict(frame_index=key[0],option_index=key[1],candidate_id=row.get('candidate_id'),
            target_speed_mps=row.get('target_speed_mps'),comparison_status=row.get('comparison_status'),
            source_reasons=row.get('reasons'),components=components,
            raw_features=dict(geometry_length_m=row.get('geometry_length_m'),min_pair_clearance_m=row.get('min_pair_clearance_m'),
                              relative_s_progress_m=row.get('relative_s_progress_m')),
            experimental_total_cost=None,score_status='no_policy' if p is None else 'components_only',
            score_reasons=[],inspection_only=True,planning_allowed=False))
    errors=[]
    if p is not None and p.get('weights') is not None:
        group,errors=_comparison(rows,f['options'],p,f)
        for r in rows:
            if (r['frame_index'],r['option_index']) not in group:
                r['score_reasons']=['not_in_requested_comparison_group'];continue
            r['score_reasons']=errors[:]
            if not errors:
                r['experimental_total_cost']=sum(p['weights'][n]*v['normalized_value'] for n,v in r['components'].items())
                r['score_status']='experimental_dimensionless_score'
            else:r['score_status']='not_comparable'
    return json_copy(dict(schema_version=1,method='offline_cost_evaluation_v1',policy=p,options=rows,
        comparison_errors=errors,inspection_only=True,planning_allowed=False,
        limitations=['No ranking, selection, control or safety approval.',
                     'Trapezoidal exposure uses supplied discrete samples, not continuous swept contact.',
                     'Signed minimum and last-time relative progress are not full rollout integrals.',
                     'Accepted audit-time binding is not authenticated producer history.']))
