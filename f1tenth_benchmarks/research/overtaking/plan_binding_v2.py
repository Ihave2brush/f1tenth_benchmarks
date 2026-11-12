"""V2 analytic content identity; no feasibility, sampling or authentication claim."""
import hashlib
import json
import math
from pathlib import Path

from .plan_binding_v1 import _tree, _integer, _finite, build_plan_binding as binding_v1

HEADER = dict(binding_schema='f1tenth_plan_binding', binding_version=2,
              hash_algorithm='sha256', canonicalization='typed-json-floathex-v1')
ORIGIN_RULE = 'spline.source_s_origin_if_key_present_else_candidate.reference_s_origin'


def content_digest(domain, value):
    tree = _tree(dict(HEADER, domain=domain, payload=value))
    return hashlib.sha256(json.dumps(tree, ensure_ascii=True, separators=(',', ':'),
                                     allow_nan=False).encode('utf-8')).hexdigest()


def _copy(value):
    _tree(value)
    return json.loads(json.dumps(value, allow_nan=False))


def geometry_evidence(manifest_path=None, expected_manifest_sha256=None):
    """Check local artifact bytes; only a caller-pinned manifest is 'verified'.

    Manifest-declared source_hashes are retained as claims, not map-file checks.
    A digest pin is a caller trust anchor, not identity authentication.
    """
    if manifest_path is None:
        if expected_manifest_sha256 is not None:
            raise ValueError('geometry_pin_without_manifest')
        return dict(status='unavailable', geometry_artifacts_verified=False,
                    map_sources_verified=False, manifest_sha256=None)
    path = Path(manifest_path)
    raw = path.read_bytes(); digest = hashlib.sha256(raw).hexdigest()
    manifest = json.loads(raw)
    _tree(manifest)
    if not isinstance(manifest, dict):
        raise ValueError('invalid_geometry_manifest')
    if expected_manifest_sha256 is not None and expected_manifest_sha256 != digest:
        raise ValueError('geometry_manifest_hash_mismatch')
    files = manifest.get('file_hashes')
    if not isinstance(files, dict) or 'reference_curve.json' not in files:
        raise ValueError('geometry_manifest_missing_curve_hash')
    checked = {}
    for name, expected in files.items():
        target = (path.parent/name).resolve()
        if path.parent.resolve() not in target.parents:
            raise ValueError('geometry_artifact_outside_manifest_directory')
        actual = hashlib.sha256(target.read_bytes()).hexdigest()
        if actual != expected:
            raise ValueError('geometry_artifact_hash_mismatch:' + name)
        checked[name] = actual
    return dict(status='verified_against_pin' if expected_manifest_sha256 is not None else 'checked_unpinned',
                geometry_artifacts_verified=expected_manifest_sha256 is not None,
                map_sources_verified=False, manifest_sha256=digest,
                geometry_id=manifest.get('geometry_id'), frame_id=manifest.get('frame_id'),
                map_id=manifest.get('map_id'), manifest_schema_version=manifest.get('schema_version'),
                file_hashes=checked, source_hashes_unverified=manifest.get('source_hashes'))


def build_plan_binding(plan, *, session_metadata=None, length_m=None,
                       geometry_manifest_path=None, expected_manifest_sha256=None):
    """Bind five ordered paths, raw analytic fields and effective origin/domain.

    Pure identity except optional explicit file verification. It never samples
    XY, changes a reference path's meaning, or writes a source artifact.
    """
    old = binding_v1(plan, session_metadata=session_metadata)
    evidence = geometry_evidence(geometry_manifest_path, expected_manifest_sha256)
    for key in ('geometry_id', 'frame_id'):
        if evidence.get(key) is not None and evidence[key] != old['source'][key]:
            raise ValueError('geometry_manifest_source_mismatch:' + key)
    settings = plan.get('generator_settings', {})
    if not isinstance(settings, dict):
        raise ValueError('invalid_generator_settings')
    recorded = settings.get('lookahead_m')
    if length_m is not None and recorded is not None and length_m != recorded:
        raise ValueError('lookahead_length_mismatch')
    length = recorded if length_m is None else length_m
    if length is not None and _finite(length, 'lookahead') <= 0:
        raise ValueError('invalid_lookahead')
    candidates = []
    for c in plan['paths']:
        spec = c.get('spline')
        analytic = None
        if spec is not None:
            if not isinstance(spec, dict) or length is None:
                raise ValueError('spline_requires_spec_and_length')
            degree = _integer(spec.get('degree'), 'spline_degree')
            knots, coefficients = spec.get('knots'), spec.get('coefficients')
            if not isinstance(knots, list) or not isinstance(coefficients, list):
                raise ValueError('invalid_spline_arrays')
            for v in knots+coefficients:
                _finite(v, 'spline_number')
            n = len(knots)-degree-1
            if n < degree+1 or len(coefficients) < n or any(a>b for a,b in zip(knots,knots[1:])):
                raise ValueError('invalid_spline_structure')
            low, high = knots[degree], knots[-degree-1]
            if not low < high or low > 0 or high < length:
                raise ValueError('incomplete_spline_domain')
            origin = spec['source_s_origin'] if 'source_s_origin' in spec else c.get('reference_s_origin')
            _finite(origin, 'resolved_s_origin')
            analytic = dict(spline=spec, reference_s_origin_present='reference_s_origin' in c,
                            reference_s_origin=c.get('reference_s_origin'), origin_rule=ORIGIN_RULE,
                            resolved_origin=origin, spline_domain_m=[low,high])
        elif c.get('output_representation') == 'analytic_spline_and_single_start_point' and c.get('sampled_valid') is True:
            raise ValueError('valid_analytic_candidate_missing_spline')
        candidates.append(dict(candidate_id=c['candidate_id'],
            polyline_candidate_sha256=old['candidate_sha256'][len(candidates)]['sha256'],
            output_representation_present='output_representation' in c,
            output_representation=c.get('output_representation'), analytic=analytic,
            path_type=c.get('path_type','state_matched_trajectory'),
            reference_only=c.get('path_type') == 'reference_centerline',
            reference_d=c.get('reference_d'), requested_target_d=c.get('requested_target_d'),
            footprint_sampled_valid=c.get('footprint_sampled_valid')))
    context = dict(source=old['source'], lookahead_length_m=length, geometry=evidence)
    payload = dict(context=context, polyline_plan_v1_sha256=old['plan_sha256'], candidates=candidates)
    digest = content_digest('ordered_plan_analytic_v2', payload)
    individual = [dict(candidate_id=c['candidate_id'], path_type=c['path_type'],
                       reference_only=c['reference_only'], output_representation=c['output_representation'],
                       resolved_s_origin=c['analytic']['resolved_origin'] if c['analytic'] else None,
                       origin_field=('spline.source_s_origin' if 'source_s_origin' in c['analytic']['spline'] else 'candidate.reference_s_origin') if c['analytic'] else None,
                       spline_domain_m=c['analytic']['spline_domain_m'] if c['analytic'] else None,
                       sha256=content_digest(
        'candidate_analytic_v2', dict(context=context,candidate=c))) for c in candidates]
    return _copy(dict(HEADER, plan_sha256=digest, polyline_plan_v1_sha256=old['plan_sha256'],
        candidate_sha256=individual, source=old['source'], lookahead_length_m=length,
        geometry_evidence=evidence, origin_rule=ORIGIN_RULE,
        geometry_artifacts_verified=evidence['geometry_artifacts_verified'],
        digest_scope='provided_plan_content_not_historical_evaluation_evidence',
        inspection_only=True, planning_allowed=False))


def verify_plan_binding(plan, binding, **kwargs):
    expected = build_plan_binding(plan, **kwargs)
    if _tree(binding) != _tree(expected):
        raise ValueError('plan_binding_v2_mismatch')
    return expected


def bind_derived_xy(binding, candidate_id, xy, *, sampling_step_m,
                    sampling_length_m, geometry_version):
    """Bind externally supplied XY and declared sampling, without claiming derivation.

    This function does not call a sampler; it proves neither that XY was sampled
    from this spline nor that a reference became a state-matched trajectory.
    """
    if not isinstance(binding, dict) or any(type(binding.get(k)) is not type(v) or binding[k]!=v for k,v in HEADER.items()):
        raise ValueError('derived_xy_requires_v2_binding')
    matches = [c for c in binding['candidate_sha256'] if type(candidate_id) is int and c['candidate_id']==candidate_id]
    if len(matches) != 1:
        raise ValueError('unknown_derived_candidate_id')
    for value in (sampling_step_m,sampling_length_m):
        if _finite(value,'sampling_parameter')<=0:
            raise ValueError('invalid_sampling_parameter')
    if binding.get('lookahead_length_m') is not None and sampling_length_m != binding['lookahead_length_m']:
        raise ValueError('derived_length_mismatch')
    if not isinstance(geometry_version,str) or not geometry_version:
        raise ValueError('missing_geometry_version')
    pinned = binding['geometry_evidence'].get('manifest_sha256')
    if pinned is not None and geometry_version != pinned:
        raise ValueError('derived_geometry_version_mismatch')
    if not isinstance(xy,list) or len(xy)<2 or any(not isinstance(p,(list,tuple)) or len(p)!=2 for p in xy):
        raise ValueError('invalid_derived_xy')
    for p in xy:
        for value in p: _finite(value,'derived_xy')
    payload = dict(analytic_plan_sha256=binding['plan_sha256'],
        analytic_candidate_sha256=matches[0]['sha256'],candidate_id=candidate_id,
        source=binding['source'],path_type=matches[0]['path_type'],
        reference_only=matches[0]['reference_only'],xy=xy,sampling_step_m=sampling_step_m,
        sampling_length_m=sampling_length_m,geometry_version=geometry_version)
    return _copy(dict(derived_xy_sha256=content_digest('externally_provided_derived_xy_v2',payload),
        **{k:v for k,v in payload.items() if k!='xy'}, derivation_verified=False,
        inspection_only=True, planning_allowed=False))
