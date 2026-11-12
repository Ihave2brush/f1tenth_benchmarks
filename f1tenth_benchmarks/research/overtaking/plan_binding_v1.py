"""Versioned plan-content equality for offline inspection, not authentication."""
import hashlib
import json
import math
from numbers import Integral, Real

SCHEMA = 'f1tenth_plan_binding'
VERSION = 1
CANONICALIZATION = 'typed-json-floathex-v1'
HEADER = dict(binding_schema=SCHEMA, binding_version=VERSION,
              hash_algorithm='sha256', canonicalization=CANONICALIZATION)


def _tree(value):
    """Typed tree: sorted string keys, ordered arrays, exact binary64 floats.

    Lists/tuples share JSON-array semantics; integer and float types differ.
    All nodes are tagged, so a literal list cannot impersonate a float tag.
    No rounding, nonfinite substitution, or arbitrary-object stringification.
    """
    if value is None:
        return ['null']
    if isinstance(value, bool):
        return ['bool', value]
    if isinstance(value, str):
        return ['str', value]
    if isinstance(value, Integral):
        return ['int', str(int(value))]
    if isinstance(value, Real):
        if not math.isfinite(value):
            raise ValueError('binding_nonfinite_number')
        number = float(value)
        if number != value:
            raise ValueError('binding_number_not_lossless_binary64')
        return ['float', number.hex()]
    if isinstance(value, (list, tuple)):
        return ['array', [_tree(v) for v in value]]
    if isinstance(value, dict):
        if not all(isinstance(k, str) for k in value):
            raise ValueError('binding_nonstring_key')
        return ['object', [[k, _tree(value[k])] for k in sorted(value)]]
    raise ValueError('binding_unsupported_value')


def _digest(domain, payload):
    envelope = dict(HEADER, domain=domain, payload=payload)
    encoded = json.dumps(_tree(envelope), ensure_ascii=True, separators=(',', ':'), allow_nan=False)
    return hashlib.sha256(encoded.encode('utf-8')).hexdigest()


def _integer(value, name):
    if isinstance(value, bool) or not isinstance(value, Integral) or value < 0:
        raise ValueError('binding_invalid_' + name)
    return int(value)


def _finite(value, name):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError('binding_invalid_' + name)
    return value


def build_plan_binding(plan, *, session_metadata=None):
    """Hash source + five ordered candidates, excluding unrelated diagnostics.

    Frame must be explicit in plan or session metadata. Candidate points are
    bound in full (including their order, validity, XY, yaw, s/d/t if present);
    no point parameter is interpreted as seconds. Missing points differ from
    an empty array. Candidate overrides and explicit/default path type origin
    are bound. Candidate hashes include source and are domain-separated from
    the ordered batch hash. Geometry/finite checks here establish identity,
    never geometric feasibility or source-state validity.
    """
    if not isinstance(plan, dict):
        raise ValueError('binding_invalid_plan')
    if session_metadata is not None and not isinstance(session_metadata, dict):
        raise ValueError('binding_invalid_metadata')
    metadata = session_metadata or {}
    geometry = plan.get('geometry_id')
    frame = plan.get('frame_id', metadata.get('frame_id'))
    if not all(isinstance(v, str) and v.strip() for v in (geometry, frame)):
        raise ValueError('binding_missing_geometry_or_frame')
    for key, expected in [('geometry_id', geometry), ('frame_id', frame)]:
        if key in metadata and metadata[key] != expected:
            raise ValueError('binding_metadata_mismatch:' + key)
    source = dict(plan_id=_integer(plan.get('plan_id'), 'plan_id'),
                  source_control_step=_integer(plan.get('source_control_step'), 'control_step'),
                  source_sim_time_s=_finite(plan.get('source_sim_time_s'), 'source_time'),
                  geometry_id=geometry, frame_id=frame)
    pose = plan.get('source_pose')
    if not isinstance(pose, (list, tuple)) or len(pose) != 3:
        raise ValueError('binding_invalid_pose')
    source['source_pose'] = [_finite(v, 'pose') for v in pose]
    paths = plan.get('paths')
    if not isinstance(paths, list) or len(paths) != 5 or not all(isinstance(c, dict) for c in paths):
        raise ValueError('binding_invalid_five_paths')
    ids = [_integer(c.get('candidate_id'), 'candidate_id') for c in paths]
    if set(ids) != {1, 2, 3, 4, 5}:
        raise ValueError('binding_invalid_candidate_ids')
    candidates = []
    for c in paths:
        candidates.append(dict(candidate_id=c['candidate_id'],
            path_type=c.get('path_type', 'state_matched_trajectory'),
            path_type_origin='explicit' if 'path_type' in c else 'legacy_default',
            sampled_valid=c.get('sampled_valid'), reason=c.get('reason'),
            geometry_id=c.get('geometry_id', geometry), frame_id=c.get('frame_id', frame),
            points_present='points' in c, points=c.get('points')))
    # Detach the returned source via strict JSON serialization after validation.
    payload = dict(source=source, plan_status=plan.get('status'), candidates=candidates)
    digest = _digest('ordered_plan', payload)
    individual = [dict(candidate_id=c['candidate_id'], sha256=_digest(
        'candidate_with_source', dict(source=source, plan_status=plan.get('status'), candidate=c)))
        for c in candidates]
    source_json = json.loads(json.dumps(source, default=lambda v: int(v) if isinstance(v, Integral) else float(v), allow_nan=False))
    return dict(HEADER, plan_sha256=digest, candidate_sha256=individual, source=source_json,
                inspection_only=True, planning_allowed=False)


def verify_plan_binding(plan, binding, *, session_metadata=None):
    """Return recomputed binding on exact match; never upgrade missing evidence."""
    if not isinstance(binding, dict):
        raise ValueError('missing_or_invalid_plan_binding')
    if any(type(binding.get(k)) is not type(v) or binding[k] != v for k, v in HEADER.items()):
        raise ValueError('unsupported_plan_binding_schema')
    expected = build_plan_binding(plan, session_metadata=session_metadata)
    if _tree(binding) != _tree(expected):
        raise ValueError('plan_binding_mismatch')
    return expected
