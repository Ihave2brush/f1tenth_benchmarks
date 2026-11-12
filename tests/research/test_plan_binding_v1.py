"""Standalone V1 canonical regression, independent of prediction/collision."""
from copy import deepcopy
import json
import math
import pytest
from binding_contract_fixtures import inputs
from f1tenth_benchmarks.research.overtaking.plan_binding_v1 import build_plan_binding, verify_plan_binding

def binding(plan,options):
    return build_plan_binding(plan,session_metadata=options['session_metadata'])


def test_deterministic_detached_and_json_roundtrip():
    p, o = inputs(); before = deepcopy((p, o))
    b = binding(p, o)
    assert b == binding(p, o)
    assert verify_plan_binding(json.loads(json.dumps(p)), json.loads(json.dumps(b)),
                               session_metadata=o['session_metadata']) == b
    assert len(b['plan_sha256']) == 64
    assert b['inspection_only'] is True and b['planning_allowed'] is False
    assert (p, o) == before
    b['source']['source_pose'][0] = 77.
    assert p['source_pose'][0] == 0.


def test_dict_order_does_not_matter():
    def reverse(v):
        if isinstance(v, dict): return {k: reverse(v[k]) for k in reversed(v)}
        if isinstance(v, list): return [reverse(x) for x in v]
        return v
    p, o = inputs()
    assert binding(p, o) == binding(reverse(p), o)


@pytest.mark.parametrize('change', ['x','y','yaw','s','d','t','valid','point_reason','point_order',
                                    'candidate_ids','candidate_order','time','pose','geometry','frame',
                                    'path_type','native','reason','plan_id','step','status'])
def test_semantic_changes_alter_batch_digest(change):
    p, o = inputs(); original = binding(p, o); c = p['paths'][0]; pt = c['points'][1]
    if change in ('x','y','yaw'): pt[change] = math.nextafter(pt[change], math.inf)
    elif change in ('s','d','t'): pt[change] = .25
    elif change == 'valid': pt['valid'] = False
    elif change == 'point_reason': pt['reason'] = 'occupied'
    elif change == 'point_order': c['points'].reverse()
    elif change == 'candidate_ids': p['paths'][0]['candidate_id'],p['paths'][1]['candidate_id'] = 2,1
    elif change == 'candidate_order': p['paths'].reverse()
    elif change == 'time': p['source_sim_time_s'] = math.nextafter(10., math.inf)
    elif change == 'pose': p['source_pose'][0] = .001
    elif change == 'geometry': p['geometry_id'] = o['session_metadata']['geometry_id'] = 'other'
    elif change == 'frame': o['session_metadata']['frame_id'] = 'odom'
    elif change == 'path_type': c['path_type'] = 'reference_centerline'
    elif change == 'native': c['sampled_valid'] = False
    elif change == 'reason': c['reason'] = 'fit_not_accepted'
    elif change == 'plan_id': p['plan_id'] = 8
    elif change == 'step': p['source_control_step'] = 5
    else: p['status'] = 'skipped'
    assert binding(p, o)['plan_sha256'] != original['plan_sha256']


def test_domains_and_candidate_scope():
    p, o = inputs(); b = binding(p, o)
    assert all(c['sha256'] != b['plan_sha256'] for c in b['candidate_sha256'])
    p['paths'].reverse(); reversed_binding = binding(p, o)
    assert b['plan_sha256'] != reversed_binding['plan_sha256']
    assert {c['candidate_id']:c['sha256'] for c in b['candidate_sha256']} == {
        c['candidate_id']:c['sha256'] for c in reversed_binding['candidate_sha256']}
    p['paths'][0]['points'][1]['x'] = 11.
    modified = binding(p, o)
    assert modified['candidate_sha256'][0] != reversed_binding['candidate_sha256'][0]
    assert modified['candidate_sha256'][1:] == reversed_binding['candidate_sha256'][1:]
    bad = deepcopy(b); bad['plan_sha256'] = b['candidate_sha256'][0]['sha256']
    with pytest.raises(ValueError, match='plan_binding_mismatch'):
        verify_plan_binding(p, bad, session_metadata=o['session_metadata'])


@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
@pytest.mark.parametrize('where', ['point','pose','time'])
def test_nonfinite_is_rejected_not_normalized(value, where):
    p, o = inputs()
    if where == 'point': p['paths'][0]['points'][1]['x'] = value
    elif where == 'pose': p['source_pose'][0] = value
    else: p['source_sim_time_s'] = value
    with pytest.raises(ValueError): binding(p, o)



def test_exact_float_encoding_and_numeric_types():
    p, o = inputs(); b = binding(p, o)
    p['paths'][0]['points'][0]['x'] = -0.0
    assert binding(p, o)['plan_sha256'] != b['plan_sha256']
    p['paths'][0]['points'][0]['x'] = 0
    assert binding(p, o)['plan_sha256'] != b['plan_sha256']


@pytest.mark.parametrize('change',['missing_frame','bad_keys','object','missing_id','duplicate_id'])
def test_unavailable_identity_or_unserializable_content(change):
    p, o = inputs()
    if change == 'missing_frame': del o['session_metadata']['frame_id']
    elif change == 'bad_keys': p['paths'][0]['points'][0][3] = 2
    elif change == 'object': p['paths'][0]['points'][0]['x'] = object()
    elif change == 'missing_id': del p['plan_id']
    else: p['paths'][0]['candidate_id'] = 2
    with pytest.raises(ValueError): binding(p, o)


def test_unrelated_diagnostics_and_available_time_are_not_bound():
    p, o = inputs(); b = binding(p, o)
    p.update(wall_time_s=999., available_sim_time_s=1000., result={'diagnostic':'changed'})
    p['paths'][0]['optimizer_status'] = 'diagnostic only'
    assert binding(p, o) == b


@pytest.mark.parametrize('change',['candidate_boolean','source_float'])
def test_binding_record_uses_strict_types(change):
    p, o = inputs(); b = binding(p, o)
    if change == 'candidate_boolean': b['candidate_sha256'][0]['candidate_id'] = True
    else: b['source']['plan_id'] = float(b['source']['plan_id'])
    with pytest.raises(ValueError, match='plan_binding_mismatch'):
        verify_plan_binding(p,b,session_metadata=o['session_metadata'])


def test_frame_must_be_explicit_and_match_metadata():
    p, o = inputs(); p['frame_id'] = 'map'
    assert build_plan_binding(p) == binding(p,o)
    o['session_metadata']['frame_id'] = 'odom'
    with pytest.raises(ValueError, match='binding_metadata_mismatch:frame_id'):
        binding(p,o)


def test_frozen_v1_vector_from_research_contract():
    p,o=inputs()
    assert binding(p,o)['plan_sha256']=='8cde9d59317849c4283a18d2a3c7065fd65ce969a27768b0216906d6c453dc42'
