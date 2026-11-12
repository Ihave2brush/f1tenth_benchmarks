"""V2 identity tests; artificial paths and artifact bytes, no optimizer."""
from copy import deepcopy
import hashlib
import json
import math

import pytest

from binding_contract_fixtures import inputs, analytic_input, artifact
from f1tenth_benchmarks.research.overtaking import plan_binding_v1 as v1
from f1tenth_benchmarks.research.overtaking import plan_binding_v2 as v2




def make_binding(p,metadata,**kw):return v2.build_plan_binding(p,session_metadata=metadata,**kw)




def test_reproducible_roundtrip_no_mutation():
    p,m=analytic_input();before=deepcopy((p,m));b=make_binding(p,m)
    assert b==make_binding(p,m)
    assert v2.verify_plan_binding(json.loads(json.dumps(p)),json.loads(json.dumps(b)),session_metadata=m)==b
    assert (p,m)==before and b['binding_version']==2
    assert b['inspection_only'] and not b['planning_allowed']
    assert not b['geometry_artifacts_verified']


def test_key_order_irrelevant_and_domain_separate_from_v1():
    p,m=analytic_input();b=make_binding(p,m)
    q=json.loads(json.dumps(p,sort_keys=True))
    assert b==make_binding(q,m)
    assert b['polyline_plan_v1_sha256']==v1.build_plan_binding(p,session_metadata=m)['plan_sha256']
    assert b['plan_sha256']!=b['polyline_plan_v1_sha256']


@pytest.mark.parametrize('field',['coefficients','knots','degree','source_s_origin','reference_s_origin',
    'output_representation','time','pose','geometry','frame','order','ids','lookahead','status','footprint','points'])
def test_key_changes_bound(field):
    p,m=analytic_input();b=make_binding(p,m);c=p['paths'][0]
    if field=='coefficients':c['spline']['coefficients'][-1]=math.nextafter(.1,math.inf)
    elif field=='knots':c['spline']['knots'][-4:]=[4.]*4
    elif field=='degree':c['spline'].update(degree=2,coefficients=[0.]*5)
    elif field=='source_s_origin':c['spline']['source_s_origin']=.2
    elif field=='reference_s_origin':c['reference_s_origin']=.2
    elif field=='output_representation':c[field]='other'
    elif field=='time':p['source_sim_time_s']=10.1
    elif field=='pose':p['source_pose'][0]=.1
    elif field=='geometry':p['geometry_id']=m['geometry_id']='other'
    elif field=='frame':m['frame_id']='odom'
    elif field=='order':p['paths'].reverse()
    elif field=='ids':p['paths'][0]['candidate_id'],p['paths'][1]['candidate_id']=2,1
    elif field=='lookahead':p['generator_settings']['lookahead_m']=2.9
    elif field=='status':p['status']='skipped'
    elif field=='footprint':c['footprint_sampled_valid']=False
    else:c['points'][0]['x']=.1
    assert make_binding(p,m)['plan_sha256']!=b['plan_sha256']
    with pytest.raises(ValueError):v2.verify_plan_binding(p,b,session_metadata=m)


def test_source_origin_precedence_but_raw_fallback_still_bound():
    p,m=analytic_input();p['paths'][0]['spline']['source_s_origin']=.1
    b=make_binding(p,m);p['paths'][0]['reference_s_origin']=.2
    assert b['plan_sha256']!=make_binding(p,m)['plan_sha256']
    # Present null must not fall back as the upstream sampler uses dict.get.
    p['paths'][0]['spline']['source_s_origin']=None
    with pytest.raises(ValueError):make_binding(p,m)


@pytest.mark.parametrize('field',['knots','coefficients','source_s_origin','reference_s_origin','time'])
@pytest.mark.parametrize('value',[float('nan'),float('inf')])
def test_nonfinite_rejected(field,value):
    p,m=analytic_input();c=p['paths'][0]
    if field in ('knots','coefficients'):c['spline'][field][0]=value
    elif field=='source_s_origin':c['spline'][field]=value
    elif field=='reference_s_origin':c[field]=value
    else:p['source_sim_time_s']=value
    with pytest.raises(ValueError):make_binding(p,m)


@pytest.mark.parametrize('change',['missing_origin','bad_domain','bad_degree','duplicate_ids','missing_length','length_disagreement','missing_spline','bad_geometry_frame'])
def test_invalid_schemas(change):
    p,m=analytic_input();c=p['paths'][0];kw={}
    if change=='missing_origin':del c['reference_s_origin']
    elif change=='bad_domain':c['spline']['knots'][-4:]=[2.]*4
    elif change=='bad_degree':c['spline']['degree']=True
    elif change=='duplicate_ids':p['paths'][1]['candidate_id']=1
    elif change=='missing_length':p.pop('generator_settings')
    elif change=='length_disagreement':kw['length_m']=2.
    elif change=='missing_spline':c.pop('spline')
    else:p['frame_id']='odom'
    with pytest.raises(ValueError):make_binding(p,m,**kw)


def test_geometry_evidence_pinned_unpinned_missing_and_no_false_map_verification(tmp_path):
    p,m=analytic_input();path,pin=artifact(tmp_path)
    unavailable=make_binding(p,m)
    unpinned=make_binding(p,m,geometry_manifest_path=path)
    verified=make_binding(p,m,geometry_manifest_path=path,expected_manifest_sha256=pin)
    assert not unavailable['geometry_artifacts_verified'] and not unpinned['geometry_artifacts_verified']
    assert verified['geometry_artifacts_verified'] and not verified['geometry_evidence']['map_sources_verified']
    assert len({b['plan_sha256'] for b in [unavailable,unpinned,verified]})==3
    with pytest.raises(ValueError):make_binding(p,m,expected_manifest_sha256=pin)
    with pytest.raises(ValueError):make_binding(p,m,geometry_manifest_path=path,expected_manifest_sha256='0'*64)


@pytest.mark.parametrize('change',['artifact','manifest_frame','outside'])
def test_geometry_evidence_mismatch(tmp_path,change):
    p,m=analytic_input();path,pin=artifact(tmp_path)
    manifest=json.loads(path.read_text())
    if change=='artifact':(path.parent/'reference_curve.json').write_text('changed')
    elif change=='manifest_frame':manifest['frame_id']='odom';path.write_text(json.dumps(manifest))
    else:manifest['file_hashes']['../outside']='0'*64;path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError):make_binding(p,m,geometry_manifest_path=path)


def test_v1_polyline_compatibility_without_analytic_domain():
    p,o=inputs();before=v1.build_plan_binding(p,session_metadata=o['session_metadata'])
    b=make_binding(p,o['session_metadata'])
    assert b['polyline_plan_v1_sha256']==before['plan_sha256']
    assert b['lookahead_length_m'] is None
    assert v1.verify_plan_binding(p,before,session_metadata=o['session_metadata'])==before
    with pytest.raises(ValueError):v1.verify_plan_binding(p,b,session_metadata=o['session_metadata'])


def test_candidate_hash_order_and_reference_semantics():
    p,m=analytic_input();p['paths'][2]['path_type']='reference_centerline'
    b=make_binding(p,m);p['paths'].reverse();r=make_binding(p,m)
    assert b['plan_sha256']!=r['plan_sha256']
    assert {c['candidate_id']:c['sha256'] for c in b['candidate_sha256']}=={c['candidate_id']:c['sha256'] for c in r['candidate_sha256']}
    assert b['candidate_sha256'][2]['reference_only']
    d=v2.bind_derived_xy(b,3,[[0.,0.],[3.,0.]],sampling_step_m=3.,sampling_length_m=3.,geometry_version='synthetic-v1')
    assert d['reference_only'] and d['path_type']=='reference_centerline' and not d['derivation_verified']
    assert d['derived_xy_sha256']!=b['plan_sha256']


@pytest.mark.parametrize('change',['xy','step','version'])
def test_derived_xy_and_sampling_are_distinct_identity(change):
    p,m=analytic_input();b=make_binding(p,m);xy=[[0.,0.],[3.,0.]]
    opts=dict(sampling_step_m=3.,sampling_length_m=3.,geometry_version='synthetic-v1')
    before=deepcopy((b,xy,opts));d=v2.bind_derived_xy(b,1,xy,**opts)
    assert (b,xy,opts)==before
    if change=='xy':xy[1][1]=.1
    elif change=='step':opts['sampling_step_m']=1.
    else:opts['geometry_version']='synthetic-v2'
    assert d['derived_xy_sha256']!=v2.bind_derived_xy(b,1,xy,**opts)['derived_xy_sha256']


def test_derived_geometry_pin_mismatch(tmp_path):
    p,m=analytic_input();path,pin=artifact(tmp_path);b=make_binding(p,m,geometry_manifest_path=path,expected_manifest_sha256=pin)
    with pytest.raises(ValueError):v2.bind_derived_xy(b,1,[[0.,0.],[3.,0.]],sampling_step_m=.02,sampling_length_m=3.,geometry_version='wrong')


def test_resolved_origin_recorded_without_rewriting_fallback():
    p,m=analytic_input();b=make_binding(p,m);c=b['candidate_sha256'][0]
    assert c['origin_field']=='candidate.reference_s_origin' and c['resolved_s_origin']==0.
    p['paths'][0]['spline']['source_s_origin']=.2
    c=make_binding(p,m)['candidate_sha256'][0]
    assert c['origin_field']=='spline.source_s_origin' and c['resolved_s_origin']==.2
    assert p['paths'][0]['reference_s_origin']==0.


def test_frozen_v2_vector_from_research_contract():
    p,m=analytic_input()
    assert make_binding(p,m)['plan_sha256']=='8401e1a20f9c46833dd3cb64d2c0a759d27000647bb188b39a7ba3a296b77016'


@pytest.mark.parametrize('degree',[3,5])
@pytest.mark.parametrize('extra',[-2,-1,0,1,2])
def test_exact_coefficient_count_required(degree,extra):
    p,m=analytic_input();s=p['paths'][0]['spline']
    s.update(degree=degree,knots=[0.]*(degree+1)+[3.]*(degree+1),coefficients=[0.]*(degree+1+extra))
    before=deepcopy(p)
    if extra:
        with pytest.raises(ValueError,match='invalid_spline_structure'):make_binding(p,m)
    else:
        assert make_binding(p,m)['binding_version']==2
    assert p==before
