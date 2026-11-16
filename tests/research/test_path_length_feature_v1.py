"""Synthetic arclength geometries and saved-format splines; no optimizer."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.special import fresnel

from f1tenth_benchmarks.research.overtaking import path_length_feature_v1 as module
from f1tenth_benchmarks.research.overtaking.plan_binding_v2 import build_plan_binding


class SyntheticCurve:
    def __init__(self,kind='straight',radius=5.):
        self.kind=kind;self.radius=radius;self.closed=kind=='circle'
        self.L=2*np.pi*radius if self.closed else 20.
    def sample(self,s):
        s=np.atleast_1d(s).astype(float)
        if not self.closed and (np.any(s<0) or np.any(s>self.L)):raise ValueError('outside_reference')
        if self.kind=='straight':
            xy=np.column_stack([s,np.zeros_like(s)]);theta=np.zeros_like(s);k=np.zeros_like(s)
        elif self.kind=='circle':
            theta=s/self.radius;k=np.full_like(s,1/self.radius)
            xy=self.radius*np.column_stack([np.sin(theta),1-np.cos(theta)])
        else:
            a=.03;z=s*np.sqrt(2*a/np.pi);fs,fc=fresnel(z)
            xy=np.sqrt(np.pi/(2*a))*np.column_stack([fc,fs]);theta=a*s*s;k=2*a*s
        tangent=np.column_stack([np.cos(theta),np.sin(theta)])
        return xy,tangent,np.arctan2(tangent[:,1],tangent[:,0]),k


class SyntheticGeometry:
    geometry_id='length-synthetic';frame_id='map'
    def __init__(self,kind='straight'):self.curve=SyntheticCurve(kind)


def case(kind='straight',amplitude=0.,offset=0.,origin=0.,length=3.,reference=False):
    geometry=SyntheticGeometry(kind)
    x,t,yaw,_=geometry.curve.sample([origin]);first=x[0]+offset*np.array([-t[0,1],t[0,0]])
    pose=[float(first[0]),float(first[1]),float(yaw[0])]
    points=[dict(x=pose[0],y=pose[1],yaw=pose[2],valid=True)]
    paths=[]
    for i in range(1,6):
        coefficients=[offset,offset,offset+amplitude,offset+amplitude,offset,offset]
        c=dict(candidate_id=i,path_type='state_matched_trajectory',geometry_id=geometry.geometry_id,
            sampled_valid=True,reason=None,points=deepcopy(points),reference_s_origin=origin,
            output_representation='analytic_spline_and_single_start_point',
            spline=dict(knots=[0.]*6+[length]*6,coefficients=coefficients,degree=5))
        if reference and i==3:
            c.update(path_type='reference_centerline',reference_d=0.,requested_target_d=0.)
            c['spline']['coefficients']=[0.]*6
            # Genuine reference first point may differ from the physical source.
            c['points']=[dict(x=float(x[0,0]),y=float(x[0,1]),yaw=float(yaw[0]),valid=True)]
        paths.append(c)
    p=dict(plan_id=1,status='completed',source_control_step=0,source_sim_time_s=0.,source_pose=pose,
           geometry_id=geometry.geometry_id,generator_settings=dict(lookahead_m=length),paths=paths)
    return p,geometry


def evaluate(p,g,**kw):return module.evaluate_path_length_features(p,g,**kw)


def test_straight_exact_and_json_and_no_mutation():
    p,g=case();before=deepcopy(p);g_before=deepcopy(vars(g.curve))
    r=evaluate(p,g)
    assert r['batch_status']=='evaluated' and p==before and vars(g.curve)==g_before
    assert [c['candidate_id'] for c in r['paths']]==[1,2,3,4,5]
    for c in r['paths']:
        assert c['length_status']=='computed' and c['frenet_integral_length_m']==pytest.approx(3.,abs=1e-12)
        assert all(x['xy_length_m']==pytest.approx(3.,abs=1e-12) for x in c['xy_lengths'])
        assert c['eligible_for_control'] is False
    assert r['comparison']['common_cartesian_end_among_computed']
    assert r['inspection_only'] and not r['planning_allowed']
    json.dumps(r,allow_nan=False)


def test_same_endpoints_detour_longer_and_density_converges():
    p,g=case(amplitude=.6);c=evaluate(p,g)['paths'][0]
    assert c['start_xy']==pytest.approx([0.,0.]) and c['end_xy']==pytest.approx([3.,0.])
    assert c['frenet_integral_length_m']>3.
    errors=[v['absolute_difference_from_frenet_m'] for v in c['xy_lengths']]
    assert errors[0]>errors[1]>errors[2]
    assert errors[-1]<1e-5
    assert c['quadrature_difference_estimate_m']<1e-10


@pytest.mark.parametrize('kind,offset,expected',[('straight',.5,3.),('circle',0.,3.),('circle',.5,2.7),('circle',-.5,3.3)])
def test_center_and_offset_analytic_lengths(kind,offset,expected):
    p,g=case(kind,offset=offset);c=evaluate(p,g)['paths'][0]
    assert c['frenet_integral_length_m']==pytest.approx(expected,abs=1e-10)
    assert c['xy_lengths'][-1]['xy_length_m']==pytest.approx(expected,abs=1e-6)


def test_correct_variable_curvature_origin_changes_length():
    for origin,expected in [(0.,2.946),(5.,2.766)]:
        p,g=case('variable',offset=.2,origin=origin);c=evaluate(p,g,sampling_steps_m=(.05,.02,.01,.001))['paths'][0]
        assert c['frenet_integral_length_m']==pytest.approx(expected,abs=1e-10)
        assert c['origin_s_m']==origin
        assert c['xy_lengths'][-1]['absolute_difference_from_frenet_m']<1e-7
        assert c['xy_lengths'][-1]['absolute_difference_from_frenet_m']<c['xy_lengths'][-2]['absolute_difference_from_frenet_m']


def test_closed_seam_unrolled_without_spline_wrap():
    p,g=case('circle');origin=g.curve.L-.5;p,g=case('circle',origin=origin)
    c=evaluate(p,g)['paths'][0]
    assert c['frenet_integral_length_m']==pytest.approx(3.)
    assert c['reference_interval_s_m'][1]>g.curve.L
    assert c['xy_lengths'][-1]['absolute_difference_from_frenet_m']<1e-6


def test_reference_length_is_not_ego_trajectory():
    p,g=case(offset=.1,reference=True);r=evaluate(p,g);c=r['paths'][2]
    assert c['length_status']=='computed' and c['reference_only']
    assert c['geometry_scope']=='reference_geometry_only'
    assert c['start_position_error_m']==pytest.approx(.1)
    assert c['frenet_integral_length_m']==pytest.approx(3.) and not c['eligible_for_control']
    assert not r['comparison']['common_cartesian_start_among_computed']


@pytest.mark.parametrize('change',['geometry','frame','point_after_binding','spline_after_binding','version'])
def test_binding_or_geometry_mismatch_rejected(change):
    p,g=case();b=build_plan_binding(p,session_metadata=dict(geometry_id=g.geometry_id,frame_id='map'))
    if change=='geometry':g.geometry_id='other'
    elif change=='frame':g.frame_id='odom'
    elif change=='point_after_binding':p['paths'][0]['points'][0]['x']=.1
    elif change=='spline_after_binding':p['paths'][0]['spline']['coefficients'][2]=.1
    else:b['binding_version']=1
    r=evaluate(p,g,binding=b)
    assert r['batch_status']=='rejected' and r['reason']
    assert all(c['frenet_integral_length_m'] is None for c in r['paths'])
    json.dumps(r,allow_nan=False)


@pytest.mark.parametrize('change',['missing_origin','nan_coefficient','invalid_knots','missing_spline',
    'missing_length','nonfinite_pose','duplicate_id','bad_candidate_id'])
def test_bad_plan_missing_and_nonfinite_keep_null(change):
    p,g=case();c=p['paths'][0]
    if change=='missing_origin':c.pop('reference_s_origin')
    elif change=='nan_coefficient':c['spline']['coefficients'][2]=float('nan')
    elif change=='invalid_knots':c['spline']['knots'][-6:]=[2.]*6
    elif change=='missing_spline':c.pop('spline')
    elif change=='missing_length':p.pop('generator_settings')
    elif change=='nonfinite_pose':p['source_pose'][0]=float('inf')
    elif change=='duplicate_id':p['paths'][1]['candidate_id']=1
    else:c['candidate_id']=float('nan')
    r=evaluate(p,g)
    assert r['batch_status']=='rejected' and all(c['frenet_integral_length_m'] is None for c in r['paths'])
    json.dumps(r,allow_nan=False)


@pytest.mark.parametrize('steps',[(0.,),(-.1,),(True,),(float('nan'),),(),(.01,.01),(1e-300,)])
def test_bad_sampling_settings_rejected(steps):
    p,g=case();r=evaluate(p,g,sampling_steps_m=steps)
    assert r['batch_status']=='rejected'


@pytest.mark.parametrize('change',['open_end','source_position','source_heading','fold','reference_invalid','not_native','unsupported_type'])
def test_individual_diagnostic_rejection(change):
    p,g=case()
    if change=='open_end':
        for c in p['paths']:c['reference_s_origin']=19.
    elif change=='source_position':p['source_pose'][1]=.2
    elif change=='source_heading':p['source_pose'][2]=.1
    elif change=='fold':
        p,g=case('circle',offset=5.)
    elif change=='reference_invalid':p['paths'][2]['path_type']='reference_centerline'
    elif change=='not_native':p['paths'][0].update(sampled_valid=False,reason='fit_not_accepted')
    else:p['paths'][0]['path_type']='other'
    r=evaluate(p,g);index=2 if change=='reference_invalid' else 0;c=r['paths'][index]
    assert c['length_status'] in ('rejected','native_rejected') and c['frenet_integral_length_m'] is None
    assert c['reason']


def test_different_endpoints_flagged_without_rank_or_selection():
    p,g=case();p['paths'][0]['spline']['coefficients'][-1]=.2
    r=evaluate(p,g)
    assert r['comparison']['common_frenet_interval_among_computed']
    assert not r['comparison']['common_cartesian_end_among_computed']
    assert 'selected_candidate_id' not in json.dumps(r) and 'score' not in r


def test_no_generation_prediction_collision_or_controller(monkeypatch):
    from f1tenth_benchmarks.research.overtaking import session,robust_adaptive_paths
    def forbidden(*a,**kw):pytest.fail('geometric feature called planner/evaluation')
    for m,name in [(session,'make_plan'),(robust_adaptive_paths,'generate_robust_paths')]:
        monkeypatch.setattr(m,name,forbidden)
    # These modules are intentionally outside the standalone Cost PR.
    import builtins
    original_import=builtins.__import__
    def guarded_import(name,globals=None,locals=None,fromlist=(),level=0):
        forbidden={'prediction_v1','collision_v1','prediction_collision_adapter_v1'}
        if name.rsplit('.',1)[-1] in forbidden or forbidden.intersection(fromlist or ()):
            raise AssertionError('cost core must not import prediction/collision')
        return original_import(name,globals,locals,fromlist,level)
    monkeypatch.setattr(builtins,'__import__',guarded_import)
    p,g=case();assert evaluate(p,g)['batch_status']=='evaluated'
    source=Path(module.__file__).read_text()
    assert 'two_car_decision' not in source and 'update_pose' not in source


def test_scalar_and_batched_sampling_orders_agree():
    p,g=case(amplitude=.6)
    a=evaluate(p,g,sampling_steps_m=(.05,.02,.01));b=evaluate(p,g,sampling_steps_m=(.01,))
    assert a['paths'][0]['xy_lengths'][-1]['xy_length_m']==b['paths'][0]['xy_lengths'][0]['xy_length_m']


def test_bound_metadata_record_verified_without_file_io():
    p,g=case();b=build_plan_binding(p,session_metadata=dict(geometry_id=g.geometry_id,frame_id='map'))
    before=deepcopy(b);r=evaluate(p,g,binding=b)
    assert r['binding_status']=='content_verified_against_supplied_record' and b==before
    assert not r['geometry_object_curve_verified'] and not r['geometry_artifacts_reverified']


def test_serialized_geometry_matches_bound_artifact_and_loaded_object(tmp_path):
    from test_robust_adaptive_paths import make_straight
    g=make_straight();source=g.reference(.5);length=.5
    p,_=case(length=length,origin=.5)
    p['geometry_id']=g.geometry_id;p['source_pose']=[*source['xy'],source['psi']]
    for c in p['paths']:
        c['geometry_id']=g.geometry_id;c['points']=[dict(x=source['xy'][0],y=source['xy'][1],yaw=source['psi'],valid=True)]
    raw=json.dumps(g.curve.to_dict(),indent=2).encode()
    (tmp_path/'reference_curve.json').write_bytes(raw)
    manifest=dict(geometry_id=g.geometry_id,frame_id='map',schema_version=1,
        file_hashes={'reference_curve.json':hashlib.sha256(raw).hexdigest()})
    mp=tmp_path/'manifest.json';mp.write_text(json.dumps(manifest));pin=hashlib.sha256(mp.read_bytes()).hexdigest()
    b=build_plan_binding(p,session_metadata=dict(geometry_id=g.geometry_id,frame_id='map'),geometry_manifest_path=mp,expected_manifest_sha256=pin)
    r=evaluate(p,g,binding=b,geometry_curve_json=raw)
    assert r['batch_status']=='evaluated' and r['geometry_object_curve_verified']
    assert r['source_artifact_evidence_claimed'] and not r['geometry_artifacts_reverified']
    assert r['paths'][0]['frenet_integral_length_m']==pytest.approx(length,abs=1e-9)
    g.curve.start_u=.1
    assert evaluate(p,g,binding=b,geometry_curve_json=raw)['reason']=='loaded_geometry_curve_mismatch'
    assert evaluate(p,g,binding=b,geometry_curve_json=raw+b' ')['reason']=='geometry_curve_artifact_hash_mismatch'


def test_pinned_binding_replayed_without_source_file_access(tmp_path,monkeypatch):
    from test_plan_binding_v2 import artifact
    p,g=case();p['geometry_id']='g';g.geometry_id='g'
    for c in p['paths']:c['geometry_id']='g'
    path,pin=artifact(tmp_path)
    b=build_plan_binding(p,session_metadata=dict(geometry_id='g',frame_id='map'),geometry_manifest_path=path,expected_manifest_sha256=pin)
    def forbidden(*a,**kw):pytest.fail('Path length pure function read a source file')
    monkeypatch.setattr(Path,'read_bytes',forbidden)
    r=evaluate(p,g,binding=b)
    assert r['batch_status']=='evaluated' and r['source_artifact_evidence_claimed']
    assert not r['geometry_object_curve_verified'] and not r['geometry_artifacts_reverified']


def test_source_origin_precedence_and_domain_rejection():
    p,g=case('variable',offset=.2,origin=5.)
    for c in p['paths']:c['spline']['source_s_origin']=5.;c['reference_s_origin']=0.
    c=evaluate(p,g)['paths'][0]
    assert c['origin_s_m']==5. and c['frenet_integral_length_m']==pytest.approx(2.766)
    for path in p['paths']:path['spline']['source_s_origin']=None
    assert evaluate(p,g)['batch_status']=='rejected'


def test_discontinuous_analytic_spline_is_not_shortened_by_integral():
    p,g=case()
    p['paths'][0]['spline']=dict(degree=1,knots=[0.,0.,1.5,1.5,3.,3.],coefficients=[0.,0.,.2,.2])
    c=evaluate(p,g)['paths'][0]
    assert c['reason']=='discontinuous_spline_not_supported' and c['frenet_integral_length_m'] is None


def test_esp_geometry_with_synthetic_spline_not_a_two_car_recording():
    from f1tenth_benchmarks.research.core.geometry import load_geometry
    g=load_geometry('maps/frenet/esp/geometry','maps/esp.yaml',inspect=True)
    p,_=case(reference=True);ref=g.reference(0.)
    p['geometry_id']=g.geometry_id;p['source_pose']=[*ref['xy'],ref['psi']]
    for c in p['paths']:
        c['geometry_id']=g.geometry_id;c['points']=[dict(x=ref['xy'][0],y=ref['xy'][1],yaw=ref['psi'],valid=True)]
    meta=dict(geometry_id=g.geometry_id,frame_id='map')
    manifest=Path('maps/frenet/esp/geometry/manifest.json')
    pin=hashlib.sha256(manifest.read_bytes()).hexdigest()
    b=build_plan_binding(p,session_metadata=meta,geometry_manifest_path=manifest,expected_manifest_sha256=pin)
    r=evaluate(p,g,binding=b,geometry_curve_json=Path('maps/frenet/esp/geometry/reference_curve.json').read_bytes(),
               sampling_steps_m=(.05,.01))
    assert r['batch_status']=='evaluated' and r['geometry_object_curve_verified']
    assert r['paths'][0]['frenet_integral_length_m']==pytest.approx(3.,abs=1e-10)
    assert r['paths'][0]['xy_lengths'][-1]['absolute_difference_from_frenet_m']<1e-5


def test_numerical_settings_are_recorded_for_reproducibility():
    p,g=case();r=evaluate(p,g,sampling_steps_m=(.1,.01),integration_panel_m=.1)
    assert r['parameters']['sampling_steps_m']==[.1,.01]
    assert r['parameters']['integration_panel_m']==.1
    assert r['parameters']['quadrature_orders']==[16,32]
