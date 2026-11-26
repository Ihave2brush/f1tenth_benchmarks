"""Portable anchored projection and synthetic sidecar tests; no simulator."""
from copy import deepcopy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from test_robust_adaptive_paths import make_straight
from test_frenet_tracking import circle
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.core.geometry.tracker import TrackingOptions
from f1tenth_benchmarks.research.overtaking import offline_rollout_frenet_v1 as module

OPTIONS=TrackingOptions(max_dt_s=.2,max_speed_mps=4.,max_acceleration_mps2=8.,position_slack_m=.05,progress_slack_m=.05)


def data(g,positions,times=None):
    samples=[]
    for i,s in enumerate(positions):
        ref=g.reference(s % g.curve.L if g.curve.closed else s)
        samples.append(dict(t_s=times[i] if times is not None else i*.05,pose=[*ref['xy'],ref['psi']]))
    sw=positions[0]%g.curve.L if g.curve.closed else positions[0]
    source=dict(pose=samples[0]['pose'],sim_time_s=10.,frenet=dict(valid=True,s_wrapped=sw,s_unwrapped=positions[0],geometry_id=g.geometry_id,frame_id='map'))
    return samples,source


def project(g,samples,source,**kw):
    return module.project_pose_series(samples,source,g,geometry_id=g.geometry_id,frame_id='map',tracking_options=OPTIONS,**kw)


def test_straight_analytic_and_immutability():
    g=make_straight();s,source=data(g,[.5,.6,.7]);before=deepcopy((s,source))
    r=project(g,s,source)
    assert r['projection_valid']
    np.testing.assert_allclose([q['s_unwrapped'] for q in r['samples']],[.5,.6,.7],atol=1e-7)
    assert (s,source)==before and json.loads(json.dumps(r,allow_nan=False))==r


@pytest.mark.parametrize('sites',[(.2,.3,.4),(-.2,-.1,0.,.1),(.1,0.,-.1,-.2)])
def test_curve_forward_and_both_seams(circle,sites):
    # Negative sites denote the previous lap, not an invented wrapped interval.
    s,source=data(circle,sites);r=project(circle,s,source)
    assert r['projection_valid'],r
    np.testing.assert_allclose([q['s_unwrapped'] for q in r['samples']],sites,atol=1e-5)


def test_nonzero_lap_anchor_and_reverse(circle):
    sites=[2*circle.curve.L+.3,2*circle.curve.L+.2,2*circle.curve.L+.1]
    s,source=data(circle,sites);r=project(circle,s,source)
    assert r['projection_valid'];np.testing.assert_allclose([q['s_unwrapped'] for q in r['samples']],sites,atol=1e-5)


@pytest.mark.parametrize('field',['s_wrapped','s_unwrapped','geometry_id','frame_id'])
def test_missing_anchor_context(field):
    g=make_straight();s,source=data(g,[.5,.6]);source['frenet'].pop(field)
    with pytest.raises(ValueError):project(g,s,source)


@pytest.mark.parametrize('change',['nan_anchor','source_time','source_pose','bad_anchor','wrong_geometry','wrong_frame','missing_options'])
def test_invalid_contracts(change):
    g=make_straight();s,source=data(g,[.5,.6]);kwargs=dict(geometry_id=g.geometry_id,frame_id='map',tracking_options=OPTIONS)
    if change=='nan_anchor':source['frenet']['s_unwrapped']=float('nan')
    if change=='source_time':source['sim_time_s']=None
    if change=='source_pose':source['pose']=[float('inf'),3.,0.]
    if change=='bad_anchor':source['frenet']['s_unwrapped']+=1.
    if change=='wrong_geometry':kwargs['geometry_id']='other'
    if change=='wrong_frame':kwargs['frame_id']='odom'
    if change=='missing_options':kwargs['tracking_options']=None
    with pytest.raises(ValueError):module.project_pose_series(s,source,g,**kwargs)


@pytest.mark.parametrize('change,reason',[
 ('gap','pose_gap'),('jump','localization_jump'),('repeat','nonmonotonic_sample_time'),
 ('no_t0','missing_t0'),('invalid_pose','invalid_sample_pose'),('heading','heading_mismatch'),('start_pose','sample_t0_source_pose_mismatch')])
def test_failures_preserved_and_no_recovery(change,reason):
    g=make_straight();s,source=data(g,[.5,.6,.7])
    if change=='gap':s[1]['t_s']=.3;s[2]['t_s']=.35
    if change=='jump':s[1]['pose'][0]+=2.
    if change=='repeat':s[1]['t_s']=0.
    if change=='no_t0':s[0]['t_s']=.01
    if change=='invalid_pose':s[1]['pose'][0]=float('nan')
    if change=='heading':s[1]['pose'][2]+=math.pi
    if change=='start_pose':s[0]['pose']=list(s[0]['pose']);s[0]['pose'][0]+=.01
    r=project(g,s,source)
    assert not r['projection_valid'] and r['reason']==reason
    assert r['samples'][-1]['s_unwrapped'] is None
    assert len(r['samples'])==3


def test_source_projection_disagrees_with_anchor():
    g=make_straight();s,source=data(g,[.5,.6]);source['frenet'].update(s_wrapped=.8,s_unwrapped=.8)
    r=project(g,s,source);assert r['reason']=='source_anchor_projection_mismatch'


class Ambiguous:
    def __init__(self,g,initial=False):self.g=g;self.initial=initial
    def __getattr__(self,name):return getattr(self.g,name)
    def to_frenet(self,*a,**kw):
        return dict(valid=False,reason='ambiguous_projection') if self.initial else self.g.to_frenet(*a,**kw)
    def projection_candidates(self,*a,**kw):
        d=self.g.projection_candidates(*a,**kw);d['candidates'].append(deepcopy(d['candidates'][0]));return d


@pytest.mark.parametrize('initial',[True,False])
def test_ambiguity_explicit_not_guessed(initial):
    g=make_straight();s,source=data(g,[.5,.6]);r=project(Ambiguous(g,initial),s,source)
    assert not r['projection_valid'] and 'ambiguous_projection' in r['reason']


def synthetic_projected(values,times):
    return dict(source_s_unwrapped_m=0.,samples=[dict(t_s=t,projection_valid=True,s_unwrapped=v) for t,v in zip(times,values)])


def test_nonuniform_times_trapezoid_analytic():
    times=[0.,.1,.3];ego=synthetic_projected([0.,.2,.6],times);lead=synthetic_projected([0.,.1,.3],times)
    r=module.relative_progress_features(ego,lead,requested_horizon_s=.3,native_complete=True)
    assert r['evaluation_complete'] and r['delta_s_relative_final_m']==pytest.approx(.3)
    assert r['progress_integral_m_s']==pytest.approx(-.045)


@pytest.mark.parametrize('cause',['native_partial','missing_endpoint','invalid_projection','no_samples'])
def test_partial_does_not_gain_full_integral(cause):
    times=[0.,.1,.3];ego=synthetic_projected([0.,.2,.6],times);lead=synthetic_projected([0.,.1,.3],times)
    if cause=='missing_endpoint':ego['samples'].pop();lead['samples'].pop()
    if cause=='invalid_projection':ego['samples'][-1]['projection_valid']=False
    if cause=='no_samples':ego['samples']=[];lead['samples']=[]
    r=module.relative_progress_features(ego,lead,requested_horizon_s=.3,native_complete=cause!='native_partial')
    assert not r['evaluation_complete'] and r['progress_integral_m_s'] is None and r['delta_s_relative_final_m'] is None


def test_mismatched_time_grid_rejected():
    e=synthetic_projected([0.,1.],[0.,.1]);l=synthetic_projected([0.,1.],[0.,.2])
    with pytest.raises(ValueError):module.relative_progress_features(e,l,requested_horizon_s=.2,native_complete=True)










@pytest.mark.parametrize('change',['backward_time','missing_t0','over_horizon','nonfinite_s'])
def test_integral_rejects_malformed_direct_input(change):
    e=synthetic_projected([0.,1.,2.],[0.,.1,.2]);l=deepcopy(e)
    if change=='backward_time':e['samples'][2]['t_s']=.05;l['samples'][2]['t_s']=.05
    if change=='missing_t0':e['samples'][0]['t_s']=.01;l['samples'][0]['t_s']=.01
    if change=='over_horizon':e['samples'][2]['t_s']=.4;l['samples'][2]['t_s']=.4
    if change=='nonfinite_s':e['samples'][2]['s_unwrapped']=float('inf')
    with pytest.raises(ValueError):module.relative_progress_features(e,l,requested_horizon_s=.2,native_complete=True)




def test_explicit_corelap_origin_shift_retains_both_s_conventions():
    g=make_straight();s,source=data(g,[.5,.6,.7])
    source['frenet'].update(origin_shift=10.,native_s_unwrapped=.5,s_unwrapped=10.5)
    result=project(g,s,source);assert result['projection_valid']
    np.testing.assert_allclose([v['s_unwrapped'] for v in result['samples']],[10.5,10.6,10.7])
    np.testing.assert_allclose([v['native_s_unwrapped'] for v in result['samples']],[.5,.6,.7])


@pytest.mark.parametrize('change',['missing_native','missing_shift','inconsistent_sum'])
def test_origin_must_be_explicit_and_consistent(change):
    g=make_straight();s,source=data(g,[.5,.6])
    source['frenet'].update(origin_shift=10.,native_s_unwrapped=.5,s_unwrapped=10.5)
    if change=='missing_native':source['frenet'].pop('native_s_unwrapped')
    if change=='missing_shift':source['frenet'].pop('origin_shift')
    if change=='inconsistent_sum':source['frenet']['origin_shift']=9.
    with pytest.raises(ValueError):project(g,s,source)


def synthetic_sidecar():
    """Versioned pose/schema math, never claimed as native ESP evidence."""
    return json.loads((Path(__file__).parent/'fixtures'/'offline_frenet_synthetic_v1.json').read_text())


def test_portable_sidecar_ids_speeds_reference_partial_and_native_status():
    g=make_straight();r=synthetic_sidecar();before=deepcopy(r)
    result=module.project_rollout_record(r,g,tracking_options=OPTIONS)
    assert r==before and json.loads(json.dumps(result,allow_nan=False))==result
    assert [o['candidate_id'] for o in result['options']]==[1,1,3,None,2]
    assert [o['target_speed_mps'] for o in result['options']]==[2.,1.5,1.5,1.,.5]
    for a,b in zip(r['options'],result['options']):
        for key in ('evaluation_id','prediction_group_id','native_safe','native_reason'):
            assert a[key]==b[key]
        assert b['native_relative_s_progress_m']==123.  # never used as projected progress
        assert b['inspection_only'] and not b['planning_allowed'] and not b['formal_comparison_qualified']
    assert result['options'][0]['progress']['progress_integral_m_s']==pytest.approx(-.02)
    assert result['options'][2]['reference_only'] and result['options'][2]['rollout_trajectory_type']=='modeled_ego_rollout'
    assert result['options'][3]['reference_only'] and result['options'][3]['reference_id']=='raceline'
    p=result['options'][4]['progress'];assert p['progress_integral_m_s'] is None
    assert p['checked_partial_progress_integral_m_s']==pytest.approx(.0025)


@pytest.mark.parametrize('change',['step','time','frame','geometry','hash','option_id','grid'])
def test_portable_sidecar_rejects_source_and_identity_changes(change):
    r=synthetic_sidecar();g=make_straight()
    if change=='step':r['source']['ego_state']['control_step']+=1
    elif change=='time':r['source']['source_sim_time_s']+=1
    elif change=='frame':r['metadata']['frame_id']='odom'
    elif change=='geometry':r['metadata']['geometry_id']='different'
    elif change=='hash':r['plan_snapshot']['source_pose'][0]+=.01
    elif change=='option_id':r['options'][1]['evaluation_id']=r['options'][0]['evaluation_id']
    else:r['options'][1]['requested_sample_times_s']=[0.,.1,.2]
    with pytest.raises(ValueError):module.project_rollout_record(r,g,tracking_options=OPTIONS)


def test_portable_multiple_prediction_groups():
    r=synthetic_sidecar();other=deepcopy(r['lead_predictions'][0]);other['prediction_group_id']='synthetic:g1'
    r['lead_predictions'].append(other);r['options'][1]['prediction_group_id']='synthetic:g1'
    result=module.project_rollout_record(r,make_straight(),tracking_options=OPTIONS)
    assert result['options'][1]['prediction_group_id']=='synthetic:g1'
    assert result['options'][1]['progress']['progress_integral_m_s']==pytest.approx(-.01)


def test_projection_never_calls_native_producers(monkeypatch):
    from f1tenth_benchmarks.research.overtaking import two_car_decision,two_car_env
    def forbidden(*args,**kwargs):raise AssertionError('native producer invoked')
    monkeypatch.setattr(two_car_decision.OvertakingSelector,'__call__',forbidden)
    monkeypatch.setattr(two_car_decision.OvertakingSelector,'_rollout',forbidden)
    monkeypatch.setattr(two_car_env.TwoCarOvertakingEnv,'step',forbidden)
    assert module.project_rollout_record(synthetic_sidecar(),make_straight(),tracking_options=OPTIONS)['options']
