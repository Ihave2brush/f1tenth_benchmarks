"""Exact math, conservative gates, immutable inputs and explicit cache invalidation."""
from copy import deepcopy
import json
import pytest
from runtime_cost_test_support_v1 import case
from test_frenet_tracking import circle
from test_offline_rollout_frenet_v1 import data
from f1tenth_benchmarks.research.overtaking import realtime_cost_core_v1 as core
from f1tenth_benchmarks.research.overtaking.realtime_cost_inputs_v1 import prepare_runtime_length_cache


def ready():
    r,c=case()
    for name in ('ego_state','lead_state'): r['source'][name]['valid']=True
    lengths=prepare_runtime_length_cache(r['plan_snapshot'],c.geometry_id,{1:3.},method='cached_xy_polyline')
    projection=core.prepare_projection_bundle(r,c)
    return r,c,lengths,projection


def run(r,c,l=None,p=None):
    return core.evaluate_runtime_cost_core(r,c,length_cache=l,projection_bundle=p)['options'][0]


def test_exact_math_immutable_json_flags():
    r,c,l,p=ready();before=deepcopy(r);o=run(r,c,l,p)
    assert r==before and o['length_m']==3
    assert o['risk_m2_s']==pytest.approx(.000375)
    assert o['progress_m_s']==pytest.approx(-.0025,abs=1e-8)
    assert o['features_complete'] and not o['planning_allowed'] and o['inspection_only']
    assert not o['formal_comparison_qualified'];json.dumps(o,allow_nan=False)


def test_repeat_and_no_mutable_projection():
    r,c,l,p=ready();assert run(r,c,l,p)==run(r,c,l,p)
    with pytest.raises(TypeError): p.entries['e0']['ego']['samples'][0]['s_unwrapped']=0
    with pytest.raises(TypeError): l.entries[1]=9


@pytest.mark.parametrize('change',['geometry','frame','source_time','pose','missing_source','group','grid','clearance','duplicate','negative_speed','source_valid','path_type'])
def test_invalid_inputs_rejected(change):
    r,c,l,p=ready()
    if change=='geometry':r['metadata']['geometry_id']='wrong'
    elif change=='frame':r['metadata']['frame_id']='wrong'
    elif change=='source_time':r['source']['source_sim_time_s']=0
    elif change=='pose':r['plan_snapshot']['source_pose']=[99,0,0]
    elif change=='missing_source':r['source']={}
    elif change=='group':r['options'][0]['prediction_group_id']='missing'
    elif change=='grid':r['options'][0]['samples'][1]['t_s']=.07
    elif change=='clearance':r['options'][0]['samples'][1]['signed_min_pair_clearance_m']=float('nan')
    elif change=='duplicate':r['options'].append(deepcopy(r['options'][0]))
    elif change=='negative_speed':r['options'][0]['target_speed_mps']=-1
    elif change=='source_valid':r['source']['ego_state']['valid']=False
    elif change=='path_type':r['plan_snapshot']['paths'][0]['path_type']='reference_centerline'
    o=run(r,c,l,p);assert o['status']=='rejected' and o['risk_m2_s'] is None and o['reasons']


@pytest.mark.parametrize('safe,status',[(False,'unsafe'),(None,'unknown')])
def test_native_gate_preserved(safe,status):
    r,c,l,p=ready();r['options'][0]['native_safe']=safe
    o=run(r,c,l,p);assert o['status']==status and o['risk_m2_s'] is not None
    assert not o['formal_comparison_qualified']


def test_reference_not_promoted():
    r,c,l,p=ready();r['plan_snapshot']['paths'][0]['path_type']='reference_centerline'
    r['options'][0]['source_path_type']='reference_centerline'
    o=run(r,c,l,p);assert o['status']=='reference_only' and o['length_scope']=='reference_geometry_only'


def test_raceline_no_plan():
    r,c,l,p=ready();r['plan_snapshot']=None;r['options'][0].update(candidate_id=None,reference_id='raceline')
    p=core.prepare_projection_bundle(r,c);o=run(r,c,None,p)
    assert o['status']=='reference_only' and o['length_m'] is None


def test_partial_not_full_cost():
    r,c,l,p=ready();r['options'][0]['samples']=r['options'][0]['samples'][:2]
    r['options'][0].update(evaluation_complete=False,last_integrated_t_s=.05)
    p=core.prepare_projection_bundle(r,c);o=run(r,c,l,p)
    assert o['risk_m2_s'] is o['progress_m_s'] is None
    assert o['partial_risk_m2_s']>0 and o['partial_progress_m_s']<0 and not o['pair_complete']


@pytest.mark.parametrize('field',['anchor','pose','time','model','group','speed','geometry'])
def test_projection_invalidation(field):
    r,c,l,p=ready()
    if field=='anchor':r['source']['ego_state']['frenet']['s_unwrapped']+=1
    elif field=='pose':r['options'][0]['samples'][1]['ego_pose'][0]+=.001
    elif field=='time':r['source']['ego_state']['sim_time_s']+=1
    elif field=='model':r['metadata']['body_length_m']=.52
    elif field=='group':r['lead_predictions'][0]['prediction_group_id']='g2'
    elif field=='speed':r['options'][0]['target_speed_mps']=1.1
    else:r['metadata']['geometry_id']='wrong'
    o=run(r,c,l,p);assert o['progress_m_s'] is None


@pytest.mark.parametrize('field',['xy','origin','spline','lookahead'])
def test_length_invalidation(field):
    r,c,l,p=ready();path=r['plan_snapshot']['paths'][0]
    if field=='xy':path['points'][1]['x']+=1
    elif field=='origin':path['reference_s_origin']=2
    elif field=='spline':path['spline']={'coefficients':[1,2]}
    else:r['plan_snapshot']['generator_settings']['lookahead_m']=4
    assert run(r,c,l,p)['length_m'] is None


def test_missing_cache_no_zero():
    r,c,l,p=ready();o=run(r,c)
    assert o['length_m'] is o['progress_m_s'] is None and o['risk_m2_s']>0


def test_many_speeds_groups_and_order():
    r,c,l,p=ready();other=deepcopy(r['options'][0]);other.update(evaluation_id='e1',target_speed_mps=.9,prediction_group_id='g1')
    g=deepcopy(r['lead_predictions'][0]);g['prediction_group_id']='g1';r['lead_predictions'].append(g);r['options'].append(other)
    p=core.prepare_projection_bundle(r,c);out=core.evaluate_runtime_cost_core(r,c,length_cache=l,projection_bundle=p)
    assert [x['evaluation_id'] for x in out['options']]==['e0','e1']
    assert [x['target_speed_mps'] for x in out['options']]==[1.,.9]


def test_frozen_batch_independent_of_later_mutation():
    r,c,l,p=ready();batch=core.prepare_runtime_inputs(r,c,length_cache=l,projection_bundle=p)
    a=core.evaluate_cost_batch(batch)['options'];r['options'][0]['samples'][1]['signed_min_pair_clearance_m']=99
    assert core.evaluate_cost_batch(batch)['options']==a


def test_hot_core_does_not_call_projection_or_io(monkeypatch):
    r,c,l,p=ready()
    def fail(*a,**k):raise AssertionError('forbidden hot-path call')
    monkeypatch.setattr(core,'project_pose_series',fail)
    monkeypatch.setattr('builtins.open',fail)
    assert run(r,c,l,p)['features_complete']


def test_invalid_projection_retains_unknown():
    r,c,l,p=ready();r['source']['ego_state']['frenet'].pop('s_unwrapped')
    p=core.prepare_projection_bundle(r,c);o=run(r,c,l,p)
    assert o['progress_m_s'] is None and o['progress_reason']


@pytest.mark.parametrize('reverse',[False,True])
def test_closed_track_seam_and_short_reverse(circle,reverse):
    r,c=case(circle);L=circle.curve.L;times=[0.,.05,.1]
    positions=[L-.03,L+.02,L+.07] if not reverse else [L+.07,L+.02,L-.03]
    ep,ego=data(circle,positions,times);lp,lead=data(circle,[L+.5,L+.525,L+.55],times)
    for actor in (ego,lead):actor.update(control_step=0,valid=True)
    r['source'].update(ego_state=ego,lead_state=lead);r['plan_snapshot']['source_pose']=ego['pose']
    for sample,e,l in zip(r['options'][0]['samples'],ep,lp):sample.update(ego_pose=e['pose'],lead_pose=l['pose'])
    r['lead_predictions'][0]['poses']=[x['pose'] for x in lp]
    p=core.prepare_projection_bundle(r,c);o=run(r,c,None,p)
    assert o['progress_m_s'] is not None
    assert o['progress_m_s']==pytest.approx(.0075 if reverse else -.0025,abs=1e-4)


def test_null_checked_horizon_is_incomplete():
    r,c,l,p=ready();r['options'][0]['checked_end_s']=None
    o=run(r,c,l,p);assert not o['pair_complete'] and o['risk_m2_s'] is None


def test_native_rejected_not_promoted():
    r,c,l,p=ready();r['plan_snapshot']['paths'][0]['sampled_valid']=False
    assert run(r,c,l,p)['status']=='rejected'
