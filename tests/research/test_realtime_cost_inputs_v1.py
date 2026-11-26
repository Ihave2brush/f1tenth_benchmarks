"""Portable input/cache contracts; preserves the research implementation."""
from copy import deepcopy
import pytest
from runtime_cost_test_support_v1 import case,POLICY,PROJECTION
from f1tenth_benchmarks.research.overtaking.realtime_cost_inputs_v1 import (
    RuntimeCostContext,prepare_runtime_length_cache,_length_key)
from test_realtime_cost_core_v1 import ready
from f1tenth_benchmarks.research.overtaking import realtime_cost_core_v1 as core


@pytest.mark.parametrize('change',['none','weights','wrong_mode','extra_component'])
def test_explicit_risk_only_policy(change):
    r,c=case();p=deepcopy(POLICY)
    if change=='none':p=None
    elif change=='weights':p['weights']={'risk':1.}
    elif change=='wrong_mode':p['components']['risk']['mode']='minimum_shortfall_squared'
    else:p['components']['length']={'scale_m':1.,'valid_length_m':[0.,10.]}
    with pytest.raises(ValueError):RuntimeCostContext(c.geometry,risk_policy=p,tracking_options=PROJECTION)


@pytest.mark.parametrize('value',[0.,-1.,float('nan'),float('inf'),True])
def test_cache_rejects_invalid_lengths(value):
    r,c=case()
    with pytest.raises(ValueError):prepare_runtime_length_cache(r['plan_snapshot'],c.geometry_id,{1:value},method='cached_frenet_integral')


def test_cache_copied_readonly_and_order_independent():
    r,c=case();before=deepcopy(r);values={1:3.}
    cache=prepare_runtime_length_cache(r['plan_snapshot'],c.geometry_id,values,method='cached_frenet_integral');values[1]=99.
    assert cache.entries[1]==3. and r==before
    with pytest.raises(TypeError):cache.entries[1]=9.
    reordered=dict(reversed(list(r['plan_snapshot'].items())))
    assert _length_key(reordered,c.geometry_id)==cache.key


def test_raw_risk_reuses_cost_v1_component():
    from f1tenth_benchmarks.research.overtaking.cost_evaluation_v1 import _component
    r,c,l,p=ready();out=core.evaluate_runtime_cost_core(r,c,length_cache=l,projection_bundle=p)['options'][0]
    samples=[dict(t_s=s['t_s'],signed_clearance_m=s['signed_min_pair_clearance_m']) for s in r['options'][0]['samples']]
    row=dict(requested_horizon_s=.1,risk_series=dict(status='reported_complete',evaluation_complete=True,samples=samples))
    expected=_component('risk',dict(c.risk_spec),row)
    assert out['risk_m2_s']==expected['raw_value']==pytest.approx(.000375)


def test_core_never_invokes_native_producers(monkeypatch):
    from f1tenth_benchmarks.research.overtaking.two_car_decision import OvertakingSelector
    from f1tenth_benchmarks.research.overtaking.two_car_env import TwoCarOvertakingEnv
    r,c,l,p=ready()
    def fail(*args,**kwargs):raise AssertionError('native producer invoked')
    monkeypatch.setattr(OvertakingSelector,'__call__',fail);monkeypatch.setattr(OvertakingSelector,'_rollout',fail)
    monkeypatch.setattr(TwoCarOvertakingEnv,'step',fail)
    out=core.evaluate_runtime_cost_core(r,c,length_cache=l,projection_bundle=p)
    assert out['total_cost'] is None and not out['planning_allowed'] and not out['formal_comparison_qualified']


def test_two_groups_have_independent_time_grids():
    r,c,l,p=ready();other=deepcopy(r['options'][0]);other.update(evaluation_id='e1',prediction_group_id='g1',
        requested_horizon_s=.08,last_integrated_t_s=.08,requested_sample_times_s=[0.,.04,.08],target_speed_mps=.9)
    for s,t in zip(other['samples'],[0.,.04,.08]):s['t_s']=t
    group=deepcopy(r['lead_predictions'][0]);group.update(prediction_group_id='g1',requested_sample_times_s=[0.,.04,.08])
    r['options'].append(other);r['lead_predictions'].append(group)
    projected=core.prepare_projection_bundle(r,c)
    result=core.evaluate_runtime_cost_core(r,c,length_cache=l,projection_bundle=projected)
    assert [x['requested_horizon_s'] for x in result['options']]==[.1,.08]
    assert result['options'][1]['risk_m2_s']==pytest.approx(.0003)
    assert result['options'][1]['progress_m_s']==pytest.approx(-.002)


def test_archive_join_allows_terminal_snapshot():
    from tools.check_runtime_cost_archive_v1 import source_frame
    r,c,l,p=ready();s=r['source'];f=dict(ego_state=s['ego_state'],lead_state=s['lead_state'],plan=r['plan_snapshot'])
    terminal=deepcopy(f)
    for role in ('ego','lead'):terminal[role+'_state'].update(control_step=1,sim_time_s=10.1)
    assert source_frame(r,[terminal,f]) is f


@pytest.mark.parametrize('kind',['duplicate','missing','lead_time','plan'])
def test_archive_join_rejects_ambiguous_or_mixed_sources(kind):
    from tools.check_runtime_cost_archive_v1 import source_frame
    r,c,l,p=ready();s=r['source'];f=deepcopy(dict(ego_state=s['ego_state'],lead_state=s['lead_state'],plan=r['plan_snapshot']))
    frames=[f]
    if kind=='duplicate':frames.append(deepcopy(f))
    elif kind=='missing':frames=[]
    elif kind=='lead_time':f['lead_state']['sim_time_s']+=1
    else:f['plan']['source_pose'][0]+=1
    with pytest.raises(ValueError):source_frame(r,frames)
