"""Fixed anchors accelerate exact equations; old validation sites stay intact."""
import json
from pathlib import Path
import numpy as np
import pytest
from test_robust_adaptive_paths import make_straight
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.overtaking.fixed_arc_lookup import FixedArcLookup,get_arc_lookup
from f1tenth_benchmarks.research.overtaking.road_table import get_road_table
from f1tenth_benchmarks.research.overtaking.robust_adaptive_paths import generate_robust_paths
from f1tenth_benchmarks.research.overtaking.session import SessionConfig


@pytest.mark.parametrize('name',['my_map','esp'])
def test_exact_reference_agreement_across_knots_and_seam(name):
    geometry=load_geometry('maps/frenet/'+name+'/geometry','maps/'+name+'.yaml',inspect=True)
    curve=geometry.curve;lookup=FixedArcLookup(curve)
    lookup.prepare_window(curve.L-.5,3.)
    rng=np.random.default_rng(42)
    knots=(curve.s_nodes-curve.start_s)%curve.L
    s=np.r_[rng.uniform(-curve.L,2*curve.L,250),knots,knots+1e-9,
            [-1e-10,0.,curve.L-1e-10,curve.L,2*curve.L]]
    expected=curve.sample(s);actual=lookup.sample(s)
    for a,b in zip(actual,expected):np.testing.assert_allclose(a,b,rtol=0,atol=2e-9)
    assert lookup.stats['inverse_points']==len(s)
    assert lookup.stats['scalar_fallbacks']<len(s)//4


def test_open_endpoints_and_invalid_inputs():
    curve=make_straight().curve;lookup=FixedArcLookup(curve)
    lookup.prepare_window(.1,2.)
    s=np.r_[0.,np.linspace(.01,curve.L-.01,80),curve.L]
    np.testing.assert_allclose(lookup.inverse_many(s),[curve.u_at_s(v) for v in s],atol=1e-12,rtol=0)
    assert len(lookup.inverse_many([]))==0
    for bad in ([-.01],[curve.L+.01],[float('nan')]):
        with pytest.raises(ValueError):lookup.inverse_many(bad)
    with pytest.raises(ValueError):FixedArcLookup(curve,max_anchors=1)


def test_window_reuses_anchors_and_bounds_storage():
    g=load_geometry('maps/frenet/my_map/geometry','maps/my_map.yaml',inspect=True)
    lookup=FixedArcLookup(g.curve,max_anchors=80)
    lookup.prepare_window(0.,3.);first=lookup.stats['anchor_misses']
    lookup.prepare_window(.12,3.)
    assert lookup.stats['anchor_hits']>50 and lookup.stats['anchor_misses']-first<=4
    for s in (1.,5.,9.,g.curve.L-.1,.1,0.,6.,12.,24.):lookup.prepare_window(s,3.)
    assert len(lookup.anchors)<=80 and lookup.stats['anchor_evictions']>0
    np.testing.assert_allclose(lookup.inverse_many([.123,11.99]),
        [g.curve.u_at_s(v) for v in [.123,11.99]],atol=1e-12,rtol=0)


def test_uncertified_roots_fall_back_to_original(monkeypatch):
    curve=make_straight().curve;lookup=FixedArcLookup(curve)
    monkeypatch.setattr(lookup,'_residual',lambda u,*args:np.ones(len(u)))
    values=[.1,.7,1.2]
    np.testing.assert_array_equal(lookup.inverse_many(values),[curve.u_at_s(v) for v in values])
    assert lookup.stats['scalar_fallbacks']==3


def test_source_version_invalidates_lookup():
    g=make_straight();table=get_road_table(g);lookup=get_arc_lookup(table)
    lookup.prepare_window(.1,2.)
    g.geometry_id+='-new'
    changed=get_road_table(g)
    assert changed is not table and get_arc_lookup(changed) is not lookup
    assert not get_arc_lookup(changed).anchors


@pytest.mark.parametrize('index',[0,25,50,75,99])
def test_same_old_sites_pose_and_valid_ids(index):
    states=json.loads(Path('docs/project/reports/my_map_3m_inputs_20261029.json').read_text())['states']
    pose=states[index]['pose'];results=[]
    for mode in ('fast_with_fallback','fixed_grid_lookup'):
        g=load_geometry('maps/frenet/my_map/geometry','maps/my_map.yaml',inspect=True)
        results.append(generate_robust_paths(g,*pose[:2],yaw=pose[2],lookahead_m=3.,
            query_mode=mode,generation_mode='direct_then_repair'))
    a,b=results
    assert [p['sampled_valid'] for p in a['paths']]==[p['sampled_valid'] for p in b['paths']]
    assert b['actual_sample_step_m']==.025 and 'validation_grid' not in b
    assert b['query_reuse']['fixed_arc_lookup']['enabled']
    assert len(a['road_sections'])==len(b['road_sections'])==121
    for p,q in zip(a['paths'],b['paths']):
        assert len(q['points'])==len(p['points'])
        assert not q['points'] or len(q['points'])==121
        np.testing.assert_array_equal([x['s_progress'] for x in p['points']],
                                      [x['s_progress'] for x in q['points']])
        if p['sampled_valid']:
            np.testing.assert_allclose([[x['x'],x['y'],x['curvature_inv_m']] for x in p['points']],
                [[x['x'],x['y'],x['curvature_inv_m']] for x in q['points']],atol=2e-6,rtol=0)
            assert q['start_position_error_m']<=1e-4
            assert abs(q['start_heading_error_rad'])<=1e-5


def test_custom_reference_uses_original_fallback():
    g=make_straight();reference=g.reference
    g.reference=lambda s:reference(s)
    result=generate_robust_paths(g,1.5,3.,lookahead_m=3.,
        query_mode='fixed_grid_lookup',generation_mode='direct_only')
    assert result['sampled_valid_count']==5
    assert not result['query_reuse']['fixed_arc_lookup']['enabled']
    assert result['query_reuse']['fixed_arc_lookup']['inverse_points']==0
    with pytest.raises(ValueError):SessionConfig(query_mode='fixed_grid_lookup',validation_step_m=.05)


def test_changed_integral_invalidates_cache_and_disables_batch_inverse():
    g=make_straight();table=get_road_table(g);lookup=get_arc_lookup(table)
    lookup.prepare_window(.1,2.)
    integral=g.curve.integral
    g.curve.integral=lambda a,b:integral(a,b)
    changed=get_road_table(g)
    assert changed is not table and not changed.standard
    result=generate_robust_paths(g,1.5,3.,lookahead_m=3.,query_mode='fixed_grid_lookup',
                                 generation_mode='direct_only')
    assert result['sampled_valid_count']==5
    assert not result['query_reuse']['fixed_arc_lookup']['enabled']
    assert result['query_reuse']['fixed_arc_lookup']['inverse_points']==0
