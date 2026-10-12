"""New constrained paths preserve V5; tests include exact segment collisions."""
import json
import math

import matplotlib
matplotlib.use('Agg', force=True)
import matplotlib.pyplot as plt
import numpy as np
import pytest
from scipy import interpolate
from scipy.interpolate import BSpline

from f1tenth_benchmarks.research.core.geometry import MapRaster, TrackGeometry, load_geometry
from f1tenth_benchmarks.research.core.geometry.extraction import ReferenceCurve
from f1tenth_benchmarks.research.overtaking.constrained_adaptive_paths import (
    generate_constrained_paths, validate_path_segments, query_road_sections)
from f1tenth_benchmarks.research.overtaking.candidate_visualization import plot_candidate_paths


def straight(narrow=False):
    gray = np.zeros((60,80), np.uint8)
    gray[10:50,5:75] = 255
    if narrow:
        gray[10:25,28:33] = 0
        gray[35:50,28:33] = 0
    raster = MapRaster(gray, dict(resolution=.1,origin=[0,0,0],negate=0,
                                  free_thresh=.2,occupied_thresh=.65))
    xy = np.array([[1,3],[2,3],[4,3],[6,3]],float)
    tck,_ = interpolate.splprep(xy.T,s=0,k=3,per=False)
    return TrackGeometry(ReferenceCurve(tck,closed=False),raster,'constrained-straight',xy[0])


def test_initial_heading_endpoints_and_limits():
    g = straight()
    r = generate_constrained_paths(g,1.5,3,yaw=.1,lookahead_m=4,sample_step_m=.1)
    assert r['sampled_valid_count'] > 0
    for path in r['paths']:
        if not path['sampled_valid']:
            continue
        first,last=path['points'][0],path['points'][-1]
        assert [first['x'],first['y']] == pytest.approx([1.5,3],abs=1e-6)
        assert first['yaw'] == pytest.approx(.1,abs=1e-6)
        assert first['d_prime'] == pytest.approx(math.tan(.1),abs=1e-6)
        assert last['d'] == pytest.approx(path['target_d'],abs=1e-6)
        assert last['d_prime'] == pytest.approx(0,abs=1e-6)
        representation=path['spline']
        spline=BSpline(representation['knots'],representation['coefficients'],representation['degree'])
        assert spline(0,nu=1)==pytest.approx(math.tan(.1),abs=1e-6)
        assert spline(4)==pytest.approx(path['target_d'],abs=1e-6)
        assert path['polyline_collision_verified'] and path['heading_matched']
        assert all(p['valid'] for p in path['points'])
        assert max(abs(p['curvature_inv_m']) for p in path['points']) <= r['max_curvature_inv_m']+1e-6
        assert max(abs(p['d_prime']) for p in path['points']) <= r['max_slope']+1e-6
    assert not r['continuous_verified'] and not r['dynamics_verified']
    json.dumps(r,allow_nan=False)


def test_narrow_corridor_no_accepted_segment_crosses_obstacles():
    g = straight(narrow=True)
    r = generate_constrained_paths(g,1.5,3,lookahead_m=4,sample_step_m=.1)
    assert r['sampled_valid_count'] > 0
    for path in r['paths']:
        if path['sampled_valid']:
            points=path['points']
            assert validate_path_segments(g.raster,[[p['x'],p['y']]for p in points])['valid']
            assert max(abs(p['curvature_inv_m'])for p in points)<=r['max_curvature_inv_m']+1e-6
        else:
            assert path['reason']
    fig,_=plot_candidate_paths(g,r)
    fig.canvas.draw();plt.close(fig)


def test_segment_supercover_detects_obstacle_away_from_midpoint():
    g=straight()
    a,b=np.array([1.5,3]),np.array([5.5,3])
    # Block at x=2, away from both endpoints and midpoint x=3.5.
    col,row=g.raster.map_to_pixel([2,3]).cell
    g.raster.occupancy[row,col]=1
    assert g.raster.classify(a)=='free' and g.raster.classify(b)=='free'
    assert g.raster.classify((a+b)/2)=='free'
    result=validate_path_segments(g.raster,[a,b])
    assert not result['valid'] and result['reason']=='segment_occupied'


@pytest.mark.parametrize('occupancy',[-1,1])
def test_corner_touch_and_rotated_grid(occupancy):
    gray=np.full((10,10),255,np.uint8)
    raster=MapRaster(gray,dict(resolution=.1,origin=[3,-2,.7],negate=0,free_thresh=.2,occupied_thresh=.65))
    # Diagonal touches corner of local cell (2,1), though it travels through
    # diagonal cells (1,1) and (2,2). Conservative supercover includes contact.
    raster.occupancy[10-1-1,2]=occupancy
    xy=raster.local_to_map(np.array([[.15,.15],[.25,.25]]))
    result=validate_path_segments(raster,xy)
    assert not result['valid']
    assert result['reason']==('segment_unknown'if occupancy==-1 else 'segment_occupied')


def test_segments_outside_and_budget():
    g=straight()
    assert not validate_path_segments(g.raster,[[1,3],[-1,3]])['valid']
    assert validate_path_segments(g.raster,[[1.5,3],[5.5,3]],max_cells=1)['reason']=='segment_budget_exceeded'
    assert validate_path_segments(g.raster,[])['reason']=='insufficient_points'
    assert validate_path_segments(g.raster,[[1,3],None])['reason']=='invalid_coordinates'


@pytest.mark.parametrize('kwargs',[{'sample_step_m':1e-320},{'lookahead_m':0},
    {'max_slope':0},{'max_curvature':float('nan')},{'max_refinements':True},
    {'lookahead_m':6},{'yaw':math.pi}])
def test_invalid_configuration(kwargs):
    args=dict(lookahead_m=3);args.update(kwargs)
    with pytest.raises(ValueError):generate_constrained_paths(straight(),1.5,3,**args)


def test_missing_boundary_fails_closed():
    g=straight()
    original=g.boundaries
    g.boundaries=lambda s:dict(original(s),d_left=None)
    sections=query_road_sections(g,[.5,1],.2)
    assert all(not s['boundary_valid']for s in sections)


def test_initial_missing_boundary_is_a_clear_error():
    g=straight()
    start=g.to_frenet([1.5,3])
    original=g.boundaries
    g.to_frenet=lambda *args,**kwargs:start
    g.boundaries=lambda s:dict(original(s),d_left=None)
    with pytest.raises(ValueError,match='boundary at start'):
        generate_constrained_paths(g,1.5,3,lookahead_m=3)


def test_invalid_sections_preserve_ids_and_v4_legend():
    g=straight()
    original=g.boundaries
    g.boundaries=lambda s:dict(original(s),valid=False,reason='missing_wall') if 1.9<s<2.1 else original(s)
    r=generate_constrained_paths(g,1.5,3,lookahead_m=4,sample_step_m=.1)
    assert r['sampled_valid_count']==0
    assert [p['candidate_id']for p in r['paths']]==[1,2,3,4,5]
    assert all(p['reason']=='invalid_road_section'for p in r['paths'])
    fig,ax=plot_candidate_paths(g,r)
    assert len([l for l in ax.lines if l.get_label().startswith('Candidate ')])==5
    plt.close(fig)


@pytest.mark.parametrize('name',['esp','aut','gbr','mco','CornerHall','my_map'])
def test_six_published_maps_exact_checks(name):
    g=load_geometry(f'maps/frenet/{name}/geometry',f'maps/{name}.yaml',inspect=True)
    r=generate_constrained_paths(g,*g.reference(1)['xy'],lookahead_m=3,sample_step_m=.1)
    assert len(r['paths'])==5
    for path in r['paths']:
        if path['points']:
            representation=path['spline']
            spline=BSpline(representation['knots'],representation['coefficients'],representation['degree'])
            ds=[p['s_progress']-representation['source_s_origin']for p in path['points']]
            np.testing.assert_allclose(spline(ds),[p['d']for p in path['points']],atol=1e-8)
        if path['sampled_valid']:
            for p in path['points']:
                assert g.to_cartesian(p['s'],p['d'])['valid']
                b=g.boundaries(p['s'])
                assert b['d_right']+.2-1e-7<=p['d']<=b['d_left']-.2+1e-7
            assert path['polyline_collision_verified']
        else:assert path['reason']
    fig,_=plot_candidate_paths(g,r)
    fig.canvas.draw();plt.close(fig)


def test_closed_seam_source_progress():
    g=load_geometry('maps/frenet/esp/geometry','maps/esp.yaml',inspect=True)
    r=generate_constrained_paths(g,*g.reference(g.curve.L-1)['xy'],lookahead_m=3)
    assert r['sampled_valid_count']>0
    for path in r['paths']:
        if path['points']:
            assert path['points'][-1]['s']==pytest.approx(2,abs=1e-4)
            assert np.all(np.diff([p['s_progress']for p in path['points']])>0)
