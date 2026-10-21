"""V5.2 hard-constraint rejection, diagnostics, terminal bounds and API checks."""
import json
import math
import os
import subprocess
import sys
from types import SimpleNamespace

import matplotlib
matplotlib.use('Agg',force=True)
import matplotlib.pyplot as plt
import numpy as np
import pytest
from scipy.interpolate import splprep
from scipy.optimize._numdiff import approx_derivative

from f1tenth_benchmarks.research.core.geometry import MapRaster,TrackGeometry,load_geometry
from f1tenth_benchmarks.research.core.geometry.extraction import ReferenceCurve
from f1tenth_benchmarks.research.overtaking import robust_adaptive_paths as v52
from f1tenth_benchmarks.research.overtaking.candidate_visualization import plot_candidate_paths


def make_straight():
    gray=np.zeros((60,80),np.uint8);gray[10:50,5:75]=255
    raster=MapRaster(gray,dict(resolution=.1,origin=[0,0,0],negate=0,free_thresh=.2,occupied_thresh=.65))
    xy=np.array([[1,3],[2,3],[4,3],[6,3]],float);tck,_=splprep(xy.T,s=0,k=3,per=False)
    return TrackGeometry(ReferenceCurve(tck,closed=False),raster,'robust-straight',xy[0])


def test_analytic_curvature_constraint_jacobian():
    basis=v52._basis(4,8);ds=np.linspace(0,4,25)
    B,D1,D2=[basis(ds,nu=i)for i in range(3)]
    c=np.linspace(-.1,.2,8);k=np.linspace(-.3,.2,25);kp=np.full(25,.125)
    def curvature(c):return v52.evaluate_path_smoothness(B@c,D1@c,D2@c,k,kp)[1]
    np.testing.assert_allclose(v52.curvature_jacobian(B,D1,D2,c,k,kp),
                              approx_derivative(curvature,c,method='3-point'),rtol=1e-5,atol=1e-6)


def test_all_seeds_diagnostics_same_limits_and_fixed_target():
    g=make_straight()
    r=v52.generate_robust_paths(g,1.5,3.1,yaw=.1,lookahead_m=4,sample_step_m=.1)
    assert r['sampled_valid_count']>0
    for path in r['paths']:
        assert set(a['seed']for a in path['attempts'])==set(v52.SEEDS)
        assert all(a['mode']=='fixed'for a in path['attempts'])
        assert len({tuple(a['limits'].values())for a in path['attempts']})==1
        for attempt in path['attempts']:
            for key in ['status','message','iterations','equality_error','min_inequality','constraint_minima']:
                assert key in attempt
        if path['sampled_valid']:
            assert sum(a['validation_accepted']is True for a in path['attempts'])==1
            p=path['points'][0]
            assert p['yaw']==pytest.approx(.1,abs=1e-6)
            assert path['start_position_error_m'] < 1e-6
            assert abs(path['start_heading_error_rad']) < 1e-6
            assert p['d_prime']==pytest.approx(math.tan(.1),abs=1e-6)
            assert path['terminal_deviation_m']==pytest.approx(0,abs=1e-6)
            assert path['polyline_collision_verified']
    assert r['planning_allowed']is False and r['inspection_only']is True
    assert not r['continuous_verified'] and not r['footprint_verified'] and not r['dynamics_verified']
    assert any(a['validation_accepted']is None for p in r['paths']for a in p['attempts'])
    json.dumps(r,allow_nan=False)
    assert r['start']['yaw'] == .1
    assert r['parameter_policy']['speed_adaptation'] is False
    assert len(r['road_sections']) == len(r['paths'][0]['points'])
    assert r['road_sections'][-1]['d_min'] == pytest.approx(-1.8)


def test_actual_offset_pose_and_yaw_near_closed_seam():
    g=load_geometry('maps/frenet/esp/geometry','maps/esp.yaml',inspect=True)
    pose=g.to_cartesian(g.curve.L-.5,.12,e_psi=.08)
    assert pose['valid']
    r=v52.generate_robust_paths(g,*pose['xy'],yaw=pose['yaw'],lookahead_m=3,max_refinements=0)
    assert r['sampled_valid_count'] > 0
    assert not r['initial_heading_assumed']
    assert r['start']['d'] == pytest.approx(.12,abs=1e-6)
    for path in r['paths']:
        if path['sampled_valid']:
            assert path['start_position_error_m'] < 1e-6
            assert abs(path['start_heading_error_rad']) < 1e-6
    assert r['road_sections'][-1]['s'] < r['road_sections'][0]['s']


def test_bounded_endpoint_explicit_and_never_outside_allowed_interval():
    g=make_straight()
    r=v52.generate_robust_paths(g,1.5,3,lookahead_m=4,max_curvature=.1,endpoint_tolerance_m=1.8)
    moved=[]
    for path in r['paths']:
        if path['sampled_valid']:
            assert abs(path['terminal_deviation_m'])<=1.8+1e-7
            assert path['actual_target_d']-path['requested_target_d']==pytest.approx(path['terminal_deviation_m'])
            assert all(abs(p['curvature_inv_m'])<=.1+1e-7 for p in path['points'])
            if path['endpoint_mode']=='bounded_preference':moved.append(path)
    assert moved,'strict curvature must require explicit endpoint displacement in this fixture'
    assert any(abs(p['terminal_deviation_m'])>.1 for p in moved)
    assert all(a['limits']['max_curvature_inv_m']==.1 for p in r['paths']for a in p['attempts'])


def test_success_flag_does_not_override_bad_constraints(monkeypatch):
    def fake(fun,c,**kwargs):
        return SimpleNamespace(x=np.full_like(c,100.),success=True,status=0,message='fake success',nit=1)
    monkeypatch.setattr(v52,'minimize',fake)
    r=v52.generate_robust_paths(make_straight(),1.5,3,lookahead_m=3)
    assert r['sampled_valid_count']==0
    assert all(not a['solver_constraints_accepted']for p in r['paths']for a in p['attempts'])
    assert all(p['reason']=='optimizer_constraints_not_satisfied'for p in r['paths'])


def test_successful_fit_rechecked_against_exact_geometry(monkeypatch):
    g=make_straight();original=g.to_cartesian
    def reject(s,d,e_psi=None):
        result=original(s,d,e_psi)
        return dict(result,valid=False,reason='injected_exact_rejection')if 1.9<s<2.1 else result
    monkeypatch.setattr(g,'to_cartesian',reject)
    r=v52.generate_robust_paths(g,1.5,3,lookahead_m=3,max_refinements=0)
    assert r['sampled_valid_count']==0
    assert all(p['reason']=='injected_exact_rejection'for p in r['paths'])


def test_polyline_collision_cannot_be_accepted(monkeypatch):
    monkeypatch.setattr(v52,'validate_path_segments',lambda *a,**k:
                        dict(valid=False,reason='segment_occupied',segment_index=3))
    r=v52.generate_robust_paths(make_straight(),1.5,3,lookahead_m=3,max_refinements=0)
    assert r['sampled_valid_count']==0
    assert all(p['reason']=='segment_occupied'for p in r['paths'])
    fig,_=plot_candidate_paths(make_straight(),r);fig.canvas.draw();plt.close(fig)


@pytest.mark.parametrize('kwargs',[{'endpoint_tolerance_m':-1},{'endpoint_tolerance_m':float('nan')},
    {'max_refinements':True},{'max_iterations':0},{'sample_step_m':1e-320},
    {'max_slope':0},{'yaw':math.pi},{'lookahead_m':6}])
def test_invalid_inputs(kwargs):
    args=dict(lookahead_m=3);args.update(kwargs)
    with pytest.raises(ValueError):v52.generate_robust_paths(make_straight(),1.5,3,**args)


def test_initial_missing_boundary_rejected_clearly():
    g=make_straight();start=g.to_frenet([1.5,3]);boundary=g.boundaries
    g.to_frenet=lambda *a,**k:start
    g.boundaries=lambda s:dict(boundary(s),d_left=None)
    with pytest.raises(ValueError,match='boundary at start'):
        v52.generate_robust_paths(g,1.5,3,lookahead_m=3)


def test_closed_seam_and_original_target_source():
    g=load_geometry('maps/frenet/esp/geometry','maps/esp.yaml',inspect=True)
    r=v52.generate_robust_paths(g,*g.reference(g.curve.L-1)['xy'],lookahead_m=3)
    assert r['sampled_valid_count']>0
    for p in r['paths']:
        if p['sampled_valid']:
            assert p['points'][-1]['s']==pytest.approx(2,abs=1e-4)
            assert np.all(np.diff([v['s_progress']for v in p['points']])>0)
            assert abs(p['terminal_deviation_m'])<1e-6


def test_cli_json_save_and_no_show(tmp_path):
    output=tmp_path/'robust.png'
    command=[sys.executable,'-B','-m','f1tenth_benchmarks.research.overtaking.robust_adaptive_paths',
        '--geometry-dir','maps/frenet/esp/geometry','--map-yaml','maps/esp.yaml',
        '--x','0','--y','0','--lookahead','3','--endpoint-tolerance','.15','--save',str(output),'--no-show']
    env=dict(os.environ,OPENBLAS_NUM_THREADS='1',MPLBACKEND='Agg',MPLCONFIGDIR=str(tmp_path/'mpl'))
    done=subprocess.run(command,capture_output=True,text=True,env=env,timeout=90)
    assert done.returncode==0,done.stderr
    r=json.loads(done.stdout)
    assert r['endpoint_tolerance_m']==.15 and r['candidate_count']==5
    assert output.read_bytes().startswith(b'\x89PNG')


@pytest.mark.parametrize('name',['esp','aut','gbr','mco','CornerHall','my_map'])
def test_real_maps_v4_format_and_fresh_exact_checks(name):
    g=load_geometry(f'maps/frenet/{name}/geometry',f'maps/{name}.yaml',inspect=True)
    r=v52.generate_robust_paths(g,*g.reference(1)['xy'],lookahead_m=3,sample_step_m=.1,
                              endpoint_tolerance_m=.15,max_refinements=1)
    assert [p['candidate_id']for p in r['paths']]==[1,2,3,4,5]
    for path in r['paths']:
        if path['sampled_valid']:
            for p in path['points']:
                assert g.to_cartesian(p['s'],p['d'])['valid']
                b=g.boundaries(p['s'])
                assert b['d_right']+.2-1e-8<=p['d']<=b['d_left']-.2+1e-8
            assert abs(path['terminal_deviation_m'])<=.15+1e-7
        else:assert path['reason']
    fig,_=plot_candidate_paths(g,r);fig.canvas.draw();plt.close(fig)
