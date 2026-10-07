"""Analytical continuous projection, branch rejection and pose contracts."""
import math
import json

import numpy as np
import pytest
from PIL import Image
from scipy import interpolate

from f1tenth_benchmarks.research.core.geometry import (
    MapRaster, TrackGeometry, load_geometry, adapt_pose, ExtractionOptions, build_geometry, load_map)
from f1tenth_benchmarks.research.core.geometry.extraction import ReferenceCurve
from f1tenth_benchmarks.research.core.geometry.build_map import save_geometry, validate_candidate
from f1tenth_benchmarks.research.core.geometry.check_frenet import check


@pytest.fixture(scope='module')
def circle():
    size, resolution = 240, .05
    y, x = np.meshgrid((np.arange(size)+.5)*resolution,
                       (np.arange(size)+.5)*resolution, indexing='ij')
    radius = np.hypot(x-6,y-6)
    gray = np.where((radius>3)&(radius<5),255,0).astype(np.uint8)[::-1]
    raster = MapRaster(gray, dict(resolution=resolution,origin=[0,0,0],negate=0,
                                  free_thresh=.2,occupied_thresh=.65))
    theta = np.linspace(0,2*np.pi,1001)
    xy = np.column_stack((6+4*np.cos(theta),6+4*np.sin(theta)))
    tck,_ = interpolate.splprep(xy.T,s=0,per=True)
    return TrackGeometry(ReferenceCurve(tck),raster,'analytic-circle',[10,6])


@pytest.mark.parametrize('theta,d', [(0,0),(.37,.7),(1.2,-.8),(3,.5),(2*np.pi-1e-6,-.2)])
def test_independent_circle_projection_and_inverse(circle,theta,d):
    xy = np.array([6,6])+(4-d)*np.array([np.cos(theta),np.sin(theta)])
    yaw=theta+np.pi/2+.1
    projected=circle.to_frenet(xy,yaw)
    assert projected['valid'], projected
    # s=0 and L coincide; compare circular distance explicitly at seam.
    expected=theta*4
    diff=abs(projected['s_wrapped']-expected)
    assert min(diff,circle.curve.L-diff)<1e-4
    assert projected['d']==pytest.approx(d,abs=1e-4)
    assert projected['e_psi']==pytest.approx(.1,abs=1e-4)
    inverse=circle.to_cartesian(expected,d,.1)
    assert inverse['valid'],inverse
    np.testing.assert_allclose(inverse['xy'],xy,atol=1e-4)
    assert not inverse['planning_allowed'] and not inverse['domain_verified']


def test_modulo_seam_reverse_queries_and_missing_yaw(circle):
    for s in [-1e-6,0,1e-6,circle.curve.L-1e-6,circle.curve.L+1e-6]:
        inverse=circle.to_cartesian(s,.3)
        assert inverse['valid'] and inverse['yaw'] is None
        projected=circle.to_frenet(inverse['xy'])
        assert projected['e_psi'] is None
        difference=abs(projected['s_wrapped']-s%circle.curve.L)
        assert min(difference,circle.curve.L-difference)<1e-5


def test_map_occupancy_nonfinite_and_singular_queries(circle):
    assert circle.to_frenet([-1,0])['reason']=='outside_map'
    assert circle.to_frenet([6,6])['reason']=='occupied'
    assert circle.to_frenet([np.nan,0])['reason']=='nonfinite_input'
    inverse=circle.to_cartesian(0,4)
    assert not inverse['valid'] and inverse['reason']=='singular_frenet'
    assert inverse['xy'] is not None
    assert circle.to_cartesian(np.nan,0)['xy'] is None


def test_hint_cannot_hide_jump_or_remote_branch(circle):
    xy=circle.to_cartesian(2,0)['xy']
    assert circle.to_frenet(xy,s_hint=2,search_window_m=.1)['valid']
    rejected=circle.to_frenet(xy,s_hint=12,search_window_m=.1)
    assert rejected['reason']=='no_projection' and rejected['s_wrapped'] is None
    with pytest.raises(ValueError):
        circle.to_frenet(xy,s_hint=2)


def test_nonlocal_duplicate_branches_are_ambiguous(circle):
    # Invalid double-wound geometry deliberately tests branch protection;
    # it must never be silently accepted merely because its points lie in free space.
    theta=np.linspace(0,4*np.pi,2001)
    xy=np.column_stack((6+4*np.cos(theta),6+4*np.sin(theta)))
    tck,_=interpolate.splprep(xy.T,s=0,per=True)
    geometry=TrackGeometry(ReferenceCurve(tck),circle.raster,'double-wound',[10,6])
    result=geometry.to_frenet([9.9,6.1])
    assert not result['valid'] and result['reason']=='ambiguous_projection'
    assert result['candidate_count']>=2 and result['s_wrapped'] is None
    assert geometry.to_frenet([9.9,6.1],s_hint=0,search_window_m=.5)['reason']=='ambiguous_projection'


def test_boundary_ray_signs_and_analytic_width(circle):
    boundary=circle.boundaries(0)
    assert boundary['valid'] and boundary['d_left']>0>boundary['d_right']
    assert boundary['d_left']==pytest.approx(1,abs=.05)
    assert boundary['d_right']==pytest.approx(-1,abs=.05)
    assert boundary['method']=='exact_grid_ray' and not boundary['interpolation_verified']


def test_pose_offset_rotation_timestamp_and_frame():
    result=adapt_pose([1,2],np.pi/2,123.,offset=[.3,-.1,.2])
    np.testing.assert_allclose(result['xy'],[1.1,2.3],atol=1e-12)
    assert result['yaw']==pytest.approx(np.pi/2+.2)
    assert result['timestamp']==123.
    with pytest.raises(ValueError,match='frame_mismatch'):
        adapt_pose([1,2],0,123,frame_id='odom')
    with pytest.raises(ValueError,match='nonfinite'):
        adapt_pose([1,2],0,np.inf)


def test_loading_checks_map_hash_and_requires_inspect(tmp_path):
    import yaml
    gray=np.zeros((100,100),np.uint8);gray[10:90,10:90]=255;gray[30:70,30:70]=0
    Image.fromarray(gray).save(tmp_path/'map.png')
    path=tmp_path/'map.yaml'
    path.write_text(yaml.safe_dump(dict(image='map.png',resolution=.1,origin=[0,0,0],negate=0,
                                       free_thresh=.2,occupied_thresh=.65)))
    raster=load_map(path)
    build=build_geometry(raster,ExtractionOptions(output_step_m=.1),[5,2],0)
    root=save_geometry(raster,build,tmp_path/'artifacts','ring',validate_candidate(raster,build))
    with pytest.raises(ValueError,match='not globally validated'):
        load_geometry(root,path)
    geometry=load_geometry(root,path,inspect=True)
    assert geometry.to_frenet(geometry.reference(0)['xy'])['valid']
    path.write_text(path.read_text()+'\n# modified input bytes\n')
    with pytest.raises(ValueError,match='geometry_mismatch'):
        load_geometry(root,path,inspect=True)


def test_inside_corridor_does_not_imply_valid_source_coordinates():
    gray=np.zeros((240,240),np.uint8);gray[20:220,20:220]=255;gray[116:124,116:124]=0
    raster=MapRaster(gray,dict(resolution=.05,origin=[0,0,0],negate=0,free_thresh=.2,occupied_thresh=.65))
    theta=np.linspace(0,2*np.pi,1001)
    xy=np.column_stack((6+4*np.cos(theta),6+1.5*np.sin(theta)))
    tck,_=interpolate.splprep(xy.T,s=0,per=True)
    geometry=TrackGeometry(ReferenceCurve(tck),raster,'ellipse',[10,6])
    # The Cartesian point is free, yet the source normal has crossed its
    # curvature singularity. Reprojection to a different segment cannot approve it.
    result=geometry.to_cartesian(0,.7)
    assert raster.classify(result['xy'])=='free'
    assert not result['valid'] and result['reason']=='singular_frenet'


def test_sampled_checker_reports_scope_and_never_approves_artifacts(circle):
    report=check(circle,count=16)
    assert report['sampled_check_passed'] and report['invalid_queries']==0
    assert report['valid_queries']==18*7
    assert report['max_s_error_m']<1e-4 and report['max_position_error_m']<1e-4
    assert not report['planning_allowed'] and not report['domain_verified']
