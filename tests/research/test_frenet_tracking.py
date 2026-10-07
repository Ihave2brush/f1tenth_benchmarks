"""Independent motion, domain membership, epoch and ROS pose-contract cases."""
import copy
import json
import math
from types import SimpleNamespace

import numpy as np
import pytest
from scipy import interpolate
from PIL import Image

from f1tenth_benchmarks.research.core.geometry import (
    MapRaster,TrackGeometry,FrenetTracker,TrackingOptions,build_domain,save_domain,load_domain)
from f1tenth_benchmarks.research.core.geometry.extraction import ReferenceCurve
from f1tenth_benchmarks.research.core.geometry import load_map,ExtractionOptions,build_geometry
from f1tenth_benchmarks.research.core.geometry.build_map import save_geometry,validate_candidate
from f1tenth_benchmarks.research.core.geometry.complete_stage_c import run,verify_stage_c_bundle


@pytest.fixture(scope='module')
def circle():
    row,col=np.indices((240,240));x=(col+.5)*.05;y=(240-row-.5)*.05
    radius=np.hypot(x-6,y-6)
    raster=MapRaster(np.where((radius>3)&(radius<5),255,0).astype(np.uint8),
        dict(resolution=.05,origin=[0,0,0],negate=0,free_thresh=.2,occupied_thresh=.65))
    theta=np.linspace(0,2*np.pi,1001)
    tck,_=interpolate.splprep(np.array([6+4*np.cos(theta),6+4*np.sin(theta)]),s=0,per=True)
    return TrackGeometry(ReferenceCurve(tck),raster,'tracking-circle',[10,6])


def pose(s):
    theta=s/4
    return [6+4*math.cos(theta),6+4*math.sin(theta)],theta+math.pi/2


def feed(tracker,s,time,**kwargs):
    xy,yaw=pose(s)
    return tracker.update(xy,yaw,time,now=time,geometry_id=tracker.geometry.geometry_id,**kwargs)


def test_forward_reverse_seam_and_repeated_laps(circle):
    tracker=FrenetTracker(circle,offset_calibrated=True,source_reference='sim_pose')
    initial=circle.curve.L-.3
    for i,s in enumerate(np.arange(initial,initial+2*circle.curve.L,.15)):
        result=feed(tracker,s,1+i*.05)
        assert result['valid'],result
        assert result['s_unwrapped']==pytest.approx(s,abs=1e-4)
    assert result['wrap_count']==2
    tracker.reset()
    for i,s in enumerate([.2,.1,0,-.1,-.2]):
        result=feed(tracker,s,10+i*.05)
        assert result['valid'],result
        assert result['s_unwrapped']==pytest.approx(s,abs=1e-4)
    assert result['wrap_count']==-1


@pytest.mark.parametrize('kind,reason',[
    ('duplicate','nonmonotonic_timestamp'),('stale','stale_pose'),
    ('future','future_timestamp'),('gap','pose_gap'),('jump','localization_jump'),
    ('frame','frame_mismatch'),('version','geometry_mismatch'),('nan','nonfinite_input'),
    ('speed','invalid_speed'),('heading','heading_mismatch')])
def test_rejection_does_not_commit_and_requires_explicit_recovery(circle,kind,reason):
    tracker=FrenetTracker(circle);assert feed(tracker,1,1)['valid']
    snapshot=copy.deepcopy(tracker.state);xy,yaw=pose(1.1)
    kwargs=dict(now=1.05,geometry_id=circle.geometry_id);timestamp=1.05
    if kind=='duplicate':timestamp=1.;kwargs['now']=1.
    if kind=='stale':kwargs['now']=1.3
    if kind=='future':kwargs['now']=1.
    if kind=='gap':timestamp=2.;kwargs['now']=2.
    if kind=='jump':xy,yaw=pose(10)
    if kind=='frame':kwargs['frame_id']='odom'
    if kind=='version':kwargs['geometry_id']='another-map'
    if kind=='nan':xy=[np.nan,0]
    if kind=='speed':kwargs['signed_speed']=100.
    if kind=='heading':yaw+=math.pi
    result=tracker.update(xy,yaw,timestamp,**kwargs)
    assert not result['valid'] and result['reason']==reason,result
    assert tracker.state==snapshot
    assert not feed(tracker,1.2,1.1)['valid']
    tracker.reset();assert feed(tracker,1.2,2.1)['valid']
    assert tracker.epoch==1


def test_geometry_replacement_resets_progress_epoch(circle):
    tracker=FrenetTracker(circle);assert feed(tracker,2,1)['valid']
    other=TrackGeometry(circle.curve,circle.raster,'new-version',[10,6])
    tracker.replace_geometry(other)
    assert tracker.state is None and tracker.epoch==1
    xy,yaw=pose(2)
    assert tracker.update(xy,yaw,2,now=2,geometry_id=circle.geometry_id)['reason']=='geometry_mismatch'
    tracker.reset();result=feed(tracker,2,3)
    assert result['valid'] and result['geometry_id']=='new-version'


def test_pose_offset_and_freshness_metadata(circle):
    # At s=0 heading is pi/2. Source lies 0.2 m behind the reference point.
    tracker=FrenetTracker(circle,offset=(.2,0,0),source_reference='base_link',offset_calibrated=True)
    result=tracker.update([10,5.8],math.pi/2,1,now=1,geometry_id=circle.geometry_id)
    assert result['valid'] and result['s_wrapped']==pytest.approx(0,abs=1e-4)
    np.testing.assert_allclose(result['xy'],[10,6],atol=1e-12)
    assert result['freshness_verified'] and result['version_verified'] and result['offset_calibrated']
    tracker.reset();result=tracker.update([10,5.8],math.pi/2,2)
    assert result['valid'] and not result['freshness_verified'] and not result['version_verified']
    assert not result['planning_allowed']


def test_ros_transform_uses_source_stamp_and_checks_child_frame(circle):
    transform=SimpleNamespace(child_frame_id='base_link',
        header=SimpleNamespace(frame_id='map',stamp=SimpleNamespace(sec=1,nanosec=50000000)),
        transform=SimpleNamespace(translation=SimpleNamespace(x=10.,y=6.),
            rotation=SimpleNamespace(x=0.,y=0.,z=math.sqrt(.5),w=math.sqrt(.5))))
    tracker=FrenetTracker(circle)
    result=tracker.update_tf(transform,now=1.06,geometry_id=circle.geometry_id)
    assert result['valid'] and result['timestamp']==pytest.approx(1.05)
    assert not result['offset_calibrated'] and result['freshness_verified']
    tracker.reset()
    assert tracker.update_tf(transform,now=1.4,geometry_id=circle.geometry_id)['reason']=='stale_pose'
    tracker.reset();transform.child_frame_id='laser'
    assert tracker.update_tf(transform,now=1.06,geometry_id=circle.geometry_id)['reason']=='source_frame_mismatch'
    tracker.reset();transform.child_frame_id='base_link';transform.transform.rotation.w=0;transform.transform.rotation.z=0
    assert tracker.update_tf(transform,now=1.06,geometry_id=circle.geometry_id)['reason']=='invalid_tf_rotation'


def test_history_resolves_a_crossing_only_with_one_motion_compatible_branch():
    # Deliberately crossed reference tests selection behavior, not map approval.
    gray=np.zeros((240,240),np.uint8);gray[10:230,10:230]=255;gray[190:200,110:120]=0
    raster=MapRaster(gray,dict(resolution=.05,origin=[0,0,0],negate=0,free_thresh=.2,occupied_thresh=.65))
    theta=np.linspace(0,2*np.pi,2001)
    tck,_=interpolate.splprep(np.array([6+3*np.sin(theta),6+1.5*np.sin(2*theta)]),s=0,per=True)
    g=TrackGeometry(ReferenceCurve(tck),raster,'crossing-test',[6,6])
    assert g.to_frenet([6,6])['reason']=='ambiguous_projection'
    tracker=FrenetTracker(g)
    for i,t in enumerate([-.04,-.02,0,.02,.04]):
        xy=[6+3*math.sin(t),6+1.5*math.sin(2*t)]
        yaw=math.atan2(3*math.cos(2*t),3*math.cos(t))
        result=tracker.update(xy,yaw,1+i*.05,now=1+i*.05,geometry_id=g.geometry_id)
        assert result['valid'],result
    assert result['selection_mode']=='continuity'
    assert not result['domain_verified'] and not result['planning_allowed']


def test_domain_sidecar_queries_are_exact_and_hash_checked(circle,tmp_path):
    atlas=build_domain(circle,sections=8,d_step_m=.25)
    assert atlas.query(.33,.5)['valid']
    assert not atlas.query(.33,4)['valid']
    assert not atlas.data['s_interpolation_allowed'] and not atlas.data['domain_verified']
    assert all(not run['continuous_verified'] for s in atlas.data['sections'] for run in s['sampled_runs'])
    root=tmp_path/'domain';identifier=save_domain(atlas,root)
    assert load_domain(root,circle).query(.33,.5)['valid']
    assert json.loads((root/'manifest.json').read_text())['domain_id']==identifier
    with pytest.raises(FileExistsError):save_domain(atlas,root)
    (root/'validity.json').write_text((root/'validity.json').read_text()+' ')
    with pytest.raises(ValueError,match='hash'):load_domain(root,circle)


def test_wrong_domain_version_and_invalid_options(circle,tmp_path):
    atlas=build_domain(circle,sections=4);save_domain(atlas,tmp_path/'domain')
    other=TrackGeometry(circle.curve,circle.raster,'other',[10,6])
    with pytest.raises(ValueError,match='geometry_mismatch'):load_domain(tmp_path/'domain',other)
    with pytest.raises(ValueError):TrackingOptions(max_dt_s=0).validate()
    with pytest.raises(ValueError):build_domain(circle,sections=3)


def test_invalid_source_domain_cannot_be_approved_by_a_different_projection():
    gray=np.zeros((240,240),np.uint8);gray[20:220,20:220]=255;gray[116:124,116:124]=0
    raster=MapRaster(gray,dict(resolution=.05,origin=[0,0,0],negate=0,free_thresh=.2,occupied_thresh=.65))
    theta=np.linspace(0,2*np.pi,1001)
    tck,_=interpolate.splprep(np.array([6+4*np.cos(theta),6+1.5*np.sin(theta)]),s=0,per=True)
    g=TrackGeometry(ReferenceCurve(tck),raster,'source-domain-test',[10,6])
    atlas=build_domain(g,sections=4,d_step_m=.5)
    result=atlas.query(0,.7)
    assert not result['valid'] and not result['inside_valid_domain']
    assert result['reason']=='singular_frenet'


def test_two_motion_compatible_local_branches_are_rejected():
    gray=np.zeros((240,240),np.uint8);gray[20:220,20:220]=255;gray[116:124,116:124]=0
    raster=MapRaster(gray,dict(resolution=.05,origin=[0,0,0],negate=0,free_thresh=.2,occupied_thresh=.65))
    theta=np.linspace(0,2*np.pi,1001)
    tck,_=interpolate.splprep(np.array([6+4*np.cos(theta),6+1.5*np.sin(theta)]),s=0,per=True)
    g=TrackGeometry(ReferenceCurve(tck),raster,'local-branch-test',[10,6])
    tracker=FrenetTracker(g,TrackingOptions(progress_slack_m=2.))
    assert tracker.update([9.9,6],math.pi/2,1,now=1)['valid']
    previous=copy.deepcopy(tracker.state)
    result=tracker.update([9.3,6],math.pi/2,1.05,now=1.05)
    assert not result['valid'] and result['reason']=='ambiguous_projection',result
    assert tracker.state==previous


def test_stage_c_bundle_roundtrip_and_report_tampering(tmp_path):
    import yaml
    gray=np.zeros((100,100),np.uint8);gray[10:90,10:90]=255;gray[30:70,30:70]=0
    Image.fromarray(gray).save(tmp_path/'map.png')
    yaml_path=tmp_path/'map.yaml'
    yaml_path.write_text(yaml.safe_dump(dict(image='map.png',resolution=.1,origin=[0,0,0],negate=0,
        free_thresh=.2,occupied_thresh=.65)))
    raster=load_map(yaml_path);build=build_geometry(raster,ExtractionOptions(),[5,2],0)
    base=save_geometry(raster,build,tmp_path/'base','fixture',validate_candidate(raster,build))
    rows=[]
    for i,s in enumerate([0,.1]):
        xy,_,psi,_=build['curve'].sample([s])
        rows.append(dict(simulation_time=1+i*.04,physical_state=[*xy[0],0,2.5,float(psi[0]),0,0]))
    samples=tmp_path/'samples.jsonl';samples.write_text('\n'.join(json.dumps(x)for x in rows))
    output=tmp_path/'stage_c'
    report=run(base,yaml_path,output,samples,sections=4,d_step_m=.25)
    assert report['stage_c_passed']
    g=TrackGeometry(build['curve'],raster,report['geometry_id'],[5,2])
    assert verify_stage_c_bundle(output,g)['passed']
    assert not verify_stage_c_bundle(output,g)['planning_allowed']
    (output/'report.json').write_text((output/'report.json').read_text()+' ')
    with pytest.raises(ValueError,match='hash mismatch'):verify_stage_c_bundle(output,g)


def test_progress_model_inconsistency_and_ambiguous_wrap(circle):
    tracker=FrenetTracker(circle);assert feed(tracker,1,1)['valid']
    snapshot=copy.deepcopy(tracker.state)
    # Cartesian motion is plausible, but conflicts with a supplied zero speed.
    result=feed(tracker,1.6,1.05,signed_speed=0.)
    assert result['reason']=='progress_jump' and tracker.state==snapshot
    tracker.reset();assert feed(tracker,1,2)['valid']
    assert feed(tracker,1.1,2.2)['reason']=='ambiguous_wrap'


def test_adjacent_wall_rejection_keeps_the_last_accepted_pose(circle):
    tracker=FrenetTracker(circle)
    assert tracker.update([9.02,6],math.pi/2,1,now=1)['valid']
    snapshot=copy.deepcopy(tracker.state)
    result=tracker.update([8.98,6],math.pi/2,1.05,now=1.05)
    assert result['reason']=='occupied' and tracker.state==snapshot


def test_initialization_cannot_accept_low_J_free_space():
    gray=np.zeros((240,240),np.uint8);gray[20:220,20:220]=255;gray[116:124,116:124]=0
    raster=MapRaster(gray,dict(resolution=.05,origin=[0,0,0],negate=0,free_thresh=.2,occupied_thresh=.65))
    theta=np.linspace(0,2*np.pi,1001)
    tck,_=interpolate.splprep(np.array([6+4*np.cos(theta),6+1.5*np.sin(theta)]),s=0,per=True)
    g=TrackGeometry(ReferenceCurve(tck),raster,'low-J-init',[10,6])
    assert raster.classify([9.44,6])=='free'
    tracker=FrenetTracker(g)
    result=tracker.update([9.44,6],math.pi/2,1,now=1)
    assert result['reason']=='singular_frenet' and tracker.state is None
