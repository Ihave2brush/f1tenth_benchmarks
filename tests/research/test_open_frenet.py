"""Open references must not wrap, close their drawing or lose source validity."""
import json
import gzip
import threading
from urllib.request import urlopen
from urllib.error import HTTPError

import numpy as np
import pytest
from scipy import interpolate
from PIL import Image

from f1tenth_benchmarks.research.core.geometry import MapRaster,TrackGeometry,FrenetTracker,load_geometry
from f1tenth_benchmarks.research.core.geometry.extraction import ReferenceCurve
from f1tenth_benchmarks.research.core.geometry.build_open_map import build_open_geometry
from f1tenth_benchmarks.research.core.geometry.build_map import save_geometry,verify_artifacts
from f1tenth_benchmarks.research.core.geometry.section_ranges import query_section
from f1tenth_benchmarks.research.core.geometry.section_ranges import load_ranges
from f1tenth_benchmarks.research.core.geometry.domain import json_bytes
from f1tenth_benchmarks.research.core.geometry.map_raster import sha256
import hashlib
from f1tenth_benchmarks.research.core.geometry.viewer import MapInspector,make_server
from f1tenth_benchmarks.research.core.geometry.complete_stage_d import audit_geometry,summarize
from f1tenth_benchmarks.research.core.geometry.check_frenet import check
from f1tenth_benchmarks.research.core.geometry.validate_stage_d import validate


@pytest.fixture
def straight():
    gray=np.zeros((60,80),np.uint8);gray[10:50,5:75]=255
    raster=MapRaster(gray,dict(resolution=.1,origin=[0,0,0],negate=0,free_thresh=.2,occupied_thresh=.65))
    points=np.array([[1,3],[2,3],[4,3],[6,3]],float)
    tck,_=interpolate.splprep(points.T,s=0,k=3,per=False)
    return TrackGeometry(ReferenceCurve(tck,closed=False),raster,'open-straight',points[0])


def test_open_endpoints_and_inverse_do_not_wrap(straight):
    assert straight.curve.L==pytest.approx(5)
    assert straight.reference(0)['xy']==pytest.approx([1,3])
    assert straight.reference(straight.curve.L)['xy']==pytest.approx([6,3])
    assert straight.distance_s(.01,4.99)==pytest.approx(4.98)
    for s in (0,.5,4.5,straight.curve.L):
        result=straight.to_cartesian(s,.2)
        assert result['valid'],result
        forward=straight.to_frenet(result['xy'])
        assert forward['s_wrapped']==pytest.approx(s,abs=1e-4)
        assert forward['d']==pytest.approx(.2,abs=1e-4)
    for s in (-.01,5.01):
        assert straight.to_cartesian(s,0)['reason']=='outside_reference'
        with pytest.raises(ValueError):query_section(straight,s)
    section=query_section(straight,5)
    assert section['s_m']==pytest.approx(5) and not section['closed']
    assert section['frenet_ranges']


def test_open_tracker_keeps_zero_lap_count(straight):
    tracker=FrenetTracker(straight)
    for i,s in enumerate((4.6,4.7,4.8,4.9)):
        result=tracker.update(straight.reference(s)['xy'],0,1+i*.1)
        assert result['valid'],result
        assert result['wrap_count']==0
        assert result['s_unwrapped']==pytest.approx(s,abs=1e-4)


@pytest.mark.parametrize('side_branch',[False,True])
def test_open_l_corridor_orientation_save_reload_and_audit(tmp_path,side_branch):
    gray=np.zeros((120,80),np.uint8)
    gray[10:110,45:65]=255;gray[10:30,10:65]=255
    if side_branch:
        gray[60:75,25:45]=255
    raster=MapRaster(gray,dict(resolution=.05,origin=[0,0,0],negate=0,free_thresh=.2,occupied_thresh=.65))
    build=build_open_geometry(raster,[2.75,.7])
    assert not build['curve'].closed
    assert build['xy'][0,1]<build['xy'][-1,1]
    assert build['xy'][0,0]>build['xy'][-1,0]
    assert all(b['valid'] for b in build['boundaries'])
    image=tmp_path/'map.png';Image.fromarray(gray).save(image)
    yaml=tmp_path/'map.yaml';yaml.write_text('image: map.png\nresolution: 0.05\norigin: [0,0,0]\nnegate: 0\nfree_thresh: 0.2\noccupied_thresh: 0.65\n')
    from f1tenth_benchmarks.research.core.geometry.map_raster import load_map
    loaded_raster=load_map(yaml)
    directory=save_geometry(loaded_raster,build,tmp_path/'geometry','l-corridor',dict(stage_b_passed=True))
    assert verify_artifacts(directory)['passed']
    geometry=load_geometry(directory,yaml,inspect=True)
    assert not geometry.curve.closed and audit_geometry(geometry,directory)['passed']
    # The primary skeleton path selects the reference, not a cropped free mask.
    assert np.array_equal(geometry.corridor,build['corridor'])
    if side_branch:
        assert build['options']['skeleton_endpoints']>=3
        assert build['corridor'][65,30] and geometry.corridor[65,30]
    assert geometry.reference(geometry.curve.L)['xy']==pytest.approx(build['xy'][-1])
    curve=json.loads((directory/'reference_curve.json').read_text());assert curve['periodic'] is False


def test_open_dense_report_preserves_terminal_section_and_interval(straight):
    report=validate(straight,.05,0)
    assert report['numeric_checks_passed'],report['failures']
    terminal=next(p for p in report['sections'] if p['s_m']==straight.curve.L)
    boundary=straight.boundaries(straight.curve.L)
    assert terminal['boundary']['left_xy']==pytest.approx(boundary['left_xy'])
    assert terminal['boundary']['right_xy']==pytest.approx(boundary['right_xy'])
    assert terminal['boundary']['left_xy']!=pytest.approx(straight.boundaries(0)['left_xy'])
    final_leaf=report['boundary_segments'][-1]
    assert final_leaf['s_end']==straight.curve.L
    assert final_leaf['midpoint_error_m']<1e-8
    assert final_leaf['sampled_tolerance_passed']
    assert report['dense_sections']==101
    assert check(straight,8)['sampled_check_passed']


@pytest.mark.parametrize('validator',[check,validate])
def test_open_validation_does_not_hide_projection_to_other_endpoint(straight,monkeypatch,validator):
    forward=straight.to_frenet
    inverse=straight.to_cartesian
    queried=[]

    def wrong_endpoint(xy,yaw=None):
        result=forward(xy,yaw)
        if result.get('valid') and result['s_wrapped']<1e-8:
            return dict(result,s_wrapped=straight.curve.L)
        return result

    def independent_inverse(s,d,e_psi=None):
        # Keep inverse/source acceptance intact to isolate a forward regression.
        monkeypatch.setattr(straight,'to_frenet',forward)
        try:
            queried.append(s)
            return inverse(s,d,e_psi)
        finally:
            monkeypatch.setattr(straight,'to_frenet',wrong_endpoint)

    monkeypatch.setattr(straight,'to_frenet',wrong_endpoint)
    monkeypatch.setattr(straight,'to_cartesian',independent_inverse)
    report=check(straight,8) if validator is check else validate(straight,.05,0)
    assert straight.curve.L in queried
    if validator is check:
        assert not report['sampled_check_passed']
        assert report['max_s_error_m']==pytest.approx(straight.curve.L)
        assert any(p['reason']=='source_coordinate_mismatch' and p['s_error_m']>4
            for p in report['failures'])
    else:
        assert not report['numeric_checks_passed']
        assert report['max_errors']['s_m']==pytest.approx(straight.curve.L)
        assert any(p['reason']=='roundtrip_error' and p['errors'][1]>4
            for p in report['failures'])


def test_open_diagnostics_separate_endpoint_issues_and_keep_terminal_s(straight):
    positions=[0.,2.5,straight.curve.L]
    report=dict(numeric_checks_passed=True,dense_step_m=.05,rejected_queries=2,
        sections=[dict(s_m=s,boundary=dict(valid=True)) for s in positions],
        rejected_points=[dict(s_m=s,d_m=.2,xy=straight.reference(s)['xy'],
            reason='ambiguous_projection') for s in (0.,straight.curve.L)],
        boundary_segments=[dict(s_start=a,s_end=b,sampled_tolerance_passed=passed,
            midpoint_error_m=.03,quarter_errors_m=[.02,.04])
            for a,b,passed in ((0.,1.,False),(1.,4.,True),(4.,straight.curve.L,False))])
    result=summarize(straight,report,dict(passed=True))
    assert result['issue_groups']==4
    assert all(not issue['crosses_seam'] for issue in result['issues'])
    low=[p for p in result['issues'] if p['category']=='ambiguous_projection']
    assert [(p['s_start_m'],p['s_end_m']) for p in low]==[(0.,0.),(straight.curve.L,straight.curve.L)]
    leaves=[p for p in result['issues'] if p['category']=='boundary_interpolation']
    assert [(p['s_start_m'],p['s_end_m']) for p in leaves]==[(0.,1.),(4.,straight.curve.L)]
    assert leaves[-1]['samples'][0]['s_m']==pytest.approx(4.5)
    assert leaves[-1]['center_xy'][-1]==pytest.approx(straight.reference(straight.curve.L)['xy'])
    assert 'open endpoints separate' in result['coverage']['grouping']


def test_open_http_rejects_s_beyond_terminal(straight):
    inspector=MapInspector(straight);assert inspector.map_data()['closed'] is False
    server=make_server([inspector],0);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base='http://127.0.0.1:'+str(server.server_port)
    try:
        with urlopen(base+'/api/section?s=5') as r:assert json.load(r)['s_m']==pytest.approx(5)
        with pytest.raises(HTTPError) as error:urlopen(base+'/api/section?s=5.01')
        assert error.value.code==400
        with pytest.raises(HTTPError):urlopen(base+'/api/inverse?s=-0.1&d=0')
    finally:server.shutdown();thread.join();server.server_close()


@pytest.mark.parametrize('compressed',[False,True])
@pytest.mark.parametrize('shared_csv',[False,True])
def test_range_bundle_hash_source_binding_and_map_rendering(straight,tmp_path,compressed,shared_csv):
    straight.raster.source_hashes=dict(yaml='yaml-hash',image='image-hash')
    summary=dict(planning_allowed=False,continuous_verified=False)
    payload=dict(schema_version=1,geometry_id=straight.geometry_id,summary=summary,
        sections=[query_section(straight,2)],geometry_audit=dict(passed=True,rows=1))
    filename='section_ranges.json.gz' if compressed else 'section_ranges.json'
    raw=json_bytes(payload)
    (tmp_path/filename).write_bytes(gzip.compress(raw,mtime=0) if compressed else raw)
    (tmp_path/'report.json').write_bytes(json_bytes(summary))
    geometry_path=tmp_path/'source_geometry' if shared_csv else None
    csv_root=geometry_path if shared_csv else tmp_path
    if shared_csv:
        geometry_path.mkdir()
        (geometry_path/'manifest.json').write_text('{}')
    for name in ('centerline.csv','boundaries.csv'):(csv_root/name).write_text('fixture')
    manifest=dict(schema_version=1,geometry_id=straight.geometry_id,source_hashes=straight.raster.source_hashes.copy(),
        file_hashes={name:sha256((csv_root if name.endswith('.csv') else tmp_path)/name)
            for name in (filename,'report.json','centerline.csv','boundaries.csv')})
    if shared_csv:
        manifest['csv_location']='source_geometry'
        manifest['source_hashes']['geometry_manifest']=sha256(geometry_path/'manifest.json')
    manifest['bundle_id']=hashlib.sha256(json_bytes(manifest)).hexdigest()
    (tmp_path/'manifest.json').write_bytes(json_bytes(manifest))
    loaded=load_ranges(tmp_path,straight,geometry_path)
    assert loaded==payload
    if shared_csv:
        with pytest.raises(ValueError,match='geometry path required'):load_ranges(tmp_path,straight)
    assert MapInspector(straight,ranges=loaded).map_data()['range_summary']==summary
    straight.raster.source_hashes['image']='wrong-source'
    with pytest.raises(ValueError,match='source map'):load_ranges(tmp_path,straight,geometry_path)
    straight.raster.source_hashes['image']='image-hash'
    (tmp_path/filename).write_text('{}')
    with pytest.raises(ValueError,match='hash'):load_ranges(tmp_path,straight,geometry_path)


def test_mixed_section_reasons_remain_in_both_filters(straight):
    section=query_section(straight,2)
    for sample,reason in zip(section['samples'],('singular_frenet','ambiguous_projection')):
        sample.update(valid=False,reason=reason)
    ranges=dict(geometry_id=straight.geometry_id,sections=[section],summary={})
    issues=MapInspector(straight,ranges=ranges).map_data()['issues']
    assert {p['category'] for p in issues}=={'singular_frenet','ambiguous_projection'}
    for issue in issues:assert all(s['reason']==issue['category'] for s in issue['samples'])
