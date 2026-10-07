"""Independent inspector coordinates, HTTP routes and dense validation checks."""
import json
from pathlib import Path
import threading
from urllib.error import HTTPError
from urllib.request import urlopen

import numpy as np
import pytest
from scipy import interpolate

from f1tenth_benchmarks.research.core.geometry import MapRaster,TrackGeometry
from f1tenth_benchmarks.research.core.geometry.extraction import ReferenceCurve
from f1tenth_benchmarks.research.core.geometry.validate_stage_d import validate,save_report,load_report
from f1tenth_benchmarks.research.core.geometry.viewer import MapInspector,make_server
from f1tenth_benchmarks.research.core.geometry.section_ranges import query_section,sample_runs
from f1tenth_benchmarks.research.core.geometry.complete_stage_d import (
    cyclic_runs,audit_geometry,summarize,save_diagnostics,load_diagnostics)


@pytest.fixture(scope='module')
def circle():
    row,col=np.indices((80,80));r=np.hypot((col+.5)*.05-2,(80-row-.5)*.05-2)
    raster=MapRaster(np.where((r>.6)&(r<1.4),255,0).astype(np.uint8),
        dict(resolution=.05,origin=[-3,5,.4],negate=0,free_thresh=.2,occupied_thresh=.65))
    theta=np.linspace(0,2*np.pi,201)
    xy=raster.local_to_map(np.array([2+np.cos(theta),2+np.sin(theta)]).T)
    tck,_=interpolate.splprep(xy.T,s=0,per=True)
    return TrackGeometry(ReferenceCurve(tck),raster,'inspector-circle',xy[0])


def test_rotated_pixel_query_and_inverse_are_same_core(circle):
    inspector=MapInspector(circle)
    xy=circle.raster.local_to_map([3,2]);pixel=circle.raster.map_to_pixel(xy)
    q=inspector.query_pixel(*pixel.pixel)
    assert q['frenet']['valid'] and q['frenet']['s_wrapped']==pytest.approx(0,abs=1e-4)
    np.testing.assert_allclose(q['xy'],xy,atol=1e-10)
    assert q['query_pixel']==pytest.approx([60,40])
    assert q['reconstruction_error_m']<1e-4
    inverse=inspector.query_sd(circle.curve.L-1e-6,.1)
    assert inverse['source_frenet']['valid'] and inverse['frenet']['valid']
    assert inverse['frenet']['d']==pytest.approx(.1,abs=1e-4)


def test_occupancy_outside_and_nonfinite_are_visible(circle):
    inspector=MapInspector(circle)
    assert inspector.query_pixel(0,0)['frenet']['reason']=='occupied'
    assert inspector.query_pixel(-2,20)['occupancy']=='outside_map'
    with pytest.raises(ValueError):inspector.query_pixel(float('nan'),0)
    with pytest.raises(ValueError):inspector.query_sd(float('inf'),0)
    result=inspector.query_sd(0,.6)
    assert not result['source_frenet']['valid']
    assert result['reconstructed_xy'] is None


def test_report_cannot_be_used_with_another_geometry(circle):
    with pytest.raises(ValueError):MapInspector(circle,dict(geometry_id='wrong'))


def test_section_ranges_preserve_road_and_require_exact_checks(circle):
    result=query_section(circle,circle.curve.L)
    assert result['s_m']==pytest.approx(0)
    road=result['road_range'];assert road['d_min_m']<0<road['d_max_m']
    assert result['frenet_ranges'] and not result['continuous_verified']
    assert result['exact_query_required'] and not result['planning_allowed']
    for sample in result['samples']:
        if sample['valid']:
            projected=circle.to_frenet(sample['xy'])
            assert projected['valid']
            assert projected['d']==pytest.approx(sample['d_m'],abs=1e-4)
    assert MapInspector(circle).query_section(0)['road_pixels']
    for kwargs in ({'s':float('nan')},{'s':0,'d_step_m':.1},{'s':0,'transition_tolerance_m':0}):
        with pytest.raises(ValueError):query_section(circle,**kwargs)


def test_section_runs_do_not_bridge_rejected_samples():
    samples=[dict(d_m=i,valid=i!=2,reason=None if i!=2 else 'ambiguous_projection') for i in range(5)]
    runs=sample_runs(samples,True)
    assert [(r['d_min_m'],r['d_max_m']) for r in runs]==[(0,1),(3,4)]
    assert all(not r['continuous_verified'] for r in runs)


def test_section_retains_physical_width_when_inner_circle_has_low_j(circle):
    row,col=np.indices((80,80));radius=np.hypot((col+.5)*.05-2,(80-row-.5)*.05-2)
    raster=MapRaster(np.where((radius>.1)&(radius<1.4),255,0).astype(np.uint8),
        dict(resolution=.05,origin=[-3,5,.4],negate=0,free_thresh=.2,occupied_thresh=.65))
    wide=TrackGeometry(circle.curve,raster,'wide-circle',circle.reference(0)['xy'])
    result=query_section(wide,0)
    assert result['road_range']['d_max_m']>.85
    assert result['frenet_ranges'][0]['d_max_m']==pytest.approx(.8,abs=.002)
    assert result['limited_ranges'][-1]['reasons'].get('singular_frenet')


def test_invalid_section_boundary_has_no_fabricated_ranges(circle,monkeypatch):
    monkeypatch.setattr(circle,'boundaries',lambda s:dict(valid=False,reason='missing_wall'))
    result=query_section(circle,0)
    assert not result['range_query_valid'] and result['reason']=='missing_wall'
    assert result['road_range'] is None and not result['frenet_ranges'] and not result['samples']


def test_loopback_http_routes_switching_and_errors(circle):
    server=make_server([MapInspector(circle,label='first'),MapInspector(circle,label='second')],0)
    thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
    base='http://127.0.0.1:'+str(server.server_port)
    try:
        assert server.server_address[0]=='127.0.0.1'
        with urlopen(base+'/') as r:assert 'Frenet'.encode() in r.read()
        with urlopen(base+'/api/catalog') as r:assert len(json.load(r))==2
        with urlopen(base+'/api/map?map=1') as r:assert json.load(r)['label']=='second'
        with urlopen(base+'/api/image?map=0') as r:assert r.read().startswith(b'\x89PNG')
        with urlopen(base+'/api/query?map=1&c=59.5&v=39.5') as r:assert json.load(r)['frenet']['valid']
        with urlopen(base+'/api/inverse?map=0&s=0&d=0') as r:assert json.load(r)['source_frenet']['valid']
        with urlopen(base+'/api/section?map=1&s=0') as r:assert json.load(r)['frenet_ranges']
        for path in ('/api/query?c=nan&v=0','/api/map?map=-1','/api/query?c=0','/../../README.md'):
            with pytest.raises(HTTPError) as exc:urlopen(base+path)
            assert exc.value.code in (400,404)
    finally:server.shutdown();thread.join();server.server_close()


def test_dense_report_independent_circle_and_hash_integrity(circle,tmp_path):
    report=validate(circle,.05,1)
    assert report['numeric_checks_passed'],report['failures']
    assert report['dense_step_m']<=.05 and report['total_sections']>report['dense_sections']
    assert report['max_errors']['s_m']<1e-4 and report['max_errors']['d_m']<1e-4
    assert report['domain_verified'] is False and report['planning_allowed'] is False
    assert not report['boundary_interpolation']['interpolation_allowed']
    assert all(not s['continuous_verified'] for s in report['boundary_segments'])
    geometry_dir=tmp_path/'g';geometry_dir.mkdir();(geometry_dir/'manifest.json').write_text('{}')
    yaml=tmp_path/'map.yaml';yaml.write_text('source')
    output=tmp_path/'report';save_report(report,geometry_dir,yaml,output)
    assert load_report(output,circle)['geometry_id']==circle.geometry_id
    with pytest.raises(FileExistsError):save_report(report,geometry_dir,yaml,output)
    (output/'report.json').write_text('{}')
    with pytest.raises(ValueError):load_report(output,circle)


@pytest.mark.parametrize('step,depth',[(.051,1),(0,1),(.05,-1),(.05,9)])
def test_invalid_dense_sampling_limits(circle,step,depth):
    with pytest.raises(ValueError):validate(circle,step,depth)


def test_diagnostic_runs_merge_seam_without_bridging_unchecked_sections():
    assert cyclic_runs([0,1,4,8,9],10)==[[8,9,0,1],[4]]
    assert cyclic_runs([],10)==[]
    assert cyclic_runs(range(4),4)==[[0,1,2,3]]


def test_saved_geometry_audit_detects_wrong_side_and_source(circle,tmp_path):
    import csv
    positions=np.linspace(0,circle.curve.L,20,endpoint=False)
    xy,_,psi,kappa=circle.curve.sample(positions)
    np.savetxt(tmp_path/'centerline.csv',np.column_stack((xy,positions,psi,kappa)),
        delimiter=',',header='x_m,y_m,s_m,psi_rad,kappa_inv_m',comments='')
    rows=[]
    for s in positions:
        b=circle.boundaries(s)
        rows.append([s,*b['left_xy'],b['d_left'],*b['right_xy'],b['d_right'],1])
    def write():
        with (tmp_path/'boundaries.csv').open('w') as stream:
            writer=csv.writer(stream)
            writer.writerow(['s_m','left_x_m','left_y_m','d_left_m','right_x_m','right_y_m','d_right_m','valid'])
            writer.writerows(rows)
    write()
    result=audit_geometry(circle,tmp_path)
    assert result['passed'] and result['complete_boundary_rows']==20
    assert result['direction']=='counterclockwise'
    rows[3][3]=-abs(rows[3][3]);rows[5][0]+=.01;write()
    result=audit_geometry(circle,tmp_path)
    assert not result['passed']
    assert {p['row'] for p in result['failures']}=={3,5}


def test_diagnostic_categories_source_provenance_and_viewer(circle,tmp_path):
    positions=np.arange(5)*circle.curve.L/5
    report=dict(geometry_id=circle.geometry_id,numeric_checks_passed=True,
        dense_step_m=.05,rejected_queries=2,
        sections=[dict(s_m=float(s),boundary=dict(valid=True)) for s in positions],
        rejected_points=[dict(s_m=float(positions[i]),d_m=.85,
            xy=circle.to_cartesian(positions[i],.85)['xy'],reason='singular_frenet') for i in (0,4)],
        boundary_segments=[dict(s_start=0.,s_end=.05,sampled_tolerance_passed=False,
            midpoint_error_m=.03,quarter_errors_m=[.02,.04])])
    result=summarize(circle,report,dict(passed=True))
    assert result['diagnostic_ready'] and result['issue_groups']==2
    low=next(p for p in result['issues'] if p['category']=='singular_frenet')
    assert low['crosses_seam'] and low['sample_count']==2
    assert all(not p['source_valid'] and p['J']<.2 for p in low['samples'])
    assert not result['planning_allowed'] and not low['continuous_verified']
    payload=MapInspector(circle,report,diagnostics=result).map_data()
    assert payload['issues'][0]['center_pixels'] and payload['issues'][0]['samples'][0]['pixel']
    assert MapInspector(circle).map_data()['issues']==[]
    geometry_dir=tmp_path/'g';geometry_dir.mkdir();(geometry_dir/'manifest.json').write_text('{}')
    report_dir=tmp_path/'report';report_dir.mkdir();(report_dir/'report.json').write_text('{}')
    output=tmp_path/'diagnostics';save_diagnostics(result,geometry_dir,report_dir,output)
    assert load_diagnostics(output,circle,report_dir,geometry_dir)['issue_groups']==2
    (geometry_dir/'manifest.json').write_text('{"changed":true}')
    with pytest.raises(ValueError):load_diagnostics(output,circle,report_dir,geometry_dir)
    with pytest.raises(FileExistsError):save_diagnostics(result,geometry_dir,report_dir,output)
    (report_dir/'report.json').write_text('{"changed":true}')
    with pytest.raises(ValueError):load_diagnostics(output,circle,report_dir)
    (output/'diagnostics.json').write_text('{}')
    with pytest.raises(ValueError):load_diagnostics(output,circle)
