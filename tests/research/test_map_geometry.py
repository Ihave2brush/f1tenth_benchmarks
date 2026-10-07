"""Independent occupancy/coordinate fixtures and offline geometry contracts."""
import csv
import json
import math

import numpy as np
from PIL import Image
import pytest
from scipy import interpolate

from f1tenth_benchmarks.research.core.geometry.map_raster import MapRaster, load_map
from f1tenth_benchmarks.research.core.geometry.extraction import (
    ExtractionOptions, ReferenceCurve, corridor_loops, ray_boundary, build_geometry)
from f1tenth_benchmarks.research.core.geometry.build_map import (
    save_geometry, verify_artifacts, validate_candidate, intersections)


def config(**overrides):
    return dict(dict(resolution=.5, origin=[10., -3., 0.], negate=0,
                     free_thresh=.2, occupied_thresh=.65), **overrides)


def test_known_rotated_pixel_centres_and_no_half_pixel_bias():
    raster = MapRaster(np.full((3, 4), 255, np.uint8), config(origin=[10., -3., math.pi/2]))
    # Bottom-left cell centre local=(.25,.25); 90 degree turn is (-.25,.25).
    np.testing.assert_allclose(raster.pixel_to_map(0, 2), [9.75, -2.75], atol=1e-12)
    result = raster.map_to_pixel([9.75, -2.75])
    assert result.cell == (0, 2) and result.inside_map
    np.testing.assert_allclose(result.pixel, [0, 2], atol=1e-12)
    c, v = np.meshgrid(np.arange(4), np.arange(3))
    for point, col, row in zip(raster.pixel_to_map(c, v).reshape(-1, 2), c.ravel(), v.ravel()):
        assert raster.map_to_pixel(point).cell == (col, row)


def test_half_open_grid_edges_outside_and_nonfinite():
    raster = MapRaster(np.full((3, 4), 255, np.uint8), config())
    assert raster.map_to_pixel([10., -3.]).cell == (0, 2)
    assert raster.map_to_pixel([10.5, -2.5]).cell == (1, 1)
    for point in [[12., -3.], [10., -1.5], [9.999, -3.], [10., -3.001]]:
        assert raster.classify(point) == 'outside_map'
        assert raster.map_to_pixel(point).cell is None
    assert raster.classify([float('nan'), 0]) == 'nonfinite_input'
    with pytest.raises(ValueError):
        raster.pixel_to_map(float('inf'), 0)


def test_occupancy_negation_threshold_equality_and_transparency():
    gray = np.array([[0, 51, 128, 204, 255]], dtype=np.uint8)
    # Exact equality tested using representable thresholds computed independently.
    raster = MapRaster(gray, config(free_thresh=1-204/255, occupied_thresh=1-51/255))
    assert raster.occupancy.tolist() == [[1, -1, -1, -1, 0]]
    raster = MapRaster(gray, config(negate=1), alpha=np.array([[255,255,255,255,0]], np.uint8))
    assert raster.occupancy.tolist() == [[0, -1, -1, 1, -1]]


@pytest.mark.parametrize('update', [dict(resolution=0), dict(resolution=float('nan')),
    dict(origin=[0, 0]), dict(origin=[0, 0, float('inf')]), dict(negate=2),
    dict(negate=True), dict(free_thresh=.8), dict(mode='scale')])
def test_invalid_map_schema_rejected(update):
    with pytest.raises(ValueError):
        MapRaster(np.zeros((2, 2), np.uint8), config(**update))


def test_yaml_relative_image_and_unsupported_image_modes(tmp_path):
    image = tmp_path/'assets'; image.mkdir()
    Image.fromarray(np.full((2, 3), 255, np.uint8)).save(image/'map.png')
    path = tmp_path/'map.yaml'
    import yaml
    path.write_text(yaml.safe_dump(dict(config(), image='assets/map.png')))
    raster = load_map(path)
    assert raster.width == 3 and raster.image_path == image/'map.png'
    assert len(raster.source_hashes['yaml']) == 64
    color = np.zeros((2, 3, 3), np.uint8); color[:, :, 0] = 255
    Image.fromarray(color).save(image/'map.png')
    with pytest.raises(ValueError, match='non-grayscale'):
        load_map(path)
    Image.fromarray(np.ones((2, 3), np.uint16)*1000).save(image/'map.png')
    with pytest.raises(ValueError, match='unsupported image mode'):
        load_map(path)


def ring():
    gray = np.zeros((100, 100), np.uint8)
    gray[10:90, 10:90] = 255
    gray[30:70, 30:70] = 0
    return MapRaster(gray, config(resolution=.1, origin=[0,0,0]))


def test_annulus_selected_by_anchor_and_exact_grid_interfaces():
    raster = ring()
    mask, loops = corridor_loops(raster, [5, 2])
    assert mask.sum() == 80*80-40*40 and len(loops) == 2
    np.testing.assert_allclose(loops[0].min(axis=0), [1,1])
    np.testing.assert_allclose(loops[0].max(axis=0), [9,9])
    np.testing.assert_allclose(loops[1].min(axis=0), [3,3])
    np.testing.assert_allclose(loops[1].max(axis=0), [7,7])
    with pytest.raises(ValueError, match='anchor'):
        corridor_loops(raster, [5,5])
    raster.occupancy[15:18,15:18] = 1
    with pytest.raises(ValueError, match='one enclosed'):
        corridor_loops(raster, [5,2])


def test_open_or_map_edge_corridor_is_not_silently_closed():
    raster = MapRaster(np.full((10, 10), 255, np.uint8), config())
    with pytest.raises(ValueError, match='map edge'):
        corridor_loops(raster, raster.pixel_to_map(4,4))
    raster = ring()
    raster.occupancy[10:31,48:52] = 1
    with pytest.raises(ValueError, match='one enclosed'):
        corridor_loops(raster, [2,5])


def test_ray_hits_cell_faces_not_pixel_centres_and_rotates():
    raster = ring()
    hit, d, kind, reason = ray_boundary(raster, [5,2], [0,1])
    np.testing.assert_allclose(hit, [5,3], atol=1e-12)
    assert d == pytest.approx(1) and kind == 'occupied' and reason is None
    hit, d, _, _ = ray_boundary(raster, [5,2], [0,7])
    np.testing.assert_allclose(hit, [5,3], atol=1e-12)
    assert d == pytest.approx(1)
    with pytest.raises(ValueError):
        ray_boundary(raster, [5,2], [0,0])
    hit, d, _, _ = ray_boundary(raster, [5,2], [0,-1])
    np.testing.assert_allclose(hit, [5,1], atol=1e-12)
    raster = MapRaster(raster.gray, config(resolution=.1, origin=[4,-2,math.pi/2]))
    hit, d, _, _ = ray_boundary(raster, raster.local_to_map([5,2]), [-1,0])
    np.testing.assert_allclose(hit, raster.local_to_map([5,3]), atol=1e-12)
    assert d == pytest.approx(1)


def test_unknown_boundary_and_blocked_diagonal_corner():
    raster = ring(); raster.occupancy[70, 50] = -1
    _, _, kind, _ = ray_boundary(raster, [5.05,2], [0,1])
    assert kind == 'unknown'
    gray = np.full((5,5),255,np.uint8); gray[3,2] = 0
    raster = MapRaster(gray, config(resolution=1, origin=[0,0,0]))
    hit, _, kind, _ = ray_boundary(raster, [1.5,1.5], np.array([1,1])/np.sqrt(2))
    np.testing.assert_allclose(hit, [2,2])
    assert kind == 'occupied'


def test_circle_curve_physical_length_and_serialization():
    theta = np.linspace(0, 2*np.pi, 1001)
    points = np.column_stack((4*np.cos(theta), 4*np.sin(theta)))
    tck, _ = interpolate.splprep(points.T, s=0, per=True)
    curve = ReferenceCurve(tck, .2)
    assert curve.L == pytest.approx(8*np.pi, abs=1e-7)
    xy, _, _, kappa = curve.sample([0, curve.L/4, curve.L-1e-6])
    assert np.linalg.norm(xy[0]-xy[-1]) == pytest.approx(1e-6, abs=1e-8)
    np.testing.assert_allclose(kappa, .25, atol=1e-5)
    saved = curve.to_dict()
    rebuilt = ReferenceCurve((saved['knots'],saved['coefficients'],saved['degree']),saved['start_u'])
    np.testing.assert_array_equal(curve.sample([0,1,4])[0], rebuilt.sample([0,1,4])[0])


def test_intersection_detection_including_collinear_overlap():
    assert intersections(np.array([[0,0],[1,0],[1,1],[0,1]], float)) == 0
    assert intersections(np.array([[0,0],[1,1],[0,1],[1,0]], float)) > 0
    assert intersections(np.array([[0,0],[3,0],[3,1],[1,0],[2,0],[0,1]], float)) > 0


def test_candidate_save_hashes_ros_first_point_and_exclusive_output(tmp_path):
    raster = ring()
    build = build_geometry(raster, ExtractionOptions(output_step_m=.1), [5,2], 0.)
    report = validate_candidate(raster, build)
    assert report['stage_b_passed']
    assert not report['planning_allowed'] and report['later_checks']['frenet_roundtrip'] == 'not_run'
    target = save_geometry(raster, build, tmp_path, 'ring', report)
    assert verify_artifacts(target)['passed']
    with (target/'waypoints_ros.csv').open() as stream:
        rows = list(csv.reader(stream))
    assert rows[0] == ['x','y'] and len(rows)-1 == len(build['s'])
    np.testing.assert_array_equal(np.array(rows[1],float),build['xy'][0])
    with pytest.raises(FileExistsError):
        save_geometry(raster, build, tmp_path, 'ring', report)
    curve_path = target/'reference_curve.json'
    data = json.loads(curve_path.read_text()); data['start_u'] += .01
    curve_path.write_text(json.dumps(data))
    with pytest.raises(ValueError, match='geometry_mismatch'):
        verify_artifacts(target)


def test_extraction_options_and_anchor_mismatch_rejected():
    with pytest.raises(ValueError):
        ExtractionOptions(output_step_m=0).validate()
    with pytest.raises(ValueError, match='anchor_mismatch'):
        build_geometry(ring(), ExtractionOptions(), [5,1.2], 0.)


def test_variable_width_track_keeps_wide_sections_and_smooth_reference():
    # Independent analytic walls: radius 5 and radius 7+1.2*cos(theta).
    # The 0.8..3.2 m radial gap intentionally exceeds the old 1.4*median rule.
    row, col = np.indices((400,400))
    x = (col+.5)*.05-10
    y = (400-row-.5)*.05-10
    radius = np.hypot(x,y); theta = np.arctan2(y,x)
    gray = np.where((radius>5)&(radius<7+1.2*np.cos(theta)),255,0).astype(np.uint8)
    raster = MapRaster(gray, config(resolution=.05,origin=[-10,-10,0]))
    build = build_geometry(raster,ExtractionOptions(method='equidistant_periodic_v1'),[6.6,0],math.pi/2)
    report = validate_candidate(raster,build)
    assert report['stage_b_passed']
    widths = [b['d_left']-b['d_right'] for b in build['boundaries']]
    assert min(widths)<1.1 and max(widths)>2.9
    assert all(b['d_left']>0>b['d_right'] for b in build['boundaries'])
    # Genuine turns have scale ~6 m; raster noise must not create metre-scale cusps.
    assert report['curvature_abs_max_inv_m']<1.
    assert build['options']['method']=='equidistant_periodic_v1'


@pytest.mark.parametrize('update', [dict(centering_smoothing_m=0),
    dict(centering_smoothing_m=float('inf')),dict(method='unknown')])
def test_invalid_centerline_refinement_options_rejected(update):
    with pytest.raises(ValueError):
        ExtractionOptions(**update).validate()


def test_distance_reference_matches_independent_circular_wall_geometry():
    row,col = np.indices((320,320))
    x=(col+.5)*.05-8;y=(320-row-.5)*.05-8
    radius=np.hypot(x,y)
    gray=np.where((radius>5)&(radius<7),255,0).astype(np.uint8)
    raster=MapRaster(gray,config(resolution=.05,origin=[-8,-8,0]))
    build=build_geometry(raster,ExtractionOptions(method='equidistant_periodic_v1'),[6,0],math.pi/2)
    np.testing.assert_allclose(np.linalg.norm(build['xy'],axis=1),6,atol=.075,rtol=0)
    assert validate_candidate(raster,build)['stage_b_passed']


def test_refined_reference_respects_rotated_map_origin():
    options=ExtractionOptions(method='equidistant_periodic_v1')
    raster=ring();base=build_geometry(raster,options,[5,2],0)
    angle=.6
    rotated=MapRaster(raster.gray,config(resolution=.1,origin=[4,-2,angle]))
    other=build_geometry(rotated,options,rotated.local_to_map([5,2]),angle)
    s=np.linspace(0,base['curve'].L,64,endpoint=False)
    expected=rotated.local_to_map(base['curve'].sample(s)[0])
    np.testing.assert_allclose(other['curve'].sample(s)[0],expected,atol=1e-6,rtol=0)
