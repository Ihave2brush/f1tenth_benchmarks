"""Tests for the GitHub V2 Hermite paths and V3 sampled validator."""
import json
import math
import subprocess
import sys

import numpy as np
import pytest
from scipy import interpolate

from f1tenth_benchmarks.research.core.geometry import MapRaster, TrackGeometry, load_geometry
from f1tenth_benchmarks.research.core.geometry.extraction import ReferenceCurve
from f1tenth_benchmarks.research.overtaking.candidate_paths import generate_candidate_paths, hermite_lateral
from f1tenth_benchmarks.research.overtaking.candidate_validation import validate_candidate_paths


def make_straight(obstacle=False, narrowing=False):
    gray = np.zeros((60, 80), np.uint8)
    gray[10:50, 5:75] = 255
    if obstacle:
        gray[29:31, 29:32] = 0
    if narrowing:
        gray[10:25, 28:33] = 0
        gray[35:50, 28:33] = 0
    raster = MapRaster(gray, dict(resolution=.1, origin=[0, 0, 0], negate=0,
                                  free_thresh=.2, occupied_thresh=.65))
    points = np.array([[1, 3], [2, 3], [4, 3], [6, 3]], float)
    tck, _ = interpolate.splprep(points.T, s=0, k=3, per=False)
    return TrackGeometry(ReferenceCurve(tck, closed=False), raster,
                         'test-straight', points[0])


@pytest.fixture
def straight():
    return make_straight()


def test_hermite_endpoints_and_zero_endpoint_slopes():
    t = np.linspace(0, 1, 11)
    d = hermite_lateral(.1, .8, t)
    assert d[0] == pytest.approx(.1) and d[-1] == pytest.approx(.8)
    assert np.all(np.diff(d) > 0)
    h = 1e-6
    assert (hermite_lateral(.1, .8, h)-.1)/h == pytest.approx(0, abs=1e-5)
    assert (.8-hermite_lateral(.1, .8, 1-h))/h == pytest.approx(0, abs=1e-5)


def test_v2_metadata_and_endpoints(straight):
    result = generate_candidate_paths(straight, 1.5, 3.1, lookahead_m=4)
    assert result['generated_path_count'] == 5
    for path in result['paths']:
        assert path['geometry_id'] == straight.geometry_id
        assert path['path_valid'] is None and path['status'] == 'generated_unvalidated'
        assert len(path['points']) == 20
        assert path['points'][0]['d'] == pytest.approx(.1)
        assert path['points'][-1]['d'] == pytest.approx(path['target_d'])


@pytest.mark.parametrize('num_points', [True, 1, 2.5])
def test_v2_invalid_point_counts(straight, num_points):
    with pytest.raises(ValueError):
        generate_candidate_paths(straight, 1.5, 3, lookahead_m=3, num_points=num_points)


def test_v3_valid_path_metadata_and_density(straight):
    result = validate_candidate_paths(straight, 1.5, 3, lookahead_m=4, sample_step_m=.3)
    assert result['sampled_valid_count'] == 5
    assert result['actual_sample_step_m'] <= .3
    assert result['verification_level'] == 'sampled_requires_exact_query'
    for flag in ('planning_allowed', 'continuous_verified', 'footprint_verified',
                 'dynamics_verified', 'opponents_verified', 'heading_verified'):
        assert result[flag] is False
    for path in result['paths']:
        assert len(path['points']) == 15 and path['first_invalid_sample'] is None
        assert path['points'][0]['x'] == pytest.approx(1.5)
        assert path['points'][-1]['s'] == pytest.approx(path['target_s'])
        assert all(p['valid'] and p['geometry_valid'] and p['margin_valid'] for p in path['points'])
    json.dumps(result, allow_nan=False)


@pytest.mark.parametrize('kwargs', [
    {'lookahead_m': 0}, {'lookahead_m': -1}, {'lookahead_m': float('nan')},
    {'sample_step_m': 0}, {'sample_step_m': float('inf')}, {'sample_step_m': 1e-320},
    {'sample_step_m': 1e-7}, {'safety_margin_m': -1},
    {'safety_margin_m': float('nan')}, {'safety_margin_m': 100},
    {'yaw': float('nan')}, {'lookahead_m': 6},
])
def test_invalid_inputs(straight, kwargs):
    args = dict(lookahead_m=3)
    args.update(kwargs)
    with pytest.raises(ValueError):
        validate_candidate_paths(straight, 1.5, 3, **args)


def test_midpath_obstacle_with_valid_endpoint():
    result = validate_candidate_paths(make_straight(obstacle=True), 1.5, 3,
                                     lookahead_m=4, sample_step_m=.1)
    center = result['paths'][2]
    assert center['target_valid'] and not center['sampled_valid']
    assert 0 < center['first_invalid_sample'] < len(center['points'])-1
    assert any(not p['geometry_valid'] and p['geometry_reason'] == 'occupied'
               for p in center['points'])


def test_variable_width_and_initial_margin():
    narrowed = validate_candidate_paths(make_straight(narrowing=True), 1.5, 3,
                                         lookahead_m=4, sample_step_m=.1)
    assert narrowed['valid_target_count'] == 5
    assert narrowed['sampled_valid_count'] < 5
    result = validate_candidate_paths(make_straight(), 1.5, 4.9, lookahead_m=3)
    assert result['sampled_valid_count'] == 0
    assert all(p['first_invalid_sample'] == 0 for p in result['paths'])


@pytest.mark.parametrize('name', ['esp', 'aut', 'gbr', 'mco', 'CornerHall', 'my_map'])
def test_six_maps_against_exact_queries(name):
    g = load_geometry(f'maps/frenet/{name}/geometry', f'maps/{name}.yaml', inspect=True)
    r = validate_candidate_paths(g, *g.reference(1)['xy'], lookahead_m=5, sample_step_m=.5)
    assert r['sampled_valid_count'] == sum(p['sampled_valid'] for p in r['paths'])
    for path in r['paths']:
        for p in path['points']:
            exact = g.to_cartesian(p['s'], p['d'])
            assert p['geometry_valid'] == exact['valid']
            assert p['valid'] == (p['boundary_valid'] and p['margin_valid'] and exact['valid'])
    json.dumps(r, allow_nan=False)


def test_closed_seam_and_lap_limit():
    g = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
    xy = g.reference(g.curve.L-1)['xy']
    r = validate_candidate_paths(g, *xy, lookahead_m=3, sample_step_m=.5)
    for path in r['paths']:
        assert path['points'][-1]['s'] == pytest.approx(2, abs=1e-4)
        assert path['points'][-1]['s_progress'] > g.curve.L
        assert np.all(np.diff([p['s_progress'] for p in path['points']]) > 0)
    with pytest.raises(ValueError):
        validate_candidate_paths(g, *xy, lookahead_m=g.curve.L)


def test_invalid_target_preserved_and_version_mismatch(straight, monkeypatch):
    import f1tenth_benchmarks.research.overtaking.candidate_validation as module
    base = generate_candidate_paths(straight, 1.5, 3, lookahead_m=3)
    base['paths'][0].update(target_valid=False, target_reason='ambiguous_projection')
    monkeypatch.setattr(module, 'generate_candidate_paths', lambda **kwargs: base)
    r = validate_candidate_paths(straight, 1.5, 3, lookahead_m=3)
    p = r['paths'][0]
    assert p['status'] == 'invalid_target' and not p['sampled_valid']
    assert p['reason'] == 'ambiguous_projection' and p['points'] == []
    base['paths'][1]['geometry_id'] = 'other'
    with pytest.raises(ValueError, match='mismatch'):
        validate_candidate_paths(straight, 1.5, 3, lookahead_m=3)


def test_bad_exact_coordinates_are_rejected(straight, monkeypatch):
    original = straight.to_cartesian
    def query(s, d):
        return dict(original(s, d), xy=None)
    # V1 calls to_cartesian too; validity remains true but V3 must reject xy=None.
    monkeypatch.setattr(straight, 'to_cartesian', query)
    r = validate_candidate_paths(straight, 1.5, 3, lookahead_m=3)
    assert r['sampled_valid_count'] == 0
    assert all(p['reason'] == 'invalid_geometry' for p in r['paths'])


def test_cli_summary_and_error():
    cmd = [sys.executable, '-B', '-m',
           'f1tenth_benchmarks.research.overtaking.candidate_validation',
           '--geometry-dir', 'maps/frenet/esp/geometry', '--map-yaml', 'maps/esp.yaml',
           '--x', '0', '--y', '0', '--lookahead', '5', '--sample-step', '1']
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout)['candidate_count'] == 5
    result = subprocess.run(cmd + ['--sample-step', '0'], capture_output=True, text=True, timeout=60)
    assert result.returncode == 2 and 'positive and finite' in result.stderr
