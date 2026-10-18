"""Direct original V5 API regressions; no continuous or dynamic safety claim."""
import json

import numpy as np
import pytest
from scipy.interpolate import splprep

from f1tenth_benchmarks.research.core.geometry import MapRaster, TrackGeometry, load_geometry
from f1tenth_benchmarks.research.core.geometry.extraction import ReferenceCurve
from f1tenth_benchmarks.research.overtaking.adaptive_candidate_paths import generate_adaptive_paths


@pytest.fixture
def straight():
    gray = np.zeros((60, 80), np.uint8)
    gray[10:50, 5:75] = 255
    raster = MapRaster(gray, dict(resolution=.1, origin=[0, 0, 0], negate=0,
                                  free_thresh=.2, occupied_thresh=.65))
    xy = np.array([[1, 3], [2, 3], [4, 3], [6, 3]], float)
    tck, _ = splprep(xy.T, s=0, k=3, per=False)
    return TrackGeometry(ReferenceCurve(tck, closed=False), raster, 'v5-review-straight', xy[0])


@pytest.fixture(scope='module')
def esp():
    return load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)


def test_open_paths_ids_metadata_and_geometry_roundtrip(straight):
    r = generate_adaptive_paths(straight, 1.5, 3.1, lookahead_m=4, sample_step_m=.25)
    assert [p['candidate_id'] for p in r['paths']] == [1, 2, 3, 4, 5]
    assert r['candidate_count'] == 5 and r['sampled_valid_count'] == 5
    assert r['geometry_id'] == straight.geometry_id and r['frame_id'] == 'map'
    assert r['inspection_only'] is True and r['planning_allowed'] is False
    for flag in ['domain_verified', 'continuous_verified', 'footprint_verified',
                 'dynamics_verified', 'opponents_verified', 'heading_verified']:
        assert r[flag] is False
    for path in r['paths']:
        assert path['geometry_id'] == r['geometry_id'] and path['frame_id'] == 'map'
        assert path['sampled_valid'] and path['first_invalid_sample'] is None
        assert path['reason'] is None and path['status'] == 'sampled_valid'
        assert [path['points'][0]['x'], path['points'][0]['y']] == pytest.approx([1.5, 3.1])
        for p in path['points']:
            assert p['s_progress'] == pytest.approx(r['start']['s'] + p['t'] * 4)
            assert p['s'] == p['s_progress']
            exact = straight.to_cartesian(p['s'], p['d'])
            assert exact['valid'] and p['valid'] and p['reason'] is None
            assert [p['x'], p['y']] == pytest.approx(exact['xy'])
            frenet = straight.to_frenet([p['x'], p['y']])
            assert frenet['valid']
            assert [frenet['s_wrapped'], frenet['d']] == pytest.approx([p['s'], p['d']], abs=1e-4)
            assert p['d_min'] <= p['d'] <= p['d_max']
            assert p['boundary_valid'] and p['geometry_valid'] and p['margin_valid']
        end = path['points'][-1]
        assert end['d'] == pytest.approx(end['d_min'] + path['target_fraction']*(end['d_max']-end['d_min']))
    json.dumps(r, allow_nan=False)


def test_closed_normal_horizon_and_seam_coordinates(esp):
    r = generate_adaptive_paths(esp, 0., 0., lookahead_m=1., sample_step_m=.25)
    assert len(r['paths']) == 5
    for path in r['paths']:
        assert any(b['s'] < a['s'] for a, b in zip(path['points'], path['points'][1:]))
        for p in path['points']:
            assert p['s'] == pytest.approx(p['s_progress'] % esp.curve.L)
            exact = esp.to_cartesian(p['s'], p['d'])
            assert p['geometry_valid'] == exact['valid']
            if exact.get('xy') is not None:
                assert [p['x'], p['y']] == pytest.approx(exact['xy'])


@pytest.mark.parametrize('extra', [0., .1])
def test_closed_one_lap_and_beyond_rejected(esp, extra):
    with pytest.raises(ValueError, match='shorter than one full lap'):
        generate_adaptive_paths(esp, 0., 0., lookahead_m=esp.curve.L + extra)


def test_open_exact_end_and_beyond(straight):
    s0 = straight.to_frenet([1.5, 3.])['s_wrapped']
    length = straight.curve.L - s0
    r = generate_adaptive_paths(straight, 1.5, 3., lookahead_m=length)
    assert r['paths'][0]['points'][-1]['s'] == pytest.approx(straight.curve.L)
    with pytest.raises(ValueError, match='open reference line'):
        generate_adaptive_paths(straight, 1.5, 3., lookahead_m=length + .01)


@pytest.mark.parametrize('side', ['d_left', 'd_right'])
@pytest.mark.parametrize('value', [None, np.nan, np.inf, -np.inf])
def test_invalid_initial_boundary_values_raise_value_error(straight, monkeypatch, side, value):
    # Isolate V5's boundary handling after a successful original projection.
    start = straight.to_frenet([1.5, 3.])
    monkeypatch.setattr(straight, 'to_frenet', lambda xy, yaw=None: dict(start))
    original = straight.boundaries
    def invalid(s):
        return dict(original(s), **{side: value})
    monkeypatch.setattr(straight, 'boundaries', invalid)
    with pytest.raises(ValueError, match='Invalid road boundary at vehicle start'):
        generate_adaptive_paths(straight, 1.5, 3., lookahead_m=4)


@pytest.mark.parametrize('mode', ['invalid', 'missing', 'narrow', 'outside'])
def test_initial_boundary_rejections(straight, monkeypatch, mode):
    start = straight.to_frenet([1.5, 3.])
    monkeypatch.setattr(straight, 'to_frenet', lambda xy, yaw=None: dict(start))
    original = straight.boundaries
    def changed(s):
        b = dict(original(s))
        if mode == 'invalid': b.update(valid=False, reason='unknown')
        elif mode == 'missing': b.pop('d_left')
        elif mode == 'narrow': b.update(d_left=.1, d_right=-.1)
        else: b.update(d_left=2., d_right=1.)
        return b
    monkeypatch.setattr(straight, 'boundaries', changed)
    with pytest.raises(ValueError):
        generate_adaptive_paths(straight, 1.5, 3., lookahead_m=4)


@pytest.mark.parametrize('mode', ['invalid', 'missing', 'narrow'])
def test_invalid_interior_section_preserves_all_ids_and_diagnostics(straight, monkeypatch, mode):
    original = straight.boundaries
    def changed(s):
        b = dict(original(s))
        if s > 1.:
            if mode == 'invalid': b.update(valid=False, reason='unknown')
            elif mode == 'missing': b.update(d_left=None)
            else: b.update(d_left=.1, d_right=-.1)
        return b
    monkeypatch.setattr(straight, 'boundaries', changed)
    r = generate_adaptive_paths(straight, 1.5, 3., lookahead_m=4, sample_step_m=.25)
    assert [p['candidate_id'] for p in r['paths']] == [1, 2, 3, 4, 5]
    assert r['sampled_valid_count'] == 0
    reason = {'invalid': 'unknown', 'missing': 'invalid_boundary', 'narrow': 'insufficient_lateral_margin'}[mode]
    for path in r['paths']:
        assert not path['sampled_valid'] and path['status'] == 'invalid_sample'
        i = path['first_invalid_sample']; p = path['points'][i]
        assert i > 0 and all(q['valid'] for q in path['points'][:i])
        assert not p['valid'] and not p['boundary_valid']
        assert p['reason'] == path['reason'] == reason
        assert p['d'] is None and p['x'] is None and p['y'] is None
    json.dumps(r, allow_nan=False)


@pytest.mark.parametrize('missing_xy', [False, True])
def test_exact_geometry_failure_keeps_reason_and_rejects_path(straight, monkeypatch, missing_xy):
    original = straight.to_cartesian
    def invalid(s, d):
        result = original(s, d)
        if s > 1.:
            result = dict(result, valid=False, reason='occupied')
            if missing_xy: result['xy'] = None
        return result
    monkeypatch.setattr(straight, 'to_cartesian', invalid)
    r = generate_adaptive_paths(straight, 1.5, 3., lookahead_m=4)
    for path in r['paths']:
        p = path['points'][path['first_invalid_sample']]
        assert not path['sampled_valid'] and path['reason'] == 'occupied'
        assert not p['valid'] and not p['geometry_valid'] and p['reason'] == 'occupied'
        assert p['boundary_valid'] and p['margin_valid']
        assert (p['x'] is None and p['y'] is None) == missing_xy


@pytest.mark.parametrize('lookahead', [0., -1., np.nan, np.inf])
def test_invalid_horizons_rejected(straight, lookahead):
    with pytest.raises(ValueError, match='positive and finite'):
        generate_adaptive_paths(straight, 1.5, 3., lookahead_m=lookahead)
