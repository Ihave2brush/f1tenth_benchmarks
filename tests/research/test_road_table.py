"""P2 conservative projection columns, exact fallback and versioned reuse."""
import json
from pathlib import Path

import numpy as np
import pytest

from test_robust_adaptive_paths import make_straight
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.overtaking.geometry_queries import FastQueryContext
from f1tenth_benchmarks.research.overtaking.road_table import RoadTable, get_road_table
from f1tenth_benchmarks.research.overtaking.robust_adaptive_paths import generate_robust_paths


def test_column_interval_skips_reverse_projection_but_keeps_xy(monkeypatch):
    geometry = make_straight()
    context = FastQueryContext(geometry)
    def forbidden(*args, **kwargs):
        pytest.fail('certified point must not do inverse projection')
    monkeypatch.setattr(context.geometry, 'to_frenet', forbidden)
    result = context.cartesian_many([1., 1.], [0., .2], [0., .1])
    assert all(r['valid'] for r in result)
    np.testing.assert_allclose(result[1]['xy'], [2., 3.2], atol=1e-12)
    assert context.table.stats['fast_accepts'] == 2


def test_uncertain_falls_back_and_invalid_result_is_reused():
    geometry = make_straight()
    context = FastQueryContext(geometry)
    first = context.cartesian_many([1.], [2.2], [.1])[0]
    assert not first['valid'] and first['reason'] == 'occupied'
    assert context.table.stats['exact_fallbacks'] == 1
    second = FastQueryContext(geometry).cartesian_many([1.], [2.2], [.2])[0]
    assert not second['valid'] and second['reason'] == first['reason']
    assert second['yaw'] != first['yaw']
    assert context.table.stats['exact_fallbacks'] == 1
    assert context.table.stats['point_hits'] == 1


def test_cache_reuse_across_generations_and_source_change_invalidates():
    geometry = make_straight()
    table = get_road_table(geometry)
    table.column(1.)
    assert get_road_table(geometry) is table
    get_road_table(geometry).column(1.)
    assert table.stats['column_hits'] == 1
    geometry.raster.occupancy[0, 0] = -1
    new = get_road_table(geometry)
    assert new is not table and new.fingerprint != table.fingerprint
    assert new.stats['invalidations'] == 1 and not new.columns


def test_custom_exact_rejection_is_not_bypassed_or_cached():
    geometry = make_straight()
    original = geometry.to_cartesian
    old = get_road_table(geometry)
    def reject(s, d, e_psi=None):
        result = original(s, d, e_psi)
        return dict(result, valid=False, reason='injected_exact_rejection')
    geometry.to_cartesian = reject
    table = get_road_table(geometry)
    assert table is not old and not table.standard
    r = generate_robust_paths(geometry, 1.5, 3., lookahead_m=3.,
                              max_refinements=0, query_mode='fast_with_fallback')
    assert r['sampled_valid_count'] == 0
    assert all(p['reason'] == 'injected_exact_rejection' for p in r['paths'])
    assert not table.points


@pytest.mark.parametrize('attribute', ['start_s', 'L'])
def test_curve_parameter_change_invalidates(attribute):
    geometry = make_straight()
    table = get_road_table(geometry)
    setattr(geometry.curve, attribute, getattr(geometry.curve, attribute)+.01)
    assert get_road_table(geometry) is not table


def test_noncontiguous_competitor_boxes_force_uncertain():
    geometry = make_straight()
    table = get_road_table(geometry)
    ref = geometry.reference(1.)
    # A disjoint competitive arc must never be approved using local J alone.
    geometry.lower = np.array([[1., 2.], [100., 100.], [1., 2.]])
    geometry.upper = np.array([[3., 4.], [101., 101.], [3., 4.]])
    assert not table._certify_interval(ref, -.2, .2)


def test_storage_is_bounded_and_s_is_not_quantized():
    table = RoadTable(make_straight(), max_columns=2, max_points=1)
    table.column(1.)
    table.column(np.nextafter(1., 2.))
    assert len(table.columns) == 2
    table.column(2.)
    assert table.stats['evictions'] == 1
    table.confirm(1., 0., dict(valid=True, reason=None))
    table.confirm(1., .1, dict(valid=False, reason='occupied'))
    assert len(table.points) == 1


@pytest.mark.parametrize('name', ['esp', 'aut', 'gbr', 'mco', 'CornerHall', 'my_map'])
def test_six_maps_fast_accepts_agree_with_independent_exact_queries(name):
    geometry = load_geometry(f'maps/frenet/{name}/geometry', f'maps/{name}.yaml', inspect=True)
    table = get_road_table(geometry)
    accepted = 0
    for s in np.linspace(0, geometry.curve.L, 13, endpoint=False):
        column = table.column(s)
        raw = column['boundary']
        if not raw['valid']:
            continue
        for d in np.linspace(raw['d_right']+1e-6, raw['d_left']-1e-6, 9):
            state = table.classify(s, d)
            if state['state'] == 'checked_valid':
                accepted += 1
                assert geometry.to_cartesian(s, d)['valid'], (name, s, d)
    assert accepted > 0
    x, y = geometry.reference(1.)['xy']
    settings = dict(lookahead_m=3., endpoint_tolerance_m=.15, max_refinements=1)
    exact = generate_robust_paths(geometry, x, y, **settings)
    fast = generate_robust_paths(geometry, x, y, query_mode='fast_with_fallback', **settings)
    fast.pop('query_reuse')
    assert fast.pop('verification_level').startswith('dense_frenet_certified')
    exact.pop('verification_level')
    assert exact == fast


def test_closed_lap_key_and_open_endpoint_do_not_alias():
    geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
    table = get_road_table(geometry)
    assert table.column(0.) is table.column(geometry.curve.L)
    open_table = get_road_table(make_straight())
    assert open_table.key(0.) != open_table.key(open_table.geometry.curve.L)


def test_csv_source_hash_is_verified(tmp_path):
    root = Path('maps/frenet/esp/geometry')
    for name in ('manifest.json', 'centerline.csv', 'boundaries.csv', 'reference_curve.json'):
        (tmp_path/name).write_bytes((root/name).read_bytes())
    (tmp_path/'boundaries.csv').write_text('corrupt')
    geometry = load_geometry(root, 'maps/esp.yaml', inspect=True)
    with pytest.raises(ValueError, match='hash mismatch'):
        get_road_table(geometry, tmp_path)
