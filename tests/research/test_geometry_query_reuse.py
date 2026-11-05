"""Exact reuse must preserve checks, isolate results and bound its storage."""
import hashlib
import json

import numpy as np
import pytest

from test_robust_adaptive_paths import make_straight
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.overtaking.geometry_queries import ExactQueryContext
from f1tenth_benchmarks.research.overtaking.robust_adaptive_paths import generate_robust_paths
from f1tenth_benchmarks.research.overtaking.verify_session import read_verified_stream


def test_generation_preserves_entire_result_and_source():
    geometry = make_straight()
    args = dict(yaw=.1, lookahead_m=3, max_refinements=1)
    before = dict(geometry.__dict__)
    expected = generate_robust_paths(geometry, 1.5, 3.1, **args)
    actual = generate_robust_paths(geometry, 1.5, 3.1, query_mode='reused_exact', **args)
    stats = actual.pop('query_reuse')
    assert actual == expected
    assert stats['boundary_hits'] > stats['boundary_misses']
    assert stats['reference_hits'] > 0
    assert stats['reference_prefetched'] > 0
    assert geometry.__dict__ == before


def test_exact_cache_does_not_round_keys_or_share_mutable_results():
    context = ExactQueryContext(make_straight(), max_entries=2)
    a = context.geometry.boundaries(1.)
    expected = dict(a)
    a['d_left'] = -999
    a['left_xy'][0] = -999
    b = context.geometry.boundaries(1.)
    assert b['d_left'] == expected['d_left']
    assert b['left_xy'][0] != -999
    context.geometry.boundaries(np.nextafter(1., 2.))
    assert context.stats['boundary_misses'] == 2
    context.geometry.boundaries(2.)
    assert len(context._boundaries) == 2 and context.stats['evictions'] > 0


def test_projection_is_not_reused_for_another_lateral_coordinate():
    context = ExactQueryContext(make_straight())
    inside = context.geometry.to_cartesian(1., 0.)
    outside = context.geometry.to_cartesian(1., 2.2)
    assert inside['valid'] and not outside['valid']
    assert outside['reason'] == 'occupied'


def test_overridden_exact_rejection_is_preserved():
    geometry = make_straight()
    original = geometry.to_cartesian
    def reject(s, d, e_psi=None):
        result = original(s, d, e_psi)
        return dict(result, valid=False, reason='injected_exact_rejection')
    geometry.to_cartesian = reject
    result = generate_robust_paths(geometry, 1.5, 3., lookahead_m=3,
                                   query_mode='reused_exact', max_refinements=0)
    assert result['sampled_valid_count'] == 0
    assert all(p['reason'] == 'injected_exact_rejection' for p in result['paths'])


def test_stream_reader_hashes_full_bytes_but_keeps_only_replay_inputs(tmp_path):
    row = dict(paths=[dict(points=[dict(x=1), dict(x=2)], attempts=[1], spline={})], result={})
    raw = (json.dumps(row, ensure_ascii=False)+'\n').encode()
    path = tmp_path/'plans.jsonl'
    path.write_bytes(raw)
    index = [dict(byte_offset=0, byte_length=len(raw))]
    records, digest = read_verified_stream(path, index, compact_plans=True)
    assert digest == hashlib.sha256(raw).hexdigest()
    assert records == [dict(paths=[dict(points=[dict(x=1)])])]
    assert read_verified_stream(path, index)[0] == [row]
    for broken in ([dict(byte_offset=1, byte_length=len(raw))],
                   [dict(byte_offset=0, byte_length=len(raw)-1)], []):
        with pytest.raises(AssertionError):
            read_verified_stream(path, broken)


@pytest.mark.parametrize('size', [0, -1, True, 1.5])
def test_invalid_cache_capacity(size):
    with pytest.raises(ValueError):
        ExactQueryContext(make_straight(), max_entries=size)


@pytest.mark.parametrize('name', ['esp', 'aut', 'gbr', 'mco', 'CornerHall', 'my_map'])
def test_reuse_keeps_complete_multimap_results(name):
    geometry = load_geometry(f'maps/frenet/{name}/geometry', f'maps/{name}.yaml', inspect=True)
    x, y = geometry.reference(1.)['xy']
    settings = dict(lookahead_m=3., endpoint_tolerance_m=.15, max_refinements=1)
    exact = generate_robust_paths(geometry, x, y, **settings)
    reused = generate_robust_paths(geometry, x, y, query_mode='reused_exact', **settings)
    reused.pop('query_reuse')
    assert exact == reused
