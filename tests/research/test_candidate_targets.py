"""V1 horizon guards and unchanged target placement; inspection only."""
from types import SimpleNamespace

import numpy as np
import pytest

from f1tenth_benchmarks.research.overtaking.candidate_targets import generate_five_targets


@pytest.fixture
def geometry():
    calls = []
    return SimpleNamespace(
        curve=SimpleNamespace(closed=True, L=10.0), geometry_id='target-test',
        to_frenet=lambda xy, yaw=None: dict(valid=True, s_wrapped=9.0, d=0.1),
        boundaries=lambda s: (calls.append(s) or dict(valid=True, d_right=-1., d_left=1.)),
        to_cartesian=lambda s, d: dict(valid=True, xy=[s, d], reason=None,
                                     planning_allowed=False, domain_verified=False),
        boundary_calls=calls,
    )


@pytest.mark.parametrize('lookahead', [0.5, 1.5, 9.9])
def test_closed_sub_lap_targets_keep_original_positions_and_metadata(geometry, lookahead):
    result = generate_five_targets(geometry, 9., .1, lookahead_m=lookahead)
    target_s = (9. + lookahead) % 10.
    assert result['target_s'] == pytest.approx(target_s)
    assert geometry.boundary_calls == [target_s]
    assert [p['candidate_id'] for p in result['candidates']] == [1, 2, 3, 4, 5]
    for p, d in zip(result['candidates'], np.linspace(-.8, .8, 5)):
        assert [p['s'], p['d'], p['x'], p['y']] == pytest.approx([target_s, d, target_s, d])
        assert p['valid'] and p['reason'] is None
        assert p['geometry_id'] == 'target-test' and p['frame_id'] == 'map'
    assert result['inspection_only'] and not result['planning_allowed']


@pytest.mark.parametrize('lookahead', [10., 10.1, 20.])
def test_closed_full_or_multiple_laps_rejected(geometry, lookahead):
    with pytest.raises(ValueError, match='shorter than one full lap'):
        generate_five_targets(geometry, 9., .1, lookahead_m=lookahead)
    assert not geometry.boundary_calls


@pytest.mark.parametrize('lookahead', [0., -1., np.nan, np.inf, -np.inf])
def test_horizon_must_be_positive_and_finite(geometry, lookahead):
    with pytest.raises(ValueError, match='lookahead_m'):
        generate_five_targets(geometry, 9., .1, lookahead_m=lookahead)


def test_open_end_is_allowed_but_beyond_end_is_rejected(geometry):
    geometry.curve.closed = False
    result = generate_five_targets(geometry, 9., .1, lookahead_m=1.)
    assert result['target_s'] == 10.
    with pytest.raises(ValueError, match='end of the open reference'):
        generate_five_targets(geometry, 9., .1, lookahead_m=1.01)


@pytest.mark.parametrize('kind', ['invalid_boundary', 'too_narrow', 'invalid_start'])
def test_existing_geometry_rejections_preserved(geometry, kind):
    if kind == 'invalid_start':
        geometry.to_frenet = lambda xy, yaw=None: dict(valid=False, reason='occupied')
    else:
        geometry.boundaries = lambda s: dict(valid=kind != 'invalid_boundary', reason='unknown',
                                            d_right=-.1, d_left=.1)
    with pytest.raises(ValueError):
        generate_five_targets(geometry, 9., .1, lookahead_m=.5)
