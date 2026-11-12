"""Exact fixed sites reuse across moving windows; never snap real endpoints."""
import numpy as np
import pytest
import math

from test_robust_adaptive_paths import make_straight
from f1tenth_benchmarks.research.overtaking.fixed_grid import FixedRoadWindow, sample_window
from f1tenth_benchmarks.research.overtaking.road_table import get_road_table
from f1tenth_benchmarks.research.overtaking.robust_adaptive_paths import generate_robust_paths
from f1tenth_benchmarks.research.overtaking.session import SessionConfig, make_plan
from f1tenth_benchmarks.research.run_fixed_grid_benchmark import audit_legacy_grid


def test_moving_window_only_adds_new_fixed_sites_and_keeps_exact_endpoints():
    window = FixedRoadWindow(.025, 30., False)
    p, s, first = window.sample(.013, 6.)
    q, t, second = window.sample(.133, 6.)
    assert p[0] == s[0] == .013 and p[-1] == s[-1] == 6.013
    assert q[0] == t[0] == .133 and q[-1] == t[-1] == 6.133
    np.testing.assert_array_equal(s, p)
    assert first['reused_grid_sites'] == 0
    assert second['reused_grid_sites'] >= 234
    assert second['newly_entered_grid_sites'] <= 6
    assert second['max_gap_m'] <= .025+1e-12
    assert not second['s_interpolation'] and not second['endpoints_snapped']


def test_closed_seam_retains_canonical_sites_and_forward_progress():
    window = FixedRoadWindow(.025, 10.013, True)
    p, s, _ = window.sample(9.95, 1.)
    q, t, info = window.sample(.057, 1.)
    assert np.all(np.diff(p) > 0) and np.all(np.diff(q) > 0)
    assert p[-1] > window.length and np.all((s >= 0) & (s < window.length))
    np.testing.assert_array_equal(np.intersect1d(s[1:-1], t[1:-1]), t[1:36])
    assert info['overlap_fraction'] > .8
    assert info['max_gap_m'] <= .025+1e-12


def test_jump_reverse_and_spacing_change_do_not_reuse_incorrect_sites():
    geometry = make_straight()
    table = get_road_table(geometry)
    _, _, first = sample_window(table, .1, 1., .025)
    _, _, jumped = sample_window(table, 2.1, 1., .025)
    _, _, reversed_window = sample_window(table, 2., 1., .025)
    assert first['reused_grid_sites'] == jumped['reused_grid_sites'] == 0
    assert reversed_window['overlap_fraction'] > .8
    old = table._fixed_grid_window
    sample_window(table, 2., 1., .02)
    assert table._fixed_grid_window is not old
    geometry.raster.occupancy[0, 0] = -1
    new = get_road_table(geometry)
    assert new is not table and not hasattr(new, '_fixed_grid_window')


@pytest.mark.parametrize('start,distance', [(-.1, 1.), (4., 100.), (0., float('nan'))])
def test_invalid_windows_rejected(start, distance):
    with pytest.raises(ValueError):
        FixedRoadWindow(.025, 10., False).sample(start, distance)


def test_fixed_grid_actual_pose_and_legacy_exact_audit():
    geometry = make_straight()
    config = SessionConfig(lookahead_m=3., generation_mode='direct_then_repair',
                           query_mode='fixed_grid_with_fallback', footprint_check_enabled=False)
    state = dict(control_step=0, sim_time_s=0., valid=True, frenet=dict(valid=True),
                 pose=[1.513, 3.1, .05])
    event = make_plan(geometry, state, config, 0)
    grid = event['result']['validation_grid']
    assert not grid['endpoints_snapped'] and grid['active_grid_sites'] > 0
    assert event['sampled_valid_count'] == 5
    for p in event['paths']:
        assert p['start_position_error_m'] < 1e-8
        assert abs(p['start_heading_error_rad']) < 1e-8
        assert p['points'][0]['s_progress'] == event['result']['start']['s']
    audit = audit_legacy_grid(make_straight(), event, config)
    assert audit['passed'] and audit['checked_count'] == 5
    second = generate_robust_paths(geometry, 1.633, 3.1, yaw=.05, lookahead_m=3.,
        query_mode='fixed_grid_with_fallback', generation_mode='direct_then_repair')
    assert second['validation_grid']['overlap_fraction'] > .9
    # Fixed sites reuse, while the moving legacy guard still needs new queries.
    assert second['validation_grid']['legacy_sampling_guard']
    assert second['query_reuse']['road_table_request']['column_misses'] < len(second['road_sections'])


def test_uncertain_custom_geometry_still_uses_exact_rejection():
    geometry = make_straight()
    original = geometry.to_cartesian
    def reject(s, d, e_psi=None):
        return dict(original(s, d, e_psi), valid=False, reason='injected_exact_rejection')
    geometry.to_cartesian = reject
    result = generate_robust_paths(geometry, 1.513, 3., lookahead_m=3.,
        query_mode='fixed_grid_with_fallback', generation_mode='direct_only')
    assert result['sampled_valid_count'] == 0
    assert result['query_reuse']['road_table_request']['exact_fallbacks'] > 0


def test_legacy_polyline_guard_cannot_be_bypassed_by_finer_union(monkeypatch):
    from f1tenth_benchmarks.research.overtaking import robust_adaptive_paths as planner
    counts = []
    def segments(raster, xy):
        counts.append(len(xy))
        return dict(valid=len(xy) != 81, reason='legacy_chord_blocked' if len(xy) == 81 else None,
                    segment_index=0 if len(xy) == 81 else None)
    monkeypatch.setattr(planner, 'validate_path_segments', segments)
    result = planner.generate_robust_paths(make_straight(), 1.513, 3., yaw=0., lookahead_m=4.,
        query_mode='fixed_grid_with_fallback', generation_mode='direct_only')
    assert result['sampled_valid_count'] == 0
    assert 81 in counts and max(counts) > 81
    assert all(p['reason'] == 'legacy_chord_blocked' for p in result['paths'])


def test_same_spacing_different_phase_can_miss_a_slope_peak():
    from f1tenth_benchmarks.research.overtaking import robust_adaptive_paths as planner
    # d'(s)=1.00001 - .5*(s-1.5)^2: old s=1.5 catches a violation,
    # but a globally anchored grid misses the narrow peak. No optimizer needed.
    length = 3.
    power = [-1., 1.00001-.5*1.5**2, .75, -1/6]
    coefficients = np.array([sum(power[j]*length**j*math.comb(i, j)/math.comb(5, j)
                                 for j in range(min(i, 3)+1)) for i in range(6)])
    basis = planner._basis(length, 6)
    progress, _, _ = FixedRoadWindow(.05, 10., False).sample(.513, length)
    fixed = progress-.513
    fixed[0], fixed[-1] = 0., length
    original = np.linspace(0, length, 61)
    def screen(grid):
        sections = [dict(d_min=-2., d_max=2.) for _ in grid]
        return planner.direct_constraint_screen(basis, coefficients, grid, sections,
            np.zeros(len(grid)), np.zeros(len(grid)), 1., 100.)
    assert screen(fixed) is None
    assert screen(original)['reason'] == 'smoothness_limit'
