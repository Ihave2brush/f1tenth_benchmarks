"""V5-inspired direct proposals preserve real pose and shared hard gates."""
import json

import numpy as np
import pytest
from scipy.interpolate import BSpline, CubicSpline

from test_robust_adaptive_paths import make_straight
from f1tenth_benchmarks.research.overtaking import robust_adaptive_paths as planner
from f1tenth_benchmarks.research.overtaking.width_fraction_paths import width_fraction_spline
from f1tenth_benchmarks.research.overtaking.session import SessionConfig, make_plan
from f1tenth_benchmarks.research.run_fixed_grid_benchmark import audit_legacy_grid


def test_width_fraction_values_actual_slopes_and_exported_spline_agree():
    dense = np.linspace(0., 6., 241)
    lo = -.8-.08*np.sin(dense)
    hi = .9+.05*np.cos(dense)
    sections = [dict(d_min=a, d_max=b) for a, b in zip(lo, hi)]
    fraction0, fraction = .4, .75
    d0, target = lo[0]+fraction0*(hi[0]-lo[0]), lo[-1]+fraction*(hi[-1]-lo[-1])
    basis, c = width_fraction_spline(dense, sections, fraction0, fraction, d0, .07, target)
    u = dense/6.
    blend = 3*u*u-2*u*u*u
    expected = lo+((1-blend)*fraction0+blend*fraction)*(hi-lo)
    raw = CubicSpline(dense, expected)
    expected += 6.*((.07-float(raw(0., nu=1)))*(u**3-2*u*u+u)
                    -float(raw(6., nu=1))*(u**3-u*u))
    np.testing.assert_allclose(basis(dense)@c, expected, rtol=0, atol=1e-12)
    np.testing.assert_allclose([basis(0.)@c, basis(0., nu=1)@c,
                               basis(6.)@c, basis(6., nu=1)@c],
                              [d0, .07, target, 0.], atol=1e-12)
    exported = BSpline(basis.t, c, basis.k)
    for order in range(3):
        np.testing.assert_allclose(basis(dense, nu=order)@c, exported(dense, nu=order))
    # Continuity of second derivative at interior knots (finite offsets).
    np.testing.assert_allclose(exported(dense[1:-1]-1e-9, nu=2),
                               exported(dense[1:-1]+1e-9, nu=2), atol=1e-5)


def test_all_valid_width_proposals_skip_optimizer_and_pass_exact_audit(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('valid width-fraction proposals must skip optimization')
    monkeypatch.setattr(planner, 'fit_multistart', forbidden)
    state = dict(control_step=0, sim_time_s=0., pose=[1.513, 3.1, .03],
                 valid=True, frenet=dict(valid=True))
    config = SessionConfig(lookahead_m=4., generation_mode='width_fraction_then_repair',
                           query_mode='fast_with_fallback', footprint_check_enabled=False)
    event = make_plan(make_straight(), state, config, 0)
    assert event['sampled_valid_count'] == event['result']['direct_valid_count'] == 5
    for path in event['paths']:
        assert not path['attempts'] and not path['repair_attempted']
        assert path['direct_attempt']['method'] == 'width_fraction_c2_hermite_pose_correction_v1'
        assert path['spline']['degree'] == 3
        assert path['start_position_error_m'] < 1e-8
        assert abs(path['start_heading_error_rad']) < 1e-8
    assert audit_legacy_grid(make_straight(), event, config)['passed']
    json.dumps(event, allow_nan=False)


def test_only_rejected_width_id_enters_same_legacy_repair(monkeypatch):
    validate = planner._validate
    def injected(geometry, basis, c, dense, sections, k, kp, target, *args):
        points, invalid, segment = validate(geometry, basis, c, dense, sections, k, kp, target, *args)
        if basis.k == 3 and target > 1.:
            points[4].update(valid=False, reason='injected_width_failure')
            return points, 4, segment
        return points, invalid, segment
    monkeypatch.setattr(planner, '_validate', injected)
    result = planner.generate_robust_paths(make_straight(), 1.513, 3., yaw=0., lookahead_m=4.,
        query_mode='fast_with_fallback', generation_mode='width_fraction_then_repair')
    assert result['sampled_valid_count'] == 5
    assert result['direct_valid_count'] == 4 and result['repaired_valid_count'] == 1
    assert all(not p['attempts'] for p in result['paths'][:4])
    assert result['paths'][-1]['repair_attempted']
    assert result['paths'][-1]['spline']['degree'] == 5


def test_width_construction_failure_repairs_only_that_id(monkeypatch):
    build = planner.width_fraction_spline
    def injected(dense, sections, fraction0, fraction, *args):
        if fraction == 0.:
            raise ValueError('injected interpolation failure')
        return build(dense, sections, fraction0, fraction, *args)
    monkeypatch.setattr(planner, 'width_fraction_spline', injected)
    result = planner.generate_robust_paths(make_straight(), 1.513, 3., yaw=0., lookahead_m=4.,
        query_mode='fast_with_fallback', generation_mode='width_fraction_then_repair')
    assert result['direct_valid_count'] == 4 and result['repaired_valid_count'] == 1
    assert result['paths'][0]['direct_attempt']['reason'] == 'width_fraction_construction_failed'
    assert all(not p['repair_attempted'] for p in result['paths'][1:])


def test_width_only_keeps_curvature_limit_and_never_repairs(monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail('width_fraction_only must not repair')
    monkeypatch.setattr(planner, 'fit_multistart', forbidden)
    result = planner.generate_robust_paths(make_straight(), 1.513, 3., yaw=0., lookahead_m=4.,
        max_curvature=.01, query_mode='fast_with_fallback', generation_mode='width_fraction_only')
    assert result['sampled_valid_count'] < 5
    for path in result['paths']:
        assert not path['attempts']
        if path['sampled_valid']:
            assert max(abs(p['curvature_inv_m']) for p in path['points']) <= .01+1e-7


def test_width_inside_samples_does_not_bypass_curvature_screen():
    dense = np.linspace(0., 4., 161)
    center = .06*np.sin(25*dense)
    sections = [dict(d_min=v-1., d_max=v+1.) for v in center]
    basis, c = width_fraction_spline(dense, sections, .5, .5, center[0], 0., center[-1])
    screening = planner.direct_constraint_screen(basis, c, dense, sections,
        np.zeros(len(dense)), np.zeros(len(dense)), 1., .5)
    assert screening['reason'] == 'smoothness_limit'
    assert screening['min_margin_slack_m'] > 0.


def test_constant_width_pose_corrected_curve_matches_direct_hermite():
    dense = np.linspace(0., 5., 201)
    sections = [dict(d_min=-1., d_max=1.) for _ in dense]
    basis, c = width_fraction_spline(dense, sections, .55, .9, .1, .06, .8)
    hermite = planner._basis(5., 6)
    hc = planner.direct_hermite_coefficients(5., .1, .06, .8)
    for order in range(3):
        np.testing.assert_allclose(basis(dense, nu=order)@c, hermite(dense, nu=order)@hc,
                                   rtol=0, atol=1e-9)


@pytest.mark.parametrize('mode', ['width_fraction_only', 'width_fraction_then_repair'])
def test_session_accepts_explicit_width_modes(mode):
    assert SessionConfig(generation_mode=mode).generation_mode == mode
