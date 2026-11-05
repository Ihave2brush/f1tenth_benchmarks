"""Budget and safety invariants for optional C2/C4 repair."""
from time import sleep
import numpy as np
from scipy.interpolate import BSpline

import test_millimetre_domain as mm_tests
from test_robust_adaptive_paths import make_straight
from f1tenth_benchmarks.research.overtaking import mm_bspline_repair as repair
from f1tenth_benchmarks.research.overtaking.session import make_plan, SessionConfig
from f1tenth_benchmarks.research.run_mm_lap_comparison import DomainGenerator
from f1tenth_benchmarks.research.run_fixed_grid_benchmark import audit_legacy_grid


def fixture(narrow=True):
    domain = mm_tests.MillimetreDomainTests.straight_domain(-.4, .4)
    if narrow:
        domain.arrays['lower'][1500:1650] = -.02
        domain.arrays['upper'][1500:1650] = .02
    geometry = make_straight()
    state = dict(control_step=0, sim_time_s=0., pose=[1.5, 3.05, 0.], valid=True, frenet=dict(valid=True))
    return domain, geometry, state


def test_aligned_seed_matches_new_endpoints_without_mutating_cache():
    basis = repair._basis(3., 8)
    coefficients = np.linspace(-.1, .2, 8)
    cached = dict(coefficients=coefficients.copy())
    start = dict(d=.03, d_prime=-.02)
    seed = repair.aligned_previous_seed(cached, start, .17, basis, 3., .12)
    curve = BSpline(basis.t, seed, basis.k)
    np.testing.assert_allclose([curve(0.), curve(0., nu=1), curve(3.), curve(3., nu=1)],
                               [.03, -.02, .17, 0.], atol=1e-12)
    np.testing.assert_array_equal(cached['coefficients'], coefficients)
    assert not np.allclose(seed, coefficients)


def test_aligned_seed_reproduces_constant_and_does_not_bypass_validation(monkeypatch):
    basis = repair._basis(3., 8)
    seed = repair.aligned_previous_seed(dict(coefficients=np.full(8, .1)),
        dict(d=.1, d_prime=0.), .1, basis, 3., .12)
    np.testing.assert_allclose(seed, .1, atol=1e-12)
    domain, geometry, state = fixture()
    def forged(*args, **kwargs):
        return np.full(8, .9), dict(solver_constraints_accepted=True, solver_success=True, iterations=1)
    monkeypatch.setattr(repair, 'fit_once', forged)
    generator = repair.BudgetedDomainGenerator(domain, budget_s=.5, warm_start_mode='aligned')
    for index in (1, 3):
        generator.previous[index] = dict(s=1.4, length=3., coefficients=np.full(8, .1))
    result = plan(domain, geometry, state, generator)
    assert result['result']['bspline_repair']['repaired_ids'] == []
    assert result['paths'][2]['sampled_valid']


def plan(domain, geometry, state, generator):
    return make_plan(geometry, state, SessionConfig(safety_margin_m=.15), 0, generator=generator)


def test_repair_adds_valid_lateral_without_changing_reference():
    domain, geometry, state = fixture()
    baseline = plan(domain, geometry, state, DomainGenerator(domain))
    result = plan(domain, geometry, state, repair.BudgetedDomainGenerator(domain, budget_s=.5))
    assert result['paths'][2] == baseline['paths'][2]
    assert result['result']['fallback_candidate_id'] == 3
    repaired = result['result']['bspline_repair']['repaired_ids']
    assert repaired and set(repaired) <= {2, 4}
    assert all(result['paths'][i-1]['sampled_valid'] for i in repaired)
    assert audit_legacy_grid(geometry, result, SessionConfig(safety_margin_m=.15))['passed']


def test_tiny_budget_preserves_reference_without_publishing_late_curve():
    domain, geometry, state = fixture()
    baseline = plan(domain, geometry, state, DomainGenerator(domain))
    result = plan(domain, geometry, state, repair.BudgetedDomainGenerator(domain, budget_s=1e-9))
    assert result['paths'] == baseline['paths']
    assert result['result']['bspline_repair']['repaired_ids'] == []
    assert result['result']['bspline_repair']['timed_out_ids']
    assert result['result']['fallback_candidate_id'] == 3


def test_already_valid_two_and_four_do_not_call_solver(monkeypatch):
    domain, geometry, state = fixture(narrow=False)
    def forbidden(*args, **kwargs):
        raise AssertionError('valid direct proposals must not be repaired')
    monkeypatch.setattr(repair, 'fit_once', forbidden)
    result = plan(domain, geometry, state, repair.BudgetedDomainGenerator(domain))
    assert result['paths'][1]['sampled_valid'] and result['paths'][3]['sampled_valid']
    assert result['result']['bspline_repair']['requested_ids'] == []


def test_solver_success_cannot_bypass_domain_validation(monkeypatch):
    domain, geometry, state = fixture()
    def forged(*args, **kwargs):
        return np.full(8, .9), dict(solver_constraints_accepted=True, solver_success=True, iterations=1)
    monkeypatch.setattr(repair, 'fit_once', forged)
    result = plan(domain, geometry, state, repair.BudgetedDomainGenerator(domain, budget_s=.2))
    assert result['result']['bspline_repair']['repaired_ids'] == []
    assert result['paths'][2]['sampled_valid']


def test_late_native_result_is_discarded_and_not_called_hard_deadline(monkeypatch):
    domain, geometry, state = fixture()
    def late(*args, **kwargs):
        sleep(.025)
        return np.zeros(8), dict(solver_constraints_accepted=True, solver_success=True, iterations=1)
    monkeypatch.setattr(repair, 'fit_once', late)
    result = plan(domain, geometry, state, repair.BudgetedDomainGenerator(domain, budget_s=.01))
    record = result['result']['bspline_repair']
    assert record['repaired_ids'] == [] and record['timed_out_ids']
    assert record['hard_deadline_enforced'] is False
    assert result['paths'][2]['sampled_valid']


def test_one_mm_narrow_cell_between_coarse_points_is_rejected():
    domain, _, _ = fixture(narrow=False)
    domain.arrays['lower'][2023] = -.01
    domain.arrays['upper'][2023] = .01
    start = dict(s=.5, d=.05, d_prime=0.)
    validator = repair.RequestValidator(domain, start, 3., 1., 1.27, lambda:None)
    basis = repair._basis(3., 8)
    result = validator.check(BSpline(basis.t, np.full(8, .05), basis.k), .05, lambda:None)
    assert not result['valid'] and result['reason'] == 'road_interval'


def test_invalid_budget_is_rejected():
    import pytest
    domain, _, _ = fixture()
    for value in (-1., float('nan')):
        with pytest.raises(ValueError):
            repair.BudgetedDomainGenerator(domain, budget_s=value)


def test_outer_repairs_are_optional_and_checked_against_the_original_geometry():
    domain, geometry, state = fixture()
    baseline = plan(domain, geometry, state, DomainGenerator(domain))
    result = plan(domain, geometry, state, repair.BudgetedDomainGenerator(
        domain, budget_s=2., repair_candidates=(1,2,4,5)))
    record = result['result']['bspline_repair']
    assert set(record['requested_ids']) == {1,2,4,5}
    assert set(record['repaired_ids']) & {1,5}
    assert result['paths'][2] == baseline['paths'][2]
    for cid in record['repaired_ids']:
        path = result['paths'][cid-1]
        assert path['boundary_check'] == 'explicit_bspline_cell_extrema'
        assert path['repair_validation']['valid']
        assert abs(path['spline']['coefficients'][0]-state['pose'][1]+3.) < 1e-7
    assert audit_legacy_grid(geometry, result, SessionConfig(safety_margin_m=.15))['passed']


def test_all_slot_timeout_keeps_the_original_candidates():
    domain, geometry, state = fixture()
    baseline = plan(domain, geometry, state, DomainGenerator(domain))
    result = plan(domain, geometry, state, repair.BudgetedDomainGenerator(
        domain, budget_s=1e-9, repair_candidates=(1,2,4,5)))
    record = result['result']['bspline_repair']
    assert set(record['timed_out_ids']) == {1,2,4,5}
    assert result['paths'] == baseline['paths']
    assert record['hard_deadline_enforced'] is False


def test_reference_slot_cannot_be_requested_as_an_outer_repair():
    import pytest
    domain, _, _ = fixture()
    for ids in [(3,), (1,1), (0,2), (2,6), (), (True,4)]:
        with pytest.raises(ValueError):
            repair.BudgetedDomainGenerator(domain, repair_candidates=ids)
