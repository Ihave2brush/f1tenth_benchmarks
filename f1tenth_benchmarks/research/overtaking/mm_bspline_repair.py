"""Optional time-budgeted B-spline repair of failed mm candidates 2 and 4.

The existing reference C3 is never replaced. Deadline checks are cooperative:
Python callbacks/refinement/validation abort and late results are discarded.
This is NOT an interruptible native solver or an end-to-end real-time deadline.
"""
import math
from time import perf_counter

import numpy as np
from scipy.interpolate import BSpline, PPoly
from scipy.optimize import minimize

from .robust_adaptive_paths import _basis, curvature_jacobian
from .constrained_adaptive_paths import evaluate_path_smoothness
from ..run_mm_lap_comparison import DomainGenerator


class RepairDeadline(Exception):
    pass


class Deadline:
    def __init__(self, end):
        self.end = end

    def __call__(self, *args):
        if perf_counter() >= self.end:
            raise RepairDeadline()


class RequestValidator:
    """Whole-s-cell road bounds plus sampled candidate-specific limits."""
    def __init__(self, domain, start, length, slope_limit, curvature_limit, guard):
        guard()
        self.domain, self.start, self.length = domain, start, length
        self.slope_limit, self.curvature_limit = slope_limit, curvature_limit
        self.edges, self.rows = domain.segment_grid(start['s'], length)
        self.lower, self.upper = domain.arrays['lower'][self.rows], domain.arrays['upper'][self.rows]
        self.sites = np.sort(np.r_[self.edges, (self.edges[:-1]+self.edges[1:])/2])
        self.k, self.kp = domain.reference(start['s']+self.sites)
        guard()

    def check(self, spline, target, guard):
        guard()
        coefficients = np.asarray(spline.c)
        if not np.isfinite(coefficients).all():
            return dict(valid=False, reason='nonfinite_coefficients', refinement_distances_m=[])
        endpoint = np.array([spline(0.), spline(0., nu=1), spline(self.length), spline(self.length, nu=1)])
        expected = np.array([self.start['d'], self.start['d_prime'], target, 0.])
        endpoint_error = float(np.max(abs(endpoint-expected)))
        d_edges = spline(self.edges)
        low, high = np.minimum(d_edges[:-1], d_edges[1:]), np.maximum(d_edges[:-1], d_edges[1:])
        roots = PPoly.from_spline(spline).derivative().roots(extrapolate=False)
        roots = roots[np.isfinite(roots) & (roots > 0.) & (roots < self.length)]
        if len(roots):
            indices = np.clip(np.searchsorted(self.edges, roots, side='right')-1, 0, len(low)-1)
            np.minimum.at(low, indices, spline(roots))
            np.maximum.at(high, indices, spline(roots))
        slack = np.minimum(low-self.lower, self.upper-high)
        guard()
        d, p, q = [spline(self.sites, nu=i) for i in range(3)]
        jacobian, curvature = evaluate_path_smoothness(d, p, q, self.k, self.kp)
        guard()
        finite = bool(np.isfinite(d+p+q+jacobian+curvature).all() and np.isfinite(slack).all())
        min_slack = float(np.min(slack))
        max_curvature = float(np.max(abs(curvature)))
        max_slope = float(np.max(abs(p)))
        min_jacobian = float(np.min(jacobian))
        road = finite and min_slack >= -1e-9
        smooth = (finite and max_curvature <= self.curvature_limit+1e-7 and
                  max_slope <= self.slope_limit+1e-7 and min_jacobian >= .2-1e-7)
        refinement = []
        if not road:
            worst = int(np.nanargmin(slack)) if np.isfinite(slack).any() else 0
            refinement.extend([float(self.edges[worst]), float(self.edges[worst+1])])
            refinement.extend(float(r) for r in roots if self.edges[worst] <= r <= self.edges[worst+1])
        if not smooth and finite:
            for values in (abs(curvature)/self.curvature_limit, abs(p)/self.slope_limit, .2/np.maximum(jacobian, 1e-12)):
                index = int(np.argmax(values))
                for j in range(max(0, index-1), min(len(self.sites), index+2)):
                    refinement.append(float(self.sites[j]))
        valid = bool(road and smooth and endpoint_error <= 1e-7)
        return dict(valid=valid, reason=None if valid else 'road_interval' if not road else
                    'smoothness_limit' if not smooth else 'endpoint_mismatch',
                    endpoint_error=endpoint_error, min_road_slack_m=min_slack,
                    max_abs_curvature_inv_m=max_curvature, max_abs_slope=max_slope,
                    min_jacobian=min_jacobian, refinement_distances_m=sorted(set(refinement)))


def fit_once(domain, start, target, basis, sites, slope_limit, curvature_limit, guard, seed=None):
    """One constrained SLSQP start using V5.2's objective and Jacobian formulas."""
    guard()
    B, D1, D2, D3 = [basis(sites, nu=i) for i in range(4)]
    source = start['s']+sites
    rows = np.stack([domain.indices(source+offset) for offset in (-1e-9, 0., 1e-9)])
    # A knot on a road-cell face must satisfy BOTH adjacent rectangles.
    # Using only floor(s/h) can leave a tiny violation on the departing cell.
    lower = domain.arrays['lower'][rows].max(axis=0)
    upper = domain.arrays['upper'][rows].min(axis=0)
    if not np.isfinite(lower+upper).all() or np.any(lower >= upper):
        return np.zeros(basis.c.shape[1]), dict(solver_success=False, solver_constraints_accepted=False,
            iterations=0, status=-1, message='incompatible_road_intervals')
    k, kp = domain.reference(start['s']+sites)
    length = float(sites[-1])
    E = np.stack([basis(0.), basis(0., nu=1), basis(length), basis(length, nu=1)])
    rhs = np.array([start['d'], start['d_prime'], target, 0.])
    u = sites/length
    first_fraction = np.clip((start['d']-lower[0])/(upper[0]-lower[0]), 0., 1.)
    last_fraction = np.clip((target-lower[-1])/(upper[-1]-lower[-1]), 0., 1.)
    blend = 3*u*u-2*u*u*u
    preferred = lower+((1-blend)*first_fraction+blend*last_fraction)*(upper-lower)
    H = B.T@B+2*D2.T@D2+.2*D3.T@D3
    linear = B.T@preferred
    if seed is None:
        initial = np.linalg.lstsq(B, preferred, rcond=None)[0]
    else:
        initial = np.asarray(seed).copy()
    initial += E.T@np.linalg.solve(E@E.T, rhs-E@initial)

    def objective(c):
        guard()
        return float((c@H@c-2*linear@c)/len(sites))

    def gradient(c):
        guard()
        return 2*(H@c-linear)/len(sites)

    def constraints(c):
        guard()
        d, p, q = B@c, D1@c, D2@c
        jacobian, curvature = evaluate_path_smoothness(d, p, q, k, kp)
        return np.r_[d-lower, upper-d, slope_limit-p, slope_limit+p, jacobian-.2,
                     curvature_limit-curvature, curvature_limit+curvature]

    def constraint_jac(c):
        guard()
        curvature = curvature_jacobian(B, D1, D2, c, k, kp)
        return np.vstack([B, -B, -D1, D1, -k[:, None]*B, -curvature, curvature])

    result = minimize(objective, initial, jac=gradient, method='SLSQP', callback=guard,
        constraints=[dict(type='eq', fun=lambda c: E@c-rhs, jac=lambda c:E),
                     dict(type='ineq', fun=constraints, jac=constraint_jac)],
        options=dict(maxiter=60, ftol=1e-9))
    guard()
    residual = constraints(result.x)
    accepted = bool(result.success and np.isfinite(result.x).all()
                    and np.max(abs(E@result.x-rhs)) <= 1e-7 and np.min(residual) >= -1e-7)
    return result.x, dict(solver_success=bool(result.success), solver_constraints_accepted=accepted,
                         iterations=int(result.nit), status=int(result.status), message=str(result.message))


def aligned_previous_seed(cached, start, target, basis, length, delta):
    """Shift the old Frenet curve; hold its tail constant, refit and match endpoints.

    This is an initial guess only. It never inherits any validity decision.
    """
    sites = np.linspace(0., length, 61)
    previous = BSpline(basis.t, cached['coefficients'], basis.k)
    preferred = previous(np.minimum(sites+delta, length))
    seed = np.linalg.lstsq(basis(sites), preferred, rcond=None)[0]
    E = np.stack([basis(0.), basis(0., nu=1), basis(length), basis(length, nu=1)])
    rhs = np.array([start['d'], start['d_prime'], target, 0.])
    return seed+E.T@np.linalg.solve(E@E.T, rhs-E@seed)


class BudgetedDomainGenerator(DomainGenerator):
    """Repair selected failed slots with one budget; C3 stays reference-only."""
    def __init__(self, domain, budget_s=.04, curvature_reserve=.01,
                 repair_candidates=(2, 4), warm_start_mode='coefficients'):
        super().__init__(domain)
        if not np.isfinite([budget_s, curvature_reserve]).all() or budget_s < 0 or curvature_reserve < 0:
            raise ValueError('invalid repair budget/reserve')
        self.budget_s, self.curvature_reserve = budget_s, curvature_reserve
        candidates = tuple(repair_candidates)
        if (not candidates or any(type(i) is not int or i not in (1, 2, 4, 5) for i in candidates)
                or len(set(candidates)) != len(candidates)):
            raise ValueError('repair candidates must be unique IDs from 1,2,4,5; C3 is reference-only')
        self.repair_candidates = candidates
        if warm_start_mode not in ('coefficients', 'aligned'):
            raise ValueError('unknown warm start mode')
        self.warm_start_mode = warm_start_mode
        self.previous = {}
        self.requests = 0

    def __call__(self, geometry, x, y, yaw, **settings):
        started = perf_counter()
        result = super().__call__(geometry, x, y, yaw, **settings)
        deadline = Deadline(started+self.budget_s)
        self.requests += 1
        # Protect the near-centre alternatives first, then try the extremes.
        # Alternate within each pair; every repaired shape is checked itself.
        inner, outer = [1, 3], [0, 4]
        if self.requests % 2 == 0:
            inner.reverse()
            outer.reverse()
        priority = [i for i in inner+outer if i+1 in self.repair_candidates]
        wanted = [i for i in priority if len(result['paths']) == 5 and not result['paths'][i]['sampled_valid']
                  and result['paths'][i].get('reason') != 'no_lateral_target'
                  and result['paths'][i].get('requested_target_d') is not None]
        details = dict(budget_s=self.budget_s, scope='generator_including_base_excluding_session_gate_and_control',
            deadline_policy='cooperative_callbacks_and_discard_late_result', hard_deadline_enforced=False,
            requested_ids=[i+1 for i in wanted], repaired_ids=[], timed_out_ids=[], attempts=[],
            curvature_reserve_inv_m=self.curvature_reserve,
            enabled_candidates=list(self.repair_candidates), priority_policy='inner_pair_then_outer_pair_alternating',
            warm_start_mode=self.warm_start_mode)
        if not wanted or self.budget_s == 0:
            details['elapsed_s'] = perf_counter()-started
            result['bspline_repair'] = details
            return result
        start, length = result['start'], settings['lookahead_m']
        effective_limit = settings['max_curvature']-self.curvature_reserve
        if effective_limit <= 0:
            raise ValueError('curvature reserve exceeds limit')
        current_position = 0
        try:
            validator = RequestValidator(self.domain, start, length, settings['max_slope'], effective_limit, deadline)
            basis = _basis(length, 8)
            for position, index in enumerate(wanted):
                current_position = position
                deadline()
                path = result['paths'][index]
                target = path['requested_target_d']
                sites = np.linspace(0., length, math.ceil(length/.05)+1)
                cached = self.previous.get(index)
                seed = None
                if cached and cached['length'] == length and ((start['s']-cached['s']) % self.domain.length) < .5:
                    seed = cached['coefficients']
                    if self.warm_start_mode == 'aligned':
                        seed = aligned_previous_seed(cached, start, target, basis, length,
                            (start['s']-cached['s']) % self.domain.length)
                        deadline()
                for refinement in range(3):
                    coefficients, diagnostic = fit_once(self.domain, start, target, basis, sites,
                        settings['max_slope'], effective_limit, deadline, seed)
                    diagnostic.update(candidate_id=index+1, refinement=refinement,
                                      warm_start=seed is not None, constraint_points=len(sites))
                    details['attempts'].append(diagnostic)
                    if not diagnostic['solver_constraints_accepted']:
                        break
                    spline = BSpline(basis.t, coefficients, basis.k)
                    check = validator.check(spline, target, deadline)
                    diagnostic['domain_check'] = check
                    if check['valid']:
                        # Validate again before publishing; no cached validity flag.
                        independent = validator.check(BSpline(basis.t.copy(), coefficients.copy(), basis.k), target, deadline)
                        if not independent['valid']:
                            break
                        reference = geometry.reference(start['s'])
                        d0, p0 = float(spline(0.)), float(spline(0., nu=1))
                        tangent = reference['tangent']
                        first = dict(x=reference['xy'][0]-d0*tangent[1],
                                     y=reference['xy'][1]+d0*tangent[0],
                                     yaw=reference['psi']+math.atan2(p0, 1-reference['kappa']*d0))
                        deadline()
                        path.update(prior_direct_reason=path['reason'], sampled_valid=True,
                            experimental_valid=True, status='sampled_valid', reason=None,
                            generation_route='budgeted_bspline_repair', boundary_valid=True,
                            boundary_check='explicit_bspline_cell_extrema',
                            smoothness_sampled_valid=True, points=[first],
                            max_abs_curvature_inv_m=check['max_abs_curvature_inv_m'],
                            min_jacobian=check['min_jacobian'],
                            spline=dict(knots=basis.t.tolist(), coefficients=coefficients.tolist(), degree=basis.k),
                            repair_validation=check)
                        details['repaired_ids'].append(index+1)
                        self.previous[index] = dict(s= start['s'], length=length, coefficients=coefficients.copy())
                        break
                    new_sites = np.unique(np.r_[sites, check['refinement_distances_m']])
                    if len(new_sites) == len(sites):
                        break
                    sites, seed = new_sites, coefficients
        except RepairDeadline:
            details['timed_out_ids'] = [i+1 for i in wanted[current_position:] if i+1 not in details['repaired_ids']]
        result['sampled_valid_count'] = sum(p['sampled_valid'] for p in result['paths'])
        result['valid_ids'] = [p['candidate_id'] for p in result['paths'] if p['sampled_valid']]
        result['requires_stop'] = result['sampled_valid_count'] == 0
        details['elapsed_s'] = perf_counter()-started
        result['bspline_repair'] = details
        return result
