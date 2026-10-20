"""Offline mathematical counterexample; no simulator or controller is executed.

Two-step scalar affine plant s+=s+u, s0=.9, finish L=1,
u in [0,.2], hard terminal convex hull [1,1.1].
An exact active-set solver for these 2D convex quadratic subproblems uses
standard Python only. This is not an OSQP execution or a F1TENTH benchmark.
"""
import itertools
import json
from pathlib import Path


def dot(a, b):
    return sum(x*y for x, y in zip(a, b))


def inverse(a):
    det = a[0][0]*a[1][1]-a[0][1]*a[1][0]
    return [[a[1][1]/det, -a[0][1]/det],
            [-a[1][0]/det, a[0][0]/det]]


def matvec(a, x):
    return [dot(row, x) for row in a]


H = [[4., -2.], [-2., 2.]]
HI = inverse(H)
BASE = [([1., 0.], .2), ([-1., 0.], 0.),
        ([0., 1.], .2), ([0., -1.], 0.),
        ([1., 1.], .2), ([-1., -1.], -.1)]


def solve(q, extra=()):
    constraints = BASE+list(extra)
    origin = matvec(HI, [-x for x in q])
    candidates = [origin]
    for a, b in constraints:
        ha = matvec(HI, a)
        multiplier = (dot(a, origin)-b)/dot(a, ha)
        candidates.append([origin[i]-multiplier*ha[i] for i in range(2)])
    for (a, b), (c, d) in itertools.combinations(constraints, 2):
        det = a[0]*c[1]-a[1]*c[0]
        if abs(det) > 1e-14:
            candidates.append([(b*c[1]-a[1]*d)/det, (a[0]*d-b*c[0])/det])
    feasible = [x for x in candidates if all(dot(a, x) <= b+1e-12 for a, b in constraints)]
    return min(feasible, key=lambda x: .5*dot(x, matvec(H, x))+dot(q, x))


def record(u):
    s = [.9, .9+u[0], .9+sum(u)]
    rate = u[0]**2+(u[1]-u[0])**2
    stages = sum(int(x < 1.-1e-12) for x in s[:-1])
    return dict(inputs=u, states=s, input_rate_cost=rate,
                literal_finish_stage_cost=stages,
                literal_paper_objective=stages+rate,
                terminal_cost_to_completed_finish=0.)


def main():
    frozen = record(solve([0., 0.]))
    # Literal indicator at stage1 is fixed within each subproblem.
    crossed = record(solve([0., 0.], [([-1., 0.], -.1)]))
    before = record(solve([0., 0.], [([1., 0.], .1-1e-9)]))
    exact = min([crossed, before], key=lambda r: r['literal_paper_objective'])
    signed = record(solve([-10., -10.]))
    signed['signed_potential'] = -10.*(signed['states'][-1]-1.)
    assert all(abs(a-b) < 1e-9 for a, b in zip(frozen['inputs'], [.04, .06]))
    assert all(abs(a-b) < 1e-9 for a, b in zip(exact['inputs'], [.1, .1]))
    assert all(abs(a-b) < 1e-9 for a, b in zip(signed['inputs'], [.08, .12]))
    assert exact['literal_paper_objective'] < frozen['literal_paper_objective']
    assert exact['literal_paper_objective'] < signed['literal_paper_objective']
    result = dict(scope='scalar mathematical counterexample; not vehicle validation',
                  qp_solver_used='independent two-dimensional active-set enumeration; not OSQP',
                  horizon=2, finish=1., hard_terminal=[1., 1.1],
                  frozen_stage_solution=frozen, signed_potential_solution=signed,
                  fixed_finish_subproblems=[crossed, before],
                  exact_piecewise_solution=exact,
                  objective_gap_frozen=frozen['literal_paper_objective']-exact['literal_paper_objective'],
                  passed=True)
    out = Path(__file__).with_suffix('.json')
    out.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(result, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
