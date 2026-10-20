"""Frozen observed ESP windows: cost-only audit, never a simulator action.

Capture the current QP matrices without solving/applying its controller.
Use fixed real-data ATV and terminal columns for every cost comparison.
Enumerate unknown finish-status masks; LP bounds prune provably fixed stages.
"""
import hashlib
import argparse
import itertools
import json
from dataclasses import replace
from pathlib import Path
from time import perf_counter
from unittest.mock import patch

import numpy as np
import osqp
from scipy import sparse
from scipy.optimize import linprog

from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.lmpc.config import load_config
from f1tenth_benchmarks.research.lmpc.controller import solve_qp
from f1tenth_benchmarks.research.lmpc.database import LapDatabase
from f1tenth_benchmarks.research.lmpc.paper_model import PaperLocalModel
from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3
from f1tenth_benchmarks.research.lmpc.safe_set import select_local_safe_set


class Captured(Exception):
    pass


def capture(initial, nominal, inputs, models, terminal, previous, settings):
    data = {}

    class Capture:
        def setup(self, **kwargs):
            data.update(kwargs)
            raise Captured()

    with patch('f1tenth_benchmarks.research.lmpc.controller.osqp.OSQP', Capture):
        try:
            solve_qp(initial, nominal, inputs, models, terminal, previous, settings)
        except Captured:
            pass
    assert data
    return data


def lp_constraints(a, low, high):
    dense = a.toarray()
    eq = np.isfinite(low) & np.isfinite(high) & (low == high)
    upper = ~eq & np.isfinite(high)
    lower = ~eq & np.isfinite(low)
    return dict(A_eq=dense[eq], b_eq=low[eq],
                A_ub=np.vstack([dense[upper], -dense[lower]]),
                b_ub=np.r_[high[upper], -low[lower]],
                bounds=[(None, None)]*a.shape[1], method='highs')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='Data/research/lmpc/r0_finish_cost_audit_20261013_003')
    parser.add_argument('--max-iterations', type=int, default=1000000)
    args = parser.parse_args()
    config_path = Path('params/research/lmpc_paper.yaml')
    cfg = load_b3(config_path)
    base = load_config(cfg['base_config'])
    db = LapDatabase.load(cfg['database'])
    geometry = load_geometry(base.adaptation['geometry_path'], base.adaptation['map_yaml'], inspect=True)
    model = PaperLocalModel(db, geometry, diagnostic_only=True)
    lap = db.laps[1]
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=False)
    n, nx, nu = 12, 6, 2
    us, ls = (n+1)*nx, (n+1)*nx+n*nu
    eps = 1e-4
    boundary_tol = 1e-6
    cases = []
    for remaining in [20, 14, 8, 2]:
        start = lap.terminal_index-remaining
        nominal = lap.states[start:start+n+1].copy()
        inputs = lap.commands[start:start+n].copy()
        initial, previous = nominal[0].copy(), lap.commands[start-1].copy()
        terminal = select_local_safe_set(db, nominal[-1])
        models = [model.linearize(nominal[k], inputs[k]) for k in range(n)]
        qp = capture(initial, nominal, inputs, models, terminal, previous, cfg['settings'])
        P, q0, A = qp['P'], qp['q'], qp['A']
        lo, hi = qp['l'].copy(), qp['u'].copy()
        constraints = lp_constraints(A, lo, hi)
        case = dict(remaining_recorded_steps=remaining, raw_source_id=int(lap.sample_ids[start]),
                    initial_state=initial.tolist(), nominal=nominal.tolist(),
                    terminal_source_ids=list(terminal.provenance),
                    rank_diagnostics=[m[3]['paper_regressors'] for m in models])
        print('window', remaining, 'source', case['raw_source_id'], flush=True)
        support = linprog(np.zeros(A.shape[1]), **constraints)
        case['base_lp_status'] = int(support.status)
        if not support.success:
            case.update(solved_comparison=False, reason='base fixed ATV/terminal/constraints infeasible',
                        message=support.message)
            cases.append(case)
            (out/'progress.json').write_text(json.dumps(cases, indent=2, allow_nan=False))
            print('base infeasible', flush=True)
            continue

        fixed, ambiguous, bounds = {}, [], []
        for k in range(1, n):
            direction = np.zeros(A.shape[1]); direction[k*nx+4] = 1.
            minimum = linprog(direction, **constraints)
            maximum = linprog(-direction, **constraints)
            assert minimum.success and maximum.success
            lower = float(minimum.fun+nominal[k, 4])
            upper = float(-maximum.fun+nominal[k, 4])
            bounds.append(dict(stage=k, min_s=lower, max_s=upper))
            if upper < db.length-eps:
                fixed[k] = 1
            elif lower > db.length+boundary_tol:
                fixed[k] = 0
            else:
                ambiguous.append(k)
        case.update(stage_lp_bounds=bounds, ambiguous_stages=ambiguous,
                    theoretical_mask_count=2**len(ambiguous), fixed_stages=fixed)
        rows = sparse.lil_matrix((n-1, A.shape[1]))
        for k in range(1, n): rows[k-1, k*nx+4] = 1.
        full_A = sparse.vstack([A, rows.tocsc()], format='csc')
        full_low = np.r_[lo, np.full(n-1, -np.inf)]
        full_high = np.r_[hi, np.full(n-1, np.inf)]
        solver = osqp.OSQP()
        solver.setup(P=P, q=q0, A=full_A, l=full_low, u=full_high,
                     verbose=False, eps_abs=1e-7, eps_rel=1e-7, max_iter=args.max_iterations,
                     polish=True, adaptive_rho=True, adaptive_rho_interval=50)
        seed = np.zeros(A.shape[1])
        nearest = np.argmin(np.linalg.norm(terminal.states-nominal[-1,None].T, axis=0))
        seed[ls+nearest] = 1.

        def reset_seed():
            solver.warm_start(x=seed.copy(), y=np.zeros(full_A.shape[0]))

        def extract(result, costs):
            if result.info.status_val != 1:
                return dict(solved=False, status=result.info.status,
                            iterations=int(result.info.iter), solve_time_s=float(result.info.run_time))
            decision = result.x
            states = nominal+decision[:us].reshape(n+1, nx)
            controls = inputs+decision[us:ls].reshape(n, nu)
            lam = decision[ls:]
            current_h = int(np.sum(states[:-1, 4] < db.length-boundary_tol))
            delta = np.vstack([controls[0]-previous, np.diff(controls, axis=0)])
            rate = float(np.sum(delta*delta*np.asarray(cfg['settings']['input_rate_weights'])))
            dynamics = max(float(np.max(abs(states[k+1]-(models[k][0]@states[k]+models[k][1]@controls[k]+models[k][2])))) for k in range(n))
            terminal_error = float(np.max(abs(states[-1]-terminal.states@lam)))
            signed_cost = float(terminal.costs@lam)
            completed_goal_reference = float(np.maximum(terminal.costs, 0.)@lam)
            return dict(solved=True, first_input=controls[0].tolist(), states=states.tolist(),
                        controls=controls.tolist(), lambda_weights=lam.tolist(), literal_stage_cost=current_h,
                        rate_cost=rate, used_terminal_cost=float(costs@lam), signed_terminal_cost=signed_cost,
                        objective_with_same_signed_J=current_h+rate+signed_cost,
                        objective_with_clamped_reference_J=current_h+rate+completed_goal_reference,
                        dynamics_residual=dynamics, terminal_residual=terminal_error,
                        simplex_residual=float(abs(lam.sum()-1)), min_lambda=float(lam.min()),
                        solve_time_s=float(result.info.run_time), status=result.info.status,
                        near_finish_stages=[k for k in range(n) if abs(states[k, 4]-db.length)<boundary_tol])

        signed = terminal.costs.copy()
        author = signed.copy()
        # Transcription of pinned BARC _SelectPoints cost-only branch:
        # old xPred is supplied as the same observed nominal to both sources.
        offsets = {}
        if np.any(nominal[:, 4] > db.length):
            for lap_id in [1, 2]:
                offset = float(db.laps[1].terminal_index) if lap_id == 1 else float(start+n-np.sum(nominal[:, 4]>db.length))
                offsets[lap_id] = offset
                for j, provenance in enumerate(terminal.provenance):
                    if provenance['lap_id'] == lap_id: author[j] += offset
        else:
            offsets = {1:0., 2:0.}
        case['author_offsets'] = offsets
        case['author_scope'] = 'cost-only transcription; same paper s-nearest terminal columns and hard constraints, not full author controller'
        for label, costs in [('current', signed), ('author_cost_only', author)]:
            q = q0.copy(); q[ls:] = costs-costs.min()
            solver.update(q=q, l=full_low, u=full_high)
            reset_seed()
            case[label] = extract(solver.solve(), costs)

        reference_summaries = {}
        for label, costs in [('paper_h_same_signed_J', signed),
                             ('paper_h_author_cost_J', author),
                             ('paper_h_clamped_reference_J', np.maximum(signed, 0.))]:
            q = q0.copy(); q[ls:] = costs-costs.min()
            best, outcomes = None, []
            started = perf_counter()
            for index, bits in enumerate(itertools.product([0, 1], repeat=len(ambiguous))):
                mask = dict(fixed); mask.update(zip(ambiguous, bits))
                low, high = full_low.copy(), full_high.copy()
                for k in range(1, n):
                    if mask[k]: high[len(hi)+k-1] = db.length-eps-nominal[k, 4]
                    else: low[len(lo)+k-1] = db.length-nominal[k, 4]
                solver.update(q=q, l=low, u=high)
                reset_seed()
                result = solver.solve()
                item = extract(result, costs)
                if item['solved']:
                    # Check the branch in physical units, not OSQP's status alone.
                    actual = np.array(item['states'])
                    violation = max([max(0., actual[k,4]-(db.length-eps)) if mask[k]
                                     else max(0., db.length-actual[k,4]) for k in range(1,n)])
                    item['branch_violation_m'] = float(violation)
                    expected_h = int(initial[4]<db.length)+sum(mask.values())
                    item['mask_stage_cost'] = expected_h
                    item['reference_objective'] = expected_h+item['used_terminal_cost']+item['rate_cost']
                    admissible = (violation <= 1e-6 and item['literal_stage_cost']==expected_h
                                  and max(item['dynamics_residual'],item['terminal_residual'],item['simplex_residual'])<=1e-5
                                  and item['min_lambda']>=-1e-7)
                    item['admissible'] = bool(admissible)
                    if admissible and (best is None or item['reference_objective']<best['reference_objective']): best=item
                outcomes.append(dict(mask_id=index, status=item['status'], solved=item['solved'],
                                     admissible=item.get('admissible',False), objective=item.get('reference_objective')))
                if index % 32 == 0: print(label, 'mask', index, '/', 2**len(ambiguous), flush=True)
            reference_summaries[label] = dict(best=best, outcomes=outcomes,
                mask_count=len(outcomes), elapsed_s=perf_counter()-started,
                all_masks_examined=True, strict_before_margin_m=eps, classification_tolerance_m=boundary_tol)
        case['references'] = reference_summaries
        case['solved_comparison'] = bool(case['current']['solved'] and case['author_cost_only']['solved']
            and all(x['best'] is not None for x in reference_summaries.values()))
        if case['solved_comparison']:
            current = case['current']
            best = reference_summaries['paper_h_same_signed_J']['best']
            case['cost_only_differences'] = dict(
                author_first_input_delta=(np.array(case['author_cost_only']['first_input'])-current['first_input']).tolist(),
                literal_h_first_input_delta=(np.array(best['first_input'])-current['first_input']).tolist(),
                current_minus_literal_h_objective=current['objective_with_same_signed_J']-best['reference_objective'])
            author_best = reference_summaries['paper_h_author_cost_J']['best']
            author_current = case['author_cost_only']
            current_author_score = author_current['literal_stage_cost']+author_current['used_terminal_cost']+author_current['rate_cost']
            case['cost_only_differences']['author_minus_literal_h_author_J'] = current_author_score-author_best['reference_objective']
            case['cost_only_differences']['author_literal_h_first_input_delta'] = (np.array(author_best['first_input'])-author_current['first_input']).tolist()
        cases.append(case)
        (out/'progress.json').write_text(json.dumps(cases, indent=2, allow_nan=False))
        print('comparison solved', case['solved_comparison'], flush=True)
    report = dict(scope='offline frozen observed ESP windows; no plant or controller action',
        config_sha256=hashlib.sha256(config_path.read_bytes()).hexdigest(), source_hashes=db.metadata['source_hashes'],
        code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        python_versions=dict(numpy=np.__version__,osqp=osqp.__version__),
        numerical_policy=dict(max_iterations=args.max_iterations, absolute_tolerance=1e-7,
                              relative_tolerance=1e-7, shared_seed='same nearest terminal one-hot, zero deviations/duals; reset before each solve',
                              note='audit iteration limit may exceed production; no tolerance relaxation'),
        paper_reference_limits=['same-signed-J isolates h only; signed suffix is not asserted as paper terminal definition',
            'diagnostic-only unidentified regression; production controller rejects these control effects',
            'clamped suffix is an explicit completed-goal reference, not claimed as author repetitive-lap cost',
            '1e-4 strict-before gap and 1e-6 boundary convention; not symbolic exactness',
            'QP constraints copied from current code, not certified equal to paper X/U',
            'full initialization database identification is not held-out model validation'],
        requested_windows=len(cases), solved_windows=sum(c.get('solved_comparison',False) for c in cases),cases=cases)
    (out/'comparison.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    print(json.dumps(dict(output=str(out), solved_windows=report['solved_windows'],requested_windows=len(cases))), flush=True)


if __name__ == '__main__':
    main()
