"""Paper local convex terminal set, LP Q-function, and successor advancement."""
from dataclasses import dataclass

import numpy as np
from scipy.optimize import linprog


class TerminalSupportError(ValueError):
    """Requested local window lacks observed successful successor data."""


def weights(value, count):
    result = np.asarray(value, float)
    if (result.shape != (count,) or not np.isfinite(result).all()
            or np.min(result) < -1e-9 or abs(result.sum()-1) > 1e-9):
        raise ValueError('lambda must be a finite simplex vector')
    result = np.maximum(result, 0.)
    return result/result.sum()


@dataclass(frozen=True)
class LocalSafeSet:
    states: np.ndarray
    successors: np.ndarray
    costs: np.ndarray
    successor_costs: np.ndarray
    provenance: tuple
    geometry_id: str
    dt: float

    def combination(self, lam):
        return self.states @ weights(lam, self.states.shape[1])

    def cost(self, lam):
        return float(self.costs @ weights(lam, self.states.shape[1]))

    def advance(self, lam):
        """Equation (10): same lambda, classified observed/periodic successors."""
        lam = weights(lam, self.states.shape[1])
        return dict(candidate=self.successors @ lam,
                    continuation_cost=float(self.successor_costs @ lam),
                    previous_cost=float(self.costs @ lam))

    def q_function(self, terminal_state):
        """Equation (7). LP is offline B2 validation; B3 shares lambda in its QP."""
        state = np.asarray(terminal_state, float)
        if state.shape != (6,) or not np.isfinite(state).all():
            raise ValueError('terminal state must be a finite six-state vector')
        # Fixed-speed history has near-dependent rows. Use an affine basis of
        # the data itself, not target-centered rows whose roundoff changes rank.
        # Directions below 1e-8 are below the 1e-7 original-state residual gate.
        # Check omitted affine directions before solving and all original rows after.
        origin = self.states[:, 0]
        centered = self.states-origin[:, None]
        left, singular, right = np.linalg.svd(centered, full_matrices=False)
        rank = int(np.sum(singular > 1e-8))
        offset = state-origin
        projected = left[:, :rank] @ (left[:, :rank].T @ offset)
        if np.max(np.abs(offset-projected)) > 1e-7:
            raise TerminalSupportError('terminal state outside local affine span')
        equality = np.vstack([right[:rank], np.ones(self.states.shape[1])])
        rhs = np.r_[(left[:, :rank].T @ offset)/singular[:rank], 1.]
        result = linprog(self.costs, A_eq=equality, b_eq=rhs,
                         bounds=(0., None), method='highs',
                         options={'primal_feasibility_tolerance':1e-9,
                                  'dual_feasibility_tolerance':1e-9})
        if not result.success:
            raise TerminalSupportError('terminal state outside local convex hull: '+result.message)
        lam = weights(result.x, self.states.shape[1])
        residual = float(np.max(np.abs(self.states @ lam-state)))
        if residual > 1e-7:
            raise TerminalSupportError('Q-function equality residual exceeds tolerance')
        return dict(value=float(self.costs @ lam), lambda_weights=lam,
                    equality_residual=residual, equality_rank=rank,
                    solver='scipy_highs_linear_program')


def select_local_safe_set(database, candidate, cycle=0, points_per_lap=20, history_laps=2):
    """Select by ||D(x-z)||^2 = (s-z_s)^2, deterministic ties by source row.

    cycle is an explicit common s translation, never periodic trajectory fabrication.
    Every selected column needs an observed or explicitly audited periodic successor.
    """
    state = np.asarray(candidate, float)
    if state.shape != (6,) or not np.isfinite(state).all():
        raise ValueError('candidate must be a finite six-state vector')
    if isinstance(cycle, bool) or not isinstance(cycle, int):
        raise ValueError('cycle must be an explicit integer')
    if points_per_lap != 20 or history_laps != 2:
        raise ValueError('B2 uses paper K=20 and latest two successful laps')
    if len(database.laps) < history_laps:
        raise TerminalSupportError('two completed successful source laps required')
    columns, successors, costs, successor_costs, provenance = [], [], [], [], []
    for lap in database.laps[-history_laps:]:
        pool = lap.terminal_pool(cycle)
        shifted = pool['states']
        if len(shifted) < points_per_lap:
            raise TerminalSupportError('insufficient local data in lap '+str(lap.lap_id))
        lower = min(shifted[0, 4], cycle*database.length) if lap.periodic_tail is not None else shifted[0, 4]
        if not lower-1e-9 <= state[4] <= shifted[-1, 4]+1e-9:
            raise TerminalSupportError('candidate outside observed s support in lap '+str(lap.lap_id))
        distance = (shifted[:, 4]-state[4])**2
        nearest = np.lexsort((np.arange(len(shifted)), distance))[:points_per_lap]
        if any(pool['transition_kinds'][i] == 'unavailable' for i in nearest):
            raise TerminalSupportError('nearest window needs unobserved successor in lap '+str(lap.lap_id))
        for i in np.sort(nearest):
            columns.append(shifted[i]); successors.append(shifted[i+1])
            costs.append(pool['costs'][i]); successor_costs.append(pool['costs'][i+1])
            provenance.append(dict(lap_id=lap.lap_id, local_index=int(i),
                                   sample_id=int(pool['sample_ids'][i]),
                                   successor_sample_id=int(pool['sample_ids'][i+1]),
                                   extension=bool(i>lap.terminal_index),
                                   transition_kind=pool['transition_kinds'][i],
                                   measured_transition=pool['transition_kinds'][i]=='observed',
                                   identification_eligible=pool['transition_kinds'][i]=='observed',
                                   cycle=cycle, command=pool['commands'][i].tolist(),
                                   effective_input=pool['effective_inputs'][i].tolist()))
    arrays = [np.asarray(columns).T, np.asarray(successors).T,
              np.asarray(costs, float), np.asarray(successor_costs, float)]
    for array in arrays:
        array.setflags(write=False)
    return LocalSafeSet(*arrays, tuple(provenance), database.geometry_id, database.dt)


def initial_terminal_candidate(database, horizon=12, cycle=0):
    """Equation (10) at t=0: x_N from latest completed successful source lap."""
    if horizon != 12 or isinstance(cycle, bool) or not isinstance(cycle, int):
        raise ValueError('paper horizon N=12 and integer cycle required')
    lap = database.laps[-1]
    if horizon >= len(lap.states)-1:
        raise TerminalSupportError('latest source lap lacks horizon successor')
    return lap.shifted_states(cycle)[horizon].copy()
