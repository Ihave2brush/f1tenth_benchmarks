"""Paper six-state observation, separate from legacy polyline lap progress."""
import numpy as np

from f1tenth_benchmarks.research.core.geometry import FrenetTracker

STATE_COLUMNS = ['vx', 'vy', 'r', 'e_psi', 's_unwrapped', 'e_y']


class StateObservationError(ValueError):
    def __init__(self, message, frenet):
        super().__init__(message)
        self.frenet = frenet


def paper_state(physical_state, frenet, timestamp, geometry_id):
    physical = np.asarray(physical_state, dtype=float)
    if physical.shape != (7,) or not np.isfinite(physical).all():
        raise ValueError('physical state must be finite [X,Y,steer,v,yaw,r,beta]')
    if not frenet.get('valid'):
        raise ValueError('invalid Frenet observation: ' + str(frenet.get('reason')))
    if frenet.get('geometry_id') != geometry_id or frenet.get('frame_id') != 'map':
        raise ValueError('Frenet geometry/frame mismatch')
    if (not np.isfinite(timestamp) or frenet.get('timestamp') != timestamp
            or not frenet.get('freshness_verified') or not frenet.get('version_verified')):
        raise ValueError('Frenet observation timestamp/version is not verified')
    # The dynamic model translates with v*cos(yaw+beta), v*sin(yaw+beta).
    v, r, beta = physical[3], physical[5], physical[6]
    result = np.array([v*np.cos(beta), v*np.sin(beta), r,
                       frenet['e_psi'], frenet['s_unwrapped'], frenet['d']])
    if not np.isfinite(result).all():
        raise ValueError('nonfinite paper state')
    return result


class StateObserver:
    """Read-only geometry inspection. Does not grant a planning permission."""
    def __init__(self, geometry):
        self.geometry = geometry
        # Model position and yaw refer to the same simulated centre of mass.
        self.tracker = FrenetTracker(geometry, source_reference='sim_cog',
                                    offset=(0., 0., 0.), offset_calibrated=True)

    def observe(self, observation):
        state = np.asarray(observation['vehicle_state'], dtype=float)
        if state.shape != (7,) or not np.isfinite(state).all():
            raise ValueError('invalid physical observation')
        timestamp = float(observation['simulation_time'])
        frenet = self.tracker.update(state[:2], float(state[4]), timestamp,
                                    now=timestamp, geometry_id=self.geometry.geometry_id,
                                    signed_speed=float(state[3]))
        try:
            result = paper_state(state, frenet, timestamp, self.geometry.geometry_id)
        except ValueError as exc:
            raise StateObservationError(str(exc), frenet) from exc
        return result, frenet
