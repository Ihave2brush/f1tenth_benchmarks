"""A8 no-delay per-action ESP simulation gate. Never changes global core permission flags.

This separate experiment interface authorizes only a supplied finite horizon
after exact candidate queries and original-model shadow integration. It cannot
be reused as a general geometry planning license or a continuous safety proof.
"""
from types import SimpleNamespace

import numpy as np

from .no_delay import NoDelayTracedDynamics as TracedDynamics
from .state import StateObserver


class ScopedPlanningGate:
    def __init__(self, geometry, validator, params, settings):
        self.geometry, self.validator, self.params, self.settings = geometry, validator, params, settings
        if geometry.planning_allowed:
            raise ValueError('B3 scope expects the unchanged diagnostic geometry')

    def check(self, physical, buffer, predicted, controls):
        check = self.validator.check(predicted[:, :6], controls)
        if not check['valid']:
            return dict(valid=False, reason='candidate_'+str(check['reason']), candidate=check)
        shadow = TracedDynamics(SimpleNamespace(**self.params))
        shadow.state = physical.copy(); shadow.steer_buffer = buffer.copy()
        shadow.input_mode = 'acceleration'
        observer = StateObserver(self.geometry)
        observer.observe(dict(vehicle_state=physical, simulation_time=0.))
        checked = 0; states = []
        # Receding horizon applies only u0. Checking hypothetical u1..uN as an
        # irrevocable open-loop sequence is an extra veto absent from the paper.
        shadow_mode=self.settings.get('shadow_mode','full_horizon')
        shadow_controls=controls[:1] if shadow_mode=='first_action' else controls
        for k, action in enumerate(shadow_controls):
            for tick in range(round(.1/self.params['timestep'])):
                state = shadow.update_pose(*action)
                footprint = self.validator.footprint.check([state[0], state[1], state[4]])
                checked += 1
                if not footprint['valid']:
                    return dict(valid=False, reason='shadow_'+str(footprint['reason']), horizon_step=k, tick=tick)
            try:
                x, fr = observer.observe(dict(vehicle_state=shadow.state, simulation_time=(k+1)*.1))
            except ValueError as exc:
                return dict(valid=False, reason='shadow_frenet_'+str(exc), horizon_step=k)
            if (np.hypot(*x[:2]) > self.settings['max_speed_mps']+1e-8 or x[0] < -1e-8
                    or abs(x[3]) > self.settings['max_abs_heading_rad']
                    or abs(x[5]) > self.settings['max_abs_lateral_m']):
                return dict(valid=False, reason='shadow_state_limit', horizon_step=k, state=x.tolist())
            states.append(x)
        return dict(valid=True, reason=None, geometry_id=self.geometry.geometry_id,
                    scope='single_first_action_esp_simulation', global_planning_allowed=False,
                    continuous_verified=False, candidate=check,
                    shadow_mode=shadow_mode,
                    shadow_physics_endpoints=checked, shadow_states=np.asarray(states).tolist())
