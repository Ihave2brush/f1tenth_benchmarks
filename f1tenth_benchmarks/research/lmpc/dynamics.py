"""Trace real actuator inputs while delegating integration to the original model.

Acceleration commands invert the existing PID at every physics substep. Steering
delay, rate limits, acceleration constraints and integration stay unchanged.
"""
import numpy as np

from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from f1tenth_benchmarks.simulator.dynamic_models import (
    pid, accl_constraints, steering_constraint, non_overshooting_steering_rate,
)
from f1tenth_benchmarks.research.core.continuous_sim import F1TenthSim_Continuous


def acceleration_target_speed(acceleration, current_speed, params):
    if not np.isfinite([acceleration, current_speed]).all():
        raise ValueError('nonfinite acceleration command')
    # Match all four branches of the original PID, including its v=0 branch.
    scale = 10. if current_speed > 0 else 2.
    denominator = params['max_v'] if acceleration > 0 else -params['min_v']
    gain = scale * params['max_a'] / denominator
    if not np.isfinite(gain) or gain <= 0:
        raise ValueError('invalid PID gain')
    return float(current_speed + acceleration / gain)


class TracedDynamics(DynamicsSimulator):
    def __init__(self, params):
        super().__init__(params)
        self.input_mode = 'target_speed'
        self.trace = []

    def update_pose(self, raw_steer, command):
        if not np.isfinite([raw_steer, command]).all():
            raise ValueError('nonfinite actuator command')
        p = self.params
        pre = self.state.copy()
        speed = command
        if self.input_mode == 'acceleration':
            speed = acceleration_target_speed(command, pre[3], dict(
                max_v=p['v_max'], min_v=p['v_min'], max_a=p['a_max']))
        elif self.input_mode != 'target_speed':
            raise ValueError('unknown input mode')
        delayed = 0. if len(self.steer_buffer) < self.steer_buffer_size else float(self.steer_buffer[-1])
        effective_target = (float(np.clip(delayed, p['s_min'], p['s_max']))
                            if self.steering_actuator_mode == 'non_overshoot' else delayed)
        requested_a, requested_sv = pid(speed, effective_target, pre[3], pre[2],
                                       p['sv_max'], p['a_max'], p['v_max'], p['v_min'])
        applied_a = accl_constraints(pre[3], requested_a, p['v_switch'],
                                     p['a_max'], p['v_min'], p['v_max'])
        applied_sv = steering_constraint(pre[2], requested_sv, p['s_min'],
                                         p['s_max'], p['sv_min'], p['sv_max'])
        if self.steering_actuator_mode == 'non_overshoot':
            applied_sv = non_overshooting_steering_rate(
                pre[2], effective_target, requested_sv, self.time_step,
                p['s_min'], p['s_max'], p['sv_min'], p['sv_max'])
        post = super().update_pose(raw_steer, speed)
        self.accel, self.steer_angle_vel = float(applied_a), float(applied_sv)
        self.trace.append(dict(input_mode=self.input_mode, command=[float(raw_steer), float(command)],
                               pid_target_speed=float(speed), delayed_target_steering=delayed,
                               requested_acceleration=float(requested_a),
                               applied_acceleration=self.accel,
                               applied_steering_rate=self.steer_angle_vel,
                               dt=self.time_step, pre_state=pre.tolist(), post_state=post.tolist()))
        if self.steering_actuator_mode == 'non_overshoot':
            self.trace[-1].update(steering_actuator_mode=self.steering_actuator_mode,
                                 effective_target_steering=effective_target)
        return post

    def reset(self, pose):
        self.trace = []
        return super().reset(pose)


class InstrumentedContinuousSim(F1TenthSim_Continuous):
    """Shared Continuous lifecycle with explicit speed/acceleration methods."""
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.dynamics_simulator = TracedDynamics(self.params)

    def _advance(self, action):
        self.dynamics_simulator.trace = []
        super()._advance(action)

    def step(self, action):
        self.dynamics_simulator.input_mode = 'target_speed'
        return super().step(action)

    def step_acceleration(self, action):
        self.dynamics_simulator.input_mode = 'acceleration'
        try:
            return super().step(action)
        finally:
            self.dynamics_simulator.input_mode = 'target_speed'
