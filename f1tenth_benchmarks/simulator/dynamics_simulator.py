import numpy as np
from numba import njit

from f1tenth_benchmarks.simulator.dynamic_models import (
    vehicle_dynamics_st, pid, non_overshooting_steering_rate,
)

'''
    params (dict, default={'mu': 1.0489, 'C_Sf':, 'C_Sr':, 'lf': 0.15875, 'lr': 0.17145, 'h': 0.074, 'm': 3.74, 'I': 0.04712, 's_min': -0.4189, 's_max': 0.4189, 'sv_min': -3.2, 'sv_max': 3.2, 'v_switch':7.319, 'a_max': 9.51, 'v_min':-5.0, 'v_max': 20.0, 'width': 0.31, 'length': 0.58}): dictionary of vehicle parameters.
    mu: surface friction coefficient
    C_Sf: Cornering stiffness coefficient, front
    C_Sr: Cornering stiffness coefficient, rear
    lf: Distance from center of gravity to front axle
    lr: Distance from center of gravity to rear axle
    h: Height of center of gravity
    m: Total mass of the vehicle
    I: Moment of inertial of the entire vehicle about the z axis
    s_min: Minimum steering angle constraint
    s_max: Maximum steering angle constraint
    sv_min: Minimum steering velocity constraint
    sv_max: Maximum steering velocity constraint
    v_switch: Switching velocity (velocity at which the acceleration is no longer able to create wheel spin)
    a_max: Maximum longitudinal acceleration
    v_min: Minimum longitudinal velocity
    v_max: Maximum longitudinal velocity
    width: width of the vehicle in meters
    length: length of the vehicle in meters
'''

class DynamicsSimulator:
    def __init__(self, params):
        np.random.seed(params.random_seed)
        self.time_step = params.timestep

        # These parameters are related to the vehicle model...,not simulation.
        self.params = vars(params)
        # Missing mode in historical configurations must retain the old plant.
        self.steering_actuator_mode = getattr(params, 'steering_actuator_mode', 'original')
        if self.steering_actuator_mode not in ('original', 'non_overshoot'):
            raise ValueError('unknown steering_actuator_mode')
        if self.steering_actuator_mode == 'non_overshoot':
            limits = [self.time_step, params.s_min, params.s_max,
                      params.sv_min, params.sv_max]
            if (not np.isfinite(limits).all() or self.time_step <= 0
                    or not params.s_min <= 0. <= params.s_max
                    or not params.sv_min < 0. < params.sv_max):
                raise ValueError('invalid non_overshoot timestep or steering limits')

        # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]
        self.state = np.zeros((7, ))

        # control inputs
        self.accel = 0.0
        self.steer_angle_vel = 0.0

        # steering delay buffer
        self.steer_buffer = np.empty((0, ))
        self.steer_buffer_size = 2

    def update_pose(self, raw_steer, vel):
        """
        Steps the vehicle's physical simulation

        Args:
            steer (float): desired steering angle
            vel (float): desired longitudinal velocity

        Returns:
            current_scan
        """
        # state is [x, y, steer_angle, vel, yaw_angle, yaw_rate, slip_angle]

        if self.steering_actuator_mode == 'non_overshoot':
            if (not np.isfinite(raw_steer) or not np.isfinite(self.state[2])
                    or not self.params['s_min'] <= self.state[2] <= self.params['s_max']):
                raise ValueError('non_overshoot requires a finite command and in-bounds wheel angle')

        # steering delay
        steer = 0.
        if self.steer_buffer.shape[0] < self.steer_buffer_size:
            steer = 0.
            self.steer_buffer = np.append(raw_steer, self.steer_buffer)
        else:
            steer = self.steer_buffer[-1]
            self.steer_buffer = self.steer_buffer[:-1]
            self.steer_buffer = np.append(raw_steer, self.steer_buffer)

        # Preserve raw commands and FIFO timing; bound the consumed target only.
        if self.steering_actuator_mode == 'non_overshoot':
            steer = np.clip(steer, self.params['s_min'], self.params['s_max'])
            previous_steer = self.state[2]

        # steering angle velocity input to steering velocity acceleration input
        accl, sv = pid(vel, steer, self.state[3], self.state[2], self.params['sv_max'], self.params['a_max'], self.params['v_max'], self.params['v_min'])
        if self.steering_actuator_mode == 'non_overshoot':
            sv = non_overshooting_steering_rate(
                self.state[2], steer, sv, self.time_step,
                self.params['s_min'], self.params['s_max'],
                self.params['sv_min'], self.params['sv_max'])
        
        f = vehicle_dynamics_st(
            self.state,
            np.array([sv, accl]),
            self.params['mu'],
            self.params['C_Sf'],
            self.params['C_Sr'],
            self.params['lf'],
            self.params['lr'],
            self.params['h'],
            self.params['m'],
            self.params['I'],
            self.params['s_min'],
            self.params['s_max'],
            self.params['sv_min'],
            self.params['sv_max'],
            self.params['v_switch'],
            self.params['a_max'],
            self.params['v_min'],
            self.params['v_max'])
        self.state = self.state + self.time_step * f
        if self.steering_actuator_mode == 'non_overshoot':
            # Guard only floating-point endpoint rounding after the rate cap.
            self.state[2] = np.clip(self.state[2],
                max(self.params['s_min'], min(previous_steer, steer)),
                min(self.params['s_max'], max(previous_steer, steer)))
        
        # bound yaw angle
        if self.state[4] > 2*np.pi:
            self.state[4] = self.state[4] - 2*np.pi
        elif self.state[4] < 0:
            self.state[4] = self.state[4] + 2*np.pi

        return self.state

    def reset(self, pose):
        # clear control inputs
        self.accel = 0.0
        self.steer_angle_vel = 0.0
        # clear state
        self.state = np.zeros((7, ))
        self.state[0:2] = pose[0:2]
        self.state[4] = pose[2]
        self.steer_buffer = np.empty((0, ))

        return self.state
