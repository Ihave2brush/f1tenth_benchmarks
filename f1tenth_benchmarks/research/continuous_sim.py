"""Research lifecycle: one reset, post-step tracking, no implicit file writes."""
import math
import numpy as np
from f1tenth_benchmarks.simulator.f1tenth_sim import F1TenthSim_TrueLocation
from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from f1tenth_benchmarks.simulator.laser_models import ScanSimulator2D
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file_with_extras
from .lap_tracking import ContinuousLapTracker, UpdateResult


class F1TenthSim_Continuous(F1TenthSim_TrueLocation):
    def __init__(self, map_name, planner_name='ResearchContinuousPP', test_id='continuous',
                 extra_params=None, tracker_config=None, timeout=300., centreline_xy=None):
        # Deliberately bypass base __init__: it enables profiling and owns legacy
        # automatic logging. Reuse physical components and collision method only.
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be positive and finite')
        self.params = load_parameter_file_with_extras('simulator_params', extra_params or {})
        if (not isinstance(self.params.n_sim_steps, int) or self.params.n_sim_steps <= 0
                or not math.isfinite(self.params.timestep) or self.params.timestep <= 0):
            raise ValueError('invalid dynamics timestep/substep count')
        self.map_name, self.planner_name, self.test_id = map_name, planner_name, test_id
        self.tracker = (ContinuousLapTracker.from_csv(f'maps/{map_name}_centerline.csv', tracker_config)
                        if centreline_xy is None else ContinuousLapTracker(centreline_xy, tracker_config))
        self.dynamics_simulator = DynamicsSimulator(self.params)
        self.scan_simulator = ScanSimulator2D(self.params.num_beams, self.params.fov,
                                              map_name, self.params.random_seed)
        self.timeout = timeout
        self.current_state = np.zeros(7)
        self.current_time = 0.
        self.total_steps = self.control_steps = self.reset_count = 0
        self.initialization_time = None
        self.initialized = False
        self.terminal_reason = None
        self.collision = self.lap_complete = False
        self.scan = None
        self.last_tracking_result = None
        self.last_pre_step_state = self.last_pre_step_buffer = None

    def __del__(self):
        # Explicit logger.finalize is the sole persistence boundary.
        pass

    @property
    def elapsed_time(self):
        return 0. if self.initialization_time is None else self.current_time-self.initialization_time

    def _advance(self, action):
        self.last_pre_step_state = self.dynamics_simulator.state.copy()
        self.last_pre_step_buffer = self.dynamics_simulator.steer_buffer.copy()
        for _ in range(self.params.n_sim_steps):
            self.current_state = self.dynamics_simulator.update_pose(action[0], action[1])
            self.current_time += self.params.timestep
        self.total_steps += 1

    def _inspect(self, initializing=False):
        self.lap_complete = False
        pose = self.current_state[[0, 1, 4]].copy()
        if not np.isfinite(self.current_state).all():
            self.terminal_reason = 'invalid_state'
        else:
            self.collision = self.check_vehicle_collision(pose)
            if self.collision:
                self.terminal_reason = 'collision'
            elif not initializing and self.elapsed_time >= self.timeout - 1e-12:
                self.terminal_reason = 'timeout'
        if self.terminal_reason:
            # Collision/timeout sample cannot create a finish event. No recovery
            # is inferred; actual physical sample remains available to the logger.
            self.last_tracking_result = UpdateResult(False, self.terminal_reason+'_terminal', None)
        else:
            self.last_tracking_result = self.tracker.update(pose, self.current_time,
                                                           float(self.current_state[3]))
            if not self.last_tracking_result.valid:
                self.terminal_reason = 'projection_invalid'
            else:
                event = self.last_tracking_result.event
                self.lap_complete = event is not None and event.kind == 'full'
        if np.isfinite(pose).all():
            self.scan = self.scan_simulator.scan(pose)
        return self.build_observation(pose), self.terminal_reason is not None

    def reset(self, start_pose=None, initial_speed=0.):
        """One initial reset. Default pose is fixed centreline s=0/tangent.

        Retain one four-substep initialization tick. Tracking/timing starts at
        its actual final sample, not at t=0. Explicit moving starts are allowed.
        """
        if self.reset_count:
            raise RuntimeError('continuous session permits only one reset')
        if start_pose is None:
            tangent = self.tracker.finish_tangent
            start_pose = np.r_[self.tracker.finish_origin, math.atan2(tangent[1], tangent[0])]
        start_pose = np.asarray(start_pose, dtype=float)
        if start_pose.shape != (3,) or not np.isfinite(start_pose).all():
            raise ValueError('start_pose must be finite [x,y,yaw]')
        if not math.isfinite(initial_speed) or abs(initial_speed) > self.tracker.config.max_speed:
            raise ValueError('invalid initial speed')
        self.current_state = self.dynamics_simulator.reset(start_pose)
        self.reset_count += 1
        self.dynamics_simulator.state[3] = initial_speed
        self._advance(np.array([0., initial_speed]))
        self.initialization_time = self.current_time
        self.initialized = True
        observation, done = self._inspect(initializing=True)
        return observation, done, start_pose.copy()

    def step(self, action):
        if not self.initialized:
            raise RuntimeError('reset before step')
        if self.terminal_reason:
            raise RuntimeError('cannot step a terminal session')
        action = np.asarray(action, dtype=float)
        if action.shape != (2,) or not np.isfinite(action).all():
            raise ValueError('action must be finite [steering,speed]')
        self._advance(action)
        self.control_steps += 1
        return self._inspect()

    def build_observation(self, pose):
        result = self.last_tracking_result
        sample = None if result is None or not result.valid else result.sample
        wrapped = None if sample is None else sample.projection.s_wrapped
        return dict(scan=None if self.scan is None else self.scan.copy(),
                    vehicle_state=self.current_state.copy(), pose=pose.copy(),
                    vehicle_speed=float(self.current_state[3]), collision=self.collision,
                    lap_complete=self.lap_complete, laptime=self.elapsed_time,
                    simulation_time=self.current_time,
                    progress=None if wrapped is None else wrapped/self.tracker.L,
                    centre_line_progress=None if wrapped is None else wrapped/self.tracker.L,
                    s_wrapped=wrapped, s_unwrapped=None if sample is None else sample.s_unwrapped,
                    projection_valid=result is not None and result.valid,
                    projection_reason=None if result is None else result.reason,
                    completed_full_laps=self.tracker.completed_full_laps,
                    lap_event=None if result is None else result.event,
                    terminal_reason=self.terminal_reason)
