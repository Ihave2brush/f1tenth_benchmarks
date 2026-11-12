"""Synchronous cars with independent baseline dynamics and three-disc sensing.

Separate research entry point: existing single-car experiments are unchanged.
Contact and clearance are envelope events, not rigid-body impact responses.
"""
import math
from itertools import combinations

import numpy as np

from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from f1tenth_benchmarks.simulator.laser_models import ScanSimulator2D
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file_with_extras
from ..core.geometry.map_raster import load_map
from .discs import ThreeDiscBody, OccupancyMap, pair_intersects, sweep_samples, opponent_scan


class MultiCarSim:
    def __init__(self, map_name, count=2, control_dt=.1, wall_clearance=.05,
                 pair_clearance=.05, max_sweep_motion=.01, extra_params=None,
                 enable_lidar=True, occupancy_map=None):
        self.params = load_parameter_file_with_extras('simulator_params', extra_params or {})
        dt = self.params.timestep
        if (not isinstance(count, int) or count < 2 or
                not np.isfinite([control_dt, dt, wall_clearance, pair_clearance, max_sweep_motion]).all() or
                min(control_dt, dt, max_sweep_motion) <= 0 or min(wall_clearance, pair_clearance) < 0):
            raise ValueError('invalid multi-car configuration')
        self.substeps = round(control_dt/dt)
        if self.substeps < 1 or not math.isclose(self.substeps*dt, control_dt, abs_tol=1e-10):
            raise ValueError('control_dt must be a multiple of physics timestep')
        self.cars = [DynamicsSimulator(self.params) for _ in range(count)]
        self.bodies = [ThreeDiscBody(self.params.vehicle_length, self.params.vehicle_width) for _ in self.cars]
        self.enable_lidar = enable_lidar
        self.scanners = [None] * count
        if enable_lidar:
            self.scanners = [ScanSimulator2D(self.params.num_beams, self.params.fov, map_name,
                                            self.params.random_seed+i) for i in range(count)]
        if occupancy_map is None:
            occupancy_map = (OccupancyMap.from_scan(self.scanners[0]) if enable_lidar
                             else OccupancyMap.from_raster(load_map(f'maps/{map_name}.yaml')))
        self.map = occupancy_map
        self.wall_clearance, self.pair_clearance = wall_clearance, pair_clearance
        self.max_sweep_motion = max_sweep_motion
        self.time = 0.
        self.physics_tick = 0
        self.initialized = False
        self.events = []
        self.done = False

    def poses(self):
        return np.array([car.state[[0, 1, 4]] for car in self.cars])

    def _check(self, before, after):
        events = {}
        for centres, padding in sweep_samples(before, after, self.bodies, self.max_sweep_motion):
            radii = [b.radius+p for b, p in zip(self.bodies, padding)]
            for i, (c, radius) in enumerate(zip(centres, radii)):
                physical = self.map.intersects(c, radius)
                if physical or self.map.intersects(c, radius+self.wall_clearance):
                    kind = 'wall_contact' if physical else 'wall_clearance'
                    events[(kind, i)] = dict(kind=kind, cars=[i])
            for i, j in combinations(range(len(self.cars)), 2):
                physical = pair_intersects(centres[i], radii[i], centres[j], radii[j])
                if physical or pair_intersects(centres[i], radii[i], centres[j], radii[j], self.pair_clearance):
                    kind = 'vehicle_contact' if physical else 'vehicle_clearance'
                    events[(kind, i, j)] = dict(kind=kind, cars=[i, j])
        return list(events.values())

    def reset(self, poses, speeds=None):
        poses = np.asarray(poses, float)
        speeds = np.zeros(len(self.cars)) if speeds is None else np.asarray(speeds, float)
        if poses.shape != (len(self.cars), 3) or not np.isfinite(poses).all():
            raise ValueError('one finite [x,y,yaw] pose per car required')
        if (speeds.shape != (len(self.cars),) or not np.isfinite(speeds).all() or
                np.any(speeds < self.params.v_min) or np.any(speeds > self.params.v_max)):
            raise ValueError('invalid initial speeds')
        for car, pose, speed in zip(self.cars, poses, speeds):
            car.reset(pose)
            car.state[3] = speed
        self.time = 0.
        self.physics_tick = 0
        self.initialized = True
        self.events = self._check(poses, poses)
        self.done = bool(self.events)
        return self.observe(), self.done

    def step(self, actions, update_actions=None):
        if not self.initialized or self.done:
            raise RuntimeError('reset a nonterminal session before stepping')
        actions = np.asarray(actions, float)
        if actions.shape != (len(self.cars), 2) or not np.isfinite(actions).all():
            raise ValueError('one finite [steering,speed] command per car required')
        for _ in range(self.substeps):
            if update_actions is not None:
                actions = update_actions(self.physics_tick, actions)
            before = self.poses()
            for car, action in zip(self.cars, actions):
                car.update_pose(*action)
            self.time += self.params.timestep
            self.physics_tick += 1
            if not all(np.isfinite(car.state).all() for car in self.cars):
                self.events = [dict(kind='invalid_state', cars=list(range(len(self.cars))))]
            else:
                self.events = self._check(before, self.poses())
            if self.events:
                for event in self.events:
                    event['time_interval_s'] = [self.time-self.params.timestep, self.time]
                self.done = True
                break
        return self.observe(), self.done

    def observe(self):
        observations = []
        poses = self.poses()
        for i, (car, scanner, pose) in enumerate(zip(self.cars, self.scanners, poses)):
            if scanner is None or not np.isfinite(pose).all():
                scan = None
            else:
                scan = scanner.scan(pose, std_dev=0.)
                angles = np.linspace(-scanner.fov/2, scanner.fov/2, scanner.num_beams)
                for j, body in enumerate(self.bodies):
                    if i != j and np.isfinite(poses[j]).all():
                        scan = np.minimum(scan, opponent_scan(pose, angles, body.centres(poses[j]),
                                                              body.radius, scanner.max_range))
            others = [dict(car_id=j, pose=poses[j].copy(), vehicle_state=other.state.copy(),
                           vehicle_speed=float(other.state[3]))
                      for j, other in enumerate(self.cars) if j != i]
            observation = dict(car_id=i, pose=pose.copy(), vehicle_state=car.state.copy(),
                               vehicle_speed=float(car.state[3]), scan=scan, other_vehicles=others,
                               simulation_time=self.time, events=list(self.events))
            if len(others) == 1:
                observation['opponent'] = others[0]
            observations.append(observation)
        return observations
