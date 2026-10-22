"""Shared fixed-speed centreline PP, reused from the verified B1 baseline.

Only the solver-independent PP class is brought from research/lmpc; no learning
or optimization dependency. The control calculation is unchanged.
"""
import numpy as np

from f1tenth_benchmarks.classic_racing.GlobalPurePursuit import (
    get_actuation, nearest_point_on_trajectory_py2,
    first_point_on_trajectory_intersecting_circle,
)


class CentrelinePurePursuit:
    def __init__(self, sampled_xy, speed, lookahead, wheelbase, max_steer):
        points = np.asarray(sampled_xy, float)
        if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or not np.isfinite(points).all():
            raise ValueError('invalid centreline points')
        if not np.isfinite([speed, lookahead, wheelbase, max_steer]).all() or min(speed, lookahead, wheelbase, max_steer) <= 0:
            raise ValueError('invalid Pure Pursuit settings')
        if np.linalg.norm(points[-1]-points[0]) < 1e-9:
            points = points[:-1]
        self.path = np.vstack([points, points[0]])
        self.diffs = np.diff(self.path, axis=0)
        self.l2s = np.sum(self.diffs**2, axis=1)
        if np.any(self.l2s < 1e-12):
            raise ValueError('degenerate centreline segment')
        self.speed, self.lookahead = speed, lookahead
        self.wheelbase, self.max_steer = wheelbase, max_steer
        self.diagnostic = None

    def plan(self, observation):
        pose = np.asarray(observation['pose'], float)
        _, distance, fraction, index = nearest_point_on_trajectory_py2(
            pose[:2], self.path, self.l2s, self.diffs)
        point, _, _ = first_point_on_trajectory_intersecting_circle(
            pose[:2], self.lookahead, self.path, index+fraction, wrap=True)
        if point is None:
            raise ValueError('no forward Pure Pursuit lookahead intersection')
        steering = float(get_actuation(pose[2], point, pose[:2],
                                       np.linalg.norm(point-pose[:2]), self.wheelbase))
        command = float(np.clip(steering, -self.max_steer, self.max_steer))
        self.diagnostic = dict(nearest_distance=float(distance),
                               unconstrained_steering=steering,
                               steering_clipped=abs(steering)>self.max_steer,
                               lookahead_xy=point.tolist())
        # No special low-speed [0,4] branch: startup uses the same 1.2 m/s target.
        return np.array([command, self.speed])
