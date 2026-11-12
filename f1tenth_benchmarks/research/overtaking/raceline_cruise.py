"""Track the stored ESP optimized line between overtaking encounters."""

from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree

from ..run_recorded_candidate_pp import OpenCandidatePP


def local_track_gap(unwrapped_gap, length):
    """Signed distance to the closest periodic copy of the other car."""
    return (unwrapped_gap + length / 2.) % length - length / 2.


class RacelineCruise:
    encounter_m = 7.
    braking_mps2 = 1.5

    def __init__(self, source, controller, max_speed):
        self.source = Path(source)
        data = np.loadtxt(self.source, delimiter=',', ndmin=2)
        if (len(data) < 40 or data.shape[1] != 7 or not np.isfinite(data).all()
                or np.any(data[:, 5] <= 0.)):
            raise ValueError('invalid ESP raceline CSV')
        self.xy = data[:, 1:3]
        self.segment_m = np.linalg.norm(np.roll(self.xy, -1, axis=0) - self.xy, axis=1)
        if not np.all((self.segment_m > 1e-5) & (self.segment_m <= .5)):
            raise ValueError('raceline must be a densely sampled closed loop')
        self.tree = cKDTree(self.xy)
        self.speed = data[:, 5]
        self.controller = controller
        if not np.isfinite(max_speed) or not 0. < max_speed <= controller.vehicle.max_speed:
            raise ValueError('invalid raceline speed ceiling')
        self.max_speed = max_speed

    def local_plan(self, pose, gap):
        _, centre = self.tree.query(pose[:2])
        indices = (centre + np.arange(-5, 46)) % len(self.xy)
        points = self.xy[indices]
        ahead = indices[5:]
        distance = np.r_[0., np.cumsum(self.segment_m[ahead[:-1]])]
        profile_ceiling = np.sqrt(np.min(self.speed[ahead]**2 + 2*self.braking_mps2*distance))
        speed = min(self.max_speed, profile_ceiling)
        if gap > 0.:
            # Reach candidate cruising speed before handing steering back to the
            # local planner. The projected gap closes more slowly than ego travels.
            speed = min(speed, np.sqrt(self.controller.config.speed_mps**2
                                      + 2*self.braking_mps2*max(0., gap-self.encounter_m)))
        ctrl = self.controller
        pp = OpenCandidatePP(points, self.max_speed, ctrl.config.pp_lookahead_m,
                             ctrl.vehicle.wheelbase, ctrl.vehicle.max_steer)
        return pp, float(speed)
