"""Three discs cover three equal rectangular strips of the body.

Map collision uses occupied pixel *squares*, not pixel-centre distances.
Pose is the geometric body centre, matching the existing simulator convention.
"""
from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ThreeDiscBody:
    length: float = .51
    width: float = .27

    def __post_init__(self):
        if not np.isfinite([self.length, self.width]).all() or min(self.length, self.width) <= 0:
            raise ValueError('body dimensions must be positive and finite')

    @property
    def radius(self):
        return math.hypot(self.length / 6, self.width / 2)

    def centres(self, pose):
        pose = np.asarray(pose, float)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            raise ValueError('pose must be finite [x,y,yaw]')
        offsets = np.array([-self.length / 3, 0., self.length / 3])
        return pose[:2] + offsets[:, None] * np.array([math.cos(pose[2]), math.sin(pose[2])])


class OccupancyMap:
    def __init__(self, occupied, resolution, origin):
        self.occupied = np.asarray(occupied, bool)
        self.origin = np.asarray(origin, float)
        if (self.occupied.ndim != 2 or not self.occupied.size or
                not math.isfinite(resolution) or resolution <= 0 or
                self.origin.shape != (3,) or not np.isfinite(self.origin).all()):
            raise ValueError('invalid occupancy map')
        self.resolution = resolution
        c, s = np.cos(self.origin[2]), np.sin(self.origin[2])
        self.rotation = np.array([[c, -s], [s, c]])

    @classmethod
    def from_raster(cls, raster):
        return cls(np.flipud(raster.occupancy != 0), raster.resolution, raster.origin)

    @classmethod
    def from_scan(cls, scan):
        # Reuse the simulator's <=128 occupancy convention and flipped rows.
        return cls(scan.map_img == 0, scan.map_resolution, scan.origin)

    def intersects(self, centres, radius):
        if not math.isfinite(radius) or radius < 0:
            raise ValueError('invalid radius')
        points = (np.asarray(centres) - self.origin[:2]) @ self.rotation
        if not np.isfinite(points).all():
            raise ValueError('invalid centres')
        h, w = self.occupied.shape
        res = self.resolution
        for x, y in points:
            # Outside map is occupied; touching counts as overlap.
            if x-radius <= 0 or y-radius <= 0 or x+radius >= w*res or y+radius >= h*res:
                return True
            c0, c1 = max(0, int(math.floor((x-radius)/res))-1), min(w-1, int(math.floor((x+radius)/res)))
            r0, r1 = max(0, int(math.floor((y-radius)/res))-1), min(h-1, int(math.floor((y+radius)/res)))
            rows, cols = np.nonzero(self.occupied[r0:r1+1, c0:c1+1])
            if len(rows):
                lo = np.column_stack([cols+c0, rows+r0])*res
                delta = np.maximum(np.maximum(lo-[x, y], [x, y]-(lo+res)), 0.)
                if np.any(np.sum(delta*delta, axis=1) <= radius*radius + 1e-14):
                    return True
        return False


def pair_intersects(a, ra, b, rb, clearance=0.):
    distance2 = np.sum((a[:, None, :] - b[None, :, :])**2, axis=2)
    return bool(np.any(distance2 <= (ra+rb+clearance)**2 + 1e-14))


def sweep_samples(before, after, bodies, max_motion=.01):
    """Enclose linear-position/shortest-yaw interpolation in midpoint discs.

    Each sample radius gains half an interval's upper bound on centre motion.
    This is conservative for the declared interpolant, not a continuous ODE
    solution or a precise contact-time estimate.
    """
    before, after = np.asarray(before, float), np.asarray(after, float)
    if (before.shape != (len(bodies), 3) or after.shape != before.shape or
            not np.isfinite([before, after]).all() or not math.isfinite(max_motion) or max_motion <= 0):
        raise ValueError('invalid sweep')
    delta = after-before
    delta[:, 2] = (delta[:, 2]+np.pi) % (2*np.pi)-np.pi
    motion = np.linalg.norm(delta[:, :2], axis=1) + np.abs(delta[:, 2])*np.array([b.length/3 for b in bodies])
    count = max(1, int(np.ceil(np.max(motion)/max_motion)))
    for k in range(count):
        poses = before + ((k+.5)/count)*delta
        yield [b.centres(p) for b, p in zip(bodies, poses)], motion/(2*count)


def opponent_scan(pose, angles, centres, radius, max_range):
    """Exact ray hits on the other car's uninflated disc union."""
    rays = np.column_stack([np.cos(angles+pose[2]), np.sin(angles+pose[2])])
    rel = centres-pose[:2]
    proj = rays @ rel.T
    norm2 = np.sum(rel*rel, axis=1)
    disc = proj*proj-(norm2-radius*radius)
    root = np.sqrt(np.maximum(disc, 0.))
    near, far = proj-root, proj+root
    hit = np.where((disc >= 0) & (far >= 0), np.maximum(near, 0.), np.inf)
    return np.minimum(np.min(hit, axis=1), max_range)
