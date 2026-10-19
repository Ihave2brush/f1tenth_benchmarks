"""Exact raster-cell footprint overlap and pointwise candidate diagnostics.

This does not approve the original geometry globally or certify intersample
trajectory validity. B1 is a restricted Cartesian reference-tracking experiment.
"""
import numpy as np

from f1tenth_benchmarks.research.core.geometry.map_raster import FREE


class FootprintChecker:
    def __init__(self, raster, length, width, margin=0.):
        if not np.isfinite([length, width, margin]).all() or min(length, width) <= 0 or margin < 0:
            raise ValueError('invalid footprint dimensions')
        self.raster = raster
        self.half = np.array([length/2 + margin, width/2 + margin])
        self.margin = margin

    def check(self, pose):
        pose = np.asarray(pose, float)
        if pose.shape != (3,) or not np.isfinite(pose).all():
            return dict(valid=False, reason='nonfinite_pose')
        raster = self.raster
        yaw = pose[2] - raster.origin[2]
        axes = np.array([[np.cos(yaw), np.sin(yaw)], [-np.sin(yaw), np.cos(yaw)]])
        centre = raster.map_to_local(pose[:2])
        corners = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]]) * self.half
        polygon = corners @ axes + centre
        extent = np.array([raster.width, raster.height]) * raster.resolution
        if np.any(polygon < 0) or np.any(polygon >= extent):
            return dict(valid=False, reason='footprint_outside_map')
        low = np.floor(polygon.min(axis=0)/raster.resolution).astype(int)
        high = np.floor(polygon.max(axis=0)/raster.resolution).astype(int)
        cc, rr = np.meshgrid(np.arange(low[0], high[0]+1), np.arange(low[1], high[1]+1))
        vv = raster.height-1-rr
        mask = raster.occupancy[vv, cc] != FREE
        if not mask.any():
            return dict(valid=True, reason=None, overlapping_nonfree_cells=0)
        cc, rr, vv = cc[mask], rr[mask], vv[mask]
        cells = (np.stack([cc, rr], axis=1)+.5)*raster.resolution
        delta = cells-centre
        # SAT axes: two body axes and two raster axes. Include touching cells.
        overlap = np.ones(len(cells), dtype=bool)
        cell_half = raster.resolution/2
        for axis in np.vstack([axes, np.eye(2)]):
            radius = np.sum(self.half * np.abs(axes @ axis)) + cell_half*np.abs(axis).sum()
            overlap &= np.abs(delta @ axis) <= radius + 1e-12
        indices = np.flatnonzero(overlap)
        if len(indices):
            i = indices[0]
            return dict(valid=False, reason='footprint_nonfree_cell',
                        cell=[int(cc[i]), int(vv[i])],
                        occupancy=int(raster.occupancy[vv[i], cc[i]]))
        return dict(valid=True, reason=None, overlapping_nonfree_cells=0)


class CandidateValidator:
    """A validator only; a passing result is not a planning authorization."""
    def __init__(self, geometry, footprint, max_speed, max_steer, max_acceleration):
        self.geometry, self.footprint = geometry, footprint
        limits = np.asarray([max_speed, max_steer, max_acceleration], float)
        if not np.isfinite(limits).all() or np.any(limits <= 0):
            raise ValueError('invalid candidate limits')
        self.max_speed, self.max_steer, self.max_acceleration = limits

    def check(self, states, controls=None):
        states = np.asarray(states, float)
        base = dict(planning_allowed=False, continuous_verified=False,
                    geometry_id=self.geometry.geometry_id,
                    verification='exact_queries_and_raster_overlap_at_supplied_points')
        if states.ndim != 2 or states.shape[1] != 6 or not len(states) or not np.isfinite(states).all():
            return dict(base, valid=False, reason='invalid_states')
        if np.any(np.linalg.norm(states[:, :2], axis=1) > self.max_speed) or np.any(states[:, 0] < 0):
            return dict(base, valid=False, reason='speed_limit')
        if controls is not None:
            controls = np.asarray(controls, float)
            if (controls.shape != (len(states)-1, 2) or not np.isfinite(controls).all()
                    or np.any(np.abs(controls[:, 0]) > self.max_steer)
                    or np.any(np.abs(controls[:, 1]) > self.max_acceleration)):
                return dict(base, valid=False, reason='control_limit_or_shape')
        for i, state in enumerate(states):
            point = self.geometry.to_cartesian(state[4], state[5], state[3])
            if not point['valid']:
                return dict(base, valid=False, reason=point['reason'], index=i)
            body = self.footprint.check([*point['xy'], point['yaw']])
            if not body['valid']:
                return dict(base, valid=False, reason=body['reason'], index=i)
        return dict(base, valid=True, reason=None, checked_points=len(states))
