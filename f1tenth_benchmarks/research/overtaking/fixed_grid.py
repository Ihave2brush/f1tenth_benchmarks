"""Fixed exact-s validation sites, with request endpoints kept unsnapped.

This changes the phase of the dense sampling policy, not road interpolation.
All reference/ray queries still use their actual coordinates. The source
RoadTable owns bounded data caches; this window retains only active site IDs.
"""
import math

import numpy as np

POLICY = 'fixed_wrapped_s_grid_exact_endpoints_v1'


class FixedRoadWindow:
    def __init__(self, spacing, length, closed):
        if not math.isfinite(spacing) or spacing <= 0:
            raise ValueError('invalid fixed grid spacing')
        self.spacing, self.length, self.closed = float(spacing), float(length), bool(closed)
        self.active = set()

    def sample(self, start, distance):
        end = start+distance
        if not all(math.isfinite(v) for v in (start, distance)) or distance <= 0:
            raise ValueError('invalid fixed grid window')
        if start < 0 or (not self.closed and end > self.length):
            raise ValueError('fixed grid window outside reference')
        if self.closed and distance >= self.length:
            raise ValueError('fixed grid window exceeds lap')
        sites = []
        first_lap = math.floor(start/self.length) if self.closed else 0
        last_lap = math.floor(end/self.length) if self.closed else 0
        for lap in range(first_lap, last_lap+1):
            origin = lap*self.length
            lo, hi = max(0., start-origin), min(self.length, end-origin)
            first = max(0, math.floor(lo/self.spacing))
            last = min(math.ceil(self.length/self.spacing), math.ceil(hi/self.spacing))
            for index in range(first, last+1):
                canonical = index*self.spacing
                progress = origin+canonical
                if canonical < self.length and start < progress < end:
                    sites.append((progress, canonical, index))
        sites.sort()
        active = {index for _, _, index in sites}
        reused = len(active & self.active)
        previous_count = len(self.active)
        self.active = active
        progress = np.array([start]+[p for p, _, _ in sites]+[end])
        source = np.array([start % self.length if self.closed else start]+
                          [s for _, s, _ in sites]+
                          [end % self.length if self.closed else end])
        summary = dict(policy=POLICY, grid_spacing_m=self.spacing, grid_anchor_s_m=0.,
            active_grid_sites=len(active), previous_grid_sites=previous_count,
            reused_grid_sites=reused, newly_entered_grid_sites=len(active)-reused,
            overlap_fraction=reused/len(active) if active else 0.,
            endpoint_sites=2, actual_site_count=len(progress),
            max_gap_m=float(np.max(np.diff(progress))),
            s_interpolation=False, endpoints_snapped=False,
            continuous_s_verified=False, storage='bounded_source_road_table_LRU')
        return progress, source, summary


def sample_window(table, start, distance, spacing):
    window = getattr(table, '_fixed_grid_window', None)
    curve = table.geometry.curve
    if (window is None or window.spacing != spacing or window.length != curve.L or
            window.closed != curve.closed):
        window = FixedRoadWindow(spacing, curve.L, curve.closed)
        table._fixed_grid_window = window
    return window.sample(start, distance)
