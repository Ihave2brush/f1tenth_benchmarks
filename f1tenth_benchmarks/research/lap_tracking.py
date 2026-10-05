"""Independent closed-polyline Frenet lap tracking; no simulator mutations.

The feasibility bound assumes locally unique projection and bounded Frenet
magnification (roughly 1 / |1-kappa*e_y|), not s-distance == driven distance.
Invalid samples latch recovery_required. Explicit recover() abandons the current
lap's timing, retains completed counts, and starts a partial at a new epoch.
"""
from dataclasses import dataclass
from typing import Optional
import math
import numpy as np


@dataclass(frozen=True)
class TrackerConfig:
    local_segments: int = 30
    max_projection_distance: float = 2.0
    ambiguity_distance: float = 0.02
    ambiguity_s: float = 1.0
    max_dt: float = 0.2
    max_speed: float = 20.0
    frenet_factor: float = 2.0
    increment_margin: float = 0.1
    start_s_tolerance: float = 0.05
    start_lateral_tolerance: float = 0.2
    standing_speed_tolerance: float = 0.05
    rearm_distance: float = 1.0


@dataclass(frozen=True)
class Projection:
    segment: int
    fraction: float
    s_wrapped: float
    distance: float
    e_y: float
    e_psi: Optional[float]


@dataclass(frozen=True)
class Sample:
    index: int
    time: float
    projection: Projection
    s_unwrapped: float
    speed: Optional[float]


@dataclass(frozen=True)
class LapEvent:
    kind: str
    lap_label: str
    completed_full_laps: int
    finish_s: float
    crossing_time_estimate: float
    lap_start_time: float
    duration: float
    alpha: float
    previous_sample: Sample
    current_sample: Sample
    epoch: int
    timing_method: str = "linear_s_interpolation"


@dataclass(frozen=True)
class UpdateResult:
    valid: bool
    reason: Optional[str]
    sample: Optional[Sample]
    event: Optional[LapEvent] = None


class ContinuousLapTracker:
    def __init__(self, centreline_xy, config=None):
        self.config = config or TrackerConfig()
        c = self.config
        positive = (c.max_projection_distance, c.ambiguity_s, c.max_dt,
                    c.max_speed, c.frenet_factor, c.rearm_distance)
        nonnegative = (c.ambiguity_distance, c.increment_margin,
                       c.start_s_tolerance, c.start_lateral_tolerance,
                       c.standing_speed_tolerance)
        if (any(not math.isfinite(v) or v <= 0 for v in positive)
                or any(not math.isfinite(v) or v < 0 for v in nonnegative)
                or not isinstance(c.local_segments, int) or c.local_segments < 1):
            raise ValueError("invalid tracker configuration")
        p = np.array(centreline_xy, dtype=float, copy=True)
        if p.ndim != 2 or p.shape[1] != 2 or len(p) < 3 or not np.isfinite(p).all():
            raise ValueError("centreline must be finite Nx2 points")
        self.had_duplicate_endpoint = bool(np.linalg.norm(p[-1] - p[0]) <= 1e-9)
        if self.had_duplicate_endpoint:
            p = p[:-1]
        self.points = p
        self.edges = np.roll(p, -1, axis=0) - p
        self.lengths = np.linalg.norm(self.edges, axis=1)
        if len(p) < 3 or np.any(self.lengths <= 1e-9):
            raise ValueError("degenerate centreline segment")
        self.starts = np.r_[0., np.cumsum(self.lengths)[:-1]]
        self.L = float(self.lengths.sum())
        if c.rearm_distance >= self.L / 2 or c.start_s_tolerance >= self.L / 2:
            raise ValueError("rearm/start tolerance must be less than half a lap")
        self.finish_origin = p[0].copy()
        self.finish_tangent = self.edges[0] / self.lengths[0]
        self.finish_normal = np.array([-self.finish_tangent[1], self.finish_tangent[0]])
        self.previous = self.current = None
        self.next_finish_s = None
        self.armed = False
        self.lap_start_time = None
        self.completed_full_laps = 0
        self.events = []
        self.recovery_required = False
        self.invalid_reason = None
        self.epoch = 0
        self._index = -1
        self._last_input_time = None

    @classmethod
    def from_csv(cls, filename, config=None):
        """Accept numeric CSV with or without one header; preserve first point."""
        with open(filename) as f:
            first = f.readline()
        try:
            [float(v) for v in first.lstrip('# ').strip().split(',')]
            skip = 0
        except ValueError:
            skip = 1
        return cls(np.loadtxt(filename, delimiter=',', skiprows=skip)[:, :2], config)

    @property
    def s_wrapped(self):
        return None if self.current is None else self.current.projection.s_wrapped

    @property
    def s_unwrapped(self):
        return None if self.current is None else self.current.s_unwrapped

    def _project(self, pose, global_search=False):
        xy = np.asarray(pose, dtype=float)
        if xy.ndim != 1 or len(xy) not in (2, 3) or not np.isfinite(xy).all():
            raise ValueError("invalid_pose")
        n = len(self.points)
        if global_search or self.current is None:
            indices = np.arange(n)
        else:
            k = self.current.projection.segment
            indices = np.unique((k + np.arange(-self.config.local_segments,
                                               self.config.local_segments + 1)) % n)
        edges = self.edges[indices]
        fractions = np.clip(np.sum((xy[:2] - self.points[indices]) * edges, axis=1)
                            / self.lengths[indices] ** 2, 0, 1)
        residual = xy[:2] - (self.points[indices] + fractions[:, None] * edges)
        distances = np.linalg.norm(residual, axis=1)
        ss = (self.starts[indices] + fractions * self.lengths[indices]) % self.L
        best = int(np.argmin(distances))
        if distances[best] > self.config.max_projection_distance:
            raise ValueError("projection_too_far")
        separation = np.abs((ss - ss[best] + self.L / 2) % self.L - self.L / 2)
        if np.any((distances <= distances[best] + self.config.ambiguity_distance)
                  & (separation > self.config.ambiguity_s)):
            raise ValueError("ambiguous_projection")
        i = int(indices[best])
        tangent = self.edges[i] / self.lengths[i]
        e_y = float(tangent[0] * residual[best, 1] - tangent[1] * residual[best, 0])
        heading = math.atan2(tangent[1], tangent[0])
        e_psi = None if len(xy) == 2 else float((xy[2] - heading + math.pi) % (2*math.pi) - math.pi)
        return Projection(i, float(fractions[best]), float(ss[best]),
                          float(distances[best]), e_y, e_psi)

    def _invalid(self, reason):
        self.recovery_required = True
        self.invalid_reason = reason
        return UpdateResult(False, reason, None)

    def _initialize(self, projection, time, speed, recovery=False):
        s = projection.s_wrapped
        at_start = (min(s, self.L-s) <= self.config.start_s_tolerance
                    and projection.distance <= self.config.start_lateral_tolerance)
        if recovery:
            # New coordinate epoch: never infer missing distance or crossings.
            at_start = False
        elif at_start and s > self.L / 2:
            s -= self.L
        self.previous = None
        self.current = Sample(self._index, time, projection, s, speed)
        self.next_finish_s = self.L
        self.lap_start_time = time
        self._full_interval = at_start
        self._label = ("standing_start" if speed is not None and abs(speed) <= self.config.standing_speed_tolerance
                       else "initial_speed_unknown" if speed is None else "moving_start") if at_start else "partial"
        self._arm_origin = s
        self.armed = not at_start
        self.recovery_required = False
        self.invalid_reason = None
        return UpdateResult(True, None, self.current)

    def update(self, pose, time, speed=None):
        """Consume actual [x,y,(yaw)] sample, simulation time and signed speed.

        Pass endpoint speeds that bound motion over each dt. Missing speed uses
        max_speed. Every attempted call receives an index, including rejection.
        """
        self._index += 1
        if not math.isfinite(time) or (self._last_input_time is not None and time <= self._last_input_time):
            return self._invalid("non_increasing_or_invalid_time")
        self._last_input_time = time
        if speed is not None and (not math.isfinite(speed) or abs(speed) > self.config.max_speed):
            return self._invalid("invalid_speed")
        if self.recovery_required:
            return UpdateResult(False, "recovery_required:" + self.invalid_reason, None)
        try:
            projection = self._project(pose)
        except ValueError as exc:
            return self._invalid(str(exc))
        if self.current is None:
            return self._initialize(projection, time, speed)
        prev = self.current
        dt = time-prev.time
        if dt > self.config.max_dt + 1e-12:
            return self._invalid("sampling_gap")
        velocities = (prev.speed, speed)
        v = self.config.max_speed if None in velocities else max(abs(x) for x in velocities)
        bound = self.config.frenet_factor*v*dt + self.config.increment_margin
        if bound >= self.L/2:
            return self._invalid("sampling_alias_risk")
        ds = (projection.s_wrapped-prev.projection.s_wrapped+self.L/2) % self.L-self.L/2
        if abs(ds) > bound:
            return self._invalid("infeasible_increment")
        curr = Sample(self._index, time, projection, prev.s_unwrapped+ds, speed)
        self.previous, self.current = prev, curr
        if not self.armed and curr.s_unwrapped-self._arm_origin >= self.config.rearm_distance:
            self.armed = True
        event = None
        if self.armed and ds > 0 and prev.s_unwrapped < self.next_finish_s <= curr.s_unwrapped:
            alpha = (self.next_finish_s-prev.s_unwrapped)/ds
            if not 0 <= alpha <= 1 or dt <= 0:
                return self._invalid("invalid_crossing_interpolation")
            crossing = prev.time+alpha*dt
            kind = "full" if self._full_interval else "partial"
            if kind == "full":
                self.completed_full_laps += 1
            event = LapEvent(kind, self._label, self.completed_full_laps,
                             self.next_finish_s, crossing, self.lap_start_time,
                             crossing-self.lap_start_time, alpha, prev, curr, self.epoch)
            self.events.append(event)
            self.lap_start_time = crossing
            self._full_interval = True
            self._label = "flying"
            self._arm_origin = self.next_finish_s
            self.next_finish_s += self.L
            self.armed = False
        return UpdateResult(True, None, curr, event)

    def recover(self, pose, time, speed=None):
        """Explicit global relocation; discard interrupted timing, no event.

        Use a fresh later sample. Caller must audit why relocation is appropriate.
        Full count survives, but unwrapped s is only continuous within an epoch.
        """
        if not self.recovery_required:
            raise ValueError("recovery is only allowed after an invalid sample")
        self._index += 1
        if not math.isfinite(time) or (self._last_input_time is not None and time <= self._last_input_time):
            return self._invalid("non_increasing_or_invalid_time")
        self._last_input_time = time
        if speed is not None and (not math.isfinite(speed) or abs(speed) > self.config.max_speed):
            return self._invalid("invalid_speed")
        try:
            projection = self._project(pose, global_search=True)
        except ValueError as exc:
            return self._invalid(str(exc))
        self.epoch += 1
        return self._initialize(projection, time, speed, recovery=True)
