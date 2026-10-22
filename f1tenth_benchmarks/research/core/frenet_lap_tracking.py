"""Reuse Continuous lap lifecycle with one core Frenet projection source."""
import numpy as np

from f1tenth_benchmarks.research.core.lap_tracking import ContinuousLapTracker, Projection
from f1tenth_benchmarks.research.core.geometry import FrenetTracker


class CoreLapTracker(ContinuousLapTracker):
    def __init__(self, geometry, sampled_xy, config=None):
        super().__init__(sampled_xy, config)
        self.geometry = geometry
        self.frenet_tracker = FrenetTracker(geometry, source_reference='sim_cog',
                                           offset_calibrated=True)
        self.L = geometry.curve.L
        reference = geometry.reference(0.)
        self.finish_origin = np.asarray(reference['xy'])
        self.finish_tangent = np.asarray(reference['tangent'])
        self.finish_normal = np.array([-self.finish_tangent[1], self.finish_tangent[0]])
        self.frenet = None
        self._origin_offset = None

    def _project(self, pose, global_search=False):
        frenet = self.frenet_tracker.update(pose[:2], float(pose[2]), self._input_time,
                                            now=self._input_time,
                                            geometry_id=self.geometry.geometry_id,
                                            signed_speed=self._input_speed)
        self.frenet = frenet
        if not frenet['valid']:
            raise ValueError(frenet['reason'])
        # Segment/fraction remain diagnostic; lifecycle uses true core arclength.
        return Projection(0, 0., frenet['s_wrapped'], abs(frenet['d']),
                          frenet['d'], frenet['e_psi'])

    def update(self, pose, time, speed=None):
        self._input_time, self._input_speed = time, speed
        result = super().update(pose, time, speed)
        if result.valid:
            native = self.frenet_tracker.state['s_unwrapped']
            if self._origin_offset is None:
                self._origin_offset = result.sample.s_unwrapped-native
            adjusted = native+self._origin_offset
            if abs(adjusted-result.sample.s_unwrapped) > 1e-8:
                raise RuntimeError('core and Continuous progress diverged')
            self.frenet = dict(self.frenet, native_s_unwrapped=native,
                               s_unwrapped=adjusted, origin_shift=self._origin_offset)
        return result

    def recover(self, *args, **kwargs):
        raise RuntimeError('invalid sessions terminate; no implicit recovery')
