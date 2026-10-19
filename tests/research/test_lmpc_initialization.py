"""B1 contracts: raster footprint, shared progress/finish, low-speed PP."""
from types import SimpleNamespace
import unittest

import numpy as np

from f1tenth_benchmarks.research.core.geometry.map_raster import MapRaster
from f1tenth_benchmarks.research.lmpc.validation import FootprintChecker, CandidateValidator
from f1tenth_benchmarks.research.lmpc.initialization import CentrelinePurePursuit, ControlledExcitation
from f1tenth_benchmarks.research.lmpc.lap_adapter import CoreLapTracker


class AnalyticCircle:
    """Independent exact circle oracle, only for lifecycle/validator tests."""
    geometry_id = 'analytic_circle'

    def __init__(self):
        self.curve = SimpleNamespace(L=20., closed=True)
        self.raster = SimpleNamespace(resolution=.05)
        self.radius = self.curve.L/(2*np.pi)

    def reference(self, s):
        angle = s/self.radius
        return dict(xy=[self.radius*np.cos(angle), self.radius*np.sin(angle)],
                    tangent=[-np.sin(angle), np.cos(angle)], psi=angle+np.pi/2)

    def to_frenet(self, xy, yaw):
        angle = np.arctan2(xy[1], xy[0]) % (2*np.pi)
        s = angle*self.radius
        heading = (yaw-angle-np.pi/2+np.pi)%(2*np.pi)-np.pi
        d = self.radius-np.linalg.norm(xy)
        return dict(valid=True, reason=None, s_wrapped=s, d=d, e_psi=heading,
                    jacobian=1-d/self.radius, projection_xy=self.reference(s)['xy'])

    def projection_candidates(self, xy, yaw):
        result = self.to_frenet(xy, yaw)
        return dict(valid=True, candidates=[dict(result, s=result['s_wrapped'], distance=abs(result['d']))])

    def to_cartesian(self, s, d, e_psi):
        if abs(d) > .5:
            return dict(valid=False, reason='outside_track')
        angle = s/self.radius
        return dict(valid=True, xy=[(self.radius-d)*np.cos(angle), (self.radius-d)*np.sin(angle)],
                    yaw=angle+np.pi/2+e_psi)


class InitialisationContracts(unittest.TestCase):
    def test_excitation_is_reproducible_bounded_and_held(self):
        settings=dict(seed=20261015,hold_steps=5,steering_amplitude_rad=.02,
                      acceleration_amplitude_mps2=.6)
        a,b=ControlledExcitation(settings),ControlledExcitation(settings)
        commands=np.array([a.apply([0.,0.],.4,9.51) for _ in range(500)])
        np.testing.assert_array_equal(commands,[b.apply([0.,0.],.4,9.51) for _ in range(500)])
        self.assertTrue(np.all(abs(commands)<=[.02,.6]))
        np.testing.assert_array_equal(commands.reshape(-1,5,2),np.repeat(commands[::5,None,:],5,axis=1))
        self.assertGreater(np.std(commands[:,0]),.005)
        self.assertGreater(np.std(commands[:,1]),.15)
        self.assertLess(abs(np.corrcoef(commands.T)[0,1]),.3)

    def test_excitation_clips_combined_commands(self):
        source=ControlledExcitation(dict(seed=1,hold_steps=5,
            steering_amplitude_rad=.02,acceleration_amplitude_mps2=.6))
        result=source.apply([.4,9.51],.4,9.51)
        np.testing.assert_allclose(result,[.4,9.51])

    def raster(self, yaw=0.):
        return MapRaster(np.full((100, 100), 255, np.uint8), dict(
            resolution=.05, origin=[0, 0, yaw], negate=0, free_thresh=.2, occupied_thresh=.65))

    def test_interior_obstacle_detected_when_four_corners_are_free(self):
        raster = self.raster()
        pose = [2., 2., 0.]
        pixel = raster.map_to_pixel(pose[:2])
        raster.occupancy[pixel.cell[1], pixel.cell[0]] = 1
        checker = FootprintChecker(raster, .58, .31)
        for x in (-.29, .29):
            for y in (-.155, .155):
                self.assertEqual(raster.classify([2+x, 2+y]), 'free')
        self.assertFalse(checker.check(pose)['valid'])

    def test_rotated_map_unknown_and_outside_are_rejected(self):
        raster = self.raster(yaw=.4)
        centre = raster.local_to_map([2., 2.])
        checker = FootprintChecker(raster, .58, .31, .05)
        pose = [*centre, 1.]
        self.assertTrue(checker.check(pose)['valid'])
        pixel = raster.map_to_pixel(centre)
        raster.occupancy[pixel.cell[1], pixel.cell[0]] = -1
        self.assertFalse(checker.check(pose)['valid'])
        self.assertFalse(checker.check([0, 0, 0])['valid'])

    def test_nonoverlapping_cell_in_rotated_bbox_is_not_a_collision(self):
        raster = self.raster()
        checker = FootprintChecker(raster, .58, .1)
        pixel = raster.map_to_pixel([2.2, 1.8])
        raster.occupancy[pixel.cell[1], pixel.cell[0]] = 1
        self.assertTrue(checker.check([2., 2., np.pi/4])['valid'])

    def test_core_progress_controls_two_full_events_and_preserves_direction(self):
        geometry = AnalyticCircle()
        points = np.array([geometry.reference(s)['xy'] for s in np.linspace(0, 20, 200, endpoint=False)])
        tracker = CoreLapTracker(geometry, points)
        result = tracker.update([*geometry.reference(0.)['xy'], np.pi/2], .1, 0.)
        self.assertTrue(result.valid)
        np.testing.assert_allclose(tracker.finish_tangent, [0., 1.], atol=1e-15)
        for i in range(1, 202):
            s = i*.2
            reference = geometry.reference(s)
            result = tracker.update([*reference['xy'], reference['psi']], .1+i*.1, 2.)
            self.assertTrue(result.valid, result.reason)
            self.assertAlmostEqual(result.sample.s_unwrapped, s, places=9)
            self.assertAlmostEqual(tracker.frenet['s_unwrapped'], s, places=9)
        self.assertEqual(tracker.completed_full_laps, 2)
        self.assertEqual(len(tracker.events), 2)
        self.assertTrue(all(event.kind=='full' for event in tracker.events))
        np.testing.assert_allclose([event.duration for event in tracker.events], [10., 10.], atol=1e-10)

    def test_low_speed_and_seam_do_not_change_initialisation_speed(self):
        geometry = AnalyticCircle()
        points = np.array([geometry.reference(s)['xy'] for s in np.linspace(0, 20, 200, endpoint=False)])
        controller = CentrelinePurePursuit(points, 1.2, .6, .33, .4)
        for s in (0., 19.9, 20.1):
            reference = geometry.reference(s)
            action = controller.plan(dict(pose=[*reference['xy'], reference['psi']], vehicle_speed=0.))
            self.assertEqual(action[1], 1.2)
            self.assertLessEqual(abs(action[0]), .4)
            self.assertGreater(action[0], 0.)

    def test_candidate_validation_rejects_invalid_points_and_inputs(self):
        geometry = AnalyticCircle()
        footprint = SimpleNamespace(check=lambda pose: dict(valid=True))
        validator = CandidateValidator(geometry, footprint, 8., .4, 9.)
        states = np.array([[1.2, 0, 0, 0, 0, 0], [1.2, 0, 0, 0, .1, 0]])
        result = validator.check(states, [[0., .1]])
        self.assertTrue(result['valid'])
        self.assertFalse(result['planning_allowed'])
        self.assertFalse(result['continuous_verified'])
        self.assertFalse(validator.check(states, [[.5, 0]])['valid'])
        states[-1, 5] = .6
        self.assertFalse(validator.check(states)['valid'])


if __name__ == '__main__':
    unittest.main()
