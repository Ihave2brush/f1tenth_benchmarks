import unittest
from dataclasses import replace
import numpy as np
from f1tenth_benchmarks.research.lap_tracking import ContinuousLapTracker, TrackerConfig


class LapTrackingTests(unittest.TestCase):
    square = np.array([[0., 0.], [25., 0.], [25., 25.], [0., 25.]])
    config = TrackerConfig(local_segments=4, max_dt=1., max_speed=20.,
                           frenet_factor=2., increment_margin=.1)

    def tracker(self, **kwargs):
        return ContinuousLapTracker(self.square, replace(self.config, **kwargs))

    def pose(self, s, offset=0.):
        s %= 100
        if s < 25:
            return [s, offset, 0]
        if s < 50:
            return [25-offset, s-25, np.pi/2]
        if s < 75:
            return [75-s, 25-offset, np.pi]
        return [offset, 100-s, -np.pi/2]

    def feed(self, tracker, ss, times=None, speed=10):
        if times is None:
            times = np.arange(len(ss), dtype=float)
        results = [tracker.update(self.pose(s), float(t), speed) for s, t in zip(ss, times)]
        self.assertTrue(all(r.valid for r in results), [r.reason for r in results])
        return results

    def test_forward_seam_and_interpolation(self):
        tr = self.tracker()
        results = self.feed(tr, [99.8, 100.2], [0, .1])
        self.assertAlmostEqual(tr.s_wrapped, .2)
        self.assertAlmostEqual(tr.s_unwrapped, 100.2)
        e = results[-1].event
        self.assertEqual(e.kind, 'partial')
        self.assertAlmostEqual(e.alpha, .5)
        self.assertAlmostEqual(e.crossing_time_estimate, .05)
        self.assertEqual((e.previous_sample.index, e.current_sample.index), (0, 1))
        self.assertEqual((e.previous_sample.time, e.current_sample.time), (0, .1))

    def test_start_two_full_laps(self):
        tr = self.tracker()
        ss = list(range(0, 211, 10))
        results = self.feed(tr, ss, speed=10)
        self.assertIsNone(results[0].event)
        self.assertEqual(tr.completed_full_laps, 2)
        self.assertEqual([e.duration for e in tr.events], [10., 10.])
        self.assertEqual([e.lap_label for e in tr.events], ['moving_start', 'flying'])
        self.assertEqual([e.finish_s for e in tr.events], [100., 200.])

    def test_start_labels_and_tolerance(self):
        for speed, label in [(0., 'standing_start'), (None, 'initial_speed_unknown'), (2., 'moving_start')]:
            tr = self.tracker()
            self.assertTrue(tr.update(self.pose(0), 0., speed).valid)
            self.assertEqual(tr._label, label)
            self.assertFalse(tr.armed)
            self.assertEqual(tr.completed_full_laps, 0)
        tr = self.tracker()
        self.feed(tr, [99.98, 100.02], [0, .1])
        self.assertFalse(tr.events)
        self.assertAlmostEqual(tr.s_unwrapped, .02)
        self.assertEqual(tr.next_finish_s, 100.)
        tr = self.tracker()
        tr.update(self.pose(.01, .3), 0, 10)
        self.assertFalse(tr._full_interval)

    def test_jitter_reverse_and_previously_counted_threshold(self):
        tr = self.tracker()
        self.feed(tr, [99.8, 100.2, 99.9, 100.3, 99.7, 100.4], np.arange(6)*.1)
        self.assertEqual(len(tr.events), 1)
        self.assertEqual(tr.next_finish_s, 200.)
        self.assertLess(tr.events[0].current_sample.s_unwrapped, 101.)
        tr = self.tracker()
        self.feed(tr, [.2, -.2, .3], [0, .1, .2], speed=-10)
        self.assertFalse(tr.events)
        self.assertAlmostEqual(tr.previous.s_unwrapped, -.2)
        self.assertAlmostEqual(tr.s_unwrapped, .3)

    def test_partial_then_full(self):
        tr = self.tracker()
        self.feed(tr, range(30, 211, 10))
        self.assertEqual([e.kind for e in tr.events], ['partial', 'full'])
        self.assertEqual([e.duration for e in tr.events], [7., 10.])
        self.assertEqual(tr.completed_full_laps, 1)

    def test_lateral_offset_crossing(self):
        # Straight seam makes off-centre projection independent of corner geometry.
        points = [[0, 0], [10, 0], [10, 10], [-10, 10], [-10, 0]]
        tr = ContinuousLapTracker(points, self.config)
        self.assertTrue(tr.update([-.2, .5, np.pi], 0., 10).valid)
        r = tr.update([.2, .5, np.pi], .1, 10)
        self.assertTrue(r.valid)
        self.assertIsNotNone(r.event)  # heading does not determine motion
        self.assertAlmostEqual(r.event.crossing_time_estimate, .05)
        self.assertAlmostEqual(r.sample.projection.e_y, .5)

    def test_closure_and_duplicate_endpoint(self):
        for points in (self.square, np.vstack([self.square, self.square[0]])):
            tr = ContinuousLapTracker(points, self.config)
            self.assertEqual(tr.L, 100.)
            r = tr.update([0., 5.], 0.)
            self.assertEqual(r.sample.projection.segment, 3)
            self.assertAlmostEqual(tr.s_wrapped, 95.)
            self.assertAlmostEqual(r.sample.projection.fraction, .8)
        self.assertTrue(tr.had_duplicate_endpoint)

    def test_anomaly_freeze_and_explicit_recovery(self):
        tr = self.tracker()
        self.feed(tr, [90.], [0.])
        r = tr.update(self.pose(110), .01, 10)
        self.assertFalse(r.valid)
        self.assertEqual(r.reason, 'infeasible_increment')
        self.assertEqual(tr.s_unwrapped, 90.)
        self.assertFalse(tr.events)
        self.assertFalse(tr.update(self.pose(111), .02, 10).valid)
        r = tr.recover(self.pose(110), .03, 10)
        self.assertTrue(r.valid)
        self.assertIsNone(r.event)
        self.assertEqual(tr.epoch, 1)
        self.assertEqual(tr.s_unwrapped, 10.)
        self.feed(tr, range(20, 211, 10), np.arange(1, 21)+.03)
        self.assertEqual([e.kind for e in tr.events], ['partial', 'full'])
        self.assertEqual(tr.completed_full_laps, 1)

    def test_invalid_times_gaps_speed_and_alias(self):
        for time, reason in [(0, 'non_increasing_or_invalid_time'), (-1, 'non_increasing_or_invalid_time'),
                             (float('nan'), 'non_increasing_or_invalid_time'), (2, 'sampling_gap')]:
            tr = self.tracker()
            tr.update(self.pose(30), 0., 10)
            r = tr.update(self.pose(31), time, 10)
            self.assertEqual(r.reason, reason)
            self.assertEqual(tr.s_unwrapped, 30.)
        tr = self.tracker(frenet_factor=3)
        tr.update(self.pose(30), 0., 20)
        self.assertEqual(tr.update(self.pose(31), 1., 20).reason, 'sampling_alias_risk')
        tr = self.tracker()
        self.assertEqual(tr.update(self.pose(30), 0., float('inf')).reason, 'invalid_speed')

    def test_projection_ambiguity_and_local_continuity(self):
        crossing = [[-10, -10], [10, 10], [-10, 10], [10, -10]]
        tr = ContinuousLapTracker(crossing, self.config)
        self.assertEqual(tr.update([0, 0], 0., 10).reason, 'ambiguous_projection')
        # A nearby parallel branch outside local indices cannot steal progress.
        points = [[0, 0], [5, 0], [10, 0], [10, 1], [5, 1], [0, 1]]
        tr = ContinuousLapTracker(points, replace(self.config, local_segments=1))
        tr.update([2, .1], 0., 10)
        r = tr.update([3, .6], .1, 10)
        self.assertTrue(r.valid)
        self.assertAlmostEqual(tr.s_unwrapped, 3.)
        tr = self.tracker()
        tr.update(self.pose(10), 0., 10)
        self.assertEqual(tr.update([100, 100], .1, 10).reason, 'projection_too_far')

    def test_real_esp_two_laps_with_default_local_search(self):
        tr = ContinuousLapTracker.from_csv('maps/esp_centerline.csv')
        distances = np.arange(0., 2*tr.L + .1, .1)
        for index, distance in enumerate(distances):
            wrapped = distance % tr.L
            segment = np.searchsorted(tr.starts, wrapped, side='right') - 1
            fraction = (wrapped-tr.starts[segment])/tr.lengths[segment]
            pose = tr.points[segment] + fraction*tr.edges[segment]
            r = tr.update(pose, index*.04, 2.5)
            self.assertTrue(r.valid, r.reason)
            self.assertAlmostEqual(tr.s_unwrapped, distance, places=8)
        self.assertEqual(tr.completed_full_laps, 2)
        for event in tr.events:
            self.assertAlmostEqual(event.duration, tr.L/2.5, places=8)

    def test_csv_preserves_numeric_first_row(self):
        tr = ContinuousLapTracker.from_csv('maps/esp_centerline.csv')
        data = np.loadtxt('maps/esp_centerline.csv', delimiter=',')[:, :2]
        np.testing.assert_array_equal(tr.points[0], data[0])
        self.assertAlmostEqual(tr.L, np.linalg.norm(np.roll(data, -1, axis=0)-data, axis=1).sum())


if __name__ == '__main__':
    unittest.main()
