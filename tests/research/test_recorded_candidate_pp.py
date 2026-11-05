import numpy as np
import unittest

from f1tenth_benchmarks.research.run_recorded_candidate_pp import OpenCandidatePP


class OpenCandidatePPTests(unittest.TestCase):
    def test_open_path_has_no_artificial_return_segment(self):
        controller = OpenCandidatePP([[0, 0], [1, 0], [2, 0]], 1.2, .6, .33, .4)
        self.assertEqual(len(controller.path), 3)
        action, detail = controller.plan(np.array([0., 0., 0.]))
        np.testing.assert_allclose(action, [0., 1.2], atol=1e-5)
        self.assertGreater(detail['lookahead_xy'][0], .59)
        with self.assertRaisesRegex(ValueError, 'open_path_lookahead_unavailable'):
            controller.plan(np.array([2., 0., 0.]))


    def test_actual_heading_drives_clamped_command(self):
        controller = OpenCandidatePP([[0, 0], [1, 0], [2, 0]], 1.2, .6, .33, .1)
        action, detail = controller.plan(np.array([0., 0., .5]))
        self.assertEqual(action[0], -.1)
        self.assertTrue(detail['steering_clipped'])
        self.assertEqual(action[1], 1.2)
