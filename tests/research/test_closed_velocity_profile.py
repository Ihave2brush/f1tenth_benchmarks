"""Closed-profile regressions and optional historical workspace acceptance."""
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pytest

from f1tenth_benchmarks.research.closed_velocity_profile import (
    build_closed_segment_lengths, compute_closed_acceleration_and_time,
    exclusive_json, solve_closed_velocity_profile, generate, validate, parameters,
    check_start_index_invariance, protected_hashes, digest, corrected_closed_solver)
import json


class ClosedProfileTests(unittest.TestCase):
    def test_square_closure_and_constant_speed_time(self):
        geometry = np.array([[0, 0, 0, 0, 0], [1, 1, 0, 0, 0],
                             [2, 1, 1, 0, 0], [3, 0, 1, 0, 0]], dtype=float)
        ds = build_closed_segment_lengths(geometry)
        np.testing.assert_array_equal(ds, [1, 1, 1, 1])
        ax, dt = compute_closed_acceleration_and_time(np.full(4, 2.), ds)
        np.testing.assert_array_equal(ax, np.zeros(4))
        self.assertEqual(dt.sum(), 2.)
        self.assertEqual(dt[:-1].sum(), 1.5)

    def test_closure_acceleration_is_not_zero_padding(self):
        v = np.array([2., 3., 4.])
        ax, dt = compute_closed_acceleration_and_time(v, np.ones(3))
        self.assertEqual(ax[-1], -6.)
        self.assertAlmostEqual(dt[-1], 1 / 3)

    def test_invalid_segments_and_speed_rejected(self):
        with self.assertRaises(ValueError):
            build_closed_segment_lengths(np.zeros((3, 5)))
        with self.assertRaises(ValueError):
            compute_closed_acceleration_and_time(np.zeros(3), np.ones(3))

    def test_solver_preserves_geometry_and_periodic_dimensions(self):
        theta = np.linspace(0, 2 * np.pi, 64, endpoint=False)
        g = np.column_stack((np.arange(64), 10 * np.cos(theta), 10 * np.sin(theta), theta, np.full(64, .1)))
        profile, ds, dt = solve_closed_velocity_profile(g,
            dict(mu=.6, max_longitudinal_acc=8.5, max_lateral_acc=8.5),
            dict(max_speed=8, vehicle_mass=3.71))
        np.testing.assert_array_equal(profile[:, :5], g)
        self.assertEqual(profile.shape, (64, 7))
        self.assertEqual(len(ds), 64)
        self.assertEqual(len(dt), 64)
        self.assertTrue(np.isfinite(profile).all())
        self.assertTrue(np.isfinite(dt).all())
        self.assertTrue(np.all((profile[:, 5] > 0) & (profile[:, 5] <= 8)))
        self.assertTrue(np.all(dt > 0))

    def test_output_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / 'report.json'
            exclusive_json(p, {'original': True})
            with self.assertRaises(FileExistsError):
                exclusive_json(p, {'original': False})
            self.assertIn('true', p.read_text())

    def test_saved_geometry_header_and_closure(self):
        base = Path('Data/racelines/mu60/esp_raceline.csv')
        target = Path('Data/racelines/mu60_closed/esp_raceline.csv')
        a, b = np.loadtxt(base, delimiter=','), np.loadtxt(target, delimiter=',')
        np.testing.assert_array_equal(a[:, :5], b[:, :5])
        self.assertTrue(target.read_text().startswith('# s,x,y'))
        p, vehicle = parameters()
        summary = validate(b, p, vehicle)
        self.assertEqual(summary['segment_count'], len(b))
        self.assertAlmostEqual(summary['planned_time_s'],
                               summary['nonclosure_time_s'] + summary['closure_time_s'])
        self.assertTrue(summary['closure_combined_limit_pass'])

    def test_existing_generation_is_rejected_before_computation(self):
        with self.assertRaises(FileExistsError):
            generate('fixed')

    def test_closed_backward_segment_alignment(self):
        ds = np.array([1., 2., 3., 4., 5.])
        # Edges traversed by reversed point order 4,3,2,1,0,4.
        np.testing.assert_array_equal(np.flip(np.roll(ds, 1)), [4, 3, 2, 1, 5])

    def test_backward_braking_nonuniform_edges_and_settled_seam(self):
        # One lateral bottleneck at point 3; all other points are straight.
        # Zero longitudinal capacity at the bottleneck makes its adjacent
        # speeds equal. Beyond those edges, v^2 grows by 2*a*distance.
        ds = np.array([.2, .7, 1.1, .3, .9, .4, 1.3, .5])
        kappa = np.zeros(8)
        kappa[3] = .5
        p = dict(mu=.6, max_longitudinal_acc=8.5, max_lateral_acc=8.5)
        vehicle = dict(max_speed=8., vehicle_mass=3.71)
        acceleration = p['mu'] * p['max_longitudinal_acc']
        corner_v2 = p['mu'] * p['max_lateral_acc'] / kappa[3]
        expected = np.full(8, vehicle['max_speed'])
        for direction in (1, -1):
            v2 = corner_v2
            for step in range(8):
                i = (3 + direction * step) % 8
                expected[i] = min(expected[i], np.sqrt(v2))
                edge = i if direction == 1 else (i - 1) % 8
                if step:
                    v2 += 2 * acceleration * ds[edge]
        for offset in range(8):
            with self.subTest(offset=offset):
                actual = corrected_closed_solver(np.roll(kappa, -offset),
                    np.roll(ds, -offset), p, vehicle)
                np.testing.assert_allclose(np.roll(actual, offset), expected,
                                           rtol=1e-12, atol=1e-10)

    def test_esp_start_index_invariance_velocity_not_only_time(self):
        g = np.loadtxt('Data/racelines/mu60/esp_raceline.csv', delimiter=',')[:, :5]
        p, vehicle = parameters()
        reference, _, _ = solve_closed_velocity_profile(g, p, vehicle)
        for offset in [0, 100, 500, 941]:
            shifted, _, _ = solve_closed_velocity_profile(np.roll(g, -offset, axis=0), p, vehicle)
            np.testing.assert_allclose(np.roll(shifted[:, 5], offset), reference[:, 5],
                                       rtol=1e-12, atol=1e-10)
            self.assertTrue(validate(shifted, p, vehicle)['strict_combined_limit_pass'])

    def test_full_n_combined_constraints_and_closure(self):
        p, vehicle = parameters()
        for name in ['mu60_closed', 'mu60_closed_regenerated']:
            data = np.loadtxt('Data/racelines/' + name + '/esp_raceline.csv', delimiter=',')
            result = validate(data, p, vehicle)
            self.assertEqual(result['segment_count'], len(data))
            self.assertLessEqual(result['max_combined_limit_residual_mps2'], 1e-9)
            self.assertEqual(result['constraint_violation_count'], 0)
            self.assertTrue(result['closure_combined_limit_pass'])
            self.assertAlmostEqual(result['planned_time_s'],
                                   result['nonclosure_time_s'] + result['closure_time_s'], places=12)

    @pytest.mark.local_archive
    def test_baseline_and_upstream_hashes(self):
        current = protected_hashes()
        for name in ['mu60_closed', 'mu60_closed_regenerated']:
            metadata = json.loads(Path('Data/raceline_data', name, 'metadata.json').read_text())
            self.assertEqual(metadata['protected_hashes'], current)
        self.assertEqual(digest('Data/racelines/mu60/esp_raceline.csv'),
                         '61ef20fd634e57befe5c9e3d8086575d507b5eaeffe5fdc54f9ea0f61b1b5a08')
        self.assertIn('Logs/ESP_GlobalPP/RawData_esp_pp_test/SimLog_esp_0.npy', current)
        self.assertIn('trajectory_planning_helpers/trajectory_planning_helpers/calc_vel_profile.py', current)

    def test_upstream_module_is_not_monkeypatched(self):
        import trajectory_planning_helpers.calc_vel_profile as helper
        original = getattr(helper, '__solver_fb_acc_profile')
        p, vehicle = parameters()
        g = np.loadtxt('Data/racelines/mu60/esp_raceline.csv', delimiter=',')[:, :5]
        check_start_index_invariance(g, p, vehicle)
        self.assertIs(getattr(helper, '__solver_fb_acc_profile'), original)


if __name__ == '__main__':
    unittest.main()
