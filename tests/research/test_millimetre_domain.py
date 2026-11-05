"""Safety-bound and envelope invariants for the opt-in offline experiment."""
import unittest
from types import SimpleNamespace

import numpy as np

from f1tenth_benchmarks.research.overtaking.millimetre_domain import (
    MillimetreDomain, curvature_upper_bound, clearance_lower_grid,
    hermite_values, hermite_segment_ranges, outside_in_boundary_checks,
    blocked_centre_tree, clearance_lower_at_points,
    vehicle_clearance_profile,
    centerline_targets,
)
from f1tenth_benchmarks.research.core.geometry.map_raster import MapRaster


class MillimetreDomainTests(unittest.TestCase):
    @staticmethod
    def straight_domain(lower=-.4, upper=.2):
        edges = np.linspace(0, 4, 4001)
        return MillimetreDomain(dict(edges=edges, sites=(edges[:-1]+edges[1:])/2,
            kappa=np.zeros(4000), lower=np.full(4000, lower), upper=np.full(4000, upper)),
            dict(step_m=.001, length_m=4.))

    def test_asymmetric_targets_anchor_c3_to_reference(self):
        targets, available = centerline_targets(-.4, .2, .001)
        np.testing.assert_allclose(targets, [-.4, -.2, 0., .1, .2])
        self.assertTrue(available.all())
        self.assertEqual(targets[2], 0.)

    def test_c3_matches_actual_start_and_returns_to_reference(self):
        domain = self.straight_domain()
        result = domain.plan(.2, .07, -.08, target_policy='centerline_return')
        centre = result['paths'][2]
        self.assertEqual(centre['target_d_m'], 0.)
        self.assertEqual(centre['target_role'], 'reference_centerline_return')
        self.assertTrue(centre['experimental_valid'])
        d, p, _ = hermite_values(np.array([0., 3.]), 3., .07, -.08, centre['target_d_m'])
        np.testing.assert_allclose(d, [.07, 0.], atol=1e-12)
        np.testing.assert_allclose(p, [-.08, 0.], atol=1e-12)

    def test_default_c3_is_reference_not_return_curve(self):
        result = self.straight_domain().plan(.2, .07, -.08)
        centre = result['paths'][2]
        self.assertEqual(centre['path_type'], 'reference_centerline')
        self.assertEqual(centre['target_role'], 'reference_centerline')
        self.assertEqual(centre['max_abs_curvature_inv_m'], 0.)
        self.assertTrue(centre['experimental_valid'])
        self.assertEqual(result['fallback_candidate_id'], 3)
        self.assertFalse(result['requires_stop'])

    def test_reference_boundary_is_always_checked_separately(self):
        domain = self.straight_domain()
        domain.arrays['lower'][300:310] = .01
        result = domain.plan(0., .1, 0.)
        self.assertFalse(result['paths'][2]['boundary_valid'])
        self.assertEqual(result['paths'][2]['boundary_check'], 'explicit_reference_domain')
        self.assertIsNone(result['fallback_candidate_id'])

    def test_reference_does_not_bypass_curvature_or_force_a_valid_path(self):
        domain = self.straight_domain()
        domain.arrays['kappa'][:] = 2.
        result = domain.plan(0., 0., 0., max_curvature=.01)
        self.assertFalse(result['paths'][2]['experimental_valid'])
        self.assertEqual(result['valid_ids'], [])
        self.assertTrue(result['requires_stop'])
        self.assertIsNone(result['fallback_candidate_id'])

    def test_aligned_c3_is_reference_everywhere(self):
        result = self.straight_domain().plan(0., 0., 0.)
        d, p, _ = hermite_values(np.linspace(0, 3, 301), 3., 0., 0., result['paths'][2]['target_d_m'])
        np.testing.assert_array_equal(d, np.zeros(301))
        np.testing.assert_array_equal(p, np.zeros(301))

    def test_forbidden_reference_is_rejected_not_shifted(self):
        result = self.straight_domain(.05, .4).plan(0., .1, 0.)
        centre = result['paths'][2]
        self.assertEqual(centre['target_d_m'], 0.)
        self.assertFalse(centre['boundary_valid'])
        self.assertFalse(centre['experimental_valid'])
        self.assertEqual(result['paths'][0]['reason'], 'no_lateral_target')
        self.assertEqual(result['paths'][1]['reason'], 'no_lateral_target')
        self.assertTrue(np.all(np.diff([p['target_d_m'] for p in result['paths']]) >= 0))

    def test_centerline_ordering_keeps_inheritance_equivalent(self):
        for bounds in [(-.4, .2), (-.2, .4), (.05, .4), (-.4, -.05)]:
            domain = self.straight_domain(*bounds)
            start = sum(bounds)/2
            inherited = domain.plan(.1, start, 0.)
            explicit = domain.plan(.1, start, 0., inherit=False)
            self.assertEqual(inherited['valid_ids'], explicit['valid_ids'])

    def test_old_width_midpoint_policy_is_explicit_only(self):
        domain = self.straight_domain()
        old = domain.plan(0., 0., 0., target_policy='terminal_width')
        self.assertAlmostEqual(old['paths'][2]['target_d_m'], -.1)
        self.assertEqual(domain.plan(0., 0., 0.)['paths'][2]['target_d_m'], 0.)

    def test_vehicle_edge_clearance_is_not_used_as_centre_margin(self):
        params = SimpleNamespace(vehicle_width=.20, width=.20, vehicle_length=.58, vehicle_boundary_clearance=.05)
        profile = vehicle_clearance_profile(params)
        self.assertAlmostEqual(profile['centre_prefilter_margin_m'], .15)
        self.assertFalse(profile['footprint_verified'])
        params.width = .31
        with self.assertRaisesRegex(ValueError, 'width fields disagree'):
            vehicle_clearance_profile(params)

    def test_curvature_bound_straight_and_parabola(self):
        straight = SimpleNamespace(coefficients=np.array([[[0., 0.], [2., 0.], [0., 0.], [0., 0.]]]))
        self.assertLess(curvature_upper_bound(straight), 1e-8)
        parabola = SimpleNamespace(coefficients=np.array([[[0., 0.], [1., 0.], [0., .2], [0., 0.]]]))
        self.assertGreaterEqual(curvature_upper_bound(parabola), .4)

    def test_degenerate_reference_fails_closed(self):
        geometry = SimpleNamespace(coefficients=np.zeros((1, 4, 2)))
        with self.assertRaises(ValueError):
            curvature_upper_bound(geometry)

    def test_clearance_treats_unknown_and_outside_as_blocked(self):
        gray = np.full((20, 20), 255, np.uint8)
        gray[10, 10] = 205  # unknown, not free
        raster = MapRaster(gray, dict(resolution=.05, origin=[0, 0, .3], negate=0,
                                      occupied_thresh=.65, free_thresh=.196))
        bounds = clearance_lower_grid(raster, np.ones(gray.shape, bool))
        self.assertEqual(bounds[10, 10], 0.)
        self.assertEqual(bounds[0, 0], 0.)
        # Nearest blocked square cannot be closer than this lower bound.
        self.assertLessEqual(bounds[5, 5], np.sqrt(5**2+5**2)*.05)

    def test_cubic_interior_extremum_not_missed(self):
        edges = np.array([0., 3.])
        lo, hi = hermite_segment_ranges(edges, 3., 0., 1., 0.)
        self.assertLess(lo[0], 1e-12)
        self.assertGreater(hi[0], .4)
        sampled = hermite_values(np.linspace(0, 3, 10001), 3., 0., 1., 0.)[0]
        self.assertGreaterEqual(hi[0]+1e-12, sampled.max())

    def test_point_clearance_is_lower_than_exact_square_distance(self):
        gray = np.full((20, 20), 255, np.uint8)
        gray[10, 10] = 0
        raster = MapRaster(gray, dict(resolution=.05, origin=[0, 0, .3], negate=0,
                                      occupied_thresh=.65, free_thresh=.196))
        tree = blocked_centre_tree(raster, np.ones(gray.shape, bool))
        rng = np.random.default_rng(91)
        local = rng.uniform(.06, .94, (500, 2))
        points = raster.local_to_map(local)
        lower = clearance_lower_at_points(raster, tree, points)
        centre = np.array([.525, .475])
        obstacle = np.linalg.norm(np.maximum(abs(local-centre)-.025, 0), axis=1)
        outside = np.min(np.column_stack([local, 1-local]), axis=1)
        self.assertTrue(np.all(lower <= np.minimum(obstacle, outside)+1e-12))

    def test_packed_lookup_respects_cell_and_outside(self):
        safe = np.array([[True, False, True], [False, True, False]])
        domain = MillimetreDomain(dict(sites=np.array([.0005, .0015]),
            packed_safe_cells=np.packbits(safe, axis=1)), dict(step_m=.001, length_m=.002,
            d_first_index=-1, d_last_index=2))
        self.assertTrue(domain.contains(.0002, -.0005))
        self.assertFalse(domain.contains(.0002, .0005))
        self.assertTrue(domain.contains(.0012, .0005))
        self.assertFalse(domain.contains(.0012, .003))

    def test_reference_derivative_uses_same_interpolated_kappa(self):
        sites = np.array([.0005, .0015, .0025, .0035])
        kappa = np.array([0., 1., 1., 0.])
        domain = MillimetreDomain(dict(sites=sites, kappa=kappa), dict(step_m=.001, length_m=.004))
        source = np.array([.00001, .0011, .00399])
        k, kp = domain.reference(source)
        expected = (np.interp(source+.001, sites, kappa, period=.004)-
                    np.interp(source-.001, sites, kappa, period=.004))/.002
        np.testing.assert_allclose(kp, expected)

    def test_ordered_hermite_inheritance_equals_explicit(self):
        rng = np.random.default_rng(37)
        edges = np.linspace(0., 3., 301)
        for _ in range(100):
            d0, slope = rng.uniform(-.2, .2, 2)
            targets = np.sort(rng.uniform(-.5, .5, 5))
            ranges = [hermite_segment_ranges(edges, 3., d0, slope, t) for t in targets]
            lower = np.full(300, rng.uniform(-.7, -.1))
            upper = np.full(300, rng.uniform(.1, .7))
            inherited = outside_in_boundary_checks(ranges, lower, upper, True)
            explicit = outside_in_boundary_checks(ranges, lower, upper, False)
            np.testing.assert_array_equal(inherited[0], explicit[0])

    def test_passing_extremes_skip_three_boundary_checks(self):
        ranges = [(np.full(3, x), np.full(3, x)) for x in np.linspace(-.4, .4, 5)]
        passed, routes, checked, inferred = outside_in_boundary_checks(ranges, np.full(3, -.5), np.full(3, .5))
        self.assertTrue(passed.all())
        self.assertEqual((checked, inferred), (2, 3))

    def test_invalid_domain_never_inherits(self):
        ranges = [(np.zeros(2), np.zeros(2)) for _ in range(5)]
        passed, _, checked, inferred = outside_in_boundary_checks(ranges, np.array([np.nan, -.5]), np.ones(2))
        self.assertFalse(passed.any())
        self.assertEqual((checked, inferred), (5, 0))

    def test_grid_seam_and_short_final_cell(self):
        length, step = .0104, .001
        edges = np.r_[np.arange(0, length, step), length]
        domain = MillimetreDomain(dict(edges=edges, sites=(edges[:-1]+edges[1:])/2),
                                  dict(step_m=step, length_m=length))
        distance, indices = domain.segment_grid(.0098, .003)
        self.assertAlmostEqual(distance[0], 0.)
        self.assertAlmostEqual(distance[-1], .003)
        self.assertLessEqual(np.diff(distance).max(), step+1e-12)
        np.testing.assert_array_equal(indices, [9, 10, 0, 1, 2])


if __name__ == '__main__':
    unittest.main()
