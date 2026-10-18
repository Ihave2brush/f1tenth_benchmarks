"""Constrained adaptive paths, retaining V5 as an unchanged baseline.

Quintic C4 B-splines track V5's preferred road fractions under sampled position,
slope and Cartesian-curvature constraints. Refine with failed validation sites.
Exact occupancy checks certify only the output polyline's point sweep, not the
spline between points, a vehicle footprint or vehicle dynamics.
"""
import argparse
import itertools
import json
import math

import numpy as np
from scipy.interpolate import BSpline
from scipy.optimize import minimize

from ..core.geometry import load_geometry
from .candidate_validation import MAX_SAMPLES


def _positive(value, name):
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be positive and finite')
    return value


def query_road_sections(geometry, source_s, margin):
    """Raw exact boundaries; never smooth road restrictions."""
    result = []
    for source in source_s:
        s = float(source % geometry.curve.L if geometry.curve.closed else source)
        boundary = geometry.boundaries(s)
        right, left = boundary.get('d_right'), boundary.get('d_left')
        valid = bool(boundary['valid'] and right is not None and left is not None
                     and np.isfinite([right, left]).all())
        lo, hi = (float(right)+margin, float(left)-margin) if valid else (None, None)
        valid = bool(valid and lo < hi)
        result.append(dict(s=s, s_progress=float(source), d_min=lo, d_max=hi,
                           boundary_valid=valid, reason=None if valid else
                           (boundary.get('reason') or 'invalid_or_narrow_boundary')))
    return result


def validate_path_segments(raster, xy, max_cells=100000):
    """Supercover all cells touched by closed segments, including grid corners.

    Works in raster local coordinates, including YAML origin rotation. Treats
    unknown/outside cells as blocked. No midpoint-only or fixed-distance test.
    """
    if len(xy) < 2:
        return dict(valid=False, reason='insufficient_points', segment_index=None)
    visited = 0
    for index, (a, b) in enumerate(zip(xy, xy[1:])):
        try:
            endpoints = np.asarray([a, b], float)
        except (TypeError, ValueError):
            endpoints = np.array([])
        if endpoints.shape != (2, 2) or not np.isfinite(endpoints).all():
            return dict(valid=False, reason='invalid_coordinates', segment_index=index)
        q = raster.map_to_local(endpoints)/raster.resolution
        if (q < 0).any() or (q[:, 0] >= raster.width).any() or (q[:, 1] >= raster.height).any():
            return dict(valid=False, reason='outside_map', segment_index=index)
        delta = q[1]-q[0]
        times = {0., 1.}
        for axis in range(2):
            if abs(delta[axis]) > 1e-14:
                lo, hi = sorted(q[:, axis])
                for edge in range(math.ceil(lo), math.floor(hi)+1):
                    t = (edge-q[0, axis])/delta[axis]
                    if 0 <= t <= 1:
                        times.add(float(t))
        times = sorted(times)
        probes = times + [(a+b)/2 for a, b in zip(times, times[1:])]
        cells = set()
        for t in probes:
            p = q[0]+t*delta
            choices = []
            for value in p:
                nearest = round(float(value))
                choices.append([nearest-1, nearest] if abs(value-nearest) < 1e-10
                               else [math.floor(value)])
            cells.update(itertools.product(*choices))
        visited += len(cells)
        if visited > max_cells:
            return dict(valid=False, reason='segment_budget_exceeded', segment_index=index)
        for col, row in sorted(cells):
            if not (0 <= col < raster.width and 0 <= row < raster.height):
                reason = 'outside_map'
            else:
                occupancy = int(raster.occupancy[raster.height-1-row, col])
                reason = {0: None, -1: 'unknown', 1: 'occupied'}[occupancy]
            if reason:
                return dict(valid=False, reason='segment_'+reason, segment_index=index,
                            cell=[col, raster.height-1-row])
    return dict(valid=True, reason=None, segment_index=None)


def _basis(length, count):
    degree = 5
    knots = np.r_[np.zeros(degree+1),
                  np.linspace(0, length, count-degree+1)[1:-1],
                  np.full(degree+1, length)]
    return BSpline(knots, np.eye(count), degree)


def _reference_data(geometry, source_s):
    refs = [geometry.reference(float(s)) for s in source_s]
    curvature = np.array([r['kappa'] for r in refs])
    derivatives = []
    for s in source_s:
        lo, hi = s-.001, s+.001
        if not geometry.curve.closed:
            lo, hi = max(0, lo), min(geometry.curve.L, hi)
        derivatives.append((geometry.reference(float(hi))['kappa']-
                            geometry.reference(float(lo))['kappa'])/(hi-lo))
    return curvature, np.array(derivatives)


def evaluate_path_smoothness(d, dp, dpp, kappa, kappa_prime):
    jacobian = 1-kappa*d
    denominator = (jacobian**2+dp**2)**1.5
    curvature = (jacobian*(kappa*jacobian+dpp)+dp*(kappa_prime*d+2*kappa*dp))/np.maximum(denominator, 1e-15)
    return jacobian, curvature


def fit_candidate_path(basis, distances, bounds, preferred, d0, initial_slope,
                       target_d, kappa, kappa_prime, max_slope, max_curvature,
                       initial=None):
    """Sample-constrained solve. Independently validate the fitted output."""
    B, D1, D2, D3 = [basis(distances, nu=i) for i in range(4)]
    lower = np.array([b['d_min'] for b in bounds])
    upper = np.array([b['d_max'] for b in bounds])
    E = np.stack([basis(0), basis(0, nu=1), basis(distances[-1]), basis(distances[-1], nu=1)])
    endpoint = np.array([d0, initial_slope, target_d, 0.])
    weights = [(B, preferred, 1.), (D2, np.zeros(len(distances)), 2.),
               (D3, np.zeros(len(distances)), .2)]
    H = sum(weight*A.T@A/len(distances) for A, _, weight in weights)
    linear = sum(weight*A.T@goal/len(distances) for A, goal, weight in weights)
    initial = np.linalg.lstsq(B, preferred, rcond=None)[0] if initial is None else initial
    def restrictions(c):
        d, dp, dpp = B@c, D1@c, D2@c
        J, curvature = evaluate_path_smoothness(d, dp, dpp, kappa, kappa_prime)
        return np.r_[d-lower, upper-d, max_slope-dp, max_slope+dp,
                     J-.2, max_curvature-curvature, max_curvature+curvature]
    solve = minimize(lambda c: float(c@H@c-2*linear@c), initial,
        jac=lambda c: 2*(H@c-linear), method='SLSQP',
        constraints=[dict(type='eq', fun=lambda c: E@c-endpoint, jac=lambda c: E),
                     dict(type='ineq', fun=restrictions)],
        options=dict(maxiter=150, ftol=1e-9))
    accepted = bool(solve.success and np.max(np.abs(E@solve.x-endpoint)) < 1e-6
                    and np.min(restrictions(solve.x)) >= -1e-7)
    return solve.x, accepted, str(solve.message)


def generate_constrained_paths(geometry, x, y, yaw=None, lookahead_m=40.,
                               safety_margin_m=.2, sample_step_m=.25,
                               max_slope=1., max_curvature=None, max_refinements=2):
    """V4-compatible result, preserving all five IDs even for rejected fits.

    Initial/terminal heading constraints assume forward travel along s. Without
    yaw the initial tangent is assumed parallel to the reference. Curvature
    limits are kinematic diagnostics only, not speed/dynamic feasibility.
    """
    length = _positive(lookahead_m, 'lookahead_m')
    step = _positive(sample_step_m, 'sample_step_m')
    max_slope = _positive(max_slope, 'max_slope')
    margin = float(safety_margin_m)
    if not math.isfinite(margin) or margin < 0:
        raise ValueError('safety_margin_m must be finite and nonnegative')
    if isinstance(max_refinements, bool) or not isinstance(max_refinements, int) or not 0 <= max_refinements <= 5:
        raise ValueError('max_refinements must be an integer from 0 to 5')
    if max_curvature is None:
        from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
        vehicle = load_parameter_file('vehicle_params')
        max_curvature = math.tan(vehicle.max_steer)/vehicle.wheelbase
    max_curvature = _positive(max_curvature, 'max_curvature')
    # Internal dense checking is independent of requested plotting resolution.
    validation_step = min(step, geometry.raster.resolution/2, .05)
    ratio = length/validation_step
    if not math.isfinite(ratio) or ratio > MAX_SAMPLES-1:
        raise ValueError(f'Path would exceed {MAX_SAMPLES} validation samples')
    start = geometry.to_frenet([x, y], yaw=yaw)
    if not start['valid']:
        raise ValueError('Invalid vehicle start: '+str(start.get('reason')))
    s0, d0 = float(start['s_wrapped']), float(start['d'])
    if geometry.curve.closed and length >= geometry.curve.L:
        raise ValueError('Lookahead must be shorter than one full lap')
    if not geometry.curve.closed and s0+length > geometry.curve.L:
        raise ValueError('Lookahead exceeds open reference')
    epsi = 0. if yaw is None else float(start['e_psi'])
    if abs(epsi) >= math.pi/2-1e-3:
        raise ValueError('Initial heading must point forward along reference')
    slope0 = (1-start['kappa']*d0)*math.tan(epsi)
    dense = np.linspace(0, length, math.ceil(ratio)+1)
    sections = query_road_sections(geometry, s0+dense, margin)
    first, last = sections[0], sections[-1]
    if not first['boundary_valid'] or not first['d_min'] <= d0 <= first['d_max']:
        raise ValueError('Invalid or insufficient boundary at start')
    f0 = (d0-first['d_min'])/(first['d_max']-first['d_min'])
    count = min(20, max(6, math.ceil(length/2)+5))
    basis = _basis(length, count)
    kappa, kprime = _reference_data(geometry, s0+dense)
    paths = []
    for cid, fraction in enumerate(np.linspace(0, 1, 5), 1):
        target = None if not last['boundary_valid'] else last['d_min']+fraction*(last['d_max']-last['d_min'])
        reason, points, coefficients = None, [], None
        validated_coefficients = None
        optimizer_message, attempts = None, 0
        if not all(s['boundary_valid'] for s in sections):
            reason = 'invalid_road_section'
        elif abs(slope0) > max_slope:
            reason = 'initial_heading_exceeds_slope_limit'
        else:
            indices = np.unique(np.r_[np.linspace(0, len(dense)-1, min(161, len(dense))).astype(int), len(dense)-1])
            for iteration in range(max_refinements+1):
                ds = dense[indices]; u = ds/length; blend = 3*u*u-2*u*u*u
                selected = [sections[i] for i in indices]
                preferred = np.array([s['d_min']+((1-b)*f0+b*fraction)*(s['d_max']-s['d_min'])
                                      for s, b in zip(selected, blend)])
                coefficients, fitted, message = fit_candidate_path(basis, ds, selected, preferred,
                    d0, slope0, target, kappa[indices], kprime[indices], max_slope,
                    max_curvature, initial=coefficients)
                optimizer_message, attempts = message, iteration+1
                if not fitted:
                    reason = 'fit_not_accepted'
                    break
                validated_coefficients = coefficients.copy()
                d, dp, dpp = [basis(dense, nu=i)@coefficients for i in range(3)]
                J, curvature = evaluate_path_smoothness(d, dp, dpp, kappa, kprime)
                points = []
                for i, (section, lateral) in enumerate(zip(sections, d)):
                    lateral = float(lateral)
                    heading = math.atan2(float(dp[i]), float(J[i]))
                    exact = geometry.to_cartesian(section['s'], lateral, e_psi=heading)
                    xy = exact.get('xy')
                    finite = xy is not None and np.shape(xy) == (2,) and np.isfinite(xy).all()
                    if exact.get('geometry_id') != geometry.geometry_id:
                        raise ValueError('Exact geometry version mismatch')
                    margin_ok = section['d_min']-1e-8 <= lateral <= section['d_max']+1e-8
                    limits_ok = abs(dp[i]) <= max_slope+1e-7 and abs(curvature[i]) <= max_curvature+1e-7
                    valid = bool(exact['valid'] and finite and margin_ok and limits_ok)
                    why = None if valid else (exact.get('reason') or
                        ('invalid_coordinates' if not finite else
                         'insufficient_lateral_margin' if not margin_ok else 'smoothness_limit'))
                    points.append(dict(section, sample_index=i, t=float(dense[i]/length),
                        d=lateral, d_prime=float(dp[i]), curvature_inv_m=float(curvature[i]),
                        x=float(xy[0]) if finite else None, y=float(xy[1]) if finite else None,
                        yaw=exact.get('yaw'), margin_valid=bool(margin_ok),
                        geometry_valid=bool(exact['valid'] and finite), valid=valid, reason=why))
                bad = [i for i,p in enumerate(points) if not p['valid']]
                if bad:
                    reason = points[bad[0]]['reason']
                    indices = np.unique(np.r_[indices, bad])
                    continue
                segments = validate_path_segments(geometry.raster, [[p['x'],p['y']] for p in points])
                if not segments['valid']:
                    reason = segments['reason']
                    index = segments['segment_index']+1
                    points[index].update(valid=False, reason=reason)
                    # Occupancy is not a convex road-fraction restriction. Reject
                    # this candidate rather than claim an obstacle-avoiding fit.
                    break
                reason = None
                break
        valid = reason is None
        invalid = next((i for i,p in enumerate(points) if not p['valid']), None)
        paths.append(dict(candidate_id=cid, geometry_id=geometry.geometry_id, frame_id='map',
            target_fraction=float(fraction), target_s=last['s'], target_d=target,
            target_valid=bool(points and points[-1]['geometry_valid'] and points[-1]['margin_valid']),
            sampled_valid=valid, status='sampled_valid' if valid else 'rejected', reason=reason,
            first_invalid_sample=invalid, points=points, polyline_collision_verified=valid,
            heading_matched=valid and yaw is not None, optimizer_message=optimizer_message,
            fit_attempts=attempts,
            spline=None if validated_coefficients is None else dict(knots=basis.t.tolist(),
                coefficients=validated_coefficients.tolist(), degree=basis.k, source_s_origin=s0)))
    return dict(schema_version=1, method='constrained_adaptive_bspline_v1',
        geometry_id=geometry.geometry_id, frame_id='map', start=dict(x=float(x), y=float(y), s=s0, d=d0),
        candidate_count=5, valid_target_count=sum(p['target_valid'] for p in paths),
        sampled_valid_count=sum(p['sampled_valid'] for p in paths), paths=paths,
        lookahead_m=length, requested_sample_step_m=step, actual_sample_step_m=length/(len(dense)-1),
        safety_margin_m=margin, max_slope=max_slope, max_curvature_inv_m=max_curvature,
        initial_heading_assumed=yaw is None, planning_allowed=False, domain_verified=False,
        inspection_only=True, continuous_verified=False, footprint_verified=False,
        dynamics_verified=False, opponents_verified=False,
        verification_level='dense_samples_and_exact_polyline_point_sweep')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry-dir', required=True)
    parser.add_argument('--map-yaml', required=True)
    for name in ('x', 'y'):
        parser.add_argument('--'+name, required=True, type=float)
    parser.add_argument('--yaw', type=float)
    parser.add_argument('--lookahead', type=float, default=40.)
    parser.add_argument('--margin', type=float, default=.2)
    parser.add_argument('--sample-step', type=float, default=.25)
    parser.add_argument('--max-slope', type=float, default=1.)
    parser.add_argument('--max-curvature', type=float)
    parser.add_argument('--save')
    parser.add_argument('--no-show', action='store_true')
    args = parser.parse_args()
    geometry = load_geometry(args.geometry_dir, args.map_yaml, inspect=True)
    try:
        result = generate_constrained_paths(geometry, args.x, args.y, args.yaw,
            args.lookahead, args.margin, args.sample_step, args.max_slope, args.max_curvature)
    except ValueError as exc:
        parser.error(str(exc))
    print(json.dumps(result, allow_nan=False))
    if args.save or not args.no_show:
        from .candidate_visualization import plot_candidate_paths
        import matplotlib.pyplot as plt
        fig, _ = plot_candidate_paths(geometry, result)
        if args.save:
            fig.savefig(args.save, dpi=180, bbox_inches='tight')
        if args.no_show:
            plt.close(fig)
        else:
            plt.show()


if __name__ == '__main__':
    main()
