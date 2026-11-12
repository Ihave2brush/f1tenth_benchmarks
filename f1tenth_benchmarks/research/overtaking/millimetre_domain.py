"""Experimental offline millimetre domain; no changes to the default planner.

Each accepted (s,d) rectangle has a conservative raster-clearance bound.
All safe cells are retained as packed bits; a single connected lateral run per
s-row is selected for the ordered-candidate experiment. Unknown and outside
are blocked. This is a path-centre domain, NOT a vehicle footprint certificate.
Frenet uniqueness and candidate curvature still have separate sampled audits.
"""
import hashlib
import json
import math
from pathlib import Path
from time import perf_counter

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

from ..core.geometry.extraction import ray_boundary
from .road_table import source_fingerprint
from .reference_path import REFERENCE_PATH, STATE_PATH

POLICY = 'offline_mm_clearance_rectangles_hermite_envelope_v2'


def vehicle_clearance_profile(params):
    """Translate body-edge request to a centre-path PREFILTER, not a footprint.

    Both simulator width fields must agree. The body's corners and its
    heading are deliberately not certified by this lateral approximation.
    """
    width = float(params.vehicle_width)
    dynamics_width = float(params.width)
    length = float(params.vehicle_length)
    clearance = float(params.vehicle_boundary_clearance)
    if not np.isfinite([width, dynamics_width, length, clearance]).all() or min(width, length) <= 0 or clearance < 0:
        raise ValueError('invalid vehicle clearance dimensions')
    if abs(width-dynamics_width) > 1e-12:
        raise ValueError('simulator width fields disagree')
    return dict(vehicle_width_m=width, vehicle_length_m=length,
        requested_body_edge_clearance_m=clearance,
        centre_prefilter_margin_m=width/2+clearance,
        footprint_verified=False,
        limitation='half-width prefilter only; oriented body corners and swept footprint not verified')


def _bernstein(power):
    degree = len(power)-1
    return np.asarray([sum(power[j]*math.comb(k, j)/math.comb(degree, j)
                           for j in range(k+1)) for k in range(degree+1)])


def curvature_upper_bound(geometry):
    """Per cubic span, bound cross(r',r'') and |r'|² in Bernstein form.

    A positive lower speed bound is mandatory. Fail closed instead of using
    a sampled maximum as a continuous curvature bound.
    """
    bounds = []
    for coefficients in geometry.coefficients:
        first = coefficients[1:]*np.arange(1, 4)[:, None]
        second = coefficients[2:]*np.array([2., 6.])[:, None]
        square = sum(np.polynomial.polynomial.polymul(first[:, a], first[:, a])
                     for a in range(2))
        cross = (np.polynomial.polynomial.polymul(first[:, 0], second[:, 1])-
                 np.polynomial.polynomial.polymul(first[:, 1], second[:, 0]))
        lower = float(_bernstein(square).min())
        if lower <= 1e-15:
            raise ValueError('cannot bound reference speed; subdivision required')
        bounds.append(float(np.abs(_bernstein(cross)).max())/lower**1.5)
    return max(bounds)*(1+1e-10)+1e-10


def clearance_lower_grid(raster, corridor):
    """Lower Euclidean clearance valid everywhere in each original map cell.

    EDT measures centre-to-centre distances. Subtract two half diagonals
    (query cell and obstacle cell); pad the map with blocked cells.
    """
    free = (raster.occupancy == 0) & corridor
    padded = np.pad(free, 1, constant_values=False)
    distance = ndimage.distance_transform_edt(padded)[1:-1, 1:-1]
    return np.maximum(0., (distance-math.sqrt(2))*raster.resolution)


def blocked_centre_tree(raster, corridor):
    """Include one outside border; no exterior square can be closer than it."""
    free = np.pad((raster.occupancy == 0) & corridor, 1, constant_values=False)
    rows, columns = np.nonzero(~free)
    points = raster.pixel_to_map(columns-1, rows-1)
    return cKDTree(points)


def clearance_lower_at_points(raster, tree, points):
    # Every blocked square lies in the disc of radius half a pixel diagonal
    # around its centre. Distance to the union of those discs is a lower bound.
    distances, _ = tree.query(np.asarray(points).reshape(-1, 2), workers=1)
    return np.maximum(0., distances-raster.resolution/math.sqrt(2)).reshape(np.shape(points)[:-1])


def hermite_values(distance, length, d0, slope0, target):
    u = np.asarray(distance)/length
    delta = target-d0
    d = d0+length*slope0*u+(3*delta-2*length*slope0)*u*u+(-2*delta+length*slope0)*u**3
    p = slope0+2*(3*delta-2*length*slope0)*u/length+3*(-2*delta+length*slope0)*u*u/length
    q = (2*(3*delta-2*length*slope0)+6*(-2*delta+length*slope0)*u)/length**2
    return d, p, q


def centerline_targets(lower, upper, step, lateral_inset_m=0.):
    """Ordered targets with C3 fixed at reference d=0, never road midpoint.

    Missing lateral space rejects that side's slots instead of moving C3 or
    relabelling positive offsets as right-side candidates. A forbidden d=0
    remains an explicit proposal and must fail the normal boundary check.
    """
    if (not np.isfinite([lower, upper, step, lateral_inset_m]).all()
            or lower >= upper or step <= 0 or lateral_inset_m < 0):
        raise ValueError('invalid terminal interval')
    targets = np.zeros(5)
    available = np.zeros(5, dtype=bool)
    available[2] = True
    right_low = math.ceil((lower+lateral_inset_m)/step-1e-10)*step
    right_high = math.floor(min(upper, 0.)/step+1e-10)*step
    if right_low < 0 and right_low <= right_high:
        targets[0] = right_low
        targets[1] = min(-step, np.rint((right_low+right_high)/(2*step))*step)
        targets[1] = np.clip(targets[1], right_low, right_high)
        available[0] = True
        available[1] = targets[0] < targets[1] < 0
    left_low = math.ceil(max(lower, 0.)/step-1e-10)*step
    left_high = math.floor((upper-lateral_inset_m)/step+1e-10)*step
    if left_high > 0 and left_low <= left_high:
        targets[4] = left_high
        targets[3] = max(step, np.rint((left_low+left_high)/(2*step))*step)
        targets[3] = np.clip(targets[3], left_low, left_high)
        available[4] = True
        available[3] = 0 < targets[3] < targets[4]
    return targets, available


def hermite_segment_ranges(edges, length, d0, slope0, target):
    """Exact cubic extrema on each s-cell, including interior stationary roots."""
    d = hermite_values(edges, length, d0, slope0, target)[0]
    lower, upper = np.minimum(d[:-1], d[1:]), np.maximum(d[:-1], d[1:])
    delta = target-d0
    polynomial = np.trim_zeros(np.array([3*(-2*delta+length*slope0),
                                       2*(3*delta-2*length*slope0), length*slope0]), 'f')
    if len(polynomial) > 1:
        for root in np.roots(polynomial):
            if abs(root.imag) < 1e-12 and 0 < root.real < 1:
                x = float(root.real*length)
                index = min(len(lower)-1, int(np.searchsorted(edges, x, side='right')-1))
                value = float(hermite_values(x, length, d0, slope0, target)[0])
                lower[index] = min(lower[index], value)
                upper[index] = max(upper[index], value)
    return lower, upper


def outside_in_boundary_checks(ranges, lower, upper, inherit=True):
    """Targets are ordered. Shared Hermite start/slope guarantees no crossing.

    Passing extremes cover every intervening target for the SAME cell bounds.
    A failed extreme says nothing about its neighbours. Never inherit curvature.
    """
    count = len(ranges)
    passed = np.zeros(count, dtype=bool)
    routes = ['not_checked']*count
    checked, inferred, right, left = 0, 0, 0, count-1
    # Walk each side inward until finding a passing curve. Only then infer
    # candidates bracketed by two passing curves.
    right_ok = left_ok = None
    while right <= left:
        if right_ok is None:
            lo, hi = ranges[right]
            passed[right] = bool(np.all(lo >= lower) and np.all(hi <= upper))
            routes[right] = 'explicit'
            checked += 1
            if passed[right]:
                right_ok = right
            right += 1
        if right <= left and left_ok is None:
            lo, hi = ranges[left]
            passed[left] = bool(np.all(lo >= lower) and np.all(hi <= upper))
            routes[left] = 'explicit'
            checked += 1
            if passed[left]:
                left_ok = left
            left -= 1
        if inherit and right_ok is not None and left_ok is not None:
            for index in range(right, left+1):
                passed[index] = True
                routes[index] = 'inherited_between_passing_extremes'
                inferred += 1
            break
        if right_ok is not None and left_ok is not None:
            for index in range(right, left+1):
                lo, hi = ranges[index]
                passed[index] = bool(np.all(lo >= lower) and np.all(hi <= upper))
                routes[index] = 'explicit'
                checked += 1
            break
        # A passing right side stays fixed while the left advances, and vice versa.
        if right_ok is not None and left_ok is None and right <= left:
            continue
        if left_ok is not None and right_ok is None and right <= left:
            continue
    return passed, routes, checked, inferred


class _LazyRanges:
    """Avoid building extrema arrays for boundary checks that are inherited."""
    def __init__(self, targets, distance, length, d0, slope0):
        self.targets, self.distance, self.length = targets, distance, length
        self.d0, self.slope0, self.cache = d0, slope0, {}

    def __len__(self):
        return len(self.targets)

    def __getitem__(self, index):
        if index not in self.cache:
            self.cache[index] = hermite_segment_ranges(self.distance, self.length,
                self.d0, self.slope0, self.targets[index])
        return self.cache[index]


class MillimetreDomain:
    def __init__(self, arrays, metadata):
        self.arrays, self.metadata = arrays, metadata
        self.step = float(metadata['step_m'])
        self.length = float(metadata['length_m'])

    @classmethod
    def build(cls, geometry, step=.001, margin=.2, window=.025, numeric_guard=.0005, progress=None):
        if not geometry.curve.closed:
            raise ValueError('first experiment supports closed tracks only')
        if not (np.isfinite([step, margin, window, numeric_guard]).all()
                and 0 < step <= .001 and margin > 0 and window >= 0 and numeric_guard > 0):
            raise ValueError('invalid millimetre domain parameters')
        started = perf_counter()
        length = geometry.curve.L
        edges = np.r_[np.arange(0., length, step), length]
        sites = (edges[:-1]+edges[1:])*.5
        xy, tangent, psi, kappa = geometry.curve.sample(sites)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        plus = geometry.curve.sample((sites+.001) % length)[3]
        minus = geometry.curve.sample((sites-.001) % length)[3]
        kp = (plus-minus)/.002
        bound = curvature_upper_bound(geometry)
        left, right = [], []
        for i in range(len(sites)):
            _, dl, _, lr = ray_boundary(geometry.raster, xy[i], normal[i], geometry.corridor)
            _, dr, _, rr = ray_boundary(geometry.raster, xy[i], -normal[i], geometry.corridor)
            if lr is not None or rr is not None:
                raise ValueError('invalid offline ray at row '+str(i))
            left.append(dl)
            right.append(-dr)
        raw_lo, raw_hi = np.array(right), np.array(left)
        d_first = math.floor(float(raw_lo.min())/step)
        d_last = math.ceil(float(raw_hi.max())/step)
        d_centres = (np.arange(d_first, d_last)+.5)*step
        max_d = max(abs(d_first*step), abs(d_last*step))
        # F_s=(1-kappa*d)t and F_d=n. Bound displacement of the whole
        # rectangle from its exact centre, plus numerical conversion reserve.
        radius = (1+bound*max_d)*np.diff(edges)/2+step/2+numeric_guard
        tree = blocked_centre_tree(geometry.raster, geometry.corridor)
        lo = np.full(len(sites), np.nan)
        hi = np.full(len(sites), np.nan)
        accepted_cells = 0
        selected_cells = 0
        packed = np.zeros((len(sites), math.ceil(len(d_centres)/8)), dtype=np.uint8)
        multiple_run_rows = 0
        for begin in range(0, len(sites), 32):
            end = min(begin+32, len(sites))
            points = xy[begin:end, None, :]+d_centres[None, :, None]*normal[begin:end, None, :]
            local = geometry.raster.map_to_local(points)/geometry.raster.resolution
            cols = np.floor(local[..., 0]).astype(int)
            rows = geometry.raster.height-1-np.floor(local[..., 1]).astype(int)
            inside = ((cols >= 0) & (cols < geometry.raster.width) &
                      (rows >= 0) & (rows < geometry.raster.height))
            safe = np.zeros(inside.shape, dtype=bool)
            clearance = clearance_lower_at_points(geometry.raster, tree, points)
            safe = inside & (clearance > margin+radius[begin:end, None])
            safe &= ((d_centres[None, :]-step/2 >= raw_lo[begin:end, None]) &
                     (d_centres[None, :]+step/2 <= raw_hi[begin:end, None]))
            packed[begin:end] = np.packbits(safe, axis=1)
            accepted_cells += int(safe.sum())
            # Preserve every safe cell on disk. Select the widest connected
            # normal-corridor run for this simple single-band experiment.
            for j, mask in enumerate(safe):
                changes = np.diff(np.r_[False, mask, False].astype(np.int8))
                starts, stops = np.flatnonzero(changes == 1), np.flatnonzero(changes == -1)
                if len(starts):
                    selected = int(np.argmax(stops-starts))
                    a, b = int(starts[selected]), int(stops[selected])
                    lo[begin+j], hi[begin+j] = (d_first+a)*step, (d_first+b)*step
                    selected_cells += b-a
                    multiple_run_rows += int(len(starts) > 1)
            if progress and begin % 2048 == 0:
                progress(dict(stage='clearance_cells', rows=end, total=len(sites)))
        # Inward envelope removes short width spikes without expanding any
        # rectangle. It is intentionally not an unconstrained spline fit.
        size = 2*math.ceil(window/step)+1
        envelope_lo = ndimage.maximum_filter1d(np.where(np.isfinite(lo), lo, np.inf), size, mode='wrap')
        envelope_hi = ndimage.minimum_filter1d(np.where(np.isfinite(hi), hi, -np.inf), size, mode='wrap')
        usable = np.isfinite(envelope_lo) & np.isfinite(envelope_hi) & (envelope_lo < envelope_hi)
        envelope_lo[~usable], envelope_hi[~usable] = np.nan, np.nan
        arrays = dict(edges=edges, sites=sites, xy=xy, normal=normal, psi=psi, kappa=kappa,
                      kappa_prime=kp, lower=envelope_lo, upper=envelope_hi,
                      raw_lower=raw_lo, raw_upper=raw_hi, cell_lower=lo, cell_upper=hi,
                      packed_safe_cells=packed)
        metadata = dict(policy=POLICY, geometry_id=geometry.geometry_id,
            source_fingerprint=source_fingerprint(geometry), source_hashes=geometry.raster.source_hashes,
            step_m=step, d_step_m=step, length_m=length, rows=len(sites),
            d_first_index=d_first, d_last_index=d_last, enumerated_cells=len(sites)*len(d_centres),
            raw_safe_cells=accepted_cells, usable_rows=int(usable.sum()),
            selected_band_cells_before_envelope=selected_cells, multiple_run_rows=multiple_run_rows,
            clearance_bound='nearest blocked-pixel centre distance minus half diagonal',
            margin_m=margin, margin_metric='Euclidean path-centre clearance, not normal ray margin',
            inward_envelope_half_window_m=window, numerical_position_reserve_m=numeric_guard,
            reference_curvature_upper_bound_inv_m=bound,
            max_cell_displacement_bound_m=float(radius.max()),
            array_bytes=sum(a.nbytes for a in arrays.values()), build_wall_time_s=perf_counter()-started,
            footprint_verified=False, dynamics_verified=False, continuous_frenet_verified=False,
            planning_allowed=False,
            safety_scope='raster-clearance rectangles with analytic displacement bound and numerical reserve',
            limitation='reference arc inversion is numerical; projection uniqueness and curvature independently sampled')
        return cls(arrays, metadata)

    def save(self, directory):
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=False)
        path = root/'domain.npz'
        np.savez_compressed(path, **self.arrays)
        metadata = dict(self.metadata, npz_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        (root/'metadata.json').write_text(json.dumps(metadata, indent=2, allow_nan=False)+'\n')

    @classmethod
    def load(cls, directory, geometry, expected_margin=None):
        root = Path(directory)
        metadata = json.loads((root/'metadata.json').read_text())
        path = root/'domain.npz'
        if metadata['policy'] != POLICY or metadata['source_fingerprint'] != source_fingerprint(geometry):
            raise ValueError('domain source or policy changed')
        if expected_margin is not None and abs(metadata['margin_m']-expected_margin) > 1e-12:
            raise ValueError('domain margin changed; rebuild the offline artifact')
        if hashlib.sha256(path.read_bytes()).hexdigest() != metadata['npz_sha256']:
            raise ValueError('domain artifact checksum mismatch')
        with np.load(path, allow_pickle=False) as data:
            arrays = {name: data[name].copy() for name in data.files}
        return cls(arrays, metadata)

    def indices(self, source_s):
        wrapped = np.asarray(source_s) % self.length
        return np.minimum((wrapped/self.step).astype(int), len(self.arrays['sites'])-1)

    def contains(self, source_s, lateral):
        """O(1) packed lookup for a whole precomputed millimetre rectangle."""
        source_s, lateral = np.broadcast_arrays(np.asarray(source_s, float), np.asarray(lateral, float))
        if not np.isfinite(source_s).all() or not np.isfinite(lateral).all():
            raise ValueError('nonfinite cell lookup')
        rows = self.indices(source_s)
        columns = np.floor(lateral/self.step).astype(int)-self.metadata['d_first_index']
        count = self.metadata['d_last_index']-self.metadata['d_first_index']
        inside = (columns >= 0) & (columns < count)
        clipped = np.clip(columns, 0, count-1)
        byte = self.arrays['packed_safe_cells'][rows, clipped//8]
        return inside & (((byte >> (7-clipped % 8)) & 1) != 0)

    def reference(self, source_s):
        wrapped = np.asarray(source_s) % self.length
        sites = self.arrays['sites']
        def interp(position):
            return np.interp(position, sites, self.arrays['kappa'], period=self.length)
        k = interp(wrapped)
        # Match the legacy finite-difference definition on the SAME lookup
        # curve. Interpolating precomputed k' separately smears knot transitions.
        kp = (interp(wrapped+.001)-interp(wrapped-.001))/.002
        return k, kp

    def segment_grid(self, s0, length):
        if not 0 < length < self.length:
            raise ValueError('horizon must be positive and shorter than lap')
        start = float(s0 % self.length)
        edges = self.arrays['edges']
        extended = np.r_[edges, edges[1:]+self.length]
        selected = extended[(extended > start) & (extended < start+length)]
        distance = np.r_[0., selected-start, length]
        indices = self.indices(start+(distance[:-1]+distance[1:])/2)
        return distance, indices

    def plan(self, s0, d0, slope0, length=3., max_slope=1., max_curvature=None,
             inherit=True, target_policy='reference_centerline', lateral_inset_m=0.):
        """Table-only planning after a separately measured vehicle projection.

        Targets lie on integer-mm d coordinates. Interior paths remain analytic
        cubic Hermite curves. No solver, ray, inverse projection or XY output.
        """
        if max_curvature is None:
            from .session import vehicle_curvature_limit
            max_curvature = vehicle_curvature_limit()
        if target_policy not in ('reference_centerline', 'centerline_return', 'terminal_width'):
            raise ValueError('invalid target_policy')
        if not np.isfinite([s0, d0, slope0, length, max_slope, max_curvature]).all():
            raise ValueError('nonfinite plan input')
        distance, indices = self.segment_grid(s0, length)
        lower, upper = self.arrays['lower'][indices], self.arrays['upper'][indices]
        end = int(self.indices(s0+length))
        terminal = [self.arrays['lower'][end], self.arrays['upper'][end]]
        if not np.isfinite(terminal).all() or terminal[0] >= terminal[1]:
            return dict(paths=[], valid_ids=[], reason='no_terminal_domain', explicit_boundary_checks=0,
                        inherited_boundary_checks=0, planning_allowed=False)
        if target_policy in ('reference_centerline', 'centerline_return'):
            targets, available = centerline_targets(*terminal, self.step, lateral_inset_m)
        else:
            targets = np.rint(np.linspace(*terminal, 5)/self.step)*self.step
            targets = np.clip(targets, *terminal)
            available = np.ones(5, dtype=bool)
        if target_policy == 'reference_centerline':
            # C3 does not share the other curves' actual start and slope.
            # Validate it independently; no inheritance from the outer family.
            family_indices = [0, 1, 3, 4]
            ranges = _LazyRanges(targets[family_indices], distance, length, d0, slope0)
            family_passed, family_routes, checked, inferred = outside_in_boundary_checks(ranges, lower, upper, inherit)
            passed = np.zeros(5, dtype=bool)
            routes = ['explicit_reference_domain']*5
            passed[family_indices] = family_passed
            for i, route in zip(family_indices, family_routes):
                routes[i] = route
            passed[2] = bool(np.all(lower <= 0.) and np.all(upper >= 0.))
            checked += 1
        else:
            ranges = _LazyRanges(targets, distance, length, d0, slope0)
            passed, routes, checked, inferred = outside_in_boundary_checks(ranges, lower, upper, inherit)
        passed &= available
        # Curvature and J stay candidate-specific; evaluate all cell edges and
        # midpoints. Continuous occupancy does not imply continuous curvature.
        sites = np.sort(np.r_[distance, (distance[:-1]+distance[1:])/2])
        k, kp = self.reference(s0+sites)
        paths = []
        for i, target in enumerate(targets):
            is_reference = target_policy == 'reference_centerline' and i == 2
            d, p, q = ((np.zeros_like(sites),)*3 if is_reference
                       else hermite_values(sites, length, d0, slope0, target))
            jacobian = 1-k*d
            numerator = jacobian*(k*jacobian+q)+p*(kp*d+2*k*p)
            curvature = numerator/np.maximum((jacobian*jacobian+p*p)**1.5, 1e-15)
            smooth = bool(np.isfinite(curvature).all() and np.all(jacobian >= .2+1e-6)
                          and np.max(abs(p)) <= max_slope-1e-6
                          and np.max(abs(curvature)) <= max_curvature-1e-5)
            paths.append(dict(candidate_id=i+1, target_d_m=float(target), boundary_valid=bool(passed[i]),
                boundary_check=routes[i], smoothness_sampled_valid=smooth,
                experimental_valid=bool(passed[i] and smooth),
                reason=('no_lateral_target' if not available[i] else None if passed[i] and smooth
                        else 'outside_safe_domain' if not passed[i] else 'smoothness_limit'),
                path_type=REFERENCE_PATH if is_reference else STATE_PATH,
                target_role=('reference_centerline' if is_reference else
                             'reference_centerline_return' if target_policy == 'centerline_return' and i == 2 else 'lateral_target'),
                max_abs_curvature_inv_m=float(np.max(abs(curvature))),
                min_jacobian=float(jacobian.min())))
        return dict(paths=paths, valid_ids=[p['candidate_id'] for p in paths if p['experimental_valid']],
                    reason=None, explicit_boundary_checks=checked, inherited_boundary_checks=inferred,
                    target_policy=target_policy,
                    fallback_candidate_id=3 if target_policy == 'reference_centerline' and paths[2]['experimental_valid'] else None,
                    requires_stop=not any(p['experimental_valid'] for p in paths),
                    interval_count=len(indices), curvature_sample_count=len(sites),
                    planning_allowed=False, footprint_verified=False, continuous_frenet_verified=False,
                    online_rays=0, online_candidate_inverse_projections=0, online_candidate_xy_conversions=0)
