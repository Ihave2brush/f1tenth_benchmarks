"""Continuous spline projection; per-query checks are not global acceptance."""
import json
import math
from pathlib import Path

import numpy as np

from .map_raster import load_map
from .extraction import ReferenceCurve, corridor_loops, ray_boundary
from scipy import ndimage
from .map_raster import FREE


def wrap(angle):
    return float((angle + np.pi) % (2*np.pi) - np.pi)


def circular_distance(a, b, length):
    difference = abs(a-b) % length
    return min(difference, length-difference)


class TrackGeometry:
    def __init__(self, curve, raster, geometry_id, anchor, planning_allowed=False):
        self.curve, self.raster, self.geometry_id = curve, raster, geometry_id
        self.planning_allowed = bool(planning_allowed)
        if curve.closed:
            self.corridor, _ = corridor_loops(raster, anchor)
        else:
            pixel=raster.map_to_pixel(anchor)
            if not pixel.inside_map or raster.classify(anchor)!='free':raise ValueError('invalid open anchor')
            labels,_=ndimage.label(raster.occupancy==FREE)
            self.corridor=labels==labels[pixel.cell[1],pixel.cell[0]]
        self._prepare_spans()

    def distance_s(self,a,b):
        return circular_distance(a,b,self.curve.L) if self.curve.closed else abs(a-b)

    def _prepare_spans(self):
        """Cubic coefficients and exact coordinate extrema on each knot span."""
        nodes = self.curve.u_nodes
        self.starts, self.widths = nodes[:-1], np.diff(nodes)
        self.coefficients = np.stack([
            self.curve.evaluate(self.starts),
            self.curve.evaluate(self.starts, 1)*self.widths[:, None],
            self.curve.evaluate(self.starts, 2)*self.widths[:, None]**2/2,
            self.curve.evaluate(self.starts, 3)*self.widths[:, None]**3/6], axis=1)
        lower, upper = [], []
        for coefficients in self.coefficients:
            low, high = [], []
            for axis in range(2):
                p = coefficients[:, axis]
                roots = np.polynomial.polynomial.polyroots([p[1], 2*p[2], 3*p[3]])
                t = [0., 1.] + [float(z.real) for z in roots if abs(z.imag)<1e-9 and 0<z.real<1]
                values = np.polynomial.polynomial.polyval(t, p)
                low.append(min(values)); high.append(max(values))
            lower.append(low); upper.append(high)
        self.lower, self.upper = np.array(lower), np.array(upper)
        # Endpoints are on the actual curve, giving a valid nearest-distance upper bound.
        self._seed_xy = self.curve.evaluate(self.starts)

    def _s_from_u(self, u):
        return float((self.curve.arc(u)-self.curve.start_s) % self.curve.L) if self.curve.closed else self.curve.arc(u)

    def reference(self, s):
        if not np.isscalar(s) or not np.isfinite(s):
            raise ValueError('nonfinite_input')
        xy, tangent, psi, kappa = self.curve.sample([float(s)])
        return dict(geometry_id=self.geometry_id, frame_id='map', s_wrapped=float(s % self.curve.L) if self.curve.closed else float(s),
                    xy=xy[0].tolist(), tangent=tangent[0].tolist(), psi=float(psi[0]), kappa=float(kappa[0]))

    def boundaries(self, s):
        reference = self.reference(s)
        t = np.array(reference['tangent']); n = np.array([-t[1], t[0]])
        left, dl, lk, lr = ray_boundary(self.raster, reference['xy'], n, self.corridor)
        right, dr, rk, rr = ray_boundary(self.raster, reference['xy'], -n, self.corridor)
        return dict(s_wrapped=reference['s_wrapped'], left_xy=None if left is None else left.tolist(),
                    right_xy=None if right is None else right.tolist(), d_left=dl,
                    d_right=None if dr is None else -dr, left_kind=lk, right_kind=rk,
                    valid=lr is None and rr is None, reason=lr or rr,
                    method='exact_grid_ray', interpolation_verified=False)

    def _candidates(self, xy):
        # Prune only spans whose exact coordinate box exceeds a true upper bound
        # plus ambiguity tolerance; never rely on a nearest CSV segment alone.
        best = float(np.linalg.norm(self._seed_xy-xy, axis=1).min())
        tolerance = self.raster.resolution*.5
        box_distance = np.linalg.norm(np.maximum(np.maximum(self.lower-xy, xy-self.upper), 0), axis=1)
        indices = np.flatnonzero(box_distance <= best+tolerance+1e-10)
        candidates = []
        for index in indices:
            p = self.coefficients[index].copy(); p[0] -= xy
            derivative = np.arange(1, 4)[:, None]*p[1:]
            stationary = np.polynomial.polynomial.polyadd(
                np.polynomial.polynomial.polymul(p[:, 0], derivative[:, 0]),
                np.polynomial.polynomial.polymul(p[:, 1], derivative[:, 1]))
            roots = np.polynomial.polynomial.polyroots(stationary)
            values = [float(z.real) for z in roots if abs(z.imag) < 1e-7 and -1e-9 <= z.real <= 1+1e-9]
            # Include a stationary knot endpoint even if a root falls just outside
            # its polynomial span due to roundoff, but not arbitrary span endpoints.
            for endpoint in (0., 1.):
                if abs(np.polynomial.polynomial.polyval(endpoint, stationary)) <= 1e-10:
                    values.append(endpoint)
            for value in values:
                value = float(np.clip(value, 0, 1))
                u = self.starts[index] + value*self.widths[index]
                if self.curve.closed:u %= 1
                point = self.curve.evaluate(u)
                a, b = self.curve.evaluate(u, 1), self.curve.evaluate(u, 2)
                second = float(np.dot(a,a)+np.dot(point-xy,b))
                if second <= 1e-10:
                    continue
                s = self._s_from_u(u)
                if any(self.distance_s(s, c['s']) < 1e-5 for c in candidates):
                    continue
                candidates.append(dict(s=s, xy=point, distance=float(np.linalg.norm(point-xy))))
        return sorted(candidates, key=lambda c: c['distance'])

    def _base(self):
        return dict(geometry_id=self.geometry_id, frame_id='map', valid=False, reason=None,
                    planning_allowed=self.planning_allowed, domain_verified=False,
                    s_wrapped=None, d=None, projection_xy=None, psi_reference=None,
                    kappa=None, e_psi=None, candidate_count=0, inside_map=False,
                    inside_track=False, inside_valid_domain=False)

    def _check_candidate(self, point, candidate, yaw=None):
        """Local geometry diagnostics; a candidate is not a global selection."""
        ref = self.reference(candidate['s'])
        tangent = np.asarray(ref['tangent'])
        normal = np.array([-tangent[1], tangent[0]])
        d = float(np.dot(point-np.asarray(candidate['xy']), normal))
        boundary = self.boundaries(candidate['s'])
        jacobian = 1-ref['kappa']*d
        residual = float(np.linalg.norm(np.asarray(ref['xy'])+d*normal-point))
        reason = None
        if not boundary['valid']:
            reason = 'invalid_boundary'
        elif not boundary['d_right'] < d < boundary['d_left']:
            reason = 'outside_track'
        elif jacobian < .2:
            reason = 'singular_frenet'
        elif residual > 1e-4:
            reason = 'projection_residual'
        return dict(s=candidate['s'], d=d, distance=candidate['distance'],
                    projection_xy=np.asarray(candidate['xy']).tolist(),
                    psi_reference=ref['psi'], tangent=ref['tangent'], kappa=ref['kappa'],
                    jacobian=jacobian, reconstruction_error_m=residual,
                    e_psi=None if yaw is None else wrap(yaw-ref['psi']),
                    boundary=boundary, valid=reason is None, reason=reason)

    def projection_candidates(self, xy, yaw=None):
        """Expose all competitive local projections for diagnostics/tracking.

        This method does not select a reliable s/d or approve a planning domain.
        The static API and the stateful tracker apply their own selection rules.
        """
        point = np.asarray(xy, float)
        if point.shape != (2,) or not np.isfinite(point).all() or (yaw is not None and not np.isfinite(yaw)):
            return dict(valid=False,reason='nonfinite_input',candidates=[])
        occupancy = self.raster.classify(point)
        if occupancy != 'free':
            return dict(valid=False,reason=occupancy,candidates=[])
        col,row = self.raster.map_to_pixel(point).cell
        if not self.corridor[row,col]:
            return dict(valid=False,reason='outside_track',candidates=[])
        candidates = self._candidates(point)
        return dict(valid=bool(candidates),reason=None if candidates else 'no_projection',
                    candidates=[self._check_candidate(point,c,yaw) for c in candidates])

    def to_frenet(self, xy, yaw=None, s_hint=None, search_window_m=None):
        result = self._base()
        point = np.asarray(xy, float)
        if point.shape != (2,) or not np.isfinite(point).all() or (yaw is not None and not np.isfinite(yaw)):
            return dict(result, reason='nonfinite_input')
        if (s_hint is None) != (search_window_m is None):
            raise ValueError('s_hint and search_window_m must be supplied together')
        if s_hint is not None and (not np.isfinite(s_hint) or not np.isfinite(search_window_m) or search_window_m <= 0):
            raise ValueError('invalid projection search window')
        occupancy = self.raster.classify(point)
        result['inside_map'] = occupancy != 'outside_map'
        if occupancy != 'free':
            return dict(result, reason=occupancy)
        cell = self.raster.map_to_pixel(point).cell
        result['inside_track'] = bool(self.corridor[cell[1], cell[0]])
        if not result['inside_track']:
            return dict(result, reason='outside_track')
        candidates = self._candidates(point)
        result['candidate_count'] = len(candidates)
        if not candidates:
            return dict(result, reason='no_projection')
        best = candidates[0]
        if any(c['distance']-best['distance'] <= self.raster.resolution*.5
               and self.distance_s(c['s'], best['s']) > 1
               for c in candidates[1:]):
            return dict(result, reason='ambiguous_projection')
        if s_hint is not None and self.distance_s(float(s_hint)%self.curve.L if self.curve.closed else float(s_hint), best['s']) > search_window_m:
            return dict(result, reason='no_projection')
        checked = self._check_candidate(point,best,yaw)
        if not checked['valid']:
            return dict(result,reason=checked['reason'])
        # valid is a per-query numerical/geometry result, not a global artifact approval.
        return dict(result, valid=True, s_wrapped=best['s'], d=checked['d'],
                    projection_xy=checked['projection_xy'], psi_reference=checked['psi_reference'],
                    kappa=checked['kappa'],e_psi=checked['e_psi'],
                    inside_valid_domain=True, jacobian=checked['jacobian'])

    def to_cartesian(self, s, d, e_psi=None):
        if not np.isscalar(s) or not np.isscalar(d) or not np.isfinite(s) or not np.isfinite(d) or (e_psi is not None and not np.isfinite(e_psi)):
            return dict(xy=None, yaw=None, valid=False, reason='nonfinite_input', geometry_id=self.geometry_id)
        if not self.curve.closed and not 0<=s<=self.curve.L:
            return dict(xy=None,yaw=None,valid=False,reason='outside_reference',geometry_id=self.geometry_id)
        ref = self.reference(s)
        tangent = np.array(ref['tangent']); normal = np.array([-tangent[1],tangent[0]])
        xy = np.array(ref['xy'])+d*normal
        checked = self.to_frenet(xy)
        valid = checked['valid']
        reason = checked['reason']
        if 1-ref['kappa']*d < .2:
            valid, reason = False, 'singular_frenet'
        if valid and (self.distance_s(checked['s_wrapped'], ref['s_wrapped'])>1e-4
                      or abs(checked['d']-d)>1e-4):
            valid, reason = False, 'ambiguous_projection'
        return dict(checked, xy=xy.tolist(), yaw=None if e_psi is None else wrap(ref['psi']+e_psi),
                    s_wrapped=ref['s_wrapped'], d=float(d), valid=valid, reason=reason,
                    inside_valid_domain=bool(valid))


def load_geometry(directory, map_yaml, inspect=False):
    from .build_map import verify_artifacts
    root = Path(directory)
    verify_artifacts(root)
    manifest = json.loads((root/'manifest.json').read_text())
    # No Stage B artifact may be used by a planner even if its report is edited.
    if not inspect:
        raise ValueError('geometry not globally validated; use inspect=True for numerical inspection')
    raster = load_map(map_yaml)
    if any(raster.source_hashes[key] != manifest['source_hashes'][key] for key in ('yaml','image')):
        raise ValueError('geometry_mismatch: source map')
    data = json.loads((root/'reference_curve.json').read_text())
    curve = ReferenceCurve((data['knots'], data['coefficients'], data['degree']), data['start_u'],data.get('periodic',True))
    return TrackGeometry(curve, raster, manifest['geometry_id'], manifest['anchor']['projected_xy'])


def adapt_pose(xy, yaw, timestamp, offset=(0.,0.,0.), frame_id='map', expected_frame='map'):
    if frame_id != expected_frame:
        raise ValueError('frame_mismatch')
    values = np.asarray([*xy, yaw, timestamp, *offset], float)
    if values.shape != (7,) or not np.isfinite(values).all():
        raise ValueError('nonfinite_input')
    c, s = math.cos(yaw), math.sin(yaw)
    rotated = np.array([[c,-s],[s,c]]) @ np.array(offset[:2])
    return dict(xy=(np.array(xy)+rotated).tolist(), yaw=wrap(yaw+offset[2]),
                timestamp=float(timestamp), frame_id=frame_id)
