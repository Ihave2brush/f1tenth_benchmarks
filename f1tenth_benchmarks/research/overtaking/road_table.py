"""Exact-s road columns with conservative lateral projection certificates.

No interpolation between CSV columns is authorized. A lateral interval is
accepted only when exact grid rays place it in the source free corridor and
polynomial bounds exclude competing projections. Uncertified columns fall back.
Caches live on the source geometry across requests; sources must be immutable
within one request. Changed source fingerprints invalidate the entire table.
"""
from collections import OrderedDict
from copy import deepcopy
import csv
import hashlib
import json
import math
from pathlib import Path
from time import perf_counter

import numpy as np

from ..core.geometry.frenet import TrackGeometry
from ..core.geometry.extraction import ReferenceCurve
from ..core.geometry.map_raster import MapRaster

POLICY = 'exact_s_bernstein_projection_v1'


def implementation_token(geometry):
    groups = ((geometry, ('reference', 'boundaries', 'to_cartesian', 'to_frenet', '_candidates',
                          '_check_candidate', 'distance_s')),
              (geometry.curve, ('sample', 'evaluate', 'integral', 'u_at_s', 'arc')),
              (geometry.raster, ('classify', 'map_to_pixel', 'map_to_local')))
    return tuple(getattr(getattr(obj, name), '__func__', getattr(obj, name))
                 for obj, names in groups for name in names)


def source_fingerprint(geometry):
    digest = hashlib.sha256()
    digest.update(json.dumps(dict(policy=POLICY, geometry_id=geometry.geometry_id,
        source_hashes=geometry.raster.source_hashes, closed=geometry.curve.closed,
        start_u=geometry.curve.start_u, start_s=geometry.curve.start_s,
        length=geometry.curve.L, degree=geometry.curve.tck[2], resolution=geometry.raster.resolution,
        origin=geometry.raster.origin.tolist()), sort_keys=True).encode())
    arrays = [*geometry.curve.tck[:2], geometry.curve.u_nodes, geometry.curve.s_nodes,
              geometry.raster.occupancy, geometry.raster.rotation, geometry.corridor,
              geometry.coefficients, geometry.lower, geometry.upper, geometry.widths,
              geometry.starts, geometry._seed_xy]
    for value in arrays:
        array = np.ascontiguousarray(value)
        digest.update(str((array.shape, array.dtype.str)).encode())
        digest.update(array.tobytes())
    return digest.hexdigest()


class RoadTable:
    def __init__(self, geometry, fingerprint=None, max_columns=32768, max_points=8192):
        if any(type(n) is not int or n < 1 for n in (max_columns, max_points)):
            raise ValueError('cache capacities must be positive integers')
        self.geometry = geometry
        self.implementation_token = implementation_token(geometry)
        self.fingerprint = fingerprint or source_fingerprint(geometry)
        self.max_columns, self.max_points = max_columns, max_points
        self.columns, self.points, self.references = OrderedDict(), OrderedDict(), OrderedDict()
        self.stats = dict(column_hits=0, column_misses=0, point_hits=0, point_misses=0,
                          fast_accepts=0, exact_fallbacks=0, certificate_valid=0,
                          shared_reference_hits=0, shared_reference_misses=0,
                          certificate_uncertain=0, invalid_exact_results=0,
                          evictions=0, invalidations=0, csv_seed_rows=0, csv_seed_wall_time_s=0.)
        # A custom geometry implementation has no certificate for these methods.
        self.standard = type(geometry) is TrackGeometry and all(getattr(getattr(geometry, name), '__func__', None)
                            is getattr(TrackGeometry, name)
                            for name in ('reference', 'boundaries', 'to_cartesian', 'to_frenet',
                                         '_candidates', '_check_candidate', 'distance_s'))
        self.standard = self.standard and type(geometry.curve) is ReferenceCurve and geometry.curve.tck[2] == 3 and all(
            getattr(getattr(geometry.curve, name), '__func__', None) is getattr(ReferenceCurve, name)
            for name in ('sample', 'evaluate', 'integral', 'u_at_s', 'arc'))
        self.standard = self.standard and type(geometry.raster) is MapRaster and all(
            getattr(getattr(geometry.raster, name), '__func__', None) is getattr(MapRaster, name)
            for name in ('classify', 'map_to_pixel', 'map_to_local'))
        # Convexity across adjacent spans needs a continuous first derivative.
        knots = geometry.curve.tck[0]
        _, multiplicities = np.unique(knots[(knots > 0) & (knots < 1)], return_counts=True)
        self.standard = self.standard and bool(np.all(multiplicities <= 2))
        if geometry.curve.closed:
            self.standard = self.standard and all(np.allclose(
                geometry.curve.evaluate(0., order), geometry.curve.evaluate(1., order),
                rtol=0., atol=1e-8) for order in (0, 1))
        self._prepare_polynomials()

    def _prepare_polynomials(self):
        coefficients = self.geometry.coefficients
        derivative = coefficients[:, 1:]*np.arange(1, 4)[None, :, None]
        second = coefficients[:, 2:]*np.array([2., 6.])[None, :, None]
        power = np.zeros((len(coefficients), 5))
        for axis in range(2):
            for i in range(3):
                for j in range(3):
                    power[:, i+j] += derivative[:, i, axis]*derivative[:, j, axis]
            for i in range(4):
                for j in range(2):
                    power[:, i+j] += coefficients[:, i, axis]*second[:, j, axis]
        # Power -> degree-four Bernstein coefficients: convex hull bounds.
        transform = np.zeros((5, 5))
        for k in range(5):
            for j in range(k+1):
                transform[k, j] = math.comb(k, j)/math.comb(4, j)
        self._convexity_base = power @ transform.T
        padded = np.zeros((len(coefficients), 5, 2))
        padded[:, :2] = second
        self._second_bernstein = np.einsum('kj,nja->nka', transform, padded)

    def key(self, s):
        value = float(s)
        return float(value % self.geometry.curve.L) if self.geometry.curve.closed else value

    def _put(self, cache, key, value, capacity):
        cache[key] = value
        cache.move_to_end(key)
        if len(cache) > capacity:
            cache.popitem(last=False)
            self.stats['evictions'] += 1

    def column(self, s, reference=None, boundary_query=None):
        key = self.key(s)
        if key in self.columns:
            self.stats['column_hits'] += 1
            self.columns.move_to_end(key)
            return self.columns[key]
        self.stats['column_misses'] += 1
        # Query the canonical coordinate used as the key, never round it.
        ref = reference if reference is not None and float(s) == key else self.geometry.reference(key)
        raw = (boundary_query or self.geometry.boundaries)(key)
        column = dict(s=key, reference=deepcopy(ref), boundary=deepcopy(raw),
                      state='uncertain', reason='projection_not_checked', intervals=None)
        self._put(self.columns, key, column, self.max_columns)
        return column

    def _certify_interval(self, reference, lo, hi):
        """Sufficient, not necessary, uniqueness test at one exact s.

        For p(d)=r(s)+d*n(s), the known source distance is |d|. Exclude
        spans whose exact curve boxes cannot compete within map_resolution/2.
        On the remaining connected arc, bound d/du[(r(u)-p) dot r'(u)]
        strictly positive with Bernstein coefficients for both endpoint p's.
        Continuity and the known source stationary point then give a unique
        competitive stationary point. Disconnected or weak bounds are uncertain.
        This certifies a d interval at fixed s, never an interval of s.
        """
        if not self.standard or not lo < hi:
            return False
        tangent = np.asarray(reference['tangent'])
        normal = np.array([-tangent[1], tangent[0]])
        endpoints = np.asarray(reference['xy'])+np.array([lo, hi])[:, None]*normal
        low, high = endpoints.min(axis=0), endpoints.max(axis=0)
        gap = np.maximum(np.maximum(self.geometry.lower-high, low-self.geometry.upper), 0.)
        radius = max(abs(lo), abs(hi))+self.geometry.raster.resolution*.5+1e-8
        selected = np.linalg.norm(gap, axis=1) <= radius
        indices = np.flatnonzero(selected)
        if not len(indices) or (self.geometry.curve.closed and len(indices) == len(selected)):
            return False
        if self.geometry.curve.closed:
            if np.count_nonzero(selected != np.roll(selected, 1)) != 2:
                return False
        elif indices[-1]-indices[0]+1 != len(indices):
            return False
        # The source point must lie in a selected span, including knot seams.
        source = np.asarray(reference['xy'])
        contains = np.all((self.geometry.lower[indices]-1e-8 <= source) &
                          (source <= self.geometry.upper[indices]+1e-8), axis=1)
        if not contains.any():
            return False
        base = self._convexity_base[indices]
        for point in endpoints:
            terms = self._second_bernstein[indices]*point[None, None, :]
            values = base-terms.sum(axis=2)
            scale = np.max(np.abs(base)+np.abs(terms).sum(axis=2))
            guard = max(1e-12, scale*1e-12)
            if not np.all(values > guard+1e-10*self.geometry.widths[indices, None]**2):
                return False
        return True

    def certify(self, column):
        if column['intervals'] is not None:
            return
        raw, ref = column['boundary'], column['reference']
        column['intervals'] = []
        if not raw['valid'] or raw.get('d_right') is None or raw.get('d_left') is None:
            column.update(state='checked_invalid', reason='invalid_boundary')
            return
        lo, hi = raw['d_right']+1e-8, raw['d_left']-1e-8
        # Stay strictly inside the local Jacobian threshold; boundary cases fall back.
        curvature = ref['kappa']
        if curvature > 0:
            hi = min(hi, (.8-1e-8)/curvature)
        elif curvature < 0:
            lo = max(lo, (.8-1e-8)/curvature)
        if self._certify_interval(ref, lo, hi):
            column.update(state='checked_valid', reason=None, intervals=[[float(lo), float(hi)]])
            self.stats['certificate_valid'] += 1
        else:
            column.update(state='uncertain', reason='projection_certificate_inconclusive')
            self.stats['certificate_uncertain'] += 1

    def classify(self, s, d):
        column = self.column(s)
        self.certify(column)
        if any(lo <= d <= hi for lo, hi in column['intervals']):
            return dict(state='checked_valid', reason=None, method=POLICY)
        key = (self.key(s), float(d))
        if key in self.points:
            checked = self.points[key]
            self.points.move_to_end(key)
            self.stats['point_hits'] += 1
            return dict(state='checked_valid' if checked['valid'] else 'checked_invalid',
                        reason=checked['reason'], method='cached_exact')
        self.stats['point_misses'] += 1
        return dict(state='uncertain', reason=column['reason'] or 'outside_certified_interval', method=POLICY)

    def confirm(self, s, d, result):
        # Heading does not enter to_cartesian validity; only geometric judgment is cached.
        if not self.standard:
            return
        checked = dict(valid=bool(result['valid']), reason=result['reason'])
        self._put(self.points, (self.key(s), float(d)), checked, self.max_points)
        if not checked['valid']:
            self.stats['invalid_exact_results'] += 1

    def seed_csv(self, directory):
        """Verify source hashes and every CSV row before adding exact columns."""
        started = perf_counter()
        root = Path(directory)
        manifest = json.loads((root/'manifest.json').read_text())
        if (manifest['geometry_id'] != self.geometry.geometry_id or any(
                manifest['source_hashes'].get(key) != value
                for key, value in self.geometry.raster.source_hashes.items())):
            raise ValueError('road table source version mismatch')
        for name in ('centerline.csv', 'boundaries.csv', 'reference_curve.json'):
            if hashlib.sha256((root/name).read_bytes()).hexdigest() != manifest['file_hashes'][name]:
                raise ValueError('road table CSV source hash mismatch')
        with (root/'boundaries.csv').open(newline='') as stream:
            for row in csv.DictReader(stream):
                s = float(row['s_m'])
                column = self.column(s)
                exact = column['boundary']
                if row['valid'] != str(int(exact['valid'])) or (row['reason'] or None) != exact['reason']:
                    raise ValueError('road table CSV status mismatch')
                for name in ('d_left', 'd_right'):
                    actual = None if row[name+'_m'] == '' else float(row[name+'_m'])
                    expected = exact[name]
                    if (actual is None) != (expected is None) or (actual is not None and
                            (not np.isfinite(actual) or abs(actual-expected) > 1e-7)):
                        raise ValueError('road table CSV boundary mismatch')
                self.certify(column)
                self.stats['csv_seed_rows'] += 1
        self.stats['csv_seed_wall_time_s'] += perf_counter()-started

    def summary(self):
        return dict(self.stats, column_entries=len(self.columns), point_entries=len(self.points),
                    reference_entries=len(self.references),
                    fingerprint=self.fingerprint, policy=POLICY,
                    max_columns=self.max_columns, max_points=self.max_points,
                    scope='source_geometry_across_generations', s_interpolation=False,
                    continuous_s_verified=False, persistent_on_disk=False)


def get_road_table(geometry, csv_directory=None):
    fingerprint = source_fingerprint(geometry)
    table = getattr(geometry, '_candidate_road_table', None)
    if (table is None or table.fingerprint != fingerprint or
            table.implementation_token != implementation_token(geometry)):
        invalidations = 0 if table is None else table.stats['invalidations']+1
        table = RoadTable(geometry, fingerprint)
        table.stats['invalidations'] = invalidations
        if csv_directory is not None:
            table.seed_csv(csv_directory)
        geometry._candidate_road_table = table
    elif csv_directory is not None and not table.stats['csv_seed_rows']:
        table.seed_csv(csv_directory)
    return table
