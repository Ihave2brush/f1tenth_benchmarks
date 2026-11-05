"""Request-local exact query reuse; no interpolation or weakened validation.

The isolated shallow copy binds the existing geometry methods to a context
whose reference/boundary queries are memoized. Projection and occupancy checks
still run for every new Cartesian point. No input is rounded or quantized.
The source curve/raster must remain immutable for the duration of a request.
"""
from collections import OrderedDict
from copy import copy, deepcopy

import numpy as np

from ..core.geometry.frenet import TrackGeometry


class ExactQueryContext:
    def __init__(self, geometry, max_entries=65536):
        if type(max_entries) is not int or max_entries < 1:
            raise ValueError('max_entries must be a positive integer')
        self.geometry = copy(geometry)
        self.max_entries = max_entries
        self._references = OrderedDict()
        self._boundaries = OrderedDict()
        self.stats = dict(reference_hits=0, reference_misses=0,
                          boundary_hits=0, boundary_misses=0,
                          reference_prefetched=0, evictions=0)
        original_reference = self.geometry.reference
        original_boundaries = self.geometry.boundaries
        self._original_reference = original_reference
        # Batching is valid only for the standard reference implementation.
        self._can_prefetch = (getattr(original_reference, '__func__', None)
                              is TrackGeometry.reference)
        self.geometry.reference = lambda s: self._query(
            s, self._references, original_reference, 'reference')
        self.geometry.boundaries = lambda s: self._query(
            s, self._boundaries, original_boundaries, 'boundary')

    def _put(self, cache, key, value):
        cache[key] = deepcopy(value)
        cache.move_to_end(key)
        if len(cache) > self.max_entries:
            cache.popitem(last=False)
            self.stats['evictions'] += 1

    def _query(self, s, cache, function, kind):
        # Preserve original invalid-input behavior rather than caching errors.
        if not np.isscalar(s) or not np.isfinite(s):
            return function(s)
        key = float(s)
        if key in cache:
            self.stats[kind+'_hits'] += 1
            cache.move_to_end(key)
            return deepcopy(cache[key])
        self.stats[kind+'_misses'] += 1
        result = function(s)
        self._put(cache, key, result)
        return deepcopy(result)

    def prefetch_reference(self, source_s):
        """Vectorize spline evaluation; retain exact scalar arc-length inversion."""
        if not self._can_prefetch:
            return
        keys = list(dict.fromkeys(float(s) for s in source_s))
        keys = [s for s in keys if s not in self._references]
        if not keys:
            return
        curve = self.geometry.curve
        xy, tangent, psi, kappa = curve.sample(np.asarray(keys))
        for i, s in enumerate(keys):
            result = dict(geometry_id=self.geometry.geometry_id, frame_id='map',
                          s_wrapped=float(s % curve.L) if curve.closed else s,
                          xy=xy[i].tolist(), tangent=tangent[i].tolist(),
                          psi=float(psi[i]), kappa=float(kappa[i]))
            self._put(self._references, s, result)
        self.stats['reference_prefetched'] += len(keys)

    def summary(self):
        return dict(self.stats, reference_entries=len(self._references),
                    boundary_entries=len(self._boundaries),
                    max_entries_per_cache=self.max_entries,
                    scope='single_generation', key_policy='exact_float_no_quantization')

    def close(self):
        """Break request-local bound-method cycles without clearing shared tables."""
        for name in ('reference', 'boundaries', 'candidate_cartesian_many', 'candidate_boundaries_many',
                     '_candidate_legacy_grid_indices'):
            self.geometry.__dict__.pop(name, None)
        self._references.clear()
        self._boundaries.clear()
        if hasattr(self, '_candidate_grid'):
            self._candidate_grid = None


class FastQueryContext(ExactQueryContext):
    """Frenet column checks plus batched forward XY; exact fallback when unsure."""
    def __init__(self, geometry):
        from .road_table import get_road_table
        self.table = get_road_table(geometry)
        self._start_stats = dict(self.table.stats)
        self._candidate_grid = None
        super().__init__(geometry)
        previous_reference = self.geometry.reference
        def reference(s):
            key = float(s)
            if key in self.table.references:
                self.table.stats['shared_reference_hits'] += 1
                self.table.references.move_to_end(key)
                return deepcopy(self.table.references[key])
            self.table.stats['shared_reference_misses'] += 1
            result = previous_reference(s)
            self.table._put(self.table.references, key, deepcopy(result), 2*self.table.max_columns)
            return result
        self.geometry.reference = reference
        exact_boundaries = self.geometry.boundaries
        def boundaries(s):
            reference = self.geometry.reference(s)
            column = self.table.column(s, reference, exact_boundaries)
            return deepcopy(column['boundary'])
        self.geometry.boundaries = boundaries
        self.geometry.candidate_cartesian_many = self.cartesian_many
        self.geometry.candidate_boundaries_many = self.boundaries_many

    def _prepare_grid(self, source_s):
        key = tuple(source_s)
        if self._candidate_grid is None or self._candidate_grid[0] != key:
            refs = [self.geometry.reference(s) for s in source_s]
            raw = [self.geometry.boundaries(s) for s in source_s]
            tangent = np.asarray([ref['tangent'] for ref in refs])
            normals = np.column_stack((-tangent[:, 1], tangent[:, 0]))
            self._candidate_grid = (key, refs, raw, np.asarray([r['xy'] for r in refs]), normals)
        return self._candidate_grid

    def boundaries_many(self, source_s):
        # Internal immutable query snapshots, independent of mutable solver bounds.
        return self._prepare_grid(source_s)[2]

    def prefetch_reference(self, source_s):
        keys = list(dict.fromkeys(float(s) for s in source_s))
        missing = []
        for key in keys:
            if key in self.table.references:
                self.table.stats['shared_reference_hits'] += 1
                self._put(self._references, key, self.table.references[key])
            else:
                missing.append(key)
        super().prefetch_reference(missing)
        for key in missing:
            if key in self._references:
                self.table.stats['shared_reference_misses'] += 1
                self.table._put(self.table.references, key, deepcopy(self._references[key]),
                                2*self.table.max_columns)

    def cartesian_many(self, source_s, lateral, headings):
        _, refs, _, base_xy, normals = self._prepare_grid(source_s)
        xy = base_xy+np.asarray(lateral)[:, None]*normals
        from ..core.geometry.frenet import wrap
        results = []
        for i, (s, d, heading) in enumerate(zip(source_s, lateral, headings)):
            state = self.table.classify(s, d)
            if state['state'] == 'uncertain':
                exact = self.geometry.to_cartesian(float(s), float(d), e_psi=float(heading))
                self.table.confirm(s, d, exact)
                self.table.stats['exact_fallbacks'] += 1
                results.append(exact)
            else:
                valid = state['state'] == 'checked_valid'
                if state['method'] != 'cached_exact':
                    self.table.stats['fast_accepts'] += 1
                results.append(dict(xy=xy[i].tolist(), yaw=wrap(refs[i]['psi']+float(heading)),
                                    valid=valid, reason=state['reason'],
                                    geometry_id=self.geometry.geometry_id))
        return results

    def summary(self):
        result = super().summary()
        result['road_table'] = self.table.summary()
        result['road_table_request'] = {key: value-self._start_stats[key]
                                       for key, value in self.table.stats.items()}
        return result


class FixedLookupQueryContext(FastQueryContext):
    """Fixed road anchors accelerate exact reference evaluation at old sites."""
    def __init__(self,geometry):
        super().__init__(geometry)
        from .fixed_arc_lookup import get_arc_lookup
        self.lookup=get_arc_lookup(self.table)
        self._lookup_start=dict(self.lookup.stats)
        self._lookup_enabled=self.table.standard and self._can_prefetch

    def prepare_window(self,start,length):
        if self._lookup_enabled:self.lookup.prepare_window(start,length)

    def prefetch_reference(self,source_s):
        if not self._lookup_enabled:
            return super().prefetch_reference(source_s)
        keys=list(dict.fromkeys(float(s) for s in source_s))
        missing=[]
        for key in keys:
            if key in self.table.references:
                self.table.stats['shared_reference_hits']+=1
                self._put(self._references,key,self.table.references[key])
            else:missing.append(key)
        if not missing:return
        xy,tangent,psi,kappa=self.lookup.sample(missing)
        curve=self.geometry.curve
        for i,key in enumerate(missing):
            result=dict(geometry_id=self.geometry.geometry_id,frame_id='map',
                s_wrapped=float(key%curve.L) if curve.closed else key,
                xy=xy[i].tolist(),tangent=tangent[i].tolist(),psi=float(psi[i]),kappa=float(kappa[i]))
            self._put(self._references,key,result)
            self.table._put(self.table.references,key,deepcopy(result),2*self.table.max_columns)
        self.stats['reference_prefetched']+=len(missing)
        self.table.stats['shared_reference_misses']+=len(missing)

    def summary(self):
        result=super().summary()
        result['fixed_arc_lookup']=self.lookup.summary()
        result['fixed_arc_lookup']['enabled']=self._lookup_enabled
        result['fixed_arc_lookup_request']={k:v-self._lookup_start[k]
                                          for k,v in self.lookup.stats.items()}
        return result
