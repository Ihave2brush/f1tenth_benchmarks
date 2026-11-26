"""Minimal context and exact-value length cache for read-only Runtime Core V1.

Extracted unchanged from realtime_cost_features_v1; no observer hooks.
"""
from copy import deepcopy
from dataclasses import dataclass
import math
from types import MappingProxyType
import numpy as np
from .cost_evaluation_v1 import _policy

def _number(v):return isinstance(v,(int,float,np.integer,np.floating)) and not isinstance(v,(bool,np.bool_)) and math.isfinite(v)
def _tuple(v):
    if isinstance(v,(list,tuple,np.ndarray)):return tuple(_tuple(x) for x in v)
    if isinstance(v,dict):return tuple((k,_tuple(v[k])) for k in sorted(v))
    return v


def _length_key(plan,geometry_id):
    """Small exact geometry tuple, not SHA/canonical JSON or historical proof."""
    if not isinstance(plan,dict):return None
    candidates=[]
    for p in plan.get('paths',[]):
        sp=p.get('spline')
        candidates.append((p.get('candidate_id'),p.get('path_type'),p.get('output_representation'),
            p.get('reference_s_origin'),_tuple(sp),tuple((v.get('x'),v.get('y')) for v in p.get('points',[]))))
    return (geometry_id,plan.get('geometry_id'),plan.get('frame_id'),plan.get('plan_id'),plan.get('source_control_step'),
        plan.get('source_sim_time_s'),_tuple(plan.get('source_pose')),_tuple(plan.get('parameters')),_tuple(plan.get('generator_settings')),_tuple(plan.get('result',{}).get('parameters')),tuple(candidates))


@dataclass(frozen=True)
class RuntimeLengthCache:
    key: tuple
    entries: object
    method: str


def prepare_runtime_length_cache(plan,geometry_id,values,*,method):
    """Caller supplies previously verified values outside the control hot path.

    This prevents stale-plan reuse, but does not authenticate supplied lengths.
    An immutable geometry/version is a caller requirement.
    """
    if method not in ('cached_frenet_integral','cached_xy_polyline'):raise ValueError('explicit_length_definition_required')
    if not isinstance(values,dict) or any(not _number(v) or v<=0 for v in values.values()):raise ValueError('invalid_length_cache')
    key=_length_key(plan,geometry_id)
    if key is None:raise ValueError('source_plan_required')
    return RuntimeLengthCache(key,MappingProxyType(dict(values)),method)


class RuntimeCostContext:
    def __init__(self,geometry,*,risk_policy,tracking_options):
        # Existing policy validation does JSON copying here only, never per call.
        p=_policy(risk_policy)
        if p is None or set(p['components'])!={'risk'} or p['components']['risk']['mode']!='exposure_integral' or p.get('weights') is not None:
            raise ValueError('risk_components_only_policy_required')
        self.geometry=geometry;self.geometry_id=geometry.geometry_id
        self.risk_spec=MappingProxyType(p['components']['risk']);self.tracking_options=deepcopy(tracking_options)
        self.tracking_options.validate()
