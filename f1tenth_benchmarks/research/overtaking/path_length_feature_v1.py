"""Offline geometric length features only: no collision, ranking or control.

No source files are read. Timings are observations, not deterministic features.
The numerical Frenet integral uses an arclength-parametrized reference curve.
"""
from copy import deepcopy
import hashlib
import json
import math
from numbers import Real, Integral
from time import perf_counter

import numpy as np
from scipy.interpolate import BSpline
from numpy.polynomial.legendre import leggauss

from .plan_binding_v1 import _tree, build_plan_binding as binding_v1
from .plan_binding_v2 import build_plan_binding as binding_v2, content_digest
from .reference_path import reference_spline_valid

MAX_NODES = 100001


def _json_result(value):
    if value is None or isinstance(value,(str,bool)):return value
    if isinstance(value,Integral):return int(value)
    if isinstance(value,Real):return float(value) if math.isfinite(value) else None
    if isinstance(value,(list,tuple)):return [_json_result(v) for v in value]
    if isinstance(value,dict):return {str(k):_json_result(v) for k,v in value.items()}
    return None


def _number(value, name, positive=False):
    if isinstance(value,bool) or not isinstance(value,Real) or not math.isfinite(value) or (positive and value<=0):
        raise ValueError('invalid_'+name)
    return float(value)


def _verify_v2_in_memory(plan, metadata, supplied):
    """Replay the fixed V2 payload with supplied evidence; never reassert file trust.

    V2's public verifier reads manifests for pinned records. Reconstruct this
    version's small canonical payload to keep this API free of filesystem IO.
    Evidence is content-bound but its historical verification is a claim here.
    """
    base = binding_v2(plan,session_metadata=metadata)
    if supplied is None: return base, 'computed_here_not_producer_binding'
    if not isinstance(supplied,dict) or type(supplied.get('binding_version')) is not int or supplied['binding_version']!=2:
        raise ValueError('requires_plan_binding_v2')
    evidence=supplied.get('geometry_evidence')
    if not isinstance(evidence,dict) or type(evidence.get('geometry_artifacts_verified')) is not bool:
        raise ValueError('invalid_binding_geometry_evidence')
    for k in ('geometry_id','frame_id'):
        if evidence.get(k) is not None and evidence[k]!=base['source'][k]: raise ValueError('binding_geometry_mismatch')
    old=binding_v1(plan,session_metadata=metadata); payloads=[]
    for c, b, original in zip(plan['paths'],base['candidate_sha256'],old['candidate_sha256']):
        spec=c.get('spline');analytic=None
        if spec is not None:
            analytic=dict(spline=spec,reference_s_origin_present='reference_s_origin' in c,
                reference_s_origin=c.get('reference_s_origin'),origin_rule=base['origin_rule'],
                resolved_origin=b['resolved_s_origin'],spline_domain_m=b['spline_domain_m'])
        payloads.append(dict(candidate_id=c['candidate_id'],polyline_candidate_sha256=original['sha256'],
            output_representation_present='output_representation' in c,output_representation=c.get('output_representation'),
            analytic=analytic,path_type=b['path_type'],reference_only=b['reference_only'],
            reference_d=c.get('reference_d'),requested_target_d=c.get('requested_target_d'),
            footprint_sampled_valid=c.get('footprint_sampled_valid')))
    context=dict(source=base['source'],lookahead_length_m=base['lookahead_length_m'],geometry=evidence)
    expected=deepcopy(base);expected['geometry_evidence']=deepcopy(evidence)
    expected['geometry_artifacts_verified']=evidence['geometry_artifacts_verified']
    expected['plan_sha256']=content_digest('ordered_plan_analytic_v2',dict(
        context=context,polyline_plan_v1_sha256=old['plan_sha256'],candidates=payloads))
    for row,c in zip(expected['candidate_sha256'],payloads):
        row['sha256']=content_digest('candidate_analytic_v2',dict(context=context,candidate=c))
    if _tree(supplied)!=_tree(expected):raise ValueError('plan_binding_v2_mismatch')
    return expected, 'content_verified_against_supplied_record'


def _sample(curve,s):
    xy,tangent,heading,kappa=curve.sample(np.asarray(s,dtype=float))
    n=len(s);xy=np.asarray(xy,float);tangent=np.asarray(tangent,float)
    heading=np.asarray(heading,float);kappa=np.asarray(kappa,float)
    if xy.shape!=(n,2) or tangent.shape!=(n,2) or heading.shape!=(n,) or kappa.shape!=(n,):
        raise ValueError('invalid_geometry_sample_shape')
    if not all(np.isfinite(a).all() for a in (xy,tangent,heading,kappa)):
        raise ValueError('nonfinite_geometry_sample')
    if not np.allclose(np.linalg.norm(tangent,axis=1),1.,rtol=0,atol=1e-7):
        raise ValueError('geometry_tangent_not_unit')
    return xy,tangent,heading,kappa


def _positions(curve,spline,origin,q):
    xy,tangent,heading,kappa=_sample(curve,origin+q)
    d=np.asarray(spline(q),float)
    dp=np.zeros(len(q)) if spline.k==0 else np.asarray(spline(q,nu=1),float)
    jacobian=1-kappa*d
    if not np.isfinite([d,dp,jacobian]).all() or np.any(jacobian<=0):
        raise ValueError('invalid_frenet_chart')
    points=xy+d[:,None]*np.column_stack((-tangent[:,1],tangent[:,0]))
    if not np.isfinite(points).all():raise ValueError('nonfinite_xy')
    return points,heading+np.arctan2(dp,jacobian),d,np.hypot(jacobian,dp)


def _integration_edges(curve,spline,origin,length,panel):
    count=math.ceil(length/panel)
    if count>MAX_NODES//32:raise ValueError('integration_budget_exceeded')
    edges=list(np.linspace(0.,length,count+1))+[float(v) for v in spline.t if 0<v<length]
    # Split at serialized reference knot arclengths and the periodic seam.
    nodes=np.asarray(getattr(curve,'s_nodes',[0.,curve.L]),float)
    if nodes.ndim!=1 or not np.isfinite(nodes).all():raise ValueError('invalid_reference_arc_nodes')
    start=float(getattr(curve,'start_s',0.))
    if curve.closed:
        for lap in range(math.floor((origin+start)/curve.L)-1,math.ceil((origin+length+start)/curve.L)+1):
            edges.extend(float(v-start+lap*curve.L-origin) for v in nodes if 0<v-start+lap*curve.L-origin<length)
    else:edges.extend(float(v-origin) for v in nodes if 0<v-origin<length)
    edges=np.unique(edges)
    if len(edges)*32>MAX_NODES:raise ValueError('integration_budget_exceeded')
    return edges


def _integral(curve,spline,origin,edges,order):
    nodes,weights=leggauss(order)
    half=np.diff(edges)/2;mid=(edges[:-1]+edges[1:])/2
    q=(mid[:,None]+half[:,None]*nodes).ravel()
    _,_,_,metric=_positions(curve,spline,origin,q)
    measured=float(np.sum(half*(metric.reshape(-1,order)@weights)))
    if not math.isfinite(measured) or measured<=0:raise ValueError('nonfinite_or_zero_integral')
    return measured


def evaluate_path_length_features(plan, geometry, *, binding=None, session_metadata=None,
                                  sampling_steps_m=(.05,.02,.01), integration_panel_m=.25,
                                  geometry_curve_json=None):
    """Five-slot geometric features; no file IO or mutation of supplied objects.

    geometry_curve_json is optional raw serialized reference_curve.json bytes
    provided by the caller, checked against the bound file hash AND the loaded
    object's core curve serialization. Without it, only geometry ID/frame are
    matched; object-content verification stays false. Invalid plan bindings
    reject the batch; native rejected candidates keep null lengths.
    """
    started=perf_counter()
    candidates=plan.get('paths',[]) if isinstance(plan,dict) else []
    candidates=candidates if isinstance(candidates,list) else []
    result=dict(schema_version=1,method='offline_path_length_feature_v1',batch_status='rejected',reason=None,
        binding_status='unverified',plan_binding=None,geometry_object_curve_verified=False,
        geometry_artifacts_reverified=False,geometry_curve_fingerprint=None,
        source_artifact_evidence_claimed=False,paths=[],comparison={},elapsed_s=None,
        inspection_only=True,planning_allowed=False,parameters=None,
        limitations=['Geometric efficiency only; no safety or collision approval.',
            'Different endpoints and reference-only paths are not interchangeable goals.',
            'Quadrature difference and density change are estimates, not proven error bounds.',
            'Supplied binding equality is not authentication or historical producer evidence.'])
    for index,c in enumerate(candidates):
        c=c if isinstance(c,dict) else {}
        result['paths'].append(dict(candidate_id=c.get('candidate_id'),input_index=index,
            path_type=c.get('path_type','state_matched_trajectory'),native_sampled_valid=c.get('sampled_valid'),
            native_reason=c.get('reason'),native_footprint_sampled_valid=c.get('footprint_sampled_valid'),
            reference_only=c.get('path_type')=='reference_centerline',length_status='not_evaluated',reason=None,
            requested_target_d_m=c.get('requested_target_d'),endpoint_mode=c.get('endpoint_mode'),
            endpoint_target_error_m=None,elapsed_s=None,
            geometry_scope=None,origin_s_m=None,reference_interval_s_m=None,endpoint_d_m=None,
            start_xy=None,end_xy=None,start_position_error_m=None,start_heading_error_rad=None,
            xy_lengths=[],frenet_integral_length_m=None,quadrature_difference_estimate_m=None,
            integration_elapsed_s=None,eligible_for_control=False,inspection_only=True,planning_allowed=False))
    try:
        if not isinstance(sampling_steps_m,(list,tuple)) or not sampling_steps_m:
            raise ValueError('missing_sampling_steps')
        steps=[_number(x,'sampling_step',True) for x in sampling_steps_m]
        if len(set(steps))!=len(steps):raise ValueError('duplicate_sampling_steps')
        panel=_number(integration_panel_m,'integration_panel',True)
        result['parameters']=dict(sampling_steps_m=steps,integration_panel_m=panel,
            quadrature_orders=[16,32],quadrature_method='composite_gauss_legendre_split_at_spline_and_reference_knots',
            max_nodes=MAX_NODES,start_position_tolerance_m=1e-4,start_heading_tolerance_rad=1e-5)
        curve=geometry.curve;track_length=_number(curve.L,'reference_length',True)
        if type(curve.closed) is not bool:raise ValueError('missing_closed_flag')
        if not isinstance(getattr(geometry,'geometry_id',None),str):raise ValueError('missing_geometry_id')
        frame=getattr(geometry,'frame_id',None)
        if frame is None and callable(getattr(geometry,'reference',None)):
            ref=geometry.reference(0.);frame=ref.get('frame_id')
            if ref.get('geometry_id')!=geometry.geometry_id:raise ValueError('geometry_reference_id_mismatch')
        if not isinstance(frame,str) or not frame:raise ValueError('missing_geometry_frame')
        metadata=dict(geometry_id=geometry.geometry_id,frame_id=frame)
        if session_metadata is not None:
            if not isinstance(session_metadata,dict) or any(session_metadata.get(k)!=v for k,v in metadata.items()):
                raise ValueError('geometry_metadata_mismatch')
            metadata=session_metadata
        bound,status=_verify_v2_in_memory(plan,metadata,binding)
        result.update(plan_binding=bound,binding_status=status,
            source_artifact_evidence_claimed=bound['geometry_artifacts_verified'])
        length=bound['lookahead_length_m']
        if length is None:raise ValueError('analytic_length_required')
        if curve.closed and length>=track_length:raise ValueError('lookahead_at_least_one_lap')
        if any(not math.isfinite(length/s) or math.ceil(length/s)+1>MAX_NODES for s in steps):
            raise ValueError('sampling_budget_exceeded')
        actual=curve.to_dict() if callable(getattr(curve,'to_dict',None)) else None
        if actual is not None:
            core={k:actual.get(k) for k in ('knots','coefficients','degree','start_u','periodic','L_m','arclength_u','arclength_s_m','parameter_range','integration')}
            result['geometry_curve_fingerprint']=content_digest('loaded_reference_curve_core',core)
        if geometry_curve_json is not None:
            raw=geometry_curve_json.encode('utf-8') if isinstance(geometry_curve_json,str) else geometry_curve_json
            if not isinstance(raw,bytes):raise ValueError('invalid_geometry_curve_json')
            evidence=bound['geometry_evidence'];expected=evidence.get('file_hashes',{}).get('reference_curve.json')
            if expected is None or hashlib.sha256(raw).hexdigest()!=expected:
                raise ValueError('geometry_curve_artifact_hash_mismatch')
            serialized=json.loads(raw);_tree(serialized)
            if actual is None or _tree({k:serialized.get(k) for k in core})!=_tree(core):
                raise ValueError('loaded_geometry_curve_mismatch')
            result['geometry_object_curve_verified']=True
        else:result['limitations'].append('Loaded geometry content matched by ID/frame only; raw curve artifact not supplied.')
        result['batch_status']='evaluated'
    except (ValueError,TypeError,KeyError,AttributeError,OverflowError) as exc:
        result['reason']=str(exc)
        for row in result['paths']:row.update(length_status='rejected',reason=str(exc))
        result['elapsed_s']=perf_counter()-started
        return _json_result(result)

    for c,row,parent in zip(candidates,result['paths'],bound['candidate_sha256']):
        row_start=perf_counter()
        if c.get('sampled_valid') is not True:
            row.update(length_status='native_rejected',reason=c.get('reason') or 'native_candidate_rejected');continue
        try:
            if row['path_type'] not in ('state_matched_trajectory','reference_centerline'):
                raise ValueError('unsupported_path_type')
            spec=c.get('spline')
            if spec is None:raise ValueError('analytic_spline_required')
            spline=BSpline(spec['knots'],spec['coefficients'],spec['degree'],extrapolate=False)
            interior=[v for v in spline.t if 0<v<length]
            if any(interior.count(v)>spline.k for v in set(interior)):
                raise ValueError('discontinuous_spline_not_supported')
            origin=float(parent['resolved_s_origin'])
            if not curve.closed and (origin<0 or origin+length>track_length):raise ValueError('outside_open_reference')
            if row['reference_only']:
                if not reference_spline_valid(c,length):raise ValueError('invalid_reference_path_contract')
                if origin!=c['reference_s_origin']:raise ValueError('reference_origin_conflict')
            endpoints,yaws,d,_=_positions(curve,spline,origin,np.array([0.,length]))
            position_error=float(np.linalg.norm(endpoints[0]-np.asarray(plan['source_pose'][:2])))
            heading_error=float((yaws[0]-plan['source_pose'][2]+np.pi)%(2*np.pi)-np.pi)
            row.update(origin_s_m=origin,reference_interval_s_m=[origin,origin+length],endpoint_d_m=float(d[-1]),
                start_xy=endpoints[0].tolist(),end_xy=endpoints[1].tolist(),start_position_error_m=position_error,
                start_heading_error_rad=heading_error,
                geometry_scope='reference_geometry_only' if row['reference_only'] else 'state_matched_geometric_curve')
            if isinstance(c.get('requested_target_d'),Real) and not isinstance(c['requested_target_d'],bool) and math.isfinite(c['requested_target_d']):
                row['endpoint_target_error_m']=float(d[-1]-c['requested_target_d'])
            if not row['reference_only'] and (position_error>1e-4 or abs(heading_error)>1e-5):
                raise ValueError('source_pose_or_yaw_mismatch')
            t=perf_counter();edges=_integration_edges(curve,spline,origin,length,panel)
            low=_integral(curve,spline,origin,edges,16);high=_integral(curve,spline,origin,edges,32)
            integral_elapsed=perf_counter()-t
            samples=[]
            for step in steps:
                t=perf_counter();q=np.linspace(0.,length,max(2,math.ceil(length/step))+1)
                xy,_,_,_=_positions(curve,spline,origin,q)
                measured=float(np.linalg.norm(np.diff(xy,axis=0),axis=1).sum())
                if not math.isfinite(measured) or measured<=0:raise ValueError('nonfinite_or_zero_xy_length')
                samples.append(dict(requested_step_m=step,actual_step_m=float(q[1]-q[0]),sample_count=len(q),
                    xy_length_m=measured,difference_from_frenet_m=measured-high,
                    absolute_difference_from_frenet_m=abs(measured-high),density_change_m=None,
                    elapsed_s=perf_counter()-t))
            ordered=sorted(samples,key=lambda x:x['actual_step_m'],reverse=True)
            for prev,next_ in zip(ordered,ordered[1:]):next_['density_change_m']=next_['xy_length_m']-prev['xy_length_m']
            row.update(length_status='computed',xy_lengths=samples,frenet_integral_length_m=high,
                quadrature_difference_estimate_m=abs(high-low),integration_elapsed_s=integral_elapsed)
        except (ValueError,TypeError,KeyError,IndexError,OverflowError) as exc:
            row.update(length_status='rejected',reason=str(exc))
        row['elapsed_s']=perf_counter()-row_start
    computed=[r for r in result['paths'] if r['length_status']=='computed']
    same_interval=bool(computed) and all(r['reference_interval_s_m']==computed[0]['reference_interval_s_m'] for r in computed)
    same_start=bool(computed) and all(np.linalg.norm(np.asarray(r['start_xy'])-computed[0]['start_xy'])<=1e-4 for r in computed)
    same_end=bool(computed) and all(np.linalg.norm(np.asarray(r['end_xy'])-computed[0]['end_xy'])<=1e-4 for r in computed)
    result['comparison']=dict(common_frenet_interval_among_computed=same_interval,
        common_cartesian_start_among_computed=same_start,common_cartesian_end_among_computed=same_end,
        computed_candidate_count=len(computed),full_five_lengths_available=len(computed)==5,
        caution='Different endpoints/reference-only/native rejection preclude an automatic shortest-path choice.')
    result['elapsed_s']=perf_counter()-started
    return _json_result(result)
