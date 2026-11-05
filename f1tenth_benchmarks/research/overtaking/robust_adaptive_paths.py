"""V5.2: multistart constrained splines with explicit bounded terminal preference.

V5.1 and all earlier generators remain unchanged. Defaults enforce the original
fixed target; endpoint_tolerance_m > 0 explicitly enables bounded terminal
preference only after all fixed-target starts fail independent validation.
Every solver attempt records residuals and status. No unchecked solver output
is accepted. Certification remains sampled geometry + output-polyline points,
not a continuous spline, vehicle footprint or dynamic safety claim.
"""
import argparse
import json
import math
from time import perf_counter

import numpy as np
from scipy.optimize import minimize

from ..core.geometry import load_geometry
from .candidate_validation import MAX_SAMPLES
from .width_fraction_paths import width_fraction_spline
from .constrained_adaptive_paths import (
    _basis, _positive, _reference_data, evaluate_path_smoothness,
    query_road_sections, validate_path_segments,
)

SEEDS = ('road_fraction', 'hermite', 'road_center')
GROUPS = ('right_margin', 'left_margin', 'positive_slope', 'negative_slope',
          'jacobian', 'positive_curvature', 'negative_curvature')


def direct_hermite_coefficients(length, d0, slope0, target):
    """Cubic Hermite in a degree-five Bernstein basis, no optimizer.

    Same smoothstep family as the reference when slope0=0, extended to match
    the actual start yaw. End slope is zero. No extra curvature boundary is
    invented. The legacy dense verifier accepts or rejects the entire curve.
    """
    powers = [d0, length*slope0, 3*(target-d0)-2*length*slope0,
              2*(d0-target)+length*slope0]
    return np.array([sum(powers[j]*math.comb(i, j)/math.comb(5, j)
                         for j in range(min(i, 3)+1)) for i in range(6)])


def direct_constraint_screen(basis, c, dense, sections, k, kp, max_slope, max_curvature):
    """Reject known constraint violations before expensive geometry checks.

    Uses the same dense grid and thresholds as _validate. Passing this screen
    never accepts a candidate: the full legacy verifier still runs afterward.
    """
    d, p, q = [basis(dense, nu=i)@c for i in range(3)]
    J, curvature = evaluate_path_smoothness(d, p, q, k, kp)
    lo = np.array([section['d_min'] for section in sections])
    hi = np.array([section['d_max'] for section in sections])
    margin = (d >= lo-1e-8) & (d <= hi+1e-8)
    smooth = ((np.abs(p) <= max_slope+1e-7) & (np.abs(curvature) <= max_curvature+1e-7)
              & (J >= .2-1e-7) & np.isfinite(d+p+q+J+curvature))
    bad = np.flatnonzero(~(margin & smooth))
    if not len(bad):
        return None
    index = int(bad[0])
    return dict(first_invalid_sample=index, reason='insufficient_lateral_margin' if not margin[index]
                else 'smoothness_limit', stage='dense_constraint_screen',
                min_margin_slack_m=float(np.minimum(d-lo, hi-d).min()),
                max_abs_curvature_inv_m=float(np.abs(curvature).max()))


def _finite_number(value):
    value = float(value)
    return value if math.isfinite(value) else None


def curvature_jacobian(B, D1, D2, c, k, kp):
    """Analytic constraint derivatives; same curvature formula as V5.1."""
    d, p, q = B@c, D1@c, D2@c
    J = 1-k*d
    W = J*J+p*p
    N = J*(k*J+q)+p*(kp*d+2*k*p)
    raw = W**1.5
    denominator = np.maximum(raw, 1e-15)
    nd, np_, nq = -2*k*k*J-k*q+kp*p, kp*d+4*k*p, J
    active = raw > 1e-15
    factor = np.zeros_like(W)
    factor[active] = 3*N[active]/W[active]**2.5
    a = nd/denominator+factor*k*J
    b = np_/denominator-factor*p
    z = nq/denominator
    return a[:,None]*B+b[:,None]*D1+z[:,None]*D2


def fit_multistart(basis, distances, bounds, preferred, d0, slope0, target,
                   k, kp, max_slope, max_curvature, endpoint_tolerance_m=0.,
                   max_iterations=150):
    """Run all three starts under identical constraints; return diagnostic fits.

    Linear endpoint projection changes guesses, never road/smoothness limits.
    Terminal displacement is confined to the intersection of raw road bounds
    and the explicitly allowed target +/- tolerance interval.
    """
    matrices = [basis(distances, nu=i) for i in range(4)]
    B,D1,D2,D3 = matrices
    lo = np.array([s['d_min']for s in bounds]); hi = np.array([s['d_max']for s in bounds])
    end_lo = max(lo[-1], target-endpoint_tolerance_m)
    end_hi = min(hi[-1], target+endpoint_tolerance_m)
    fixed = endpoint_tolerance_m == 0
    if fixed:
        E = np.stack([basis(0),basis(0,nu=1),basis(distances[-1]),basis(distances[-1],nu=1)])
        rhs = np.array([d0,slope0,target,0.])
    else:
        E = np.stack([basis(0),basis(0,nu=1),basis(distances[-1],nu=1)])
        rhs = np.array([d0,slope0,0.])
    endpoint_row = basis(distances[-1])
    weights = [(B,preferred,1.),(D2,np.zeros(len(distances)),2.),(D3,np.zeros(len(distances)),.2)]
    H = sum(w*A.T@A/len(distances)for A,_,w in weights)
    linear = sum(w*A.T@y/len(distances)for A,y,w in weights)
    # Explicit terminal preference in bounded mode; constraints stay hard.
    if not fixed:
        H = H+10*np.outer(endpoint_row,endpoint_row)
        linear = linear+10*target*endpoint_row
    def objective(c):return float(c@H@c-2*linear@c)
    def restrictions(c):
        d,p,q=B@c,D1@c,D2@c
        J,curvature=evaluate_path_smoothness(d,p,q,k,kp)
        values=np.r_[d-lo,hi-d,max_slope-p,max_slope+p,J-.2,
                     max_curvature-curvature,max_curvature+curvature]
        if not fixed:
            values=np.r_[values,endpoint_row@c-end_lo,end_hi-endpoint_row@c]
        return values
    def restriction_jac(c):
        C=curvature_jacobian(B,D1,D2,c,k,kp)
        values=np.vstack([B,-B,-D1,D1,-k[:,None]*B,-C,C])
        return values if fixed else np.vstack([values,endpoint_row,-endpoint_row])
    u=distances/distances[-1]
    hermite=(2*u**3-3*u**2+1)*d0+(u**3-2*u**2+u)*distances[-1]*slope0+(-2*u**3+3*u**2)*target
    guesses=dict(road_fraction=preferred,hermite=hermite,road_center=(lo+hi)/2)
    outputs=[]
    for name in SEEDS:
        c=np.linalg.lstsq(B,guesses[name],rcond=None)[0]
        if not fixed:
            # Choose an in-range initial endpoint, then project start and slopes.
            initial_end=float(np.clip(endpoint_row@c,end_lo,end_hi))
            initial_E=np.vstack([E,endpoint_row]);initial_rhs=np.r_[rhs,initial_end]
        else:initial_E,initial_rhs=E,rhs
        c=c+initial_E.T@np.linalg.solve(initial_E@initial_E.T,initial_rhs-initial_E@c)
        initial_min=_finite_number(np.min(restrictions(c)))
        result=minimize(objective,c,jac=lambda c:2*(H@c-linear),method='SLSQP',
            constraints=[dict(type='eq',fun=lambda c:E@c-rhs,jac=lambda c:E),
                         dict(type='ineq',fun=restrictions,jac=restriction_jac)],
            options=dict(maxiter=max_iterations,ftol=1e-9))
        finite=bool(np.isfinite(result.x).all())
        eq=E@result.x-rhs
        inequality=restrictions(result.x)
        eq_error=float(np.max(np.abs(eq)))
        min_ineq=float(np.min(inequality))
        objective_value=_finite_number(objective(result.x))
        accepted=bool(result.success and finite and np.isfinite(eq_error)and np.isfinite(min_ineq)
                      and objective_value is not None and eq_error<1e-6 and min_ineq>=-1e-7)
        grouped={name:_finite_number(np.min(values))for name,values in
                 zip(GROUPS,np.split(inequality[:7*len(distances)],7))}
        if not fixed:grouped['endpoint_interval']=_finite_number(np.min(inequality[-2:]))
        diagnostics=dict(seed=name,mode='fixed'if fixed else'bounded_preference',
            solver_success=bool(result.success),status=int(result.status),message=str(result.message),
            iterations=int(result.nit),equality_error=_finite_number(eq_error),
            equality_residuals=[_finite_number(v)for v in eq],min_inequality=_finite_number(min_ineq),
            constraint_minima=grouped,initial_min_inequality=initial_min,
            initial_equality_error=_finite_number(np.max(np.abs(E@c-rhs))),
            solver_constraints_accepted=accepted,objective=objective_value,
            terminal_deviation_m=_finite_number(endpoint_row@result.x-target),
            target_interval_m=[float(end_lo),float(end_hi)])
        outputs.append((result.x,diagnostics))
    return outputs


def _validate(geometry,basis,c,dense,sections,k,kp,target,endpoint_tolerance_m,
              d0,slope0,max_slope,max_curvature):
    """Exact geometry checks, independent of solver success and solver arrays."""
    d,p,q=[basis(dense,nu=i)@c for i in range(3)]
    J,curvature=evaluate_path_smoothness(d,p,q,k,kp)
    batch = getattr(geometry, 'candidate_cartesian_many', None)
    converted = None
    if batch is not None:
        headings = [math.atan2(float(slope), float(jacobian)) for slope, jacobian in zip(p, J)]
        converted = batch([section['s'] for section in sections], d, headings)
    boundary_batch = getattr(geometry, 'candidate_boundaries_many', None)
    raw_sections = (None if boundary_batch is None else
                    boundary_batch([section['s'] for section in sections]))
    points=[]
    for i,(distance,lateral)in enumerate(zip(dense,d)):
        section=sections[i]
        # Use the exact query interface, not mutable solver bound arrays.
        raw=geometry.boundaries(section['s']) if raw_sections is None else raw_sections[i]
        right,left=raw.get('d_right'),raw.get('d_left')
        boundary_valid=bool(raw['valid']and right is not None and left is not None
            and np.isfinite([right,left]).all()and right+section['margin']<left-section['margin'])
        margin_valid=bool(boundary_valid and raw['d_right']+section['margin']-1e-8<=lateral<=raw['d_left']-section['margin']+1e-8)
        heading=math.atan2(float(p[i]),float(J[i]))
        exact=(geometry.to_cartesian(section['s'],float(lateral),e_psi=heading)
               if converted is None else converted[i])
        if exact.get('geometry_id')!=geometry.geometry_id:raise ValueError('Exact geometry version mismatch')
        xy=exact.get('xy');finite=xy is not None and np.shape(xy)==(2,)and np.isfinite(xy).all()
        geometric=bool(exact['valid']and finite)
        smooth=bool(abs(p[i])<=max_slope+1e-7 and abs(curvature[i])<=max_curvature+1e-7 and J[i]>=.2-1e-7)
        reason=None if geometric and boundary_valid and margin_valid and smooth else (
            'invalid_boundary'if not boundary_valid else exact.get('reason')or
            ('invalid_coordinates'if not finite else 'insufficient_lateral_margin'if not margin_valid else 'smoothness_limit'))
        points.append(dict(sample_index=i,t=float(distance/dense[-1]),s=section['s'],s_progress=section['s_progress'],
            d=float(lateral),d_prime=float(p[i]),curvature_inv_m=float(curvature[i]),
            x=float(xy[0])if finite else None,y=float(xy[1])if finite else None,yaw=exact.get('yaw'),
            boundary_valid=boundary_valid,margin_valid=margin_valid,geometry_valid=geometric,
            valid=reason is None,reason=reason,
            lateral_margin_m=float(min(lateral-raw['d_right'],raw['d_left']-lateral))if boundary_valid else None))
    endpoint_error=abs(d[-1]-target)
    initial_error=max(abs(d[0]-d0),abs(p[0]-slope0),abs(p[-1]))
    if endpoint_error>endpoint_tolerance_m+1e-7:
        points[-1].update(valid=False,reason='endpoint_deviation_exceeded')
    if initial_error>1e-6:points[0].update(valid=False,reason='endpoint_equality_error')
    invalid=next((i for i,v in enumerate(points)if not v['valid']),None)
    segment=None
    if invalid is None:
        segment=validate_path_segments(geometry.raster,[[v['x'],v['y']]for v in points])
        if not segment['valid']:
            invalid=segment['segment_index']+1
            points[invalid].update(valid=False,reason=segment['reason'])
    legacy_indices = getattr(geometry, '_candidate_legacy_grid_indices', None)
    if invalid is None and legacy_indices is not None:
        # A denser union polyline can bend away from a blocked legacy chord.
        # Preserve the legacy chord checks as well as all its sampled sites.
        legacy_segment = validate_path_segments(geometry.raster,
            [[points[i]['x'], points[i]['y']] for i in legacy_indices])
        if not legacy_segment['valid']:
            invalid = int(legacy_indices[legacy_segment['segment_index']+1])
            segment = dict(legacy_segment, sampling_policy='legacy_grid_guard')
            points[invalid].update(valid=False, reason=legacy_segment['reason'])
    return points,invalid,segment


def generate_robust_paths(geometry,x,y,yaw=None,lookahead_m=3.,safety_margin_m=.2,
                          sample_step_m=.25,max_slope=1.,max_curvature=None,
                          endpoint_tolerance_m=0.,max_refinements=2,max_iterations=150,
                          query_mode='exact', generation_mode='optimized', validation_step_m=None):
    generation_started = perf_counter()
    if generation_mode not in ('optimized', 'direct_only', 'direct_then_repair',
                               'width_fraction_only', 'width_fraction_then_repair'):
        raise ValueError('invalid generation_mode')
    width_fraction_mode = generation_mode.startswith('width_fraction_')
    direct_only_mode = generation_mode in ('direct_only', 'width_fraction_only')
    if query_mode not in ('exact', 'reused_exact', 'fast_with_fallback', 'fixed_grid_with_fallback', 'fixed_grid_lookup'):
        raise ValueError('invalid query_mode')
    context = None
    if query_mode != 'exact':
        from .geometry_queries import ExactQueryContext, FastQueryContext, FixedLookupQueryContext
        context_type = (FixedLookupQueryContext if query_mode == 'fixed_grid_lookup' else
                        FastQueryContext if query_mode in ('fast_with_fallback', 'fixed_grid_with_fallback')
                        else ExactQueryContext)
        context = context_type(geometry)
        geometry = context.geometry
    length=_positive(lookahead_m,'lookahead_m');step=_positive(sample_step_m,'sample_step_m')
    max_slope=_positive(max_slope,'max_slope')
    margin=float(safety_margin_m);tolerance=float(endpoint_tolerance_m)
    if not math.isfinite(margin)or margin<0:raise ValueError('invalid safety_margin_m')
    if not math.isfinite(tolerance)or tolerance<0:raise ValueError('invalid endpoint_tolerance_m')
    for value,name,maximum in [(max_refinements,'max_refinements',5),(max_iterations,'max_iterations',1000)]:
        if isinstance(value,bool)or not isinstance(value,int)or value<(0 if name=='max_refinements'else 1)or value>maximum:
            raise ValueError('invalid '+name)
    if max_curvature is None:
        from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
        vehicle=load_parameter_file('vehicle_params')
        max_curvature=math.tan(vehicle.max_steer)/vehicle.wheelbase
    max_curvature=_positive(max_curvature,'max_curvature')
    # Explicit research override: coarsening changes the sampled acceptance policy.
    validation_step = min(step, geometry.raster.resolution/2, .05)
    if validation_step_m is not None:
        validation_step = _positive(validation_step_m, 'validation_step_m')
        if validation_step > .05 or query_mode in ('fixed_grid_with_fallback', 'fixed_grid_lookup'):
            raise ValueError('validation_step_m requires <= .05 m and non-fixed-grid mode')
    ratio=length/validation_step
    if not math.isfinite(ratio)or ratio>MAX_SAMPLES-1:raise ValueError('validation sampling budget exceeded')
    start=geometry.to_frenet([x,y],yaw=yaw)
    if not start['valid']:raise ValueError('Invalid start: '+str(start.get('reason')))
    s0,d0=float(start['s_wrapped']),float(start['d'])
    if geometry.curve.closed and length>=geometry.curve.L:raise ValueError('lookahead must be shorter than full lap')
    if not geometry.curve.closed and s0+length>geometry.curve.L:raise ValueError('lookahead exceeds open reference')
    epsi=0. if yaw is None else float(start['e_psi'])
    if abs(epsi)>=math.pi/2-1e-3:raise ValueError('initial heading must point forward')
    slope0=(1-start['kappa']*d0)*math.tan(epsi)
    dense=np.linspace(0,length,math.ceil(ratio)+1)
    source_s = s0+dense
    grid_summary = None
    legacy_constraint_indices = None
    if query_mode == 'fixed_grid_with_fallback':
        from .fixed_grid import sample_window
        legacy_dense, legacy_source = dense, source_s
        progress_s, fixed_source, grid_summary = sample_window(
            context.table, s0, length, length/math.ceil(ratio))
        fixed_dense = progress_s-s0
        fixed_dense[0], fixed_dense[-1] = 0., length
        # Fixed-site certificates do not certify the gaps in s. Keep every
        # previous validation site until a continuous interval policy exists.
        sites = {float(d): float(s % geometry.curve.L if geometry.curve.closed else s)
                 for d, s in zip(legacy_dense, legacy_source)}
        for distance, source in zip(fixed_dense, fixed_source):
            sites.setdefault(float(distance), float(source))
        dense = np.array(sorted(sites))
        source_s = np.array([sites[float(d)] for d in dense])
        progress_s = s0+dense
        legacy_indices = np.searchsorted(dense, legacy_dense)
        geometry._candidate_legacy_grid_indices = legacy_indices
        legacy_constraint_indices = legacy_indices[np.unique(np.linspace(
            0, len(legacy_dense)-1, min(161, len(legacy_dense))).astype(int))]
        grid_summary.update(fixed_site_count=grid_summary['actual_site_count'],
            legacy_guard_site_count=len(legacy_dense), actual_site_count=len(dense),
            max_gap_m=float(np.max(np.diff(dense))), legacy_sampling_guard=True,
            interval_reuse_certified=False)
        if len(dense) > MAX_SAMPLES:
            raise ValueError('validation sampling budget exceeded')
    if query_mode == 'fixed_grid_lookup':
        context.prepare_window(s0, length)
    if context is not None:
        wrapped = source_s % geometry.curve.L if geometry.curve.closed else source_s
        low, high = source_s-.001, source_s+.001
        if not geometry.curve.closed:
            low, high = np.maximum(0, low), np.minimum(geometry.curve.L, high)
        context.prefetch_reference(np.r_[source_s, wrapped, low, high])
    sections=query_road_sections(geometry,source_s,margin)
    if grid_summary is not None:
        for section, progress in zip(sections, progress_s):
            section['s_progress'] = float(progress)
    for section in sections:section['margin']=margin
    first,last=sections[0],sections[-1]
    if not first['boundary_valid']or not first['d_min']<=d0<=first['d_max']:
        raise ValueError('Invalid or insufficient boundary at start')
    fraction0=(d0-first['d_min'])/(first['d_max']-first['d_min'])
    basis=_basis(length,min(20,max(6,math.ceil(length/2)+5)))
    k,kp=_reference_data(geometry,source_s)
    preparation_s = perf_counter()-generation_started
    direct_basis = _basis(length, 6) if generation_mode != 'optimized' else None
    paths=[]
    for cid,fraction in enumerate(np.linspace(0,1,5),1):
        candidate_started = perf_counter()
        path_basis = diagnostic_basis = basis
        direct_attempt = None
        direct_generation_s = direct_screen_s = direct_validation_s = repair_s = 0.
        target=None if not last['boundary_valid']else last['d_min']+fraction*(last['d_max']-last['d_min'])
        attempts=[];selected=None;diagnostic=None;reason='no_validated_solution'
        if not all(v['boundary_valid']for v in sections):reason='invalid_road_section'
        elif abs(slope0)>max_slope:reason='initial_heading_exceeds_slope_limit'
        else:
            if generation_mode != 'optimized':
                started = perf_counter()
                construction_error = None
                if width_fraction_mode:
                    try:
                        direct_basis, direct_c = width_fraction_spline(
                            dense, sections, fraction0, fraction, d0, slope0, target)
                    except (ValueError, np.linalg.LinAlgError) as exc:
                        construction_error = str(exc)
                    direct_method = 'width_fraction_c2_hermite_pose_correction_v1'
                    direct_seed = 'direct_width_fraction'
                else:
                    direct_c = direct_hermite_coefficients(length, d0, slope0, target)
                    direct_method = 'cubic_hermite_actual_start_heading'
                    direct_seed = 'direct_hermite'
                direct_generation_s = perf_counter()-started
                if construction_error is not None:
                    direct_attempt = dict(method=direct_method, accepted=False,
                        reason='width_fraction_construction_failed', stage='construction',
                        detail=construction_error, segment_check=None)
                    if direct_only_mode:
                        reason = direct_attempt['reason']
                else:
                    started = perf_counter()
                    screening = direct_constraint_screen(direct_basis, direct_c, dense, sections,
                        k, kp, max_slope, max_curvature)
                    direct_screen_s = perf_counter()-started
                    if screening is not None:
                        direct_attempt = dict(screening, method=direct_method,
                                              accepted=False, segment_check=None)
                        if direct_only_mode:
                            reason = screening['reason']
                    else:
                        started = perf_counter()
                        direct_points, direct_invalid, direct_segment = _validate(
                            geometry, direct_basis, direct_c, dense, sections, k, kp,
                            target, 0., d0, slope0, max_slope, max_curvature)
                        direct_validation_s = perf_counter()-started
                        direct_attempt = dict(method=direct_method,
                            accepted=direct_invalid is None, first_invalid_sample=direct_invalid,
                            reason=None if direct_invalid is None else direct_points[direct_invalid]['reason'],
                            segment_check=direct_segment, stage='full_validation')
                        diagnostic = (direct_c.copy(), direct_points, direct_invalid, 0.)
                        diagnostic_basis = direct_basis
                        if direct_invalid is None:
                            selected = (0., direct_c, direct_points, 0., direct_seed)
                            path_basis = direct_basis
                            reason = None
            modes = ([] if selected is not None or direct_only_mode else
                     [0.]+([tolerance]if tolerance>0 else[]))
            repair_started = perf_counter()
            for allowed in modes:
                indices=(np.unique(np.linspace(0,len(dense)-1,min(161,len(dense))).astype(int))
                         if legacy_constraint_indices is None else legacy_constraint_indices.copy())
                for refinement in range(max_refinements+1):
                    ds=dense[indices];u=ds/length;blend=3*u*u-2*u*u*u
                    bounds=[sections[i]for i in indices]
                    preferred=np.array([v['d_min']+((1-b)*fraction0+b*fraction)*(v['d_max']-v['d_min'])for v,b in zip(bounds,blend)])
                    fits=fit_multistart(basis,ds,bounds,preferred,d0,slope0,target,k[indices],kp[indices],
                        max_slope,max_curvature,allowed,max_iterations)
                    new_indices=[];valid_fits=[]
                    for c,detail in fits:
                        detail.update(refinement=refinement,constraint_sites=len(indices),
                            limits=dict(safety_margin_m=margin,max_slope=max_slope,max_curvature_inv_m=max_curvature,min_jacobian=.2),
                            validation_accepted=False,validation_reason='solver_constraints_rejected')
                        attempts.append(detail)
                        if detail['solver_constraints_accepted']:
                            detail['validation_accepted']=None
                            detail['validation_reason']='not_selected_for_validation'
                    ranked=sorted((fit for fit in fits if fit[1]['solver_constraints_accepted']),
                                  key=lambda fit:float(fit[1]['objective']))
                    for c,detail in ranked:
                        points,invalid,segment=_validate(geometry,basis,c,dense,sections,k,kp,target,allowed,d0,slope0,max_slope,max_curvature)
                        detail['validation_accepted']=invalid is None
                        detail['validation_reason']=None if invalid is None else points[invalid]['reason']
                        detail['first_invalid_sample']=invalid
                        detail['segment_check']=segment
                        detail['dense_min_lateral_margin_m']=min(
                            (p['lateral_margin_m']for p in points if p['lateral_margin_m']is not None),default=None)
                        detail['dense_max_abs_slope']=max(abs(p['d_prime'])for p in points)
                        detail['dense_max_abs_curvature_inv_m']=max(abs(p['curvature_inv_m'])for p in points)
                        diagnostic=(c.copy(),points,invalid,allowed)
                        diagnostic_basis = basis
                        if invalid is None:
                            valid_fits.append((float(detail['objective']),c.copy(),points,allowed,detail['seed']))
                            break
                        else:new_indices.extend(i for i,p in enumerate(points)if not p['valid'])
                    if valid_fits:
                        selected=min(valid_fits,key=lambda v:v[0]);reason=None
                        break
                    if not new_indices:break
                    expanded=np.unique(np.r_[indices,new_indices])
                    if np.array_equal(expanded,indices):break
                    indices=expanded
                if selected is not None:break
            repair_s = perf_counter()-repair_started if modes else 0.
        if selected is not None:
            _,c,points,allowed,seed=selected;invalid=None
        elif diagnostic is not None:
            c,points,invalid,allowed=diagnostic;seed=None
            path_basis = diagnostic_basis
        else:c=None;points=[];invalid=None;allowed=None;seed=None
        if selected is None and reason=='no_validated_solution':
            reason=points[invalid]['reason']if invalid is not None else'optimizer_constraints_not_satisfied'
        actual=None if c is None else float(path_basis(length)@c)
        deviation=None if actual is None or target is None else actual-target
        first_point=points[0]if points else None
        start_position_error=None if first_point is None or first_point['x']is None or first_point['y']is None else float(
            math.hypot(first_point['x']-x,first_point['y']-y))
        start_heading_error=None if yaw is None or first_point is None or first_point['yaw']is None else float(
            (first_point['yaw']-yaw+math.pi)%(2*math.pi)-math.pi)
        paths.append(dict(candidate_id=cid,geometry_id=geometry.geometry_id,frame_id='map',
            target_fraction=float(fraction),requested_target_d=target,target_d=actual if actual is not None else target,target_s=last['s'],
            actual_target_d=actual,terminal_deviation_m=deviation,endpoint_tolerance_m=tolerance,
            endpoint_mode=None if allowed is None else 'fixed'if allowed==0 else'bounded_preference',
            selected_seed=seed,target_valid=bool(points and points[-1]['geometry_valid']and points[-1]['margin_valid']),
            sampled_valid=selected is not None,status='sampled_valid'if selected is not None else'rejected',
            reason=reason,first_invalid_sample=invalid,points=points,attempts=attempts,
            polyline_collision_verified=selected is not None,heading_matched=selected is not None and yaw is not None,
            start_position_error_m=start_position_error,start_heading_error_rad=start_heading_error,
            spline=None if c is None else dict(knots=path_basis.t.tolist(),coefficients=c.tolist(),degree=path_basis.k,source_s_origin=s0)))
        if generation_mode != 'optimized':
            paths[-1].update(direct_attempt=direct_attempt,
                generation_route=('direct' if seed in ('direct_hermite', 'direct_width_fraction') else
                                  'repaired' if selected is not None else 'rejected'),
                repair_attempted=bool(attempts),
                timing=dict(direct_generation_s=direct_generation_s,
                            direct_screen_s=direct_screen_s, direct_validation_s=direct_validation_s, repair_s=repair_s,
                            candidate_total_s=perf_counter()-candidate_started))
    result = dict(schema_version=1,method='robust_constrained_adaptive_v52',geometry_id=geometry.geometry_id,frame_id='map',
        start=dict(x=float(x),y=float(y),yaw=None if yaw is None else float(yaw),s=s0,d=d0,
                   e_psi=epsi,d_prime=slope0),paths=paths,candidate_count=5,
        valid_target_count=sum(p['target_valid']for p in paths),sampled_valid_count=sum(p['sampled_valid']for p in paths),
        lookahead_m=length,safety_margin_m=margin,requested_sample_step_m=step,actual_sample_step_m=length/(len(dense)-1),
        max_slope=max_slope,max_curvature_inv_m=max_curvature,endpoint_tolerance_m=tolerance,initial_heading_assumed=yaw is None,
        parameter_policy=dict(lookahead='fixed_input',margin='fixed_input',
                              targets='five_equal_fractions_of_exact_terminal_section',
                              speed_adaptation=False),
        road_sections=sections,
        inspection_only=True,planning_allowed=False,domain_verified=False,continuous_verified=False,
        footprint_verified=False,dynamics_verified=False,opponents_verified=False,
        verification_level='dense_original_geometry_and_exact_polyline_point_sweep')
    if validation_step_m is not None:
        result['validation_sampling_policy'] = dict(
            requested_override_m=validation_step_m, raster_half_cap_overridden=True,
            original_dense_grid_verified=False)
    if context is not None:
        result['query_reuse'] = context.summary()
        if query_mode in ('fast_with_fallback', 'fixed_grid_with_fallback', 'fixed_grid_lookup'):
            result['verification_level'] = 'dense_frenet_certified_columns_exact_fallback_and_polyline_point_sweep'
        context.close()
    if grid_summary is not None:
        result['validation_grid'] = grid_summary
        result['actual_sample_step_m'] = float(np.max(np.diff(dense)))
        result['verification_level'] = 'fixed_grid_with_legacy_sites_and_polyline_guard'
    if generation_mode != 'optimized':
        elapsed = perf_counter()-generation_started
        result.update(method=('direct_width_fraction_' if width_fraction_mode else 'direct_hermite_')+generation_mode, generation_mode=generation_mode,
            timing=dict(preparation_s=preparation_s,
                direct_generation_s=sum(p['timing']['direct_generation_s'] for p in paths),
                direct_screen_s=sum(p['timing']['direct_screen_s'] for p in paths),
                direct_validation_s=sum(p['timing']['direct_validation_s'] for p in paths),
                repair_s=sum(p['timing']['repair_s'] for p in paths), generation_total_s=elapsed),
            budget_observation=dict(target_cycle_s=.1, completed_within_target=elapsed <= .1,
                timely_valid_count=result['sampled_valid_count'] if elapsed <= .1 else 0,
                deadline_enforced=False, scope='generation_only_excludes_session_and_io'),
            direct_valid_count=sum(p['generation_route']=='direct' for p in paths),
            repaired_valid_count=sum(p['generation_route']=='repaired' for p in paths))
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry-dir',required=True);parser.add_argument('--map-yaml',required=True)
    parser.add_argument('--x',type=float,required=True);parser.add_argument('--y',type=float,required=True)
    parser.add_argument('--yaw',type=float);parser.add_argument('--lookahead',type=float,default=3.)
    parser.add_argument('--margin',type=float,default=.2);parser.add_argument('--sample-step',type=float,default=.25)
    parser.add_argument('--endpoint-tolerance',type=float,default=0.)
    parser.add_argument('--max-slope',type=float,default=1.);parser.add_argument('--max-curvature',type=float)
    parser.add_argument('--query-mode', choices=('exact', 'reused_exact', 'fast_with_fallback', 'fixed_grid_with_fallback', 'fixed_grid_lookup'), default='exact')
    parser.add_argument('--generation-mode', choices=('optimized', 'direct_only', 'direct_then_repair', 'width_fraction_only', 'width_fraction_then_repair'), default='optimized')
    parser.add_argument('--save');parser.add_argument('--no-show',action='store_true')
    args=parser.parse_args();g=load_geometry(args.geometry_dir,args.map_yaml,inspect=True)
    try:r=generate_robust_paths(g,args.x,args.y,args.yaw,args.lookahead,args.margin,args.sample_step,
                               args.max_slope,args.max_curvature,args.endpoint_tolerance,
                               query_mode=args.query_mode, generation_mode=args.generation_mode)
    except ValueError as exc:parser.error(str(exc))
    print(json.dumps(r,allow_nan=False))
    if args.save or not args.no_show:
        from .candidate_visualization import plot_candidate_paths
        import matplotlib.pyplot as plt
        fig,_=plot_candidate_paths(g,r)
        if args.save:fig.savefig(args.save,dpi=180,bbox_inches='tight')
        if args.no_show:plt.close(fig)
        else:plt.show()


if __name__=='__main__':main()
