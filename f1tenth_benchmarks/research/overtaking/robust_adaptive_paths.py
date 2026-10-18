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

import numpy as np
from scipy.optimize import minimize

from ..core.geometry import load_geometry
from .candidate_validation import MAX_SAMPLES
from .constrained_adaptive_paths import (
    _basis, _positive, _reference_data, evaluate_path_smoothness,
    query_road_sections, validate_path_segments,
)

SEEDS = ('road_fraction', 'hermite', 'road_center')
GROUPS = ('right_margin', 'left_margin', 'positive_slope', 'negative_slope',
          'jacobian', 'positive_curvature', 'negative_curvature')


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
    """Fresh original geometry/boundary queries, independent of solver success."""
    d,p,q=[basis(dense,nu=i)@c for i in range(3)]
    J,curvature=evaluate_path_smoothness(d,p,q,k,kp)
    points=[]
    for i,(distance,lateral)in enumerate(zip(dense,d)):
        section=sections[i]
        # Use fresh boundaries, not the arrays used by the optimizer.
        raw=geometry.boundaries(section['s'])
        right,left=raw.get('d_right'),raw.get('d_left')
        boundary_valid=bool(raw['valid']and right is not None and left is not None
            and np.isfinite([right,left]).all()and right+section['margin']<left-section['margin'])
        margin_valid=bool(boundary_valid and raw['d_right']+section['margin']-1e-8<=lateral<=raw['d_left']-section['margin']+1e-8)
        heading=math.atan2(float(p[i]),float(J[i]))
        exact=geometry.to_cartesian(section['s'],float(lateral),e_psi=heading)
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
    return points,invalid,segment


def generate_robust_paths(geometry,x,y,yaw=None,lookahead_m=40.,safety_margin_m=.2,
                          sample_step_m=.25,max_slope=1.,max_curvature=None,
                          endpoint_tolerance_m=0.,max_refinements=2,max_iterations=150):
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
    ratio=length/min(step,geometry.raster.resolution/2,.05)
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
    sections=query_road_sections(geometry,s0+dense,margin)
    for section in sections:section['margin']=margin
    first,last=sections[0],sections[-1]
    if not first['boundary_valid']or not first['d_min']<=d0<=first['d_max']:
        raise ValueError('Invalid or insufficient boundary at start')
    fraction0=(d0-first['d_min'])/(first['d_max']-first['d_min'])
    basis=_basis(length,min(20,max(6,math.ceil(length/2)+5)))
    k,kp=_reference_data(geometry,s0+dense)
    paths=[]
    for cid,fraction in enumerate(np.linspace(0,1,5),1):
        target=None if not last['boundary_valid']else last['d_min']+fraction*(last['d_max']-last['d_min'])
        attempts=[];selected=None;diagnostic=None;reason='no_validated_solution'
        if not all(v['boundary_valid']for v in sections):reason='invalid_road_section'
        elif abs(slope0)>max_slope:reason='initial_heading_exceeds_slope_limit'
        else:
            modes=[0.]+([tolerance]if tolerance>0 else[])
            for allowed in modes:
                indices=np.unique(np.linspace(0,len(dense)-1,min(161,len(dense))).astype(int))
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
        if selected is not None:
            _,c,points,allowed,seed=selected;invalid=None
        elif diagnostic is not None:
            c,points,invalid,allowed=diagnostic;seed=None
        else:c=None;points=[];invalid=None;allowed=None;seed=None
        if selected is None and reason=='no_validated_solution':
            reason=points[invalid]['reason']if invalid is not None else'optimizer_constraints_not_satisfied'
        actual=None if c is None else float(basis(length)@c)
        deviation=None if actual is None or target is None else actual-target
        paths.append(dict(candidate_id=cid,geometry_id=geometry.geometry_id,frame_id='map',
            target_fraction=float(fraction),requested_target_d=target,target_d=actual if actual is not None else target,target_s=last['s'],
            actual_target_d=actual,terminal_deviation_m=deviation,endpoint_tolerance_m=tolerance,
            endpoint_mode=None if allowed is None else 'fixed'if allowed==0 else'bounded_preference',
            selected_seed=seed,target_valid=bool(points and points[-1]['geometry_valid']and points[-1]['margin_valid']),
            sampled_valid=selected is not None,status='sampled_valid'if selected is not None else'rejected',
            reason=reason,first_invalid_sample=invalid,points=points,attempts=attempts,
            polyline_collision_verified=selected is not None,heading_matched=selected is not None and yaw is not None,
            spline=None if c is None else dict(knots=basis.t.tolist(),coefficients=c.tolist(),degree=basis.k,source_s_origin=s0)))
    return dict(schema_version=1,method='robust_constrained_adaptive_v52',geometry_id=geometry.geometry_id,frame_id='map',
        start=dict(x=float(x),y=float(y),s=s0,d=d0),paths=paths,candidate_count=5,
        valid_target_count=sum(p['target_valid']for p in paths),sampled_valid_count=sum(p['sampled_valid']for p in paths),
        lookahead_m=length,safety_margin_m=margin,requested_sample_step_m=step,actual_sample_step_m=length/(len(dense)-1),
        max_slope=max_slope,max_curvature_inv_m=max_curvature,endpoint_tolerance_m=tolerance,initial_heading_assumed=yaw is None,
        inspection_only=True,planning_allowed=False,domain_verified=False,continuous_verified=False,
        footprint_verified=False,dynamics_verified=False,opponents_verified=False,
        verification_level='dense_original_geometry_and_exact_polyline_point_sweep')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry-dir',required=True);parser.add_argument('--map-yaml',required=True)
    parser.add_argument('--x',type=float,required=True);parser.add_argument('--y',type=float,required=True)
    parser.add_argument('--yaw',type=float);parser.add_argument('--lookahead',type=float,default=40.)
    parser.add_argument('--margin',type=float,default=.2);parser.add_argument('--sample-step',type=float,default=.25)
    parser.add_argument('--endpoint-tolerance',type=float,default=0.)
    parser.add_argument('--max-slope',type=float,default=1.);parser.add_argument('--max-curvature',type=float)
    parser.add_argument('--save');parser.add_argument('--no-show',action='store_true')
    args=parser.parse_args();g=load_geometry(args.geometry_dir,args.map_yaml,inspect=True)
    try:r=generate_robust_paths(g,args.x,args.y,args.yaw,args.lookahead,args.margin,args.sample_step,
                               args.max_slope,args.max_curvature,args.endpoint_tolerance)
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
