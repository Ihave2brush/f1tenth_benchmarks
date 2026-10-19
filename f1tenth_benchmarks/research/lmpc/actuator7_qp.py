"""Isolated A7 QP: seven prediction states, six terminal equalities.

Fork of controller.py; baseline solver is unchanged.
"""
from time import perf_counter
from dataclasses import replace

import numpy as np
import osqp
from scipy import sparse
from scipy.linalg import null_space
from scipy.optimize import linprog

from .safe_set import initial_terminal_candidate, select_local_safe_set

from .controller import MPCFailure


def solve_qp(initial, nominal, controls, models, terminal, previous_input, settings, stage_regions=None, finish_s=None,
             _workspace=None):
    n, nx, nu = 12, nominal.shape[1], 2
    if nx != 7:
        raise ValueError('A7 QP requires seven states')
    count = terminal.states.shape[1]
    u_start, lam_start = (n+1)*nx, (n+1)*nx+n*nu
    size = lam_start+count
    # Variables are deviations from nominal states/inputs, plus actual lambda.
    p = np.zeros((size, size)); q = np.zeros(size)
    rate = np.asarray(settings['input_rate_weights'])
    for k in range(n):
        for c in range(2):
            vec = np.zeros(size); vec[u_start+2*k+c] = 1.
            offset = controls[k, c]-previous_input[c] if k == 0 else controls[k, c]-controls[k-1, c]
            if k:
                vec[u_start+2*(k-1)+c] = -1.
            p += 2*rate[c]*np.outer(vec, vec); q += 2*rate[c]*offset*vec
    q[lam_start:] = terminal.costs-terminal.costs.min()
    matrix, low, high, physical_scale, roles = [], [], [], [], []

    def add(row, lb, ub=None, scale=1., role=None):
        row = np.atleast_2d(row)
        matrix.extend(row)
        low.extend(np.broadcast_to(lb, (len(row),)))
        high.extend(np.broadcast_to(lb if ub is None else ub, (len(row),)))
        physical_scale.extend([scale]*len(row))
        roles.extend([role]*len(row))

    row = np.zeros((nx, size)); row[:, :nx] = np.eye(nx)
    add(row, initial-nominal[0])
    for k, (a, b, c, _) in enumerate(models):
        row = np.zeros((nx, size))
        row[:, nx*(k+1):nx*(k+2)] = np.eye(nx)
        row[:, nx*k:nx*(k+1)] = -a
        row[:, u_start+2*k:u_start+2*k+2] = -b
        add(row, a@nominal[k]+b@controls[k]+c-nominal[k+1])
    # Rotate terminal equalities into a fixed affine basis; retain all six rows.
    left, singular, _ = np.linalg.svd(terminal.states-terminal.states[:, :1], full_matrices=False)
    transform = np.diag(1/np.maximum(singular, .05))@left.T
    row = np.zeros((6, size)); row[:, n*nx:n*nx+6] = transform
    origin = nominal[n, :6]
    row[:, lam_start:] = -transform@(terminal.states-origin[:, None])
    add(row, np.zeros(6), scale=np.inf)  # original six-state residual checked below
    row = np.zeros(size); row[lam_start:] = 1.
    add(row, 1.)
    row = np.zeros((count, size)); row[:, lam_start:] = np.eye(count)
    add(row, 0., 1.)
    trust = np.asarray(settings['prediction_trust_box']) if nx == 9 else np.full(nx, np.inf)
    paper_bounds=nx==7 and settings.get('constraint_mode')=='paper_simulator_polytope'
    for k in (range(n) if paper_bounds else range(1,n+1)):
        row = np.zeros((nx, size)); row[:, k*nx:(k+1)*nx] = np.eye(nx)
        if paper_bounds:
            # X is explicitly chosen for ESP, not unpublished BARC numbers.
            # Speed: inscribed 32-sided polygon inside ||(vx,vy)|| <= 8.
            # Remove legacy independent vy/r and progress boxes.
            lb=np.array([0.,-np.inf,-np.inf,-settings['max_abs_heading_rad'],-np.inf,
                         -settings['max_abs_lateral_m'], -np.inf])-nominal[k]
            ub=np.array([np.inf,np.inf,np.inf,settings['max_abs_heading_rad'],np.inf,
                         settings['max_abs_lateral_m'], np.inf])-nominal[k]
            mask=np.isfinite(lb)|np.isfinite(ub)
            add(row[mask],lb[mask],ub[mask])
            angles=(np.arange(32)+.5)*2*np.pi/32
            polygon=np.column_stack([np.cos(angles),np.sin(angles)])
            speed_row=np.zeros((32,size));speed_row[:,k*nx:k*nx+2]=polygon
            add(speed_row,-np.inf,settings['max_speed_mps']*np.cos(np.pi/32)-polygon@nominal[k,:2],role='speed_polygon')
            continue
        lb = np.array([settings['equality_gate'], -.5, -5., -settings['max_abs_heading_rad'], initial[4],
                       -settings['max_abs_lateral_m'], -.4189, -.4, -.4])[:nx]-nominal[k]
        ub = np.array([np.sqrt(settings['max_speed_mps']**2-.5**2)-settings['equality_gate'], .5, 5., settings['max_abs_heading_rad'],
                       initial[4]+12*.1*settings['max_speed_mps']*2,
                       settings['max_abs_lateral_m'], .4189, .4, .4])[:nx]-nominal[k]
        add(row, np.maximum(lb, -trust), np.minimum(ub, trust))
    for k in range(n):
        if 'exploration_vx_limits_mps' in settings:
            caps=np.asarray(settings['exploration_vx_limits_mps'],float)
            if caps.shape!=(n,) or not np.isfinite(caps).all() or np.any(caps<=0):
                raise ValueError('invalid local longitudinal exploration limits')
            row=np.zeros(size);row[(k+1)*nx]=1.
            add(row,-np.inf,caps[k]-nominal[k+1,0])
        # A3 engineering exploration bound: all FUTURE nodes x1..xN,
        # including terminal; longitudinal vx, independent of total-speed X.
        if 'exploration_vx_limit_mps' in settings:
            cap=settings['exploration_vx_limit_mps']
            if not np.isfinite(cap) or cap<=0:
                raise ValueError('invalid longitudinal exploration limit')
            row=np.zeros(size);row[(k+1)*nx]=1.
            add(row,-np.inf,cap-nominal[k+1,0])
    for k in range(n):
        row = np.zeros((2, size)); row[:, u_start+2*k:u_start+2*k+2] = np.eye(2)
        # Reserve the residual gate's width inside physical command bounds.
        limit = (np.array([.4189, settings['max_acceleration_mps2']]) if paper_bounds else
                 np.array([.4, settings['max_acceleration_mps2']])-settings['equality_gate'])
        add(row, -limit-controls[k], limit-controls[k])
        if 'exploration_steering_bounds_rad' in settings:
            bounds=np.asarray(settings['exploration_steering_bounds_rad'],float)
            if (bounds.shape!=(n,2) or not np.isfinite(bounds).all()
                    or np.any(bounds[:,0]>bounds[:,1]) or np.any(abs(bounds)>.4189+1e-12)):
                raise ValueError('invalid steering exploration bounds')
            add(row[0],bounds[k,0]-controls[k,0],bounds[k,1]-controls[k,0])
    # Reachability under the requested 20 ms hold + 80 ms ideal slew model.
    # This is not an exact guarantee for the simulator's discrete bang-bang servo.
    for k in range(n):
        row = np.zeros(size); row[u_start+2*k] = 1.; row[k*nx+6] = -1.
        offset = controls[k,0]-nominal[k,6]
        add(row, -.256-offset, .256-offset)
    if stage_regions is not None:
        if finish_s is None or len(stage_regions)!=n:raise ValueError('invalid stage regions')
        for k,region in enumerate(stage_regions):
            row=np.zeros(size);row[k*nx+4]=1.
            if region==-1:add(row,-np.inf,np.inf)
            elif region==1:add(row,-np.inf,np.nextafter(finish_s,-np.inf)-nominal[k,4])
            elif region==0:add(row,finish_s-nominal[k,4],np.inf)
            else:raise ValueError('invalid stage region')
    constraint = np.asarray(matrix); lower, upper = np.array(low), np.array(high)
    if np.any(lower > upper):
        raise MPCFailure('empty prediction/trust bounds', dict(status='invalid_bounds'))
    # Equivalent variable/row equilibration; acceptance remains in original units.
    variable_scale=np.ones(size)
    if nx==7:
        variable_scale[:u_start]=np.tile([1.,1.,1.,.1,1.,.1,.1],n+1)
        variable_scale[u_start:lam_start]=np.tile([.1,1.],n)
        variable_scale[lam_start:]=1./count
    equilibrated=constraint*variable_scale
    row_scale=np.maximum(np.max(abs(equilibrated),axis=1),1e-12)
    ps=p*variable_scale[:,None]*variable_scale;qs=q*variable_scale
    ac=equilibrated/row_scale[:,None];lc=lower/row_scale;uc=upper/row_scale
    reference=np.zeros(size);basis=np.eye(size)
    if paper_bounds:
        # Eliminate E*v=d, rather than asking ADMM to repeatedly approach a
        # very thin terminal affine subspace. No hard equality is softened.
        eq=lc==uc;up=(~eq)&np.isfinite(uc);down=(~eq)&np.isfinite(lc)
        reused=_workspace is not None and 'basis' in _workspace
        if reused:
            if (not np.array_equal(ac[eq],_workspace['equalities'])
                    or not np.array_equal(lc[eq],_workspace['equality_values'])):
                raise ValueError('branch workspace may only reuse identical QP equalities')
            reference=_workspace['reference'];basis=_workspace['basis']
            reference_kind=_workspace['reference_kind']
        else:
            feasibility=linprog(np.zeros(size),A_eq=ac[eq],b_eq=lc[eq],
                A_ub=np.vstack([ac[up],-ac[down]]),b_ub=np.r_[uc[up],-lc[down]],
                bounds=[(None,None)]*size,method='highs',
                options={'primal_feasibility_tolerance':1e-9,'dual_feasibility_tolerance':1e-9})
            if not feasibility.success and feasibility.status==2:
                raise MPCFailure('QP affine feasibility: '+feasibility.message,
                    dict(status='primal infeasible',status_val=osqp.constant('OSQP_PRIMAL_INFEASIBLE')))
            # LP is an initializer, not the QP solver. An unknown LP status must
            # not be mistaken for infeasibility; let OSQP decide afterwards.
            reference=feasibility.x if feasibility.success else np.linalg.lstsq(ac[eq],lc[eq],
                rcond=np.finfo(float).eps*max(ac[eq].shape))[0]
            basis=null_space(ac[eq],rcond=np.finfo(float).eps*max(ac[eq].shape))
            reference_kind='LP feasible point' if feasibility.success else 'affine least squares'
            if _workspace is not None:
                _workspace.update(reference=reference,basis=basis,reference_kind=reference_kind,
                    equalities=ac[eq].copy(),equality_values=lc[eq].copy())
        reduced=ac[~eq]@basis;offset=ac[~eq]@reference
        normalizer=np.maximum(np.max(abs(reduced),axis=1),1e-12)
        lc=(lc[~eq]-offset)/normalizer;uc=(uc[~eq]-offset)/normalizer
        ac=reduced/normalizer[:,None]
        qs=basis.T@(ps@reference+qs);ps=basis.T@ps@basis;ps=(ps+ps.T)/2
    # Exact constraint generation: solve without currently unused speed facets,
    # then check ALL original rows. If a facet is violated, add it and re-solve.
    # A relaxed optimum feasible for the full problem is also its optimum.
    remaining=np.flatnonzero(~eq) if paper_bounds else np.arange(len(lower))
    speed_facets=np.asarray(roles,dtype=object)[remaining]=='speed_polygon'
    active=~speed_facets
    warm=None
    if _workspace is not None and 'solver' in _workspace:
        if (not np.array_equal(ac,_workspace['matrix']) or not np.array_equal(ps,_workspace['hessian'])
                or not np.array_equal(qs,_workspace['linear'])):
            raise ValueError('branch workspace may only reuse identical QP matrices/objective')
        active=_workspace['active'].copy();warm=_workspace.get('warm')
    elapsed=0.;total_iterations=0;cut_rounds=0
    while True:
        if _workspace is not None and 'solver' in _workspace and np.array_equal(active,_workspace['active']):
            solver=_workspace['solver'];solver.update(l=lc[active],u=uc[active])
        else:
            solver=osqp.OSQP()
            solver.setup(P=sparse.csc_matrix(np.triu(ps)),q=qs,A=sparse.csc_matrix(ac[active]),
                l=lc[active],u=uc[active],verbose=False,eps_abs=settings['qp_absolute_tolerance'],
                eps_rel=settings['qp_absolute_tolerance'],max_iter=settings['qp_max_iterations'],
                polish=True,polish_refine_iter=10,adaptive_rho=True,adaptive_rho_interval=50)
            if _workspace is not None:
                _workspace.update(solver=solver,matrix=ac.copy(),hessian=ps.copy(),linear=qs.copy(),active=active.copy())
        if warm is not None:solver.warm_start(x=warm[0],y=warm[1][active])
        start=perf_counter();result=solver.solve();elapsed+=perf_counter()-start
        total_iterations+=result.info.iter
        if result.info.status_val!=1:break
        trial=(reference+basis@result.x)*variable_scale
        violation_rows=np.maximum(lower-constraint@trial,constraint@trial-upper)
        cuts=speed_facets & ~active & (violation_rows[remaining]>settings['qp_absolute_tolerance'])
        full_dual=np.zeros(len(lc));full_dual[active]=result.y
        warm=(result.x.copy(),full_dual)
        if _workspace is not None:_workspace['warm']=warm
        if not cuts.any():break
        active|=cuts;cut_rounds+=1
    info = dict(status=result.info.status, status_val=result.info.status_val,
                iterations=total_iterations, primal_residual=result.info.pri_res,
                dual_residual=result.info.dua_res, solve_time_s=elapsed,
                generated_speed_facets=int(np.sum(active&speed_facets)),constraint_generation_rounds=cut_rounds)
    if result.info.status_val != 1:
        raise MPCFailure('OSQP: '+result.info.status, info)
    v = (reference+basis@result.x)*variable_scale
    raw_violation = np.maximum(lower-constraint@v, constraint@v-upper)
    violation = np.max(raw_violation/np.asarray(physical_scale))
    states = nominal+v[:u_start].reshape(n+1, nx)
    states[0] = initial
    inputs = controls+v[u_start:lam_start].reshape(n, 2)
    command_projection=0.
    if paper_bounds:
        projected=np.clip(inputs,-np.array([.4189,settings['max_acceleration_mps2']]),
                          np.array([.4189,settings['max_acceleration_mps2']]))
        command_projection=float(np.max(abs(projected-inputs)))
        if command_projection:
            # Represent a numerically saturated command exactly at its limit.
            # Recheck original dynamics, not only the command difference.
            inputs=projected;v[u_start:lam_start]=(inputs-controls).ravel()
            raw_violation=np.maximum(lower-constraint@v,constraint@v-upper)
            violation=np.max(raw_violation/np.asarray(physical_scale))
    lam = np.maximum(v[lam_start:], 0.); lam /= lam.sum()
    terminal_error = np.max(np.abs(states[-1, :6]-terminal.states@lam))
    info.update(constraint_violation=float(violation), scaled_constraint_violation=float(raw_violation.max()),
                terminal_residual=float(terminal_error),
                simplex_sum=float(lam.sum()), minimum_raw_lambda=float(v[lam_start:].min()),
                terminal_cost=float(terminal.costs@lam),constraint_mode=settings.get('constraint_mode','legacy'),
                command_roundoff_projection=command_projection,
                state_constraint_indices=list(range(n) if paper_bounds else range(1,n+1)),
                variable_and_row_equilibration=True,equality_elimination=paper_bounds,
                optimization_dimension=int(basis.shape[1]),
                equality_reference=reference_kind if paper_bounds else 'none',
                branch_workspace_reused=bool(paper_bounds and reused))
    if max(violation, terminal_error) > settings['equality_gate']:
        raise MPCFailure('QP residual gate failed', info)
    return states, inputs, lam, info


def solve_finish_qp(initial,nominal,controls,models,terminal,previous_input,settings,finish_s):
    """Literal h(s) plus active-finish Q-function, using convex QP branches.

    An indicator is not a quadratic function. Away from the finish, conservative
    affine interval bounds prove h constant. Near it, branch on uncertain stages;
    each branch is an ordinary QP, never a frozen nominal indicator approximation.
    """
    terminal=replace(terminal,costs=np.maximum(terminal.costs,0.))
    regions=[-1]*12
    if settings.get('constraint_mode')=='paper_simulator_polytope':
        low=np.asarray(initial).copy();high=low.copy()
        affine_origin=np.asarray(initial).copy();input_map=np.zeros((len(initial),24))
        all_limits=np.tile([.4189,settings['max_acceleration_mps2']],12)
        for k,(a,b,c,_) in enumerate(models):
            # The whole horizon is affine in its bounded controls. This second
            # enclosure keeps cancellations across time that interval recursion
            # loses; intersecting both enclosures cannot exclude a feasible path.
            radius=float(abs(input_map[4])@all_limits)
            padding=1e-8+32*np.finfo(float).eps*(abs(affine_origin[4])+radius)
            lower_s=max(low[4],affine_origin[4]-radius-padding)
            upper_s=min(high[4],affine_origin[4]+radius+padding)
            if upper_s<finish_s:regions[k]=1
            elif lower_s>=finish_s:regions[k]=0
            # Interval enclosure of affine s update under the selected X/U.
            lo=np.array([0.,-settings['max_speed_mps'],-np.inf,-settings['max_abs_heading_rad'],low[4],-settings['max_abs_lateral_m'],-np.inf])
            hi=np.array([settings['max_speed_mps'],settings['max_speed_mps'],np.inf,settings['max_abs_heading_rad'],high[4],settings['max_abs_lateral_m'],np.inf])
            if k==0:lo=hi=np.asarray(initial)
            al=a[4];bl=b[4];limits=np.array([.4189,settings['max_acceleration_mps2']])
            nz=al!=0
            next_lo=float(np.sum(np.where(al[nz]>=0,al[nz]*lo[nz],al[nz]*hi[nz]))-np.sum(abs(bl)*limits)+c[4])
            next_hi=float(np.sum(np.where(al[nz]>=0,al[nz]*hi[nz],al[nz]*lo[nz]))+np.sum(abs(bl)*limits)+c[4])
            low[4],high[4]=next_lo,next_hi
            affine_origin=a@affine_origin+c
            input_map=a@input_map;input_map[:,2*k:2*k+2]+=b
    pending=[tuple(regions)];best=None;best_cost=np.inf;attempts=0;pruned=0;started=perf_counter()
    workspace={} if settings.get('constraint_mode')=='paper_simulator_polytope' else None
    while pending:
        branch=pending.pop();attempts+=1
        try:
            states,actions,lam,info=solve_qp(initial,nominal,controls,models,terminal,previous_input,
                settings,branch,finish_s,_workspace=workspace)
        except MPCFailure as exc:
            if exc.diagnostic.get('status_val')==osqp.constant('OSQP_PRIMAL_INFEASIBLE'):
                pruned+=1;continue
            exc.diagnostic.update(finish_branch=list(branch),finish_qp_attempts=attempts)
            raise
        actual=(states[:-1,4]<finish_s).astype(int)
        boundary_adjustment=0.
        for k,region in enumerate(branch):
            if region==-1 or region==actual[k]:continue
            if k>0 and abs(states[k,4]-finish_s)<=settings['equality_gate']:
                target=np.nextafter(finish_s,-np.inf) if region==1 else finish_s
                boundary_adjustment=max(boundary_adjustment,abs(target-states[k,4]))
                states[k,4]=target
        actual=(states[:-1,4]<finish_s).astype(int)
        if boundary_adjustment:
            residual=max(float(np.max(abs(states[k+1]-a@states[k]-b@actions[k]-c)))
                for k,(a,b,c,_) in enumerate(models))
            if residual>settings['equality_gate']:
                raise MPCFailure('finish boundary representation violates dynamics',dict(dynamic_residual=residual))
            info.update(finish_boundary_adjustment_m=boundary_adjustment,dynamic_residual_after_boundary=residual)
        if any(region!=-1 and region!=actual[k] for k,region in enumerate(branch)):
            raise MPCFailure('finish indicator boundary unresolved at numerical precision',
                dict(status='finish_boundary_ambiguous',finish_branch=list(branch),stage_s=states[:-1,4].tolist()))
        rate=float(np.sum(np.diff(np.vstack([previous_input,actions]),axis=0)**2*np.asarray(settings['input_rate_weights'])))
        base=float(terminal.costs@lam)+rate
        objective=base+int(actual.sum())
        if objective<best_cost:
            best_cost=objective;best=(states,actions,lam,info)
        lower=base+sum(region==1 for region in branch)
        if lower>=best_cost-settings['qp_absolute_tolerance']:
            pruned+=1;continue
        # Only a stage classified before finish can improve the incumbent h.
        uncertain=[k for k,r in enumerate(branch) if r==-1]
        if uncertain:
            k=next((k for k in uncertain if actual[k]==1),uncertain[0])
            before=list(branch);before[k]=1
            after=list(branch);after[k]=0
            pending.extend([tuple(before),tuple(after)])
    if best is None:raise MPCFailure('no feasible finish-indicator QP branch',dict(status='primal infeasible',status_val=3))
    states,actions,lam,info=best
    info.update(finish_cost_mode='literal_indicator',stage_cost=int(np.sum(states[:-1,4]<finish_s)),
        objective_with_stage_and_rate=float(best_cost),finish_qp_attempts=attempts,finish_pruned_branches=pruned,
        finish_total_s=perf_counter()-started,active_finish_terminal_cost='zero after active finish; stored signed potential preserved',
        finish_screened_stages=sum(r!=-1 for r in regions),
        indicator_solver='QP branch enumeration with affine interval screening; numerical QP tolerances apply')
    return states,actions,lam,info
