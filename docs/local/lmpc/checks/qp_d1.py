"""QP-D1: saved matrices only; no controller imports, simulation or parameter scan.

Run from repository root in Python 3.9.25 / numpy 1.22.0 / OSQP 0.6.3.
Requires existing A5-NO and S1 artifacts. Refuses to overwrite its output.
"""
import argparse
import csv
import hashlib
import json
import platform
from pathlib import Path
from time import perf_counter

import numpy as np
import scipy
from scipy import sparse
import osqp


BASE = Path('Data/research/lmpc/experiments')
NO = BASE/'A5_NO_full_20261019_001'
S1 = BASE/'A5_NO_S1_20261019_001'
SETTINGS = dict(verbose=False, eps_abs=1e-7, eps_rel=1e-7,
    max_iter=100000, polish=True, polish_refine_iter=10,
    adaptive_rho=True, adaptive_rho_interval=50, scaling=10, rho=.1,
    warm_start=True, scaled_termination=False)


def read(p):
    return json.loads(p.read_text())


def native(v):
    if isinstance(v, np.ndarray): return native(v.tolist())
    if isinstance(v, np.generic): return native(v.item())
    if isinstance(v, dict): return {str(k): native(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)): return [native(x) for x in v]
    if isinstance(v, float) and not np.isfinite(v): return str(v)
    return v


def save(p, v):
    p.write_text(json.dumps(native(v), ensure_ascii=False, indent=2, allow_nan=False)+'\n')


def digest(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def load(p):
    # Do not load the legacy object-array roles entry; no pickle required.
    with np.load(p, allow_pickle=False) as f:
        return {k: f[k] for k in f.files if k != 'roles'}


def build(m, settings, branch, finish):
    """Independently rebuild physical full QP in saved deviation coordinates."""
    P=np.zeros((142,142)); q=np.zeros(142); rows=[]; low=[]; high=[]; labels=[]
    def add(row, l, u=None, kind='', stage=None, channel=None):
        row=np.atleast_2d(row); count=len(row)
        rows.extend(row); low.extend(np.broadcast_to(l,(count,)))
        high.extend(np.broadcast_to(l if u is None else u,(count,)))
        for i in range(count): labels.append(dict(row=len(labels),kind=kind,stage=stage,channel=channel,index=i))
    for k in range(12):
        for c in range(2):
            v=np.zeros(142);v[78+2*k+c]=1
            offset=m['inputs'][k,c]-(m['previous_input'][c] if k==0 else m['inputs'][k-1,c])
            if k:v[78+2*(k-1)+c]=-1
            P+=2*settings['input_rate_weights'][c]*np.outer(v,v)
            q+=2*settings['input_rate_weights'][c]*offset*v
    q[102:]=m['terminal_costs']-m['terminal_costs'].min()
    row=np.zeros((6,142));row[:,:6]=np.eye(6)
    add(row,m['initial']-m['nominal'][0],kind='initial',stage=0)
    for k in range(12):
        row=np.zeros((6,142));row[:,6*(k+1):6*(k+2)]=np.eye(6)
        row[:,6*k:6*(k+1)]=-m['A'][k];row[:,78+2*k:80+2*k]=-m['B'][k]
        add(row,m['A'][k]@m['nominal'][k]+m['B'][k]@m['inputs'][k]+m['C'][k]-m['nominal'][k+1],kind='dynamics',stage=k)
    row=np.zeros((6,142));row[:,72:78]=np.eye(6);row[:,102:]=-m['terminal_states']
    add(row,-m['nominal'][12],kind='terminal_physical',stage=12)
    row=np.zeros(142);row[102:]=1;add(row,1,kind='simplex',stage=12)
    row=np.zeros((40,142));row[:,102:]=np.eye(40);add(row,0,1,kind='lambda',stage=12)
    for k in range(12):
        row=np.zeros((6,142));row[:,k*6:k*6+6]=np.eye(6)
        lb=np.array([0,-np.inf,-np.inf,-settings['max_abs_heading_rad'],-np.inf,-settings['max_abs_lateral_m']])-m['nominal'][k]
        ub=np.array([np.inf,np.inf,np.inf,settings['max_abs_heading_rad'],np.inf,settings['max_abs_lateral_m']])-m['nominal'][k]
        mask=np.isfinite(lb)|np.isfinite(ub);add(row[mask],lb[mask],ub[mask],kind='state',stage=k)
        angles=(np.arange(32)+.5)*2*np.pi/32;poly=np.column_stack([np.cos(angles),np.sin(angles)])
        row=np.zeros((32,142));row[:,k*6:k*6+2]=poly
        add(row,-np.inf,settings['max_speed_mps']*np.cos(np.pi/32)-poly@m['nominal'][k,:2],kind='speed',stage=k)
    for k,cap in enumerate(settings['exploration_vx_limits_mps']):
        row=np.zeros(142);row[(k+1)*6]=1
        add(row,-np.inf,cap-m['nominal'][k+1,0],kind='local_vx',stage=k+1)
    for k in range(12):
        row=np.zeros((2,142));row[:,78+2*k:80+2*k]=np.eye(2)
        lim=np.array([.4189,settings['max_acceleration_mps2']])
        add(row,-lim-m['inputs'][k],lim-m['inputs'][k],kind='input',stage=k)
        lo,hi=settings['exploration_steering_bounds_rad'][k]
        add(row[0],lo-m['inputs'][k,0],hi-m['inputs'][k,0],kind='local_steering',stage=k)
    for k,region in enumerate(branch):
        row=np.zeros(142);row[k*6+4]=1
        if region==1:l,u=-np.inf,np.nextafter(finish,-np.inf)-m['nominal'][k,4]
        elif region==0:l,u=finish-m['nominal'][k,4],np.inf
        else:l,u=-np.inf,np.inf
        add(row,l,u,kind='finish',stage=k)
    return P,q,np.array(rows),np.array(low),np.array(high),labels


def violation(A,l,u,x):
    return np.maximum(np.maximum(l-A@x,A@x-u),0)


def metrics(x,y,P,q,A,l,u,m,settings,labels,saved):
    vals=A@x; v=violation(A,l,u,x); eq=l==u
    # Equality multipliers are recovered in the common physical formulation.
    # Reduced solves have no equality duals. Inequality duals are unscaled exactly.
    y=y.copy(); y[eq]=np.linalg.lstsq(A[eq].T,-(P@x+q+A[~eq].T@y[~eq]),rcond=None)[0]
    stat=P@x+q+A.T@y
    comp=[];sign=[];support=0.
    for i in range(len(y)):
        if eq[i]:support+=l[i]*y[i];continue
        if y[i]>=0:
            if np.isfinite(u[i]):comp.append(abs(y[i]*(u[i]-vals[i])));support+=u[i]*y[i]
            else:sign.append(y[i])
        else:
            if np.isfinite(l[i]):comp.append(abs(y[i]*(vals[i]-l[i])));support+=l[i]*y[i]
            else:sign.append(-y[i])
    states=m['nominal']+x[:78].reshape(13,6);inputs=m['inputs']+x[78:102].reshape(12,2);lam=x[102:]
    dynamics=np.array([states[k+1]-m['A'][k]@states[k]-m['B'][k]@inputs[k]-m['C'][k] for k in range(12)])
    objective=float(.5*x@P@x+q@x)
    rate=float(np.sum(np.diff(np.vstack([m['previous_input'],inputs]),axis=0)**2*np.array(settings['input_rate_weights'])))
    physical=rate+float(m['terminal_costs']@lam)+int(np.sum(states[:-1,4]<settings['_finish']))
    groups={kind:float(v[[i for i,d in enumerate(labels) if d['kind']==kind]].max()) for kind in set(d['kind'] for d in labels)}
    worst=[dict(labels[i],value=vals[i],lower=l[i],upper=u[i],violation=v[i]) for i in np.argsort(v)[::-1][:12]]
    out=dict(objective=objective,physical_cost_with_stage=physical,input_rate_cost=rate,
        terminal_cost=float(m['terminal_costs']@lam),all_original_row_violation=float(v.max()),
        saved_assembled_row_violation=float(violation(saved['constraint'],saved['lower'],saved['upper'],x).max()),
        initial_error=float(abs(states[0]-m['initial']).max()),dynamics_error=float(abs(dynamics).max()),
        terminal_error=float(abs(states[-1]-m['terminal_states']@lam).max()),
        simplex_error=float(abs(lam.sum()-1)),lambda_min=float(lam.min()),lambda_max=float(lam.max()),
        stationarity_inf=float(abs(stat).max()),complementarity_inf=float(max(comp,default=0)),
        dual_sign_violation=float(max(sign,default=0)),duality_gap=float(x@P@x+q@x+support),
        constraint_groups=groups,worst_rows=worst,equality_duals='least-squares recovery in common physical full QP')
    out['verified_feasible']=out['all_original_row_violation']<=1e-5 and out['saved_assembled_row_violation']<=1e-5
    out['verified_kkt']=out['verified_feasible'] and out['stationarity_inf']<=1e-5 and out['complementarity_inf']<=1e-5 and out['dual_sign_violation']<=1e-7 and abs(out['duality_gap'])<=1e-5
    return out,y


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);args=parser.parse_args()
    out=Path(args.output);out.mkdir(parents=True,exist_ok=False)
    assert osqp.__version__=='0.6.3' and np.__version__=='1.22.0' and platform.python_version()=='3.9.25'
    protected=list(Path('f1tenth_benchmarks').rglob('*.py'))+list(Path('params').rglob('*.yaml'))+list(Path('tests').rglob('*.py'))
    protected+=list(NO.rglob('*'))+list(S1.rglob('*'))
    protected=sorted(set(p for p in protected if p.is_file() and '__pycache__' not in p.parts))
    before={str(p):digest(p) for p in protected};save(out/'protected_before.json',before)
    baseline=read(NO/'qp2_diagnosis_20261019_001/effective_solver_settings.json')
    for k in ['eps_abs','eps_rel','max_iter','polish','polish_refine_iter','adaptive_rho','adaptive_rho_interval']:
        assert SETTINGS[k]==baseline[k]
    env=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__,osqp=osqp.__version__,settings=SETTINGS)
    save(out/'environment.json',env)
    summary={}; tables=[]
    for number,src in [(1,S1/'qp_snapshots/qp_0001'),(2,NO/'qp2_diagnosis_20261019_001')]:
        dest=out/('qp%d'%number);dest.mkdir();old=load(src/'original_qp.npz');red=load(src/'solver_qp.npz');m=load(src/'model_and_terminal.npz')
        sess=read(NO/'run/session.json');settings=sess['config']['settings'].copy()
        if number==1:
            context=read(S1/'qp_assemblies/assembly_0001/context.json');settings=context['settings'];branch=m['finish_branch']
            # First snapshot came from S1, but models/history/handoff match A5-NO.
            assert read(S1/'handoff_verification.json')['passed']
            verification=read(S1/'verification.json')
        else:
            fd=read(NO/'run/failure.json')['failure_detail'];branch=fd['finish_branch']
            settings['exploration_vx_limits_mps']=[d['effective_vx_cap_mps'] for d in fd['steering_exploration']]
            settings['exploration_steering_bounds_rad']=[[-d['effective_cap_rad'],d['effective_cap_rad']] for d in fd['steering_exploration']]
        # All saved finish rows provide the same physical finish threshold.
        candidates=[old['upper'][593+k]+m['nominal'][k,4] for k,b in enumerate(branch) if b==1]
        finish=float(max(candidates));settings['_finish']=np.nextafter(finish,np.inf)
        P,q,A,l,u,labels=build(m,settings,branch,settings['_finish'])
        assert P.shape==(142,142) and A.shape==(605,142)
        np.testing.assert_array_equal(P,old['p']);np.testing.assert_array_equal(q,old['q'])
        nonterm=np.r_[np.arange(78),np.arange(84,605)]
        np.testing.assert_array_equal(A[nonterm],old['constraint'][nonterm])
        np.testing.assert_array_equal(l[nonterm],old['lower'][nonterm]);np.testing.assert_array_equal(u[nonterm],old['upper'][nonterm])
        left,sv,_=np.linalg.svd(m['terminal_states']-m['terminal_states'][:,:1],full_matrices=False)
        T=np.diag(1/np.maximum(sv,.05))@left.T;offset=T@m['nominal'][12]
        term=T@A[78:84]+offset[:,None]*A[84]
        terr=float(abs(term-old['constraint'][78:84]).max());assert terr<1e-12
        assert abs(T@l[78:84]+offset-old['lower'][78:84]).max()<1e-12
        eq=l==u;scale=old['variable_scale'];basis=old['basis'];ref=old['reference']
        lift=basis*scale[:,None];origin=ref*scale
        # Saved reduced Hessian is symmetric before upper-triangle serialization.
        redP=red['P']+np.triu(red['P'],1).T
        np.testing.assert_allclose(lift.T@P@lift,redP,atol=1e-14,rtol=1e-10)
        np.testing.assert_allclose(lift.T@(P@origin+q),red['q'],atol=1e-12,rtol=1e-10)
        check=dict(source=str(src),source_kind='S1 runtime snapshot, first A5-NO model/history verified' if number==1 else 'previously verified offline reconstruction; original runtime matrix unavailable',
            variables={'states_deviation':'0:78, x0..x12, [vx,vy,r,e_psi,s,e_y]','inputs_deviation':'78:102, u0..u11, [delta_cmd,a_cmd]','lambda':'102:142','slack':0},
            dimensions=[142,605],equality_rows=int(sum(eq)),terminal_transformation_error=terr,
            original_nonterminal_matrices_exact=True,original_cost_exact=True,
            canonical_terminal='xN-D lambda=0; removed SVD rotation/scaling and centering via sum(lambda)=1',
            source_sha256={f:digest(src/f) for f in ['original_qp.npz','solver_qp.npz','model_and_terminal.npz']})
        save(dest/'input_verification.json',check);save(dest/'constraint_labels.json',labels)
        np.savez_compressed(dest/'full_physical_qp.npz',P=P,q=q,A=A,l=l,u=u)
        tests=[('reduced_saved',redP,red['q'],red['A'],red['l'],red['u']),
            ('reduced_all',old['ps'],old['qs'],old['ac'],old['lc'],old['uc']),
            ('full_assembled',P,q,old['constraint'],old['lower'],old['upper']),
            ('full_physical',P,q,A,l,u)]
        results={};solutions={}
        for name,sp,sq,sa,sl,su in tests:
            solver=osqp.OSQP();start=perf_counter()
            solver.setup(P=sparse.csc_matrix(np.triu(sp)),q=sq,A=sparse.csc_matrix(sa),l=sl,u=su,**SETTINGS)
            setup=perf_counter()-start;start=perf_counter();res=solver.solve();elapsed=perf_counter()-start
            x=origin+lift@res.x if name.startswith('reduced') else res.x.copy()
            y=np.zeros(605)
            if name.startswith('reduced'):
                idx=old['remaining'][old['active']] if name=='reduced_saved' else old['remaining']
                norm=old['normalizer'][old['active']] if name=='reduced_saved' else old['normalizer']
                y[idx]=res.y/(old['row_scale'][idx]*norm)
            else:y=res.y.copy()
            met,physical_y=metrics(x,y,P,q,A,l,u,m,settings,labels,old)
            record=dict(status=res.info.status,status_val=res.info.status_val,iterations=res.info.iter,
                primal_residual=res.info.pri_res,dual_residual=res.info.dua_res,setup_wall_s=setup,
                solve_wall_s=elapsed,osqp_setup_s=res.info.setup_time,osqp_solve_s=res.info.solve_time,
                osqp_polish_s=res.info.polish_time,rho_updates=res.info.rho_updates,
                native_stationarity=float(abs(sp@res.x+sq+sa.T@res.y).max()),**met)
            np.savez_compressed(dest/(name+'.npz'),solver_x=res.x,solver_y=res.y,physical_x=x,physical_y=physical_y)
            save(dest/(name+'.json'),record);results[name]=record;solutions[name]=x
            tables.append(dict(qp=number,path=name,**{k:record[k] for k in ['status','iterations','primal_residual','dual_residual','solve_wall_s','lambda_min','objective','all_original_row_violation','stationarity_inf','complementarity_inf','duality_gap','verified_kkt']}))
            print('QP',number,name,record['status'],record['iterations'],'violation',record['all_original_row_violation'],'KKT',record['verified_kkt'],flush=True)
        comparisons={}
        for name,x in solutions.items():
            reference=solutions['full_physical'];d=x-reference
            comparisons[name]=dict(objective_difference=results[name]['objective']-results['full_physical']['objective'],
                state_max_difference=float(abs(d[:78]).max()),input_max_difference=float(abs(d[78:102]).max()),
                lambda_max_difference=float(abs(d[102:]).max()),terminal_convex_state_difference=float(abs(m['terminal_states']@d[102:]).max()),
                comparable_as_optima=results[name]['verified_kkt'] and results['full_physical']['verified_kkt'])
        save(dest/'comparisons.json',comparisons)
        summary['qp%d'%number]=dict(input_verification=check,results=results,comparisons=comparisons)
    changed=[p for p,h in before.items() if not Path(p).exists() or digest(Path(p))!=h]
    save(out/'integrity.json',dict(protected_files=len(before),changed=changed));assert not changed
    save(out/'summary.json',summary)
    with (out/'results.csv').open('w') as f:
        writer=csv.DictWriter(f,fieldnames=list(tables[0]));writer.writeheader();writer.writerows(tables)
    print('COMPLETE',out,flush=True)


if __name__=='__main__': main()
