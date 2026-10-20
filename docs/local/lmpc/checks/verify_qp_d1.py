"""Verify archived QP-D1 certificates without running any optimizer."""
import argparse
import json
from pathlib import Path
import numpy as np


def main():
    p=argparse.ArgumentParser();p.add_argument('output');args=p.parse_args();root=Path(args.output)
    report={}
    original_first=json.loads(Path('Data/research/lmpc/experiments/A5_NO_full_20261019_001/run/controls.jsonl').open().readline())
    for n in [1,2]:
        d=root/('qp%d'%n);f=np.load(d/'full_physical_qp.npz');P,q,A,l,u=[f[k] for k in ['P','q','A','l','u']]
        eigen=np.linalg.eigvalsh(P);assert eigen.min()>=-1e-12
        inputs=json.loads((d/'input_verification.json').read_text())
        model=np.load(Path(inputs['source'])/'model_and_terminal.npz')
        if n==1:
            for k in range(12):
                for key in ['A','B','C']:np.testing.assert_array_equal(model[key][k],original_first['prediction_models'][k][key])
            np.testing.assert_array_equal(model['initial'],original_first['actual_state'])
            baseline=np.r_[(np.array(original_first['states'])-model['nominal']).ravel(),
                (np.array(original_first['controls'])-model['inputs']).ravel(),original_first['lambda_weights']]
            baseline_cost=float(.5*baseline@P@baseline+q@baseline)
        else:baseline_cost=None
        checks={}
        for name in ['reduced_saved','reduced_all','full_assembled','full_physical']:
            z=np.load(d/(name+'.npz'));x,y=z['physical_x'],z['physical_y'];ax=A@x
            violation=np.maximum(np.maximum(l-ax,ax-u),0).max()
            stationarity=abs(P@x+q+A.T@y).max()
            comp=[];dual_bound=0.;sign=0.
            for i in range(len(y)):
                if l[i]==u[i]:dual_bound+=l[i]*y[i]
                elif y[i]>0:
                    if np.isfinite(u[i]):comp.append(abs(y[i]*(u[i]-ax[i])));dual_bound+=u[i]*y[i]
                    else:sign=max(sign,y[i])
                else:
                    if np.isfinite(l[i]):comp.append(abs(y[i]*(ax[i]-l[i])));dual_bound+=l[i]*y[i]
                    else:sign=max(sign,-y[i])
            gap=float(x@P@x+q@x+dual_bound);objective=float(.5*x@P@x+q@x)
            valid=bool(violation<=1e-5 and stationarity<=1e-5 and max(comp,default=0)<=1e-5 and sign<=1e-7 and abs(gap)<=1e-5)
            expected=(n!=2 or name!='reduced_saved');assert valid==expected
            checks[name]=dict(primal_violation=float(violation),stationarity=float(stationarity),
                complementarity=float(max(comp,default=0)),dual_sign=float(sign),gap=gap,objective=objective,verified_optimal=valid)
        report['qp%d'%n]=dict(P_min_eigenvalue=float(eigen.min()),P_max_eigenvalue=float(eigen.max()),
            P_rank=int(np.linalg.matrix_rank(P)),first_original_runtime_ABC_exact=(n==1),
            first_original_applied_solution_cost=baseline_cost,
            first_original_cost_difference_to_full=None if n==2 else baseline_cost-checks['full_physical']['objective'],
            checks=checks)
    (root/'certificate_verification.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))


if __name__=='__main__':main()
