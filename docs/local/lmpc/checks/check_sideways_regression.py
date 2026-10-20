"""Read-only independent SVD audit of saved fixed-ATV regression evidence."""
import json
from pathlib import Path
import numpy as np

base=Path('Data/research/lmpc')
validation=json.loads((base/'r3_fixed_atv_20261014_001/validation.json').read_text())
db=json.loads((base/'b2_esp_observed_20261012_001/database.json').read_text())
rows=[json.loads(s) for s in (Path(db['source_directory'])/'samples.jsonl').read_text().splitlines()]
worst=max(validation['records'],key=lambda r:np.max(np.abs(r['errors'])))
checks=[]
for stage in (0,1):
    diagnostic=worst['prediction_models'][stage]['identification']
    ids=np.array(diagnostic['source_ids']);query=np.array(worst['nominal_states'][stage])
    states=np.array([rows[i]['paper_state'] for i in ids])
    controls=np.array([rows[i+1]['command'] for i in ids])
    targets=np.array([rows[i+1]['paper_state'][:3] for i in ids])
    distance=np.sum(((states[:,:3]-query[:3])*[.1,1,1])**2,axis=1)
    weights=.75*np.maximum(1-(distance/10)**2,0)
    design=np.column_stack([states[:,:3],controls[:,0],np.ones(80)])
    weighted=design*np.sqrt(weights[:,None])
    left,singular,right=np.linalg.svd(weighted,full_matrices=False)
    keep=singular>singular[0]*1e-8
    for channel in (1,2):
        y=targets[:,channel]*np.sqrt(weights)
        # Explicit truncated pseudoinverse, independent from np.linalg.lstsq.
        theta=right[keep].T@((left[:,keep].T@y)/singular[keep])
        recorded=np.array(diagnostic['coefficients'][channel])
        normal_residual=right[keep]@(weighted.T@(weighted@theta-y))
        # Perturb targets along the weakest retained data direction by only
        # 1e-6 in weighted 2-norm; this is a counterfactual, not training data.
        perturbation=left[:,np.where(keep)[0][-1]]*1e-6
        perturbed=right[keep].T@((left[:,keep].T@(y+perturbation))/singular[keep])
        checks.append(dict(stage=stage,channel=['vx','vy','r'][channel],
            unique_source_ids=len(set(ids.tolist())),rank=int(keep.sum()),
            singular_values=singular.tolist(),effective_condition=float(singular[0]/singular[keep][-1]),
            explicit_svd_coefficients=theta.tolist(),recorded_coefficients=recorded.tolist(),
            coefficient_relative_difference=float(np.linalg.norm(theta-recorded)/np.linalg.norm(recorded)),
            retained_normal_equation_residual=float(np.linalg.norm(normal_residual)),
            weighted_rmse=float(np.sqrt(np.average((design@theta-targets[:,channel])**2,weights=weights))),
            feature_ranges=np.ptp(design,axis=0).tolist(),
            feature_correlation=np.corrcoef(design[:,:4].T).tolist(),
            weakest_retained_feature_direction=right[np.where(keep)[0][-1]].tolist(),
            target_perturbation_weighted_norm=float(np.linalg.norm(perturbation)),
            coefficient_change_norm=float(np.linalg.norm(perturbed-theta))))
result=dict(raw_source_id=worst['raw_source_id'],nominal_source_id=worst['nominal_source_ids'][0],
    design_order=['vx','vy','r','delta','1'],transition='x_i, command_(i+1), x_(i+1)',checks=checks)
out=base/'r3_sideways_regression_audit_20261014_001'
out.mkdir(exist_ok=False)
(out/'audit.json').write_text(json.dumps(result,indent=2,allow_nan=False))
for c in checks:
    print({k:c[k] for k in ('stage','channel','rank','effective_condition',
        'coefficient_relative_difference','retained_normal_equation_residual',
        'weighted_rmse','coefficient_change_norm','singular_values','feature_ranges')})
