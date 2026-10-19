"""A5/A6 offline identification: A4 pool/P80, input-aware L1/h15 kernel.

A5/A6 differ ONLY in the steering used for neighbor distance and kernel weights.
Both regress and predict using delta_cmd. A6 selection needs measured angle at x_t.
This model is diagnostic-only; installing it in PaperLocalLMPC is prohibited.
"""
import json
from pathlib import Path
import numpy as np
from .paper_model import PaperLocalModel


class InputKernelModel(PaperLocalModel):
    def __init__(self,database,geometry,steering_signal,training_source_ids=None):
        if steering_signal not in ('delta_cmd','delta_real'):
            raise ValueError('unknown steering signal')
        super().__init__(database,geometry,training_source_ids=training_source_ids,
            diagnostic_only=True,acceleration_gain_policy='dt_cos_beta_when_unidentified',
            identification_gate='report_only')
        self.steering_signal=steering_signal
        self.command_controls=self.controls.copy()
        self.selection_controls=self.controls.copy()
        if steering_signal=='delta_real':
            rows=[json.loads(s) for s in (Path(database.metadata['source_directory'])/'samples.jsonl').read_text().splitlines()]
            self.selection_controls[:,0]=[rows[int(i)]['physical_state'][2] for i in self.source_ids]

    def select(self,z,u,selection_steering=None):
        z=np.asarray(z,dtype=float);u=np.asarray(u,dtype=float)
        if z.shape!=(6,) or u.shape!=(2,) or not np.isfinite(np.r_[z,u]).all():
            raise ValueError('state and aligned steering/acceleration query required')
        selection_u=u.copy()
        if self.steering_signal=='delta_real':
            if selection_steering is None or not np.isfinite(selection_steering):
                raise ValueError('A6 selection requires measured real angle at state time')
            selection_u[0]=selection_steering
        difference=np.column_stack([self.states[:,:3]-z[:3],self.selection_controls-selection_u])
        distance=np.sum(abs(difference)*[.1,1.,1.,1.,1.],axis=1)
        # Preserve A4's pooled 80-point size, not BARC's per-lap 10 x 6.
        idx=np.lexsort((self.source_ids,distance))[:80]
        weight=.75*np.maximum(1-(distance[idx]/15.)**2,0.)
        if np.count_nonzero(weight)<5:raise ValueError('insufficient input-kernel samples')
        return idx,weight,distance

    def identify(self,z,u=None,selection_steering=None):
        idx,weight,distance=self.select(z,u,selection_steering)
        coefficients=[];channels=[]
        for channel,entry in enumerate([1,0,0]):
            design=np.column_stack([self.states[idx,:3],self.controls[idx,entry],np.ones(len(idx))])
            weighted=design*np.sqrt(weight[:,None]);target=self.targets[idx,channel]*np.sqrt(weight)
            theta,_,rank,singular=np.linalg.lstsq(weighted,target,rcond=1e-8)
            other=weighted[:,[0,1,2,4]]
            other_rank=int(np.sum(np.linalg.svd(other,compute_uv=False)>singular[0]*1e-8))
            identifiable=int(rank)>other_rank;source='observed_regression' if identifiable else 'unidentified'
            if channel==0 and not identifiable:
                beta=0. if np.hypot(z[0],z[1])<=1e-8 else float(np.arctan2(z[1],z[0]))
                gain=self.dt*np.cos(beta)
                rest=np.linalg.lstsq(other,target-weighted[:,3]*gain,rcond=1e-8)[0]
                theta=np.array([*rest[:3],gain,rest[3]]);source='dt_cos_beta_initialization'
            coefficients.append(theta)
            channels.append(dict(channel=['vx','vy','r'][channel],rank=int(rank),
                input_identifiable=identifiable,input_effect_source=source,
                singular_values=singular.tolist(),coefficients=theta.tolist(),
                weighted_rmse=float(np.sqrt(np.average((design@theta-self.targets[idx,channel])**2,weights=weight)))))
        theta=np.array(coefficients)
        return theta,dict(model='A4_pool_input_L1_kernel',steering_signal=self.steering_signal,
            selection_steering_signal=self.steering_signal,regression_steering_signal='delta_cmd',
            source_ids=self.source_ids[idx].tolist(),points=len(idx),diagnostic_only=True,
            kernel='Epanechnikov',bandwidth=15.,distance='weighted_L1(vx,vy,r,delta,a)',
            positive_weights=int(np.sum(weight>0)),nearest_distance=float(distance[idx[0]]),
            farthest_distance=float(distance[idx[-1]]),coefficients=theta.tolist(),paper_regressors=channels)
