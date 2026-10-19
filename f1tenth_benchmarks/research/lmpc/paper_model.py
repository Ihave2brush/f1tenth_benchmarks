"""Paper equations (12)-(17): six states, direct five-coefficient regression.

An explicit optional engineering initialization supplies only an unidentified
longitudinal input gain. No residual ridge, actuator memory or surrogate servo.
The simulator's real actuator remains outside the learned prediction model.
"""
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from .database import sha256
from .local_model import kinematic_step


class PaperIdentificationError(ValueError):
    """The measured local data do not determine a required control effect."""
    def __init__(self, message, diagnostic):
        super().__init__(message)
        self.diagnostic = diagnostic


def build_atv_horizon(model, nominal_states, nominal_inputs):
    """IV-C: identify/linearize at every nominal point before optimization.

    No predicted state from this horizon is fed back into identification.
    The caller supplies the shifted previous solution or an explicit first seed.
    """
    states=np.asarray(nominal_states); inputs=np.asarray(nominal_inputs)
    if (states.shape!=(len(inputs)+1,6) or inputs.ndim!=2 or inputs.shape[1]!=2
            or not np.isfinite(states).all() or not np.isfinite(inputs).all()):
        raise ValueError('invalid six-state ATV nominal horizon')
    models=[]
    for k in range(len(inputs)):
        try:
            models.append(model.linearize(states[k],inputs[k]))
        except PaperIdentificationError as exc:
            exc.diagnostic['horizon_step']=k
            raise
    return models


def rollout_atv(initial, inputs, models):
    """Evaluate equation (15) with the already constructed A/B/C sequence."""
    if len(inputs)!=len(models):raise ValueError('ATV horizon/input length mismatch')
    state=np.asarray(initial).copy(); predictions=[]
    for u,(a,b,c,_) in zip(inputs,models):
        state=a@state+b@u+c
        if not np.isfinite(state).all():raise ValueError('nonfinite ATV prediction')
        predictions.append(state.copy())
    return np.asarray(predictions)


class PaperLocalModel:
    def __init__(self, database, geometry, training_source_ids=None, diagnostic_only=False,
                 acceleration_gain_policy='strict', identification_gate='strict'):
        if type(diagnostic_only) is not bool:
            raise ValueError('diagnostic_only must be boolean')
        self.diagnostic_only = diagnostic_only
        if acceleration_gain_policy not in ('strict', 'dt_cos_beta_when_unidentified'):
            raise ValueError('invalid acceleration_gain_policy')
        self.acceleration_gain_policy = acceleration_gain_policy
        if identification_gate not in ('strict', 'report_only'):
            raise ValueError('invalid identification_gate')
        self.identification_gate = identification_gate
        self.database,self.geometry,self.dt=database,geometry,database.dt
        if not database.metadata.get('acceleration_interface'):
            raise ValueError('paper regression requires recorded delta/a commands, not averaged PP acceleration')
        root=Path(database.metadata['source_directory'])
        for name,digest in database.metadata['source_hashes'].items():
            if sha256(root/name)!=digest:raise ValueError('paper raw source hash mismatch')
        rows=[json.loads(s) for s in (root/'samples.jsonl').read_text().splitlines()]
        eligible=np.unique(np.concatenate([lap.sample_ids[:-1] for lap in database.laps]))
        if training_source_ids is not None:
            requested=np.asarray(training_source_ids)
            if (requested.ndim!=1 or requested.dtype.kind not in 'iu' or len(requested)<80
                    or len(np.unique(requested))!=len(requested) or not np.isin(requested,eligible).all()):
                raise ValueError('training IDs must be unique observed transitions')
            eligible=np.sort(requested)
        self.source_ids=eligible
        states=[]; controls=[]; targets=[]
        for i in eligible:
            before,after=rows[i],rows[i+1]
            if after['input_mode']!='acceleration':raise ValueError('non-command acceleration in paper data')
            states.append(before['paper_state']);controls.append(after['command']);targets.append(after['paper_state'][:3])
        self.states,self.controls,self.targets=map(np.asarray,(states,controls,targets))

    def identify(self,z,u=None):
        begin=perf_counter()
        distance=np.sum(((self.states[:,:3]-z[:3])*[.1,1.,1.])**2,axis=1)
        idx=np.lexsort((self.source_ids,distance))[:80]
        argument=distance[idx]/10.; weight=.75*np.maximum(1-argument**2,0.)
        if np.count_nonzero(weight)<5:raise ValueError('insufficient weighted paper samples')
        theta=[]; diagnostics=[]
        for channel,entry in enumerate([1,0,0]):
            design=np.column_stack([self.states[idx,:3],self.controls[idx,entry],np.ones(len(idx))])
            weighted=design*np.sqrt(weight[:,None]); target=self.targets[idx,channel]*np.sqrt(weight)
            # SVD minimum-norm WLS: numerical rank handling, not ridge or a prior.
            coefficient,_,rank,singular=np.linalg.lstsq(weighted,target,rcond=1e-8)
            # Is the input column independent of the velocity/intercept columns?
            # A constant or policy-collinear command has no identifiable gain.
            # Use the same absolute rank threshold as the original WLS.
            threshold = singular[0]*1e-8
            other = weighted[:, [0, 1, 2, 4]]
            other_rank = int(np.sum(np.linalg.svd(other, compute_uv=False)>threshold))
            input_residual = weighted[:, 3]-other@np.linalg.lstsq(other, weighted[:, 3], rcond=1e-8)[0]
            identifiable = int(rank)>other_rank
            effect_source = 'observed_regression' if identifiable else 'unidentified'
            beta = None
            if (channel == 0 and not identifiable and
                    getattr(self, 'acceleration_gain_policy', 'strict') == 'dt_cos_beta_when_unidentified'):
                # beta is velocity slip in the body frame, not yaw or e_psi.
                # The stationary seed uses beta=0. This is an approximation,
                # not a measured gain or a guarantee about actuator saturation.
                beta = 0. if np.hypot(z[0], z[1]) <= 1e-8 else float(np.arctan2(z[1], z[0]))
                gain = self.dt*np.cos(beta)
                rest = np.linalg.lstsq(other, target-weighted[:,3]*gain, rcond=1e-8)[0]
                coefficient = np.array([*rest[:3], gain, rest[3]])
                effect_source = 'dt_cos_beta_initialization'
            theta.append(coefficient)
            diagnostics.append(dict(rank=int(rank),singular_values=singular.tolist(),
                channel=['vx','vy','r'][channel], input_column=['a','delta','delta'][channel],
                rank_without_input=other_rank, input_identifiable=identifiable,
                input_ready=identifiable or effect_source=='dt_cos_beta_initialization',
                input_effect_source=effect_source, initialization_beta_rad=beta,
                fully_identified=int(rank)==5,
                input_information_norm=float(np.linalg.norm(input_residual)),
                input_range=[float(design[:,3].min()),float(design[:,3].max())],
                coefficients=coefficient.tolist(),weighted_rmse=float(np.sqrt(np.average((design@coefficient-self.targets[idx,channel])**2,weights=weight)))))
        coefficients=np.asarray(theta)
        diagnostic=dict(model='paper_direct_five_coefficient',state_dimension=6,
            source_ids=self.source_ids[idx].tolist(),measured_only=True,points=80,
            kernel='Epanechnikov',kernel_argument='squared_Q_distance / h',bandwidth=10.,
            paper_regressors=diagnostics,coefficients=coefficients.tolist(),
            diagnostic_only=getattr(self,'diagnostic_only',False),
            acceleration_gain_policy=getattr(self,'acceleration_gain_policy','strict'),
            identification_gate=getattr(self,'identification_gate','strict'),
            identification_time_s=perf_counter()-begin)
        missing=[d['channel'] for d in diagnostics if not d['input_identifiable']]
        diagnostic['unidentified_input_channels']=missing
        diagnostic['initialized_input_channels']=[d['channel'] for d in diagnostics
            if d['input_effect_source']=='dt_cos_beta_initialization']
        unready=[d['channel'] for d in diagnostics if not d['input_ready']]
        diagnostic['unready_input_channels']=unready
        if (unready and not getattr(self,'diagnostic_only',False)
                and getattr(self,'identification_gate','strict')=='strict'):
            raise PaperIdentificationError('local paper data cannot identify input effects: '+', '.join(unready), diagnostic)
        return coefficients,diagnostic

    def evaluate(self,z,u,fit=None):
        theta=self.identify(z,u)[0] if fit is None else np.asarray(fit)
        velocities=theta[:,:3]@z[:3]+theta[:,3]*np.asarray(u)[[1,0,0]]+theta[:,4]
        return np.r_[velocities,kinematic_step(z,self.geometry,self.dt)]

    def linearize(self,z,u):
        theta,diagnostic=self.identify(z,u)
        a=np.zeros((6,6));b=np.zeros((6,2))
        a[:3,:3]=theta[:,:3]
        b[0,1],b[1,0],b[2,0]=theta[:,3]
        for i in range(6):
            h=1e-3 if i==4 else 1e-5; perturb=np.eye(6)[i]*h
            a[3:,i]=(kinematic_step(z+perturb,self.geometry,self.dt)-kinematic_step(z-perturb,self.geometry,self.dt))/(2*h)
        value=self.evaluate(z,u,theta)
        diagnostic.update(linearization='paper_ATV_direct_velocity_and_frenet_euler',
                          numerical_derivative='central difference for equation12; exact regression coefficients for equation14')
        return a,b,value-a@z-b@u,diagnostic
