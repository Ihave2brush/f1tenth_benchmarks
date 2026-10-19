"""A7 experimental actuator-state model; baseline paper model remains unchanged.

The deterministic simulator is discrete bang-bang with a two-tick FIFO. Seven
states alone do not encode that FIFO. The requested endpoint=command row and
continuous-slew sensitivities are explicitly approximations. Exact nominal
FIFO replay anchors the affine mean; it does not make off-nominal slopes exact.
"""
import numpy as np
from .paper_model import PaperLocalModel, PaperIdentificationError
from .local_model import kinematic_step


def replay_actuator(delta, buffer, command, steps=10, tick=.01):
    """Original scalar FIFO/PID/Euler steering for the fixed ESP configuration."""
    buffer=list(buffer); pre=[]
    for _ in range(steps):
        delayed=buffer[-1] if len(buffer)>=2 else 0.
        buffer=[float(command)]+buffer[:1]
        diff=delayed-delta
        rate=float(np.sign(diff)*3.2) if abs(diff)>1e-4 else 0.
        if (delta<=-.4189 and rate<=0) or (delta>=.4189 and rate>=0):
            rate=0.
        pre.append(delta);delta+=tick*rate
    return float(delta), np.asarray(buffer), float(np.mean(pre))


def mean_affine(delta, command, nominal_mean):
    """Hold 20 ms, slew for 80 ms; exact nominal mean via offset d.

    Within the reachable regime, ideal continuous mean is
    delta + .8*gap - gap*abs(gap)/(.64). Its command derivative is
    alpha=.8-abs(gap)/.32. At saturation alpha=0. We do not claim this
    derivative equals that of the discrete servo (which has jump surfaces).
    """
    alpha=max(0., .8-abs(command-delta)/.32)
    d=nominal_mean-(1-alpha)*delta-alpha*command
    return float(alpha),float(d)


class Actuator7Model(PaperLocalModel):
    def __init__(self,*args,**kwargs):
        super().__init__(*args,**kwargs)
        if not np.isclose(self.dt,.1,rtol=0,atol=1e-12):
            raise ValueError('A7 fixed actuator timing requires dt=.1')
        database=self.database
        # Keep command controls available for the unchanged exploration envelope.
        effective={}
        for lap in database.laps:
            for sid,value in zip(lap.sample_ids[:-1],lap.effective_inputs):
                if int(sid) in effective:
                    np.testing.assert_allclose(effective[int(sid)],value,rtol=0,atol=1e-12)
                effective[int(sid)]=value
        self.regression=object.__new__(PaperLocalModel)
        self.regression.__dict__=self.__dict__.copy()
        self.regression.controls=self.controls.copy()
        self.regression.controls[:,0]=[effective[int(i)][0] for i in self.source_ids]
        # Longitudinal regressor remains the recorded acceleration command.
        self.effective_steering=self.regression.controls[:,0].copy()

    def identify(self,z,u=None):
        theta,diag=self.regression.identify(np.asarray(z)[:6],u)
        diag.update(model='A7_five_coefficient_mean_actual_steering',state_dimension=7,
                    regression_columns=5,terminal_dimension=6,
                    selection_metric='unchanged velocities only: diag(.1,1,1,0,0,0,0)')
        for row in diag['paper_regressors'][1:]:row['input_column']='mean_delta_act'
        return theta,diag

    def evaluate_affine_mean(self,z,u,diag):
        z=np.asarray(z);u=np.asarray(u);theta=np.asarray(diag['coefficients'])
        alpha=diag['actuator']['alpha'];d=diag['actuator']['d']
        mean=(1-alpha)*z[6]+alpha*u[0]+d
        velocities=theta[:,:3]@z[:3]+theta[:,3]*np.array([u[1],mean,mean])+theta[:,4]
        return np.r_[velocities,kinematic_step(z[:6],self.geometry,self.dt),u[0]]

    def linearize_stage(self,z,u,buffer):
        theta,diag=self.identify(z,u)
        end,next_buffer,mean=replay_actuator(z[6],buffer,u[0])
        alpha,d=mean_affine(z[6],u[0],mean)
        diag['actuator']=dict(alpha=alpha,d=d,nominal_exact_mean_rad=mean,
            nominal_exact_endpoint_rad=end,nominal_endpoint_approximation_error_rad=float(u[0]-end),
            measured_or_nominal_buffer=np.asarray(buffer).tolist(),
            endpoint_transition='delta_next=delta_cmd; idealized approximation',
            mean_sensitivity='continuous hold/slew; frozen nominal FIFO offset')
        a=np.zeros((7,7));b=np.zeros((7,2))
        a[:3,:3]=theta[:,:3];a[1:3,6]=(1-alpha)*theta[1:3,3]
        b[0,1]=theta[0,3];b[1:3,0]=alpha*theta[1:3,3];b[6,0]=1.
        for i in range(6):
            h=1e-3 if i==4 else 1e-5;v=np.eye(6)[i]*h
            a[3:6,i]=(kinematic_step(z[:6]+v,self.geometry,self.dt)-kinematic_step(z[:6]-v,self.geometry,self.dt))/(2*h)
        value=self.evaluate_affine_mean(z,u,diag)
        return (a,b,value-a@z-b@u,diag),next_buffer


def build_atv_horizon(model,nominal_states,nominal_inputs,buffer):
    states=np.asarray(nominal_states);inputs=np.asarray(nominal_inputs)
    if (states.shape!=(len(inputs)+1,7) or inputs.shape!=(12,2)
            or not np.isfinite(states).all() or not np.isfinite(inputs).all()
            or np.asarray(buffer).shape!=(2,) or not np.isfinite(buffer).all()):
        raise ValueError('invalid seven-state horizon or two-tick FIFO')
    models=[]
    for k,u in enumerate(inputs):
        try:
            item,buffer=model.linearize_stage(states[k],u,buffer)
            models.append(item)
        except PaperIdentificationError as exc:
            exc.diagnostic['horizon_step']=k;raise
    return models
