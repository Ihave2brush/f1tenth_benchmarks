"""A5/A6 closed-loop adapters; real angle affects ONLY neighbor selection."""
import numpy as np
from .input_kernel_model import InputKernelModel
from .paper_controller import PaperLocalLMPC
from .dynamics import pid,steering_constraint


def steering_zero_deadzone(action,threshold):
    action=np.asarray(action,dtype=float).copy()
    if action.shape!=(2,) or not np.isfinite(action).all() or not np.isfinite(threshold) or threshold<=0:
        raise ValueError('invalid steering deadzone/action')
    if abs(action[0])<threshold:action[0]=0.
    return action


def nominal_steering_angles(angle,buffer,controls,params,dt=.1):
    """Exact original steering-only substeps along the nominal command sequence."""
    angle=float(angle);buffer=np.asarray(buffer,dtype=float).copy();result=[]
    steps=round(dt/params['timestep'])
    if abs(steps*params['timestep']-dt)>1e-12:raise ValueError('noninteger steering substeps')
    for command in controls:
        result.append(angle)
        for _ in range(steps):
            delayed=0. if len(buffer)<2 else float(buffer[-1])
            if len(buffer)>=2:buffer=buffer[:-1]
            buffer=np.r_[float(command[0]),buffer]
            _,rate=pid(0.,delayed,0.,angle,params['sv_max'],params['a_max'],params['v_max'],params['v_min'])
            rate=steering_constraint(angle,rate,params['s_min'],params['s_max'],params['sv_min'],params['sv_max'])
            angle+=params['timestep']*rate
    return np.asarray(result)


class ClosedLoopInputKernelModel(InputKernelModel):
    def __init__(self,database,geometry,steering_signal):
        super().__init__(database,geometry,steering_signal)
        self.diagnostic_only=False
        self.selection_horizon=None;self.selection_index=0

    def prepare(self,angles):
        angles=np.asarray(angles,dtype=float)
        if angles.shape!=(12,) or not np.isfinite(angles).all():raise ValueError('invalid nominal actual angles')
        self.selection_horizon=angles.copy();self.selection_index=0

    def identify(self,z,u=None,selection_steering=None):
        if self.selection_horizon is None or self.selection_index>=12:
            raise ValueError('measured actuator context must prepare each horizon')
        angle=float(self.selection_horizon[self.selection_index])
        theta,diag=super().identify(z,u,selection_steering=angle if self.steering_signal=='delta_real' else None)
        diag.update(diagnostic_only=False,horizon_step=self.selection_index,
            nominal_actual_steering_rad=angle,selection_query_steering_rad=angle if self.steering_signal=='delta_real' else float(u[0]),
            nominal_command=np.asarray(u).tolist(),measured_only=True,state_dimension=6)
        return theta,diag

    def linearize(self,z,u):
        result=super().linearize(z,u)
        self.selection_index+=1
        return result


class InputKernelLMPC(PaperLocalLMPC):
    def plan_with_actuator(self,initial,physical,buffer,params):
        controls=(self.first_horizon[1] if self.previous is None else
                  np.vstack([self.previous[1][1:],self.previous_terminal_input]))
        angles=nominal_steering_angles(physical[2],buffer,controls,params)
        self.model.prepare(angles)
        action,record=super().plan(initial)
        record['nominal_actual_steering_rad']=angles.tolist()
        record['selection_actual_angle_policy']='measured at k0; exact steering-only rollout along nominal commands at k1..11; no future measurement'
        return action,record
