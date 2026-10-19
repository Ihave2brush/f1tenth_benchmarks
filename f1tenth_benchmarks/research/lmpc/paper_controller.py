"""Single-QP paper local LMPC, shifted trajectory with successor candidate end."""
from time import perf_counter
import numpy as np
from .controller import solve_qp, solve_finish_qp, MPCFailure
from .safe_set import initial_terminal_candidate, select_local_safe_set
from .paper_model import PaperIdentificationError, build_atv_horizon


def steering_exploration_bounds(model,models,multiplier,scope):
    """Symmetric command envelopes, intersected with original physical bounds."""
    if multiplier!=1.2 or scope not in ('local_80','global_history'):
        raise ValueError('invalid steering exploration policy')
    result=[];details=[]
    for k,(_,_,_,diag) in enumerate(models):
        ids=(np.asarray(diag['source_ids']) if scope=='local_80' else model.source_ids)
        idx=np.searchsorted(model.source_ids,ids)
        if not np.array_equal(model.source_ids[idx],ids):raise ValueError('unknown steering source IDs')
        commands=model.controls[idx,0]
        historical=float(np.max(abs(commands)));cap=min(.4189,multiplier*historical)
        result.append([-cap,cap])
        details.append(dict(stage=k,source_count=len(ids),historical_max_abs_command_rad=historical,
            historical_signed_range_rad=[float(commands.min()),float(commands.max())],
            untruncated_cap_rad=multiplier*historical,effective_cap_rad=cap))
    return result,details


class PaperLocalLMPC:
    def __init__(self,database,model,settings):
        if getattr(model,'diagnostic_only',False):
            raise ValueError('diagnostic-only regression cannot be used to control the vehicle')
        self.database,self.model,self.settings=database,model,settings
        self.candidate=initial_terminal_candidate(database,cycle=0)
        self.previous=None; self.previous_input=np.zeros(2)

    def set_first_horizon(self,states,inputs,previous_input):
        """Explicit first seed for a moving handoff; not a previous optimum."""
        states=np.asarray(states,dtype=float);inputs=np.asarray(inputs,dtype=float)
        previous_input=np.asarray(previous_input,dtype=float)
        if (self.previous is not None or states.shape!=(13,6) or inputs.shape!=(12,2)
                or previous_input.shape!=(2,) or not np.isfinite(states).all()
                or not np.isfinite(inputs).all() or not np.isfinite(previous_input).all()):
            raise ValueError('invalid first horizon seed')
        self.first_horizon=(states.copy(),inputs.copy())
        self.previous_input=previous_input.copy()
        self.candidate=states[-1].copy()

    def plan(self,initial):
        begin=perf_counter(); terminal=select_local_safe_set(self.database,self.candidate,cycle=0)
        if self.previous is None:
            # IV-C does not specify the first-ever horizon seed; use recorded
            # standing-start trajectory, not latest flying states after reset.
            if hasattr(self,'first_horizon'):
                nominal,inputs=[v.copy() for v in self.first_horizon]
            else:
                lap=self.database.laps[0]
                nominal=lap.states[:13].copy(); inputs=lap.commands[:12].copy()
        else:
            old_states,old_inputs=self.previous
            nominal=np.vstack([old_states[1:],self.candidate])
            inputs=np.vstack([old_inputs[1:],self.previous_terminal_input])
        try:
            models=build_atv_horizon(self.model,nominal,inputs)
        except PaperIdentificationError as exc:
            raise MPCFailure(str(exc),dict(failure_kind='unidentifiable_paper_input',
                horizon_step=exc.diagnostic['horizon_step'],candidate=self.candidate.tolist(),
                identification=exc.diagnostic, qp_attempts=0)) from exc
        except ValueError as exc:
            raise MPCFailure('ATV model construction: '+str(exc),dict(failure_kind='model_construction',
                candidate=self.candidate.tolist(),nominal_states=nominal.tolist(),qp_attempts=0)) from exc
        steering_details=None
        if 'historical_steering_multiplier' in self.settings:
            bounds,steering_details=steering_exploration_bounds(self.model,models,
                self.settings['historical_steering_multiplier'],self.settings['steering_scope'])
            self.settings['exploration_steering_bounds_rad']=bounds
        if self.settings.get('vx_scope')=='local_80':
            caps=[]
            for k,(_,_,_,diag) in enumerate(models):
                idx=np.searchsorted(self.model.source_ids,diag['source_ids'])
                historical=float(np.max(self.model.states[idx,0]))
                cap=min(self.settings['max_speed_mps'],1.2*historical)
                caps.append(cap)
                steering_details[k].update(historical_max_vx_mps=historical,effective_vx_cap_mps=cap,
                    constrained_state_node=k+1)
            self.settings['exploration_vx_limits_mps']=caps
        try:
            if self.settings.get('finish_cost_mode')=='literal_indicator':
                states,actions,lam,qp=solve_finish_qp(initial,nominal,inputs,models,terminal,
                    self.previous_input,self.settings,self.database.length)
            else:
                states,actions,lam,qp=solve_qp(initial,nominal,inputs,models,terminal,self.previous_input,self.settings)
        except MPCFailure as exc:
            exc.diagnostic.update(candidate=self.candidate.tolist(),identification=[m[3] for m in models],sqp_iteration=0)
            if steering_details is not None:exc.diagnostic['steering_exploration']=steering_details
            raise
        nonlinear=np.array([self.model.evaluate(states[k],actions[k],np.array(models[k][3]['coefficients'])) for k in range(12)])
        qp.update(sqp_iteration=0,nonlinear_defect=np.max(abs(nonlinear-states[1:]),axis=0).tolist())
        self.previous=(states,actions);self.previous_input=actions[0].copy()
        # Input at S_i must be u_(i+1), not the command at D_i.
        historical_controls=[]
        for p in terminal.provenance:
            lap=next(l for l in self.database.laps if l.lap_id==p['lap_id'])
            pool=lap.terminal_pool(p['cycle']);i=p['local_index']+1
            if i>=len(pool['commands']):raise ValueError('terminal successor lacks outgoing nominal input')
            historical_controls.append(pool['commands'][i])
        historical_controls=np.asarray(historical_controls)
        self.previous_terminal_input=historical_controls.T@lam
        old_candidate=self.candidate.copy();self.candidate=terminal.advance(lam)['candidate']
        return actions[0],dict(states=states.tolist(),controls=actions.tolist(),lambda_weights=lam.tolist(),
            steering_exploration=steering_details,
            terminal_sources=list(terminal.provenance),terminal_candidate=old_candidate.tolist(),
            next_terminal_candidate=self.candidate.tolist(),qp=[qp],identification=[m[3] for m in models],
            prediction_models=[dict(A=m[0].tolist(),B=m[1].tolist(),C=m[2].tolist()) for m in models],
            nominal_stage_cost=int(np.sum(states[:-1,4]<self.database.length)),
            stage_cost_policy=self.settings.get('finish_cost_mode','legacy'),
            warm_start_mode='paper_shifted_solution_plus_successor_candidate',model_and_qp_time_s=perf_counter()-begin)
