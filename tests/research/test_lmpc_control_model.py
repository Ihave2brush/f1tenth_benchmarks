"""B2.5 checks of body/frame integration and measured-state warm starts."""
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np

from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
from f1tenth_benchmarks.research.lmpc.dynamics import TracedDynamics
from f1tenth_benchmarks.research.lmpc.local_model import (
    physical_step, geometric_kinematic_step, augmented,
)
from f1tenth_benchmarks.research.lmpc.controller import LocalLMPC, MPCFailure


class ControlModelTests(unittest.TestCase):
    def test_geometric_pose_matches_original_plant_in_rotated_frame(self):
        params=load_parameter_file('simulator_params')
        plant=TracedDynamics(params)
        plant.state=np.array([10.,.1,.032,1.2,.2,.2,.01])
        plant.steer_buffer=np.array([.07,-.03])
        z=augmented([1.2*np.cos(.01),1.2*np.sin(.01),.2,.2,10.,.1],plant.state,plant.steer_buffer)
        action=np.array([.04,.2])
        _, physical=physical_step(z,action,vars(params),.1,return_physical=True)
        geometry=SimpleNamespace(curve=SimpleNamespace(L=200.),
            reference=lambda s:dict(xy=[s,0.],tangent=[1.,0.],psi=0.),
            to_frenet=lambda xy,yaw,**kw:dict(valid=True,s_wrapped=xy[0],d=xy[1],e_psi=yaw))
        predicted=geometric_kinematic_step(z,physical,geometry)
        plant.input_mode='acceleration'
        for _ in range(10): plant.update_pose(*action)
        np.testing.assert_allclose(predicted,plant.state[[4,0,1]],atol=1e-12,rtol=0)

    def test_geometric_projection_refuses_invalid_domain(self):
        geometry=SimpleNamespace(curve=SimpleNamespace(L=200.),
            reference=lambda s:dict(xy=[s,0.],tangent=[1.,0.],psi=0.),
            to_frenet=lambda *a,**kw:dict(valid=False,reason='outside_track'))
        with self.assertRaisesRegex(ValueError,'outside_track'):
            geometric_kinematic_step(np.zeros(9),np.zeros(7),geometry)

    def test_measured_reroll_has_no_dynamics_defect_after_state_disturbance(self):
        initial=np.zeros(9); initial[0]=1.2; initial[5]=.1
        old_states=np.zeros((13,9)); old_inputs=np.tile([.01,.1],(12,1))
        def evaluate(z,u,fit=None):
            next_state=z.copy(); next_state[4]+=.1*z[0]; next_state[5]+=.01*u[0]
            return next_state
        model=SimpleNamespace(evaluate=evaluate,
            identify=lambda z,u:(np.zeros((9,3)),np.r_[z[:3],z[6:9],u],{}),
            linearize=lambda z,u:(np.eye(9),np.zeros((9,2)),np.zeros(9),{}))
        database=SimpleNamespace(laps=[SimpleNamespace(states=np.zeros((1,6)))])
        checked=[]
        def check(initial,nominal,inputs,*args):
            checked.append(max(np.max(abs(evaluate(nominal[k],inputs[k])-nominal[k+1])) for k in range(12)))
            raise MPCFailure('diagnostic stop',{})
        with patch('f1tenth_benchmarks.research.lmpc.controller.initial_terminal_candidate',return_value=np.zeros(6)), \
             patch('f1tenth_benchmarks.research.lmpc.controller.select_local_safe_set',return_value=None), \
             patch('f1tenth_benchmarks.research.lmpc.controller.solve_qp',side_effect=check):
            for mode in ['shifted','reroll_measured']:
                controller=LocalLMPC(database,model,dict(sqp_iterations=2,warm_start_mode=mode))
                controller.previous=(old_states,old_inputs)
                with self.assertRaises(MPCFailure): controller.plan(initial)
        self.assertGreater(checked[0],.1)
        self.assertLess(checked[1],1e-12)


if __name__=='__main__':unittest.main()
