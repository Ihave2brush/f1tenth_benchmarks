"""Independent checks for A7 dimensions, actuator timing and terminal boundary."""
import unittest
from types import SimpleNamespace
import numpy as np
from f1tenth_benchmarks.research.lmpc.actuator7_model import Actuator7Model, replay_actuator, mean_affine
from f1tenth_benchmarks.research.lmpc.actuator7_qp import solve_finish_qp
from f1tenth_benchmarks.research.lmpc.paper_model import PaperLocalModel
from f1tenth_benchmarks.research.lmpc.safe_set import LocalSafeSet
from f1tenth_benchmarks.research.lmpc.run_actuator7 import load_b3
from f1tenth_benchmarks.research.lmpc.dynamics import TracedDynamics
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file

class Actuator7Tests(unittest.TestCase):
    def test_discrete_replay_matches_original_including_reversal_and_bounds(self):
        p=load_parameter_file('simulator_params');plant=TracedDynamics(p)
        plant.state[3]=2.5;plant.steer_buffer=np.array([.1,.1]);plant.input_mode='acceleration'
        delta=plant.state[2];buffer=plant.steer_buffer.copy()
        for command in [.15,-.2,.4,-.4189,0.,.001]:
            delta,buffer,mean=replay_actuator(delta,buffer,command)
            plant.trace=[]
            for _ in range(10):plant.update_pose(command,0.)
            self.assertEqual(delta,plant.state[2]);np.testing.assert_array_equal(buffer,plant.steer_buffer)
            self.assertAlmostEqual(mean,np.mean([t['pre_state'][2] for t in plant.trace]),places=15)

    def test_reachability_does_not_make_endpoint_exact(self):
        end,_,_=replay_actuator(0.,[0.,0.],.01)
        self.assertLess(abs(.01),.256)
        self.assertGreater(abs(end-.01),.001)

    def test_split_and_row_seven_with_five_regressors(self):
        rng=np.random.RandomState(81);reg=object.__new__(PaperLocalModel)
        reg.states=rng.normal(0,.05,(120,6));reg.states[:,0]+=2.
        reg.controls=rng.normal(0,.05,(120,2));reg.source_ids=np.arange(120);reg.dt=.1
        truth=np.array([[.9,.1,0.,.1,0.],[0.,.8,.1,.2,0.],[0.,.2,.7,.4,0.]])
        reg.targets=reg.states[:,:3]@truth[:,:3].T+reg.controls[:,[1,0,0]]*truth[:,3]+truth[:,4]
        m=object.__new__(Actuator7Model);m.regression=reg;m.dt=.1
        m.geometry=SimpleNamespace(reference=lambda s:{'kappa':0.})
        z=np.r_[reg.states[4],.02];u=np.array([.08,.1])
        (a,b,c,d),_=m.linearize_stage(z,u,[0.,0.])
        self.assertEqual(a.shape,(7,7));self.assertEqual(b.shape,(7,2))
        self.assertEqual(np.asarray(d['coefficients']).shape,(3,5))
        alpha=d['actuator']['alpha']
        np.testing.assert_allclose(a[1:3,6],(1-alpha)*truth[1:3,3],atol=1e-12)
        np.testing.assert_allclose(b[1:3,0],alpha*truth[1:3,3],atol=1e-12)
        np.testing.assert_array_equal(a[6],np.zeros(7));np.testing.assert_array_equal(b[6],[1.,0.])
        np.testing.assert_allclose(a@z+b@u+c,m.evaluate_affine_mean(z,u,d),atol=1e-12)
        x,y=mean_affine(.02,.08,d['actuator']['nominal_exact_mean_rad'])
        self.assertAlmostEqual((1-x)*.02+x*.08+y,d['actuator']['nominal_exact_mean_rad'])

    def test_qp_terminal_keeps_six_rows_and_reach_bound_all_stages(self):
        cfg=load_b3('params/research/lmpc_actuator7.yaml');settings=cfg['settings']
        x=np.array([1.,0.,0.,0.,0.,0.,.1]);nom=np.tile(x,(13,1));u=np.tile([.1,0.],(12,1))
        a=np.eye(7);a[6]=0.;b=np.zeros((7,2));b[6,0]=1.
        models=[(a,b,np.zeros(7),{})]*12
        d=np.tile(x[:6,None],(1,40))
        terminal=LocalSafeSet(d,d,np.ones(40),np.zeros(40),(),'',.1)
        xs,us,lam,info=solve_finish_qp(x,nom,u,models,terminal,np.array([.1,0.]),settings,100.)
        self.assertLess(np.max(abs(xs[-1,:6]-d@lam)),1e-5)
        self.assertAlmostEqual(xs[-1,6],.1,places=6)
        self.assertLess(np.max(abs(us[:,0]-xs[:-1,6])),.256+1e-7)
        self.assertEqual(info['finish_screened_stages'],12)
        # Activate the bound: the previous command is .4, actual angle is zero.
        # A seventh terminal equality would unnecessarily force the endpoint.
        x[6]=0.;u[:,0]=.4;nom[:,6]=.4;nom[0,6]=0.
        xs,us,lam,info=solve_finish_qp(x,nom,u,models,terminal,np.array([.4,0.]),settings,100.)
        self.assertAlmostEqual(us[0,0],.256,places=5)
        self.assertGreater(xs[-1,6],.25)
        self.assertLess(np.max(abs(us[:,0]-xs[:-1,6])),.256+1e-7)


if __name__=='__main__':unittest.main()
