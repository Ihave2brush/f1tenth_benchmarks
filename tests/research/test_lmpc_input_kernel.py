import unittest
import numpy as np
from f1tenth_benchmarks.research.lmpc.input_kernel_model import InputKernelModel
from f1tenth_benchmarks.research.lmpc.paper_controller import PaperLocalLMPC


class InputKernelTests(unittest.TestCase):
    def test_steering_deadzone_only_zeros_strictly_small_steering(self):
        from f1tenth_benchmarks.research.lmpc.input_kernel_controller import steering_zero_deadzone
        for angle in [-.00499,0.,.00499]:
            action=np.array([angle,2.3]);filtered=steering_zero_deadzone(action,.005)
            np.testing.assert_array_equal(filtered,[0.,2.3])
            self.assertEqual(action[0],angle)
        for angle in [-.005,.005,-.1,.1]:
            np.testing.assert_array_equal(steering_zero_deadzone([angle,-1.],.005),[angle,-1.])

    def model(self):
        m=object.__new__(InputKernelModel);rng=np.random.default_rng(123)
        m.states=rng.normal(0,.1,(160,6));m.states[:,0]+=2.5
        m.controls=rng.normal(0,.2,(160,2));m.source_ids=np.arange(160)
        m.selection_controls=m.controls.copy()
        m.dt=.1;m.diagnostic_only=True;m.steering_signal='delta_cmd'
        theta=np.array([[1.,.01,.02,.1,0.],[.01,.7,.1,.4,.02],[.02,5.,.6,7.,-.03]])
        m.targets=m.states[:,:3]@theta[:,:3].T+m.controls[:,[1,0,0]]*theta[:,3]+theta[:,4]
        return m,theta

    def test_selection_and_kernel_match_independent_L1_oracle(self):
        m,_=self.model();z=m.states[9];u=m.controls[9]
        idx,w,d=m.select(z,u)
        oracle=np.array([.1*abs(x[0]-z[0])+abs(x[1]-z[1])+abs(x[2]-z[2])+
            abs(v[0]-u[0])+abs(v[1]-u[1]) for x,v in zip(m.states,m.controls)])
        np.testing.assert_allclose(d,oracle,atol=1e-15)
        np.testing.assert_array_equal(idx,np.argsort(oracle)[:80])
        np.testing.assert_allclose(w,.75*(1-(oracle[idx]/15)**2))

    def test_WLS_recovers_known_coefficients_and_rejects_controller_install(self):
        m,truth=self.model();theta,diag=m.identify(m.states[9],m.controls[9])
        np.testing.assert_allclose(theta,truth,atol=1e-12)
        self.assertEqual(diag['points'],80)
        with self.assertRaisesRegex(ValueError,'diagnostic-only'):PaperLocalLMPC(None,m,{})

    def test_input_changes_neighbors_and_kernel_has_finite_support(self):
        m,_=self.model();z=m.states[9]
        left=m.select(z,[-.4,0])[0];right=m.select(z,[.4,0])[0]
        self.assertFalse(np.array_equal(left,right))
        with self.assertRaisesRegex(ValueError,'insufficient'):m.select(z,[30.,0])

    def test_A6_query_requires_real_angle_and_does_not_assume_command(self):
        m,_=self.model();m.steering_signal='delta_real'
        with self.assertRaises(ValueError):m.identify(m.states[9])
        with self.assertRaisesRegex(ValueError,'measured real'):m.identify(m.states[9],m.controls[9])

    def test_real_angle_changes_only_selection_not_regression(self):
        m,truth=self.model();original_controls=m.controls.copy()
        baseline=m.select(m.states[9],m.controls[9])[0]
        m.steering_signal='delta_real'
        m.selection_controls[:,0]=-m.controls[:,0]
        theta,diag=m.identify(m.states[9],m.controls[9],selection_steering=.4)
        self.assertFalse(np.array_equal(diag['source_ids'],baseline))
        np.testing.assert_array_equal(m.controls,original_controls)
        np.testing.assert_allclose(theta,truth,atol=1e-12)
        self.assertEqual(diag['regression_steering_signal'],'delta_cmd')

    def test_equal_selection_angles_produce_identical_cmd_and_real_fits(self):
        m,_=self.model();z=m.states[9];u=m.controls[9]
        original,diag=m.identify(z,u)
        m.steering_signal='delta_real'
        changed,real_diag=m.identify(z,u,selection_steering=u[0])
        np.testing.assert_array_equal(diag['source_ids'],real_diag['source_ids'])
        np.testing.assert_array_equal(original,changed)

    def test_nominal_real_angle_rollout_matches_original_actuator(self):
        from f1tenth_benchmarks.research.lmpc.input_kernel_controller import nominal_steering_angles
        from f1tenth_benchmarks.research.lmpc.dynamics import TracedDynamics
        from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
        params=load_parameter_file('simulator_params')
        commands=np.array([[.4189,0.],[.4189,0.],[-.2,0.],[-.2,0.],[0.,0.]])
        for buffer in ([],[-.0549,-.0549]):
            plant=TracedDynamics(params);plant.state[2]=-.064;plant.state[3]=2.5
            plant.steer_buffer=np.asarray(buffer);plant.input_mode='acceleration'
            expected=[]
            for command in commands:
                expected.append(plant.state[2])
                for _ in range(10):plant.update_pose(*command)
            predicted=nominal_steering_angles(-.064,buffer,commands,vars(params))
            np.testing.assert_allclose(predicted,expected,atol=1e-15,rtol=0)
            self.assertEqual(predicted[0],-.064)
            if buffer:self.assertAlmostEqual(predicted[1],.192)

    def test_closed_loop_configs_keep_A4_constraints_and_common_regression(self):
        from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3
        base=load_b3('params/research/lmpc_paper_a4.yaml')
        for name,signal in [('a5','delta_cmd'),('a6','delta_real')]:
            cfg=load_b3('params/research/lmpc_paper_'+name+'.yaml')
            self.assertEqual(cfg['settings'],base['settings'])
            self.assertEqual(cfg['exploration'],base['exploration'])
            self.assertEqual(cfg['kernel_selection']['selection_steering'],signal)
            self.assertEqual(cfg['kernel_selection']['regression_steering'],'delta_cmd')
