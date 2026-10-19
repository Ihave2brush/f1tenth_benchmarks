"""Steering actuator invariants; no controller optimization or LMPC driving."""
from argparse import Namespace
import unittest
import numpy as np

from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from f1tenth_benchmarks.simulator.dynamic_models import vehicle_dynamics_st
from f1tenth_benchmarks.research.lmpc.dynamics import TracedDynamics
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file


def params(mode=None, **overrides):
    values=vars(load_parameter_file('simulator_params')).copy()
    if mode is not None:values['steering_actuator_mode']=mode
    values.update(overrides)
    return Namespace(**values)


class SteeringActuatorTests(unittest.TestCase):
    def test_missing_mode_matches_explicit_original_full_state(self):
        left= DynamicsSimulator(params());right=DynamicsSimulator(params('original'))
        rng=np.random.default_rng(20261019)
        for _ in range(1000):
            command=[rng.uniform(-.7,.7),rng.uniform(-1.,3.)]
            np.testing.assert_array_equal(left.update_pose(*command),right.update_pose(*command))
            np.testing.assert_array_equal(left.steer_buffer,right.steer_buffer)

    def test_micro_command_reaches_target_without_alternating(self):
        for target in [-.00010438870829536026,.00010438870829536026,-.001,.001,-.031,.031]:
            with self.subTest(target=target):
                sim=DynamicsSimulator(params('non_overshoot'))
                values=[sim.update_pose(target,0.)[2] for _ in range(30)]
                np.testing.assert_array_equal(values[:2],[0.,0.])
                np.testing.assert_allclose(values[2:],target,rtol=0,atol=1e-18)

    def test_original_micro_command_still_oscillates(self):
        sim=DynamicsSimulator(params('original'))
        values=[sim.update_pose(-.00010438870829536026,0.)[2] for _ in range(10)]
        np.testing.assert_array_equal(values,[0.,0.,-.032,0.,-.032,0.,-.032,0.,-.032,0.])

    def test_original_and_fixed_differ_only_after_delayed_target_consumed(self):
        old=DynamicsSimulator(params('original'));new=DynamicsSimulator(params('non_overshoot'))
        for _ in range(2):np.testing.assert_array_equal(old.update_pose(.001,2.5),new.update_pose(.001,2.5))
        old.update_pose(.001,2.5);new.update_pose(.001,2.5)
        self.assertEqual(old.state[2],.032);self.assertEqual(new.state[2],.001)
        np.testing.assert_array_equal(old.steer_buffer,new.steer_buffer)

    def test_deadband_is_preserved_in_new_mode(self):
        for target in [-.0001,-.00005,0.,.00005,.0001]:
            sim=DynamicsSimulator(params('non_overshoot'))
            for _ in range(20):sim.update_pose(target,0.)
            self.assertEqual(sim.state[2],0.)

    def test_large_positive_negative_targets_are_monotone_and_rate_limited(self):
        sim=DynamicsSimulator(params('non_overshoot'))
        for target in [.4,-.4,.4189,-.4189,0.]:
            previous=sim.state[2]
            for tick in range(60):
                post=sim.update_pose(target,0.)[2]
                self.assertLessEqual(abs(post-previous),.032+1e-15)
                if tick>=2:
                    self.assertGreaterEqual(post,min(previous,target))
                    self.assertLessEqual(post,max(previous,target))
                previous=post
            self.assertAlmostEqual(post,target,places=14)

    def test_out_of_range_commands_use_bounded_targets_and_do_not_escape_limits(self):
        sim=TracedDynamics(params('non_overshoot'))
        for raw in [10.,-10.,.4189,-.4189,0.]:
            for _ in range(80):
                sim.update_pose(raw,0.)
                self.assertGreaterEqual(sim.state[2],-.4189);self.assertLessEqual(sim.state[2],.4189)
                effective=sim.trace[-1]['effective_target_steering']
                self.assertGreaterEqual(effective,-.4189);self.assertLessEqual(effective,.4189)
            self.assertAlmostEqual(sim.state[2],np.clip(raw,-.4189,.4189))

    def test_near_physical_limits_does_not_cross_them(self):
        for initial,target in [(.4188,.4189),(-.4188,-.4189),(.41885,1.),(-.41885,-1.)]:
            sim=DynamicsSimulator(params('non_overshoot'));sim.state[2]=initial
            sim.steer_buffer=np.full(2,target)
            post=sim.update_pose(target,0.)[2]
            self.assertTrue(-.4189<=post<=.4189)
            self.assertLessEqual(abs(post-initial),.032)

    def test_fifo_retains_two_ticks_and_raw_command_order(self):
        sim=TracedDynamics(params('non_overshoot'))
        sequence=[.01,-.02,.03,-.04,.05]
        for i,command in enumerate(sequence):
            sim.update_pose(command,0.)
            self.assertEqual(sim.trace[-1]['delayed_target_steering'],sequence[i-2] if i>=2 else 0.)
            np.testing.assert_array_equal(sim.steer_buffer,list(reversed(sequence[max(0,i-1):i+1])))

    def test_continuous_reversals_never_cross_consumed_target(self):
        sim=TracedDynamics(params('non_overshoot'))
        sequence=[.011,-.009,.19,-.23,.4189,-.4189,1.,-1.]*40
        for command in sequence:
            pre=sim.state[2];sim.update_pose(command,0.);tick=sim.trace[-1];target=tick['effective_target_steering']
            self.assertGreaterEqual(sim.state[2],min(pre,target));self.assertLessEqual(sim.state[2],max(pre,target))
            self.assertLessEqual(abs(sim.state[2]-pre),.032+1e-15)

    def test_invariants_across_timesteps_and_asymmetric_rate_limits(self):
        rng=np.random.default_rng(7)
        for dt in [.001,.01,.04]:
            sim=TracedDynamics(params('non_overshoot',timestep=dt,sv_min=-1.7,sv_max=3.2))
            for command in rng.uniform(-1.,1.,300):
                pre=sim.state[2];sim.update_pose(command,0.);target=sim.trace[-1]['effective_target_steering']
                rate=(sim.state[2]-pre)/dt
                self.assertGreaterEqual(rate,-1.7-1e-12);self.assertLessEqual(rate,3.2+1e-12)
                self.assertTrue(-.4189<=sim.state[2]<=.4189)
                self.assertTrue(min(pre,target)<=sim.state[2]<=max(pre,target))

    def test_rate_cap_is_applied_before_low_speed_vehicle_derivative(self):
        sim=DynamicsSimulator(params('non_overshoot'));sim.state[3]=.3;sim.steer_buffer=np.full(2,.001)
        pre=sim.state.copy();p=sim.params
        derivative=vehicle_dynamics_st(pre,np.array([.1,0.]),*[p[k] for k in
            ['mu','C_Sf','C_Sr','lf','lr','h','m','I','s_min','s_max','sv_min','sv_max','v_switch','a_max','v_min','v_max']])
        expected=pre+.01*derivative
        np.testing.assert_allclose(sim.update_pose(.001,.3),expected,rtol=0,atol=1e-16)
        self.assertGreater(sim.state[5],0.)

    def test_tracing_matches_uninstrumented_in_both_modes(self):
        for mode in ['original','non_overshoot']:
            base=DynamicsSimulator(params(mode));trace=TracedDynamics(params(mode))
            for target in [.001,-.001,.4,-.4,10.,-10.]*10:
                np.testing.assert_array_equal(base.update_pose(target,1.2),trace.update_pose(target,1.2))
                tick=trace.trace[-1]
                self.assertAlmostEqual((tick['post_state'][2]-tick['pre_state'][2])/.01,tick['applied_steering_rate'],places=12)
                np.testing.assert_array_equal(base.steer_buffer,trace.steer_buffer)

    def test_acceleration_interface_keeps_requested_acceleration(self):
        for mode in ['original','non_overshoot']:
            sim=TracedDynamics(params(mode));sim.input_mode='acceleration'
            for acceleration in [-.3,0.,.3,1.]:
                sim.state[3]=2.5;sim.update_pose(.001,acceleration)
                self.assertAlmostEqual(sim.trace[-1]['applied_acceleration'],acceleration,places=12)

    def test_reset_clears_fifo_and_retains_selected_mode(self):
        sim=DynamicsSimulator(params('non_overshoot'))
        for _ in range(4):sim.update_pose(.1,0.)
        sim.reset([1.,2.,.3]);self.assertEqual(sim.steering_actuator_mode,'non_overshoot')
        self.assertEqual(len(sim.steer_buffer),0);self.assertEqual(sim.state[2],0.)
        sim.update_pose(.1,0.);self.assertEqual(sim.state[2],0.)

    def test_invalid_modes_and_new_mode_parameters_are_rejected(self):
        with self.assertRaises(ValueError):DynamicsSimulator(params('unknown'))
        for override in [dict(timestep=0.),dict(timestep=float('nan')),dict(sv_min=0.),dict(sv_max=-1.),dict(s_min=.1)]:
            with self.subTest(override=override),self.assertRaises(ValueError):DynamicsSimulator(params('non_overshoot',**override))

    def test_invalid_new_mode_wheel_state_and_command_are_rejected(self):
        sim=DynamicsSimulator(params('non_overshoot'))
        with self.assertRaises(ValueError):sim.update_pose(float('nan'),0.)
        sim.state[2]=.5
        with self.assertRaises(ValueError):sim.update_pose(0.,0.)


if __name__=='__main__':unittest.main()
