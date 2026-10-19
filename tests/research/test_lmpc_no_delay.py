"""A8: remove FIFO only, preserve six-state prediction and physical rate limit."""
from pathlib import Path
import unittest
import numpy as np
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
from f1tenth_benchmarks.simulator.dynamic_models import pid,vehicle_dynamics_st
from f1tenth_benchmarks.research.lmpc.dynamics import TracedDynamics
from f1tenth_benchmarks.research.lmpc.no_delay import NoDelayTracedDynamics, replay
from f1tenth_benchmarks.research.lmpc.no_delay_scope import ScopedPlanningGate
from f1tenth_benchmarks.research.lmpc.run_no_delay import load_b3

class NoDelayTests(unittest.TestCase):
    def test_current_command_immediate_but_rate_limited(self):
        params=load_parameter_file('simulator_params')
        old=TracedDynamics(params);new=NoDelayTracedDynamics(params)
        old.update_pose(.2,0.);new.update_pose(.2,0.)
        self.assertEqual(old.trace[-1]['delayed_target_steering'],0.)
        self.assertEqual(new.trace[-1]['delayed_target_steering'],.2)
        self.assertAlmostEqual(new.state[2],.032)
        self.assertEqual(old.state[2],0.)
        new.update_pose(-.2,0.)
        self.assertEqual(new.trace[-1]['delayed_target_steering'],-.2)
        self.assertAlmostEqual(new.state[2],0.)

    def test_matches_direct_original_vehicle_oracle_without_fifo(self):
        params=load_parameter_file('simulator_params');plant=NoDelayTracedDynamics(params)
        plant.state[3]=2.5;plant.steer_buffer=np.array([-.4,.4])
        oracle=plant.state.copy();p=vars(params)
        for command in [.1,-.2,.4189,0.,-.001]*4:
            a,sv=pid(2.5,command,oracle[3],oracle[2],p['sv_max'],p['a_max'],p['v_max'],p['v_min'])
            derivative=vehicle_dynamics_st(oracle,np.array([sv,a]),*[p[k] for k in
                ['mu','C_Sf','C_Sr','lf','lr','h','m','I','s_min','s_max','sv_min','sv_max','v_switch','a_max','v_min','v_max']])
            oracle=oracle+p['timestep']*derivative
            if oracle[4]>2*np.pi:oracle[4]-=2*np.pi
            elif oracle[4]<0:oracle[4]+=2*np.pi
            plant.update_pose(command,2.5)
            np.testing.assert_array_equal(plant.state,oracle)
            self.assertLessEqual(abs(plant.trace[-1]['applied_steering_rate']),3.2)

    def test_no_delay_replay_accepts_acceleration_trace(self):
        params=load_parameter_file('simulator_params');plant=NoDelayTracedDynamics(params)
        plant.reset([0.,0.,0.]);plant.input_mode='acceleration';rows=[]
        for steer in [.1,-.2,.001]:
            plant.trace=[]
            for _ in range(10):plant.update_pose(steer,.3)
            rows.append(dict(physics_ticks=plant.trace,physical_state=plant.state.tolist(),steering_buffer=plant.steer_buffer.tolist()))
        result=replay(rows,vars(params),[0.,0.,0.]);self.assertTrue(result['passed'])
        self.assertEqual(result['physics_ticks'],30)

    def test_shadow_uses_same_no_delay_actuator_and_six_state_candidate(self):
        import f1tenth_benchmarks.research.lmpc.no_delay_scope as scope
        self.assertIs(scope.TracedDynamics,NoDelayTracedDynamics)
        config=load_b3('params/research/lmpc_paper_a8.yaml')
        self.assertEqual(config['experiment_id'],'A8')
        self.assertNotIn('initialization_source',config)
        self.assertEqual(config['prediction_value_mode'],'paper_direct')
        from f1tenth_benchmarks.research.lmpc import run_no_delay as runner
        self.assertIs(runner.replay,replay)
        # Controller and learned model imports in this runner are baseline six-state.
        self.assertNotIn('actuator7_',Path(runner.__file__).read_text())

if __name__=='__main__':unittest.main()
