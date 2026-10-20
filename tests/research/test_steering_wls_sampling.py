"""Sampling-policy and data-interface tests, without WLS fitting."""
import copy
import unittest
import numpy as np
from f1tenth_benchmarks.research.collect_steering_wls import (
    DEFAULT_POLICY, speed_reference, compose_command, state_safety, quality_gate, alignment_verified,
)
from f1tenth_benchmarks.research.lmpc.initialization import ControlledExcitation


class SteeringWLSSamplingTests(unittest.TestCase):
    def excitation(self,seed):return ControlledExcitation(dict(seed=seed,**DEFAULT_POLICY['excitation']))

    def test_training_and_heldout_speed_schedules(self):
        p=DEFAULT_POLICY
        self.assertEqual(speed_reference('train',0,0,0,100,p),2.5)
        self.assertEqual(speed_reference('train',1,101,80,100,p),3.)
        for s,expected in [(0,2.5),(24.99,2.5),(25,3.),(49.9,3.),(50,2.5),(75,3.),(-1e-13,2.5)]:
            self.assertEqual(speed_reference('test',0,s,0,100,p),expected)

    def test_short_and_qualification_schedules(self):
        p=DEFAULT_POLICY
        self.assertEqual(speed_reference('short_validation',0,0,0,100,p),2.5)
        self.assertEqual(speed_reference('short_validation',0,10,6,100,p),3.)
        self.assertEqual(speed_reference('high_speed_qualification',0,0,0,100,p),3.)
        with self.assertRaises(ValueError):speed_reference('unknown',0,0,0,100,p)

    def test_same_role_seed_produces_identical_held_offsets(self):
        a=self.excitation(DEFAULT_POLICY['role_seeds']['train']);b=self.excitation(DEFAULT_POLICY['role_seeds']['train'])
        values=[]
        for i in range(40):
            a.apply([.03,.1],.4,9.51);b.apply([-.1,-.2],.4,9.51)
            np.testing.assert_array_equal(a.offset,b.offset);values.append(a.offset.copy())
            self.assertLessEqual(abs(a.offset[0]),.02);self.assertLessEqual(abs(a.offset[1]),.6)
        for start in range(0,40,5):np.testing.assert_array_equal(values[start:start+5],np.tile(values[start],(5,1)))

    def test_train_test_and_prechecks_have_different_seeds(self):
        seeds=DEFAULT_POLICY['role_seeds'];self.assertEqual(len(set(seeds.values())),4)
        a=self.excitation(seeds['train']);b=self.excitation(seeds['test']);a.apply([0,0],.4,9.51);b.apply([0,0],.4,9.51)
        self.assertFalse(np.array_equal(a.offset,b.offset))

    def test_command_second_column_is_acceleration_not_target_speed(self):
        exc=self.excitation(7)
        command,detail=compose_command([.03,3.],2.9,exc,.1,.4,9.51)
        self.assertAlmostEqual(detail['baseline_acceleration_command'],1.)
        self.assertAlmostEqual(command[1],1.+exc.offset[1])
        self.assertEqual(detail['pp_target_speed'],3.)
        self.assertNotEqual(command[1],3.)

    def test_clip_is_recorded_without_rewriting_raw_offsets(self):
        exc=self.excitation(7)
        command,detail=compose_command([.4,3.],0.,exc,.1,.4,9.51)
        self.assertTrue(detail['clipped']);self.assertTrue(np.all(abs(command)<=[.4,9.51]))
        np.testing.assert_allclose(detail['unclipped_command'],[.4,9.51]+exc.offset)

    def test_safety_is_observation_only_and_uses_frozen_thresholds(self):
        p=DEFAULT_POLICY;physical=np.zeros(7);x=np.array([3.,0,0,0,0,0.])
        self.assertIsNone(state_safety(x,physical,p))
        for column,value,reason in [(5,.251,'lateral_limit'),(3,.501,'heading_limit'),(0,3.301,'speed_limit')]:
            candidate=x.copy();candidate[column]=value
            self.assertEqual(state_safety(candidate,physical,p),reason)
        self.assertEqual(state_safety(None,physical,p),'invalid_state_or_frenet')

    def test_failed_prerequisite_blocks_later_stages(self):
        good=dict(completed_target=True,safe=True,alignment_passed=True,excitation_passed=True,near_high_speed_samples=100,near_low_speed_samples=100)
        self.assertTrue(quality_gate(good,'train',DEFAULT_POLICY)[0])
        for key in ['completed_target','safe','alignment_passed','excitation_passed']:
            bad=copy.deepcopy(good);bad[key]=False
            passed,reasons=quality_gate(bad,'high_speed_qualification',DEFAULT_POLICY)
            self.assertFalse(passed);self.assertIn(key,reasons)
        bad=copy.deepcopy(good);bad['near_high_speed_samples']=49
        self.assertFalse(quality_gate(bad,'high_speed_qualification',DEFAULT_POLICY)[0])

    def test_common_support_policy_is_not_A5_kernel_replacement(self):
        p=DEFAULT_POLICY['common_support']
        self.assertEqual(p['features'],['vx','vy','r','delta_cmd','a_cmd'])
        self.assertEqual(len(p['scales']),5);self.assertEqual(p['nearest_distance_max'],1.)
        self.assertIn('evaluation mask only',p['scope'])

    def test_safety_stop_does_not_invalidate_verified_alignment(self):
        self.assertTrue(alignment_verified(126,126,4e-15,2e-12))
        self.assertFalse(alignment_verified(126,125,4e-15,2e-12))
        self.assertFalse(alignment_verified(126,126,3e-12,2e-12))
        self.assertFalse(alignment_verified(0,0,0.,2e-12))


if __name__=='__main__':unittest.main()
