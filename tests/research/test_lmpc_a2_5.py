"""A2.5 isolates acceleration excitation without changing A2's random stream."""
import tempfile
from pathlib import Path
import unittest
import numpy as np
import yaml
from f1tenth_benchmarks.research.lmpc.config import load_config
from f1tenth_benchmarks.research.lmpc.initialization import ControlledExcitation
from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3

class AccelerationOnlyTests(unittest.TestCase):
    def test_only_steering_amplitude_differs_from_A2(self):
        a=load_config('params/research/lmpc_a2.yaml').resolved
        b=load_config('params/research/lmpc_a2_5.yaml').resolved
        for k in ('paper','adaptation','initialization'):self.assertEqual(a[k],b[k])
        expected=dict(a['experiment']['excitation'],steering_amplitude_rad=0.)
        self.assertEqual(b['experiment']['excitation'],expected)
        controller=load_b3('params/research/lmpc_paper_a2_5.yaml')
        old=load_b3('params/research/lmpc_paper_a2.yaml')
        self.assertEqual(controller['settings'],old['settings'])
        self.assertNotIn('initialization_source',controller)
        self.assertEqual(controller['prediction_value_mode'],'paper_direct')

    def test_steering_is_PP_and_acceleration_random_sequence_is_preserved(self):
        a=ControlledExcitation(load_config('params/research/lmpc_a2.yaml').resolved['experiment']['excitation'])
        b=ControlledExcitation(load_config('params/research/lmpc_a2_5.yaml').resolved['experiment']['excitation'])
        offsets=[]
        for i in range(100):
            baseline=[.1*np.sin(i),.2];a.apply(baseline,.4189,9.51)
            command=b.apply(baseline,.4189,9.51)
            self.assertEqual(command[0],baseline[0])
            self.assertEqual(b.offset[0],0.)
            self.assertEqual(b.offset[1],a.offset[1])
            self.assertLessEqual(abs(b.offset[1]),.6);offsets.append(b.offset[1])
        self.assertGreater(np.ptp(offsets),.1)

    def check_rejected(self,config):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'bad.yaml';p.write_text(yaml.safe_dump(config))
            with self.assertRaises(ValueError):load_config(p)

    def test_A2_still_requires_nonzero_steering_excitation(self):
        c=load_config('params/research/lmpc_a2.yaml').resolved
        c['experiment']['excitation']['steering_amplitude_rad']=0.;self.check_rejected(c)

    def test_A2_5_rejects_steering_noise_and_zero_acceleration(self):
        for key,value in [('steering_amplitude_rad',.01),('acceleration_amplitude_mps2',0.)]:
            c=load_config('params/research/lmpc_a2_5.yaml').resolved
            c['experiment']['excitation'][key]=value;self.check_rejected(c)

if __name__=='__main__':unittest.main()
