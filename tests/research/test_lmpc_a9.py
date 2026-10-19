"""A9 preserves A5 control settings and switches only its history to A2.5."""
from pathlib import Path
import tempfile
import unittest
import yaml
from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3,run
from f1tenth_benchmarks.research.lmpc.config import load_config

class A9Tests(unittest.TestCase):
    def test_A5_kernel_settings_and_exploration_preserved(self):
        a=load_b3('params/research/lmpc_paper_a5.yaml')
        b=load_b3('params/research/lmpc_paper_a9.yaml')
        for key in ['kernel_selection','exploration','settings','prediction_value_mode']:
            self.assertEqual(a[key],b[key])
        self.assertNotIn('command_filter',b)
        base=load_config(b['base_config'])
        self.assertEqual(base.experiment_id,'A2.5')
        self.assertEqual(base.resolved['experiment']['excitation']['steering_amplitude_rad'],0.)
        self.assertIn('A2_5_lmpc_',b['initialization_source']['directory'])

    def test_A9_rejects_actual_angle_selection_and_deadzone(self):
        for field in ['kernel','deadzone']:
            c=load_b3('params/research/lmpc_paper_a9.yaml')
            if field=='kernel':c['kernel_selection']['selection_steering']='delta_real'
            else:c['command_filter']={'steering_zero_deadzone_rad':.005}
            with tempfile.TemporaryDirectory() as d:
                p=Path(d)/'bad.yaml';p.write_text(yaml.safe_dump(c))
                with self.assertRaises(ValueError):load_b3(p)

    def test_A9_rejects_A2_base_before_starting_a_run(self):
        c=load_b3('params/research/lmpc_paper_a9.yaml');c['base_config']='params/research/lmpc_a2.yaml'
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'bad.yaml';p.write_text(yaml.safe_dump(c));out=Path(d)/'run'
            with self.assertRaisesRegex(ValueError,'A2.5'):run(p,out)
            self.assertFalse(out.exists())

if __name__=='__main__':unittest.main()
