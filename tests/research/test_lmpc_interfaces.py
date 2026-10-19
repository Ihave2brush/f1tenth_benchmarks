"""B0 contracts: state semantics, actuator equivalence, timing and failure logs."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import yaml

from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from f1tenth_benchmarks.simulator.dynamic_models import pid
from f1tenth_benchmarks.research.lmpc.config import load_config
from f1tenth_benchmarks.research.lmpc.state import paper_state, StateObservationError
from f1tenth_benchmarks.research.lmpc.dynamics import TracedDynamics, acceleration_target_speed
from f1tenth_benchmarks.research.lmpc.run_smoke import run_smoke, replay


class LmpcInterfaceTests(unittest.TestCase):
    def test_A2_requires_explicit_bounded_excitation(self):
        from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3
        config=load_config('params/research/lmpc_a2.yaml')
        self.assertEqual(config.initialization_speed,2.5)
        self.assertEqual(config.paper,load_config('params/research/lmpc_a1.yaml').paper)
        a1=load_b3('params/research/lmpc_paper_a1.yaml')
        a2=load_b3('params/research/lmpc_paper_a2.yaml')
        self.assertEqual(a1['settings'],a2['settings'])
        self.assertEqual(load_config(a2['base_config']).experiment_id,'A2')
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'config.yaml'
            for key,value in [('seed',True),('hold_steps',0),('steering_amplitude_rad',.03),
                              ('acceleration_amplitude_mps2',float('nan'))]:
                data=copy.deepcopy(config.resolved);data['experiment']['excitation'][key]=value
                path.write_text(yaml.safe_dump(data))
                with self.assertRaises(ValueError):load_config(path)
            data=copy.deepcopy(config.resolved);del data['experiment']['excitation']
            path.write_text(yaml.safe_dump(data))
            with self.assertRaises(ValueError):load_config(path)

    def test_paper_settings_and_integer_physics_substeps(self):
        config = load_config()
        self.assertEqual(config.substeps(.01), 10)
        self.assertEqual(config.paper['horizon'], 12)
        with self.assertRaises(ValueError):
            config.substeps(.03)

    def test_A0_A1_only_override_explicit_initialization_reference(self):
        from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3
        a0=load_config('params/research/lmpc_a0.yaml')
        a1=load_config('params/research/lmpc_a1.yaml')
        self.assertEqual((a0.experiment_id,a0.initialization_speed),('A0',1.2))
        self.assertEqual((a1.experiment_id,a1.initialization_speed),('A1',2.5))
        self.assertEqual(a0.paper,a1.paper)
        self.assertEqual(a0.initialization,a1.initialization)
        self.assertEqual(a0.adaptation,a1.adaptation)
        p0=load_b3('params/research/lmpc_paper_a0.yaml')
        p1=load_b3('params/research/lmpc_paper_a1.yaml')
        self.assertEqual(p0['settings'],p1['settings'])
        self.assertEqual(load_config(p1['base_config']).initialization_speed,2.5)

    def test_experiment_label_and_speed_cannot_silently_disagree(self):
        original=load_config().resolved
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'config.yaml'
            for experiment in (dict(id='A1',initialization_speed_mps=1.2),
                    dict(id='A0',initialization_speed_mps=2.5),
                    dict(id='A1',initialization_speed_mps=True),
                    dict(id='A2',initialization_speed_mps=3.)):
                data=copy.deepcopy(original);data['experiment']=experiment
                path.write_text(yaml.safe_dump(data))
                with self.assertRaises(ValueError):load_config(path)

    def test_config_rejects_silent_deviations_and_boolean_diagonals(self):
        original = load_config().resolved
        cases = []
        for value in (13, True, 12.0):
            data = copy.deepcopy(original)
            data['paper']['horizon'] = value
            cases.append(data)
        data = copy.deepcopy(original)
        data['paper']['terminal_distance_diagonal'][0] = False
        cases.append(data)
        data = copy.deepcopy(original)
        data['adaptation']['purpose'] = 'planning'
        cases.append(data)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'config.yaml'
            for data in cases:
                path.write_text(yaml.safe_dump(data))
                with self.assertRaises(ValueError):
                    load_config(path)

    def valid_frenet(self):
        return dict(valid=True, geometry_id='test', frame_id='map', timestamp=1.,
                    freshness_verified=True, version_verified=True,
                    e_psi=.2, s_unwrapped=101., d=-.1)

    def test_velocity_projection_forward_reverse_and_standing(self):
        for velocity in (0., 1.2, -1.2):
            physical = [1, 2, .1, velocity, .4, .3, .15]
            result = paper_state(physical, self.valid_frenet(), 1., 'test')
            expected = [velocity*np.cos(.15), velocity*np.sin(.15), .3, .2, 101., -.1]
            np.testing.assert_allclose(result, expected)
            # Match the original global translation after rotation by yaw.
            yaw = physical[4]
            rotation = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            np.testing.assert_allclose(rotation @ result[:2],
                                       velocity*np.array([np.cos(yaw+.15), np.sin(yaw+.15)]))

    def test_state_rejects_stale_invalid_and_mismatched_geometry(self):
        for key, value in [('valid', False), ('timestamp', .9), ('geometry_id', 'other'),
                           ('frame_id', 'odom'), ('version_verified', False),
                           ('freshness_verified', False), ('d', float('nan'))]:
            frenet = self.valid_frenet()
            frenet[key] = value
            with self.assertRaises(ValueError):
                paper_state(np.zeros(7), frenet, 1., 'test')

    def test_pid_inversion_all_direction_branches(self):
        params = dict(max_v=20., min_v=-5., max_a=9.51)
        for speed in (0., .3, 1.2, -1.2):
            for acceleration in (-1., 0., 1.):
                target = acceleration_target_speed(acceleration, speed, params)
                actual, _ = pid(target, 0., speed, 0., 3.2, 9.51, 20., -5.)
                self.assertAlmostEqual(actual, acceleration)

    def test_tracing_does_not_change_original_speed_dynamics(self):
        params = load_parameter_file('simulator_params')
        original, traced = DynamicsSimulator(params), TracedDynamics(params)
        for dynamics in (original, traced):
            dynamics.reset([0., 0., 0.])
        for index in range(50):
            command = [.08 if index > 20 else 0., 1.2]
            expected = original.update_pose(*command)
            actual = traced.update_pose(*command)
            np.testing.assert_array_equal(actual, expected)
            np.testing.assert_array_equal(traced.steer_buffer, original.steer_buffer)
            tick = traced.trace[-1]
            self.assertAlmostEqual((actual[3]-tick['pre_state'][3])/params.timestep,
                                   tick['applied_acceleration'])
            self.assertAlmostEqual((actual[2]-tick['pre_state'][2])/params.timestep,
                                   tick['applied_steering_rate'])

    def test_acceleration_is_constrained_by_the_original_model(self):
        params = load_parameter_file('simulator_params')
        dynamics = TracedDynamics(params)
        dynamics.reset([0., 0., 0.])
        dynamics.state[3] = 1.2
        dynamics.input_mode = 'acceleration'
        dynamics.update_pose(0., 100.)
        tick = dynamics.trace[-1]
        self.assertAlmostEqual(tick['requested_acceleration'], 100.)
        self.assertAlmostEqual(tick['applied_acceleration'], params.a_max)

    def test_smoke_replay_and_output_exclusivity(self):
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/'smoke'
            report = run_smoke(output_dir=output)
            self.assertTrue(report['passed'])
            self.assertEqual(report['reset_count'], 1)
            self.assertEqual(report['initialization_laps_accepted'], 0)
            self.assertFalse(report['planning_allowed'])
            self.assertFalse(report['safe_set_ready'])
            rows = [json.loads(line) for line in (output/'samples.jsonl').read_text().splitlines()]
            self.assertEqual(len(rows), 22)
            self.assertEqual(rows[-1]['input_mode'], 'acceleration')
            self.assertEqual(len(rows[-1]['physics_ticks']), 10)
            rows[-1]['physics_ticks'][-1]['post_state'][0] += .001
            with self.assertRaises(AssertionError):
                replay(rows, report['simulator_params'], report['start_pose'])
            with self.assertRaises(FileExistsError):
                run_smoke(output_dir=output)

    def test_frenet_failure_preserves_the_rejected_physical_sample(self):
        rejection = dict(valid=False, reason='ambiguous_projection', geometry_id='test')
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/'failure'
            with patch('f1tenth_benchmarks.research.lmpc.run_smoke.StateObserver.observe',
                       side_effect=[(np.zeros(6), self.valid_frenet()),
                                    StateObservationError('invalid Frenet', rejection)]):
                with self.assertRaises(StateObservationError):
                    run_smoke(output_dir=output)
            report = json.loads((output/'session.json').read_text())
            rows = [json.loads(line) for line in (output/'samples.jsonl').read_text().splitlines()]
            self.assertFalse(report['passed'])
            self.assertEqual(report['samples'], 2)
            self.assertIsNone(rows[-1]['paper_state'])
            self.assertEqual(rows[-1]['frenet']['reason'], 'ambiguous_projection')
            self.assertEqual(len(rows[-1]['physics_ticks']), 10)
            self.assertTrue(replay(rows, report['simulator_params'], report['start_pose'])['passed'])


if __name__ == '__main__':
    unittest.main()
