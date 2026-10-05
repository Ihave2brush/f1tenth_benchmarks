"""Lifecycle/serialization acceptance, with controlled dynamics and real smoke checks."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import numpy as np
from f1tenth_benchmarks.research.continuous_sim import F1TenthSim_Continuous
from f1tenth_benchmarks.research.continuous_logging import ContinuousLapLogger
from f1tenth_benchmarks.research.run_continuous_pp import run_continuous_laps, verify_session
from f1tenth_benchmarks.research.lap_tracking import ContinuousLapTracker

POINTS = np.array([[0., 0.], [2., 0.], [2., 2.], [-2., 2.], [-2., 0.]])


class ControlledDynamics:
    def __init__(self, params):
        self.dt = params.timestep
        self.reference = ContinuousLapTracker(POINTS)
        self.state = np.zeros(7)
        self.steer_buffer = np.array([])
        self.reset_calls = self.update_calls = 0

    def reset(self, pose):
        self.reset_calls += 1
        self.state = np.zeros(7)
        self.state[[0, 1, 4]] = pose
        self.s = self.reference._project(pose, global_search=True).s_wrapped
        self.steer_buffer = np.array([])
        return self.state

    def update_pose(self, steer, speed):
        self.update_calls += 1
        self.steer_buffer = np.r_[steer, self.steer_buffer][:2]
        self.s += speed*self.dt
        wrapped = self.s % self.reference.L
        i = np.searchsorted(self.reference.starts, wrapped, side='right')-1
        f = (wrapped-self.reference.starts[i])/self.reference.lengths[i]
        xy = self.reference.points[i] + f*self.reference.edges[i]
        yaw = np.arctan2(self.reference.edges[i, 1], self.reference.edges[i, 0])
        self.state = np.array([*xy, steer, speed, yaw, .42, .13])
        return self.state


class ControlledScan:
    def __init__(self, *args):
        self.blocked = False

    def check_location(self, xy):
        return self.blocked

    def scan(self, pose):
        return np.ones(3)


class ConstantPlanner:
    name = 'ControlledPlanner'

    def __init__(self, action=(.1, 10.)):
        self.action = np.array(action)
        self.calls = 0

    def plan(self, observation):
        self.calls += 1
        return self.action.copy()


class ContinuousAcceptanceTests(unittest.TestCase):
    def setUp(self):
        self.patches = [patch('f1tenth_benchmarks.research.continuous_sim.DynamicsSimulator', ControlledDynamics),
                        patch('f1tenth_benchmarks.research.continuous_sim.ScanSimulator2D', ControlledScan)]
        for p in self.patches:
            p.start()
            self.addCleanup(p.stop)

    def sim(self, **kwargs):
        return F1TenthSim_Continuous('esp', centreline_xy=POINTS, **kwargs)

    def test_initialization_tick_and_observation_copy(self):
        sim = self.sim()
        obs, done, start = sim.reset()
        self.assertFalse(done)
        np.testing.assert_array_equal(start, [0, 0, 0])
        self.assertAlmostEqual(sim.current_time, .04)
        self.assertAlmostEqual(sim.elapsed_time, 0.)
        self.assertEqual(sim.total_steps, 1)
        self.assertEqual(sim.control_steps, 0)
        self.assertEqual(sim.dynamics_simulator.update_calls, 4)
        self.assertEqual(sim.dynamics_simulator.reset_calls, 1)
        self.assertEqual(sim.tracker.current.index, 0)
        self.assertEqual(sim.tracker.completed_full_laps, 0)
        self.assertFalse(sim.tracker.events)
        saved = obs['vehicle_state'].copy()
        sim.step([.1, 10])
        np.testing.assert_array_equal(obs['vehicle_state'], saved)
        with self.assertRaises(RuntimeError):
            sim.reset()

    def test_crossing_and_next_step_preserve_state_and_buffer(self):
        sim = self.sim()
        sim.reset([-.2, 0, 0])
        obs, done = sim.step([.1, 10])
        self.assertFalse(done)
        self.assertEqual(obs['lap_event'].kind, 'partial')
        state = sim.current_state.copy()
        buffer = sim.dynamics_simulator.steer_buffer.copy()
        self.assertEqual(len(buffer), 2)
        sim.step([.2, 10])
        np.testing.assert_array_equal(sim.last_pre_step_state, state)
        np.testing.assert_array_equal(sim.last_pre_step_buffer, buffer)
        self.assertEqual(sim.dynamics_simulator.reset_calls, 1)
        self.assertEqual(sim.dynamics_simulator.update_calls, 12)
        np.testing.assert_array_equal(sim.current_state[5:], [.42, .13])

    def test_runner_three_full_laps_and_post_step_artifacts(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim, planner = self.sim(), ConstantPlanner()
            out = Path(tmp)/'session'
            summary = run_continuous_laps(sim, planner, 3, out, save_scans=True)
            self.assertEqual(summary['end_reason'], 'requested_laps_reached')
            self.assertEqual(summary['completed_full_laps'], 3)
            self.assertEqual(summary['partial_event_count'], 0)
            self.assertEqual(summary['reset_count'], 1)
            self.assertEqual(planner.calls, sim.control_steps)
            self.assertEqual(sim.total_steps, sim.control_steps+1)
            self.assertEqual(sim.dynamics_simulator.update_calls, 4*sim.total_steps)
            events = [json.loads(line) for line in (out/'events.jsonl').read_text().splitlines()]
            self.assertEqual([e['lap_label'] for e in events], ['standing_start', 'flying', 'flying'])
            np.testing.assert_allclose([e['duration'] for e in events], [1.2]*3, atol=1e-12)
            rows = [json.loads(line) for line in (out/'samples.jsonl').read_text().splitlines()]
            raw = np.load(out/'SimLog_esp_session.npy')
            self.assertEqual(raw.shape, (sim.total_steps, 10))
            np.testing.assert_array_equal(raw[:, :7], [r['physical_state'] for r in rows])
            self.assertEqual(np.load(out/'ScanLog_esp_session.npy').shape, (sim.total_steps, 3))
            for e in events:
                j = e['post_step_row_index']
                np.testing.assert_array_equal(e['post_step_physical_state'], rows[j]['physical_state'])
                if j+1 < len(rows):
                    np.testing.assert_array_equal(rows[j+1]['pre_step_state'], rows[j]['physical_state'])
                    np.testing.assert_array_equal(rows[j+1]['pre_step_steering_buffer'], rows[j]['post_step_steering_buffer'])
                self.assertAlmostEqual(e['duration'], e['crossing_time_estimate']-e['lap_start_time'])

    def test_partial_does_not_count_towards_requested_laps(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim = self.sim()
            summary = run_continuous_laps(sim, ConstantPlanner(), 2, Path(tmp)/'partial', start_pose=[-.2, 0, 0])
            self.assertEqual(summary['completed_full_laps'], 2)
            self.assertEqual(summary['full_event_count'], 2)
            self.assertEqual(summary['partial_event_count'], 1)
            self.assertEqual(summary['end_reason'], 'requested_laps_reached')

    def test_collision_crossing_precedence_and_terminal_guard(self):
        sim = self.sim()
        sim.reset([-.2, 0, 0])
        sim.scan_simulator.blocked = True
        obs, done = sim.step([.1, 10])
        self.assertTrue(done)
        self.assertEqual(obs['terminal_reason'], 'collision')
        self.assertFalse(sim.tracker.events)
        with self.assertRaises(RuntimeError):
            sim.step([.1, 10])
        with tempfile.TemporaryDirectory() as tmp:
            other = self.sim()
            other.scan_simulator.blocked = True
            planner = ConstantPlanner()
            summary = run_continuous_laps(other, planner, 3, Path(tmp)/'collision')
            self.assertEqual(summary['end_reason'], 'collision')
            self.assertEqual(summary['completed_full_laps'], 0)
            self.assertEqual(planner.calls, 0)

    def test_timeout_after_250_seconds_is_not_lap_complete(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim = self.sim(timeout=250.04)
            summary = run_continuous_laps(sim, ConstantPlanner([0, 0]), 1, Path(tmp)/'timeout')
            self.assertEqual(summary['end_reason'], 'timeout')
            self.assertEqual(summary['completed_full_laps'], 0)
            self.assertGreaterEqual(summary['elapsed_time'], 250.)
            self.assertFalse(sim.lap_complete)

    def test_projection_invalid_terminates_without_recovery(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim = self.sim(extra_params={'timestep': .1})
            summary = run_continuous_laps(sim, ConstantPlanner(), 1, Path(tmp)/'invalid')
            self.assertEqual(summary['end_reason'], 'projection_invalid')
            self.assertEqual(summary['tracker_invalid_reason'], 'sampling_gap')
            self.assertEqual(summary['tracker_epoch'], 0)
            self.assertEqual(summary['completed_full_laps'], 0)

    def test_finalize_is_idempotent_and_exclusive(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim = self.sim()
            sim.reset()
            out = Path(tmp)/'log'
            logger = ContinuousLapLogger(out, {}, save_scans=True)
            logger.record(sim, initializing=True)
            first = logger.finalize('timeout', sim)
            snapshots = {p.name: p.read_bytes() for p in out.iterdir()}
            self.assertIs(logger.finalize('collision', sim), first)
            self.assertEqual(snapshots, {p.name: p.read_bytes() for p in out.iterdir()})
            self.assertFalse(logger.scans or logger.samples or logger.rows or logger.events)
            with self.assertRaises(FileExistsError):
                ContinuousLapLogger(out, {})
            with self.assertRaises(RuntimeError):
                logger.record(sim)

    def test_runner_exception_finalizes_actual_samples(self):
        class FailingPlanner:
            def plan(self, observation):
                raise RuntimeError('planner failed')
        with tempfile.TemporaryDirectory() as tmp:
            out = Path(tmp)/'error'
            with self.assertRaisesRegex(RuntimeError, 'planner failed'):
                run_continuous_laps(self.sim(), FailingPlanner(), 1, out)
            saved = json.loads((out/'session.json').read_text())
            self.assertEqual(saved['end_reason'], 'error')
            self.assertEqual(saved['sample_count'], 1)
            self.assertIn('planner failed', saved['error'])


class RealDynamicsSmokeTests(unittest.TestCase):
    def test_real_timeout_moving_start_replay_and_corruption_detection(self):
        with tempfile.TemporaryDirectory() as tmp:
            sim = F1TenthSim_Continuous('esp', timeout=.12)
            out = Path(tmp)/'real_timeout'
            summary = run_continuous_laps(sim, ConstantPlanner([0, 4]), 1, out,
                                          initial_speed=.5, save_scans=True)
            self.assertEqual(summary['end_reason'], 'timeout')
            self.assertEqual(summary['completed_full_laps'], 0)
            self.assertEqual(sim.tracker._label, 'moving_start')
            self.assertTrue(verify_session(out, require_complete=False)['passed'])
            rows = [json.loads(line) for line in (out/'samples.jsonl').read_text().splitlines()]
            rows[1]['pre_step_steering_buffer'][0] = .3
            (out/'samples.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in rows))
            with self.assertRaises(AssertionError):
                verify_session(out, require_complete=False)

    def test_real_esp_short_steps_and_collision(self):
        sim = F1TenthSim_Continuous('esp')
        obs, done, pose = sim.reset()
        self.assertFalse(done)
        np.testing.assert_array_equal(pose[:2], sim.tracker.finish_origin)
        self.assertEqual(sim.tracker._label, 'standing_start')
        for _ in range(10):
            obs, done = sim.step([0, 4])
            self.assertFalse(done, obs['terminal_reason'])
            self.assertTrue(obs['projection_valid'])
        self.assertEqual(sim.reset_count, 1)
        self.assertEqual(sim.total_steps, 11)
        wall = F1TenthSim_Continuous('esp')
        _, done, _ = wall.reset([-1000, -1000, 0])
        self.assertTrue(done)
        self.assertEqual(wall.terminal_reason, 'collision')
        self.assertEqual(wall.tracker.completed_full_laps, 0)


if __name__ == '__main__':
    unittest.main()
