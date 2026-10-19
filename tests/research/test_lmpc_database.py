"""Independent B2 cost/selection oracles and unavailable-successor boundaries."""
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from f1tenth_benchmarks.research.lmpc.database import Lap, LapDatabase
from f1tenth_benchmarks.research.lmpc.safe_set import (
    TerminalSupportError, initial_terminal_candidate, select_local_safe_set)


def analytic_database():
    laps = []
    for lap_id, start, end, last in [(1, 0, 100, 200), (2, 100, 200, 200)]:
        ids = np.arange(start, last+1)
        states = np.zeros((len(ids), 6))
        states[:, 0] = 1.
        states[:, 4] = (ids-start)/10.
        laps.append(Lap(lap_id, start, end, states, np.zeros((len(ids), 7)),
                        np.zeros((len(ids)-1, 2)), np.zeros((len(ids)-1, 2)),
                        ids/10., ids, end-ids, 10., .1, {'duration':10.}))
    return LapDatabase(laps, dict(schema_version=1, geometry_id='analytic', length=10.,
                                 dt=.1, available_through_row=200))


class DatabaseContracts(unittest.TestCase):
    def setUp(self):
        self.db = analytic_database()

    def test_discrete_cost_and_observed_extension(self):
        first, second = self.db.laps
        np.testing.assert_array_equal(first.costs[99:103], [1, 0, -1, -2])
        self.assertEqual(first.sample_ids[101], second.sample_ids[1])
        np.testing.assert_array_equal(first.states[101, :4], second.states[1, :4])
        self.assertEqual(first.states[101, 4]-second.states[1, 4], 10.)
        self.assertFalse(second.successor_available[-1])

    def test_k_nearest_oracle_and_distance_ignores_other_states(self):
        candidate = np.array([99., -90., 10., 8., 3.025, 50.])
        local = select_local_safe_set(self.db, candidate)
        for lap in self.db.laps:
            expected = sorted(range(len(lap.states)), key=lambda i:abs(i/10.-3.025))[:20]
            actual = [p['local_index'] for p in local.provenance if p['lap_id']==lap.lap_id]
            self.assertEqual(actual, sorted(expected))
        self.assertEqual(local.states.shape, (6, 40))
        for i, p in enumerate(local.provenance):
            self.assertEqual(p['successor_sample_id'], p['sample_id']+1)
            self.assertAlmostEqual(local.successors[4, i]-local.states[4, i], .1)

    def test_q_function_against_analytic_cost(self):
        local = select_local_safe_set(self.db, [1, 0, 0, 0, 3, 0])
        # All stored columns satisfy J=100-10s, an independent closed-form LP oracle.
        result = local.q_function([1, 0, 0, 0, 3.025, 0])
        self.assertAlmostEqual(result['value'], 69.75, places=8)
        known = local.q_function(local.states[:, 9])
        self.assertAlmostEqual(known['value'], local.costs[9], places=8)
        np.testing.assert_allclose(local.combination(result['lambda_weights']),
                                   [1, 0, 0, 0, 3.025, 0], atol=1e-8)
        advance = local.advance(result['lambda_weights'])
        self.assertAlmostEqual(advance['candidate'][4], 3.125)
        self.assertAlmostEqual(advance['continuation_cost'], 68.75)

    def test_q_function_rejects_outside_hull_and_bad_lambda(self):
        local = select_local_safe_set(self.db, [1, 0, 0, 0, 3, 0])
        with self.assertRaises(TerminalSupportError):
            local.q_function([2, 0, 0, 0, 3, 0])
        for lam in [np.zeros(40), np.full(40, np.nan), np.r_[-1., np.ones(39)/19.5]]:
            with self.assertRaises(ValueError):
                local.advance(lam)

    def test_almost_constant_state_rows_keep_physical_residual_small(self):
        for lap in self.db.laps:
            lap.states[:, 0] += np.sin(np.arange(len(lap.states)))*1e-12
            lap.states[:, 1] = np.cos(np.arange(len(lap.states)))*1e-12
        local = select_local_safe_set(self.db, [1, 0, 0, 0, 3, 0])
        lam = np.zeros(40); lam[3] = .3; lam[25] = .7
        target = local.combination(lam)
        result = local.q_function(target)
        self.assertLess(result['equality_residual'], 1e-9)
        self.assertAlmostEqual(result['value'], 100-10*target[4], places=8)

    def test_missing_successor_is_not_replaced_by_distant_or_periodic_data(self):
        for s in [9.5, 10., 10.1, -.1]:
            with self.assertRaises(TerminalSupportError):
                select_local_safe_set(self.db, [1, 0, 0, 0, s, 0])
        local = select_local_safe_set(self.db, [1, 0, 0, 0, 8, 0])
        self.assertEqual(local.states.shape[1], 40)

    def test_explicit_coordinate_shift_preserves_cost_and_successors(self):
        normal = select_local_safe_set(self.db, [1, 0, 0, 0, 3, 0])
        shifted = select_local_safe_set(self.db, [1, 0, 0, 0, 23, 0], cycle=2)
        np.testing.assert_allclose(shifted.states[4], normal.states[4]+20)
        np.testing.assert_allclose(shifted.successors[4], normal.successors[4]+20)
        np.testing.assert_array_equal(shifted.costs, normal.costs)
        np.testing.assert_array_equal(initial_terminal_candidate(self.db), self.db.laps[-1].states[12])

    def test_roundtrip_and_corruption(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)/'db'
            self.db.save(root)
            restored = LapDatabase.load(root)
            np.testing.assert_array_equal(restored.laps[0].states, self.db.laps[0].states)
            with self.assertRaises(FileExistsError):
                self.db.save(root)
            with (root/'lap_001.npz').open('ab') as stream:
                stream.write(b'corrupt')
            with self.assertRaises(ValueError):
                LapDatabase.load(root)

    def test_import_controls_belong_to_outgoing_transition_and_cutoff(self):
        # Three observed samples; each destination row records the incoming control.
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            report = dict(stage='B1_initialization', passed=True, initialization_laps_accepted=2,
                          checks={'verified':True}, sample_count=3, frenet_length=1., geometry_id='test',
                          config={'paper':{'control_dt':.1}})
            rows = []
            for i, s in enumerate([0., 1.1, 2.1]):
                command = [i*.1, 1.2]
                tick = dict(command=command, pre_state=[0.]*7, post_state=[0.]*7,
                            dt=.1, applied_acceleration=float(i))
                rows.append(dict(time=i*.1, paper_state=[1., 0., 0., 0., s, 0.],
                                 physical_state=[0.]*7, reset_count=1, terminal_reason=None,
                                 input_mode='target_speed', command=command, physics_ticks=[tick],
                                 frenet=dict(valid=True, geometry_id='test', frame_id='map', timestamp=i*.1,
                                             freshness_verified=True, version_verified=True)))
            events = [dict(kind='full', completed_full_laps=i, row_index=i, finish_s=float(i), duration=.1)
                      for i in [1, 2]]
            (root/'session.json').write_text(json.dumps(report))
            for name, values in [('samples.jsonl', rows), ('events.jsonl', events)]:
                (root/name).write_text('\n'.join(json.dumps(v) for v in values))
            db = LapDatabase.from_b1(root)
            np.testing.assert_array_equal(db.laps[0].commands, [[.1, 1.2], [.2, 1.2]])
            np.testing.assert_array_equal(db.laps[0].effective_inputs[:, 1], [1., 2.])
            prefix = LapDatabase.from_b1(root, available_through_row=1)
            self.assertEqual(len(prefix.laps), 1)
            self.assertEqual(prefix.laps[0].sample_ids[-1], 1)
            # Break raw physical linkage: importer must fail rather than infer a transition.
            rows[1]['physics_ticks'][0]['pre_state'][0] = 1.
            (root/'samples.jsonl').write_text('\n'.join(json.dumps(v) for v in rows))
            with self.assertRaises(AssertionError):
                LapDatabase.from_b1(root)


if __name__ == '__main__':
    unittest.main()
