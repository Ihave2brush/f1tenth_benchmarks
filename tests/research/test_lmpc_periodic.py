"""Periodic history is finite, causal, cost-continuous and explicitly approximate."""
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

import numpy as np

from test_lmpc_database import analytic_database
from f1tenth_benchmarks.research.lmpc.database import LapDatabase, PeriodicTail
from f1tenth_benchmarks.research.lmpc.periodic import load_policy, check_seam_bounds
from f1tenth_benchmarks.research.lmpc.safe_set import select_local_safe_set, TerminalSupportError


def periodic_database():
    observed = analytic_database()
    lap = observed.laps[-1]
    count = 40
    policy = load_policy(); policy['prefix_steps'] = count
    audit = dict(passed=True, geometry_id=observed.geometry_id, policy=policy,
                 state_residual=[0.]*6, geometry_valid=True, footprint_valid=True,
                 prefix_candidate_valid=True, forward_progress=True,
                 position_residual_m=0., yaw_residual_rad=0., steering_residual_rad=0.,
                 incoming_buffer_difference_rad=0., outgoing_buffer_residual_rad=0.)
    indices = np.arange(1, count+1)
    states = lap.states[indices].copy(); states[:, 4] += observed.length
    tail = PeriodicTail(states, lap.physical_states[indices].copy(), lap.commands[:count].copy(),
                        lap.effective_inputs[:count].copy(), lap.sample_ids[indices].copy(),
                        lap.times[indices].copy(), -indices, audit)
    return LapDatabase([observed.laps[0], replace(lap, periodic_tail=tail)], observed.metadata)


class PeriodicContracts(unittest.TestCase):
    def test_seam_and_prefix_have_distinct_provenance(self):
        db = periodic_database()
        local = select_local_safe_set(db, [1, 0, 0, 0, 10., 0])
        seam = [p for p in local.provenance if p['transition_kind']=='approximate_periodic_seam']
        self.assertEqual(len(seam), 1)
        self.assertEqual((seam[0]['sample_id'], seam[0]['successor_sample_id']), (200, 101))
        self.assertFalse(seam[0]['measured_transition'])
        self.assertFalse(seam[0]['identification_eligible'])
        replay = [p for p in local.provenance if p['transition_kind']=='historical_periodic_replay']
        self.assertTrue(replay)
        self.assertTrue(all(not p['measured_transition'] and not p['identification_eligible'] for p in replay))
        self.assertEqual(db.laps[-1].sample_ids[-1], 200)
        self.assertEqual(len(db.laps[-1].states), 101)

    def test_signed_cost_and_lambda_at_finish(self):
        local = select_local_safe_set(periodic_database(), [1, 0, 0, 0, 10., 0])
        q = local.q_function([1, 0, 0, 0, 10.025, 0])
        self.assertAlmostEqual(q['value'], -.25)
        advanced = local.advance(q['lambda_weights'])
        self.assertAlmostEqual(advanced['candidate'][4], 10.125)
        self.assertAlmostEqual(advanced['continuation_cost'], -1.25)

    def test_window_coverage_and_finite_limit(self):
        db = periodic_database()
        for s in np.linspace(0, 12, 61):
            self.assertEqual(select_local_safe_set(db, [1, 0, 0, 0, s, 0]).states.shape, (6, 40))
        with self.assertRaises(TerminalSupportError):
            select_local_safe_set(db, [1, 0, 0, 0, 14., 0])

    def test_coordinate_shift_does_not_shift_source_ids_or_costs(self):
        db = periodic_database()
        local = select_local_safe_set(db, [1, 0, 0, 0, 10., 0])
        moved = select_local_safe_set(db, [1, 0, 0, 0, 30., 0], cycle=2)
        np.testing.assert_array_equal(moved.costs, local.costs)
        self.assertEqual(local.provenance, tuple(dict(p, cycle=0) for p in moved.provenance))
        np.testing.assert_allclose(moved.successors[4], local.successors[4]+20.)

    def test_periodic_roundtrip_preserves_classification(self):
        db = periodic_database()
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)/'db'; db.save(root)
            loaded = LapDatabase.load(root)
            self.assertEqual(loaded.laps[-1].periodic_tail.audit, db.laps[-1].periodic_tail.audit)
            self.assertEqual(loaded.laps[-1].terminal_pool()['transition_kinds'], db.laps[-1].terminal_pool()['transition_kinds'])
            np.testing.assert_array_equal(loaded.laps[-1].periodic_tail.source_sample_ids,
                                          db.laps[-1].periodic_tail.source_sample_ids)
            with (root/'lap_002_periodic.npz').open('ab') as stream:
                stream.write(b'corruption')
            with self.assertRaises(ValueError):
                LapDatabase.load(root)

    def test_future_sources_or_modified_template_rejected(self):
        db = periodic_database()
        for mutation in ['future', 'state', 'cost']:
            tail = db.laps[-1].periodic_tail
            changed = replace(tail, source_sample_ids=tail.source_sample_ids.copy(),
                              states=tail.states.copy(), costs=tail.costs.copy())
            if mutation == 'future':
                changed.source_sample_ids[-1] = 201
            elif mutation == 'state':
                changed.states[0, 0] += .1
            else:
                changed.costs[0] = 0
            with self.assertRaises((ValueError, AssertionError)):
                LapDatabase([db.laps[0], replace(db.laps[1], periodic_tail=changed)], db.metadata)

    def test_failed_audit_cannot_be_enabled_by_passed_flag(self):
        db = periodic_database()
        tail = db.laps[-1].periodic_tail
        audit = dict(tail.audit, position_residual_m=.2, passed=True)
        self.assertFalse(check_seam_bounds(audit, audit['policy'])['passed'])
        with self.assertRaises(ValueError):
            LapDatabase([db.laps[0], replace(db.laps[1], periodic_tail=replace(tail, audit=audit))], db.metadata)

    def test_malformed_policy_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'policy.yaml'
            path.write_text('schema_version: 1\nmode: historical_periodic_reuse\nprefix_steps: true\n')
            with self.assertRaises(ValueError):
                load_policy(path)


if __name__ == '__main__':
    unittest.main()
