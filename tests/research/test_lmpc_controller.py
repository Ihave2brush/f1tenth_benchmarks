"""B3 tests: physical adapter identity, identification scope and QP feasibility."""
import unittest
from types import SimpleNamespace

import numpy as np

from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
from f1tenth_benchmarks.research.lmpc.controller import solve_qp, MPCFailure
from f1tenth_benchmarks.research.lmpc.dynamics import TracedDynamics
from f1tenth_benchmarks.research.lmpc.local_model import physical_step, augmented, LocalModel, kinematic_step
from f1tenth_benchmarks.research.lmpc.planning_scope import ScopedPlanningGate
from f1tenth_benchmarks.research.lmpc.safe_set import LocalSafeSet
from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3


class ControllerTests(unittest.TestCase):
    def test_original_physical_prior_matches_real_acceleration_and_delay(self):
        params = load_parameter_file('simulator_params')
        plant = TracedDynamics(params)
        plant.state = np.array([1., 2., .032, 1.2, .9, .2, .01])
        plant.steer_buffer = np.array([.07, -.03])
        state = np.array([1.2*np.cos(.01), 1.2*np.sin(.01), .2, 0., 0., 0.])
        z = augmented(state, plant.state, plant.steer_buffer)
        predicted = physical_step(z, np.array([.04, .2]), vars(params), .1)
        plant.input_mode = 'acceleration'
        for _ in range(10):
            plant.update_pose(.04, .2)
        actual = np.r_[plant.state[3]*np.cos(plant.state[6]), plant.state[3]*np.sin(plant.state[6]),
                       plant.state[5], plant.state[2], plant.steer_buffer]
        np.testing.assert_allclose(predicted, actual, rtol=0, atol=1e-12)

    def test_paper_kinematics_order_and_curvature(self):
        geometry = SimpleNamespace(reference=lambda s: {'kappa':.2})
        x = np.array([1.2, .03, .4, .1, 5., .2])
        result = kinematic_step(x, geometry, .1)
        ds = (1.2*np.cos(.1)-.03*np.sin(.1))/.96
        np.testing.assert_allclose(result, [.1+.1*(.4-.2*ds), 5+.1*ds,
                                           .2+.1*(1.2*np.sin(.1)+.03*np.cos(.1))])

    def test_surrogate_linearization_retains_original_nominal_value(self):
        model = object.__new__(LocalModel)
        model.params = vars(load_parameter_file('simulator_params'))
        model.dt = .1
        model.geometry = SimpleNamespace(reference=lambda s: {'kappa':0.})
        model.identify = lambda z, u: (np.zeros((9, 3)), np.r_[z[:3], z[6:9], u], {})
        z = np.array([1.2, 0., 0., 0., 0., 0., .032, .04, .04])
        u = np.array([.04, .2])
        a, b, c, diagnostic = model.linearize(z, u)
        np.testing.assert_allclose(a@z+b@u+c, model.evaluate(z, u), atol=1e-12)
        self.assertGreater(np.max(abs(model.evaluate(z, u)-model.evaluate(z, u, surrogate=True))), 1e-4)
        self.assertIn('surrogate', diagnostic['linearization'])

    def test_identification_selects_80_unique_measured_rows_and_paper_metric(self):
        rng = np.random.RandomState(4)
        model = object.__new__(LocalModel)
        model.source_ids = np.arange(120)
        model.states = rng.normal(size=(120, 9))
        model.controls = rng.normal(size=(120, 2))
        model.features = np.column_stack([model.states[:, :3], model.states[:, 6:9], model.controls])
        model.targets = model.states[:, :3]*.9
        model.residuals = np.zeros((120, 3))
        model.ridge = .1; model.distance_diagonal = np.array([.1, 1., 1.])
        coeff, _, diag = model.identify(np.zeros(9), np.zeros(2))
        expected = np.argsort(np.sum((model.states[:, :3]*[.1, 1., 1.])**2, axis=1))[:80]
        np.testing.assert_array_equal(diag['source_ids'], expected)
        np.testing.assert_array_equal(coeff, np.zeros((9, 3)))
        self.assertTrue(diag['measured_only'])

    def fixture(self, terminal_speed=1.2):
        initial = np.zeros(9); initial[0] = 1.2
        a = np.eye(9); a[4, 0] = .1
        b = np.zeros((9, 2)); b[0, 1] = .1
        nominal = np.array([np.r_[1.2, 0., 0., 0., .12*k, 0., 0., 0., 0.] for k in range(13)])
        states = np.zeros((6, 40)); states[0] = terminal_speed; states[4] = np.linspace(.5, 3., 40)
        terminal = LocalSafeSet(states, states.copy(), (10-states[4])*10,
                                (10-states[4])*10-1, tuple(), 'test', .1)
        settings = load_b3('params/research/lmpc_b3.yaml')['settings']
        return initial, nominal, np.zeros((12, 2)), [(a, b, np.zeros(9), {})]*12, terminal, settings

    def test_qp_enforces_shared_terminal_simplex_and_dynamics(self):
        initial, nominal, controls, models, terminal, settings = self.fixture()
        states, inputs, lam, info = solve_qp(initial, nominal, controls, models, terminal, np.zeros(2), settings)
        self.assertEqual(info['status'], 'solved')
        np.testing.assert_allclose(states[-1, :6], terminal.states@lam, atol=1e-5)
        for k, (a, b, c, _) in enumerate(models):
            np.testing.assert_allclose(states[k+1], a@states[k]+b@inputs[k]+c, atol=1e-5)
        self.assertGreater(states[-1, 4], nominal[-1, 4])
        self.assertAlmostEqual(lam.sum(), 1.)
        self.assertGreaterEqual(lam.min(), 0.)

    def test_unreachable_terminal_refuses_control(self):
        initial, nominal, controls, models, terminal, settings = self.fixture(20.)
        with self.assertRaises(MPCFailure):
            solve_qp(initial, nominal, controls, models, terminal, np.zeros(2), settings)

    def test_candidate_rejection_has_no_action_permission(self):
        geometry = SimpleNamespace(planning_allowed=False)
        validator = SimpleNamespace(check=lambda x, u: dict(valid=False, reason='ambiguous_projection'))
        gate = ScopedPlanningGate(geometry, validator, {}, {})
        result = gate.check(np.zeros(7), np.zeros(2), np.zeros((13, 9)), np.zeros((12, 2)))
        self.assertFalse(result['valid'])
        self.assertIn('ambiguous_projection', result['reason'])


if __name__ == '__main__':
    unittest.main()
