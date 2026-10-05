"""Portable imports, real CSV loading, and a bounded headless PP integration."""
import importlib

import numpy as np
import pytest

from f1tenth_benchmarks.classic_racing.GlobalPurePursuit import GlobalPurePursuit
from f1tenth_benchmarks.research.closed_velocity_profile import digest
from f1tenth_benchmarks.research.continuous_sim import F1TenthSim_Continuous
from f1tenth_benchmarks.utils.track_utils import RaceTrack


@pytest.mark.parametrize('name', [
    'f1tenth_benchmarks',
    'f1tenth_benchmarks.research.closed_velocity_profile',
    'f1tenth_benchmarks.research.lap_tracking',
    'f1tenth_benchmarks.research.continuous_sim',
    'f1tenth_benchmarks.research.continuous_logging',
    'f1tenth_benchmarks.research.run_continuous_pp',
    'trajectory_planning_helpers.calc_vel_profile',
])
def test_required_imports(name):
    importlib.import_module(name)


@pytest.mark.parametrize('raceline_set', ['mu60', 'mu60_closed'])
def test_esp_raceline_structure_and_real_loader(raceline_set):
    filename = f'Data/racelines/{raceline_set}/esp_raceline.csv'
    raw = np.loadtxt(filename, delimiter=',')
    assert raw.ndim == 2 and raw.shape[1] == 7 and len(raw) > 3
    assert np.isfinite(raw).all()
    assert np.all(np.diff(raw[:, 0]) > 0)
    assert np.all((raw[:, 5] > 0) & (raw[:, 5] <= 8 + 1e-9))
    closed_edges = np.roll(raw[:, 1:3], -1, axis=0) - raw[:, 1:3]
    assert np.all(np.linalg.norm(closed_edges, axis=1) > 0)

    # Upstream always skips one row: mu60 has no header, closed has a header.
    # Protect the existing loader behavior without changing the baseline CSV.
    loaded_rows = raw[1:] if raceline_set == 'mu60' else raw
    track = RaceTrack('esp', raceline_set)
    np.testing.assert_array_equal(track.path, loaded_rows[:, 1:3])
    np.testing.assert_array_equal(track.speeds, loaded_rows[:, 5])


def test_original_mu60_bytes_are_preserved():
    assert digest('Data/racelines/mu60/esp_raceline.csv') == (
        '61ef20fd634e57befe5c9e3d8086575d507b5eaeffe5fdc54f9ea0f61b1b5a08')


def test_real_pure_pursuit_ten_headless_steps():
    planner = GlobalPurePursuit('regression_smoke', init_folder=False,
                               extra_params={'racetrack_set': 'mu60_closed'})
    planner.set_map('esp')
    sim = F1TenthSim_Continuous('esp')
    obs, done, _ = sim.reset()
    assert not done
    for _ in range(10):
        action = planner.plan(obs)
        assert action.shape == (2,) and np.isfinite(action).all()
        obs, done = sim.step(action)
        assert not done, obs['terminal_reason']
        assert np.isfinite(obs['vehicle_state']).all()
        assert np.isfinite(obs['scan']).all()
        assert obs['projection_valid']
    assert sim.reset_count == 1
    assert sim.control_steps == 10
    assert sim.tracker.completed_full_laps == 0
    assert obs['vehicle_speed'] > 0
    assert sim.elapsed_time == pytest.approx(.4)
