import numpy as np
import pytest

from f1tenth_benchmarks.research.collision.discs import (
    ThreeDiscBody, OccupancyMap, pair_intersects, sweep_samples, opponent_scan,
)
from f1tenth_benchmarks.research.collision.multi_car import MultiCarSim


def test_three_discs_cover_rotated_rectangle():
    body = ThreeDiscBody()
    assert body.radius == pytest.approx(.15953056133543817)
    x, y = np.meshgrid(np.linspace(-.255, .255, 121), np.linspace(-.135, .135, 51))
    local = np.column_stack([x.ravel(), y.ravel()])
    for yaw in [0., .7, 2.8]:
        c, s = np.cos(yaw), np.sin(yaw)
        points = local @ np.array([[c, s], [-s, c]]) + [2, 3]
        centres = body.centres([2, 3, yaw])
        distances = np.linalg.norm(points[:, None]-centres, axis=2)
        assert np.max(np.min(distances, axis=1)) <= body.radius+1e-12


def test_pixel_area_and_bounds_not_only_pixel_centres():
    occupied = np.zeros((10, 10), bool)
    occupied[5, 5] = True
    grid = OccupancyMap(occupied, .1, [0, 0, 0])
    assert grid.intersects([[.49, .55]], .011)
    assert not grid.intersects([[.48, .55]], .01)
    assert grid.intersects([[.05, .4]], .06)
    rotated = OccupancyMap(occupied, .1, [2, 3, np.pi/2])
    assert rotated.intersects([[2-.55, 3+.49]], .011)


def test_pair_clearance_applied_once():
    a, b = np.array([[0., 0.]]), np.array([[.44, 0.]])
    assert not pair_intersects(a, .2, b, .2)
    assert pair_intersects(a, .2, b, .2, .05)
    assert not pair_intersects(a, .2, np.array([[.46, 0.]]), .2, .05)


def test_sweep_detects_crossing_between_clear_endpoints():
    bodies = [ThreeDiscBody(), ThreeDiscBody()]
    before = np.array([[1, 2, 0], [3, 2, np.pi]])
    after = np.array([[3, 2, 0], [1, 2, np.pi]])
    assert not pair_intersects(bodies[0].centres(before[0]), bodies[0].radius,
                               bodies[1].centres(before[1]), bodies[1].radius)
    assert any(pair_intersects(c[0], bodies[0].radius+p[0], c[1], bodies[1].radius+p[1])
               for c, p in sweep_samples(before, after, bodies))


def test_opponent_lidar_front_and_behind():
    assert opponent_scan(np.zeros(3), np.array([0.]), np.array([[2., 0.]]), .2, 30)[0] == pytest.approx(1.8)
    assert opponent_scan(np.zeros(3), np.array([np.pi]), np.array([[2., 0.]]), .2, 30)[0] == 30


def make_sim():
    sim = MultiCarSim('my_map')
    # Isolate dynamics/lifecycle tests from narrow map geometry.
    sim.map = OccupancyMap(np.zeros((200, 200), bool), .1, [-10, -10, 0])
    return sim


def test_two_independent_cars_step_and_reset():
    sim = make_sim()
    obs, done = sim.reset([[0, 0, 0], [2, 0, 0]])
    assert not done
    obs, done = sim.step([[0, 1.2], [0, 0]])
    assert not done
    assert obs[0]['pose'][0] > 0
    assert obs[1]['pose'][0] == 2
    assert obs[0]['simulation_time'] == pytest.approx(.1)
    assert not np.shares_memory(sim.cars[0].state, sim.cars[1].state)
    assert np.isfinite(obs[0]['scan']).all()
    sim.reset([[0, 0, 0], [2, 0, 0]])
    assert all(len(c.steer_buffer) == 0 for c in sim.cars)


def test_initial_overlap_is_terminal():
    sim = make_sim()
    _, done = sim.reset([[0, 0, 0], [0, 0, 0]])
    assert done
    assert any(e['kind'] == 'vehicle_contact' for e in sim.events)
    with pytest.raises(RuntimeError):
        sim.step([[0, 0], [0, 0]])


def test_catch_up_stops_before_full_control_interval():
    sim = make_sim()
    _, done = sim.reset([[0, 0, 0], [.75, 0, 0]], [2., 0.])
    assert not done
    _, done = sim.step([[0, 2.], [0, 0.]])
    assert done and sim.time < .1
    assert any(e['kind'] == 'vehicle_clearance' for e in sim.events)


def test_steering_clock_keeps_phase_across_planning_steps():
    sim = MultiCarSim('my_map', enable_lidar=False,
                      occupancy_map=OccupancyMap(np.zeros((200, 200), bool), .1, [-10, -10, 0]))
    sim.reset([[0, 0, 0], [2, 0, 0]])
    ticks = []

    def update(physics_tick, commands):
        if physics_tick % 4 == 0:
            ticks.append(physics_tick)
        return commands

    for _ in range(2):
        _, done = sim.step([[0., 0.], [0., 0.]], update_actions=update)
        assert not done
    assert ticks == [0, 4, 8, 12, 16]
    assert sim.physics_tick == 20
    assert sim.time == pytest.approx(.2)
