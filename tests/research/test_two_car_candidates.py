import numpy as np
import pytest

from f1tenth_benchmarks.research.collision.discs import OccupancyMap
from f1tenth_benchmarks.research.collision.multi_car import MultiCarSim
from f1tenth_benchmarks.research.core.geometry.map_raster import load_map
from f1tenth_benchmarks.research.overtaking.millimetre_domain import MillimetreDomain
from f1tenth_benchmarks.research.overtaking.raceline_cruise import local_track_gap
from f1tenth_benchmarks.research.overtaking.track_data import TrackData
from f1tenth_benchmarks.research.overtaking.two_car_env import TwoCarOvertakingEnv, select_candidate_default


@pytest.fixture(scope='module')
def track_and_domain():
    track = TrackData('maps/frenet/my_map/geometry', 'maps/my_map.yaml')
    return track, MillimetreDomain.build(track.geometry, margin=.185)


def test_no_lidar_reads_trinary_map_and_exposes_copied_truth(monkeypatch):
    def reject_scanner(*args, **kwargs):
        pytest.fail('LiDAR constructor called')

    monkeypatch.setattr('f1tenth_benchmarks.research.collision.multi_car.ScanSimulator2D', reject_scanner)
    sim = MultiCarSim('my_map', enable_lidar=False)
    raster = load_map('maps/my_map.yaml')
    assert np.array_equal(sim.map.occupied, np.flipud(raster.occupancy != 0))
    observations = sim.observe()
    ego, lead = observations
    assert ego['scan'] is None and lead['scan'] is None
    opponent = ego['other_vehicles'][0]
    assert opponent['car_id'] == 1
    assert np.array_equal(opponent['vehicle_state'], lead['vehicle_state'])
    opponent['vehicle_state'][3] = 1.2
    assert sim.cars[1].state[3] == 0.


def test_shared_occupancy_preserves_wall_and_pair_events():
    occupancy = OccupancyMap(np.zeros((60, 60), bool), .1, [0, 0, 0])
    with_lidar = MultiCarSim('my_map', occupancy_map=occupancy)
    without_lidar = MultiCarSim('my_map', occupancy_map=occupancy, enable_lidar=False)
    for poses, kind in (
        ([[.30, 2, 0], [2, 2, 0]], 'wall_contact'),
        ([[.35, 2, 0], [2, 2, 0]], 'wall_clearance'),
        ([[2, 2, 0], [2.60, 2, 0]], 'vehicle_contact'),
        ([[2, 2, 0], [2.68, 2, 0]], 'vehicle_clearance'),
    ):
        assert with_lidar.reset(poses)[1]
        assert without_lidar.reset(poses)[1]
        assert with_lidar.events == without_lidar.events
        assert [event['kind'] for event in without_lidar.events] == [kind]


def test_sync_plan_and_track_across_spawn_seam(track_and_domain):
    track, domain = track_and_domain
    env = TwoCarOvertakingEnv(track, domain, lead_speed=1.2, ego_start_s=track.geometry.curve.L - 1.)
    first = env.reset()
    assert first['lead_state']['delta_s_unwrapped_m'] == pytest.approx(2.)
    for step in range(3):
        transition, frame = env.step()
        plan = transition['plan']
        assert plan['source_control_step'] == step
        assert [path['candidate_id'] for path in plan['paths']] == [1, 2, 3, 4, 5]
        assert transition['selected_candidate_id'] == 3
        assert plan['paths'][2]['footprint_sampled_valid']
        assert not transition['ego_control']['fallback_used']
        assert frame['terminal_reason'] is None
        assert frame['ego_state']['sim_time_s'] == frame['lead_state']['sim_time_s']
        assert frame['time_s'] == pytest.approx((step + 1) * .1)
    for key in ('ego_state', 'lead_state'):
        assert frame[key]['speed_mps'] > .8
        assert frame[key]['pose'] != first[key]['pose']
    assert env.reset()['time_s'] == 0.


def test_empty_selection_commands_zero_speed(track_and_domain):
    def choose_none(plan, ego, lead):
        rejected = [dict(path, sampled_valid=False) for path in plan['paths']]
        return select_candidate_default(dict(paths=rejected), ego, lead)

    env = TwoCarOvertakingEnv(*track_and_domain, selector=choose_none)
    env.reset()
    transition, frame = env.step()
    assert transition['selected_candidate_id'] is None
    assert transition['actions'][0] == [0., 0.]
    assert transition['ego_control']['fallback_used']
    assert frame['ego_state']['speed_mps'] == 0.
    assert frame['lead_state']['speed_mps'] > 0.


def test_phase2_checks_pair_conflict_before_commanding_follow_speed(track_and_domain, monkeypatch):
    env = TwoCarOvertakingEnv(*track_and_domain, phase=2, gap=1.2)
    reset = env.sim.reset
    monkeypatch.setattr(env.sim, 'reset', lambda poses: reset(poses, [1.2, .9]))
    first = env.reset()
    transition, frame = env.step()
    assert transition['decision']['mode'] == 'follow'
    assert transition['decision']['lead_prediction_poses'][0] == first['lead_state']['pose']
    assert transition['selected_candidate_id'] == 3
    assert transition['actions'][0][1] < first['ego_state']['speed_mps']
    cruise = transition['costs']['options'][0]
    assert cruise['candidate_id'] == 3
    assert cruise['reason'] == 'predicted_pair_clearance'
    assert cruise['first_rejection_s'] > 0.
    assert frame['ego_state']['speed_mps'] < first['ego_state']['speed_mps']
    assert not frame['events']


@pytest.mark.parametrize('unwrapped,expected', [(-2., -2.), (-233., 7.), (-473., 7.), (-482., -2.)])
def test_periodic_overtaking_gap_distinguishes_a_pass_from_lapping(unwrapped, expected):
    assert local_track_gap(unwrapped, 240.) == pytest.approx(expected)
