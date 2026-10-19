"""Audited, approximate periodic terminal reuse of an existing successful prefix."""
from argparse import Namespace
from dataclasses import replace
import json
from pathlib import Path

import numpy as np
import yaml

from .database import LapDatabase, PeriodicTail, sha256


def load_policy(path='params/research/lmpc_periodic.yaml'):
    policy = yaml.safe_load(Path(path).read_text())
    keys = {'schema_version', 'mode', 'prefix_steps', 'max_state_residual',
            'max_position_residual_m', 'max_yaw_residual_rad',
            'max_actual_steering_residual_rad', 'max_incoming_buffer_difference_rad',
            'max_outgoing_buffer_residual_rad', 'source', 'scope'}
    if (not isinstance(policy, dict) or set(policy) != keys or type(policy['schema_version']) is not int
            or policy['schema_version'] != 1 or policy['mode'] != 'historical_periodic_reuse'
            or type(policy['prefix_steps']) is not int or policy['prefix_steps'] < 21
            or policy['scope'] != 'approximate_terminal_history_only_not_identification_or_measured_seam'):
        raise ValueError('invalid periodic policy')
    residual = np.asarray(policy['max_state_residual'], float)
    if residual.shape != (6,) or not np.isfinite(residual).all() or np.any(residual <= 0):
        raise ValueError('invalid seam state bounds')
    for key in keys:
        if key.startswith('max_') and key != 'max_state_residual':
            if type(policy[key]) not in (int, float) or not np.isfinite(policy[key]) or policy[key] <= 0:
                raise ValueError('invalid seam bound '+key)
    return policy


def check_seam_bounds(audit, policy):
    """Explicit engineering gates; not an exact-transition or nonlinear safety proof."""
    checks = dict(state_residual=bool(np.all(np.asarray(audit['state_residual']) <= policy['max_state_residual'])),
                  geometry=bool(audit['geometry_valid']), footprint=bool(audit['footprint_valid']),
                  prefix_candidate=bool(audit['prefix_candidate_valid']),
                  forward_progress=bool(audit['forward_progress']),
                  position=audit['position_residual_m'] <= policy['max_position_residual_m'],
                  yaw=audit['yaw_residual_rad'] <= policy['max_yaw_residual_rad'],
                  steering=audit['steering_residual_rad'] <= policy['max_actual_steering_residual_rad'],
                  incoming_buffer=audit['incoming_buffer_difference_rad'] <= policy['max_incoming_buffer_difference_rad'],
                  outgoing_buffer=audit['outgoing_buffer_residual_rad'] <= policy['max_outgoing_buffer_residual_rad'])
    return dict(passed=all(checks.values()), checks=checks)


def attach_periodic_tail(database, source, policy_path='params/research/lmpc_periodic.yaml'):
    """Only reuse available states from the same completed flying lap.

    There is one approximate seam; all later prefix edges correspond to actual
    past transitions, translated by L. No sampled state or input is interpolated.
    """
    # Artifact loading/bound checks do not need the simulator or its JIT cache.
    from f1tenth_benchmarks.research.core.geometry import load_geometry
    from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
    from .state import StateObserver
    from .validation import FootprintChecker, CandidateValidator
    policy = load_policy(policy_path)
    root = Path(source)
    if {name:sha256(root/name) for name in database.metadata['source_hashes']} != database.metadata['source_hashes']:
        raise ValueError('periodic source hashes differ from observed database')
    report = json.loads((root/'session.json').read_text())
    rows = [json.loads(line) for line in (root/'samples.jsonl').read_text().splitlines()]
    lap = database.laps[-1]
    count = policy['prefix_steps']
    if (len(database.laps) < 2 or lap.periodic_tail is not None or lap.terminal_index != len(lap.states)-1
            or count >= lap.terminal_index or lap.event.get('lap_label') != 'flying'):
        raise ValueError('requires a completed flying lap with a missing tail and sufficient observed prefix')
    if lap.finish_row > database.metadata['available_through_row']:
        raise ValueError('source lap not available at snapshot cutoff')
    geometry = load_geometry(report['config']['adaptation']['geometry_path'],
                             report['config']['adaptation']['map_yaml'], inspect=True)
    if geometry.geometry_id != database.geometry_id:
        raise ValueError('periodic geometry mismatch')
    physics = report['simulator_params']; vehicle = report['vehicle_params']
    footprint = FootprintChecker(geometry.raster, physics['vehicle_length'], physics['vehicle_width'],
                                 report['footprint_margin_m'])
    states = lap.states[1:count+1].copy(); states[:, 4] += database.length
    validator = CandidateValidator(geometry, footprint, vehicle['max_speed'], vehicle['max_steer'], physics['a_max'])
    prefix_check = validator.check(states)
    endpoint = rows[lap.finish_row]
    initial_prefix = rows[lap.start_row]
    next_prefix = rows[lap.start_row+1]
    dynamics = DynamicsSimulator(Namespace(**physics))
    # Seed complete actual endpoint, including delayed actuator memory. No reset.
    dynamics.state = np.asarray(endpoint['physical_state'], float).copy()
    dynamics.steer_buffer = np.asarray(endpoint['steering_buffer'], float).copy()
    incoming_difference = float(np.max(np.abs(dynamics.steer_buffer-np.asarray(initial_prefix['steering_buffer']))))
    trajectory, footprint_valid = [dynamics.state.tolist()], True
    for tick in next_prefix['physics_ticks']:
        if tick['dt'] != dynamics.time_step:
            raise ValueError('seam substep dt mismatch')
        footprint_valid &= footprint.check(dynamics.state[[0, 1, 4]])['valid']
        dynamics.update_pose(tick['command'][0], tick['pid_target_speed'])
        footprint_valid &= footprint.check(dynamics.state[[0, 1, 4]])['valid']
        trajectory.append(dynamics.state.tolist())
    observer = StateObserver(geometry)
    predicted, frenet = observer.observe(dict(vehicle_state=dynamics.state,
                                              simulation_time=endpoint['time']+database.dt))
    predicted[4] += round((states[0, 4]-predicted[4])/database.length)*database.length
    target = np.asarray(next_prefix['physical_state'])
    yaw_error = (dynamics.state[4]-target[4]+np.pi) % (2*np.pi)-np.pi
    audit = dict(geometry_id=database.geometry_id, policy=policy, policy_sha256=sha256(policy_path),
                 seam_kind='approximate_periodic_seam', measured_transition=False,
                 identification_eligible=False, source_lap_id=lap.lap_id,
                 from_sample_id=lap.finish_row, to_source_sample_id=lap.start_row+1,
                 source_prefix_start_sample_id=lap.start_row,
                 copied_state_translation_m=database.length,
                 state_residual=np.abs(predicted-states[0]).tolist(),
                 predicted_state=predicted.tolist(), template_state=states[0].tolist(),
                 position_residual_m=float(np.linalg.norm(dynamics.state[:2]-target[:2])),
                 yaw_residual_rad=float(abs(yaw_error)),
                 steering_residual_rad=float(abs(dynamics.state[2]-target[2])),
                 incoming_buffer_difference_rad=incoming_difference,
                 outgoing_buffer_residual_rad=float(np.max(np.abs(dynamics.steer_buffer-np.asarray(next_prefix['steering_buffer'])))),
                 source_incoming_buffer=initial_prefix['steering_buffer'],
                 endpoint_buffer=endpoint['steering_buffer'],
                 predicted_outgoing_buffer=dynamics.steer_buffer.tolist(),
                 template_outgoing_buffer=next_prefix['steering_buffer'],
                 forward_progress=bool(lap.states[-1, 4]<states[0, 4]),
                 geometry_valid=bool(frenet['valid']), footprint_valid=bool(footprint_valid),
                 prefix_candidate_valid=bool(prefix_check['valid']),
                 diagnostic_predicted_physical_trajectory=trajectory,
                 exact_dynamics_successor=False,
                 model='original dynamics, historical control/PID targets, complete actual endpoint memory')
    audit.update(check_seam_bounds(audit, policy))
    if not audit['passed']:
        raise ValueError('periodic seam audit failed: '+json.dumps(audit, allow_nan=False))
    indices = np.arange(1, count+1)
    tail = PeriodicTail(states, lap.physical_states[indices].copy(), lap.commands[:count].copy(),
                        lap.effective_inputs[:count].copy(), lap.sample_ids[indices].copy(),
                        lap.times[indices].copy(), lap.costs[-1]-indices, audit)
    laps = list(database.laps); laps[-1] = replace(lap, periodic_tail=tail)
    metadata = dict(database.metadata, terminal_extension_mode=policy['mode'],
                    periodic_policy_path=str(policy_path), periodic_policy_sha256=sha256(policy_path),
                    periodic_extension_scope=policy['scope'], historical_source_laps=len(laps),
                    new_observed_laps=0, measured_seam=False,
                    max_candidate_overhang_m=report['config']['paper']['horizon']*database.dt*vehicle['max_speed'])
    return LapDatabase(laps, metadata)
