"""B1: fixed-speed centreline PP, two continuous ESP laps, exact diagnostics.

The new candidate interface is a validator, not a grant to optimize Frenet routes.
Original geometry approval flags stay false. Successful laps remain raw B1 data
until B2 defines cost/terminal-continuation semantics and imports the database.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform

import numpy as np

from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
from .config import load_config
from .dynamics import InstrumentedContinuousSim
from .initialization import CentrelinePurePursuit, ControlledExcitation
from .lap_adapter import CoreLapTracker
from .state import paper_state, STATE_COLUMNS
from .validation import FootprintChecker, CandidateValidator
from .run_smoke import replay


class ValidatedContinuousSim(InstrumentedContinuousSim):
    def __init__(self, *args, footprint, **kwargs):
        super().__init__(*args, **kwargs)
        self.footprint = footprint
        self.footprint_failure = None
        self.footprint_checks = 0

    def _advance(self, action):
        super()._advance(action)
        for index, tick in enumerate(self.dynamics_simulator.trace):
            for endpoint in ('pre_state', 'post_state'):
                state = tick[endpoint]
                check = self.footprint.check([state[0], state[1], state[4]])
                self.footprint_checks += 1
                if not check['valid']:
                    self.footprint_failure = dict(check, tick=index, endpoint=endpoint,
                                                  control_step=self.control_steps)
                    self.terminal_reason = 'footprint_rejected'
                    return


def run_initialization(config_path='params/research/lmpc.yaml', output_dir=None,
                       collection_laps=2, acceleration_interface=False):
    config = load_config(config_path)
    paper, settings = config.paper, config.initialization
    reference_speed=config.initialization_speed
    excitation_settings=config.resolved.get('experiment',{}).get('excitation')
    if excitation_settings and not acceleration_interface:
        raise ValueError('A2 excitation requires acceleration interface')
    excitation=ControlledExcitation(excitation_settings) if excitation_settings else None
    if type(collection_laps) is not int or collection_laps not in (2, 3):
        raise ValueError('collect two source laps, optionally a third real continuation lap')
    root = Path(output_dir) if output_dir else Path('Data/research/lmpc')/(
        'b1_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    root.mkdir(parents=True, exist_ok=False)
    rows, events, failure = [], [], None
    report = dict(schema_version=1, stage='B1_initialization', passed=False,
                  initialization_laps_accepted=0, safe_set_ready=False, q_function_ready=False,
                  planning_allowed=False, continuous_verified=False,
                  config=config.resolved, state_columns=STATE_COLUMNS,
                  input_columns=['steering_command', 'target_speed'],
                  startup=f'standing; target_speed={reference_speed} throughout; no reset at finish',
                  experiment_id=config.experiment_id,initialization_speed_mps=reference_speed,
                  python_version=platform.python_version(), numpy_version=np.__version__)
    report.update(collection_laps=collection_laps, acceleration_interface=acceleration_interface,
                  observed_continuation_laps=collection_laps-2)
    report['excitation']=excitation_settings
    if acceleration_interface:
        report.update(input_columns=['steering_command', 'acceleration_command'],
                      startup=f'standing; PP speed reference {reference_speed}; bounded control-interval acceleration; no finish reset',
                      acceleration_adapter='a=clip((v_ref-v)/control_dt, -a_max, a_max); original per-substep PID inversion')
    sim = None
    try:
        geometry = load_geometry(config.adaptation['geometry_path'], config.adaptation['map_yaml'], inspect=True)
        physics = load_parameter_file('simulator_params')
        vehicle = load_parameter_file('vehicle_params')
        centre_path = Path(config.adaptation['geometry_path'])/'centerline.csv'
        sampled = np.loadtxt(centre_path, delimiter=',', skiprows=1)
        # Geometry load verifies hashes. Independently bind CSV positions to its curve.
        xy, tangent, heading, curvature = geometry.curve.sample(sampled[:, 2])
        if not np.allclose(xy, sampled[:, :2], rtol=0, atol=1e-8):
            raise RuntimeError('centreline CSV is not the loaded core curve')
        gap = np.max(np.diff(np.r_[sampled[:, 2], geometry.curve.L]))
        if gap > settings['preflight_step_m']+1e-8:
            raise RuntimeError('centreline samples exceed preflight resolution')
        footprint = FootprintChecker(geometry.raster, physics.vehicle_length,
                                     physics.vehicle_width, settings['footprint_margin_m'])
        validator = CandidateValidator(geometry, footprint, vehicle.max_speed,
                                       vehicle.max_steer, physics.a_max)
        initial_states = np.zeros((len(sampled), 6))
        initial_states[:, 0], initial_states[:, 4] = reference_speed, sampled[:, 2]
        preflight = validator.check(initial_states)
        nominal = np.arctan(vehicle.wheelbase*curvature)
        report.update(geometry_id=geometry.geometry_id, frenet_length=geometry.curve.L,
                      source_map_hashes=geometry.raster.source_hashes,
                      preflight=dict(preflight, sample_step_m=float(gap),
                                     max_nominal_steering=float(np.max(np.abs(nominal))),
                                     nominal_over_limit_samples=int(np.sum(np.abs(nominal)>vehicle.max_steer)),
                                     nominal_steering_note='kinematic diagnostic; finite tracking error and dynamics tested separately'),
                      code_hashes={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
                                   for p in sorted(Path(__file__).parent.glob('*.py'))},
                      input_hashes={str(p):hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in
                                    [config.path, centre_path, 'params/simulator_params.yaml',
                                     'params/vehicle_params.yaml',
                                     Path(config.adaptation['geometry_path'])/'manifest.json']})
        (root/'preflight.json').write_text(json.dumps(report['preflight'], indent=2))
        if not preflight['valid']:
            raise RuntimeError('preflight failed: '+str(preflight))
        controller = CentrelinePurePursuit(sampled[:, :2], reference_speed,
                                          settings['lookahead_m'], vehicle.wheelbase, vehicle.max_steer)
        sim = ValidatedContinuousSim('esp', footprint=footprint, timeout=settings['timeout'],
                                     extra_params={'n_sim_steps':config.substeps(physics.timestep)},
                                     centreline_xy=sampled[:, :2])
        sim.tracker = CoreLapTracker(geometry, sampled[:, :2])
        reference = geometry.reference(0.)
        start = np.r_[reference['xy'], reference['psi']]
        report.update(simulator_params=vars(sim.params), vehicle_params=vars(vehicle),
                      start_pose=start.tolist(), finish_origin=sim.tracker.finish_origin.tolist(),
                      finish_tangent=sim.tracker.finish_tangent.tolist(),
                      tracking_source='core_frenet_only; Continuous lifecycle reused',
                      footprint_margin_m=settings['footprint_margin_m'], physics_dt=physics.timestep)

        def record(obs, action, initializing=False):
            frenet = sim.tracker.frenet
            row = dict(time=sim.current_time, elapsed_time=sim.elapsed_time,
                       physical_state=sim.current_state.tolist(), paper_state=None,
                       frenet=frenet, completed_full_laps=sim.tracker.completed_full_laps,
                       reset_count=sim.reset_count, command=list(action),
                       input_mode='target_speed' if initializing or not acceleration_interface else 'acceleration',
                       target_speed_reference=reference_speed,
                       initializing=initializing, physics_ticks=sim.dynamics_simulator.trace,
                       steering_buffer=sim.dynamics_simulator.steer_buffer.tolist(),
                       controller_diagnostic=controller.diagnostic,
                       excitation_diagnostic=None if excitation is None else excitation.diagnostic,
                       terminal_reason=sim.terminal_reason)
            rows.append(row)
            if obs['projection_valid']:
                row['paper_state'] = paper_state(sim.current_state, frenet, sim.current_time,
                                                 geometry.geometry_id).tolist()
            if obs['lap_event']:
                event = asdict(obs['lap_event'])
                event.update(row_index=len(rows)-1, physical_sample_time=sim.current_time)
                events.append(event)
                print('completed lap', event['completed_full_laps'], 'time', round(event['duration'], 6), flush=True)

        obs, done, _ = sim.reset(start)
        record(obs, [0., 0.], initializing=True)
        while not done and sim.tracker.completed_full_laps < collection_laps:
            action = controller.plan(obs)
            if acceleration_interface:
                action = [action[0], float(np.clip((action[1]-sim.current_state[3])/paper['control_dt'],
                                                   -physics.a_max, physics.a_max))]
                if excitation is not None:
                    action=excitation.apply(action, vehicle.max_steer, physics.a_max)
                obs, done = sim.step_acceleration(action)
            else:
                obs, done = sim.step(action)
            record(obs, action)
            if not done:
                state = rows[-1]['paper_state']
                if abs(state[5]) > settings['max_abs_e_y_m']:
                    raise RuntimeError('lateral tracking limit exceeded')
                if abs(state[3]) > settings['max_abs_e_psi_rad']:
                    raise RuntimeError('heading tracking limit exceeded')
                if sim.elapsed_time >= settings['settled_after_s'] and abs(
                        sim.current_state[3]-reference_speed) > settings['max_settled_speed_error_mps']:
                    raise RuntimeError('settled speed tolerance exceeded')
            if sim.control_steps % 500 == 0:
                print('step', sim.control_steps, 'elapsed', round(sim.elapsed_time, 1), flush=True)
        if done:
            raise RuntimeError('simulation terminated: '+str(sim.terminal_reason))
        states = np.array([row['paper_state'] for row in rows])
        settled = [row for row in rows if row['elapsed_time'] >= settings['settled_after_s']]
        max_speed_error = max(abs(row['physical_state'][3]-reference_speed) for row in settled)
        ticks = [tick for row in rows for tick in row['physics_ticks']]
        steering_valid = all(physics.s_min-1e-12 <= tick['post_state'][2] <= physics.s_max+1e-12 for tick in ticks)
        dt = np.diff([row['time'] for row in rows])
        np.testing.assert_allclose(dt, paper['control_dt'], rtol=0, atol=1e-10)
        checks = dict(source_laps_and_continuation=len(events)==collection_laps and all(e['kind']=='full' for e in events),
                      one_reset=sim.reset_count==1, frenet_valid=all(row['frenet']['valid'] for row in rows),
                      no_collision=not sim.collision, footprint_valid=sim.footprint_failure is None,
                      lateral_error=float(np.max(np.abs(states[:, 5]))) <= settings['max_abs_e_y_m'],
                      heading_error=float(np.max(np.abs(states[:, 3]))) <= settings['max_abs_e_psi_rad'],
                      settled_speed=max_speed_error <= settings['max_settled_speed_error_mps'],
                      actual_steering_within_physical_limits=steering_valid, control_dt=True,
                      exact_target_speed=all((row['target_speed_reference'] if acceleration_interface else row['command'][1])
                                             ==reference_speed for row in rows[1:]))
        verification = replay(rows, vars(sim.params), start)
        checks['physical_replay'] = verification['passed']
        report.update(checks=checks, replay=verification, lap_times=[e['duration'] for e in events],
                      max_abs_e_y=float(np.max(np.abs(states[:, 5]))),
                      max_abs_e_psi=float(np.max(np.abs(states[:, 3]))),
                      max_settled_speed_error=max_speed_error,
                      max_actual_steering=max(abs(t['post_state'][2]) for t in ticks),
                      clipped_command_intervals=sum(bool(row['controller_diagnostic']['steering_clipped'])
                                                    for row in rows[1:]),
                      footprint_checks=sim.footprint_checks)
        if not all(checks.values()):
            raise RuntimeError('B1 acceptance failed: '+str(checks))
        report.update(passed=True, initialization_laps_accepted=paper['initialization_laps'])
    except Exception as exc:
        failure = type(exc).__name__+': '+str(exc)
        raise
    finally:
        report.update(error=failure, sample_count=len(rows),
                      completed_full_laps=0 if sim is None else sim.tracker.completed_full_laps,
                      reset_count=0 if sim is None else sim.reset_count,
                      footprint_failure=None if sim is None else sim.footprint_failure)
        (root/'session.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        for filename, values in [('samples.jsonl', rows), ('events.jsonl', events)]:
            with (root/filename).open('x') as stream:
                for value in values:
                    stream.write(json.dumps(value, allow_nan=False)+'\n')
    return dict(output_dir=str(root.resolve()), **report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='params/research/lmpc.yaml')
    parser.add_argument('--output-dir')
    parser.add_argument('--collection-laps', type=int, choices=[2, 3], default=2)
    parser.add_argument('--acceleration-interface', action='store_true', help='record genuine delta/a commands for paper regression')
    args = parser.parse_args()
    report = run_initialization(args.config, args.output_dir, args.collection_laps, args.acceleration_interface)
    print(json.dumps({k:report[k] for k in ['output_dir','passed','lap_times','max_abs_e_y',
                                         'max_settled_speed_error','replay']}, indent=2))


if __name__ == '__main__':
    main()
