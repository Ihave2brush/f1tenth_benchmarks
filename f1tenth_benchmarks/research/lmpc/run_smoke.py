"""B0 only: ESP observation, actuator tracing and independent physical replay.

Run from repository root. This does not run PP, accept two initial laps, build a
safe set or authorize Frenet planning. All outputs have explicit input semantics.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import platform
from pathlib import Path
from argparse import Namespace

import numpy as np

from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from .config import load_config
from .dynamics import InstrumentedContinuousSim
from .state import StateObserver, StateObservationError, STATE_COLUMNS


def replay(rows, params, start_pose):
    """Original uninstrumented physical model replays recorded PID speed inputs."""
    dynamics = DynamicsSimulator(Namespace(**params))
    dynamics.reset(start_pose)
    count = 0
    for row in rows:
        for tick in row['physics_ticks']:
            np.testing.assert_allclose(dynamics.state, tick['pre_state'], rtol=0, atol=1e-12)
            state = dynamics.update_pose(tick['command'][0], tick['pid_target_speed'])
            np.testing.assert_allclose(state, tick['post_state'], rtol=0, atol=1e-12)
            count += 1
        np.testing.assert_allclose(dynamics.state, row['physical_state'], rtol=0, atol=1e-12)
        np.testing.assert_array_equal(dynamics.steer_buffer, row['steering_buffer'])
    return dict(passed=True, physics_ticks=count, tolerance=1e-12,
                method='original_dynamics_and_recorded_pid_targets')


def run_smoke(config_path='params/research/lmpc.yaml', output_dir=None):
    config = load_config(config_path)
    adaptation, paper = config.adaptation, config.paper
    geometry = load_geometry(adaptation['geometry_path'], adaptation['map_yaml'], inspect=True)
    observer = StateObserver(geometry)
    physics = load_parameter_file('simulator_params')
    sim = InstrumentedContinuousSim('esp', timeout=adaptation['timeout'],
                                    extra_params={'n_sim_steps': config.substeps(physics.timestep)})
    reference = geometry.reference(0.)
    start = np.r_[reference['xy'], reference['psi']]
    if output_dir is None:
        output_dir = Path('Data/research/lmpc') / ('b0_' + datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    root = Path(output_dir)
    root.mkdir(parents=True, exist_ok=False)
    rows, error = [], None
    report = dict(stage='B0_observation_smoke', acceptance='not_run', passed=False,
                  schema_version=1, python_version=platform.python_version(),
                  numpy_version=np.__version__,
                  planning_allowed=False, initialization_laps_accepted=0,
                  safe_set_ready=False, q_function_ready=False,
                  state_columns=STATE_COLUMNS, legacy_action_columns=['steering', 'target_speed'],
                  paper_action_columns=['steering', 'commanded_acceleration'],
                  config=config.resolved, config_path=str(config.path),
                  geometry_id=geometry.geometry_id, frenet_length=geometry.curve.L,
                  legacy_lap_length=sim.tracker.L, simulator_params=vars(sim.params),
                  start_pose=start.tolist(), control_dt=paper['control_dt'],
                  input_hashes={str(p): hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in
                                [config.path, 'params/simulator_params.yaml', 'params/vehicle_params.yaml',
                                 Path(adaptation['geometry_path'])/'manifest.json']})
    report['source_map_hashes'] = geometry.raster.source_hashes
    report['code_hashes'] = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in
                             sorted(Path(__file__).parent.glob('*.py'))}

    def record(obs, command, mode):
        row = dict(time=sim.current_time, physical_state=sim.current_state.tolist(),
                         paper_state=None, frenet=None,
                         legacy_s=obs['s_unwrapped'], legacy_full_laps=obs['completed_full_laps'],
                         reset_count=sim.reset_count, command=command, input_mode=mode,
                         physics_ticks=sim.dynamics_simulator.trace,
                         steering_buffer=sim.dynamics_simulator.steer_buffer.tolist())
        # Append physical evidence before any observation failure can interrupt.
        rows.append(row)
        try:
            state, frenet = observer.observe(obs)
        except StateObservationError as exc:
            row['frenet'] = exc.frenet
            raise
        row.update(paper_state=state.tolist(), frenet=frenet)

    try:
        obs, done, _ = sim.reset(start)
        record(obs, [0., 0.], 'initialization_target_speed')
        if done:
            raise RuntimeError('terminal during reset: ' + str(sim.terminal_reason))
        for _ in range(adaptation['smoke_steps']):
            action = [0., paper['initialization_speed']]
            obs, done = sim.step(action)
            record(obs, action, 'target_speed')
            if done:
                raise RuntimeError('terminal in short speed smoke: ' + str(sim.terminal_reason))
        # Exercise the acceleration adapter while staying on the initial straight.
        obs, done = sim.step_acceleration([0., .1])
        record(obs, [0., .1], 'acceleration')
        if done:
            raise RuntimeError('terminal in acceleration smoke: ' + str(sim.terminal_reason))
        dt = np.diff([r['time'] for r in rows])
        np.testing.assert_allclose(dt, paper['control_dt'], rtol=0, atol=1e-12)
        verification = replay(rows, vars(sim.params), start)
        report.update(acceptance='short_smoke_only', passed=True, replay=verification)
    except Exception as exc:
        error = type(exc).__name__ + ': ' + str(exc)
        raise
    finally:
        report.update(error=error, samples=len(rows), reset_count=sim.reset_count,
                      terminal_reason=sim.terminal_reason,
                      max_abs_e_y=max((abs(r['paper_state'][5]) for r in rows
                                      if r['paper_state'] is not None), default=None))
        (root/'session.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        with (root/'samples.jsonl').open('x') as f:
            for row in rows:
                f.write(json.dumps(row, allow_nan=False) + '\n')
    return dict(output_dir=str(root.resolve()), **report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='params/research/lmpc.yaml')
    parser.add_argument('--output-dir')
    args = parser.parse_args()
    print(json.dumps(run_smoke(args.config, args.output_dir), indent=2))


if __name__ == '__main__':
    main()
