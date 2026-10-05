"""Exclusive session artifacts; physical samples and estimated boundaries separate."""
import csv
from dataclasses import asdict
import json
from pathlib import Path
import numpy as np


def _json_safe(value):
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, float) and not np.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


class ContinuousLapLogger:
    def __init__(self, directory, metadata, save_scans=False):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.metadata = _json_safe(metadata)
        self.save_scans = save_scans
        self.samples, self.events, self.rows, self.scans = [], [], [], []
        self.finalized = False
        self.summary = None

    def record(self, sim, action=None, initializing=False):
        if self.finalized:
            raise RuntimeError('logger already finalized')
        result = sim.last_tracking_result
        sample = result.sample if result.valid else None
        action = np.zeros(2) if action is None else np.array(action, copy=True)
        state = sim.current_state.copy()
        self.samples.append(_json_safe(dict(
            row_index=len(self.samples), total_steps=sim.total_steps,
            control_steps=sim.control_steps, initializing=initializing,
            simulation_time=sim.current_time, elapsed_time=sim.elapsed_time,
            physical_state=state, action_for_preceding_interval=action,
            pre_step_state=sim.last_pre_step_state,
            pre_step_steering_buffer=sim.last_pre_step_buffer,
            post_step_steering_buffer=sim.dynamics_simulator.steer_buffer.copy(),
            projection_valid=result.valid, projection_reason=result.reason,
            tracker_sample=None if sample is None else asdict(sample),
            tracker_epoch=sim.tracker.epoch, reset_count=sim.reset_count,
            collision=sim.collision, terminal_reason=sim.terminal_reason)))
        progress = np.nan if sample is None else sample.projection.s_wrapped/sim.tracker.L
        self.rows.append(np.r_[state, action, progress])
        if self.save_scans:
            self.scans.append(None if sim.scan is None else sim.scan.copy())
        if result.event:
            event = asdict(result.event)
            event.update(post_step_physical_state=state.tolist(),
                         post_step_sample_time=sim.current_time,
                         post_step_row_index=len(self.samples)-1,
                         physical_state_semantics='actual_post_step_sample_not_exact_crossing')
            self.events.append(_json_safe(event))

    def finalize(self, reason, sim, error=None):
        if self.finalized:
            return self.summary
        full = [e for e in self.events if e['kind'] == 'full']
        self.summary = _json_safe(dict(
            schema_version=1, metadata=self.metadata, end_reason=reason, error=error,
            completed_full_laps=sim.tracker.completed_full_laps,
            full_event_count=len(full), partial_event_count=len(self.events)-len(full),
            full_lap_times=[e['duration'] for e in full],
            simulation_time=sim.current_time, elapsed_time=sim.elapsed_time,
            initialization_time=sim.initialization_time, total_steps=sim.total_steps,
            control_steps=sim.control_steps, reset_count=sim.reset_count,
            sample_count=len(self.samples), collision=sim.collision,
            tracker_invalid_reason=sim.tracker.invalid_reason,
            tracker_epoch=sim.tracker.epoch,
            npy_columns=['x','y','steering','velocity','yaw','yaw_rate','slip',
                         'preceding_action_steering','preceding_action_speed','wrapped_progress'],
            state_semantics='post_step_physical_samples',
            crossing_time_semantics='linear_s_interpolation_estimate'))
        if len(full) != sim.tracker.completed_full_laps:
            raise RuntimeError('logger full events and tracker count disagree')
        for name, values in [('samples.jsonl', self.samples), ('events.jsonl', self.events)]:
            with (self.directory/name).open('x') as f:
                for value in values:
                    f.write(json.dumps(value, allow_nan=False)+'\n')
        with (self.directory/'session.json').open('x') as f:
            json.dump(self.summary, f, indent=2, allow_nan=False)
        fields = ['kind', 'lap_label', 'completed_full_laps', 'epoch', 'finish_s',
                  'lap_start_time', 'crossing_time_estimate', 'duration', 'alpha']
        with (self.directory/'lap_results.csv').open('x', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
            writer.writeheader()
            writer.writerows(self.events)
        with (self.directory/f'SimLog_{sim.map_name}_session.npy').open('xb') as f:
            np.save(f, np.asarray(self.rows).reshape((-1, 10)))
        if self.save_scans:
            if any(scan is None for scan in self.scans):
                raise RuntimeError('cannot save missing scans')
            with (self.directory/f'ScanLog_{sim.map_name}_session.npy').open('xb') as f:
                np.save(f, np.asarray(self.scans))
        self.finalized = True
        self.samples.clear()
        self.events.clear()
        self.rows.clear()
        self.scans.clear()
        return self.summary
