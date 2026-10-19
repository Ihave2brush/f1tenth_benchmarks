"""Observed successful laps; discrete finish samples and signed continuation costs.

Observed arrays never include periodic copies. Optional audited tails are separate.
Controls are stored separately from measured substep-equivalent actuator inputs.
"""
import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


@dataclass
class PeriodicTail:
    states: np.ndarray
    physical_states: np.ndarray
    commands: np.ndarray
    effective_inputs: np.ndarray
    source_sample_ids: np.ndarray
    source_times: np.ndarray
    costs: np.ndarray
    audit: dict


@dataclass
class Lap:
    lap_id: int
    start_row: int
    finish_row: int
    states: np.ndarray
    physical_states: np.ndarray
    commands: np.ndarray
    effective_inputs: np.ndarray
    times: np.ndarray
    sample_ids: np.ndarray
    costs: np.ndarray
    length: float
    dt: float
    event: dict
    periodic_tail: PeriodicTail = None

    @property
    def terminal_index(self):
        return self.finish_row-self.start_row

    @property
    def successor_available(self):
        return np.arange(len(self.states)) < len(self.states)-1

    @property
    def cost_seconds(self):
        return self.costs*self.dt

    def shifted_states(self, cycle):
        result = self.states.copy()
        result[:, 4] += cycle*self.length
        return result

    def terminal_pool(self, cycle=0):
        """Observed data plus explicitly classified periodic terminal templates.

        The final observed state's outgoing edge is an approximate seam. It must
        never be consumed as a measured transition by model identification.
        """
        states, costs, ids = self.states, self.costs, self.sample_ids
        commands, effective = self.commands, self.effective_inputs
        kinds = ['observed']*(len(states)-1)+['unavailable']
        if self.periodic_tail is not None:
            tail = self.periodic_tail
            states = np.vstack([states, tail.states])
            costs = np.r_[costs, tail.costs]
            ids = np.r_[ids, tail.source_sample_ids]
            commands = np.vstack([commands, tail.commands])
            effective = np.vstack([effective, tail.effective_inputs])
            kinds[-1] = 'approximate_periodic_seam'
            kinds += ['historical_periodic_replay']*(len(tail.states)-1)+['unavailable']
        states = states.copy(); states[:, 4] += cycle*self.length
        return dict(states=states, costs=costs, sample_ids=ids,
                    commands=commands, effective_inputs=effective, transition_kinds=kinds)


class LapDatabase:
    def __init__(self, laps, metadata):
        self.laps = tuple(laps)
        self.metadata = metadata
        self.length = float(metadata['length'])
        self.dt = float(metadata['dt'])
        self.geometry_id = metadata['geometry_id']
        self._validate()

    def _validate(self):
        if not np.isfinite([self.length, self.dt]).all() or min(self.length, self.dt) <= 0:
            raise ValueError('invalid database length/dt')
        ids = [lap.lap_id for lap in self.laps]
        if not ids or ids != sorted(set(ids)):
            raise ValueError('laps must be unique and ordered')
        cutoff = self.metadata['available_through_row']
        for lap in self.laps:
            n = len(lap.states)
            if (lap.states.shape != (n, 6) or lap.physical_states.shape != (n, 7)
                    or lap.commands.shape != (n-1, 2) or lap.effective_inputs.shape != (n-1, 2)
                    or lap.times.shape != (n,) or lap.costs.shape != (n,)
                    or lap.sample_ids.shape != (n,)):
                raise ValueError('invalid lap array shape')
            if not all(np.isfinite(a).all() for a in
                       [lap.states, lap.physical_states, lap.commands, lap.effective_inputs, lap.times, lap.costs]):
                raise ValueError('nonfinite lap data')
            if lap.length != self.length or lap.dt != self.dt:
                raise ValueError('inconsistent lap geometry/time')
            if not 0 < lap.terminal_index < n or lap.sample_ids[-1] > cutoff:
                raise ValueError('incomplete lap or future sample')
            np.testing.assert_array_equal(lap.sample_ids, np.arange(lap.start_row, lap.start_row+n))
            np.testing.assert_allclose(np.diff(lap.times), self.dt, rtol=0, atol=1e-9)
            np.testing.assert_array_equal(lap.costs, lap.terminal_index-np.arange(n))
            if np.any(np.diff(lap.states[:, 4]) < -1e-9):
                raise ValueError('nonmonotone lap progress')
            t = lap.terminal_index
            if not lap.states[t-1, 4] < self.length <= lap.states[t, 4]:
                raise ValueError('discrete terminal does not bracket finish')
            tail = lap.periodic_tail
            if tail is not None:
                m = len(tail.states)
                if (lap.terminal_index != n-1 or m < 21 or tail.states.shape != (m, 6)
                        or tail.physical_states.shape != (m, 7) or tail.commands.shape != (m, 2)
                        or tail.effective_inputs.shape != (m, 2) or tail.costs.shape != (m,)
                        or tail.source_sample_ids.shape != (m,) or tail.source_times.shape != (m,)):
                    raise ValueError('invalid periodic tail shape/scope')
                if not tail.audit.get('passed') or tail.audit.get('geometry_id') != self.geometry_id:
                    raise ValueError('periodic seam is not audited for this geometry')
                from .periodic import check_seam_bounds
                if not check_seam_bounds(tail.audit, tail.audit['policy'])['passed']:
                    raise ValueError('periodic audit residuals violate stored engineering bounds')
                for a in [tail.states, tail.physical_states, tail.commands, tail.effective_inputs,
                          tail.costs, tail.source_times]:
                    if not np.isfinite(a).all():
                        raise ValueError('nonfinite periodic template')
                if np.any(tail.source_sample_ids > cutoff):
                    raise ValueError('periodic template references future data')
                source_indices = tail.source_sample_ids-lap.start_row
                if (np.any(source_indices <= 0) or np.any(source_indices >= lap.terminal_index)
                        or np.any(np.diff(source_indices) != 1) or source_indices[0] != 1):
                    raise ValueError('periodic template must reuse the same successful lap prefix')
                expected = lap.states[source_indices].copy(); expected[:, 4] += self.length
                np.testing.assert_array_equal(tail.states, expected)
                np.testing.assert_array_equal(tail.physical_states, lap.physical_states[source_indices])
                np.testing.assert_array_equal(tail.source_times, lap.times[source_indices])
                np.testing.assert_array_equal(tail.commands, lap.commands[source_indices-1])
                np.testing.assert_array_equal(tail.effective_inputs, lap.effective_inputs[source_indices-1])
                np.testing.assert_array_equal(tail.costs, lap.costs[-1]-np.arange(1, m+1))
                if np.any(np.diff(np.r_[lap.states[-1, 4], tail.states[:, 4]]) <= 0):
                    raise ValueError('nonmonotone periodic seam')

    @classmethod
    def from_b1(cls, directory, available_through_row=None):
        root = Path(directory)
        report = json.loads((root/'session.json').read_text())
        if (report.get('stage') != 'B1_initialization' or not report.get('passed')
                or report.get('initialization_laps_accepted') != 2
                or not all(report.get('checks', {}).values())):
            raise ValueError('requires accepted two-lap B1 evidence')
        rows = [json.loads(line) for line in (root/'samples.jsonl').read_text().splitlines()]
        events = [json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
        collected = report.get('collection_laps', 2)
        if collected not in (2, 3) or len(rows) != report['sample_count'] or len(events) != collected:
            raise ValueError('source counts disagree')
        cutoff = len(rows)-1 if available_through_row is None else available_through_row
        if isinstance(cutoff, bool) or not isinstance(cutoff, int) or not 0 <= cutoff < len(rows):
            raise ValueError('invalid availability cutoff')
        length = report['frenet_length']
        dt = report['config']['paper']['control_dt']
        for i, row in enumerate(rows[:cutoff+1]):
            fr = row['frenet']
            if (not fr['valid'] or fr['geometry_id'] != report['geometry_id']
                    or fr['frame_id'] != 'map' or fr['timestamp'] != row['time']
                    or not fr['freshness_verified'] or not fr['version_verified']
                    or row['reset_count'] != 1 or row['terminal_reason'] is not None):
                raise ValueError('invalid source observation')
            if i:
                ticks = row['physics_ticks']
                if not ticks or row['input_mode'] not in ('target_speed', 'acceleration'):
                    raise ValueError('missing actuator trace')
                np.testing.assert_array_equal(ticks[0]['pre_state'], rows[i-1]['physical_state'])
                for j, tick in enumerate(ticks):
                    np.testing.assert_array_equal(tick['command'], row['command'])
                    if j:
                        np.testing.assert_array_equal(tick['pre_state'], ticks[j-1]['post_state'])
                np.testing.assert_array_equal(ticks[-1]['post_state'], row['physical_state'])
                np.testing.assert_allclose(sum(t['dt'] for t in ticks), dt, rtol=0, atol=1e-10)
        ends = []
        for j, event in enumerate(events):
            end = event['row_index']
            if (event['kind'] != 'full' or event['completed_full_laps'] != j+1
                    or end <= (ends[-1] if ends else 0) or end >= len(rows)
                    or not np.isclose(event['finish_s'], (j+1)*length)):
                raise ValueError('invalid full-lap event')
            if end <= cutoff:
                ends.append(end)
        laps = []
        for j, end in enumerate(ends[:2]):
            start = 0 if j == 0 else ends[j-1]
            # Extend only through subsequent *completed successful* data available now.
            last = ends[min(j+1, len(ends)-1)]
            selected = rows[start:last+1]
            states = np.asarray([r['paper_state'] for r in selected], float)
            states[:, 4] -= j*length
            commands, effective = [], []
            for row in selected[1:]:
                ticks = row['physics_ticks']
                commands.append(row['command'])
                effective.append([sum(t['pre_state'][2]*t['dt'] for t in ticks)/dt,
                                  sum(t['applied_acceleration']*t['dt'] for t in ticks)/dt])
            laps.append(Lap(j+1, start, end, states,
                            np.asarray([r['physical_state'] for r in selected]),
                            np.asarray(commands), np.asarray(effective),
                            np.asarray([r['time'] for r in selected]),
                            np.arange(start, last+1), end-np.arange(start, last+1),
                            length, dt, events[j]))
        metadata = dict(schema_version=1, stage='B2_database', geometry_id=report['geometry_id'],
                        length=length, dt=dt, available_through_row=cutoff,
                        source_directory=str(root.resolve()),
                        source_hashes={name:sha256(root/name) for name in
                                       ['session.json', 'samples.jsonl', 'events.jsonl']},
                        alignment='first observed sample after finish; shared boundary sample',
                        cost='signed T-t; suffix is continuation potential, not physical negative travel time',
                        cost_units='control steps; cost_seconds = cost_steps * control_dt',
                        input_columns=['steering_command', 'acceleration_command' if report.get('acceleration_interface') else 'target_speed'],
                        effective_input_columns=['time_mean_actual_steering', 'time_mean_applied_acceleration'],
                        effective_input_scope='descriptive substep averages; not an exact constant-input six-state model',
                        actuator_memory='physical states and source steering_buffer/substep traces retained',
                        planning_allowed=False, continuous_verified=False)
        metadata.update(terminal_extension_mode='observed_continuous_laps' if collected == 3 else 'observed_available_prefix',
                        historical_source_laps=len(laps), observed_continuation_laps=collected-2,
                        new_observed_source_laps=0, acceleration_interface=report.get('acceleration_interface', False))
        return cls(laps, metadata)

    def save(self, directory):
        root = Path(directory)
        root.mkdir(parents=True, exist_ok=False)
        metadata = dict(self.metadata, laps=[])
        for lap in self.laps:
            name = 'lap_%03d.npz' % lap.lap_id
            np.savez_compressed(root/name, **{key:getattr(lap, key) for key in
                                ['states', 'physical_states', 'commands', 'effective_inputs',
                                 'times', 'sample_ids', 'costs', 'cost_seconds']})
            entry = dict(lap_id=lap.lap_id, start_row=lap.start_row,
                                        finish_row=lap.finish_row, event=lap.event,
                                        file=name, sha256=sha256(root/name))
            if lap.periodic_tail is not None:
                tail_name = 'lap_%03d_periodic.npz' % lap.lap_id
                np.savez_compressed(root/tail_name, **{key:getattr(lap.periodic_tail, key) for key in
                                    ['states', 'physical_states', 'commands', 'effective_inputs',
                                     'source_sample_ids', 'source_times', 'costs']})
                entry['periodic_tail'] = dict(file=tail_name, sha256=sha256(root/tail_name),
                                               audit=lap.periodic_tail.audit)
            metadata['laps'].append(entry)
        (root/'database.json').write_text(json.dumps(metadata, indent=2, allow_nan=False))

    @classmethod
    def load(cls, directory):
        root = Path(directory)
        metadata = json.loads((root/'database.json').read_text())
        if metadata.get('schema_version') != 1:
            raise ValueError('unsupported database schema')
        laps = []
        for entry in metadata['laps']:
            name = entry['file']
            if Path(name).name != name or sha256(root/name) != entry['sha256']:
                raise ValueError('invalid lap artifact/hash')
            with np.load(root/name, allow_pickle=False) as data:
                arrays = {key:data[key].copy() for key in data.files}
            np.testing.assert_allclose(arrays.pop('cost_seconds'), arrays['costs']*metadata['dt'], rtol=0, atol=1e-12)
            if 'periodic_tail' in entry:
                info = entry['periodic_tail']; tail_name = info['file']
                if Path(tail_name).name != tail_name or sha256(root/tail_name) != info['sha256']:
                    raise ValueError('invalid periodic artifact/hash')
                with np.load(root/tail_name, allow_pickle=False) as data:
                    arrays['periodic_tail'] = PeriodicTail(audit=info['audit'], **{key:data[key].copy() for key in data.files})
            laps.append(Lap(entry['lap_id'], entry['start_row'], entry['finish_row'],
                            length=metadata['length'], dt=metadata['dt'], event=entry['event'], **arrays))
        return cls(laps, metadata)
