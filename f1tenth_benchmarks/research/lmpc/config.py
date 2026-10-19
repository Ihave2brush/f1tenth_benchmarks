"""Explicit, strict Track B configuration; no changes to benchmark defaults."""
from dataclasses import dataclass
from pathlib import Path
import math

import yaml


PAPER = dict(initialization_laps=2, initialization_speed=1.2, control_dt=.1,
             horizon=12, history_laps=2, safe_points_per_lap=20,
             terminal_distance_diagonal=[0, 0, 0, 0, 1, 0],
             identification_distance_diagonal=[.1, 1, 1, 0, 0, 0],
             identification_points=80, kernel_bandwidth=10,
             kernel='epanechnikov', solver='osqp')


@dataclass(frozen=True)
class ExperimentConfig:
    path: Path
    resolved: dict

    @property
    def paper(self):
        return self.resolved['paper']

    @property
    def adaptation(self):
        return self.resolved['adaptation']

    @property
    def initialization(self):
        return self.resolved['initialization']

    @property
    def experiment_id(self):
        return self.resolved.get('experiment',{}).get('id','A0')

    @property
    def initialization_speed(self):
        return self.resolved.get('experiment',{}).get('initialization_speed_mps',self.paper['initialization_speed'])

    def substeps(self, physics_dt):
        if not math.isfinite(physics_dt) or physics_dt <= 0:
            raise ValueError('physics_dt must be positive and finite')
        count = self.paper['control_dt'] / physics_dt
        if abs(count - round(count)) > 1e-10 or round(count) < 1:
            raise ValueError('control_dt must be an integer multiple of physics_dt')
        return int(round(count))


def load_config(path='params/research/lmpc.yaml'):
    path = Path(path)
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict) or set(data)-{'experiment'} != {
            'schema_version', 'map_name', 'paper', 'adaptation', 'initialization', 'sources'}:
        raise ValueError('invalid Track B configuration keys')
    if 'experiment' in data:
        experiment=data['experiment']
        if (not isinstance(experiment,dict) or set(experiment)-{'excitation'}!={'id','initialization_speed_mps'}
                or experiment['id'] not in ('A0','A1','A2','A2.5')
                or type(experiment['initialization_speed_mps']) not in (int,float)
                or experiment['initialization_speed_mps']!={'A0':1.2,'A1':2.5,'A2':2.5,'A2.5':2.5}[experiment['id']]):
            raise ValueError('experiment must pair A0 with 1.2 or A1/A2/A2.5 with 2.5 m/s')
        excitation=experiment.get('excitation')
        if experiment['id'] in ('A2','A2.5'):
            if (not isinstance(excitation,dict) or set(excitation)!={'seed','hold_steps','steering_amplitude_rad','acceleration_amplitude_mps2'}
                    or type(excitation['seed']) is not int or excitation['seed']<0
                    or type(excitation['hold_steps']) is not int or excitation['hold_steps']<1
                    or any(type(excitation[k]) not in (int,float) or not math.isfinite(excitation[k])
                           or not (0<=excitation[k]<=limit if experiment['id']=='A2.5' and k=='steering_amplitude_rad'
                                   else 0<excitation[k]<=limit) for k,limit in
                           [('steering_amplitude_rad',.02),('acceleration_amplitude_mps2',.6)])):
                raise ValueError('A2/A2.5 requires bounded reproducible excitation')
            if experiment['id']=='A2.5' and excitation['steering_amplitude_rad']!=0:
                raise ValueError('A2.5 requires zero steering excitation')
        elif excitation is not None:
            raise ValueError('excitation is restricted to A2/A2.5')
    if type(data['schema_version']) is not int or data['schema_version'] != 1:
        raise ValueError('unsupported schema_version')
    if data['map_name'] != 'esp':
        raise ValueError('this experiment currently supports ESP only')
    paper = data['paper']
    if not isinstance(paper, dict) or set(paper) != set(PAPER):
        raise ValueError('missing or unknown paper parameter')
    # This first milestone intentionally permits no silent paper deviations.
    for name, expected in PAPER.items():
        value = paper[name]
        if value != expected or isinstance(value, bool):
            raise ValueError('paper parameter differs from reference: ' + name)
        if isinstance(expected, int) and type(value) is not int:
            raise ValueError('paper integer required: ' + name)
        if isinstance(expected, list) and (
                not isinstance(value, list) or any(type(v) not in (int, float) for v in value)):
            raise ValueError('invalid distance diagonal: ' + name)
    adaptation = data['adaptation']
    if not isinstance(adaptation, dict) or set(adaptation) != {
            'geometry_path', 'map_yaml', 'purpose', 'timeout', 'smoke_steps'}:
        raise ValueError('invalid adaptation keys')
    if adaptation['purpose'] != 'observation_and_replay_only':
        raise ValueError('planning is not enabled by this configuration')
    for name in ('geometry_path', 'map_yaml'):
        if not isinstance(adaptation[name], str) or not adaptation[name]:
            raise ValueError('invalid asset path: ' + name)
    if (type(adaptation['smoke_steps']) is not int or adaptation['smoke_steps'] < 1
            or type(adaptation['timeout']) not in (int, float)
            or not math.isfinite(adaptation['timeout'])
            or adaptation['timeout'] <= adaptation['smoke_steps'] * paper['control_dt']):
        raise ValueError('invalid smoke_steps or timeout')
    if not isinstance(data['sources'], dict) or set(data['sources']) != {
            'paper', 'map_name', 'adaptation', 'vehicle'} or any(
                not isinstance(v, str) or not v for v in data['sources'].values()):
        raise ValueError('parameter sources are required')
    initialization = data['initialization']
    if not isinstance(initialization, dict) or set(initialization) != {
            'timeout', 'lookahead_m', 'max_abs_e_y_m', 'max_abs_e_psi_rad',
            'settled_after_s', 'max_settled_speed_error_mps', 'footprint_margin_m',
            'preflight_step_m'} or any(
                type(v) not in (int, float) or not math.isfinite(v) or v <= 0
                for v in initialization.values()):
        raise ValueError('invalid initialization settings')
    if initialization['max_abs_e_psi_rad'] >= math.pi:
        raise ValueError('heading limit must be less than pi')
    return ExperimentConfig(path.resolve(), data)
