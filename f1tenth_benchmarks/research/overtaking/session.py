"""Streaming recording and synchronous planning; candidates never control the car."""
from dataclasses import asdict, dataclass, field, replace
import json
import math
from pathlib import Path
from time import perf_counter

from .robust_adaptive_paths import generate_robust_paths
from .reference_path import REFERENCE_PATH, STATE_PATH, check_reference_path


def vehicle_curvature_limit():
    """Use the same steering/wheelbase parameters as the PP controller."""
    import yaml
    parameters = yaml.safe_load((Path(__file__).resolve().parents[3]/'params'/'vehicle_params.yaml').read_text())
    return math.tan(float(parameters['max_steer']))/float(parameters['wheelbase'])


@dataclass(frozen=True)
class SessionConfig:
    speed_mps: float = 1.2
    pp_lookahead_m: float = None  # resolved by map: MY 0.3 m, ESP 0.6 m
    control_dt_s: float = .1
    timeout_s: float = 300.
    laps: int = 1
    plan_period_s: float = .1
    lookahead_m: float = 3.
    safety_margin_m: float = .2
    sample_step_m: float = .25
    max_slope: float = 1.
    max_curvature_inv_m: float = field(default_factory=vehicle_curvature_limit)
    endpoint_tolerance_m: float = 0.
    max_refinements: int = 2
    max_iterations: int = 150
    max_start_position_error_m: float = 1e-4
    max_start_heading_error_rad: float = 1e-5
    max_lateral_error_m: float = .25
    max_heading_error_rad: float = .5
    max_control_steps: int = 0  # zero means no short-preview limit
    query_mode: str = 'exact'
    generation_mode: str = 'optimized'
    validation_step_m: float = None  # research override; None keeps original raster-half policy

    def __post_init__(self):
        nonnegative = ('safety_margin_m', 'endpoint_tolerance_m')
        integer_limits = dict(laps=(1, 100), max_refinements=(0, 5), max_iterations=(1, 1000),
                              max_control_steps=(0, 10000000))
        for name, value in asdict(self).items():
            if name == 'pp_lookahead_m':
                if value is not None and (isinstance(value, bool) or not math.isfinite(value) or value <= 0):
                    raise ValueError('invalid pp_lookahead_m')
                continue
            if name == 'validation_step_m':
                if value is not None and (isinstance(value, bool) or not math.isfinite(value)
                                         or not 0 < value <= .05
                                         or self.query_mode in ('fixed_grid_with_fallback', 'fixed_grid_lookup')):
                    raise ValueError('invalid validation_step_m')
                continue
            if name == 'query_mode':
                if value not in ('exact', 'reused_exact', 'fast_with_fallback', 'fixed_grid_with_fallback', 'fixed_grid_lookup'):
                    raise ValueError('invalid query_mode')
                continue
            if name == 'generation_mode':
                if value not in ('optimized', 'direct_only', 'direct_then_repair',
                                  'width_fraction_only', 'width_fraction_then_repair'):
                    raise ValueError('invalid generation_mode')
                continue
            if name in integer_limits:
                low, high = integer_limits[name]
                if type(value) is not int or not low <= value <= high:
                    raise ValueError('invalid '+name)
            elif isinstance(value, bool) or not math.isfinite(value) or (
                    value < 0 if name in nonnegative else value <= 0):
                raise ValueError('invalid '+name)

    def for_map(self, map_id):
        """Resolve an automatic PP distance, preserving explicit overrides."""
        if self.pp_lookahead_m is not None:
            return self
        return replace(self, pp_lookahead_m=.3 if map_id == 'my_map' else .6)


def reject_paths(reason, status='rejected'):
    return [dict(candidate_id=i, status=status, sampled_valid=False, reason=reason,
                 points=[], start_position_error_m=None, start_heading_error_rad=None)
            for i in range(1, 6)]


def make_plan(geometry, state, config, plan_id, trigger='periodic', generator=None):
    """Five slots even on invalid localization or whole-generator rejection."""
    generator = generator or generate_robust_paths
    started = perf_counter()
    event = dict(schema_version=1, plan_id=plan_id, trigger=trigger,
                 source_control_step=state['control_step'], source_sim_time_s=state['sim_time_s'],
                 available_control_step=state['control_step'], available_sim_time_s=state['sim_time_s'],
                 source_pose=state['pose'], geometry_id=geometry.geometry_id,
                 validation_scope='sampled_center_points_and_polyline_only',
                 footprint_verified=False, planning_allowed=False,
                 generator_settings={k: v for k, v in asdict(config).items() if k in (
                     'lookahead_m', 'safety_margin_m', 'sample_step_m', 'max_slope',
                     'max_curvature_inv_m', 'endpoint_tolerance_m', 'max_refinements', 'max_iterations',
                     'query_mode', 'generation_mode')})
    if config.validation_step_m is not None:
        event['generator_settings']['validation_step_m'] = config.validation_step_m
    if not state['valid'] or not state['frenet']['valid']:
        event.update(status='skipped', reason=state.get('reason') or state['frenet'].get('reason'),
                     paths=reject_paths('invalid_source_state', 'skipped'))
    else:
        x, y, yaw = state['pose']
        try:
            result = generator(geometry, x, y, yaw=yaw,
                lookahead_m=config.lookahead_m, safety_margin_m=config.safety_margin_m,
                sample_step_m=config.sample_step_m, max_slope=config.max_slope,
                max_curvature=config.max_curvature_inv_m,
                endpoint_tolerance_m=config.endpoint_tolerance_m,
                max_refinements=config.max_refinements, max_iterations=config.max_iterations,
                query_mode=config.query_mode,
                **({'validation_step_m': config.validation_step_m}
                   if config.validation_step_m is not None else {}),
                **({'generation_mode': config.generation_mode} if config.generation_mode != 'optimized' else {}))
        except ValueError as exc:
            event.update(status='rejected', reason=str(exc), paths=reject_paths(str(exc)))
        else:
            paths = result['paths']
            if [p['candidate_id'] for p in paths] != [1, 2, 3, 4, 5]:
                raise RuntimeError('generator must return exactly five ordered IDs')
            # Gate independently against the actual source pose. A historical
            # heading_matched flag or a duplicated diagnostic cannot bypass it.
            for path in paths:
                points = path['points']
                position_error = heading_error = None
                if points:
                    first = points[0]
                    try:
                        position_error = math.hypot(first['x']-x, first['y']-y)
                        heading_error = (first['yaw']-yaw+math.pi) % (2*math.pi)-math.pi
                    except (KeyError, TypeError, ValueError):
                        pass
                path.update(start_position_error_m=position_error, start_heading_error_rad=heading_error)
                path_type = path.get('path_type', STATE_PATH)
                if path_type == REFERENCE_PATH:
                    if path['sampled_valid']:
                        check = check_reference_path(geometry, path, state['pose'], config.lookahead_m,
                            config.max_lateral_error_m, config.max_heading_error_rad)
                        path['reference_contract_check'] = check
                        if not check['valid']:
                            path.update(sampled_valid=False, status='rejected', reason=check['reason'])
                    # A genuine d=0 reference is expected to differ from the
                    # current vehicle pose. Never apply the trajectory gate.
                    continue
                if path_type != STATE_PATH:
                    path.update(sampled_valid=False, status='rejected', reason='unknown_path_type')
                    continue
                if path['sampled_valid'] and (
                    position_error is None or heading_error is None or
                    not math.isfinite(position_error) or not math.isfinite(heading_error) or
                    position_error > config.max_start_position_error_m or
                    abs(heading_error) > config.max_start_heading_error_rad):
                    path.update(sampled_valid=False, status='rejected', reason='start_pose_mismatch')
            result['sampled_valid_count'] = sum(p['sampled_valid'] for p in paths)
            if 'valid_ids' in result:
                result['valid_ids'] = [p['candidate_id'] for p in paths if p['sampled_valid']]
            if 'fallback_candidate_id' in result:
                result['fallback_candidate_id'] = (3 if paths[2].get('path_type') == REFERENCE_PATH
                                                    and paths[2]['sampled_valid'] else None)
                result['requires_stop'] = result['sampled_valid_count'] == 0
            event.update(status='completed', reason=None, paths=paths,
                         result={k: v for k, v in result.items() if k != 'paths'})
    event['sampled_valid_count'] = sum(p['sampled_valid'] for p in event['paths'])
    event['wall_time_s'] = perf_counter()-started
    if config.generation_mode != 'optimized':
        event['cycle_budget_observation'] = dict(target_s=config.control_dt_s,
            completed_within_target=event['wall_time_s'] <= config.control_dt_s,
            timely_valid_count=event['sampled_valid_count'] if event['wall_time_s'] <= config.control_dt_s else 0,
            deadline_enforced=False, scope='make_plan_excludes_recording_and_control')
    return event


class PlanScheduler:
    """Sample the latest state at each due simulation-time boundary; no catch-up."""
    def __init__(self, period_s, origin_s):
        if not math.isfinite(period_s) or period_s <= 0 or not math.isfinite(origin_s):
            raise ValueError('invalid schedule')
        self.period_s, self.origin_s = period_s, origin_s
        self.next_tick = 0

    def due(self, time_s):
        if time_s+1e-9 < self.origin_s+self.next_tick*self.period_s:
            return False
        self.next_tick = math.floor((time_s-self.origin_s+1e-9)/self.period_s)+1
        return True


class SessionWriter:
    """Exclusive sessions, append-only streams and reconstructible byte offsets."""
    def __init__(self, directory, metadata):
        self.root = Path(directory)
        self.root.mkdir(parents=True, exist_ok=False)
        self.streams = {}
        self.index = {name: [] for name in ('states', 'actions', 'plans')}
        self.closed = False
        self.last_state = self.last_plan = None
        try:
            with (self.root/'metadata.json').open('x') as stream:
                json.dump(metadata, stream, indent=2, allow_nan=False)
            for name in self.index:
                self.streams[name] = (self.root/(name+'.jsonl')).open('xb')
        except BaseException:
            for stream in self.streams.values():
                stream.close()
            raise

    def append(self, name, value):
        if self.closed:
            raise RuntimeError('session is closed')
        encoded = (json.dumps(value, allow_nan=False, separators=(',', ':'))+'\n').encode()
        stream = self.streams[name]
        if name == 'states':
            key = dict(control_step=value['control_step'], sim_time_s=value['sim_time_s'])
            self.last_state = key
        elif name == 'plans':
            key = dict(plan_id=value['plan_id'], source_control_step=value['source_control_step'],
                       source_sim_time_s=value['source_sim_time_s'])
            self.last_plan = key
        else:
            key = dict(source_control_step=value['source_control_step'], target_control_step=value['target_control_step'])
        self.index[name].append(dict(key, byte_offset=stream.tell(), byte_length=len(encoded)))
        stream.write(encoded)
        stream.flush()

    def finalize(self, summary):
        if self.closed:
            return
        try:
            for stream in self.streams.values():
                stream.close()
            with (self.root/'playback_index.json').open('x') as stream:
                json.dump(dict(schema_version=1, streams=self.index), stream, allow_nan=False)
            with (self.root/'summary.json').open('x') as stream:
                json.dump(summary, stream, indent=2, allow_nan=False)
        finally:
            self.closed = True


def latest_plan(plans, control_step):
    """Playback lookup; failures replace prior results, and future plans stay hidden."""
    eligible = [p for p in plans if max(p['source_control_step'], p['available_control_step']) <= control_step]
    return max(eligible, key=lambda p: (p['source_control_step'], p['plan_id'])) if eligible else None
