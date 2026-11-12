"""Synchronous, LiDAR-free two-car candidate tracking and overtaking."""

import math
from time import perf_counter

import numpy as np
from scipy.interpolate import BSpline

from ..collision.discs import OccupancyMap
from ..collision.multi_car import MultiCarSim
from ..core.centreline_control import CentrelinePurePursuit
from ..core.frenet_lap_tracking import CoreLapTracker
from ..core.geometry.tracker import TrackingOptions
from ..run_recorded_candidate_pp import OpenCandidatePP
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
from .mm_bspline_repair import BudgetedDomainGenerator
from .raceline_cruise import RacelineCruise, local_track_gap
from .session import SessionConfig, make_plan
from .two_car_decision import OvertakingSelector


def sample_candidate_xy(geometry, path, length_m, step_m=.02):
    """Sample an analytic candidate, or use its recorded polyline."""
    if not np.isfinite([length_m, step_m]).all() or min(length_m, step_m) <= 0:
        raise ValueError('invalid candidate sampling settings')
    spec = path.get('spline')
    if spec is not None:
        spline = BSpline(spec['knots'], spec['coefficients'], spec['degree'], extrapolate=False)
        origin = float(spec.get('source_s_origin', path.get('reference_s_origin', math.nan)))
        if not math.isfinite(origin) or spline.t[spline.k] > 0 or spline.t[-spline.k-1] < length_m:
            raise ValueError('invalid candidate spline domain')
        distance = np.linspace(0., length_m, max(2, math.ceil(length_m / step_m)) + 1)
        xy, tangent, _, _ = geometry.curve.sample(origin + distance)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        points = xy + spline(distance)[:, None] * normal
    else:
        points = np.array([[p['x'], p['y']] for p in path.get('points', [])], dtype=float)
    if points.ndim != 2 or points.shape[1] != 2 or len(points) < 3 or not np.isfinite(points).all():
        raise ValueError('candidate requires at least three finite XY points')
    return points


def select_candidate_default(plan_event, ego_state, lead_state, preferred_order=(3, 2, 4, 1, 5)):
    """Pick the first candidate that passes the planning and footprint gates."""
    valid = {p['candidate_id']: p for p in plan_event['paths']
             if p['sampled_valid'] and p.get('footprint_sampled_valid')}
    path = next((valid[i] for i in preferred_order if i in valid), None)
    return dict(selected_candidate_id=path['candidate_id'] if path else None,
                selected_path=path, selection_reason='preferred_valid' if path else 'no_valid_candidate',
                costs={})


class EgoCandidateController:
    def __init__(self, geometry, config, vehicle, speed_linked_pp=None):
        self.geometry = geometry
        self.config = config
        self.vehicle = vehicle
        self.speed_linked_pp = speed_linked_pp

    def lookahead(self, speed):
        if self.speed_linked_pp is None:
            return self.config.pp_lookahead_m
        pp = self.speed_linked_pp
        return pp.tal_constant_lookahead + max(0., speed) * pp.tal_variable_lookahead / self.vehicle.max_speed

    def pursuit(self, path):
        xy = sample_candidate_xy(self.geometry, path, self.config.lookahead_m)
        return OpenCandidatePP(xy, self.config.speed_mps, self.config.pp_lookahead_m,
                               self.vehicle.wheelbase, self.vehicle.max_steer)

    def plan(self, pose, path, target_speed=None):
        if path is None:
            return [0., 0.], dict(fallback_used=True, reason='no_selected_candidate')
        try:
            action, diagnostic = self.pursuit(path).plan(np.asarray(pose, dtype=float))
        except ValueError as exc:
            return [0., 0.], dict(fallback_used=True, reason=str(exc))
        if target_speed is not None:
            action[1] = target_speed
        return action.tolist(), dict(diagnostic, fallback_used=False)


class TwoCarOvertakingEnv:
    """Plan from one shared snapshot, then advance both cars together."""
    # Reserve space for the return before the tight ESP bend at s=184–202 m.
    race_pass_wait_zone = (170., 202.)

    def __init__(self, track, domain, ego_speed=1.2, lead_speed=.9, gap=2., ego_start_s=0.,
                 budget_s=.04, wall_clearance=.05, pair_clearance=.05,
                 selector=select_candidate_default, phase=1, original_esp_control=False, path_lookahead_m=3.,
                 rolling_start=False, curve_speed_control=False, measured_bend_caps=(),
                 raceline_path=None, raceline_speed=None, raceline_candidates=True):
        self.track = track
        self.geometry = track.geometry
        self.length = self.geometry.curve.L
        self.vehicle = load_parameter_file('vehicle_params')
        if (not np.isfinite([ego_speed, lead_speed, gap, ego_start_s, budget_s]).all()
                or min(ego_speed, lead_speed) <= 0 or max(ego_speed, lead_speed) > self.vehicle.max_speed
                or not self.geometry.curve.closed or not 0 < gap < self.length or budget_s < 0):
            raise ValueError('invalid two-car settings')
        self.map_id = track.manifest['map_id']
        if original_esp_control and (self.map_id != 'esp' or phase != 2):
            raise ValueError('original control comparison requires ESP Phase 2')
        if curve_speed_control and not original_esp_control:
            raise ValueError('curve speed control requires original ESP control')
        if measured_bend_caps and not curve_speed_control:
            raise ValueError('measured bend caps require curve speed control')
        self.original_esp_control = original_esp_control
        self.sim = MultiCarSim(self.map_id, wall_clearance=wall_clearance, pair_clearance=pair_clearance,
                               enable_lidar=False, occupancy_map=OccupancyMap.from_raster(self.geometry.raster))
        body = self.sim.bodies[0]
        self.config = SessionConfig(speed_mps=ego_speed, lookahead_m=path_lookahead_m, generation_mode='direct_only',
                                    safety_margin_m=body.width / 2 + wall_clearance,
                                    footprint_check_enabled=True,
                                    footprint_length_m=body.length, footprint_width_m=body.width,
                                    footprint_clearance_m=wall_clearance).for_map(self.map_id)
        if (not math.isclose(domain.metadata['margin_m'], self.config.safety_margin_m, abs_tol=1e-12)
                or self.config.lookahead_m >= self.length):
            raise ValueError('incompatible planning margin or track length')
        self.lead = CentrelinePurePursuit(track.control_path, lead_speed, self.config.pp_lookahead_m,
                                        self.vehicle.wheelbase, self.vehicle.max_steer)
        pp_settings = load_parameter_file('GlobalPurePursuit') if original_esp_control else None
        self.steer_ticks = self.sim.params.n_sim_steps if original_esp_control else self.sim.substeps
        self.ego = EgoCandidateController(self.geometry, self.config, self.vehicle, pp_settings)
        self.raceline = None
        self.raceline_candidates = raceline_candidates
        if raceline_path is not None:
            if not original_esp_control or not curve_speed_control:
                raise ValueError('raceline switching requires ESP Phase 2 curve speed control')
            self.raceline = RacelineCruise(raceline_path, self.ego,
                                          ego_speed if raceline_speed is None else raceline_speed)
        elif raceline_speed is not None:
            raise ValueError('raceline speed requires a raceline file')
        if self.ego.lookahead(ego_speed) + .15 >= path_lookahead_m:
            raise ValueError('candidate path is too short for PP lookahead')
        self.start_s = (ego_start_s, ego_start_s + gap)
        self.initial_speeds = (ego_speed, lead_speed) if rolling_start else None
        if phase not in (1, 2):
            raise ValueError('invalid overtaking phase')
        self.domain, self.budget_s, self.phase = domain, budget_s, phase
        self.selector = (OvertakingSelector(self.geometry, self.sim, self.ego, ego_speed, self.steer_ticks,
                                            confirm_pass=original_esp_control, curve_speed_control=curve_speed_control,
                                            measured_bend_caps=measured_bend_caps)
                         if phase == 2 else selector)
        self.frame = None

    def reset(self):
        options = TrackingOptions(max_speed_mps=self.vehicle.max_speed)
        self.trackers = [CoreLapTracker(self.geometry, self.track.control_path, frenet_options=options)
                         for _ in self.start_s]
        self.generator = BudgetedDomainGenerator(self.domain, budget_s=self.budget_s,
                                                 repair_candidates=(1, 2, 4, 5),
                                                 lateral_inset_m=.12 if self.phase == 2 else 0.)
        refs = [self.geometry.reference(s % self.length) for s in self.start_s]
        poses = [[*r['xy'], r['psi']] for r in refs]
        observations, _ = (self.sim.reset(poses) if self.initial_speeds is None
                           else self.sim.reset(poses, speeds=self.initial_speeds))
        if self.phase == 2:
            self.selector.reset()
        self.held_actions = np.zeros((2, 2))
        self.control_step = 0
        self.offsets = None
        self.encounter_active = self.raceline is not None
        return self._observe(observations)

    def _observe(self, observations):
        ego_obs = observations[0]
        states = []
        for tracker, sample in zip(self.trackers, (ego_obs, ego_obs['opponent'])):
            pose = sample['pose']
            speed = sample['vehicle_speed']
            result = tracker.update(pose, ego_obs['simulation_time'], speed)
            states.append(dict(car_id=sample['car_id'], pose=pose.tolist(), speed_mps=speed,
                               physical_state=sample['vehicle_state'].tolist(), valid=result.valid,
                               reason=result.reason, control_step=self.control_step,
                               sim_time_s=ego_obs['simulation_time'],
                               frenet=tracker.frenet if result.valid else dict(valid=False, reason=result.reason),
                               completed_full_laps=tracker.completed_full_laps))
        ego, lead = states
        lead.update(delta_s_wrapped_m=None, delta_s_unwrapped_m=None,
                    euclidean_distance_m=math.dist(ego['pose'][:2], lead['pose'][:2]))
        valid = ego['valid'] and lead['valid']
        if valid:
            raw = [state['frenet']['s_unwrapped'] for state in states]
            if self.offsets is None:
                self.offsets = [round((start - s) / self.length) * self.length
                                for start, s in zip(self.start_s, raw)]
            lead['delta_s_wrapped_m'] = (lead['frenet']['s_wrapped'] - ego['frenet']['s_wrapped']) % self.length
            lead['delta_s_unwrapped_m'] = raw[1] + self.offsets[1] - raw[0] - self.offsets[0]
            if self.raceline is not None:
                lead['delta_s_local_m'] = local_track_gap(lead['delta_s_unwrapped_m'], self.length)
        reason = ('envelope_event' if self.sim.done else None if valid else 'tracking_error')
        self.frame = dict(time_s=ego_obs['simulation_time'], ego_state=ego, lead_state=lead,
                          events=ego_obs['events'], terminal_reason=reason)
        return self.frame

    def _step_with_steering_updates(self, path, target_speed, pursuit=None):
        actions = self.held_actions.copy()
        actions[:, 1] = [target_speed, self.lead.speed]
        pp = self.ego.pursuit(path) if path is not None else pursuit
        if pp is None:
            actions[0] = [0., 0.]
        diagnostic = dict(fallback_used=pp is None)
        updates = []

        def update(tick, current):
            recompute = tick % self.steer_ticks == 0
            if recompute:
                rear, front = self.sim.cars
                if pp is not None:
                    try:
                        command, info = pp.plan(rear.state[[0, 1, 4]], self.ego.lookahead(rear.state[3]))
                        current[0, 0] = command[0]
                        if not updates:
                            diagnostic.update(info)
                    except ValueError as exc:
                        current[0] = [0., 0.]
                        diagnostic.update(fallback_used=True, reason=str(exc))
                current[1] = self.lead.plan(dict(pose=front.state[[0, 1, 4]]), self.ego.lookahead(front.state[3]))
            if not updates or recompute:
                updates.append(dict(physics_tick=tick, time_s=self.sim.time,
                                    steering_recomputed=recompute, actions=current.tolist()))
            return current

        observations, _ = self.sim.step(actions, update_actions=update)
        self.held_actions = actions
        self.selector.held_steering = float(actions[0, 0])
        return observations, updates[0]['actions'], diagnostic, updates

    def _make_plan(self, ego, lead):
        if self.phase == 2:
            policy = self.selector.plan_policy(lead, ego)
            if policy != self.generator.target_policy:
                self.generator.previous.clear()
                self.generator.target_policy = policy
        return make_plan(self.geometry, ego, self.config, self.control_step, generator=self.generator)

    def step(self):
        if self.frame is None or self.frame['terminal_reason'] is not None:
            raise RuntimeError('reset a nonterminal session before stepping')
        started = perf_counter()
        ego, lead = self.frame['ego_state'], self.frame['lead_state']
        race_options = []
        if self.raceline is not None:
            lead = dict(lead, delta_s_unwrapped_m=lead['delta_s_local_m'])
            gap = lead['delta_s_unwrapped_m']
            encounter_started = not self.encounter_active and 0. <= gap <= self.raceline.encounter_m
            self.encounter_active |= encounter_started
            if not self.encounter_active:
                pp, speed, race_options, decision = self.selector.try_raceline(self.raceline, ego, lead)
                if pp is not None:
                    plan = self._make_plan(ego, lead) if self.raceline_candidates else None
                    observations, actions, diagnostic, updates = self._step_with_steering_updates(None, speed, pp)
                    self.control_step += 1
                    frame = self._observe(observations)
                    return dict(plan=plan, selected_candidate_id=None, selection_reason='raceline',
                                costs=dict(options=race_options), actions=actions, ego_control=diagnostic,
                                steering_updates=updates, decision=decision, raceline_xy=pp.path.tolist(),
                                cycle_wall_time_s=perf_counter()-started), frame
        plan = self._make_plan(ego, lead)
        passes_before = self.selector.completed_passes if self.raceline is not None else None
        if self.raceline is not None:
            start, end = self.race_pass_wait_zone
            pass_start_allowed = not start <= ego['frenet']['s_wrapped'] < end
            choice = self.selector(plan, ego, lead, pass_start_allowed=pass_start_allowed)
        else:
            choice = self.selector(plan, ego, lead)
        if self.raceline is not None:
            if self.selector.completed_passes > passes_before:
                self.encounter_active = False
            choice['decision'].update(local_track_gap_m=gap,
                                      encounter_started=encounter_started,
                                      pass_start_allowed=pass_start_allowed,
                                      raceline_rejected=bool(race_options))
            choice['costs']['options'][:0] = race_options
        selected_id = choice['selected_candidate_id']
        path = next((p for p in plan['paths'] if p['candidate_id'] == selected_id
                     and p['sampled_valid'] and p.get('footprint_sampled_valid')), None)
        if selected_id is not None and path is None:
            raise ValueError('selector must return a valid candidate ID or None')
        target_speed = choice.get('target_speed_mps')
        if target_speed is not None and not 0. <= target_speed <= self.config.speed_mps:
            raise ValueError('selector returned an invalid speed')
        if self.original_esp_control:
            observations, actions, diagnostic, updates = self._step_with_steering_updates(path, target_speed)
        else:
            ego_action, diagnostic = self.ego.plan(ego['pose'], path, target_speed)
            actions = [ego_action, self.lead.plan(lead).tolist()]
            observations, _ = self.sim.step(actions)
        self.control_step += 1
        frame = self._observe(observations)
        transition = dict(plan=plan, selected_candidate_id=selected_id, selection_reason=choice['selection_reason'],
                          costs=choice['costs'], actions=actions, ego_control=diagnostic,
                          cycle_wall_time_s=perf_counter() - started)
        if self.original_esp_control:
            transition['steering_updates'] = updates
        if self.phase == 2:
            transition['decision'] = choice['decision']
        return transition, frame
