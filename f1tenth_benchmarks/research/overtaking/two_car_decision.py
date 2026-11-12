"""Phase 2 selection using time-aligned rollouts and ordered preferences."""

from copy import copy
from time import perf_counter

import numpy as np


class CurveSpeedLimit:
    """Periodic speed ceiling that reserves braking distance before bends."""

    def __init__(self, curve, cruise_speed, measured_bend_caps=()):
        self.length = curve.L
        self.s = np.linspace(0., curve.L, int(np.ceil(curve.L / .05)), endpoint=False)
        self.curvature = np.abs(curve.sample(self.s)[3])
        self.cruise_speed = cruise_speed
        self.lateral_accel_mps2 = 3.5
        self.braking_mps2 = 1.5
        self.preview_m = 8.
        self.measured_bend_caps = measured_bend_caps
        self.distances = np.linspace(0., self.preview_m, 81)

    def at(self, s):
        sites = (s + self.distances) % self.length
        kappa = np.interp(sites, self.s, self.curvature, period=self.length)
        corner_speed_sq = self.lateral_accel_mps2 / np.maximum(kappa, 1e-6)
        for start, end, speed in self.measured_bend_caps:
            corner_speed_sq[(sites >= start) & (sites <= end)] = speed**2
        # Reserve the distance travelled during one planning step before braking.
        brake_distance = np.maximum(0., self.distances - .1 * self.cruise_speed)
        braking_ceiling_sq = corner_speed_sq + 2 * self.braking_mps2 * brake_distance
        return min(self.cruise_speed, float(np.sqrt(braking_ceiling_sq.min())))


class OvertakingSelector:
    """Filter predicted wall and pair conflicts before ranking candidate/speed pairs."""

    def __init__(self, geometry, sim, controller, cruise_speed, steer_ticks, confirm_pass=False, curve_speed_control=False, measured_bend_caps=()):
        self.geometry = geometry
        self.sim = sim
        self.controller = controller
        self.cruise_speed = cruise_speed
        self.steer_ticks = steer_ticks
        self.confirm_pass = confirm_pass
        self.speed_limit = CurveSpeedLimit(geometry.curve, cruise_speed, measured_bend_caps) if curve_speed_control else None
        self.body = sim.bodies[0]
        self.dt = sim.params.timestep
        self.sample_steps = max(1, round(.05 / self.dt))
        self.sample_dt = self.sample_steps * self.dt
        self.max_horizon = 2.
        self.pair_margin = sim.pair_clearance + .04
        self.wall_margin = sim.wall_clearance + .02
        self.follow_gap = 1.15
        self.return_gap = 1.0
        self.reset()

    def reset(self):
        self.held_steering = 0.
        self.side = None
        self.mode = 'cruise'
        self.retry_after_s = 0.
        self.completed_passes = 0
        self.pending_side = None
        self.pending_steps = 0

    def try_raceline(self, raceline, ego, lead):
        """Approve the optimized reference with the usual time-aligned body checks."""
        started = perf_counter()
        gap = lead['delta_s_unwrapped_m']
        pp, ceiling = raceline.local_plan(ego['pose'], gap)
        times = np.arange(17) * self.sample_dt
        poses, lead_rate = self._predict_lead(lead, times)
        lead_centres = [self.body.centres(pose) for pose in poses]
        segment_m = np.sqrt(pp.l2s)
        path_s = np.r_[0., np.cumsum(segment_m)]
        options = []
        for speed in (ceiling, .85 * ceiling, .70 * ceiling):
            result = self._rollout(pp, speed, lead_centres, lead_rate, times, path_s=path_s)
            options.append(dict(candidate_id=None, reference='raceline', target_speed_mps=speed, **result))
            if result['safe']:
                self.mode = 'raceline'
                return pp, speed, options, dict(
                    mode='raceline', side=None, completed_passes=self.completed_passes,
                    local_track_gap_m=gap, cruise_speed_ceiling_mps=ceiling,
                    pair_threshold_m=self.pair_margin, wall_threshold_m=self.wall_margin,
                    lead_s_rate_mps=lead_rate, lead_prediction_poses=poses.tolist(),
                    sample_dt_s=self.sample_dt, horizon_s=float(times[-1]),
                    wall_time_s=perf_counter()-started)
        return None, None, options, None

    def plan_policy(self, lead, ego=None):
        if self.side is not None and (self.mode == 'return' or lead['delta_s_unwrapped_m'] <= -self.return_gap):
            return 'centerline_return'
        if self.speed_limit is not None and self.side is None and ego is not None:
            frenet = ego['frenet']
            if abs(frenet['d']) > .15 or abs(frenet['e_psi']) > .30:
                return 'centerline_return'
        return 'reference_centerline'

    def _predict_lead(self, state, times):
        frenet = state['frenet']
        rate = max(0., state['speed_mps'] * np.cos(frenet['e_psi']) / frenet['jacobian'])
        xy, tangent, heading, _ = self.geometry.curve.sample(frenet['s_wrapped'] + rate * times)
        normal = np.column_stack((-tangent[:, 1], tangent[:, 0]))
        yaw = heading + frenet['e_psi'] * np.maximum(1. - times / .3, 0.)
        poses = np.column_stack((xy + frenet['d'] * normal, yaw))
        poses[0] = state['pose']
        return poses, rate

    def _rollout(self, pp, speed, lead_centres, lead_rate, times, path_s=None):
        # Copy simulator state and steering delay without reseeding or advancing the real car.
        car = copy(self.sim.cars[0])
        car.state = car.state.copy()
        car.steer_buffer = car.steer_buffer.copy()
        first_index = pp.nearest(car.state[:2])[2:]
        steer = self.held_steering
        minimum = float('inf')
        first_issue = None
        for i, t in enumerate(times):
            if i:
                try:
                    for substep in range(self.sample_steps):
                        tick = (i - 1) * self.sample_steps + substep
                        if (self.sim.physics_tick + tick) % self.steer_ticks == 0:
                            action, _ = pp.plan(car.state[[0, 1, 4]], self.controller.lookahead(car.state[3]))
                            steer = action[0]
                        car.update_pose(steer, speed)
                except ValueError:
                    if first_issue is None:
                        first_issue = ('lookahead_unavailable', float(t))
                    break
            centres = self.body.centres(car.state[[0, 1, 4]])
            clearance = float(np.linalg.norm(centres[:, None] - lead_centres[i][None, :], axis=2).min()
                              - 2 * self.body.radius)
            minimum = min(minimum, clearance)
            if first_issue is None:
                if clearance <= self.pair_margin:
                    first_issue = ('predicted_pair_clearance', float(t))
                elif self.sim.map.intersects(centres, self.body.radius + self.wall_margin):
                    first_issue = ('predicted_wall_clearance', float(t))
        final_index = pp.nearest(car.state[:2])[2:]
        if path_s is None:
            s_per_segment = self.controller.config.lookahead_m / (len(pp.path) - 1)
            advance = (sum(final_index) - sum(first_index)) * s_per_segment
        else:
            def at(projection):
                fraction, index = projection
                return path_s[index] + fraction*(path_s[index+1]-path_s[index])
            advance = at(final_index) - at(first_index)
        relative_progress = advance - lead_rate * t
        return dict(safe=first_issue is None, reason=first_issue[0] if first_issue else None,
                    first_rejection_s=first_issue[1] if first_issue else None, min_pair_clearance_m=minimum,
                    relative_s_progress_m=relative_progress)

    def __call__(self, plan_event, ego, lead, *, pass_start_allowed=True):
        started = perf_counter()
        config = self.controller.config
        cruise_speed = self.cruise_speed if self.speed_limit is None else self.speed_limit.at(ego['frenet']['s_wrapped'])
        # An open PP path needs room beyond the last predicted ego position.
        max_speed = max(cruise_speed, ego['speed_mps'])
        reserve = .30 if self.speed_limit is not None else .15
        usable_path = config.lookahead_m - self.controller.lookahead(max_speed) - reserve
        horizon = min(self.max_horizon, usable_path / max_speed)
        times = np.arange(max(1, int(horizon / self.sample_dt)) + 1) * self.sample_dt
        poses, lead_rate = self._predict_lead(lead, times)
        lead_centres = [self.body.centres(pose) for pose in poses]
        paths = {p['candidate_id']: p for p in plan_event['paths']}
        options = []
        controllers = {}

        def evaluate(candidate_id, speed, require_lane_room=False):
            path = paths[candidate_id]
            result = dict(candidate_id=candidate_id, target_speed_mps=speed, safe=False)
            if not path['sampled_valid'] or not path.get('footprint_sampled_valid'):
                result.update(reason='static_rejection', static_reason=path['reason'])
            elif require_lane_room and abs(path['target_d_m'] - lead['frenet']['d']) <= (
                    2 * self.body.radius + self.pair_margin):
                result.update(reason='insufficient_lateral_room')
            else:
                try:
                    if candidate_id not in controllers:
                        controllers[candidate_id] = self.controller.pursuit(path)
                    pp = controllers[candidate_id]
                except ValueError as exc:
                    result.update(reason='invalid_tracking_path', static_reason=str(exc))
                else:
                    result.update(self._rollout(pp, speed, lead_centres, lead_rate, times))
                    if (require_lane_room and self.confirm_pass and result['safe']
                            and result['relative_s_progress_m'] <= .05):
                        result.update(safe=False, reason='insufficient_pass_progress')
            options.append(result)
            return result

        def best_side(ids, speed, require_lane_room=False):
            safe = [opt for i in ids if (opt := evaluate(i, speed, require_lane_room))['safe']]
            if not safe:
                return None
            clearance = max(opt['min_pair_clearance_m'] for opt in safe)
            comparable = [opt for opt in safe if opt['min_pair_clearance_m'] >= clearance - .015]
            return max(comparable, key=lambda opt: (opt['relative_s_progress_m'], -abs(opt['candidate_id']-3)))

        def safe_follow(speed, ceiling):
            speeds = sorted({speed, min(speed, ceiling), .5 * min(speed, ceiling)}, reverse=True)
            for target in speeds:
                if target < cruise_speed:
                    option = evaluate(3, target)
                    if option['safe']:
                        return option
            return None

        gap = lead['delta_s_unwrapped_m']
        now = ego['sim_time_s']
        follow_speed = float(np.clip(lead['speed_mps'] + .7 * (gap - self.follow_gap), 0., cruise_speed))
        # Once ahead, slow down only if a safe continuation demands it. Slowing
        # to the old lead's speed as a rule would invite it to catch up again.
        side_speed = follow_speed if gap >= 0. else min(cruise_speed, lead['speed_mps'])
        selected = None
        mode = 'yield'
        side_available = False
        if self.side is not None:
            if self.plan_policy(lead) == 'centerline_return':
                centre = evaluate(3, cruise_speed)
                if centre['safe']:
                    selected, mode = centre, 'return'
                    if (gap <= -self.return_gap and abs(ego['frenet']['d']) < .06
                            and abs(ego['frenet']['e_psi']) < .15):
                        self.side, mode = None, 'cruise'
                        self.completed_passes += 1
            if selected is None:
                same_side = (2, 1) if self.side < 0 else (4, 5)
                selected = best_side(same_side, cruise_speed)
                if selected is None and side_speed < cruise_speed:
                    selected = best_side(same_side, side_speed)
                if selected is not None:
                    mode = 'pass'
                elif gap > 0.:
                    # When a lane closes before passing, try a checked return
                    # behind the lead instead of holding the side indefinitely.
                    retreat_speed = min(follow_speed, lead['speed_mps'])
                    retreat = safe_follow(retreat_speed, retreat_speed)
                    if retreat is not None:
                        selected, mode = retreat, 'follow'
                        self.side = None
                        self.retry_after_s = now + 2.
        else:
            cruise = evaluate(3, cruise_speed)
            hold_gap = self.mode == 'follow' and gap < self.follow_gap + .25 and follow_speed < cruise_speed
            if self.confirm_pass and gap < 0.:
                hold_gap = False
            if cruise['safe'] and not hold_gap:
                selected, mode = cruise, 'cruise'
            elif (pass_start_allowed and cruise['reason'] == 'predicted_pair_clearance' and gap > 0.
                  and lead_rate < cruise_speed and now >= self.retry_after_s):
                sides = (2, 4, 1, 5)
                selected = best_side(sides, cruise_speed, require_lane_room=True)
                if selected is None and follow_speed < cruise_speed:
                    selected = best_side(sides, follow_speed, require_lane_room=True)
                if selected is not None:
                    side_available = True
                    side = -1 if selected['candidate_id'] < 3 else 1
                    if self.confirm_pass and self.mode == 'follow':
                        self.pending_steps = self.pending_steps + 1 if side == self.pending_side else 1
                        self.pending_side = side
                        if self.pending_steps < 3:
                            selected = None
                    if selected is not None:
                        self.side, mode = side, 'pass'
            if selected is None:
                if self.confirm_pass and gap < 0.:
                    selected = safe_follow(.85 * cruise_speed, .75 * cruise_speed)
                    if selected is not None:
                        mode = 'pace'
                else:
                    selected = safe_follow(follow_speed, lead['speed_mps'])
                    if selected is not None:
                        mode = 'follow'

        if mode != 'follow' or not side_available:
            self.pending_side, self.pending_steps = None, 0
        self.mode = mode
        cid = selected['candidate_id'] if selected else None
        speed = selected['target_speed_mps'] if selected else 0.
        return dict(selected_candidate_id=cid, selected_path=paths.get(cid), target_speed_mps=speed,
                    selection_reason=mode, costs=dict(options=options), decision=dict(
                        mode=mode, side=self.side, completed_passes=self.completed_passes,
                        pending_side=self.pending_side, pending_steps=self.pending_steps,
                        cruise_speed_ceiling_mps=cruise_speed,
                        pair_threshold_m=self.pair_margin, wall_threshold_m=self.wall_margin,
                        lead_s_rate_mps=float(lead_rate), lead_prediction_poses=poses.tolist(),
                        sample_dt_s=self.sample_dt, horizon_s=float(times[-1]),
                        wall_time_s=perf_counter()-started))
