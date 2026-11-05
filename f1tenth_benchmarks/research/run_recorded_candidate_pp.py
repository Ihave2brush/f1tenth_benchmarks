"""Branch a recorded physical state and follow one fixed open candidate.

No replanning, state reset, artificial path closure, or mutation of the source.
Collision uses the existing simulator four-corner checker at each physics tick;
it is not a full swept-footprint certificate.
"""
import argparse
from argparse import Namespace
import hashlib
import json
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from f1tenth_benchmarks.classic_racing.GlobalPurePursuit import (
    get_actuation, nearest_point_on_trajectory_py2,
    first_point_on_trajectory_intersecting_circle,
)
from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from f1tenth_benchmarks.simulator.f1tenth_sim import F1TenthSimBase
from f1tenth_benchmarks.simulator.laser_models import ScanSimulator2D
from .core.geometry.map_raster import load_map


class OpenCandidatePP:
    def __init__(self, points, speed, lookahead, wheelbase, max_steer):
        self.path = np.asarray(points, float)
        if self.path.ndim != 2 or self.path.shape[1] != 2 or len(self.path) < 3:
            raise ValueError('invalid open candidate')
        self.diffs = np.diff(self.path, axis=0)
        self.l2s = np.sum(self.diffs**2, axis=1)
        if not np.isfinite(self.path).all() or np.any(self.l2s < 1e-12):
            raise ValueError('nonfinite or degenerate candidate')
        if not np.isfinite([speed, lookahead, wheelbase, max_steer]).all() or min(speed, lookahead, wheelbase, max_steer) <= 0:
            raise ValueError('invalid PP parameters')
        self.speed, self.lookahead = speed, lookahead
        self.wheelbase, self.max_steer = wheelbase, max_steer

    def nearest(self, xy):
        return nearest_point_on_trajectory_py2(xy, self.path, self.l2s, self.diffs)

    def plan(self, pose):
        _, distance, fraction, index = self.nearest(pose[:2])
        target, _, _ = first_point_on_trajectory_intersecting_circle(
            pose[:2], self.lookahead, self.path, index+fraction, wrap=False)
        if target is None:
            raise ValueError('open_path_lookahead_unavailable')
        steer = float(get_actuation(pose[2], target, pose[:2],
                                   np.linalg.norm(target-pose[:2]), self.wheelbase))
        return np.array([np.clip(steer, -self.max_steer, self.max_steer), self.speed]), dict(
            nearest_error_m=float(distance), lookahead_xy=target.tolist(),
            unconstrained_steering_rad=steer, steering_clipped=abs(steer)>self.max_steer)


def read_indexed(root, index, name, row):
    entry = index['streams'][name][row]
    with (root/(name+'.jsonl')).open('rb') as stream:
        stream.seek(entry['byte_offset'])
        raw = stream.read(entry['byte_length'])
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


def run(source, output, step=807, candidate_id=1, duration=3., regenerate_lookahead=None):
    source, output = Path(source), Path(output)
    if output.exists():
        raise FileExistsError('choose a new experiment directory')
    meta = json.loads((source/'metadata.json').read_text())
    index = json.loads((source/'playback_index.json').read_text())
    state, state_hash = read_indexed(source, index, 'states', step)
    plan, plan_hash = read_indexed(source, index, 'plans', step)
    original_plan_hash = plan_hash
    if regenerate_lookahead is not None:
        from .core.geometry import load_geometry
        from .overtaking.session import SessionConfig, make_plan
        geometry = load_geometry('maps/frenet/esp/geometry', 'maps/esp.yaml', inspect=True)
        if geometry.geometry_id != meta['geometry_id']:
            raise ValueError('regeneration geometry mismatch')
        config = replace(SessionConfig(**meta['config']), lookahead_m=regenerate_lookahead)
        plan = make_plan(geometry, state, config, step, 'fixed_candidate_experiment')
        plan_hash = hashlib.sha256(json.dumps(plan, sort_keys=True, allow_nan=False).encode()).hexdigest()
    candidate = next(p for p in plan['paths'] if p['candidate_id'] == candidate_id)
    if not state['valid'] or not candidate['sampled_valid']:
        if regenerate_lookahead is not None:
            output.mkdir(parents=True, exist_ok=False)
            (output/'regenerated_plan.json').write_text(json.dumps(plan, indent=2, allow_nan=False)+'\n')
            (output/'summary.json').write_text(json.dumps(dict(
                tracking_started=False, source_step=step, candidate_id=candidate_id,
                candidate_lookahead_m=regenerate_lookahead,
                end_reason=candidate.get('reason') or 'invalid_source',
                sampled_valid_count=plan['sampled_valid_count']), indent=2)+'\n')
        raise ValueError('requires a valid source and accepted candidate')
    if plan['source_control_step'] != step or plan['source_pose'] != state['pose']:
        raise ValueError('candidate source mismatch')
    if plan['geometry_id'] != meta['geometry_id']:
        raise ValueError('candidate geometry mismatch')
    physics, vehicle = meta['simulator_params'], meta['vehicle_params']
    dt, substeps = physics['timestep'], physics['n_sim_steps']
    control_dt = dt*substeps
    count = round(duration/control_dt)
    if duration <= 0 or not np.isclose(count*control_dt, duration, atol=1e-10, rtol=0):
        raise ValueError('duration must be a positive integer number of control intervals')
    raster = load_map('maps/esp.yaml')
    if raster.source_hashes != meta['track']['source_hashes']:
        raise ValueError('source map mismatch')
    path = np.array([[p['x'], p['y']] for p in candidate['points']])
    controller = OpenCandidatePP(path, meta['config']['speed_mps'],
        meta['config']['pp_lookahead_m'], vehicle['wheelbase'], vehicle['max_steer'])
    np.testing.assert_allclose(path[0], state['pose'][:2], atol=1e-4, rtol=0)
    # Reproduce the source's preceding physical interval before branching.
    dynamics = DynamicsSimulator(Namespace(**physics))
    dynamics.state = np.array(state['pre_step_state'], float)
    dynamics.steer_buffer = np.array(state['pre_step_steering_buffer'], float)
    action, _ = read_indexed(source, index, 'actions', step-1)
    for _ in range(substeps):
        dynamics.update_pose(*action['action'])
    np.testing.assert_allclose(dynamics.state, state['physical_state'], atol=1e-12, rtol=0)
    np.testing.assert_array_equal(dynamics.steer_buffer, state['steering_buffer'])
    scan = ScanSimulator2D(physics['num_beams'], physics['fov'], meta['map'], physics['random_seed'])
    collision_context = SimpleNamespace(params=Namespace(**physics), scan_simulator=scan)
    def collision():
        return bool(F1TenthSimBase.check_vehicle_collision(
            collision_context, dynamics.state[[0, 1, 4]]))
    def sample(tick):
        pose = dynamics.state[[0, 1, 4]]
        _, error, fraction, segment = controller.nearest(pose[:2])
        return dict(time_s=tick*dt, physical_state=dynamics.state.tolist(),
            pose=pose.tolist(), steering_buffer=dynamics.steer_buffer.tolist(),
            candidate_error_m=float(error), candidate_index=float(segment+fraction),
            candidate_d_m=float(np.interp(segment+fraction, np.arange(len(path)),
                                         [p['d'] for p in candidate['points']])))
    initial_collision = collision()
    frames, ticks, actions = [sample(0)], [sample(0)], []
    hit, reason = initial_collision, 'initial_collision' if initial_collision else 'duration_reached'
    for k in range(count):
        if hit:
            break
        try:
            command, diagnostic = controller.plan(dynamics.state[[0, 1, 4]])
        except ValueError as exc:
            reason = str(exc)
            break
        actions.append(dict(control_step=k, time_s=k*control_dt,
                            action=command.tolist(), diagnostic=diagnostic))
        for j in range(substeps):
            dynamics.update_pose(*command)
            tick = k*substeps+j+1
            hit = collision()
            ticks.append(sample(tick))
            if hit:
                reason = 'collision'
                break
        frames.append(sample(tick))
        if hit:
            break
    # Replay all new actions from the saved source, independently of the controller.
    replay = DynamicsSimulator(Namespace(**physics))
    replay.state = np.array(state['physical_state'], float)
    replay.steer_buffer = np.array(state['steering_buffer'], float)
    for tick, saved in enumerate(ticks[1:]):
        replay.update_pose(*actions[tick//substeps]['action'])
        np.testing.assert_allclose(replay.state, saved['physical_state'], atol=1e-12, rtol=0)
        np.testing.assert_array_equal(replay.steer_buffer, saved['steering_buffer'])
    summary = dict(source_session=str(source), source_step=step, candidate_id=candidate_id,
        source_sim_time_s=state['sim_time_s'], geometry_id=meta['geometry_id'],
        speed_target_mps=controller.speed, pp_lookahead_m=controller.lookahead,
        requested_duration_s=duration, actual_duration_s=ticks[-1]['time_s'],
        control_intervals=len(actions), physics_ticks=len(ticks)-1,
        collision=hit, collision_check='existing_simulator_four_corners_each_physics_tick',
        end_reason=reason, fixed_candidate=True, replanning=False, open_path=True,
        max_candidate_error_m=max(t['candidate_error_m'] for t in ticks),
        final_candidate_error_m=ticks[-1]['candidate_error_m'],
        candidate_d_at_final_projection_m=ticks[-1]['candidate_d_m'],
        traveled_distance_m=float(sum(np.linalg.norm(np.array(b['pose'][:2])-a['pose'][:2])
                                     for a, b in zip(ticks, ticks[1:]))),
        steering_clipped_count=sum(a['diagnostic']['steering_clipped'] for a in actions),
        source_interval_replay_passed=True, experiment_replay_passed=True,
        swept_footprint_verified=False, source_state_sha256=state_hash,
        source_plan_sha256=plan_hash, code_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    summary.update(candidate_regenerated=regenerate_lookahead is not None,
        candidate_lookahead_m=plan['generator_settings']['lookahead_m'],
        generation_wall_time_s=plan['wall_time_s'],
        generated_valid_candidate_count=plan['sampled_valid_count'],
        original_source_plan_sha256=original_plan_hash)
    output.mkdir(parents=True, exist_ok=False)
    if regenerate_lookahead is not None:
        (output/'regenerated_plan.json').write_text(json.dumps(plan, indent=2, allow_nan=False)+'\n')
    data = dict(summary=summary, frames=frames, physics_samples=ticks, actions=actions,
        candidate_xy=path.tolist(), source_state=state)
    (output/'experiment.json').write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
    (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    draw(output, data, raster, plan)
    print(json.dumps(summary, indent=2))
    return summary


def failed_candidate_diagnostic(plan, geometry, candidate_id):
    """Reproduce a failed first-round fit for display, never accept it as a path.

    Original V5.2 omits coefficients if all starts fail solver acceptance. Match
    the stored first-round diagnostics before reconstructing this display line.
    """
    from .overtaking.robust_adaptive_paths import fit_multistart
    from .overtaking.constrained_adaptive_paths import _basis, _reference_data, evaluate_path_smoothness
    candidate = next(p for p in plan['paths'] if p['candidate_id'] == candidate_id)
    if candidate['sampled_valid'] or candidate['points']:
        raise ValueError('diagnostic reconstruction is only for missing rejected paths')
    settings, result = plan['generator_settings'], plan['result']
    length, sections = settings['lookahead_m'], result['road_sections']
    dense = np.linspace(0., length, len(sections))
    indices = np.unique(np.linspace(0, len(dense)-1, min(161, len(dense))).astype(int))
    distances = dense[indices]
    basis = _basis(length, min(20, max(6, int(np.ceil(length/2))+5)))
    bounds = [sections[i] for i in indices]
    start = result['start']
    fraction0 = (start['d']-sections[0]['d_min'])/(sections[0]['d_max']-sections[0]['d_min'])
    u = distances/length
    blend = 3*u*u-2*u*u*u
    preferred = np.array([v['d_min']+((1-b)*fraction0+b*candidate['target_fraction'])*
                          (v['d_max']-v['d_min']) for v, b in zip(bounds, blend)])
    k, kp = _reference_data(geometry, np.array([sections[i]['s_progress'] for i in indices]))
    fits = fit_multistart(basis, distances, bounds, preferred, start['d'], start['d_prime'],
        candidate['requested_target_d'], k, kp, settings['max_slope'],
        settings['max_curvature_inv_m'], 0., settings['max_iterations'])
    for _, diagnostic in fits:
        saved = next(a for a in candidate['attempts'] if a['refinement'] == 0 and a['seed'] == diagnostic['seed'])
        if saved['status'] != diagnostic['status'] or not np.isclose(
                saved['objective'], diagnostic['objective'], rtol=1e-7, atol=1e-8):
            raise ValueError('reconstructed fit differs from recorded attempt')
    c, diagnostic = max(fits, key=lambda fit: fit[1]['min_inequality'])
    if diagnostic['solver_constraints_accepted']:
        raise ValueError('expected a rejected diagnostic fit')
    ref, tangent, _, _ = geometry.curve.sample([s['s_progress'] for s in sections])
    normal = np.column_stack([-tangent[:, 1], tangent[:, 0]])
    xy = ref+(basis(dense)@c)[:, None]*normal
    d, dp, dpp = [basis(dense, nu=i)@c for i in range(3)]
    kd, kpd = _reference_data(geometry, np.array([s['s_progress'] for s in sections]))
    jacobian, curvature = evaluate_path_smoothness(d, dp, dpp, kd, kpd)
    lo, hi = np.array([s['d_min'] for s in sections]), np.array([s['d_max'] for s in sections])
    return dict(candidate_id=candidate_id, seed=diagnostic['seed'],
                stored_attempt_matched=True, solver_accepted=False,
                validation_performed=False, solver_diagnostic=diagnostic,
                frenet_profile=[dict(distance_m=float(a), d=float(b), d_prime=float(v),
                    curvature_inv_m=float(w), jacobian=float(j),
                    d_min=float(l), d_max=float(h))
                    for a,b,v,w,j,l,h in zip(dense,d,dp,curvature,jacobian,lo,hi)],
                points=[dict(x=float(x), y=float(y)) for x, y in xy])


def draw(output, data, raster, candidate_plan=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon
    from PIL import Image
    path = np.array(data['candidate_xy'])
    frames = data['frames']
    poses = np.array([f['pose'] for f in frames])
    pixels = np.array([raster.map_to_pixel(x).pixel for x in path])
    trace = np.array([raster.map_to_pixel(x[:2]).pixel for x in poses])
    paths = candidate_plan['paths'] if candidate_plan else [dict(
        candidate_id=data['summary']['candidate_id'], sampled_valid=True,
        points=[dict(x=x, y=y) for x, y in path])]
    colors = ['#00a4ba', '#dd8e00', '#19945b', '#ed6a9c', '#965de2']
    layers = []
    for candidate in paths:
        xy = np.array([[p['x'], p['y']] for p in candidate.get('diagnostic_points', candidate['points'])
                       if p.get('x') is not None and p.get('y') is not None])
        if len(xy):
            layers.append((candidate, np.array([raster.map_to_pixel(x).pixel for x in xy])))
    local = np.vstack([p for _, p in layers]+[trace])
    fig, ax = plt.subplots(figsize=(14, 7), dpi=100)
    ax.imshow(raster.gray, cmap='gray', vmin=0, vmax=255)
    for candidate, px in layers:
        cid = candidate['candidate_id']
        label = f"Candidate {cid}" + (': rejected diagnostic' if candidate.get('diagnostic_points')
                                     else ' (rejected)' if not candidate['sampled_valid'] else '')
        ax.plot(px[:, 0], px[:, 1], color=colors[(cid-1)%5], lw=2,
                ls='-' if candidate['sampled_valid'] else '--', label=label)
        ax.text(px[-1, 0]+2, px[-1, 1], str(cid), color=colors[(cid-1)%5], fontsize=10)
    line, = ax.plot([], [], color='#ff3b30', lw=2.5, label='Actual trajectory (follows 1)')
    dot, = ax.plot([], [], 'o', color='#ff3b30', ms=4)
    target, = ax.plot([], [], 'x', color='black', label='PP lookahead target')
    car = Polygon(np.zeros((4, 2)), facecolor='#ff3b30', edgecolor='black', alpha=.8,
                  label='Vehicle body')
    ax.add_patch(car)
    nose, = ax.plot([], [], color='black', lw=1.5)
    ax.plot(trace[0, 0], trace[0, 1], 'o', color='black', ms=5, label='Step 807 source')
    ax.set_xlim(local[:, 0].min()-15, local[:, 0].max()+15)
    ax.set_ylim(local[:, 1].max()+15, local[:, 1].min()-15)
    ax.set_aspect('equal'); ax.legend(loc='upper left', bbox_to_anchor=(1.01, 1), fontsize=9)
    ax.set_axis_off(); fig.subplots_adjust(right=.73)
    images = []
    for i, frame in enumerate(frames):
        line.set_data(trace[:i+1, 0], trace[:i+1, 1])
        dot.set_data([trace[i, 0]], [trace[i, 1]])
        pose = np.array(frame['pose'])
        corners = np.array([[.29, .155], [.29, -.155], [-.29, -.155], [-.29, .155]])
        c, s = np.cos(pose[2]), np.sin(pose[2])
        corners = corners @ np.array([[c, s], [-s, c]]) + pose[:2]
        car.set_xy([raster.map_to_pixel(x).pixel for x in corners])
        front = raster.map_to_pixel(pose[:2]+.29*np.array([c,s])).pixel
        nose.set_data([trace[i,0],front[0]],[trace[i,1],front[1]])
        if i < len(data['actions']):
            xy = data['actions'][i]['diagnostic']['lookahead_xy']
            px = raster.map_to_pixel(xy).pixel
            target.set_data([px[0]], [px[1]])
        else:
            target.set_data([], [])
        ax.set_title(f"Step {data['summary']['source_step']} / candidate {data['summary']['candidate_id']} | t={frame['time_s']:.1f} s | "
                     f"tracking error={frame['candidate_error_m']*100:.2f} cm")
        fig.canvas.draw()
        images.append(Image.fromarray(np.asarray(fig.canvas.buffer_rgba()).copy()).convert('RGB'))
    # Never savefig with a different dpi/bbox inside this loop: it can resize
    # the canvas and leave mismatched GIF frames with stale legend fragments.
    images[0].save(output/'map_step807.png')
    images[-1].save(output/'trajectory.png')
    palette = images[0].quantize(colors=256)
    animation = [frame.quantize(palette=palette, dither=Image.NONE) for frame in images]
    animation[0].save(output/'playback.gif', save_all=True, append_images=animation[1:],
                   duration=100, loop=0, disposal=2, optimize=False)
    plt.close(fig)
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.imshow(raster.gray, cmap='gray', vmin=0, vmax=255)
    for candidate, px in layers:
        cid = candidate['candidate_id']
        ax.plot(px[:, 0], px[:, 1], color=colors[(cid-1)%5], lw=2,
                ls='-' if candidate['sampled_valid'] else '--')
    ax.plot(trace[0, 0], trace[0, 1], 'ro', ms=7)
    ax.annotate('Step 807 / 10 m candidates', trace[0], xytext=(15, 25),
                textcoords='offset points', color='red',
                arrowprops=dict(arrowstyle='->', color='red'))
    ax.set_title('ESP map: regenerated candidates at step 807')
    ax.set_axis_off()
    fig.savefig(output/'map_overview.png', dpi=150, bbox_inches='tight')
    plt.close(fig)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    parser.add_argument('--step', type=int, default=807)
    parser.add_argument('--candidate', type=int, default=1)
    parser.add_argument('--duration', type=float, default=3.)
    parser.add_argument('--regenerate-lookahead', type=float,
                        help='generate five new candidates once at the saved pose, using this horizon in meters')
    args = parser.parse_args()
    if args.step < 1:
        parser.error('step must have a preceding control interval')
    run(args.session, args.output_dir, args.step, args.candidate, args.duration,
        args.regenerate_lookahead)
