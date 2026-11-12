"""Headless two-car centreline PP demo with replay JSON and a trajectory plot."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from time import perf_counter

import numpy as np

from .collision.multi_car import MultiCarSim
from .core.centreline_control import CentrelinePurePursuit
from .overtaking.track_data import TrackData
from .run_centerline_candidates import source_version
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file


def run(output, map_name='my_map', seconds=12., gap=1.5, speeds=(1.2, 1.2),
        wall_clearance=.05, pair_clearance=.05):
    if not np.isfinite([seconds, gap, *speeds]).all() or seconds <= 0 or gap <= 0 or min(speeds) < 0:
        raise ValueError('invalid duration, gap or speeds')
    out = Path(output)
    if out.exists():
        raise FileExistsError('choose a new output directory')
    track = TrackData(f'maps/frenet/{map_name}/geometry', f'maps/{map_name}.yaml')
    if gap >= track.geometry.curve.L:
        raise ValueError('gap must be smaller than track length')
    vehicle = load_parameter_file('vehicle_params')
    if max(speeds) > vehicle.max_speed:
        raise ValueError('speed exceeds research vehicle limit')
    sim = MultiCarSim(map_name, wall_clearance=wall_clearance, pair_clearance=pair_clearance)
    poses = []
    for s in (0., gap):
        ref = track.geometry.reference(s)
        poses.append([*ref['xy'], ref['psi']])
    planners = [None if speed == 0 else CentrelinePurePursuit(
        track.control_path, speed, .3 if map_name == 'my_map' else .6,
        vehicle.wheelbase, vehicle.max_steer) for speed in speeds]
    obs, done = sim.reset(poses)
    frames, timings = [], []

    def record(actions, cycle_ms=None):
        frames.append(dict(time_s=sim.time, states=[car.state.tolist() for car in sim.cars],
                           actions=actions, cycle_ms=cycle_ms, events=list(sim.events)))
    record(None)
    for _ in range(int(np.ceil(seconds/.1))):
        if done:
            break
        start = perf_counter()
        actions = [np.zeros(2) if p is None else p.plan(o) for p, o in zip(planners, obs)]
        obs, done = sim.step(actions)
        timings.append((perf_counter()-start)*1000)
        record(np.asarray(actions).tolist(), timings[-1])
    report = dict(map=map_name, duration_s=sim.time, steps=len(timings),
                  terminal_reason='envelope_event' if done else 'duration_reached', events=sim.events,
                  body=asdict(sim.bodies[0]), disc_radius_m=sim.bodies[0].radius,
                  disc_offsets_m=[-sim.bodies[0].length/3, 0., sim.bodies[0].length/3], wall_clearance_m=wall_clearance,
                  pair_clearance_m=pair_clearance, physics_dt_s=sim.params.timestep,
                  control_dt_s=.1, max_sweep_motion_m=sim.max_sweep_motion,
                  spawn_gap_along_track_m=gap, target_speeds_mps=list(speeds),
                  cycle_ms=dict(median=float(np.median(timings)), p95=float(np.percentile(timings, 95)),
                                maximum=max(timings)) if timings else None,
                  collision_model='three_disc_conservative_interpolated_sweep',
                  reference_point='geometric_body_centre_assumption',
                  lidar='static_map_plus_uninflated_opponent_discs_no_noise',
                  candidate_control_enabled=False, source=source_version())
    out.mkdir(parents=True)
    (out/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    (out/'frames.json').write_text(json.dumps(frames)+'\n')
    plot(sim, frames, track.control_path, out/'preview.png')
    print(json.dumps({k: v for k, v in report.items() if k != 'source'}, indent=2))
    return report


def plot(sim, frames, path, output, candidate_frame=None):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon
    from matplotlib.transforms import Affine2D
    fig, axes = plt.subplots(1, 2, figsize=(12, 6))
    scanner = sim.scanners[0]
    if scanner is None:
        image, resolution, origin = (~sim.map.occupied * 255, sim.map.resolution, sim.map.origin)
    else:
        image, resolution, origin = scanner.map_img, scanner.map_resolution, scanner.origin
    h, w = image.shape
    final = frames[-1] if candidate_frame is None else candidate_frame
    for ax, frame, label in zip(axes, [frames[0], final],
                                ['Initial', 'Final' if candidate_frame is None else 'Last plan']):
        transform = Affine2D().rotate(origin[2]).translate(*origin[:2])+ax.transData
        ax.imshow(image, origin='lower', cmap='gray', vmin=0, vmax=255,
                  extent=[0, w*resolution, 0, h*resolution], transform=transform)
        ax.plot(path[:, 0], path[:, 1], '--', color='.6', lw=.7)
        if ax is axes[1] and candidate_frame is not None:
            for candidate in frame['candidates']:
                selected = candidate['id'] == frame['selected_candidate_id']
                ax.plot(*candidate['xy'].T, ls='-' if candidate['valid'] else '--',
                        lw=2.2 if selected else 1.1, zorder=4 if selected else 3,
                        label=f"C{candidate['id']}" + (' selected' if selected else ''))
            if frame['candidates']:
                ax.legend(loc='best', fontsize='small')
        for i, (body, state, color) in enumerate(zip(sim.bodies, frame['states'], ['#087f8c', '#d45b20'])):
            pose = np.array(state)[[0, 1, 4]]
            if not np.isfinite(pose).all():
                continue
            history = np.array([f['states'][i][:2] for f in frames])
            if ax is axes[1]:
                ax.plot(*history.T, color=color, lw=1)
            c, s = np.cos(pose[2]), np.sin(pose[2])
            corners = np.array([[-1,-1],[1,-1],[1,1],[-1,1]])*[body.length/2, body.width/2]
            corners = corners @ np.array([[c, s],[-s, c]]) + pose[:2]
            ax.add_patch(Polygon(corners, color=color, alpha=.25))
            for centre in body.centres(pose):
                ax.add_patch(Circle(centre, body.radius, fill=False, color=color))
                ax.add_patch(Circle(centre, body.radius+sim.wall_clearance, fill=False, color=color, ls=':', alpha=.6))
            ax.text(*pose[:2], str(i), color='black', weight='bold')
        ax.set(xlim=(path[:,0].min()-1, path[:,0].max()+1), ylim=(path[:,1].min()-1, path[:,1].max()+1),
               title=f'{label} / {frame["time_s"]:.2f}s', xlabel='x [m]', ylabel='y [m]', aspect='equal')
    fig.suptitle('Three-disc envelopes: car 0 / car 1; dotted = wall clearance')
    fig.tight_layout()
    fig.savefig(output, dpi=150)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--map', choices=['my_map', 'esp'], default='my_map')
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--gap', type=float, default=1.5)
    parser.add_argument('--ego-speed', type=float, default=1.2)
    parser.add_argument('--lead-speed', type=float, default=1.2)
    parser.add_argument('--wall-clearance', type=float, default=.05)
    parser.add_argument('--pair-clearance', type=float, default=.05)
    args = parser.parse_args()
    run(args.output, args.map, args.seconds, args.gap, (args.ego_speed, args.lead_speed),
        args.wall_clearance, args.pair_clearance)


if __name__ == '__main__':
    main()
