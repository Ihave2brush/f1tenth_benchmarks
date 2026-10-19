"""Compact B1 scientific figure from saved physical samples and circle events."""
import argparse
import json
from pathlib import Path

import numpy as np


def plot_initialization(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    root = Path(directory)
    report = json.loads((root/'session.json').read_text())
    if not report['passed']:
        raise ValueError('plotting requires a passed B1 session')
    rows = [json.loads(line) for line in (root/'samples.jsonl').read_text().splitlines()]
    events = [json.loads(line) for line in (root/'events.jsonl').read_text().splitlines()]
    path = Path(report['config']['adaptation']['geometry_path'])/'centerline.csv'
    centre = np.loadtxt(path, delimiter=',', skiprows=1)
    physical = np.array([r['physical_state'] for r in rows])
    states = np.array([r['paper_state'] for r in rows])
    time = np.array([r['elapsed_time'] for r in rows])
    settings = report['config']['initialization']
    figure, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    axes[0, 0].plot(centre[:, 0], centre[:, 1], color='black', linewidth=1.7, label='core centreline')
    start = 0
    colors = ['#1565c0', '#e67700']
    for i, event in enumerate(events):
        stop = event['row_index']+1
        progress = states[start:stop, 4]-i*report['frenet_length']
        label = f"lap {i+1}: {event['duration']:.3f} s"
        axes[0, 0].plot(physical[start:stop, 0], physical[start:stop, 1], color=colors[i], linewidth=.8, label=label)
        axes[0, 1].plot(progress, states[start:stop, 5], color=colors[i], linewidth=.9, label=f'lap {i+1}')
        axes[1, 1].plot(progress, physical[start:stop, 2], color=colors[i], linewidth=.8, label=f'lap {i+1}')
        # Boundary physical sample is shared for visualization, not a synthetic crossing.
        start = stop-1
    axes[0, 0].set(xlabel='map X [m]', ylabel='map Y [m]', title='ESP: Cartesian reference tracking')
    axes[0, 0].set_aspect('equal')
    axes[0, 0].legend(fontsize=8)
    for value in (-settings['max_abs_e_y_m'], settings['max_abs_e_y_m']):
        axes[0, 1].axhline(value, linestyle='--', color='gray', linewidth=.8)
    axes[0, 1].set(xlabel='core progress in lap [m]', ylabel='lateral offset [m]',
                   title=f"Maximum |e_y|: {report['max_abs_e_y']*100:.2f} cm")
    axes[0, 1].legend(fontsize=8)
    axes[1, 0].plot(time, physical[:, 3], color=colors[0], label='actual speed')
    axes[1, 0].axhline(1.2, color='black', linestyle='--', linewidth=.8, label='1.2 m/s target')
    axes[1, 0].axvline(settings['settled_after_s'], color='gray', linestyle=':', label='settled gate')
    axes[1, 0].set(xlabel='elapsed time [s]', ylabel='speed [m/s]', title='Standing start and constant speed')
    axes[1, 0].legend(fontsize=8)
    limit = report['vehicle_params']['max_steer']
    axes[1, 1].axhline(limit, color='gray', linestyle='--', linewidth=.8)
    axes[1, 1].axhline(-limit, color='gray', linestyle='--', linewidth=.8)
    axes[1, 1].set(xlabel='core progress in lap [m]', ylabel='actual steering [rad]', title='Actuator behavior by lap')
    axes[1, 1].legend(fontsize=8)
    for axis in axes.flat:
        axis.grid(alpha=.2)
    figure.suptitle('B1 accepted: two PP laps at 10 Hz; no LMPC learning yet', fontsize=13)
    destination = root/'initialization_overview.png'
    if destination.exists():
        raise FileExistsError(destination)
    figure.savefig(destination, dpi=160)
    plt.close(figure)
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    args = parser.parse_args()
    print(plot_initialization(args.directory))


if __name__ == '__main__':
    main()
