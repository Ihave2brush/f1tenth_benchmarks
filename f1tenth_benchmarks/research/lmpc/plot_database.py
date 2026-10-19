"""Static B2 cost coordinates and projected local terminal columns."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

from .database import LapDatabase
from .safe_set import select_local_safe_set


def plot_database(directory):
    root = Path(directory)
    output = root/'database_overview.png'
    if output.exists():
        raise FileExistsError(output)
    db = LapDatabase.load(root)
    fig, axes = plt.subplots(1, 2, figsize=(12, 4.6), constrained_layout=True)
    colors = ['#2878b5', '#dc7136']
    for lap, color in zip(db.laps, colors):
        axes[0].plot(lap.states[:, 4], lap.costs, color=color, label='Lap %d' % lap.lap_id)
        if lap.periodic_tail is not None:
            tail = lap.periodic_tail
            axes[0].plot(np.r_[lap.states[-1, 4], tail.states[:, 4]], np.r_[lap.costs[-1], tail.costs],
                         color=color, linestyle='--', linewidth=2, label='Lap %d periodic template' % lap.lap_id)
    axes[0].axvline(db.length, color='black', linestyle='--', label='Finish s=L')
    boundary = (db.laps[-1].states[-1, 4]+db.laps[-1].states[-21, 4])/2
    if db.laps[-1].periodic_tail is None:
        axes[0].axvspan(boundary, db.length, color='#b82c32', alpha=.2)
    else:
        axes[0].axvspan(db.length, db.length+db.metadata['max_candidate_overhang_m'],
                       color='#389966', alpha=.15)
    axes[0].axhline(0, color='gray', linewidth=.6)
    axes[0].set(xlabel='Source-lap progress s [m]', ylabel='Signed cost J=T-t [steps]',
                title='Cost coordinates; dashed = periodic template')
    axes[0].legend()
    candidate = db.laps[-1].states[12].copy()
    candidate[4] = db.length if db.laps[-1].periodic_tail is not None else db.length*.25
    local = select_local_safe_set(db, candidate)
    lam = np.zeros(40); lam[9], lam[30] = .35, .65
    terminal = local.combination(lam)
    result = local.q_function(terminal)
    for lap_id, color in zip([lap.lap_id for lap in db.laps[-2:]], colors):
        ids = [i for i, p in enumerate(local.provenance) if p['lap_id']==lap_id]
        axes[1].scatter(local.states[4, ids], local.states[5, ids]*100,
                        color=color, label='Lap %d: 20 columns' % lap_id)
    axes[1].scatter([terminal[4]], [terminal[5]*100], marker='*', s=180, color='black',
                    label='Six-state feasible convex combination')
    periodic_ids = [i for i, p in enumerate(local.provenance) if p['transition_kind']=='historical_periodic_replay']
    seam_ids = [i for i, p in enumerate(local.provenance) if p['transition_kind']=='approximate_periodic_seam']
    if periodic_ids:
        axes[1].scatter(local.states[4, periodic_ids], local.states[5, periodic_ids]*100,
                        marker='s', s=95, facecolors='none', edgecolors=colors[-1], label='Reused prefix')
    if seam_ids:
        axes[1].scatter(local.states[4, seam_ids], local.states[5, seam_ids]*100,
                        marker='x', s=100, color='#b82c32', label='Approximate seam edge')
    axes[1].set(xlabel='Source-lap progress s [m]', ylabel='Lateral error e_y [cm]',
                title='Local set projection (not a safety certificate)')
    axes[1].text(.02, .03, 'Q = %.6f steps; residual = %.2g' %
                 (result['value'], result['equality_residual']), transform=axes[1].transAxes)
    axes[1].legend(fontsize=7, loc='best')
    for ax in axes:
        ax.grid(alpha=.2)
    fig.savefig(output, dpi=180)
    plt.close(fig)
    return output


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory')
    print(plot_database(parser.parse_args().directory))
