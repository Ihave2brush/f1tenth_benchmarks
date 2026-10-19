"""Plot saved six-state LMPC runs, including stopped experiments; never re-simulate.

Forecast curves are recorded QP state nodes, not free ATV rollouts or measured
future states. Commands are aligned with the start of each applied interval.
"""
import argparse
import json
from pathlib import Path
import numpy as np


def plot_run(directory):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import yaml
    root=Path(directory);report=json.loads((root/'session.json').read_text())
    rows=[json.loads(x) for x in (root/'samples.jsonl').read_text().splitlines()]
    decisions=[json.loads(x) for x in (root/'controls.jsonl').read_text().splitlines()]
    applied=[c for c in decisions if 'actual_one_step_prediction_error' in c]
    h=report['handoff']['raw_sample_id'];local=rows[h:h+len(applied)+1]
    if not applied or len(local)!=len(applied)+1:raise ValueError('missing applied control evidence')
    origin=report['handoff']['time'];time=np.array([r['time']-origin for r in local])
    x=np.array([r['paper_state'] for r in local]);x[:,4]-=report['handoff']['s_origin']
    physical=np.array([r['physical_state'] for r in local]);u=np.array([r['command'] for r in local[1:]])
    predictions=np.array([c['states'][1] for c in applied])
    assert predictions.shape==(len(applied),6)
    ticks=[tick for row in local[1:] for tick in row['physics_ticks']]
    tick_time=np.r_[0.,np.cumsum([tick['dt'] for tick in ticks])]
    tick_angle=np.r_[physical[0,2],[tick['post_state'][2] for tick in ticks]]
    actual_accel=np.array([sum(t['applied_acceleration']*t['dt'] for t in r['physics_ticks'])/.1 for r in local[1:]])
    base=yaml.safe_load(Path(report['config']['base_config']).read_text())
    centre=np.loadtxt(Path(base['adaptation']['geometry_path'])/'centerline.csv',delimiter=',',skiprows=1)
    experiment=report['experiment_id'];out=root/'figures';out.mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.titlesize':11,'axes.spines.top':False,'axes.spines.right':False,
                         'legend.frameon':False,'savefig.facecolor':'white'})
    blue='#1763a6';orange='#d97706';red='#c62945';gray='#7c8797'
    stop=time[-1];saved=[]
    def finish(fig,name):
        for ext in ('png','svg'):
            p=out/(name+'.'+ext);fig.savefig(p,dpi=180,bbox_inches='tight');saved.append(str(p))
        plt.close(fig)
    def axis_style(ax):
        ax.grid(alpha=.18);ax.axvline(stop,color=red,lw=1,ls=':',label='planning stopped')
        ax.set_xlim(0,stop+.07);ax.set_xlabel('Time since LMPC handoff [s]')
    def stairs(ax,values,label,color):
        ax.step(time,np.r_[values,values[-1]],where='post',label=label,color=color,lw=1.6)
    fig,axs=plt.subplots(3,2,figsize=(13,10),constrained_layout=True)
    ax=axs[0,0];ax.plot(time,x[:,4]-x[0,4],color=blue,lw=2,label='actual distance')
    ax.set(title='Progress from handoff',ylabel='Distance [m]');ax.legend(fontsize=8)
    ax=axs[0,1];ax.plot(time,x[:,5]*100,color=blue,marker='.',label='actual lateral error')
    ax.set(title='Lateral tracking',ylabel='e_y [cm]');ax.axhline(0,color=gray,lw=.8)
    ax.text(.02,.95,'Experiment limit: +/-25 cm',transform=ax.transAxes,va='top',color=gray,fontsize=8)
    ax=axs[1,0];ax.plot(time,x[:,0],color=blue,lw=2,label='actual vx')
    ax.plot(time[1:],predictions[:,0],color=orange,ls='--',marker='.',label='one-step QP forecast')
    ax.set(title='Longitudinal speed',ylabel='vx [m/s]');ax.legend(fontsize=8)
    ax=axs[1,1];stairs(ax,u[:,0],'applied command',orange)
    ax.plot(tick_time,tick_angle,color=blue,lw=1.3,label='actual angle (10 ms ticks)')
    ax.set(title='Steering: command vs. actual',ylabel='Steering [rad]');ax.legend(fontsize=8)
    ax=axs[2,0];ax.plot(time,x[:,2],color=blue,lw=1.8,label='actual yaw rate')
    ax.plot(time[1:],predictions[:,2],color=orange,ls='--',marker='.',label='one-step QP forecast')
    ax.set(title='Yaw-rate prediction',ylabel='r [rad/s]');ax.legend(fontsize=8)
    ax=axs[2,1];stairs(ax,u[:,1],'applied command',orange)
    ax.plot(time[:-1]+.05,actual_accel,color=blue,marker='.',label='interval-mean applied acceleration')
    ax.set(title='Acceleration',ylabel='Acceleration [m/s^2]');ax.legend(fontsize=8)
    for ax in axs.flat:axis_style(ax)
    fig.suptitle(f'{experiment} | {len(applied)} applied actions, {stop:.1f} s | planning stopped at s={x[-1,4]:.2f} m\n'
                 'Stopped before action 18: QP reported primal infeasible' if experiment=='A9' else f'{experiment} | recorded closed-loop signals',fontsize=14)
    finish(fig,'A9_control_overview' if experiment=='A9' else 'control_overview')

    fig,axs=plt.subplots(2,3,figsize=(15,8),constrained_layout=True)
    names=['vx [m/s]','vy [m/s]','r [rad/s]','e_psi [rad]','s [m]','e_y [m]']
    colors=plt.cm.viridis(np.linspace(.15,.85,len(applied)))
    for channel,ax in enumerate(axs.flat):
        for i,c in enumerate(applied):
            forecast=np.array(c['states']);ft=(c['time']-origin)+np.arange(len(forecast))*.1
            ax.plot(ft,forecast[:,channel],color=colors[i],alpha=.35,lw=.9)
        final=applied[-1];z=np.array(final['states']);ft=(final['time']-origin)+np.arange(13)*.1
        ax.plot(ft,z[:,channel],color=orange,lw=2,ls='--',label='last solved QP horizon (#17)' if experiment=='A9' else 'last solved QP horizon')
        ax.plot(time,x[:,channel],color='#162634',lw=2.1,marker='.',label='actual (observed only)')
        ax.axvline(stop,color=red,lw=1,ls=':');ax.axvspan(stop,max(ft[-1],stop),color=gray,alpha=.06)
        ax.grid(alpha=.18);ax.set(xlabel='Time since handoff [s]',ylabel=names[channel],title=names[channel].split(' [')[0])
    axs[0,0].legend(fontsize=8)
    fig.suptitle(f'{experiment} | 12-step QP forecasts vs. actual observations\n'
                 'Thin curves: successive solved horizons. Shaded region: no future observations; failed QP has no solution.',fontsize=13)
    finish(fig,'A9_prediction_horizons' if experiment=='A9' else 'prediction_horizons')

    fig,axs=plt.subplots(1,2,figsize=(13,7),constrained_layout=True)
    map_yaml=Path(base['adaptation']['map_yaml']);mp=yaml.safe_load(map_yaml.read_text());img=plt.imread(map_yaml.parent/mp['image'])
    ox,oy,yaw=mp['origin'];assert yaw==0.
    extent=[ox,ox+img.shape[1]*mp['resolution'],oy,oy+img.shape[0]*mp['resolution']]
    ax=axs[0];ax.imshow(img,cmap='gray',origin='upper',extent=extent,vmin=0,vmax=1,alpha=.8)
    pp=np.array([r['physical_state'][:2] for r in rows[:h+1]])
    ax.plot(pp[:,0],pp[:,1],color='#8593a6',lw=.7,label='three PP laps (replayed)')
    ax.plot(physical[:,0],physical[:,1],color=blue,lw=2,label='LMPC actual')
    ax.scatter(*physical[0,:2],s=45,color='#16836a',zorder=5,label='handoff')
    ax.scatter(*physical[-1,:2],s=65,marker='X',color=red,zorder=5,label='planning stop')
    ax.set_aspect('equal');ax.set(xlabel='Map X [m]',ylabel='Map Y [m]',title='ESP map: full initialization and short LMPC segment')
    ax.legend(fontsize=8,loc='upper left',frameon=True,facecolor='white',framealpha=.95)
    ax=axs[1];ax.plot(centre[:,0],centre[:,1],color=gray,ls='--',lw=1,label='centreline')
    ax.plot(physical[:,0],physical[:,1],color=blue,lw=2,marker='.',label='LMPC actual')
    ax.scatter(*physical[0,:2],s=45,color='#16836a',zorder=5)
    ax.scatter(*physical[-1,:2],s=65,marker='X',color=red,zorder=5)
    ax.annotate(f'Stop: s={x[-1,4]:.2f} m\ne_y={x[-1,5]*100:.2f} cm',physical[-1,:2],xytext=(-115,45),textcoords='offset points',
                arrowprops={'arrowstyle':'->','color':red},fontsize=10,color=red)
    ax.set(xlim=(physical[:,0].min()-.3,physical[:,0].max()+.5),ylim=(physical[:,1].min()-.10,physical[:,1].max()+.15),
           xlabel='Map X [m]',ylabel='Map Y [m]',title='LMPC close-up (X and Y scales differ)')
    ax.grid(alpha=.18);ax.legend(fontsize=8,loc='lower left')
    fig.suptitle(f'{experiment} | Actual trajectory and stop location\nThe red marker is a planning failure, not a recorded collision.',fontsize=13)
    finish(fig,'A9_trajectory' if experiment=='A9' else 'trajectory')
    manifest=dict(source_session=str(root/'session.json'),experiment=experiment,applied_actions=len(applied),
        observed_duration_s=float(stop),last_progress_m=float(x[-1,4]),
        forecast_semantics='recorded QP state nodes; not free ATV rollout; no failed-QP solution drawn',files=saved)
    (out/'manifest.json').write_text(json.dumps(manifest,indent=2))
    print(json.dumps(manifest,indent=2))
    return saved

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('directory');args=p.parse_args();plot_run(args.directory)
