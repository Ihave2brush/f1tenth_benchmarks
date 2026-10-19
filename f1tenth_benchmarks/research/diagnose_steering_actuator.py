"""Isolated actuator/PP comparison and existing unit tests; no LMPC driving.

Existing synthetic WLS tests run unchanged; no WLS/QP experiment or history update.
"""
import argparse
import ast
from dataclasses import asdict
import hashlib
import importlib.util
import json
from pathlib import Path
import platform
import unittest

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from f1tenth_benchmarks.research.lmpc.dynamics import TracedDynamics
from f1tenth_benchmarks.research.lmpc.initialization import CentrelinePurePursuit
from f1tenth_benchmarks.research.lmpc.run_initialization import ValidatedContinuousSim
from f1tenth_benchmarks.research.lmpc.lap_adapter import CoreLapTracker
from f1tenth_benchmarks.research.lmpc.validation import FootprintChecker
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_text())
def lines(path):return [json.loads(line) for line in Path(path).read_text().splitlines()]
def clean(value):
    if isinstance(value,np.ndarray):return clean(value.tolist())
    if isinstance(value,np.generic):return clean(value.item())
    if isinstance(value,dict):return {str(k):clean(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [clean(v) for v in value]
    return value
def write(path,value):Path(path).write_text(json.dumps(clean(value),indent=2,ensure_ascii=False,allow_nan=False))


def baseline_class(root):
    path=root/'baseline_source/dynamics_simulator.py'
    spec=importlib.util.spec_from_file_location('steering_before_change',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    before=ast.parse((root/'baseline_source/dynamic_models.py').read_text())
    after=ast.parse(Path('f1tenth_benchmarks/simulator/dynamic_models.py').read_text())
    new={n.name:ast.dump(n,include_attributes=False) for n in after.body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef))}
    identical={n.name:new[n.name]==ast.dump(n,include_attributes=False) for n in before.body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef))}
    assert all(identical.values())
    write(root/'unchanged_dynamics_functions.json',identical)
    return module.DynamicsSimulator


def test_suite(root):
    patterns=['test_steering_actuator.py','test_lmpc_interfaces.py','test_lmpc_input_kernel.py',
              'test_lmpc_no_delay.py','test_continuous_laps.py']
    suite=unittest.TestSuite()
    for pattern in patterns:suite.addTests(unittest.defaultTestLoader.discover('tests/research',pattern=pattern))
    with (root/'test_results.txt').open('w') as stream:
        result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    summary=dict(tests_run=result.testsRun,failures=len(result.failures),errors=len(result.errors),
                 skipped=len(result.skipped),passed=result.wasSuccessful(),patterns=patterns,
                 scope='actuator and default-plant regressions; no LMPC closed-loop runner')
    write(root/'test_results.json',summary)
    if not result.wasSuccessful():raise RuntimeError('actuator/regression tests failed; see test_results.txt')
    print('Unit and existing regressions:',summary,flush=True)
    return summary


def legacy_regression(root,Legacy):
    parameters=vars(load_parameter_file('simulator_params')).copy()
    from argparse import Namespace
    legacy=Legacy(Namespace(**parameters));implicit=DynamicsSimulator(Namespace(**parameters))
    explicit=DynamicsSimulator(Namespace(**parameters,steering_actuator_mode='original'))
    rng=np.random.default_rng(20261019)
    for sim in [legacy,implicit,explicit]:sim.reset([1.,2.,.2])
    for i in range(3000):
        command=[rng.uniform(-1.,1.),rng.uniform(-1.,4.)]
        expected=legacy.update_pose(*command)
        for sim in [implicit,explicit]:
            np.testing.assert_array_equal(expected,sim.update_pose(*command))
            np.testing.assert_array_equal(legacy.steer_buffer,sim.steer_buffer)
    original=Path('Data/research/lmpc/experiments/A5_lmpc_20261015_001')
    session=read(original/'session.json');rows=lines(original/'samples.jsonl')
    plant=DynamicsSimulator(Namespace(**session['simulator_params']))
    plant.reset(session['start_pose']);count=0;maximum=0.
    for row in rows:
        for tick in row['physics_ticks']:
            maximum=max(maximum,float(abs(plant.state-tick['pre_state']).max()))
            actual=plant.update_pose(tick['command'][0],tick['pid_target_speed'])
            maximum=max(maximum,float(abs(actual-tick['post_state']).max()))
            np.testing.assert_allclose(actual,tick['post_state'],rtol=0,atol=1e-12);count+=1
        np.testing.assert_allclose(plant.state,row['physical_state'],rtol=0,atol=1e-12)
        np.testing.assert_array_equal(plant.steer_buffer,row['steering_buffer'])
    result=dict(passed=True,random_ticks=3000,default_and_explicit_original_equal_to_before_change='bitwise full state and FIFO',
        A5_recorded_physics_ticks=count,A5_max_state_error=maximum,tolerance=1e-12,
        note='Historical command/substep playback only; no LMPC optimization, closed-loop experiment or source mutation')
    write(root/'original_regression.json',result);print('Original regression:',result,flush=True)
    return result


def actuator_sequences(root,Legacy):
    from argparse import Namespace
    sequences=dict(micro=np.full(200,-.00010438870829536026),
        large=np.repeat([.35,-.35,0.],60),limits=np.repeat([2.,-2.,.4189,-.4189],60),
        reversals=np.tile(np.repeat([.011,-.009,.25,-.25],4),20))
    output=root/'actuator';output.mkdir()
    summaries={};arrays={}
    for name,sequence in sequences.items():
        mode_results={}
        for mode in ['original','non_overshoot']:
            p=vars(load_parameter_file('simulator_params')).copy();p['steering_actuator_mode']=mode
            sim=TracedDynamics(Namespace(**p));sim.state[3]=2.5
            reference=Legacy(Namespace(**p));reference.state[3]=2.5
            rows=[];crossings=0;bounds=0;fifo_correct=True;max_rate=0.
            for i,command in enumerate(sequence):
                sim.update_pose(command,2.5);tick=sim.trace[-1]
                pre=np.array(tick['pre_state']);post=np.array(tick['post_state'])
                delayed=tick['delayed_target_steering'];effective=tick.get('effective_target_steering',delayed)
                expected=sequence[i-2] if i>=2 else 0.
                fifo_correct &= delayed==expected
                crossings+=int((pre[2]-effective)*(post[2]-effective)<-1e-15)
                bounds+=int(not p['s_min']<=post[2]<=p['s_max'])
                max_rate=max(max_rate,abs(tick['applied_steering_rate']))
                if mode=='original':
                    np.testing.assert_array_equal(post,reference.update_pose(command,2.5))
                    np.testing.assert_array_equal(sim.steer_buffer,reference.steer_buffer)
                rows.append(dict(tick=i,time_s=(i+1)*p['timestep'],mode=mode,raw_command=command,
                    delayed_target=delayed,effective_target=effective,actual_steering=post[2],
                    applied_rate=tick['applied_steering_rate'],yaw_rate=post[5],vy=post[3]*np.sin(post[6]),
                    pre_state=pre,post_state=post,steering_buffer=sim.steer_buffer.copy()))
            write(output/f'{name}_{mode}.json',dict(mode=mode,dt=p['timestep'],target_speed=2.5,samples=rows))
            import csv
            with (output/f'{name}_{mode}.csv').open('w',newline='') as f:
                keys=[key for key in rows[0] if key not in ['pre_state','post_state','steering_buffer']]
                writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerows({key:r[key] for key in keys} for r in rows)
            array=np.array([[r[key] for key in ['time_s','raw_command','effective_target','actual_steering','yaw_rate','vy']] for r in rows]);arrays[name,mode]=array
            tail=array[-100:]
            mode_results[mode]=dict(ticks=len(rows),target_crossings=crossings,wheel_bound_violations=bounds,
                fifo_exact=bool(fifo_correct),max_abs_applied_rate=max_rate,
                tail_steering_peak_to_peak=np.ptp(tail[:,3]),tail_yaw_rmse=np.sqrt(np.mean(tail[:,4]**2)),tail_vy_rmse=np.sqrt(np.mean(tail[:,5]**2)))
            if mode=='non_overshoot':assert crossings==0 and bounds==0 and fifo_correct and max_rate<=3.2+1e-12
        summaries[name]=mode_results
    write(root/'actuator_summary.json',summaries)
    return summaries,arrays


def pp_run(root,mode,laps,geometry,Legacy):
    output=root/'pp'/mode;output.mkdir(parents=True)
    physics=load_parameter_file('simulator_params');vehicle=load_parameter_file('vehicle_params')
    sampled=np.loadtxt('maps/frenet/esp/geometry/centerline.csv',delimiter=',',skiprows=1)
    footprint=FootprintChecker(geometry.raster,physics.vehicle_length,physics.vehicle_width,.05)
    sim=ValidatedContinuousSim('esp',footprint=footprint,extra_params={'n_sim_steps':10,'steering_actuator_mode':mode},
        timeout=240.,centreline_xy=sampled[:,:2])
    sim.tracker=CoreLapTracker(geometry,sampled[:,:2])
    planner=CentrelinePurePursuit(sampled[:,:2],2.5,.6,vehicle.wheelbase,vehicle.max_steer)
    reference=geometry.reference(0.);pose=np.r_[reference['xy'],reference['psi']]
    old=Legacy(sim.params);old.reset(pose)
    from argparse import Namespace
    replay=DynamicsSimulator(Namespace(**vars(sim.params)));replay.reset(pose)
    rows=[];events=[];ticks_total=0;max_replay=0.;max_legacy=0.;overshoots=0;max_rate=0.;bounds=0
    metadata=dict(mode=mode,steering_actuator_mode=mode,map='esp',requested_laps=laps,
        control_dt=.1,physics_dt=.01,FIFO_ticks=2,PP_reference_speed=2.5,PP_lookahead=.6,wheelbase=vehicle.wheelbase,max_command_steer=vehicle.max_steer,
        command_interface='existing target_speed PID, no acceleration adapter, no excitation',start_pose=pose,initial_speed=0.,
        geometry_id=geometry.geometry_id,simulator_params=vars(sim.params),tracker_config=asdict(sim.tracker.config),
        source='fresh independent PP session; never appended to an LMPC database',
        code_hashes={str(p):digest(p) for p in [Path('f1tenth_benchmarks/simulator/dynamics_simulator.py'),Path('f1tenth_benchmarks/simulator/dynamic_models.py'),Path('f1tenth_benchmarks/research/lmpc/dynamics.py'),Path('f1tenth_benchmarks/research/lmpc/initialization.py')]})
    write(output/'metadata.json',metadata)
    with (output/'samples.jsonl').open('x') as stream:
        def record(action,initializing=False):
            nonlocal ticks_total,max_replay,max_legacy,overshoots,max_rate,bounds
            for tick in sim.dynamics_simulator.trace:
                pre=np.array(tick['pre_state']);post=np.array(tick['post_state'])
                replayed=replay.update_pose(tick['command'][0],tick['pid_target_speed'])
                max_replay=max(max_replay,float(abs(replayed-post).max()))
                np.testing.assert_array_equal(replayed,post)
                if mode=='original':
                    legacy_post=old.update_pose(tick['command'][0],tick['pid_target_speed'])
                    max_legacy=max(max_legacy,float(abs(legacy_post-post).max()));np.testing.assert_array_equal(legacy_post,post)
                effective=tick.get('effective_target_steering',tick['delayed_target_steering'])
                overshoots+=int((pre[2]-effective)*(post[2]-effective)<-1e-15)
                max_rate=max(max_rate,abs(tick['applied_steering_rate']))
                bounds+=int(not physics.s_min<=post[2]<=physics.s_max);ticks_total+=1
            state=sim.current_state.copy();fr=sim.tracker.frenet
            curvature=geometry.reference(fr['s_wrapped'])['kappa'] if fr and fr['valid'] else None
            row=dict(mode=mode,sample_id=len(rows),time_s=sim.elapsed_time,simulation_time=sim.current_time,
                initializing=initializing,command=np.asarray(action).tolist(),physical_state=state.tolist(),
                actual_steering=float(state[2]),yaw_rate=float(state[5]),vy=float(state[3]*np.sin(state[6])),vx=float(state[3]*np.cos(state[6])),
                s_unwrapped=fr['s_unwrapped'] if fr and fr['valid'] else None,ey=fr['d'] if fr and fr['valid'] else None,
                curvature=curvature,steering_buffer=sim.dynamics_simulator.steer_buffer.tolist(),
                physics_ticks=sim.dynamics_simulator.trace,completed_full_laps=sim.tracker.completed_full_laps,
                terminal_reason=sim.terminal_reason)
            stream.write(json.dumps(clean(row),allow_nan=False)+'\n');stream.flush();rows.append(row)
            if obs['lap_event'] is not None:events.append(asdict(obs['lap_event']))
        obs,done,_=sim.reset(pose);record([0.,0.],True)
        while not done and sim.tracker.completed_full_laps<laps:
            action=planner.plan(obs);obs,done=sim.step(action);record(action)
            if len(rows)%500==0:print('PP',mode,'steps',len(rows)-1,'laps',sim.tracker.completed_full_laps,flush=True)
    # Both modes get an independent complete mode-tagged history. No cross-mode source import.
    write(output/'events.json',events)
    array=np.array([[r['time_s'],r['actual_steering'],r['yaw_rate'],r['vy'],r['vx'],r['ey'],r['s_unwrapped']] for r in rows])
    np.savez_compressed(output/'trajectory.npz',data=array,columns=np.array(['time','steering','yaw_rate','vy','vx','ey','s_unwrapped']),
        physical_states=np.array([r['physical_state'] for r in rows]),commands=np.array([r['command'] for r in rows]),mode=np.array(mode))
    import csv
    with (output/'trajectory.csv').open('w',newline='') as f:
        writer=csv.writer(f);writer.writerow(['time_s','actual_steering_rad','yaw_rate_rad_s','vy_mps','vx_mps','ey_m','s_unwrapped_m']);writer.writerows(array)
    straight=np.array([r['curvature'] is not None and abs(r['curvature'])<=.005 and r['vx']>=2. for r in rows])
    metrics=dict(mode=mode,completed_laps=sim.tracker.completed_full_laps,requested_laps=laps,
        success=sim.tracker.completed_full_laps==laps and not done,end_reason=sim.terminal_reason or 'requested_laps_reached',
        elapsed_s=sim.elapsed_time,control_steps=sim.control_steps,physics_ticks=ticks_total,
        reset_count=sim.reset_count,lap_times=[e['duration'] for e in events if e['kind']=='full'],
        max_abs_ey=float(abs(array[:,5]).max()),max_abs_yaw=float(abs(array[:,2]).max()),max_abs_vy=float(abs(array[:,3]).max()),
        target_crossings=overshoots,wheel_bound_violations=bounds,max_abs_applied_rate=max_rate,
        footprint_checks=sim.footprint_checks,footprint_failure=sim.footprint_failure,
        straight_filter='abs(kappa)<=.005 m^-1 and vx>=2m/s; not a fixed-state causal comparison',
        straight_samples=int(straight.sum()),straight_yaw_rmse=float(np.sqrt(np.mean(array[straight,2]**2))) if straight.any() else None,
        straight_vy_rmse=float(np.sqrt(np.mean(array[straight,3]**2))) if straight.any() else None,
        independent_base_replay_max_error=max_replay,
        original_prechange_reference_max_error=max_legacy if mode=='original' else None)
    write(output/'session.json',dict(metadata=metadata,**metrics))
    if mode=='non_overshoot':assert overshoots==0 and bounds==0 and max_rate<=3.2+1e-12
    print('PP finished:',metrics,flush=True)
    return metrics,array


def figures(root,actuator,pp):
    output=root/'figures';output.mkdir(exist_ok=True)
    plt.rcParams.update({'font.size':10,'axes.grid':True,'grid.alpha':.25})
    for scenario in ['micro','large','limits','reversals']:
        fig,axs=plt.subplots(3,1,figsize=(13,9),constrained_layout=True)
        for mode,color in [('original','#d55e00'),('non_overshoot','#0072b2')]:
            arr=actuator[scenario,mode]
            for ax,col,title in zip(axs,[3,4,5],['Actual steering (rad)','Yaw rate (rad/s)','Lateral velocity (m/s)']):
                ax.plot(arr[:,0],arr[:,col],label=mode,color=color);ax.set(xlabel='Time (s)',ylabel=title);ax.legend()
        axs[0].plot(arr[:,0],arr[:,1],'--',color='.5',label='Raw command');axs[0].legend()
        fig.suptitle(f'Identical command sequence: {scenario}, dt=.01 s, speed=2.5m/s')
        fig.savefig(output/f'actuator_{scenario}.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(4,1,figsize=(14,12),constrained_layout=True)
    for mode,color in [('original','#d55e00'),('non_overshoot','#0072b2')]:
        arr=pp[mode]
        for ax,col,title in zip(axs,[1,2,3,5],['Actual steering (rad)','Yaw rate (rad/s)','Lateral velocity (m/s)','Lateral tracking error (m)']):
            ax.plot(arr[:,0],arr[:,col],label=mode,color=color,lw=.8);ax.set(xlabel='Time since initialization (s)',ylabel=title);ax.legend()
    fig.suptitle('Independent PP closed loops: same settings; commands differ after states diverge')
    fig.savefig(output/'pp_full_comparison.png',dpi=170);plt.close(fig)
    fig,axs=plt.subplots(3,1,figsize=(13,9),constrained_layout=True)
    for mode,color in [('original','#d55e00'),('non_overshoot','#0072b2')]:
        rows=lines(root/'pp'/mode/'samples.jsonl');ticks=[]
        for row in rows[1:51]:
            for i,tick in enumerate(row['physics_ticks']):
                post=tick['post_state'];ticks.append([row['time_s']-.1+(i+1)*.01,post[2],post[5],post[3]*np.sin(post[6])])
        arr=np.array(ticks)
        for ax,col,label in zip(axs,[1,2,3],['Steering (rad)','Yaw rate (rad/s)','vy (m/s)']):ax.plot(arr[:,0],arr[:,col],label=mode,color=color);ax.set(xlabel='Time (s)',ylabel=label);ax.legend()
    fig.suptitle('PP first 5s: recorded 100Hz physics substeps')
    fig.savefig(output/'pp_first5s_100hz.png',dpi=170);plt.close(fig)


def run(root,laps):
    root=Path(root);assert (root/'baseline_source').is_dir(),'create pre-change baseline before editing production sources'
    if (root/'summary.json').exists():raise FileExistsError('use a new diagnostic output directory')
    protected=read(root/'protected_before.json')
    assert all(digest(path)==value for path,value in protected.items()),'protected source changed before analysis'
    Legacy=baseline_class(root);tests=test_suite(root);regression=legacy_regression(root,Legacy)
    actuator,arrays=actuator_sequences(root,Legacy)
    geometry=load_geometry('maps/frenet/esp/geometry','maps/esp.yaml',inspect=True)
    pp={};trajectories={}
    for mode in ['original','non_overshoot']:pp[mode],trajectories[mode]=pp_run(root,mode,laps,geometry,Legacy)
    figures(root,arrays,trajectories)
    changed=[path for path,value in protected.items() if digest(path)!=value]
    assert not changed
    summary=dict(date='2026-10-19 Asia/Taipei',tests=tests,original_regression=regression,actuator=actuator,PP=pp,
        protected_files=len(protected),protected_files_changed=changed,
        modes_separated=True,no_LMPC_run=True,no_WLS_or_QP_change=True,
        next_WLS_diagnostic_ready=tests['passed'] and regression['passed'] and all(r['success'] for r in pp.values()),
        WLS_scope='Controlled comparison only; not validation of a learned model or acceptance of these unexcited two-lap PP histories as LMPC initialization',
        versions=dict(python=platform.python_version(),numpy=np.__version__))
    write(root/'summary.json',summary)
    print('Complete:',root,flush=True)
    return summary


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir',required=True);parser.add_argument('--pp-laps',type=int,default=2)
    args=parser.parse_args()
    if args.pp_laps not in [1,2]:parser.error('this isolated diagnostic supports one or two PP laps')
    run(args.output_dir,args.pp_laps)


if __name__=='__main__':main()
