"""Mode-separated identification-only sampling and QC; never fit WLS or run LMPC."""
import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import unittest

import numpy as np
from scipy.spatial.distance import cdist
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.lmpc.initialization import CentrelinePurePursuit, ControlledExcitation
from f1tenth_benchmarks.research.lmpc.run_initialization import ValidatedContinuousSim
from f1tenth_benchmarks.research.lmpc.lap_adapter import CoreLapTracker
from f1tenth_benchmarks.research.lmpc.state import paper_state, STATE_COLUMNS
from f1tenth_benchmarks.research.lmpc.validation import FootprintChecker
from f1tenth_benchmarks.simulator.dynamics_simulator import DynamicsSimulator
from f1tenth_benchmarks.simulator.dynamic_models import pid, accl_constraints
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file

MODES=('original','non_overshoot')
ROLES=('short_validation','high_speed_qualification','train','test')
DEFAULT_POLICY=dict(schema_version=1,map='esp',control_dt=.1,physics_dt=.01,
    low_speed_mps=2.5,high_speed_mps=3.,lookahead_m=.6,short_duration_s=12.,short_switch_s=6.,
    role_seeds=dict(short_validation=20261021,high_speed_qualification=20261022,train=20261019,test=20261020),
    excitation=dict(hold_steps=5,steering_amplitude_rad=.02,acceleration_amplitude_mps2=.6),
    safety=dict(max_abs_e_y_m=.25,max_abs_e_psi_rad=.5,max_speed_mps=3.3,footprint_margin_m=.05),
    qc=dict(command_acceleration_tolerance=2e-12,state_replay_tolerance=1e-12,
        minimum_warm_positive_a=5,minimum_warm_negative_a=5,warm_after_s=3.,accel_event_threshold=.05,
        minimum_offset_std_delta=.005,minimum_offset_std_a=.15,
        speed_region_halfwidth_mps=.15,minimum_samples_per_speed_region=50,minimum_test_common_per_speed_region=30),
    common_support=dict(features=['vx','vy','r','delta_cmd','a_cmd'],
        scales=[.25,.05,.5,.05,.5],metric='scaled_L1',nearest_distance_max=1.,
        require_training_marginal_box_intersection=True,
        scope='evaluation mask only; no change to A5 neighbor selection or weights; not a convex-hull certificate'))

def clean(x):
    if isinstance(x,np.ndarray):return clean(x.tolist())
    if isinstance(x,np.generic):return clean(x.item())
    if isinstance(x,dict):return {str(k):clean(v) for k,v in x.items()}
    if isinstance(x,(list,tuple)):return [clean(v) for v in x]
    return x
def read(p):return json.loads(Path(p).read_text())
def write(p,value):Path(p).write_text(json.dumps(clean(value),ensure_ascii=False,indent=2,allow_nan=False))
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def csv_write(p,rows):
    if not rows:return
    keys=list(dict.fromkeys(k for r in rows for k in r))
    with Path(p).open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader()
        for r in rows:writer.writerow({k:json.dumps(clean(v),allow_nan=False) if isinstance(v,(list,tuple,dict,np.ndarray)) else clean(v) for k,v in r.items()})

def speed_reference(role,completed_laps,s_unwrapped,elapsed_s,length,policy):
    low,high=policy['low_speed_mps'],policy['high_speed_mps']
    if role=='short_validation':return low if elapsed_s<policy['short_switch_s']-1e-10 else high
    if role=='high_speed_qualification':return high
    if role=='train':return low if completed_laps==0 else high
    if role=='test':
        segment=int((max(0.,s_unwrapped)%length)/length*4)
        return low if segment%2==0 else high
    raise ValueError('unknown sampling role')

def compose_command(pp_action,current_scalar_speed,excitation,control_dt,max_steer,max_a):
    pp_action=np.asarray(pp_action,float)
    baseline=np.array([pp_action[0],np.clip((pp_action[1]-current_scalar_speed)/control_dt,-max_a,max_a)])
    command=np.asarray(excitation.apply(baseline,max_steer,max_a))
    detail=dict(pp_delta_cmd=float(pp_action[0]),pp_target_speed=float(pp_action[1]),
        baseline_acceleration_command=float(baseline[1]),excitation=excitation.diagnostic,
        unclipped_command=baseline+excitation.offset,
        clipped=bool(np.any(command!=baseline+excitation.offset)),
        interface='acceleration command held for the control interval; PID target speed is a separate per-physics-step value')
    return command,detail

def state_safety(state,physical,policy):
    if state is None or not np.isfinite(state).all() or not np.isfinite(physical).all():return 'invalid_state_or_frenet'
    p=policy['safety']
    if abs(state[5])>p['max_abs_e_y_m']:return 'lateral_limit'
    if abs(state[3])>p['max_abs_e_psi_rad']:return 'heading_limit'
    if np.hypot(*state[:2])>p['max_speed_mps'] or state[0]<-1e-12:return 'speed_limit'
    return None

def quality_gate(metrics,role,policy):
    reasons=[]
    for name in ['completed_target','safe','alignment_passed','excitation_passed']:
        if not metrics.get(name):reasons.append(name)
    if role=='high_speed_qualification' and metrics.get('near_high_speed_samples',0)<policy['qc']['minimum_samples_per_speed_region']:reasons.append('high_speed_coverage')
    if role in ['train','test']:
        for key in ['near_low_speed_samples','near_high_speed_samples']:
            if metrics.get(key,0)<policy['qc']['minimum_samples_per_speed_region']:reasons.append(key)
    return not reasons,reasons

def alignment_verified(written_records,checked_records,max_acceleration_error,tolerance):
    """Keep alignment evidence distinct from a later safety/coverage rejection."""
    return (written_records>0 and written_records==checked_records
            and max_acceleration_error<=tolerance)

def collect_session(root,mode,role,policy,geom):
    from argparse import Namespace
    output=root/role/mode;output.mkdir(parents=True,exist_ok=False)
    session_id=f'{root.name}/{role}/{mode}'
    physics=load_parameter_file('simulator_params');vehicle=load_parameter_file('vehicle_params')
    sampled=np.loadtxt('maps/frenet/esp/geometry/centerline.csv',delimiter=',',skiprows=1)
    footprint=FootprintChecker(geom.raster,physics.vehicle_length,physics.vehicle_width,policy['safety']['footprint_margin_m'])
    sim=ValidatedContinuousSim('esp',footprint=footprint,timeout=240.,centreline_xy=sampled[:,:2],
        extra_params={'n_sim_steps':10,'steering_actuator_mode':mode})
    sim.tracker=CoreLapTracker(geom,sampled[:,:2])
    pp=CentrelinePurePursuit(sampled[:,:2],policy['low_speed_mps'],policy['lookahead_m'],vehicle.wheelbase,vehicle.max_steer)
    seed=policy['role_seeds'][role];excitation=ControlledExcitation(dict(seed=seed,**policy['excitation']))
    ref=geom.reference(0.);pose=np.r_[ref['xy'],ref['psi']]
    replay=DynamicsSimulator(Namespace(**vars(sim.params)));replay.reset(pose)
    target_laps=2 if role=='train' else 1
    metadata=dict(schema_version=1,session_id=session_id,mode=mode,partition=role,seed=seed,
        state_columns=STATE_COLUMNS,input_columns=['delta_cmd','a_cmd'],acceleration_interface=True,
        transition_alignment='sample i -- sample[i+1].command --> sample i+1',
        control_dt=policy['control_dt'],physics_dt=policy['physics_dt'],geometry_id=geom.geometry_id,
        frenet_length=geom.curve.L,start_pose=pose,initial_speed=0.,initial_fifo_policy='empty before the unchanged 10-tick reset initialization',
        requested_laps=target_laps if role!='short_validation' else None,requested_seconds=policy['short_duration_s'] if role=='short_validation' else None,
        simulator_params=vars(sim.params),vehicle_params=vars(vehicle),tracker_config=asdict(sim.tracker.config),policy=policy,
        status='identification data only; no LMPC database, Safe Set, Q-function or WLS fit',
        source_hashes={str(p):digest(p) for p in [Path(__file__),Path('params/simulator_params.yaml'),Path('params/vehicle_params.yaml'),
            Path('f1tenth_benchmarks/research/lmpc/initialization.py'),Path('f1tenth_benchmarks/research/lmpc/dynamics.py'),
            Path('f1tenth_benchmarks/simulator/dynamic_models.py'),Path('f1tenth_benchmarks/simulator/dynamics_simulator.py'),
            Path('maps/frenet/esp/geometry/manifest.json')]})
    write(output/'metadata.json',metadata)
    rows=[];events=[];transitions=[];failure=None;end_reason=None;max_replay=0.;max_a_error=0.;max_time_error=0.
    physical_ticks=0;checked_records=0;offsets=[];applied_a_min=np.inf;applied_a_max=-np.inf
    def record(stream,obs,command,detail=None,initializing=False):
        nonlocal max_replay,max_a_error,max_time_error,physical_ticks,checked_records,applied_a_min,applied_a_max
        fr=sim.tracker.frenet
        state=None
        if fr is not None and fr.get('valid') and fr.get('timestamp')==sim.current_time:
            state=paper_state(sim.current_state,fr,sim.current_time,geom.geometry_id)
        row=dict(session_id=session_id,mode=mode,partition=role,sample_id=len(rows),time=sim.current_time,elapsed_time=sim.elapsed_time,
            physical_state=sim.current_state.tolist(),paper_state=None if state is None else state.tolist(),frenet=fr,
            command=np.asarray(command).tolist(),input_mode='target_speed' if initializing else 'acceleration',initializing=initializing,
            steering_buffer=sim.dynamics_simulator.steer_buffer.tolist(),completed_full_laps=sim.tracker.completed_full_laps,
            reset_count=sim.reset_count,physics_ticks=sim.dynamics_simulator.trace,control_diagnostic=detail,
            terminal_reason=sim.terminal_reason,safety_reason=state_safety(state,sim.current_state,policy))
        # Save actual evidence before assertions, so a rejected transition remains reviewable.
        stream.write(json.dumps(clean(row),allow_nan=False)+'\n');stream.flush()
        previous=rows[-1] if rows else None;rows.append(row)
        if not initializing:
            assert previous is not None
            assert len(row['physics_ticks'])==10
            max_time_error=max(max_time_error,abs(row['time']-previous['time']-.1))
            assert max_time_error<1e-9
            assert np.array_equal(row['physics_ticks'][0]['pre_state'],previous['physical_state'])
            assert np.array_equal(row['physics_ticks'][-1]['post_state'],row['physical_state'])
        fifo=list(replay.steer_buffer)
        for j,tick in enumerate(row['physics_ticks']):
            pre=np.array(tick['pre_state']);post=np.array(tick['post_state'])
            assert np.array_equal(replay.state,pre)
            if j:assert np.array_equal(pre,row['physics_ticks'][j-1]['post_state'])
            delayed=fifo[-1] if len(fifo)>=2 else 0.
            assert delayed==tick['delayed_target_steering']
            if len(fifo)>=2:fifo=fifo[:-1]
            fifo=[command[0]]+fifo
            if not initializing:
                assert tick['input_mode']=='acceleration' and np.array_equal(tick['command'],command)
                max_a_error=max(max_a_error,abs(tick['requested_acceleration']-command[1]))
                p=sim.params
                gain=(10. if pre[3]>0 else 2.)*p.a_max/(p.v_max if command[1]>0 else -p.v_min)
                expected_speed=pre[3]+command[1]/gain
                assert abs(expected_speed-tick['pid_target_speed'])<1e-12
                a,rate=pid(tick['pid_target_speed'],tick.get('effective_target_steering',delayed),pre[3],pre[2],p.sv_max,p.a_max,p.v_max,p.v_min)
                assert abs(a-command[1])<policy['qc']['command_acceleration_tolerance']
                expected_applied=accl_constraints(pre[3],a,p.v_switch,p.a_max,p.v_min,p.v_max)
                assert abs(expected_applied-tick['applied_acceleration'])<1e-12
                applied_a_min=min(applied_a_min,tick['applied_acceleration']);applied_a_max=max(applied_a_max,tick['applied_acceleration'])
            result=replay.update_pose(command[0],tick['pid_target_speed'])
            max_replay=max(max_replay,float(abs(result-post).max()))
            assert max_replay<=policy['qc']['state_replay_tolerance']
            assert sim.params.s_min<=post[2]<=sim.params.s_max
            assert np.array_equal(replay.steer_buffer,fifo)
            physical_ticks+=1
        assert np.array_equal(row['steering_buffer'],replay.steer_buffer)
        if obs.get('lap_event') is not None:
            event=asdict(obs['lap_event']);event.update(row_index=row['sample_id'],session_id=session_id,mode=mode,partition=role)
            events.append(event)
        if previous is not None and not initializing:
            tr=dict(session_id=session_id,mode=mode,partition=role,source_sample_id=previous['sample_id'],target_sample_id=row['sample_id'],
                source_id=f'{session_id}:{previous["sample_id"]}',target_id=f'{session_id}:{row["sample_id"]}',
                time=previous['time'],next_time=row['time'],elapsed_time=previous['elapsed_time'],
                command=row['command'],delta_cmd=command[0],a_cmd=command[1],input_mode='acceleration',
                x=previous['paper_state'],x_next=row['paper_state'],physical_state=previous['physical_state'],physical_state_next=row['physical_state'],
                steering_buffer=previous['steering_buffer'],steering_buffer_next=row['steering_buffer'],
                v_ref=detail['pp_target_speed'],excitation_offset=detail['excitation']['offset'],
                clip=detail['clipped'],completed_laps_before=previous['completed_full_laps'],
                oracle_mean_actual_steering=sum(t['pre_state'][2]*t['dt'] for t in row['physics_ticks'])/.1,
                mean_applied_acceleration_descriptive=sum(t['applied_acceleration']*t['dt'] for t in row['physics_ticks'])/.1,
                valid=state is not None and previous['paper_state'] is not None and row['safety_reason'] is None and row['terminal_reason'] is None)
            transitions.append(tr)
        checked_records+=1
        return row
    with (output/'samples.jsonl').open('x') as stream:
        try:
            obs,done,_=sim.reset(pose);row=record(stream,obs,[0.,0.],initializing=True)
            if done or row['safety_reason']:raise RuntimeError(row['safety_reason'] or sim.terminal_reason)
            while not done:
                if role=='short_validation' and sim.elapsed_time>=policy['short_duration_s']-1e-9:end_reason='short_target_reached';break
                if role!='short_validation' and sim.tracker.completed_full_laps>=target_laps:end_reason='requested_laps_reached';break
                state=np.array(row['paper_state'])
                pp.speed=speed_reference(role,sim.tracker.completed_full_laps,state[4],sim.elapsed_time,geom.curve.L,policy)
                command,detail=compose_command(pp.plan(obs),sim.current_state[3],excitation,.1,vehicle.max_steer,sim.params.a_max)
                offsets.append(detail['excitation']['offset'])
                obs,done=sim.step_acceleration(command);row=record(stream,obs,command,detail)
                if done or row['safety_reason']:raise RuntimeError(row['safety_reason'] or sim.terminal_reason)
                if sim.control_steps%400==0:print(role,mode,'steps',sim.control_steps,'laps',sim.tracker.completed_full_laps,flush=True)
        except Exception as exc:
            failure=f'{type(exc).__name__}: {exc}';end_reason='rejected'
            write(output/'failure.json',dict(error=failure,last_sample_id=len(rows)-1,footprint_failure=sim.footprint_failure,terminal_reason=sim.terminal_reason))
    with (output/'events.jsonl').open('x') as f:
        for event in events:f.write(json.dumps(clean(event),allow_nan=False)+'\n')
    csv_write(output/'transitions.csv',transitions)
    valid=[tr for tr in transitions if tr['valid']]
    if valid:
        np.savez_compressed(output/'transitions.npz',states=np.array([tr['x'] for tr in valid]),next_states=np.array([tr['x_next'] for tr in valid]),
            inputs=np.array([tr['command'] for tr in valid]),source_sample_ids=np.array([tr['source_sample_id'] for tr in valid]),
            target_sample_ids=np.array([tr['target_sample_id'] for tr in valid]),times=np.array([tr['time'] for tr in valid]),next_times=np.array([tr['next_time'] for tr in valid]),
            physical_states=np.array([tr['physical_state'] for tr in valid]),physical_next_states=np.array([tr['physical_state_next'] for tr in valid]),
            initial_fifos=np.array([tr['steering_buffer'] for tr in valid]),mode=np.array(mode),partition=np.array(role),session_id=np.array(session_id),
            state_columns=np.array(STATE_COLUMNS),input_columns=np.array(['delta_cmd','a_cmd']))
    u=np.array([tr['command'] for tr in valid]).reshape(-1,2);x=np.array([tr['x'] for tr in valid]).reshape(-1,6)
    elapsed=np.array([tr['elapsed_time'] for tr in valid]);warm=elapsed>=policy['qc']['warm_after_s']
    ref=np.array([tr['v_ref'] for tr in valid]);warm_a=u[warm,1] if len(u) else np.array([])
    offset=np.array(offsets).reshape(-1,2)
    positive=int(np.sum(warm_a>policy['qc']['accel_event_threshold']));negative=int(np.sum(warm_a<-policy['qc']['accel_event_threshold']))
    offset_std=offset.std(0) if len(offset) else np.zeros(2)
    observed=np.array([r['paper_state'] for r in rows if r['paper_state'] is not None]).reshape(-1,6)
    excitation_passed=(positive>=policy['qc']['minimum_warm_positive_a'] and negative>=policy['qc']['minimum_warm_negative_a']
        and offset_std[0]>=policy['qc']['minimum_offset_std_delta'] and offset_std[1]>=policy['qc']['minimum_offset_std_a'])
    metrics=dict(session_id=session_id,mode=mode,partition=role,seed=seed,samples=len(rows),transitions=len(transitions),valid_transitions=len(valid),
        completed_laps=sim.tracker.completed_full_laps,physics_ticks=physical_ticks,elapsed_s=sim.elapsed_time,reset_count=sim.reset_count,
        completed_target=(end_reason=='short_target_reached' if role=='short_validation' else sim.tracker.completed_full_laps==target_laps),
        end_reason=end_reason,error=failure,safe=failure is None and sim.footprint_failure is None,
        alignment_passed=alignment_verified(len(rows),checked_records,max_a_error,policy['qc']['command_acceleration_tolerance']),
        excitation_passed=bool(excitation_passed),warm_positive_a=positive,warm_negative_a=negative,offset_std=offset_std,
        near_low_speed_samples=int(np.sum(abs(x[:,0]-policy['low_speed_mps'])<=policy['qc']['speed_region_halfwidth_mps'])),
        near_high_speed_samples=int(np.sum(abs(x[:,0]-policy['high_speed_mps'])<=policy['qc']['speed_region_halfwidth_mps'])),
        replay_max_state_error=max_replay,requested_acceleration_max_error=max_a_error,max_timing_error_s=max_time_error,
        max_abs_ey=float(abs(observed[:,5]).max()) if len(observed) else None,max_abs_epsi=float(abs(observed[:,3]).max()) if len(observed) else None,
        vx_range=[float(x[:,0].min()),float(x[:,0].max())] if len(x) else None,
        delta_range=[float(u[:,0].min()),float(u[:,0].max())] if len(u) else None,
        a_cmd_range=[float(u[:,1].min()),float(u[:,1].max())] if len(u) else None,
        applied_acceleration_range=[applied_a_min,applied_a_max] if len(u) else None,
        command_clip_count=sum(tr['clip'] for tr in valid),lap_times=[e['duration'] for e in events],
        footprint_checks=sim.footprint_checks,footprint_failure=sim.footprint_failure,
        input_interface='true delta_cmd/a_cmd; not target speed or realized acceleration',
        excluded_from_formal_identification=role not in ['train','test'])
    passed,reasons=quality_gate(metrics,role,policy);metrics.update(passed=passed,rejection_reasons=reasons)
    write(output/'session.json',metrics)
    write(output/'data_hashes.json',{p.name:digest(p) for p in output.iterdir() if p.is_file()})
    print('SESSION',role,mode,'passed',passed,'laps',metrics['completed_laps'],'transitions',len(valid),'error',failure,'gate',reasons,flush=True)
    return metrics

def paired_checks(root,role,policy):
    left=read(root/role/'original'/'metadata.json');right=read(root/role/'non_overshoot'/'metadata.json')
    for key in ['seed','start_pose','initial_speed','initial_fifo_policy','control_dt','physics_dt','geometry_id','policy']:
        assert left[key]==right[key],key
    l=np.load(root/role/'original'/'transitions.npz');r=np.load(root/role/'non_overshoot'/'transitions.npz')
    def offsets(mode):
        rows=[json.loads(s) for s in (root/role/mode/'samples.jsonl').read_text().splitlines()]
        return np.array([row['control_diagnostic']['excitation']['offset'] for row in rows[1:]])
    a,b=offsets('original'),offsets('non_overshoot');count=min(len(a),len(b));assert np.array_equal(a[:count],b[:count])
    return dict(role=role,paired_parameters_initial_conditions=True,excitation_offsets_identical_on_common_control_indices=True,
        common_control_indices=count,raw_commands_equal=bool(np.array_equal(l['inputs'][:count],r['inputs'][:count])),
        note='PP feedback and lap/position speed switches may yield different commands despite matched offsets and policy')

def support_analysis(root,policy):
    arrays={role:{mode:np.load(root/role/mode/'transitions.npz') for mode in MODES} for role in ['train','test']}
    features=lambda z:np.c_[z['states'][:,:3],z['inputs']]
    train={m:features(arrays['train'][m]) for m in MODES}
    lower=np.maximum(train['original'].min(0),train['non_overshoot'].min(0));upper=np.minimum(train['original'].max(0),train['non_overshoot'].max(0))
    scale=np.array(policy['common_support']['scales']);bounds=np.c_[lower,upper]
    (root/'common_support').mkdir();results=[];distribution=[];mask_rows=[]
    for role in ['train','test']:
        for mode in MODES:
            z=arrays[role][mode];q=features(z)
            for j,name in enumerate(policy['common_support']['features']):
                distribution.append(dict(mode=mode,partition=role,feature=name,n=len(q),min=q[:,j].min(),q01=np.quantile(q[:,j],.01),
                    q25=np.quantile(q[:,j],.25),median=np.median(q[:,j]),q75=np.quantile(q[:,j],.75),q99=np.quantile(q[:,j],.99),max=q[:,j].max(),std=q[:,j].std()))
            if role!='test':continue
            inside=np.all((q>=lower)&(q<=upper),axis=1);distances={};indices={}
            for other in MODES:
                ds=[];ix=[]
                for start in range(0,len(q),128):
                    matrix=cdist(q[start:start+128]/scale,train[other]/scale,metric='cityblock')
                    arg=matrix.argmin(1);ds.extend(matrix[np.arange(len(arg)),arg]);ix.extend(arg)
                distances[other]=np.array(ds);indices[other]=np.array(ix)
            mask=inside.copy()
            for other in MODES:mask&=distances[other]<=policy['common_support']['nearest_distance_max']
            for i in range(len(q)):
                mask_rows.append(dict(mode=mode,partition='test',session_id=str(z['session_id']),source_sample_id=z['source_sample_ids'][i],target_sample_id=z['target_sample_ids'][i],
                    common_box=inside[i],common_support=mask[i],nearest_original_scaled_L1=distances['original'][i],nearest_non_overshoot_scaled_L1=distances['non_overshoot'][i],
                    nearest_original_train_source_sample=arrays['train']['original']['source_sample_ids'][indices['original'][i]],
                    nearest_non_overshoot_train_source_sample=arrays['train']['non_overshoot']['source_sample_ids'][indices['non_overshoot'][i]],
                    **{name:q[i,j] for j,name in enumerate(policy['common_support']['features'])}))
            csv_write(root/'common_support'/f'{mode}_test_mask.csv',[r for r in mask_rows if r['mode']==mode])
            results.append(dict(mode=mode,test_n=len(q),box_supported_n=int(inside.sum()),common_supported_n=int(mask.sum()),fraction=float(mask.mean()),
                speed_low_common=int(np.sum(mask&(abs(q[:,0]-2.5)<=.15))),speed_high_common=int(np.sum(mask&(abs(q[:,0]-3.)<=.15)))))
    csv_write(root/'input_distributions.csv',distribution)
    save=dict(policy=policy['common_support'],training_marginal_intersection=bounds,results=results,
        limitation='5D marginal box plus scaled nearest-neighbor support; not a joint convex-hull certificate. Opposite-mode data are used only to flag evaluation support, never for fitting.')
    write(root/'common_support'/'summary.json',save)
    figures(root,arrays,mask_rows,policy)
    return save

def figures(root,arrays,masks,policy):
    out=root/'figures';out.mkdir(exist_ok=True)
    for role in ['train','test']:
        fig,axs=plt.subplots(4,1,figsize=(14,11),constrained_layout=True)
        for mode,color in [('original','#d55e00'),('non_overshoot','#0072b2')]:
            z=arrays[role][mode];time=z['times']-z['times'][0]
            for ax,values,label in zip(axs,[z['states'][:,0],z['inputs'][:,0],z['inputs'][:,1],z['states'][:,5]],['vx (m/s)','delta_cmd (rad)','true a_cmd (m/s2)','ey (m)']):
                ax.plot(time,values,color=color,label=mode,lw=.8);ax.set(xlabel='Time (s)',ylabel=label);ax.grid(alpha=.2);ax.legend()
        fig.suptitle(f'{role}: independent mode-specific PP feedback, matched excitation policy')
        fig.savefig(out/f'{role}_states_inputs.png',dpi=160);plt.close(fig)
    fig,axs=plt.subplots(1,2,figsize=(13,5),constrained_layout=True)
    for mode,color in [('original','#d55e00'),('non_overshoot','#0072b2')]:
        rows=[r for r in masks if r['mode']==mode];a=np.array([r['a_cmd'] for r in rows]);vx=np.array([r['vx'] for r in rows]);mask=np.array([r['common_support'] for r in rows])
        axs[0].scatter(vx,a,s=5,alpha=.3,color=color,label=mode+' all test');axs[1].scatter(vx[mask],a[mask],s=5,alpha=.3,color=color,label=mode+' common')
    for ax,title in zip(axs,['All held-out test transitions','Predefined common-support subset']):ax.set(xlabel='vx (m/s)',ylabel='a_cmd (m/s2)',title=title);ax.legend();ax.grid(alpha=.2)
    fig.savefig(out/'common_support.png',dpi=160);plt.close(fig)

def run(root):
    root=Path(root);root.mkdir(parents=True,exist_ok=True)
    if (root/'sampling_policy.json').exists():raise FileExistsError('use a new run directory; never overwrite evidence')
    policy=json.loads(json.dumps(DEFAULT_POLICY));write(root/'sampling_policy.json',policy)
    protected=set()
    for base in ['Data/research/lmpc','Data/research/steering_actuator','Data/research/steering_wls/data_readiness_20261019_001',
                 'f1tenth_benchmarks/research/lmpc','f1tenth_benchmarks/simulator','maps/frenet/esp/geometry','params']:
        protected.update(str(p) for p in Path(base).rglob('*') if p.is_file() and '__pycache__' not in p.parts)
    before={p:digest(p) for p in sorted(protected)};write(root/'protected_before.json',before)
    (root/'design_review.md').write_text('''# 採樣前固定設計與最小修改

依最新要求，以2.5/3.0m/s取代舊方案1.5/2.5。A5六維paper_state、delta_cmd/a_cmd與i→i+1對齊沿用原實作。只新增獨立collector與單元測試，不改既有PP、激勵器、recording基類、致動器、WLS、QP或LMPC。

順序固定：兩模式各12秒short_validation；均通過後各一圈含同樣激勵的3.0m/s high_speed_qualification；均通過後每模式兩圈train（圈1=2.5、圈2=3.0）與一次獨立重設的一圈test（四個等長s區段交替2.5/3.0）。正式資料六圈；前置兩圈資格檢查與短時資料不納入正式training或test。

role內兩模式同seed與起點，train/test seed不同。激勵沿用ControlledExcitation每5步保持delta±.02rad、a±.6m/s²。使用原加速度介面及每子步PID inversion，輸出command[1]是真正施加的a_cmd。v_ref、baseline a、offset、clip、requested/applied a與inner target_speed分欄保存。

安全門檻預先固定ey .25m、heading .5rad、speed3.3m/s、車身margin .05m、物理輪角及轉向速率界限。驗證鏈、加速度命令一致性、激勵有效性或任一安全條件不通過，保留失敗資料並停止後續階段；不改門檻、不調PP。

共同支持只以兩模式train集合定義5D邊界交集與scaled-L1最近距離≤1，尺度[vx,vy,r,delta,a]=[.25,.05,.5,.05,.5]。先固定政策再觀察資料，不修改A5選點、不擬合模型；每模式test另存mask，未來誤差須分all/common報告。
''')
    suite=unittest.TestSuite()
    for pattern in ['test_steering_wls_sampling.py','test_steering_actuator.py']:
        suite.addTests(unittest.defaultTestLoader.discover('tests/research',pattern=pattern))
    with (root/'unit_tests.txt').open('w') as stream:result=unittest.TextTestRunner(stream=stream,verbosity=2).run(suite)
    write(root/'unit_tests.json',dict(tests=result.testsRun,failures=len(result.failures),errors=len(result.errors),passed=result.wasSuccessful()))
    sessions=[];pairs=[];stopped=None
    if not result.wasSuccessful():stopped='unit_tests'
    geom=load_geometry('maps/frenet/esp/geometry','maps/esp.yaml',inspect=True)
    if stopped is None:
        for role in ROLES:
            for mode in MODES:
                metrics=collect_session(root,mode,role,policy,geom);sessions.append(metrics)
                if not metrics['passed']:
                    stopped=f'{role}/{mode}';break
            if stopped:break
            pairs.append(paired_checks(root,role,policy))
    support=None
    if stopped is None:
        train_ids={tr['session_id'] for tr in sessions if tr['partition']=='train'};test_ids={tr['session_id'] for tr in sessions if tr['partition']=='test'}
        assert train_ids.isdisjoint(test_ids)
        support=support_analysis(root,policy)
    after={p:digest(p) for p in before};assert before==after
    csv_write(root/'session_summary.csv',sessions)
    support_adequate=(support is not None and all(r['speed_low_common']>=policy['qc']['minimum_test_common_per_speed_region']
        and r['speed_high_common']>=policy['qc']['minimum_test_common_per_speed_region'] for r in support['results']))
    summary=dict(complete=stopped is None,stopped_at=stopped,sessions=sessions,paired_checks=pairs,common_support=support,
        common_support_adequate=bool(support_adequate),ready_for_controlled_WLS_comparison=bool(stopped is None and support_adequate),
        formal_laps=sum(r['completed_laps'] for r in sessions if r['partition'] in ['train','test']),
        qualification_laps=sum(r['completed_laps'] for r in sessions if r['partition']=='high_speed_qualification'),
        WLS_fits=False,QP_changes=False,LMPC_runs=False,protected_file_count=len(before),protected_unchanged=before==after,
        versions=dict(python=platform.python_version(),numpy=np.__version__),date='2026-10-19 Asia/Taipei')
    write(root/'summary.json',summary);write(root/'verification.json',dict(before=before,after=after,unchanged=True,no_model_fit=True))
    print('SAMPLING COMPLETE',summary['complete'],'STOP',stopped,'formal_laps',summary['formal_laps'],flush=True)
    return summary

def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output-dir',required=True);args=p.parse_args();run(args.output_dir)

if __name__=='__main__':main()
