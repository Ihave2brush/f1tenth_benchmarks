"""ESP LMPC: paper mode collects three PP laps and hands off without reset."""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import platform
from time import perf_counter

import numpy as np
import osqp
import yaml

from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.utils.BasePlanner import load_parameter_file
from .config import load_config
from .controller import LocalLMPC, MPCFailure
from .database import LapDatabase, sha256
from .lap_adapter import CoreLapTracker
from .local_model import LocalModel, augmented
from .planning_scope import ScopedPlanningGate
from .run_initialization import ValidatedContinuousSim
from .run_smoke import replay
from .state import paper_state, STATE_COLUMNS
from .validation import CandidateValidator, FootprintChecker
from .initialization import CentrelinePurePursuit, ControlledExcitation


def load_b3(path):
    value = yaml.safe_load(Path(path).read_text())
    if (not isinstance(value, dict) or set(value)-{'experiment_id','initialization_source','exploration','kernel_selection','command_filter'} != {'schema_version', 'base_config', 'database', 'scope', 'prediction_value_mode', 'settings', 'sources'}
            or type(value['schema_version']) is not int or value['schema_version'] != 1
            or value['scope'] != 'esp_simulation_per_action_validation'):
        raise ValueError('invalid B3 configuration/scope')
    if {'experiment_id','initialization_source','exploration'} & set(value):
        exploration=value.get('exploration',{})
        base_exploration={k:v for k,v in exploration.items() if k not in ('historical_steering_multiplier','steering_scope','vx_scope')} if isinstance(exploration,dict) else None
        if (value.get('experiment_id') not in ('A3','A4','A5','A6','A7') or value['prediction_value_mode']!='paper_direct'
                or base_exploration!={'historical_vx_multiplier':1.2,'future_nodes':12,'update_on':'successful_history_only'}
                or not isinstance(value.get('initialization_source'),dict)
                or set(value['initialization_source'])!={'directory','database'}
                or any(not isinstance(v,str) or not v for v in value['initialization_source'].values())):
            raise ValueError('A3 requires explicit A2 source and 12-node historical exploration policy')
        if value['experiment_id'] in ('A4','A5','A6','A7'):
            if (exploration.get('vx_scope')!='local_80' or exploration.get('steering_scope')!='local_80'
                    or exploration.get('historical_steering_multiplier')!=1.2):
                raise ValueError('A4 requires local-80 velocity and steering bounds at multiplier 1.2')
        elif 'vx_scope' in exploration:
            raise ValueError('local velocity exploration is restricted to A4')
        if {'historical_steering_multiplier','steering_scope'} & set(exploration):
            if (exploration.get('historical_steering_multiplier')!=1.2
                    or exploration.get('steering_scope') not in ('local_80','global_history')):
                raise ValueError('A3 steering exploration requires multiplier 1.2 and explicit scope')
    if value.get('experiment_id') in ('A5','A6'):
        expected_kernel=dict(distance='input_aware_weighted_L1',bandwidth=15,points=80,
            selection_steering='delta_cmd' if value['experiment_id']=='A5' else 'delta_real',regression_steering='delta_cmd')
        if value.get('kernel_selection')!=expected_kernel:raise ValueError('invalid selection-only A5/A6 kernel')
    elif 'kernel_selection' in value:raise ValueError('input-aware kernel restricted to A5/A6')
    if 'command_filter' in value:
        if (value.get('experiment_id')!='A5' or value['command_filter']!={'steering_zero_deadzone_rad':.005}):
            raise ValueError('this filter experiment requires A5 with 0.005 rad zero deadzone')
    if value['prediction_value_mode'] not in ('smoothed_servo', 'original_pid', 'original_pid_geometric',
                                             'original_pid_geometric_secant',
                                             'paper_direct',
                                             'smoothed_servo_geometric'):
        raise ValueError('invalid prediction_value_mode')
    expected = {'timeout_s', 'footprint_margin_m', 'max_speed_mps', 'max_abs_heading_rad',
                'max_abs_lateral_m', 'max_acceleration_mps2', 'residual_ridge', 'input_rate_weights',
                'qp_absolute_tolerance', 'qp_max_iterations', 'equality_gate', 'sqp_iterations', 'prediction_trust_box'}
    s = value['settings']
    if not isinstance(s, dict) or set(s)-{'warm_start_mode', 'acceleration_gain_policy', 'identification_gate', 'constraint_mode', 'finish_cost_mode', 'shadow_mode'} != expected:
        raise ValueError('invalid B3 settings')
    if s.get('warm_start_mode', 'shifted') not in ('shifted', 'reroll_measured'):
        raise ValueError('invalid warm_start_mode')
    if s.get('acceleration_gain_policy', 'strict') not in ('strict', 'dt_cos_beta_when_unidentified'):
        raise ValueError('invalid acceleration_gain_policy')
    if s.get('identification_gate', 'strict') not in ('strict', 'report_only'):
        raise ValueError('invalid identification_gate')
    if s.get('constraint_mode','legacy') not in ('legacy','paper_simulator_polytope'):
        raise ValueError('invalid constraint_mode')
    if s.get('finish_cost_mode','legacy') not in ('legacy','literal_indicator'):
        raise ValueError('invalid finish_cost_mode')
    if s.get('shadow_mode','full_horizon') not in ('full_horizon','first_action'):
        raise ValueError('invalid shadow_mode')
    for key, item in s.items():
        if key in ('warm_start_mode', 'acceleration_gain_policy', 'identification_gate', 'constraint_mode', 'finish_cost_mode', 'shadow_mode'):
            continue
        a = np.asarray(item if isinstance(item, list) else [item])
        if a.dtype.kind not in 'if' or not np.isfinite(a).all() or np.any(a <= 0):
            raise ValueError('invalid positive setting: '+key)
    if len(s['input_rate_weights']) != 2 or len(s['prediction_trust_box']) != 9:
        raise ValueError('invalid B3 vector setting')
    if (type(s['sqp_iterations']) is not int or type(s['qp_max_iterations']) is not int
            or not .5 < s['max_speed_mps'] <= 8 or s['max_acceleration_mps2'] > 9.51
            or s['max_abs_heading_rad'] >= np.pi):
        raise ValueError('B3 setting violates experiment/physical limits')
    if (not isinstance(value['sources'], dict) or set(value['sources']) != {'paper', 'adaptations', 'limits', 'numerics'}
            or any(not isinstance(v, str) or not v for v in value['sources'].values())):
        raise ValueError('B3 parameter sources required')
    return value


def run(config_path, output_dir=None, max_steps=None, stop_progress_m=None):
    settings_file = load_b3(config_path); settings = settings_file['settings'].copy()
    if settings_file.get('experiment_id') != 'A7':
        raise ValueError('use run_lmpc for baseline experiments; this runner requires A7')
    paper_mode = True
    if paper_mode and settings['sqp_iterations'] != 1:
        raise ValueError('paper mode builds one fixed ATV horizon per control step')
    base = load_config(settings_file['base_config'])
    excitation_settings=base.resolved.get('experiment',{}).get('excitation')
    if excitation_settings and not paper_mode:
        raise ValueError('A2 excitation requires paper mode continuous PP initialization')
    root = Path(output_dir) if output_dir else Path('Data/research/lmpc')/('b3_'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ'))
    root.mkdir(parents=True, exist_ok=False)
    report = dict(stage='B3_local_lmpc', passed=False, learning_demonstrated=False,
                  planning_allowed=False, domain_verified=False, continuous_verified=False,
                  config=settings_file, paper=base.paper, state_columns=STATE_COLUMNS,
                  model='A7_ideal_endpoint_actuator_state' if paper_mode else 'original_actuator_prior_plus_local_affine_residual',
                  stationary_new_experiment=not paper_mode, terminal_history_fixed=True,
                  experiment_id=settings_file.get('experiment_id',base.experiment_id),initialization_speed_mps=base.initialization_speed,
                  initialization_excitation=excitation_settings,
                  python=platform.python_version(), numpy=np.__version__, osqp=osqp.__version__,
                  code_hashes={str(p):sha256(p) for p in sorted(Path(__file__).parent.glob('*.py'))},
                  input_hashes={str(p):sha256(p) for p in ([Path(config_path), base.path]+(
                                [] if paper_mode else [Path(settings_file['database'])/'database.json']))})
    if stop_progress_m is not None:
        if not np.isfinite(stop_progress_m) or stop_progress_m <= 0:
            raise ValueError('stop progress must be finite and positive')
        report.update(stage='B2.5_short_closed_loop', probe_only=True,
                      short_target_progress_m=float(stop_progress_m), short_target_reached=False)
        (root/'preregistered_short_scope.json').write_text(json.dumps(dict(
            target_progress_m=float(stop_progress_m), max_steps=max_steps,
            config_hash=sha256(config_path), code_hashes=report['code_hashes'],
            scope='three PP laps then continuous LMPC probe' if paper_mode else 'stationary start through first bend'), indent=2))
    rows, events, timings, errors = [], [], [], []
    sim = None; failure = None; controller = None
    sample_stream = (root/'samples.jsonl').open('x')
    control_stream = (root/'controls.jsonl').open('x')
    event_stream = (root/'events.jsonl').open('x')
    try:
        geometry = load_geometry(base.adaptation['geometry_path'], base.adaptation['map_yaml'], inspect=True)
        database = None if paper_mode else LapDatabase.load(settings_file['database'])
        if database is not None and (database.geometry_id != geometry.geometry_id or database.dt != .1):
            raise ValueError('B3 database/core frame mismatch')
        physics = load_parameter_file('simulator_params')
        vehicle = load_parameter_file('vehicle_params')
        if not np.allclose([physics.timestep,physics.sv_min,physics.sv_max,physics.s_min,physics.s_max],
                           [.01,-3.2,3.2,-.4189,.4189],rtol=0,atol=1e-12):
            raise ValueError('A7 scalar actuator requires fixed ESP tick/rate/angle parameters')
        sampled = np.loadtxt(Path(base.adaptation['geometry_path'])/'centerline.csv', delimiter=',', skiprows=1)
        footprint = FootprintChecker(geometry.raster, physics.vehicle_length, physics.vehicle_width,
                                     settings['footprint_margin_m'])
        validator = CandidateValidator(geometry, footprint, settings['max_speed_mps'],
                                       physics.s_max if paper_mode else vehicle.max_steer, settings['max_acceleration_mps2'])
        sim = ValidatedContinuousSim('esp', footprint=footprint,
                                    timeout=settings['timeout_s']+(base.initialization['timeout'] if paper_mode else 0),
                                    extra_params={'n_sim_steps':base.substeps(physics.timestep)}, centreline_xy=sampled[:, :2])
        sim.tracker = CoreLapTracker(geometry, sampled[:, :2])
        ref = geometry.reference(0.); start_pose = np.r_[ref['xy'], ref['psi']]
        report.update(geometry_id=geometry.geometry_id, length=geometry.curve.L, simulator_params=vars(sim.params),
                      start_pose=start_pose.tolist(), source_database=None if paper_mode else settings_file['database'],
                      initial_history_policy='fresh_three_PP_laps_in_same_session' if paper_mode else 'configured_database',
                      historical_npz_hashes={} if paper_mode else {p.name:sha256(p) for p in Path(settings_file['database']).glob('*.npz')})
        if paper_mode:
            from .actuator7_model import Actuator7Model as PaperLocalModel
            from .actuator7_controller import Actuator7LMPC as PaperLocalLMPC
            source_spec=settings_file.get('initialization_source')
            replay_rows=None
            if source_spec:
                from .verify_initialization import verify_initialization
                source_verification=verify_initialization(source_spec['directory'])
                replay_rows=[json.loads(s) for s in (Path(source_spec['directory'])/'samples.jsonl').read_text().splitlines()]
                source_session=json.loads((Path(source_spec['directory'])/'session.json').read_text())
                if source_session['experiment_id']!='A2' or source_session['config']!=base.resolved:
                    raise ValueError('A3 must reuse the configured A2 initialization')
                report.update(reused_A2_initialization=source_spec,source_initialization_verification=source_verification)
        else:
            model = LocalModel(database, geometry, vars(sim.params), settings['residual_ridge'], settings_file['prediction_value_mode'])
            controller = LocalLMPC(database, model, settings)
        report.update(identification_source_count=0 if paper_mode else len(model.source_ids), identification_measured_only=True,
                      prediction_state_dimension=7 if paper_mode else 9,
                      historical_prior_residual_rmse=None if paper_mode else np.sqrt(np.mean(model.residuals**2, axis=0)).tolist())
        gate = ScopedPlanningGate(geometry, validator, vars(sim.params), settings)

        phase='pp' if paper_mode else 'lmpc';s_origin=0.;handoff_index=0;lmpc_start_steps=0
        pp=None
        excitation=ControlledExcitation(excitation_settings) if excitation_settings else None
        def record(obs, action, initializing=False):
            state = paper_state(sim.current_state, sim.tracker.frenet, sim.current_time, geometry.geometry_id) if obs['projection_valid'] else None
            row = dict(time=sim.current_time, elapsed_time=sim.elapsed_time,
                       physical_state=sim.current_state.tolist(), paper_state=None if state is None else state.tolist(),
                       frenet=sim.tracker.frenet, completed_full_laps=sim.tracker.completed_full_laps,
                       reset_count=sim.reset_count, command=np.asarray(action).tolist(),
                       input_mode='target_speed' if initializing else 'acceleration', initializing=initializing,
                       phase=phase,sample_id=len(rows),target_speed_reference=base.initialization_speed if phase=='pp' else None,
                       physics_ticks=sim.dynamics_simulator.trace, steering_buffer=sim.dynamics_simulator.steer_buffer.tolist(),
                       controller_diagnostic=pp.diagnostic if phase=='pp' and pp is not None else None,
                       excitation_diagnostic=excitation.diagnostic if phase=='pp' and excitation is not None else None,
                       terminal_reason=sim.terminal_reason)
            local=None if state is None else state.copy()
            if local is not None:local[4]-=s_origin
            row['lmpc_state']=local.tolist() if phase=='lmpc' and local is not None else None
            rows.append(row); sample_stream.write(json.dumps(row, allow_nan=False)+'\n'); sample_stream.flush()
            if obs['lap_event']:
                event = asdict(obs['lap_event']); event.update(row_index=len(rows)-1, physical_sample_time=sim.current_time)
                events.append(event); event_stream.write(json.dumps(event)+'\n'); event_stream.flush()
            return local

        obs, done, _ = sim.reset(start_pose)
        state = record(obs, [0., 0.], True)
        if paper_mode and replay_rows is not None:
            np.testing.assert_allclose(sim.current_state,replay_rows[0]['physical_state'],rtol=0,atol=1e-12)
        if paper_mode:
            pp=CentrelinePurePursuit(sampled[:,:2],base.initialization_speed,
                base.initialization['lookahead_m'],vehicle.wheelbase,vehicle.max_steer)
            while not done and sim.tracker.completed_full_laps<3:
                action=pp.plan(obs)
                action[1]=np.clip((action[1]-sim.current_state[3])/base.paper['control_dt'],-physics.a_max,physics.a_max)
                if excitation is not None:
                    action=excitation.apply(action,vehicle.max_steer,physics.a_max)
                if replay_rows is not None:
                    original=replay_rows[len(rows)]
                    np.testing.assert_allclose(action,original['command'],rtol=0,atol=1e-12)
                    action=original['command']
                obs,done=sim.step_acceleration(action);state=record(obs,action)
                if replay_rows is not None:
                    np.testing.assert_allclose(sim.current_state,original['physical_state'],rtol=0,atol=1e-12)
                    np.testing.assert_array_equal(sim.dynamics_simulator.steer_buffer,original['steering_buffer'])
                if (done or state is None or abs(state[3])>base.initialization['max_abs_e_psi_rad']
                        or abs(state[5])>base.initialization['max_abs_e_y_m']
                        or sim.elapsed_time>base.initialization['timeout']):
                    raise MPCFailure('PP initialization failed',dict(terminal_reason=sim.terminal_reason))
                if sim.tracker.completed_full_laps and obs['lap_event']:
                    print('PP lap',sim.tracker.completed_full_laps,'time',round(sim.elapsed_time,3),flush=True)
            if sim.tracker.completed_full_laps!=3:raise ValueError('three PP laps required')
            source=root/'initialization';source.mkdir()
            checks=dict(three_full_laps=len(events)==3 and all(e['kind']=='full' for e in events),
                one_reset=sim.reset_count==1,no_collision=not sim.collision,
                footprint_valid=sim.footprint_failure is None,physical_replay=replay(rows,vars(sim.params),start_pose)['passed'])
            if not all(checks.values()):raise ValueError('PP source acceptance failed')
            init_report=dict(stage='B1_initialization',passed=True,initialization_laps_accepted=2,
                experiment_id=base.experiment_id,initialization_speed_mps=base.initialization_speed,
                collection_laps=3,acceleration_interface=True,excitation=excitation_settings,
                checks=checks,sample_count=len(rows),
                frenet_length=geometry.curve.L,geometry_id=geometry.geometry_id,config=base.resolved,
                simulator_params=vars(sim.params),vehicle_params=vars(vehicle),start_pose=start_pose.tolist(),
                input_hashes={str(p):sha256(p) for p in (base.path,Path(base.adaptation['geometry_path'])/'centerline.csv',
                    Path('params/simulator_params.yaml'),Path('params/vehicle_params.yaml'),
                    Path(base.adaptation['geometry_path'])/'manifest.json')})
            (source/'session.json').write_text(json.dumps(init_report,indent=2))
            for name,values in [('samples.jsonl',rows),('events.jsonl',events)]:
                with (source/name).open('x') as stream:
                    for value in values:stream.write(json.dumps(value,allow_nan=False)+'\n')
            from .verify_initialization import verify_initialization
            initialization_verification=verify_initialization(source)
            if replay_rows is not None:
                if len(rows)!=len(replay_rows):raise ValueError('A2 replay length mismatch')
                database=LapDatabase.load(source_spec['database'])
                if database.metadata['source_hashes']!={name:sha256(Path(source_spec['directory'])/name)
                        for name in database.metadata['source_hashes']}:
                    raise ValueError('A2 source database mismatch')
                report['A2_replay_exact']=True
            else:
                database=LapDatabase.from_b1(source)
            database.save(root/'initial_database')
            if 'kernel_selection' in settings_file:
                from .input_kernel_controller import ClosedLoopInputKernelModel,InputKernelLMPC
                model=ClosedLoopInputKernelModel(database,geometry,settings_file['kernel_selection']['selection_steering'])
                report['kernel_selection']=settings_file['kernel_selection']
            else:
                model=PaperLocalModel(database,geometry,
                    acceleration_gain_policy=settings.get('acceleration_gain_policy','strict'),
                    identification_gate=settings.get('identification_gate','strict'))
            if 'exploration' in settings_file:
                historical_max=float(np.max(model.states[:,0]))
                cap=min(settings['max_speed_mps'],historical_max*settings_file['exploration']['historical_vx_multiplier'])
                if settings_file['exploration'].get('vx_scope')=='local_80':
                    settings['vx_scope']='local_80'
                else:
                    settings['exploration_vx_limit_mps']=cap
                report['exploration']=dict(historical_max_vx_mps=historical_max,vx_limit_mps=cap,
                    multiplier=1.2,future_nodes=list(range(1,13)),
                    source_database_sha256=sha256(Path(source_spec['database'])/'database.json'),
                    update_policy='only after a new successful lap is included in history; fixed during this first-lap test')
                if 'historical_steering_multiplier' in settings_file['exploration']:
                    settings['historical_steering_multiplier']=settings_file['exploration']['historical_steering_multiplier']
                    settings['steering_scope']=settings_file['exploration']['steering_scope']
                    report['exploration']['steering']=dict(multiplier=1.2,scope=settings['steering_scope'],
                        definition='abs command <= min(.4189, 1.2*max(abs historical command)); local_80 uses each nominal WLS selection; global_history uses all WLS history; u0..u11',
                        historical_max_abs_rad=float(np.max(abs(model.controls[:,0]))))
                if settings.get('vx_scope')=='local_80':
                    report['exploration'].pop('vx_limit_mps',None)
                    report['exploration'].update(vx_scope='local_80',
                        definition='For each stage k, select original WLS 80 at nominal x[k]; bound x[k+1].vx by 1.2*max selected vx and abs u[k].steering by 1.2*max selected abs command, intersect physical limits; no global vx cap.')
            controller=(InputKernelLMPC(database,model,settings) if 'kernel_selection' in settings_file else
                        PaperLocalLMPC(database,model,settings))
            handoff_index=len(rows)-1;lmpc_start_steps=sim.control_steps
            s_origin=3*database.length;state=state.copy();state[4]-=s_origin
            nominal=database.laps[-1].states[:13].copy();nominal[0]=state
            nominal=np.column_stack([nominal,database.laps[-1].physical_states[:13,2]])
            nominal[0,6]=sim.current_state[2]
            controller.set_first_horizon(nominal,database.laps[-1].commands[:12],np.asarray(rows[-1]['command']))
            phase='lmpc';report.update(initialization_verification=initialization_verification,
                source_database=str(root/'initial_database'),initialization_completed_laps=3,
                handoff=dict(raw_sample_id=handoff_index,time=sim.current_time,reset_count=sim.reset_count,
                    physical_state=sim.current_state.tolist(),steering_buffer=sim.dynamics_simulator.steer_buffer.tolist(),
                    previous_input=rows[-1]['command'],local_state=state.tolist(),s_origin=s_origin),
                first_nominal_source_ids=database.laps[-1].sample_ids[:13].tolist(),
                historical_npz_hashes={p.name:sha256(p) for p in (root/'initial_database').glob('*.npz')},
                identification_source_count=len(model.source_ids),stationary_new_experiment=False)
            (root/'handoff.json').write_text(json.dumps(report['handoff'],indent=2))
        target_laps=4 if paper_mode else 1
        while not done and sim.tracker.completed_full_laps < target_laps:
            if stop_progress_m is not None and state is not None and state[4] >= stop_progress_m:
                report['short_target_reached'] = True
                break
            if max_steps is not None and sim.control_steps-lmpc_start_steps >= max_steps:
                report['probe_only'] = True
                break
            begin = perf_counter()
            initial = np.r_[state,sim.current_state[2]]
            controller.steering_buffer=sim.dynamics_simulator.steer_buffer.copy()
            diagnostic = dict(step=sim.control_steps, time=sim.current_time, actual_state=initial.tolist())
            if 'exploration' in settings_file and 'kernel_selection' not in settings_file:
                nominal=(controller.first_horizon[0] if controller.previous is None else
                         np.vstack([controller.previous[0][1:],np.r_[controller.candidate,controller.previous_terminal_input[0]]]))
                support=[]
                for k,z in enumerate(nominal[:-1]):
                    distance=np.sum(((model.states[:,:3]-z[:3])*[.1,1.,1.])**2,axis=1)
                    idx=np.lexsort((model.source_ids,distance))[:80]
                    support.append(dict(horizon_index=k,query_state=z.tolist(),
                        positive_kernel_weights=int(np.sum(distance[idx]<10)),
                        minimum_squared_Q_distance=float(distance[idx[0]]),
                        selected_yaw_rate_range=[float(model.states[idx,2].min()),float(model.states[idx,2].max())]))
                diagnostic['atv_kernel_support']=support
                if 'exploration_vx_limit_mps' in settings:
                    diagnostic['exploration_vx_limit_mps']=settings['exploration_vx_limit_mps']
            try:
                if 'kernel_selection' in settings_file:
                    action,control=controller.plan_with_actuator(initial,sim.current_state,
                        sim.dynamics_simulator.steer_buffer,vars(sim.params))
                else:
                    action, control = controller.plan(initial)
                diagnostic.update(control)
                applied_controls=np.asarray(control['controls']).copy()
                applied_model_next=np.asarray(control['states'][1]).copy()
                if 'command_filter' in settings_file:
                    from .input_kernel_controller import steering_zero_deadzone
                    threshold=settings_file['command_filter']['steering_zero_deadzone_rad']
                    optimized_action=np.asarray(action).copy()
                    action=steering_zero_deadzone(action,threshold)
                    applied_controls[0]=action
                    m=control['prediction_models'][0]
                    applied_model_next=np.asarray(m['A'])@initial+np.asarray(m['B'])@action+np.asarray(m['C'])
                    diagnostic['command_filter']=dict(threshold_rad=threshold,
                        optimized_action=optimized_action.tolist(),applied_action=action.tolist(),
                        steering_zeroed=bool(action[0]!=optimized_action[0]),
                        applied_model_next_state=applied_model_next.tolist(),
                        scope='post-QP first-action deadzone; optimized horizon retained as optimization record')
                permit = gate.check(sim.current_state, sim.dynamics_simulator.steer_buffer,
                                    np.asarray(control['states']), applied_controls)
                diagnostic['planning_gate'] = permit
                if not permit['valid']:
                    raise MPCFailure('scoped planning gate rejected: '+permit['reason'], diagnostic)
                control_time = perf_counter()-begin
                diagnostic['decision_time_s'] = control_time
                if 'command_filter' in settings_file:
                    controller.previous_input=np.asarray(action).copy()
                obs, done = sim.step_acceleration(action)
                state = record(obs, action)
                if state is not None:
                    prediction_error = state-applied_model_next[:6]
                    errors.append(prediction_error)
                    diagnostic['actual_one_step_prediction_error'] = prediction_error.tolist()
                    diagnostic['actual_actuator_endpoint_error_rad']=float(sim.current_state[2]-applied_model_next[6])
                    if 'command_filter' in settings_file:
                        diagnostic['optimized_one_step_prediction_error']=(state-np.asarray(control['states'][1])[:6]).tolist()
                if done:
                    raise MPCFailure('simulation terminated: '+str(sim.terminal_reason), diagnostic)
                if (abs(state[3]) > settings['max_abs_heading_rad'] or abs(state[5]) > settings['max_abs_lateral_m']
                        or np.linalg.norm(state[:2]) > settings['max_speed_mps']+1e-8 or state[0] < -1e-8):
                    raise MPCFailure('actual state limit', diagnostic)
            except MPCFailure as exc:
                diagnostic['failure'] = str(exc)
                if exc.diagnostic is not diagnostic:
                    diagnostic['failure_detail'] = exc.diagnostic
                (root/'failure.json').write_text(json.dumps(diagnostic, indent=2, allow_nan=False))
                raise
            finally:
                diagnostic['control_time_s'] = perf_counter()-begin
                timings.append(diagnostic['control_time_s'])
                control_stream.write(json.dumps(diagnostic, allow_nan=False)+'\n'); control_stream.flush()
            if sim.control_steps % 50 == 0:
                print('step', sim.control_steps, 's', round(state[4], 3), 'v', round(state[0], 3),
                      'control_s', round(timings[-1], 3), flush=True)
        if sim.tracker.completed_full_laps == target_laps:
            verification = replay(rows, vars(sim.params), start_pose)
            checks = dict(full_lap=len(events)==target_laps and all(e['kind']=='full' for e in events), one_reset=sim.reset_count==1,
                          no_collision=not sim.collision, footprint_valid=sim.footprint_failure is None,
                          frenet_valid=all(r['frenet']['valid'] for r in rows), physical_replay=verification['passed'])
            report.update(checks=checks, replay=verification, passed=all(checks.values()), lap_time_s=events[-1]['duration'])
            # Cost known only now; save a new observed successful lap separately.
            lap_rows=rows[handoff_index:]
            states = np.asarray([r['paper_state'] for r in lap_rows]);states[:,4]-=s_origin
            terminal_index = events[-1]['row_index']-handoff_index
            np.savez_compressed(root/('experiment_lap_001.npz' if report['passed'] else 'unaccepted_lap_001.npz'), states=states,
                                physical_states=np.asarray([r['physical_state'] for r in lap_rows]),
                                commands=np.asarray([r['command'] for r in lap_rows[1:]]),
                                times=np.asarray([r['time'] for r in lap_rows]), costs=terminal_index-np.arange(len(states)),
                                input_columns=np.array(['steering_command', 'acceleration_command']))
    except Exception as exc:
        failure = type(exc).__name__+': '+str(exc)
        report['error'] = failure
        print(failure, flush=True)
    finally:
        sample_stream.close(); control_stream.close(); event_stream.close()
        report.update(sample_count=len(rows), control_steps=0 if sim is None else sim.control_steps,
                      completed_full_laps=0 if sim is None else sim.tracker.completed_full_laps,
                      completed_lmpc_laps=0 if sim is None else max(0,sim.tracker.completed_full_laps-(3 if paper_mode else 0)),
                      reset_count=0 if sim is None else sim.reset_count,
                      error=failure, realtime_100ms_passed=bool(timings) and max(timings) <= .1,
                      control_time_summary=None if not timings else dict(mean_s=float(np.mean(timings)),
                          p95_s=float(np.percentile(timings, 95)), max_s=float(max(timings))),
                      prediction_error_rmse=None if not errors else np.sqrt(np.mean(np.asarray(errors)**2, axis=0)).tolist())
        (root/'session.json').write_text(json.dumps(report, indent=2, allow_nan=False))
        print(json.dumps({k:report.get(k) for k in ['passed', 'error', 'control_steps', 'lap_time_s', 'realtime_100ms_passed']}, indent=2), flush=True)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', default='params/research/lmpc_actuator7.yaml')
    parser.add_argument('--output-dir')
    parser.add_argument('--max-steps', type=int, help='diagnostic probe, never full B3 acceptance')
    parser.add_argument('--stop-progress-m', type=float, help='preregistered B2.5 short target, not a full lap')
    args = parser.parse_args()
    if args.max_steps is not None and args.max_steps < 1:
        parser.error('--max-steps must be positive')
    if args.stop_progress_m is not None and (not np.isfinite(args.stop_progress_m) or args.stop_progress_m <= 0):
        parser.error('--stop-progress-m must be finite and positive')
    report = run(args.config, args.output_dir, args.max_steps, args.stop_progress_m)
    if report.get('error') or (not report['passed'] and not report.get('probe_only')):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
