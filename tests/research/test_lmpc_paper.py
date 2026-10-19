"""Independent oracles for direct paper regression, input order and six-state QP."""
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np
from f1tenth_benchmarks.research.lmpc.paper_model import (PaperLocalModel, PaperIdentificationError,
    build_atv_horizon, rollout_atv)
from f1tenth_benchmarks.research.lmpc.paper_controller import PaperLocalLMPC, steering_exploration_bounds
from f1tenth_benchmarks.research.lmpc.controller import solve_qp, solve_finish_qp, MPCFailure
from f1tenth_benchmarks.research.lmpc.safe_set import LocalSafeSet
from f1tenth_benchmarks.research.lmpc.run_lmpc import load_b3
from f1tenth_benchmarks.research.lmpc.planning_scope import ScopedPlanningGate


class PaperTests(unittest.TestCase):
    def model(self):
        rng=np.random.RandomState(5);m=object.__new__(PaperLocalModel)
        m.states=rng.normal(0,.1,(120,6));m.states[:,0]+=1.2
        m.controls=rng.normal(0,.1,(120,2));m.source_ids=np.arange(120);m.dt=.1
        theta=np.array([[.9,.1,.02,.1,.01],[.02,.7,.1,.2,-.01],[.01,.1,.8,.3,.02]])
        m.targets=m.states[:,:3]@theta[:,:3].T+m.controls[:,[1,0,0]]*theta[:,3]+theta[:,4]
        m.geometry=SimpleNamespace(reference=lambda s:{'kappa':0.})
        return m,theta

    def test_recovers_known_five_coefficients_without_prior_or_ridge(self):
        m,truth=self.model();z=m.states[8];u=m.controls[8]
        fit,diag=m.identify(z,u)
        np.testing.assert_allclose(fit,truth,atol=1e-12,rtol=0)
        self.assertEqual(len(diag['source_ids']),80)
        self.assertEqual(diag['state_dimension'],6)
        np.testing.assert_allclose(m.evaluate(z,u,fit)[:3],m.targets[8],atol=1e-12)

    def test_input_column_order_and_linearization_value(self):
        m,truth=self.model();z=m.states[8];u=m.controls[8]
        a,b,c,diag=m.linearize(z,u)
        self.assertEqual(b[0,0],0.)
        np.testing.assert_allclose(b[:,1],[.1,0,0,0,0,0],atol=1e-12)
        np.testing.assert_allclose(b[:,0],[0,.2,.3,0,0,0],atol=1e-12)
        np.testing.assert_allclose(a@z+b@u+c,m.evaluate(z,u),atol=1e-12)

    def test_constant_acceleration_is_rejected_despite_eighty_distinct_samples(self):
        m,_=self.model();m.controls[:,1]=0.
        with self.assertRaises(PaperIdentificationError) as context:
            m.identify(m.states[8])
        diagnostic=context.exception.diagnostic
        self.assertEqual(len(set(diagnostic['source_ids'])),80)
        self.assertIn('vx',diagnostic['unidentified_input_channels'])
        vx=diagnostic['paper_regressors'][0]
        self.assertEqual(vx['rank'],vx['rank_without_input'])

    def test_varying_acceleration_collinear_with_state_is_also_rejected(self):
        m,_=self.model();m.controls[:,1]=m.states[:,0]
        self.assertGreater(np.ptp(m.controls[:,1]),.01)
        with self.assertRaises(PaperIdentificationError) as context:
            m.identify(m.states[8])
        self.assertIn('vx',context.exception.diagnostic['unidentified_input_channels'])

    def test_missing_state_excitation_is_distinct_from_identifiable_input(self):
        m,truth=self.model();m.states[:,1:3]=0.
        m.targets=m.states[:,:3]@truth[:,:3].T+m.controls[:,[1,0,0]]*truth[:,3]+truth[:,4]
        fit,diag=m.identify(m.states[8])
        self.assertTrue(all(d['input_identifiable'] for d in diag['paper_regressors']))
        self.assertTrue(all(not d['fully_identified'] for d in diag['paper_regressors']))
        np.testing.assert_allclose(fit[:,3],truth[:,3],atol=1e-12)

    def test_unidentified_gain_initialization_refits_constant_and_collinear_input(self):
        for command in ('zero', 'constant', 'collinear'):
            with self.subTest(command=command):
                m,truth=self.model()
                m.acceleration_gain_policy='dt_cos_beta_when_unidentified'
                m.controls[:,1]=0. if command=='zero' else (.4 if command=='constant' else m.states[:,0])
                m.targets=m.states[:,:3]@truth[:,:3].T+m.controls[:,[1,0,0]]*truth[:,3]+truth[:,4]
                z=m.states[8].copy();z[:2]=[1.2,.9];z[3]=1.1
                fit,diag=m.identify(z)
                self.assertAlmostEqual(fit[0,3],.08)
                vx=diag['paper_regressors'][0]
                self.assertFalse(vx['input_identifiable'])
                self.assertTrue(vx['input_ready'])
                self.assertEqual(vx['input_effect_source'],'dt_cos_beta_initialization')
                self.assertLess(vx['weighted_rmse'],1e-12)
                a,b,c,_=m.linearize(z,np.array([.03,.2]))
                self.assertAlmostEqual(b[0,1],.08)
                np.testing.assert_allclose(a@z+b@np.array([.03,.2])+c,
                    m.evaluate(z,np.array([.03,.2])),atol=1e-12)

    def test_identifiable_gain_is_learned_instead_of_forced_to_initialization(self):
        m,truth=self.model();m.acceleration_gain_policy='dt_cos_beta_when_unidentified'
        truth[0,3]=.07
        m.targets=m.states[:,:3]@truth[:,:3].T+m.controls[:,[1,0,0]]*truth[:,3]+truth[:,4]
        fit,diag=m.identify(m.states[8])
        np.testing.assert_allclose(fit,truth,atol=1e-12)
        self.assertEqual(diag['paper_regressors'][0]['input_effect_source'],'observed_regression')

    def test_stationary_seed_uses_zero_beta_and_lateral_guard_remains(self):
        m,truth=self.model();m.acceleration_gain_policy='dt_cos_beta_when_unidentified'
        m.controls[:,1]=0.
        z=m.states[8].copy();z[:2]=0.
        fit,diag=m.identify(z)
        self.assertAlmostEqual(fit[0,3],m.dt)
        self.assertEqual(diag['paper_regressors'][0]['initialization_beta_rad'],0.)
        m.controls[:,0]=0.
        with self.assertRaises(PaperIdentificationError) as context:m.identify(z)
        self.assertEqual(context.exception.diagnostic['unready_input_channels'],['vy','r'])

    def test_paper_configuration_enables_approved_gain_policy(self):
        settings=load_b3('params/research/lmpc_paper.yaml')['settings']
        self.assertEqual(settings['acceleration_gain_policy'],'dt_cos_beta_when_unidentified')
        self.assertEqual(settings['identification_gate'],'report_only')

    def test_paper_rank_diagnostic_does_not_block_model_construction(self):
        m,_=self.model();m.controls[:]=0.
        m.identification_gate='report_only'
        fit,diag=m.identify(m.states[8])
        self.assertEqual(diag['unidentified_input_channels'],['vx','vy','r'])
        self.assertEqual(fit.shape,(3,5))
        self.assertTrue(np.isfinite(fit).all())

    def test_moving_first_seed_preserves_previous_control_and_terminal_candidate(self):
        m,_=self.model();c=PaperLocalLMPC(SimpleNamespace(
            laps=[SimpleNamespace(states=m.states[:14],shifted_states=lambda cycle:m.states[:14])]),m,{})
        nominal=m.states[:13];inputs=m.controls[:12];old=np.array([.03,.01])
        c.set_first_horizon(nominal,inputs,old)
        np.testing.assert_array_equal(c.candidate,nominal[-1])
        np.testing.assert_array_equal(c.previous_input,old)
        with patch('f1tenth_benchmarks.research.lmpc.paper_controller.select_local_safe_set',return_value=object()), \
             patch('f1tenth_benchmarks.research.lmpc.paper_controller.solve_qp',side_effect=MPCFailure('stop',{})) as solver:
            with self.assertRaises(MPCFailure):c.plan(nominal[0])
        np.testing.assert_array_equal(solver.call_args.args[1],nominal)
        np.testing.assert_array_equal(solver.call_args.args[2],inputs)
        np.testing.assert_array_equal(solver.call_args.args[5],old)

    def test_atv_builds_all_models_at_nominal_then_rolls_without_identification(self):
        nominal=np.zeros((13,6));nominal[:,0]=np.arange(13)
        inputs=np.zeros((12,2));calls=[]
        def linearize(z,u):
            calls.append(z.copy())
            a=np.eye(6);a[0,0]=.5
            b=np.zeros((6,2));b[0,1]=.1
            c=np.zeros(6);c[0]=z[0]
            return a,b,c,{}
        models=build_atv_horizon(SimpleNamespace(linearize=linearize),nominal,inputs)
        np.testing.assert_array_equal(calls,nominal[:-1])
        initial=np.zeros(6);initial[0]=100.
        actual_inputs=inputs.copy();actual_inputs[:,1]=2.
        predicted=rollout_atv(initial,actual_inputs,models)
        oracle=[];value=100.
        for k in range(12):
            value=.5*value+.2+k;oracle.append(value)
        np.testing.assert_allclose(predicted[:,0],oracle)
        self.assertEqual(len(calls),12)

    def test_atv_nominal_value_matches_original_model_at_each_stage(self):
        m,_=self.model();nominal=m.states[:13];inputs=m.controls[:12]
        models=build_atv_horizon(m,nominal,inputs)
        for k,(a,b,c,_) in enumerate(models):
            np.testing.assert_allclose(a@nominal[k]+b@inputs[k]+c,
                m.evaluate(nominal[k],inputs[k]),atol=1e-12)

    def test_controller_never_attempts_qp_when_input_effect_is_unknown(self):
        m,_=self.model();m.controls[:,1]=0.
        initial=m.states[8].copy()
        c=object.__new__(PaperLocalLMPC)
        c.database=SimpleNamespace(laps=[SimpleNamespace(states=np.tile(initial,(13,1)),commands=np.zeros((12,2)))])
        c.model=m;c.settings={};c.candidate=initial;c.previous=None;c.previous_input=np.zeros(2)
        with patch('f1tenth_benchmarks.research.lmpc.paper_controller.select_local_safe_set',return_value=object()), \
             patch('f1tenth_benchmarks.research.lmpc.paper_controller.solve_qp') as solve:
            with self.assertRaises(MPCFailure) as context:c.plan(initial)
        solve.assert_not_called()
        self.assertEqual(context.exception.diagnostic['qp_attempts'],0)
        self.assertEqual(context.exception.diagnostic['failure_kind'],'unidentifiable_paper_input')

    def test_diagnostic_model_cannot_be_installed_in_controller(self):
        m,_=self.model();m.diagnostic_only=True
        with self.assertRaisesRegex(ValueError,'diagnostic-only'):
            PaperLocalLMPC(None,m,{})

    def test_six_state_qp_enforces_shared_terminal_and_model(self):
        initial=np.array([1.2,0,0,0,0,0.]);a=np.eye(6);a[4,0]=.1
        b=np.zeros((6,2));b[0,1]=.1
        nominal=np.tile(initial,(13,1));nominal[:,4]=np.arange(13)*.12
        terminal_states=np.zeros((6,40));terminal_states[0]=1.2;terminal_states[4]=np.linspace(.5,3,40)
        terminal=LocalSafeSet(terminal_states,terminal_states.copy(),(10-terminal_states[4])*10,
                              (10-terminal_states[4])*10-1,tuple(),'test',.1)
        settings=load_b3('params/research/lmpc_paper.yaml')['settings']
        models=[(a,b,np.zeros(6),{})]*12
        states,inputs,lam,info=solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings)
        np.testing.assert_allclose(states[-1],terminal.states@lam,atol=1e-5)
        for k in range(12):np.testing.assert_allclose(states[k+1],a@states[k]+b@inputs[k],atol=1e-5)
        self.assertEqual(info['status'],'solved')
        self.assertEqual(info['state_constraint_indices'],list(range(12)))

    def test_A3_vx_bound_covers_all_future_nodes_including_terminal(self):
        settings=load_b3('params/research/lmpc_paper_a3.yaml')['settings'].copy()
        self.assertEqual(settings['max_acceleration_mps2'],9.51)
        settings['exploration_vx_limit_mps']=1.5
        initial=np.array([1.2,0,0,0,0,0.]);a=np.eye(6);a[4,0]=.1
        b=np.zeros((6,2));b[0,1]=.1
        nominal=np.tile(initial,(13,1));nominal[:,4]=np.arange(13)*.12
        values=np.zeros((6,40));values[0]=1.2;values[4]=np.linspace(.5,3,40)
        terminal=LocalSafeSet(values,values.copy(),(10-values[4])*100,
            (10-values[4])*100-1,tuple(),'test',.1)
        models=[(a,b,np.zeros(6),{})]*12
        states,inputs,_,_=solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings)
        self.assertLessEqual(np.max(states[1:,0]),1.5+1e-5)
        self.assertGreater(np.max(states[1:,0]),1.49)
        self.assertLessEqual(np.max(abs(inputs[:,1])),9.51+1e-5)
        values[0]=1.7
        with self.assertRaises(MPCFailure):
            solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings)

    def test_steering_scope_uses_selected_commands_and_physical_intersection(self):
        model=SimpleNamespace(source_ids=np.arange(120),controls=np.zeros((120,2)))
        model.controls[:80,0]=-.02;model.controls[80:,0]=.4
        models=[(None,None,None,dict(source_ids=list(range(80))))]*12
        local,_=steering_exploration_bounds(model,models,1.2,'local_80')
        global_bounds,_=steering_exploration_bounds(model,models,1.2,'global_history')
        np.testing.assert_allclose(local,np.tile([-.024,.024],(12,1)))
        np.testing.assert_allclose(global_bounds,np.tile([-.4189,.4189],(12,1)))

    def test_A4_local_speed_bounds_cover_each_future_node_and_terminal(self):
        config=load_b3('params/research/lmpc_paper_a4.yaml')
        self.assertEqual(config['experiment_id'],'A4')
        self.assertEqual(config['exploration']['vx_scope'],'local_80')
        settings=config['settings'].copy();caps=np.linspace(1.25,1.5,12)
        settings['exploration_vx_limits_mps']=caps.tolist()
        initial=np.array([1.2,0,0,0,0,0.]);a=np.eye(6);a[4,0]=.1
        b=np.zeros((6,2));b[0,1]=.1
        nominal=np.tile(initial,(13,1));nominal[:,4]=np.arange(13)*.12
        values=np.zeros((6,40));values[0]=1.2;values[4]=np.linspace(.5,3,40)
        terminal=LocalSafeSet(values,values.copy(),(10-values[4])*100,(10-values[4])*100-1,tuple(),'test',.1)
        models=[(a,b,np.zeros(6),{})]*12
        states,_,_,_=solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings)
        self.assertTrue(np.all(states[1:,0]<=caps+1e-5))
        self.assertGreater(states[10,0],1.45)
        settings['exploration_vx_limits_mps'][-1]=1.1
        with self.assertRaises(MPCFailure):
            solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings)

    def test_steering_exploration_constrains_each_input_and_can_reject_qp(self):
        settings=load_b3('params/research/lmpc_paper_a3.yaml')['settings'].copy()
        caps=np.arange(1,13)*.01
        settings['exploration_steering_bounds_rad']=np.column_stack([-caps,caps]).tolist()
        initial=np.array([1.2,0,0,0,0,0.]);a=np.eye(6);a[4,0]=.1
        b=np.zeros((6,2));b[5,0]=.1
        nominal=np.tile(initial,(13,1));nominal[:,4]=np.arange(13)*.12
        values=np.zeros((6,40));values[0]=1.2;values[4]=np.linspace(.5,3,40);values[5]=.1*caps.sum()
        terminal=LocalSafeSet(values,values.copy(),np.ones(40)*10,np.ones(40)*9,tuple(),'test',.1)
        models=[(a,b,np.zeros(6),{})]*12
        _,inputs,_,_=solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings)
        np.testing.assert_allclose(inputs[:,0],caps,atol=1e-5,rtol=0)
        values[5]+=.001
        with self.assertRaises(MPCFailure):
            solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings)

    def test_finish_indicator_is_optimized_and_suffix_cost_is_not_negative_time(self):
        initial=np.array([1.2,0,0,0,0,0.]);a=np.eye(6);a[4,0]=.1
        b=np.zeros((6,2));b[0,1]=.1
        nominal=np.tile(initial,(13,1));nominal[:,4]=np.arange(13)*.12
        terminal_states=np.zeros((6,40));terminal_states[0]=1.2
        terminal_states[4]=np.linspace(1.05,2.5,40)
        terminal=LocalSafeSet(terminal_states,terminal_states.copy(),
            np.linspace(-1,-15,40),np.linspace(-2,-16,40),tuple(),'test',.1)
        settings=load_b3('params/research/lmpc_paper.yaml')['settings']
        models=[(a,b,np.zeros(6),{})]*12
        states,inputs,lam,info=solve_finish_qp(initial,nominal,np.zeros((12,2)),models,
            terminal,np.zeros(2),settings,1.)
        self.assertGreater(info['finish_qp_attempts'],1)
        self.assertEqual(info['stage_cost'],int(np.sum(states[:-1,4]<1.)))
        self.assertEqual(info['terminal_cost'],0.)
        rate=np.sum(np.diff(np.vstack([np.zeros(2),inputs]),axis=0)**2*settings['input_rate_weights'])
        self.assertAlmostEqual(info['objective_with_stage_and_rate'],info['stage_cost']+rate,places=6)
        self.assertLess(info['objective_with_stage_and_rate'],9.)
        np.testing.assert_allclose(states[-1],terminal.states@lam,atol=1e-5)

    def test_state_X_includes_measured_x0_and_excludes_terminal_xN(self):
        initial=np.array([1.2,0,0,0,0,0.]);a=np.eye(6);a[4,0]=.1
        b=np.zeros((6,2));b[3,0]=1.
        nominal=np.tile(initial,(13,1));nominal[:,4]=np.arange(13)*.12
        states=np.zeros((6,40));states[0]=1.2;states[3]=.6;states[4]=np.linspace(.5,3,40)
        terminal=LocalSafeSet(states,states.copy(),np.ones(40),np.zeros(40),tuple(),'test',.1)
        settings=load_b3('params/research/lmpc_paper.yaml')['settings']
        models=[(a,b,np.zeros(6),{})]*12
        predicted,_,_,_=solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings)
        self.assertAlmostEqual(predicted[-1,3],.6,places=6)
        self.assertLessEqual(np.max(abs(predicted[:-1,3])),.5+1e-5)
        invalid=initial.copy();invalid[3]=.6
        with self.assertRaises(MPCFailure):
            solve_qp(invalid,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings)

    def test_shadow_first_action_does_not_execute_hypothetical_future_inputs(self):
        physical=np.zeros(7);physical[3]=1.2
        geometry=SimpleNamespace(planning_allowed=False,geometry_id='test')
        validator=SimpleNamespace(check=lambda *a:dict(valid=True),
            footprint=SimpleNamespace(check=lambda *a:dict(valid=True)))
        def observation(data):
            state=np.array([1.2,0,0,0,0,0.])
            if data['simulation_time']>=.2:state[5]=.4
            return state,{}
        with patch('f1tenth_benchmarks.research.lmpc.planning_scope.TracedDynamics') as plant, \
             patch('f1tenth_benchmarks.research.lmpc.planning_scope.StateObserver') as observer:
            plant.return_value.update_pose.return_value=physical
            observer.return_value.observe.side_effect=observation
            settings=dict(max_speed_mps=8,max_abs_heading_rad=.5,max_abs_lateral_m=.25,shadow_mode='first_action')
            gate=ScopedPlanningGate(geometry,validator,dict(timestep=.01),settings)
            result=gate.check(physical,np.zeros(2),np.zeros((13,6)),np.zeros((12,2)))
            self.assertTrue(result['valid']);self.assertEqual(result['shadow_physics_endpoints'],10)
            settings['shadow_mode']='full_horizon'
            result=gate.check(physical,np.zeros(2),np.zeros((13,6)),np.zeros((12,2)))
            self.assertFalse(result['valid']);self.assertEqual(result['horizon_step'],1)

    def test_unknown_feasibility_initializer_status_is_not_infeasibility(self):
        initial=np.array([1.2,0,0,0,0,0.]);a=np.eye(6);a[4,0]=.1
        b=np.zeros((6,2));b[0,1]=.1
        nominal=np.tile(initial,(13,1));nominal[:,4]=np.arange(13)*.12
        states=np.zeros((6,40));states[0]=1.2;states[4]=np.linspace(.5,3,40)
        terminal=LocalSafeSet(states,states.copy(),np.ones(40),np.zeros(40),tuple(),'test',.1)
        settings=load_b3('params/research/lmpc_paper.yaml')['settings']
        with patch('f1tenth_benchmarks.research.lmpc.controller.linprog',
                return_value=SimpleNamespace(success=False,status=4,message='unknown',x=None)):
            predicted,_,lam,info=solve_qp(initial,nominal,np.zeros((12,2)),
                [(a,b,np.zeros(6),{})]*12,terminal,np.zeros(2),settings)
        self.assertEqual(info['status'],'solved')
        self.assertEqual(info['equality_reference'],'affine least squares')
        np.testing.assert_allclose(predicted[-1],terminal.states@lam,atol=1e-5)

    def test_speed_facets_are_added_when_relaxed_optimum_exceeds_speed_cap(self):
        initial=np.array([7.,0,0,0,0,0.]);a=np.eye(6);a[4,0]=.1
        b=np.zeros((6,2));b[0,1]=.1
        nominal=np.tile(initial,(13,1));nominal[:,4]=np.arange(13)*.7
        states=np.zeros((6,40));states[0]=7.;states[4]=np.linspace(8.3,12,40)
        costs=(20-states[4])*20
        terminal=LocalSafeSet(states,states.copy(),costs,costs-1,tuple(),'test',.1)
        settings=load_b3('params/research/lmpc_paper.yaml')['settings'];workspace={}
        models=[(a,b,np.zeros(6),{})]*12
        first=solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings,
            [1]*12,20.,_workspace=workspace)
        predicted,_,lam,info=first
        self.assertGreater(info['generated_speed_facets'],0)
        self.assertGreater(info['constraint_generation_rounds'],0)
        self.assertLessEqual(np.max(np.linalg.norm(predicted[:-1,:2],axis=1)),8.+1e-5)
        np.testing.assert_allclose(predicted[-1],terminal.states@lam,atol=1e-5)
        solver=workspace['solver']
        second=solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings,
            [1]*11+[-1],20.,_workspace=workspace)
        self.assertIs(workspace['solver'],solver)
        self.assertTrue(second[3]['branch_workspace_reused'])
        self.assertLessEqual(second[3]['constraint_violation'],1e-5)
        np.testing.assert_allclose(second[1],first[1],atol=1e-4)

    def test_branch_workspace_rejects_changed_objective(self):
        initial=np.array([1.2,0,0,0,0,0.]);a=np.eye(6);a[4,0]=.1
        b=np.zeros((6,2));b[0,1]=.1
        nominal=np.tile(initial,(13,1));nominal[:,4]=np.arange(13)*.12
        states=np.zeros((6,40));states[0]=1.2;states[4]=np.linspace(.5,3,40)
        terminal=LocalSafeSet(states,states.copy(),np.ones(40),np.zeros(40),tuple(),'test',.1)
        settings=load_b3('params/research/lmpc_paper.yaml')['settings'];workspace={}
        models=[(a,b,np.zeros(6),{})]*12
        solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings,
            [1]*12,10.,_workspace=workspace)
        with self.assertRaisesRegex(ValueError,'identical QP'):
            solve_qp(initial,nominal,np.zeros((12,2)),models,terminal,np.array([0.,.1]),settings,
                [1]*12,10.,_workspace=workspace)
        changed=initial.copy();changed[0]+=.01
        with self.assertRaisesRegex(ValueError,'identical QP equalities'):
            solve_qp(changed,nominal,np.zeros((12,2)),models,terminal,np.zeros(2),settings,
                [1]*12,10.,_workspace=workspace)


if __name__=='__main__':unittest.main()
