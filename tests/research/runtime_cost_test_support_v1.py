"""Synthetic pose/time math; no saved archive or native rollout required."""
from test_robust_adaptive_paths import make_straight
from test_offline_rollout_frenet_v1 import data
from f1tenth_benchmarks.research.core.geometry.tracker import TrackingOptions
from f1tenth_benchmarks.research.overtaking import realtime_cost_inputs_v1 as module

POLICY=dict(schema_version=1,policy_id='synthetic_runtime_components_no_weights',experimental=True,
    components=dict(risk=dict(mode='exposure_integral',clearance_ref_m=.2,clearance_scale_m=.1,
        time_scale_s=2.,valid_clearance_m=[-10.,100.],valid_horizon_s=[.01,2.])))
PROJECTION=TrackingOptions(max_dt_s=.2,max_speed_mps=4.,max_acceleration_mps2=8.,position_slack_m=.05,progress_slack_m=.05)

def case(g=None):
    g=g or make_straight();times=[0.,.05,.1]
    es,e=data(g,[.5,.55,.6],times);ls,l=data(g,[1.5,1.525,1.55],times)
    for state in (e,l):state['control_step']=0
    plan=dict(plan_id=0,source_control_step=0,source_sim_time_s=10.,source_pose=e['pose'],geometry_id=g.geometry_id,frame_id='map',generator_settings=dict(lookahead_m=3.),paths=[dict(candidate_id=1,path_type='state_matched_trajectory',points=[dict(x=.5,y=0.),dict(x=3.5,y=0.)])])
    samples=[dict(t_s=t,ego_pose=a['pose'],lead_pose=b['pose'],signed_min_pair_clearance_m=c) for t,a,b,c in zip(times,es,ls,[.3,.15,.1])]
    option=dict(candidate_id=1,target_speed_mps=1.,evaluation_id='e0',prediction_group_id='g0',source_path_type='state_matched_trajectory',reference_only=False,rollout_trajectory_type='modeled_ego_rollout',requested_sample_times_s=times,requested_horizon_s=.1,samples=samples,last_integrated_t_s=.1,evaluation_complete=True,native_safe=True,native_reason=None,tracking_path_id=0)
    record=dict(source=dict(source_control_step=0,source_sim_time_s=10.,ego_state=e,lead_state=l),metadata=dict(geometry_id=g.geometry_id,frame_id='map',model='three_discs'),plan_snapshot=plan,options=[option],lead_predictions=[dict(prediction_group_id='g0',requested_sample_times_s=times,poses=[p['pose'] for p in ls])],tracking_paths=[dict(tracking_path_id=0,xy=[[.5,0.],[3.5,0.]],l2s=[9.])])
    ctx=module.RuntimeCostContext(g,risk_policy=POLICY,tracking_options=PROJECTION)
    return record,ctx
