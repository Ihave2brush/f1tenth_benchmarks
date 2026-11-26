"""Optional saved-ESP golden replay; no simulator, trust upgrade or generation."""
import argparse
import json
import math
from pathlib import Path
import sys
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from f1tenth_benchmarks.research.core.geometry import load_geometry
from f1tenth_benchmarks.research.core.geometry.tracker import TrackingOptions
from f1tenth_benchmarks.research.overtaking.rollout_diagnostics_v1 import validate_record
from f1tenth_benchmarks.research.overtaking.realtime_cost_inputs_v1 import RuntimeCostContext,prepare_runtime_length_cache
from f1tenth_benchmarks.research.overtaking.realtime_cost_core_v1 import prepare_projection_bundle,evaluate_runtime_cost_core


def source_frame(record,frames):
    """Terminal snapshots are allowed; join by the actual ego source clock."""
    src=record['source'];clock=(src['source_control_step'],src['source_sim_time_s'])
    matches=[f for f in frames if (f.get('ego_state',{}).get('control_step'),f.get('ego_state',{}).get('sim_time_s'))==clock]
    if len(matches)!=1:raise ValueError('missing_or_ambiguous_source_frame')
    frame=matches[0]
    if record['plan_snapshot']!=frame.get('plan'):raise ValueError('frame_plan_mismatch')
    for role in ('ego','lead'):
        state=frame.get(role+'_state',{});source=src[role+'_state']
        if (state.get('control_step'),state.get('sim_time_s'),state.get('pose'))!=(clock[0],clock[1],source['pose']):
            raise ValueError('frame_state_mismatch:'+role)
    return frame


def main():
    p=argparse.ArgumentParser();p.add_argument('--cases-root',type=Path,required=True)
    p.add_argument('--geometry-dir',type=Path,required=True);p.add_argument('--map-yaml',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    if a.output.exists():raise FileExistsError(a.output)
    geometry=load_geometry(str(a.geometry_dir),str(a.map_yaml),inspect=True);results=[];verified=0;maximum=0.
    for name in ('smoke','follow','slow','close'):
        folder=a.cases_root/name
        read=lambda path:json.loads((folder/path).read_text())
        groups=read('group_audit.json');frames=read('frames.json');golden=read('summary.json');expected=read('frenet_projection.json')
        records=[json.loads(line) for line in (folder/'rollout_diagnostics.jsonl').read_text().splitlines() if line.strip()]
        if not len(records)==len(groups)==len(expected):raise ValueError('source_file_count_mismatch')
        for record,group,old_projection in zip(records,groups,expected):
            validate_record(record)
            source_frame(record,frames)
            policy=group['options'][0]['cost_components']['policy']
            context=RuntimeCostContext(geometry,risk_policy=policy,tracking_options=TrackingOptions(**old_projection['tracking_options']))
            lengths={r['candidate_id']:r['frenet_integral_length_m'] for r in group['path_length_results'][0]['result']['paths'] if r['frenet_integral_length_m'] is not None}
            cache=prepare_runtime_length_cache(record['plan_snapshot'],geometry.geometry_id,lengths,method='cached_frenet_integral')
            projection=prepare_projection_bundle(record,context)
            new=evaluate_runtime_cost_core(record,context,length_cache=cache,projection_bundle=projection)
            source=record['source']
            old=[g for g in golden if (g['source_control_step'],g['source_sim_time_s'])==(source['source_control_step'],source['source_sim_time_s'])]
            if len(old)!=len(new['options']):raise ValueError('option_count_mismatch')
            for row,gold in zip(new['options'],old):
                for key in ('candidate_id','evaluation_id','prediction_group_id','target_speed_mps','native_safe','native_reason','source_control_step','source_sim_time_s'):
                    if row[key]!=gold[key]:raise ValueError('identity_or_native_status:'+key)
                for key,gkey in [('length_m','J_length_m'),('risk_m2_s','J_risk_m2_s'),('progress_m_s','J_progress_m_s'),('partial_risk_m2_s','partial_risk_m2_s'),('partial_progress_m_s','partial_progress_m_s')]:
                    x,y=row[key],gold[gkey]
                    if (x is None)!=(y is None):raise ValueError('null_mismatch:'+key)
                    if x is not None:
                        if type(x) not in (int,float) or type(y) not in (int,float) or not math.isfinite(x) or not math.isfinite(y):raise ValueError('nonfinite_cost:'+key)
                        maximum=max(maximum,abs(x-y))
                        if abs(x-y)>1e-9:raise ValueError('cost_mismatch:'+key)
                if row['planning_allowed'] or row['formal_comparison_qualified']:raise ValueError('research_flag_mismatch')
                verified+=1
            results.append(dict(case=name,source_step=source['source_control_step'],result=new))
    out=dict(options_verified=verified,maximum_cost_error=maximum,tolerance=1e-9,results=results,
        source_binding='saved_geometry_lengths_are_supplied_values_not_new_producer_authentication',
        inspection_only=True,planning_allowed=False,formal_comparison_qualified=False)
    with a.output.open('x') as f:json.dump(out,f,indent=2,allow_nan=False)
    print(json.dumps(dict(options_verified=verified,maximum_cost_error=maximum)))

if __name__=='__main__':main()
