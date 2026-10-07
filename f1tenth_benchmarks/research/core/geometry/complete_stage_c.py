"""Exclusive ESP Stage C sidecar, diagnosis and timestamped tracking replay."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import platform
import time

import numpy as np
import scipy

from .frenet import load_geometry
from .map_raster import sha256
from .domain import build_domain,save_domain,load_domain,json_bytes
from .tracker import FrenetTracker,TrackingOptions
from .check_frenet import check


def diagnose_source(geometry,s,d):
    ref=geometry.reference(s);t=np.asarray(ref['tangent'])
    xy=np.asarray(ref['xy'])+d*np.array([-t[1],t[0]])
    result=geometry.to_cartesian(s,d)
    candidates=geometry.projection_candidates(xy)['candidates']
    source_j=1-ref['kappa']*d
    cause=('source_J_margin' if source_j<.2 else
           'nearest_projection_J_margin' if candidates and candidates[0]['jacobian']<.2 else
           result['reason'])
    return dict(s_m=s,d_m=d,xy=xy.tolist(),occupancy=geometry.raster.classify(xy),
                source_kappa=ref['kappa'],source_J=source_j,valid=result['valid'],
                reason=result['reason'],cause=cause,candidates=candidates)


def replay(geometry,records,lateral_noise_m=0.):
    tracker=FrenetTracker(geometry,source_reference='sim_pose',offset_calibrated=True)
    failures=[];elapsed=[];jacobians=[];last=None;minimum=None;inverse_errors=[]
    for i,row in enumerate(records):
        state=row['physical_state'];point=np.asarray(state[:2]);yaw=state[4];timestamp=row['simulation_time']
        if lateral_noise_m:
            ref=geometry.to_frenet(point,yaw)
            if not ref['valid']:
                raise ValueError('nominal replay pose invalid before perturbation')
            tangent=geometry.reference(ref['s_wrapped'])['tangent']
            point=point+lateral_noise_m*np.sin(i*.17)*np.array([-tangent[1],tangent[0]])
        start=time.perf_counter()
        result=tracker.update(point,yaw,timestamp,now=timestamp,geometry_id=geometry.geometry_id)
        elapsed.append((time.perf_counter()-start)*1000)
        if not result['valid']:
            failures.append(dict(row=i,reason=result['reason']))
            # Record a failed stream; do not auto-reset and conceal it.
            continue
        last=result;jacobians.append(result['jacobian'])
        if minimum is None or result['jacobian']<minimum['jacobian']:
            minimum=dict(result,row=i)
        ref=geometry.reference(result['s_wrapped']);t=ref['tangent']
        reconstructed=np.asarray(ref['xy'])+result['d']*np.array([-t[1],t[0]])
        inverse_errors.append(float(np.linalg.norm(reconstructed-point)))
    return dict(samples=len(records),accepted=len(jacobians),failures=failures,
                failure_reasons=dict(Counter(x['reason']for x in failures)),
                lateral_noise_amplitude_m=lateral_noise_m,perturbation='sin(row*0.17) along nominal normal',
                last=last,minimum_J_pose=minimum,
                min_J=min(jacobians) if jacobians else None,
                max_reconstruction_error_m=max(inverse_errors) if inverse_errors else None,
                latency_ms=dict(mean=float(np.mean(elapsed)),p95=float(np.percentile(elapsed,95)),max=max(elapsed)),
                passed=bool(jacobians) and not failures and max(inverse_errors)<=1e-4)


def run(geometry_path,map_yaml,output_dir,samples_path=None,points_path=None,sections=512,d_step_m=.1):
    geometry=load_geometry(geometry_path,map_yaml,inspect=True)
    root=Path(output_dir);root.mkdir(parents=True,exist_ok=False)
    sampled=check(geometry,sections)
    sources=[diagnose_source(geometry,p['s'],p['d']) for p in sampled['invalid_details']]
    fixed=[]
    if points_path is not None:
        for p in json.loads(Path(points_path).read_text()):
            xy=p.get('query_xy',p.get('xy'))
            result=geometry.to_frenet(xy)
            diagnostics=geometry.projection_candidates(xy)
            fixed.append(dict(xy=xy,result=result,diagnostics=diagnostics,
                cause='nearest_projection_J_margin' if diagnostics['candidates']
                    and diagnostics['candidates'][0]['jacobian']<.2 else result['reason']))
    centres=[p['s_m'] for p in sources]
    centres += [p['diagnostics']['candidates'][0]['s'] for p in fixed
                if not p['result']['valid'] and p['diagnostics']['candidates']]
    extra=[s+delta for s in centres for delta in np.arange(-.5,.50001,.05)]
    print('Building sampled source-domain model; critical regions receive additional sections',flush=True)
    atlas=build_domain(geometry,sections,d_step_m,extra,
        progress=lambda p:print('domain',p['completed'],'/',p['total'],flush=True))
    domain_id=save_domain(atlas,root/'domain');loaded=load_domain(root/'domain',geometry)
    centre_failures=[s['s_m'] for s in atlas.data['sections']if not loaded.query(s['s_m'],0)['valid']]
    profile_samples=[p for s in atlas.data['sections']for p in s['samples']]
    report=dict(stage='C',geometry_id=geometry.geometry_id,domain_id=domain_id,
        planning_allowed=False,domain_verified=False,sampled_roundtrip=sampled,
        rejected_source_diagnosis=sources,fixed_points=fixed,
        domain=dict(sections=len(atlas.data['sections']),queries=len(profile_samples),
            valid_queries=sum(p['valid']for p in profile_samples),
            invalid_reasons=dict(Counter(p['reason']for p in profile_samples if not p['valid'])),
            centre_failures=centre_failures,s_interpolation_allowed=False,d_interpolation_allowed=False,
            membership_method='exact_source_query'),tracking={},
        limitations=['Stage D dense/adaptive full-domain acceptance remains pending',
                     'No controller deployment, real vehicle or new closed-loop run',
                     'Tracking tolerances are configurable engineering defaults, not hardware calibration'])
    inputs={'map_yaml':sha256(map_yaml)}
    if points_path is not None:inputs['diagnostic_points']=sha256(points_path)
    if samples_path is not None:
        inputs['replay_samples']=sha256(samples_path)
        records=[json.loads(x)for x in Path(samples_path).read_text().splitlines()]
        if not records:raise ValueError('empty replay records')
        for noise in (0.,.025):
            print('Replaying',len(records),'poses; lateral perturbation',noise,'m',flush=True)
            result=replay(geometry,records,noise)
            report['tracking']['nominal' if noise==0 else 'perturbed_025m']=result
            print('replay accepted',result['accepted'],'/',result['samples'],'passed',result['passed'],flush=True)
    report['stage_c_passed']=(sampled['sampled_check_passed'] and not centre_failures
        and bool(report['tracking']) and all(p['passed']for p in report['tracking'].values()))
    report['stage_c_scope']='API, exact source-domain model, stateful tracking and replay; global approval excluded'
    (root/'report.json').write_bytes(json_bytes(report))
    contract=dict(schema_version=1,required_tf_fields=['header.frame_id','child_frame_id','header.stamp',
        'transform.translation','transform.rotation'],expected_target_frame='map',
        default_source_reference='base_link',offset_calibrated=False,
        required_context=['matching_clock_now','geometry_id'],options=asdict(TrackingOptions()),
        recovery='explicit reset after any rejected pose',planning_allowed=False)
    (root/'pose_contract.json').write_bytes(json_bytes(contract))
    names=['domain/validity.json','domain/manifest.json','report.json','pose_contract.json']
    code_root=Path(__file__).parent
    manifest=dict(schema_version=1,stage='C',geometry_id=geometry.geometry_id,domain_id=domain_id,
        source_hashes=inputs,code_hashes={name:sha256(code_root/name)for name in
            ('frenet.py','domain.py','tracker.py','complete_stage_c.py','extraction.py','map_raster.py')},
        software=dict(python=platform.python_version(),numpy=np.__version__,scipy=scipy.__version__),
        file_hashes={name:sha256(root/name)for name in names},planning_allowed=False,domain_verified=False)
    manifest['bundle_id']=hashlib.sha256(json_bytes(manifest)).hexdigest()
    (root/'manifest.json').write_bytes(json_bytes(manifest))
    verify_stage_c_bundle(root,geometry)
    return report


def verify_stage_c_bundle(directory,geometry):
    root=Path(directory);manifest=json.loads((root/'manifest.json').read_text())
    claimed=manifest.pop('bundle_id',None)
    if (hashlib.sha256(json_bytes(manifest)).hexdigest()!=claimed
        or manifest.get('stage')!='C' or manifest.get('schema_version')!=1
        or manifest.get('geometry_id')!=geometry.geometry_id
        or manifest.get('planning_allowed') is not False or manifest.get('domain_verified') is not False):
        raise ValueError('Stage C manifest mismatch')
    expected={'domain/validity.json','domain/manifest.json','report.json','pose_contract.json'}
    if set(manifest['file_hashes'])!=expected:
        raise ValueError('Stage C invalid file set')
    for name,digest in manifest['file_hashes'].items():
        if sha256(root/name)!=digest:
            raise ValueError('Stage C file hash mismatch: '+name)
    atlas=load_domain(root/'domain',geometry)
    if manifest['domain_id']!=hashlib.sha256(json_bytes(atlas.data)).hexdigest():
        raise ValueError('Stage C domain mismatch')
    report=json.loads((root/'report.json').read_text())
    if report['geometry_id']!=geometry.geometry_id or report['domain_id']!=manifest['domain_id']:
        raise ValueError('Stage C report mismatch')
    return dict(passed=True,bundle_id=claimed,geometry_id=geometry.geometry_id,
                stage_c_passed=report['stage_c_passed'],planning_allowed=False,domain_verified=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry',type=Path,required=True)
    parser.add_argument('--map-yaml',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--samples',type=Path)
    parser.add_argument('--points-json',type=Path)
    parser.add_argument('--sections',type=int,default=512)
    parser.add_argument('--d-step-m',type=float,default=.1)
    args=parser.parse_args()
    report=run(args.geometry,args.map_yaml,args.output_dir,args.samples,args.points_json,args.sections,args.d_step_m)
    print(json.dumps(dict(stage_c_passed=report['stage_c_passed'],geometry_id=report['geometry_id'],
        domain=report['domain'],planning_allowed=False),indent=2))
    if not report['stage_c_passed']:raise SystemExit(1)


if __name__=='__main__':main()
