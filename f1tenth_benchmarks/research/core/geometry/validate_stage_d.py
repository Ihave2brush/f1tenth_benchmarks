"""Dense and adaptive inspect-mode checks; never approve unsampled domains."""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path

import numpy as np

from .domain import json_bytes
from .frenet import load_geometry, circular_distance, wrap
from .map_raster import sha256


def validate(geometry, step_m=.05, max_depth=4, progress=None):
    if not np.isfinite(step_m) or step_m<=0 or step_m>min(.05,geometry.raster.resolution):
        raise ValueError('step must meet Stage D map-resolution limit')
    if not isinstance(max_depth,int) or not 0<=max_depth<=8:
        raise ValueError('invalid adaptive depth')
    count=int(np.ceil(geometry.curve.L/step_m));spacing=geometry.curve.L/count
    sections={};errors=[];failures=[];rejected=[]

    def section(s):
        s=float(s%geometry.curve.L)
        if s in sections:return sections[s]
        ref=geometry.reference(s);boundary=geometry.boundaries(s)
        if not boundary['valid']:
            failures.append(dict(s_m=s,reason='invalid_boundary'))
            result=dict(s_m=s,boundary=boundary,samples=[],signature=[])
            sections[s]=result;return result
        lower,upper=boundary['d_right'],boundary['d_left'];k=ref['kappa']
        necessary_lower=max(lower,.8/k) if k<0 else lower
        necessary_upper=min(upper,.8/k) if k>0 else upper
        epsilon=max(1e-6,geometry.raster.resolution*1e-5)
        values=[0.]
        # Both the physical road and necessary J-limited range are inspected.
        for bound in (lower,upper,necessary_lower,necessary_upper):
            values += [bound*f for f in (.25,.5,.9)]
            values.append(bound-np.sign(bound)*epsilon)
        t=np.asarray(ref['tangent']);normal=np.array([-t[1],t[0]])
        samples=[]
        for d in sorted(set(float(v)for v in values)):
            xy=np.asarray(ref['xy'])+d*normal
            result=geometry.to_cartesian(s,d,.13)
            row=dict(d_m=d,valid=result['valid'],reason=result['reason'])
            if not result['valid']:
                rejected.append(dict(s_m=s,d_m=d,xy=xy.tolist(),reason=result['reason']))
                if d==0:failures.append(dict(s_m=s,reason='invalid_centre'))
            else:
                forward=geometry.to_frenet(xy,ref['psi']+.13)
                if not forward['valid']:
                    failures.append(dict(s_m=s,d_m=d,reason='forward_inverse_disagreement'))
                else:
                    projected=geometry.reference(forward['s_wrapped'])
                    tangent=np.asarray(projected['tangent']);normal_back=np.array([-tangent[1],tangent[0]])
                    roundtrip=np.asarray(projected['xy'])+forward['d']*normal_back
                    err=[float(np.linalg.norm(roundtrip-xy)),
                         circular_distance(s,forward['s_wrapped'],geometry.curve.L),
                         abs(d-forward['d']),abs(wrap(result['yaw']-(ref['psi']+.13)))]
                    errors.append(err)
                    if max(err)>1e-4:failures.append(dict(s_m=s,d_m=d,reason='roundtrip_error',errors=err))
            samples.append(row)
        result=dict(s_m=s,boundary=boundary,kappa=k,necessary_d_min=necessary_lower,
                    necessary_d_max=necessary_upper,samples=samples,
                    signature=[(p['valid'],p['reason'])for p in samples])
        sections[s]=result;return result

    positions=np.arange(count)*spacing
    for i,s in enumerate(positions):
        section(s)
        if progress and i%256==0:progress(dict(phase='dense',completed=i,total=count))
    section(1e-6);section(geometry.curve.L-1e-6)
    boundary_checks=[]

    def refine(a,b,depth):
        left,right=section(a),section(b);mid=(a+b)/2;middle=section(mid)
        if not all(p['boundary']['valid']for p in (left,middle,right)):
            boundary_checks.append(dict(s_start=a,s_end=b,interpolation_allowed=False,reason='invalid_boundary'));return
        error=max(float(np.linalg.norm((np.asarray(left['boundary'][key])+np.asarray(right['boundary'][key]))/2
                      -np.asarray(middle['boundary'][key])))for key in ('left_xy','right_xy'))
        changed=(left['signature']!=middle['signature'] or middle['signature']!=right['signature'])
        curved=max(abs(left['kappa']-middle['kappa']),abs(middle['kappa']-right['kappa']))>.1
        if depth<max_depth and (error>geometry.raster.resolution*.5 or changed or curved):
            refine(a,mid,depth+1);refine(mid,b,depth+1)
        else:
            boundary_checks.append(dict(s_start=a,s_end=b,midpoint_error_m=error,
                interpolation_allowed=False,midpoint_tolerance_passed=error<=geometry.raster.resolution*.5,
                depth=depth,domain_status_change=changed,continuous_verified=False,
                reason='exact_query_required'))

    for i,s in enumerate(positions):
        refine(float(s),float(s+spacing),0)
        if progress and i%256==0:progress(dict(phase='adaptive',completed=i,total=count))
    finite=[b['midpoint_error_m']for b in boundary_checks if 'midpoint_error_m'in b]
    # Quarter-point checks on every final leaf reduce midpoint-only blind spots.
    for leaf in boundary_checks:
        if 'midpoint_error_m' not in leaf:continue
        a,b=leaf['s_start'],leaf['s_end'];ends=[section(a)['boundary'],section(b)['boundary']]
        checks=[]
        for fraction in (.25,.75):
            actual=geometry.boundaries(a+fraction*(b-a))
            error=max(float(np.linalg.norm((1-fraction)*np.asarray(ends[0][key])+fraction*np.asarray(ends[1][key])
                         -np.asarray(actual[key])))for key in ('left_xy','right_xy')) if actual['valid'] else None
            checks.append(error)
        leaf['quarter_errors_m']=checks
        leaf['sampled_tolerance_passed']=leaf['midpoint_tolerance_passed'] and all(e is not None and e<=geometry.raster.resolution*.5 for e in checks)
    unresolved=sum(not x.get('sampled_tolerance_passed',False) for x in boundary_checks)
    maxima=np.max(errors,axis=0).tolist() if errors else [None]*4
    return dict(schema_version=1,stage='D',geometry_id=geometry.geometry_id,
        dense_step_m=spacing,dense_sections=count,total_sections=len(sections),adaptive_max_depth=max_depth,
        valid_queries=len(errors),rejected_queries=len(rejected),rejection_reasons=dict(Counter(p['reason']for p in rejected)),
        max_errors=dict(zip(('position_m','s_m','d_m','yaw_rad'),maxima)),failures=failures,
        numeric_checks_passed=bool(errors) and not failures,
        boundary_interpolation=dict(tolerance_m=geometry.raster.resolution*.5,leaf_segments=len(boundary_checks),
            unresolved_sampled_segments=unresolved,max_midpoint_error_m=max(finite) if finite else None,
            interpolation_allowed=False,method='exact_grid_ray_at_runtime',continuous_verified=False),
        domain_verified=False,planning_allowed=False,stage_d_complete=False,
        limitations=['Finite sampling does not certify continuous s/d intervals',
                     'CSV boundary interpolation remains disabled, including segments passing finite checks',
                     'GUI acceptance and continuous-domain decisions are recorded separately'],
        sections=sorted(sections.values(),key=lambda p:p['s_m']),rejected_points=rejected,boundary_segments=boundary_checks)


def save_report(report,geometry_path,map_yaml,output_dir):
    root=Path(output_dir);root.mkdir(parents=True,exist_ok=False)
    (root/'report.json').write_bytes(json_bytes(report))
    code=Path(__file__)
    manifest=dict(schema_version=1,stage='D',geometry_id=report['geometry_id'],
        source_hashes=dict(geometry_manifest=sha256(Path(geometry_path)/'manifest.json'),map_yaml=sha256(map_yaml)),
        code_hashes={code.name:sha256(code)},file_hashes={'report.json':sha256(root/'report.json')},
        planning_allowed=False,domain_verified=False)
    manifest['bundle_id']=hashlib.sha256(json_bytes(manifest)).hexdigest()
    (root/'manifest.json').write_bytes(json_bytes(manifest))


def load_report(directory,geometry):
    root=Path(directory);m=json.loads((root/'manifest.json').read_text());claimed=m.pop('bundle_id',None)
    if (hashlib.sha256(json_bytes(m)).hexdigest()!=claimed or m.get('schema_version')!=1
        or m.get('stage')!='D' or m.get('geometry_id')!=geometry.geometry_id
        or m.get('planning_allowed') is not False or m.get('domain_verified') is not False
        or m.get('file_hashes')!={'report.json':sha256(root/'report.json')}):
        raise ValueError('Stage D report hash/version mismatch')
    report=json.loads((root/'report.json').read_text())
    if report.get('geometry_id')!=geometry.geometry_id or report.get('planning_allowed') is not False or report.get('domain_verified') is not False:
        raise ValueError('Stage D report mismatch')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry',type=Path,required=True);parser.add_argument('--map-yaml',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True);parser.add_argument('--step-m',type=float,default=.05)
    parser.add_argument('--max-depth',type=int,default=4);args=parser.parse_args()
    g=load_geometry(args.geometry,args.map_yaml,inspect=True)
    report=validate(g,args.step_m,args.max_depth,lambda p:print(json.dumps(p),flush=True))
    save_report(report,args.geometry,args.map_yaml,args.output_dir)
    print(json.dumps({k:v for k,v in report.items()if k not in ('sections','rejected_points','boundary_segments')},indent=2))
    if not report['numeric_checks_passed']:raise SystemExit(1)


if __name__=='__main__':main()
