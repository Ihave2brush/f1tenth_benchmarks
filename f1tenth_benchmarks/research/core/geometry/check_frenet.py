"""Sampled inspect-mode roundtrip report; does not approve a geometry version."""
import argparse
import json
from pathlib import Path

import numpy as np

from .frenet import load_geometry, wrap


def check(geometry, count=512):
    errors, s_errors, d_errors, yaw_errors = [], [], [], []
    invalid, failures = [], []
    positions = np.linspace(0, geometry.curve.L, count, endpoint=not geometry.curve.closed).tolist()
    positions += [1e-6, geometry.curve.L-1e-6]
    for s in positions:
        ref = geometry.reference(s)
        boundary = geometry.boundaries(s)
        if not boundary['valid']:
            failures.append(dict(s=s,reason='invalid_boundary')); continue
        lateral = [0.]+[fraction*boundary[key] for key in ('d_left','d_right') for fraction in (.25,.5,.9)]
        tangent=np.array(ref['tangent']); normal=np.array([-tangent[1],tangent[0]])
        for d in lateral:
            # Reference construction is independent of to_cartesian implementation.
            xy=np.array(ref['xy'])+d*normal
            intended=geometry.to_cartesian(s,d,.13)
            if not intended['valid']:
                invalid.append(dict(s=s,d=d,reason=intended['reason']))
                if d==0:
                    failures.append(dict(s=s,d=d,reason=intended['reason']))
                continue
            result=geometry.to_frenet(xy,ref['psi']+.13)
            if not result['valid']:
                invalid.append(dict(s=s,d=d,reason=result['reason']))
                if d==0:
                    failures.append(dict(s=s,d=d,reason=result['reason']))
                continue
            inverse=geometry.to_cartesian(result['s_wrapped'],result['d'],result['e_psi'])
            if not inverse['valid']:
                failures.append(dict(s=s,d=d,reason=inverse['reason'])); continue
            errors.append(float(np.linalg.norm(np.array(inverse['xy'])-xy)))
            s_errors.append(geometry.distance_s(s,result['s_wrapped']))
            d_errors.append(abs(result['d']-d))
            yaw_errors.append(abs(wrap(inverse['yaw']-(ref['psi']+.13))))
            if s_errors[-1]>1e-4 or d_errors[-1]>1e-4:
                failures.append(dict(s=s,d=d,reason='source_coordinate_mismatch',
                                     s_error_m=s_errors[-1],d_error_m=d_errors[-1]))
    passed=bool(errors) and not failures and max(errors)<=1e-4 and max(s_errors)<=1e-4 and max(d_errors)<=1e-4 and max(yaw_errors)<=1e-4
    return dict(stage='C',sampled_check_passed=passed,geometry_id=geometry.geometry_id,
                planning_allowed=False,domain_verified=False,s_sections=len(positions),
                valid_queries=len(errors),invalid_queries=len(invalid),invalid_details=invalid,
                failures=failures,max_position_error_m=max(errors) if errors else None,
                max_s_error_m=max(s_errors) if errors else None,max_d_error_m=max(d_errors) if errors else None,
                max_yaw_error_rad=max(yaw_errors) if errors else None,
                p95_position_error_m=float(np.percentile(errors,95)) if errors else None,
                limitations=['sampled queries do not establish the full valid domain',
                             '0.05 m dense/adaptive validation and interactive viewer remain Stage D',
                             'boundary CSV interpolation is not used; query boundaries use exact grid rays'])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry',type=Path,required=True)
    parser.add_argument('--map-yaml',type=Path,required=True)
    parser.add_argument('--output-dir',type=Path,required=True)
    parser.add_argument('--sections',type=int,default=512)
    args=parser.parse_args()
    if args.sections<4:
        parser.error('--sections must be at least four')
    geometry=load_geometry(args.geometry,args.map_yaml,inspect=True)
    report=check(geometry,args.sections)
    args.output_dir.mkdir(parents=True,exist_ok=False)
    (args.output_dir/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    summary={k:v for k,v in report.items() if k not in ('invalid_details','failures')}
    print(json.dumps(summary,indent=2))
    if not report['sampled_check_passed']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
