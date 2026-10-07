"""Exact road limits and sampled Frenet guides; every candidate needs an exact check."""
import argparse
from collections import Counter
import hashlib
import gzip
import json
from pathlib import Path

import numpy as np

from .domain import json_bytes
from .frenet import load_geometry
from .map_raster import sha256
from .complete_stage_d import audit_geometry


def load_ranges(directory,geometry,geometry_path=None):
    root=Path(directory);manifest=json.loads((root/'manifest.json').read_text())
    bundle=manifest.pop('bundle_id',None)
    files=set(manifest.get('file_hashes',{}))
    payload_name='section_ranges.json.gz' if 'section_ranges.json.gz' in files else 'section_ranges.json'
    csv_location=manifest.get('csv_location','bundle')
    if csv_location not in ('bundle','source_geometry'):
        raise ValueError('range CSV location mismatch')
    if csv_location=='source_geometry' and geometry_path is None:
        raise ValueError('source geometry path required for range CSV')
    required={payload_name,'report.json','centerline.csv','boundaries.csv'}
    if (bundle!=hashlib.sha256(json_bytes(manifest)).hexdigest() or
        manifest.get('schema_version')!=1 or manifest.get('geometry_id')!=geometry.geometry_id or
        set(manifest.get('file_hashes',{}))!=required):raise ValueError('range manifest mismatch')
    for name,digest in manifest['file_hashes'].items():
        source=Path(geometry_path) if csv_location=='source_geometry' and name in ('centerline.csv','boundaries.csv') else root
        if sha256(source/name)!=digest:raise ValueError('range artifact hash mismatch')
    if any(manifest['source_hashes'][key]!=geometry.raster.source_hashes[key] for key in ('yaml','image')):
        raise ValueError('range source map mismatch')
    if geometry_path is not None and sha256(Path(geometry_path)/'manifest.json')!=manifest['source_hashes']['geometry_manifest']:
        raise ValueError('range source geometry mismatch')
    raw=(root/payload_name).read_bytes()
    payload=json.loads(gzip.decompress(raw) if payload_name.endswith('.gz') else raw)
    if (payload.get('geometry_id')!=geometry.geometry_id or payload.get('schema_version')!=1 or
        payload.get('summary')!=json.loads((root/'report.json').read_text()) or
        payload['summary'].get('planning_allowed') is not False or
        payload['summary'].get('continuous_verified') is not False):raise ValueError('range payload mismatch')
    return payload


def sample_runs(samples, valid, split_reason=False):
    runs=[];current=[]
    for sample in samples:
        if sample['valid']==valid:
            if current and split_reason and current[-1]['reason']!=sample['reason']:
                runs.append(current);current=[]
            current.append(sample)
        elif current:
            runs.append(current);current=[]
    if current:runs.append(current)
    return [dict(d_min_m=r[0]['d_m'],d_max_m=r[-1]['d_m'],sample_count=len(r),
        reasons=dict(Counter(p['reason'] for p in r if p['reason'])),
        continuous_verified=False) for r in runs]


def query_section(geometry, s, d_step_m=.025, transition_tolerance_m=.0001):
    if (not np.isscalar(d_step_m) or not np.isfinite(d_step_m) or
            not 0<d_step_m<=min(.025,geometry.raster.resolution/2)):
        raise ValueError('d step must be positive and meet map-resolution limit')
    if not np.isscalar(s) or not np.isfinite(s):raise ValueError('nonfinite s')
    if not np.isscalar(transition_tolerance_m) or not np.isfinite(transition_tolerance_m) or not 0<transition_tolerance_m<=d_step_m:
        raise ValueError('invalid transition tolerance')
    ref=geometry.reference(s);boundary=geometry.boundaries(s)
    result=dict(schema_version=1,geometry_id=geometry.geometry_id,frame_id='map',
        closed=geometry.curve.closed,s_domain_m=[0.,geometry.curve.L],
        s_m=ref['s_wrapped'],reference=ref,road_range=None,frenet_ranges=[],
        limited_ranges=[],samples=[],reason=None,range_query_valid=boundary['valid'],
        verification_level='sampled_requires_exact_query',continuous_verified=False,
        exact_query_required=True,cartesian_validation_required=True,
        domain_verified=False,planning_allowed=False,
        sampling=dict(d_step_m=d_step_m,transition_tolerance_m=transition_tolerance_m),
        boundary=boundary)
    if not boundary['valid']:
        return dict(result,reason=boundary['reason'] or 'invalid_boundary')
    lower,upper=boundary['d_right'],boundary['d_left']
    if lower is None or upper is None or not lower<0<upper:
        return dict(result,range_query_valid=False,reason='invalid_boundary_signs')
    result['road_range']=dict(d_min_m=lower,d_max_m=upper,left_xy=boundary['left_xy'],
        right_xy=boundary['right_xy'],endpoint_semantics='physical occupancy interfaces, not approved vehicle positions')
    epsilon=min(1e-6,(upper-lower)/1000)
    a,b=lower+epsilon,upper-epsilon
    values=np.linspace(a,b,int(np.ceil((b-a)/d_step_m))+1).tolist()+[0.]
    k=ref['kappa']
    if k!=0:
        critical=.8/k
        values += [critical-epsilon,critical,critical+epsilon]
    values=sorted(set(float(d) for d in values if a<=d<=b))
    cache={}
    def sample(d):
        if d not in cache:
            checked=geometry.to_cartesian(ref['s_wrapped'],d)
            cache[d]=dict(d_m=d,valid=bool(checked['valid']),reason=checked['reason'],
                J=1-k*d,xy=checked['xy'])
        return cache[d]
    for d in values:sample(d)
    # Refine only observed status transitions; unobserved pockets are not certified.
    def refine(left,right):
        first,last=sample(left),sample(right)
        if (first['valid'],first['reason'])==(last['valid'],last['reason']):return
        if right-left<=transition_tolerance_m:return
        mid=(left+right)/2
        if mid==left or mid==right:return
        sample(mid);refine(left,mid);refine(mid,right)
    for left,right in zip(values,values[1:]):refine(left,right)
    samples=[cache[d] for d in sorted(cache)]
    result.update(samples=samples,frenet_ranges=sample_runs(samples,True),
        limited_ranges=sample_runs(samples,False,True),
        rejection_reasons=dict(Counter(p['reason'] for p in samples if not p['valid'])))
    result['sampling']['sample_count']=len(samples)
    result['sampling']['max_observed_gap_m']=max(np.diff([p['d_m'] for p in samples]))
    if not result['frenet_ranges']:result['reason']='no_accepted_samples'
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('geometry','map-yaml','output-dir'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--s-step-m',type=float,default=.2)
    parser.add_argument('--d-step-m',type=float,default=.025)
    parser.add_argument('--extra-s',type=float,nargs='*',default=[])
    args=parser.parse_args()
    code_hashes={n:sha256(Path(__file__).with_name(n)) for n in
        ('section_ranges.py','frenet.py','extraction.py','build_map.py','map_raster.py','complete_stage_d.py')}
    if not np.isfinite(args.s_step_m) or not 0<args.s_step_m<=.2 or not np.isfinite(args.extra_s).all():
        parser.error('s step must be in (0, 0.2]; extra s must be finite')
    g=load_geometry(args.geometry,args.map_yaml,inspect=True)
    audit=audit_geometry(g,args.geometry)
    positions=np.linspace(0,g.curve.L,int(np.ceil(g.curve.L/args.s_step_m))+(0 if g.curve.closed else 1),endpoint=not g.curve.closed).tolist()
    raw=positions+[1e-6,g.curve.L-1e-6]+args.extra_s
    if not g.curve.closed and any(not 0<=s<=g.curve.L for s in raw):parser.error('extra s outside open reference')
    positions=sorted(set(float(s%g.curve.L) if g.curve.closed else float(s) for s in raw))
    root=args.output_dir;root.mkdir(parents=True,exist_ok=False)
    sections=[]
    for i,s in enumerate(positions):
        sections.append(query_section(g,s,args.d_step_m))
        if i%64==0:print('Sections: '+str(i+1)+'/'+str(len(positions)),flush=True)
    summary=dict(schema_version=1,geometry_id=g.geometry_id,sections=len(sections),
        boundary_failures=sum(not p['range_query_valid'] for p in sections),
        sections_without_accepted_samples=sum(not p['frenet_ranges'] for p in sections),
        limited_sections=sum(bool(p['limited_ranges']) for p in sections),
        sampled_queries=sum(len(p['samples']) for p in sections),
        rejected_queries=sum(sum(p.get('rejection_reasons',{}).values()) for p in sections),
        rejection_reasons=dict(sum((Counter(p.get('rejection_reasons',{})) for p in sections),Counter())),
        numerical_checks_passed=True,range_interface_passed=False,
        verification_level='sampled_requires_exact_query',continuous_verified=False,
        domain_verified=False,planning_allowed=False)
    failures=[];max_error=0.;roundtrip_checks=0
    # Check both endpoints of each accepted run and its central sample by forward projection.
    for section in sections:
        ref=section['reference'];n=np.array([-ref['tangent'][1],ref['tangent'][0]])
        selected={0.}
        for run in section['frenet_ranges']:
            selected.update((run['d_min_m'],run['d_max_m']))
            accepted=[p['d_m'] for p in section['samples'] if run['d_min_m']<=p['d_m']<=run['d_max_m']]
            selected.add(accepted[len(accepted)//2])
        for p in section['samples']:
            if not p['valid']:continue
            error=float(np.linalg.norm(np.asarray(ref['xy'])+p['d_m']*n-p['xy']))
            if p['d_m'] in selected:
                roundtrip_checks+=1;forward=g.to_frenet(p['xy'])
                if not forward['valid']:
                    failures.append(dict(s_m=section['s_m'],d_m=p['d_m'],reason=forward['reason']));continue
                ds=g.distance_s(forward['s_wrapped'],section['s_m'])
                error=max(error,ds,abs(forward['d']-p['d_m']))
            max_error=max(error,max_error)
            if error>1e-4:failures.append(dict(s_m=section['s_m'],d_m=p['d_m'],error_m=error))
    summary.update(max_roundtrip_error_m=max_error,roundtrip_checks=roundtrip_checks,failures=failures,
        numerical_checks_passed=not failures,
        range_interface_passed=not failures and audit['passed'] and not summary['boundary_failures'] and not summary['sections_without_accepted_samples'])
    payload=dict(schema_version=1,geometry_id=g.geometry_id,summary=summary,sections=sections,
        s_interpolation_allowed=False,d_interpolation_allowed=False,geometry_audit=audit,
        limitations=['Accepted sampled intervals do not certify unobserved d or s',
            'Use to_cartesian(s,d) for each candidate and Cartesian car-footprint/dynamics checks afterward'])
    (root/'section_ranges.json').write_bytes(json_bytes(payload))
    (root/'report.json').write_bytes(json_bytes(summary))
    for filename in ('centerline.csv','boundaries.csv'):
        (root/filename).write_bytes((args.geometry/filename).read_bytes())
    manifest=dict(schema_version=1,geometry_id=g.geometry_id,
        source_hashes=dict(geometry_manifest=sha256(args.geometry/'manifest.json'),
            yaml=sha256(args.map_yaml),image=g.raster.source_hashes['image']),
        code_hashes=code_hashes,
        sampling=dict(s_step_m=args.s_step_m,d_step_m=args.d_step_m,extra_s=args.extra_s),
        file_hashes={n:sha256(root/n) for n in ('section_ranges.json','report.json','centerline.csv','boundaries.csv')},
        planning_allowed=False,domain_verified=False)
    manifest['bundle_id']=hashlib.sha256(json_bytes(manifest)).hexdigest()
    (root/'manifest.json').write_bytes(json_bytes(manifest))
    print(json_bytes(summary).decode(),flush=True)
    if not summary['range_interface_passed']:raise SystemExit(1)


if __name__=='__main__':main()
