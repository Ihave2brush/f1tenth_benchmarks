"""Audit saved geometry and index sampled diagnostics without approving planning."""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from .domain import json_bytes
from .frenet import load_geometry, wrap
from .map_raster import sha256
from .validate_stage_d import load_report


LABELS = {'singular_frenet': 'Frenet 餘裕不足',
          'ambiguous_projection': '投影歧義',
          'invalid_boundary': '邊界異常',
          'boundary_interpolation': '僅插值誤差'}


def cyclic_runs(indices, count, closed=True):
    """Group consecutive samples; merge endpoint runs only for closed tracks."""
    indices = sorted(set(indices))
    if not indices:
        return []
    runs = [[indices[0]]]
    for index in indices[1:]:
        if index == runs[-1][-1] + 1:
            runs[-1].append(index)
        else:
            runs.append([index])
    if closed and len(runs) > 1 and runs[0][0] == 0 and runs[-1][-1] == count-1:
        runs = [runs[-1]+runs[0]] + runs[1:-1]
    return runs


def audit_geometry(geometry, directory):
    root = Path(directory)
    center = np.loadtxt(root/'centerline.csv', delimiter=',', skiprows=1, ndmin=2)
    with (root/'boundaries.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    failures = []
    if center.shape[1] != 5 or not np.isfinite(center).all():
        raise ValueError('invalid centerline columns or nonfinite values')
    if len(rows) != len(center):
        raise ValueError('centerline/boundary row mismatch')
    positions = center[:, 2]
    if positions[0] != 0 or np.any(np.diff(positions) <= 0) or (positions[-1] >= geometry.curve.L if geometry.curve.closed else positions[-1] > geometry.curve.L):
        raise ValueError('invalid saved arc-length order')
    xy, tangent, psi, kappa = geometry.curve.sample(positions)
    errors = dict(center_xy_m=float(np.linalg.norm(xy-center[:, :2], axis=1).max()),
                  heading_rad=max(abs(wrap(a-b)) for a,b in zip(psi,center[:,3])),
                  curvature_inv_m=float(np.max(np.abs(kappa-center[:,4]))),
                  boundary_xy_m=0., boundary_d_m=0.)
    widths = []; low_j = Counter()
    for i, row in enumerate(rows):
        try:
            values = np.array([float(row[key]) for key in
                ('s_m','left_x_m','left_y_m','d_left_m','right_x_m','right_y_m','d_right_m')])
            if not np.isfinite(values).all() or row['valid'] != '1':
                raise ValueError('missing or invalid boundary')
            s,lx,ly,dl,rx,ry,dr = values
            if abs(s-positions[i]) > 1e-8 or not dr < 0 < dl:
                raise ValueError('boundary source or sign mismatch')
            boundary = geometry.boundaries(s)
            if not boundary['valid']:
                raise ValueError('exact boundary query failed')
            n = np.array([-tangent[i,1], tangent[i,0]])
            for side, point, d in (('left',[lx,ly],dl),('right',[rx,ry],dr)):
                errors['boundary_xy_m'] = max(errors['boundary_xy_m'],
                    float(np.linalg.norm(np.asarray(point)-boundary[side+'_xy'])),
                    float(np.linalg.norm(np.asarray(point)-(xy[i]+d*n))))
                errors['boundary_d_m'] = max(errors['boundary_d_m'], abs(d-boundary['d_'+side]))
                if 1-kappa[i]*d < .2:
                    low_j[side] += 1
            widths.append([dl,-dr,dl-dr])
        except (KeyError, ValueError, TypeError) as exc:
            failures.append(dict(row=i,s_m=float(positions[i]),reason=str(exc)))
    if any(v > 1e-4 for v in errors.values()):
        failures.append(dict(reason='saved_geometry_disagreement',errors=errors))
    signed_area = float(np.sum(xy[:,0]*np.roll(xy[:,1],-1)-xy[:,1]*np.roll(xy[:,0],-1))/2)
    widths = np.asarray(widths)
    return dict(passed=not failures, rows=len(center), complete_boundary_rows=len(widths),
        failures=failures, max_errors=errors, origin_xy=xy[0].tolist(), L_m=geometry.curve.L,
        direction=('clockwise' if signed_area < 0 else 'counterclockwise') if geometry.curve.closed else 'open_start_to_end',
        width_min_m=None if not len(widths) else widths.min(axis=0).tolist(),
        width_max_m=None if not len(widths) else widths.max(axis=0).tolist(),
        width_columns=['left','right','total'], boundary_low_J_samples=dict(low_j),
        boundary_sd_semantics='source cross-section, not a claim of unique inverse projection')


def summarize(geometry, report, audit):
    sections = report['sections']; count = len(sections)
    indices = {p['s_m']: i for i,p in enumerate(sections)}
    reasons = {}
    for point in report['rejected_points']:
        reasons.setdefault(point['reason'], {}).setdefault(indices[point['s_m']], []).append(point)
    for i,section in enumerate(sections):
        if not section['boundary']['valid']:
            ref=geometry.reference(section['s_m'])
            reasons.setdefault('invalid_boundary', {}).setdefault(i, []).append(
                dict(s_m=section['s_m'],d_m=0.,xy=ref['xy'],reason='invalid_boundary'))
    issues = []

    def issue(reason, positions, samples, crosses_seam=False):
        start, end = positions[0], positions[-1]
        jacobians=[p['J'] for p in samples if 'J' in p]
        # These are sampled extents, not certified boundaries of an invalid domain.
        issues.append(dict(id='D-'+str(len(issues)+1).zfill(3), category=reason,
            label=LABELS.get(reason, '其他拒絕：'+reason), s_start_m=float(start),
            s_end_m=float(end), crosses_seam=bool(crosses_seam),
            sample_count=len(samples), samples=samples,
            min_J=min(jacobians) if jacobians else None,
            max_J=max(jacobians) if jacobians else None,
            nonpositive_J_samples=sum(j<=0 for j in jacobians),
            example_index=int(np.argmin(jacobians)) if reason=='singular_frenet' and jacobians else 0,
            center_xy=[geometry.reference(s)['xy'] for s in positions],
            suggested_action=('保留精確邊界查詢；不啟用此處插值' if reason=='boundary_interpolation'
                else '保留原道路；規劃前需確認路線是否進入此範圍'),
            continuous_verified=False))

    for reason, by_index in sorted(reasons.items()):
        for run in cyclic_runs(by_index,count,geometry.curve.closed):
            samples=[]
            for i in run:
                for point in by_index[i]:
                    ref=geometry.reference(point['s_m'])
                    samples.append(dict(point, J=1-ref['kappa']*point['d_m'],
                        kappa_inv_m=ref['kappa'], source_valid=False))
            issue(reason,[sections[i]['s_m'] for i in run],samples,
                  any(a>b for a,b in zip(run,run[1:])))
    leaves=report['boundary_segments']
    bad=[i for i,p in enumerate(leaves) if not p.get('sampled_tolerance_passed',False)]
    def section_s(s):
        return float(s%geometry.curve.L) if geometry.curve.closed else float(s)

    for run in cyclic_runs(bad,len(leaves),geometry.curve.closed):
        positions=[leaves[i]['s_start'] for i in run]+[section_s(leaves[run[-1]]['s_end'])]
        samples=[dict(s_m=section_s((leaves[i]['s_start']+leaves[i]['s_end'])/2),
            xy=geometry.reference((leaves[i]['s_start']+leaves[i]['s_end'])/2)['xy'],
            d_m=0., midpoint_error_m=leaves[i].get('midpoint_error_m'),
            quarter_errors_m=leaves[i].get('quarter_errors_m'), source_valid=None) for i in run]
        issue('boundary_interpolation',positions,samples,
              geometry.curve.closed and (any(a>b for a,b in zip(run,run[1:]))
                  or leaves[run[-1]]['s_end']>=geometry.curve.L))
    return dict(schema_version=1,stage='D_diagnostics',geometry_id=geometry.geometry_id,
        diagnostic_ready=bool(audit['passed'] and report['numeric_checks_passed']),
        geometry_audit=audit, issues=issues, issue_groups=len(issues),
        groups_by_category=dict(Counter(p['category'] for p in issues)),
        rejected_source_queries=report['rejected_queries'],
        interpolation_failed_leaves=len(bad),
        coverage=dict(sections=count,dense_step_m=report['dense_step_m'],
            grouping=('consecutive sampled sections or boundary leaves; cyclic seam merged'
                if geometry.curve.closed else 'consecutive sampled sections or boundary leaves; open endpoints separate'),
            continuous_verified=False), domain_verified=False,planning_allowed=False,
        limitations=['Diagnostic completion is separate from planning approval',
            'Sampled extents do not certify all points inside or outside each group',
            'GUI acceptance is saved separately; historical D report remains unchanged'])


def save_diagnostics(result, geometry_path, report_path, output_dir):
    root=Path(output_dir);root.mkdir(parents=True,exist_ok=False)
    (root/'diagnostics.json').write_bytes(json_bytes(result))
    manifest=dict(schema_version=1,stage='D_diagnostics',geometry_id=result['geometry_id'],
        source_hashes=dict(geometry_manifest=sha256(Path(geometry_path)/'manifest.json'),
            dense_report=sha256(Path(report_path)/'report.json')),
        code_hashes={Path(__file__).name:sha256(__file__)},
        file_hashes={'diagnostics.json':sha256(root/'diagnostics.json')})
    manifest['bundle_id']=hashlib.sha256(json_bytes(manifest)).hexdigest()
    (root/'manifest.json').write_bytes(json_bytes(manifest))


def load_diagnostics(directory, geometry, report_path=None, geometry_path=None):
    root=Path(directory);manifest=json.loads((root/'manifest.json').read_text())
    bundle=manifest.pop('bundle_id',None)
    if (bundle!=hashlib.sha256(json_bytes(manifest)).hexdigest()
        or manifest.get('schema_version')!=1 or manifest.get('stage')!='D_diagnostics'
        or manifest.get('geometry_id')!=geometry.geometry_id
        or manifest.get('file_hashes')!={'diagnostics.json':sha256(root/'diagnostics.json')}):
        raise ValueError('diagnostics hash/version mismatch')
    if report_path is not None and manifest['source_hashes']['dense_report']!=sha256(Path(report_path)/'report.json'):
        raise ValueError('diagnostics source report mismatch')
    if geometry_path is not None and manifest['source_hashes']['geometry_manifest']!=sha256(Path(geometry_path)/'manifest.json'):
        raise ValueError('diagnostics source geometry mismatch')
    result=json.loads((root/'diagnostics.json').read_text())
    if (result.get('geometry_id')!=geometry.geometry_id or result.get('schema_version')!=1
        or result.get('stage')!='D_diagnostics' or result.get('domain_verified') is not False
        or result.get('planning_allowed') is not False):
        raise ValueError('diagnostics mismatch')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('geometry','map-yaml','report','output-dir'):
        parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args()
    geometry=load_geometry(args.geometry,args.map_yaml,inspect=True)
    report=load_report(args.report,geometry)
    result=summarize(geometry,report,audit_geometry(geometry,args.geometry))
    save_diagnostics(result,args.geometry,args.report,args.output_dir)
    print(json.dumps({k:v for k,v in result.items() if k!='issues'},ensure_ascii=False,indent=2))
    if not result['diagnostic_ready']:
        raise SystemExit(1)


if __name__=='__main__':
    main()
