"""Stage B candidate generation, quantitative checks and exclusive artifacts."""
import argparse
import csv
from io import StringIO
import hashlib
import json
from pathlib import Path
import platform

import numpy as np
import scipy
from scipy import integrate
from scipy.spatial import cKDTree

from .map_raster import load_map, sha256
from .extraction import ExtractionOptions, build_geometry


def json_bytes(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'),
                      ensure_ascii=False, allow_nan=False).encode('utf-8')


def csv_bytes(header, rows):
    output = StringIO(newline='')
    writer = csv.writer(output, lineterminator='\n')
    writer.writerow(header)
    def format_value(value):
        if value is None:
            return ''
        if isinstance(value, (float, np.floating)):
            if not np.isfinite(value):
                raise ValueError('nonfinite CSV value')
            return format(value, '.17g')
        return value
    writer.writerows([[format_value(v) for v in row] for row in rows])
    return output.getvalue().encode('utf-8')


def intersections(xy,closed=True):
    end = np.roll(xy, -1, axis=0) if closed else xy[1:]
    if not closed:xy=xy[:-1]
    midpoint = (xy+end)/2
    length = np.linalg.norm(end-xy, axis=1)
    tree = cKDTree(midpoint)
    count = 0
    for i, j in tree.query_pairs(float(length.max()) + 1e-9):
        if j-i == 1 or (closed and i == 0 and j == len(xy)-1):
            continue
        a, b = end[i]-xy[i], end[j]-xy[j]
        delta = xy[j]-xy[i]
        cross = a[0]*b[1]-a[1]*b[0]
        if abs(cross) < 1e-12:
            # Nonadjacent collinear overlaps/touches are invalid too.
            if abs(delta[0]*a[1]-delta[1]*a[0]) < 1e-10:
                t = np.dot(delta, a)/np.dot(a, a)
                u = np.dot(end[j]-xy[i], a)/np.dot(a, a)
                count += int(max(min(t, u), 0) <= min(max(t, u), 1))
            continue
        t = (delta[0]*b[1]-delta[1]*b[0])/cross
        u = (delta[0]*a[1]-delta[1]*a[0])/cross
        if -1e-10 <= t <= 1+1e-10 and -1e-10 <= u <= 1+1e-10:
            count += 1
    return count


def validate_candidate(raster, build):
    curve = build['curve']
    # Quarter-pixel checks on the continuous spline, separate from export samples.
    s = np.linspace(0, curve.L, int(np.ceil(curve.L/(raster.resolution/4))), endpoint=False)
    xy, tangent, _, kappa = curve.sample(s)
    bad_cells = [i for i, point in enumerate(xy) if raster.classify(point) != 'free']
    crossings = intersections(xy)
    seam_position = float(np.linalg.norm(curve.evaluate(0)-curve.evaluate(1)))
    seam_tangent = float(np.linalg.norm(curve.evaluate(0, 1)/np.linalg.norm(curve.evaluate(0, 1))
                                        - curve.evaluate(1, 1)/np.linalg.norm(curve.evaluate(1, 1))))
    def curvature(u):
        a, b = curve.evaluate(u, 1), curve.evaluate(u, 2)
        return float((a[0]*b[1]-a[1]*b[0])/np.linalg.norm(a)**3)
    seam_curvature = abs(curvature(0)-curvature(1))
    independent = sum(integrate.quad(lambda u: float(np.linalg.norm(curve.evaluate(u, 1))),
                                    a, b, epsabs=1e-11, epsrel=1e-11)[0]
                      for a, b in zip(curve.u_nodes[:-1], curve.u_nodes[1:]))
    invalid = [i for i, b in enumerate(build['boundaries']) if not b['valid']]
    left = [b['d_left'] for b in build['boundaries'] if b['valid']]
    right = [-b['d_right'] for b in build['boundaries'] if b['valid']]
    checks = dict(
        anchor=True, annular_topology=True,
        centerline_free=len(bad_cells) == 0,
        sampled_nonselfintersection=crossings == 0,
        finite_nondegenerate_derivatives=bool(np.isfinite(tangent).all() and np.isfinite(kappa).all()),
        seam_position=seam_position <= 1e-8,
        seam_tangent=seam_tangent <= 1e-8,
        seam_curvature=seam_curvature <= 1e-7,
        arclength=abs(independent-curve.L) <= max(1e-5, 1e-6*curve.L),
        boundary_samples=len(invalid) == 0)
    return dict(stage='B', stage_b_passed=all(checks.values()),
                status='rejected', planning_allowed=False,
                reason='stages_C_D_not_validated', checks=checks,
                later_checks=dict(frenet_roundtrip='not_run', projection_uniqueness='not_run',
                                  boundary_interpolation='not_run', valid_domain='not_run',
                                  interactive_inspection='not_run'),
                L_m=curve.L, centerline_samples=len(build['s']),
                occupancy_check_samples=len(s), occupancy_check_step_m=curve.L/len(s),
                nonfree_indices=bad_cells, sampled_intersections=crossings,
                invalid_boundary_indices=invalid,
                seam_position_error_m=seam_position, seam_tangent_error=seam_tangent,
                seam_curvature_error_inv_m=seam_curvature,
                independent_arclength_m=independent,
                arclength_difference_m=abs(independent-curve.L),
                min_left_width_m=min(left) if left else None,
                min_right_width_m=min(right) if right else None,
                curvature_abs_max_inv_m=float(np.max(np.abs(kappa))),
                curvature_abs_p95_inv_m=float(np.percentile(np.abs(kappa), 95)),
                refinement=build.get('refinement'),
                anchor=build['anchor'],
                limitations=['sampled topology checks are not a continuous proof',
                             'boundary CSV samples are candidates; interpolation unvalidated',
                             'no full Frenet valid domain or controller safety validation'])


def payloads(build):
    center = np.column_stack((build['xy'], build['s'], build['psi'], build['kappa']))
    boundary_rows = []
    for s, b in zip(build['s'], build['boundaries']):
        left = [None, None] if b['left'] is None else b['left']
        right = [None, None] if b['right'] is None else b['right']
        boundary_rows.append([s, *left, b['d_left'], *right, b['d_right'],
                              b['left_kind'], b['right_kind'], int(b['valid']), b['reason']])
    return {
        'centerline.csv': csv_bytes(['x_m','y_m','s_m','psi_rad','kappa_inv_m'], center),
        'boundaries.csv': csv_bytes(['s_m','left_x_m','left_y_m','d_left_m','right_x_m','right_y_m',
                                     'd_right_m','left_kind','right_kind','valid','reason'], boundary_rows),
        'waypoints_ros.csv': csv_bytes(['x','y'], build['xy']),
        'reference_curve.json': json_bytes(build['curve'].to_dict()),
        'validity.json': json_bytes(dict(validated=False, intervals=[], reason='stage_C_D_pending'))}


def save_geometry(raster, build, output_root, map_id, validation):
    if not isinstance(map_id, str) or not map_id or Path(map_id).name != map_id or map_id in ('.', '..'):
        raise ValueError('map_id must be one path component')
    files = payloads(build)
    manifest = dict(schema_version=1, artifact_stage='B', map_id=map_id, frame_id='map',
                    units=dict(position='m', heading='rad', curvature='1/m'), closed=build['curve'].closed,
                    L_m=build['curve'].L, source_hashes=raster.source_hashes,
                    source_paths=dict(yaml=raster.yaml_path.name if raster.yaml_path else None,
                                      image=raster.config.get('image')),
                    map_config=raster.config, image_width=raster.width, image_height=raster.height,
                    anchor=build['anchor'], extraction=build['options'],
                    software=dict(python=platform.python_version(), numpy=np.__version__, scipy=scipy.__version__),
                    file_hashes={name: hashlib.sha256(content).hexdigest() for name, content in files.items()})
    geometry_id = hashlib.sha256(json_bytes(manifest)).hexdigest()
    target = Path(output_root)/map_id/geometry_id
    target.mkdir(parents=True, exist_ok=False)
    manifest['geometry_id'] = geometry_id
    for name, content in files.items():
        with (target/name).open('xb') as stream:
            stream.write(content)
    (target/'manifest.json').write_bytes(json_bytes(manifest))
    validation = dict(validation, geometry_id=geometry_id)
    (target/'validation.json').write_bytes(json_bytes(validation))
    return target


def verify_artifacts(directory):
    root = Path(directory)
    manifest = json.loads((root/'manifest.json').read_text())
    if manifest.get('schema_version') != 1 or manifest.get('artifact_stage') != 'B':
        raise ValueError('unsupported geometry schema or stage')
    claimed = manifest.pop('geometry_id')
    if hashlib.sha256(json_bytes(manifest)).hexdigest() != claimed:
        raise ValueError('geometry_mismatch: manifest hash')
    if set(manifest['file_hashes']) != set(['centerline.csv','boundaries.csv','waypoints_ros.csv',
                                         'reference_curve.json','validity.json']):
        raise ValueError('invalid artifact file set')
    for name, expected in manifest['file_hashes'].items():
        if sha256(root/name) != expected:
            raise ValueError('geometry_mismatch: ' + name)
    curve_data = json.loads((root/'reference_curve.json').read_text())
    from .extraction import ReferenceCurve
    curve = ReferenceCurve((curve_data['knots'], curve_data['coefficients'], curve_data['degree']),
                           curve_data['start_u'],curve_data.get('periodic',True))
    if curve.closed != manifest.get('closed',True):raise ValueError('geometry_mismatch: topology')
    if not np.isclose(curve.L, manifest['L_m'], rtol=1e-12, atol=1e-10):
        raise ValueError('geometry_mismatch: curve length')
    center = np.loadtxt(root/'centerline.csv', delimiter=',', skiprows=1, ndmin=2)
    waypoints = np.loadtxt(root/'waypoints_ros.csv', delimiter=',', skiprows=1, ndmin=2)
    if center.shape[1] != 5 or not np.isfinite(center).all() or center[0, 2] != 0 or not np.all(np.diff(center[:, 2]) > 0) or (center[-1, 2] >= curve.L if curve.closed else center[-1, 2] > curve.L):
        raise ValueError('invalid centerline schema')
    np.testing.assert_array_equal(center[:, :2], waypoints)
    xy, _, psi, kappa = curve.sample(center[:, 2])
    np.testing.assert_allclose(center[:, :2], xy, atol=1e-10, rtol=1e-12)
    np.testing.assert_allclose(center[:, 3:], np.column_stack((psi, kappa)), atol=1e-10, rtol=1e-12)
    return dict(passed=True, geometry_id=claimed, samples=len(center), L_m=curve.L,
                planning_allowed=False, reason='stage_C_D_pending')


def render_overlay(raster, build, target):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    # Plot in pixel-centre coordinates; rotated maps remain exactly aligned.
    def pixel(points):
        q = raster.map_to_local(np.asarray(points))/raster.resolution
        return np.column_stack((q[:, 0]-.5, raster.height-q[:, 1]-.5))
    fig, ax = plt.subplots(figsize=(15, 7))
    ax.imshow(raster.gray, cmap='gray', vmin=0, vmax=255)
    for key, color in [('left','#ef4444'), ('right','#3b82f6')]:
        points = [b[key] if b[key] is not None else [np.nan, np.nan] for b in build['boundaries']]
        p = pixel(points); ax.plot(p[:, 0], p[:, 1], color=color, lw=.7, label=key+' boundary candidate')
    p = pixel(build['xy']); ax.plot(p[:, 0], p[:, 1], color='#22c55e', lw=.8, label='new centreline')
    ax.scatter(*p[0], c='#f59e0b', marker='*', s=100, label='s=0')
    xy, tangent, _, _ = build['curve'].sample([0.])
    arrow = pixel(np.vstack((xy[0], xy[0]+tangent[0]*2)))
    ax.annotate('', xy=arrow[1], xytext=arrow[0], arrowprops=dict(arrowstyle='->', color='#f59e0b', lw=2))
    ax.set_title('Stage B: map geometry candidates (Frenet domain pending)')
    ax.set_xlabel('image column (pixel centres)'); ax.set_ylabel('image row (pixel centres)')
    ax.legend(loc='upper right', fontsize=8); fig.tight_layout()
    fig.savefig(Path(target)/'overlay.png', dpi=160)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map-yaml', type=Path)
    parser.add_argument('--anchor-csv', type=Path)
    parser.add_argument('--map-id')
    parser.add_argument('--output-root', type=Path, default=Path('Data/research/geometry'))
    parser.add_argument('--verify-only', type=Path)
    parser.add_argument('--step', type=float, default=.05)
    parser.add_argument('--centerline-method', choices=['nearest_wall_midpoint_v1','equidistant_periodic_v1'],
                        default='nearest_wall_midpoint_v1')
    parser.add_argument('--centering-smoothing-m', type=float, default=.3)
    args = parser.parse_args()
    if args.verify_only:
        print(json.dumps(verify_artifacts(args.verify_only), indent=2)); return
    if args.map_yaml is None or args.anchor_csv is None:
        parser.error('--map-yaml and --anchor-csv are required')
    raster = load_map(args.map_yaml)
    old = np.loadtxt(args.anchor_csv, delimiter=',', comments='#', ndmin=2)
    if len(old) < 2 or old.shape[1] < 2 or not np.isfinite(old[:, :2]).all():
        raise ValueError('invalid anchor CSV')
    heading = float(np.arctan2(*(old[1, :2]-old[0, :2])[::-1]))
    build = build_geometry(raster, ExtractionOptions(output_step_m=args.step,
                           method=args.centerline_method,centering_smoothing_m=args.centering_smoothing_m),
                           old[0, :2], heading)
    validation = validate_candidate(raster, build)
    # Anchor source is part of provenance, not only its extracted numerical pose.
    raster.source_hashes['anchor_csv'] = sha256(args.anchor_csv)
    raster.config['anchor_csv_source'] = args.anchor_csv.name
    target = save_geometry(raster, build, args.output_root, args.map_id or args.map_yaml.stem, validation)
    render_overlay(raster, build, target)
    verification = verify_artifacts(target)
    print(json.dumps(dict(output=str(target), validation=validation, artifact_verification=verification), indent=2))
    if not validation['stage_b_passed']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
