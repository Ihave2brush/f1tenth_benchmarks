"""CSV control samples with same-version JSON geometry, for inspection only.

No boundary interpolation is used. Every runtime boundary query traverses the
original map. Loading this bundle never grants planning or vehicle clearance.
"""
import argparse
import csv
from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np

from ..core.geometry import load_geometry
from ..core.geometry.frenet import wrap
from ..core.geometry.map_raster import sha256


class TrackAuditError(ValueError):
    def __init__(self, report):
        super().__init__('CSV/JSON track audit failed')
        self.report = report


@dataclass(frozen=True)
class AuditLimits:
    # Numerical export agreement, not a physical clearance tolerance.
    position_m: float = 1e-8
    heading_rad: float = 1e-8
    curvature_inv_m: float = 1e-8
    boundary_m: float = 1e-7

    def __post_init__(self):
        if any(not np.isfinite(v) or v <= 0 for v in vars(self).values()):
            raise ValueError('audit limits must be finite and positive')


class TrackData:
    """One loaded source bundle shared by controller, generator and recording."""

    def __init__(self, directory, map_yaml, limits=None):
        root = Path(directory)
        # Includes manifest, all artifact hashes and source-map checks; the
        # existing loader also compares CSV x/y/psi/kappa with the JSON curve.
        self.geometry = load_geometry(root, map_yaml, inspect=True)
        self.manifest = json.loads((root/'manifest.json').read_text())
        if self.manifest['frame_id'] != 'map':
            raise ValueError('unsupported track frame')
        self.centerline = np.loadtxt(root/'centerline.csv', delimiter=',', skiprows=1, ndmin=2)
        with (root/'boundaries.csv').open(newline='') as stream:
            reader = csv.DictReader(stream)
            if reader.fieldnames != ['s_m', 'left_x_m', 'left_y_m', 'd_left_m',
                                     'right_x_m', 'right_y_m', 'd_right_m',
                                     'left_kind', 'right_kind', 'valid', 'reason']:
                raise ValueError('invalid boundary header')
            self.boundary_samples = list(reader)
        if len(self.centerline) < 2 or len(self.boundary_samples) != len(self.centerline):
            raise ValueError('boundary/centerline row count mismatch')
        self.control_path = self.centerline[:, :2].copy()
        if self.geometry.curve.closed:
            self.control_path = np.vstack((self.control_path, self.control_path[0]))
        for array in (self.centerline, self.control_path):
            array.setflags(write=False)
        self.provenance = dict(
            geometry_id=self.geometry.geometry_id, frame_id='map',
            data_mode='csv_samples_with_same_version_json',
            reference_method='reference_curve.json', boundary_method='exact_grid_ray',
            manifest_sha256=sha256(root/'manifest.json'),
            file_hashes=dict(self.manifest['file_hashes']),
            source_hashes=dict(self.geometry.raster.source_hashes),
            inspection_only=True, planning_allowed=False, footprint_verified=False)
        self.audit = self._audit(limits or AuditLimits())
        if not self.audit['passed']:
            raise TrackAuditError(self.audit)

    def _audit(self, limits):
        g = self.geometry
        s = self.centerline[:, 2]
        xy, _, psi, kappa = g.curve.sample(s)
        errors = dict(
            position_m=float(np.max(np.linalg.norm(xy-self.centerline[:, :2], axis=1))),
            heading_rad=float(max(abs(wrap(a-b)) for a, b in zip(psi, self.centerline[:, 3]))),
            curvature_inv_m=float(np.max(np.abs(kappa-self.centerline[:, 4]))),
            boundary_m=0.)
        bad_rows = []
        for i, row in enumerate(self.boundary_samples):
            exact = g.boundaries(float(s[i]))
            reason = None
            try:
                if not np.isfinite(float(row['s_m'])) or abs(float(row['s_m'])-s[i]) > limits.position_m:
                    reason = 'boundary_s_mismatch'
                elif row['valid'] != str(int(exact['valid'])) or (row['reason'] or None) != exact['reason']:
                    reason = 'boundary_status_mismatch'
                for side in ('left', 'right'):
                    if row[side+'_kind'] != exact[side+'_kind']:
                        reason = reason or 'boundary_kind_mismatch'
                    expected = exact[side+'_xy']
                    fields = [side+'_x_m', side+'_y_m', 'd_'+side+'_m']
                    values = ([None, None] if expected is None else expected) + [exact['d_'+side]]
                    for key, value in zip(fields, values):
                        if value is None:
                            if row[key] != '':
                                reason = reason or 'boundary_null_mismatch'
                        else:
                            error = abs(float(row[key])-value)
                            if not np.isfinite(error):
                                reason = reason or 'nonfinite_boundary'
                            else:
                                errors['boundary_m'] = max(errors['boundary_m'], error)
                                if error > limits.boundary_m:
                                    reason = reason or 'boundary_value_mismatch'
            except (KeyError, TypeError, ValueError):
                reason = 'invalid_boundary_schema'
            if reason:
                bad_rows.append(dict(index=i, s_m=float(s[i]), reason=reason))

        # Check every intersample interval, including the last-to-first seam.
        # Polyline/linear-boundary differences are diagnostics only: neither is
        # substituted for the exact continuous geometry used by the generator.
        ends = np.r_[s[1:], g.curve.L] if g.curve.closed else s[1:]
        starts = s if g.curve.closed else s[:-1]
        mids = (starts+ends)/2
        mid_xy, _, _, _ = g.curve.sample(mids)
        next_xy = np.roll(self.centerline[:, :2], -1, axis=0) if g.curve.closed else self.centerline[1:, :2]
        chord_xy = (self.centerline[:len(mids), :2]+next_xy)/2
        midpoint_errors = np.linalg.norm(mid_xy-chord_xy, axis=1)
        invalid_intervals = []
        for i, mid in enumerate(mids):
            exact = g.boundaries(float(mid))
            if not exact['valid']:
                invalid_intervals.append(dict(start_s_m=float(starts[i]), end_s_m=float(ends[i]),
                                              reason=exact['reason']))
        # Compare equivalent s values across the seam, including derivatives.
        seam = None
        seam_ok = True
        if g.curve.closed:
            # Use the unwrapped spline endpoints, not reference(L), which
            # already wraps to zero and would hide an endpoint discontinuity.
            positions = g.curve.evaluate([0., 1.])
            first = g.curve.evaluate([0., 1.], 1)
            second = g.curve.evaluate([0., 1.], 2)
            headings = np.arctan2(first[:, 1], first[:, 0])
            curvatures = (first[:, 0]*second[:, 1]-first[:, 1]*second[:, 0])/np.linalg.norm(first, axis=1)**3
            seam = dict(position_m=float(np.linalg.norm(positions[0]-positions[1])),
                        heading_rad=abs(wrap(headings[0]-headings[1])),
                        curvature_inv_m=float(abs(curvatures[0]-curvatures[1])))
            seam_ok = all(v <= getattr(limits, key) for key, v in seam.items())
        checks = dict(sample_agreement=all(v <= getattr(limits, key) for key, v in errors.items()),
                      boundary_rows=not bad_rows, seam=seam_ok)
        return dict(schema_version=1, provenance=self.provenance, passed=all(checks.values()),
                    checks=checks, limits=vars(limits), max_errors=errors,
                    centerline_samples=len(s), boundary_rows=len(self.boundary_samples),
                    rejected_boundary_rows=bad_rows, seam=seam,
                    interval_probe_count=len(mids), invalid_midpoint_intervals=invalid_intervals,
                    csv_chord_midpoint_error_max_m=float(np.max(midpoint_errors)),
                    interval_policy='exact JSON reference and original-map boundary queries; no interpolation',
                    runtime_invalid_policy='reject invalid exact queries; midpoint probes are not a domain proof',
                    planning_allowed=False, domain_verified=False, continuous_verified=False,
                    footprint_verified=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry-dir', required=True, type=Path)
    parser.add_argument('--map-yaml', required=True, type=Path)
    parser.add_argument('--report', required=True, type=Path)
    args = parser.parse_args()
    try:
        report = TrackData(args.geometry_dir, args.map_yaml).audit
    except TrackAuditError as exc:
        report = exc.report
    # Exclusive creation preserves previous evidence.
    with args.report.open('x') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(dict(passed=report['passed'], report=str(args.report))))
    if not report['passed']:
        raise SystemExit(2)


if __name__ == '__main__':
    main()
