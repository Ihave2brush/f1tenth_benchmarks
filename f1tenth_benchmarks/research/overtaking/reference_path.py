"""C3 reference-path schema, distinct from actual-state matched trajectories.

A reference may be offset from the vehicle. It must be identically d=0 with
an independently checked origin and first point. No footprint/dynamics claim.
"""
import math
import numpy as np

REFERENCE_PATH = 'reference_centerline'
STATE_PATH = 'state_matched_trajectory'


def reference_spline_valid(path, length):
    try:
        spline = path['spline']
        coefficients = np.asarray(spline['coefficients'], float)
        knots = np.asarray(spline['knots'], float)
        return bool(path['candidate_id'] == 3 and path.get('path_type') == REFERENCE_PATH
            and spline['degree'] == 5 and coefficients.shape == (6,)
            and knots.shape == (12,) and np.isfinite(coefficients).all() and np.isfinite(knots).all()
            and np.all(coefficients == 0.) and np.array_equal(knots, [0.]*6+[length]*6)
            and path['requested_target_d'] == 0. and path['reference_d'] == 0.
            and np.isfinite(path['reference_s_origin']))
    except (KeyError, TypeError, ValueError):
        return False


def check_reference_path(geometry, path, source_pose, length, max_lateral_error=None, max_heading_error=None):
    if not reference_spline_valid(path, length) or path.get('geometry_id') != geometry.geometry_id:
        return dict(valid=False, reason='invalid_reference_path_contract')
    try:
        projection = geometry.to_frenet(source_pose[:2], yaw=source_pose[2])
        if not projection['valid']:
            return dict(valid=False, reason='invalid_reference_source')
        origin = float(path['reference_s_origin'])
        if geometry.distance_s(origin, projection['s_wrapped']) > 1e-6:
            return dict(valid=False, reason='reference_origin_mismatch')
        reference = geometry.reference(origin)
        first = path['points'][0]
        position_error = math.hypot(first['x']-reference['xy'][0], first['y']-reference['xy'][1])
        heading_error = (first['yaw']-reference['psi']+math.pi) % (2*math.pi)-math.pi
        valid = bool(np.isfinite([position_error, heading_error]).all()
                     and position_error <= 1e-7 and abs(heading_error) <= 1e-7)
        reason = None if valid else 'reference_point_mismatch'
        if valid and ((max_lateral_error is not None and abs(projection['d']) > max_lateral_error) or
                      (max_heading_error is not None and abs(projection['e_psi']) > max_heading_error)):
            valid, reason = False, 'reference_tracking_envelope_exceeded'
        return dict(valid=valid, reason=reason,
            reference_start_position_error_m=position_error,
            reference_start_heading_error_rad=heading_error,
            tracking_lateral_error_m=float(projection['d']),
            tracking_heading_error_rad=float(projection['e_psi']))
    except (KeyError, IndexError, TypeError, ValueError):
        return dict(valid=False, reason='invalid_reference_path_contract')
