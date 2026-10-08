
"""
Validate five Frenet candidate paths on variable-width tracks.

V3:
- Reuse V2 candidate targets and Hermite interpolation.
- Resample paths at a smaller interval.
- Check local boundaries and exact Frenet geometry.
- Record invalid samples and rejection reasons.

Sampled validity does not guarantee continuous vehicle safety.
"""

import argparse
import json

import numpy as np

from f1tenth_benchmarks.research.core.geometry.frenet import load_geometry
from f1tenth_benchmarks.research.overtaking.candidate_paths import (
    generate_candidate_paths,
    hermite_lateral,
)


def validate_candidate_paths(
    geometry,
    x,
    y,
    yaw=None,
    lookahead_m=40.0,
    safety_margin_m=0.20,
    sample_step_m=0.25,
):

    # ---------------------------------------------------------
    # Validate input parameters
    lookahead_m = float(lookahead_m)
    safety_margin_m = float(safety_margin_m)
    sample_step_m = float(sample_step_m)

    if not np.isfinite(lookahead_m) or lookahead_m <= 0:
        raise ValueError(
            "lookahead_m must be positive and finite."
        )

    if not np.isfinite(safety_margin_m) or safety_margin_m < 0:
        raise ValueError(
            "safety_margin_m must be finite and non-negative."
        )

    if not np.isfinite(sample_step_m) or sample_step_m <= 0:
        raise ValueError(
            "sample_step_m must be positive and finite."
        )

    # ---------------------------------------------------------
    # Generate V2 paths
    base_result = generate_candidate_paths(
        geometry=geometry,
        x=x,
        y=y,
        yaw=yaw,
        lookahead_m=lookahead_m,
        safety_margin_m=safety_margin_m,
        num_points=20,
    )

    geometry_id = base_result["geometry_id"]
    frame_id = base_result["frame_id"]

    if geometry_id != geometry.geometry_id or frame_id != "map":
        raise ValueError("Geometry version or frame mismatch.")

    s0 = float(base_result["start"]["s"])
    d0 = float(base_result["start"]["d"])
    track_length = float(geometry.curve.L)

    # ---------------------------------------------------------
    # Create denser sampling positions
    segment_count = max(
        1,
        int(np.ceil(lookahead_m / sample_step_m)),
    )

    t_values = np.linspace(
        0.0,
        1.0,
        segment_count + 1,
    )

    actual_sample_step_m = lookahead_m / segment_count

    validated_paths = []

    # ---------------------------------------------------------
    # Validate each candidate path
    for path in base_result["paths"]:

        candidate_id = path["candidate_id"]
        target_valid = bool(path["target_valid"])

        if (
            path["geometry_id"] != geometry_id
            or path["frame_id"] != frame_id
        ):
            raise ValueError(
                f"Candidate {candidate_id} geometry mismatch."
            )

        # ---------------------------------------------------------
        # Preserve invalid V1 targets
        if not target_valid:
            validated_paths.append(
                {
                    "candidate_id": candidate_id,
                    "geometry_id": geometry_id,
                    "frame_id": frame_id,
                    "target_s": path["target_s"],
                    "target_d": path["target_d"],
                    "target_valid": False,
                    "sampled_valid": False,
                    "status": "invalid_target",
                    "reason": (
                        path.get("target_reason")
                        or "invalid_target"
                    ),
                    "first_invalid_sample": None,
                    "points": [],
                }
            )
            continue

        # ---------------------------------------------------------
        # Check the entire Hermite path
        checked_points = []
        first_invalid_sample = None
        first_invalid_reason = None

        for index, t in enumerate(t_values):

            t = float(t)
            s_progress = s0 + t * lookahead_m

            if geometry.curve.closed:
                s = s_progress % track_length
            else:
                s = s_progress

            d = float(
                hermite_lateral(
                    d_start=d0,
                    d_target=float(path["target_d"]),
                    t=t,
                )
            )

            # ---------------------------------------------------------
            # Query local road boundaries
            boundary = geometry.boundaries(s)

            boundary_valid = bool(boundary["valid"])

            d_left = None
            d_right = None
            d_min = None
            d_max = None
            margin_valid = False

            if boundary_valid:
                left = boundary.get("d_left")
                right = boundary.get("d_right")

                if (
                    left is not None
                    and right is not None
                    and np.isfinite([left, right]).all()
                ):
                    d_left = float(left)
                    d_right = float(right)

                    d_min = d_right + safety_margin_m
                    d_max = d_left - safety_margin_m

                    margin_valid = bool(
                        d_min <= d <= d_max
                    )
                else:
                    boundary_valid = False

            # ---------------------------------------------------------
            # Check exact Frenet geometry
            exact = geometry.to_cartesian(s, d)
            geometry_valid = bool(exact["valid"])

            xy = exact.get("xy")

            x_map = None
            y_map = None

            if xy is not None:
                xy_array = np.asarray(xy, dtype=float)

                if (
                    xy_array.shape == (2,)
                    and np.isfinite(xy_array).all()
                ):
                    x_map = float(xy_array[0])
                    y_map = float(xy_array[1])

            # ---------------------------------------------------------
            # Determine sample validity
            if not boundary_valid:
                reason = (
                    boundary.get("reason")
                    or "invalid_boundary"
                )

            elif not geometry_valid:
                reason = (
                    exact.get("reason")
                    or "invalid_geometry"
                )

            elif not margin_valid:
                reason = "insufficient_lateral_margin"

            else:
                reason = None

            point_valid = reason is None

            if not point_valid and first_invalid_sample is None:
                first_invalid_sample = index
                first_invalid_reason = reason

            checked_points.append(
                {
                    "sample_index": index,
                    "t": t,
                    "s_progress": float(s_progress),
                    "s": float(s),
                    "d": d,
                    "x": x_map,
                    "y": y_map,
                    "d_right": d_right,
                    "d_left": d_left,
                    "d_min": d_min,
                    "d_max": d_max,
                    "boundary_valid": boundary_valid,
                    "margin_valid": margin_valid,
                    "geometry_valid": geometry_valid,
                    "valid": point_valid,
                    "reason": reason,
                }
            )

        # ---------------------------------------------------------
        # Store path validation result
        sampled_valid = first_invalid_sample is None

        validated_paths.append(
            {
                "candidate_id": candidate_id,
                "geometry_id": geometry_id,
                "frame_id": frame_id,
                "target_s": path["target_s"],
                "target_d": path["target_d"],
                "target_valid": True,
                "sampled_valid": sampled_valid,
                "status": (
                    "sampled_valid"
                    if sampled_valid
                    else "invalid_sample"
                ),
                "reason": first_invalid_reason,
                "first_invalid_sample": first_invalid_sample,
                "points": checked_points,
            }
        )

    # ---------------------------------------------------------
    # Count valid sampled paths
    valid_count = sum(
        path["sampled_valid"]
        for path in validated_paths
    )

    return {
        "schema_version": 1,
        "geometry_id": geometry_id,
        "frame_id": frame_id,
        "planning_allowed": False,
        "domain_verified": False,
        "inspection_only": True,
        "continuous_verified": False,
        "footprint_verified": False,
        "start": base_result["start"],
        "lookahead_m": lookahead_m,
        "safety_margin_m": safety_margin_m,
        "requested_sample_step_m": sample_step_m,
        "actual_sample_step_m": actual_sample_step_m,
        "candidate_count": len(validated_paths),
        "valid_target_count": base_result["valid_target_count"],
        "sampled_valid_count": int(valid_count),
        "paths": validated_paths,
    }


def main():

    parser = argparse.ArgumentParser(
        description="Validate five Frenet candidate paths."
    )

    parser.add_argument("--geometry-dir", required=True)
    parser.add_argument("--map-yaml", required=True)

    parser.add_argument("--x", required=True, type=float)
    parser.add_argument("--y", required=True, type=float)
    parser.add_argument("--yaw", type=float, default=None)

    parser.add_argument(
        "--lookahead",
        type=float,
        default=40.0,
    )

    parser.add_argument(
        "--margin",
        type=float,
        default=0.20,
    )

    parser.add_argument(
        "--sample-step",
        type=float,
        default=0.25,
    )

    args = parser.parse_args()

    # ---------------------------------------------------------
    # Load geometry
    geometry = load_geometry(
        args.geometry_dir,
        args.map_yaml,
        inspect=True,
    )

    # ---------------------------------------------------------
    # Validate paths
    result = validate_candidate_paths(
        geometry=geometry,
        x=args.x,
        y=args.y,
        yaw=args.yaw,
        lookahead_m=args.lookahead,
        safety_margin_m=args.margin,
        sample_step_m=args.sample_step,
    )

    # ---------------------------------------------------------
    # Print summary
    summary = {
        "geometry_id": result["geometry_id"],
        "candidate_count": result["candidate_count"],
        "valid_target_count": result["valid_target_count"],
        "sampled_valid_count": result["sampled_valid_count"],
        "actual_sample_step_m": result["actual_sample_step_m"],
        "inspection_only": result["inspection_only"],
        "paths": [
            {
                "candidate_id": path["candidate_id"],
                "sampled_valid": path["sampled_valid"],
                "reason": path["reason"],
                "first_invalid_sample": path["first_invalid_sample"],
            }
            for path in result["paths"]
        ],
    }

    print(json.dumps(summary, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
