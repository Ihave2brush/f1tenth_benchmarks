
"""
Generate five Frenet candidate paths using cubic Hermite interpolation.

V2:
- Read five targets from candidate_targets.py.
- Generate 20 Frenet points for each valid target.
- Preserve invalid targets and geometry metadata.
- Leave full-path validation to V3.
"""

import argparse
import json

import numpy as np

from f1tenth_benchmarks.research.core.geometry.frenet import load_geometry
from f1tenth_benchmarks.research.overtaking.candidate_targets import (
    generate_five_targets,
)


def hermite_lateral(d_start, d_target, t):

    # ---------------------------------------------------------
    # Cubic Hermite interpolation
    blend = 3.0 * t**2 - 2.0 * t**3

    return d_start + (d_target - d_start) * blend


def generate_candidate_paths(
    geometry,
    x,
    y,
    yaw=None,
    lookahead_m=40.0,
    safety_margin_m=0.20,
    num_points=20,
):

    # ---------------------------------------------------------
    # Validate path parameters
    lookahead_m = float(lookahead_m)

    if not np.isfinite(lookahead_m) or lookahead_m <= 0:
        raise ValueError(
            "lookahead_m must be finite and positive."
        )

    if isinstance(num_points, (bool, np.bool_)) or not isinstance(
        num_points, (int, np.integer)
    ):
        raise ValueError("num_points must be an integer >= 2.")

    if num_points < 2:
        raise ValueError("num_points must be at least 2.")

    track_length = float(geometry.curve.L)

    if not np.isfinite(track_length) or track_length <= 0:
        raise ValueError("Invalid reference line length.")

    if geometry.curve.closed and lookahead_m >= track_length:
        raise ValueError(
            "Lookahead must be shorter than one full lap. "
            f"Track length={track_length:.3f} m, "
            f"requested={lookahead_m:.3f} m."
        )

    # ---------------------------------------------------------
    # Get five candidate targets from V1
    target_result = generate_five_targets(
        geometry=geometry,
        x=x,
        y=y,
        yaw=yaw,
        lookahead_m=lookahead_m,
        safety_margin_m=safety_margin_m,
    )

    s0 = float(target_result["start"]["s"])
    d0 = float(target_result["start"]["d"])

    geometry_id = target_result["geometry_id"]
    frame_id = target_result["frame_id"]

    # ---------------------------------------------------------
    # Check geometry metadata
    if geometry_id != geometry.geometry_id or frame_id != "map":
        raise ValueError("Geometry version or frame mismatch.")

    # ---------------------------------------------------------
    # Generate normalized sample positions
    t_values = np.linspace(
        0.0,
        1.0,
        num_points,
    )

    paths = []

    # ---------------------------------------------------------
    # Generate a path for each target
    for candidate in target_result["candidates"]:

        candidate_id = candidate["candidate_id"]
        d_target = float(candidate["d"])
        target_valid = bool(candidate["valid"])

        if (
            candidate["geometry_id"] != geometry_id
            or candidate["frame_id"] != frame_id
        ):
            raise ValueError(
                f"Candidate {candidate_id} geometry mismatch."
            )

        # ---------------------------------------------------------
        # Preserve invalid targets
        if not target_valid:

            paths.append(
                {
                    "candidate_id": candidate_id,
                    "geometry_id": geometry_id,
                    "frame_id": frame_id,
                    "target_s": float(candidate["s"]),
                    "target_d": d_target,
                    "target_valid": False,
                    "path_valid": False,
                    "status": "invalid_target",
                    "reason": "invalid_target",
                    "target_reason": candidate.get("reason"),
                    "points": [],
                }
            )

            continue

        # ---------------------------------------------------------
        # Generate Hermite path points
        points = []

        for t in t_values:

            s_progress = s0 + float(t) * lookahead_m

            if geometry.curve.closed:
                s = s_progress % track_length
            else:
                s = s_progress

            d = hermite_lateral(
                d_start=d0,
                d_target=d_target,
                t=float(t),
            )

            points.append(
                {
                    "t": float(t),
                    "s_progress": float(s_progress),
                    "s": float(s),
                    "d": float(d),
                }
            )

        # ---------------------------------------------------------
        # Store generated path
        paths.append(
            {
                "candidate_id": candidate_id,
                "geometry_id": geometry_id,
                "frame_id": frame_id,
                "target_s": float(candidate["s"]),
                "target_d": d_target,
                "target_valid": True,
                "path_valid": None,
                "status": "generated_unvalidated",
                "reason": None,
                "target_reason": None,
                "points": points,
            }
        )

    # ---------------------------------------------------------
    # Count generated paths
    generated_count = sum(
        path["status"] == "generated_unvalidated"
        for path in paths
    )

    # ---------------------------------------------------------
    # Return result
    return {
        "geometry_id": geometry_id,
        "frame_id": frame_id,
        "planning_allowed": target_result["planning_allowed"],
        "domain_verified": target_result["domain_verified"],
        "inspection_only": True,
        "start": target_result["start"],
        "lookahead_m": lookahead_m,
        "num_points": int(num_points),
        "safety_margin_m": float(safety_margin_m),
        "boundary_at_target": target_result["boundary"],
        "usable_range_at_target": target_result["usable_range"],
        "candidate_count": len(paths),
        "valid_target_count": target_result["valid_candidate_count"],
        "generated_path_count": int(generated_count),
        "paths": paths,
    }


def main():

    parser = argparse.ArgumentParser(
        description="Generate five Frenet candidate paths."
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
        "--points",
        type=int,
        default=20,
    )

    args = parser.parse_args()

    # ---------------------------------------------------------
    # Load Frenet geometry
    geometry = load_geometry(
        args.geometry_dir,
        args.map_yaml,
        inspect=True,
    )

    # ---------------------------------------------------------
    # Generate candidate paths
    result = generate_candidate_paths(
        geometry=geometry,
        x=args.x,
        y=args.y,
        yaw=args.yaw,
        lookahead_m=args.lookahead,
        safety_margin_m=args.margin,
        num_points=args.points,
    )

    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
