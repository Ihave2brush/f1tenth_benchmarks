
"""
Generate five adaptive candidate paths using local road width.

V5:
- Query road width along the entire planning distance.
- Transition from the current lateral position to five road fractions.
- Check each Frenet point using exact geometry queries.
- Keep the V3 result format for V4 visualization.

This is sampled inspection, not a vehicle safety certification.
"""

import argparse
import json

import numpy as np

from f1tenth_benchmarks.research.core.geometry.frenet import load_geometry
from f1tenth_benchmarks.research.overtaking.candidate_validation import (
    MAX_SAMPLES,
)


def adaptive_blend(t):
    return 3.0 * t**2 - 2.0 * t**3


def generate_adaptive_paths(
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
    x = float(x)
    y = float(y)
    lookahead_m = float(lookahead_m)
    safety_margin_m = float(safety_margin_m)
    sample_step_m = float(sample_step_m)

    if yaw is not None:
        yaw = float(yaw)

    if not np.isfinite([x, y]).all():
        raise ValueError("Vehicle position must be finite.")

    if yaw is not None and not np.isfinite(yaw):
        raise ValueError("yaw must be finite.")

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

    ratio = lookahead_m / sample_step_m

    if not np.isfinite(ratio) or ratio > MAX_SAMPLES - 1:
        raise ValueError(
            f"Path would exceed {MAX_SAMPLES} samples."
        )

    # ---------------------------------------------------------
    # Convert vehicle position to Frenet
    start = geometry.to_frenet([x, y], yaw=yaw)

    if not start["valid"]:
        raise ValueError(
            "Current vehicle position cannot be converted to Frenet. "
            f"Reason: {start.get('reason')}"
        )

    s0 = float(start["s_wrapped"])
    d0 = float(start["d"])

    track_length = float(geometry.curve.L)

    if not np.isfinite(track_length) or track_length <= 0:
        raise ValueError("Invalid reference line length.")

    if geometry.curve.closed:
        if lookahead_m >= track_length:
            raise ValueError(
                "Lookahead must be shorter than one full lap."
            )
    elif s0 + lookahead_m > track_length:
        raise ValueError(
            "Lookahead exceeds the open reference line."
        )

    # ---------------------------------------------------------
    # Check initial road width
    initial_boundary = geometry.boundaries(s0)

    if not initial_boundary["valid"]:
        raise ValueError(
            "Invalid road boundary at vehicle start."
        )

    initial_min = (
        float(initial_boundary["d_right"]) + safety_margin_m
    )
    initial_max = (
        float(initial_boundary["d_left"]) - safety_margin_m
    )

    if initial_min >= initial_max:
        raise ValueError(
            "Insufficient road width at vehicle start."
        )

    if not initial_min <= d0 <= initial_max:
        raise ValueError(
            "Vehicle start is outside the usable road range."
        )

    initial_fraction = (
        (d0 - initial_min) / (initial_max - initial_min)
    )

    # ---------------------------------------------------------
    # Create dense sample positions
    segment_count = max(1, int(np.ceil(ratio)))

    t_values = np.linspace(
        0.0,
        1.0,
        segment_count + 1,
    )

    actual_sample_step_m = lookahead_m / segment_count

    # ---------------------------------------------------------
    # Query road boundaries along the path
    sections = []

    for t in t_values:
        t = float(t)
        s_progress = s0 + t * lookahead_m

        if geometry.curve.closed:
            s = s_progress % track_length
        else:
            s = s_progress

        boundary = geometry.boundaries(s)
        boundary_valid = bool(boundary["valid"])

        d_min = None
        d_max = None
        reason = boundary.get("reason")

        if boundary_valid:
            right = boundary.get("d_right")
            left = boundary.get("d_left")

            if (
                right is None
                or left is None
                or not np.isfinite([right, left]).all()
            ):
                boundary_valid = False
                reason = "invalid_boundary"
            else:
                d_min = float(right) + safety_margin_m
                d_max = float(left) - safety_margin_m

                if d_min >= d_max:
                    boundary_valid = False
                    reason = "insufficient_lateral_margin"

        sections.append(
            {
                "t": t,
                "s_progress": float(s_progress),
                "s": float(s),
                "d_min": d_min,
                "d_max": d_max,
                "boundary_valid": boundary_valid,
                "reason": reason,
            }
        )

    # ---------------------------------------------------------
    # Generate five road-width-adaptive paths
    fractions = np.linspace(0.0, 1.0, 5)
    paths = []

    for candidate_id, target_fraction in enumerate(
        fractions, start=1
    ):

        points = []
        first_invalid_sample = None
        first_invalid_reason = None

        for index, section in enumerate(sections):

            t = section["t"]
            blend = adaptive_blend(t)

            lateral_fraction = (
                (1.0 - blend) * initial_fraction
                + blend * float(target_fraction)
            )

            d = None
            x_map = None
            y_map = None
            geometry_valid = False
            margin_valid = False

            reason = section["reason"]

            if section["boundary_valid"]:

                d_min = section["d_min"]
                d_max = section["d_max"]

                d = float(
                    d_min + lateral_fraction * (d_max - d_min)
                )

                margin_valid = bool(
                    d_min <= d <= d_max
                )

                exact = geometry.to_cartesian(
                    section["s"],
                    d,
                )

                if exact.get("geometry_id") != geometry.geometry_id:
                    raise ValueError(
                        "Exact query geometry version mismatch."
                    )

                geometry_valid = bool(exact["valid"])
                reason = exact.get("reason")

                xy = exact.get("xy")

                if xy is not None:
                    xy_array = np.asarray(xy, dtype=float)

                    if (
                        xy_array.shape == (2,)
                        and np.isfinite(xy_array).all()
                    ):
                        x_map = float(xy_array[0])
                        y_map = float(xy_array[1])

                if x_map is None or y_map is None:
                    geometry_valid = False
                    reason = reason or "invalid_geometry"

                if not geometry_valid:
                    reason = reason or "invalid_geometry"

                elif not margin_valid:
                    reason = "insufficient_lateral_margin"

                else:
                    reason = None

            point_valid = (
                section["boundary_valid"]
                and geometry_valid
                and margin_valid
            )

            if not point_valid and first_invalid_sample is None:
                first_invalid_sample = index
                first_invalid_reason = reason or "invalid_sample"

            points.append(
                {
                    "sample_index": index,
                    "t": t,
                    "s_progress": section["s_progress"],
                    "s": section["s"],
                    "d": d,
                    "x": x_map,
                    "y": y_map,
                    "lateral_fraction": float(
                        lateral_fraction
                    ),
                    "d_min": section["d_min"],
                    "d_max": section["d_max"],
                    "boundary_valid": section["boundary_valid"],
                    "margin_valid": margin_valid,
                    "geometry_valid": geometry_valid,
                    "valid": bool(point_valid),
                    "reason": reason,
                }
            )

        # ---------------------------------------------------------
        # Store candidate result
        sampled_valid = first_invalid_sample is None

        target_point = points[-1]

        paths.append(
            {
                "candidate_id": candidate_id,
                "geometry_id": geometry.geometry_id,
                "frame_id": "map",
                "target_fraction": float(target_fraction),
                "target_s": target_point["s"],
                "target_d": target_point["d"],
                "target_valid": bool(target_point["valid"]),
                "sampled_valid": sampled_valid,
                "status": (
                    "sampled_valid"
                    if sampled_valid
                    else "invalid_sample"
                ),
                "reason": first_invalid_reason,
                "first_invalid_sample": first_invalid_sample,
                "points": points,
            }
        )

    # ---------------------------------------------------------
    # Return result
    valid_count = sum(
        path["sampled_valid"] for path in paths
    )

    return {
        "schema_version": 1,
        "method": "adaptive_width_fraction_v1",
        "geometry_id": geometry.geometry_id,
        "frame_id": "map",
        "planning_allowed": False,
        "domain_verified": False,
        "inspection_only": True,
        "continuous_verified": False,
        "footprint_verified": False,
        "dynamics_verified": False,
        "opponents_verified": False,
        "heading_verified": False,
        "verification_level": "sampled_requires_exact_query",
        "start": {
            "x": x,
            "y": y,
            "s": s0,
            "d": d0,
        },
        "initial_fraction": float(initial_fraction),
        "lookahead_m": lookahead_m,
        "safety_margin_m": safety_margin_m,
        "requested_sample_step_m": sample_step_m,
        "actual_sample_step_m": actual_sample_step_m,
        "candidate_count": len(paths),
        "valid_target_count": sum(
            path["target_valid"] for path in paths
        ),
        "sampled_valid_count": int(valid_count),
        "paths": paths,
    }


def main():

    parser = argparse.ArgumentParser(
        description="Generate adaptive-width Frenet paths."
    )

    parser.add_argument("--geometry-dir", required=True)
    parser.add_argument("--map-yaml", required=True)

    parser.add_argument("--x", required=True, type=float)
    parser.add_argument("--y", required=True, type=float)
    parser.add_argument("--yaw", type=float, default=None)

    parser.add_argument("--lookahead", type=float, default=40.0)
    parser.add_argument("--margin", type=float, default=0.20)
    parser.add_argument("--sample-step", type=float, default=0.25)

    parser.add_argument(
        "--save",
        default=None,
        help="Optional output figure path.",
    )

    parser.add_argument(
        "--no-show",
        action="store_true",
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
    # Generate adaptive paths
    try:
        result = generate_adaptive_paths(
            geometry=geometry,
            x=args.x,
            y=args.y,
            yaw=args.yaw,
            lookahead_m=args.lookahead,
            safety_margin_m=args.margin,
            sample_step_m=args.sample_step,
        )
    except ValueError as exc:
        parser.error(str(exc))

    # ---------------------------------------------------------
    # Print result summary
    summary = {
        "method": result["method"],
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

    # ---------------------------------------------------------
    # Reuse V4 visualization if requested
    if args.save is not None or not args.no_show:

        from f1tenth_benchmarks.research.overtaking.candidate_visualization import (
            plot_candidate_paths,
        )

        import matplotlib.pyplot as plt

        fig, _ = plot_candidate_paths(geometry, result)

        if args.save is not None:
            fig.savefig(
                args.save,
                dpi=180,
                bbox_inches="tight",
            )
            print(f"Figure saved: {args.save}")

        if args.no_show:
            plt.close(fig)
        else:
            plt.show()


if __name__ == "__main__":
    main()
