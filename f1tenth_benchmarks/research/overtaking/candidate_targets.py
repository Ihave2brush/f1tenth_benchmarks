"""
Generate five candidate target points in Frenet coordinates.

This version:
1. Converts the current vehicle position from Cartesian to Frenet.
2. Looks ahead a fixed distance.
3. Reads the road boundaries at the target position.
4. Generates five evenly spaced lateral target points.
5. Checks each target with the Frenet geometry interface.

Complete trajectories are not generated yet.
"""

import argparse
import json

import numpy as np

from f1tenth_benchmarks.research.core.geometry.frenet import load_geometry


def generate_five_targets(
    geometry,
    x,
    y,
    yaw=None,
    lookahead_m=40.0,
    safety_margin_m=0.20,
):

    # ---------------------------------------------------------
    # Validate input parameters
    lookahead_m = float(lookahead_m)
    safety_margin_m = float(safety_margin_m)

    if not np.isfinite(lookahead_m) or lookahead_m < 0:
        raise ValueError(
            "lookahead_m must be finite and non-negative."
        )

    if (
        not np.isfinite(safety_margin_m)
        or safety_margin_m < 0
    ):
        raise ValueError(
            "safety_margin_m must be finite and non-negative."
        )

    # ---------------------------------------------------------
    # Convert current vehicle position to Frenet
    start = geometry.to_frenet([x, y], yaw=yaw)

    if not start["valid"]:
        raise ValueError(
            "Current vehicle position cannot be converted to Frenet. "
            f"Reason: {start['reason']}"
        )

    s0 = float(start["s_wrapped"])
    d0 = float(start["d"])

    # ---------------------------------------------------------
    # Calculate target longitudinal position
    s_target_raw = s0 + lookahead_m

    if geometry.curve.closed:
        s_target = s_target_raw % geometry.curve.L

    else:
        if s_target_raw > geometry.curve.L:
            raise ValueError(
                "Target point exceeds the end of the open reference line."
            )

        s_target = s_target_raw

    # ---------------------------------------------------------
    # Read road boundaries at target s
    boundary = geometry.boundaries(s_target)

    if not boundary["valid"]:
        raise ValueError(
            "Cannot obtain valid road boundaries at target s. "
            f"Reason: {boundary['reason']}"
        )

    d_left = float(boundary["d_left"])
    d_right = float(boundary["d_right"])

    # ---------------------------------------------------------
    # Apply safety margin
    d_min = d_right + safety_margin_m
    d_max = d_left - safety_margin_m

    if d_min >= d_max:
        raise ValueError(
            "Road is too narrow after applying the safety margin."
        )

    # ---------------------------------------------------------
    # Generate five evenly spaced lateral targets
    lateral_targets = np.linspace(
        d_min,
        d_max,
        5,
    )

    candidates = []

    for index, d_target in enumerate(
        lateral_targets,
        start=1,
    ):
        d_target = float(d_target)

        # Exact geometry check for this Frenet source coordinate
        checked = geometry.to_cartesian(
            s_target,
            d_target,
        )

        xy = checked.get("xy")

        if xy is None:
            x_target = None
            y_target = None
        else:
            x_target = float(xy[0])
            y_target = float(xy[1])

        candidates.append(
            {
                "candidate_id": index,
                "s": float(s_target),
                "d": d_target,
                "x": x_target,
                "y": y_target,
                "valid": bool(checked["valid"]),
                "reason": checked["reason"],
                "planning_allowed": bool(
                    checked.get(
                        "planning_allowed",
                        False,
                    )
                ),
                "domain_verified": bool(
                    checked.get(
                        "domain_verified",
                        False,
                    )
                ),
            }
        )

    # ---------------------------------------------------------
    # Count valid local target queries
    valid_candidate_count = sum(
        candidate["valid"]
        for candidate in candidates
    )

    planning_allowed = bool(
        start.get(
            "planning_allowed",
            getattr(
                geometry,
                "planning_allowed",
                False,
            ),
        )
    )

    domain_verified = bool(
        start.get(
            "domain_verified",
            False,
        )
    )

    # ---------------------------------------------------------
    # Return result
    return {
        "start": {
            "x": float(x),
            "y": float(y),
            "s": s0,
            "d": d0,
        },
        "lookahead_m": lookahead_m,
        "target_s": float(s_target),
        "boundary": {
            "d_right": d_right,
            "d_left": d_left,
        },
        "usable_range": {
            "d_min": d_min,
            "d_max": d_max,
        },
        "safety_margin_m": safety_margin_m,
        "valid_candidate_count": int(
            valid_candidate_count
        ),
        "planning_allowed": planning_allowed,
        "domain_verified": domain_verified,
        "inspection_only": (
            not planning_allowed
            or not domain_verified
        ),
        "candidates": candidates,
    }


def main():

    parser = argparse.ArgumentParser(
        description="Generate five Frenet candidate target points."
    )

    parser.add_argument(
        "--geometry-dir",
        required=True,
        help="Path to Frenet geometry directory.",
    )

    parser.add_argument(
        "--map-yaml",
        required=True,
        help="Path to map YAML.",
    )

    parser.add_argument(
        "--x",
        required=True,
        type=float,
        help="Current vehicle x position [m].",
    )

    parser.add_argument(
        "--y",
        required=True,
        type=float,
        help="Current vehicle y position [m].",
    )

    parser.add_argument(
        "--yaw",
        type=float,
        default=None,
        help="Current vehicle yaw [rad].",
    )

    parser.add_argument(
        "--lookahead",
        type=float,
        default=40.0,
        help="Lookahead distance [m].",
    )

    parser.add_argument(
        "--margin",
        type=float,
        default=0.20,
        help="Safety margin from road boundary [m].",
    )

    args = parser.parse_args()

    # ---------------------------------------------------------
    # Load Frenet geometry for inspection
    geometry = load_geometry(
        args.geometry_dir,
        args.map_yaml,
        inspect=True,
    )

    # ---------------------------------------------------------
    # Generate five target points
    result = generate_five_targets(
        geometry=geometry,
        x=args.x,
        y=args.y,
        yaw=args.yaw,
        lookahead_m=args.lookahead,
        safety_margin_m=args.margin,
    )

    print(
        json.dumps(
            result,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
