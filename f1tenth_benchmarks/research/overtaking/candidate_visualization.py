
"""
Visualize five Frenet candidate paths on the track map.

V4:
- Reuse V3 validation results.
- Plot centerline and sampled road boundaries.
- Show valid/invalid candidate paths and rejection points.
- Optionally save the figure.

This is a visualization tool, not a vehicle controller.
"""

import argparse

import matplotlib.pyplot as plt
import numpy as np

from f1tenth_benchmarks.research.core.geometry.frenet import load_geometry
from f1tenth_benchmarks.research.overtaking.candidate_validation import (
    validate_candidate_paths,
)


def plot_candidate_paths(geometry, result):

    # ---------------------------------------------------------
    # Check geometry version
    if result["geometry_id"] != geometry.geometry_id:
        raise ValueError("Geometry version mismatch.")

    if result["frame_id"] != "map":
        raise ValueError("Coordinate frame mismatch.")

    fig, ax = plt.subplots(figsize=(11, 8))

    colors = plt.get_cmap("tab10").colors

    # ---------------------------------------------------------
    # Plot reference centerline
    closed = geometry.curve.closed
    track_length = float(geometry.curve.L)

    s_values = np.linspace(
        0.0,
        track_length,
        350,
        endpoint=True,
    )

    center_xy, _, _, _ = geometry.curve.sample(s_values)

    ax.plot(
        center_xy[:, 0],
        center_xy[:, 1],
        color="gray",
        linestyle="--",
        linewidth=1.2,
        label="Reference centerline",
        zorder=2,
    )

    # ---------------------------------------------------------
    # Plot sampled road boundaries
    boundary_s = np.linspace(
        0.0,
        track_length,
        240,
        endpoint=not closed,
    )

    left_points = []
    right_points = []

    for s in boundary_s:
        boundary = geometry.boundaries(float(s))

        if not boundary["valid"]:
            continue

        left = boundary.get("left_xy")
        right = boundary.get("right_xy")

        if left is not None:
            left_points.append(left)

        if right is not None:
            right_points.append(right)

    if left_points:
        left_array = np.asarray(left_points, dtype=float)

        ax.scatter(
            left_array[:, 0],
            left_array[:, 1],
            color="tab:red",
            s=7,
            alpha=0.65,
            label="Left boundary samples",
            zorder=1,
        )

    if right_points:
        right_array = np.asarray(right_points, dtype=float)

        ax.scatter(
            right_array[:, 0],
            right_array[:, 1],
            color="tab:orange",
            s=7,
            alpha=0.65,
            label="Right boundary samples",
            zorder=1,
        )

    # ---------------------------------------------------------
    # Plot five candidate paths
    for path in result["paths"]:

        candidate_id = path["candidate_id"]
        sampled_valid = bool(path["sampled_valid"])
        points = path["points"]

        color = colors[(candidate_id - 1) % len(colors)]

        status = "sampled OK" if sampled_valid else "rejected"

        label = f"Candidate {candidate_id}: {status}"

        if not points:
            reason = path.get("reason") or "invalid_target"

            ax.plot(
                [],
                [],
                color=color,
                linestyle="--",
                label=f"Candidate {candidate_id}: {reason}",
            )
            continue

        # ---------------------------------------------------------
        # Keep invalid coordinates visible without joining gaps
        xs = []
        ys = []

        for point in points:
            x_map = point.get("x")
            y_map = point.get("y")

            if (
                x_map is not None
                and y_map is not None
                and np.isfinite([x_map, y_map]).all()
            ):
                xs.append(float(x_map))
                ys.append(float(y_map))
            else:
                xs.append(np.nan)
                ys.append(np.nan)

        linestyle = "-" if sampled_valid else "--"

        ax.plot(
            xs,
            ys,
            color=color,
            linestyle=linestyle,
            linewidth=1.8,
            label=label,
            zorder=3,
        )

        # ---------------------------------------------------------
        # Mark first invalid point
        if not sampled_valid:
            invalid_index = path.get("first_invalid_sample")

            if (
                invalid_index is not None
                and 0 <= invalid_index < len(points)
            ):
                bad_point = points[invalid_index]

                bad_x = bad_point.get("x")
                bad_y = bad_point.get("y")

                if (
                    bad_x is not None
                    and bad_y is not None
                    and np.isfinite([bad_x, bad_y]).all()
                ):
                    ax.scatter(
                        bad_x,
                        bad_y,
                        color="red",
                        marker="x",
                        s=90,
                        linewidths=2.0,
                        zorder=5,
                    )

                    ax.annotate(
                        f"C{candidate_id}: "
                        f"{bad_point.get('reason') or 'invalid'}",
                        (bad_x, bad_y),
                        xytext=(6, 8),
                        textcoords="offset points",
                        fontsize=8,
                        color="darkred",
                    )

    # ---------------------------------------------------------
    # Mark vehicle start position
    start = result["start"]

    ax.scatter(
        start["x"],
        start["y"],
        color="black",
        marker="o",
        s=65,
        label="Vehicle start",
        zorder=6,
    )

    # ---------------------------------------------------------
    # Figure settings
    valid_count = result["sampled_valid_count"]
    total_count = result["candidate_count"]

    ax.set_title(
        f"Five Frenet Candidate Paths "
        f"(sampled OK: {valid_count}/{total_count})"
    )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")

    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)

    ax.legend(
        loc="best",
        fontsize=8,
    )

    plt.tight_layout()

    return fig, ax


def main():

    parser = argparse.ArgumentParser(
        description="Visualize five Frenet candidate paths."
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

    parser.add_argument(
        "--save",
        default=None,
        help="Optional output image path.",
    )

    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Save or generate without opening a plot window.",
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
    # Generate and validate five paths
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
    # Plot result
    fig, _ = plot_candidate_paths(
        geometry,
        result,
    )

    if args.save is not None:
        fig.savefig(
            args.save,
            dpi=180,
            bbox_inches="tight",
        )
        print(f"Figure saved: {args.save}")

    print(
        f"Sampled-valid paths: "
        f"{result['sampled_valid_count']}/"
        f"{result['candidate_count']}"
    )

    if args.no_show:
        plt.close(fig)
    else:
        plt.show()


if __name__ == "__main__":
    main()
