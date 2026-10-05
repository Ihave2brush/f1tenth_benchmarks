"""Stage 1: isolated closed speed planning and reproducible comparisons.

Run from the repository root: python -m f1tenth_benchmarks.research.closed_velocity_profile all
Only --replace-stage1 allows replacing the two named research sets and report.
No simulator or plotting is invoked. Upstream helper functions are never patched.
"""
import argparse
import csv
import hashlib
import importlib.metadata
import json
import platform
import subprocess
import shutil
import os
from pathlib import Path

import numpy as np
import trajectory_planning_helpers as tph
import yaml

HEADER = "s,x,y,heading,curvature,velocity,acceleration"
BASE = Path("Data/racelines/mu60/esp_raceline.csv")
SETS = ("mu60_closed", "mu60_closed_regenerated")
REPORT = Path("Data/research/continuous_laps/stage1")


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def protected_hashes():
    roots = [Path("Logs"), Path("Data/min_curve_lines"),
             Path("Data/smooth_centre_lines"), Path("Data/racelines/mu60"),
             Path("Data/raceline_data/mu60"), Path("params"),
             Path("f1tenth_benchmarks/classic_racing/RaceTrackGenerator.py")]
    files = []
    for root in roots:
        files.extend([root] if root.is_file() else sorted(p for p in root.rglob("*") if p.is_file()))
    files.extend(sorted(Path("trajectory_planning_helpers").rglob("*.py")))
    return {str(p): digest(p) for p in files}


def parameters():
    # Saved generator parameters have an argparse.Namespace YAML tag.
    class Loader(yaml.SafeLoader):
        pass
    Loader.add_constructor("tag:yaml.org,2002:python/object:argparse.Namespace",
                           lambda loader, node: loader.construct_mapping(node))
    p = yaml.load(Path("Data/raceline_data/mu60/params.yaml").read_text(), Loader=Loader)
    vehicle = yaml.safe_load(Path("params/vehicle_params.yaml").read_text())
    return p, vehicle


def build_closed_segment_lengths(geometry):
    xy = np.asarray(geometry)[:, 1:3]
    if len(xy) < 3 or not np.isfinite(geometry).all():
        raise ValueError("At least three finite geometry points required")
    ds = np.linalg.norm(np.roll(xy, -1, axis=0) - xy, axis=1)
    if np.any(ds <= 0):
        raise ValueError("Duplicate points / nonpositive closed segment")
    return ds


def compute_closed_acceleration_and_time(v, ds):
    if len(v) != len(ds) or not np.isfinite(v).all() or np.any(v <= 0) or np.any(ds <= 0):
        raise ValueError("Positive N speeds and N segment lengths required")
    extended = np.append(v, v[0])
    ax = tph.calc_ax_profile.calc_ax_profile(extended, ds, eq_length_output=False)
    times = tph.calc_t_profile.calc_t_profile(extended, ds, ax_profile=ax)
    independent = 2 * ds / (v + np.roll(v, -1))
    np.testing.assert_allclose(np.diff(times), independent, rtol=1e-9, atol=1e-10)
    return ax, independent


def corrected_closed_solver(kappa, ds, p, vehicle):
    """Upstream closed forward/backward orchestration with exactly two fixes.

    Scope: the baseline's constant GGV/mu, no drag, exponent=1, no filter.
    Reuse upstream propagation unchanged, without monkeypatching module globals.
    """
    n = len(kappa)
    if len(ds) != n:
        raise ValueError("Closed solver requires N curvature values and N lengths")
    radii = np.divide(1., np.abs(kappa), out=np.full(n, np.inf), where=kappa != 0)
    mu = np.full(n, p["mu"])
    ggv = np.array([[0, p["max_longitudinal_acc"], p["max_lateral_acc"]],
                    [vehicle["max_speed"], p["max_longitudinal_acc"], p["max_lateral_acc"]]])
    # Same lateral initialization as upstream's converged constant-GGV loop.
    initial = np.minimum(np.sqrt(mu * p["max_lateral_acc"] * radii), vehicle["max_speed"])
    twice = lambda a: np.concatenate((a, a), axis=0)
    common = dict(p_ggv=np.repeat(ggv[None, :, :], 2 * n, axis=0),
                  ax_max_machines=ggv[:, :2].copy(), v_max=vehicle["max_speed"],
                  radii=twice(radii), mu=twice(mu), dyn_model_exp=1.,
                  drag_coeff=0., m_veh=vehicle["vehicle_mass"])
    propagate = tph.calc_vel_profile.__solver_fb_acc_profile
    forward = propagate(**common, el_lengths=twice(ds),
                        vx_profile=twice(initial), backwards=False)
    forward = twice(forward[n:])  # Upstream uses settled second forward lap.
    # Bug 1 (upstream lines 414-418): flip(ds) starts with ds[N-1],
    # but reversed velocities start v[N-1] -> v[N-2], whose edge is ds[N-2].
    # roll(ds,+1) before upstream flip aligns every reverse edge, including seam.
    backward = propagate(**common, el_lengths=np.roll(twice(ds), 1),
                         vx_profile=forward, backwards=True)
    # Bug 2 (upstream lines 389-390): after flip back, the settled second
    # reverse traversal occupies the FIRST half in original ordering.
    # Taking the last half selects the unsettled boundary and depends on start.
    return backward[:n].copy()


def solve_closed_velocity_profile(geometry, p, vehicle):
    ds = build_closed_segment_lengths(geometry)
    v = corrected_closed_solver(geometry[:, 4], ds, p, vehicle)
    ax, dt = compute_closed_acceleration_and_time(v, ds)
    return np.column_stack((geometry, v, ax)), ds, dt


def regenerate_minimum_curvature_geometry(p):
    # Match RaceTrackGenerator's numerical calls, without constructing its writer.
    from f1tenth_benchmarks.utils.track_utils import CentreLine
    cl = CentreLine("esp", "Data/smooth_centre_lines/")
    track = np.column_stack((cl.path, cl.widths - p["vehicle_width"] / 2))
    if tph.check_normals_crossing.check_normals_crossing(track, cl.nvecs):
        raise ValueError("Centreline normals cross")
    path_cl = np.row_stack((cl.path, cl.path[0]))
    lengths_cl = np.append(cl.el_lengths, cl.el_lengths[0])
    _, _, matrix, _ = tph.calc_splines.calc_splines(path_cl, lengths_cl)
    alpha, error = tph.opt_min_curv.opt_min_curv(track, cl.nvecs, matrix, 1, 0,
                                               print_debug=True, closed=True)
    result = tph.create_raceline.create_raceline(cl.path, cl.nvecs, alpha, p["raceline_step"])
    xy, spline_s, ds = result[0], result[6], result[8]
    psi, kappa = tph.calc_head_curv_num.calc_head_curv_num(xy, ds, True)
    chord_s = np.insert(np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1)), 0, 0)
    return np.column_stack((chord_s, xy, psi, kappa)), np.column_stack((spline_s, xy, psi, kappa)), float(error)


def validate(profile, p, vehicle):
    ds = build_closed_segment_lengths(profile[:, :5])
    v, ax = profile[:, 5], profile[:, 6]
    expected_ax, dt = compute_closed_acceleration_and_time(v, ds)
    np.testing.assert_allclose(ax, expected_ax, rtol=1e-12, atol=1e-12)
    lateral = v ** 2 * np.abs(profile[:, 4])
    # Same combined-force exponent (1) and constant GGV as the helper solver.
    lateral_limit = p["mu"] * p["max_lateral_acc"]
    longitudinal_limit = p["mu"] * p["max_longitudinal_acc"]
    capacity = longitudinal_limit * np.maximum(0, 1 - lateral / lateral_limit)
    next_capacity = np.roll(capacity, -1)
    allowable = np.where(ax >= 0, capacity, np.minimum(capacity, next_capacity))
    residual = np.maximum(0, np.abs(ax) - allowable)
    summary = dict(points=len(v), segment_count=len(ds), lap_length_m=float(ds.sum()),
                   nonclosure_length_m=float(ds[:-1].sum()), closure_length_m=float(ds[-1]),
                   planned_time_s=float(dt.sum()), nonclosure_time_s=float(dt[:-1].sum()),
                   closure_time_s=float(dt[-1]), first_speed_mps=float(v[0]), last_speed_mps=float(v[-1]),
                   closure_acceleration_mps2=float(ax[-1]), max_lateral_acceleration=float(lateral.max()),
                   max_combined_limit_residual_mps2=float(residual.max()),
                   closure_limit_residual_mps2=float(residual[-1]))
    if np.any(v > vehicle["max_speed"] + 1e-9) or lateral.max() > lateral_limit + 1e-6:
        raise ValueError("Speed/lateral limit violation")
    if np.abs(ax).max() > longitudinal_limit + 1e-6:
        raise ValueError("Absolute longitudinal limit violation")
    # Do not conceal the helper's discrete combined-force residual or silently
    # change its speed output. Strict feasibility remains a separate acceptance.
    summary["combined_limit_tolerance_mps2"] = 1e-9
    summary["constraint_violation_count"] = int(np.sum(residual > 1e-9))
    summary["strict_combined_limit_pass"] = bool(residual.max() <= 1e-9)
    summary["closure_combined_limit_pass"] = bool(residual[-1] <= 1e-9)
    return summary


def check_start_index_invariance(geometry, p, vehicle):
    offsets = sorted(set((0, len(geometry) // 12, len(geometry) // 2, 3 * len(geometry) // 4)))
    reference, _, _ = solve_closed_velocity_profile(geometry, p, vehicle)
    maximum = 0.
    for offset in offsets:
        shifted, _, _ = solve_closed_velocity_profile(np.roll(geometry, -offset, axis=0), p, vehicle)
        np.testing.assert_allclose(np.roll(shifted[:, 5], offset), reference[:, 5], rtol=1e-12, atol=1e-10)
        if not validate(shifted, p, vehicle)["strict_combined_limit_pass"]:
            raise ValueError("Shifted closed profile violates combined constraints")
        maximum = max(maximum, float(np.abs(np.roll(shifted[:, 5], offset) - reference[:, 5]).max()))
    return dict(pass_result=True, offsets=offsets, rtol=1e-12, atol_mps=1e-10,
                max_velocity_difference_mps=maximum)


def exclusive_json(path, value, replace=False):
    with Path(path).open("w" if replace else "x") as f:
        json.dump(value, f, indent=2, allow_nan=False)
        f.write("\n")


def generate(mode, resume_incomplete=False, replace=False):
    name = SETS[mode == "regenerated"]
    out = Path("Data/racelines") / name
    meta = Path("Data/raceline_data") / name
    incomplete = out.exists() or meta.exists()
    if incomplete and not resume_incomplete and not replace:
        raise FileExistsError("Refusing existing output directories: " + name)
    if incomplete and not replace and (not out.is_dir() or not meta.is_dir() or
                       sorted(p.name for p in out.iterdir()) != ["esp_raceline.csv"] or
                       list(meta.iterdir())):
        raise FileExistsError("Only an exact incomplete CSV with empty metadata directory can be resumed")
    p, vehicle = parameters()
    versions = {k: importlib.metadata.version(k) for k in
                ("numpy", "scipy", "quadprog", "numba")}
    git_head = (subprocess.check_output(["git", "rev-parse", "HEAD"], text=True).strip()
                if Path(".git").exists() and shutil.which("git") else os.environ.get("STAGE1_GIT_HEAD"))
    before = protected_hashes()
    base = np.loadtxt(BASE, delimiter=",")
    mincurve, error = None, None
    if mode == "fixed":
        geometry = base[:, :5].copy()
    else:
        geometry, mincurve, error = regenerate_minimum_curvature_geometry(p)
    profile, _, _ = solve_closed_velocity_profile(geometry, p, vehicle)
    summary = validate(profile, p, vehicle)
    if not summary["strict_combined_limit_pass"]:
        raise ValueError("Corrected closed constraint acceptance failed")
    invariance = check_start_index_invariance(geometry, p, vehicle)
    assert before == protected_hashes(), "Protected input changed during computation"
    out.mkdir(parents=True, exist_ok=replace or resume_incomplete)
    meta.mkdir(parents=True, exist_ok=replace or resume_incomplete)
    target = out / "esp_raceline.csv"
    if not incomplete or replace:
        with target.open("w" if replace else "x") as f:
            np.savetxt(f, profile, delimiter=",", fmt="%.18e", header=HEADER)
    loaded = np.loadtxt(target, delimiter=",")
    np.testing.assert_array_equal(loaded, profile)
    if mode == "fixed":
        np.testing.assert_array_equal(loaded[:, :5], base[:, :5])
    # Exercise the real baseline loader with the new comment header.
    from f1tenth_benchmarks.utils.track_utils import RaceTrack
    track = RaceTrack("esp", name)
    np.testing.assert_array_equal(track.path, profile[:, 1:3])
    np.testing.assert_array_equal(track.speeds, profile[:, 5])
    if mincurve is not None:
        with (meta / "esp_min_curve_line.csv").open("w" if replace else "x") as f:
            np.savetxt(f, mincurve, delimiter=",", header="s,x,y,heading,curvature")
    provenance = dict(mode=mode, parameters=p, vehicle=vehicle, closed=True, drag_coeff=0,
                      effective_geometry_kappa_bound=1, geometry_optimizer_error=error,
                      baseline_sha256=digest(BASE), output_sha256=digest(target),
                      protected_hashes=before, python=platform.python_version(), packages=versions,
                      git_head=git_head,
                      solver="research_corrected_closed_two_index_fixes",
                      start_index_invariance=invariance, source_sha256=digest(__file__), summary=summary)
    exclusive_json(meta / "metadata.json", provenance, replace=replace)
    assert before == protected_hashes(), "Protected inputs changed"
    print(json.dumps({name: summary}, indent=2))


def compare_profiles(write_report=True, replace=False):
    if write_report and REPORT.exists() and not replace:
        raise FileExistsError("Refusing existing comparison directory")
    p, vehicle = parameters()
    base = np.loadtxt(BASE, delimiter=",")
    profiles = {name: np.loadtxt(Path("Data/racelines") / name / "esp_raceline.csv", delimiter=",") for name in SETS}
    np.testing.assert_array_equal(profiles[SETS[0]][:, :5], base[:, :5])
    summaries = {name: validate(profile, p, vehicle) for name, profile in profiles.items()}
    ds = build_closed_segment_lengths(base[:, :5])
    _, dt = compute_closed_acceleration_and_time(base[:, 5], ds)
    base_closed = base.copy()
    base_closed[:, 6], _ = compute_closed_acceleration_and_time(base[:, 5], ds)
    summaries["baseline_old_mu60"] = dict(points=len(base), segment_count=len(base)-1,
        lap_length_m=float(ds[:-1].sum()), closure_length_m=float(ds[-1]), closure_included=False,
        planned_time_s=float(dt[:-1].sum()), max_velocity_difference_mps=0.,
        max_combined_limit_residual_mps2=validate(base_closed,p,vehicle)["max_combined_limit_residual_mps2"],
        constraint_violation_count=0, constraint_scope="all N closed edges evaluated using baseline velocity")
    summaries["baseline_old_velocity_plus_closure"] = validate(base_closed, p, vehicle)
    summaries["baseline_old_velocity_plus_closure"]["max_velocity_difference_mps"] = 0.
    differences = {}
    for name, profile in profiles.items():
        # Compare a common normalized closed chord arc, including the seam.
        lengths = build_closed_segment_lengths(profile[:, :5])
        u = np.linspace(0, 1, max(len(base), len(profile)), endpoint=False)
        def resample(data, segments):
            arc = np.insert(np.cumsum(segments), 0, 0) / segments.sum()
            result = []
            for col in range(1, 7):
                values = np.append(data[:, col], data[0, col])
                if col == 3:
                    values = np.unwrap(values)
                result.append(np.interp(u, arc, values))
            return np.array(result).T
        a, b = resample(base, ds), resample(profile, lengths)
        delta = b - a
        delta[:, 2] = np.arctan2(np.sin(delta[:, 2]), np.cos(delta[:, 2]))
        differences[name] = dict(max_position_difference_m=float(np.linalg.norm(delta[:, :2], axis=1).max()),
                                 max_heading_difference_rad=float(np.abs(delta[:, 2]).max()),
                                 max_curvature_difference=float(np.abs(delta[:, 3]).max()),
                                 max_speed_difference_mps=float(np.abs(delta[:, 4]).max()),
                                 max_acceleration_difference_mps2=float(np.abs(delta[:, 5]).max()),
                                 exact_geometry_equal=bool(np.array_equal(base[:, :5], profile[:, :5])))
    protected = protected_hashes()
    for name in SETS:
        metadata = json.loads((Path("Data/raceline_data") / name / "metadata.json").read_text())
        assert metadata["protected_hashes"] == protected, "Protected hashes changed since generation"
        assert metadata["output_sha256"] == digest(Path("Data/racelines") / name / "esp_raceline.csv")
        summaries[name]["max_velocity_difference_mps"] = differences[name]["max_speed_difference_mps"]
    invariance = {name: check_start_index_invariance(profile[:, :5], p, vehicle) for name, profile in profiles.items()}
    accepted = all(summaries[name]["strict_combined_limit_pass"] for name in SETS)
    if not accepted:
        raise ValueError("Stage 1 corrected constraints failed")
    report = dict(summaries=summaries, differences=differences, protected_inputs_unchanged=True,
                  fixed_geometry_exact=True, geometry_comparison="common normalized closed chord arc",
                  strict_combined_limit_pass=accepted, start_index_invariance=invariance,
                  baseline_hash_verification=protected, stage1_numerical_acceptance="PASS",
                  solver="research_corrected_closed_two_index_fixes",
                  caveat="Baseline constant-GGV model; does not assert full simulator dynamic feasibility")
    if write_report:
        REPORT.mkdir(parents=True, exist_ok=replace)
        exclusive_json(REPORT / "comparison.json", report, replace=replace)
        with (REPORT / "comparison.csv").open("w" if replace else "x", newline="") as f:
            fields = sorted(set().union(*(s.keys() for s in summaries.values())))
            writer = csv.DictWriter(f, fieldnames=["raceline_set"] + fields)
            writer.writeheader()
            for name, summary in summaries.items():
                writer.writerow(dict(raceline_set=name, **summary))
    print(json.dumps(report, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("operation", choices=("fixed", "regenerated", "compare", "all"))
    parser.add_argument("--resume-incomplete", action="store_true",
                        help="Verify existing CSV exactly, then finish missing metadata; never rewrite CSV")
    parser.add_argument("--verify-only", action="store_true",
                        help="With compare: recompute checks and print, without writing reports")
    parser.add_argument("--replace-stage1", action="store_true", help="Replace ONLY the two research sets and Stage 1 report")
    args = parser.parse_args()
    if args.verify_only and args.operation != "compare":
        parser.error("--verify-only requires compare")
    if args.resume_incomplete and args.operation not in ("fixed", "regenerated"):
        parser.error("--resume-incomplete requires fixed or regenerated")
    if args.replace_stage1 and (args.verify_only or args.resume_incomplete):
        parser.error("Replacement cannot be combined with verification/resume")
    if args.operation == "all":
        targets = [REPORT] + [Path(root) / name for name in SETS for root in ("Data/racelines", "Data/raceline_data")]
        if any(path.exists() for path in targets) and not args.replace_stage1:
            raise FileExistsError("Stage 1 output exists; use compare or inspect it, never overwrite")
        before = protected_hashes()
        # Compare original persisted hashes BEFORE overwriting metadata.
        for name in SETS:
            old = Path("Data/raceline_data") / name / "metadata.json"
            if old.exists():
                for path, sha in json.loads(old.read_text())["protected_hashes"].items():
                    if before.get(path) != sha:
                        raise ValueError("Protected input changed since raw generation: " + path)
        generate("fixed", replace=args.replace_stage1)
        generate("regenerated", replace=args.replace_stage1)
        compare_profiles(replace=args.replace_stage1)
        assert before == protected_hashes()
    elif args.operation == "compare":
        compare_profiles(write_report=not args.verify_only, replace=args.replace_stage1)
    else:
        generate(args.operation, args.resume_incomplete, replace=args.replace_stage1)


if __name__ == "__main__":
    main()
