# Offline Frenet Progress V1

Read-only projection of recorded modeled ego/lead poses. Depends on Diagnostics
PR #12 (`validate_record`, `snapshot_sha256`) and the existing Geometry Tracker.
No Cost PR, Group Audit, Producer/Model Evidence, accelerator or ROS dependency.

## API

- `project_pose_series(samples, source_state, geometry, *, geometry_id, frame_id, tracking_options, anchor_tolerance_m=1e-4)` uses a new private native Tracker and explicit source wrapped/unwrapped s anchors.
- `relative_progress_features(ego, lead, *, requested_horizon_s, native_complete)` returns final relative displacement and its negative trapezoidal integral (m*s).
- `project_rollout_record(record, geometry, *, tracking_options, frame_id='map', anchor_tolerance_m=1e-4)` validates a Diagnostics V1 record and preserves every option identity and order.

No interpolation, live tracker updates, ambiguity recovery or native rollout.
Invalid samples invalidate the remaining trajectory; only the contiguous valid
prefix has a partial integral. Missing/full costs remain null as appropriate.
C3/raceline reference geometry is distinct from the modeled ego rollout.
Native approximate `relative_s_progress_m` is preserved separately.

## Reproduce (Python 3.9)

From the repository root, install `requirements-ci.txt` in a Python 3.9 environment:

```bash
python -m pip install -r requirements-ci.txt
python -m pytest tests/research/test_offline_rollout_frenet_v1.py tests/research/test_rollout_diagnostics_v1.py tests/research/test_frenet_tracking.py -q
```

Tests use repository-relative geometry helpers and
`tests/research/fixtures/offline_frenet_synthetic_v1.json`. This is explicitly
artificial schema/pose math, not an ESP or native dynamics result. Its signed
clearance/native flags are synthetic labels, not measured collision evidence.
It covers repeated candidate speeds, groups, C3, raceline and partial data.

Original ESP/my_map archives are not bundled. Optional archive replay:

```bash
python tools/check_offline_frenet_archive_v1.py --source PATH_TO_CASE/rollout_diagnostics.jsonl --geometry-dir maps/frenet/esp/geometry --map-yaml maps/esp.yaml --expected PATH_TO_CASE/frenet_projection.json --output NEW_projection.json
```

The expected archive explicitly supplies the original TrackingOptions and
anchor tolerance. All fields/statuses are compared, numeric tolerance 1e-9.
Output is exclusively created. No archive download or simulation is triggered.

## Integration notes

This branch stacks on PR #12. Only the projection module, portable tests,
synthetic fixture, replay checker and this document are new. Dependency changes
remain attributable to #12. Merge #12 first, then review the incremental diff.
No native Selector, PP, Dynamics, Collision or safety thresholds are changed
by this layer. `inspection_only=true`, `planning_allowed=false`; no formal
comparison or historical authentication is granted. Exact projection is
expensive; this is not a 10 Hz deployment claim.
