# Opt-in two-car rollout diagnostics V1

This research sidecar copies native rollout results. It does not predict again,
query collisions again, project Frenet coordinates, calculate costs, rank options,
or issue controls. All records have `inspection_only=true`, `planning_allowed=false`.

## Enable and read

Python: `TwoCarOvertakingEnv(..., phase=2, rollout_diagnostics=True)`.
The default is `False`; requesting diagnostics in phase 1 is rejected.
After each enabled `step()`, read `env.rollout_diagnostics.last_record`.

The existing runner also accepts `rollout_diagnostics=True` and the CLI flag:

```bash
python -m f1tenth_benchmarks.research.run_two_car_candidates \
  --map my_map --phase 2 --seconds 0.3 --rollout-diagnostics \
  --output /path/to/a/new/diagnostics-run
```

Use a new output directory. The runner preserves native `frames.json` and
`report.json` schemas and writes the separate `rollout_diagnostics.jsonl` after
the simulation loop. `write_sidecar(path, records)` uses exclusive creation.
Read JSONL and call `validate_record(record)`; invalid evidence raises `ValueError`.
Invalid records can be preserved by the writer but cannot pass the strict reader.

## Schema and evidence scope

Schema: `two_car_rollout_diagnostics_v1`, version 1.

- `source`: copied ego/lead state, `source_control_step`, `source_sim_time_s`.
  These are the source snapshot clocks, not plan availability or point parameters.
- `metadata`: geometry/frame, model/body dimensions/radius/reference, physics and
  controller settings, physics tick, held steering and steering buffer at source.
- Configured simulator settings are `configured_pair_clearance_m` and
  `configured_wall_clearance_m`. The `physical_*` fields are `null`: settings are
  not measured distances. Old local research records used misleading physical_*
  names for settings; they remain readable and are never rewritten by this PR.
- `pair_threshold_m` / `wall_threshold_m` are native Selector thresholds.
  `samples[].signed_min_pair_clearance_m` is the native signed three-disc result,
  not a clipped single-circle clearance or a real-vehicle measurement.
- `lead_predictions`: per-group requested times, poses, native model and s rate.
  Lead prediction is modeled, not known true future motion.
- `options`: evaluation ID, group ID, native returned order, candidate ID and
  target speed. Repeated candidate IDs at different speeds are separate options.
  ID scope is one collector/run; cross-session consumers must retain session identity.
- `candidate_id=null`, `reference_id='raceline'` preserves independent raceline
  options. Each prediction group has its own times; do not apply the candidate
  horizon to raceline options. A raceline-only record may have no plan.
- `samples`: only actual checked grid points, ego/lead poses, ego speed, native
  signed clearance and wall-check status. Unqueried wall checks stay `null`.
- `requested_horizon_s`, `checked_start_s`, `checked_end_s`, checked count,
  `last_integrated_t_s`, `failed_target_grid_t_s`, termination/error operation
  distinguish requested, sampled and partially integrated physics time.
- `native_safe`, `native_reason`, `first_rejection_s`, and native progress remain
  native diagnostics. First rejection may precede a fully sampled grid. A complete
  pair grid does not imply a complete wall check or continuous swept safety.
- A C3 `reference_centerline` stays reference-only; `modeled_ego_rollout` describes
  copied physics poses, not promotion of that reference to an executable trajectory.

## Timing and behavior

Inside prediction/rollout hooks, only existing arrays/scalars are copied or
appended. `finish_frame()` runs after native `_step()` and physical advancement;
it serializes, hashes and validates outside the native Selector evaluation loop.
File I/O runs after the runner's simulation loop. Native decision/cycle timing
fields do not include all sidecar finalization or final output costs.

Enabled diagnostics still consumes CPU/memory. No hard real-time guarantee is
claimed. Long runs retain records and repeated snapshots in memory; streaming
and large-run performance require separate review.

Behavior tests compare disabled vs enabled runs using identical plans, because
wall-clock-budgeted generation must not be independently rerun and mistaken for
an instrumentation difference. Selection, speeds, controls, safe/reason, event
and physics state/buffer/tick outputs must match, excluding wall-clock fields.

## Producer Evidence is deliberately separate

This PR has no Plan Binding/Cost/Frenet imports. It requires only current master,
which already merged PR #10. It does not depend on Cost PR #11.

`binding_status='unverified_snapshot_only'`, `plan_binding_v2=null`, and
`historical_producer_binding_verified=false` are intentional. Snapshot SHA256
establishes descriptive content identity, not producer-time capture or authentication.

The existing ESP research ProducerTrace is not shipped: it lives in a Cost/Frenet
harness and hashes at plan/option capture time. A later standalone Producer Evidence
PR should split immutable pre-evaluation snapshot capture from post-decision
canonicalization/hash/I/O, explicitly record both phases, and retain false
historical-authentication/control flags. Do not relabel a later digest as a
pre-evaluation hash or silently upgrade existing V1 records.

## Verification

```bash
python -m pytest -q tests/research/test_rollout_diagnostics_v1.py \
  tests/research/test_two_car_candidates.py \
  tests/research/test_three_disc_multi_car.py \
  tests/research/test_candidate_footprint.py \
  tests/research/test_recorded_candidate_pp.py
```

Tests cover paired physical replay, unchanged native Selector results, grids,
partial failure, post-gate rejection, C3/raceline, strict reader validation,
source/plan immutability, JSON round-trip and exclusive sidecar output.
They do not establish real-vehicle safety or complete an overtaking controller.
