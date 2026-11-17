# Group-Aware Audit / Rollout Cost Adapter V1

## Summary

Read-only bridges between saved native rollout diagnostics and offline Cost V1.
No generation, simulation, Frenet projection, new collision model, new risk formula,
ranking, or control. All results are inspection-only and planning-disabled.

## Dependencies and branch scope

This integration branch temporarily contains the exact Cost PR #11 files and the
fixed Diagnostics PR #12 files. These are prerequisites, not new Adapter features.
The new contribution is two modules, their two tests, and this document. Prefer
merging the prerequisites first; then refresh the Adapter branch against master
without overwriting published history. Never append Adapter changes to #11/#12.

## API

```python
from f1tenth_benchmarks.research.overtaking.rollout_cost_adapter_v1 import (
    read_rollout_sidecar, adapt_rollout_cost_inputs,
)
from f1tenth_benchmarks.research.overtaking.two_car_group_audit_v1 import (
    evaluate_grouped_rollout,
)

# Existing saved state/frame/report/metadata and existing length results only.
result = evaluate_grouped_rollout(
    source_frame, report, metadata, sidecar_record,
    risk_policy=explicit_experimental_policy,
    path_length_results=saved_length_results,
    source_classification="saved_two_car_simulation",  # use "synthetic" for fixtures
)
```

`read_rollout_sidecar(path)` strictly validates JSONL, returns records and file
SHA256. `adapt_rollout_cost_inputs(sidecar, source_frame, audit_result,
path_length_results, *, plan_binding=None, geometry_manifest_path=None,
expected_manifest_sha256=None, existing_cost_features=None)` joins exactly one
saved source. A sidecar file must select exactly one source clock match.

Direct adaptation requires the native Audit and sidecar prediction grids to match.
It rejects mixed native grids; it never silently overwrites the original Audit.

`evaluate_grouped_rollout` handles multiple groups by preserving `native_audit`
and constructing explicitly labeled single-option derived views. Each view uses
the recorded group's grid and keeps native option index, evaluation ID, group ID,
original frame/sidecar hashes, plus view-local option index 0. View digests identify
views; they are not producer-time evidence for the original frame.

The group API retains its saved-simulation default for prior callers. Synthetic
callers must explicitly pass `source_classification="synthetic"`; Audit records
this classification as a caller assertion, not verified source authentication.
The original descriptive metadata classification is retained separately.

## Semantics and limits

- Candidate-speed options and native order are preserved. Repeated candidate IDs
  at different speeds are distinct; raceline has null candidate ID.
- C3 reference geometry stays reference-only. Copied physics rollout poses are
  separately labeled modeled ego rollout, not an executable candidate approval.
- Requested and checked time ranges remain distinct. Partial integral diagnostics
  cannot become full-horizon low costs; unknown/unsafe/rejected are not promoted.
- Signed three-disc clearance stays signed. Configured pair/wall margins are
  settings; physical measured fields remain null. Documented legacy V1 setting
  aliases are readable but are never exported as measured distances.
- Missing producer evidence keeps `producer_binding_verified=false` and
  `formal_comparison_qualified=false`, even with complete sample grids and valid
  audit-time Plan Binding V2. No experimental total cost is authorized by this join.
- Existing Cost Feature/Evaluation implements the mathematics. This bridge does
  not duplicate risk integration, change normalization, choose weights or select.
- Supplying saved length results avoids geometry sampling. If omitted, the group
  API explicitly requires geometry and uses existing Path Length V1; that optional
  mode has geometry cost and must not be described as pure data processing.

## Validation

```bash
python -m pytest -q tests/research/test_rollout_cost_adapter_v1.py \
  tests/research/test_two_car_group_audit_v1.py \
  tests/research/test_path_length_feature_v1.py \
  tests/research/test_cost_feature_v1.py \
  tests/research/test_cost_evaluation_v1.py
```

Default tests use synthetic saved-format contracts without local archives or
network access. Separate local verification replays existing primary my_map and
ESP archives read-only, reuses saved lengths, checks option/group identity and raw
risk values, and compares source hashes. Those archives are not included here;
local archive verification is not claimed as reproducible public CI coverage.
