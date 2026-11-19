# Producer Evidence Core V1

## Summary

Data capture, content verification and exclusive evidence output. This core has
no Diagnostics subclass, Plan callback, native hook wiring, Selector/controller
accessor, Cost integration, Model validation or ESP harness. It does not prove
that a caller actually invoked capture before native evaluation.

## Dependencies and PR scope

Plan Binding V2 is already merged. This core also depends on PR #12 at `0fe05d8`
for the existing strict `validate_record` and `snapshot_sha256` helpers. Reusing
those helpers preserves the Diagnostics schema/hash contract; they are not copied
into a second implementation. NumPy is an existing project dependency.

This branch is stacked on #12. Relative to current master it temporarily includes
#12's eight prerequisite files plus three new Core files. Only the top Core commit
is new here. It does not depend on Cost #11, Adapter #13 or Model Core #14. Wait
for #12 before merging; review this as a dependent Draft if #12 is still open.

## API and input contract

```python
capture = ProducerCaptureV1(
    source, session_metadata, runtime_metadata, length_m=lookahead_m
)
capture.capture_plan(plan)
capture.capture_group(group)
capture.capture_option(option)
capture.capture_rollout_input(evaluation_id, pp_data)
evidence = capture.finalize(
    saved_diagnostics_record,
    geometry_manifest_path=manifest_path,
    expected_manifest_sha256=manifest_pin,
)
verify_producer_evidence_v1(evidence, saved_diagnostics_record,
                            session_metadata=session_metadata)
write_producer_evidence(new_jsonl_path, [evidence])
```

- `source` uses the Diagnostics source object: explicit source control step/time,
  ego/lead states. Point parameter `t` or plan availability is not a source clock.
- Session metadata requires geometry/frame, declared producer commit, source code
  hash claims and physics/vehicle parameter snapshots. No missing values are filled.
- Runtime metadata is supplied data with the explicit V1 `RUNTIME_KEYS` schema.
  The core does not query a simulator or derive body dimensions.
- Group/option fields follow Diagnostics V1. Evaluation IDs, group IDs and native
  return order remain separate from capture sequence. Different speeds/groups are
  not merged. IDs require run/source provenance for cross-session identity.
- `pp_data` is a data holder with `path`, `speed`, `lookahead`, `wheelbase`,
  `max_steer` attributes. Capture reads/copies them; it never calls a PP algorithm.

Capture stores private frozen mappings/tuples and independent read-only array
copies. Canonicalization, hashes, Plan Binding V2, schema/content checks and IO
are deferred to finalize/verify/writer. Shape accesses necessary to identify a
capture may reject missing identities immediately. Full malformed data is never
accepted as verified evidence. Inputs and existing sidecars remain unchanged.

Raceline may precede Plan. Keep capture sequence and `plan_present_at_capture`;
never bind a later Plan as the cause of an earlier raceline evaluation. Candidate
options require Plan capture first. Raceline-only evidence has no fabricated Plan
Binding. C3 reference geometry stays reference-only even with modeled samples.

## Digest and verification semantics

Schema `f1tenth_producer_evidence_v1`, integer version 1. The Evidence domain,
`producer_evidence_compare`, snapshot JSON hash and Binding V2 rules are unchanged
from the research implementation. The opt-in legacy verifier was moved unchanged
in meaning from the research Adapter, without importing any Cost module.

`verify_optional_evidence(..., allow_legacy_esp=False)` rejects legacy by default.
Explicit opt-in supports `esp_producer_evidence_v1` archives as legacy content,
with `new_capture_contract_compliant=false`; it cannot retrofit new capture timing.
Malformed new evidence cannot downgrade to legacy. Missing evidence stays missing.
Legacy verification is read-only; the new writer does not rewrite legacy records.

A successful verifier checks local content and declared sequence against an actual
validated sidecar. It does not independently authenticate physical capture time,
Git history, current code hash claims, feasibility or safety. Model policy data may
be retained in metadata but is not validated by this core or promoted to trust.

Writer checks envelope/schema/flags/digest then uses exclusive creation. Full
source verification still requires the sidecar/metadata verifier; writing alone is
not approval. All results retain inspection-only, planning-disabled, no historical
authentication, no producer binding approval and no formal comparison qualification.
Unknown, partial, unsafe, rejected and reference-only evidence is not upgraded.

## Validation

```bash
python -m pytest -q tests/research/test_producer_evidence_v1.py \
  tests/research/test_plan_binding_v1.py tests/research/test_plan_binding_v2.py \
  tests/research/test_rollout_diagnostics_v1.py
```

Public tests use independent synthetic data and the real Diagnostics schema writer,
without other research fixtures, saved local archives, Git history, network access,
or model execution. Python 3.9 is supported. Historical ESP verification is an
additional local read-only compatibility check, not public CI archive coverage.

## Notes

Runtime hooks belong in a later PR: opt-in subclass, source/runtime provider, Plan
callback and post-step finalization. That integration must test disabled/enabled
native decisions, speed/actions, physics and sidecar parity using identical saved
plans. This Core PR alone is not completed producer-time simulator integration.
No safety gates, native files, weights, ranking or controls change in the Core commit.
