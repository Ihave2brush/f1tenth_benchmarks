# Model Provenance Core V1

## Summary

Pure data/content verification for an explicit local model policy. It separates
producer checkout identity, declared native model source, code/parameter identity,
behavior equivalence and historical authentication. Only Plan Binding V2's
canonical digest helper is imported; no dependency on PR #11, #12 or #13.

## API

- `hash_model_files(root)`: SHA256 of the fixed, reviewed 19-file `MODEL_PATHS`
  scope; symlinks escaping the explicit root are rejected before reading.
- `model_parameters(runtime)`: copy the independent V1 `PARAMETER_KEYS` data schema.
  This is not a collector/controller import or a feasibility check.
- `build_model_policy_v1(native_model_source, code_hashes, parameters, diagnostics_version)`.
- `model_policy_digest(policy)`.
- `build_model_provenance_v1(*, producer_commit, native_model_source,
  model_code_hashes, model_parameters_snapshot, diagnostics_version)`.
- `verify_model_provenance_v1(contract, *, trusted_policy=None,
  expected_policy_sha256=None, source_root=None, observed_parameters=None,
  actual_producer_commit=None, observed_diagnostics_version=None)`.

Schema: `f1tenth_model_provenance_v1`, integer version 1. Domain-separated digests
reuse Plan Binding V2 typed JSON. Extracting parameter keys from the prior research
collector does not change their values or valid existing digests.

```python
result = verify_model_provenance_v1(
    contract,
    trusted_policy=reviewed_policy,
    expected_policy_sha256=separately_supplied_pin,
    source_root=explicit_source_directory,
    observed_parameters=caller_observed_runtime_parameters,
    actual_producer_commit=caller_observed_checkout_commit,
    observed_diagnostics_version=caller_observed_versions,
)
```

Do not treat hashing an untrusted policy next to itself as independent approval.
Policy selection and the pin's trust source remain caller responsibilities. Supply
actual observations, rather than copying claims from the contract as evidence.

## Verification semantics

| Status | Meaning |
|---|---|
| `verified_content_against_explicit_pin` | Actual scoped file bytes and supplied parameter/version observations match claims and the explicitly pinned policy. |
| `unconfirmed` | Required policy/pin, observations or source files are missing/unavailable. |
| `inconsistent` | Digests, claims, policy, observations, versions or containment rules contradict each other. |

`verification_reasons` retains concrete reasons. A new producer commit is not
added to an allowlist. Matching commits cannot hide dirty scoped model files.
Unrelated dirty files outside `MODEL_PATHS` do not invalidate scoped identity;
verification does not declare the whole repository clean or fully identified.

No Git executable or `.git` directory is required. The library compares the
explicit caller-observed commit; it does not independently authenticate Git
history or inspect checkout status. Source archives must retain that observation
externally, or leave it missing and receive `unconfirmed`.

Parameters are content records, not physical validity/safety certification. The
19-file scope does not authenticate the complete binary/library dependency graph.
Native source is a declaration, not automatic proof of byte identity or behavior
with that version. Diagnostic version identifiers are explicit data, not runtime
module execution or automatic source discovery.

All results retain `inspection_only=true`, `planning_allowed=false`,
`formal_comparison_qualified=false`, `model_behavior_equivalence_verified=false`
and `historical_authentication_verified=false`. No costs, weights, ranking,
selection, controls, Audit override or compatibility view are produced.

## Validation

```bash
python -m pytest -q tests/research/test_model_provenance_v1.py \
  tests/research/test_plan_binding_v1.py tests/research/test_plan_binding_v2.py
```

Tests use explicit synthetic manifests/parameters in temporary roots, not ESP
archives, live simulation, network access or another research module's fixtures.
They cover old/new commit identity, changed model bytes/parameters, missing pins,
contradictory digests/versions, source archives, dirty Git checkouts, input
immutability, root containment and import/execution independence. Python 3.9 is
supported. Runtime capture, Producer Evidence, Audit integration and saved ESP
research stay in separate future changes.
