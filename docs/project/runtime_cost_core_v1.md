# Runtime Cost Core V1

Read-only in-memory batch features, not a real-time guarantee or Cost Selector.
This branch stacks Diagnostics #12, Offline Frenet Progress V1, and Cost #11.
Only runtime inputs/core, portable tests/support, two small tools and this
file are new. No Group Audit #13, Model #14 or Producer #15 dependency.
The Cost #11 commit `37244ef` is copied as dependency commit `dffc423`;
its seven files are byte-identical. The Frenet parent is `06bf4f4`,
stacked on Diagnostics head `0fe05d8`. These are dependencies, not new runtime work.

## API

```python
from f1tenth_benchmarks.research.overtaking.realtime_cost_inputs_v1 import (
    RuntimeCostContext, prepare_runtime_length_cache)
from f1tenth_benchmarks.research.overtaking.realtime_cost_core_v1 import (
    prepare_projection_bundle, prepare_runtime_inputs, evaluate_cost_batch,
    evaluate_runtime_cost_core)

context = RuntimeCostContext(geometry, risk_policy=explicit_policy,
                             tracking_options=explicit_tracking_options)
lengths = prepare_runtime_length_cache(plan, geometry.geometry_id,
                                      verified_length_values,
                                      method='cached_frenet_integral')
projection = prepare_projection_bundle(record, context)  # expensive preprocessing
batch = prepare_runtime_inputs(record, context, length_cache=lengths,
                               projection_bundle=projection)
result = evaluate_cost_batch(batch)
# Or adapter + core with the same prepared inputs:
result = evaluate_runtime_cost_core(record, context, length_cache=lengths,
                                    projection_bundle=projection)
```

`record` is the Diagnostics V1 in-memory source/metadata/plan_snapshot/options/
lead_predictions structure. Optional projection preprocessing uses the existing
exact Tracker; it is explicitly not inside `evaluate_cost_batch`. Each ego has
a private tracker and each source prediction group can share its lead series.

- Length: supplied cached geometry length (m); reference geometry is not modeled ego distance.
- Risk: original Cost V1 `_component` exposure integral (m^2*s).
- Progress: original anchored Frenet negative relative-displacement integral (m*s).
- Risk policy is explicit, exposure-only and without weights; initialization uses existing Cost V1 `_policy`.

The tiny context/cache definitions were mechanically extracted from the old
research observer module; no hooks or second formula implementation were copied.
Core code differs from the research source only in that import. Cost V1 private
helpers `_policy`/`_component` remain a documented version-pinned dependency;
changing them requires parity tests, not silent replacement.

Exact value keys invalidate changed plan/source/grid/anchor/model inputs; they
are not producer authentication. Caller-supplied lengths require prior geometry
validation. Missing data stay null; unsafe/unknown/rejected/reference-only status
never grants comparison or control. `features_complete` means fields available,
not safe/eligible. Complete and prefix integrals remain distinct. No model trust
is promoted. `total_cost=null`, no weights, ranking or selected candidate.

## Reproduce (Python 3.9, repository root)

```bash
python -m pip install -r requirements-ci.txt
python -m pytest tests/research/test_realtime_cost_core_v1.py tests/research/test_realtime_cost_inputs_v1.py tests/research/test_offline_rollout_frenet_v1.py tests/research/test_cost_evaluation_v1.py tests/research/test_cost_feature_v1.py tests/research/test_path_length_feature_v1.py -q
python tools/benchmark_runtime_cost_core_v1.py --repeats 200 --warmup 20
```

The benchmark deliberately uses shipped synthetic test support (one option,
one group, three samples); it requires the tests checkout and pytest, and
excludes projection, geometry, rollout, IO and ROS. Quantiles are not Jetson
or full-cycle performance. No machine-specific absolute paths are used.

Optional original ESP verification (archives not bundled):

```bash
python tools/check_runtime_cost_archive_v1.py --cases-root PATH_TO_esp_cases_001 --geometry-dir maps/frenet/esp/geometry --map-yaml maps/esp.yaml --output NEW_runtime_parity.json
```

Requires each case's diagnostics, frames, group_audit, summary and saved Frenet
projection; reuses supplied archived lengths/policies and never reruns Group
Audit or claims new provenance. Output creation is exclusive. Synthetic tests
are the mandatory portable acceptance path.

No native Selector/PP/Dynamics/Collision or thresholds changed by this layer.
No Exact Accelerator, Geometry V2, Worker, ROS or new weights included.
`inspection_only=true`, `planning_allowed=false`, `formal_comparison_qualified=false`.
Full new-rollout cost currently exceeds 100 ms in research measurements; this
PR exposes a testable component without claiming vehicle 10 Hz readiness.
