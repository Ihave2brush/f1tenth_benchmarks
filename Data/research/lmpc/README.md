> Branch ownership: LMPC code and documentation are maintained on `research/lmpc`; `master` remains the overtaking mainline. Only this layout README is versioned here; raw sessions remain ignored.

> Current documentation (2026-10-20): [LMPC index](../../../docs/local/lmpc/README.md), [status](../../../docs/local/lmpc/completed.md), and [latest progress](../../../docs/local/lmpc/progress_20261020.md). Dated sections below describe their own artifacts; they are not all current controller settings. Raw experiment files were not moved during documentation cleanup.

# Track B session artifacts

## A5-COMPARE factorial experiment (2026-10-20)

`experiments/A5_COMPARE_20261020_001/report.md` records four new independent runs
with the same accepted NO history and exact handoff. C00 original/Reduced applied
23 commands / 6.988937 m; C01 original/Full, C10 NO/Reduced, C11 NO/Full applied
6 commands each. All stopped at their own OSQP iteration limits; none completed
a lap. Common-prefix analysis uses six applied commands, not unequal-length RMSE.
NO removes overshoot; Full lowers common-prefix cycle time; neither established
successful sustained LMPC. Every failed QP and raw solution is archived per case.
No production edits, new PP, solver retries, lambda projections, commits or pushes.
Older experiments below are historical references, not reused factorial cells.

## A5-NO-FULL short closed loop (2026-10-20)

`experiments/A5_NO_FULL_20261020_001/report.md` records 6 applied commands /
1.772930 m / 0.6 s, then QP7 hit 100000 iterations. No retry or extension.
The 30-step target did not pass. The first six QPs satisfy original constraints
and KKT; later one-step yaw prediction and rate-limited steering tracking diverge.
`live_qps/qp_0007/` preserves the full failure. One JSON logging interruption was
recovered through an exact checkpoint replay without re-solving QP1; both segments
remain archived and the replayed first transition is counted once. Production,
accepted NO history and QP-D1 artifacts are unchanged.

## QP-D1 offline comparison (2026-10-20)

`experiments/QP_D1_20261020_001/report.md` contains the full numerical report,
eight fixed-matrix cases and standalone replay/certificate scripts. With the
original OSQP settings, Full physical solves QP1/QP2 in 1575/1050 iterations;
all original constraints and KKT pass. Reduced all also solves both, so no single
transformation is isolated as the cause. No production or closed-loop changes.
See [2026-10-20 progress](../../../docs/local/lmpc/progress_20261020.md).

## Latest local evidence (2026-10-19)

Start with [2026-10-19 progress](../../../docs/local/lmpc/archive/20261020_document_cleanup/progress_20261019.md)
and [author QP implementation comparison](../../../docs/local/lmpc/archive/20261020_document_cleanup/qp_implementation_comparison_20261019.md).
`experiments/A5_NO_full_20261019_001/` contains fresh non_overshoot initialization
and a one-action LMPC run; its `qp2_diagnosis_20261019_001/` contains the offline
reconstruction, original-constraint feasibility proof and solver diagnostics.
`experiments/A5_NO_S1_20261019_001/` reuses that history with scaling=0: saved QP2
solved, but live QP1 reached the iteration limit and no command was applied.
Its `qp_snapshots/qp_0001/` contains the actual runtime failure matrices and last
iterate. QP2 reconstruction and S1 runtime snapshots are distinct evidence.
No successful LMPC lap or universal solver setting is accepted. Original A2/A5,
reports and raw artifacts remain in place; older sections below are historical.

B0 observation/replay runs now create a new `b0_<timestamp>/` directory per run.
See `f1tenth_benchmarks/research/lmpc/README.md` for the entry point and acceptance
scope. These are short diagnostics, not accepted initialization laps or learning.

B1 runs create `b1_<timestamp>/` with raw two-lap centreline PP data, preflight,
events and acceptance. Independent verification and figures may be added as new
files. B2 builds a separate database from accepted B1 data.

Future learning runs belong in a new `<session>/` directory per run.
Record resolved configuration, geometry version/hash, initialization laps,
lap database, safe set, Q-function data, and evaluation. Identify source laps
and state/action semantics explicitly before sharing data between controllers.
Do not overwrite existing sessions. B3 now has an explicit scoped simulation runner; acceptance is session-specific.

## B2 artifacts

`b2_esp_database_20261008_002/` contains `database.json`, two hash-checked lap
NPZ files, `acceptance.json` and `database_overview.png`. Built from accepted B1
data without new driving. The earlier `_001/` is preliminary accepted evidence.
Both local sets and Q-function checks pass; latest-lap post-finish continuation
is absent, and the final terminal window is explicitly rejected. These artifacts
are local ignored experiment evidence. See `docs/local/lmpc/archive/b2_report_20261008.md`.

## Historical B2 periodic history

`b2_esp_periodic_20261008_002/` is the historical periodic B2 result. It preserves the two
observed laps and adds a separate `lap_002_periodic.npz` reusing 128 past prefix
states. The one-step seam is approximate, audited and excluded from identification.
423 local windows including the finish and N*dt*max_speed overhang pass acceptance.
`resume_snapshot.json` retains the legacy endpoint; the historical B3 runner starts stationary and does not restore it. No new observed lap or
LMPC driving run was added. See `docs/local/lmpc/archive/b2_periodic_report_20261008.md`.

## B3 artifacts

`b3_probe_20261012_*/` are isolated diagnostic attempts, never accepted laps.
`b3_esp_pilot_20261012_*/` are separate full-lap attempts with the pilot cap and
explicit hybrid model. None has completed a lap; `_002` reached 171 steps /
23.8867 m before infeasibility. Independent audits validate executed fragments,
not full B3 success. `b3_final_probe_20261012_001` checks the code/config saved for that historical run.
`samples.jsonl` flushes real transitions, `controls.jsonl` records all optimization
and source diagnostics, and `failure.json` identifies a rejected attempt.
A successful complete lap alone creates a new observed experiment NPZ with T-t.
All historical B1/B2 NPZ data remain immutable; B4 updates are separate work.

## Current paper-mode evidence (2026-10-14)

The current paper runner collects three continuous PP laps, keeps the first two
as sources with genuine continuation, then hands off without reset. The accepted
first LMPC action and second-step model failure are recorded in
`r4_continuous_handoff_20261014_003/`. No successful LMPC lap or learning improvement
is accepted. The QP benchmark is `r4_qp_speed_benchmark_20261014_003/benchmark.json`;
QP timing and complete control timing have separate acceptance.

Core local documentation: [LMPC overview](../../../docs/local/lmpc/README.md),
[completed work](../../../docs/local/lmpc/completed.md),
[paper method](../../../docs/local/lmpc/paper_method.md), and
[2026-10-14 progress](../../../docs/local/lmpc/archive/20261020_document_cleanup/progress_20261014.md).

## Daily evidence review (2026-10-15)

Historical daily overview: [2026-10-15 progress](../../../docs/local/lmpc/archive/20261020_document_cleanup/progress_20261015.md).
A0–A8 closed-loop attempts and M1/T1 offline diagnostics remain separate artifacts.
A5 reached 22 applied actions / 6.6811 m after handoff; all runs have zero successful
LMPC laps. Use `experiments/A5_A6_kernel_20261015_002/` for the corrected selection-only
comparison; `_001` is superseded. Use `experiments/A7_actuator7_20261015_002/` for the
complete A7 failure record. A8 is an isolated no-FIFO plant variant with fresh data,
not acceptance of the original plant. Iteration limits are not proofs of infeasibility.
The 2026-10-14 timing benchmark does not certify today's changed models/constraints.
