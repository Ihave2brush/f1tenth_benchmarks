# Track B session artifacts

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
[2026-10-14 progress](../../../docs/local/lmpc/progress_20261014.md).

## Daily evidence review (2026-10-15)

Latest local overview: [2026-10-15 progress](../../../docs/local/lmpc/progress_20261015.md).
A0–A8 closed-loop attempts and M1/T1 offline diagnostics remain separate artifacts.
A5 reached 22 applied actions / 6.6811 m after handoff; all runs have zero successful
LMPC laps. Use `experiments/A5_A6_kernel_20261015_002/` for the corrected selection-only
comparison; `_001` is superseded. Use `experiments/A7_actuator7_20261015_002/` for the
complete A7 failure record. A8 is an isolated no-FIFO plant variant with fresh data,
not acceptance of the original plant. Iteration limits are not proofs of infeasibility.
The 2026-10-14 timing benchmark does not certify today's changed models/constraints.
