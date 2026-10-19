# Track B: ESP Local LMPC

Current milestone: B0/B1 initialization and B2 offline lap database, local convex
safe set, Q-function and historical-successor advancement have passed acceptance.
Revised B2 reuses a successful flying-lap prefix in a separate audited periodic
terminal tail. Full-lap neighbor support is available with an approximate seam,
without new initialization laps. No LMPC learning runner is active. The local execution plan and
detailed report are in `docs/local/lmpc/`, intentionally excluded from Git.

## Interfaces

- `config.py`: explicit schema and paper values in `params/research/lmpc.yaml`;
  no implicit merging with benchmark files.
- `state.py`: `[vx,vy,r,e_psi,s_unwrapped,e_y]` from the physical centre-of-mass
  state and a timestamp/version-verified core Frenet observation. Legacy
  polyline progress stays separate.
- `dynamics.py`: instrumented shared Continuous lifecycle. `step()` retains
  `[steering,target_speed]`; `step_acceleration()` accepts
  `[steering,commanded_acceleration]` by inverting the existing PID at every
  physics substep. Actual acceleration remains subject to the original limits.
  Both methods preserve steering delay and rate limits.
- `run_smoke.py`: one initialization, 10 Hz speed-mode straight-line exercise,
  one acceleration-mode interval, and replay by the original physical model.
- `lap_adapter.py`: Continuous lap lifecycle driven by the same core Frenet
  observations as the six-state data; true core arclength and exact finish frame.
- `initialization.py`: Cartesian sampled-centreline Pure Pursuit with a constant
  1.2 m/s target from standing start, including below 1 m/s.
- `validation.py`: exact rectangle/nonfree-raster-cell overlap with configurable
  footprint margin, and pointwise Frenet/input-bound candidate diagnostics.
  This is not a dynamics-transition certificate or a planning authorization.
- `run_initialization.py`: preflight, two PP laps, per-substep footprint checks,
  acceptance limits and preserved failure records.
- `verify_initialization.py`: independent physical replay, actuator trace,
  Frenet and lap-event reconstruction from saved samples.
- `plot_initialization.py`: figure from saved B1 samples and events.
- `database.py`: accepted successful laps with source row IDs, outgoing commands,
  measured actuator averages, physical states and actual successors; immutable
  availability frontier per database snapshot and hash-checked NPZ persistence.
- `safe_set.py`: K=20 nearest points by s from each of the latest two laps;
  convex terminal combination, offline LP Q-function and same-lambda successors.
- `build_database.py`: independently reverify B1, build B2 artifacts, check
  nearest-neighbor oracles, costs, Q-function, coordinate shifts and no-future access.
- `plot_database.py`: observed signed costs and an explicitly labeled projection
  of the six-state terminal columns.
- `periodic.py`: original-model seam diagnostics seeded with the complete B1
  endpoint and steering buffer, explicit engineering bounds and classified reuse.
- `build_periodic.py`: revised B2 acceptance, finite periodic tail and B3 resume seed.

The six-state observation is not yet a complete predictive model of actuator
memory. Original physical state and steering buffer are also recorded. B1 is a
restricted Cartesian reference-tracking experiment with exact diagnostics.
Original Frenet approval flags remain false; arbitrary Frenet route optimization
and intersample trajectory certification are not enabled by this experiment.

## Run from repository root

Use the existing Python 3.9 research environment; OSQP is not required for B0.

```bash
python -B -m f1tenth_benchmarks.research.lmpc.run_smoke
python -B -m f1tenth_benchmarks.research.lmpc.run_initialization
python -B -m unittest discover -s tests/research -p test_lmpc_interfaces.py -v
python -B -m unittest discover -s tests/research -p test_lmpc_initialization.py -v
python -B -m unittest discover -s tests/research -p test_lmpc_database.py -v
python -B -m f1tenth_benchmarks.research.lmpc.build_database SESSION_DIR --output-dir NEW_B2_DIR
python -B -m f1tenth_benchmarks.research.lmpc.plot_database NEW_B2_DIR
python -B -m f1tenth_benchmarks.research.lmpc.build_periodic SESSION_DIR --output-dir NEW_PERIODIC_DIR
python -B -m unittest discover -s tests/research -p test_lmpc_periodic.py -v
```

Each run creates a new `Data/research/lmpc/b0_<timestamp>/` unless an explicit
`--output-dir` is supplied. Existing destinations are rejected. `session.json`
records configuration, input hashes, distinct Frenet/legacy lengths, acceptance
scope and replay outcome. `samples.jsonl` records physical/six-state samples,
Frenet diagnostics, commands, every applied actuator substep and steering buffer.
Failures preserve physical samples collected before rejection.

The initialization entry point creates `b1_<timestamp>/`, or a new explicit
`--output-dir`, and also saves `preflight.json` and `events.jsonl`. Limits are
explicit engineering settings in `initialization`, separate from paper values.
To independently inspect and plot an accepted session, from repository root:

```bash
python -B -m f1tenth_benchmarks.research.lmpc.verify_initialization SESSION_DIR --output NEW_VERIFICATION_JSON
python -B -m f1tenth_benchmarks.research.lmpc.plot_initialization SESSION_DIR
```

The candidate checker only checks supplied points, input bounds and Cartesian
footprints. A future MPC must also enforce its prediction model, validity between
points and actuator memory. Body checks at 0.01 s endpoints do not prove a
continuous-time swept-volume guarantee. Two accepted laps are raw initialization
evidence. B2 uses first sampled state at/after each crossing as the discrete
terminal; boundary samples are shared between adjacent laps, never interpolated.
J=T-t is saved in steps and seconds. Observed suffix states keep their original
lap-relative s and signed decreasing J. Negative suffix J is an engineering
continuation potential, not a claim of negative physical travel time; the paper
does not give a numerical suffix-cost rule. Interpolated lap times remain separate.

For the current two-lap evidence, lap 1 extends through the observed lap 2; lap 2
has no post-finish extension. Source controls for x_i -> x_(i+1) are read from
destination row i+1 (the raw logger records incoming controls). Average actual
steering/acceleration describe the substep trace and are not an exact constant
paper input. Raw steering memory and traces remain in the hashed source session.

Observed-only selection refuses windows with missing successors. Revised B2
accepts an explicitly audited periodic successor, but never silently substitutes
a farther point or marks the approximate seam as measured.
The cycle argument translates s by an explicit integer multiple of L, preserving
source identity and costs; it does not fabricate periodic physical successors.
Equation (10) uses the same lambda on successor columns. Equation (7) is tested
as an LP using SciPy HiGHS; B3 will put the shared lambda and cost into its OSQP
QP. LP equalities use a fixed data-origin affine SVD basis (1e-8 rank threshold),
an affine-span check before solving, followed by
an original six-state residual check at 1e-7. This is numerical conditioning,
not authorization of nonlinear safety or an exact actuator-memory model.

The original observed-only artifacts fail closed at the final window. Revised
`b2_esp_periodic_20261008_002` adds 128 existing lap-2 prefix states shifted by L,
with signed costs continuing -1,-2,... and source IDs/timestamps retained.
Observed lap arrays are unchanged. `params/research/lmpc_periodic.yaml` holds
engineering seam tolerances, separate from the frozen B1/paper configuration.
The original model checks the one-step seam, including incoming/outgoing steering
memory, six-state/Cartesian error and substep endpoint footprints. This is bounded
approximate terminal history, not an exact dynamics successor or a feasibility
certificate. Both the seam and virtual replay columns have
`identification_eligible=false`; model identification uses original observed
arrays, never duplicated virtual transitions.

423 windows from s=0 through L+9.6 m pass 20-per-lap selection, source-availability,
cost and successor checks. The 9.6 m overhang is N*dt*max_speed. The finite tail's
last node still rejects selection without a successor. B3 loads these historical laps into a new stationary-start experiment (cycle=0).
The legacy `resume_snapshot.json` is not loaded; the user changed the experiment
on 2026-10-12. Reset samples are never spliced into the historical flying seam.

Always interpret `passed` with `stage`: B0 is a short smoke, B1 is acceptance of
two fixed-speed PP laps. Neither stage certifies LMPC learning performance.


## B3 pilot controller

`run_lmpc` loads the separate `params/research/lmpc_b3.yaml`. The frozen B1
configuration still grants observation/replay only. A new per-action experiment
gate checks the QP candidate with exact core queries and Cartesian footprints,
then previews its full control sequence using the original plant/PID/delay at
0.01 s endpoints. A rejection stops without applying that command. This scope
is ESP simulation only; it never promotes global core flags or certifies an
unsampled continuous domain. There is no PP fallback.

```bash
python -B -m f1tenth_benchmarks.research.lmpc.run_lmpc --output-dir NEW_SESSION
python -B -m f1tenth_benchmarks.research.lmpc.verify_lmpc SESSION --output NEW_AUDIT_JSON
```

`--max-steps N` is a diagnostic probe, never full-lap acceptance. Each control
saves the 40 terminal sources, lambda, shared terminal cost, successor candidate,
12 affine prediction models, weighted identification diagnostics, QP residuals,
shadow checks and actual one-step error. Raw transitions are flushed immediately.
Only a complete accepted lap receives `experiment_lap_001.npz` with known T-t;
initial history stays fixed for this first experiment lap. B4 automatic history
updates are not implemented. Independent audit replays the raw PID targets through
the original model and checks the QP equations and measured-only source IDs.

This executable pilot is a **hybrid model adaptation**, not a strict paper
reproduction: it adds actual steering and two delay entries to the paper's six
states, uses a physical prior plus P=80 local weighted affine residuals, and uses
a differentiable clipped proportional servo only in prediction. Actual and shadow
plants keep the original bang-bang PID. The unregularized paper five-coefficient
regressors/ranks/errors remain diagnostic outputs. Regularization, rate weights,
trust boxes and a 1.5 m/s pilot speed cap are explicit engineering parameters;
N=12, dt=0.1, K=20 per source, P/Q/h/kernel remain at the paper values.

The terminal six-state convex constraint remains hard. SQP refinement is explicit.
The finish indicator stage cost is frozen along the nominal horizon inside each
QP, an approximation around finish, rather than a mixed-integer finish optimizer.
Failures and changed configurations remain separate sessions. Read the local B3
report for the actual acceptance status, including the separate 100 ms budget.

The final `prediction_value_mode` explicitly chooses `smoothed_servo` (default)
or `original_pid` nominal values with surrogate sensitivities (failed diagnostic
variant). There is no automatic switching. `requirements-b3.txt` pins OSQP/qdldl
for an isolated system-site-packages venv; the existing simulation numpy/scipy
versions must not be silently replaced. Current verification used the dedicated
`f1tenth-lmpc-b3` container and `/tmp/lmpc-venv/bin/python`.
