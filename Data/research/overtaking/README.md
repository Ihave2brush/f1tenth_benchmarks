# Track A session artifacts

Overtaking inspection runs use a new `<session>/` directory per run. From the
repository root, in the existing simulation Python environment:

```bash
python -B -m f1tenth_benchmarks.research.run_centerline_candidates \
  --output-dir Data/research/overtaking/NEW_SESSION --laps 1 --plan-period 2 --lookahead 3
python -B -m f1tenth_benchmarks.research.overtaking.verify_session \
  Data/research/overtaking/NEW_SESSION --report NEW_VERIFICATION.json
```

The car follows the shared CSV centreline using fixed-speed Pure Pursuit.
Five V5.2 candidates are generated synchronously from actual x/y/yaw and saved
for inspection only. They never take control. This runner does not import LMPC.

Schema version 1 records metadata/configuration/code hashes, `states.jsonl`,
`actions.jsonl`, `plans.jsonl`, `playback_index.json` and `summary.json`. Plans
contain five IDs including failures. Streams flush after each record; handled
exceptions finalize partial output. An abrupt process kill can leave no index
or summary, but the complete JSONL records remain recoverable.

Do not reuse or overwrite an existing session. Raw outputs remain local and
ignored by Git; selected verification reports belong in docs/project/reports.
Browser playback and opponent/selection/control logic are not implemented yet.
