# Research regression testing

The default `pytest` suite protects the research baseline relied on by future
Frenet, candidate-line, and overtaking development. It does not expand testing
to unrelated upstream algorithms or change production code, vehicle dynamics,
existing raceline bytes, or the Docker environment.

## Environment and data

Use Python 3.9 and `requirements-ci.txt`. Numerical dependencies match the
recorded baseline; Pillow and pandas match the existing `f1tenth-sim:full` image.
pytest is a test-only addition. The upstream requirements are unchanged.
Initialize the pinned `trajectory_planning_helpers` submodule recursively;
do not update its revision to install tests.

The exact original `Data/racelines/mu60/esp_raceline.csv` and
`Data/raceline_data/mu60/params.yaml` must be included in the next commit.
The `.gitignore` exceptions expose only these two additional baseline inputs.
The closed CSVs, metadata, ESP maps/centreline, and vehicle/simulator parameters
are already tracked. Missing required inputs fail tests instead of being skipped.

The original mu60 CSV has no header, while the upstream `RaceTrack` loader
always skips one row. Tests protect the current loader result, including this
behavior; they do not fix the loader or rewrite numerical inputs.

## Default checks

- Import the five research modules, package, and velocity helper.
- Validate seven-column, finite, ordered ESP CSV data and real loader output.
- Preserve the SHA256 of the original mu60 CSV.
- Check positive, finite closed speeds and times, all N edges including closure,
  geometry preservation, combined constraints, and ESP start-index invariance.
- Check backward braking on nonuniform edges against an independent analytical
  one-corner example, at every possible starting index. Both the reversed-edge
  alignment and settled-half selection affect this result.
- Retain lap tracker and controlled lifecycle/logging acceptance tests.
- Run short real ESP dynamics checks, plus ten real Pure Pursuit control steps
  with the corrected raceline, fixed configured seed, and no GUI or persistence.

The controlled-dynamics tests can simulate multiple artificial laps or a long
simulation timeout, but do not run full real vehicle laps. Existing runner/logger
tests persist only in temporary directories.

## Optional local archive check

`pytest -m local_archive` verifies the original protected-file manifests. It
requires untracked historical Logs and archived planning inputs, so it is not a
portable CI check. The original acceptance assertion remains unchanged.

The exact 41.52 s original-runner result and full-session replay remain optional
manual acceptance activities. Fixed seeds and historical replay evidence do not
establish bitwise determinism across dependency versions and hardware.

## CI boundary

After local and fresh-checkout verification, CI uses a single Linux
Python 3.9 job: recursive checkout, install the test requirements and editable
packages, then `pytest` with `MPLBACKEND=Agg`. No full real laps, optimization,
training, graphical reports, historical Logs, Docker publishing, or deployment.
The workflow is implemented in `.github/workflows/ci.yml`; its first
GitHub-hosted execution remains to be verified after commit and push.

## Verified results (2026-10-05)

- Existing baseline image, Python 3.9.25, with pytest installed in a temporary
  venv: default suite **46 passed, 1 deselected**, 4.43 s.
- Fresh source snapshot consisting only of tracked files, pinned submodule
  sources, and the proposed additions (including the two exact baseline inputs):
  independent venv without system site-packages, installed only
  `requirements-ci.txt` plus editable packages: **46 passed, 1 deselected**,
  4.48 s. No original Logs archive was present. `pip check` passed.
- Original workspace's optional manifest acceptance: **1 passed**, 0.35 s.
- In-memory mutation checks using the actual new braking test: restoring the
  reverse-edge bug failed all eight starting offsets; restoring the closed-slice
  bug failed four offsets. Production sources remained unchanged.
- Repository mounts were read-only during validation; caches and test outputs
  were temporary. `git diff --check` passed.

These are local results, not a GitHub-hosted runner result. The new files and
previously ignored inputs have not been committed or pushed.

## GitHub Actions workflow

The existing draft `.github/workflows/ci.yml` has been replaced with this
configuration. It retains the draft's action major versions and tests research
branch pushes as well as pull requests. The authoritative source is the workflow
file; the example below summarizes the same installation and test commands.

```yaml
name: Research regression

on: [push, pull_request, workflow_dispatch]

permissions:
  contents: read

concurrency:
  group: research-${{ github.workflow }}-${{ github.ref }}
  cancel-in-progress: true

jobs:
  test:
    name: Python 3.9 research tests
    runs-on: ubuntu-22.04
    timeout-minutes: 15
    env:
      MPLBACKEND: Agg
      PYTHONDONTWRITEBYTECODE: "1"
      NUMBA_CACHE_DIR: ${{ runner.temp }}/numba
      MPLCONFIGDIR: ${{ runner.temp }}/matplotlib
    steps:
      - uses: actions/checkout@v4
        with:
          submodules: recursive
      - uses: actions/setup-python@v5
        with:
          python-version: "3.9"
          cache: pip
          cache-dependency-path: requirements-ci.txt
      - name: Install research test environment
        run: |
          python -m pip install -r requirements-ci.txt
          python -m pip install --no-deps -e trajectory_planning_helpers -e .
          python -m pip check
      - name: Run regression tests
        run: pytest
```

The new CSV and YAML inputs must be committed with the tests before this workflow
is enabled. Existing Docker files and unrelated local research plans are outside
this change.
