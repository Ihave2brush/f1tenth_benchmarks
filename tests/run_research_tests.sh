#!/usr/bin/env bash
# Run current research tests without rebuilding the baseline Docker image.
set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -e PYTHONDONTWRITEBYTECODE=1 \
  -e NUMBA_CACHE_DIR=/tmp/research_numba \
  -e MPLCONFIGDIR=/tmp/research_matplotlib \
  -e MPLBACKEND=Agg \
  -e PIP_CACHE_DIR=/tmp/research_pip \
  -v "${repo_root}:/workspace:ro" \
  -w /workspace \
  f1tenth-sim:full \
  bash -euc '
    python -m venv --system-site-packages /tmp/research-test-env
    test_python=/tmp/research-test-env/bin/python
    "$test_python" -m pip install --disable-pip-version-check -r requirements-ci.txt
    # A normal installation avoids the modern editable namespace conflict.
    # Copy only the helper build inputs; the repository mount stays read-only.
    mkdir /tmp/research-helper
    cp trajectory_planning_helpers/setup.py trajectory_planning_helpers/README.md /tmp/research-helper/
    cp -R trajectory_planning_helpers/trajectory_planning_helpers /tmp/research-helper/
    "$test_python" -m pip install --disable-pip-version-check --use-pep517 --no-deps /tmp/research-helper
    "$test_python" -m pip check
    "$test_python" -B -m pytest -p no:cacheprovider "$@"
  ' research-tests "$@"
