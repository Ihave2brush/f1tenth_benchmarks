"""Keep upstream relative asset paths usable without changing production code."""
from pathlib import Path

import pytest


@pytest.fixture(autouse=True)
def repository_cwd(monkeypatch):
    monkeypatch.chdir(Path(__file__).resolve().parents[2])
