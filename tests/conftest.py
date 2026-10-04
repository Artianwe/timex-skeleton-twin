import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from twin.dynamics.simulator import Simulator  # noqa: E402
from twin.model import build_model  # noqa: E402
from twin.params import load_params  # noqa: E402


@pytest.fixture(scope="session")
def P():
    return load_params()


@pytest.fixture(scope="session")
def model(P):
    return build_model(P)


@pytest.fixture(scope="session")
def sim(model):
    return Simulator(model)


@pytest.fixture(scope="session")
def T_full(model):
    return float(model.spring.torque_out(model.spring.n_dev))
