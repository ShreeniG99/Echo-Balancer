"""The ESP32 firmware core (firmware/include/echo_core.hpp) must reproduce
the Python KF + NIS + gates on identical measurements. Compiled with the
host g++; skipped when no C++ compiler is available."""

import shutil
import subprocess
from pathlib import Path

import numpy as np
import pytest

from sim.control import design_lqr_balance
from sim.estimator import (
    KalmanState, discretize, initial_covariance, measurement_matrix,
    measurement_noise, predict, process_noise, update,
)
from sim.gate import GateMode, initial_gate_state, initial_tilt_gate_state, step_gate, step_tilt_gate
from sim.params import (
    default_disturbance_params, default_estimator_params, default_gate_params,
    default_lqr_balance_params, default_plant_params, default_run_params,
    default_sensor_params, default_tilt_gate_params,
)

FW = Path(__file__).resolve().parents[1] / "firmware"
pytestmark = pytest.mark.skipif(shutil.which("g++") is None, reason="no g++")


@pytest.fixture(scope="module")
def binary(tmp_path_factory) -> Path:
    exe = tmp_path_factory.mktemp("fw") / "host_check"
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-o", str(exe), str(FW / "test" / "host_check.cpp")], check=True)
    return exe


def test_generated_header_is_current():
    """echo_params.h must be what gen_params_header.py would write now."""
    import importlib.util
    spec = importlib.util.spec_from_file_location("gen", FW / "tools" / "gen_params_header.py")
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)
    before = gen.OUT.read_text()
    gen.main()
    assert gen.OUT.read_text() == before


def test_cpp_matches_python(binary):
    p, sp, ep = default_plant_params(), default_sensor_params(), default_estimator_params()
    gp, tp, rp = default_gate_params(), default_tilt_gate_params(), default_run_params()
    vb = default_disturbance_params().battery_droop_v_nominal
    dt = rp.dt_control
    Ad, Bd = discretize(p, dt)
    C, Q, R = measurement_matrix(), process_noise(sp, ep, dt), measurement_noise(sp)
    K = design_lqr_balance(p, default_lqr_balance_params())

    rng = np.random.default_rng(7)
    n = 2400  # 12 s: white noise, then a persistent psi_acc inconsistency from k=600
    Y = np.column_stack([
        rng.normal(0, 0.005, n), rng.normal(0, sp.sigma_gyro, n), rng.normal(0, sp.sigma_accel, n),
    ])
    Y[600:, 2] += 0.15

    kf = KalmanState(np.zeros(5), initial_covariance(ep))
    gs, ts, u_prev = initial_gate_state(), initial_tilt_gate_state(), 0.0
    expected = []
    for y in Y:
        kf = predict(kf, Ad, Bd, u_prev, Q)
        kf, nis = update(kf, y, C, R)
        gs, eps = step_gate(gs, nis, dt, gp)
        ts = step_tilt_gate(ts, kf.x_hat[1], dt, tp)
        u = float(-K @ kf.x_hat[:4])
        v = float(np.clip(u / 2, -vb, vb))
        u_prev = 2 * v
        expected.append((nis, eps, list(GateMode).index(gs.mode), list(GateMode).index(ts.mode), kf.x_hat[1], u))
    expected = np.array(expected)

    out = subprocess.run([str(binary)], input="\n".join(" ".join(f"{v:.17g}" for v in y) for y in Y),
                         capture_output=True, text=True, check=True).stdout
    got = np.array([[float(t) for t in line.split()] for line in out.splitlines()])

    assert got.shape == expected.shape
    np.testing.assert_allclose(got[:, [0, 1, 4, 5]], expected[:, [0, 1, 4, 5]], rtol=1e-6, atol=1e-9)
    np.testing.assert_array_equal(got[:, [2, 3]], expected[:, [2, 3]])
    assert len(set(expected[:, 2])) > 1 and len(set(expected[:, 3])) > 1, "transitions must be exercised"


@pytest.fixture(scope="module")
def scenarios_bin(tmp_path_factory) -> Path:
    exe = tmp_path_factory.mktemp("fw") / "host_scenarios"
    subprocess.run(["g++", "-std=c++17", "-O2", "-Wall", "-o", str(exe), str(FW / "test" / "host_scenarios.cpp")], check=True)
    return exe


def _scenario(exe: Path, name: str, seed: int, secs: float = 60.0, onset: float = 5.0) -> np.ndarray:
    out = subprocess.run([str(exe), name, str(seed), str(secs), str(onset)], capture_output=True, text=True, check=True).stdout
    return np.array([[float(t) for t in line.split()] for line in out.splitlines()])


@pytest.mark.parametrize("seed", [1, 2, 3])
def test_nominal_60s_no_mode_change(scenarios_bin, seed):
    """CLAUDE.md section 11 (gate): no mode change on a nominal 60 s run; mean NIS near chi2(3)'s 3."""
    d = _scenario(scenarios_bin, "none", seed)
    assert d[:, 3].max() == 0 and d[:, 4].max() == 0
    assert 2.7 < d[:, 1].mean() < 3.3


@pytest.mark.parametrize("scenario", ["gyro_bias", "accel_noise", "accel_tilt"])
@pytest.mark.parametrize("seed", [1, 2, 3])
def test_sensor_faults_enter_non_normal_within_window(scenarios_bin, scenario, seed):
    """Sensor faults at the stated section 8 magnitudes (params.py) leave NORMAL within
    the detection window, and never before onset."""
    d = _scenario(scenarios_bin, scenario, seed)
    t, mode = d[:, 0], d[:, 3]
    assert mode[t < 5.0].max() == 0
    hit = np.nonzero((mode > 0) & (t >= 5.0))[0]
    assert len(hit) and t[hit[0]] - 5.0 <= default_disturbance_params().detection_window_s
