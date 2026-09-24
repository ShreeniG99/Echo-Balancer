"""Tests for sim/run.py (CLAUDE.md section 4/11/12)."""

import numpy as np
import pandas as pd

from sim.disturbances import push_psi_dot_kick, sensor_fault_gyro_bias_step, surface_change_plant_params
from sim.params import default_disturbance_params
from sim.run import ControllerKind, EpisodeConfig, run_episode


def test_same_config_and_seed_gives_identical_dataframe():
    """Section 11: 'Same (config, seed) => identical DataFrame.'"""
    cfg = EpisodeConfig(controller=ControllerKind.NIS_GATE, T=2.0)
    df1 = run_episode(cfg, seed=7)
    df2 = run_episode(cfg, seed=7)
    pd.testing.assert_frame_equal(df1, df2)


def test_different_seed_gives_different_dataframe():
    cfg = EpisodeConfig(controller=ControllerKind.NIS_GATE, T=2.0)
    df1 = run_episode(cfg, seed=1)
    df2 = run_episode(cfg, seed=2)
    assert not df1["psi"].equals(df2["psi"])


def test_nominal_60s_wall_following_run_stays_normal():
    """Section 11's nominal-run requirement, exercised through run.py's real
    closed loop (design_lqr_speed_servo + wall-following), not the simpler
    balance-only loop tests/test_gate.py already covers. See
    sim.params.default_evaluation_gate_params' docstring for why this
    closed loop needs its own calibrated GateParams."""
    for seed in range(5):
        cfg = EpisodeConfig(controller=ControllerKind.NIS_GATE, T=60.0)
        df = run_episode(cfg, seed=seed)
        assert set(df["mode"].unique()) == {"NORMAL"}, f"seed {seed} tripped the gate on a nominal run"
        assert not df.attrs["fell"]


def test_naive_controller_ignores_disturbance():
    """NAIVE (CLAUDE.md section 12, controller 1): always NORMAL, regardless
    of what the NIS is doing."""
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        x_true = x_true.copy()
        x_true[4] = push_psi_dot_kick(x_true[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x_true, b_g_true, p, sensor_p, dp.battery_droop_v_nominal

    cfg = EpisodeConfig(controller=ControllerKind.NAIVE, T=20.0, disturbance=apply_disturbance)
    df = run_episode(cfg, seed=42)
    assert set(df["mode"].unique()) == {"NORMAL"}


def test_nis_gate_detects_push_within_window():
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        x_true = x_true.copy()
        x_true[4] = push_psi_dot_kick(x_true[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x_true, b_g_true, p, sensor_p, dp.battery_droop_v_nominal

    cfg = EpisodeConfig(controller=ControllerKind.NIS_GATE, T=20.0, disturbance=apply_disturbance)
    df = run_episode(cfg, seed=42)

    onset_idx = int(round(dp.push_onset / 0.005))
    deadline_idx = onset_idx + int(round(dp.detection_window_s / 0.005))
    window = df.iloc[onset_idx:deadline_idx]
    assert (window["mode"] != "NORMAL").any(), "NIS gate never left NORMAL within the detection window"


def test_tilt_gate_ignores_a_pure_estimation_disturbance():
    """TILT_GATE (CLAUDE.md section 12, controller 2) only ever looks at
    the estimated psi -- a gyro-bias fault (which the NIS gate reliably
    detects, per test_nis_gate_detects_push_within_window's sibling
    scenarios in tests/test_gate.py) perturbs the psi_dot measurement, not
    psi itself, so it should not move the tilt gate at all."""
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        b_g_true = sensor_fault_gyro_bias_step(
            b_g_true, t, dp.gyro_bias_fault_onset, 0.005, dp.gyro_bias_fault_magnitude
        )
        return x_true, b_g_true, p, sensor_p, dp.battery_droop_v_nominal

    cfg = EpisodeConfig(controller=ControllerKind.TILT_GATE, T=10.0, disturbance=apply_disturbance)
    df = run_episode(cfg, seed=42)
    assert set(df["mode"].unique()) == {"NORMAL"}


def test_fall_ends_episode_early_with_motors_off():
    """CLAUDE.md section 5: '|psi| > 45 deg ends the episode.' Uses a
    surface-change fw beyond the documented ~0.03 instability margin (see
    project-context/OPEN_PROBLEMS.md) to reliably force a fall well before
    T, on the NAIVE controller (no gate slowdown to delay it)."""
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        p_now = surface_change_plant_params(t, dp.surface_change_onset, dp.surface_change_duration, 0.05, p)
        return x_true, b_g_true, p_now, sensor_p, dp.battery_droop_v_nominal

    cfg = EpisodeConfig(controller=ControllerKind.NAIVE, T=15.0, disturbance=apply_disturbance)
    df = run_episode(cfg, seed=0)

    assert df.attrs["fell"]
    assert len(df) < int(round(cfg.T / 0.005))
    assert abs(df["psi"].iloc[-1]) <= np.radians(45.0)
