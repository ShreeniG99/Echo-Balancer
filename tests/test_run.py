import numpy as np
import pandas as pd
import pytest
import scipy.stats as stats

from sim.disturbances import (
    battery_droop_v_batt,
    payload_shift_plant_params,
    push_psi_dot_kick,
    sensor_fault_accel_noise_params,
    sensor_fault_gyro_bias_step,
    surface_change_plant_params,
)
from sim.params import default_disturbance_params
from sim.run import ControllerType, EpisodeConfig, run_episode


def test_reproducibility_same_config_and_seed_gives_identical_dataframe():
    """CLAUDE.md section 11: 'Same (config, seed) => identical DataFrame.'"""
    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=2.0, wall_following=False)

    df1 = run_episode(config, seed=7)
    df2 = run_episode(config, seed=7)

    pd.testing.assert_frame_equal(df1, df2)


def test_run_episode_returns_expected_columns():
    config = EpisodeConfig(controller=ControllerType.NAIVE, T=1.0, wall_following=False)

    df = run_episode(config, seed=0)

    expected_columns = {
        "t", "theta", "psi", "phi", "theta_dot", "psi_dot", "phi_dot",
        "pos_x", "pos_y", "mode", "nis", "epsilon",
        "front_dist", "right_dist", "v_l", "v_r", "fallen",
    }
    assert expected_columns == set(df.columns)
    assert len(df) == int(round(1.0 / 0.005))  # dt_control=5ms


def test_naive_controller_stays_normal_even_under_a_disturbance_that_would_trigger_the_gate():
    dp = default_disturbance_params()

    def apply_push(t, x, bg, p, sp):
        x = x.copy()
        x[4] = push_psi_dot_kick(x[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x, bg, p, sp, dp.battery_droop_v_nominal

    config = EpisodeConfig(controller=ControllerType.NAIVE, T=10.0, wall_following=False, disturbance=apply_push)
    df = run_episode(config, seed=42)

    assert set(df["mode"]) == {"NORMAL"}


def test_balance_only_nominal_60s_matches_existing_gate_calibration():
    """Regression check: run_episode's wall_following=False path must
    reproduce tests/test_gate.py::test_no_mode_change_on_nominal_60s_run
    and tests/test_estimator.py::test_nis_consistent_on_nominal_60s_run
    exactly (same algorithm, refactored) -- GateParams/EstimatorParams need
    no recalibration for this path. Verified pre-plan: seed=42, 60s,
    mode_set={'NORMAL'}, max epsilon=895.96 (well under tau1=1300)."""
    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=60.0, wall_following=False)

    df = run_episode(config, seed=42)

    assert set(df["mode"]) == {"NORMAL"}
    chi2_3 = stats.chi2(df=3)
    mean_nis = df["nis"].mean()
    frac_above_95 = (df["nis"] > chi2_3.ppf(0.95)).mean()
    assert chi2_3.ppf(0.025) <= mean_nis <= chi2_3.ppf(0.975)
    assert 0.01 <= frac_above_95 <= 0.10
    assert not df["fallen"].any()


def _first_non_normal_latency(df, onset, window_s):
    onset_idx = int(round(onset / 0.005))
    deadline_idx = onset_idx + int(round(window_s / 0.005))
    window = df.iloc[onset_idx:min(deadline_idx, len(df))]
    non_normal = window[window["mode"] != "NORMAL"]
    return None if non_normal.empty else non_normal["t"].iloc[0] - onset


@pytest.mark.parametrize("name,onset_attr,build_disturbance", [
    ("surface_change", "surface_change_onset", lambda dp: (
        lambda t, x, bg, p, sp: (x, bg, surface_change_plant_params(t, dp.surface_change_onset, dp.surface_change_duration, dp.surface_change_fw, p), sp, dp.battery_droop_v_nominal)
    )),
    ("gyro_bias_fault", "gyro_bias_fault_onset", lambda dp: (
        lambda t, x, bg, p, sp: (x, sensor_fault_gyro_bias_step(bg, t, dp.gyro_bias_fault_onset, 0.005, dp.gyro_bias_fault_magnitude), p, sp, dp.battery_droop_v_nominal)
    )),
    ("accel_noise_fault", "accel_noise_fault_onset", lambda dp: (
        lambda t, x, bg, p, sp: (x, bg, p, sensor_fault_accel_noise_params(t, dp.accel_noise_fault_onset, dp.accel_noise_fault_duration, dp.accel_noise_fault_multiplier, sp), dp.battery_droop_v_nominal)
    )),
])
def test_nis_gate_detects_each_disturbance_within_window(name, onset_attr, build_disturbance):
    """Regression check against tests/test_gate.py's existing (already
    passing) per-disturbance detection tests -- run_episode's balance-only
    + NIS_GATE path is the same algorithm, so it must keep detecting these
    within the same CLAUDE.md section 11 2s window."""
    dp = default_disturbance_params()
    onset = getattr(dp, onset_attr)
    config = EpisodeConfig(
        controller=ControllerType.NIS_GATE, T=20.0, wall_following=False,
        disturbance=build_disturbance(dp),
    )

    df = run_episode(config, seed=42)

    latency = _first_non_normal_latency(df, onset, dp.detection_window_s)
    assert latency is not None, f"{name}: never left NORMAL within {dp.detection_window_s}s of onset"


@pytest.mark.parametrize("name,expect_detected,expected_latency_s", [
    ("gyro_bias_fault", True, 0.740),
    ("payload_shift", True, 0.555),
])
def test_tilt_threshold_controller_detection(name, expect_detected, expected_latency_s):
    """Verified pre-plan against the balance-only architecture: the
    tilt-threshold baseline detects gyro_bias_fault at 0.740s and
    payload_shift (+its required companion tilt) at 0.555s -- see the plan
    doc's "Design decision 4." (push/surface_change/accel_noise_fault are
    verified MISSES for this baseline -- see
    test_tilt_threshold_controller_misses_push below.)"""
    dp = default_disturbance_params()
    if name == "gyro_bias_fault":
        onset = dp.gyro_bias_fault_onset
        disturbance = lambda t, x, bg, p, sp: (
            x, sensor_fault_gyro_bias_step(bg, t, dp.gyro_bias_fault_onset, 0.005, dp.gyro_bias_fault_magnitude), p, sp, dp.battery_droop_v_nominal
        )
        x0_psi_deg = 0.0
    else:
        onset = dp.payload_shift_onset
        disturbance = lambda t, x, bg, p, sp: (
            x, bg, payload_shift_plant_params(t, dp.payload_shift_onset, dp.payload_shift_delta_M, dp.payload_shift_delta_L, p), sp, dp.battery_droop_v_nominal
        )
        x0_psi_deg = dp.payload_shift_test_psi0_deg

    config = EpisodeConfig(
        controller=ControllerType.TILT_THRESHOLD, T=20.0, wall_following=False,
        x0_psi_deg=x0_psi_deg, disturbance=disturbance,
    )
    df = run_episode(config, seed=42)

    latency = _first_non_normal_latency(df, onset, dp.detection_window_s)
    assert latency is not None
    assert latency == pytest.approx(expected_latency_s, abs=0.01)


def test_tilt_threshold_controller_misses_push():
    """The tilt-threshold baseline's known, expected blind spot -- push's
    psi excursion never exceeds the nominal noise floor. Contrast with
    test_nis_gate_detects_each_disturbance_within_window, which the NIS
    gate passes for the same class of disturbance."""
    dp = default_disturbance_params()

    def apply_push(t, x, bg, p, sp):
        x = x.copy()
        x[4] = push_psi_dot_kick(x[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x, bg, p, sp, dp.battery_droop_v_nominal

    config = EpisodeConfig(controller=ControllerType.TILT_THRESHOLD, T=20.0, wall_following=False, disturbance=apply_push)
    df = run_episode(config, seed=42)

    latency = _first_non_normal_latency(df, dp.push_onset, dp.detection_window_s)
    assert latency is None


def test_fall_condition_ends_episode_and_cuts_motors():
    """CLAUDE.md section 5: '|psi| > 45 deg ends the episode (motors
    off).' Starting beyond the threshold falls on the very first tick."""
    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=5.0, wall_following=False, x0_psi_deg=50.0)

    df = run_episode(config, seed=42)

    assert len(df) == 1
    assert bool(df["fallen"].iloc[0]) is True
    assert df["v_l"].iloc[0] == 0.0
    assert df["v_r"].iloc[0] == 0.0
