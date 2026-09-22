import pandas as pd
import pytest
import scipy.stats as stats

from sim.disturbances import (
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


@pytest.mark.parametrize("name,expected_latency_s", [
    ("gyro_bias_fault", 0.740),
    ("payload_shift", 0.555),
])
def test_tilt_threshold_controller_detection(name, expected_latency_s):
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


def test_voltage_clips_per_motor_independently_when_saturated():
    """A large enough initial tilt drives the LQR command past V_batt --
    v_l/v_r are each clipped independently to +/-V_batt (not the combined
    command clipped then split in half). Both hit exactly the same clip
    bound here since v_l_cmd==v_r_cmd in the balance-only path (no
    differential drive), but this locks in the clip-per-motor code path
    in sim/run.py, distinct from tests/test_gate.py's
    _closed_loop_with_gate harness (which clips the combined command
    before splitting) -- see sim/run.py's module docstring and the plan
    doc's Task 4 code-review note.

    x0_psi_deg=25.0 was determined empirically: with
    K4 = design_lqr_balance(default_plant_params(), default_lqr_balance_params()),
    -K4 @ [0, radians(25), 0, 0] / 2 ~= 10.76 V, comfortably past the
    7.4V nominal V_batt (>40% over) while 25 deg is comfortably under the
    45 deg fall threshold. The KF's state estimate starts at zero and is
    only partially corrected by the first measurement update, so
    saturation is not visible until the second control tick (index 1,
    t=0.005s) rather than the very first -- confirmed by direct
    inspection of v_l/v_r across the first two ticks at several
    candidate tilts."""
    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=0.01, wall_following=False, x0_psi_deg=25.0)

    df = run_episode(config, seed=42)

    v_batt = default_disturbance_params().battery_droop_v_nominal
    assert len(df) == 2
    assert not df["fallen"].any()
    assert abs(df["v_l"].iloc[1]) == pytest.approx(v_batt)
    assert abs(df["v_r"].iloc[1]) == pytest.approx(v_batt)


def test_corridor_nominal_60s_stays_normal_at_the_calibrated_safe_speed():
    """Verified pre-plan: seed=42, 60s, wall_following=True,
    corridor_theta_dot_ref_nominal=0.3 (RunParams default) -- completes all
    12000 steps with mode_set={'NORMAL'}, max epsilon=909.13 (comparable to
    the balance-only nominal range of 823-951, well under tau1=1300). See
    the plan doc's "Design decision 3" -- tests/test_wall_following.py's
    own theta_dot_ref_nominal=2.0 is NOT used here; it was verified unsafe
    once the gate is actually watching (max epsilon 1605.56)."""
    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=60.0, wall_following=True)

    df = run_episode(config, seed=42)

    assert len(df) == int(round(60.0 / 0.005))
    assert set(df["mode"]) == {"NORMAL"}
    assert not df["fallen"].any()
    assert df["epsilon"].max() < 1300.0


def test_corridor_episode_makes_forward_progress_and_stays_in_corridor():
    config = EpisodeConfig(controller=ControllerType.NAIVE, T=20.0, wall_following=True)

    df = run_episode(config, seed=42)

    assert (df["pos_y"] > 0.0).all()
    assert (df["pos_y"] < 1.0).all()  # default corridor width
    assert df["theta"].iloc[-1] > df["theta"].iloc[0]  # rolled forward
    assert not df["front_dist"].isna().any()
    assert not df["right_dist"].isna().any()


def test_corridor_push_disturbance_is_a_known_miss_for_the_nis_gate():
    """Documents, rather than hides, the plan doc's "Design decision 1"
    finding: push-type disturbances are not reliably detected once the
    control law is design_lqr_speed_servo, even at the calibrated-safe
    corridor speed. Verified pre-plan: seed=42, max epsilon in the
    detection window = 1085.51, under tau1=1300 -- MISSED. This is an
    inherited, pre-existing thin margin (design_lqr_balance's own push
    margin was already only 3.5% over tau1), not a step-6 regression."""
    dp = default_disturbance_params()

    def apply_push(t, x, bg, p, sp):
        x = x.copy()
        x[4] = push_psi_dot_kick(x[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x, bg, p, sp, dp.battery_droop_v_nominal

    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=20.0, wall_following=True, disturbance=apply_push)
    df = run_episode(config, seed=42)

    latency = _first_non_normal_latency(df, dp.push_onset, dp.detection_window_s)
    assert latency is None


def test_cautious_mode_scales_down_corridor_speed_reference():
    """Force CAUTIOUS via the tilt-threshold controller (a large initial
    tilt trips it almost immediately, per test_tilt_threshold_controller_
    detection's calibration) and confirm the robot's realized forward
    speed drops relative to a NORMAL-mode run -- CLAUDE.md section 10:
    "CAUTIOUS (speed ref x 0.4 ...)"."""
    normal_config = EpisodeConfig(controller=ControllerType.NAIVE, T=10.0, wall_following=True)
    normal_df = run_episode(normal_config, seed=42)
    normal_progress = normal_df["theta"].iloc[-1] - normal_df["theta"].iloc[0]

    dp = default_disturbance_params()
    cautious_config = EpisodeConfig(
        controller=ControllerType.TILT_THRESHOLD, T=10.0, wall_following=True,
        x0_psi_deg=dp.payload_shift_test_psi0_deg,  # 2 deg -- enough to trip CAUTIOUS quickly, verified in Task 4
    )
    cautious_df = run_episode(cautious_config, seed=42)
    cautious_progress = cautious_df["theta"].iloc[-1] - cautious_df["theta"].iloc[0]

    assert "CAUTIOUS" in set(cautious_df["mode"])
    assert cautious_progress < normal_progress
