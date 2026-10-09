import numpy as np
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


def test_corridor_push_missed_by_long_window_only_caught_by_short_window():
    """The plan doc's "Design decision 1" finding, kept on record: with the long window alone, a push
    during corridor driving is MISSED (seed=42: max epsilon in the detection window 1085.51 < tau1=1300;
    a thin, inherited margin, not a step-6 regression). The short-window spike detector, part of the
    default gate since 2026-10-09, catches the same push on the same seed."""
    from sim.params import default_gate_params, long_window_only_gate_params

    dp = default_disturbance_params()

    def apply_push(t, x, bg, p, sp):
        x = x.copy()
        x[4] = push_psi_dot_kick(x[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x, bg, p, sp, dp.battery_droop_v_nominal

    def latency(gate_p):
        config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=20.0, wall_following=True,
                               disturbance=apply_push, gate_params=gate_p)
        return _first_non_normal_latency(run_episode(config, seed=42), dp.push_onset, dp.detection_window_s)

    assert latency(long_window_only_gate_params()) is None
    caught = latency(default_gate_params())
    assert caught is not None and caught < 0.1


def test_cautious_mode_scales_down_corridor_speed_reference():
    """CLAUDE.md section 10: "CAUTIOUS (speed ref x 0.4 ...)". CAUTIOUS is forced
    deterministically with a GateParams override (tau1 below any epsilon, tau2
    unreachable), so both runs share the same seed AND initial condition and only
    the mode differs. (The previous version relied on a 2 deg initial tilt tripping
    the tilt gate; that never survives the 0.5 s dwell -- it was ultrasonic-dropout
    yaw kicks that tripped it, fixed by sim.sensors.hold_on_dropout.)"""
    from sim.params import GateParams

    normal_df = run_episode(EpisodeConfig(controller=ControllerType.NAIVE, T=10.0, wall_following=True), seed=42)
    normal_progress = normal_df["theta"].iloc[-1] - normal_df["theta"].iloc[0]

    always_cautious = GateParams(N=20, tau1=1e-6, tau2=1e12, tau1_exit=0.0, tau2_exit=1e11, T_dwell=0.5)
    cautious_df = run_episode(
        EpisodeConfig(controller=ControllerType.NIS_GATE, T=10.0, wall_following=True, gate_params=always_cautious),
        seed=42,
    )
    cautious_progress = cautious_df["theta"].iloc[-1] - cautious_df["theta"].iloc[0]

    assert (cautious_df["mode"].iloc[200:] == "CAUTIOUS").all()
    assert cautious_progress < 0.7 * normal_progress


def test_side_ultrasonic_dropout_does_not_kick_yaw():
    """A 2% dropout returns max range; held by hold_on_dropout it must not reach the
    wall-following loop. Regression: before the fix, nominal 60 s corridor runs fell
    in 2/10 (naive) to 5/10 (tilt gate) seeds; naive seed 3 was one of them."""
    from sim.params import default_sensor_params
    from sim.sensors import hold_on_dropout

    sp = default_sensor_params()
    assert hold_on_dropout(sp.ultrasonic_range_max, 0.48, sp) == 0.48
    assert hold_on_dropout(0.51, 0.48, sp) == 0.51
    assert hold_on_dropout(sp.ultrasonic_range_max, None, sp) == sp.ultrasonic_range_max

    df = run_episode(EpisodeConfig(controller=ControllerType.NAIVE, T=60.0, wall_following=True), seed=3)
    assert not df["fallen"].any()
    assert np.degrees(np.abs(df["phi"])).max() < 30.0


def test_halt_mode_zeroes_corridor_speed_reference():
    """CLAUDE.md section 10: HALT sets speed ref = 0, 'keep balancing in
    place' -- verify this end-to-end in a wall-following episode (the
    `theta_dot_ref, K5 = 0.0, K5_nominal` branch in sim/run.py), not just
    at the toy step_tilt_gate level (tests/test_gate.py).

    x0_psi_deg=15.0 (with T=8.0s) was determined empirically: it is the
    smallest tested initial tilt that reliably drives the tilt-threshold
    gate all the way to HALT (10 deg only reaches CAUTIOUS), while staying
    comfortably under the 45 deg fall threshold. seed=42, matching every
    other test in this file.

    Finding (code review, step 6): HALT is NOT reached instantly. The
    tilt-threshold gate's T_dwell=0.5s (TiltGateParams) blocks any
    transition -- including the very first NORMAL->HALT one -- until 0.5s
    have elapsed, so the robot spends that first 0.5s recovering from the
    15 deg tilt under ordinary NORMAL (full-speed) control before HALT
    ever engages; by then psi has mostly recovered (~1.3deg) but the pitch
    recovery transient (dominant closed-loop pole ~-0.3 to -0.4 rad/s,
    time constant ~2.5-4s) is far from settled. Consequently theta_dot
    during the subsequent 0.5s HALT window does NOT sit near zero -- it
    overshoots to ~+-0.85 rad/s (LARGER in magnitude than the 0.3 rad/s
    NORMAL-mode cruise speed, RunParams.corridor_theta_dot_ref_nominal),
    because freezing theta_ref (theta_dot_ref=0) is itself a step change
    the position-tracking LQR must react to. This was cross-checked by
    deliberately reintroducing two candidate HALT bugs (theta_dot_ref =
    base_ref instead of 0.0; K5 = K5_cautious instead of K5_nominal) --
    both produced SMALLER peak |theta_dot| (~0.30 and ~0.56 rad/s
    respectively) than the correct code's ~0.85 rad/s, because they avoid
    that reference-step discontinuity. So a tight "theta_dot stays near
    zero" bound is not a reliable regression guard for this specific
    mistake and is deliberately not used here; what IS a robust guard
    (and what this test asserts) is that the branch executes without
    diverging: HALT is reached, the episode never falls, no state goes
    non-finite, theta_dot stays well-bounded (< 3.0 rad/s, comfortably
    below the ~7.8 rad/s peak seen during the initial 15 deg free-tilt
    recovery itself, so still catches a genuine blow-up), and the gate
    later returns all the way to NORMAL, demonstrating the closed loop is
    stable rather than stuck or diverging."""
    config = EpisodeConfig(
        controller=ControllerType.TILT_THRESHOLD, T=8.0, wall_following=True,
        x0_psi_deg=15.0,
    )
    df = run_episode(config, seed=42)

    assert "HALT" in set(df["mode"])
    assert not df["fallen"].any()
    assert np.isfinite(df[["theta", "psi", "phi", "theta_dot", "psi_dot", "phi_dot"]].to_numpy()).all()

    halt_rows = df[df["mode"] == "HALT"]
    assert not halt_rows.empty
    # Bounded, not diverging -- see docstring for why a tight near-zero
    # bound is not the right (or a reliable) check here.
    assert halt_rows["theta_dot"].abs().max() < 3.0

    # The gate recovers all the way back to NORMAL later in the episode --
    # evidence the HALT branch leaves the closed loop stable, not stuck.
    assert "NORMAL" in set(df["mode"].iloc[halt_rows.index[-1]:])


def test_fallback_prevents_payload_shift_fall():
    """FallbackParams (softer HALT gains + KF Q inflation outside NORMAL), ported
    from the firmware: seed 1's payload shift falls without it and is held upright
    with it (nonlinear plant). The gate trips at the same time either way."""
    from experiments.run_batch import DISTURBANCE_SCENARIOS

    fn, T, _, psi0 = DISTURBANCE_SCENARIOS["payload_shift"]
    runs = {
        fb: run_episode(EpisodeConfig(ControllerType.NIS_GATE, T, x0_psi_deg=psi0, disturbance=fn, fallback=fb), seed=1)
        for fb in (False, True)
    }
    assert runs[False]["fallen"].any()
    assert not runs[True]["fallen"].any()
    assert abs(runs[True]["psi"]).max() < np.radians(15.0)
    first_trip = {fb: df.loc[df["mode"] != "NORMAL", "t"].iloc[0] for fb, df in runs.items()}
    assert first_trip[False] == first_trip[True]
