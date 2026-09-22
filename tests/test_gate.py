from sim.gate import GateMode, GateState, initial_gate_state, step_gate
from sim.params import GateParams


def _toy_gate_params() -> GateParams:
    """Small N so test sequences are easy to hand-verify; not the real
    N=200 config (that's exercised by the closed-loop tests later in this
    file)."""
    return GateParams(N=3, tau1=10.0, tau2=20.0, tau1_exit=4.0, tau2_exit=10.0, T_dwell=2.0)


def test_initial_gate_state():
    state = initial_gate_state()

    assert state.mode is GateMode.NORMAL
    assert state.nis_window == ()
    assert state.time_in_mode == 0.0


def test_no_transition_while_window_not_full():
    gp = _toy_gate_params()
    state = initial_gate_state()

    for nis in [100.0, 100.0]:  # only 2 samples; N=3, window never fills
        state, _epsilon = step_gate(state, nis, dt=1.0, gate_p=gp)
        assert state.mode is GateMode.NORMAL


def test_normal_to_cautious_and_back_with_dwell():
    gp = _toy_gate_params()
    state = initial_gate_state()

    # (nis, expected_mode, expected_epsilon) per step, computed by hand-running
    # the exact state machine below -- see plan Task 2 for the derivation.
    expected = [
        (1.0, GateMode.NORMAL, 1.0),
        (1.0, GateMode.NORMAL, 2.0),
        (1.0, GateMode.NORMAL, 3.0),
        (15.0, GateMode.CAUTIOUS, 17.0),  # window=(1,1,15) full, eps=17>tau1=10
        (1.0, GateMode.CAUTIOUS, 17.0),   # window=(1,15,1), dwell blocks exit anyway (eps=17 > tau1_exit=4)
        (1.0, GateMode.CAUTIOUS, 17.0),   # window=(15,1,1)
        (1.0, GateMode.NORMAL, 3.0),      # window=(1,1,1), eps=3<tau1_exit=4, dwell (2.0s) satisfied
        (1.0, GateMode.NORMAL, 3.0),
    ]
    for nis, expected_mode, expected_eps in expected:
        state, eps = step_gate(state, nis, dt=1.0, gate_p=gp)
        assert state.mode is expected_mode
        assert eps == expected_eps


def test_normal_to_halt_skip_on_full_window():
    gp = _toy_gate_params()
    state = initial_gate_state()

    for nis in [1.0, 1.0, 1.0]:
        state, _eps = step_gate(state, nis, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.NORMAL

    state, eps = step_gate(state, 25.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.HALT
    assert eps == 27.0


def test_halt_to_cautious_exit():
    gp = _toy_gate_params()
    state = GateState(mode=GateMode.HALT, nis_window=(100.0, 100.0, 100.0), time_in_mode=5.0)

    state, eps = step_gate(state, 1.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.HALT
    assert eps == 201.0

    state, eps = step_gate(state, 1.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.HALT
    assert eps == 102.0

    state, eps = step_gate(state, 1.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.CAUTIOUS
    assert eps == 3.0


import numpy as np

from sim.control import design_lqr_balance
from sim.disturbances import clip_voltage
from sim.estimator import (
    KalmanState,
    discretize,
    initial_covariance,
    measurement_matrix,
    measurement_noise,
    predict,
    process_noise,
    update,
)
from sim.integrate import rk4_step
from sim.plant import f as plant_f
from sim.params import (
    default_disturbance_params,
    default_estimator_params,
    default_gate_params,
    default_lqr_balance_params,
    default_plant_params,
    default_sensor_params,
)
from sim.sensors import accelerometer, encoder, gyro


def _closed_loop_with_gate(seed, T, apply_disturbance=None, x0_psi_deg=0.0):
    """200Hz/1kHz closed loop (plant + sensors + KF + LQR, as in
    tests/test_estimator.py::_run_nominal_closed_loop) with the gate
    stepped alongside the KF each control tick.

    `apply_disturbance`, if given, is called each tick as
    apply_disturbance(t, x_true, b_g_true, p, sensor_p) ->
    (x_true, b_g_true, p_now, sensor_p_now, v_batt_now), letting a test
    apply a one-shot state kick and/or substitute disturbed params for
    that instant. Returns the list of GateMode values, one per control
    tick.
    """
    p = default_plant_params()
    sensor_p = default_sensor_params()
    est_p = default_estimator_params()
    lqr_p = default_lqr_balance_params()
    gate_p = default_gate_params()
    dp = default_disturbance_params()
    nominal_v_batt = dp.battery_droop_v_nominal
    K = design_lqr_balance(p, lqr_p)

    dt_control = 0.005
    dt_plant = 0.001
    n_sub = int(round(dt_control / dt_plant))

    Ad, Bd = discretize(p, dt_control)
    C = measurement_matrix()
    Q = process_noise(sensor_p, est_p, dt_control)
    R = measurement_noise(sensor_p)

    rng = np.random.default_rng(seed)
    n_steps = int(round(T / dt_control))

    x_true = np.array([0.0, np.radians(x0_psi_deg), 0.0, 0.0, 0.0, 0.0])
    b_g_true = 0.0
    kf = KalmanState(x_hat=np.zeros(5), P=initial_covariance(est_p))
    gate_state = initial_gate_state()
    u_prev = 0.0
    modes = []

    for k in range(n_steps):
        t = k * dt_control
        if apply_disturbance is not None:
            x_true, b_g_true, p_now, sensor_p_now, v_batt_now = apply_disturbance(
                t, x_true, b_g_true, p, sensor_p
            )
        else:
            p_now, sensor_p_now, v_batt_now = p, sensor_p, nominal_v_batt

        def xdotf(x, uu, pp=p_now):
            return plant_f(x, uu[0], uu[1], pp)

        xdot_true = xdotf(x_true, (u_prev / 2, u_prev / 2))
        theta_ddot_true = xdot_true[3]

        theta_enc = encoder(x_true[0], x_true[1], x_true[2], p_now, sensor_p_now)
        psi_dot_meas, b_g_true = gyro(x_true[4], b_g_true, dt_control, sensor_p_now, rng)
        psi_acc_meas = accelerometer(x_true[1], theta_ddot_true, p_now, sensor_p_now, rng)
        y = np.array([theta_enc, psi_dot_meas, psi_acc_meas])

        kf = predict(kf, Ad, Bd, u_prev, Q)
        kf, nis = update(kf, y, C, R)
        gate_state, _epsilon = step_gate(gate_state, nis, dt_control, gate_p)
        modes.append(gate_state.mode)

        u_cmd = -K @ kf.x_hat[:4]
        u = clip_voltage(u_cmd, v_batt_now)
        u_prev = u
        for _ in range(n_sub):
            x_true = rk4_step(xdotf, x_true, (u / 2, u / 2), dt_plant)

    return modes


def test_no_mode_change_on_nominal_60s_run():
    """Section 11: 'Gate: no mode change on a nominal 60s run (seeded).'

    Verified with seed=42 here, and independently with seeds 0-9 before
    this plan was written (see plan doc's threshold-calibration table) --
    max epsilon across all 10 was 959.1, safely under GateParams.tau1=1300.
    """
    modes = _closed_loop_with_gate(seed=42, T=60.0)

    assert set(modes) == {GateMode.NORMAL}


from sim.disturbances import (
    battery_droop_v_batt,
    payload_shift_plant_params,
    push_psi_dot_kick,
    sensor_fault_accel_noise_params,
    sensor_fault_gyro_bias_step,
    surface_change_plant_params,
)


def _assert_enters_cautious_within(modes, onset, window_s, dt_control=0.005):
    onset_idx = int(round(onset / dt_control))
    deadline_idx = onset_idx + int(round(window_s / dt_control))
    first_non_normal = next(
        (i for i in range(onset_idx, min(deadline_idx, len(modes))) if modes[i] is not GateMode.NORMAL),
        None,
    )
    assert first_non_normal is not None, f"never left NORMAL within {window_s}s of onset (t={onset}s)"


def test_enters_cautious_after_push():
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        x_true = x_true.copy()
        x_true[4] = push_psi_dot_kick(x_true[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x_true, b_g_true, p, sensor_p, dp.battery_droop_v_nominal

    modes = _closed_loop_with_gate(seed=42, T=20.0, apply_disturbance=apply_disturbance)
    _assert_enters_cautious_within(modes, onset=dp.push_onset, window_s=dp.detection_window_s)


def test_enters_cautious_after_surface_change():
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        p_now = surface_change_plant_params(
            t, dp.surface_change_onset, dp.surface_change_duration, dp.surface_change_fw, p
        )
        return x_true, b_g_true, p_now, sensor_p, dp.battery_droop_v_nominal

    modes = _closed_loop_with_gate(seed=42, T=20.0, apply_disturbance=apply_disturbance)
    _assert_enters_cautious_within(modes, onset=dp.surface_change_onset, window_s=dp.detection_window_s)


def test_enters_cautious_after_gyro_bias_fault():
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        b_g_true = sensor_fault_gyro_bias_step(
            b_g_true, t, dp.gyro_bias_fault_onset, 0.005, dp.gyro_bias_fault_magnitude
        )
        return x_true, b_g_true, p, sensor_p, dp.battery_droop_v_nominal

    modes = _closed_loop_with_gate(seed=42, T=20.0, apply_disturbance=apply_disturbance)
    _assert_enters_cautious_within(modes, onset=dp.gyro_bias_fault_onset, window_s=dp.detection_window_s)


def test_enters_cautious_after_accel_noise_fault():
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        sensor_p_now = sensor_fault_accel_noise_params(
            t, dp.accel_noise_fault_onset, dp.accel_noise_fault_duration, dp.accel_noise_fault_multiplier, sensor_p
        )
        return x_true, b_g_true, p, sensor_p_now, dp.battery_droop_v_nominal

    modes = _closed_loop_with_gate(seed=42, T=20.0, apply_disturbance=apply_disturbance)
    _assert_enters_cautious_within(modes, onset=dp.accel_noise_fault_onset, window_s=dp.detection_window_s)


def test_enters_cautious_after_payload_shift():
    """Needs a mild concurrent tilt to expose the mass/CoM mismatch --
    near-perfect equilibrium doesn't accelerate enough for even this large
    (dM=1kg, dL=0.1m) shift to move the NIS. See plan doc: substantially
    smaller, more "realistic" shifts (dM up to 0.5kg, dL up to 0.05m) did
    not trigger detection at all, even combined with tilt -- a genuine,
    reportable sensitivity limit of the current tuning, not a bug."""
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        p_now = payload_shift_plant_params(
            t, dp.payload_shift_onset, dp.payload_shift_delta_M, dp.payload_shift_delta_L, p
        )
        return x_true, b_g_true, p_now, sensor_p, dp.battery_droop_v_nominal

    modes = _closed_loop_with_gate(
        seed=42, T=20.0, apply_disturbance=apply_disturbance, x0_psi_deg=dp.payload_shift_test_psi0_deg
    )
    _assert_enters_cautious_within(modes, onset=dp.payload_shift_onset, window_s=dp.detection_window_s)


def test_enters_cautious_after_battery_droop():
    """Battery droop alone near equilibrium never binds the voltage
    ceiling (nominal commands are far below even a badly drooped
    battery's limit) -- see plan doc. Paired here with a small companion
    push, itself independently verified to be too small to trigger
    detection on its own, to give the controller genuine voltage demand
    the drooped battery can't fully supply.

    The companion push fires at battery_droop_onset + battery_droop_duration
    (once the ramp has actually finished, not at the droop's own onset):
    battery_droop_v_batt RAMPS from v_nominal to v_drooped over that
    duration (CLAUDE.md: "V_batt ramps down"), so at t=battery_droop_onset
    itself V_batt is still ~v_nominal -- a push applied there would hit an
    essentially-undrooped battery and prove nothing about degraded
    recovery. Detection is measured from this same push-onset reference,
    not from battery_droop_onset, since that's when the actual
    controller-relevant event happens."""
    dp = default_disturbance_params()
    push_onset = dp.battery_droop_onset + dp.battery_droop_duration

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        v_batt = battery_droop_v_batt(
            t, dp.battery_droop_onset, dp.battery_droop_duration, dp.battery_droop_v_nominal, dp.battery_droop_v_drooped
        )
        x_true = x_true.copy()
        x_true[4] = push_psi_dot_kick(
            x_true[4], t, push_onset, 0.005, dp.battery_droop_companion_push_magnitude
        )
        return x_true, b_g_true, p, sensor_p, v_batt

    modes = _closed_loop_with_gate(seed=42, T=20.0, apply_disturbance=apply_disturbance)
    _assert_enters_cautious_within(modes, onset=push_onset, window_s=dp.detection_window_s)


from sim.gate import TiltGateState, initial_tilt_gate_state, step_tilt_gate
from sim.params import TiltGateParams


def _toy_tilt_gate_params() -> TiltGateParams:
    """Small, easy-to-hand-verify thresholds; not the real calibrated
    values (those are exercised by the run_episode-level tests in
    tests/test_run.py)."""
    return TiltGateParams(psi1=1.0, psi2=2.0, psi1_exit=0.4, psi2_exit=1.0, T_dwell=2.0)


def test_initial_tilt_gate_state():
    state = initial_tilt_gate_state()
    assert state.mode is GateMode.NORMAL
    assert state.time_in_mode == 0.0


def test_tilt_gate_normal_to_cautious_and_back_with_dwell():
    tgp = _toy_tilt_gate_params()
    state = initial_tilt_gate_state()

    # (psi_hat, expected_mode) per step, hand-derived from step_tilt_gate's
    # logic -- no windowing, so this is simpler than step_gate's toy sequence.
    expected = [
        (0.5, GateMode.NORMAL),   # time_in_mode=1.0, dwell (2.0) not yet satisfied
        (0.5, GateMode.NORMAL),   # time_in_mode=2.0, dwell satisfied, |0.5|<=psi1(1.0): stays NORMAL
        (1.5, GateMode.CAUTIOUS),  # time_in_mode=3.0, |1.5|>psi1: -> CAUTIOUS, time_in_mode resets to 0
        (1.5, GateMode.CAUTIOUS),  # time_in_mode=1.0, dwell blocks any transition
        (0.2, GateMode.NORMAL),   # time_in_mode=2.0, dwell satisfied, |0.2|<psi1_exit(0.4): -> NORMAL
    ]
    for psi_hat, expected_mode in expected:
        state = step_tilt_gate(state, psi_hat, dt=1.0, tilt_gate_p=tgp)
        assert state.mode is expected_mode


def test_tilt_gate_normal_to_halt_direct():
    tgp = _toy_tilt_gate_params()
    state = TiltGateState(mode=GateMode.NORMAL, time_in_mode=5.0)  # dwell already satisfied

    state = step_tilt_gate(state, 2.5, dt=1.0, tilt_gate_p=tgp)

    assert state.mode is GateMode.HALT
    assert state.time_in_mode == 0.0


def test_tilt_gate_halt_to_cautious_exit():
    tgp = _toy_tilt_gate_params()
    state = TiltGateState(mode=GateMode.HALT, time_in_mode=5.0)  # dwell already satisfied

    state = step_tilt_gate(state, 0.5, dt=1.0, tilt_gate_p=tgp)

    assert state.mode is GateMode.CAUTIOUS
    assert state.time_in_mode == 0.0
