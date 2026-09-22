"""Tests for sim.params module."""

import math

from sim.params import default_plant_params, LQRBalanceParams, default_lqr_balance_params


def test_default_plant_params_derived_quantities():
    p = default_plant_params()

    assert p.L == p.H / 2
    assert math.isclose(p.Jw, p.m * p.R**2 / 2)
    assert math.isclose(p.Jpsi, p.M * p.L**2 / 3)
    assert math.isclose(p.Jphi, p.M * (p.W**2 + p.D**2) / 12)


def test_plant_params_is_frozen():
    p = default_plant_params()
    try:
        p.M = 1.0
        assert False, "PlantParams should be frozen"
    except AttributeError:
        pass


def test_default_lqr_balance_params():
    lp = default_lqr_balance_params()

    assert lp.Q_theta == 1.0
    assert lp.Q_psi == 1e3
    assert lp.Q_theta_dot == 1.0
    assert lp.Q_psi_dot == 1.0
    assert lp.R == 1e2


def test_lqr_balance_params_is_frozen():
    lp = default_lqr_balance_params()
    try:
        lp.R = 1.0
        assert False, "LQRBalanceParams should be frozen"
    except AttributeError:
        pass


from sim.params import (
    EstimatorParams,
    SensorParams,
    default_estimator_params,
    default_sensor_params,
)


def test_default_sensor_params():
    sp = default_sensor_params()

    assert sp.sigma_gyro == 0.005
    assert sp.gyro_bias_walk == 1e-4
    assert sp.sigma_accel == 0.02
    assert sp.encoder_cpr == 360.0
    assert sp.ultrasonic_sigma == 0.003
    assert sp.ultrasonic_range_min == 0.02
    assert sp.ultrasonic_range_max == 4.0
    assert sp.ultrasonic_dropout_prob == 0.02
    assert sp.ultrasonic_rate_hz == 20.0


def test_sensor_params_is_frozen():
    sp = default_sensor_params()
    try:
        sp.sigma_gyro = 1.0
        assert False, "SensorParams should be frozen"
    except AttributeError:
        pass


def test_default_estimator_params():
    ep = default_estimator_params()

    assert ep.Q_theta == 1e-8
    assert ep.Q_psi == 1e-8
    assert ep.Q_theta_dot == 1e-6
    assert ep.Q_psi_dot == 1e-6
    assert ep.P0_theta == 1e-4
    assert ep.P0_psi == 1e-4
    assert ep.P0_theta_dot == 1e-4
    assert ep.P0_psi_dot == 1e-4
    assert ep.P0_bg == 1e-6


def test_estimator_params_is_frozen():
    ep = default_estimator_params()
    try:
        ep.Q_theta = 1.0
        assert False, "EstimatorParams should be frozen"
    except AttributeError:
        pass


from sim.params import (
    DisturbanceParams,
    GateParams,
    default_disturbance_params,
    default_gate_params,
)


def test_default_gate_params():
    gp = default_gate_params()

    assert gp.N == 200
    assert gp.tau1 == 1300.0
    assert gp.tau2 == 1800.0
    assert gp.tau1_exit == 1040.0
    assert gp.tau2_exit == 1300.0
    assert gp.T_dwell == 0.5


def test_gate_params_is_frozen():
    gp = default_gate_params()
    try:
        gp.tau1 = 1.0
        assert False, "GateParams should be frozen"
    except AttributeError:
        pass


def test_default_disturbance_params():
    dp = default_disturbance_params()

    assert dp.surface_change_onset == 5.0
    assert dp.surface_change_duration == 10.0
    assert dp.surface_change_fw == 0.025
    assert dp.battery_droop_onset == 5.0
    assert dp.battery_droop_duration == 1.0
    assert dp.battery_droop_v_nominal == 7.4
    assert dp.battery_droop_v_drooped == 0.1
    assert dp.battery_droop_companion_push_magnitude == 0.05
    assert dp.push_onset == 5.0
    assert dp.push_magnitude == 0.1
    assert dp.gyro_bias_fault_onset == 5.0
    assert dp.gyro_bias_fault_magnitude == 0.05
    assert dp.accel_noise_fault_onset == 5.0
    assert dp.accel_noise_fault_duration == 10.0
    assert dp.accel_noise_fault_multiplier == 5.0
    assert dp.payload_shift_onset == 5.0
    assert dp.payload_shift_delta_M == 1.0
    assert dp.payload_shift_delta_L == 0.1
    assert dp.payload_shift_test_psi0_deg == 2.0
    assert dp.detection_window_s == 2.0


def test_disturbance_params_is_frozen():
    dp = default_disturbance_params()
    try:
        dp.push_magnitude = 1.0
        assert False, "DisturbanceParams should be frozen"
    except AttributeError:
        pass


def test_gate_params_ordering_invariants():
    """CLAUDE.md section 10: tau1 < tau2, with hysteresis exits below their
    entries. A typo swapping tau1_exit/tau2_exit (or similar) would pass
    every other test in this file and only surface as a mysteriously
    broken 60-second closed-loop gate test -- so pin the ordering directly."""
    gp = default_gate_params()

    assert gp.tau1 < gp.tau2
    assert gp.tau1_exit < gp.tau1
    assert gp.tau2_exit < gp.tau2


from sim.params import (
    CorridorParams,
    SpeedServoParams,
    WallFollowParams,
    YawControlParams,
    default_corridor_params,
    default_speed_servo_params,
    default_wall_follow_params,
    default_yaw_control_params,
)


def test_default_corridor_params():
    cp = default_corridor_params()
    assert cp.width == 1.0


def test_default_corridor_params_ray_parallel_eps():
    cp = default_corridor_params()
    assert cp.ray_parallel_eps == 1e-9


def test_corridor_params_is_frozen():
    cp = default_corridor_params()
    try:
        cp.width = 2.0
        assert False, "CorridorParams should be frozen"
    except AttributeError:
        pass


def test_default_speed_servo_params():
    sp = default_speed_servo_params()
    assert sp.Q_theta == 1.0
    assert sp.Q_psi == 1e3
    assert sp.Q_theta_dot == 1.0
    assert sp.Q_psi_dot == 1.0
    assert sp.Q_integral == 10.0
    assert sp.R == 1e2


def test_speed_servo_params_is_frozen():
    sp = default_speed_servo_params()
    try:
        sp.R = 1.0
        assert False, "SpeedServoParams should be frozen"
    except AttributeError:
        pass


def test_default_yaw_control_params():
    yp = default_yaw_control_params()
    assert yp.Kp == 3.0


def test_yaw_control_params_is_frozen():
    yp = default_yaw_control_params()
    try:
        yp.Kp = 1.0
        assert False, "YawControlParams should be frozen"
    except AttributeError:
        pass


def test_default_wall_follow_params():
    wp = default_wall_follow_params()
    assert wp.target_distance == 0.5
    assert wp.Kp == 2.0
    assert wp.Kd == 0.5
    assert wp.front_slow_threshold == 0.5
    assert wp.front_slow_factor == 0.3


def test_wall_follow_params_is_frozen():
    wp = default_wall_follow_params()
    try:
        wp.Kp = 1.0
        assert False, "WallFollowParams should be frozen"
    except AttributeError:
        pass


import math as _math

from sim.params import (
    RunParams,
    TiltGateParams,
    default_run_params,
    default_speed_servo_params_cautious,
    default_tilt_gate_params,
)


def test_default_run_params():
    rp = default_run_params()
    assert rp.dt_plant == 0.001
    assert rp.dt_control == 0.005
    assert rp.fall_psi_threshold == _math.radians(45.0)
    assert rp.cautious_speed_scale == 0.4
    assert rp.corridor_theta_dot_ref_nominal == 0.3


def test_run_params_is_frozen():
    rp = default_run_params()
    try:
        rp.dt_control = 0.01
        assert False, "RunParams should be frozen"
    except AttributeError:
        pass


def test_default_tilt_gate_params():
    tp = default_tilt_gate_params()
    assert tp.psi1 == 0.0087
    assert tp.psi2 == 0.0175
    assert tp.psi1_exit == 0.006
    assert tp.psi2_exit == 0.012
    assert tp.T_dwell == 0.5


def test_tilt_gate_params_is_frozen():
    tp = default_tilt_gate_params()
    try:
        tp.psi1 = 1.0
        assert False, "TiltGateParams should be frozen"
    except AttributeError:
        pass


def test_default_speed_servo_params_cautious_has_softer_q_than_nominal():
    from sim.params import default_speed_servo_params
    nominal = default_speed_servo_params()
    cautious = default_speed_servo_params_cautious()
    assert cautious.Q_theta == nominal.Q_theta / 5
    assert cautious.Q_psi == nominal.Q_psi / 5
    assert cautious.Q_theta_dot == nominal.Q_theta_dot / 5
    assert cautious.Q_psi_dot == nominal.Q_psi_dot / 5
    assert cautious.Q_integral == nominal.Q_integral / 5
    assert cautious.R == nominal.R  # R unchanged -- only Q is "softer" per CLAUDE.md section 10
