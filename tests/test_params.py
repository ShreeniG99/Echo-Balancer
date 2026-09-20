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
