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
