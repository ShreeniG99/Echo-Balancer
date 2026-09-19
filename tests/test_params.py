"""Tests for sim.params module."""

import math

from sim.params import default_plant_params


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
