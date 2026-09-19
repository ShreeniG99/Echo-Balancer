import numpy as np

from sim.params import default_plant_params
from sim.plant import f


def test_equilibrium_is_fixed_point():
    """Upright, at rest, no input: all accelerations must be zero."""
    p = default_plant_params()
    x = np.zeros(6)
    xdot = f(x, 0.0, 0.0, p)
    assert np.allclose(xdot, 0.0)


def test_state_derivative_shape():
    p = default_plant_params()
    x = np.array([0.1, 0.05, 0.2, 0.3, 0.1, 0.4])
    xdot = f(x, 1.0, -1.0, p)
    assert xdot.shape == (6,)
