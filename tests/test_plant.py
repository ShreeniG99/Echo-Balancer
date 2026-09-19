from dataclasses import replace

import numpy as np

from sim.integrate import simulate
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


def _total_energy(x: np.ndarray, p) -> float:
    """Mechanical energy T + V for the plant (see plan Task 5 derivation)."""
    _theta, psi, _phi, theta_dot, psi_dot, phi_dot = x

    A11 = (2 * p.m + p.M) * p.R**2 + 2 * p.Jw + 2 * p.n**2 * p.Jm
    A12 = p.M * p.L * p.R * np.cos(psi) - 2 * p.n**2 * p.Jm
    A22 = p.M * p.L**2 + p.Jpsi + 2 * p.n**2 * p.Jm
    A33 = (
        0.5 * p.m * p.W**2
        + p.Jphi
        + (p.W**2 / (2 * p.R**2)) * (p.Jw + p.n**2 * p.Jm)
        + p.M * p.L**2 * np.sin(psi) ** 2
    )

    T = (
        0.5 * A11 * theta_dot**2
        + A12 * theta_dot * psi_dot
        + 0.5 * A22 * psi_dot**2
        + 0.5 * A33 * phi_dot**2
    )
    V = p.M * p.g * p.L * np.cos(psi)
    return T + V


def test_energy_conserved_no_input_no_friction():
    # "no friction, f_m = 0" (f_w, f_m) is not enough on its own: beta still
    # contains the motor back-EMF damping term n^2*Kt*Kb/Rm, which dissipates
    # energy even at zero applied voltage. Zero it out via Kb for this
    # idealized conservative-system check (see plan "Design decision").
    p = replace(default_plant_params(), fw=0.0, fm=0.0, Kb=0.0)

    dt = 1e-3
    n_steps = 5000  # 5 s
    x0 = np.array([0.0, np.radians(5.0), 0.0, 0.0, 0.0, 0.0])

    def xdot(x, u):
        return f(x, u[0], u[1], p)

    xs = simulate(xdot, x0, u_fn=lambda t, x: (0.0, 0.0), dt=dt, n_steps=n_steps)
    energies = np.array([_total_energy(x, p) for x in xs])

    e0 = energies[0]
    max_rel_dev = np.max(np.abs(energies - e0) / abs(e0))
    assert max_rel_dev < 1e-3


def test_falls_without_control():
    p = default_plant_params()
    dt = 1e-3
    n_steps = 2000  # 2 s; open-loop unstable pole ~7.44 rad/s, falls fast
    x0 = np.array([0.0, np.radians(1.0), 0.0, 0.0, 0.0, 0.0])

    def xdot(x, u):
        return f(x, u[0], u[1], p)

    xs = simulate(xdot, x0, u_fn=lambda t, x: (0.0, 0.0), dt=dt, n_steps=n_steps)
    psi = xs[:, 1]

    assert abs(psi[-1]) > abs(psi[0])
    assert np.max(np.abs(psi)) > np.radians(45.0)
