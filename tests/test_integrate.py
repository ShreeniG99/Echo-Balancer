import numpy as np

from sim.integrate import rk4_step, simulate


def test_rk4_step_matches_exponential_decay():
    """dx/dt = -x, x(0) = 1 => x(t) = exp(-t). RK4 should be very close over one step."""
    def xdot(x, u):
        return -x

    x0 = np.array([1.0])
    dt = 0.01
    x1 = rk4_step(xdot, x0, u=(), dt=dt)
    assert np.isclose(x1[0], np.exp(-dt), rtol=1e-6)


def test_simulate_holds_input_zero_order_and_matches_step_count():
    def xdot(x, u):
        return np.array([u[0]])  # constant velocity u[0]

    x0 = np.array([0.0])
    dt = 0.1
    n_steps = 10
    xs = simulate(xdot, x0, u_fn=lambda t, x: (2.0,), dt=dt, n_steps=n_steps)

    assert xs.shape == (n_steps + 1, 1)
    assert np.isclose(xs[-1, 0], 2.0 * dt * n_steps, rtol=1e-9)
