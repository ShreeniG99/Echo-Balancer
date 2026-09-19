import numpy as np

from sim.integrate import rk4_step, simulate


def test_rk4_step_is_fourth_order_accurate():
    """One-step local truncation error for RK4 is O(dt^5), so halving dt
    should shrink the error by ~2^5=32x. This distinguishes RK4 from lower-
    order methods, which a looser tolerance on a single dt cannot."""
    def xdot(x, u):
        return -x

    x0 = np.array([1.0])

    def error_at(dt):
        x1 = rk4_step(xdot, x0, u=(), dt=dt)
        return abs(x1[0] - np.exp(-dt))

    dt = 0.1
    e_full = error_at(dt)
    e_half = error_at(dt / 2)
    ratio = e_full / e_half
    assert 28.0 < ratio < 36.0, f"expected ~32x error reduction (5th-order LTE), got {ratio}x"


def test_zero_order_hold_freezes_u_across_rk4_stages():
    """Verify that u is sampled exactly once per step (ZOH), not on every RK4 stage."""
    u_calls = []
    f_calls = []

    def u_fn(t, x):
        u_calls.append(t)
        return (t,)

    def xdot(x, u):
        f_calls.append(u[0])
        return np.array([u[0]])

    x0 = np.array([0.0])
    dt = 0.1
    n_steps = 5
    xs = simulate(xdot, x0, u_fn=u_fn, dt=dt, n_steps=n_steps)

    # Verify u_fn was called exactly once per step, not once per RK4 stage
    assert len(u_calls) == n_steps, f"u_fn called {len(u_calls)} times, expected {n_steps}"

    # Verify dynamics function was called 4x per step (RK4 stages)
    assert len(f_calls) == 4 * n_steps, f"xdot called {len(f_calls)} times, expected {4 * n_steps}"

    # Verify u held constant across all 4 RK4 stages within each step
    for i in range(n_steps):
        stage_us = f_calls[4 * i : 4 * i + 4]
        assert all(
            u == stage_us[0] for u in stage_us
        ), f"step {i}: u not held constant across stages {stage_us}"

    # Verify output shape and step count
    assert xs.shape == (n_steps + 1, 1), f"expected shape ({n_steps + 1}, 1), got {xs.shape}"
