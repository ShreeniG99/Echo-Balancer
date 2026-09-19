import numpy as np

from sim.control import design_lqr_balance
from sim.integrate import simulate
from sim.linearize import linearize_planar
from sim.params import default_lqr_balance_params, default_plant_params
from sim.plant import f as plant_f


def test_lqr_gain_matches_sanity_value():
    p = default_plant_params()
    lqr_p = default_lqr_balance_params()
    K = design_lqr_balance(p, lqr_p)

    # CLAUDE.md section 6.1: K ~= [-0.10, -49.3, -2.09, -4.55]
    target = np.array([-0.10, -49.3, -2.09, -4.55])
    np.testing.assert_allclose(K, target, rtol=1e-2, atol=0.05)


def test_lqr_closed_loop_poles_are_stable():
    p = default_plant_params()
    lqr_p = default_lqr_balance_params()
    K = design_lqr_balance(p, lqr_p)

    A_planar, B_planar = linearize_planar(p)
    A_closed_loop = A_planar - np.outer(B_planar, K)
    eigs = np.linalg.eigvals(A_closed_loop)

    assert np.all(eigs.real < 0)


def test_recovers_from_10_degree_pitch_in_nonlinear_sim():
    """Section 11: 'LQR closed-loop poles all Re < 0; recovers from psi0 =
    10 deg in the nonlinear sim.' Full 6-state plant.f, RK4 at dt=1ms,
    full-state feedback u=-K@[theta,psi,theta_dot,psi_dot] recomputed every
    step (see plan Task 4 "Design decision" -- this is not yet the 200 Hz
    ZOH loop from section 7, which needs sim/run.py)."""
    p = default_plant_params()
    lqr_p = default_lqr_balance_params()
    K = design_lqr_balance(p, lqr_p)

    def xdot(x, u):
        return plant_f(x, u[0], u[1], p)

    def u_fn(t, x):
        planar_state = np.array([x[0], x[1], x[3], x[4]])
        u = -K @ planar_state
        return (u / 2.0, u / 2.0)

    dt = 1e-3
    n_steps = 3000  # 3 s
    psi0_deg = 10.0
    x0 = np.array([0.0, np.radians(psi0_deg), 0.0, 0.0, 0.0, 0.0])

    xs = simulate(xdot, x0, u_fn, dt, n_steps)
    psi_deg = np.degrees(xs[:, 1])

    # Never falls, and never overshoots its own starting tilt.
    assert np.max(np.abs(psi_deg)) <= psi0_deg + 1e-6
    # Settles close to upright well before the fall threshold matters.
    assert abs(psi_deg[-1]) < 0.1
