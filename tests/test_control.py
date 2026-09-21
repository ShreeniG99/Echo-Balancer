import numpy as np
import pytest

from sim.control import design_lqr_balance, design_lqr_speed_servo
from sim.control import front_threshold_speed_adjust, wall_following_control, yaw_p_control
from sim.integrate import rk4_step, simulate
from sim.linearize import linearize_planar
from sim.params import default_lqr_balance_params, default_plant_params, default_speed_servo_params
from sim.params import YawControlParams, default_wall_follow_params
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
    ZOH loop from section 7, which needs sim/run.py).

    Note: at psi0=10 deg, sin(psi) and psi differ by <0.1%, so this stays
    close to the linear regime the poles test already covers analytically.
    Its real standalone value is as an end-to-end integration check (plant
    + integrator + controller wired together correctly), not a nonlinear
    stress test -- a future test at a larger angle (nearer the 45 deg fall
    limit) would be the one to actually probe nonlinear robustness.
    """
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

    # Never falls, and never overshoots its own starting tilt. This
    # no-overshoot bound is a property of the current default Q/R producing
    # an overdamped, non-oscillatory closed loop -- not a general LQR
    # guarantee. If LQRBalanceParams' defaults are ever retuned and this
    # starts failing, that's a signal to re-derive expectations here, not
    # necessarily evidence of a regression.
    assert np.max(np.abs(psi_deg)) <= psi0_deg + 1e-6
    # Settles close to upright well before the fall threshold matters.
    assert abs(psi_deg[-1]) < 0.1


def test_speed_servo_gain_matches_reference():
    p = default_plant_params()
    sp = default_speed_servo_params()

    K = design_lqr_speed_servo(p, sp)

    # Verified against this repo's actual sim/linearize.py before this plan
    # was written -- see plan doc "Design decision: a real speed servo LQR
    # is required".
    target = np.array([-0.90627756, -53.30474389, -2.31169767, -5.01088606, -0.31622777])
    np.testing.assert_allclose(K, target, rtol=1e-4)


def test_speed_servo_closed_loop_poles_stable():
    p = default_plant_params()
    sp = default_speed_servo_params()
    K = design_lqr_speed_servo(p, sp)

    A_planar, B_planar = linearize_planar(p)
    A_aug = np.zeros((5, 5))
    A_aug[:4, :4] = A_planar
    A_aug[4, 0] = 1.0
    B_aug = np.zeros(5)
    B_aug[:4] = B_planar

    eigs = np.linalg.eigvals(A_aug - np.outer(B_aug, K))
    assert np.all(eigs.real < 0)


def test_speed_servo_tracks_constant_forward_speed_in_nonlinear_sim():
    """Verified pre-plan: steady-state theta_dot tracking error was
    0.006/0.018/0.030 at theta_dot_ref=1/3/5 rad/s over 15s runs -- a
    generous margin (0.1) is used here, well above any of those, to avoid
    a flaky bound while still catching a real transcription bug (e.g. the
    42% error the non-integral design produced, which this margin would
    clearly reject)."""
    p = default_plant_params()
    sp = default_speed_servo_params()
    K = design_lqr_speed_servo(p, sp)

    dt_plant, dt_control, n_sub = 0.001, 0.005, 5
    theta_dot_ref = 3.0

    def xdotf(x, uu):
        return plant_f(x, uu[0], uu[1], p)

    x = np.zeros(6)
    theta_ref = 0.0
    z = 0.0
    n_steps = int(15.0 / dt_control)
    theta_dot_hist = []
    for _ in range(n_steps):
        theta_ref += theta_dot_ref * dt_control
        err = np.array([x[0] - theta_ref, x[1], x[3] - theta_dot_ref, x[4], z])
        u = -K @ err
        z += (x[0] - theta_ref) * dt_control
        theta_dot_hist.append(x[3])
        for _ in range(n_sub):
            x = rk4_step(xdotf, x, (u / 2, u / 2), dt_plant)

    steady_theta_dot = np.mean(theta_dot_hist[-200:])
    assert abs(steady_theta_dot - theta_dot_ref) < 0.1


def test_yaw_p_control_formula():
    yaw_p = YawControlParams(Kp=3.0)

    assert yaw_p_control(1.0, 0.4, yaw_p) == pytest.approx(3.0 * (1.0 - 0.4))
    assert yaw_p_control(0.0, 0.0, yaw_p) == 0.0


def test_wall_following_control_formula_and_state_threading():
    wall_p = default_wall_follow_params()

    phi_dot_ref, new_prev_error = wall_following_control(0.2, 0.1, 0.05, wall_p)

    expected = wall_p.Kp * 0.2 + wall_p.Kd * (0.2 - 0.1) / 0.05
    assert phi_dot_ref == pytest.approx(expected)
    assert new_prev_error == 0.2


def test_front_threshold_speed_adjust_slows_down_below_threshold():
    wall_p = default_wall_follow_params()

    slowed = front_threshold_speed_adjust(0.3, 2.0, wall_p)
    unaffected = front_threshold_speed_adjust(1.0, 2.0, wall_p)

    assert slowed == pytest.approx(2.0 * wall_p.front_slow_factor)
    assert unaffected == 2.0
