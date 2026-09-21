"""Controllers (CLAUDE.md section 9).

design_lqr_balance: the balance LQR on the planar model
[theta, psi, theta_dot, psi_dot] with u = v_l + v_r (build-order step 2).

design_lqr_speed_servo: a separate 5-state integral-augmented LQR for
forward-speed reference tracking (build-order step 5); does not modify
or replace design_lqr_balance.

yaw_p_control, wall_following_control, front_threshold_speed_adjust:
the rest of section 9's wall-following stack (build-order step 5).
yaw_p_control is proportional-only, not PD -- see YawControlParams'
docstring and the plan doc's "Design decision: yaw control is
proportional-only".
"""

import numpy as np
from scipy.linalg import solve_continuous_are

from sim.linearize import linearize_planar
from sim.params import LQRBalanceParams, PlantParams, SpeedServoParams, WallFollowParams, YawControlParams


def design_lqr_balance(p: PlantParams, lqr_p: LQRBalanceParams) -> np.ndarray:
    """Balance-LQR state-feedback gain K (shape (4,)) on the planar model,
    such that u = v_l + v_r = -K @ [theta, psi, theta_dot, psi_dot].
    """
    A, B = linearize_planar(p)
    Q = np.diag([lqr_p.Q_theta, lqr_p.Q_psi, lqr_p.Q_theta_dot, lqr_p.Q_psi_dot])
    R = np.array([[lqr_p.R]])

    B_col = B.reshape(-1, 1)
    P = solve_continuous_are(A, B_col, Q, R)
    K = np.linalg.inv(R) @ B_col.T @ P
    return K.flatten()


def design_lqr_speed_servo(p: PlantParams, speed_servo_p: SpeedServoParams) -> np.ndarray:
    """5-state integral-augmented LQR gain for forward-speed tracking
    (CLAUDE.md section 9: "plus integral of (theta - theta_ref) for speed
    servo"). State [theta-theta_ref, psi, theta_dot-theta_dot_ref, psi_dot,
    integral(theta-theta_ref)], u = v_l + v_r.

    Completely separate from design_lqr_balance -- does not modify or
    replace it. Callers apply K to the *error* state at runtime (theta_ref,
    theta_dot_ref, and the running integral are the caller's
    responsibility, not this function's). Under a ramping theta_ref, the
    substitution theta_dot = e3 + theta_dot_ref introduces constant bias
    terms into the error dynamics (via the nonzero A_planar[2,2] and
    A_planar[3,2] planar-coupling entries) that are absent from this
    function's nominal (theta_ref=0) design -- the closed-loop dynamics
    are NOT identical to the nominal case. What holds regardless: integral
    action drives the tracking-relevant error states (theta, theta_dot,
    psi, psi_dot) to exactly zero at steady state (a Type-1 servo
    rejecting the ramp-induced disturbance), at the cost of a nonzero
    steady-state integrator value (z_eq != 0). Verified numerically --
    see code review discussion, commit c2a25ee follow-up.

    Returns K, shape (5,).
    """
    A_planar, B_planar = linearize_planar(p)
    A_aug = np.zeros((5, 5))
    A_aug[:4, :4] = A_planar
    A_aug[4, 0] = 1.0  # d(integral)/dt = theta - theta_ref
    B_aug = np.zeros(5)
    B_aug[:4] = B_planar

    Q = np.diag([
        speed_servo_p.Q_theta, speed_servo_p.Q_psi, speed_servo_p.Q_theta_dot,
        speed_servo_p.Q_psi_dot, speed_servo_p.Q_integral,
    ])
    R = np.array([[speed_servo_p.R]])

    B_col = B_aug.reshape(-1, 1)
    P = solve_continuous_are(A_aug, B_col, Q, R)
    K = np.linalg.inv(R) @ B_col.T @ P
    return K.flatten()


def yaw_p_control(phi_dot_ref: float, phi_dot: float, yaw_p: YawControlParams) -> float:
    """CLAUDE.md section 9: "Yaw: PD on phi_dot with differential
    voltage." Proportional-only in practice -- see YawControlParams'
    docstring for why (the plant's fast yaw dynamics relative to the
    200Hz control rate make any nonzero Kd destabilize this loop).
    Returns the differential voltage (v_r - v_l).
    """
    return yaw_p.Kp * (phi_dot_ref - phi_dot)


def wall_following_control(
    distance_error: float, prev_distance_error: float, dt: float, wall_p: WallFollowParams
) -> tuple[float, float]:
    """CLAUDE.md section 9: "side ultrasonic distance error -> PD ->
    yaw-rate reference." distance_error = target_distance - side_reading
    (positive when too close to the wall). Returns
    (phi_dot_ref, new_prev_distance_error) for the caller to thread into
    the next call.
    """
    d_error = (distance_error - prev_distance_error) / dt
    phi_dot_ref = wall_p.Kp * distance_error + wall_p.Kd * d_error
    return phi_dot_ref, distance_error


def front_threshold_speed_adjust(front_distance: float, nominal_theta_dot_ref: float, wall_p: WallFollowParams) -> float:
    """CLAUDE.md section 9: "Front ultrasonic below threshold -> slow
    down." """
    if front_distance < wall_p.front_slow_threshold:
        return nominal_theta_dot_ref * wall_p.front_slow_factor
    return nominal_theta_dot_ref
