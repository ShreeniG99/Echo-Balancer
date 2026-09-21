"""Balance controller (CLAUDE.md section 9).

This step implements only the balance LQR on the planar model
[theta, psi, theta_dot, psi_dot] with u = v_l + v_r. The speed-servo
integral term, yaw PD, and wall-following described in section 9 are not
part of section 14 build-order step 2 and are not implemented here.
"""

import numpy as np
from scipy.linalg import solve_continuous_are

from sim.linearize import linearize_planar
from sim.params import LQRBalanceParams, PlantParams, SpeedServoParams


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
    responsibility, not this function's), since the closed-loop error
    dynamics under a constant-rate reference match this nominal
    (theta_ref=0) design exactly -- see the plan doc.
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
