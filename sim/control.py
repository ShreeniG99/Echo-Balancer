"""Balance controller (CLAUDE.md section 9).

This step implements only the balance LQR on the planar model
[theta, psi, theta_dot, psi_dot] with u = v_l + v_r. The speed-servo
integral term, yaw PD, and wall-following described in section 9 are not
part of section 14 build-order step 2 and are not implemented here.
"""

import numpy as np
from scipy.linalg import solve_continuous_are

from sim.linearize import linearize_planar
from sim.params import LQRBalanceParams, PlantParams


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
