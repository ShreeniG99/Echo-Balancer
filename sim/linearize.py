"""Analytic linearization of the plant at the upright equilibrium
(CLAUDE.md sections 5 and 9), cross-checked against a numeric Jacobian of
sim.plant.f in tests/test_linearize.py.

At x=0 (upright, at rest), u=(0,0), the plant is a fixed point (see
tests/test_plant.py). The generalized forces F_theta/F_psi/F_phi are already
exactly linear in (theta_dot, psi_dot, phi_dot, v_l, v_r); the only nonlinear
term surviving to first order is M*g*L*sin(psi) ~= M*g*L*psi in F_psi. Because
of this, the planar (theta, psi) block and the yaw (phi) block decouple
completely at linearization: common-mode drive (v_l + v_r) only ever affects
theta/psi, differential drive (v_r - v_l) only ever affects phi. This module
computes each block directly rather than deriving one large symbolic
Jacobian by hand.
"""

import numpy as np

from sim.params import PlantParams


def linearize(p: PlantParams) -> tuple[np.ndarray, np.ndarray]:
    """Analytic Jacobian of sim.plant.f at x=0, u=(0, 0).

    Returns (A, B) for the full 6-state system,
    x = [theta, psi, phi, theta_dot, psi_dot, phi_dot], u = [v_l, v_r].
    """
    alpha = p.n * p.Kt / p.Rm
    beta = p.n**2 * p.Kt * p.Kb / p.Rm + p.fm

    A11 = (2 * p.m + p.M) * p.R**2 + 2 * p.Jw + 2 * p.n**2 * p.Jm
    A12 = p.M * p.L * p.R - 2 * p.n**2 * p.Jm  # cos(0) = 1
    A22 = p.M * p.L**2 + p.Jpsi + 2 * p.n**2 * p.Jm
    A33 = (
        0.5 * p.m * p.W**2
        + p.Jphi
        + (p.W**2 / (2 * p.R**2)) * (p.Jw + p.n**2 * p.Jm)
    )  # sin(0) = 0

    M_planar = np.array([[A11, A12], [A12, A22]])
    M_planar_inv = np.linalg.inv(M_planar)

    # Columns: [psi, theta_dot, psi_dot]. Rows: [b_theta, b_psi].
    d_b_d_state = np.array([
        [0.0, -2 * (beta + p.fw), 2 * beta],
        [p.M * p.g * p.L, 2 * beta, -2 * beta],
    ])
    # Columns: [v_l, v_r]. Rows: [b_theta, b_psi].
    d_b_d_input = np.array([
        [alpha, alpha],
        [-alpha, -alpha],
    ])

    d_qddot_d_state = M_planar_inv @ d_b_d_state  # 2x3: [theta_ddot; psi_ddot]
    d_qddot_d_input = M_planar_inv @ d_b_d_input  # 2x2

    d_F_phi_d_phidot = -(p.W**2 / (2 * p.R**2)) * (beta + p.fw)
    d_F_phi_d_input = np.array([-(p.W / (2 * p.R)) * alpha, (p.W / (2 * p.R)) * alpha])
    d_phiddot_d_phidot = d_F_phi_d_phidot / A33
    d_phiddot_d_input = d_F_phi_d_input / A33

    A = np.zeros((6, 6))
    B = np.zeros((6, 2))

    A[0, 3] = 1.0  # d(theta)/dt = theta_dot
    A[1, 4] = 1.0  # d(psi)/dt = psi_dot
    A[2, 5] = 1.0  # d(phi)/dt = phi_dot

    A[3, 1] = d_qddot_d_state[0, 0]
    A[3, 3] = d_qddot_d_state[0, 1]
    A[3, 4] = d_qddot_d_state[0, 2]
    A[4, 1] = d_qddot_d_state[1, 0]
    A[4, 3] = d_qddot_d_state[1, 1]
    A[4, 4] = d_qddot_d_state[1, 2]

    B[3, :] = d_qddot_d_input[0, :]
    B[4, :] = d_qddot_d_input[1, :]

    A[5, 5] = d_phiddot_d_phidot
    B[5, :] = d_phiddot_d_input

    return A, B


def linearize_planar(p: PlantParams) -> tuple[np.ndarray, np.ndarray]:
    """Reduced planar balance model (CLAUDE.md section 9): state
    [theta, psi, theta_dot, psi_dot], single input u = v_l + v_r.

    Extracted from linearize(): F_theta and F_psi depend only on the sum
    (v_l + v_r), never on v_l and v_r individually, so the two input columns
    of the full B are identical in these rows and either one is exactly
    d(qddot)/d(v_l + v_r).
    """
    A, B = linearize(p)
    idx = [0, 1, 3, 4]  # theta, psi, theta_dot, psi_dot
    A_planar = A[np.ix_(idx, idx)]
    B_planar = B[idx, 0]
    return A_planar, B_planar
