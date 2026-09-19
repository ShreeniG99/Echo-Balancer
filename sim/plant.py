"""Nonlinear plant dynamics (CLAUDE.md section 5).

State x = [theta, psi, phi, theta_dot, psi_dot, phi_dot]:
  theta = mean wheel angle [rad]
  psi   = body pitch, 0 = upright, + = leaning forward [rad]
  phi   = yaw [rad]
Inputs v_l, v_r: motor voltages [V].
"""

import numpy as np

from sim.params import PlantParams


def f(x: np.ndarray, v_l: float, v_r: float, p: PlantParams) -> np.ndarray:
    theta, psi, phi, theta_dot, psi_dot, phi_dot = x

    alpha = p.n * p.Kt / p.Rm
    beta = p.n**2 * p.Kt * p.Kb / p.Rm + p.fm

    F_theta = (
        alpha * (v_l + v_r)
        - 2 * (beta + p.fw) * theta_dot
        + 2 * beta * psi_dot
    )
    F_psi = (
        -alpha * (v_l + v_r)
        + 2 * beta * theta_dot
        - 2 * beta * psi_dot
    )
    F_phi = (
        (p.W / (2 * p.R)) * alpha * (v_r - v_l)
        - (p.W**2 / (2 * p.R**2)) * (beta + p.fw) * phi_dot
    )

    # Coupled 2x2 system for theta_ddot, psi_ddot.
    A11 = (2 * p.m + p.M) * p.R**2 + 2 * p.Jw + 2 * p.n**2 * p.Jm
    A12 = p.M * p.L * p.R * np.cos(psi) - 2 * p.n**2 * p.Jm
    A22 = p.M * p.L**2 + p.Jpsi + 2 * p.n**2 * p.Jm

    A = np.array([[A11, A12], [A12, A22]])
    b = np.array([
        F_theta + p.M * p.L * p.R * psi_dot**2 * np.sin(psi),
        F_psi + p.M * p.g * p.L * np.sin(psi)
        + p.M * p.L**2 * phi_dot**2 * np.sin(psi) * np.cos(psi),
    ])
    theta_ddot, psi_ddot = np.linalg.solve(A, b)

    # Decoupled scalar equation for phi_ddot.
    A33 = (
        0.5 * p.m * p.W**2
        + p.Jphi
        + (p.W**2 / (2 * p.R**2)) * (p.Jw + p.n**2 * p.Jm)
        + p.M * p.L**2 * np.sin(psi) ** 2
    )
    phi_ddot = (
        F_phi - 2 * p.M * p.L**2 * psi_dot * phi_dot * np.sin(psi) * np.cos(psi)
    ) / A33

    return np.array([theta_dot, psi_dot, phi_dot, theta_ddot, psi_ddot, phi_ddot])
