"""Discrete linear Kalman filter + NIS for the balance estimator
(CLAUDE.md section 9).

States (5): [theta, psi, theta_dot, psi_dot, b_g] (b_g = gyro bias).
Measurements (3): y = [theta_enc, psi_dot_gyro, psi_acc].

The filter always runs on the *nominal* linear model (CLAUDE.md section 8):
any mismatch between this and the true (possibly nonlinear or disturbed)
plant is what the innovation is meant to detect.
"""

from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm

from sim.linearize import linearize
from sim.params import EstimatorParams, PlantParams, SensorParams


def discretize(p: PlantParams, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """ZOH-discretize the 5-state augmented model
    [theta, psi, theta_dot, psi_dot, b_g], u = v_l + v_r, at rate dt.

    b_g has no deterministic dynamics of its own (d(b_g)/dt = 0); its
    random-walk behavior is process noise, added via Q in predict(), not
    part of A. Uses the block-matrix ("Van Loan") method, which works even
    though the augmented A is singular (a b_g row of zeros, and a theta row
    with no restoring term) -- the same reason a plain
    Bd = A^-1(Ad - I)B formula would fail here.
    """
    A6, B6 = linearize(p)
    idx = [0, 1, 3, 4]  # theta, psi, theta_dot, psi_dot
    A_planar = A6[np.ix_(idx, idx)]
    B_planar = B6[idx, 0]

    A_aug = np.zeros((5, 5))
    A_aug[:4, :4] = A_planar
    B_aug = np.zeros(5)
    B_aug[:4] = B_planar

    M = np.zeros((6, 6))
    M[:5, :5] = A_aug
    M[:5, 5] = B_aug
    Md = expm(M * dt)
    return Md[:5, :5], Md[:5, 5]


def measurement_matrix() -> np.ndarray:
    """C for y = [theta_enc, psi_dot_gyro, psi_acc] from
    x = [theta, psi, theta_dot, psi_dot, b_g].

    theta_enc ~= theta - psi; psi_dot_gyro ~= psi_dot + b_g; psi_acc ~= psi
    (the nominal model has no theta_ddot term -- that corruption during
    motion is exactly the mismatch NIS is meant to catch).
    """
    return np.array([
        [1.0, -1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 1.0],
        [0.0, 1.0, 0.0, 0.0, 0.0],
    ])


def process_noise(sensor_p: SensorParams, est_p: EstimatorParams, dt: float) -> np.ndarray:
    """Q (5x5 diagonal). The gyro-bias term is derived directly from the
    sensor's bias-walk rate; the other four come from EstimatorParams
    (empirically tuned -- see the plan doc for this build step)."""
    Q_bg = sensor_p.gyro_bias_walk**2 * dt
    return np.diag([est_p.Q_theta, est_p.Q_psi, est_p.Q_theta_dot, est_p.Q_psi_dot, Q_bg])


def measurement_noise(sensor_p: SensorParams) -> np.ndarray:
    """R (3x3 diagonal): encoder quantization modeled as uniform noise
    (variance = step^2/12), plus the gyro and accelerometer noise stds."""
    step = 2 * np.pi / sensor_p.encoder_cpr
    encoder_var = step**2 / 12.0
    return np.diag([encoder_var, sensor_p.sigma_gyro**2, sensor_p.sigma_accel**2])


def initial_covariance(est_p: EstimatorParams) -> np.ndarray:
    return np.diag([
        est_p.P0_theta, est_p.P0_psi, est_p.P0_theta_dot, est_p.P0_psi_dot, est_p.P0_bg,
    ])


@dataclass(frozen=True)
class KalmanState:
    x_hat: np.ndarray  # shape (5,)
    P: np.ndarray      # shape (5, 5)


def predict(state: KalmanState, Ad: np.ndarray, Bd: np.ndarray, u: float, Q: np.ndarray) -> KalmanState:
    x_pred = Ad @ state.x_hat + Bd * u
    P_pred = Ad @ state.P @ Ad.T + Q
    return KalmanState(x_pred, 0.5 * (P_pred + P_pred.T))


def update(
    state: KalmanState, y: np.ndarray, C: np.ndarray, R: np.ndarray
) -> tuple[KalmanState, float]:
    """Measurement update. Returns (updated state, NIS)."""
    innovation = y - C @ state.x_hat
    S = C @ state.P @ C.T + R
    K = state.P @ C.T @ np.linalg.inv(S)
    x_upd = state.x_hat + K @ innovation
    I_KC = np.eye(state.x_hat.shape[0]) - K @ C
    P_upd = I_KC @ state.P @ I_KC.T + K @ R @ K.T  # Joseph form (numerically stable)
    nis = innovation @ np.linalg.solve(S, innovation)
    return KalmanState(x_upd, P_upd), nis
