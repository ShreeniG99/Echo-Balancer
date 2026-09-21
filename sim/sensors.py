"""Sensor models (CLAUDE.md section 7).

Gyro, accelerometer, and encoder feed the balance Kalman filter
(sim/estimator.py); ultrasonic does not (it isn't part of the KF's
measurement vector per CLAUDE.md section 9) -- it drives wall-following
instead (sim/control.py), using ray-cast distances from sim/world.py as
its true_distance input.
"""

import numpy as np

from sim.params import PlantParams, SensorParams


def gyro(
    psi_dot_true: float,
    b_g: float,
    dt: float,
    sensor_p: SensorParams,
    rng: np.random.Generator,
) -> tuple[float, float]:
    """Gyro measurement of psi_dot, plus the bias's random-walk update.

    b_g is the bias in effect for *this* measurement; the returned
    (incremented) bias is what the next call should pass in -- the same
    explicit-state-threading pattern sim.integrate.rk4_step uses.
    """
    psi_dot_meas = psi_dot_true + b_g + rng.normal(0.0, sensor_p.sigma_gyro)
    b_g_next = b_g + rng.normal(0.0, sensor_p.gyro_bias_walk * np.sqrt(dt))
    return psi_dot_meas, b_g_next


def accelerometer(
    psi: float,
    theta_ddot: float,
    p: PlantParams,
    sensor_p: SensorParams,
    rng: np.random.Generator,
) -> float:
    """Accelerometer-derived tilt angle psi_acc = atan2(a_x, a_z).

    Models the IMU at the wheel axle (CLAUDE.md section 6.2: "mounted at or
    near the pivot axis"): the axle's height above the ground is fixed at
    R regardless of body lean (zero vertical acceleration), and its
    horizontal position is exactly R*theta (rolling without slip), so its
    true specific force in the world frame is exactly (R*theta_ddot, g).
    Rotating into the body frame:
    """
    a_x = p.R * theta_ddot * np.cos(psi) + p.g * np.sin(psi)
    a_z = -p.R * theta_ddot * np.sin(psi) + p.g * np.cos(psi)
    return np.arctan2(a_x, a_z) + rng.normal(0.0, sensor_p.sigma_accel)


def encoder(
    theta: float,
    psi: float,
    phi: float,
    p: PlantParams,
    sensor_p: SensorParams,
) -> float:
    """Averaged, quantized wheel-angle-relative-to-body encoder reading.

    Each wheel's raw angle relative to the body (theta_l - psi, theta_r -
    psi) is quantized to the encoder's tick resolution independently (real
    encoders quantize in hardware before any averaging can happen), then
    the two quantized readings are averaged. No rng: quantization is
    deterministic given the true state.
    """
    half_track_over_R = p.W / (2 * p.R)
    theta_l = theta - half_track_over_R * phi
    theta_r = theta + half_track_over_R * phi

    step = 2 * np.pi / sensor_p.encoder_cpr
    q_l = np.round((theta_l - psi) / step) * step
    q_r = np.round((theta_r - psi) / step) * step
    return (q_l + q_r) / 2.0


def ultrasonic(true_distance: float, sensor_p: SensorParams, rng: np.random.Generator) -> float:
    """HC-SR04 model (CLAUDE.md section 7): 20Hz, range 0.02-4m, sigma~3mm,
    2% dropout probability (returns max range instead of a noisy reading).
    """
    if rng.random() < sensor_p.ultrasonic_dropout_prob:
        return sensor_p.ultrasonic_range_max
    measured = true_distance + rng.normal(0.0, sensor_p.ultrasonic_sigma)
    return float(np.clip(measured, sensor_p.ultrasonic_range_min, sensor_p.ultrasonic_range_max))
