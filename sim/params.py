"""Physical parameters for the Echo Balancer plant.

All numeric values from CLAUDE.md section 6.1 (Simulation v0, NXTway-GS
values) live here. No other module may define a numeric physical constant.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PlantParams:
    """Physical parameters of the Echo Balancer robot."""

    g: float      # m/s^2, gravity
    m: float      # kg, wheel mass (each)
    R: float      # m, wheel radius
    Jw: float     # kg*m^2, wheel inertia, derived: m*R^2/2
    M: float      # kg, body mass
    W: float      # m, body width (wheel track)
    D: float      # m, body depth
    H: float      # m, body height
    L: float      # m, axle to body CoM, derived: H/2
    Jpsi: float   # kg*m^2, body pitch inertia, derived: M*L^2/3
    Jphi: float   # kg*m^2, body yaw inertia, derived: M*(W^2+D^2)/12
    Jm: float     # kg*m^2, motor rotor inertia
    Rm: float     # ohm, armature resistance
    Kb: float     # V*s/rad, back-EMF constant
    Kt: float     # N*m/A, torque constant
    n: float      # gear ratio
    fm: float     # body-motor friction coefficient
    fw: float     # wheel-floor friction coefficient


def default_plant_params() -> PlantParams:
    """Return NXTway-GS values for simulation v0 (CLAUDE.md section 6.1)."""
    g = 9.81
    m = 0.03
    R = 0.04
    M = 0.6
    W = 0.14
    D = 0.04
    H = 0.144
    L = H / 2
    Jw = m * R**2 / 2
    Jpsi = M * L**2 / 3
    Jphi = M * (W**2 + D**2) / 12
    Jm = 1e-5
    Rm = 6.69
    Kb = 0.468
    Kt = 0.317
    n = 1.0
    fm = 0.0022
    fw = 0.0
    return PlantParams(
        g=g, m=m, R=R, Jw=Jw, M=M, W=W, D=D, H=H, L=L,
        Jpsi=Jpsi, Jphi=Jphi, Jm=Jm, Rm=Rm, Kb=Kb, Kt=Kt,
        n=n, fm=fm, fw=fw,
    )


@dataclass(frozen=True)
class LQRBalanceParams:
    """Balance-LQR state weights, on the planar model
    [theta, psi, theta_dot, psi_dot], input u = v_l + v_r.

    CLAUDE.md section 6.1's sanity-check example is the only LQR design
    point the spec gives, so it's also the default here.
    """

    Q_theta: float      # weight on theta (wheel position)
    Q_psi: float        # weight on psi (body pitch) -- dominant term
    Q_theta_dot: float  # weight on theta_dot
    Q_psi_dot: float    # weight on psi_dot
    R: float            # weight on u = v_l + v_r


def default_lqr_balance_params() -> LQRBalanceParams:
    """CLAUDE.md section 6.1: Q=diag(1, 1e3, 1, 1), R=1e2."""
    return LQRBalanceParams(Q_theta=1.0, Q_psi=1e3, Q_theta_dot=1.0, Q_psi_dot=1.0, R=1e2)


@dataclass(frozen=True)
class SensorParams:
    """Sensor noise model (CLAUDE.md section 7). All values are starting
    points, not measured/calibrated hardware specs.

    ultrasonic_* fields are not yet consumed by any sensor function -- the
    ultrasonic model needs 2D corridor ray-casting (sim/world.py, build
    order step 5) -- but CLAUDE.md section 7 says all sigma values belong
    here regardless, so they're recorded now.
    """

    sigma_gyro: float               # rad/s, gyro white noise std per sample
    gyro_bias_walk: float           # rad/s/sqrt(s), gyro bias random-walk rate
    sigma_accel: float              # rad, accelerometer-derived tilt noise std
    encoder_cpr: float              # counts/rev, encoder resolution
    ultrasonic_sigma: float         # m, ultrasonic range noise std
    ultrasonic_range_min: float     # m
    ultrasonic_range_max: float     # m
    ultrasonic_dropout_prob: float  # probability a reading returns max range
    ultrasonic_rate_hz: float       # Hz, ultrasonic sample rate


def default_sensor_params() -> SensorParams:
    """CLAUDE.md section 7 starting-point values."""
    return SensorParams(
        sigma_gyro=0.005,
        gyro_bias_walk=1e-4,
        sigma_accel=0.02,
        encoder_cpr=360.0,
        ultrasonic_sigma=0.003,
        ultrasonic_range_min=0.02,
        ultrasonic_range_max=4.0,
        ultrasonic_dropout_prob=0.02,
        ultrasonic_rate_hz=20.0,
    )


@dataclass(frozen=True)
class EstimatorParams:
    """Balance-KF tuning for the 5-state model
    [theta, psi, theta_dot, psi_dot, b_g]: process noise on the four planar
    states, and initial covariance. The gyro-bias process noise is derived
    from SensorParams.gyro_bias_walk and the control-loop dt, not tuned
    here (see sim/estimator.py::process_noise).

    These defaults were found empirically against the section 11 NIS
    consistency test -- see docs/superpowers/plans/2026-09-20-sensors-
    estimator-nis-step3.md for the tuning process and why these values
    (not some other combination) were chosen.
    """

    Q_theta: float
    Q_psi: float
    Q_theta_dot: float
    Q_psi_dot: float
    P0_theta: float
    P0_psi: float
    P0_theta_dot: float
    P0_psi_dot: float
    P0_bg: float


def default_estimator_params() -> EstimatorParams:
    return EstimatorParams(
        Q_theta=1e-8,
        Q_psi=1e-8,
        Q_theta_dot=1e-6,
        Q_psi_dot=1e-6,
        P0_theta=1e-4,
        P0_psi=1e-4,
        P0_theta_dot=1e-4,
        P0_psi_dot=1e-4,
        P0_bg=1e-6,
    )
