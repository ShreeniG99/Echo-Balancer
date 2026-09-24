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

    ultrasonic_* fields are consumed by sim.sensors.ultrasonic, which
    combines them with 2D corridor ray-casting (sim/world.py) to model the
    HC-SR04.
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


@dataclass(frozen=True)
class GateParams:
    """Normal/Cautious/Halt gate tuning (CLAUDE.md section 10) for the
    windowed statistic epsilon_k = sum of the last N NIS samples.

    CLAUDE.md says tau1 < tau2 should be "chi2(3N) quantiles." Empirically
    (see docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md),
    this is not achievable for this closed loop: even chi2(3*200=600)'s
    0.999999 quantile (~779) is below the observed nominal-run maximum of
    epsilon_k (~959, across 10 independent 60s seeded runs) -- successive
    NIS samples here are correlated (KF smoothing plus a slow ~10s
    theta-position closed-loop pole), so the window sum's tail is far
    heavier than the i.i.d. chi2(3N) model predicts, at any quantile
    level. tau1/tau2 are therefore calibrated directly against real
    simulated nominal runs with margin, not derived from a chi2 quantile
    formula.

    This calibration is specifically against the plain 4-state balance-
    only design_lqr_balance closed loop with the robot near-stationary
    (tests/test_gate.py's own closed loop) -- see
    default_evaluation_gate_params for the separate recalibration needed
    once the robot is actually driving (sim/run.py, step 6): a different
    closed loop (design_lqr_speed_servo, continuous forward motion) has
    genuinely different nominal NIS statistics, not just a different
    "reasonable" choice of tau, so the two must not be merged into one
    default without breaking whichever closed loop's tests were
    calibrated against the other.
    """

    N: int              # window size (# of 5ms samples); 200 = 1s window
    tau1: float          # CAUTIOUS entry (and NORMAL->HALT skip threshold)
    tau2: float          # HALT entry
    tau1_exit: float     # CAUTIOUS -> NORMAL hysteresis exit (< tau1)
    tau2_exit: float     # HALT -> CAUTIOUS hysteresis exit (< tau2)
    T_dwell: float       # seconds, minimum time in a mode before any transition


def default_gate_params() -> GateParams:
    return GateParams(N=200, tau1=1300.0, tau2=1800.0, tau1_exit=1040.0, tau2_exit=1300.0, T_dwell=0.5)


def default_evaluation_gate_params() -> GateParams:
    """Recalibrated in sim/run.py's development (step 6) for the closed
    loop the section 12 evaluation actually drives -- the 5-state
    integral-augmented speed-servo LQR (sim.control.design_lqr_speed_servo)
    plus wall-following, continuously commanding real forward motion, not
    the plain 4-state balance-only design_lqr_balance GateParams' own
    default is calibrated against with the robot near-stationary. This is
    the speed-servo-driven-closed-loop recalibration GateParams' docstring
    already anticipated ("Root cause: a slow ~10s theta-position
    closed-loop pole from the still-missing section 9 speed-servo integral
    term"): with the robot actually driving, physical disturbances that
    act through velocity-dependent terms (surface-change friction acting
    on theta_dot; the plant's own phi_dot^2*sin(psi)*cos(psi) yaw->pitch
    coupling) are far more consequential than at near-zero speed, which
    widens rather than narrows the nominal/disturbed separation:
    nominal-run epsilon max across 10 seeds is ~604 under this closed loop
    (vs. GateParams' own ~959 under its different closed loop), and the
    weakest disturbance response (push) peaks at ~1094 -- both checked in
    sim/run.py's development. tau1=900/tau2=1400 sit with margin in that
    gap and above it respectively; tau1_exit/tau2_exit keep GateParams'
    default hysteresis ratios (0.8 and ~0.72) rather than being
    independently re-derived.
    """
    return GateParams(N=200, tau1=900.0, tau2=1400.0, tau1_exit=720.0, tau2_exit=1000.0, T_dwell=0.5)


@dataclass(frozen=True)
class DisturbanceParams:
    """Onset/duration/magnitude for each CLAUDE.md section 8 disturbance.
    See docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md for
    the full empirical derivation. Load-bearing caveats that matter if
    you're tuning these, not just reading them:

    - surface_change_fw=0.025 sits close to a genuine plant instability:
      f_w >= ~0.03 causes the closed-loop nonlinear simulation to diverge
      (NaN/inf), not just "fall over." Don't raise this without margin.
    - payload_shift and battery_droop do NOT trigger gate detection in
      isolation near equilibrium -- payload_shift needs a concurrent mild
      tilt (payload_shift_test_psi0_deg) to expose the mass/CoM mismatch
      at all, and even much larger, more "realistic" shifts (delta_M up
      to 0.5kg, delta_L up to 0.05m) never trigger detection even with
      tilt; battery_droop needs a concurrent voltage demand
      (battery_droop_companion_push_magnitude) since nominal near-
      equilibrium commands never approach even a badly drooped ceiling.
      These are genuine sensitivity limits of the current NIS/Q/R tuning,
      not bugs.
    - battery_droop_companion_push_magnitude and payload_shift_test_psi0_deg
      are test-harness setup values (what it takes to make the disturbance
      observable at all), NOT part of the disturbance's own physical
      profile per CLAUDE.md section 8. Don't feed them into section 12's
      cross-controller evaluation as if they were part of the canonical
      battery-droop/payload-shift scenario -- that would silently give
      those two disturbances an extra "helper" perturbation the other
      three don't get, undermining "controllers compared on identical
      disturbance seeds."
    """

    surface_change_onset: float
    surface_change_duration: float
    surface_change_fw: float

    battery_droop_onset: float
    battery_droop_duration: float
    battery_droop_v_nominal: float
    battery_droop_v_drooped: float
    battery_droop_companion_push_magnitude: float  # test-harness setup, not part of the disturbance profile itself -- see class docstring

    push_onset: float
    push_magnitude: float

    gyro_bias_fault_onset: float
    gyro_bias_fault_magnitude: float

    accel_noise_fault_onset: float
    accel_noise_fault_duration: float
    accel_noise_fault_multiplier: float

    payload_shift_onset: float
    payload_shift_delta_M: float
    payload_shift_delta_L: float
    payload_shift_test_psi0_deg: float  # test-harness setup, not part of the disturbance profile itself -- see class docstring

    detection_window_s: float


def default_disturbance_params() -> DisturbanceParams:
    return DisturbanceParams(
        surface_change_onset=5.0,
        surface_change_duration=10.0,
        surface_change_fw=0.025,
        battery_droop_onset=5.0,
        battery_droop_duration=1.0,
        battery_droop_v_nominal=7.4,
        battery_droop_v_drooped=0.1,
        battery_droop_companion_push_magnitude=0.05,
        push_onset=5.0,
        push_magnitude=0.1,
        gyro_bias_fault_onset=5.0,
        gyro_bias_fault_magnitude=0.05,
        accel_noise_fault_onset=5.0,
        accel_noise_fault_duration=10.0,
        accel_noise_fault_multiplier=5.0,
        payload_shift_onset=5.0,
        payload_shift_delta_M=1.0,
        payload_shift_delta_L=0.1,
        payload_shift_test_psi0_deg=2.0,
        detection_window_s=2.0,
    )


@dataclass(frozen=True)
class CorridorParams:
    """2D corridor geometry (CLAUDE.md section 4/9). Two infinite parallel
    walls at y=0 and y=width -- no corners or dead ends, see
    docs/superpowers/plans/2026-09-21-world-wallfollowing-step5.md
    "Design decision: the corridor is two infinite parallel walls"."""

    width: float  # m, distance between the two walls
    ray_parallel_eps: float  # threshold below which sin(ray_heading) is treated as parallel to the walls in sim.world.cast_ray


def default_corridor_params() -> CorridorParams:
    return CorridorParams(width=1.0, ray_parallel_eps=1e-9)


@dataclass(frozen=True)
class SpeedServoParams:
    """5-state integral-augmented LQR weights for forward-speed tracking
    (CLAUDE.md section 9: "plus integral of (theta - theta_ref) for speed
    servo"). Completely separate from LQRBalanceParams (step 2) -- this
    does not replace the balance-only LQR, it's an additional, optional
    augmentation. See sim.control.design_lqr_speed_servo.
    """

    Q_theta: float      # weight on theta - theta_ref (position tracking error)
    Q_psi: float        # weight on psi (body pitch) -- dominant term
    Q_theta_dot: float  # weight on theta_dot - theta_dot_ref
    Q_psi_dot: float    # weight on psi_dot
    Q_integral: float  # weight on z = integral(theta - theta_ref)
    R: float            # weight on u = v_l + v_r


def default_speed_servo_params() -> SpeedServoParams:
    return SpeedServoParams(Q_theta=1.0, Q_psi=1e3, Q_theta_dot=1.0, Q_psi_dot=1.0, Q_integral=10.0, R=1e2)


@dataclass(frozen=True)
class YawControlParams:
    """Yaw-rate tracking gain (CLAUDE.md section 9: "PD on phi_dot").
    Proportional-only, not PD -- see docs/superpowers/plans/2026-09-21-
    world-wallfollowing-step5.md "Design decision: yaw control is
    proportional-only". Any nonzero derivative gain destabilizes this
    loop given the plant's fast yaw dynamics relative to the 200Hz control
    rate -- do not add a Kd field without re-reading that section first.
    """

    Kp: float


def default_yaw_control_params() -> YawControlParams:
    return YawControlParams(Kp=3.0)


@dataclass(frozen=True)
class CautiousParams:
    """CAUTIOUS-mode speed derating (CLAUDE.md section 10: "speed ref x
    0.4"). Applied on top of whatever theta_dot_ref front_threshold_speed_adjust
    already produced.
    """

    speed_ref_factor: float


def default_cautious_params() -> CautiousParams:
    return CautiousParams(speed_ref_factor=0.4)


def default_cautious_speed_servo_params() -> "SpeedServoParams":
    """CAUTIOUS-mode LQR gain set (CLAUDE.md section 10: "softer Q in the
    LQR gain set"). Same 5-state structure as default_speed_servo_params;
    the speed-tracking-error weights (Q_theta, Q_theta_dot, Q_integral) are
    reduced to 1/4 of NORMAL's nominal 1/1/10 while Q_psi (balance
    stability) is left unchanged -- trades speed-tracking aggressiveness
    for gentler control effort in CAUTIOUS, not balance robustness.
    Starting point, not tuned against data yet.
    """
    return SpeedServoParams(Q_theta=0.25, Q_psi=1e3, Q_theta_dot=0.25, Q_psi_dot=1.0, Q_integral=2.5, R=1e2)


@dataclass(frozen=True)
class TiltGateParams:
    """Tilt-threshold gate baseline (CLAUDE.md section 12, controller 2):
    mode switches on |psi| thresholds only -- no Kalman filter/NIS
    involved. Same hysteresis + minimum-dwell structure as GateParams for
    a fair comparison, but the thresholds are in degrees of tilt, not
    epsilon (windowed NIS sum), so they are calibrated independently. Set
    with margin below the section 5 fall condition (|psi| > 45 deg).
    Starting point, not tuned against data yet.
    """

    tau1_deg: float       # CAUTIOUS entry |psi| threshold
    tau2_deg: float       # HALT entry |psi| threshold
    tau1_exit_deg: float  # CAUTIOUS -> NORMAL hysteresis exit (< tau1_deg)
    tau2_exit_deg: float  # HALT -> CAUTIOUS hysteresis exit (< tau2_deg)
    T_dwell: float        # seconds, minimum time in a mode before any transition


def default_tilt_gate_params() -> TiltGateParams:
    return TiltGateParams(tau1_deg=5.0, tau2_deg=15.0, tau1_exit_deg=3.0, tau2_exit_deg=10.0, T_dwell=0.5)


@dataclass(frozen=True)
class SpeedProfileParams:
    """Forward-speed reference ramp: theta_dot_ref moves toward its target
    (from front_threshold_speed_adjust / the gate's mode) at a bounded
    acceleration rather than stepping instantly. An instantaneous 0 -> 2
    rad/s step at t=0 is not a realistic motor command, and was found
    (sim/run.py's development) to itself produce a transient large enough
    to spuriously trip the NIS gate on an otherwise-nominal, undisturbed
    run -- confirmed by the gate behaving exactly per the section-11
    calibration once the step was replaced with this ramp. Starting
    point, not tuned against data beyond removing that artifact.
    """

    accel_limit: float  # rad/s^2, max |d(theta_dot_ref)/dt|


def default_speed_profile_params() -> SpeedProfileParams:
    return SpeedProfileParams(accel_limit=2.0)


@dataclass(frozen=True)
class WallFollowParams:
    """Wall-following outer loop and front-threshold slowdown (CLAUDE.md
    section 9). Kp/Kd act on (target_distance - side_ultrasonic_reading)
    to produce a phi_dot reference for YawControlParams' inner loop.
    """

    target_distance: float     # m, desired distance from the followed wall
    Kp: float                  # distance-error -> phi_dot_ref proportional gain
    Kd: float                  # distance-error -> phi_dot_ref derivative gain
    front_slow_threshold: float  # m, front ultrasonic reading below this triggers slowdown
    front_slow_factor: float     # multiplier applied to the forward speed reference when triggered


def default_wall_follow_params() -> WallFollowParams:
    return WallFollowParams(target_distance=0.5, Kp=2.0, Kd=0.5, front_slow_threshold=0.5, front_slow_factor=0.3)
