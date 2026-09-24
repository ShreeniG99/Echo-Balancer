"""One closed-loop episode -> DataFrame (CLAUDE.md section 4, section 12).

Unifies the closed-loop plumbing that had accumulated independently across
tests/test_estimator.py, tests/test_gate.py, and tests/test_wall_following.py
(plant + RK4 at dt_plant, sensors + Kalman filter + gate at dt_control,
ultrasonic + wall-following at dt_ultrasonic, corridor pose integration at
dt_plant) into one reusable episode runner, parameterized by which of the
three CLAUDE.md section 12 controllers is driving the mode:

- NAIVE: always NORMAL (full speed, nominal LQR gain), no gate at all.
- TILT_GATE: sim.gate.step_tilt_gate on the KF-estimated psi (a real robot
  only has its own state estimate available, same as the NIS gate).
- NIS_GATE: sim.gate.step_gate on the KF's NIS, as built in step 4.

The Kalman filter always runs regardless of controller (it's what the
balance/speed-servo loop feeds on -- CLAUDE.md section 9 doesn't gate
estimation on which safety controller is selected), so NIS is logged every
tick for all three controllers, enabling apples-to-apples post-hoc
detection-delay comparisons in analysis/metrics.py even for NAIVE/TILT_GATE
episodes where NIS isn't driving anything.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

import numpy as np
import pandas as pd

from sim.control import (
    design_lqr_balance,
    design_lqr_speed_servo,
    front_threshold_speed_adjust,
    wall_following_control,
    yaw_p_control,
)
from sim.disturbances import clip_voltage
from sim.estimator import (
    KalmanState,
    discretize,
    initial_covariance,
    measurement_matrix,
    measurement_noise,
    predict,
    process_noise,
    update,
)
from sim.gate import (
    GateMode,
    initial_gate_state,
    initial_tilt_gate_state,
    step_gate,
    step_tilt_gate,
)
from sim.integrate import rk4_step
from sim.params import (
    CautiousParams,
    CorridorParams,
    DisturbanceParams,
    EstimatorParams,
    GateParams,
    LQRBalanceParams,
    PlantParams,
    SensorParams,
    SpeedProfileParams,
    SpeedServoParams,
    TiltGateParams,
    WallFollowParams,
    YawControlParams,
    default_cautious_params,
    default_cautious_speed_servo_params,
    default_corridor_params,
    default_disturbance_params,
    default_estimator_params,
    default_evaluation_gate_params,
    default_lqr_balance_params,
    default_plant_params,
    default_sensor_params,
    default_speed_profile_params,
    default_speed_servo_params,
    default_tilt_gate_params,
    default_wall_follow_params,
    default_yaw_control_params,
)
from sim.plant import f as plant_f
from sim.sensors import accelerometer, encoder, gyro, ultrasonic
from sim.world import cast_ray, pose_velocity

FALL_PSI_DEG = 45.0  # CLAUDE.md section 5: |psi| > 45 deg ends the episode

# Same shape as the apply_disturbance callback in tests/test_gate.py:
# (t, x_true, b_g_true, p, sensor_p) -> (x_true, b_g_true, p_now, sensor_p_now, v_batt_now)
DisturbanceFn = Callable[
    [float, np.ndarray, float, PlantParams, SensorParams],
    tuple[np.ndarray, float, PlantParams, SensorParams, float],
]


class ControllerKind(Enum):
    NAIVE = "naive"
    TILT_GATE = "tilt_gate"
    NIS_GATE = "nis_gate"


@dataclass(frozen=True)
class EpisodeConfig:
    """Full specification of one episode. Combined with a seed, this must
    reproduce an identical DataFrame (CLAUDE.md section 11)."""

    controller: ControllerKind
    T: float = 60.0
    theta_dot_ref_nominal: float = 2.0
    x0_psi_deg: float = 0.0
    wall_following: bool = True
    disturbance: Optional[DisturbanceFn] = None
    plant_p: PlantParams = None
    sensor_p: SensorParams = None
    lqr_p: LQRBalanceParams = None
    speed_servo_p: SpeedServoParams = None
    cautious_speed_servo_p: SpeedServoParams = None
    yaw_p: YawControlParams = None
    wall_p: WallFollowParams = None
    corridor_p: CorridorParams = None
    est_p: EstimatorParams = None
    gate_p: GateParams = None
    tilt_gate_p: TiltGateParams = None
    cautious_p: CautiousParams = None
    speed_profile_p: SpeedProfileParams = None
    disturbance_p: DisturbanceParams = None
    y0: Optional[float] = None  # initial lateral position; default set in run_episode

    def resolved(self) -> "EpisodeConfig":
        """Fill in any None fields with CLAUDE.md-default params, so
        run_episode never has to special-case a missing param set."""
        defaults = dict(
            plant_p=default_plant_params(),
            sensor_p=default_sensor_params(),
            lqr_p=default_lqr_balance_params(),
            speed_servo_p=default_speed_servo_params(),
            cautious_speed_servo_p=default_cautious_speed_servo_params(),
            yaw_p=default_yaw_control_params(),
            wall_p=default_wall_follow_params(),
            corridor_p=default_corridor_params(),
            est_p=default_estimator_params(),
            gate_p=default_evaluation_gate_params(),
            tilt_gate_p=default_tilt_gate_params(),
            cautious_p=default_cautious_params(),
            speed_profile_p=default_speed_profile_params(),
            disturbance_p=default_disturbance_params(),
        )
        updates = {k: v for k, v in defaults.items() if getattr(self, k) is None}
        return _replace_dataclass(self, **updates) if updates else self


def _replace_dataclass(cfg: EpisodeConfig, **updates) -> EpisodeConfig:
    from dataclasses import replace

    return replace(cfg, **updates)


def run_episode(config: EpisodeConfig, seed: int) -> pd.DataFrame:
    """Run one closed-loop episode. Deterministic in (config, seed).

    Returns a DataFrame with one row per control tick (dt_control=5ms)
    actually simulated -- the episode stops early (fewer rows than
    T/dt_control) if the robot falls (CLAUDE.md section 5: |psi| > 45 deg,
    motors off).
    """
    cfg = config.resolved()
    p = cfg.plant_p
    sensor_p = cfg.sensor_p
    dp = cfg.disturbance_p
    nominal_v_batt = dp.battery_droop_v_nominal

    K_balance = design_lqr_balance(p, cfg.lqr_p)
    K_normal = design_lqr_speed_servo(p, cfg.speed_servo_p)
    K_cautious = design_lqr_speed_servo(p, cfg.cautious_speed_servo_p)

    dt_plant = 0.001
    dt_control = 0.005
    n_sub = int(round(dt_control / dt_plant))
    dt_ultrasonic = 1.0 / sensor_p.ultrasonic_rate_hz
    n_control_per_ultrasonic = int(round(dt_ultrasonic / dt_control))

    Ad, Bd = discretize(p, dt_control)
    C = measurement_matrix()
    Q = process_noise(sensor_p, cfg.est_p, dt_control)
    R = measurement_noise(sensor_p)

    rng = np.random.default_rng(seed)
    n_steps = int(round(cfg.T / dt_control))

    y0 = cfg.y0 if cfg.y0 is not None else max(cfg.wall_p.target_distance - 0.2, 0.05)

    x_true = np.array([0.0, np.radians(cfg.x0_psi_deg), 0.0, 0.0, 0.0, 0.0])
    b_g_true = 0.0
    pos = np.array([0.0, y0])
    kf = KalmanState(x_hat=np.zeros(5), P=initial_covariance(cfg.est_p))
    gate_state = initial_gate_state()
    tilt_state = initial_tilt_gate_state()

    theta_ref = 0.0
    z = 0.0
    phi_dot_ref = 0.0
    theta_dot_ref_wall = cfg.theta_dot_ref_nominal
    theta_dot_ref_current = 0.0  # ramped toward the target at speed_profile_p.accel_limit -- see SpeedProfileParams
    u_prev = 0.0
    front_reading = cfg.wall_p.front_slow_threshold * 2.0  # clear until the first ultrasonic sample
    # cast_ray at heading-pi/2 (heading=0 initially) measures distance to the
    # y=0 wall, i.e. pos.y itself -- NOT corridor width minus y.
    right_reading = y0
    # Seeded from the true initial distance error, not 0.0: a 0.0 seed would make the
    # very first wall_following_control derivative term spike toward the entire
    # initial offset in one dt_ultrasonic step (classic PID cold-start derivative
    # kick) -- confirmed via sim/run.py development to itself trip the NIS gate on
    # an otherwise-nominal run through the phi_dot^2*sin(psi)*cos(psi) yaw->pitch
    # coupling term in sim.plant.f, not a real disturbance.
    prev_wall_err = cfg.wall_p.target_distance - right_reading

    rows = []
    fell = False
    fall_time = None

    for k in range(n_steps):
        t = k * dt_control

        if cfg.disturbance is not None:
            x_true, b_g_true, p_now, sensor_p_now, v_batt_now = cfg.disturbance(t, x_true, b_g_true, p, sensor_p)
        else:
            p_now, sensor_p_now, v_batt_now = p, sensor_p, nominal_v_batt

        if abs(x_true[1]) > np.radians(FALL_PSI_DEG):
            fell = True
            fall_time = t
            break

        def xdotf(x, uu, pp=p_now):
            return plant_f(x, uu[0], uu[1], pp)

        theta_ddot_true = xdotf(x_true, (u_prev / 2, u_prev / 2))[3]

        theta_enc = encoder(x_true[0], x_true[1], x_true[2], p_now, sensor_p_now)
        psi_dot_meas, b_g_true = gyro(x_true[4], b_g_true, dt_control, sensor_p_now, rng)
        psi_acc_meas = accelerometer(x_true[1], theta_ddot_true, p_now, sensor_p_now, rng)
        y_meas = np.array([theta_enc, psi_dot_meas, psi_acc_meas])

        kf = predict(kf, Ad, Bd, u_prev, Q)
        kf, nis = update(kf, y_meas, C, R)

        if cfg.wall_following and k % n_control_per_ultrasonic == 0:
            heading = x_true[2]
            true_right = cast_ray(pos[0], pos[1], heading - np.pi / 2, cfg.corridor_p, 0.02, 4.0)
            true_front = cast_ray(pos[0], pos[1], heading, cfg.corridor_p, 0.02, 4.0)
            right_reading = ultrasonic(true_right, sensor_p_now, rng)
            front_reading = ultrasonic(true_front, sensor_p_now, rng)

            # A right-sensor dropout (CLAUDE.md section 7: 2% probability, HC-SR04
            # model returns exactly ultrasonic_range_max) is an implausible single-
            # sample jump for a sensor bounded by the corridor's own width -- fed
            # straight into wall_following_control's derivative term, it produces a
            # multi-rad/s yaw-rate command out of a single dropped reading, which
            # then couples into pitch via sim.plant.f's phi_dot^2*sin(psi)*cos(psi)
            # term and trips the NIS gate on an otherwise-nominal run. A real
            # controller wouldn't act on a reading it can identify as a dropout, so
            # this tick's control update is skipped (holding the previous
            # phi_dot_ref/prev_wall_err) rather than treating the dropout as data.
            if right_reading < sensor_p_now.ultrasonic_range_max:
                wall_err = cfg.wall_p.target_distance - right_reading
                phi_dot_ref, prev_wall_err = wall_following_control(wall_err, prev_wall_err, dt_ultrasonic, cfg.wall_p)
            theta_dot_ref_wall = front_threshold_speed_adjust(front_reading, cfg.theta_dot_ref_nominal, cfg.wall_p)

        if cfg.controller is ControllerKind.NIS_GATE:
            gate_state, epsilon = step_gate(gate_state, nis, dt_control, cfg.gate_p)
            mode = gate_state.mode
        elif cfg.controller is ControllerKind.TILT_GATE:
            tilt_state = step_tilt_gate(tilt_state, kf.x_hat[1], dt_control, cfg.tilt_gate_p)
            mode = tilt_state.mode
            epsilon = float("nan")
        else:
            mode = GateMode.NORMAL
            epsilon = float("nan")

        if mode is GateMode.HALT:
            theta_dot_ref_target = 0.0
            K5 = K_normal
        elif mode is GateMode.CAUTIOUS:
            theta_dot_ref_target = theta_dot_ref_wall * cfg.cautious_p.speed_ref_factor
            K5 = K_cautious
        else:
            theta_dot_ref_target = theta_dot_ref_wall if cfg.wall_following else cfg.theta_dot_ref_nominal
            K5 = K_normal

        accel_step = cfg.speed_profile_p.accel_limit * dt_control
        theta_dot_ref_current += np.clip(theta_dot_ref_target - theta_dot_ref_current, -accel_step, accel_step)
        theta_dot_ref = theta_dot_ref_current

        err5 = np.array([kf.x_hat[0] - theta_ref, kf.x_hat[1], kf.x_hat[2] - theta_dot_ref, kf.x_hat[3], z])
        theta_ref += theta_dot_ref * dt_control
        z += (kf.x_hat[0] - theta_ref) * dt_control
        u_common = -K5 @ err5

        diff_v = yaw_p_control(phi_dot_ref, x_true[5], cfg.yaw_p) if cfg.wall_following else 0.0
        v_l = clip_voltage(u_common / 2 - diff_v / 2, v_batt_now)
        v_r = clip_voltage(u_common / 2 + diff_v / 2, v_batt_now)
        u_prev = v_l + v_r

        rows.append(dict(
            t=t, theta=x_true[0], psi=x_true[1], phi=x_true[2],
            theta_dot=x_true[3], psi_dot=x_true[4], phi_dot=x_true[5],
            x=pos[0], y=pos[1],
            theta_hat=kf.x_hat[0], psi_hat=kf.x_hat[1],
            theta_dot_hat=kf.x_hat[2], psi_dot_hat=kf.x_hat[3], b_g_hat=kf.x_hat[4],
            nis=nis, epsilon=epsilon, mode=mode.value,
            theta_dot_ref=theta_dot_ref, phi_dot_ref=phi_dot_ref,
            front_reading=front_reading, right_reading=right_reading,
            v_l=v_l, v_r=v_r, fell=False,
        ))

        for _ in range(n_sub):
            x_true = rk4_step(xdotf, x_true, (v_l, v_r), dt_plant)
            if cfg.wall_following:
                pos = rk4_step(lambda pp, uu: pose_velocity(pp, uu, p), pos, (x_true[3], x_true[2]), dt_plant)

    df = pd.DataFrame(rows)
    if fell:
        df.loc[df.index[-1], "fell"] = True
    df.attrs["controller"] = cfg.controller.value
    df.attrs["seed"] = seed
    df.attrs["fell"] = fell
    df.attrs["fall_time"] = fall_time
    df.attrs["T"] = cfg.T
    df.attrs["progress"] = float(df["x"].iloc[-1]) if len(df) else 0.0
    return df
