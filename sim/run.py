"""General-purpose closed-loop episode runner (CLAUDE.md section 4).

run_episode(config, seed) drives the full plant/sensor/estimator/gate
stack for one episode and returns a per-control-tick pandas DataFrame,
unifying what were three separate, overlapping test-only closed-loop
harnesses (tests/test_estimator.py::_run_nominal_closed_loop,
tests/test_gate.py::_closed_loop_with_gate,
tests/test_wall_following.py::test_follows_wall_without_crashing) into one
general-purpose implementation. See docs/superpowers/plans/2026-09-22-run-
batch-metrics-step6.md for the pre-verified design decisions this makes:

- The balance/gate control law is ALWAYS sim.control.design_lqr_balance,
  driven by the Kalman filter estimate -- exactly reproducing the
  already-calibrated step 3/4 architecture (GateParams tau1/tau2 stay
  valid) -- UNLESS wall_following=True, which requires
  sim.control.design_lqr_speed_servo instead (design_lqr_balance cannot
  track a nonzero speed reference without steady-state error).
- wall_following=True control (speed-servo + yaw) is fed by the TRUE
  plant state, not the KF estimate, matching tests/test_wall_following.py's
  precedent. The KF/both gates always run in parallel regardless of
  wall_following, observing whatever voltage was actually applied, purely
  for logging/mode-decision purposes.
- Both gates (sim.gate.step_gate for the NIS-based mode,
  sim.gate.step_tilt_gate for the tilt-threshold baseline) always run
  every tick regardless of config.controller, so all three controllers'
  episodes are directly comparable on identical seeds/disturbances
  (CLAUDE.md section 12) -- only which gate's mode actually drives control
  differs.
- Voltage clipping (CLAUDE.md section 5: "v = clip(u_cmd, -V_batt,
  +V_batt)") is applied PER MOTOR, to v_l/v_r independently, not to the
  combined LQR command before it is split in half. This is an
  intentional, tested divergence from tests/test_gate.py's
  _closed_loop_with_gate harness, which clips the combined command
  (`u = clip_voltage(u_cmd, v_batt)`) and then implicitly splits it in
  two by applying (u/2, u/2) to the plant. The two are equivalent only
  while both are unsaturated, i.e. |u_cmd| <= V_batt (every
  currently-tested nominal/disturbance scenario); they start diverging
  as soon as |u_cmd| > V_batt, which is where the old clip-then-split
  method begins saturating. (|u_cmd| > 2*V_batt is a different,
  unrelated threshold -- that's where the new per-motor clip itself
  starts saturating, since each motor sees |u_cmd/2| > V_batt.) See
  test_voltage_clips_per_motor_independently_when_saturated in
  tests/test_run.py. Per-motor clipping is used here because v_l/v_r are
  literally the plant's two actual motor-voltage inputs
  (sim.plant.f(x, v_l, v_r, p)), and it is also the only form that
  generalizes correctly to Task 5's wall-following episodes, where
  differential yaw drive makes v_l != v_r -- clipping the summed command
  and then splitting it in half would be wrong there.
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
    param_set,
    GateParams,
    PlantParams,
    SensorParams,
    default_corridor_params,
    default_estimator_params,
    default_fallback_params,
    default_gate_params,
    default_lqr_balance_params,
    default_run_params,
    default_speed_servo_params,
    default_speed_servo_params_cautious,
    default_tilt_gate_params,
    default_wall_follow_params,
    default_yaw_control_params,
)
from sim.plant import f as plant_f
from sim.sensors import accelerometer, encoder, gyro, hold_on_dropout, ultrasonic
from sim.world import cast_ray, pose_velocity

DisturbanceFn = Callable[
    [float, np.ndarray, float, PlantParams, SensorParams],
    tuple[np.ndarray, float, PlantParams, SensorParams, float],
]


class ControllerType(str, Enum):
    """The three controllers compared in CLAUDE.md section 12."""

    NAIVE = "naive"
    TILT_THRESHOLD = "tilt_threshold"
    NIS_GATE = "nis_gate"


@dataclass(frozen=True)
class EpisodeConfig:
    """One episode's setup. (config, seed) must reproduce an identical
    DataFrame (CLAUDE.md section 11)."""

    controller: ControllerType
    T: float                                  # s, episode duration (may end earlier on a fall)
    wall_following: bool = False
    x0_psi_deg: float = 0.0
    disturbance: Optional[DisturbanceFn] = None
    disturbance_name: str = "none"            # label only, for experiments/run_batch.py's output
    gate_params: Optional[GateParams] = None  # NIS-gate override (threshold selection re-simulates with this); None = default_gate_params()
    param_set: str = "sim_v0"                # CLAUDE.md 6.2: "sim_v0" (NXTway-GS) or "hardware_v1" (see sim.params.param_set)
    fallback: bool = True                     # outside NORMAL: KF Q x FallbackParams.q_inflation, HALT on the softer gain set (see FallbackParams)


def run_episode(config: EpisodeConfig, seed: int) -> pd.DataFrame:
    ps = param_set(config.param_set)
    p = ps.plant
    sensor_p = ps.sensor
    est_p = default_estimator_params()
    lqr_p = default_lqr_balance_params()
    gate_p = config.gate_params if config.gate_params is not None else default_gate_params()
    tilt_gate_p = default_tilt_gate_params()
    run_p = default_run_params()

    K4 = design_lqr_balance(p, lqr_p)

    wall_p = default_wall_follow_params()

    if config.wall_following:
        K5_nominal = design_lqr_speed_servo(p, default_speed_servo_params())
        K5_cautious = design_lqr_speed_servo(p, default_speed_servo_params_cautious())
        yaw_p = default_yaw_control_params()
        corridor_p = default_corridor_params()

        def pose_dotf(pp, uu):
            return pose_velocity(pp, uu, p)

    dt_control = run_p.dt_control
    dt_plant = run_p.dt_plant
    n_sub = int(round(dt_control / dt_plant))
    n_control_per_ultrasonic = int(round((1.0 / sensor_p.ultrasonic_rate_hz) / dt_control))

    Ad, Bd = discretize(p, dt_control)
    C = measurement_matrix()
    Q = process_noise(sensor_p, est_p, dt_control)
    R = measurement_noise(sensor_p)
    Q_fallback = Q * default_fallback_params().q_inflation
    mode = GateMode.NORMAL

    rng = np.random.default_rng(seed)
    n_steps = int(round(config.T / dt_control))

    x_true = np.array([0.0, np.radians(config.x0_psi_deg), 0.0, 0.0, 0.0, 0.0])
    pos = np.array([0.0, wall_p.target_distance])
    b_g_true = 0.0
    kf = KalmanState(x_hat=np.zeros(5), P=initial_covariance(est_p))
    nis_gate_state = initial_gate_state()
    tilt_gate_state = initial_tilt_gate_state()
    u_prev = 0.0
    theta_ref = 0.0
    z = 0.0
    phi_dot_ref = 0.0
    prev_wall_err = 0.0
    front_reading = sensor_p.ultrasonic_range_max
    right_reading = sensor_p.ultrasonic_range_max
    right_valid: float | None = None  # last non-dropout side reading (sim.sensors.hold_on_dropout)

    rows = []

    for k in range(n_steps):
        t = k * dt_control

        if config.disturbance is not None:
            x_true, b_g_true, p_now, sensor_p_now, v_batt_now = config.disturbance(
                t, x_true, b_g_true, p, sensor_p
            )
        else:
            p_now, sensor_p_now, v_batt_now = p, sensor_p, ps.v_batt_nominal

        def xdotf(x, uu, pp=p_now):
            return plant_f(x, uu[0], uu[1], pp)

        xdot_true = xdotf(x_true, (u_prev / 2, u_prev / 2))
        theta_ddot_true = xdot_true[3]

        theta_enc = encoder(x_true[0], x_true[1], x_true[2], p_now, sensor_p_now)
        psi_dot_meas, b_g_true = gyro(x_true[4], b_g_true, dt_control, sensor_p_now, rng)
        psi_acc_meas = accelerometer(x_true[1], theta_ddot_true, p_now, sensor_p_now, rng)
        y = np.array([theta_enc, psi_dot_meas, psi_acc_meas])

        # Fallback: once the driving gate has left NORMAL (previous tick's decision), the KF
        # trusts the sensors over its nominal model (FallbackParams, firmware parity).
        Q_now = Q_fallback if (config.fallback and mode is not GateMode.NORMAL) else Q
        kf = predict(kf, Ad, Bd, u_prev, Q_now)
        kf, nis = update(kf, y, C, R)
        nis_gate_state, epsilon = step_gate(nis_gate_state, nis, dt_control, gate_p)
        tilt_gate_state = step_tilt_gate(tilt_gate_state, kf.x_hat[1], dt_control, tilt_gate_p)

        if config.controller is ControllerType.NAIVE:
            mode = GateMode.NORMAL
        elif config.controller is ControllerType.TILT_THRESHOLD:
            mode = tilt_gate_state.mode
        else:
            mode = nis_gate_state.mode

        fallen = bool(abs(x_true[1]) > run_p.fall_psi_threshold)

        if config.wall_following:
            if k % n_control_per_ultrasonic == 0:
                heading = x_true[2]
                true_right = cast_ray(pos[0], pos[1], heading - np.pi / 2, corridor_p, 0.02, 4.0)
                true_front = cast_ray(pos[0], pos[1], heading, corridor_p, 0.02, 4.0)
                right_reading = ultrasonic(true_right, sensor_p, rng)
                right_valid = hold_on_dropout(right_reading, right_valid, sensor_p)
                front_reading = ultrasonic(true_front, sensor_p, rng)
                wall_err = wall_p.target_distance - right_valid
                phi_dot_ref, prev_wall_err = wall_following_control(
                    wall_err, prev_wall_err, 1.0 / sensor_p.ultrasonic_rate_hz, wall_p
                )

            base_ref = front_threshold_speed_adjust(front_reading, run_p.corridor_theta_dot_ref_nominal, wall_p)
            if mode is GateMode.NORMAL:
                theta_dot_ref, K5 = base_ref, K5_nominal
            elif mode is GateMode.CAUTIOUS:
                theta_dot_ref, K5 = base_ref * run_p.cautious_speed_scale, K5_cautious
            else:  # HALT: speed ref = 0, "keep balancing in place" -- softer gains with fallback, else nominal (plan doc "Design decision 6")
                theta_dot_ref, K5 = 0.0, (K5_cautious if config.fallback else K5_nominal)

            err5 = np.array([x_true[0] - theta_ref, x_true[1], x_true[3] - theta_dot_ref, x_true[4], z])
            theta_ref += theta_dot_ref * dt_control
            u_common = -K5 @ err5
            z += (x_true[0] - theta_ref) * dt_control
            diff_v = yaw_p_control(phi_dot_ref, x_true[5], yaw_p)
            v_l_cmd = u_common / 2 - diff_v / 2
            v_r_cmd = u_common / 2 + diff_v / 2
        else:
            u_cmd = -K4 @ kf.x_hat[:4]
            v_l_cmd = v_r_cmd = u_cmd / 2

        if fallen:
            v_l, v_r = 0.0, 0.0
        else:
            # Per-motor clip (not combined-then-split) -- see module docstring.
            v_l = clip_voltage(v_l_cmd, v_batt_now)
            v_r = clip_voltage(v_r_cmd, v_batt_now)
        u_prev = v_l + v_r

        rows.append({
            "t": t,
            "theta": x_true[0], "psi": x_true[1], "phi": x_true[2],
            "theta_dot": x_true[3], "psi_dot": x_true[4], "phi_dot": x_true[5],
            "pos_x": pos[0] if config.wall_following else np.nan,
            "pos_y": pos[1] if config.wall_following else np.nan,
            "mode": mode.value,
            "nis": nis, "epsilon": epsilon,
            "front_dist": front_reading if config.wall_following else np.nan,
            "right_dist": right_reading if config.wall_following else np.nan,
            "v_l": v_l, "v_r": v_r,
            "fallen": fallen,
        })

        if fallen:
            break

        for _ in range(n_sub):
            x_true = rk4_step(xdotf, x_true, (v_l, v_r), dt_plant)
            if config.wall_following:
                pos = rk4_step(pose_dotf, pos, (x_true[3], x_true[2]), dt_plant)

    return pd.DataFrame(rows)
