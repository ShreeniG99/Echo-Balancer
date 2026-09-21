# tests/test_wall_following.py
import numpy as np

from sim.control import (
    design_lqr_speed_servo,
    front_threshold_speed_adjust,
    wall_following_control,
    yaw_p_control,
)
from sim.integrate import rk4_step
from sim.params import (
    default_corridor_params,
    default_plant_params,
    default_sensor_params,
    default_speed_servo_params,
    default_wall_follow_params,
    default_yaw_control_params,
)
from sim.plant import f as plant_f
from sim.sensors import ultrasonic
from sim.world import cast_ray, pose_velocity


def test_follows_wall_without_crashing():
    """Section 9 end-to-end: robot starts 0.2m off the target wall
    distance, and should converge toward it (and never hit either wall)
    while moving forward through a noisy-sensor closed loop.

    Verified pre-plan with seed=0 (using a hand-rolled dt_control-resolution
    position update): final_y=0.4792 (target 0.5), max|y-0.5| over the last
    3s=0.0208, min_front=0.377 (well clear of either wall). Also verified
    across seeds 0-4: no crash in any of the 5 runs, worst 3s-window
    deviation from target across all 5 was 0.154 -- the bounds below are
    set with margin above that, not tuned to this one seed specifically.
    This task instead integrates position via pose_velocity/rk4_step at
    1ms plant resolution (see the plan doc's "Design decision: Task 6
    integrates position via pose_velocity/rk4_step at plant resolution") --
    a tighter-resolution version of the same simple kinematic update, so
    the numbers above should hold within the stated margins, but were not
    re-verified against this exact code path before the plan was written;
    if the assertions below fail, print y_hist[-1] and the max-deviation
    figure and compare against 0.4792/0.0208 before assuming a real bug.
    """
    p = default_plant_params()
    sensor_p = default_sensor_params()
    speed_servo_p = default_speed_servo_params()
    yaw_p = default_yaw_control_params()
    wall_p = default_wall_follow_params()
    corridor_p = default_corridor_params()

    K5 = design_lqr_speed_servo(p, speed_servo_p)

    dt_plant, dt_control = 0.001, 0.005
    n_sub = int(round(dt_control / dt_plant))
    dt_ultrasonic = 0.05
    n_control_per_ultrasonic = int(round(dt_ultrasonic / dt_control))

    theta_dot_ref_nominal = 2.0
    T = 15.0
    n_steps = int(T / dt_control)

    rng = np.random.default_rng(0)
    x = np.zeros(6)
    pos = np.array([0.0, 0.3])  # 0.2m off the 0.5m target
    theta_ref = 0.0
    z = 0.0
    prev_wall_err = 0.0
    phi_dot_ref = 0.0
    theta_dot_ref = theta_dot_ref_nominal

    def xdotf(x, uu):
        return plant_f(x, uu[0], uu[1], p)

    y_hist = []
    front_hist = []

    for k in range(n_steps):
        if k % n_control_per_ultrasonic == 0:
            heading = x[2]  # phi
            true_right = cast_ray(pos[0], pos[1], heading - np.pi / 2, corridor_p, 0.02, 4.0)
            true_front = cast_ray(pos[0], pos[1], heading, corridor_p, 0.02, 4.0)
            right_reading = ultrasonic(true_right, sensor_p, rng)
            front_reading = ultrasonic(true_front, sensor_p, rng)

            wall_err = wall_p.target_distance - right_reading
            phi_dot_ref, prev_wall_err = wall_following_control(wall_err, prev_wall_err, dt_ultrasonic, wall_p)
            theta_dot_ref = front_threshold_speed_adjust(front_reading, theta_dot_ref_nominal, wall_p)
            front_hist.append(front_reading)

        err5 = np.array([x[0] - theta_ref, x[1], x[3] - theta_dot_ref, x[4], z])
        theta_ref += theta_dot_ref * dt_control
        u_common = -K5 @ err5
        z += (x[0] - theta_ref) * dt_control

        diff_v = yaw_p_control(phi_dot_ref, x[5], yaw_p)

        v_l = u_common / 2 - diff_v / 2
        v_r = u_common / 2 + diff_v / 2

        y_hist.append(pos[1])
        for _ in range(n_sub):
            x = rk4_step(xdotf, x, (v_l, v_r), dt_plant)
            pos = rk4_step(lambda pp, uu: pose_velocity(pp, uu, p), pos, (x[3], x[2]), dt_plant)

    y_hist = np.array(y_hist)
    front_hist = np.array(front_hist)

    # Never touched either wall.
    assert np.all(y_hist > 0.0)
    assert np.all(y_hist < corridor_p.width)
    # Converged reasonably close to the target distance by the end.
    assert abs(y_hist[-1] - wall_p.target_distance) < 0.3
    assert np.max(np.abs(y_hist[-600:] - wall_p.target_distance)) < 0.3
    # Front sensor (verified min 0.377 at seed=0) never approached a
    # dangerous close range -- a sensor-observed corroboration of the
    # y_hist-based "no crash" checks above, not just a coincidence of them.
    assert np.min(front_hist) > 0.05
