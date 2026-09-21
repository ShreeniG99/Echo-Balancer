import numpy as np
import pytest

from sim.integrate import rk4_step
from sim.params import CorridorParams, default_plant_params
from sim.world import cast_ray, pose_velocity


def test_cast_ray_straight_up_and_down_from_center():
    cp = CorridorParams(width=1.0, ray_parallel_eps=1e-9)

    assert cast_ray(0.0, 0.5, np.radians(90), cp, 0.02, 4.0) == pytest.approx(0.5)
    assert cast_ray(0.0, 0.5, np.radians(-90), cp, 0.02, 4.0) == pytest.approx(0.5)


def test_cast_ray_parallel_to_walls_returns_max_range():
    cp = CorridorParams(width=1.0, ray_parallel_eps=1e-9)

    assert cast_ray(0.0, 0.5, np.radians(0), cp, 0.02, 4.0) == 4.0


def test_cast_ray_near_wall():
    cp = CorridorParams(width=1.0, ray_parallel_eps=1e-9)

    assert cast_ray(0.0, 0.1, np.radians(90), cp, 0.02, 4.0) == pytest.approx(0.9)
    assert cast_ray(0.0, 0.1, np.radians(-90), cp, 0.02, 4.0) == pytest.approx(0.1)


def test_cast_ray_diagonal():
    cp = CorridorParams(width=1.0, ray_parallel_eps=1e-9)

    assert cast_ray(0.0, 0.5, np.radians(135), cp, 0.02, 4.0) == pytest.approx(0.5 * np.sqrt(2))


def test_cast_ray_clips_to_range_bounds():
    cp = CorridorParams(width=10.0, ray_parallel_eps=1e-9)

    # true distance 0.001m, below range_min=0.02 -> clipped up
    assert cast_ray(0.0, 0.001, np.radians(-90), cp, 0.02, 4.0) == 0.02
    # true distance 9.5m (half of a 10m-wide corridor's far wall), above range_max=4.0 -> clipped down
    assert cast_ray(0.0, 0.5, np.radians(90), cp, 0.02, 4.0) == 4.0


def test_pose_velocity_matches_unicycle_kinematics():
    p = default_plant_params()

    vel_along_x = pose_velocity(np.array([0.0, 0.0]), (2.0, 0.0), p)
    assert vel_along_x[0] == pytest.approx(p.R * 2.0)
    assert vel_along_x[1] == pytest.approx(0.0, abs=1e-9)

    vel_along_y = pose_velocity(np.array([0.0, 0.0]), (2.0, np.pi / 2), p)
    assert vel_along_y[0] == pytest.approx(0.0, abs=1e-9)
    assert vel_along_y[1] == pytest.approx(p.R * 2.0)


def test_pose_integrates_via_rk4_step():
    p = default_plant_params()

    def vel_fn(pos, u):
        return pose_velocity(pos, u, p)

    pos = np.array([0.0, 0.0])
    dt = 0.01
    for _ in range(100):  # 1 simulated second, theta_dot=2 rad/s, phi=0 (heading +x)
        pos = rk4_step(vel_fn, pos, (2.0, 0.0), dt)

    assert pos[0] == pytest.approx(p.R * 2.0 * 1.0, rel=1e-6)
    assert pos[1] == pytest.approx(0.0, abs=1e-9)
