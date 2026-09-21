"""2D corridor geometry, ray casting, and robot pose kinematics
(CLAUDE.md section 4; section 7's ultrasonic model needs the ray casting
here). The corridor is two infinite parallel walls -- no corners or dead
ends; see the plan doc's "Design decision: the corridor is two infinite
parallel walls".
"""

import numpy as np

from sim.params import CorridorParams, PlantParams


def cast_ray(
    x: float, y: float, ray_heading: float, corridor_p: CorridorParams, range_min: float, range_max: float
) -> float:
    """Distance from (x, y) to the nearest corridor wall along ray_heading
    (an absolute world-frame angle, radians), clipped to
    [range_min, range_max]. Walls are at y=0 and y=corridor_p.width.
    """
    sin_h = np.sin(ray_heading)
    eps = 1e-9
    if sin_h > eps:
        dist = (corridor_p.width - y) / sin_h
    elif sin_h < -eps:
        dist = -y / sin_h
    else:
        dist = range_max
    return float(np.clip(dist, range_min, range_max))


def pose_velocity(pos: np.ndarray, u: tuple[float, float], p: PlantParams) -> np.ndarray:
    """Unicycle kinematics for the robot's (x, y) position: pos=[x, y],
    u=(theta_dot, phi). Rolling without slip (matches the encoder/plant
    model's own convention: axle-midpoint speed = R*theta_dot, heading =
    phi). Designed to be used with sim.integrate.rk4_step, holding
    (theta_dot, phi) constant across a step like any other ZOH input --
    matching sim.plant.f's (x, u) -> xdot calling convention.
    """
    theta_dot, phi = u
    return np.array([p.R * theta_dot * np.cos(phi), p.R * theta_dot * np.sin(phi)])
