"""Scheduled disturbance profiles (CLAUDE.md section 8).

Each disturbance is a pure function of elapsed time t (plus onset/
duration/magnitude), returning the *disturbed* value(s) to use for that
instant -- the caller (a test harness today; sim/run.py later) substitutes
these in place of the nominal values when calling sim.plant.f / sim.sensors
functions. The estimator's own Ad/Bd/Q/R are always built from the
*nominal* PlantParams/SensorParams (CLAUDE.md section 8: "The estimator
always uses the nominal model") -- these functions never touch
sim.estimator or sim.plant.
"""

import numpy as np
from dataclasses import replace

from sim.params import PlantParams, SensorParams


def surface_change_plant_params(
    t: float, onset: float, duration: float, fw_value: float, nominal_p: PlantParams
) -> PlantParams:
    """f_w steps from 0 to fw_value during [onset, onset+duration), then
    reverts (CLAUDE.md #1: a loose-gravel patch, not a permanent change)."""
    if onset <= t < onset + duration:
        return replace(nominal_p, fw=fw_value)
    return nominal_p


def battery_droop_v_batt(t: float, onset: float, duration: float, v_full: float, v_drooped: float) -> float:
    """V_batt ramps linearly from v_full to v_drooped over
    [onset, onset+duration), then holds at v_drooped (CLAUDE.md #2: a real
    battery does not recover on its own)."""
    if t < onset:
        return v_full
    if t >= onset + duration:
        return v_drooped
    frac = (t - onset) / duration
    return v_full + frac * (v_drooped - v_full)


def clip_voltage(u_cmd: float, v_batt: float) -> float:
    """CLAUDE.md section 5: v = clip(u_cmd, -V_batt, +V_batt)."""
    return float(np.clip(u_cmd, -v_batt, v_batt))


def push_psi_dot_kick(psi_dot: float, t: float, onset: float, dt: float, magnitude: float) -> float:
    """CLAUDE.md #3: an impulse torque on psi, modeled as an instantaneous
    psi_dot kick (exact via the impulse-momentum theorem for some implied
    angular impulse; sized directly in rad/s rather than requiring an
    inertia conversion). Applied once, in the control step whose interval
    contains `onset`."""
    if onset <= t < onset + dt:
        return psi_dot + magnitude
    return psi_dot


def sensor_fault_gyro_bias_step(b_g: float, t: float, onset: float, dt: float, magnitude: float) -> float:
    """CLAUDE.md #4 (variant a): a persistent gyro bias step at onset."""
    if onset <= t < onset + dt:
        return b_g + magnitude
    return b_g


def sensor_fault_accel_noise_params(
    t: float, onset: float, duration: float, multiplier: float, nominal_sp: SensorParams
) -> SensorParams:
    """CLAUDE.md #4 (variant b): accelerometer noise std x k during
    [onset, onset+duration)."""
    if onset <= t < onset + duration:
        return replace(nominal_sp, sigma_accel=nominal_sp.sigma_accel * multiplier)
    return nominal_sp


def payload_shift_plant_params(
    t: float, onset: float, delta_M: float, delta_L: float, nominal_p: PlantParams
) -> PlantParams:
    """CLAUDE.md #5: M and/or L shift at onset and stay shifted (a payload
    picked up and not put back down). Plant only -- the estimator keeps
    the nominal model (CLAUDE.md section 8)."""
    if t >= onset:
        return replace(nominal_p, M=nominal_p.M + delta_M, L=nominal_p.L + delta_L)
    return nominal_p
