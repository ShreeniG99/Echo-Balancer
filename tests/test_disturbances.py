import numpy as np

from sim.disturbances import (
    battery_droop_v_batt,
    clip_voltage,
    payload_shift_plant_params,
    push_psi_dot_kick,
    sensor_fault_accel_noise_params,
    sensor_fault_gyro_bias_step,
    surface_change_plant_params,
)
from sim.params import default_plant_params, default_sensor_params


def test_surface_change_active_only_during_window():
    p = default_plant_params()

    before = surface_change_plant_params(t=4.9, onset=5.0, duration=2.0, fw_value=0.1, nominal_p=p)
    during = surface_change_plant_params(t=6.0, onset=5.0, duration=2.0, fw_value=0.1, nominal_p=p)
    after = surface_change_plant_params(t=7.1, onset=5.0, duration=2.0, fw_value=0.1, nominal_p=p)

    assert before.fw == p.fw
    assert during.fw == 0.1
    assert after.fw == p.fw


def test_battery_droop_ramps_linearly_then_holds():
    before = battery_droop_v_batt(t=4.0, onset=5.0, duration=2.0, v_full=7.4, v_drooped=6.0)
    mid = battery_droop_v_batt(t=6.0, onset=5.0, duration=2.0, v_full=7.4, v_drooped=6.0)
    after = battery_droop_v_batt(t=10.0, onset=5.0, duration=2.0, v_full=7.4, v_drooped=6.0)

    assert before == 7.4
    assert np.isclose(mid, 7.4 + 0.5 * (6.0 - 7.4))
    assert after == 6.0


def test_clip_voltage():
    assert clip_voltage(3.0, 7.4) == 3.0
    assert clip_voltage(10.0, 7.4) == 7.4
    assert clip_voltage(-10.0, 7.4) == -7.4


def test_push_psi_dot_kick_applies_once_at_onset():
    before = push_psi_dot_kick(psi_dot=0.5, t=4.995, onset=5.0, dt=0.005, magnitude=1.0)
    at_onset = push_psi_dot_kick(psi_dot=0.5, t=5.0, onset=5.0, dt=0.005, magnitude=1.0)
    after = push_psi_dot_kick(psi_dot=0.5, t=5.005, onset=5.0, dt=0.005, magnitude=1.0)

    assert before == 0.5
    assert at_onset == 1.5
    assert after == 0.5


def test_sensor_fault_gyro_bias_step_applies_once_at_onset():
    before = sensor_fault_gyro_bias_step(b_g=0.01, t=4.995, onset=5.0, dt=0.005, magnitude=0.05)
    at_onset = sensor_fault_gyro_bias_step(b_g=0.01, t=5.0, onset=5.0, dt=0.005, magnitude=0.05)

    assert before == 0.01
    assert np.isclose(at_onset, 0.06)


def test_sensor_fault_accel_noise_params_scales_sigma_during_window():
    sp = default_sensor_params()

    before = sensor_fault_accel_noise_params(t=4.9, onset=5.0, duration=2.0, multiplier=5.0, nominal_sp=sp)
    during = sensor_fault_accel_noise_params(t=6.0, onset=5.0, duration=2.0, multiplier=5.0, nominal_sp=sp)

    assert before.sigma_accel == sp.sigma_accel
    assert during.sigma_accel == sp.sigma_accel * 5.0


def test_payload_shift_persists_after_onset():
    p = default_plant_params()

    before = payload_shift_plant_params(t=4.9, onset=5.0, delta_M=1.0, delta_L=0.1, nominal_p=p)
    after = payload_shift_plant_params(t=100.0, onset=5.0, delta_M=1.0, delta_L=0.1, nominal_p=p)

    assert before.M == p.M
    assert np.isclose(after.M, p.M + 1.0)
    assert np.isclose(after.L, p.L + 0.1)
