import numpy as np

from sim.params import default_plant_params, default_sensor_params
from sim.sensors import accelerometer, encoder, gyro, ultrasonic


def test_encoder_reads_theta_minus_psi_when_no_yaw():
    p = default_plant_params()
    sp = default_sensor_params()
    theta, psi, phi = 0.1, 0.02, 0.0

    reading = encoder(theta, psi, phi, p, sp)

    step = 2 * np.pi / sp.encoder_cpr
    expected = round((theta - psi) / step) * step
    assert abs(reading - expected) < 1e-12


def test_encoder_quantizes_to_step_multiples():
    p = default_plant_params()
    sp = default_sensor_params()
    step = 2 * np.pi / sp.encoder_cpr

    reading = encoder(0.123456, 0.0, 0.0, p, sp)

    assert np.isclose(round(reading / step), reading / step)


def test_gyro_mean_and_std_match_spec_over_many_samples():
    sp = default_sensor_params()
    rng = np.random.default_rng(0)
    psi_dot_true = 0.3
    b_g = 0.01
    residuals = []
    for _ in range(20000):
        meas, b_g = gyro(psi_dot_true, b_g, dt=0.005, sensor_p=sp, rng=rng)
        residuals.append(meas - b_g)  # b_g already includes this step's walk
    residuals = np.array(residuals)

    assert abs(np.mean(residuals) - psi_dot_true) < 0.001
    assert abs(np.std(residuals) - sp.sigma_gyro) < 0.001


def test_accelerometer_reads_psi_at_equilibrium_on_average():
    p = default_plant_params()
    sp = default_sensor_params()
    rng = np.random.default_rng(0)
    psi = np.radians(5.0)

    samples = [accelerometer(psi, 0.0, p, sp, rng) for _ in range(20000)]

    assert abs(np.mean(samples) - psi) < 0.001
    assert abs(np.std(samples) - sp.sigma_accel) < 0.001


def test_accelerometer_is_corrupted_by_linear_acceleration():
    p = default_plant_params()
    sp = default_sensor_params()
    rng = np.random.default_rng(0)
    psi = 0.0

    at_rest = np.mean([accelerometer(psi, 0.0, p, sp, rng) for _ in range(2000)])
    accelerating = np.mean([accelerometer(psi, 5.0, p, sp, rng) for _ in range(2000)])

    assert abs(accelerating - at_rest) > 0.01


def test_encoder_quantizes_per_wheel_before_averaging():
    """The two per-wheel readings are quantized independently, THEN
    averaged -- not averaged first and quantized once. These give
    different answers whenever yaw (phi) is nonzero. See the plan's
    "Sensor models" design-decision section for why this reading was
    chosen over the simpler alternative."""
    p = default_plant_params()
    sp = default_sensor_params()
    step = 2 * np.pi / sp.encoder_cpr

    theta = 0.3 * step
    psi = 0.0
    half_track_over_R = p.W / (2 * p.R)
    phi = 0.4 * step / half_track_over_R  # chosen so half_track_over_R * phi = 0.4*step

    reading = encoder(theta, psi, phi, p, sp)

    average_then_quantize = round(
        ((theta - half_track_over_R * phi - psi) + (theta + half_track_over_R * phi - psi))
        / 2
        / step
    ) * step

    # The two approaches must disagree here (per-wheel quantization then
    # averaging is what's actually implemented).
    assert not np.isclose(reading, average_then_quantize)
    assert np.isclose(reading, 0.5 * step)


def test_ultrasonic_dropout_rate_matches_spec():
    sp = default_sensor_params()
    rng = np.random.default_rng(0)

    samples = [ultrasonic(1.0, sp, rng) for _ in range(20000)]
    frac_at_max_range = np.mean(np.isclose(samples, sp.ultrasonic_range_max))

    assert abs(frac_at_max_range - sp.ultrasonic_dropout_prob) < 0.01


def test_ultrasonic_noise_mean_and_std_excluding_dropouts():
    sp = default_sensor_params()
    rng = np.random.default_rng(0)
    true_distance = 1.0

    non_dropout_samples = []
    while len(non_dropout_samples) < 20000:
        reading = ultrasonic(true_distance, sp, rng)
        if not np.isclose(reading, sp.ultrasonic_range_max):
            non_dropout_samples.append(reading)
    samples = np.array(non_dropout_samples)

    assert abs(np.mean(samples) - true_distance) < 0.001
    assert abs(np.std(samples) - sp.ultrasonic_sigma) < 0.001


def test_ultrasonic_clips_out_of_range_true_distance():
    sp = default_sensor_params()
    rng = np.random.default_rng(0)

    reading = ultrasonic(100.0, sp, rng)

    assert reading == sp.ultrasonic_range_max
