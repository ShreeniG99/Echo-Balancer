import numpy as np

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
from sim.params import default_estimator_params, default_plant_params, default_sensor_params


def test_discretize_bias_state_is_a_pure_integrator():
    p = default_plant_params()

    Ad, Bd = discretize(p, dt=0.005)

    assert Ad.shape == (5, 5)
    assert Bd.shape == (5,)
    np.testing.assert_allclose(Ad[4, :], [0.0, 0.0, 0.0, 0.0, 1.0])
    assert Bd[4] == 0.0


def test_measurement_matrix_values():
    C = measurement_matrix()

    expected = np.array([
        [1.0, -1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 1.0],
        [0.0, 1.0, 0.0, 0.0, 0.0],
    ])
    np.testing.assert_array_equal(C, expected)


def test_process_and_measurement_noise_shapes_and_values():
    sensor_p = default_sensor_params()
    est_p = default_estimator_params()

    Q = process_noise(sensor_p, est_p, dt=0.005)
    R = measurement_noise(sensor_p)
    P0 = initial_covariance(est_p)

    assert Q.shape == (5, 5)
    assert R.shape == (3, 3)
    assert P0.shape == (5, 5)
    assert Q[4, 4] == sensor_p.gyro_bias_walk**2 * 0.005
    assert R[1, 1] == sensor_p.sigma_gyro**2
    assert R[2, 2] == sensor_p.sigma_accel**2


def test_update_never_increases_uncertainty():
    """Standard KF property: a measurement update cannot increase the
    trace of the state covariance."""
    C = measurement_matrix()
    R = np.diag([1e-4, 1e-4, 1e-4])
    state = KalmanState(x_hat=np.zeros(5), P=np.diag([1e-2, 1e-2, 1e-2, 1e-2, 1e-2]))
    y = np.array([0.01, -0.02, 0.03])

    updated, nis = update(state, y, C, R)

    assert np.trace(updated.P) <= np.trace(state.P)
    assert nis >= 0.0


def test_nis_matches_hand_computed_scalar_example():
    """1-state, 1-measurement toy case: innovation=3-0=3, S=P+R=4+1=5,
    NIS = innovation^2 / S = 9/5 = 1.8."""
    state = KalmanState(x_hat=np.array([0.0]), P=np.array([[4.0]]))
    C = np.array([[1.0]])
    R = np.array([[1.0]])
    y = np.array([3.0])

    _, nis = update(state, y, C, R)

    assert np.isclose(nis, 1.8)
