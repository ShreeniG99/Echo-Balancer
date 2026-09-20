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
    dt = 0.005

    Q = process_noise(sensor_p, est_p, dt)
    R = measurement_noise(sensor_p)
    P0 = initial_covariance(est_p)

    assert Q.shape == (5, 5)
    assert R.shape == (3, 3)
    assert P0.shape == (5, 5)

    np.testing.assert_allclose(
        np.diag(Q),
        [est_p.Q_theta, est_p.Q_psi, est_p.Q_theta_dot, est_p.Q_psi_dot, sensor_p.gyro_bias_walk**2 * dt],
    )
    step = 2 * np.pi / sensor_p.encoder_cpr
    np.testing.assert_allclose(
        np.diag(R), [step**2 / 12.0, sensor_p.sigma_gyro**2, sensor_p.sigma_accel**2]
    )
    np.testing.assert_allclose(
        np.diag(P0),
        [est_p.P0_theta, est_p.P0_psi, est_p.P0_theta_dot, est_p.P0_psi_dot, est_p.P0_bg],
    )


def test_predict_matches_direct_formula():
    Ad = np.array([[1.0, 0.5], [0.0, 1.0]])
    Bd = np.array([0.1, 0.2])
    Q = np.diag([0.01, 0.02])
    state = KalmanState(x_hat=np.array([1.0, 2.0]), P=np.diag([0.5, 0.5]))
    u = 3.0

    predicted = predict(state, Ad, Bd, u, Q)

    expected_x = Ad @ state.x_hat + Bd * u
    expected_P = Ad @ state.P @ Ad.T + Q
    np.testing.assert_allclose(predicted.x_hat, expected_x)
    np.testing.assert_allclose(predicted.P, expected_P)


def test_discretize_planar_block_matches_direct_ode_integration():
    """Independent ground truth: integrate dx/dt = A_planar @ x + B_planar * u
    directly with scipy.integrate.solve_ivp over one dt, and compare against
    Ad[:4,:4] @ x0 + Bd[:4] * u."""
    from scipy.integrate import solve_ivp

    from sim.linearize import linearize_planar

    p = default_plant_params()
    dt = 0.005
    A_planar, B_planar = linearize_planar(p)

    Ad, Bd = discretize(p, dt)

    x0 = np.array([0.01, -0.02, 0.03, -0.04])
    u = 0.5

    def rhs(t, x):
        return A_planar @ x + B_planar * u

    sol = solve_ivp(rhs, [0, dt], x0, rtol=1e-12, atol=1e-14)
    x_direct = sol.y[:, -1]

    x_via_discretize = Ad[:4, :4] @ x0 + Bd[:4] * u

    np.testing.assert_allclose(x_via_discretize, x_direct, rtol=1e-6, atol=1e-9)


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
