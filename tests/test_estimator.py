import numpy as np
import scipy.stats as stats

from sim.control import design_lqr_balance
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
from sim.integrate import rk4_step
from sim.params import (
    default_estimator_params,
    default_lqr_balance_params,
    default_plant_params,
    default_sensor_params,
)
from sim.plant import f as plant_f
from sim.sensors import accelerometer, encoder, gyro


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


def _run_nominal_closed_loop(seed: int, T: float = 60.0) -> np.ndarray:
    """60s of nominal (no disturbance) closed-loop balancing: full 6-state
    nonlinear plant.f, RK4 at dt_plant=1ms, KF+LQR at dt_control=5ms (ZOH
    on voltage between control ticks). Returns the per-control-step NIS
    array (length T/dt_control).
    """
    p = default_plant_params()
    sensor_p = default_sensor_params()
    est_p = default_estimator_params()
    lqr_p = default_lqr_balance_params()

    K = design_lqr_balance(p, lqr_p)
    dt_control = 0.005
    dt_plant = 0.001
    n_sub = int(round(dt_control / dt_plant))

    Ad, Bd = discretize(p, dt_control)
    C = measurement_matrix()
    Q = process_noise(sensor_p, est_p, dt_control)
    R = measurement_noise(sensor_p)

    rng = np.random.default_rng(seed)
    n_steps = int(round(T / dt_control))

    x_true = np.zeros(6)
    b_g_true = 0.0
    kf = KalmanState(x_hat=np.zeros(5), P=initial_covariance(est_p))
    u_prev = 0.0
    nis_history = np.zeros(n_steps)

    def xdotf(x, uu):
        return plant_f(x, uu[0], uu[1], p)

    for k in range(n_steps):
        xdot_true = plant_f(x_true, u_prev / 2, u_prev / 2, p)
        theta_ddot_true = xdot_true[3]

        theta_enc = encoder(x_true[0], x_true[1], x_true[2], p, sensor_p)
        psi_dot_meas, b_g_true = gyro(x_true[4], b_g_true, dt_control, sensor_p, rng)
        psi_acc_meas = accelerometer(x_true[1], theta_ddot_true, p, sensor_p, rng)
        y = np.array([theta_enc, psi_dot_meas, psi_acc_meas])

        kf = predict(kf, Ad, Bd, u_prev, Q)
        kf, nis = update(kf, y, C, R)
        nis_history[k] = nis

        u = -K @ kf.x_hat[:4]
        u_prev = u

        for _ in range(n_sub):
            x_true = rk4_step(xdotf, x_true, (u / 2, u / 2), dt_plant)

    return nis_history


def test_nis_consistent_on_nominal_60s_run():
    """Section 11: 'KF, nominal plant, no disturbances, 60 s: mean NIS
    within the 95% interval for chi2(3) averaged over the run; ~5% of
    samples above the chi2(3) 95% quantile.'

    See the plan's "Design decision" section for why the acceptance
    interval on the mean is chi2(3)'s own 95% interval, not the much
    tighter interval for the mean of 12000 i.i.d. chi2(3) samples: this
    closed loop's NIS samples are correlated (through the KF's own
    smoothing and the near-deterministic encoder-quantization pattern), so
    the naive variance-shrinks-as-1/N assumption underlying that tighter
    interval does not hold here -- confirmed empirically (mean NIS varied
    between 2.67 and 3.07 across just 4 seeds with zero added process
    noise, ~10x wider than the +/-0.044 the i.i.d. formula predicts).

    Verified with seeds {0, 1, 7, 42, 123} -- all pass with mean in
    [2.81, 2.95] and fraction-above in [0.037, 0.043]. If your
    transcription doesn't pass with seed=42, that's almost certainly a
    transcription bug (check function call order and rng draw order
    against the plan's pre-verified script first) -- it is not a
    tolerance to loosen.
    """
    chi2_3 = stats.chi2(df=3)
    nis = _run_nominal_closed_loop(seed=42, T=60.0)

    mean_nis = np.mean(nis)
    frac_above_95 = np.mean(nis > chi2_3.ppf(0.95))

    assert chi2_3.ppf(0.025) <= mean_nis <= chi2_3.ppf(0.975)
    assert 0.01 <= frac_above_95 <= 0.10
