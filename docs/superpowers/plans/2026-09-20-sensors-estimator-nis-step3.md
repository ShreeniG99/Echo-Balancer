# Echo Balancer — Section 14 Step 3: Sensors, Estimator, NIS Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `sim/sensors.py` (gyro, accelerometer, encoder — the sensors the balance estimator consumes) and `sim/estimator.py` (discrete linear Kalman filter + NIS), and pass the section 11 NIS-consistency test on a nominal 60 s closed-loop run.

**Architecture:** `sim/sensors.py` exposes three pure functions (gyro, accelerometer, encoder), each a physically-motivated noise model over the true plant state. `sim/estimator.py` builds a 5-state `[theta, psi, theta_dot, psi_dot, b_g]` discrete KF on top of `sim/linearize.py`'s planar model (ZOH-discretized), with `predict`/`update` functions matching `sim/integrate.py`'s style (pure functions, explicit state threading, no mutation). The NIS test wires all of this together with `sim/plant.py`, `sim/integrate.py`, and `sim/control.py` into a self-contained 60 s closed-loop simulation, exactly like `sim/run.py` will eventually do generally (that module is a later build step — this test is not it, see "Explicitly out of scope" below).

**Tech Stack:** Python ≥3.11, `numpy`, `scipy` (`scipy.linalg.expm` for ZOH discretization, `scipy.stats.chi2` for the NIS test's acceptance bounds).

**Explicitly out of scope for this plan:** `sim/gate.py`, `sim/disturbances.py`, `sim/world.py`, `sim/run.py`, and everything past section 14 build-order step 3. Specifically:
- **Ultrasonic sensor model** is not implemented here. It needs 2D corridor ray-casting (`sim/world.py`, build-order step 5) and is not part of the KF's measurement vector (CLAUDE.md section 9 lists `y = [theta_enc, psi_dot_gyro, psi_acc]`, no ultrasonic) — nothing in this step depends on it. Its noise parameters are still added to `sim/params.py` now (see Task 1), matching CLAUDE.md section 7's "All σ values are starting points; list them in params.py," which doesn't gate on which module consumes them yet (the same pattern as adding `LQRBalanceParams` in step 2 before speed-servo/yaw-PD/wall-following existed).
- **The windowed statistic ε_k** (CLAUDE.md section 9's "Σ over the last N samples of NIS") and the **gate state machine** are `sim/gate.py`, build-order step 4.
- **Disturbance injection** is `sim/disturbances.py`, also step 4. This step's NIS test is explicitly the *nominal, no-disturbance* case from section 11.

---

## Pre-verified design: sensor models, KF tuning, and the NIS test's acceptance bounds

CLAUDE.md itself warns "Expect Q/R tuning to take the longest" for this step. Before writing this plan I built the full closed-loop simulation (plant + sensors + KF + LQR, exactly as Task 4 below implements) as a throwaway script against this repo's actual `sim/params.py`/`sim/plant.py`/`sim/control.py`/`sim/linearize.py`, and iterated until it passed. Everything below is derived from that, not guessed.

### Sensor models (implemented in Task 2)

**Accelerometer geometry.** CLAUDE.md section 6.2 says the IMU is "mounted at or near the pivot axis." Modeling it exactly at the wheel axle gives a clean, *exact* (not approximate) specific-force formula: the axle's height above the ground is fixed at `R` regardless of body lean, so it has **zero vertical acceleration**, and its horizontal position is exactly `R*theta` (rolling without slip), so its horizontal acceleration is exactly `R*theta_ddot` — no dependence on `L` or any lean-induced offset. The true specific force in the world frame is therefore `(R*theta_ddot, g)` (the `g` is the accelerometer's reaction-force reading at rest, standard convention). Rotating into the body frame by the pitch `psi`:
```
a_x = R*theta_ddot*cos(psi) + g*sin(psi)
a_z = -R*theta_ddot*sin(psi) + g*cos(psi)
psi_acc = atan2(a_x, a_z) + noise
```
Check: at `theta_ddot=0`, this gives `atan2(g*sin(psi), g*cos(psi)) = psi` exactly — matches the "reads pure tilt when stationary" requirement. The `theta_ddot` term is exactly the "includes the body's linear acceleration, so it is corrupted during motion" behavior CLAUDE.md section 7 describes.

**Encoder quantization order.** CLAUDE.md: "θ_enc = θ − ψ (per wheel, then averaged), quantized to 2π/CPR." Real encoders physically quantize before any averaging happens (you can't un-quantize hardware tick counts), so this plan quantizes each wheel's `(theta_l - psi)` / `(theta_r - psi)` independently to the tick resolution, *then* averages the two quantized readings — not the other, mathematically-simpler reading where you'd average first and quantize once (which would make the whole computation collapse to `quantize(theta - psi)` with no dependence on yaw at all, since `theta_l`/`theta_r`'s yaw offsets cancel exactly under averaging *before* quantization). The chosen (per-wheel-quantize-then-average) reading is more physically faithful to real hardware and is what's implemented. If you disagree with this reading of the spec, flag it — this doesn't change the KF's C matrix or Q/R tuning either way since with no yaw excited (`phi=0` throughout the NIS test), both readings agree exactly.

**Gyro bias.** A random walk: `b_g[k+1] = b_g[k] + N(0, (gyro_bias_walk * sqrt(dt))^2)`, where `gyro_bias_walk = 1e-4 rad/s/sqrt(s)` (CLAUDE.md section 7's diffusion-coefficient-style units convert to a per-step increment std this way). The measurement at step `k` uses the *current* (pre-increment) bias; the returned, incremented bias feeds the next call — the same explicit-state-threading pattern `sim/integrate.py::rk4_step` already uses for the plant state.

### Kalman filter design (implemented in Task 3)

5-state `[theta, psi, theta_dot, psi_dot, b_g]`, ZOH-discretized at `dt=0.005` from the *planar* linearization (`sim/linearize.py::linearize`, reduced to `[theta,psi,theta_dot,psi_dot]` the same way `linearize_planar` does — `b_g` has no deterministic dynamics of its own, `d(b_g)/dt=0`, so its row/column of `A` is zero; only its process noise `Q_bg = gyro_bias_walk^2 * dt` is nonzero). Discretization uses the block-matrix ("Van Loan") method via `scipy.linalg.expm`, which handles the singular augmented `A` correctly (same reasoning `linearize.py` already documents about `A(0)` being evaluated once, just one level up: here it's ZOH discretization of a matrix with a zero eigenvalue, not linearization, but the numerical technique needed — the block-exponential trick — is standard exactly because plain `Ad = expm(A*dt)`, `Bd = A^-1(Ad-I)B` fails when `A` is singular).

`C` (3×5, `y = [theta_enc, psi_dot_gyro, psi_acc]`):
```
theta_enc   ~= theta - psi           -> row [ 1, -1,  0, 0, 0]
psi_dot_gyro ~= psi_dot + b_g        -> row [ 0,  0,  0, 1, 1]
psi_acc     ~= psi                    -> row [ 0,  1,  0, 0, 0]
```
Note `psi_acc`'s row has **no** `theta_ddot` term — the nominal model the KF runs on doesn't know about the accelerometer's motion-corruption term. That mismatch (present even in this "nominal" test, since the LQR is always applying *some* small corrective voltage) is exactly what the innovation is measuring, per CLAUDE.md section 8: "The estimator always uses the nominal model. The mismatch is what the innovation is meant to detect."

`R` (3×3 diagonal): encoder quantization modeled as uniform-noise variance `(2*pi/CPR)^2/12`, plus `sigma_gyro^2`, `sigma_accel^2` directly from CLAUDE.md section 7 (both are already specified "per sample," i.e. already the correct discrete-time measurement-noise std — no `dt` scaling needed, unlike the bias walk).

**Q and initial P (the actual tuning).** This is genuinely empirical — CLAUDE.md gives no target values here (unlike the LQR sanity-check example in step 2). The final, verified defaults:
```
Q_theta = Q_psi = 1e-8
Q_theta_dot = Q_psi_dot = 1e-6
P0_theta = P0_psi = P0_theta_dot = P0_psi_dot = 1e-4
P0_bg = 1e-6
```
Tuning notes, so you understand *why* these specific numbers and don't "fix" a future failure by fiddling them blindly:
- **The initial covariance matters more than you'd think.** An early attempt used a much larger, "safely uncertain" initial `P` (`diag(1e-2,1e-2,1e-2,1e-2,1e-4)`). This caused a huge, short-lived transient in the first ~10 control steps (~50ms): the filter's early overreaction to sensor noise fed large voltage commands into the plant, producing large `theta_ddot`, which corrupted the accelerometer reading (see the geometry note above), which produced NIS values of 200-600 in that transient window alone — even though the steady-state (post-transient) behavior was fine. This is a textbook KF startup-transient artifact, not a tuning bug in Q/R. The fix is a smaller, more realistic initial `P` (we *do* know the robot starts upright at rest — `P0=1e-4`-ish, not `1e-2`), which eliminates the transient (`max NIS` over the full 60s run drops from ~600 to ~20-25, consistent with `chi2(3)`'s own tail).
- **Larger Q makes the mean NIS *lower*, not higher** (counter to a naive "more process noise = more uncertainty = higher NIS" intuition) — because in this specific *nominal, no real model mismatch* scenario, extra `Q` just inflates the filter's own predicted uncertainty `S` beyond what's actually needed, so the same-size innovations produce smaller `ν'S⁻¹ν`. The needed `Q` here is small — just enough to keep the filter numerically well-behaved and to leave headroom for real disturbances later (`sim/disturbances.py`, step 4) — not a large "trust the model less" buffer.

### Design decision: the NIS test's acceptance interval is chi2(3)'s own 95% interval, not the tighter "mean of 12000 i.i.d. samples" interval

CLAUDE.md section 11: *"mean NIS within the 95% interval for χ²(3) averaged over the run; ≈5% of samples above the χ²(3) 95% quantile."* The literal-statistics reading of "95% interval for [the mean of N] χ²(3) [samples]" would be: since a sum of N i.i.d. `chi2(3)` variables is `chi2(3N)`, the sample mean's 95% interval is `[chi2(3N).ppf(0.025)/N, chi2(3N).ppf(0.975)/N]`. For a 60s run at 200Hz, `N=12000`, and this works out to **`[2.956, 3.044]`** — a razor-thin ±1.5% band.

I tested this directly: running the *same* closed-loop simulation with **zero** added process noise (`Q_theta=Q_psi=Q_theta_dot=Q_psi_dot=0`, i.e. the theoretically "most correct" setting for a truly nominal run) across 4 different seeds gave sample means of **2.875, 2.831, 2.671, 3.066** — a spread of 0.4, roughly **10x wider** than the ±0.044 the i.i.d. formula predicts. This confirms the i.i.d. assumption is wrong for this system: successive NIS samples are correlated (through the KF's own smoothing and the near-deterministic encoder-quantization pattern), so the textbook "variance of the mean shrinks as 1/N" formula doesn't apply, and no amount of additional Q/R tuning will reliably hit that tight interval — it's not an achievable target, it's a mismatched statistical model.

**Decision:** implement the test using `chi2(3)`'s own 95% interval (`[chi2(3).ppf(0.025), chi2(3).ppf(0.975)] ≈ [0.216, 9.348]`) as the acceptance band for the run's mean NIS, and `[0.01, 0.10]` (a symmetric-ish practical band around the literal "≈5%") for the fraction exceeding the 95% quantile. This is still a real, discriminating test — it correctly rejected every badly-tuned candidate found during exploration (e.g. mean NIS of 940 from one bad Q choice, or the ~600-peak startup-transient bug above), while robustly passing the well-tuned design across many seeds (verified seeds `{0, 1, 7, 42, 123}` all pass with means in `[2.81, 2.95]` and fractions in `[0.037, 0.043]`). If you think the tighter interval is actually what's intended and want to pursue it, that's a much larger undertaking (likely needs averaging over many independent 60s Monte Carlo runs rather than one, to legitimately shrink the interval) — flag it rather than silently picking a different bound.

---

## Task 1: `sim/params.py` — sensor and estimator parameters

**Files:**
- Modify: `sim/params.py` (append; do not change any existing class/function)
- Test: `tests/test_params.py` (append)

- [ ] **Step 1: Write the failing test**

Append to `tests/test_params.py`:

```python
from sim.params import (
    EstimatorParams,
    SensorParams,
    default_estimator_params,
    default_sensor_params,
)


def test_default_sensor_params():
    sp = default_sensor_params()

    assert sp.sigma_gyro == 0.005
    assert sp.gyro_bias_walk == 1e-4
    assert sp.sigma_accel == 0.02
    assert sp.encoder_cpr == 360.0
    assert sp.ultrasonic_sigma == 0.003
    assert sp.ultrasonic_range_min == 0.02
    assert sp.ultrasonic_range_max == 4.0
    assert sp.ultrasonic_dropout_prob == 0.02
    assert sp.ultrasonic_rate_hz == 20.0


def test_sensor_params_is_frozen():
    sp = default_sensor_params()
    try:
        sp.sigma_gyro = 1.0
        assert False, "SensorParams should be frozen"
    except AttributeError:
        pass


def test_default_estimator_params():
    ep = default_estimator_params()

    assert ep.Q_theta == 1e-8
    assert ep.Q_psi == 1e-8
    assert ep.Q_theta_dot == 1e-6
    assert ep.Q_psi_dot == 1e-6
    assert ep.P0_theta == 1e-4
    assert ep.P0_psi == 1e-4
    assert ep.P0_theta_dot == 1e-4
    assert ep.P0_psi_dot == 1e-4
    assert ep.P0_bg == 1e-6


def test_estimator_params_is_frozen():
    ep = default_estimator_params()
    try:
        ep.Q_theta = 1.0
        assert False, "EstimatorParams should be frozen"
    except AttributeError:
        pass
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_params.py -v`
Expected: FAIL with `ImportError: cannot import name 'SensorParams' from 'sim.params'`

- [ ] **Step 3: Append to `sim/params.py`**

```python
@dataclass(frozen=True)
class SensorParams:
    """Sensor noise model (CLAUDE.md section 7). All values are starting
    points, not measured/calibrated hardware specs.

    ultrasonic_* fields are not yet consumed by any sensor function -- the
    ultrasonic model needs 2D corridor ray-casting (sim/world.py, build
    order step 5) -- but CLAUDE.md section 7 says all sigma values belong
    here regardless, so they're recorded now.
    """

    sigma_gyro: float               # rad/s, gyro white noise std per sample
    gyro_bias_walk: float           # rad/s/sqrt(s), gyro bias random-walk rate
    sigma_accel: float              # rad, accelerometer-derived tilt noise std
    encoder_cpr: float               # counts/rev, encoder resolution
    ultrasonic_sigma: float          # m, ultrasonic range noise std
    ultrasonic_range_min: float      # m
    ultrasonic_range_max: float      # m
    ultrasonic_dropout_prob: float   # probability a reading returns max range
    ultrasonic_rate_hz: float        # Hz, ultrasonic sample rate


def default_sensor_params() -> SensorParams:
    """CLAUDE.md section 7 starting-point values."""
    return SensorParams(
        sigma_gyro=0.005,
        gyro_bias_walk=1e-4,
        sigma_accel=0.02,
        encoder_cpr=360.0,
        ultrasonic_sigma=0.003,
        ultrasonic_range_min=0.02,
        ultrasonic_range_max=4.0,
        ultrasonic_dropout_prob=0.02,
        ultrasonic_rate_hz=20.0,
    )


@dataclass(frozen=True)
class EstimatorParams:
    """Balance-KF tuning for the 5-state model
    [theta, psi, theta_dot, psi_dot, b_g]: process noise on the four planar
    states, and initial covariance. The gyro-bias process noise is derived
    from SensorParams.gyro_bias_walk and the control-loop dt, not tuned
    here (see sim/estimator.py::process_noise).

    These defaults were found empirically against the section 11 NIS
    consistency test -- see docs/superpowers/plans/2026-09-20-sensors-
    estimator-nis-step3.md for the tuning process and why these values
    (not some other combination) were chosen.
    """

    Q_theta: float
    Q_psi: float
    Q_theta_dot: float
    Q_psi_dot: float
    P0_theta: float
    P0_psi: float
    P0_theta_dot: float
    P0_psi_dot: float
    P0_bg: float


def default_estimator_params() -> EstimatorParams:
    return EstimatorParams(
        Q_theta=1e-8,
        Q_psi=1e-8,
        Q_theta_dot=1e-6,
        Q_psi_dot=1e-6,
        P0_theta=1e-4,
        P0_psi=1e-4,
        P0_theta_dot=1e-4,
        P0_psi_dot=1e-4,
        P0_bg=1e-6,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_params.py -v`
Expected: PASS (8 tests: 4 pre-existing + 4 new)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 16 pre-existing tests + 4 new = 20), no regressions

- [ ] **Step 6: Commit**

```bash
git add sim/params.py tests/test_params.py
git commit -m "feat: add SensorParams and EstimatorParams (CLAUDE.md sec 7)"
```

---

## Task 2: `sim/sensors.py` — gyro, accelerometer, encoder

**Files:**
- Create: `sim/sensors.py`
- Test: `tests/test_sensors.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_sensors.py
import numpy as np

from sim.params import default_plant_params, default_sensor_params
from sim.sensors import accelerometer, encoder, gyro


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

    assert abs(accelerating - at_rest) > 0.05
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_sensors.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.sensors'`

- [ ] **Step 3: Write `sim/sensors.py`**

```python
"""Sensor models (CLAUDE.md section 7).

Gyro, accelerometer, and encoder -- the three sensors the balance Kalman
filter (sim/estimator.py) consumes. The ultrasonic model needs 2D corridor
ray-casting and belongs in sim/world.py (build order step 5); it isn't part
of the KF's measurement vector (CLAUDE.md section 9), so its absence here
doesn't block the estimator.
"""

import numpy as np

from sim.params import PlantParams, SensorParams


def gyro(
    psi_dot_true: float,
    b_g: float,
    dt: float,
    sensor_p: SensorParams,
    rng: np.random.Generator,
) -> tuple[float, float]:
    """Gyro measurement of psi_dot, plus the bias's random-walk update.

    b_g is the bias in effect for *this* measurement; the returned
    (incremented) bias is what the next call should pass in -- the same
    explicit-state-threading pattern sim.integrate.rk4_step uses.
    """
    psi_dot_meas = psi_dot_true + b_g + rng.normal(0.0, sensor_p.sigma_gyro)
    b_g_next = b_g + rng.normal(0.0, sensor_p.gyro_bias_walk * np.sqrt(dt))
    return psi_dot_meas, b_g_next


def accelerometer(
    psi: float,
    theta_ddot: float,
    p: PlantParams,
    sensor_p: SensorParams,
    rng: np.random.Generator,
) -> float:
    """Accelerometer-derived tilt angle psi_acc = atan2(a_x, a_z).

    Models the IMU at the wheel axle (CLAUDE.md section 6.2: "mounted at or
    near the pivot axis"): the axle's height above the ground is fixed at
    R regardless of body lean (zero vertical acceleration), and its
    horizontal position is exactly R*theta (rolling without slip), so its
    true specific force in the world frame is exactly (R*theta_ddot, g).
    Rotating into the body frame:
    """
    a_x = p.R * theta_ddot * np.cos(psi) + p.g * np.sin(psi)
    a_z = -p.R * theta_ddot * np.sin(psi) + p.g * np.cos(psi)
    return np.arctan2(a_x, a_z) + rng.normal(0.0, sensor_p.sigma_accel)


def encoder(
    theta: float,
    psi: float,
    phi: float,
    p: PlantParams,
    sensor_p: SensorParams,
) -> float:
    """Averaged, quantized wheel-angle-relative-to-body encoder reading.

    Each wheel's raw angle relative to the body (theta_l - psi, theta_r -
    psi) is quantized to the encoder's tick resolution independently (real
    encoders quantize in hardware before any averaging can happen), then
    the two quantized readings are averaged. No rng: quantization is
    deterministic given the true state.
    """
    half_track_over_R = p.W / (2 * p.R)
    theta_l = theta - half_track_over_R * phi
    theta_r = theta + half_track_over_R * phi

    step = 2 * np.pi / sensor_p.encoder_cpr
    q_l = np.round((theta_l - psi) / step) * step
    q_r = np.round((theta_r - psi) / step) * step
    return (q_l + q_r) / 2.0
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_sensors.py -v`
Expected: PASS (5 tests)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 25 tests: 20 pre-existing + 5 new)

- [ ] **Step 6: Commit**

```bash
git add sim/sensors.py tests/test_sensors.py
git commit -m "feat: add gyro, accelerometer, encoder sensor models"
```

---

## Task 3: `sim/estimator.py` — Kalman filter core

**Files:**
- Create: `sim/estimator.py`
- Test: `tests/test_estimator.py` (this task adds basic unit tests; Task 4 appends the section 11 NIS test to the same file)

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_estimator.py
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_estimator.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.estimator'`

- [ ] **Step 3: Write `sim/estimator.py`**

```python
"""Discrete linear Kalman filter + NIS for the balance estimator
(CLAUDE.md section 9).

States (5): [theta, psi, theta_dot, psi_dot, b_g] (b_g = gyro bias).
Measurements (3): y = [theta_enc, psi_dot_gyro, psi_acc].

The filter always runs on the *nominal* linear model (CLAUDE.md section 8):
any mismatch between this and the true (possibly nonlinear or disturbed)
plant is what the innovation is meant to detect.
"""

from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm

from sim.linearize import linearize
from sim.params import EstimatorParams, PlantParams, SensorParams


def discretize(p: PlantParams, dt: float) -> tuple[np.ndarray, np.ndarray]:
    """ZOH-discretize the 5-state augmented model
    [theta, psi, theta_dot, psi_dot, b_g], u = v_l + v_r, at rate dt.

    b_g has no deterministic dynamics of its own (d(b_g)/dt = 0); its
    random-walk behavior is process noise, added via Q in predict(), not
    part of A. Uses the block-matrix ("Van Loan") method, which works even
    though the augmented A is singular (a b_g row of zeros, and a theta row
    with no restoring term) -- the same reason a plain
    Bd = A^-1(Ad - I)B formula would fail here.
    """
    A6, B6 = linearize(p)
    idx = [0, 1, 3, 4]  # theta, psi, theta_dot, psi_dot
    A_planar = A6[np.ix_(idx, idx)]
    B_planar = B6[idx, 0]

    A_aug = np.zeros((5, 5))
    A_aug[:4, :4] = A_planar
    B_aug = np.zeros(5)
    B_aug[:4] = B_planar

    M = np.zeros((6, 6))
    M[:5, :5] = A_aug
    M[:5, 5] = B_aug
    Md = expm(M * dt)
    return Md[:5, :5], Md[:5, 5]


def measurement_matrix() -> np.ndarray:
    """C for y = [theta_enc, psi_dot_gyro, psi_acc] from
    x = [theta, psi, theta_dot, psi_dot, b_g].

    theta_enc ~= theta - psi; psi_dot_gyro ~= psi_dot + b_g; psi_acc ~= psi
    (the nominal model has no theta_ddot term -- that corruption during
    motion is exactly the mismatch NIS is meant to catch).
    """
    return np.array([
        [1.0, -1.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0, 1.0],
        [0.0, 1.0, 0.0, 0.0, 0.0],
    ])


def process_noise(sensor_p: SensorParams, est_p: EstimatorParams, dt: float) -> np.ndarray:
    """Q (5x5 diagonal). The gyro-bias term is derived directly from the
    sensor's bias-walk rate; the other four come from EstimatorParams
    (empirically tuned -- see the plan doc for this build step)."""
    Q_bg = sensor_p.gyro_bias_walk**2 * dt
    return np.diag([est_p.Q_theta, est_p.Q_psi, est_p.Q_theta_dot, est_p.Q_psi_dot, Q_bg])


def measurement_noise(sensor_p: SensorParams) -> np.ndarray:
    """R (3x3 diagonal): encoder quantization modeled as uniform noise
    (variance = step^2/12), plus the gyro and accelerometer noise stds."""
    step = 2 * np.pi / sensor_p.encoder_cpr
    encoder_var = step**2 / 12.0
    return np.diag([encoder_var, sensor_p.sigma_gyro**2, sensor_p.sigma_accel**2])


def initial_covariance(est_p: EstimatorParams) -> np.ndarray:
    return np.diag([
        est_p.P0_theta, est_p.P0_psi, est_p.P0_theta_dot, est_p.P0_psi_dot, est_p.P0_bg,
    ])


@dataclass(frozen=True)
class KalmanState:
    x_hat: np.ndarray  # shape (5,)
    P: np.ndarray      # shape (5, 5)


def predict(state: KalmanState, Ad: np.ndarray, Bd: np.ndarray, u: float, Q: np.ndarray) -> KalmanState:
    x_pred = Ad @ state.x_hat + Bd * u
    P_pred = Ad @ state.P @ Ad.T + Q
    return KalmanState(x_pred, 0.5 * (P_pred + P_pred.T))


def update(
    state: KalmanState, y: np.ndarray, C: np.ndarray, R: np.ndarray
) -> tuple[KalmanState, float]:
    """Measurement update. Returns (updated state, NIS)."""
    innovation = y - C @ state.x_hat
    S = C @ state.P @ C.T + R
    K = state.P @ C.T @ np.linalg.inv(S)
    x_upd = state.x_hat + K @ innovation
    I_KC = np.eye(state.x_hat.shape[0]) - K @ C
    P_upd = I_KC @ state.P @ I_KC.T + K @ R @ K.T  # Joseph form (numerically stable)
    nis = innovation @ np.linalg.solve(S, innovation)
    return KalmanState(x_upd, P_upd), nis
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_estimator.py -v`
Expected: PASS (5 tests)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 30 tests: 25 pre-existing + 5 new)

- [ ] **Step 6: Commit**

```bash
git add sim/estimator.py tests/test_estimator.py
git commit -m "feat: add discrete Kalman filter (predict/update) and NIS"
```

---

## Task 4: Section 11 NIS consistency test (nominal 60s closed-loop run)

**Files:**
- Modify: `tests/test_estimator.py` (append to the file created in Task 3)

CLAUDE.md section 11: *"KF, nominal plant, no disturbances, 60 s: mean NIS within the 95 % interval for χ²(3) averaged over the run; ≈ 5 % of samples above the χ²(3) 95 % quantile."*

This test wires together `sim.plant.f` (nonlinear plant), `sim.integrate.rk4_step` (1ms RK4 substeps), `sim.sensors` (gyro/accelerometer/encoder), `sim.estimator` (predict/update), and `sim.control.design_lqr_balance` (state-feedback control on the *estimated* state) into a 200Hz-control / 1kHz-plant closed loop, run for 60 simulated seconds with no injected disturbance. See the plan's "Pre-verified design" section above for why the acceptance bounds are `chi2(3)`'s own 95% interval rather than the much tighter (and, empirically, wrong-for-this-system) interval for the mean of 12000 i.i.d. samples.

**This test takes roughly 20-60 seconds of wall-clock time to run** (240,000 RK4 substeps of the nonlinear plant, in pure Python) — that's expected, not a hang.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_estimator.py`:

```python
import scipy.stats as stats

from sim.control import design_lqr_balance
from sim.integrate import rk4_step
from sim.plant import f as plant_f
from sim.sensors import accelerometer, encoder, gyro
from sim.params import default_lqr_balance_params


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
```

- [ ] **Step 2: Run test to verify it fails, then debug if needed**

Run: `uv run pytest tests/test_estimator.py::test_nis_consistent_on_nominal_60s_run -v -s`
Expected: since all the underlying pieces (`plant.py`, `integrate.py`, `sensors.py`, `estimator.py`, `control.py`) already exist and are individually tested, this should actually run (slowly — see the runtime note above) and PASS on the first try, since this exact code was verified against this exact repo before the plan was written. If it fails, do not adjust the seed or loosen the bounds — instead, print `np.mean(nis)` and `np.mean(nis > chi2_3.ppf(0.95))` and compare against the expected `[2.81, 2.95]` / `[0.037, 0.043]` ranges; a large discrepancy means a transcription bug (most likely: wrong argument order to `gyro`/`accelerometer`/`encoder`/`predict`/`update`, or `u_prev` vs. the newly-computed `u` swapped somewhere in the loop).

- [ ] **Step 3: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (31 tests: 30 pre-existing + this one). Total suite runtime will jump to roughly 20-60+ seconds because of this one test.

- [ ] **Step 4: Commit**

```bash
git add tests/test_estimator.py
git commit -m "test: add section 11 NIS consistency test on nominal 60s closed loop"
```

---

## Self-review notes

- **Spec coverage:** Section 14 step 3 ("sensors.py → estimator.py → NIS test") ✓. CLAUDE.md section 7's gyro/accelerometer/encoder models ✓ (Task 2); ultrasonic explicitly deferred with justification. Section 9's KF (5-state, 3-measurement, innovation/NIS) ✓ (Task 3). Section 11's NIS consistency test ✓ (Task 4), with the acceptance-interval ambiguity resolved and documented rather than silently picked. Section 3's "no magic numbers," "every stochastic function takes an explicit rng," "reproducible from (config, seed)" ✓ throughout (gyro/accelerometer take explicit `rng`; encoder is deterministic and correctly takes none; the NIS test is seeded).
- **Placeholder scan:** no TBD/TODO; every step has runnable code and exact expected output (with realistic runtime caveats for Task 4's slow test, not hidden).
- **Type consistency:** `gyro(psi_dot_true, b_g, dt, sensor_p, rng) -> (meas, b_g_next)`, `accelerometer(psi, theta_ddot, p, sensor_p, rng) -> float`, `encoder(theta, psi, phi, p, sensor_p) -> float` signatures are identical between Task 2's implementation and Task 4's closed-loop test usage. `KalmanState(x_hat, P)`, `predict(state, Ad, Bd, u, Q) -> KalmanState`, `update(state, y, C, R) -> (KalmanState, nis)` are used identically across Task 3's tests and Task 4. `SensorParams`/`EstimatorParams` field names match between Task 1's definitions and their use in Tasks 2-4.
- **Explicitly deferred (not this plan):** ultrasonic sensor model (needs `sim/world.py`, step 5), windowed NIS statistic ε_k and the gate state machine (`sim/gate.py`, step 4), disturbance injection (`sim/disturbances.py`, step 4) and the corresponding "gate switches under disturbance" test, the general-purpose multi-rate closed-loop simulator (`sim/run.py`, step 6 — Task 4's closed-loop harness is test-only, not a reusable module).
