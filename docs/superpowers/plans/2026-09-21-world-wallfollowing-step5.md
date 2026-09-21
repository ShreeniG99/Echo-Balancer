# Echo Balancer — Section 14 Step 5: World, Wall-Following Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `sim/world.py` (2D corridor geometry, ray casting, robot pose kinematics), finally wire up the ultrasonic sensor model in `sim/sensors.py` (deferred since step 3), and implement the CLAUDE.md section 9 wall-following stack in `sim/control.py` — speed servo, yaw control, wall-following outer loop, front-threshold slowdown — with a closed-loop demonstration that the robot follows a corridor wall without crashing.

**Architecture:** `sim/world.py` is pure geometry: `cast_ray` (ray-vs-corridor-wall intersection) and `pose_velocity` (unicycle kinematics, designed to plug into `sim/integrate.py::rk4_step` exactly like `sim/plant.py::f` does). `sim/sensors.py` gains `ultrasonic`, matching the existing gyro/accelerometer/encoder pattern. `sim/control.py` gains three independent, composable pieces: `design_lqr_speed_servo` (a new, separate 5-state integral-augmented LQR — does **not** modify the existing `design_lqr_balance`/`LQRBalanceParams` from step 2, which stays exactly as approved), `yaw_p_control` (proportional-only, not PD — see "Design decision" below), and `wall_following_control` / `front_threshold_speed_adjust`. A closed-loop test demonstrates the full stack together.

**Tech Stack:** Python ≥3.11, `numpy`, `scipy` (`scipy.linalg.solve_continuous_are`, already a dependency since step 2).

**Explicitly out of scope for this plan:** `sim/run.py`, `experiments/`, `analysis/`, `quantum/`, and everything past section 14 build-order step 5. Also explicitly deferred, with reasoning:
- **State estimation for wall-following.** The balance/speed-servo control law in this plan's closed-loop test uses the *true* plant state directly (bypassing the step-3 Kalman filter), and the yaw loop uses the *true* φ̇ directly (there is no yaw-rate sensor in `SensorParams` — the gyro model only measures ψ̇, matching a single-axis balance IMU convention established in step 3). This mirrors the precedent already set by step 2's LQR recovery test (`docs/superpowers/plans/2026-09-19-linearize-lqr-step2.md`, "the recovery test uses per-step feedback, not the 200Hz ZOH loop") — a documented, temporary simplification pending `sim/run.py`'s real multi-sensor-fusion architecture, not a design flaw to fix now. Only the wall-following distance signal goes through a real, noisy sensor model (`ultrasonic`), since that's the part CLAUDE.md section 9 actually specifies as sensor-driven.
- **Corridor corners/dead-ends.** The corridor is modeled as two infinite parallel walls (no turns, no end walls) — see "Design decision" below.
- **`sim/run.py`-style general closed-loop runner.** This plan's integration test is test-only, same caveat as every closed-loop harness since step 3.

---

## Pre-verified design: this step required designing two new control loops from scratch

Section 9's wall-following description ("side ultrasonic distance error → PD → yaw-rate reference") assumes forward speed control and yaw-rate control already exist. **Neither did** — both are explicitly listed in section 9 but were out of scope for steps 1-4's balance-only LQR. Building and verifying them turned out to be the bulk of this step's work. Everything below was run against this repo's actual `sim/plant.py`/`sim/linearize.py`/`sim/control.py` before writing this plan.

### Design decision: a real "speed servo" LQR is required — reference-tracking on the existing 4-state balance LQR is not accurate enough

CLAUDE.md section 9: *"LQR on the linearized planar model, states [θ, ψ, θ̇, ψ̇] **plus integral of (θ − θ_ref) for speed servo**."* First attempt: keep the existing (approved, unmodified) 4-state `design_lqr_balance` and just feed it a moving reference (`u = -K·[θ-θ_ref(t), ψ, θ̇-θ̇_ref, ψ̇]` with `θ_ref(t)` ramping at the desired speed). Result: **42% steady-state speed error** (commanding `θ̇_ref=1.0 rad/s` settles at `θ̇≈1.42 rad/s`; `θ̇_ref=5.0` settles at `≈7.10`) — a textbook type-0-system steady-state error, since pure state feedback has no integral action to drive a ramp-reference error to zero.

**Fix:** built a genuinely separate 5-state integral-augmented LQR, `design_lqr_speed_servo` — state `[θ-θ_ref, ψ, θ̇-θ̇_ref, ψ̇, z]` where `z=∫(θ-θ_ref)dt`, reusing `sim/linearize.py::linearize_planar`'s existing `A_planar`/`B_planar` with one appended integrator row (`dz/dt = θ-θ_ref`). **This is a new function, not a replacement** — `design_lqr_balance`/`LQRBalanceParams` (step 2, used by every closed-loop test since) are completely unmodified. Verified with `Q=diag(1, 1e3, 1, 1, 10)`, `R=1e2`:
```
K = [-0.90627756, -53.30474389, -2.31169767, -5.01088606, -0.31622777]
closed-loop eigenvalues: -241.825, -0.394±0.389j, -7.659, -6.344  (all stable)
```
Steady-state tracking error at `θ̇_ref∈{1,3,5}`: `0.006, 0.018, 0.030` — 20-70x better than without integral action. (Side note, not exploited by this plan but worth recording: the dominant closed-loop pole moves from `-0.097` (step 2, no integral) to `-0.394±0.389j` — this is the same slow pole that step 4's `DECISIONS.md`/`FAILED_APPROACHES.md` identified as the root cause of the gate's χ²(3N)-quantile problem. Adding this speed servo would likely help that too, but revisiting the gate's thresholds is out of scope here.)

### Design decision: yaw control is proportional-only, not PD — the plant's yaw dynamics are too fast for derivative action at 200Hz

CLAUDE.md section 9: *"Yaw: PD on φ̇ with differential voltage."* The plant's actual yaw dynamics (`sim/linearize.py::linearize`, row/col 5) are `φ̈ ≈ -95.57·φ̇ + 106.16·(v_r-v_l)`— an **open-loop pole at ≈-95.6 rad/s**, i.e. a ~10ms time constant, uncomfortably fast relative to the 200Hz (5ms) control rate.

Tested a discrete PD (`u = Kp·(φ̇_ref-φ̇) + Kd·d(error)/dt`, `dt=5ms`): **any nonzero `Kd` destabilizes the loop almost immediately** — `Kd=0.01` is borderline, `Kd=0.02` diverges to `~1e36` within 3 seconds, `Kd=0.05` diverges outright. This is a genuine discrete-time sampling problem (a large-gain derivative term amplifying a already-fast plant pole at a fixed sample rate), not a tuning imprecision — no amount of care with `Kd` near this system's actual dynamics avoids it. Proportional-only (`Kp` alone) is stable up to `Kp≈8-10` before its own instability onset; `Kp=3.0` gives a comfortable margin (less than half that) with `~62%` steady-state φ̇-tracking (a P-only first-order regulator's inherent nonzero steady-state error — expected, not a bug).

**Decision:** `yaw_p_control` is implemented as proportional-only; `YawControlParams` has no `Kd` field. The incomplete inner-loop tracking is compensated for by the outer wall-following loop, which reacts to the *actual* measured distance error regardless of why φ̇ undershot its reference — verified below. If a future need genuinely requires tighter yaw tracking, that requires either a faster control rate or restructuring the yaw loop (e.g. state feedback using more than just φ̇), not a `Kd` tweak on this design.

### Design decision: Task 6 integrates position via `pose_velocity`/`rk4_step` at plant resolution (1ms), not a hand-rolled formula at control resolution (5ms)

`sim/world.py::pose_velocity` is designed to plug into `sim.integrate.rk4_step` exactly like `sim/plant.py::f` does, holding its input `(θ̇, φ)` constant across one RK4 step — the same ZOH pattern `plant.f` uses for `(v_l, v_r)`. But unlike `v_l`/`v_r` (a true DAC-held control input), `φ` is a genuine plant *state*, continuously evolving. Freezing it is only a good approximation if the freeze window is short relative to how fast φ actually changes. Task 6 therefore calls `pose_velocity` via `rk4_step` **once per 1ms plant substep** (using the just-updated `x[2]` after each `plant.f` RK4 step), not once per 5ms control tick — a 5x tighter freeze window, bounding the approximation error to the same timescale the plant dynamics themselves are already being integrated at. (For a state-independent, input-frozen velocity field, RK4 collapses algebraically to a single Euler step — `pos + dt·vel` — so this is equivalent to plain Euler integration at 1kHz, not "real" RK4 accuracy; the value of calling `rk4_step` here is consistency with the rest of the codebase's integration convention, not extra numerical accuracy.) This also ensures `pose_velocity` is the function actually exercised by the closed-loop demo, rather than a parallel hand-rolled position update that would leave it untested by its only specified consumer.

### Design decision: the corridor is two infinite parallel walls, no corners or dead ends

Simplest geometry sufficient to test wall-following: walls at `y=0` and `y=width`, no bound on `x`. This keeps `cast_ray`'s geometry a one-line case split (no segment-endpoint intersection tests) and is consistent with CLAUDE.md's own build-order note ("if behind schedule, cut wall-following first") suggesting a minimal implementation is appropriate. One consequence: the front-ultrasonic "slow down" trigger (section 9) can only fire due to *heading drift toward a side wall*, not a genuine head-on obstacle, since there's nothing ahead to hit in this geometry. `front_threshold_speed_adjust` is implemented and unit-tested as a correct, general function regardless; a corridor with an actual dead-end (to exercise it meaningfully end-to-end) is a natural extension for `sim/world.py` later, not required by CLAUDE.md section 9's literal wording.

### Wall-following closed-loop verification (with real ultrasonic noise/dropout)

Full stack (speed servo + P-only yaw + wall-following outer loop, all together, `theta_dot_ref=2.0 rad/s`, `target_distance=0.5m` in a `1.0m`-wide corridor, robot starting `0.2m` off-target at `y0=0.3`) across 5 seeds, **all with real `ultrasonic()` noise and 2% dropout** (not the idealized true-distance signal): no crash (`y` stayed inside `(0, 1.0)` throughout) in any of the 5 runs; final `y` ranged `0.346-0.525` (target `0.5`); worst 3-second-window deviation from target across all 5 seeds was `0.154`. `Kp_wall=2.0, Kd_wall=0.5` chosen from a small sweep as the best-behaved of the candidates tried (tighter than `Kp_wall≤1.0`, which converged to `y≈0.61-0.63` — a large persistent bias — and no worse than `Kp_wall=2.0` alone). The checked-in test uses seed=0 specifically (`final_y=0.4792`, `max|y-0.5|` over the last 3s `=0.0208`, `min_front=0.377` — well clear of both walls).

---

## Task 1: `sim/params.py` — `CorridorParams`, `SpeedServoParams`, `YawControlParams`, `WallFollowParams`

**Files:**
- Modify: `sim/params.py` (append; do not change any existing class/function)
- Test: `tests/test_params.py` (append)

- [ ] **Step 1: Write the failing test**

Append to `tests/test_params.py`:

```python
from sim.params import (
    CorridorParams,
    SpeedServoParams,
    WallFollowParams,
    YawControlParams,
    default_corridor_params,
    default_speed_servo_params,
    default_wall_follow_params,
    default_yaw_control_params,
)


def test_default_corridor_params():
    cp = default_corridor_params()
    assert cp.width == 1.0


def test_corridor_params_is_frozen():
    cp = default_corridor_params()
    try:
        cp.width = 2.0
        assert False, "CorridorParams should be frozen"
    except AttributeError:
        pass


def test_default_speed_servo_params():
    sp = default_speed_servo_params()
    assert sp.Q_theta == 1.0
    assert sp.Q_psi == 1e3
    assert sp.Q_theta_dot == 1.0
    assert sp.Q_psi_dot == 1.0
    assert sp.Q_integral == 10.0
    assert sp.R == 1e2


def test_speed_servo_params_is_frozen():
    sp = default_speed_servo_params()
    try:
        sp.R = 1.0
        assert False, "SpeedServoParams should be frozen"
    except AttributeError:
        pass


def test_default_yaw_control_params():
    yp = default_yaw_control_params()
    assert yp.Kp == 3.0


def test_yaw_control_params_is_frozen():
    yp = default_yaw_control_params()
    try:
        yp.Kp = 1.0
        assert False, "YawControlParams should be frozen"
    except AttributeError:
        pass


def test_default_wall_follow_params():
    wp = default_wall_follow_params()
    assert wp.target_distance == 0.5
    assert wp.Kp == 2.0
    assert wp.Kd == 0.5
    assert wp.front_slow_threshold == 0.5
    assert wp.front_slow_factor == 0.3


def test_wall_follow_params_is_frozen():
    wp = default_wall_follow_params()
    try:
        wp.Kp = 1.0
        assert False, "WallFollowParams should be frozen"
    except AttributeError:
        pass
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_params.py -v`
Expected: FAIL with `ImportError: cannot import name 'CorridorParams' from 'sim.params'`

- [ ] **Step 3: Append to `sim/params.py`**

```python
@dataclass(frozen=True)
class CorridorParams:
    """2D corridor geometry (CLAUDE.md section 4/9). Two infinite parallel
    walls at y=0 and y=width -- no corners or dead ends, see
    docs/superpowers/plans/2026-09-21-world-wallfollowing-step5.md
    "Design decision: the corridor is two infinite parallel walls"."""

    width: float  # m, distance between the two walls


def default_corridor_params() -> CorridorParams:
    return CorridorParams(width=1.0)


@dataclass(frozen=True)
class SpeedServoParams:
    """5-state integral-augmented LQR weights for forward-speed tracking
    (CLAUDE.md section 9: "plus integral of (theta - theta_ref) for speed
    servo"). Completely separate from LQRBalanceParams (step 2) -- this
    does not replace the balance-only LQR, it's an additional, optional
    augmentation. See sim.control.design_lqr_speed_servo.
    """

    Q_theta: float
    Q_psi: float
    Q_theta_dot: float
    Q_psi_dot: float
    Q_integral: float  # weight on z = integral(theta - theta_ref)
    R: float


def default_speed_servo_params() -> SpeedServoParams:
    return SpeedServoParams(Q_theta=1.0, Q_psi=1e3, Q_theta_dot=1.0, Q_psi_dot=1.0, Q_integral=10.0, R=1e2)


@dataclass(frozen=True)
class YawControlParams:
    """Yaw-rate tracking gain (CLAUDE.md section 9: "PD on phi_dot").
    Proportional-only, not PD -- see docs/superpowers/plans/2026-09-21-
    world-wallfollowing-step5.md "Design decision: yaw control is
    proportional-only". Any nonzero derivative gain destabilizes this
    loop given the plant's fast yaw dynamics relative to the 200Hz control
    rate -- do not add a Kd field without re-reading that section first.
    """

    Kp: float


def default_yaw_control_params() -> YawControlParams:
    return YawControlParams(Kp=3.0)


@dataclass(frozen=True)
class WallFollowParams:
    """Wall-following outer loop and front-threshold slowdown (CLAUDE.md
    section 9). Kp/Kd act on (target_distance - side_ultrasonic_reading)
    to produce a phi_dot reference for YawControlParams' inner loop.
    """

    target_distance: float     # m, desired distance from the followed wall
    Kp: float                  # distance-error -> phi_dot_ref proportional gain
    Kd: float                  # distance-error -> phi_dot_ref derivative gain
    front_slow_threshold: float  # m, front ultrasonic reading below this triggers slowdown
    front_slow_factor: float     # multiplier applied to the forward speed reference when triggered


def default_wall_follow_params() -> WallFollowParams:
    return WallFollowParams(target_distance=0.5, Kp=2.0, Kd=0.5, front_slow_threshold=0.5, front_slow_factor=0.3)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_params.py -v`
Expected: PASS (21 tests: 13 pre-existing + 8 new)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 66 tests: 58 pre-existing + 8 new), no regressions

- [ ] **Step 6: Commit**

```bash
git add sim/params.py tests/test_params.py
git commit -m "feat: add CorridorParams, SpeedServoParams, YawControlParams, WallFollowParams"
```

---

## Task 2: `sim/world.py` — corridor geometry, ray casting, pose kinematics

**Files:**
- Create: `sim/world.py`
- Test: `tests/test_world.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_world.py
import numpy as np
import pytest

from sim.integrate import rk4_step
from sim.params import CorridorParams, default_plant_params
from sim.world import cast_ray, pose_velocity


def test_cast_ray_straight_up_and_down_from_center():
    cp = CorridorParams(width=1.0)

    assert cast_ray(0.0, 0.5, np.radians(90), cp, 0.02, 4.0) == pytest.approx(0.5)
    assert cast_ray(0.0, 0.5, np.radians(-90), cp, 0.02, 4.0) == pytest.approx(0.5)


def test_cast_ray_parallel_to_walls_returns_max_range():
    cp = CorridorParams(width=1.0)

    assert cast_ray(0.0, 0.5, np.radians(0), cp, 0.02, 4.0) == 4.0


def test_cast_ray_near_wall():
    cp = CorridorParams(width=1.0)

    assert cast_ray(0.0, 0.1, np.radians(90), cp, 0.02, 4.0) == pytest.approx(0.9)
    assert cast_ray(0.0, 0.1, np.radians(-90), cp, 0.02, 4.0) == pytest.approx(0.1)


def test_cast_ray_diagonal():
    cp = CorridorParams(width=1.0)

    assert cast_ray(0.0, 0.5, np.radians(135), cp, 0.02, 4.0) == pytest.approx(0.5 * np.sqrt(2))


def test_cast_ray_clips_to_range_bounds():
    cp = CorridorParams(width=10.0)

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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_world.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.world'`

- [ ] **Step 3: Write `sim/world.py`**

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_world.py -v`
Expected: PASS (7 tests)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 73 tests: 66 pre-existing + 7 new)

- [ ] **Step 6: Commit**

```bash
git add sim/world.py tests/test_world.py
git commit -m "feat: add corridor ray casting and pose kinematics"
```

---

## Task 3: `sim/sensors.py` — ultrasonic sensor model

**Files:**
- Modify: `sim/sensors.py` (append `ultrasonic`; do not change `gyro`/`accelerometer`/`encoder`)
- Test: `tests/test_sensors.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_sensors.py`:

```python
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
```

Also add `ultrasonic` to the existing `from sim.sensors import ...` line at the top of `tests/test_sensors.py`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_sensors.py -v`
Expected: FAIL with `ImportError: cannot import name 'ultrasonic' from 'sim.sensors'`

- [ ] **Step 3: Append to `sim/sensors.py`**

```python
def ultrasonic(true_distance: float, sensor_p: SensorParams, rng: np.random.Generator) -> float:
    """HC-SR04 model (CLAUDE.md section 7): 20Hz, range 0.02-4m, sigma~3mm,
    2% dropout probability (returns max range instead of a noisy reading).
    """
    if rng.random() < sensor_p.ultrasonic_dropout_prob:
        return sensor_p.ultrasonic_range_max
    measured = true_distance + rng.normal(0.0, sensor_p.ultrasonic_sigma)
    return float(np.clip(measured, sensor_p.ultrasonic_range_min, sensor_p.ultrasonic_range_max))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_sensors.py -v`
Expected: PASS (9 tests: 6 pre-existing + 3 new)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 76 tests: 73 pre-existing + 3 new)

- [ ] **Step 6: Commit**

```bash
git add sim/sensors.py tests/test_sensors.py
git commit -m "feat: add ultrasonic sensor model"
```

---

## Task 4: `sim/control.py` — speed servo LQR

**Files:**
- Modify: `sim/control.py` (append `design_lqr_speed_servo`; do not change `design_lqr_balance`)
- Test: `tests/test_control.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_control.py`:

```python
from sim.control import design_lqr_speed_servo
from sim.params import default_speed_servo_params


def test_speed_servo_gain_matches_reference():
    p = default_plant_params()
    sp = default_speed_servo_params()

    K = design_lqr_speed_servo(p, sp)

    # Verified against this repo's actual sim/linearize.py before this plan
    # was written -- see plan doc "Design decision: a real speed servo LQR
    # is required".
    target = np.array([-0.90627756, -53.30474389, -2.31169767, -5.01088606, -0.31622777])
    np.testing.assert_allclose(K, target, rtol=1e-4)


def test_speed_servo_closed_loop_poles_stable():
    p = default_plant_params()
    sp = default_speed_servo_params()
    K = design_lqr_speed_servo(p, sp)

    A_planar, B_planar = linearize_planar(p)
    A_aug = np.zeros((5, 5))
    A_aug[:4, :4] = A_planar
    A_aug[4, 0] = 1.0
    B_aug = np.zeros(5)
    B_aug[:4] = B_planar

    eigs = np.linalg.eigvals(A_aug - np.outer(B_aug, K))
    assert np.all(eigs.real < 0)


def test_speed_servo_tracks_constant_forward_speed_in_nonlinear_sim():
    """Verified pre-plan: steady-state theta_dot tracking error was
    0.006/0.018/0.030 at theta_dot_ref=1/3/5 rad/s over 15s runs -- a
    generous margin (0.1) is used here, well above any of those, to avoid
    a flaky bound while still catching a real transcription bug (e.g. the
    42% error the non-integral design produced, which this margin would
    clearly reject)."""
    p = default_plant_params()
    sp = default_speed_servo_params()
    K = design_lqr_speed_servo(p, sp)

    dt_plant, dt_control, n_sub = 0.001, 0.005, 5
    theta_dot_ref = 3.0

    def xdotf(x, uu):
        return plant_f(x, uu[0], uu[1], p)

    x = np.zeros(6)
    theta_ref = 0.0
    z = 0.0
    n_steps = int(15.0 / dt_control)
    theta_dot_hist = []
    for _ in range(n_steps):
        theta_ref += theta_dot_ref * dt_control
        err = np.array([x[0] - theta_ref, x[1], x[3] - theta_dot_ref, x[4], z])
        u = -K @ err
        z += (x[0] - theta_ref) * dt_control
        theta_dot_hist.append(x[3])
        for _ in range(n_sub):
            x = rk4_step(xdotf, x, (u / 2, u / 2), dt_plant)

    steady_theta_dot = np.mean(theta_dot_hist[-200:])
    assert abs(steady_theta_dot - theta_dot_ref) < 0.1
```

Also add these imports to the top of `tests/test_control.py` if not already present: `from sim.integrate import rk4_step`, `from sim.linearize import linearize_planar`, `from sim.plant import f as plant_f`, `import numpy as np`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_control.py -v`
Expected: FAIL with `ImportError: cannot import name 'design_lqr_speed_servo' from 'sim.control'`

- [ ] **Step 3: Append to `sim/control.py`**

```python
def design_lqr_speed_servo(p: PlantParams, speed_servo_p: SpeedServoParams) -> np.ndarray:
    """5-state integral-augmented LQR gain for forward-speed tracking
    (CLAUDE.md section 9: "plus integral of (theta - theta_ref) for speed
    servo"). State [theta-theta_ref, psi, theta_dot-theta_dot_ref, psi_dot,
    integral(theta-theta_ref)], u = v_l + v_r.

    Completely separate from design_lqr_balance -- does not modify or
    replace it. Callers apply K to the *error* state at runtime (theta_ref,
    theta_dot_ref, and the running integral are the caller's
    responsibility, not this function's), since the closed-loop error
    dynamics under a constant-rate reference match this nominal
    (theta_ref=0) design exactly -- see the plan doc.
    """
    A_planar, B_planar = linearize_planar(p)
    A_aug = np.zeros((5, 5))
    A_aug[:4, :4] = A_planar
    A_aug[4, 0] = 1.0  # d(integral)/dt = theta - theta_ref
    B_aug = np.zeros(5)
    B_aug[:4] = B_planar

    Q = np.diag([
        speed_servo_p.Q_theta, speed_servo_p.Q_psi, speed_servo_p.Q_theta_dot,
        speed_servo_p.Q_psi_dot, speed_servo_p.Q_integral,
    ])
    R = np.array([[speed_servo_p.R]])

    B_col = B_aug.reshape(-1, 1)
    P = solve_continuous_are(A_aug, B_col, Q, R)
    K = np.linalg.inv(R) @ B_col.T @ P
    return K.flatten()
```

Add `SpeedServoParams` to the existing `from sim.params import ...` line at the top of `sim/control.py`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_control.py -v`
Expected: PASS (10 tests: 7 pre-existing + 3 new)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 79 tests: 76 pre-existing + 3 new)

- [ ] **Step 6: Commit**

```bash
git add sim/control.py tests/test_control.py
git commit -m "feat: add integral-augmented speed-servo LQR"
```

---

## Task 5: `sim/control.py` — yaw control, wall-following, front-threshold slowdown

**Files:**
- Modify: `sim/control.py` (append; do not change `design_lqr_balance`, `design_lqr_speed_servo`)
- Test: `tests/test_control.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_control.py`:

```python
from sim.control import front_threshold_speed_adjust, wall_following_control, yaw_p_control
from sim.params import YawControlParams, default_wall_follow_params


def test_yaw_p_control_formula():
    yaw_p = YawControlParams(Kp=3.0)

    assert yaw_p_control(1.0, 0.4, yaw_p) == pytest.approx(3.0 * (1.0 - 0.4))
    assert yaw_p_control(0.0, 0.0, yaw_p) == 0.0


def test_wall_following_control_formula_and_state_threading():
    wall_p = default_wall_follow_params()

    phi_dot_ref, new_prev_error = wall_following_control(0.2, 0.1, 0.05, wall_p)

    expected = wall_p.Kp * 0.2 + wall_p.Kd * (0.2 - 0.1) / 0.05
    assert phi_dot_ref == pytest.approx(expected)
    assert new_prev_error == 0.2


def test_front_threshold_speed_adjust_slows_down_below_threshold():
    wall_p = default_wall_follow_params()

    slowed = front_threshold_speed_adjust(0.3, 2.0, wall_p)
    unaffected = front_threshold_speed_adjust(1.0, 2.0, wall_p)

    assert slowed == pytest.approx(2.0 * wall_p.front_slow_factor)
    assert unaffected == 2.0
```

Also add `import pytest` to the top of `tests/test_control.py` if not already present.

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_control.py -v`
Expected: FAIL with `ImportError: cannot import name 'yaw_p_control' from 'sim.control'`

- [ ] **Step 3: Append to `sim/control.py`**

```python
def yaw_p_control(phi_dot_ref: float, phi_dot: float, yaw_p: YawControlParams) -> float:
    """CLAUDE.md section 9: "Yaw: PD on phi_dot with differential
    voltage." Proportional-only in practice -- see YawControlParams'
    docstring and the plan doc's "Design decision: yaw control is
    proportional-only" for why. Returns the differential voltage
    (v_r - v_l).
    """
    return yaw_p.Kp * (phi_dot_ref - phi_dot)


def wall_following_control(
    distance_error: float, prev_distance_error: float, dt: float, wall_p: WallFollowParams
) -> tuple[float, float]:
    """CLAUDE.md section 9: "side ultrasonic distance error -> PD ->
    yaw-rate reference." distance_error = target_distance - side_reading
    (positive when too close to the wall). Returns
    (phi_dot_ref, new_prev_distance_error) for the caller to thread into
    the next call.
    """
    d_error = (distance_error - prev_distance_error) / dt
    phi_dot_ref = wall_p.Kp * distance_error + wall_p.Kd * d_error
    return phi_dot_ref, distance_error


def front_threshold_speed_adjust(front_distance: float, nominal_theta_dot_ref: float, wall_p: WallFollowParams) -> float:
    """CLAUDE.md section 9: "Front ultrasonic below threshold -> slow
    down." """
    if front_distance < wall_p.front_slow_threshold:
        return nominal_theta_dot_ref * wall_p.front_slow_factor
    return nominal_theta_dot_ref
```

Add `WallFollowParams`, `YawControlParams` to the existing `from sim.params import ...` line at the top of `sim/control.py`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_control.py -v`
Expected: PASS (13 tests: 10 pre-existing + 3 new)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 82 tests: 79 pre-existing + 3 new)

- [ ] **Step 6: Commit**

```bash
git add sim/control.py tests/test_control.py
git commit -m "feat: add yaw P control, wall-following outer loop, front-threshold slowdown"
```

---

## Task 6: Closed-loop wall-following demonstration

**Files:**
- Create: `tests/test_wall_following.py`

Demonstrates the full stack together: nonlinear plant, RK4 integration, true-state balance/speed-servo feedback, true-φ̇ yaw P control, noisy `ultrasonic` readings driving the wall-following outer loop, corridor ray casting, and pose integration — all wired into one closed loop. Per the plan's "Explicitly out of scope" section, this intentionally bypasses the Kalman filter for balance/speed state (uses true state directly) and has no yaw-rate sensor (uses true φ̇ directly); only the wall-following distance signal goes through a real, noisy sensor.

**This test takes several seconds of wall-clock time** (15 simulated seconds at 1kHz plant / 200Hz control / 20Hz ultrasonic).

- [ ] **Step 1: Write the failing test**

```python
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
```

- [ ] **Step 2: Run test to verify it passes**

Run: `uv run pytest tests/test_wall_following.py -v`
Expected: PASS (a few seconds wall-clock). If it fails, do not widen the `0.3` margins — print `y_hist[-1]` and `np.max(np.abs(y_hist[-600:] - wall_p.target_distance))` and compare against the pre-verified `0.4792`/`0.0208` figures; a large discrepancy means a transcription bug in the closed-loop wiring (check argument order into `wall_following_control`/`yaw_p_control`/`front_threshold_speed_adjust`/`cast_ray`/`ultrasonic`, or the pose-update line), not a tuning problem.

- [ ] **Step 3: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (83 tests: 82 pre-existing + 1 new)

- [ ] **Step 4: Commit**

```bash
git add tests/test_wall_following.py
git commit -m "test: add closed-loop wall-following demonstration"
```

---

## Self-review notes

- **Spec coverage:** Section 14 step 5 ("world.py → wall-following") ✓. Section 4's `sim/world.py` (2D corridor geometry + ray casting) ✓ (Task 2). Section 7's ultrasonic model (HC-SR04, 20Hz, range 0.02-4m, σ≈3mm, 2% dropout) ✓ (Task 3) — closes a gap left open since step 3. Section 9's full wall-following description: speed servo ✓ (Task 4), yaw PD (implemented as P, with reasoning) ✓ (Task 5), wall-following outer loop ✓ (Task 5), front-threshold slowdown ✓ (Task 5), demonstrated together ✓ (Task 6).
- **Placeholder scan:** no TBD/TODO; every step has runnable code and exact expected output, with realistic runtime caveats stated plainly.
- **Type consistency:** `cast_ray(x, y, ray_heading, corridor_p, range_min, range_max)` and `pose_velocity(pos, u, p)` signatures match between Task 2's definition, its own tests, and Task 6's usage. `design_lqr_speed_servo(p, speed_servo_p) -> K` (shape `(5,)`) matches between Task 4's definition, tests, and Task 6. `yaw_p_control`, `wall_following_control`, `front_threshold_speed_adjust` signatures match between Task 5's definitions, tests, and Task 6. `CorridorParams`/`SpeedServoParams`/`YawControlParams`/`WallFollowParams` field names match between Task 1's definitions and every later usage.
- **Explicitly deferred, with reasoning given inline:** state estimation for wall-following/yaw (uses true state; `sim/run.py` territory), corridor corners/dead-ends (front-threshold slowdown can't be meaningfully exercised end-to-end without one — flagged, not silently skipped), `sim/run.py` itself.
- **What's NOT touched, and why that's safe:** `sim/plant.py`, `sim/integrate.py`, `sim/linearize.py`, `sim/estimator.py`, `sim/gate.py`, `sim/disturbances.py` are unmodified. `sim/control.py::design_lqr_balance`/`sim/params.py::LQRBalanceParams` (step 2) are unmodified — every test that depends on them (step 2's recovery test, step 3's NIS test, step 4's gate tests) continues to exercise the exact same code path as before.
