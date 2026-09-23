# Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `sim/run.py` (the general-purpose closed-loop episode runner, CLAUDE.md section 4: "one closed-loop episode -> DataFrame"), `experiments/run_batch.py` (batch evaluation across the three CLAUDE.md section 12 controllers), and `analysis/metrics.py` (fall rate, false-fallback, missed-fallback, detection delay, progress) — Milestone 2.

**Architecture:** `sim/run.py::run_episode(config, seed)` unifies the three overlapping test-only closed-loop harnesses that have accumulated since steps 3-5 (`tests/test_estimator.py::_run_nominal_closed_loop`, `tests/test_gate.py::_closed_loop_with_gate`, `tests/test_wall_following.py::test_follows_wall_without_crashing`) into one general implementation, resolving the "should run.py unify these" question from `project-context/OPEN_PROBLEMS.md`. It does this via an `EpisodeConfig.wall_following` flag that selects between two control-law paths — it does **not** force everything onto one shared control law (see "Pre-verified design" below for why that was tried and rejected). `experiments/run_batch.py` runs the naive/tilt-threshold/NIS-gate controllers across seeds and disturbance scenarios via `joblib`, writing Parquet outputs. `analysis/metrics.py` computes CLAUDE.md section 12's metrics from those outputs.

**Tech Stack:** Python ≥3.11, existing `numpy`/`scipy`, newly added `pandas`, `pyarrow` (Parquet), `joblib` (batch parallelism).

**Explicitly out of scope for this plan:** `analysis/plots.py`, `analysis/animate.py` (step 8), `quantum/qubo.py` (step 7 — also still blocked on `qiskit`/`qiskit-optimization` per `DECISIONS.md`), corridor dead-ends/corners in `sim/world.py`, hardware parameters (§6.2), threshold-sensitivity sweeps (CLAUDE.md §12's last metric — that's what `quantum/qubo.py`'s grid search actually re-simulates against, per §13; this step's `analysis/metrics.py` computes the other four metrics only). Also NOT attempted: fixing the corridor-mode false-fallback finding below (a materially larger undertaking, in the same spirit as prior deferrals — see `FAILED_APPROACHES.md`'s existing "fixing this properly would mean..." entries).

---

## Pre-verified design: five load-bearing findings from running this repo's actual code before writing this plan

All numbers below were produced by scripts run against this repo's real `sim/` modules (not hypothetical), the same practice steps 3-5 used.

### Design decision 1: `run_episode` does NOT unify onto one shared control law — `design_lqr_balance` for balance-only episodes, `design_lqr_speed_servo` for wall-following episodes

The obvious "clean" unification would be to always use `design_lqr_speed_servo` (with `theta_dot_ref=0` for balance-in-place), since a constant zero reference reduces exactly to the already-verified nominal (`theta_ref=0`) case (not the "ramping reference" case `sim/control.py`'s own docstring warns about). This was tried and measured against the exact push-disturbance closed loop (`tests/test_gate.py::test_enters_cautious_after_push`, seed=42):

```
                              max epsilon in the push-response window   tau1
design_lqr_balance (existing, KF-fed):        1344.87                  1300.0   <- detects (barely: 3.5% over)
design_lqr_speed_servo (KF-fed, theta_dot_ref=0):  1134.88             1300.0   <- MISSES
design_lqr_speed_servo (true-state-fed, theta_dot_ref=0): 1111.20      1300.0   <- MISSES
design_lqr_speed_servo (true-state-fed, theta_dot_ref=2.0): 1186.52    1300.0   <- MISSES
```

The instantaneous NIS spike at onset is nearly identical either way (~378-379 in all four cases) — what differs is the closed loop's *recovery* dynamics over the following ~0.5-1s, which the speed-servo gain (steady-state-tracking-tuned, `Q_integral=10`) damps slightly faster than the balance-only gain, producing a smaller windowed-sum peak. The *original* `design_lqr_balance` architecture's own push-detection margin was already thin (3.5% over `tau1`) — not a new step-6 discovery, but the margin does not survive *any* tested variant of switching to the speed-servo law, regardless of which state feeds it. The battery-droop scenario (which also uses `push_psi_dot_kick` internally) shows the identical pattern.

**Decision:** `run_episode`'s balance/gate control law is `design_lqr_balance`, fed by the KF estimate, for every episode where `wall_following=False` — this is *exactly* `tests/test_gate.py::_closed_loop_with_gate`'s algorithm, refactored, not re-derived. `GateParams.tau1`/`tau2` need no recalibration. `design_lqr_speed_servo` is used **only** when `wall_following=True` (required regardless, since `design_lqr_balance` cannot track a nonzero speed reference at all — 42% steady-state error, `FAILED_APPROACHES.md`).

### Design decision 2: wall-following episodes feed `design_lqr_speed_servo` from the TRUE state, not the KF estimate — matching step 5's existing precedent

Tested KF-estimate-fed speed-servo control specifically to see whether it was worth deviating from step 5's "state estimation for wall-following is deferred" simplification. Result (design decision 1's table, rows 2 vs 3): KF-fed and true-state-fed give near-identical push-detection behavior (1134.88 vs 1111.20) — no meaningful benefit to the extra complexity of routing the KF estimate into the wall-following control law. **Decision:** keep step 5's existing true-state-feedback precedent unchanged (also: no yaw-rate sensor exists to estimate `phi_dot` at all, so yaw control was always going to need true state regardless). The KF and both gates (`step_gate`, `step_tilt_gate`) still run every tick in `run_episode` regardless of `wall_following`, observing whatever voltage was actually applied — this is what makes `mode` available to throttle wall-following speed (CLAUDE.md section 10) even though the KF isn't in the balance control loop for these episodes.

### Design decision 3 (CONFIRMED finding, promoted from `OPEN_PROBLEMS.md`'s "suspected"): sustained forward motion with active yaw control significantly inflates epsilon, and interacts with `sim/sensors.py::encoder`'s per-wheel-quantization-before-averaging in a way `OPEN_PROBLEMS.md` had flagged as untested

`OPEN_PROBLEMS.md`'s "Suspected problems" section already flagged: *"this is a hand-constructed unit-level case [nonzero-yaw encoder quantization], not yet exercised by a real closed-loop run with actual yaw motion (that needs `sim/world.py`/wall-following to generate)."* This plan is the first time the KF/gate has ever run alongside genuine wall-following (yaw + speed-servo) motion, and it confirms the suspicion:

```
theta_dot_ref (straight-line only, no yaw):  0.0 -> mean_NIS=1.93  max_eps=488   (20s)
                                              0.05->2.87  678       0.10->2.93  682
                                              0.50->2.97  749       1.00->3.09  1195
                                              2.00->3.41  2465 (mode leaves NORMAL, even reaches HALT)

theta_dot_ref (full wall-following, with yaw), 60s nominal runs, seed=42, tau1=1300:
  0.1 rad/s: max_eps = 211,676,599  (numerical blow-up -- the robot fell; without a fall-check
             this "runs" for the full 60s on a diverged nonlinear state producing meaningless
             NIS. This is exactly why CLAUDE.md section 5's fall condition is not optional
             plumbing -- see Task 4.)
  0.2 rad/s: max_eps = 689.00   (safe)
  0.3 rad/s: max_eps = 909.13   (safe, comparable to balance-only's own 823-951 nominal range)
  0.5 rad/s: max_eps = 1605.56  (UNSAFE -- exceeds tau1, enters CAUTIOUS spuriously)
```

The relationship is not simply monotonic in speed (0.1 is the *worst* case, not the best) — plausibly a genuine low-speed wall-following resonance (a fixed-gain steering correction is proportionally larger relative to a very slow forward speed), but this plan does not attempt to fully characterize why; it only characterizes what's *safe*. `tests/test_wall_following.py`'s own `theta_dot_ref_nominal=2.0` (chosen there purely for a fast kinematic demo, with no KF/gate in that loop at all) is **not** compatible with the existing `GateParams` calibration once the gate is actually watching. **Decision:** `sim/run.py`'s corridor episodes default to `corridor_theta_dot_ref_nominal=0.3` rad/s (a new `RunParams` field, not `WallFollowParams.target_distance`'s neighborhood or `tests/test_wall_following.py`'s `2.0` — deliberately different, and documented as such) — verified via the exact `run_episode` corridor algorithm (fall-check included): completes the full 60s with `mode_set={NORMAL}`, `max_eps=909.13`.

**This is promoted to `OPEN_PROBLEMS.md`'s "Confirmed problems" in Task 8** (not fixed — fixing would mean either a theta_dot-dependent process-noise term in `EstimatorParams.Q_theta` or an average-then-quantize encoder redesign, either a materially larger undertaking than this step).

### Design decision 4: `TiltGateParams` calibration — the tilt-threshold baseline genuinely misses 3 of 6 disturbance types, by design, not by miscalibration

Calibrated `psi1=0.0087` rad (0.5°), `psi2=0.0175` rad (1.0°), `psi1_exit=0.006`, `psi2_exit=0.012`, `T_dwell=0.5` (reusing the NIS gate's dwell) against the KF-estimated `psi` (`kf.x_hat[1]` — a real robot has no noise-free `psi`), on the **balance-only** architecture (decision 1). Verified:

- **Zero false trips** across a 10-seed, 60s nominal sweep (mode stays `{NORMAL}` for every seed).
- **Detects** `gyro_bias_fault` (latency 0.740s), `battery_droop`+companion push (0.465s), `payload_shift`+tilt (0.555s) — all within the 2.0s detection window.
- **Misses** `push`, `surface_change`, `accel_noise_fault` within the 2.0s window (peak `|psi_hat|` for these stays within, or barely above, the ~0.002-0.005 rad nominal noise floor — `accel_noise_fault` in particular only corrupts a *measurement*, never perturbing true `psi` at all, so a tilt-only gate is structurally blind to it).

This is not a bug to fix by lowering thresholds further — CLAUDE.md section 12 frames tilt-threshold as *"the plain safety-filter baseline"*, and a 3/6 detection rate versus the NIS gate's 6/6 (existing, `CURRENT_STATE.md`) is exactly the kind of contrast the evaluation is designed to surface honestly.

### Design decision 5: CAUTIOUS mode's "softer Q" speed-servo gain — verified stable, chosen over reducing `Q_psi` alone

CLAUDE.md section 10: CAUTIOUS uses "softer Q in the LQR gain set." Reducing `Q_psi` alone (the dominant weight, 1e3 vs `Q_theta`/`Q_theta_dot`/`Q_psi_dot`'s `1.0`) barely changes `K` at all (`Q_psi/5`: `||K||` ratio 0.998 vs nominal) because it's already three orders of magnitude larger than the other weights. Dividing **all five** `SpeedServoParams` weights by 5 (mathematically identical to `R x5`, since LQR gain is invariant to a uniform `Q`/`R` rescale) gives a genuinely different, gentler gain:

```
nominal:  K=[-0.906, -53.30, -2.312, -5.011, -0.316]   poles: -241.83, -7.66, -6.34, -0.394±0.389j
cautious: K=[-0.583, -51.54, -2.222, -4.814, -0.141]   poles: -241.32, -7.49, -6.48, -0.263±0.261j
```

All poles stable; dominant pole moves from `-0.394` to `-0.263` (~33% slower/gentler), `||K||` ratio 0.967. **Decision:** `default_speed_servo_params_cautious()` divides every `SpeedServoParams` weight by 5.

### Design decision 6: mode's effect on control only applies to wall-following episodes

CLAUDE.md section 10: NORMAL = full speed ref; CAUTIOUS = speed ref x0.4 + softer gain; HALT = speed ref = 0, "keep balancing in place." For `wall_following=False` episodes there is no speed reference at all (`theta_dot_ref` is always 0) — CAUTIOUS's `x0.4` and HALT's `=0` are both no-ops on an already-zero reference, and the spec's own "softer Q" language is stated for the LQR gain **set** used for reference tracking, which balance-only episodes don't do. So balance-only episodes log `mode` (from whichever gate the controller selects) but it has no observable effect on control — exactly matching the already-built, disconnected `step_gate`/`step_tilt_gate` behavior from steps 3-5. Mode only changes physical behavior in `wall_following=True` episodes. **HALT uses the *nominal* (not cautious) speed-servo gain** — CLAUDE.md section 10 only says "softer Q" for CAUTIOUS; this is a literal-reading choice, consistent with how prior steps read ambiguous spec clauses literally (`DECISIONS.md`'s "NORMAL->HALT skip" and "push is a velocity kick" entries).

### Design decision 7: reproducing CLAUDE.md section 5's fall condition is not optional — corridor episodes can genuinely diverge without it

Decision 3's `theta_dot_ref=0.1` blow-up (`max_eps` reaching `2.1e8`) is almost certainly an undetected fall: without the fall check, the simulation keeps stepping a diverged nonlinear state through a KF built for the linearized-at-upright regime, producing meaningless numbers rather than a clean failure. `run_episode` (Task 4) checks `abs(psi) > fall_psi_threshold` **every tick**, before computing control, and truncates the episode (motors off, `fallen=True` on the final row) the instant it trips — verified: `x0_psi_deg=50` (`>45°` already) produces a 1-row DataFrame with `fallen=True`, `v_l=v_r=0.0`.

---

## Task 1: `pyproject.toml` — add `pandas`, `pyarrow`, `joblib`

**Files:**
- Modify: `pyproject.toml`

- [ ] **Step 1: Add the three dependencies**

```toml
[project]
name = "echo-balancer"
version = "0.1.0"
requires-python = ">=3.11"
dependencies = [
    "numpy>=2.4.6",
    "scipy>=1.17.1",
    "pandas>=2.2",
    "pyarrow>=18.0",
    "joblib>=1.4",
]

[dependency-groups]
dev = [
    "pytest>=9.1.1",
]

[tool.pytest.ini_options]
pythonpath = ["."]
testpaths = ["tests"]
```

- [ ] **Step 2: Sync the environment**

Run: `uv sync`
Expected: resolves and installs `pandas`, `pyarrow`, `joblib` (and their transitive deps) with no errors; `uv.lock` is updated.

- [ ] **Step 3: Verify the imports work**

Run: `uv run python -c "import pandas, pyarrow, joblib; print(pandas.__version__, pyarrow.__version__, joblib.__version__)"`
Expected: prints three version strings, no `ImportError`.

- [ ] **Step 4: Commit**

```bash
git add pyproject.toml uv.lock
git commit -m "$(cat <<'EOF'
build: add pandas, pyarrow, joblib for step 6 (run.py DataFrame output, batch parallelism)

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 2: `sim/params.py` — `RunParams`, `TiltGateParams`, `default_speed_servo_params_cautious`

**Files:**
- Modify: `sim/params.py` (append; add `import math` to the top-of-file imports; do not change any existing class/function)
- Test: `tests/test_params.py` (append)

- [ ] **Step 1: Write the failing test**

Append to `tests/test_params.py`:

```python
import math as _math

from sim.params import (
    RunParams,
    TiltGateParams,
    default_run_params,
    default_speed_servo_params_cautious,
    default_tilt_gate_params,
)


def test_default_run_params():
    rp = default_run_params()
    assert rp.dt_plant == 0.001
    assert rp.dt_control == 0.005
    assert rp.fall_psi_threshold == _math.radians(45.0)
    assert rp.cautious_speed_scale == 0.4
    assert rp.corridor_theta_dot_ref_nominal == 0.3


def test_run_params_is_frozen():
    rp = default_run_params()
    try:
        rp.dt_control = 0.01
        assert False, "RunParams should be frozen"
    except AttributeError:
        pass


def test_default_tilt_gate_params():
    tp = default_tilt_gate_params()
    assert tp.psi1 == 0.0087
    assert tp.psi2 == 0.0175
    assert tp.psi1_exit == 0.006
    assert tp.psi2_exit == 0.012
    assert tp.T_dwell == 0.5


def test_tilt_gate_params_is_frozen():
    tp = default_tilt_gate_params()
    try:
        tp.psi1 = 1.0
        assert False, "TiltGateParams should be frozen"
    except AttributeError:
        pass


def test_default_speed_servo_params_cautious_has_softer_q_than_nominal():
    from sim.params import default_speed_servo_params
    nominal = default_speed_servo_params()
    cautious = default_speed_servo_params_cautious()
    assert cautious.Q_theta == nominal.Q_theta / 5
    assert cautious.Q_psi == nominal.Q_psi / 5
    assert cautious.Q_theta_dot == nominal.Q_theta_dot / 5
    assert cautious.Q_psi_dot == nominal.Q_psi_dot / 5
    assert cautious.Q_integral == nominal.Q_integral / 5
    assert cautious.R == nominal.R  # R unchanged -- only Q is "softer" per CLAUDE.md section 10
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_params.py -v -k "run_params or tilt_gate_params or cautious"`
Expected: FAIL with `ImportError: cannot import name 'RunParams' from 'sim.params'`

- [ ] **Step 3: Append to `sim/params.py`**

First, add `import math` alongside the existing `from dataclasses import dataclass` line at the top of the file. Then append:

```python
@dataclass(frozen=True)
class RunParams:
    """Timing and cross-cutting constants for sim.run's closed-loop episode
    runner (CLAUDE.md sections 5, 7, 10). dt_plant/dt_control were
    previously hardcoded identically across three separate test-only
    closed-loop harnesses -- centralized here per CLAUDE.md section 3's
    "no magic numbers anywhere else," now that sim/run.py is a real module.
    """

    dt_plant: float                       # s, plant RK4 step (section 7, 1kHz)
    dt_control: float                     # s, control/estimation loop (section 7, 200Hz)
    fall_psi_threshold: float             # rad, |psi| beyond this ends the episode (section 5)
    cautious_speed_scale: float           # CAUTIOUS mode speed-ref multiplier (section 10)
    corridor_theta_dot_ref_nominal: float # rad/s, default forward-speed reference for wall-following episodes -- see docs/superpowers/plans/2026-09-22-run-batch-metrics-step6.md "Design decision 3": 0.3, NOT tests/test_wall_following.py's 2.0 (verified unsafe once the gate is actually watching: max epsilon 1605.6 over a 60s nominal run, vs tau1=1300).


def default_run_params() -> RunParams:
    return RunParams(
        dt_plant=0.001,
        dt_control=0.005,
        fall_psi_threshold=math.radians(45.0),
        cautious_speed_scale=0.4,
        corridor_theta_dot_ref_nominal=0.3,
    )


@dataclass(frozen=True)
class TiltGateParams:
    """Normal/Cautious/Halt thresholds for the tilt-threshold baseline gate
    (CLAUDE.md section 12: "mode switches on |psi| thresholds only"), keyed
    directly on the KF-estimated psi each tick (no windowed statistic,
    unlike the NIS gate -- a real robot has no access to true noise-free
    psi, so this uses kf.x_hat[1], never x_true[1]). Empirically calibrated
    against the balance-only nominal/disturbance closed loop -- see
    docs/superpowers/plans/2026-09-22-run-batch-metrics-step6.md "Design
    decision 4." This baseline is EXPECTED to miss disturbances that don't
    perturb true psi (accel_noise_fault) or that recover too fast to
    accumulate a visible tilt (push, surface_change) -- that gap versus the
    NIS gate's higher sensitivity is the point of comparing them.
    """

    psi1: float        # rad, CAUTIOUS entry
    psi2: float        # rad, HALT entry
    psi1_exit: float   # rad, CAUTIOUS -> NORMAL hysteresis exit
    psi2_exit: float   # rad, HALT -> CAUTIOUS hysteresis exit
    T_dwell: float      # s, minimum time in a mode before any transition


def default_tilt_gate_params() -> TiltGateParams:
    return TiltGateParams(psi1=0.0087, psi2=0.0175, psi1_exit=0.006, psi2_exit=0.012, T_dwell=0.5)


def default_speed_servo_params_cautious() -> SpeedServoParams:
    """CLAUDE.md section 10: CAUTIOUS mode uses "softer Q in the LQR gain
    set." All five Q weights of default_speed_servo_params() divided by 5
    (R unchanged) -- LQR gain is invariant to a uniform Q/R rescale, so
    this is exactly equivalent to R x5. Verified stable: dominant
    closed-loop pole -0.263 vs nominal -0.394 (~33% slower/gentler
    response), ||K|| ratio 0.967. Chosen over reducing Q_psi alone, which
    barely changes K at all since Q_psi already dominates Q_theta/
    Q_theta_dot/Q_psi_dot by 3 orders of magnitude -- see the plan doc's
    "Design decision 5" for the full pole comparison.
    """
    return SpeedServoParams(Q_theta=0.2, Q_psi=200.0, Q_theta_dot=0.2, Q_psi_dot=0.2, Q_integral=2.0, R=1e2)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_params.py -v`
Expected: all tests PASS (existing + new).

- [ ] **Step 5: Commit**

```bash
git add sim/params.py tests/test_params.py
git commit -m "$(cat <<'EOF'
feat: add RunParams, TiltGateParams, cautious speed-servo gain params

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 3: `sim/gate.py` — tilt-threshold baseline gate (`TiltGateState`, `initial_tilt_gate_state`, `step_tilt_gate`)

**Files:**
- Modify: `sim/gate.py` (append; add `TiltGateParams` to the existing `from sim.params import GateParams` line)
- Test: `tests/test_gate.py` (append)

- [ ] **Step 1: Write the failing test**

Append to `tests/test_gate.py`:

```python
from sim.gate import TiltGateState, initial_tilt_gate_state, step_tilt_gate
from sim.params import TiltGateParams


def _toy_tilt_gate_params() -> TiltGateParams:
    """Small, easy-to-hand-verify thresholds; not the real calibrated
    values (those are exercised by the run_episode-level tests in
    tests/test_run.py)."""
    return TiltGateParams(psi1=1.0, psi2=2.0, psi1_exit=0.4, psi2_exit=1.0, T_dwell=2.0)


def test_initial_tilt_gate_state():
    state = initial_tilt_gate_state()
    assert state.mode is GateMode.NORMAL
    assert state.time_in_mode == 0.0


def test_tilt_gate_normal_to_cautious_and_back_with_dwell():
    tgp = _toy_tilt_gate_params()
    state = initial_tilt_gate_state()

    # (psi_hat, expected_mode) per step, hand-derived from step_tilt_gate's
    # logic -- no windowing, so this is simpler than step_gate's toy sequence.
    expected = [
        (0.5, GateMode.NORMAL),   # time_in_mode=1.0, dwell (2.0) not yet satisfied
        (0.5, GateMode.NORMAL),   # time_in_mode=2.0, dwell satisfied, |0.5|<=psi1(1.0): stays NORMAL
        (1.5, GateMode.CAUTIOUS),  # time_in_mode=3.0, |1.5|>psi1: -> CAUTIOUS, time_in_mode resets to 0
        (1.5, GateMode.CAUTIOUS),  # time_in_mode=1.0, dwell blocks any transition
        (0.2, GateMode.NORMAL),   # time_in_mode=2.0, dwell satisfied, |0.2|<psi1_exit(0.4): -> NORMAL
    ]
    for psi_hat, expected_mode in expected:
        state = step_tilt_gate(state, psi_hat, dt=1.0, tilt_gate_p=tgp)
        assert state.mode is expected_mode


def test_tilt_gate_normal_to_halt_direct():
    tgp = _toy_tilt_gate_params()
    state = TiltGateState(mode=GateMode.NORMAL, time_in_mode=5.0)  # dwell already satisfied

    state = step_tilt_gate(state, 2.5, dt=1.0, tilt_gate_p=tgp)

    assert state.mode is GateMode.HALT
    assert state.time_in_mode == 0.0


def test_tilt_gate_halt_to_cautious_exit():
    tgp = _toy_tilt_gate_params()
    state = TiltGateState(mode=GateMode.HALT, time_in_mode=5.0)  # dwell already satisfied

    state = step_tilt_gate(state, 0.5, dt=1.0, tilt_gate_p=tgp)

    assert state.mode is GateMode.CAUTIOUS
    assert state.time_in_mode == 0.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_gate.py -v -k tilt`
Expected: FAIL with `ImportError: cannot import name 'TiltGateState' from 'sim.gate'`

- [ ] **Step 3: Append to `sim/gate.py`**

Change the existing import line:
```python
from sim.params import GateParams
```
to:
```python
from sim.params import GateParams, TiltGateParams
```

Then append to the end of the file:

```python
@dataclass(frozen=True)
class TiltGateState:
    mode: GateMode
    time_in_mode: float


def initial_tilt_gate_state() -> TiltGateState:
    return TiltGateState(mode=GateMode.NORMAL, time_in_mode=0.0)


def step_tilt_gate(state: TiltGateState, psi_hat: float, dt: float, tilt_gate_p: TiltGateParams) -> TiltGateState:
    """Advance the tilt-threshold baseline gate by one control-loop sample
    (CLAUDE.md section 12: "mode switches on |psi| thresholds only").
    Unlike step_gate's windowed epsilon_k, this acts directly on the
    KF-estimated |psi| each tick -- there is no window, so no
    window-fullness gate is needed (contrast with step_gate's
    `window_full` check).
    """
    abs_psi = abs(psi_hat)
    time_in_mode = state.time_in_mode + dt
    can_transition = time_in_mode >= tilt_gate_p.T_dwell

    new_mode = state.mode
    if can_transition:
        if state.mode is GateMode.NORMAL:
            if abs_psi > tilt_gate_p.psi2:
                new_mode = GateMode.HALT
            elif abs_psi > tilt_gate_p.psi1:
                new_mode = GateMode.CAUTIOUS
        elif state.mode is GateMode.CAUTIOUS:
            if abs_psi > tilt_gate_p.psi2:
                new_mode = GateMode.HALT
            elif abs_psi < tilt_gate_p.psi1_exit:
                new_mode = GateMode.NORMAL
        elif state.mode is GateMode.HALT:
            if abs_psi < tilt_gate_p.psi2_exit:
                new_mode = GateMode.CAUTIOUS

    if new_mode != state.mode:
        time_in_mode = 0.0

    return TiltGateState(mode=new_mode, time_in_mode=time_in_mode)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_gate.py -v`
Expected: all tests PASS (existing + new).

- [ ] **Step 5: Commit**

```bash
git add sim/gate.py tests/test_gate.py
git commit -m "$(cat <<'EOF'
feat: add tilt-threshold baseline gate (step_tilt_gate)

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 4: `sim/run.py` — `ControllerType`, `EpisodeConfig`, `run_episode` (balance-only path + fall condition)

**Files:**
- Create: `sim/run.py`
- Test: `tests/test_run.py`

This task builds the balance-only (`wall_following=False`) path only — exactly `tests/test_gate.py::_closed_loop_with_gate`'s algorithm, refactored to return a DataFrame and support all three controllers plus the fall condition. Task 5 adds the wall-following path.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_run.py`:

```python
import numpy as np
import pandas as pd
import pytest
import scipy.stats as stats

from sim.disturbances import (
    battery_droop_v_batt,
    payload_shift_plant_params,
    push_psi_dot_kick,
    sensor_fault_accel_noise_params,
    sensor_fault_gyro_bias_step,
    surface_change_plant_params,
)
from sim.params import default_disturbance_params
from sim.run import ControllerType, EpisodeConfig, run_episode


def test_reproducibility_same_config_and_seed_gives_identical_dataframe():
    """CLAUDE.md section 11: 'Same (config, seed) => identical DataFrame.'"""
    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=2.0, wall_following=False)

    df1 = run_episode(config, seed=7)
    df2 = run_episode(config, seed=7)

    pd.testing.assert_frame_equal(df1, df2)


def test_run_episode_returns_expected_columns():
    config = EpisodeConfig(controller=ControllerType.NAIVE, T=1.0, wall_following=False)

    df = run_episode(config, seed=0)

    expected_columns = {
        "t", "theta", "psi", "phi", "theta_dot", "psi_dot", "phi_dot",
        "pos_x", "pos_y", "mode", "nis", "epsilon",
        "front_dist", "right_dist", "v_l", "v_r", "fallen",
    }
    assert expected_columns == set(df.columns)
    assert len(df) == int(round(1.0 / 0.005))  # dt_control=5ms


def test_naive_controller_stays_normal_even_under_a_disturbance_that_would_trigger_the_gate():
    dp = default_disturbance_params()

    def apply_push(t, x, bg, p, sp):
        x = x.copy()
        x[4] = push_psi_dot_kick(x[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x, bg, p, sp, dp.battery_droop_v_nominal

    config = EpisodeConfig(controller=ControllerType.NAIVE, T=10.0, wall_following=False, disturbance=apply_push)
    df = run_episode(config, seed=42)

    assert set(df["mode"]) == {"NORMAL"}


def test_balance_only_nominal_60s_matches_existing_gate_calibration():
    """Regression check: run_episode's wall_following=False path must
    reproduce tests/test_gate.py::test_no_mode_change_on_nominal_60s_run
    and tests/test_estimator.py::test_nis_consistent_on_nominal_60s_run
    exactly (same algorithm, refactored) -- GateParams/EstimatorParams need
    no recalibration for this path. Verified pre-plan: seed=42, 60s,
    mode_set={'NORMAL'}, max epsilon=895.96 (well under tau1=1300)."""
    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=60.0, wall_following=False)

    df = run_episode(config, seed=42)

    assert set(df["mode"]) == {"NORMAL"}
    chi2_3 = stats.chi2(df=3)
    mean_nis = df["nis"].mean()
    frac_above_95 = (df["nis"] > chi2_3.ppf(0.95)).mean()
    assert chi2_3.ppf(0.025) <= mean_nis <= chi2_3.ppf(0.975)
    assert 0.01 <= frac_above_95 <= 0.10
    assert not df["fallen"].any()


def _first_non_normal_latency(df, onset, window_s):
    onset_idx = int(round(onset / 0.005))
    deadline_idx = onset_idx + int(round(window_s / 0.005))
    window = df.iloc[onset_idx:min(deadline_idx, len(df))]
    non_normal = window[window["mode"] != "NORMAL"]
    return None if non_normal.empty else non_normal["t"].iloc[0] - onset


@pytest.mark.parametrize("name,onset_attr,build_disturbance", [
    ("surface_change", "surface_change_onset", lambda dp: (
        lambda t, x, bg, p, sp: (x, bg, surface_change_plant_params(t, dp.surface_change_onset, dp.surface_change_duration, dp.surface_change_fw, p), sp, dp.battery_droop_v_nominal)
    )),
    ("gyro_bias_fault", "gyro_bias_fault_onset", lambda dp: (
        lambda t, x, bg, p, sp: (x, sensor_fault_gyro_bias_step(bg, t, dp.gyro_bias_fault_onset, 0.005, dp.gyro_bias_fault_magnitude), p, sp, dp.battery_droop_v_nominal)
    )),
    ("accel_noise_fault", "accel_noise_fault_onset", lambda dp: (
        lambda t, x, bg, p, sp: (x, bg, p, sensor_fault_accel_noise_params(t, dp.accel_noise_fault_onset, dp.accel_noise_fault_duration, dp.accel_noise_fault_multiplier, sp), dp.battery_droop_v_nominal)
    )),
])
def test_nis_gate_detects_each_disturbance_within_window(name, onset_attr, build_disturbance):
    """Regression check against tests/test_gate.py's existing (already
    passing) per-disturbance detection tests -- run_episode's balance-only
    + NIS_GATE path is the same algorithm, so it must keep detecting these
    within the same CLAUDE.md section 11 2s window."""
    dp = default_disturbance_params()
    onset = getattr(dp, onset_attr)
    config = EpisodeConfig(
        controller=ControllerType.NIS_GATE, T=20.0, wall_following=False,
        disturbance=build_disturbance(dp),
    )

    df = run_episode(config, seed=42)

    latency = _first_non_normal_latency(df, onset, dp.detection_window_s)
    assert latency is not None, f"{name}: never left NORMAL within {dp.detection_window_s}s of onset"


@pytest.mark.parametrize("name,expect_detected,expected_latency_s", [
    ("gyro_bias_fault", True, 0.740),
    ("payload_shift", True, 0.555),
])
def test_tilt_threshold_controller_detection(name, expect_detected, expected_latency_s):
    """Verified pre-plan against the balance-only architecture: the
    tilt-threshold baseline detects gyro_bias_fault at 0.740s and
    payload_shift (+its required companion tilt) at 0.555s -- see the plan
    doc's "Design decision 4." (push/surface_change/accel_noise_fault are
    verified MISSES for this baseline -- see
    test_tilt_threshold_controller_misses_push below.)"""
    dp = default_disturbance_params()
    if name == "gyro_bias_fault":
        onset = dp.gyro_bias_fault_onset
        disturbance = lambda t, x, bg, p, sp: (
            x, sensor_fault_gyro_bias_step(bg, t, dp.gyro_bias_fault_onset, 0.005, dp.gyro_bias_fault_magnitude), p, sp, dp.battery_droop_v_nominal
        )
        x0_psi_deg = 0.0
    else:
        onset = dp.payload_shift_onset
        disturbance = lambda t, x, bg, p, sp: (
            x, bg, payload_shift_plant_params(t, dp.payload_shift_onset, dp.payload_shift_delta_M, dp.payload_shift_delta_L, p), sp, dp.battery_droop_v_nominal
        )
        x0_psi_deg = dp.payload_shift_test_psi0_deg

    config = EpisodeConfig(
        controller=ControllerType.TILT_THRESHOLD, T=20.0, wall_following=False,
        x0_psi_deg=x0_psi_deg, disturbance=disturbance,
    )
    df = run_episode(config, seed=42)

    latency = _first_non_normal_latency(df, onset, dp.detection_window_s)
    assert latency is not None
    assert latency == pytest.approx(expected_latency_s, abs=0.01)


def test_tilt_threshold_controller_misses_push():
    """The tilt-threshold baseline's known, expected blind spot -- push's
    psi excursion never exceeds the nominal noise floor. Contrast with
    test_nis_gate_detects_each_disturbance_within_window, which the NIS
    gate passes for the same class of disturbance."""
    dp = default_disturbance_params()

    def apply_push(t, x, bg, p, sp):
        x = x.copy()
        x[4] = push_psi_dot_kick(x[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x, bg, p, sp, dp.battery_droop_v_nominal

    config = EpisodeConfig(controller=ControllerType.TILT_THRESHOLD, T=20.0, wall_following=False, disturbance=apply_push)
    df = run_episode(config, seed=42)

    latency = _first_non_normal_latency(df, dp.push_onset, dp.detection_window_s)
    assert latency is None


def test_fall_condition_ends_episode_and_cuts_motors():
    """CLAUDE.md section 5: '|psi| > 45 deg ends the episode (motors
    off).' Starting beyond the threshold falls on the very first tick."""
    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=5.0, wall_following=False, x0_psi_deg=50.0)

    df = run_episode(config, seed=42)

    assert len(df) == 1
    assert bool(df["fallen"].iloc[0]) is True
    assert df["v_l"].iloc[0] == 0.0
    assert df["v_r"].iloc[0] == 0.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_run.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.run'`

- [ ] **Step 3: Create `sim/run.py`**

```python
"""General-purpose closed-loop episode runner (CLAUDE.md section 4).

run_episode(config, seed) drives the full plant/sensor/estimator/gate
stack for one episode and returns a per-control-tick pandas DataFrame,
unifying what were three separate, overlapping test-only closed-loop
harnesses (tests/test_estimator.py::_run_nominal_closed_loop,
tests/test_gate.py::_closed_loop_with_gate,
tests/test_wall_following.py::test_follows_wall_without_crashing) into one
general-purpose implementation. See docs/superpowers/plans/2026-09-22-run-
batch-metrics-step6.md for the pre-verified design decisions this makes:

- The balance/gate control law is ALWAYS sim.control.design_lqr_balance,
  driven by the Kalman filter estimate -- exactly reproducing the
  already-calibrated step 3/4 architecture (GateParams tau1/tau2 stay
  valid) -- UNLESS wall_following=True, which requires
  sim.control.design_lqr_speed_servo instead (design_lqr_balance cannot
  track a nonzero speed reference without steady-state error).
- wall_following=True control (speed-servo + yaw) is fed by the TRUE
  plant state, not the KF estimate, matching tests/test_wall_following.py's
  precedent. The KF/both gates always run in parallel regardless of
  wall_following, observing whatever voltage was actually applied, purely
  for logging/mode-decision purposes.
- Both gates (sim.gate.step_gate for the NIS-based mode,
  sim.gate.step_tilt_gate for the tilt-threshold baseline) always run
  every tick regardless of config.controller, so all three controllers'
  episodes are directly comparable on identical seeds/disturbances
  (CLAUDE.md section 12) -- only which gate's mode actually drives control
  differs.
"""

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional

import numpy as np
import pandas as pd

from sim.control import design_lqr_balance
from sim.disturbances import clip_voltage
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
from sim.gate import (
    GateMode,
    initial_gate_state,
    initial_tilt_gate_state,
    step_gate,
    step_tilt_gate,
)
from sim.integrate import rk4_step
from sim.params import (
    PlantParams,
    SensorParams,
    default_disturbance_params,
    default_estimator_params,
    default_gate_params,
    default_lqr_balance_params,
    default_plant_params,
    default_run_params,
    default_sensor_params,
    default_tilt_gate_params,
)
from sim.plant import f as plant_f
from sim.sensors import accelerometer, encoder, gyro

DisturbanceFn = Callable[
    [float, np.ndarray, float, PlantParams, SensorParams],
    tuple[np.ndarray, float, PlantParams, SensorParams, float],
]


class ControllerType(str, Enum):
    """The three controllers compared in CLAUDE.md section 12."""

    NAIVE = "naive"
    TILT_THRESHOLD = "tilt_threshold"
    NIS_GATE = "nis_gate"


@dataclass(frozen=True)
class EpisodeConfig:
    """One episode's setup. (config, seed) must reproduce an identical
    DataFrame (CLAUDE.md section 11)."""

    controller: ControllerType
    T: float                                  # s, episode duration (may end earlier on a fall)
    wall_following: bool = False
    x0_psi_deg: float = 0.0
    disturbance: Optional[DisturbanceFn] = None
    disturbance_name: str = "none"            # label only, for experiments/run_batch.py's output


def run_episode(config: EpisodeConfig, seed: int) -> pd.DataFrame:
    p = default_plant_params()
    sensor_p = default_sensor_params()
    est_p = default_estimator_params()
    lqr_p = default_lqr_balance_params()
    gate_p = default_gate_params()
    tilt_gate_p = default_tilt_gate_params()
    run_p = default_run_params()
    dp = default_disturbance_params()

    K4 = design_lqr_balance(p, lqr_p)

    dt_control = run_p.dt_control
    dt_plant = run_p.dt_plant
    n_sub = int(round(dt_control / dt_plant))

    Ad, Bd = discretize(p, dt_control)
    C = measurement_matrix()
    Q = process_noise(sensor_p, est_p, dt_control)
    R = measurement_noise(sensor_p)

    rng = np.random.default_rng(seed)
    n_steps = int(round(config.T / dt_control))

    x_true = np.array([0.0, np.radians(config.x0_psi_deg), 0.0, 0.0, 0.0, 0.0])
    b_g_true = 0.0
    kf = KalmanState(x_hat=np.zeros(5), P=initial_covariance(est_p))
    nis_gate_state = initial_gate_state()
    tilt_gate_state = initial_tilt_gate_state()
    u_prev = 0.0

    rows = []

    for k in range(n_steps):
        t = k * dt_control

        if config.disturbance is not None:
            x_true, b_g_true, p_now, sensor_p_now, v_batt_now = config.disturbance(
                t, x_true, b_g_true, p, sensor_p
            )
        else:
            p_now, sensor_p_now, v_batt_now = p, sensor_p, dp.battery_droop_v_nominal

        def xdotf(x, uu, pp=p_now):
            return plant_f(x, uu[0], uu[1], pp)

        xdot_true = xdotf(x_true, (u_prev / 2, u_prev / 2))
        theta_ddot_true = xdot_true[3]

        theta_enc = encoder(x_true[0], x_true[1], x_true[2], p_now, sensor_p_now)
        psi_dot_meas, b_g_true = gyro(x_true[4], b_g_true, dt_control, sensor_p_now, rng)
        psi_acc_meas = accelerometer(x_true[1], theta_ddot_true, p_now, sensor_p_now, rng)
        y = np.array([theta_enc, psi_dot_meas, psi_acc_meas])

        kf = predict(kf, Ad, Bd, u_prev, Q)
        kf, nis = update(kf, y, C, R)
        nis_gate_state, epsilon = step_gate(nis_gate_state, nis, dt_control, gate_p)
        tilt_gate_state = step_tilt_gate(tilt_gate_state, kf.x_hat[1], dt_control, tilt_gate_p)

        if config.controller is ControllerType.NAIVE:
            mode = GateMode.NORMAL
        elif config.controller is ControllerType.TILT_THRESHOLD:
            mode = tilt_gate_state.mode
        else:
            mode = nis_gate_state.mode

        fallen = bool(abs(x_true[1]) > run_p.fall_psi_threshold)

        u_cmd = -K4 @ kf.x_hat[:4]
        v_l_cmd = v_r_cmd = u_cmd / 2

        if fallen:
            v_l, v_r = 0.0, 0.0
        else:
            v_l = clip_voltage(v_l_cmd, v_batt_now)
            v_r = clip_voltage(v_r_cmd, v_batt_now)
        u_prev = v_l + v_r

        rows.append({
            "t": t,
            "theta": x_true[0], "psi": x_true[1], "phi": x_true[2],
            "theta_dot": x_true[3], "psi_dot": x_true[4], "phi_dot": x_true[5],
            "pos_x": np.nan, "pos_y": np.nan,
            "mode": mode.value,
            "nis": nis, "epsilon": epsilon,
            "front_dist": np.nan, "right_dist": np.nan,
            "v_l": v_l, "v_r": v_r,
            "fallen": fallen,
        })

        if fallen:
            break

        for _ in range(n_sub):
            x_true = rk4_step(xdotf, x_true, (v_l, v_r), dt_plant)

    return pd.DataFrame(rows)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_run.py -v`
Expected: all tests PASS. In particular `test_balance_only_nominal_60s_matches_existing_gate_calibration` and the three parametrized `test_nis_gate_detects_each_disturbance_within_window` cases confirm the refactor didn't change behavior.

- [ ] **Step 5: Run the full existing suite to confirm no regressions**

Run: `uv run pytest -q`
Expected: all previously-passing tests still pass (this task touches no existing files besides the gate.py/params.py additions from Tasks 2-3).

- [ ] **Step 6: Commit**

```bash
git add sim/run.py tests/test_run.py
git commit -m "$(cat <<'EOF'
feat: add sim/run.py balance-only episode runner (naive/tilt-threshold/NIS-gate controllers, fall condition)

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 5: `sim/run.py` — wall-following (corridor) path

**Files:**
- Modify: `sim/run.py`
- Test: `tests/test_run.py` (append)

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_run.py`:

```python
def test_corridor_nominal_60s_stays_normal_at_the_calibrated_safe_speed():
    """Verified pre-plan: seed=42, 60s, wall_following=True,
    corridor_theta_dot_ref_nominal=0.3 (RunParams default) -- completes all
    12000 steps with mode_set={'NORMAL'}, max epsilon=909.13 (comparable to
    the balance-only nominal range of 823-951, well under tau1=1300). See
    the plan doc's "Design decision 3" -- tests/test_wall_following.py's
    own theta_dot_ref_nominal=2.0 is NOT used here; it was verified unsafe
    once the gate is actually watching (max epsilon 1605.56)."""
    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=60.0, wall_following=True)

    df = run_episode(config, seed=42)

    assert len(df) == int(round(60.0 / 0.005))
    assert set(df["mode"]) == {"NORMAL"}
    assert not df["fallen"].any()
    assert df["epsilon"].max() < 1300.0


def test_corridor_episode_makes_forward_progress_and_stays_in_corridor():
    config = EpisodeConfig(controller=ControllerType.NAIVE, T=20.0, wall_following=True)

    df = run_episode(config, seed=42)

    assert (df["pos_y"] > 0.0).all()
    assert (df["pos_y"] < 1.0).all()  # default corridor width
    assert df["theta"].iloc[-1] > df["theta"].iloc[0]  # rolled forward
    assert not df["front_dist"].isna().any()
    assert not df["right_dist"].isna().any()


def test_corridor_push_disturbance_is_a_known_miss_for_the_nis_gate():
    """Documents, rather than hides, the plan doc's "Design decision 1"
    finding: push-type disturbances are not reliably detected once the
    control law is design_lqr_speed_servo, even at the calibrated-safe
    corridor speed. Verified pre-plan: seed=42, max epsilon in the
    detection window = 1085.51, under tau1=1300 -- MISSED. This is an
    inherited, pre-existing thin margin (design_lqr_balance's own push
    margin was already only 3.5% over tau1), not a step-6 regression."""
    dp = default_disturbance_params()

    def apply_push(t, x, bg, p, sp):
        x = x.copy()
        x[4] = push_psi_dot_kick(x[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x, bg, p, sp, dp.battery_droop_v_nominal

    config = EpisodeConfig(controller=ControllerType.NIS_GATE, T=20.0, wall_following=True, disturbance=apply_push)
    df = run_episode(config, seed=42)

    latency = _first_non_normal_latency(df, dp.push_onset, dp.detection_window_s)
    assert latency is None


def test_cautious_mode_scales_down_corridor_speed_reference():
    """Force CAUTIOUS via the tilt-threshold controller (a large initial
    tilt trips it almost immediately, per test_tilt_threshold_controller_
    detection's calibration) and confirm the robot's realized forward
    speed drops relative to a NORMAL-mode run -- CLAUDE.md section 10:
    "CAUTIOUS (speed ref x 0.4 ...)"."""
    normal_config = EpisodeConfig(controller=ControllerType.NAIVE, T=10.0, wall_following=True)
    normal_df = run_episode(normal_config, seed=42)
    normal_progress = normal_df["theta"].iloc[-1] - normal_df["theta"].iloc[0]

    dp = default_disturbance_params()
    cautious_config = EpisodeConfig(
        controller=ControllerType.TILT_THRESHOLD, T=10.0, wall_following=True,
        x0_psi_deg=dp.payload_shift_test_psi0_deg,  # 2 deg -- enough to trip CAUTIOUS quickly, verified in Task 4
    )
    cautious_df = run_episode(cautious_config, seed=42)
    cautious_progress = cautious_df["theta"].iloc[-1] - cautious_df["theta"].iloc[0]

    assert "CAUTIOUS" in set(cautious_df["mode"])
    assert cautious_progress < normal_progress
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_run.py -v -k corridor or cautious_mode`
Expected: FAIL — `wall_following=True` currently produces the same balance-only behavior (all-NaN `pos_x`/`pos_y`/`front_dist`/`right_dist`, no speed tracking), so `test_corridor_episode_makes_forward_progress_and_stays_in_corridor` and the others fail on the NaN/no-progress assertions.

- [ ] **Step 3: Extend `sim/run.py` with the wall-following path**

Replace the top-of-file imports with:

```python
from sim.control import (
    design_lqr_balance,
    design_lqr_speed_servo,
    front_threshold_speed_adjust,
    wall_following_control,
    yaw_p_control,
)
from sim.disturbances import clip_voltage
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
from sim.gate import (
    GateMode,
    initial_gate_state,
    initial_tilt_gate_state,
    step_gate,
    step_tilt_gate,
)
from sim.integrate import rk4_step
from sim.params import (
    PlantParams,
    SensorParams,
    default_corridor_params,
    default_disturbance_params,
    default_estimator_params,
    default_gate_params,
    default_lqr_balance_params,
    default_plant_params,
    default_run_params,
    default_sensor_params,
    default_speed_servo_params,
    default_speed_servo_params_cautious,
    default_tilt_gate_params,
    default_wall_follow_params,
    default_yaw_control_params,
)
from sim.plant import f as plant_f
from sim.sensors import accelerometer, encoder, gyro, ultrasonic
from sim.world import cast_ray, pose_velocity
```

(This removes the in-loop `from sim.sensors import ...` from Task 4 — move that import here.)

Replace the body of `run_episode` with:

```python
def run_episode(config: EpisodeConfig, seed: int) -> pd.DataFrame:
    p = default_plant_params()
    sensor_p = default_sensor_params()
    est_p = default_estimator_params()
    lqr_p = default_lqr_balance_params()
    gate_p = default_gate_params()
    tilt_gate_p = default_tilt_gate_params()
    run_p = default_run_params()
    dp = default_disturbance_params()

    K4 = design_lqr_balance(p, lqr_p)

    wall_p = default_wall_follow_params()

    if config.wall_following:
        K5_nominal = design_lqr_speed_servo(p, default_speed_servo_params())
        K5_cautious = design_lqr_speed_servo(p, default_speed_servo_params_cautious())
        yaw_p = default_yaw_control_params()
        corridor_p = default_corridor_params()

        def pose_dotf(pp, uu):
            return pose_velocity(pp, uu, p)

    dt_control = run_p.dt_control
    dt_plant = run_p.dt_plant
    n_sub = int(round(dt_control / dt_plant))
    n_control_per_ultrasonic = int(round((1.0 / sensor_p.ultrasonic_rate_hz) / dt_control))

    Ad, Bd = discretize(p, dt_control)
    C = measurement_matrix()
    Q = process_noise(sensor_p, est_p, dt_control)
    R = measurement_noise(sensor_p)

    rng = np.random.default_rng(seed)
    n_steps = int(round(config.T / dt_control))

    x_true = np.array([0.0, np.radians(config.x0_psi_deg), 0.0, 0.0, 0.0, 0.0])
    pos = np.array([0.0, wall_p.target_distance])
    b_g_true = 0.0
    kf = KalmanState(x_hat=np.zeros(5), P=initial_covariance(est_p))
    nis_gate_state = initial_gate_state()
    tilt_gate_state = initial_tilt_gate_state()
    u_prev = 0.0
    theta_ref = 0.0
    z = 0.0
    phi_dot_ref = 0.0
    prev_wall_err = 0.0
    front_reading = sensor_p.ultrasonic_range_max
    right_reading = sensor_p.ultrasonic_range_max

    rows = []

    for k in range(n_steps):
        t = k * dt_control

        if config.disturbance is not None:
            x_true, b_g_true, p_now, sensor_p_now, v_batt_now = config.disturbance(
                t, x_true, b_g_true, p, sensor_p
            )
        else:
            p_now, sensor_p_now, v_batt_now = p, sensor_p, dp.battery_droop_v_nominal

        def xdotf(x, uu, pp=p_now):
            return plant_f(x, uu[0], uu[1], pp)

        xdot_true = xdotf(x_true, (u_prev / 2, u_prev / 2))
        theta_ddot_true = xdot_true[3]

        theta_enc = encoder(x_true[0], x_true[1], x_true[2], p_now, sensor_p_now)
        psi_dot_meas, b_g_true = gyro(x_true[4], b_g_true, dt_control, sensor_p_now, rng)
        psi_acc_meas = accelerometer(x_true[1], theta_ddot_true, p_now, sensor_p_now, rng)
        y = np.array([theta_enc, psi_dot_meas, psi_acc_meas])

        kf = predict(kf, Ad, Bd, u_prev, Q)
        kf, nis = update(kf, y, C, R)
        nis_gate_state, epsilon = step_gate(nis_gate_state, nis, dt_control, gate_p)
        tilt_gate_state = step_tilt_gate(tilt_gate_state, kf.x_hat[1], dt_control, tilt_gate_p)

        if config.controller is ControllerType.NAIVE:
            mode = GateMode.NORMAL
        elif config.controller is ControllerType.TILT_THRESHOLD:
            mode = tilt_gate_state.mode
        else:
            mode = nis_gate_state.mode

        fallen = bool(abs(x_true[1]) > run_p.fall_psi_threshold)

        if config.wall_following:
            if k % n_control_per_ultrasonic == 0:
                heading = x_true[2]
                true_right = cast_ray(pos[0], pos[1], heading - np.pi / 2, corridor_p, 0.02, 4.0)
                true_front = cast_ray(pos[0], pos[1], heading, corridor_p, 0.02, 4.0)
                right_reading = ultrasonic(true_right, sensor_p, rng)
                front_reading = ultrasonic(true_front, sensor_p, rng)
                wall_err = wall_p.target_distance - right_reading
                phi_dot_ref, prev_wall_err = wall_following_control(
                    wall_err, prev_wall_err, 1.0 / sensor_p.ultrasonic_rate_hz, wall_p
                )

            base_ref = front_threshold_speed_adjust(front_reading, run_p.corridor_theta_dot_ref_nominal, wall_p)
            if mode is GateMode.NORMAL:
                theta_dot_ref, K5 = base_ref, K5_nominal
            elif mode is GateMode.CAUTIOUS:
                theta_dot_ref, K5 = base_ref * run_p.cautious_speed_scale, K5_cautious
            else:  # HALT: speed ref = 0, "keep balancing in place" -- nominal gain (see plan doc "Design decision 6")
                theta_dot_ref, K5 = 0.0, K5_nominal

            err5 = np.array([x_true[0] - theta_ref, x_true[1], x_true[3] - theta_dot_ref, x_true[4], z])
            theta_ref += theta_dot_ref * dt_control
            u_common = -K5 @ err5
            z += (x_true[0] - theta_ref) * dt_control
            diff_v = yaw_p_control(phi_dot_ref, x_true[5], yaw_p)
            v_l_cmd = u_common / 2 - diff_v / 2
            v_r_cmd = u_common / 2 + diff_v / 2
        else:
            u_cmd = -K4 @ kf.x_hat[:4]
            v_l_cmd = v_r_cmd = u_cmd / 2

        if fallen:
            v_l, v_r = 0.0, 0.0
        else:
            v_l = clip_voltage(v_l_cmd, v_batt_now)
            v_r = clip_voltage(v_r_cmd, v_batt_now)
        u_prev = v_l + v_r

        rows.append({
            "t": t,
            "theta": x_true[0], "psi": x_true[1], "phi": x_true[2],
            "theta_dot": x_true[3], "psi_dot": x_true[4], "phi_dot": x_true[5],
            "pos_x": pos[0] if config.wall_following else np.nan,
            "pos_y": pos[1] if config.wall_following else np.nan,
            "mode": mode.value,
            "nis": nis, "epsilon": epsilon,
            "front_dist": front_reading if config.wall_following else np.nan,
            "right_dist": right_reading if config.wall_following else np.nan,
            "v_l": v_l, "v_r": v_r,
            "fallen": fallen,
        })

        if fallen:
            break

        for _ in range(n_sub):
            x_true = rk4_step(xdotf, x_true, (v_l, v_r), dt_plant)
            if config.wall_following:
                pos = rk4_step(pose_dotf, pos, (x_true[3], x_true[2]), dt_plant)

    return pd.DataFrame(rows)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_run.py -v`
Expected: all tests PASS, including the Task 4 balance-only tests (unaffected) and the new corridor tests.

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -q`
Expected: all tests pass. Note the full suite now takes noticeably longer (the two new 60s corridor tests add real wall-clock time, similar to the existing 60s balance-only tests) — this is expected, not a regression.

- [ ] **Step 6: Commit**

```bash
git add sim/run.py tests/test_run.py
git commit -m "$(cat <<'EOF'
feat: add wall-following (corridor) path to sim/run.py with mode-scaled speed reference

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 6: `analysis/metrics.py`

**Files:**
- Create: `analysis/__init__.py` (empty)
- Create: `analysis/metrics.py`
- Test: `tests/test_metrics.py`

Metrics operate on the two-DataFrame shape `experiments/run_batch.py` (Task 7) produces: `steps_df` (per-control-tick rows, tagged with `episode_id`) and `episodes_df` (one row per episode: `episode_id`, `controller`, `scenario`, `seed`, `wall_following`, `disturbance_onset` (`NaN` for the nominal scenario), `fell`). Tested here against small, hand-built synthetic DataFrames — no full simulation needed to test the aggregation logic itself.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_metrics.py`:

```python
import numpy as np
import pandas as pd
import pytest

from analysis.metrics import (
    detection_delay,
    fall_rate,
    false_fallback_fraction,
    missed_fallback,
    progress,
)


def _episode(episode_id, controller, scenario, fell, disturbance_onset=np.nan):
    return {
        "episode_id": episode_id, "controller": controller, "scenario": scenario,
        "seed": 0, "wall_following": False, "disturbance_onset": disturbance_onset, "fell": fell,
    }


def _step(episode_id, t, mode, theta=0.0, fallen=False):
    return {"episode_id": episode_id, "t": t, "mode": mode, "theta": theta, "fallen": fallen}


def test_fall_rate_computes_fraction_per_controller():
    episodes_df = pd.DataFrame([
        _episode("e1", "naive", "nominal", fell=False),
        _episode("e2", "naive", "push", fell=True),
        _episode("e3", "nis_gate", "nominal", fell=False),
        _episode("e4", "nis_gate", "push", fell=False),
    ])

    result = fall_rate(episodes_df)

    assert result["naive"] == 0.5
    assert result["nis_gate"] == 0.0


def test_false_fallback_fraction_uses_only_nominal_scenario():
    episodes_df = pd.DataFrame([
        _episode("e1", "nis_gate", "nominal", fell=False),
        _episode("e2", "nis_gate", "push", fell=False),  # excluded: not nominal
    ])
    steps_df = pd.DataFrame([
        _step("e1", 0.0, "NORMAL"), _step("e1", 0.005, "NORMAL"),
        _step("e1", 0.010, "CAUTIOUS"), _step("e1", 0.015, "NORMAL"),
        _step("e2", 0.0, "HALT"), _step("e2", 0.005, "HALT"),  # ignored: not the nominal scenario
    ])

    result = false_fallback_fraction(steps_df, episodes_df)

    assert result["nis_gate"] == 0.25  # 1 of 4 nominal-episode ticks was non-NORMAL


def test_detection_delay_measures_onset_to_first_non_normal():
    episodes_df = pd.DataFrame([_episode("e1", "nis_gate", "push", fell=False, disturbance_onset=5.0)])
    steps_df = pd.DataFrame([
        _step("e1", 4.995, "NORMAL"), _step("e1", 5.000, "NORMAL"),
        _step("e1", 5.005, "NORMAL"), _step("e1", 5.010, "CAUTIOUS"),
    ])

    result = detection_delay(steps_df, episodes_df)

    assert result.loc[result["episode_id"] == "e1", "detection_delay_s"].iloc[0] == pytest.approx(0.010, abs=1e-9)


def test_detection_delay_is_nan_when_never_detected():
    episodes_df = pd.DataFrame([_episode("e1", "tilt_threshold", "push", fell=False, disturbance_onset=5.0)])
    steps_df = pd.DataFrame([_step("e1", 5.0, "NORMAL"), _step("e1", 6.0, "NORMAL")])

    result = detection_delay(steps_df, episodes_df)

    assert np.isnan(result.loc[result["episode_id"] == "e1", "detection_delay_s"].iloc[0])


def test_progress_is_wheel_radius_times_delta_theta():
    episodes_df = pd.DataFrame([_episode("e1", "naive", "nominal", fell=False)])
    steps_df = pd.DataFrame([
        _step("e1", 0.0, "NORMAL", theta=0.0),
        _step("e1", 1.0, "NORMAL", theta=2.5),
    ])

    result = progress(steps_df, episodes_df)

    from sim.params import default_plant_params
    expected = default_plant_params().R * 2.5
    assert result.loc[result["episode_id"] == "e1", "progress_m"].iloc[0] == pytest.approx(expected, abs=1e-9)


def test_missed_fallback_true_when_still_normal_half_a_second_before_the_fall():
    episodes_df = pd.DataFrame([_episode("e1", "nis_gate", "push", fell=True, disturbance_onset=5.0)])
    steps_df = pd.DataFrame([
        _step("e1", 4.0, "NORMAL"),
        _step("e1", 4.5, "NORMAL"),   # exactly 0.5s before the fall -- still NORMAL
        _step("e1", 4.9, "NORMAL"),
        _step("e1", 5.0, "NORMAL", fallen=True),  # the fall itself
    ])

    result = missed_fallback(steps_df, episodes_df)

    assert bool(result.loc[result["episode_id"] == "e1", "missed_fallback"].iloc[0]) is True


def test_missed_fallback_false_when_already_non_normal_half_a_second_before_the_fall():
    episodes_df = pd.DataFrame([_episode("e1", "nis_gate", "push", fell=True, disturbance_onset=5.0)])
    steps_df = pd.DataFrame([
        _step("e1", 4.0, "NORMAL"),
        _step("e1", 4.5, "CAUTIOUS"),  # already caught 0.5s before the fall
        _step("e1", 4.9, "CAUTIOUS"),
        _step("e1", 5.0, "CAUTIOUS", fallen=True),
    ])

    result = missed_fallback(steps_df, episodes_df)

    assert bool(result.loc[result["episode_id"] == "e1", "missed_fallback"].iloc[0]) is False


def test_missed_fallback_only_includes_fallen_episodes():
    episodes_df = pd.DataFrame([
        _episode("e1", "nis_gate", "push", fell=True, disturbance_onset=5.0),
        _episode("e2", "nis_gate", "nominal", fell=False),
    ])
    steps_df = pd.DataFrame([
        _step("e1", 5.0, "NORMAL", fallen=True),
        _step("e2", 5.0, "NORMAL"),
    ])

    result = missed_fallback(steps_df, episodes_df)

    assert set(result["episode_id"]) == {"e1"}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_metrics.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'analysis'`

- [ ] **Step 3: Create `analysis/__init__.py` and `analysis/metrics.py`**

`analysis/__init__.py`: empty file.

`analysis/metrics.py`:

```python
"""Batch evaluation metrics (CLAUDE.md section 12). Operate on the
(steps_df, episodes_df) pair experiments/run_batch.py produces:
steps_df has one row per control tick (columns include at least
"episode_id", "t", "mode", "theta", "fallen"); episodes_df has one row per
episode ("episode_id", "controller", "scenario", "disturbance_onset"
(NaN for the nominal scenario), "fell").

Threshold-sensitivity sweeps (CLAUDE.md section 12's last metric) are not
computed here -- they're what quantum/qubo.py's grid search re-simulates
against (section 13), a later build-order step.
"""

import numpy as np
import pandas as pd

from sim.params import default_plant_params


def fall_rate(episodes_df: pd.DataFrame) -> pd.Series:
    """Fraction of episodes that fell, grouped by controller."""
    return episodes_df.groupby("controller")["fell"].mean()


def false_fallback_fraction(steps_df: pd.DataFrame, episodes_df: pd.DataFrame) -> pd.Series:
    """Fraction of disturbance-free ("nominal" scenario) time spent
    outside NORMAL, grouped by controller."""
    nominal_episodes = episodes_df.loc[episodes_df["scenario"] == "nominal", ["episode_id", "controller"]]
    merged = steps_df.merge(nominal_episodes, on="episode_id", how="inner")
    return merged.groupby("controller")["mode"].apply(lambda modes: (modes != "NORMAL").mean())


def missed_fallback(steps_df: pd.DataFrame, episodes_df: pd.DataFrame) -> pd.DataFrame:
    """Falls where the gate was still NORMAL 0.5s before the fall, one row
    per fallen episode."""
    rows = []
    for _, ep in episodes_df[episodes_df["fell"]].iterrows():
        ep_steps = steps_df[steps_df["episode_id"] == ep["episode_id"]].sort_values("t")
        fall_t = ep_steps.loc[ep_steps["fallen"], "t"].iloc[0]
        before = ep_steps[ep_steps["t"] <= fall_t - 0.5]
        was_normal = before.empty or before["mode"].iloc[-1] == "NORMAL"
        rows.append({"episode_id": ep["episode_id"], "controller": ep["controller"], "missed_fallback": bool(was_normal)})
    return pd.DataFrame(rows)


def detection_delay(steps_df: pd.DataFrame, episodes_df: pd.DataFrame) -> pd.DataFrame:
    """Disturbance onset -> first non-NORMAL tick, one row per episode
    with a known disturbance onset (NaN if never detected)."""
    rows = []
    disturbed = episodes_df[episodes_df["disturbance_onset"].notna()]
    for _, ep in disturbed.iterrows():
        ep_steps = steps_df[steps_df["episode_id"] == ep["episode_id"]].sort_values("t")
        after_onset = ep_steps[ep_steps["t"] >= ep["disturbance_onset"]]
        non_normal = after_onset[after_onset["mode"] != "NORMAL"]
        delay = (non_normal["t"].iloc[0] - ep["disturbance_onset"]) if not non_normal.empty else np.nan
        rows.append({
            "episode_id": ep["episode_id"], "controller": ep["controller"],
            "scenario": ep["scenario"], "detection_delay_s": delay,
        })
    return pd.DataFrame(rows)


def progress(steps_df: pd.DataFrame, episodes_df: pd.DataFrame) -> pd.DataFrame:
    """Distance travelled (R * delta-theta) per episode -- CLAUDE.md
    section 12: "distance travelled along the corridor (cost of
    caution)." """
    R = default_plant_params().R
    rows = []
    for episode_id, ep_steps in steps_df.groupby("episode_id"):
        ep_steps = ep_steps.sort_values("t")
        dist = R * (ep_steps["theta"].iloc[-1] - ep_steps["theta"].iloc[0])
        rows.append({"episode_id": episode_id, "progress_m": dist})
    result = pd.DataFrame(rows)
    return result.merge(episodes_df[["episode_id", "controller", "scenario"]], on="episode_id")


def summarize_batch(steps_df: pd.DataFrame, episodes_df: pd.DataFrame) -> pd.DataFrame:
    """One row per controller: fall rate, false-fallback fraction, mean
    detection delay, mean progress -- CLAUDE.md section 12's metrics
    table."""
    fr = fall_rate(episodes_df)
    ff = false_fallback_fraction(steps_df, episodes_df)
    dd = detection_delay(steps_df, episodes_df).groupby("controller")["detection_delay_s"].mean()
    pr = progress(steps_df, episodes_df).groupby("controller")["progress_m"].mean()
    return pd.DataFrame({
        "fall_rate": fr,
        "false_fallback_fraction": ff,
        "mean_detection_delay_s": dd,
        "mean_progress_m": pr,
    })
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_metrics.py -v`
Expected: all PASS.

- [ ] **Step 5: Commit**

```bash
git add analysis/__init__.py analysis/metrics.py tests/test_metrics.py
git commit -m "$(cat <<'EOF'
feat: add analysis/metrics.py (fall rate, false-fallback, missed-fallback, detection delay, progress)

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 7: `experiments/run_batch.py`

**Files:**
- Create: `experiments/__init__.py` (empty)
- Create: `experiments/run_batch.py`
- Test: `tests/test_run_batch.py`

Two separate batches (per this plan's "Design decision 1/3"): **balance-only** (`wall_following=False`) across all 7 scenarios (nominal + 6 disturbances) for the core detection metrics, and **corridor** (`wall_following=True`) on the nominal scenario only for the Progress metric.

- [ ] **Step 1: Write the failing test**

Create `tests/test_run_batch.py`:

```python
import pandas as pd

from experiments.run_batch import DISTURBANCE_SCENARIOS, run_batch
from sim.run import ControllerType


def test_run_batch_balance_only_smoke_test(tmp_path, monkeypatch):
    """Short T and few seeds -- this is a wiring/schema smoke test, not a
    calibration check (those live in tests/test_run.py)."""
    monkeypatch.setattr("experiments.run_batch.SEEDS", [0, 1])
    monkeypatch.setattr("experiments.run_batch.CONTROLLERS", [ControllerType.NAIVE, ControllerType.NIS_GATE])
    # Shrink every scenario's duration so the smoke test runs fast.
    short_scenarios = {
        name: (fn, 1.0, onset, x0) for name, (fn, _T, onset, x0) in DISTURBANCE_SCENARIOS.items()
    }
    monkeypatch.setattr("experiments.run_batch.DISTURBANCE_SCENARIOS", short_scenarios)

    steps_df, episodes_df = run_batch(wall_following=False, scenario_names=["nominal", "push"])

    assert isinstance(steps_df, pd.DataFrame)
    assert isinstance(episodes_df, pd.DataFrame)
    assert len(episodes_df) == 2 * 2 * 2  # 2 controllers x 2 scenarios x 2 seeds
    assert set(episodes_df["episode_id"]) == set(steps_df["episode_id"].unique())
    assert {"episode_id", "controller", "scenario", "seed", "wall_following", "disturbance_onset", "fell"} <= set(episodes_df.columns)


def test_run_batch_writes_readable_parquet(tmp_path, monkeypatch):
    monkeypatch.setattr("experiments.run_batch.SEEDS", [0])
    monkeypatch.setattr("experiments.run_batch.CONTROLLERS", [ControllerType.NAIVE])
    short_scenarios = {"nominal": (None, 1.0, None, 0.0)}
    monkeypatch.setattr("experiments.run_batch.DISTURBANCE_SCENARIOS", short_scenarios)
    monkeypatch.setattr("experiments.run_batch.RESULTS_DIR", tmp_path)

    from experiments.run_batch import main
    main()

    steps = pd.read_parquet(tmp_path / "steps_balance.parquet")
    episodes = pd.read_parquet(tmp_path / "episodes_balance.parquet")
    assert len(episodes) == 1
    assert len(steps) == int(round(1.0 / 0.005))
    steps_cor = pd.read_parquet(tmp_path / "steps_corridor.parquet")
    episodes_cor = pd.read_parquet(tmp_path / "episodes_corridor.parquet")
    assert len(episodes_cor) == 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_run_batch.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'experiments'`

- [ ] **Step 3: Create `experiments/__init__.py` and `experiments/run_batch.py`**

`experiments/__init__.py`: empty file.

`experiments/run_batch.py`:

```python
"""Batch evaluation across the three CLAUDE.md section 12 controllers.
Produces two Parquet outputs per batch under experiments/results/:
<name>.parquet (one row per episode, summary metadata) and
steps_<name>.parquet (per-control-tick log, tagged by episode_id).

Two separate batches -- see docs/superpowers/plans/2026-09-22-run-batch-
metrics-step6.md "Design decision 1/3":
- balance-only (wall_following=False): all 7 scenarios (nominal + 6
  disturbances) -- the core NIS-detection metrics, reproducing the
  already-calibrated step 3/4 architecture exactly.
- corridor (wall_following=True, corridor_theta_dot_ref_nominal=0.3): the
  nominal scenario only -- the Progress metric.
"""
from pathlib import Path

import pandas as pd
from joblib import Parallel, delayed

from sim.disturbances import (
    battery_droop_v_batt,
    payload_shift_plant_params,
    push_psi_dot_kick,
    sensor_fault_accel_noise_params,
    sensor_fault_gyro_bias_step,
    surface_change_plant_params,
)
from sim.params import default_disturbance_params
from sim.run import ControllerType, EpisodeConfig, run_episode

RESULTS_DIR = Path(__file__).resolve().parent / "results"
CONTROLLERS = [ControllerType.NAIVE, ControllerType.TILT_THRESHOLD, ControllerType.NIS_GATE]
SEEDS = list(range(10))
DP = default_disturbance_params()


def _push(t, x, bg, p, sp):
    x = x.copy()
    x[4] = push_psi_dot_kick(x[4], t, DP.push_onset, 0.005, DP.push_magnitude)
    return x, bg, p, sp, DP.battery_droop_v_nominal


def _surface_change(t, x, bg, p, sp):
    return x, bg, surface_change_plant_params(
        t, DP.surface_change_onset, DP.surface_change_duration, DP.surface_change_fw, p
    ), sp, DP.battery_droop_v_nominal


def _gyro_bias_fault(t, x, bg, p, sp):
    return x, sensor_fault_gyro_bias_step(
        bg, t, DP.gyro_bias_fault_onset, 0.005, DP.gyro_bias_fault_magnitude
    ), p, sp, DP.battery_droop_v_nominal


def _accel_noise_fault(t, x, bg, p, sp):
    return x, bg, p, sensor_fault_accel_noise_params(
        t, DP.accel_noise_fault_onset, DP.accel_noise_fault_duration, DP.accel_noise_fault_multiplier, sp
    ), DP.battery_droop_v_nominal


_BATTERY_PUSH_ONSET = DP.battery_droop_onset + DP.battery_droop_duration


def _battery_droop(t, x, bg, p, sp):
    v = battery_droop_v_batt(
        t, DP.battery_droop_onset, DP.battery_droop_duration, DP.battery_droop_v_nominal, DP.battery_droop_v_drooped
    )
    x = x.copy()
    x[4] = push_psi_dot_kick(x[4], t, _BATTERY_PUSH_ONSET, 0.005, DP.battery_droop_companion_push_magnitude)
    return x, bg, p, sp, v


def _payload_shift(t, x, bg, p, sp):
    return x, bg, payload_shift_plant_params(
        t, DP.payload_shift_onset, DP.payload_shift_delta_M, DP.payload_shift_delta_L, p
    ), sp, DP.battery_droop_v_nominal


# scenario name -> (disturbance_fn or None, T, disturbance_onset or None, x0_psi_deg)
DISTURBANCE_SCENARIOS = {
    "nominal": (None, 60.0, None, 0.0),
    "push": (_push, 20.0, DP.push_onset, 0.0),
    "surface_change": (_surface_change, 20.0, DP.surface_change_onset, 0.0),
    "gyro_bias_fault": (_gyro_bias_fault, 20.0, DP.gyro_bias_fault_onset, 0.0),
    "accel_noise_fault": (_accel_noise_fault, 20.0, DP.accel_noise_fault_onset, 0.0),
    "battery_droop": (_battery_droop, 20.0, _BATTERY_PUSH_ONSET, 0.0),
    "payload_shift": (_payload_shift, 20.0, DP.payload_shift_onset, DP.payload_shift_test_psi0_deg),
}


def _run_one(controller, scenario_name, seed, wall_following):
    fn, T, onset, x0_psi_deg = DISTURBANCE_SCENARIOS[scenario_name]
    config = EpisodeConfig(
        controller=controller, T=T, wall_following=wall_following,
        x0_psi_deg=x0_psi_deg, disturbance=fn, disturbance_name=scenario_name,
    )
    steps_df = run_episode(config, seed).copy()
    episode_id = f"{controller.value}_{scenario_name}_{seed}_{'corridor' if wall_following else 'balance'}"
    steps_df["episode_id"] = episode_id
    summary = {
        "episode_id": episode_id,
        "controller": controller.value,
        "scenario": scenario_name,
        "seed": seed,
        "wall_following": wall_following,
        "disturbance_onset": onset,
        "fell": bool(steps_df["fallen"].any()),
    }
    return steps_df, summary


def run_batch(wall_following: bool, scenario_names: list[str]) -> tuple[pd.DataFrame, pd.DataFrame]:
    jobs = [
        (controller, scenario_name, seed)
        for controller in CONTROLLERS
        for scenario_name in scenario_names
        for seed in SEEDS
    ]
    results = Parallel(n_jobs=-1)(
        delayed(_run_one)(controller, scenario_name, seed, wall_following)
        for controller, scenario_name, seed in jobs
    )
    steps_dfs, summaries = zip(*results)
    steps_df = pd.concat(steps_dfs, ignore_index=True)
    episodes_df = pd.DataFrame(summaries)
    return steps_df, episodes_df


def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    balance_scenarios = list(DISTURBANCE_SCENARIOS.keys())
    steps_bal, episodes_bal = run_batch(wall_following=False, scenario_names=balance_scenarios)
    steps_bal.to_parquet(RESULTS_DIR / "steps_balance.parquet")
    episodes_bal.to_parquet(RESULTS_DIR / "episodes_balance.parquet")

    steps_cor, episodes_cor = run_batch(wall_following=True, scenario_names=["nominal"])
    steps_cor.to_parquet(RESULTS_DIR / "steps_corridor.parquet")
    episodes_cor.to_parquet(RESULTS_DIR / "episodes_corridor.parquet")


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_run_batch.py -v`
Expected: PASS.

- [ ] **Step 5: Add `experiments/results/` to `.gitignore`**

Append to `.gitignore`:
```
experiments/results/
```

(Batch outputs are regenerable, potentially large Parquet files — matching the existing `nxtway_gs/`/`graphify-out/cache/` gitignore precedent, not committed.)

- [ ] **Step 6: Run the full suite**

Run: `uv run pytest -q`
Expected: all tests pass.

- [ ] **Step 7: Commit**

```bash
git add experiments/__init__.py experiments/run_batch.py tests/test_run_batch.py .gitignore
git commit -m "$(cat <<'EOF'
feat: add experiments/run_batch.py (balance-only + corridor batches, joblib parallel, Parquet output)

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```

---

## Task 8: Update `project-context/`

**Files:**
- Modify: `project-context/MODEL_HANDOFF.md`
- Modify: `project-context/CURRENT_STATE.md`
- Modify: `project-context/DECISIONS.md`
- Modify: `project-context/FAILED_APPROACHES.md`
- Modify: `project-context/OPEN_PROBLEMS.md`

- [ ] **Step 1: Run the full test suite one more time and record the actual count**

Run: `uv run pytest -q`
Record the exact `N passed` figure printed — use this real number (not a guess) everywhere below that says `<N>`.

- [ ] **Step 2: Append to `project-context/DECISIONS.md`**

```markdown
---

### Decision: `sim/run.py` unifies the three closed-loop harnesses via an `EpisodeConfig.wall_following` flag, not a single shared control law
- **What:** `run_episode` uses `design_lqr_balance` (KF-fed) for every episode where `wall_following=False`, exactly reproducing `tests/test_gate.py`/`tests/test_estimator.py`'s already-calibrated algorithm. `wall_following=True` episodes use `design_lqr_speed_servo` (true-state-fed, matching step 5's precedent), required regardless since `design_lqr_balance` cannot track a nonzero speed reference.
- **Reason:** Measured that switching the balance/gate control law to `design_lqr_speed_servo` (either KF-fed or true-state-fed, at `theta_dot_ref=0` or nonzero) erodes the push-disturbance detection margin below `tau1=1300` in every variant tried (1344.87 for the existing `design_lqr_balance`, vs. 1111-1187 for every speed-servo variant) — `GateParams` was calibrated against `design_lqr_balance` specifically and does not transfer.
- **Alternatives considered:** a single shared `design_lqr_speed_servo` control law everywhere (rejected — breaks push/battery-droop detection, would require a full GateParams recalibration, a materially larger undertaking than this step).
- **Date:** 2026-09-22 (`docs/superpowers/plans/2026-09-22-run-batch-metrics-step6.md`).
- **Still current:** Yes.

---

### Decision: corridor episodes default to `corridor_theta_dot_ref_nominal=0.3` rad/s, not `tests/test_wall_following.py`'s `2.0`
- **What:** `RunParams.corridor_theta_dot_ref_nominal=0.3`.
- **Reason:** With the KF/gate actually watching a real wall-following (yaw-active) closed loop for the first time, `2.0` rad/s produces `max_epsilon=1605.56` over a nominal 60s run (exceeds `tau1=1300`, spurious CAUTIOUS). Swept `0.1/0.2/0.3/0.5`: `0.1` diverges/falls (`max_epsilon` reaches `~2.1e8` without a fall-check), `0.2`→689, `0.3`→909 (comparable to the balance-only nominal range of 823-951), `0.5`→1605 (unsafe). `0.3` was chosen as the best-margined safe value.
- **Alternatives considered:** `2.0` (rejected, unsafe once the gate is watching); `0.5` (rejected, exceeds `tau1`); `0.1`/lower (rejected — the relationship is not monotonic in speed, and `0.1` specifically destabilizes the closed loop entirely, not fully explained here).
- **Date:** 2026-09-22.
- **Still current:** Yes.

---

### Decision: `default_speed_servo_params_cautious()` divides every `SpeedServoParams` Q weight by 5
- **What:** CAUTIOUS mode's "softer Q" gain set (CLAUDE.md section 10).
- **Reason:** Reducing `Q_psi` alone (the dominant weight) barely changes `K` (`||K||` ratio 0.998) since it already dominates the other weights by 3 orders of magnitude. Dividing all five weights by 5 (mathematically equivalent to `R x5`) gives a genuinely gentler gain — verified stable, dominant pole `-0.263` vs nominal `-0.394` (~33% slower), `||K||` ratio 0.967.
- **Date:** 2026-09-22.
- **Still current:** Yes.

---

### Decision: HALT mode uses the nominal (not cautious) speed-servo gain
- **What:** Only CAUTIOUS uses `default_speed_servo_params_cautious()`; HALT sets `theta_dot_ref=0` but keeps the nominal gain.
- **Reason:** CLAUDE.md section 10 states "softer Q" only for CAUTIOUS, not HALT — a literal reading, consistent with prior steps' approach to ambiguous spec clauses (see the "NORMAL->HALT skip" and "push is a velocity kick" decisions above).
- **Date:** 2026-09-22.
- **Still current:** Yes.
```

- [ ] **Step 3: Append to `project-context/FAILED_APPROACHES.md`**

```markdown
---

### Failed: using `design_lqr_speed_servo` (instead of `design_lqr_balance`) as a single shared control law for every `sim/run.py` episode
- **Approach tried:** Feed the KF estimate (or, separately, the true state) into `design_lqr_speed_servo` with `theta_dot_ref=0` for balance-in-place episodes, on the reasoning that a constant zero reference reduces exactly to the already-verified nominal design.
- **What happened:** Nominal-run epsilon behavior was fine in every variant (comparable to the existing `design_lqr_balance` range), but push-disturbance detection broke in *all four* tested variants (KF-fed/true-state-fed x `theta_dot_ref` 0/nonzero): max epsilon in the push-response window ranged 1111-1187, all below `tau1=1300`, versus `design_lqr_balance`'s own (already-thin) 1344.87.
- **Why abandoned:** `GateParams.tau1`/`tau2` were calibrated specifically against `design_lqr_balance`'s closed-loop response; the speed-servo gain's slightly faster damping of the psi_dot-kick response is enough to drop the windowed-sum epsilon peak below `tau1`, and there was no clean way to keep both control-law unification and the existing calibration.
- **What should not be repeated:** Don't unify `sim/run.py`'s control law onto `design_lqr_speed_servo` "for cleanliness" without re-verifying push/battery-droop detection against the new gain — the margin is thin enough that even a modest gain change tips it.
- **Source:** `docs/superpowers/plans/2026-09-22-run-batch-metrics-step6.md`, "Design decision 1."

---

### Failed: `tests/test_wall_following.py`'s `theta_dot_ref_nominal=2.0` as the corridor evaluation speed
- **Approach tried:** Reuse the existing wall-following demo's forward speed (`2.0` rad/s) for `sim/run.py`'s corridor episodes, now with the KF/gate actually observing the loop.
- **What happened:** A nominal (undisturbed) 60s run at `2.0` rad/s reaches `max_epsilon=1605.56`, exceeding `tau1=1300` and spuriously leaving NORMAL — a false-fallback purely from sustained forward motion plus active yaw correction, not any injected disturbance.
- **Why abandoned:** `2.0` rad/s was originally chosen in step 5 purely for a fast kinematic demo with no KF/gate in that loop at all; it was never validated against the gate's calibration. `0.3` rad/s (swept alongside `0.1/0.2/0.5`) keeps `max_epsilon` at `909.13`, comparable to the balance-only nominal range.
- **What should not be repeated:** Don't assume a control/kinematics-only demo's chosen speed transfers to a configuration that also includes the estimator/gate — sustained nonzero `theta_dot` was never exercised against `GateParams` before this step.
- **Source:** same plan doc, "Design decision 3."
```

- [ ] **Step 4: Update `project-context/OPEN_PROBLEMS.md`**

Move the existing "Suspected problems" entry (the one starting `~~Encoder quantize-then-average...~~ **Resolved, stale as of 2026-09-21.**`) — replace its "What's still genuinely open" sentence and promote it into "Confirmed problems," and remove the now-resolved "Whether/how `sim/run.py` (step 6) should unify..." entry from "Unknowns" (it's answered by this step). Apply this diff:

In "Confirmed problems / gaps", add:

```markdown
- **Sustained nonzero `theta_dot` (forward motion) with active yaw correction significantly inflates the windowed NIS statistic epsilon_k, confirming the encoder-per-wheel-quantization-under-yaw suspicion this file previously flagged as untested.** A nominal (undisturbed) 60s wall-following run at `theta_dot_ref=2.0` rad/s reaches `max_epsilon=1605.56`, exceeding `tau1=1300` — a false-fallback from motion alone. Straight-line-only motion (no yaw) at the same speed only reaches `max_epsilon≈2465` at `theta_dot_ref=2.0` too, but yaw correction makes lower speeds unsafe as well (e.g. `0.5` rad/s: straight-line-only was comfortably safe in isolation, but with yaw active the safe ceiling drops well below that). `sim/run.py`'s corridor episodes use `theta_dot_ref_nominal=0.3` (verified: `max_epsilon=909.13`) specifically because of this. Root cause not fully characterized (plausible: `EstimatorParams.Q_theta=1e-8` is far too small to absorb the apparent process noise from per-wheel encoder quantization sweeping through many bins per second at speed) — fixing this (e.g. a `theta_dot`-dependent `Q_theta`, or an average-then-quantize encoder redesign) is out of scope for step 6.
- **A very low nonzero corridor speed (`theta_dot_ref_nominal=0.1` rad/s) destabilizes the closed loop entirely** (observed epsilon reaching `~2.1e8` without a fall-check present, almost certainly an undetected fall) — the relationship between corridor speed and stability/false-fallback is not monotonic. Not investigated further; `sim/run.py`'s fall condition (CLAUDE.md section 5) exists specifically so this terminates cleanly (`fallen=True`) instead of producing meaningless downstream numbers.
```

Update the encoder-quantization entry (currently under "Suspected problems") to read:

```markdown
- **Encoder quantize-then-average vs. average-then-quantize matters under real yaw motion, confirmed.** `tests/test_sensors.py::test_encoder_quantizes_per_wheel_before_averaging` already confirmed the per-wheel reading behaves as intended in a hand-constructed unit case; step 6's `sim/run.py` corridor episodes are the first *real closed-loop* exercise of this under active yaw control, and the interaction is large enough to be the leading suspected cause of the sustained-motion false-fallback finding above (see "Confirmed problems").
```

Remove the "Whether/how `sim/run.py` (step 6) should unify..." bullet from "Unknowns" entirely — it's resolved (see `DECISIONS.md`).

- [ ] **Step 5: Update `project-context/CURRENT_STATE.md`**

Update the header date/commit, move step 6 from "What does not exist yet" into "What currently works," using the real test count from Step 1 above (replace `<N>` with the actual number), and describe: `sim/run.py` (`run_episode`, `EpisodeConfig`, `ControllerType`, both control-law paths, fall condition), `sim/gate.py`'s `step_tilt_gate` addition, `sim/params.py`'s `RunParams`/`TiltGateParams`/cautious speed-servo params, `experiments/run_batch.py`, `analysis/metrics.py`. State plainly the two confirmed findings from Task 8 Step 4 (corridor false-fallback at speed, push/battery-droop's thin/broken margin under speed-servo) as real, load-bearing results of this step, not bugs.

- [ ] **Step 6: Rewrite `project-context/MODEL_HANDOFF.md`**

Update "Last updated" date/commit. Update "Current state in one paragraph" to describe Milestone 2 as achieved (unified `sim/run.py`, three controllers, batch evaluation, metrics). Add step 6's key decisions/findings (Task 8 Steps 2-4, condensed) to "Decisions you must not silently re-litigate" and "Current problems." Update "What the next model should do" to point at build-order step 7 (`quantum/qubo.py` — CLAUDE.md section 13, still blocked on verifying `qiskit`/`qiskit-optimization` imports per the existing `DECISIONS.md` entry) or step 8 (`analysis/plots.py`/`analysis/animate.py`) per the user's next priority.

- [ ] **Step 7: Run graphify update**

Run: `graphify update .`
Expected: graph refreshed to include the new/changed modules (`sim/run.py`, `analysis/`, `experiments/`, `sim/gate.py`/`sim/params.py` additions).

- [ ] **Step 8: Final full-suite verification**

Run: `uv run pytest -q`
Expected: all `<N>` tests pass (same count recorded in Step 1 — this task only touches docs + the graphify cache, not code).

- [ ] **Step 9: Commit**

```bash
git add project-context/ graphify-out/
git commit -m "$(cat <<'EOF'
docs: update project-context for step 6 completion (run.py unification, batch evaluation, metrics)

Co-Authored-By: Claude Sonnet 5 <noreply@anthropic.com>
EOF
)"
```
