# Echo Balancer — Section 14 Step 4: Gate, Disturbances Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `sim/gate.py` (the Normal/Cautious/Halt state machine on the windowed NIS statistic ε_k — the project's actual contribution) and `sim/disturbances.py` (the five CLAUDE.md section 8 disturbance profiles), and pass the section 11 gate tests, achieving build-order **Milestone 1: gate switches under one disturbance** (and, per section 11's literal wording, under each of the five).

**Architecture:** `sim/gate.py` exposes a pure `step_gate(state, nis_k, dt, gate_p) -> (new_state, epsilon_k)` function (explicit state threading, matching `sim/estimator.py::predict`/`update`'s style) implementing the windowed-sum statistic and the hysteresis/dwell-gated mode machine. `sim/disturbances.py` exposes one pure function per disturbance profile — each takes the current time plus onset/duration/magnitude and returns the *disturbed* plant/sensor params (or a one-shot state kick) for the caller (a test harness today, `sim/run.py` later) to substitute in place of the nominal values. The estimator's own `Ad`/`Bd`/`Q`/`R` are always built from nominal params — disturbances never touch `sim/estimator.py` or `sim/plant.py` themselves.

**Tech Stack:** Python ≥3.11, `numpy`, `scipy` (`scipy.stats` — currently unused by the plan's checked-in code but note the option for future gate-tuning work).

**Explicitly out of scope for this plan:** `sim/world.py`, `sim/run.py`, `experiments/`, `analysis/`, `quantum/`, and everything past section 14 build-order step 4. The closed-loop harness built in this plan's tests is test-only (mirrors `tests/test_estimator.py::_run_nominal_closed_loop`'s established pattern from step 3) — it is explicitly *not* a preview of `sim/run.py`'s eventual design, same caveat as before.

---

## Pre-verified design: this took by far the most empirical exploration of any step so far

CLAUDE.md itself doesn't give numeric targets for `GateParams` (τ1, τ2, N, T_dwell) or disturbance magnitudes — same situation as `EstimatorParams` in step 3, except this time the exploration surfaced a **provable**, not just empirical, conflict with the spec's literal wording. Everything below was run against this repo's actual `sim/plant.py`/`sim/control.py`/`sim/estimator.py`/`sim/sensors.py` before writing this plan.

### Design decision: τ1/τ2 cannot be literal χ²(3N) quantiles, at any quantile level

CLAUDE.md section 10: *"Transitions on ε_k with thresholds τ₁ < τ₂ (as χ²(3N) quantiles)."*

First attempt: pick "reasonable" quantiles (0.95 / 0.999) of `chi2(3N)`. Result: a **nominal, undisturbed** 60s run immediately left NORMAL and reached HALT (max ε_k = 229.7 against a τ2 of 173.6 at N=40). This is the same "correlated NIS samples" phenomenon already documented for the step-3 NIS test, but here it's worse: I swept N up to 200 (chi2(600), a 1-second window) and even 800/1200, and in every case the empirical nominal-run maximum of ε_k *exceeds even the χ²(3N) 0.999999 quantile*:

| N | window | χ²(3N) 0.999999 quantile | empirical max ε_k over 10 independent 60s seeded runs |
|---|---|---|---|
| 200 | 1.0 s | 779.3 | 959.1 |
| 400 | 2.0 s | (not needed — trend already clear) | 1712.9 |
| 800 | 4.0 s | | 3135.6 |
| 1200 | 6.0 s | | 4508.2 |

At N=200 specifically: `chi2(600).cdf(959.1) ≈ 1.0` to floating-point precision — there is *no* quantile level, however extreme, at which "τ1/τ2 as χ²(3N) quantiles" reproduces a threshold above the observed nominal maximum. Root cause: the closed-loop system (from step 2) has a slow θ-position pole at ≈ −0.097 rad/s (≈10s time constant, since no speed-servo integral action exists yet — CLAUDE.md section 9 describes one, explicitly out of scope through step 2). This makes the NIS sequence's *mean* wander slowly (rolling 1-second-window mean NIS ranges from ~1.8 to ~4.5 across one 60s run, not tightly concentrated around 3), which the i.i.d.-samples assumption behind "sum of N chi2(3) ~ chi2(3N)" cannot capture, at any N.

**Decision:** `GateParams.tau1`/`tau2` (and their hysteresis-exit counterparts) are empirically calibrated directly against real simulated nominal runs, with margin, exactly like `EstimatorParams` in step 3 — not derived from a chi2 quantile formula. Chosen: **N=200** (1s window — long enough to average out sensor noise, short enough for sub-second disturbance detection), **τ1=1300** (~35% margin above the observed 10-seed nominal max of 959.1), **τ2=1800**, **τ1_exit=1040** (80% of τ1), **τ2_exit=1300** (= τ1, i.e. HALT relaxes back to CAUTIOUS at the same level CAUTIOUS would have entered), **T_dwell=0.5s**. Verified: 10 independent 60s seeded nominal runs (seeds 0-9) never leave NORMAL under these thresholds; the checked-in test uses seed=42 specifically (also verified clean). If you disagree with this resolution and want to pursue literal χ²(3N) quantiles anyway, you would need to first fix the underlying slow-pole/correlated-NIS issue (e.g. add the speed-servo integral term from section 9) — that's a materially larger undertaking than this build step, so it's flagged here rather than attempted.

### Design decision: the NORMAL→HALT "skip" exception is read as "window not yet full", not "sustained over a separate timer"

CLAUDE.md: *"No NORMAL → HALT skip unless ε_k > τ₂ for a full window."* Two readings are possible: (a) ε_k (already a sum over the trailing N samples) exceeding τ2 *is itself* "a full window" of evidence, so a direct NORMAL→HALT transition is allowed exactly whenever the window is fully populated (not still in its N-sample warm-up period) and ε_k > τ2; or (b) some *additional* temporal-persistence timer separate from the window itself. Reading (a) is implemented: it's simpler, requires no extra state beyond what `step_gate` already tracks, and is a faithful literal reading (ε_k *is* "a window's" worth of evidence by construction). Verified: an extreme disturbance (e.g. a 0.2 rad/s push) does skip straight from NORMAL to HALT in one step once the window is full, exactly as intended; smaller disturbances (e.g. a 0.1 rad/s push) correctly land on CAUTIOUS first.

### Disturbance magnitudes — five profiles, each independently verified, two with genuine surprises

Each below reports the **working, checked-in** magnitude plus the actual detection latency observed (all comfortably under the chosen `detection_window_s=2.0` bound), verified with the plant/estimator/control code exactly as it exists in this repo (seed=42):

1. **Push** (impulse torque on ψ, modeled as an instantaneous ψ̇ kick — see "Design decision: push is a velocity kick" below): `magnitude=0.1 rad/s` → CAUTIOUS at **+0.030s**. Clean: no numerical issues, doesn't skip to HALT. (Kicks ≥0.2 rad/s *do* skip straight to HALT — consistent with the "skip" design above, not a bug.)
2. **Surface change** (`f_w`: 0→magnitude): `magnitude=0.025` → CAUTIOUS at **+1.225s**. **Important finding: `f_w ≥ 0.03` causes the closed-loop nonlinear simulation to genuinely diverge (`NaN`/`inf`, verified via explicit finiteness checks, not just "falls over")** — the current controller has a real, fairly narrow stability margin against added wheel-floor friction. `0.025` was chosen with a safety margin below this instability boundary, not merely because it "works" — going much closer to `0.03` is not a case of "loosen the tolerance," it's approaching a genuine stability limit worth knowing about for the write-up.
3. **Sensor fault, gyro bias step**: `magnitude=0.05 rad/s` → CAUTIOUS at **+0.395s**.
4. **Sensor fault, accelerometer noise ×k**: `multiplier=5.0` → CAUTIOUS at **+0.110s**.
5. **Payload/CoM shift** (`M`,`L` shift, plant only): `delta_M=1.0 kg, delta_L=0.1 m` → CAUTIOUS at **+0.450s**, *but only when combined with a mild 2° initial tilt* (a near-perfectly-balanced robot doesn't accelerate enough to expose an inertia mismatch). **Finding: substantially larger, still-plausible shifts (`delta_M` up to 0.5kg ≈ 83% of body mass, `delta_L` up to 0.05m) never triggered detection at all**, even combined with tilt — the current NIS-based detector is measurably less sensitive to smooth mass/CoM changes than to the other four disturbance types, which all involve a more abrupt or higher-frequency signature. This is a real, reportable characteristic of the current Q/R tuning, not a bug to chase.
6. **Battery droop** (`V_batt` ramps down, requires implementing voltage clipping — see below): `v_drooped=0.1V` (from nominal 7.4V) **alone, near equilibrium, never triggers detection at all — even at v_drooped=0.2V with a 5° tilt** — nominal voltage demand near equilibrium is far below even a badly drooped battery's ceiling, so clipping never binds. Working test: pair the droop with a small companion push (`magnitude=0.05 rad/s`, itself independently verified to be **too small to trigger detection on its own**), fired *after* the droop's ramp finishes (`battery_droop_onset + battery_droop_duration`, not at the droop's own onset — an earlier draft of this check fired the push at the droop's onset, while `V_batt` is still ramping and thus ~nominal at that exact instant, which does not trigger detection; caught and fixed during Task 5's implementation) → CAUTIOUS at **+1.055s after the push** (**+2.055s after the droop's own onset**). This demonstrates battery droop's actual failure mode correctly: it doesn't matter in isolation, it degrades the controller's ability to correct *other* disturbances.

### Design decision: push is a ψ̇ velocity kick, not a torque impulse requiring an inertia conversion

CLAUDE.md section 8 calls this "impulse torque on ψ." Modeling it as an instantaneous jump in ψ̇ (rather than computing an angular impulse `J` and converting via `Δψ̇ = J/I_eff`) is exact by the impulse-momentum theorem for *some* implied `J`, and avoids inventing an "effective inertia" constant with no clear source in the spec. Sized directly in rad/s.

### Design decision: voltage clipping (`clip_voltage`, `V_batt`) lands in `sim/disturbances.py`, not `sim/plant.py`

CLAUDE.md section 5 describes `v = clip(u_cmd, ±V_batt)`, but section 8 explicitly says *"V_batt is a state of the disturbance model."* Since `V_batt` only ever varies via the battery-droop disturbance, and `sim/plant.py` is already fully tested/approved and takes already-clipped `v_l, v_r` directly (no signature change needed), the clipping utility belongs with the disturbance model that owns `V_batt`, not inside `plant.py`. This keeps `plant.py` completely untouched by this step.

---

## Task 1: `sim/params.py` — `GateParams` and `DisturbanceParams`

**Files:**
- Modify: `sim/params.py` (append; do not change any existing class/function)
- Test: `tests/test_params.py` (append)

- [ ] **Step 1: Write the failing test**

Append to `tests/test_params.py`:

```python
from sim.params import (
    DisturbanceParams,
    GateParams,
    default_disturbance_params,
    default_gate_params,
)


def test_default_gate_params():
    gp = default_gate_params()

    assert gp.N == 200
    assert gp.tau1 == 1300.0
    assert gp.tau2 == 1800.0
    assert gp.tau1_exit == 1040.0
    assert gp.tau2_exit == 1300.0
    assert gp.T_dwell == 0.5


def test_gate_params_is_frozen():
    gp = default_gate_params()
    try:
        gp.tau1 = 1.0
        assert False, "GateParams should be frozen"
    except AttributeError:
        pass


def test_default_disturbance_params():
    dp = default_disturbance_params()

    assert dp.surface_change_onset == 5.0
    assert dp.surface_change_duration == 10.0
    assert dp.surface_change_fw == 0.025
    assert dp.battery_droop_onset == 5.0
    assert dp.battery_droop_duration == 1.0
    assert dp.battery_droop_v_nominal == 7.4
    assert dp.battery_droop_v_drooped == 0.1
    assert dp.battery_droop_companion_push_magnitude == 0.05
    assert dp.push_onset == 5.0
    assert dp.push_magnitude == 0.1
    assert dp.gyro_bias_fault_onset == 5.0
    assert dp.gyro_bias_fault_magnitude == 0.05
    assert dp.accel_noise_fault_onset == 5.0
    assert dp.accel_noise_fault_duration == 10.0
    assert dp.accel_noise_fault_multiplier == 5.0
    assert dp.payload_shift_onset == 5.0
    assert dp.payload_shift_delta_M == 1.0
    assert dp.payload_shift_delta_L == 0.1
    assert dp.payload_shift_test_psi0_deg == 2.0
    assert dp.detection_window_s == 2.0


def test_disturbance_params_is_frozen():
    dp = default_disturbance_params()
    try:
        dp.push_magnitude = 1.0
        assert False, "DisturbanceParams should be frozen"
    except AttributeError:
        pass


def test_gate_params_ordering_invariants():
    """CLAUDE.md section 10: tau1 < tau2, with hysteresis exits below their
    entries. A typo swapping tau1_exit/tau2_exit (or similar) would pass
    every other test in this file and only surface as a mysteriously
    broken 60-second closed-loop gate test -- so pin the ordering directly."""
    gp = default_gate_params()

    assert gp.tau1 < gp.tau2
    assert gp.tau1_exit < gp.tau1
    assert gp.tau2_exit < gp.tau2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_params.py -v`
Expected: FAIL with `ImportError: cannot import name 'GateParams' from 'sim.params'`

- [ ] **Step 3: Append to `sim/params.py`**

```python
@dataclass(frozen=True)
class GateParams:
    """Normal/Cautious/Halt gate tuning (CLAUDE.md section 10) for the
    windowed statistic epsilon_k = sum of the last N NIS samples.

    CLAUDE.md says tau1 < tau2 should be "chi2(3N) quantiles." Empirically
    (see docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md),
    this is not achievable for this closed loop: even chi2(3*200=600)'s
    0.999999 quantile (~779) is below the observed nominal-run maximum of
    epsilon_k (~959, across 10 independent 60s seeded runs) -- successive
    NIS samples here are correlated (KF smoothing plus a slow ~10s
    theta-position closed-loop pole), so the window sum's tail is far
    heavier than the i.i.d. chi2(3N) model predicts, at any quantile
    level. tau1/tau2 are therefore calibrated directly against real
    simulated nominal runs with margin, not derived from a chi2 quantile
    formula.
    """

    N: int              # window size (# of 5ms samples); 200 = 1s window
    tau1: float          # CAUTIOUS entry (and NORMAL->HALT skip threshold)
    tau2: float          # HALT entry
    tau1_exit: float     # CAUTIOUS -> NORMAL hysteresis exit (< tau1)
    tau2_exit: float     # HALT -> CAUTIOUS hysteresis exit (< tau2)
    T_dwell: float       # seconds, minimum time in a mode before any transition


def default_gate_params() -> GateParams:
    return GateParams(N=200, tau1=1300.0, tau2=1800.0, tau1_exit=1040.0, tau2_exit=1300.0, T_dwell=0.5)


@dataclass(frozen=True)
class DisturbanceParams:
    """Onset/duration/magnitude for each CLAUDE.md section 8 disturbance.
    See docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md for
    the full empirical derivation. Load-bearing caveats that matter if
    you're tuning these, not just reading them:

    - surface_change_fw=0.025 sits close to a genuine plant instability:
      f_w >= ~0.03 causes the closed-loop nonlinear simulation to diverge
      (NaN/inf), not just "fall over." Don't raise this without margin.
    - payload_shift and battery_droop do NOT trigger gate detection in
      isolation near equilibrium -- payload_shift needs a concurrent mild
      tilt (payload_shift_test_psi0_deg) to expose the mass/CoM mismatch
      at all, and even much larger, more "realistic" shifts (delta_M up
      to 0.5kg, delta_L up to 0.05m) never trigger detection even with
      tilt; battery_droop needs a concurrent voltage demand
      (battery_droop_companion_push_magnitude) since nominal near-
      equilibrium commands never approach even a badly drooped ceiling.
      These are genuine sensitivity limits of the current NIS/Q/R tuning,
      not bugs.
    - battery_droop_companion_push_magnitude and payload_shift_test_psi0_deg
      are test-harness setup values (what it takes to make the disturbance
      observable at all), NOT part of the disturbance's own physical
      profile per CLAUDE.md section 8. Don't feed them into section 12's
      cross-controller evaluation as if they were part of the canonical
      battery-droop/payload-shift scenario -- that would silently give
      those two disturbances an extra "helper" perturbation the other
      three don't get, undermining "controllers compared on identical
      disturbance seeds."
    """

    surface_change_onset: float
    surface_change_duration: float
    surface_change_fw: float

    battery_droop_onset: float
    battery_droop_duration: float
    battery_droop_v_nominal: float
    battery_droop_v_drooped: float
    battery_droop_companion_push_magnitude: float  # test-harness setup, not part of the disturbance profile itself -- see class docstring

    push_onset: float
    push_magnitude: float

    gyro_bias_fault_onset: float
    gyro_bias_fault_magnitude: float

    accel_noise_fault_onset: float
    accel_noise_fault_duration: float
    accel_noise_fault_multiplier: float

    payload_shift_onset: float
    payload_shift_delta_M: float
    payload_shift_delta_L: float
    payload_shift_test_psi0_deg: float  # test-harness setup, not part of the disturbance profile itself -- see class docstring

    detection_window_s: float


def default_disturbance_params() -> DisturbanceParams:
    return DisturbanceParams(
        surface_change_onset=5.0,
        surface_change_duration=10.0,
        surface_change_fw=0.025,
        battery_droop_onset=5.0,
        battery_droop_duration=1.0,
        battery_droop_v_nominal=7.4,
        battery_droop_v_drooped=0.1,
        battery_droop_companion_push_magnitude=0.05,
        push_onset=5.0,
        push_magnitude=0.1,
        gyro_bias_fault_onset=5.0,
        gyro_bias_fault_magnitude=0.05,
        accel_noise_fault_onset=5.0,
        accel_noise_fault_duration=10.0,
        accel_noise_fault_multiplier=5.0,
        payload_shift_onset=5.0,
        payload_shift_delta_M=1.0,
        payload_shift_delta_L=0.1,
        payload_shift_test_psi0_deg=2.0,
        detection_window_s=2.0,
    )
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_params.py -v`
Expected: PASS (13 tests: 8 pre-existing + 5 new)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 39 tests: 34 pre-existing + 5 new), no regressions

- [ ] **Step 6: Commit**

```bash
git add sim/params.py tests/test_params.py
git commit -m "feat: add GateParams and DisturbanceParams (CLAUDE.md sec 8, 10)"
```

---

## Task 2: `sim/gate.py` — Normal/Cautious/Halt state machine

**Files:**
- Create: `sim/gate.py`
- Test: `tests/test_gate.py` (this task adds unit tests on the state machine logic; later tasks append closed-loop integration tests to the same file)

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_gate.py
from sim.gate import GateMode, GateState, initial_gate_state, step_gate
from sim.params import GateParams


def _toy_gate_params() -> GateParams:
    """Small N so test sequences are easy to hand-verify; not the real
    N=200 config (that's exercised by the closed-loop tests later in this
    file)."""
    return GateParams(N=3, tau1=10.0, tau2=20.0, tau1_exit=4.0, tau2_exit=10.0, T_dwell=2.0)


def test_initial_gate_state():
    state = initial_gate_state()

    assert state.mode is GateMode.NORMAL
    assert state.nis_window == ()
    assert state.time_in_mode == 0.0


def test_no_transition_while_window_not_full():
    gp = _toy_gate_params()
    state = initial_gate_state()

    for nis in [100.0, 100.0]:  # only 2 samples; N=3, window never fills
        state, _epsilon = step_gate(state, nis, dt=1.0, gate_p=gp)
        assert state.mode is GateMode.NORMAL


def test_normal_to_cautious_and_back_with_dwell():
    gp = _toy_gate_params()
    state = initial_gate_state()

    # (nis, expected_mode, expected_epsilon) per step, computed by hand-running
    # the exact state machine below -- see plan Task 2 for the derivation.
    expected = [
        (1.0, GateMode.NORMAL, 1.0),
        (1.0, GateMode.NORMAL, 2.0),
        (1.0, GateMode.NORMAL, 3.0),
        (15.0, GateMode.CAUTIOUS, 17.0),  # window=(1,1,15) full, eps=17>tau1=10
        (1.0, GateMode.CAUTIOUS, 17.0),   # window=(1,15,1), dwell blocks exit anyway (eps=17 > tau1_exit=4)
        (1.0, GateMode.CAUTIOUS, 17.0),   # window=(15,1,1)
        (1.0, GateMode.NORMAL, 3.0),      # window=(1,1,1), eps=3<tau1_exit=4, dwell (2.0s) satisfied
        (1.0, GateMode.NORMAL, 3.0),
    ]
    for nis, expected_mode, expected_eps in expected:
        state, eps = step_gate(state, nis, dt=1.0, gate_p=gp)
        assert state.mode is expected_mode
        assert eps == expected_eps


def test_normal_to_halt_skip_on_full_window():
    gp = _toy_gate_params()
    state = initial_gate_state()

    for nis in [1.0, 1.0, 1.0]:
        state, _eps = step_gate(state, nis, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.NORMAL

    state, eps = step_gate(state, 25.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.HALT
    assert eps == 27.0


def test_halt_to_cautious_exit():
    gp = _toy_gate_params()
    state = GateState(mode=GateMode.HALT, nis_window=(100.0, 100.0, 100.0), time_in_mode=5.0)

    state, eps = step_gate(state, 1.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.HALT
    assert eps == 201.0

    state, eps = step_gate(state, 1.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.HALT
    assert eps == 102.0

    state, eps = step_gate(state, 1.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.CAUTIOUS
    assert eps == 3.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_gate.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.gate'`

- [ ] **Step 3: Write `sim/gate.py`**

```python
"""Normal/Cautious/Halt gate state machine (CLAUDE.md section 10).

Watches the windowed NIS statistic epsilon_k = sum of the last N NIS
values (CLAUDE.md section 9) and switches between three modes on
thresholds tau1 < tau2, with hysteresis (tau1_exit < tau1, tau2_exit <
tau2) and a minimum dwell time before any transition. See GateParams'
docstring in sim/params.py for why tau1/tau2 are empirically calibrated
rather than literal chi2(3N) quantiles.
"""

from dataclasses import dataclass
from enum import Enum

from sim.params import GateParams


class GateMode(Enum):
    NORMAL = "NORMAL"
    CAUTIOUS = "CAUTIOUS"
    HALT = "HALT"


@dataclass(frozen=True)
class GateState:
    mode: GateMode
    nis_window: tuple[float, ...]
    time_in_mode: float


def initial_gate_state() -> GateState:
    return GateState(mode=GateMode.NORMAL, nis_window=(), time_in_mode=0.0)


def step_gate(state: GateState, nis_k: float, dt: float, gate_p: GateParams) -> tuple[GateState, float]:
    """Advance the gate by one control-loop sample. Returns (new_state, epsilon_k).

    epsilon_k is returned even when the window isn't yet full (sum of
    whatever has accumulated so far), but mode transitions only ever
    happen once the window is full -- CLAUDE.md's "No NORMAL -> HALT skip
    unless epsilon_k > tau2 for a full window" is read here as: a direct
    NORMAL -> HALT transition is allowed exactly when the window is fully
    populated and epsilon_k > tau2 (epsilon_k, being an N-sample sum, IS
    "a full window" of evidence by construction; the exception is about
    not acting on an under-filled, still-warming-up window).
    """
    window = state.nis_window + (nis_k,)
    if len(window) > gate_p.N:
        window = window[-gate_p.N:]
    window_full = len(window) == gate_p.N
    epsilon = sum(window)

    time_in_mode = state.time_in_mode + dt
    can_transition = time_in_mode >= gate_p.T_dwell

    new_mode = state.mode
    if can_transition and window_full:
        if state.mode is GateMode.NORMAL:
            if epsilon > gate_p.tau2:
                new_mode = GateMode.HALT
            elif epsilon > gate_p.tau1:
                new_mode = GateMode.CAUTIOUS
        elif state.mode is GateMode.CAUTIOUS:
            if epsilon > gate_p.tau2:
                new_mode = GateMode.HALT
            elif epsilon < gate_p.tau1_exit:
                new_mode = GateMode.NORMAL
        elif state.mode is GateMode.HALT:
            if epsilon < gate_p.tau2_exit:
                new_mode = GateMode.CAUTIOUS

    if new_mode != state.mode:
        time_in_mode = 0.0

    return GateState(mode=new_mode, nis_window=window, time_in_mode=time_in_mode), epsilon
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_gate.py -v`
Expected: PASS (5 tests)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 44 tests: 39 pre-existing + 5 new)

- [ ] **Step 6: Commit**

```bash
git add sim/gate.py tests/test_gate.py
git commit -m "feat: add gate state machine (Normal/Cautious/Halt, windowed NIS)"
```

---

## Task 3: `sim/disturbances.py` — the five disturbance profiles

**Files:**
- Create: `sim/disturbances.py`
- Test: `tests/test_disturbances.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_disturbances.py
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_disturbances.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.disturbances'`

- [ ] **Step 3: Write `sim/disturbances.py`**

```python
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_disturbances.py -v`
Expected: PASS (7 tests)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all 51 tests: 44 pre-existing + 7 new)

- [ ] **Step 6: Commit**

```bash
git add sim/disturbances.py tests/test_disturbances.py
git commit -m "feat: add disturbance profiles (surface, battery, push, sensor fault x2, payload)"
```

---

## Task 4: Section 11 gate test — no mode change on a nominal 60s run

**Files:**
- Modify: `tests/test_gate.py` (append to the file created in Task 2)

CLAUDE.md section 11: *"Gate: no mode change on a nominal 60 s run (seeded)."* This wires `sim.plant.f`, `sim.integrate.rk4_step`, `sim.sensors`, `sim.estimator`, `sim.control.design_lqr_balance`, and the new `sim.gate.step_gate` together into the same 200Hz/1kHz closed loop `tests/test_estimator.py::_run_nominal_closed_loop` established in step 3, with the gate stepped alongside the KF each control tick.

**This test takes roughly 20-60 seconds of wall-clock time** (240,000 RK4 substeps), same as the step-3 NIS test — expected, not a hang.

- [ ] **Step 1: Write the failing test**

Append to `tests/test_gate.py`:

```python
import numpy as np

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
from sim.integrate import rk4_step
from sim.plant import f as plant_f
from sim.params import (
    default_disturbance_params,
    default_estimator_params,
    default_gate_params,
    default_lqr_balance_params,
    default_plant_params,
    default_sensor_params,
)
from sim.sensors import accelerometer, encoder, gyro


def _closed_loop_with_gate(seed, T, apply_disturbance=None, x0_psi_deg=0.0):
    """200Hz/1kHz closed loop (plant + sensors + KF + LQR, as in
    tests/test_estimator.py::_run_nominal_closed_loop) with the gate
    stepped alongside the KF each control tick.

    `apply_disturbance`, if given, is called each tick as
    apply_disturbance(t, x_true, b_g_true, p, sensor_p) ->
    (x_true, b_g_true, p_now, sensor_p_now, v_batt_now), letting a test
    apply a one-shot state kick and/or substitute disturbed params for
    that instant. Returns the list of GateMode values, one per control
    tick.
    """
    p = default_plant_params()
    sensor_p = default_sensor_params()
    est_p = default_estimator_params()
    lqr_p = default_lqr_balance_params()
    gate_p = default_gate_params()
    dp = default_disturbance_params()
    nominal_v_batt = dp.battery_droop_v_nominal
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

    x_true = np.array([0.0, np.radians(x0_psi_deg), 0.0, 0.0, 0.0, 0.0])
    b_g_true = 0.0
    kf = KalmanState(x_hat=np.zeros(5), P=initial_covariance(est_p))
    gate_state = initial_gate_state()
    u_prev = 0.0
    modes = []

    for k in range(n_steps):
        t = k * dt_control
        if apply_disturbance is not None:
            x_true, b_g_true, p_now, sensor_p_now, v_batt_now = apply_disturbance(
                t, x_true, b_g_true, p, sensor_p
            )
        else:
            p_now, sensor_p_now, v_batt_now = p, sensor_p, nominal_v_batt

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
        gate_state, _epsilon = step_gate(gate_state, nis, dt_control, gate_p)
        modes.append(gate_state.mode)

        u_cmd = -K @ kf.x_hat[:4]
        u = clip_voltage(u_cmd, v_batt_now)
        u_prev = u
        for _ in range(n_sub):
            x_true = rk4_step(xdotf, x_true, (u / 2, u / 2), dt_plant)

    return modes


def test_no_mode_change_on_nominal_60s_run():
    """Section 11: 'Gate: no mode change on a nominal 60s run (seeded).'

    Verified with seed=42 here, and independently with seeds 0-9 before
    this plan was written (see plan doc's threshold-calibration table) --
    max epsilon across all 10 was 959.1, safely under GateParams.tau1=1300.
    """
    modes = _closed_loop_with_gate(seed=42, T=60.0)

    assert set(modes) == {GateMode.NORMAL}
```

- [ ] **Step 2: Run test to verify it passes**

Run: `uv run pytest tests/test_gate.py::test_no_mode_change_on_nominal_60s_run -v`
Expected: PASS (~20-60s wall-clock). If it fails, do not loosen `GateParams`' thresholds — print `max(epsilon)` observed during the run and compare against the 959.1 figure above; a large discrepancy means a transcription bug in the closed-loop wiring (check argument order against `tests/test_estimator.py`'s established harness), not a tuning problem.

- [ ] **Step 3: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (52 tests: 51 pre-existing + this one)

- [ ] **Step 4: Commit**

```bash
git add tests/test_gate.py
git commit -m "test: add section 11 no-mode-change-on-nominal-60s gate test"
```

---

## Task 5: Section 11 gate tests — enters CAUTIOUS under each disturbance (Milestone 1)

**Files:**
- Modify: `tests/test_gate.py` (append to the file extended in Task 4)

CLAUDE.md section 11: *"enters CAUTIOUS within X s of each disturbance at a stated magnitude."* This is build-order **Milestone 1: gate switches under one disturbance** — and, per the literal test wording, under each of the five. Uses `_closed_loop_with_gate` from Task 4 with a per-disturbance `apply_disturbance` closure built from the actual `sim.disturbances` functions (not reimplemented inline), so these tests genuinely exercise Task 3's code, not just a copy of its logic.

**Each of these 6 tests takes roughly 7-20 seconds of wall-clock time** (20 simulated seconds each, shorter than Task 4's 60s test).

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_gate.py`:

```python
from sim.disturbances import (
    battery_droop_v_batt,
    payload_shift_plant_params,
    push_psi_dot_kick,
    sensor_fault_accel_noise_params,
    sensor_fault_gyro_bias_step,
    surface_change_plant_params,
)


def _assert_enters_cautious_within(modes, onset, window_s, dt_control=0.005):
    onset_idx = int(round(onset / dt_control))
    deadline_idx = onset_idx + int(round(window_s / dt_control))
    first_non_normal = next(
        (i for i in range(onset_idx, min(deadline_idx, len(modes))) if modes[i] is not GateMode.NORMAL),
        None,
    )
    assert first_non_normal is not None, f"never left NORMAL within {window_s}s of onset (t={onset}s)"


def test_enters_cautious_after_push():
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        x_true = x_true.copy()
        x_true[4] = push_psi_dot_kick(x_true[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x_true, b_g_true, p, sensor_p, dp.battery_droop_v_nominal

    modes = _closed_loop_with_gate(seed=42, T=20.0, apply_disturbance=apply_disturbance)
    _assert_enters_cautious_within(modes, onset=dp.push_onset, window_s=dp.detection_window_s)


def test_enters_cautious_after_surface_change():
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        p_now = surface_change_plant_params(
            t, dp.surface_change_onset, dp.surface_change_duration, dp.surface_change_fw, p
        )
        return x_true, b_g_true, p_now, sensor_p, dp.battery_droop_v_nominal

    modes = _closed_loop_with_gate(seed=42, T=20.0, apply_disturbance=apply_disturbance)
    _assert_enters_cautious_within(modes, onset=dp.surface_change_onset, window_s=dp.detection_window_s)


def test_enters_cautious_after_gyro_bias_fault():
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        b_g_true = sensor_fault_gyro_bias_step(
            b_g_true, t, dp.gyro_bias_fault_onset, 0.005, dp.gyro_bias_fault_magnitude
        )
        return x_true, b_g_true, p, sensor_p, dp.battery_droop_v_nominal

    modes = _closed_loop_with_gate(seed=42, T=20.0, apply_disturbance=apply_disturbance)
    _assert_enters_cautious_within(modes, onset=dp.gyro_bias_fault_onset, window_s=dp.detection_window_s)


def test_enters_cautious_after_accel_noise_fault():
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        sensor_p_now = sensor_fault_accel_noise_params(
            t, dp.accel_noise_fault_onset, dp.accel_noise_fault_duration, dp.accel_noise_fault_multiplier, sensor_p
        )
        return x_true, b_g_true, p, sensor_p_now, dp.battery_droop_v_nominal

    modes = _closed_loop_with_gate(seed=42, T=20.0, apply_disturbance=apply_disturbance)
    _assert_enters_cautious_within(modes, onset=dp.accel_noise_fault_onset, window_s=dp.detection_window_s)


def test_enters_cautious_after_payload_shift():
    """Needs a mild concurrent tilt to expose the mass/CoM mismatch --
    near-perfect equilibrium doesn't accelerate enough for even this large
    (dM=1kg, dL=0.1m) shift to move the NIS. See plan doc: substantially
    smaller, more "realistic" shifts (dM up to 0.5kg, dL up to 0.05m) did
    not trigger detection at all, even combined with tilt -- a genuine,
    reportable sensitivity limit of the current tuning, not a bug."""
    dp = default_disturbance_params()

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        p_now = payload_shift_plant_params(
            t, dp.payload_shift_onset, dp.payload_shift_delta_M, dp.payload_shift_delta_L, p
        )
        return x_true, b_g_true, p_now, sensor_p, dp.battery_droop_v_nominal

    modes = _closed_loop_with_gate(
        seed=42, T=20.0, apply_disturbance=apply_disturbance, x0_psi_deg=dp.payload_shift_test_psi0_deg
    )
    _assert_enters_cautious_within(modes, onset=dp.payload_shift_onset, window_s=dp.detection_window_s)


def test_enters_cautious_after_battery_droop():
    """Battery droop alone near equilibrium never binds the voltage
    ceiling (nominal commands are far below even a badly drooped
    battery's limit) -- see plan doc. Paired here with a small companion
    push, itself independently verified to be too small to trigger
    detection on its own, to give the controller genuine voltage demand
    the drooped battery can't fully supply.

    The companion push fires at battery_droop_onset + battery_droop_duration
    (once the ramp has actually finished, not at the droop's own onset):
    battery_droop_v_batt RAMPS from v_nominal to v_drooped over that
    duration (CLAUDE.md: "V_batt ramps down"), so at t=battery_droop_onset
    itself V_batt is still ~v_nominal -- a push applied there would hit an
    essentially-undrooped battery and prove nothing about degraded
    recovery. Detection is measured from this same push-onset reference,
    not from battery_droop_onset, since that's when the actual
    controller-relevant event happens."""
    dp = default_disturbance_params()
    push_onset = dp.battery_droop_onset + dp.battery_droop_duration

    def apply_disturbance(t, x_true, b_g_true, p, sensor_p):
        v_batt = battery_droop_v_batt(
            t, dp.battery_droop_onset, dp.battery_droop_duration, dp.battery_droop_v_nominal, dp.battery_droop_v_drooped
        )
        x_true = x_true.copy()
        x_true[4] = push_psi_dot_kick(
            x_true[4], t, push_onset, 0.005, dp.battery_droop_companion_push_magnitude
        )
        return x_true, b_g_true, p, sensor_p, v_batt

    modes = _closed_loop_with_gate(seed=42, T=20.0, apply_disturbance=apply_disturbance)
    _assert_enters_cautious_within(modes, onset=push_onset, window_s=dp.detection_window_s)
```

- [ ] **Step 2: Run tests to verify they pass**

Run: `uv run pytest tests/test_gate.py -v`
Expected: PASS (all tests in this file: 5 unit + 1 nominal-60s + 6 disturbance-detection = 12). If any of the 6 disturbance tests fails, do not widen `detection_window_s` or change the disturbance magnitude to "make it pass" — print the actual first-non-NORMAL time (or confirm it truly never leaves NORMAL) and compare against this plan's per-disturbance latency table; a large discrepancy from the documented ~0.03-1.23s figures means a transcription bug (most likely: wrong argument order into a `sim.disturbances` function, or the `apply_disturbance` closure not actually being invoked because of a copy/mutation bug on `x_true`).

- [ ] **Step 3: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (58 tests: 52 pre-existing + 6 new). **Total suite runtime will be several minutes** (the 60s nominal gate test plus six 20s disturbance tests, all doing 1kHz nonlinear RK4 integration in pure Python) — expected, not a regression.

- [ ] **Step 4: Commit**

```bash
git add tests/test_gate.py
git commit -m "test: add section 11 per-disturbance CAUTIOUS-detection gate tests (Milestone 1)"
```

---

## Self-review notes

- **Spec coverage:** Section 14 step 4 ("gate.py → disturbances.py") ✓, **Milestone 1: gate switches under one disturbance** ✓ (achieved 6 times over — Task 5 covers all 5 CLAUDE.md section 8 disturbance categories, with 2 variants for "sensor fault"). Section 10's mode set, hysteresis, dwell time, and the NORMAL→HALT skip exception ✓ (Task 2, with the skip's ambiguous wording resolved and documented). Section 9's windowed statistic ε_k ✓ (Task 2, `step_gate`'s `epsilon` return value). Section 8's five disturbance profiles ✓ (Task 3), including finally implementing section 5's deferred voltage-clipping note (`clip_voltage`, Task 3) and section 8's "V_batt is a state of the disturbance model" (`battery_droop_v_batt`, Task 3). Section 8's "the estimator always uses the nominal model" ✓ — every `apply_disturbance` closure in Task 5 only ever substitutes `p_now`/`sensor_p_now`/`v_batt_now` for the *true-plant* and *sensor-generation* calls; `Ad`, `Bd`, `Q`, `R` (built once from nominal params) are never touched. Section 11's two gate-specific tests ✓ (Tasks 4 and 5).
- **Placeholder scan:** no TBD/TODO; every step has runnable code, exact expected output, and realistic runtime caveats stated plainly (not hidden).
- **Type consistency:** `GateMode`/`GateState`/`initial_gate_state`/`step_gate` signatures are identical between Task 2's definition, its own tests, and Task 4/5's closed-loop harness. All six `sim.disturbances` function signatures are identical between Task 3's definitions, Task 3's own unit tests, and Task 5's `apply_disturbance` closures (Task 5 calls the real functions, not reimplementations). `GateParams`/`DisturbanceParams` field names match between Task 1's definitions and every later usage.
- **Explicitly deferred (not this plan):** `sim/world.py` (corridor + ray casting, needed for the ultrasonic sensor and wall-following), `sim/run.py` (the general-purpose closed-loop runner — this plan's `_closed_loop_with_gate` is test-only, same caveat as step 3's harness), everything in `experiments/`/`analysis/`/`quantum/`.
- **Honest limitations surfaced by this step, worth carrying into the project's write-up rather than quietly fixing:** (1) the controller has a real stability boundary near `f_w≈0.03` (built without a speed-servo integral term yet); (2) NIS-based detection is measurably less sensitive to smooth mass/CoM shifts than to the other four disturbance types under the current `EstimatorParams` tuning; (3) battery droop only manifests when paired with a genuine voltage demand, not in isolation near equilibrium. None of these are bugs to chase during this step — they're real properties of the system as built so far.
