# Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement `sim/linearize.py` (analytic + numeric-checked Jacobian of the plant at the upright equilibrium) and the balance LQR in `sim/control.py`, with the section 11 tests this step covers: numeric-vs-analytic Jacobian, linear eigenvalues vs. the section 6.1 sanity values, LQR closed-loop poles, and nonlinear recovery from ψ₀=10°.

**Architecture:** `sim/linearize.py` exposes `linearize(p)` (full 6-state analytic Jacobian, matched against a numeric finite-difference Jacobian of `sim/plant.py::f`) and `linearize_planar(p)` (the reduced 4-state balance model CLAUDE.md section 9 describes: state `[theta, psi, theta_dot, psi_dot]`, single input `u = v_l + v_r`). `sim/control.py` adds `design_lqr_balance(p, lqr_p)`, solving the continuous algebraic Riccati equation (via `scipy.linalg.solve_continuous_are`) on the planar model to get a state-feedback gain `K`. Q/R weights are parameters (`sim/params.py`), not magic numbers.

**Tech Stack:** Python ≥3.11, `numpy`, `scipy` (new dependency this step, for `scipy.linalg.solve_continuous_are`).

**Explicitly out of scope for this plan:** `sim/sensors.py`, `sim/estimator.py`, `sim/gate.py`, `sim/disturbances.py`, `sim/world.py`, `sim/run.py`, and everything else past section 14 build-order step 2. Also out of scope within `control.py` itself: the speed servo integral term, yaw PD, and wall-following (CLAUDE.md section 9 lists these, but the build order's step 2 is specifically "linearize.py → LQR → recovery test" — a pure balance LQR). The recovery test in this plan uses simple per-plant-step (1 ms) state feedback, not the full 200 Hz ZOH multi-rate control loop from section 7 — see "Design decision" below.

---

## Pre-verified math

Before writing this plan I independently derived and numerically verified everything below (ran it against the actual `sim/params.py`/`sim/plant.py` from step 1). The implementer should not need to re-derive anything — just transcribe the code — but the derivation is included so it can be checked.

**Why the planar (θ, ψ) block and yaw (φ) block fully decouple at linearization:** `F_θ` and `F_ψ` are already *exactly* linear in `(θ̇, ψ̇, v_l, v_r)` — no linearization needed there. The only nonlinear term that survives to first order is `M·g·L·sin(ψ) ≈ M·g·L·ψ` in `F_ψ`'s effective forcing (the `ψ̇²`, `φ̇²` velocity-squared terms are second-order and vanish). Similarly `F_φ` is already exactly linear in `(φ̇, v_l, v_r)`, with no θ/ψ coupling term surviving. And since the mass matrix `A(ψ)` is evaluated where the un-linearized forcing `b(ψ=0, v=0, ẋ=0)` is exactly zero, the ψ-dependence of `A(ψ)` itself drops out of the linearization (product rule: `d/dx[A⁻¹(x)·b(x)] = A⁻¹·db/dx` when `b=0` at the base point). Net effect: differential drive `(v_r-v_l)` only ever affects yaw, common-mode drive `(v_l+v_r)` only ever affects `θ,ψ` — matching the physical intuition and letting `linearize.py` compute each block independently instead of one large symbolic Jacobian.

**Verification run** (`uv run python` from the project root, with `sim/params.py`/`sim/plant.py` from step 1):
- Full 6×6 analytic `A` eigenvalues: `[0, -241.194, 7.437, -6.520, 0, -95.568]` (the last two are the yaw block: an integrator + a damping pole).
- Planar 4×4 `A_planar` eigenvalues: `[0, -241.194, 7.437, -6.520]` — matches CLAUDE.md section 6.1's `{0, −241, +7.44, −6.52}` almost to display precision.
- Numeric (central-difference) Jacobian of `plant.f` vs. the analytic `A, B`: `max|A - A_numeric| ≈ 9.6e-10`, `max|B - B_numeric| ≈ 1.4e-14` — both far inside the required `rtol=1e-6`.
- LQR with `Q=diag(1, 1e3, 1, 1)`, `R=1e2` on the planar model: `K = [-0.100, -49.302, -2.092, -4.545]` — matches CLAUDE.md's `K ≈ [−0.10, −49.3, −2.09, −4.55]`. Closed-loop eigenvalues: `[-241.8, -0.097, -7.659, -6.344]` — all real parts negative.
- Nonlinear recovery from `ψ₀=10°` (full 6-state `plant.f`, RK4 at `dt=1ms`, `u = -K·[θ,ψ,θ̇,ψ̇]` recomputed every step, `v_l=v_r=u/2`): `ψ` never exceeds its initial `10°` (monotonic decay, no overshoot), reaches `-0.06°` by `t=1s` and `0.003°` by `t=2-3s`. Commanded `u=v_l+v_r` starts at `8.6V` and decays — a physically plausible magnitude for a 7.4-8.4V battery (we aren't clipping voltage yet; that's `sim/disturbances.py`'s job later).

## Design decision: the recovery test uses per-step feedback, not a 200 Hz ZOH loop

CLAUDE.md section 7 specifies a 200 Hz (5 ms) control/estimation loop with ZOH on voltage, separate from the 1 ms plant integration rate — but that multi-rate architecture (sample state, hold voltage for 5 ms while the plant integrates at 1 ms) is the job of `sim/run.py` (section 14 step 6), once there's a real sensor/estimator pipeline whose output is what actually gets sampled and held. There's no estimator yet — this step's "recovery test" is a full-state-feedback check of the LQR gain itself, straight from CLAUDE.md section 11's literal requirement ("recovers from ψ₀=10° in the nonlinear sim"). Recomputing `u=-K·x` every 1 ms plant step (rather than every 5 ms) is strictly more conservative for stability, not less, so this doesn't overstate the controller's robustness — it just doesn't yet test the coarser-grained ZOH loop, which isn't buildable until later steps exist. This is flagged here so it isn't mistaken for the final control-loop architecture.

## Design decision: Q/R weights are parameters, not magic numbers

CLAUDE.md's only concrete LQR design point anywhere in the spec is the section 6.1 sanity-check example (`Q=diag(1, 1e3, 1, 1)`, `R=1e2`). Per section 3's "no magic numbers anywhere else" rule, these live in `sim/params.py` as a new frozen dataclass (`LQRBalanceParams`), not hardcoded inside `sim/control.py`. This also happens to be the only LQR tuning the spec gives us, so it's the correct default for now; it can be retuned later without touching `control.py`.

---

## Task 1: Add scipy dependency

**Files:**
- Modify: `pyproject.toml`, `uv.lock`

- [ ] **Step 1: Add scipy**

```bash
cd "C:\Dev\Echo Balancer"
uv add scipy
```

This is needed for `scipy.linalg.solve_continuous_are` in Task 4 (LQR design via the continuous algebraic Riccati equation). If `uv add scipy` reports scipy is already present (it may already be in `pyproject.toml` from prior exploration), that's fine — just confirm it's there and move on to the commit.

- [ ] **Step 2: Verify it imports**

```bash
uv run python -c "import scipy.linalg; print(scipy.__version__)"
```

Expected: prints a version string, no error.

- [ ] **Step 3: Commit**

```bash
git add pyproject.toml uv.lock
git commit -m "chore: add scipy dependency for LQR design"
```

---

## Task 2: `sim/linearize.py` — analytic + numeric-verified Jacobian

**Files:**
- Create: `sim/linearize.py`
- Test: `tests/test_linearize.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_linearize.py
import numpy as np

from sim.linearize import linearize, linearize_planar
from sim.params import default_plant_params
from sim.plant import f as plant_f


def _numeric_jacobian(p, eps=1e-6):
    """Central-difference Jacobian of plant_f at x=0, u=(0,0)."""
    x0 = np.zeros(6)
    n, m = 6, 2
    A = np.zeros((n, n))
    B = np.zeros((n, m))
    for i in range(n):
        dx = np.zeros(n)
        dx[i] = eps
        f_plus = plant_f(x0 + dx, 0.0, 0.0, p)
        f_minus = plant_f(x0 - dx, 0.0, 0.0, p)
        A[:, i] = (f_plus - f_minus) / (2 * eps)
    for j in range(m):
        u_plus = [eps if k == j else 0.0 for k in range(m)]
        u_minus = [-eps if k == j else 0.0 for k in range(m)]
        f_plus = plant_f(x0, u_plus[0], u_plus[1], p)
        f_minus = plant_f(x0, u_minus[0], u_minus[1], p)
        B[:, j] = (f_plus - f_minus) / (2 * eps)
    return A, B


def test_numeric_jacobian_matches_analytic():
    p = default_plant_params()
    A_analytic, B_analytic = linearize(p)
    A_numeric, B_numeric = _numeric_jacobian(p)

    np.testing.assert_allclose(A_analytic, A_numeric, rtol=1e-6, atol=1e-8)
    np.testing.assert_allclose(B_analytic, B_numeric, rtol=1e-6, atol=1e-8)


def test_planar_eigenvalues_match_sanity_values():
    p = default_plant_params()
    A_planar, _ = linearize_planar(p)
    eigs = np.sort(np.linalg.eigvals(A_planar).real)

    # CLAUDE.md section 6.1: open-loop eigenvalues ~= {0, -241, +7.44, -6.52} rad/s.
    target = np.sort(np.array([0.0, -241.0, 7.44, -6.52]))
    np.testing.assert_allclose(eigs, target, rtol=1e-2, atol=0.05)


def test_planar_is_a_consistent_reduction_of_the_full_system():
    """B's v_l and v_r columns must be identical in the planar rows: F_theta
    and F_psi depend only on (v_l + v_r), never on v_l, v_r individually."""
    p = default_plant_params()
    A, B = linearize(p)
    planar_rows = [0, 1, 3, 4]
    np.testing.assert_allclose(B[planar_rows, 0], B[planar_rows, 1])
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_linearize.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.linearize'`

- [ ] **Step 3: Write `sim/linearize.py`**

```python
"""Analytic linearization of the plant at the upright equilibrium
(CLAUDE.md sections 5 and 9), cross-checked against a numeric Jacobian of
sim.plant.f in tests/test_linearize.py.

At x=0 (upright, at rest), u=(0,0), the plant is a fixed point (see
tests/test_plant.py). The generalized forces F_theta/F_psi/F_phi are already
exactly linear in (theta_dot, psi_dot, phi_dot, v_l, v_r); the only nonlinear
term surviving to first order is M*g*L*sin(psi) ~= M*g*L*psi in F_psi. Because
of this, the planar (theta, psi) block and the yaw (phi) block decouple
completely at linearization: common-mode drive (v_l + v_r) only ever affects
theta/psi, differential drive (v_r - v_l) only ever affects phi. This module
computes each block directly rather than deriving one large symbolic
Jacobian by hand.
"""

import numpy as np

from sim.params import PlantParams


def linearize(p: PlantParams) -> tuple[np.ndarray, np.ndarray]:
    """Analytic Jacobian of sim.plant.f at x=0, u=(0, 0).

    Returns (A, B) for the full 6-state system,
    x = [theta, psi, phi, theta_dot, psi_dot, phi_dot], u = [v_l, v_r].
    """
    alpha = p.n * p.Kt / p.Rm
    beta = p.n**2 * p.Kt * p.Kb / p.Rm + p.fm

    A11 = (2 * p.m + p.M) * p.R**2 + 2 * p.Jw + 2 * p.n**2 * p.Jm
    A12 = p.M * p.L * p.R - 2 * p.n**2 * p.Jm  # cos(0) = 1
    A22 = p.M * p.L**2 + p.Jpsi + 2 * p.n**2 * p.Jm
    A33 = (
        0.5 * p.m * p.W**2
        + p.Jphi
        + (p.W**2 / (2 * p.R**2)) * (p.Jw + p.n**2 * p.Jm)
    )  # sin(0) = 0

    M_planar = np.array([[A11, A12], [A12, A22]])
    M_planar_inv = np.linalg.inv(M_planar)

    # Columns: [psi, theta_dot, psi_dot]. Rows: [b_theta, b_psi].
    d_b_d_state = np.array([
        [0.0, -2 * (beta + p.fw), 2 * beta],
        [p.M * p.g * p.L, 2 * beta, -2 * beta],
    ])
    # Columns: [v_l, v_r]. Rows: [b_theta, b_psi].
    d_b_d_input = np.array([
        [alpha, alpha],
        [-alpha, -alpha],
    ])

    d_qddot_d_state = M_planar_inv @ d_b_d_state  # 2x3: [theta_ddot; psi_ddot]
    d_qddot_d_input = M_planar_inv @ d_b_d_input  # 2x2

    d_F_phi_d_phidot = -(p.W**2 / (2 * p.R**2)) * (beta + p.fw)
    d_F_phi_d_input = np.array([-(p.W / (2 * p.R)) * alpha, (p.W / (2 * p.R)) * alpha])
    d_phiddot_d_phidot = d_F_phi_d_phidot / A33
    d_phiddot_d_input = d_F_phi_d_input / A33

    A = np.zeros((6, 6))
    B = np.zeros((6, 2))

    A[0, 3] = 1.0  # d(theta)/dt = theta_dot
    A[1, 4] = 1.0  # d(psi)/dt = psi_dot
    A[2, 5] = 1.0  # d(phi)/dt = phi_dot

    A[3, 1] = d_qddot_d_state[0, 0]
    A[3, 3] = d_qddot_d_state[0, 1]
    A[3, 4] = d_qddot_d_state[0, 2]
    A[4, 1] = d_qddot_d_state[1, 0]
    A[4, 3] = d_qddot_d_state[1, 1]
    A[4, 4] = d_qddot_d_state[1, 2]

    B[3, :] = d_qddot_d_input[0, :]
    B[4, :] = d_qddot_d_input[1, :]

    A[5, 5] = d_phiddot_d_phidot
    B[5, :] = d_phiddot_d_input

    return A, B


def linearize_planar(p: PlantParams) -> tuple[np.ndarray, np.ndarray]:
    """Reduced planar balance model (CLAUDE.md section 9): state
    [theta, psi, theta_dot, psi_dot], single input u = v_l + v_r.

    Extracted from linearize(): F_theta and F_psi depend only on the sum
    (v_l + v_r), never on v_l and v_r individually, so the two input columns
    of the full B are identical in these rows and either one is exactly
    d(qddot)/d(v_l + v_r).
    """
    A, B = linearize(p)
    idx = [0, 1, 3, 4]  # theta, psi, theta_dot, psi_dot
    A_planar = A[np.ix_(idx, idx)]
    B_planar = B[idx, 0]
    return A_planar, B_planar
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_linearize.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add sim/linearize.py tests/test_linearize.py
git commit -m "feat: add analytic plant linearization (full + planar), verified vs numeric Jacobian"
```

---

## Task 3: `sim/params.py` — LQR balance weights

**Files:**
- Modify: `sim/params.py` (append; do not change `PlantParams`/`default_plant_params`)
- Test: `tests/test_params.py` (append)

- [ ] **Step 1: Write the failing test**

Append to `tests/test_params.py`:

```python
from sim.params import LQRBalanceParams, default_lqr_balance_params


def test_default_lqr_balance_params():
    lp = default_lqr_balance_params()

    assert lp.Q_theta == 1.0
    assert lp.Q_psi == 1e3
    assert lp.Q_theta_dot == 1.0
    assert lp.Q_psi_dot == 1.0
    assert lp.R == 1e2


def test_lqr_balance_params_is_frozen():
    lp = default_lqr_balance_params()
    try:
        lp.R = 1.0
        assert False, "LQRBalanceParams should be frozen"
    except AttributeError:
        pass
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_params.py -v`
Expected: FAIL with `ImportError: cannot import name 'LQRBalanceParams' from 'sim.params'`

- [ ] **Step 3: Append to `sim/params.py`**

Add this to the end of `sim/params.py` (after `default_plant_params`), keeping everything already in the file unchanged:

```python
@dataclass(frozen=True)
class LQRBalanceParams:
    """Balance-LQR state weights, on the planar model
    [theta, psi, theta_dot, psi_dot], input u = v_l + v_r.

    CLAUDE.md section 6.1's sanity-check example is the only LQR design
    point the spec gives, so it's also the default here.
    """

    Q_theta: float      # weight on theta (wheel position)
    Q_psi: float        # weight on psi (body pitch) -- dominant term
    Q_theta_dot: float  # weight on theta_dot
    Q_psi_dot: float    # weight on psi_dot
    R: float            # weight on u = v_l + v_r


def default_lqr_balance_params() -> LQRBalanceParams:
    """CLAUDE.md section 6.1: Q=diag(1, 1e3, 1, 1), R=1e2."""
    return LQRBalanceParams(Q_theta=1.0, Q_psi=1e3, Q_theta_dot=1.0, Q_psi_dot=1.0, R=1e2)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_params.py -v`
Expected: PASS (4 tests: the 2 existing `PlantParams` tests plus the 2 new ones)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all tests, no regressions)

- [ ] **Step 6: Commit**

```bash
git add sim/params.py tests/test_params.py
git commit -m "feat: add LQRBalanceParams (CLAUDE.md sec 6.1 LQR sanity-check weights)"
```

---

## Task 4: `sim/control.py` — balance LQR design, pole test, nonlinear recovery test

**Files:**
- Create: `sim/control.py`
- Test: `tests/test_control.py`

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_control.py
import numpy as np

from sim.control import design_lqr_balance
from sim.integrate import simulate
from sim.linearize import linearize_planar
from sim.params import default_lqr_balance_params, default_plant_params
from sim.plant import f as plant_f


def test_lqr_gain_matches_sanity_value():
    p = default_plant_params()
    lqr_p = default_lqr_balance_params()
    K = design_lqr_balance(p, lqr_p)

    # CLAUDE.md section 6.1: K ~= [-0.10, -49.3, -2.09, -4.55]
    target = np.array([-0.10, -49.3, -2.09, -4.55])
    np.testing.assert_allclose(K, target, rtol=1e-2, atol=0.05)


def test_lqr_closed_loop_poles_are_stable():
    p = default_plant_params()
    lqr_p = default_lqr_balance_params()
    K = design_lqr_balance(p, lqr_p)

    A_planar, B_planar = linearize_planar(p)
    A_closed_loop = A_planar - np.outer(B_planar, K)
    eigs = np.linalg.eigvals(A_closed_loop)

    assert np.all(eigs.real < 0)


def test_recovers_from_10_degree_pitch_in_nonlinear_sim():
    """Section 11: 'LQR closed-loop poles all Re < 0; recovers from psi0 =
    10 deg in the nonlinear sim.' Full 6-state plant.f, RK4 at dt=1ms,
    full-state feedback u=-K@[theta,psi,theta_dot,psi_dot] recomputed every
    step (see plan Task 4 "Design decision" -- this is not yet the 200 Hz
    ZOH loop from section 7, which needs sim/run.py)."""
    p = default_plant_params()
    lqr_p = default_lqr_balance_params()
    K = design_lqr_balance(p, lqr_p)

    def xdot(x, u):
        return plant_f(x, u[0], u[1], p)

    def u_fn(t, x):
        planar_state = np.array([x[0], x[1], x[3], x[4]])
        u = -K @ planar_state
        return (u / 2.0, u / 2.0)

    dt = 1e-3
    n_steps = 3000  # 3 s
    psi0_deg = 10.0
    x0 = np.array([0.0, np.radians(psi0_deg), 0.0, 0.0, 0.0, 0.0])

    xs = simulate(xdot, x0, u_fn, dt, n_steps)
    psi_deg = np.degrees(xs[:, 1])

    # Never falls, and never overshoots its own starting tilt.
    assert np.max(np.abs(psi_deg)) <= psi0_deg + 1e-6
    # Settles close to upright well before the fall threshold matters.
    assert abs(psi_deg[-1]) < 0.1
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `uv run pytest tests/test_control.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.control'`

- [ ] **Step 3: Write `sim/control.py`**

```python
"""Balance controller (CLAUDE.md section 9).

This step implements only the balance LQR on the planar model
[theta, psi, theta_dot, psi_dot] with u = v_l + v_r. The speed-servo
integral term, yaw PD, and wall-following described in section 9 are not
part of section 14 build-order step 2 and are not implemented here.
"""

import numpy as np
from scipy.linalg import solve_continuous_are

from sim.linearize import linearize_planar
from sim.params import LQRBalanceParams, PlantParams


def design_lqr_balance(p: PlantParams, lqr_p: LQRBalanceParams) -> np.ndarray:
    """Balance-LQR state-feedback gain K (shape (4,)) on the planar model,
    such that u = v_l + v_r = -K @ [theta, psi, theta_dot, psi_dot].
    """
    A, B = linearize_planar(p)
    Q = np.diag([lqr_p.Q_theta, lqr_p.Q_psi, lqr_p.Q_theta_dot, lqr_p.Q_psi_dot])
    R = np.array([[lqr_p.R]])

    B_col = B.reshape(-1, 1)
    P = solve_continuous_are(A, B_col, Q, R)
    K = np.linalg.inv(R) @ B_col.T @ P
    return K.flatten()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `uv run pytest tests/test_control.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Run the full suite**

Run: `uv run pytest -v`
Expected: PASS (all tests — 2 params + 2 plant smoke + 2 integrate + 2 section-11 plant + 3 linearize + 2 params (LQR) + 3 control = 16 total)

- [ ] **Step 6: Commit**

```bash
git add sim/control.py tests/test_control.py
git commit -m "feat: add balance LQR design, pole test, nonlinear recovery test"
```

---

## Self-review notes

- **Spec coverage:** Section 14 step 2 ("linearize.py → LQR → recovery test") ✓. Section 11's four tests for this step: numeric-vs-analytic Jacobian ✓ (Task 2), linear eigenvalues vs. sanity values ✓ (Task 2), LQR closed-loop poles Re<0 ✓ (Task 4), recovers from ψ₀=10° in the nonlinear sim ✓ (Task 4). Section 5's `np.linalg.solve`-not-hand-inverted rule doesn't directly apply to `linearize.py` (it's `plant.py`'s rule, already satisfied in step 1), but `linearize_planar`'s underlying `linearize()` reuses the same mass-matrix structure. Section 3's "no magic numbers" rule: LQR Q/R weights moved to `sim/params.py` (Task 3), not hardcoded in `control.py`.
- **Placeholder scan:** no TBD/TODO; every step has runnable code and exact expected output.
- **Type consistency:** `linearize(p) -> (A, B)` and `linearize_planar(p) -> (A_planar, B_planar)` signatures are used identically in Task 2's tests and Task 4's `design_lqr_balance`/tests. `design_lqr_balance(p, lqr_p) -> K` (shape `(4,)`) is used consistently across all three `test_control.py` tests. `LQRBalanceParams`/`default_lqr_balance_params` field names (`Q_theta, Q_psi, Q_theta_dot, Q_psi_dot, R`) match between Task 3's definition and Task 4's usage.
- **Explicitly deferred (not this plan):** speed-servo integral action, yaw PD, wall-following (section 9, later build-order steps), the 200 Hz ZOH multi-rate control loop (section 7, needs `sim/run.py`), voltage clipping via `V_batt` (needs `sim/disturbances.py`).
