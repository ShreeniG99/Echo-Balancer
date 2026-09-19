# Echo Balancer — Section 14 Step 1: Plant Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stand up the uv project and implement the nonlinear plant model (`sim/params.py`, `sim/plant.py`, `sim/integrate.py`) with the section 11 tests that apply to the plant alone (energy conservation, open-loop fall), per CLAUDE.md section 14 build-order step 1.

**Architecture:** `sim/params.py` holds a frozen `PlantParams` dataclass built by a `default_plant_params()` factory (all magic numbers from CLAUDE.md section 6.1 live here only). `sim/plant.py` exposes a pure function `f(x, v_l, v_r, p) -> xdot` implementing the coupled θ/ψ system (solved via `np.linalg.solve`) plus the decoupled φ equation. `sim/integrate.py` provides a generic fixed-step RK4 stepper and a `simulate` loop that holds the control input constant per step (zero-order hold). Tests exercise the plant purely through these two modules — no controller, estimator, or gate exists yet.

**Tech Stack:** Python ≥3.11, `uv`, `numpy`, `pytest`.

**Explicitly out of scope for this plan (per user instruction "don't write any other modules"):** `sim/linearize.py`, `sim/control.py`, `sim/estimator.py`, `sim/gate.py`, `sim/disturbances.py`, `sim/world.py`, `sim/run.py`, everything under `experiments/`, `analysis/`, `quantum/`. This also means the section 11 test "Numeric Jacobian of `plant.f` at upright equals analytic A, B" is **not** included here — it needs `sim/linearize.py`'s analytic A, B, which is step 2 in the build order.

---

## Open item to flag before/while executing

CLAUDE.md section 5 says plainly: *"the φ equation and the α/β forms come from memory of the NXTway-GS document and are not yet checked against the PDF... First task: get the NXTway-GS PDF (MATLAB File Exchange #19147) and confirm section 3 equations and the parameter table against this file."* Section 14's own step 1 literally starts with that verification, before `params.py`.

This plan does **not** perform that PDF verification — I have no access to MATLAB File Exchange #19147, and the user's task description scoped this request to uv/params/plant/integrate/tests only. The plan implements the θ, ψ, φ equations and the α/β forcing terms exactly as transcribed in CLAUDE.md section 5, unverified. I independently re-derived the Lagrangian (kinetic + potential energy) that produces all three equations of motion exactly as written (see Task 5 below) — this confirms internal *consistency* of the transcription (it's a valid, energy-conserving mechanical system when undamped), but it does **not** confirm the transcription matches the original NXTway-GS source. If you have the PDF, that check should still happen before trusting these equations for real hardware or publication.

## Design decision: the energy-conservation test needs the back-EMF term zeroed too

Section 11 says: *"Plant, no input, no friction, f_m = 0: total energy conserved to < 0.1% over 5 s."* I derived the system's Lagrangian (Task 5) to get an exact expression for total mechanical energy, then checked what makes `dE/dt = 0`.

The damping coefficient in the model is `β = n²·Kt·Kb/Rm + f_m`. With `v_l = v_r = 0`, the forcing terms don't vanish — they still contain `β`, because `β` models **back-EMF resistive damping through the motor windings** (a real electrical effect: even at zero applied voltage, if the motor circuit is closed, motion induces back-EMF, which drives current through `Rm` and dissipates energy as heat). This is physically distinct from `f_w` (wheel/floor friction) and `f_m` (body/motor mechanical friction), and it is *not* covered by "no friction, f_m = 0" alone.

Working the math (shown in Task 5): with only `f_w = 0` and `f_m = 0`, `dE/dt = -2β(θ̇ - ψ̇)² - (W²/2R²)·β·φ̇²`, which is negative whenever `β ≠ 0` — i.e., the system would *not* conserve energy with the default hardware `Kt`/`Kb`, and the test would correctly fail. So the energy-conservation test must additionally zero the back-EMF term, done by overriding `Kb = 0.0` in that test's params instance only (not in `default_plant_params()`, which must keep the real hardware values). I verified this is the only viable reading: any other zeroing (e.g. `Kt = 0`) works too since both appear as a product in `β`; I chose `Kb` because it's the more legible name for "no motor damping" in the test. This override is local to the test — it does not change default params or plant.py behavior.

If this reasoning looks wrong to you, or you'd rather I model motor coast/brake modes explicitly (which is a real thing on the TB6612FNG driver and would make this override unnecessary), tell me and I'll adjust before implementing — that would add a small amount of scope (a `coast: bool` argument to `f`), which the current instruction didn't ask for, so I left it out.

---

## Task 1: uv project scaffolding

**Files:**
- Create: `pyproject.toml`
- Create: `sim/__init__.py`
- Create: `tests/__init__.py`
- Create: `.gitignore`

`uv` is not currently installed on this machine (checked: not on PATH in bash or PowerShell). Install it first.

- [ ] **Step 1: Install uv**

```bash
powershell -c "irm https://astral.sh/uv/install.ps1 | iex"
```

Then open a fresh shell (or re-source PATH) and confirm:

```bash
uv --version
```

Expected: prints a version like `uv 0.x.y`.

- [ ] **Step 2: Initialize git and the uv project**

```bash
cd "/c/Dev/Echo Balancer"
git init
uv init --bare --name echo-balancer --python 3.11
```

`--bare` avoids uv scaffolding a default `echo_balancer/` package and `main.py` we don't want — we're using the `sim/` layout from CLAUDE.md section 4 instead.

- [ ] **Step 3: Add dependencies needed for this step only**

```bash
uv add numpy
uv add --dev pytest
```

(Leave `scipy`, `python-control`, `pandas`, `pyarrow`, `joblib`, `matplotlib`, `qiskit`, `qiskit-optimization` for the build-order steps that actually need them — section 3's full stack list is a project-wide target, not a step-1 requirement, and pinning `qiskit`/`qiskit-optimization` now without being able to verify `GroverOptimizer` imports (step 7) risks a pin we'll just redo later.)

- [ ] **Step 4: Configure pytest to find the `sim` package**

Edit `pyproject.toml`, add:

```toml
[tool.pytest.ini_options]
pythonpath = ["."]
testpaths = ["tests"]
```

- [ ] **Step 5: Create package directories**

```bash
mkdir -p sim tests
touch sim/__init__.py tests/__init__.py
```

- [ ] **Step 6: Add `.gitignore`**

```
.venv/
__pycache__/
*.pyc
.pytest_cache/
*.egg-info/
```

- [ ] **Step 7: Commit**

```bash
git add pyproject.toml uv.lock sim/__init__.py tests/__init__.py .gitignore .python-version
git commit -m "chore: scaffold uv project with sim/tests layout"
```

---

## Task 2: `sim/params.py` — plant parameters

**Files:**
- Create: `sim/params.py`
- Test: `tests/test_params.py`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_params.py
import math

from sim.params import default_plant_params


def test_default_plant_params_derived_quantities():
    p = default_plant_params()

    assert p.L == p.H / 2
    assert math.isclose(p.Jw, p.m * p.R**2 / 2)
    assert math.isclose(p.Jpsi, p.M * p.L**2 / 3)
    assert math.isclose(p.Jphi, p.M * (p.W**2 + p.D**2) / 12)


def test_plant_params_is_frozen():
    p = default_plant_params()
    try:
        p.M = 1.0
        assert False, "PlantParams should be frozen"
    except AttributeError:
        pass
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_params.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.params'`

- [ ] **Step 3: Write `sim/params.py`**

```python
"""Physical parameters for the Echo Balancer plant.

All numeric values from CLAUDE.md section 6.1 (Simulation v0, NXTway-GS
values) live here. No other module may define a numeric physical constant.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class PlantParams:
    g: float      # m/s^2, gravity
    m: float      # kg, wheel mass (each)
    R: float      # m, wheel radius
    Jw: float     # kg*m^2, wheel inertia, derived: m*R^2/2
    M: float      # kg, body mass
    W: float      # m, body width (wheel track)
    D: float      # m, body depth
    H: float      # m, body height
    L: float      # m, axle to body CoM, derived: H/2
    Jpsi: float   # kg*m^2, body pitch inertia, derived: M*L^2/3
    Jphi: float   # kg*m^2, body yaw inertia, derived: M*(W^2+D^2)/12
    Jm: float     # kg*m^2, motor rotor inertia
    Rm: float     # ohm, armature resistance
    Kb: float     # V*s/rad, back-EMF constant
    Kt: float     # N*m/A, torque constant
    n: float      # gear ratio
    fm: float     # body-motor friction coefficient
    fw: float     # wheel-floor friction coefficient


def default_plant_params() -> PlantParams:
    """NXTway-GS values, simulation v0 (CLAUDE.md section 6.1)."""
    g = 9.81
    m = 0.03
    R = 0.04
    M = 0.6
    W = 0.14
    D = 0.04
    H = 0.144
    L = H / 2
    Jw = m * R**2 / 2
    Jpsi = M * L**2 / 3
    Jphi = M * (W**2 + D**2) / 12
    Jm = 1e-5
    Rm = 6.69
    Kb = 0.468
    Kt = 0.317
    n = 1.0
    fm = 0.0022
    fw = 0.0
    return PlantParams(
        g=g, m=m, R=R, Jw=Jw, M=M, W=W, D=D, H=H, L=L,
        Jpsi=Jpsi, Jphi=Jphi, Jm=Jm, Rm=Rm, Kb=Kb, Kt=Kt,
        n=n, fm=fm, fw=fw,
    )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_params.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add sim/params.py tests/test_params.py
git commit -m "feat: add PlantParams and default_plant_params (CLAUDE.md sec 6.1)"
```

---

## Task 3: `sim/plant.py` — nonlinear dynamics

**Files:**
- Create: `sim/plant.py`
- Test: `tests/test_plant.py` (this task adds a smoke test; Task 5 adds the section 11 tests to the same file)

- [ ] **Step 1: Write the failing smoke test**

```python
# tests/test_plant.py
import numpy as np

from sim.params import default_plant_params
from sim.plant import f


def test_equilibrium_is_fixed_point():
    """Upright, at rest, no input: all accelerations must be zero."""
    p = default_plant_params()
    x = np.zeros(6)
    xdot = f(x, 0.0, 0.0, p)
    assert np.allclose(xdot, 0.0)


def test_state_derivative_shape():
    p = default_plant_params()
    x = np.array([0.1, 0.05, 0.2, 0.3, 0.1, 0.4])
    xdot = f(x, 1.0, -1.0, p)
    assert xdot.shape == (6,)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_plant.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.plant'`

- [ ] **Step 3: Write `sim/plant.py`**

```python
"""Nonlinear plant dynamics (CLAUDE.md section 5).

State x = [theta, psi, phi, theta_dot, psi_dot, phi_dot]:
  theta = mean wheel angle [rad]
  psi   = body pitch, 0 = upright, + = leaning forward [rad]
  phi   = yaw [rad]
Inputs v_l, v_r: motor voltages [V].
"""

import numpy as np

from sim.params import PlantParams


def f(x: np.ndarray, v_l: float, v_r: float, p: PlantParams) -> np.ndarray:
    theta, psi, phi, theta_dot, psi_dot, phi_dot = x

    alpha = p.n * p.Kt / p.Rm
    beta = p.n**2 * p.Kt * p.Kb / p.Rm + p.fm

    F_theta = (
        alpha * (v_l + v_r)
        - 2 * (beta + p.fw) * theta_dot
        + 2 * beta * psi_dot
    )
    F_psi = (
        -alpha * (v_l + v_r)
        + 2 * beta * theta_dot
        - 2 * beta * psi_dot
    )
    F_phi = (
        (p.W / (2 * p.R)) * alpha * (v_r - v_l)
        - (p.W**2 / (2 * p.R**2)) * (beta + p.fw) * phi_dot
    )

    # Coupled 2x2 system for theta_ddot, psi_ddot.
    A11 = (2 * p.m + p.M) * p.R**2 + 2 * p.Jw + 2 * p.n**2 * p.Jm
    A12 = p.M * p.L * p.R * np.cos(psi) - 2 * p.n**2 * p.Jm
    A22 = p.M * p.L**2 + p.Jpsi + 2 * p.n**2 * p.Jm

    A = np.array([[A11, A12], [A12, A22]])
    b = np.array([
        F_theta + p.M * p.L * p.R * psi_dot**2 * np.sin(psi),
        F_psi + p.M * p.g * p.L * np.sin(psi)
        + p.M * p.L**2 * phi_dot**2 * np.sin(psi) * np.cos(psi),
    ])
    theta_ddot, psi_ddot = np.linalg.solve(A, b)

    # Decoupled scalar equation for phi_ddot.
    A33 = (
        0.5 * p.m * p.W**2
        + p.Jphi
        + (p.W**2 / (2 * p.R**2)) * (p.Jw + p.n**2 * p.Jm)
        + p.M * p.L**2 * np.sin(psi) ** 2
    )
    phi_ddot = (
        F_phi - 2 * p.M * p.L**2 * psi_dot * phi_dot * np.sin(psi) * np.cos(psi)
    ) / A33

    return np.array([theta_dot, psi_dot, phi_dot, theta_ddot, psi_ddot, phi_ddot])
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_plant.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add sim/plant.py tests/test_plant.py
git commit -m "feat: add nonlinear plant dynamics f(x, v_l, v_r, p)"
```

---

## Task 4: `sim/integrate.py` — fixed-step RK4

**Files:**
- Create: `sim/integrate.py`
- Test: `tests/test_integrate.py`

- [ ] **Step 1: Write the failing test**

Verify RK4 against a known analytic solution (exponential decay) before trusting it on the plant.

```python
# tests/test_integrate.py
import numpy as np

from sim.integrate import rk4_step, simulate


def test_rk4_step_matches_exponential_decay():
    """dx/dt = -x, x(0) = 1 => x(t) = exp(-t). RK4 should be very close over one step."""
    def xdot(x, u):
        return -x

    x0 = np.array([1.0])
    dt = 0.01
    x1 = rk4_step(xdot, x0, u=(), dt=dt)
    assert np.isclose(x1[0], np.exp(-dt), rtol=1e-6)


def test_simulate_holds_input_zero_order_and_matches_step_count():
    def xdot(x, u):
        return np.array([u[0]])  # constant velocity u[0]

    x0 = np.array([0.0])
    dt = 0.1
    n_steps = 10
    xs = simulate(xdot, x0, u_fn=lambda t, x: (2.0,), dt=dt, n_steps=n_steps)

    assert xs.shape == (n_steps + 1, 1)
    assert np.isclose(xs[-1, 0], 2.0 * dt * n_steps, rtol=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_integrate.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'sim.integrate'`

- [ ] **Step 3: Write `sim/integrate.py`**

```python
"""Fixed-step RK4 integration with zero-order-hold control (CLAUDE.md section 5)."""

from typing import Callable, Tuple

import numpy as np

State = np.ndarray
Input = Tuple[float, ...]
DynamicsFn = Callable[[State, Input], State]
InputFn = Callable[[float, State], Input]


def rk4_step(f: DynamicsFn, x: State, u: Input, dt: float) -> State:
    """One fixed-step RK4 update. `u` is held constant across the step (ZOH)."""
    k1 = f(x, u)
    k2 = f(x + dt / 2 * k1, u)
    k3 = f(x + dt / 2 * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate(
    f: DynamicsFn,
    x0: State,
    u_fn: InputFn,
    dt: float,
    n_steps: int,
) -> np.ndarray:
    """Integrate `f` for `n_steps` of size `dt`, sampling `u_fn(t, x)` once per step.

    Returns an (n_steps + 1, len(x0)) array of states, including x0.
    """
    xs = np.zeros((n_steps + 1, x0.shape[0]))
    xs[0] = x0
    x = x0.copy()
    t = 0.0
    for i in range(n_steps):
        u = u_fn(t, x)
        x = rk4_step(f, x, u, dt)
        xs[i + 1] = x
        t += dt
    return xs
```

- [ ] **Step 4: Run test to verify it passes**

Run: `uv run pytest tests/test_integrate.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add sim/integrate.py tests/test_integrate.py
git commit -m "feat: add fixed-step RK4 integrator with ZOH control input"
```

---

## Task 5: Section 11 plant tests (energy conservation, open-loop fall)

**Files:**
- Modify: `tests/test_plant.py` (append to the file created in Task 3)

These are the two section 11 tests that only need `plant.py` + `integrate.py`:
> - Plant, no input, no friction, f_m = 0: total energy conserved to < 0.1% over 5 s.
> - Plant released from ψ = 1°, no control: |ψ| grows, robot falls.

**Derivation used for `total_energy`** (so the reviewer can check it against CLAUDE.md section 5 independently): writing the Lagrangian `T - V` with
`T = ½·A11·θ̇² + A12(ψ)·θ̇ψ̇ + ½·A22·ψ̇² + ½·A33(ψ)·φ̇²` and `V = M·g·L·cos(ψ)`
(where `A11, A12, A22, A33` are exactly the mass-matrix terms already in `plant.py`), applying Euler-Lagrange to each of θ, ψ, φ reproduces the three equations of motion in CLAUDE.md section 5 **exactly**, including the `MLR·ψ̇²·sinψ`, `ML²·φ̇²·sinψ·cosψ`, and `2ML²·ψ̇φ̇·sinψ·cosψ` terms. This is a strong internal-consistency check on the transcription (see "Open item" above — it does not substitute for checking the original PDF).

From this Lagrangian, `dE/dt = F_theta·θ̇ + F_psi·ψ̇ + F_phi·φ̇` in general. With `v_l = v_r = 0` and `f_w = f_m = 0`, substituting the `F_*` expressions gives:

`dE/dt = -2β(θ̇ - ψ̇)² - (W²/2R²)·β·φ̇²`, where `β = n²·Kt·Kb/Rm`.

This is ≤ 0 whenever `β ≠ 0` — energy only conserves if `β = 0` too, which is why the test below overrides `Kb = 0.0` (see "Design decision" above).

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_plant.py`:

```python
from dataclasses import replace

from sim.integrate import simulate
from sim.params import default_plant_params
from sim.plant import f


def _total_energy(x: np.ndarray, p) -> float:
    """Mechanical energy T + V for the plant (see plan Task 5 derivation)."""
    _theta, psi, _phi, theta_dot, psi_dot, phi_dot = x

    A11 = (2 * p.m + p.M) * p.R**2 + 2 * p.Jw + 2 * p.n**2 * p.Jm
    A12 = p.M * p.L * p.R * np.cos(psi) - 2 * p.n**2 * p.Jm
    A22 = p.M * p.L**2 + p.Jpsi + 2 * p.n**2 * p.Jm
    A33 = (
        0.5 * p.m * p.W**2
        + p.Jphi
        + (p.W**2 / (2 * p.R**2)) * (p.Jw + p.n**2 * p.Jm)
        + p.M * p.L**2 * np.sin(psi) ** 2
    )

    T = (
        0.5 * A11 * theta_dot**2
        + A12 * theta_dot * psi_dot
        + 0.5 * A22 * psi_dot**2
        + 0.5 * A33 * phi_dot**2
    )
    V = p.M * p.g * p.L * np.cos(psi)
    return T + V


def test_energy_conserved_no_input_no_friction():
    # "no friction, f_m = 0" (f_w, f_m) is not enough on its own: beta still
    # contains the motor back-EMF damping term n^2*Kt*Kb/Rm, which dissipates
    # energy even at zero applied voltage. Zero it out via Kb for this
    # idealized conservative-system check (see plan "Design decision").
    p = replace(default_plant_params(), fw=0.0, fm=0.0, Kb=0.0)

    dt = 1e-3
    n_steps = 5000  # 5 s
    x0 = np.array([0.0, np.radians(5.0), 0.0, 0.0, 0.0, 0.0])

    def xdot(x, u):
        return f(x, u[0], u[1], p)

    xs = simulate(xdot, x0, u_fn=lambda t, x: (0.0, 0.0), dt=dt, n_steps=n_steps)
    energies = np.array([_total_energy(x, p) for x in xs])

    e0 = energies[0]
    max_rel_dev = np.max(np.abs(energies - e0) / abs(e0))
    assert max_rel_dev < 1e-3


def test_falls_without_control():
    p = default_plant_params()
    dt = 1e-3
    n_steps = 2000  # 2 s; open-loop unstable pole ~7.44 rad/s, falls fast
    x0 = np.array([0.0, np.radians(1.0), 0.0, 0.0, 0.0, 0.0])

    def xdot(x, u):
        return f(x, u[0], u[1], p)

    xs = simulate(xdot, x0, u_fn=lambda t, x: (0.0, 0.0), dt=dt, n_steps=n_steps)
    psi = xs[:, 1]

    assert abs(psi[-1]) > abs(psi[0])
    assert np.max(np.abs(psi)) > np.radians(45.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `uv run pytest tests/test_plant.py -v -k "energy_conserved or falls_without_control"`
Expected: FAIL — at this point `plant.py` and `integrate.py` already exist from Tasks 3-4, so this should actually run and either pass or fail on the physics/assertions, not on imports. If it fails here, that's the real signal to debug (see "Do not fix a failing physics test by loosening its tolerance" — CLAUDE.md section 3). Given the derivation above, it is expected to pass as written; treat any failure as a bug to find, not a tolerance to loosen.

- [ ] **Step 3: (only if Step 2 failed) Debug**

There is no new production code to write for this task — `plant.py` and `integrate.py` are already correct per Tasks 3-4's own tests. If the energy or fall test fails, the bug is most likely in `_total_energy`'s transcription of the mass-matrix terms (they must match `plant.py`'s `A11/A12/A22/A33` verbatim) or in the `Kb=0.0` override not actually reaching `f()`. Use `superpowers:systematic-debugging` rather than adjusting the `1e-3` / `45.0` thresholds.

- [ ] **Step 4: Run full test suite to verify everything passes**

Run: `uv run pytest -v`
Expected: PASS, 8 tests total (2 params + 2 plant smoke + 2 integrate + 2 section-11 plant tests)

- [ ] **Step 5: Commit**

```bash
git add tests/test_plant.py
git commit -m "test: add section 11 energy-conservation and open-loop-fall plant tests"
```

---

## Self-review notes

- **Spec coverage:** `params.py` (section 6.1 table, frozen dataclass, no magic numbers elsewhere) ✓. `plant.py` (section 5 equations, `np.linalg.solve` for the 2x2 system, not hand-inverted) ✓. `integrate.py` (fixed-step RK4, ZOH — ZOH is structural: `u` is sampled once per `rk4_step` call and reused across all four stages) ✓. Section 11 plant-only tests (energy conservation, open-loop fall) ✓. Explicitly deferred: PDF verification (flagged above, not something I can do without the source document), voltage clipping via `V_batt` (section 5 note — `V_batt` is stated to be a disturbance-model *state*, which is `sim/disturbances.py`, step 4; adding it now would mean inventing a value with no home), numeric-Jacobian test (needs `sim/linearize.py`, step 2).
- **Placeholder scan:** no TBD/TODO markers; every step has runnable code and exact commands.
- **Type consistency:** `f(x, v_l, v_r, p)` signature is identical across Tasks 3 and 5; `simulate(f, x0, u_fn, dt, n_steps)` and `rk4_step(f, x, u, dt)` signatures match between Task 4's definition and Task 5's usage; `PlantParams` field names (`Jw, L, Jpsi, Jphi, Rm, Kb, Kt, fm, fw`) are used consistently in `plant.py` and both test files.
