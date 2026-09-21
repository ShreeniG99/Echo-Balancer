# Project Context — Echo Balancer

Durable orientation for anyone (human or model) picking up this project cold.
This file describes what the project *is*; see [CURRENT_STATE.md](CURRENT_STATE.md) for
what's actually built right now.

## Purpose / problem statement

Echo Balancer is a simulation of a two-wheeled self-balancing robot that monitors its
own control confidence using the **Kalman-filter normalized innovation squared
(NIS, χ²)** and switches between three modes — **Normal → Cautious → Halt** — when
the filter's model stops matching reality (disturbances, sensor faults, payload
shifts). The gate's thresholds (τ₁, τ₂, window size N, dwell time) are selected
offline by comparing classical exhaustive/grid search against **Grover Adaptive
Search** (`qiskit_optimization.GroverOptimizer`) on the same QUBO.

**The contribution is the gate + its honest evaluation, not the robot and not a
quantum speedup claim.** If Grover doesn't beat classical search here, the paper
says so.

The full, authoritative spec is [`CLAUDE.md`](../CLAUDE.md) at the repo root — this
file is a compressed index into it, not a replacement. When they disagree, `CLAUDE.md`
wins; stop and ask the user rather than guessing.

## Architecture (planned, per `CLAUDE.md` §4)

```
sim/params.py        # ALL physical/sensor/controller/gate parameters (no magic numbers elsewhere)
sim/plant.py          # nonlinear dynamics f(x, v_l, v_r, disturbances)
sim/integrate.py       # fixed-step RK4, zero-order-hold control
sim/linearize.py       # analytic A,B at upright equilibrium + numeric check
sim/sensors.py         # gyro, accelerometer, encoders, ultrasonic
sim/control.py         # LQR balance, speed/yaw servo, wall-following
sim/estimator.py       # Kalman filter + NIS
sim/gate.py            # Normal/Cautious/Halt state machine            [NOT YET BUILT]
sim/disturbances.py    # scheduled disturbance profiles                [NOT YET BUILT]
sim/world.py           # 2D corridor geometry + ray casting            [NOT YET BUILT]
sim/run.py             # one closed-loop episode -> DataFrame          [NOT YET BUILT]
experiments/run_batch.py                                               [NOT YET BUILT]
analysis/metrics.py, analysis/plots.py, analysis/animate.py            [NOT YET BUILT]
quantum/qubo.py        # cost table -> QUBO; grid vs GroverOptimizer   [NOT YET BUILT]
tests/                 # one test module per sim/ module, required before a module is "done"
```

See [CURRENT_STATE.md](CURRENT_STATE.md) for exactly which of these exist today.

## Key components (built so far)

- **`sim/params.py`** — every physical/sensor/LQR/estimator constant as a frozen
  dataclass + `default_*()` factory. This is the single source of truth for numbers;
  nothing else in `sim/` may hardcode a physical constant.
- **`sim/plant.py`** — nonlinear 6-state EOM (θ, ψ, φ + rates) from the NXTway-GS
  model, solved via `np.linalg.solve` on the coupled θ/ψ 2×2 system per step.
- **`sim/integrate.py`** — generic fixed-step RK4 (`rk4_step`) + a ZOH `simulate` loop.
- **`sim/linearize.py`** — analytic Jacobian `linearize(p)` (full 6-state) and
  `linearize_planar(p)` (reduced 4-state `[θ,ψ,θ̇,ψ̇]`), both verified against a
  numeric finite-difference Jacobian.
- **`sim/control.py`** — `design_lqr_balance(p, lqr_p)`: continuous-ARE LQR gain on
  the planar model. Speed servo, yaw PD, and wall-following are spec'd (§9) but not
  yet implemented — see below.
- **`sim/sensors.py`** — `gyro`, `accelerometer`, `encoder` pure noise models.
  Ultrasonic is deferred to `sim/world.py` (needs corridor ray-casting).
- **`sim/estimator.py`** — 5-state `[θ,ψ,θ̇,ψ̇,b_g]` discrete KF (`discretize`,
  `predict`, `update`) with NIS computed in `update`.

## Technologies

Python ≥3.11 (dev venv currently 3.12.8), `uv` for env/deps, `numpy`, `scipy`
(`solve_continuous_are`, `expm`, `chi2`), `pytest`. Not yet added:
`python-control`, `pandas`+`pyarrow`, `joblib`, `matplotlib`, `qiskit` +
`qiskit-optimization` (deliberately deferred — see
[DECISIONS.md](DECISIONS.md)).

## Project-specific terminology

- **NIS** — Normalized Innovation Squared, `ν'S⁻¹ν`, the KF's own self-consistency
  statistic; ~χ²(3) when the filter's model matches reality (3 = measurement dim).
- **ε_k** — the *windowed* statistic, sum of the last N samples of NIS, ~χ²(3N).
  The gate switches on ε_k, not raw NIS. (Not yet implemented — `sim/gate.py`.)
- **Gate** — the Normal/Cautious/Halt state machine driven by ε_k vs. thresholds
  τ₁ < τ₂, with hysteresis and minimum dwell time.
- **Planar model** — the reduced 4-state `[θ,ψ,θ̇,ψ̇]` balance-only linearization;
  yaw (φ) fully decouples from it at the equilibrium (see
  [DECISIONS.md](DECISIONS.md)).
- **b_g** — gyro bias, modeled as a random walk and estimated as a 5th KF state.
- **NXTway-GS** — the LEGO Mindstorms self-balancing robot (MathWorks File Exchange
  #19147) whose plant equations this project's `sim/plant.py` is transcribed from.
  Reference PDFs live in `nxtway_gs/` (gitignored, not redistributed).

## Constraints (from `CLAUDE.md`, non-negotiable unless the user changes the spec)

- No magic numbers outside `sim/params.py`.
- Every stochastic function takes an explicit `np.random.Generator`; every run is
  reproducible from `(config, seed)`.
- Pure functions, no global state, explicit state threading (see how `gyro`'s bias
  and `rk4_step`'s state are passed in/out rather than mutated).
- SI units internally; degrees only in plot/log labels suffixed `_deg`.
- A module isn't "done" until its `tests/` counterpart passes.
- Never loosen a failing physics test's tolerance to make it pass — find the bug.
- Out of scope, do not build: cave arena, magnet-polarity detection, CAD, 3D physics
  engines (MuJoCo/Gazebo/CoppeliaSim), ROS, GUIs, online learning, neural networks.
- Grover Adaptive Search makes **no speedup claim** — report the comparison honestly
  either way.

## Important files

| File | Role |
|---|---|
| [`CLAUDE.md`](../CLAUDE.md) | The full spec. Single source of truth. Read before every task. |
| `sim/params.py` | All numeric constants. |
| `docs/superpowers/plans/*.md` | Step-by-step implementation plans with embedded design-decision writeups and pre-verified math for each build step. Read these for *why*, not just *what*. |
| `tests/` | The executable definition of "done" for each module (CLAUDE.md §11). |
| `graphify-out/graph.json` | Queryable code-structure graph (see below). |

## External / official resources

- Y. Yamamoto, *NXTway-GS Model-Based Design*, MathWorks File Exchange #19147 —
  plant equations and parameter table. Local copy: `nxtway_gs/docs/` (gitignored).
- S. Kim, S. Kwon, IJCAS 13(4) 2015 — derivation cross-check only (its parameters
  are for a 41 kg robot; do not use its numbers).
- Mehra & Peschon, *Automatica* 1971 — NIS/χ² foundation.
- Gilliam, Woerner, Gonciulea, *Quantum* 5:428, 2021 — Grover Adaptive Search /
  `GroverOptimizer`.
- Graphify (this repo's knowledge-graph tool): https://github.com/Graphify-Labs/graphify

## What a new model must understand before touching code

1. Read `CLAUDE.md` in full — it is checked into the repo precisely so no model has
   to be told the spec verbally.
2. Read [CURRENT_STATE.md](CURRENT_STATE.md) for what's actually built vs. planned.
3. Read [DECISIONS.md](DECISIONS.md) and [FAILED_APPROACHES.md](FAILED_APPROACHES.md)
   before changing tuned parameters (especially `EstimatorParams` in `sim/params.py`)
   or test tolerances — several non-obvious choices there are the result of real
   empirical debugging, not arbitrary defaults.
4. Prefer `graphify query "<question>"` / `graphify explain "<symbol>"` /
   `graphify path "<A>" "<B>"` over grepping the whole repo for code-structure
   questions (see [MODEL_HANDOFF.md](MODEL_HANDOFF.md)).
