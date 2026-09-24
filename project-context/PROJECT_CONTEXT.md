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
sim/gate.py            # Normal/Cautious/Halt state machine
sim/disturbances.py    # scheduled disturbance profiles
sim/world.py           # 2D corridor geometry + ray casting
sim/run.py             # one closed-loop episode -> DataFrame
experiments/run_batch.py  # batch evaluation across the 3 controllers x 7 scenarios x N seeds
analysis/metrics.py    # fall rate / false-fallback / missed-fallback / detection delay / progress
analysis/plots.py      # episode timeline, cross-controller comparison, tau1/tau2 sensitivity heatmap
analysis/animate.py    # 3D scene render (PyVista/VTK, off-screen) of a logged episode -> MP4
quantum/qubo.py        # cost table -> quadratic surrogate -> grid vs GroverOptimizer
tests/                 # one test module per sim/ module, required before a module is "done"
```

**Every file `CLAUDE.md` §4 names now exists — all build-order steps (§14)
are complete.** See [CURRENT_STATE.md](CURRENT_STATE.md) for exact detail and
the real cross-controller/Grover results.

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
- **`sim/control.py`** — `design_lqr_balance` (balance-only 4-state LQR),
  `design_lqr_speed_servo` (separate 5-state integral-augmented LQR for forward-
  speed tracking), `yaw_p_control` (proportional-only, permanently — see
  DECISIONS.md), `wall_following_control`, `front_threshold_speed_adjust`.
- **`sim/sensors.py`** — `gyro`, `accelerometer`, `encoder`, `ultrasonic`
  (HC-SR04 model, consumes `sim.world.cast_ray`'s output).
- **`sim/estimator.py`** — 5-state `[θ,ψ,θ̇,ψ̇,b_g]` discrete KF (`discretize`,
  `predict`, `update`) with NIS computed in `update`.
- **`sim/gate.py`** — `step_gate` (windowed-NIS Normal/Cautious/Halt state
  machine) and `step_tilt_gate` (the tilt-threshold baseline controller,
  CLAUDE.md §12).
- **`sim/world.py`** — `cast_ray` (two-infinite-parallel-walls corridor
  geometry), `pose_velocity` (unicycle kinematics).
- **`sim/run.py`** — `run_episode(config, seed) -> DataFrame`, the one real
  closed-loop implementation, parameterized by `ControllerKind`
  (NAIVE/TILT_GATE/NIS_GATE).
- **`experiments/run_batch.py` + `analysis/metrics.py`** — the Milestone 2
  cross-controller comparison. Real result: NIS gate has zero missed
  detections across 60 disturbance episodes vs. tilt-threshold's 50% miss
  rate; fall rate 42.9%→28.6% (naive→NIS gate).
- **`quantum/qubo.py`** — Milestone 3. Fits a quadratic surrogate to the true
  64-candidate cost table (GroverOptimizer can't take an arbitrary black-box
  objective), runs GroverOptimizer multiple trials (it's probabilistic, not a
  single-shot guarantee). Real result: no speedup (by design), and the
  surrogate's true optimum margin is only ~0.12% of its range — a genuine
  value-qubit-resolution limit, not a bug.
- **`analysis/animate.py`** — 3D scene render (PyVista/VTK, off-screen) of any
  logged episode. **Scope change from the original 2D side/top-view plan,
  user-approved** — see DECISIONS.md. Self-manages a virtual display
  (`_ensure_display`) since this container has no GPU/EGL/OSMesa.
- **`analysis/plots.py`** — `plot_episode_timeline` (psi/epsilon over time,
  mode-shaded), `plot_controller_comparison` (small-multiple bars, one metric
  per panel), `plot_threshold_sensitivity` (a heatmap over
  `analysis.metrics.threshold_sensitivity`'s tau1/tau2 sweep — CLAUDE.md
  §12's "threshold sensitivity" metric). Plain matplotlib, `Agg` backend —
  no display needed, unlike `animate.py`'s PyVista/VTK.

## Technologies

Python ≥3.11, `uv` for env/deps, `numpy`, `scipy` (`solve_continuous_are`,
`expm`, `chi2`), `pandas`+`pyarrow` (Parquet), `joblib` (batch parallelism),
`matplotlib` (static plots, `Agg` backend), `pyvista`+`vtk`+`imageio`/
`imageio-ffmpeg` (3D animation, see the scope-change note below),
`qiskit`==2.5.2 + `qiskit-optimization`==0.7.0 + `qiskit-aer`==0.17.2
(verified `GroverOptimizer` import before writing quantum code), `pytest`.
Not yet added as a direct dependency: `python-control` (nothing imports it —
`scipy`'s `solve_continuous_are` has covered every LQR design so far).

## Project-specific terminology

- **NIS** — Normalized Innovation Squared, `ν'S⁻¹ν`, the KF's own self-consistency
  statistic; ~χ²(3) when the filter's model matches reality (3 = measurement dim).
- **ε_k** — the *windowed* statistic, sum of the last N samples of NIS, ~χ²(3N)
  in theory (empirically, the tail is much heavier than that i.i.d. model
  predicts — see DECISIONS.md/FAILED_APPROACHES.md). The gate switches on ε_k,
  not raw NIS.
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
- Out of scope, do not build: cave arena, magnet-polarity detection, CAD, physics
  engines (MuJoCo/Gazebo/CoppeliaSim), ROS, GUIs, online learning, neural networks.
  **3D rendering specifically is now in scope** (user-approved mid-project scope
  change, see DECISIONS.md) — `analysis/animate.py` renders an already-simulated
  episode's logged state as a 3D scene (PyVista/VTK, off-screen only, no physics
  computed there). This doesn't reopen the physics-engine/ROS/GUI bans.
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
