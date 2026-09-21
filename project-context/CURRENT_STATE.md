# Current State — Echo Balancer

Last verified: 2026-09-20, against branch `master` at commit `a7d8406`
(`git log --oneline -1`). **Re-verify against `git log` if this looks stale —
this file is a snapshot, not a live view.**

## What currently works (verified: 34/34 tests passing, `uv run pytest -q`)

Build order per `CLAUDE.md` §14 — steps 1-3 complete:

1. **Plant model** (`sim/params.py`, `sim/plant.py`, `sim/integrate.py`)
   - Energy conservation to <0.1% over 5s with `f_m=0` and `Kb=0` (back-EMF zeroed
     too — see [DECISIONS.md](DECISIONS.md)).
   - Open-loop fall from ψ=1° with no control.
2. **Linearization + LQR** (`sim/linearize.py`, `sim/control.py`)
   - Numeric-vs-analytic Jacobian match (rtol 1e-6).
   - Linear eigenvalues match `CLAUDE.md` §6.1 sanity values
     (`{0, -241, +7.44, -6.52}`).
   - LQR closed-loop poles all Re<0; recovers from ψ₀=10° in the nonlinear sim
     (per-plant-step feedback, not yet the 200Hz ZOH loop — see
     [DECISIONS.md](DECISIONS.md)).
3. **Sensors + estimator + NIS** (`sim/sensors.py`, `sim/estimator.py`)
   - Gyro/accelerometer/encoder noise models match spec.
   - 5-state KF (`[θ,ψ,θ̇,ψ̇,b_g]`) predict/update, NIS computed in `update`.
   - §11 NIS-consistency test: nominal 60s closed loop, mean NIS and
     fraction-above-95%-quantile both within the (empirically-justified,
     see [DECISIONS.md](DECISIONS.md)) acceptance bands. **This test takes
     ~20-60s wall-clock (210s measured in this session) — that's expected, not a hang.**

Full test run (this session): `34 passed in 210.00s`.

## What does not exist yet (build order steps 4-8, not started)

- `sim/gate.py` — the Normal/Cautious/Halt state machine and the windowed ε_k
  statistic. **This is the actual contribution of the project and hasn't been
  built yet.**
- `sim/disturbances.py` — the 5 scheduled disturbance profiles (§8).
- `sim/world.py` — 2D corridor + ray casting (blocks the ultrasonic sensor model
  and wall-following).
- `sim/run.py` — the general-purpose closed-loop episode runner. Note: the §11 NIS
  test's closed loop (in `tests/test_estimator.py::_run_nominal_closed_loop`) is a
  test-only harness, explicitly *not* a preview of `run.py`'s design.
- `experiments/run_batch.py`, `analysis/metrics.py`, `analysis/plots.py`,
  `analysis/animate.py` — batch evaluation and reporting.
- `quantum/qubo.py` — QUBO formulation + grid vs. `GroverOptimizer` comparison.
  `qiskit`/`qiskit-optimization` are not yet in `pyproject.toml` (deliberately —
  see [DECISIONS.md](DECISIONS.md)).
- Speed-servo integral term, yaw PD, and wall-following in `sim/control.py` (spec'd
  in §9, scoped out of steps 1-2 explicitly).
- Hardware parameter set (`CLAUDE.md` §6.2, `REAL_SPEC`/`MEASURED`/`ESTIMATED`/
  `PLACEHOLDER` tags) — no hardware has arrived yet; only the NXTway-GS §6.1 values
  are in use.

## Current implementation details worth knowing

- **`sim/params.py`** currently defines: `PlantParams`, `LQRBalanceParams`,
  `SensorParams`, `EstimatorParams` — each with a `default_*()` factory. No
  `GateParams` or `DisturbanceParams` yet (they belong to the not-yet-built modules).
- **`EstimatorParams` defaults** (`Q_theta=Q_psi=1e-8`, `Q_theta_dot=Q_psi_dot=1e-6`,
  `P0_*=1e-4` except `P0_bg=1e-6`) are empirically tuned, not derived from a spec
  target — see [DECISIONS.md](DECISIONS.md) for the tuning story and
  [FAILED_APPROACHES.md](FAILED_APPROACHES.md) before changing them.
- Test suite: 8 test modules under `tests/`, one per `sim/` module (`test_params`,
  `test_plant`, `test_integrate`, `test_linearize`, `test_control`, `test_sensors`,
  `test_estimator`). No `test_gate.py`, `test_run.py`, etc. yet.

## Current configuration

- Python 3.12.8 (venv), managed by `uv` (`uv.lock` checked in).
- `pyproject.toml` deps: `numpy`, `scipy`; dev: `pytest`. `pythonpath=["."]`,
  `testpaths=["tests"]`.
- Git remote: `https://github.com/ShreeniG99/Echo-Balancer.git`, branch `master`.
- `nxtway_gs/` and `license.txt` are gitignored (third-party reference material,
  not redistributed).

## Current objective

Per `CLAUDE.md` §14, the next milestone is **build-order step 4**: `sim/gate.py` →
`sim/disturbances.py`, targeting "**Milestone 1: gate switches under one
disturbance.**" If behind schedule after that milestone, the spec says cut
wall-following first — not the gate itself.

## Immediate next steps

1. Add `GateParams` (τ₁, τ₂, N, T_dwell) to `sim/params.py`.
2. Implement `sim/gate.py`: the ε_k windowed-NIS statistic and the
   Normal→Cautious→Halt state machine with hysteresis and minimum dwell time,
   per `CLAUDE.md` §10 (no NORMAL→HALT skip without a full window above τ₂).
3. Implement `sim/disturbances.py`: the 5 scheduled disturbance profiles (§8),
   remembering the estimator must keep using the *nominal* model even when the
   plant is disturbed — that mismatch is the whole point.
4. Write the §11 gate tests: no mode change on a nominal 60s seeded run; enters
   CAUTIOUS within a stated time of each disturbance at a stated magnitude.
5. Only then move to `sim/world.py` (corridor + wall-following).

## Uncommitted / unusual repo state as of this session

- `research/"echo_balancer_detailed (1).pdf"` and `research/echo_balancer_research.pdf`
  (moved there from the repo root on 2026-09-21) sit untracked (not gitignored,
  not committed). Their purpose/provenance is unclear — flagged in
  [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md), not assumed.
- This session (2026-09-20) added `.claude/` (Graphify's Claude Code
  integration), `.graphifyignore`, `graphify-out/`, and `project-context/` — all
  new, none committed yet. See [MODEL_HANDOFF.md](MODEL_HANDOFF.md) for exactly
  what changed and why.
