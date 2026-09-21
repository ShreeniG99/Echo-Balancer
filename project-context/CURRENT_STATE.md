# Current State — Echo Balancer

Last verified: 2026-09-21, against branch `master` at commit `12be336`
(`git log --oneline -1`). **Re-verify against `git log` if this looks stale —
this file is a snapshot, not a live view.**

## What currently works (verified: 58/58 tests passing, `uv run pytest -q`)

Build order per `CLAUDE.md` §14 — steps 1-4 complete:

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
     see [DECISIONS.md](DECISIONS.md)) acceptance bands.
4. **Gate + disturbances** (`sim/gate.py`, `sim/disturbances.py`) — **Milestone 1 achieved**
   - `sim/gate.py`: windowed ε_k statistic (sum of last N=200 NIS samples) and the
     Normal/Cautious/Halt state machine with hysteresis and 0.5s minimum dwell
     time. τ1/τ2 are empirically calibrated, **not** literal χ²(3N) quantiles —
     see [DECISIONS.md](DECISIONS.md) and [FAILED_APPROACHES.md](FAILED_APPROACHES.md),
     this is a provable, not just empirical, deviation from `CLAUDE.md` §10's
     literal wording.
   - `sim/disturbances.py`: all 5 §8 disturbance profiles (6 functions, since
     "sensor fault" has 2 variants) as pure functions, plus `clip_voltage`
     (finally implementing the §5 voltage-clipping note deferred since step 1).
   - §11 gate tests: no mode change on a nominal 60s seeded run ✓; enters
     CAUTIOUS within 2s of each of the 6 disturbance scenarios ✓ (detection
     latencies ~0.03s-2.06s depending on disturbance — see plan doc).
   - **Two real findings from this step, worth carrying into any write-up:** the
     controller has a genuine stability boundary near `f_w≈0.03` (NaN/inf, not
     just falling over); NIS-based detection is measurably less sensitive to
     payload/CoM shifts and to isolated battery droop than to the other
     disturbance types under the current tuning — see
     [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md).

Full test run (this session): `58 passed` (~2 minutes — two ~60s-equivalent
closed-loop tests: the §11 NIS test and the nominal-60s gate test, plus six
~20s-equivalent per-disturbance gate tests).

## What does not exist yet (build order steps 5-8, not started)

- `sim/world.py` — 2D corridor + ray casting (blocks the ultrasonic sensor model
  and wall-following). **This is the next step.**
- `sim/run.py` — the general-purpose closed-loop episode runner. Note: both the
  §11 NIS test's closed loop (`tests/test_estimator.py::_run_nominal_closed_loop`)
  and the gate tests' closed loop (`tests/test_gate.py::_closed_loop_with_gate`)
  are test-only harnesses with some duplication between them, explicitly *not*
  a preview of `run.py`'s eventual design — see
  [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md) for the "should these be unified" question.
- `experiments/run_batch.py`, `analysis/metrics.py`, `analysis/plots.py`,
  `analysis/animate.py` — batch evaluation and reporting.
- `quantum/qubo.py` — QUBO formulation + grid vs. `GroverOptimizer` comparison.
  `qiskit`/`qiskit-optimization` are not yet in `pyproject.toml` (deliberately —
  see [DECISIONS.md](DECISIONS.md)).
- Speed-servo integral term, yaw PD, and wall-following in `sim/control.py` (spec'd
  in §9, scoped out of steps 1-4 explicitly). **Note:** the missing speed-servo
  integral term is the root cause of the gate's chi2(3N)-quantile deviation
  (see [FAILED_APPROACHES.md](FAILED_APPROACHES.md)) — worth remembering if this
  gets revisited.
- Hardware parameter set (`CLAUDE.md` §6.2, `REAL_SPEC`/`MEASURED`/`ESTIMATED`/
  `PLACEHOLDER` tags) — no hardware has arrived yet; only the NXTway-GS §6.1 values
  are in use.

## Current implementation details worth knowing

- **`sim/params.py`** currently defines: `PlantParams`, `LQRBalanceParams`,
  `SensorParams`, `EstimatorParams`, `GateParams`, `DisturbanceParams` — each with
  a `default_*()` factory.
- **`EstimatorParams` defaults** (`Q_theta=Q_psi=1e-8`, `Q_theta_dot=Q_psi_dot=1e-6`,
  `P0_*=1e-4` except `P0_bg=1e-6`) are empirically tuned, not derived from a spec
  target — see [DECISIONS.md](DECISIONS.md) for the tuning story and
  [FAILED_APPROACHES.md](FAILED_APPROACHES.md) before changing them.
- **`GateParams` defaults** (`N=200, tau1=1300, tau2=1800, tau1_exit=1040,
  tau2_exit=1300, T_dwell=0.5`) are likewise empirically calibrated — same caveat.
- **`DisturbanceParams`** bundles genuine disturbance-profile values with two
  test-harness-only setup values (`battery_droop_companion_push_magnitude`,
  `payload_shift_test_psi0_deg`) needed to make those two disturbances observable
  at all — see `sim/params.py`'s own docstring before using these in `experiments/`.
- Test suite: 10 test modules under `tests/`, one per `sim/` module (`test_params`,
  `test_plant`, `test_integrate`, `test_linearize`, `test_control`, `test_sensors`,
  `test_estimator`, `test_gate`, plus `test_disturbances`). No `test_world.py`,
  `test_run.py`, etc. yet.
- `tests/test_gate.py` is now 12 tests / ~300+ lines spanning fast unit tests
  (toy `N=3` state-machine logic) and slow closed-loop integration tests (60s +
  six 20s simulations, ~90s total) — flagged as a candidate for splitting into
  two files if it grows further (not urgent).

## Current configuration

- Python 3.12.8 (venv), managed by `uv` (`uv.lock` checked in).
- `pyproject.toml` deps: `numpy`, `scipy`; dev: `pytest`. `pythonpath=["."]`,
  `testpaths=["tests"]`.
- Git remote: `https://github.com/ShreeniG99/Echo-Balancer.git`, branch `master`.
- `nxtway_gs/` and `license.txt` are gitignored (third-party reference material,
  not redistributed). `graphify-out/cost.json` and `graphify-out/cache/` are also
  gitignored (per-machine, regenerable).

## Current objective

Per `CLAUDE.md` §14, the next build-order step is **step 5**: `sim/world.py` →
wall-following. If behind schedule, the spec says cut wall-following first — the
2D corridor/ray-casting core of `world.py` (needed for the ultrasonic model) is
lower-priority to drop than the gate was.

## Immediate next steps

1. Implement `sim/world.py`: 2D corridor geometry + ray casting (`CLAUDE.md` §7's
   ultrasonic model — HC-SR04, 20Hz, range 0.02-4m, σ≈3mm, 2% dropout — has been
   waiting on this since step 3; `SensorParams` already has the ultrasonic fields).
2. Implement wall-following in `sim/control.py`: side ultrasonic distance error →
   PD → yaw-rate reference; front ultrasonic below threshold → slow down (§9).
3. This is also the natural point to reconsider the yaw PD and speed-servo
   integral term from §9 that steps 1-4 deliberately deferred — check whether
   wall-following needs them before deferring further.

## Uncommitted / unusual repo state as of this session

- `research/echo_balancer_research.pdf` and `research/"echo_balancer_detailed (1).pdf"`
  remain untracked and ungitignored — purpose/provenance still unconfirmed with
  the user (flagged in [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md), not assumed). This has
  been true since 2026-09-20/21 and hasn't blocked anything; revisit only if the
  user raises it.
- Everything else is now committed (as of `12be336`): `.claude/`, `.graphifyignore`,
  `graphify-out/`, `project-context/`, and this step's plan doc.
