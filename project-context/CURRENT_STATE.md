# Current State — Echo Balancer

Last verified: 2026-09-21, against branch `master` at commit `96f914b`
(`git log --oneline -1`). **Re-verify against `git log` if this looks stale —
this file is a snapshot, not a live view.**

## What currently works (verified: 84/84 tests passing, `uv run pytest -q`)

Build order per `CLAUDE.md` §14 — steps 1-5 complete:

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

5. **World + wall-following** (`sim/world.py`, `sim/sensors.py::ultrasonic`,
   `sim/control.py` additions) — **"Full wall-following" per the user's explicit
   choice, not a reduced version**
   - `sim/world.py`: `cast_ray` (ray-vs-two-infinite-parallel-walls geometry,
     `ray_parallel_eps` tolerance now in `CorridorParams` not hardcoded) and
     `pose_velocity` (unicycle kinematics, plugs into `rk4_step` like `plant.f`).
   - `sim/sensors.py::ultrasonic`: HC-SR04 model (20Hz, 0.02-4m, σ≈3mm, 2%
     dropout) — closes a gap left open since step 3.
   - `sim/control.py::design_lqr_speed_servo`: a **new, separate** 5-state
     integral-augmented LQR (does not modify `design_lqr_balance`) — needed
     because feeding a moving reference into the 4-state balance LQR gives 42%
     steady-state speed error (no integral action). Verified gain
     `K=[-0.906,-53.30,-2.312,-5.011,-0.316]`, all closed-loop poles stable.
   - `sim/control.py::yaw_p_control`: **proportional-only, not PD** — the
     plant's yaw pole (~-95.6 rad/s) is too fast relative to the 200Hz control
     rate for any nonzero `Kd` to stay stable (verified: `Kd=0.02` diverges to
     ~1e36 within 3s). `YawControlParams` deliberately has no `Kd` field.
   - `sim/control.py::wall_following_control`, `front_threshold_speed_adjust`:
     side-distance-error → PD → yaw-rate reference; front-distance threshold →
     speed slowdown, per §9.
   - `tests/test_wall_following.py`: one closed-loop integration test — full
     nonlinear plant + RK4, true-state speed-servo feedback, true-φ̇ yaw
     control, **real noisy/dropout-prone `ultrasonic()`** driving the outer
     loop, corridor ray casting, pose integration via `pose_velocity`/`rk4_step`
     at 1ms plant resolution. Robot starts 0.2m off target, converges to
     within 0.021 of the 0.5m target distance over 15 simulated seconds,
     never touches either wall. Bypasses the Kalman filter for balance/speed
     state (uses true state directly, matching step 2's recovery-test
     precedent) — a documented, temporary simplification pending `run.py`.

Full test run (this session): `84 passed` (~40-55s depending on machine — the
new wall-following closed-loop test adds ~2.5s for its 15-simulated-second run
at 1kHz/200Hz/20Hz).

## What does not exist yet (build order steps 6-8, not started)

- `sim/run.py` — the general-purpose closed-loop episode runner. Note: the
  §11 NIS test's closed loop (`tests/test_estimator.py::_run_nominal_closed_loop`),
  the gate tests' closed loop (`tests/test_gate.py::_closed_loop_with_gate`),
  and now the wall-following closed loop (`tests/test_wall_following.py`) are
  THREE test-only harnesses with overlapping plumbing, explicitly *not*
  a preview of `run.py`'s eventual design — see
  [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md) for the "should these be unified" question.
  **This is the next step.**
- `experiments/run_batch.py`, `analysis/metrics.py`, `analysis/plots.py`,
  `analysis/animate.py` — batch evaluation and reporting (Milestone 2).
- `quantum/qubo.py` — QUBO formulation + grid vs. `GroverOptimizer` comparison.
  `qiskit`/`qiskit-optimization` are not yet in `pyproject.toml` (deliberately —
  see [DECISIONS.md](DECISIONS.md)).
- Corridor corners/dead-ends in `sim/world.py` — the current corridor is two
  infinite parallel walls, so `front_threshold_speed_adjust` is implemented and
  unit-tested but has nothing to meaningfully trigger it head-on in the closed
  loop yet.
- Hardware parameter set (`CLAUDE.md` §6.2, `REAL_SPEC`/`MEASURED`/`ESTIMATED`/
  `PLACEHOLDER` tags) — no hardware has arrived yet; only the NXTway-GS §6.1 values
  are in use.

## Current implementation details worth knowing

- **`sim/params.py`** currently defines: `PlantParams`, `LQRBalanceParams`,
  `SensorParams`, `EstimatorParams`, `GateParams`, `DisturbanceParams`,
  `CorridorParams`, `SpeedServoParams`, `YawControlParams`, `WallFollowParams`
  — each with a `default_*()` factory.
- **`sim/control.py`** now has five public functions: `design_lqr_balance`
  (step 2), `design_lqr_speed_servo` (step 5, separate from balance),
  `yaw_p_control`, `wall_following_control`, `front_threshold_speed_adjust`
  (step 5).
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
- Test suite: 12 test modules under `tests/`, one per `sim/` module (`test_params`,
  `test_plant`, `test_integrate`, `test_linearize`, `test_control`, `test_sensors`,
  `test_estimator`, `test_gate`, `test_disturbances`, `test_world`) plus
  `test_wall_following.py` (integration-only, no matching `sim/` module). No
  `test_run.py` yet.
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

Per `CLAUDE.md` §14, the next build-order step is **step 6**: `sim/run.py` →
`experiments/run_batch.py` → `analysis/metrics.py` (Milestone 2: "metrics table
for all three controllers" — naive, tilt-threshold gate, NIS gate).

## Immediate next steps

1. Design `sim/run.py`: one closed-loop episode → DataFrame, per `CLAUDE.md` §4.
   Consider unifying the three overlapping test-only closed-loop harnesses that
   now exist (`test_estimator.py`, `test_gate.py`, `test_wall_following.py`)
   rather than writing a fourth independent implementation — see
   [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md).
2. Implement the three controllers to compare (§12): naive (always NORMAL),
   tilt-threshold gate (|ψ| thresholds only), NIS gate (already built, steps
   3-4). Decide whether/how the step-5 wall-following stack (speed servo, yaw,
   wall-following) integrates into `run.py`'s episodes, or whether `run.py`'s
   first cut is balance-only-in-a-corridor and wall-following comes later.
3. `experiments/run_batch.py` + `analysis/metrics.py`: fall rate, false-fallback,
   missed-fallback, detection delay, progress — per §12's metrics list.

## Uncommitted / unusual repo state as of this session

- `research/echo_balancer_research.pdf` and `research/"echo_balancer_detailed (1).pdf"`
  remain untracked and ungitignored — purpose/provenance still unconfirmed with
  the user (flagged in [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md), not assumed). This has
  been true since 2026-09-20/21 and hasn't blocked anything; revisit only if the
  user raises it.
- Everything else is now committed (as of `96f914b`): `.claude/`, `.graphifyignore`,
  `graphify-out/`, `project-context/`, and all five steps' plan docs (step 5's
  plan doc was briefly untracked mid-step, fixed in the closing commit).
