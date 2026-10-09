# Current State — Echo Balancer

Last verified: 2026-09-22, against branch `step6-run-batch-metrics` (worktree
`.worktrees/step6-run-batch-metrics`) at commit `0d29d83` plus this session's
`project-context/` update. **Re-verify against `git log` if this looks stale —
this file is a snapshot, not a live view.**

## What currently works (verified: 122/122 tests passing, `uv run pytest -q`)

Build order per `CLAUDE.md` §14 — steps 1-6 complete:

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

6. **`sim/run.py`, `experiments/run_batch.py`, `analysis/metrics.py`** (step 6) — **Milestone 2 achieved**
   - `sim/run.py::run_episode(config, seed) -> pd.DataFrame`: the general-purpose
     closed-loop episode runner (`CLAUDE.md` §4), unifying the three overlapping
     test-only closed-loop harnesses that had accumulated across
     `tests/test_estimator.py`, `tests/test_gate.py`, and `tests/test_wall_following.py`
     into one implementation — resolving the "should these be unified" question
     [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md) previously left open. `ControllerType`
     (`NAIVE`/`TILT_THRESHOLD`/`NIS_GATE`) and `EpisodeConfig`
     (`controller`, `T`, `wall_following`, `x0_psi_deg`, `disturbance`,
     `disturbance_name`) select the episode. **Does not** unify onto one shared
     control law: `wall_following=False` episodes always use
     `design_lqr_balance` fed by the KF estimate (byte-for-byte the already-
     calibrated step 3/4 algorithm — `GateParams` needs no recalibration);
     `wall_following=True` episodes use `design_lqr_speed_servo` fed by the
     TRUE plant state (matching step 5's precedent), with a mode-scaled speed
     reference (NORMAL: full `corridor_theta_dot_ref_nominal`; CAUTIOUS:
     x0.4 and the new cautious gain `K5_cautious`; HALT: `0.0`, nominal gain)
     — see [DECISIONS.md](DECISIONS.md) for why a single shared control law
     was tried and rejected (it silently breaks push-disturbance detection).
     Both `step_gate` (NIS) and the new `step_tilt_gate` (tilt-threshold
     baseline) always run every tick regardless of which one drives `mode`,
     so all three controllers are directly comparable on identical
     seeds/disturbances. Implements `CLAUDE.md` §5's fall condition
     (`|psi| > 45°` truncates the episode, motors off, `fallen=True`) —
     verified this is not optional plumbing (see below). Voltage clipping is
     applied per-motor (`v_l`, `v_r` independently), a deliberate, tested
     divergence from the old test harnesses' clip-then-split approach, needed
     for wall-following's differential yaw drive.
   - `sim/gate.py`'s `TiltGateState`/`initial_tilt_gate_state`/`step_tilt_gate`:
     the CLAUDE.md §12 "plain safety-filter baseline" — mode switches on
     `|kf.x_hat[1]|` (KF-estimated psi, never true psi) thresholds only, no
     windowed statistic. Calibrated (`TiltGateParams`, `sim/params.py`) to
     have zero false trips over a 10-seed nominal sweep, and to detect
     `gyro_bias_fault`/`battery_droop`/`payload_shift` but — by design, not
     miscalibration — to miss `push`/`surface_change`/`accel_noise_fault`
     (CLAUDE.md §12 frames this contrast as the point of the comparison).
   - `sim/params.py` additions: `RunParams` (`dt_plant`, `dt_control`,
     `fall_psi_threshold`, `cautious_speed_scale`,
     `corridor_theta_dot_ref_nominal=0.3` — deliberately not
     `tests/test_wall_following.py`'s `2.0`, see [DECISIONS.md](DECISIONS.md)),
     `TiltGateParams`, and `default_speed_servo_params_cautious()` (all five
     `SpeedServoParams` Q weights divided by 5, verified stable).
   - `experiments/run_batch.py`: runs all three controllers across 10 seeds
     and 7 scenarios (nominal + 6 `CLAUDE.md` §8 disturbances) for the
     balance-only architecture, plus a separate nominal-only corridor batch
     (for the Progress metric), via `joblib.Parallel`, writing
     `steps_*.parquet`/`episodes_*.parquet` under `experiments/results/`
     (gitignored, regenerable). Workers receive the resolved scenario tuple
     as an explicit parameter rather than looking it up from the module-level
     `DISTURBANCE_SCENARIOS` dict — required because `joblib`'s `loky` backend
     spawns fresh worker processes on this Windows machine (`spawn` is the
     only available start method), so a worker-side lookup would silently
     read on-disk module state instead of a parent-process-patched value
     (caught by a test failing with 12000 rows instead of 200) — see
     [DECISIONS.md](DECISIONS.md).
   - `analysis/metrics.py`: `fall_rate`, `false_fallback_fraction`,
     `missed_fallback`, `detection_delay`, `progress` — five of `CLAUDE.md`
     §12's six metrics (threshold-sensitivity sweeps are deferred to
     `quantum/qubo.py`'s grid search, step 7). Operates on the
     `(steps_df, episodes_df)` pair `run_batch.py` produces; tested against
     small hand-built synthetic DataFrames, no full simulation needed.
   - **Real findings from this step, worth carrying into any write-up, not bugs
     to chase:** sustained forward motion with active yaw correction
     significantly inflates the windowed NIS statistic (confirms a suspicion
     `OPEN_PROBLEMS.md` had flagged as untested) — `run.py`'s corridor default
     speed (`0.3` rad/s) was chosen specifically to stay safe under this
     effect, well below `tests/test_wall_following.py`'s own `2.0` rad/s demo
     speed; push-disturbance detection, already thin in the balance-only
     architecture (3.5% margin over `tau1`), does not survive the switch to
     `design_lqr_speed_servo` in corridor mode at all (a known, expected miss,
     not a regression); and HALT mode's transient overshoots the NORMAL-mode
     cruise speed before settling, because `TiltGateParams.T_dwell` is much
     shorter than the pitch-recovery pole's time constant — see
     [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md) for the full detail on all three.
     **A fourth finding, discovered later while verifying the `progress()`
     empty-DataFrame fix against a real end-to-end batch run (not the tests'
     synthetic DataFrames):** the real 10-seed nominal corridor batch shows a
     ~20% fall rate independent of gate choice (`naive`/`nis_gate` both 0.2,
     same falling seeds, near-identical fall times) plus an additional
     mode-switching-correlated fall rate specific to `tilt_threshold` (0.5).
     `corridor_theta_dot_ref_nominal=0.3`'s "safe" designation (Decision
     below) was only ever validated against gate-triggering at `seed=42`, not
     fall rate across a real seed population — see
     [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md) and [DECISIONS.md](DECISIONS.md).

Full test run (this session): `126 passed` (several minutes — step 6 adds two
new 60s closed-loop corridor tests plus a 10-seed x 7-scenario x 3-controller
batch smoke test on top of the existing steps 1-5 suite).

## What does not exist yet (build order steps 7-8, not started)

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
  `CorridorParams`, `SpeedServoParams`, `YawControlParams`, `WallFollowParams`,
  and (step 6) `RunParams`, `TiltGateParams` — each with a `default_*()`
  factory, plus `default_speed_servo_params_cautious()` alongside
  `default_speed_servo_params()`.
- **`sim/control.py`** still has the same five public functions from step 5:
  `design_lqr_balance` (step 2), `design_lqr_speed_servo` (step 5, separate
  from balance), `yaw_p_control`, `wall_following_control`,
  `front_threshold_speed_adjust` — step 6 did not modify `control.py`, only
  composed these functions inside `sim/run.py`.
- **`sim/gate.py`** (step 6 addition): `TiltGateState`,
  `initial_tilt_gate_state`, `step_tilt_gate` — the tilt-threshold baseline,
  alongside the existing NIS-based `GateState`/`step_gate` (step 4, unmodified).
- **`sim/run.py`** (new, step 6): `ControllerType`, `EpisodeConfig`,
  `run_episode(config, seed) -> pd.DataFrame`. See "What currently works"
  above for the full design; short version — one function drives the whole
  plant/sensor/estimator/gate/control stack for one episode and returns a
  per-control-tick DataFrame with columns `t, theta, psi, phi, theta_dot,
  psi_dot, phi_dot, pos_x, pos_y, mode, nis, epsilon, front_dist, right_dist,
  v_l, v_r, fallen` (`pos_x`/`pos_y`/`front_dist`/`right_dist` are `NaN` for
  `wall_following=False` episodes).
- **`EstimatorParams` defaults** (`Q_theta=Q_psi=1e-8`, `Q_theta_dot=Q_psi_dot=1e-6`,
  `P0_*=1e-4` except `P0_bg=1e-6`) are empirically tuned, not derived from a spec
  target — see [DECISIONS.md](DECISIONS.md) for the tuning story and
  [FAILED_APPROACHES.md](FAILED_APPROACHES.md) before changing them. Step 6
  confirmed (didn't need to retune) that these values hold up unmodified for
  the balance-only path; the corridor path's false-fallback-at-speed finding
  is a *separate*, newly-confirmed gap in the same params (see
  [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md)), not evidence these defaults are wrong
  for what they were originally tuned against.
- **`GateParams` defaults** (`N=200, tau1=1300, tau2=1800, tau1_exit=1040,
  tau2_exit=1300, T_dwell=0.5`) are likewise empirically calibrated — same caveat.
- **`TiltGateParams` defaults** (`psi1=0.0087, psi2=0.0175, psi1_exit=0.006,
  psi2_exit=0.012, T_dwell=0.5`) are step 6's own empirical calibration for the
  baseline gate — see [DECISIONS.md](DECISIONS.md)'s "Design decision 4."
- **`DisturbanceParams`** bundles genuine disturbance-profile values with two
  test-harness-only setup values (`battery_droop_companion_push_magnitude`,
  `payload_shift_test_psi0_deg`) needed to make those two disturbances observable
  at all — see `sim/params.py`'s own docstring before using these in `experiments/`.
  Step 6's `experiments/run_batch.py` fires the battery-droop companion push at
  `battery_droop_onset + battery_droop_duration` (after the ramp completes), not
  at the droop's own onset — see `FAILED_APPROACHES.md`'s existing entry on why.
- Test suite: 15 test modules under `tests/` — the 12 from steps 1-5
  (`test_params`, `test_plant`, `test_integrate`, `test_linearize`,
  `test_control`, `test_sensors`, `test_estimator`, `test_gate`,
  `test_disturbances`, `test_world`, `test_wall_following`) plus step 6's
  three new modules: `test_run.py` (14 tests: balance-only + corridor
  `run_episode` behavior, reproducibility, fall condition), `test_metrics.py`
  (8 tests: `analysis/metrics.py` against small synthetic DataFrames),
  `test_run_batch.py` (3 tests: `experiments/run_batch.py` wiring/schema
  smoke tests plus the multi-episode-concat regression test added after the
  worker-scenario-lookup bug).
- `tests/test_gate.py` is still ~12 tests spanning fast unit tests (toy `N=3`
  state-machine logic) and slow closed-loop integration tests — unchanged by
  step 6 (the new tilt-gate unit tests live alongside it, fast, no closed loop).

## Current configuration

- Python 3.12.8 (venv), managed by `uv` (`uv.lock` checked in).
- `pyproject.toml` deps (step 6 added `pandas`, `pyarrow`, `joblib`): `numpy`,
  `scipy`, `pandas`, `pyarrow`, `joblib`; dev: `pytest`. `pythonpath=["."]`,
  `testpaths=["tests"]`.
- Git remote: `https://github.com/ShreeniG99/Echo-Balancer.git`. This session's
  work happened on branch `step6-run-batch-metrics` in the worktree
  `.worktrees/step6-run-batch-metrics` (per
  `docs/superpowers/plans/2026-09-22-run-batch-metrics-step6.md`'s own
  isolation instructions), not directly on `master`.
- `nxtway_gs/` and `license.txt` are gitignored (third-party reference material,
  not redistributed). `graphify-out/cost.json` and `graphify-out/cache/` are also
  gitignored (per-machine, regenerable). Step 6 added `experiments/results/`
  to `.gitignore` (batch-run Parquet outputs, regenerable, potentially large).

## Current objective

Per `CLAUDE.md` §14, Milestone 2 ("metrics table for all three controllers")
is now achieved. The next build-order step is **step 7**: `quantum/qubo.py`
(QUBO formulation of gate-threshold selection, grid search vs.
`GroverOptimizer`, Milestone 3) — still blocked on verifying
`qiskit`/`qiskit-optimization` imports first, per the existing
[DECISIONS.md](DECISIONS.md) entry ("not yet in `pyproject.toml`, deliberately").
Step 8 (`analysis/plots.py`, `analysis/animate.py`) is the alternative next
step if quantum work is deferred further.

## Immediate next steps

1. If continuing toward Milestone 3: verify `qiskit`/`qiskit-optimization`
   import cleanly and `GroverOptimizer` is usable (CLAUDE.md §3's explicit
   precondition) *before* pinning versions in `pyproject.toml` or writing any
   `quantum/qubo.py` code.
2. `quantum/qubo.py`: discretize `tau1`/`tau2`/`N`/`T_dwell` candidates,
   re-simulate the evaluation set per candidate (log replay is invalid —
   CLAUDE.md §13), build the QUBO with infeasibility penalties
   (`tau1 >= tau2`), solve via exhaustive search and `GroverOptimizer` (Aer
   simulator), report oracle calls/wall-clock/qubit count honestly either way
   — no speedup claim to defend.
3. Alternatively/in parallel: `analysis/plots.py`, `analysis/animate.py`
   (step 8) — 2D side + top view, background color keyed to gate mode, using
   `experiments/run_batch.py`'s Parquet outputs as the data source.

## Uncommitted / unusual repo state as of this session

- This session ran entirely inside the `.worktrees/step6-run-batch-metrics`
  git worktree on branch `step6-run-batch-metrics`, not in the main checkout —
  the main checkout's `research/` PDF situation (untracked, ungitignored,
  purpose unconfirmed — see [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md)) does not
  exist in this worktree at all and was not touched or re-investigated here.
- As of this update, `project-context/*.md` are the only modified files in
  this worktree; `graphify-out/` is refreshed and staged alongside them. All
  seven of step 6's implementation commits (Tasks 1-7: deps, params, tilt
  gate, `run.py` balance-only path, `run.py` corridor path, `metrics.py`,
  `run_batch.py`) plus this task's docs commit are the full history added on
  top of `master`'s `96f914b` in this worktree/branch.

## 2026-10-08 — Wokwi firmware (extra, off the §14 build order)
`firmware/` (PlatformIO + `diagram.json` + `wokwi.toml`): ESP32 runs KF+NIS+gates at 200 Hz against a virtual linear robot; faults via MPU6050 sliders, KY-040 knob, serial keys. Host-verified vs Python (`tests/test_firmware_core.py`, 14 tests; full suite 140 pass). **Not yet built for ESP32 or run in Wokwi** (sandbox blocks PlatformIO/Wokwi); `main.cpp` only syntax-checked against stubs, `diagram.json` pin names unverified. See `firmware/README.md`. Steps 7 (qubo) and 8 (plots/animate) remain next.
Update (same day): firmware now closes the gate->control loop (speed-servo LQR, NORMAL 0.3 rad/s / CAUTIOUS x0.4 softer gains / HALT 0, gains generated from params). Push is detected 0/3 seeds; payload shift is flagged but the nominal LQR can't stabilise it (falls 3/3). `analysis/plot_firmware_gate.py` -> `docs/firmware_gate_comparison.png`; matplotlib added to pyproject. Beginner run guide: `firmware/WOKWI_GUIDE.md`. Still unverified in real Wokwi.
Update 2: target board ESP32-S3 (pins re-mapped). Payload shift now detected 10/10 and survived 10/10 in the firmware (softer HALT gains + KF Q x100, `FallbackParams`); earlier "LQR can't stabilise it" claim was wrong and is corrected in `firmware/README.md`. 146 tests pass. Still not run in real Wokwi.

## 2026-10-08 — Build-order steps 7 and 8 complete
- `quantum/formulation.py`, `quantum/cost_table.py`, `quantum/qubo.py` (+ `tests/test_qubo.py`). Results: `docs/results/qubo_*`.
- `analysis/plots.py` and `analysis/animate.py` write the figures in `docs/figures/` and the metrics CSVs in `docs/results/`.
- `sim/run.py` changes:
  - fallback (default on);
  - `gate_params` override;
  - side-ultrasonic dropout hold (`sim/sensors.py::hold_on_dropout`).
- `sim/params.py` adds `FallbackParams` and `QuboParams`.
- New dependencies: matplotlib, qiskit 2.5.2, qiskit-optimization 0.7.0, qiskit-aer 0.17.2.
- Full write-up: `docs/RESULTS.md`. Tests: 174 pass.

## 2026-10-09 — Follow-up round
- `experiments/heldout_fallback.py` and `experiments/short_window_eval.py` (held-out checks).
- Opt-in short-window detector in `sim/gate.py`.
- `ParamSet` / `param_set()` / `hardware_v1_param_set()` in `sim/params.py`; `EpisodeConfig.param_set`.
- `QuboParams` now 10 seeds and `max_simulated_qubits=24`.
- CI workflow added; graphify graph refreshed.
- Write-up: `docs/RESULTS.md` sections 2, 4 and 6. Hardware asks: `docs/HARDWARE_CHECKLIST.md`.

## 2026-10-09 — Decisions A and B
- Short-window detector ON by default (Python, cost-table candidates, firmware).
- Every result was re-run: batch, 10-seed cost table, Grover, held-out evaluations, figures.
- CLAUDE.md sections 4 and 10 updated.
- RESULTS.md rewritten for the new default.
