# Model Handoff — Echo Balancer

**Read this first.** It's the short version of everything else in
`project-context/`. Follow the links only when you need more depth on a specific
point. Last updated: 2026-09-22, commit `0d29d83` on branch
`step6-run-batch-metrics` (worktree `.worktrees/step6-run-batch-metrics`, off
`master`'s `96f914b`), plus this session's `project-context/` update.

## What this project is (one paragraph)

A simulated two-wheeled self-balancing robot that gates its own control mode
(Normal/Cautious/Halt) on the Kalman filter's normalized innovation squared
(NIS/χ²) — detecting when the filter's model of reality has drifted (disturbance,
sensor fault, payload shift) and responding safely. Threshold selection is
compared classically vs. via Grover Adaptive Search (Qiskit), with **no speedup
claim** — report whatever the comparison actually shows. Full spec: `CLAUDE.md`
at repo root. Longer version: [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md).

## Current state in one paragraph

Build-order steps 1-6 of 8 are done and tested (122/122 passing): nonlinear
plant, RK4 integration, analytic linearization, balance LQR, sensor noise
models, a 5-state Kalman filter with NIS, the Normal/Cautious/Halt gate with
all 5 CLAUDE.md §8 disturbance profiles, `sim/world.py` (2D corridor + ray
casting + pose kinematics), the ultrasonic sensor model, the full CLAUDE.md §9
wall-following stack (speed-servo LQR, yaw control, wall-following outer loop,
front-threshold slowdown) — and, as of this step, `sim/run.py::run_episode`
(the general-purpose closed-loop episode runner, unifying what were three
overlapping test-only closed-loop harnesses), the tilt-threshold baseline gate
(`sim/gate.py::step_tilt_gate`), `experiments/run_batch.py` (batch evaluation
across all three CLAUDE.md §12 controllers via `joblib`, Parquet output), and
`analysis/metrics.py` (fall rate, false-fallback, missed-fallback, detection
delay, progress). **Build-order Milestone 1 is achieved six times over** (step
4) and **Milestone 2 ("metrics table for all three controllers") is now
achieved** (step 6). **"Full wall-following" (not a reduced version) was built
per the user's explicit choice** when step 5 turned out to need two new
control loops from scratch — see "Decisions" below. **Step 7 —
`quantum/qubo.py` (Milestone 3) — has not been started**, and remains blocked
on verifying `qiskit`/`qiskit-optimization` imports first (CLAUDE.md §3's own
precondition). That, or step 8 (`analysis/plots.py`/`animate.py`), is the next
work. Full detail: [CURRENT_STATE.md](CURRENT_STATE.md).

## Architecture you need to know

`sim/params.py` holds every numeric constant (no magic numbers elsewhere).
`sim/plant.py` → `sim/integrate.py` → `sim/linearize.py` → `sim/control.py` →
`sim/sensors.py` → `sim/estimator.py` → `sim/gate.py` + `sim/disturbances.py` →
`sim/world.py` → `sim/run.py` is the built chain, with `experiments/run_batch.py`
and `analysis/metrics.py` consuming `sim/run.py`'s output on top. `sim/control.py`
still holds the same five functions from step 5: `design_lqr_balance` (step 2,
unmodified), `design_lqr_speed_servo` (step 5, a genuinely separate 5-state
integral-augmented LQR — does not replace balance), `yaw_p_control`,
`wall_following_control`, `front_threshold_speed_adjust` — step 6 composes
these inside `sim/run.py` rather than modifying `control.py` itself.
`sim/gate.py` now holds two independent gates: `step_gate` (NIS-based, step 4,
unmodified) and `step_tilt_gate` (tilt-threshold baseline, step 6, no windowed
statistic — acts directly on `|kf.x_hat[1]|` each tick). `sim/run.py::run_episode`
is the single place all of this gets wired together into one closed-loop
episode; it does **not** force `wall_following=True` and `wall_following=False`
episodes onto the same control law (see "Decisions" below — that unification
was tried and breaks push-disturbance detection). Every module has a matching
file in `tests/`; a module isn't "done" without passing tests. Full layout:
[PROJECT_CONTEXT.md](PROJECT_CONTEXT.md).

## Decisions you must not silently re-litigate

- Grover comparison reports honestly either way — no speedup claim to defend.
- `EstimatorParams` defaults are empirically tuned against the §11 NIS test —
  don't retune on intuition.
- The §11 NIS test's acceptance band is `chi2(3)`'s own 95% interval, not the
  tighter i.i.d.-mean interval — not achievable for this system (correlated
  samples).
- **`GateParams.tau1`/`tau2` are empirically calibrated, not literal χ²(3N)
  quantiles — this one is *provable*, not just empirically inconvenient.** Even
  `chi2(600)`'s 0.999999 quantile is below the observed nominal-run maximum, at
  every window size tried. Root cause: a slow ~10s θ-position closed-loop pole
  from the still-missing §9 speed-servo integral term.
- The NORMAL→HALT "skip" exception fires when the ε_k window is fully populated
  and above τ2 — not on a separate sustained-duration timer.
- Push disturbance = an instantaneous ψ̇ velocity kick, not a torque impulse
  needing an inertia conversion.
- Voltage clipping (`clip_voltage`) lives in `sim/disturbances.py`, not
  `sim/plant.py` — keeps `plant.py` untouched, since V_batt is "a state of the
  disturbance model" per §8.
- Battery-droop and payload-shift disturbances need a companion condition
  (a small push; a mild tilt) to be observable at all — genuine sensitivity
  limits of the current tuning, not bugs, and NOT part of either disturbance's
  canonical §8 profile (don't feed the companion fields into §12 evaluation).
- **A real "speed servo" LQR is a genuinely separate 5-state function
  (`design_lqr_speed_servo`), not a moving-reference hack on the existing
  4-state `design_lqr_balance`.** Feeding a moving reference into the 4-state
  LQR gives 42% steady-state speed error (no integral action). Don't try that
  shortcut again — build the integral-augmented state instead.
- **Yaw control is proportional-only, not PD, permanently** — `YawControlParams`
  deliberately has no `Kd` field. The plant's yaw pole (~-95.6 rad/s) is too fast
  relative to the 200Hz control rate: any nonzero `Kd` destabilizes the loop
  (`Kd=0.02` diverges to ~1e36 within 3s). This is a genuine discrete-sampling
  problem, not an undertuned gain — don't re-add `Kd` without a faster control
  rate or a restructured (state-feedback) yaw loop.
- **The corridor is two infinite parallel walls, no corners/dead-ends** — a
  deliberate MVP simplification (`sim/world.py::cast_ray`). Consequence:
  `front_threshold_speed_adjust` is implemented and unit-tested but can't be
  meaningfully exercised end-to-end (nothing ahead to trigger it head-on) until
  a corridor with an actual dead-end exists.
- `sim/world.py::pose_velocity` is called via `rk4_step` **once per 1ms plant
  substep**, not once per 5ms control tick — because its `phi` input is a
  genuine plant state (continuously evolving), not a true ZOH-held control
  input like `v_l`/`v_r`; freezing it for only 1ms bounds the approximation
  error to the plant's own integration resolution. (For a state-independent
  frozen-input velocity field this is algebraically identical to Euler — the
  point of using `rk4_step` here is convention-consistency with the rest of the
  codebase, not extra numerical accuracy.)
- **`sim/run.py` does NOT unify onto one shared control law.**
  `wall_following=False` uses `design_lqr_balance` (KF-fed, exactly the
  already-calibrated step 3/4 algorithm); `wall_following=True` uses
  `design_lqr_speed_servo` (true-state-fed, matching step 5). Switching
  balance-only episodes to `design_lqr_speed_servo` (any KF-fed/true-state-fed,
  zero/nonzero reference variant) was tried and measured — it erodes the
  push-disturbance detection margin below `tau1` in every case (1111-1187 vs.
  `tau1=1300`, against `design_lqr_balance`'s own already-thin 1344.87). Don't
  retry this "cleanup" without re-verifying push/battery-droop detection.
- **Corridor episodes default to `corridor_theta_dot_ref_nominal=0.3` rad/s,
  not `tests/test_wall_following.py`'s `2.0`.** `2.0` rad/s produces
  `max_epsilon=1605.56` over a nominal 60s run once the KF/gate is actually
  watching (exceeds `tau1=1300`) — a false-fallback from sustained motion
  alone, not any disturbance. `0.3` was chosen from a sweep as the best-margined
  safe value (`max_epsilon=909.13`); the speed/safety relationship is **not**
  monotonic (`0.1` rad/s destabilizes the closed loop entirely). **This "safe"
  designation was only ever validated against gate-triggering (epsilon vs
  `tau1`) at `seed=42` — a later real 10-seed batch run found a genuine ~20%
  nominal fall rate at this speed, independent of gate choice. Do not read
  "safe" as "never falls"; see "Current problems" below and
  [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md).**
- **CAUTIOUS mode's "softer Q" speed-servo gain divides all five
  `SpeedServoParams` Q weights by 5** (not just `Q_psi`, which barely moves `K`
  since it already dominates by 3 orders of magnitude). HALT uses the
  *nominal* (not cautious) gain — CLAUDE.md §10 only says "softer Q" for
  CAUTIOUS, a literal reading consistent with prior steps' approach to
  ambiguous spec clauses.
- **`experiments/run_batch.py`'s workers take the resolved scenario tuple as
  an explicit parameter, not a module-level lookup.** `joblib`'s `loky`
  backend uses `spawn`-only worker processes on Windows (no fork/forkserver),
  so a worker-side lookup of `DISTURBANCE_SCENARIOS` would silently read
  on-disk module state instead of a parent-process-patched value — this is a
  general correctness pattern, not a Windows-only workaround, and must not be
  "simplified" back even when porting to a fork-capable POSIX system (the bug
  would just go latent again, not disappear).
- Full list with reasons: [DECISIONS.md](DECISIONS.md).

## Failed approaches — do not repeat

- Large initial KF covariance `P0` (e.g. `1e-2`) → causes a 200-600 NIS startup
  transient. Use small `P0` (robot starts upright at rest, that's known).
- Raising `Q` to raise mean NIS toward 3 → does the *opposite* in the nominal
  (no-mismatch) case.
- Trying "smarter" χ²(3N) quantiles or larger `N` for the gate thresholds → the
  slow-pole issue makes this unfixable by quantile-picking; the threshold has to
  be empirical, or the slow pole has to be fixed first (§9 speed-servo term).
- Firing the battery-droop test's companion push at the droop's own onset →
  `battery_droop_v_batt` *ramps*, doesn't step, so V_batt is still ~nominal at
  that instant. Fire the push at `onset + duration` instead.
- Using `design_lqr_speed_servo` as a single shared control law for every
  `sim/run.py` episode (instead of only for `wall_following=True`) → breaks
  push-disturbance detection in every variant tried, see "Decisions" above.
- Reusing `tests/test_wall_following.py`'s `theta_dot_ref_nominal=2.0` as the
  corridor evaluation speed → false-fallback from motion alone once the gate
  is actually watching (that test never had a gate in its loop at all).
- Assuming HALT mode makes `theta_dot` settle near zero within one `T_dwell`
  window → it overshoots well past the NORMAL cruise speed instead, because
  `T_dwell` is much shorter than the pitch-recovery pole's time constant; a
  bug-injection check confirmed a naive "near zero" bound would be
  non-discriminating (two seeded bugs both produced *smaller* overshoot than
  correct code).
- Full list: [FAILED_APPROACHES.md](FAILED_APPROACHES.md).

## Current problems

- **Surface-change disturbance sits close to a real plant instability**
  (`f_w≥~0.03` causes NaN/inf) — a genuine controller stability limit worth
  reporting, not a bug to fix in this step.
- **NIS detection is measurably less sensitive to payload/CoM shifts and to
  isolated battery droop** than to the other disturbance types, under the
  current tuning — a real, reportable limitation.
- Two untracked PDFs sit under `research/` with unclear purpose — don't assume,
  ask the user.
- Hardware params (§6.2) are entirely placeholder; `qiskit`/`qiskit-optimization`
  not yet installed/verified.
- **`front_threshold_speed_adjust` has no end-to-end exercise** — the corridor
  has no dead-end/obstacle ahead to trigger it via the real closed loop; only
  unit-tested in isolation. Not a bug, just untested-in-context.
- **Sustained forward motion with active yaw correction significantly
  inflates the windowed NIS statistic** — confirms a suspicion this file
  previously flagged as untested (encoder per-wheel quantization under yaw).
  `2.0` rad/s produces a false-fallback purely from motion; `0.3` rad/s is
  safe. The speed/safety relationship is not monotonic (`0.1` rad/s
  destabilizes the closed loop entirely). Root cause not fully characterized;
  fixing it (a `theta_dot`-dependent process-noise term, or an
  average-then-quantize encoder redesign) was explicitly out of scope for step 6.
- **Push-type disturbances are not reliably detected by the NIS gate once
  corridor/wall-following mode is active** — an inherited, pre-existing thin
  margin (design_lqr_balance's own push margin was only 3.5% over `tau1`), not
  a step-6 regression, but expect `run_batch.py`'s metrics table to show a
  real "missed"/delayed detection for push specifically under the NIS-gate
  controller in corridor mode.
- **HALT mode's transient overshoots the NORMAL cruise speed before
  settling** — `TiltGateParams.T_dwell` is much shorter than the
  pitch-recovery pole's time constant, so HALT can engage mid-recovery-transient.
- **A real 10-seed nominal corridor batch run shows a ~20% fall rate independent
  of gate choice** (`naive`/`nis_gate` both 0.2, same falling seeds 3/5,
  near-identical fall times; `tilt_threshold` 0.5, its extra falls correlated
  with its own mode-switching) at the "safe" `corridor_theta_dot_ref_nominal=0.3`
  speed — found only via a real end-to-end batch run, not any of step 6's own
  (narrower) tests. Fall-rate safety at this speed is an open question, not a
  solved one.
- Full list: [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md).

## Exact config/parameters in force right now

```
PlantParams:       CLAUDE.md §6.1 NXTway-GS values (g=9.81, m=0.03, R=0.04, M=0.6, ...)
LQRBalanceParams:  Q=diag(1, 1e3, 1, 1), R=1e2
SensorParams:      sigma_gyro=0.005, gyro_bias_walk=1e-4, sigma_accel=0.02,
                   encoder_cpr=360, ultrasonic: range 0.02-4m, sigma=3mm,
                   dropout=2% -- now consumed by sim.sensors.ultrasonic (step 5)
CorridorParams:    width=1.0m, ray_parallel_eps=1e-9 (cast_ray's near-parallel
                   tolerance -- moved here from a hardcoded value per CLAUDE.md §3)
SpeedServoParams:  Q=diag(1, 1e3, 1, 1, 10) [theta,psi,theta_dot,psi_dot,integral],
                   R=1e2 -- gain K=[-0.906,-53.30,-2.312,-5.011,-0.316] (verified)
YawControlParams:  Kp=3.0 (proportional-only, no Kd -- see Decisions)
WallFollowParams:  target_distance=0.5m, Kp=2.0, Kd=0.5, front_slow_threshold=0.5m,
                   front_slow_factor=0.3
EstimatorParams:   Q_theta=Q_psi=1e-8, Q_theta_dot=Q_psi_dot=1e-6,
                   P0_theta=P0_psi=P0_theta_dot=P0_psi_dot=1e-4, P0_bg=1e-6
GateParams:        N=200 (1s window), tau1=1300, tau2=1800, tau1_exit=1040,
                   tau2_exit=1300, T_dwell=0.5s -- empirical, NOT chi2(3N) quantiles
TiltGateParams:    psi1=0.0087 rad (0.5deg), psi2=0.0175 rad (1.0deg),
                   psi1_exit=0.006, psi2_exit=0.012, T_dwell=0.5s -- baseline
                   gate, keyed on kf.x_hat[1] (KF-estimated psi), no window
RunParams:         dt_plant=0.001, dt_control=0.005, fall_psi_threshold=45deg,
                   cautious_speed_scale=0.4, corridor_theta_dot_ref_nominal=0.3
                   rad/s -- deliberately NOT tests/test_wall_following.py's 2.0
                   (verified unsafe once the gate is watching: max epsilon 1605.6)
SpeedServoParams (cautious): all five default_speed_servo_params() Q weights
                   /5, R unchanged -- gain K=[-0.583,-51.54,-2.222,-4.814,-0.141],
                   dominant pole -0.263 vs nominal -0.394 (~33% gentler)
DisturbanceParams: surface_change_fw=0.025 (near-instability margin), push=0.1 rad/s,
                   gyro_bias_fault=0.05 rad/s, accel_noise_fault=5x,
                   payload_shift dM=1.0kg/dL=0.1m (+2 deg tilt needed),
                   battery_droop v_drooped=0.1V (+0.05 rad/s companion push, fired
                   AFTER the 1s droop ramp completes), detection_window_s=2.0
Loop rates:        dt_plant=1ms (RK4), dt_control=5ms (200Hz, ZOH) -- now wired
                   into the general sim/run.py::run_episode (step 6), which
                   unified the three overlapping test-only closed-loop harnesses
                   that used to duplicate this plumbing
```

## Important files

`CLAUDE.md` (spec, source of truth) · `sim/params.py` (all constants) ·
`docs/superpowers/plans/*.md` (per-step design rationale, read before touching
tuned values) · `tests/` (definition of done) · `graphify-out/graph.json`
(queryable code graph — see below).

## What the previous model (this session) was doing

Implemented build-order step 6 (`sim/run.py`, the tilt-threshold baseline gate
in `sim/gate.py`, `experiments/run_batch.py`, `analysis/metrics.py` —
Milestone 2) via `superpowers:subagent-driven-development`/task-based
execution of `docs/superpowers/plans/2026-09-22-run-batch-metrics-step6.md`,
in an isolated worktree (`.worktrees/step6-run-batch-metrics`, branch
`step6-run-batch-metrics`) per the plan's own instructions. This step's
central design question — explicitly resolved, not deferred — was
[OPEN_PROBLEMS.md](OPEN_PROBLEMS.md)'s long-standing "should the three
overlapping test-only closed-loop harnesses be unified": yes, into
`sim/run.py::run_episode`, but *not* onto one shared control law (measured
that unifying onto `design_lqr_speed_servo` everywhere breaks push-disturbance
detection — see "Decisions" above). The plan itself was unusually
empirically-grounded before being written: seven numbered "pre-verified
design" findings (control-law choice, corridor speed safety, the
encoder-under-yaw interaction, tilt-gate calibration, cautious-gain choice,
mode-scaling scope, and the fall-condition necessity) were each measured
against the real repo before any task code was drafted, the same practice
steps 3-5 used. Seven tasks (deps; params; tilt gate; `run.py` balance-only
path; `run.py` corridor path; `metrics.py`; `run_batch.py`), each committed
independently. One real bug was caught and fixed during Task 7 (not by a
plan-following mistake, but by the plan's own draft code being wrong in a way
only Windows exposes): `joblib`'s `loky` backend uses `spawn`-only worker
processes here, so a worker-side lookup of the module-level
`DISTURBANCE_SCENARIOS` dict silently read on-disk state instead of a
test's `monkeypatch`-patched value — caught by a test producing 12000 rows
instead of 200. Fixed by threading the resolved scenario tuple through
`delayed()` explicitly instead. All 122 tests pass (84 before this step, 38
new pytest-collected cases, verified via `git diff 96f914b..ed551a0` test-function
counts plus `pytest --collect-only`). Breakdown, in collected-case counts
(noted where this differs from test-function counts, due to parametrization):
`test_run.py` — 14 test functions / 17 collected cases (two are parametrized:
`test_nis_gate_detects_each_disturbance_within_window` expands to 3 cases,
`test_tilt_threshold_controller_detection` expands to 2); `test_metrics.py` —
8 functions / 8 cases; `test_run_batch.py` — 3 functions / 3 cases (28
collected cases across these three new files, all unparametrized except the
two noted above); plus 10 new test functions / 10 collected cases (no
parametrization added) from `test_params.py` (6 new tests) and `test_gate.py`
(4 new tests) additions for `RunParams`/`TiltGateParams`/the cautious
gain/`step_tilt_gate`. 28 + 10 = 38, matching the total.

**Graphify status:** refreshed via `graphify update .` at the end of this step
(the codebase changed substantially — `sim/run.py`, `analysis/`,
`experiments/`, and the `sim/gate.py`/`sim/params.py` additions are now in the
graph). Still no deep semantic doc-prose extraction (needs an LLM backend,
none configured); git hooks still not installed (user hasn't opted in).

## What the next model should do

- If continuing simulation work: start build-order **step 7**
  (`quantum/qubo.py`, CLAUDE.md §14's "Milestone 3: grid vs Grover") per
  [CURRENT_STATE.md](CURRENT_STATE.md)'s "Immediate next steps." The
  precondition CLAUDE.md §3 states — "verify `GroverOptimizer` imports before
  writing quantum code" — is still unmet; do this *before* pinning
  `qiskit`/`qiskit-optimization` versions in `pyproject.toml` (see the
  existing [DECISIONS.md](DECISIONS.md) entry on why this was deliberately
  deferred). Alternatively, **step 8** (`analysis/plots.py`,
  `analysis/animate.py` — 2D side + top view, background color keyed to gate
  mode) has no such blocker and could go first if quantum setup stalls.
- Either step will want to actually run `experiments/run_batch.py::main()`
  once (not just its smoke tests) to produce real Parquet output under
  `experiments/results/` (gitignored) — step 6 only verified the wiring/schema
  via short monkeypatched smoke tests, not a full 10-seed x 7-scenario batch,
  since `sim/run.py`'s own calibration tests already cover the numeric
  correctness this would re-exercise more slowly.
- Use `superpowers:writing-plans` the way the six existing
  `docs/superpowers/plans/` documents were written, for consistency — and
  budget real time for empirical verification before writing the plan, the
  way steps 3-6 did.
- Run `graphify query "<question>"` before grepping the whole repo for a
  code-structure question (a `PreToolUse` hook enforces this), and update this
  file plus the relevant sibling file (`CURRENT_STATE.md` for state changes,
  `DECISIONS.md` for new decisions, `FAILED_APPROACHES.md` if something is tried
  and doesn't work) when you're done — not for every small step, just at a
  natural stopping point.

## Constraints the next model must respect

No magic numbers outside `params.py`. Every stochastic function takes an
explicit `rng`. Pure functions, explicit state threading, no global state. Never
loosen a failing physics test's tolerance — find the bug (step 4 found two real
bugs this way: the sensor-param-threading gap in the gate harness, and the
battery-droop push timing; step 6 found one more: the `joblib`/spawn
worker-scenario-lookup bug — see "What the previous model was doing" above).
Don't build anything on the out-of-scope list (`CLAUDE.md` §2: no
CAD/ROS/GUIs/neural nets/3D physics engines). Full spec: `CLAUDE.md`.
