# Model Handoff — Echo Balancer

**Read this first.** It's the short version of everything else in
`project-context/`. Follow the links only when you need more depth on a specific
point. Last updated: 2026-09-21, commit `96f914b` on `master`.

## What this project is (one paragraph)

A simulated two-wheeled self-balancing robot that gates its own control mode
(Normal/Cautious/Halt) on the Kalman filter's normalized innovation squared
(NIS/χ²) — detecting when the filter's model of reality has drifted (disturbance,
sensor fault, payload shift) and responding safely. Threshold selection is
compared classically vs. via Grover Adaptive Search (Qiskit), with **no speedup
claim** — report whatever the comparison actually shows. Full spec: `CLAUDE.md`
at repo root. Longer version: [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md).

## Current state in one paragraph

Build-order steps 1-5 of 8 are done and tested (84/84 passing): nonlinear plant,
RK4 integration, analytic linearization, balance LQR, sensor noise models, a
5-state Kalman filter with NIS, the Normal/Cautious/Halt gate with all 5
CLAUDE.md §8 disturbance profiles, and — as of this step — `sim/world.py` (2D
corridor + ray casting + pose kinematics), the ultrasonic sensor model, and the
full CLAUDE.md §9 wall-following stack (speed-servo LQR, yaw control,
wall-following outer loop, front-threshold slowdown), demonstrated together in
one closed-loop test with real sensor noise/dropout. **Build-order Milestone 1
is achieved six times over** (step 4). **"Full wall-following" (not a reduced
version) was built per the user's explicit choice** when step 5 turned out to
need two new control loops from scratch — see "Decisions" below. **Step 6 —
`sim/world.py`'s corridor extended for `run.py`/`run_batch.py`/`metrics.py`
(Milestone 2) — has not been started.** That is the next work. Full detail:
[CURRENT_STATE.md](CURRENT_STATE.md).

## Architecture you need to know

`sim/params.py` holds every numeric constant (no magic numbers elsewhere).
`sim/plant.py` → `sim/integrate.py` → `sim/linearize.py` → `sim/control.py` →
`sim/sensors.py` → `sim/estimator.py` → `sim/gate.py` + `sim/disturbances.py` →
`sim/world.py` is the built chain. `sim/control.py` now holds five functions:
`design_lqr_balance` (step 2, unmodified), `design_lqr_speed_servo` (step 5, a
genuinely separate 5-state integral-augmented LQR — does not replace balance),
`yaw_p_control`, `wall_following_control`, `front_threshold_speed_adjust` (step
5). Every module has a matching file in `tests/`; a module isn't "done" without
passing tests. Full layout: [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md).

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
DisturbanceParams: surface_change_fw=0.025 (near-instability margin), push=0.1 rad/s,
                   gyro_bias_fault=0.05 rad/s, accel_noise_fault=5x,
                   payload_shift dM=1.0kg/dL=0.1m (+2 deg tilt needed),
                   battery_droop v_drooped=0.1V (+0.05 rad/s companion push, fired
                   AFTER the 1s droop ramp completes), detection_window_s=2.0
Loop rates:        dt_plant=1ms (RK4), dt_control=5ms (200Hz, ZOH -- not yet wired
                   into a general run.py; only exists inside test-only harnesses
                   in tests/test_estimator.py and tests/test_gate.py, which
                   duplicate a fair amount of closed-loop plumbing between them)
```

## Important files

`CLAUDE.md` (spec, source of truth) · `sim/params.py` (all constants) ·
`docs/superpowers/plans/*.md` (per-step design rationale, read before touching
tuned values) · `tests/` (definition of done) · `graphify-out/graph.json`
(queryable code graph — see below).

## What the previous model (this session) was doing

Implemented build-order step 5 (`sim/world.py`, ultrasonic sensor, §9
wall-following stack in `sim/control.py`) via
`superpowers:subagent-driven-development`, following the same plan-first,
two-stage-review pattern as steps 1-4. This step needed substantially more
design work than any prior step: §9's wall-following description assumes
forward-speed and yaw-rate control already exist, but neither did (steps 1-4
were balance-only). Mid-step, after discovering this, the controller used
`AskUserQuestion` to check whether to follow CLAUDE.md's own "cut
wall-following first if behind schedule" guidance — **the user explicitly chose
"Full wall-following"**, i.e. build the complete stack as originally scoped,
not a reduced version. That decision was honored throughout: a genuinely
separate 5-state integral-augmented speed-servo LQR was built (42% steady-state
error without it), and yaw control was built proportional-only after
discovering PD is genuinely unstable at this plant's yaw pole speed vs. the
200Hz control rate (not a tuning problem — verified numerically).

Six tasks, each independently spec-reviewed and code-quality-reviewed via fresh
subagents; three of six needed one fix-loop iteration (a magic-number rule
violation, a mathematically-incorrect docstring claim about ramping-reference
dynamics caught by an unusually rigorous reviewer, and two rounds of stale
"not yet implemented" docstrings that kept slipping through single-function
reviews because they were module-level). A final holistic review after all six
tasks caught one more stale module docstring and an untracked plan doc, both
fixed in a closing commit. One notable orchestrator-level design decision made
mid-review (not by an implementer): after a code reviewer flagged that
`pose_velocity` (Task 2) was designed to be used by Task 6 but Task 6's original
pseudocode used a hand-rolled formula instead — leaving `pose_velocity`
untested by its only consumer — the controller edited the plan doc directly to
change Task 6 to integrate position via `pose_velocity`/`rk4_step` at 1ms plant
resolution instead of 5ms control resolution. This was re-verified when Task 6
was actually implemented: results matched the old formula's pre-verified
numbers to within ~0.0002, confirming the change was safe. All 84 tests pass.

**Graphify status (inherited from the prior session, not touched this
session — the codebase changed a lot; run `graphify update .` before trusting
the graph):** code-only graph (328+ nodes as of the last known-good build), no
deep semantic doc-prose extraction (needs an LLM backend, none configured), git
hooks not installed (user hasn't opted in).

## What the next model should do

- If continuing simulation work: start build-order **step 6**
  (`sim/run.py` → `experiments/run_batch.py` → `analysis/metrics.py`,
  CLAUDE.md §14's "Milestone 2: metrics table for all three controllers") per
  [CURRENT_STATE.md](CURRENT_STATE.md)'s "Immediate next steps." Use
  `superpowers:writing-plans` the way the five existing `docs/superpowers/plans/`
  documents were written, for consistency — and budget real time for empirical
  verification before writing the plan, the way steps 3-5 did.
- `sim/run.py` will likely want to unify the closed-loop-harness duplication
  that's accumulated across `tests/test_estimator.py`,`tests/test_gate.py`, and
  now `tests/test_wall_following.py` (three independent test-only closed loops
  with overlapping plumbing) — see [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md).
- Run `graphify query "<question>"` before grepping the whole repo for a
  code-structure question (a `PreToolUse` hook enforces this), and update this
  file plus the relevant sibling file (`CURRENT_STATE.md` for state changes,
  `DECISIONS.md` for new decisions, `FAILED_APPROACHES.md` if something is tried
  and doesn't work) when you're done — not for every small step, just at a
  natural stopping point.

## Constraints the next model must respect

No magic numbers outside `params.py`. Every stochastic function takes an
explicit `rng`. Pure functions, explicit state threading, no global state. Never
loosen a failing physics test's tolerance — find the bug (this step found two
real bugs this way: the sensor-param-threading gap in the gate harness, and the
battery-droop push timing). Don't build anything on the out-of-scope list
(`CLAUDE.md` §2: no CAD/ROS/GUIs/neural nets/3D physics engines). Full spec:
`CLAUDE.md`.
