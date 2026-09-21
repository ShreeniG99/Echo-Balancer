# Model Handoff — Echo Balancer

**Read this first.** It's the short version of everything else in
`project-context/`. Follow the links only when you need more depth on a specific
point. Last updated: 2026-09-21, commit `12be336` on `master`.

## What this project is (one paragraph)

A simulated two-wheeled self-balancing robot that gates its own control mode
(Normal/Cautious/Halt) on the Kalman filter's normalized innovation squared
(NIS/χ²) — detecting when the filter's model of reality has drifted (disturbance,
sensor fault, payload shift) and responding safely. Threshold selection is
compared classically vs. via Grover Adaptive Search (Qiskit), with **no speedup
claim** — report whatever the comparison actually shows. Full spec: `CLAUDE.md`
at repo root. Longer version: [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md).

## Current state in one paragraph

Build-order steps 1-4 of 8 are done and tested (58/58 passing): nonlinear plant,
RK4 integration, analytic linearization, balance LQR, sensor noise models, a
5-state Kalman filter with NIS, and — as of this step — the actual gate
(Normal/Cautious/Halt state machine on the windowed NIS statistic) plus all 5
CLAUDE.md §8 disturbance profiles. **Build-order Milestone 1 ("gate switches
under one disturbance") is achieved, six times over** (one test per disturbance
category, two variants for "sensor fault"). **Step 5 — `sim/world.py` (2D
corridor + ray casting) and wall-following — has not been started.** That is the
next work. Full detail: [CURRENT_STATE.md](CURRENT_STATE.md).

## Architecture you need to know

`sim/params.py` holds every numeric constant (no magic numbers elsewhere).
`sim/plant.py` → `sim/integrate.py` → `sim/linearize.py` → `sim/control.py` →
`sim/sensors.py` → `sim/estimator.py` → `sim/gate.py` + `sim/disturbances.py` is
the built chain; `sim/world.py` is the next link. Every module has a matching
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
  not yet installed/verified. Full list: [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md).

## Exact config/parameters in force right now

```
PlantParams:       CLAUDE.md §6.1 NXTway-GS values (g=9.81, m=0.03, R=0.04, M=0.6, ...)
LQRBalanceParams:  Q=diag(1, 1e3, 1, 1), R=1e2
SensorParams:      sigma_gyro=0.005, gyro_bias_walk=1e-4, sigma_accel=0.02,
                   encoder_cpr=360, ultrasonic_* set but unused (no world.py yet)
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

Implemented build-order step 4 (`sim/gate.py`, `sim/disturbances.py`) via
`superpowers:subagent-driven-development`, following the same plan-first,
two-stage-review pattern as steps 1-3. This step required by far the most
empirical exploration of the four so far: discovered that literal χ²(3N)
quantiles are provably unachievable for the gate thresholds (not just
empirically inconvenient, like the earlier NIS-test finding), discovered a
genuine plant instability boundary near `f_w≈0.03`, and discovered that two of
five disturbances (payload shift, battery droop) don't trigger detection in
isolation near equilibrium. One implementer subagent run for the final task
took an anomalously long time (~3 hours) and returned with work incomplete; the
controller verified the partial state directly, found a real bug in the
battery-droop test's timing (companion push fired before the voltage ramp had
actually drooped — an artifact of an exploratory script using a step function
where the real implementation uses a ramp), fixed it directly, and confirmed via
a full test run before committing. All 58 tests pass. Also, per the user's
request, committed everything that had accumulated uncommitted from the prior
session's Graphify/`project-context/` setup (see below), and updated
`project-context/` itself to reflect this step's findings.

**Graphify status (inherited from the prior session, unchanged this session):**
code-only graph (328+ nodes after this step's updates — re-run `graphify update .`
if this looks stale), no deep semantic doc-prose extraction (needs an LLM
backend, none configured), git hooks not installed (user hasn't opted in).

## What the next model should do

- If continuing simulation work: start build-order **step 5** (`sim/world.py`,
  then wall-following in `sim/control.py`) per
  [CURRENT_STATE.md](CURRENT_STATE.md)'s "Immediate next steps." Use
  `superpowers:writing-plans` the way the four existing `docs/superpowers/plans/`
  documents were written, for consistency — and budget real time for empirical
  verification before writing the plan, the way steps 3-4 did, rather than
  guessing at ray-casting/wall-following parameters.
- Consider whether `sim/world.py`/wall-following actually needs the §9
  speed-servo integral term and yaw PD that steps 1-4 deferred — if so, that's
  probably worth building first, and it would also fix the gate's chi2(3N)
  slow-pole issue as a side effect.
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
