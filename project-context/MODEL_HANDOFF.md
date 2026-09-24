# Model Handoff — Echo Balancer

**Read this first.** It's the short version of everything else in
`project-context/`. Follow the links only when you need more depth on a specific
point. Last updated: 2026-09-24, commit `197e88f` on
`claude/amazing-thompson-o1hp87`.

## What this project is (one paragraph)

A simulated two-wheeled self-balancing robot that gates its own control mode
(Normal/Cautious/Halt) on the Kalman filter's normalized innovation squared
(NIS/χ²) — detecting when the filter's model of reality has drifted (disturbance,
sensor fault, payload shift) and responding safely. Threshold selection is
compared classically vs. via Grover Adaptive Search (Qiskit), with **no speedup
claim** — report whatever the comparison actually shows. Full spec: `CLAUDE.md`
at repo root. Longer version: [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md).

## Current state in one paragraph

**All 8 build-order steps (`CLAUDE.md` §14) are complete.** This session did
steps 6-8: `sim/run.py` (one real closed-loop episode runner, unifying three
previously-duplicated test harnesses) + `experiments/run_batch.py` +
`analysis/metrics.py` (**Milestone 2**: real cross-controller comparison — the
NIS gate has zero missed detections across 60 disturbance episodes vs. the
tilt-threshold baseline's 50% miss rate, cutting fall rate 42.9%→28.6% at a
modest progress cost); `quantum/qubo.py` (**Milestone 3**: 64-candidate grid,
classical exhaustive search vs. `GroverOptimizer` on a fitted quadratic
surrogate — no speedup claim confirmed trivially, and a genuinely interesting
finding that the surrogate's true optimum margin is only ~0.12% of the
objective's range, a real value-qubit-resolution bottleneck); and, per an
explicit **user-approved mid-session scope change**, a 3D video demo
(`analysis/animate.py`, PyVista/VTK, off-screen) replacing the originally-planned
2D side/top-view animation, followed by `analysis/plots.py` (episode timeline,
cross-controller comparison bars, tau1/tau2 sensitivity heatmap) on explicit
request. **Every file `CLAUDE.md` §4 names now exists.** 121/121 tests pass.
Full detail: [CURRENT_STATE.md](CURRENT_STATE.md).

## Architecture you need to know

`sim/params.py` holds every numeric constant (no magic numbers elsewhere).
`sim/plant.py` → `sim/integrate.py` → `sim/linearize.py` → `sim/control.py` →
`sim/sensors.py` → `sim/estimator.py` → `sim/gate.py` + `sim/disturbances.py` →
`sim/world.py` → **`sim/run.py`** (the real closed-loop runner, new this
session) → **`experiments/run_batch.py`** → **`analysis/metrics.py`** →
**`quantum/qubo.py`** → **`analysis/animate.py`** is the full built chain.
`sim/run.py::run_episode(config, seed) -> DataFrame` is parameterized by
`ControllerKind` (NAIVE/TILT_GATE/NIS_GATE, CLAUDE.md §12's three controllers)
and is now what actually drives control from the KF estimate (not true state) —
this was step 5's known "pending run.py" simplification, now resolved.

## Decisions you must not silently re-litigate

- Grover comparison reports honestly either way — no speedup claim to defend.
- **`sim.params.default_evaluation_gate_params()` (tau1=900/tau2=1400) and
  `sim.params.default_gate_params()` (tau1=1300/tau2=1800) are BOTH correct —
  for two genuinely different closed loops** (the real driving/speed-servo
  loop `sim.run` uses, vs. the balance-only loop `tests/test_gate.py`'s own
  harness uses). Do not merge them or "fix" one to match the other.
- **A speed reference must ramp (`SpeedProfileParams.accel_limit`), never step
  instantly, inside any KF/NIS-gated closed loop** — an instant step alone was
  enough to spuriously trip the gate on a nominal run.
- **Ultrasonic dropout readings (`reading == sensor_p.ultrasonic_range_max`
  exactly) must be held, not fed into `wall_following_control`** — a dropout's
  derivative-term reaction alone produced a 35 rad/s spurious yaw command.
- `GroverOptimizer`'s objective must be a fitted quadratic surrogate of the
  true (re-simulated) cost table, never the true table directly — `qiskit`'s
  `QuadraticProgram` can't represent an arbitrary black-box function over 6
  bits. Report both the surrogate-vs-Grover match and the surrogate-vs-true
  match separately.
- **`GroverOptimizer.solve()` is probabilistic — never trust a single call.**
  Run several trials (`grover_trials`), keep the best, report the hit rate.
- 3D animation (`analysis/animate.py`, PyVista) replaces the original 2D
  side/top-view plan — **user-approved scope change**, done only after
  Milestones 2/3 per the user's own sequencing request. Doesn't reopen the
  CAD/physics-engine/ROS/GUI bans (no physics computed in `animate.py`, always
  off-screen).
- `EstimatorParams` defaults are empirically tuned against the §11 NIS test —
  don't retune on intuition.
- Push disturbance = an instantaneous ψ̇ velocity kick, not a torque impulse
  needing an inertia conversion.
- Voltage clipping (`clip_voltage`) lives in `sim/disturbances.py`.
- Yaw control is proportional-only, permanently (`YawControlParams` has no
  `Kd` field) — the plant's yaw pole is too fast for any nonzero `Kd` at 200Hz.
- The corridor is two infinite parallel walls, no corners/dead-ends — still
  true after step 6; `front_threshold_speed_adjust` still has nothing to
  meaningfully trigger it head-on.
- Full list with reasons: [DECISIONS.md](DECISIONS.md).

## Failed approaches — do not repeat

- Instantaneous 0→full-speed step in `sim/run.py` — spuriously trips the NIS
  gate. Use the bounded-acceleration ramp instead.
- `prev_wall_err` seeded at 0.0 (or from the wrong wall's distance) — a
  classic PID cold-start derivative kick, producing a multi-rad/s spurious yaw
  command on the very first ultrasonic sample.
- Feeding ultrasonic dropout readings straight into wall-following's PD
  controller — same derivative-kick failure mode, triggered mid-episode.
- Treating a single `GroverOptimizer.solve()` call as authoritative — it has a
  real, large failure probability at this problem's scale (observed 0/5, 0/30
  hit rates in production runs).
- Large initial KF covariance `P0`; raising `Q` to raise mean NIS toward 3 in
  the nominal case (does the opposite); "smarter" χ²(3N) quantiles for the
  gate thresholds (provably unachievable, not fixable by quantile-picking).
- Full list: [FAILED_APPROACHES.md](FAILED_APPROACHES.md).

## Current problems

- **`GroverOptimizer` cannot reliably resolve this project's real QUBO
  surrogate's optimum at a classically-simulable qubit count** — the true
  optimum's margin over its closest rival is only ~0.12% of the objective's
  range. A genuine, reportable finding about value-qubit-based GAS on
  continuous-valued cost landscapes, not a bug.
- **`battery_droop` and `payload_shift` are unsurvivable (100% fall rate)
  regardless of controller**, even though the NIS gate detects both quickly —
  detection isn't the same as survivability here. Not yet root-caused further.
- Surface-change disturbance sits close to a real plant instability
  (`f_w≥~0.03` causes NaN/inf) — unchanged from before this session.
- Hardware params (§6.2) are entirely placeholder.
- Two untracked PDFs under `research/` with unclear purpose — don't assume,
  ask the user. `analysis/results/` (demo video + plot PNGs) is gitignored,
  following the same precedent as `/experiments/results/` -- ask the user
  first if rendered output ever needs to live in git instead.
- Full list: [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md).

## Exact config/parameters in force right now

```
PlantParams:       CLAUDE.md §6.1 NXTway-GS values (unchanged)
GateParams (default_gate_params):            tau1=1300, tau2=1800 -- balance-only closed loop (tests/test_gate.py)
GateParams (default_evaluation_gate_params): tau1=900,  tau2=1400 -- sim.run's real closed loop (speed-servo + wall-following)
                                              tau1_exit/tau2_exit keep the same hysteresis ratios (0.8, ~0.714) in both
SpeedProfileParams: accel_limit=2.0 rad/s^2 (new -- fixes the instant-speed-step bug)
CautiousParams:     speed_ref_factor=0.4 (CLAUDE.md section 10)
TiltGateParams:     tau1_deg=5, tau2_deg=15, tau1_exit_deg=3, tau2_exit_deg=10, T_dwell=0.5 (new -- tilt-only baseline)
QuboSearchParams:   tau1_grid=(700,900,1100,1300), tau2_grid=(1000,1200,1400,1600), N_grid=(150,200),
                    T_dwell_grid=(0.3,0.5) -- 64 candidates, 6 bits
                    w_fall=10, w_false_fallback=2, w_progress=1, infeasible_penalty=20
                    seeds=(0,1,2), nominal_T=20s, disturbance_T=15s (shorter than Milestone 2's batch, for tractability)
experiments.run_batch defaults: 7 scenarios x 3 controllers x 10 seeds, nominal_T=60s, disturbance_T=30s
```

## Important files

`CLAUDE.md` (spec, source of truth, updated this session for the 3D scope
change) · `sim/params.py` (all constants) · `tests/` (definition of done) ·
`experiments/results/` (gitignored batch output — regenerate via
`uv run python -m experiments.run_batch`) · `analysis/results/` (demo video,
untracked) · `graphify-out/graph.json` (queryable code graph — run
`graphify update .` before trusting it, not touched this session).

## What the previous model (this session) was doing

Started from a request to check project state and produce a 3D video demo.
User first asked to defer the 3D work until after build-order steps 6
(`run.py`/Milestone 2) and 7 (`qubo.py`/Milestone 3) were done — both were then
implemented in full this session, followed by the 3D video per the user's
original request. Two genuine implementation bugs were found and fixed during
`sim/run.py`'s integration (an instant speed-reference step, and unfiltered
ultrasonic dropout readings — both spuriously tripping the NIS gate on nominal
runs; see `FAILED_APPROACHES.md`), requiring a `GateParams` recalibration
specific to the new closed loop (kept as a separate function, not an
overwrite, to avoid breaking `tests/test_gate.py`'s own correct calibration).
`quantum/qubo.py` required building a quadratic surrogate for `GroverOptimizer`
(the true cost table isn't naturally quadratic) and running Grover multiple
trials (a single run is unreliable) — the production run's most interesting
finding was a razor-thin (~0.12%) margin between the true optimum and its
closest rival, a genuine value-qubit precision limit, reported honestly rather
than tuned away. `CLAUDE.md` was updated to reflect the user-approved 3D scope
change before `analysis/animate.py` was built. One test flake
(`test_animate.py` failing under concurrent heavy CPU load from an unrelated
background job) was diagnosed as resource contention, not a real bug, and
confirmed by a clean re-run in isolation.

All work is committed and pushed to `claude/amazing-thompson-o1hp87` through
commit `197e88f` (note: that commit's message only describes the qubo.py
change it also happens to include — `CLAUDE.md`/`animate.py`/`test_animate.py`
were unintentionally bundled into it via a broad `git add -A`; the diff itself
is correct and complete, just under-described by the message).

`analysis/plots.py` was built right after (episode timeline, cross-controller
comparison bars, tau1/tau2 sensitivity heatmap), on explicit follow-up
request. One real bug found there too: passing string labels directly to
`ax.bar(labels, values, ...)` silently dropped a category from the *saved*
figure once a bar's height was `NaN` (visible only in the rendered PNG, not
right after `ax.bar()` returns) — fixed with explicit numeric x-positions.

**Graphify status:** not touched this session — the codebase changed
substantially (6 new modules, ~39 new tests); run `graphify update .` before
trusting the graph.

## What the next model should do

- Every file `CLAUDE.md` §4 names now exists. Ask the user what's next: a
  paper/write-up using the real numbers now available (Milestone 2's
  cross-controller comparison, Milestone 3's Grover findings, the plots/video
  already rendered), a hardware parameter pass, or something else entirely.
- If asked to investigate further: the `battery_droop`/`payload_shift`
  unsurvivability finding (100% fall rate regardless of controller, despite
  fast detection) is a natural next research question, not yet root-caused.
- Run `graphify query "<question>"` before grepping the whole repo for a
  code-structure question, and update this file plus the relevant sibling file
  when you're done with a natural stopping point.

## Constraints the next model must respect

No magic numbers outside `params.py`. Every stochastic function takes an
explicit `rng`. Pure functions, explicit state threading, no global state. Never
loosen a failing physics test's tolerance — find the bug (this session found
three real bugs this way: the speed-step transient, the dropout-derivative-kick,
and an initial wrong-wall `prev_wall_err` seed). Don't build anything on the
out-of-scope list (`CLAUDE.md` §2: no CAD/ROS/GUIs/neural nets/physics
engines — 3D *rendering* specifically is now permitted, per the user-approved
scope change, but only as `analysis/animate.py`'s offline scene render, not as
a reason to add any actual physics engine, ROS, or interactive GUI). Full spec:
`CLAUDE.md`.
