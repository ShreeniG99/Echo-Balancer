# Model Handoff — Echo Balancer

**Read this first.** It's the short version of everything else in
`project-context/`. Follow the links only when you need more depth on a specific
point. Last updated: 2026-09-20, commit `a7d8406` on `master`.

## What this project is (one paragraph)

A simulated two-wheeled self-balancing robot that gates its own control mode
(Normal/Cautious/Halt) on the Kalman filter's normalized innovation squared
(NIS/χ²) — detecting when the filter's model of reality has drifted (disturbance,
sensor fault, payload shift) and responding safely. Threshold selection is
compared classically vs. via Grover Adaptive Search (Qiskit), with **no speedup
claim** — report whatever the comparison actually shows. Full spec: `CLAUDE.md`
at repo root. Longer version: [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md).

## Current state in one paragraph

Build-order steps 1-3 of 8 are done and tested (34/34 passing): nonlinear plant,
RK4 integration, analytic linearization, balance LQR, sensor noise models, and a
5-state Kalman filter with NIS. **Step 4 — `sim/gate.py` (the actual gate) and
`sim/disturbances.py` — has not been started.** That is the next work. Full
detail: [CURRENT_STATE.md](CURRENT_STATE.md).

## Architecture you need to know

`sim/params.py` holds every numeric constant (no magic numbers elsewhere).
`sim/plant.py` → `sim/integrate.py` → `sim/linearize.py` → `sim/control.py` →
`sim/sensors.py` → `sim/estimator.py` is the built chain; `sim/gate.py` is the
next link. Every module has a matching file in `tests/`; a module isn't "done"
without passing tests. Full layout: [PROJECT_CONTEXT.md](PROJECT_CONTEXT.md).

## Decisions you must not silently re-litigate

- Grover comparison reports honestly either way — no speedup claim to defend.
- `EstimatorParams` defaults (`Q_theta=Q_psi=1e-8`, `Q_theta_dot=Q_psi_dot=1e-6`,
  `P0_*=1e-4`/`1e-6`) are empirically tuned against the §11 NIS test — don't
  retune on intuition (see below).
- NIS test's acceptance band is intentionally `chi2(3)`'s own 95% interval, not
  the much tighter i.i.d.-mean interval — that tighter band was tested and is not
  achievable for this system (correlated samples).
- Full list with reasons: [DECISIONS.md](DECISIONS.md).

## Failed approaches — do not repeat

- Large initial KF covariance `P0` (e.g. `1e-2`) → causes a 200-600 NIS startup
  transient. Use small `P0` (robot starts upright at rest, that's known).
- Raising `Q` to raise mean NIS toward 3 → does the *opposite* in the nominal
  (no-mismatch) case. Full list: [FAILED_APPROACHES.md](FAILED_APPROACHES.md).

## Current problems

- `sim/gate.py` doesn't exist — that's the project's actual contribution, still
  unbuilt.
- Two untracked PDFs sit at the repo root with unclear purpose — don't assume,
  ask the user.
- Hardware params (§6.2) are entirely placeholder; `qiskit`/`qiskit-optimization`
  not yet installed/verified. Full list: [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md).

## Exact config/parameters in force right now

```
PlantParams:      CLAUDE.md §6.1 NXTway-GS values (g=9.81, m=0.03, R=0.04, M=0.6, ...)
LQRBalanceParams: Q=diag(1, 1e3, 1, 1), R=1e2
SensorParams:     sigma_gyro=0.005, gyro_bias_walk=1e-4, sigma_accel=0.02,
                  encoder_cpr=360, ultrasonic_* set but unused (no world.py yet)
EstimatorParams:  Q_theta=Q_psi=1e-8, Q_theta_dot=Q_psi_dot=1e-6,
                  P0_theta=P0_psi=P0_theta_dot=P0_psi_dot=1e-4, P0_bg=1e-6
Loop rates:       dt_plant=1ms (RK4), dt_control=5ms (200Hz, ZOH — not yet wired
                  into a general run.py; only exists inside the §11 test harness)
```

## Important files

`CLAUDE.md` (spec, source of truth) · `sim/params.py` (all constants) ·
`docs/superpowers/plans/*.md` (per-step design rationale, read before touching
tuned values) · `tests/` (definition of done) · `graphify-out/graph.json`
(queryable code graph — see below).

## What the previous model (this session) was doing

Not implementing simulation code — this session set up a **persistent
project-knowledge system**: installed Graphify (`graphifyy` via `uv tool
install`), ran a project-scoped Claude Code integration
(`graphify install --project --platform claude`), built the code-structure graph
(`graphify-out/graph.json`, `GRAPH_REPORT.md`, `graph.html` — **code-only**;
docs/PDFs were not semantically extracted, see below), and wrote this
`project-context/` layer plus a Claude-Chat-facing summary
(`CLAUDE_PROJECT_CONTEXT.md`). No simulation code was touched. Verified: full
test suite still passes (34/34), no source files modified.

**One thing left half-done, by design, not by accident:** the graph (328 nodes,
597 edges after `graphify update .`) covers code (full AST, including
`# NOTE:`/rationale-comment nodes from `sim/*.py`) and markdown *structure*
(section headings, as `document`-type nodes) for `CLAUDE.md`, the plan docs, and
`project-context/*.md` — all done locally, 0 token cost. It does **not** yet have
deep semantic extraction of doc *prose* (cross-linking concepts mentioned inside
paragraphs) or either of the two root PDFs, both of which need an LLM backend.
Attempting `--backend claude-cli` for community labeling failed with a
nested-session hook conflict in this environment
(`SessionEnd hook ... Hook cancelled`) — it fell back gracefully to placeholder
community names, which is why some of `GRAPH_REPORT.md`'s communities are named
after a hub symbol (`"PlantParams"`, `"f"`, ...) rather than an LLM-written
description. No `ANTHROPIC_API_KEY`/`OPENAI_API_KEY`/`GEMINI_API_KEY` is
configured in this environment. If you have a working key or a `/graphify` skill
invocation available, running full (non-`--code-only`) extraction would add
semantic doc-prose nodes and could label communities more descriptively.

## What the next model should do

- If continuing simulation work: start build-order step 4
  (`sim/gate.py`, `sim/disturbances.py`) per
  [CURRENT_STATE.md](CURRENT_STATE.md)'s "Immediate next steps." Use
  `superpowers:writing-plans` the way the three existing `docs/superpowers/plans/`
  documents were written, for consistency.
- If continuing the knowledge-graph setup: consider running full (non-code-only)
  Graphify extraction once an LLM backend is available, and/or `graphify hook
  install` if the user explicitly wants automatic git-triggered graph rebuilds
  (deliberately not installed this session — see [DECISIONS.md](DECISIONS.md)).
- Either way: run `graphify query "<question>"` before grepping the whole repo
  for a code-structure question, and update this file plus the relevant sibling
  file (`CURRENT_STATE.md` for state changes, `DECISIONS.md` for new decisions,
  `FAILED_APPROACHES.md` if something is tried and doesn't work) when you're done
  — not for every small step, just at a natural stopping point.

## Constraints the next model must respect

No magic numbers outside `params.py`. Every stochastic function takes an
explicit `rng`. Pure functions, explicit state threading, no global state. Never
loosen a failing physics test's tolerance — find the bug. Don't build anything on
the out-of-scope list (`CLAUDE.md` §2: no CAD/ROS/GUIs/neural nets/3D physics
engines). Full spec: `CLAUDE.md`.
