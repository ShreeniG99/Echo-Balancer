# Echo Balancer — Claude Project Knowledge

Upload this single file to a Claude Project's knowledge base to give a fresh
Claude Chat conversation the context it needs without re-explaining the project.
It is intentionally self-contained and does not assume access to the repo.

## Project overview

Echo Balancer is a simulation study of a two-wheeled self-balancing robot that
monitors its own control confidence via the Kalman-filter normalized innovation
squared (NIS, χ²) and switches between three safety modes — **Normal → Cautious →
Halt** — as that confidence degrades (disturbances, sensor faults, payload
shifts). A balancing robot can't just cut its motors when uncertain, so "Halt"
still means actively balancing in place, just not moving.

## Goal

Build the plant simulation, the NIS-based safety gate, and an honest evaluation
comparing three controllers (naive always-on, a tilt-threshold baseline, and the
NIS gate) on fall rate, false-fallback rate, missed-fallback rate, detection
delay, and progress. Separately, compare classical exhaustive search against
Grover Adaptive Search (Qiskit) for offline gate-threshold selection — with an
explicit **no speedup claim**; the contribution is the QUBO formulation, and the
result is reported honestly whichever way it lands.

## Current state

Steps 1-3 of an 8-step build order are complete and fully tested (34/34 tests
passing as of 2026-09-20): nonlinear plant dynamics, RK4 integration, analytic
linearization + balance LQR, sensor noise models, and a 5-state Kalman filter
with NIS. **The gate itself (`sim/gate.py`) — the project's actual
contribution — has not been built yet.** That's the current objective.

## Architecture

```
sim/params.py    all physical/sensor/controller constants (single source of truth)
sim/plant.py     nonlinear 6-state dynamics (θ, ψ, φ + rates)
sim/integrate.py fixed-step RK4, zero-order-hold control
sim/linearize.py analytic Jacobian, verified numerically
sim/control.py   balance LQR (speed servo / yaw PD / wall-following: not yet built)
sim/sensors.py   gyro, accelerometer, encoder noise models
sim/estimator.py 5-state Kalman filter + NIS
sim/gate.py      Normal/Cautious/Halt state machine          — NOT YET BUILT
sim/disturbances.py, sim/world.py, sim/run.py                — NOT YET BUILT
experiments/, analysis/, quantum/                             — NOT YET BUILT
```

## Important decisions

- The recovery/energy-conservation tests each needed a non-obvious fix to match
  the spec's intent exactly (e.g. the energy-conservation test must zero
  back-EMF damping, not just mechanical friction, or it fails for a real physical
  reason, not a bug).
- The Kalman filter's tuning (`EstimatorParams` in `sim/params.py`) was found
  empirically, not derived from a spec target, and two intuitive-seeming
  adjustments (bigger initial uncertainty, bigger process noise) both turned out
  to make things worse, not better — see "Failed approaches" below.
- The statistical acceptance test for NIS consistency deliberately uses a wider
  interval than a literal reading of the spec would suggest, because the tighter
  interval was tested directly and shown to be statistically unachievable for
  this system (correlated samples, not i.i.d.).

## Failed approaches (do not re-suggest these)

- Widening the Kalman filter's initial covariance "to be safe" — causes a large
  startup transient (NIS spiking to 200-600) rather than helping.
- Raising Kalman process noise `Q` to push the mean NIS statistic closer to its
  theoretical target of 3 — has the opposite effect in the nominal (no real
  disturbance) case.
- Treating the NIS-consistency test's acceptance band as the tight interval you'd
  get by naively averaging many independent samples — empirically ~10x too
  tight for this system's correlated NIS sequence.

## Current problems

- The safety gate (`sim/gate.py`) doesn't exist yet — everything built so far is
  the foundation under it.
- Two PDF files sit in the repo root with unclear purpose/provenance — not yet
  triaged.
- No real hardware measurements exist yet; all sensor/motor parameters are
  simulation-only values from the NXTway-GS reference robot, not the project's
  own ESP32/GY-87 hardware.
- `qiskit`/`qiskit-optimization` aren't installed yet; their compatibility needs
  verifying before any quantum code is written.

## Important files (in the repository, for reference — not attached here)

`CLAUDE.md` (the full spec, source of truth for everything above) ·
`sim/params.py` (every numeric constant) · `docs/superpowers/plans/*.md`
(detailed design rationale per build step) · `project-context/MODEL_HANDOFF.md`
(the fuller version of this file, kept current in the repo).

## Constraints

No magic numbers outside `sim/params.py`. Every stochastic function takes an
explicit random generator; every run must be reproducible from `(config, seed)`.
Pure functions, no global mutable state. SI units internally, degrees only in
plot/log labels. Never loosen a failing physics test's tolerance to make it
pass — find the actual bug. Explicitly out of scope: cave arenas, magnet-polarity
detection, CAD, 3D physics engines, ROS, GUIs, online learning, neural networks.

## Current task

Design and implement `sim/gate.py`: the windowed NIS statistic (ε_k, sum of the
last N raw NIS samples) and the Normal/Cautious/Halt state machine with
hysteresis and a minimum dwell time, per the spec's §10. Then
`sim/disturbances.py` for the 5 scheduled disturbance profiles, targeting the
milestone "gate switches under one disturbance."

## How to continue

If you're picking this up in a fresh Claude Chat or a different model without
the full repository open: ask the person to paste the relevant source file(s)
for the specific module you're working on (this document deliberately omits
source code) or to open the conversation from Claude Code, which has the full
repo, the `project-context/` files, and a queryable code graph (Graphify)
available. Don't guess at function signatures or parameter values — everything
concrete above is accurate as of 2026-09-20 but the repo is the live source of
truth.
