# Open Problems — Echo Balancer

Separated into confirmed facts, suspected-but-unverified problems, and genuine
unknowns. Don't upgrade a "suspected" or "unknown" to "confirmed" without
actually checking.

## Confirmed problems / gaps

- **The gate itself doesn't exist yet.** `sim/gate.py` (Normal/Cautious/Halt state
  machine, the windowed ε_k statistic) is the actual contribution of the project
  per `CLAUDE.md` §1, and it hasn't been built (build-order step 4, not started).
  Everything built so far (plant/estimator/NIS) is the foundation it sits on.
- **Two PDFs live under `research/`** (`research/echo_balancer_research.pdf`,
  `research/"echo_balancer_detailed (1).pdf"` — moved there from the repo root
  on 2026-09-21), still untracked and not gitignored, purpose unstated anywhere
  in the repo. They may be reference material the user is drafting from. Still
  excluded from the Graphify graph via `.graphifyignore` (PDF semantic
  extraction needs an LLM backend, none configured). **Do not assume their
  content** — ask the user before treating them as authoritative project
  knowledge.
- **Hardware parameters (`CLAUDE.md` §6.2) are entirely unfilled.** No
  `REAL_SPEC`/`MEASURED`/`ESTIMATED`/`PLACEHOLDER`-tagged values exist yet for the
  ESP32/GY-87/TB6612FNG hardware — only the NXTway-GS §6.1 values are in use. Any
  paper claim must currently be scoped to simulation only.
- **`qiskit`/`qiskit-optimization` are not installed or import-verified.** §3
  requires verifying `GroverOptimizer` imports before writing any quantum code;
  this hasn't happened. Don't assume a compatible version pair without checking —
  the spec explicitly warns pinning blind risks a redo.
- **One Japanese-language reference PDF has a garbled filename**
  (`nxtway_gs/docs/japanese/NXTway-GS âéâfâïâxü[âXèJö¡.pdf` — Shift-JIS mojibake).
  Cosmetic; low priority; the directory is gitignored reference material anyway.

## Suspected problems (plausible, not verified)

- ~~Encoder quantize-then-average vs. average-then-quantize may matter once yaw
  is exercised~~ **Resolved, stale as of 2026-09-21.** A dedicated φ≠0 test
  (`tests/test_sensors.py::test_encoder_quantizes_per_wheel_before_averaging`,
  commit `2580160`) already exists and confirms the per-wheel reading behaves
  as intended with nonzero yaw. What's still genuinely open: this is a
  hand-constructed unit-level case, not yet exercised by a real closed-loop run
  with actual yaw motion (that needs `sim/world.py`/wall-following to generate).
- **The per-plant-step (1ms) feedback recovery test may not represent the final
  200Hz ZOH control loop's stability margin.** It was explicitly built as a
  temporary, more-conservative stand-in (see
  [DECISIONS.md](DECISIONS.md)) until `sim/run.py` exists with the real
  multi-rate architecture. It is plausible (not confirmed) that the coarser 5ms
  ZOH loop behaves differently enough to need its own recovery test once built.

## Unknowns (genuinely open, not yet investigated)

- **Whether Grover Adaptive Search shows any speedup over classical grid search
  on this project's QUBO.** By design the project makes no claim either way —
  this is meant to be discovered empirically once `quantum/qubo.py` exists, not
  assumed in advance.
- **Real-hardware sensor noise characteristics.** All σ values in
  `SensorParams` are explicitly "starting points" (`CLAUDE.md` §7), not measured —
  actual GY-87/encoder/ultrasonic noise on the real ESP32 build is unknown until
  hardware arrives and is characterized.
- **Whether the stricter i.i.d.-mean NIS interval is achievable via multi-run
  Monte Carlo averaging**, as floated as a possible future direction in the
  estimator plan doc — never attempted, cost/complexity unassessed.
- **Whether the two untracked root PDFs contain content the user wants folded
  into `project-context/` or the Graphify graph** — see "Confirmed problems"
  above; this is a "what does the user want" unknown, not a technical one.
