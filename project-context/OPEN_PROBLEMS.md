# Open Problems — Echo Balancer

Separated into confirmed facts, suspected-but-unverified problems, and genuine
unknowns. Don't upgrade a "suspected" or "unknown" to "confirmed" without
actually checking.

## Confirmed problems / gaps

- **The controller has a genuine, fairly narrow stability boundary near
  `f_w≈0.03`.** Built without the §9 speed-servo integral term yet. Confirmed via
  explicit finiteness checks (not just "falls over"): the closed-loop nonlinear
  simulation produces NaN/inf at `f_w≥0.03`. The surface-change disturbance's
  default magnitude (`0.025`) was chosen with margin below this, not just because
  it "works." Worth reporting in the write-up as a real controller limitation.
- **NIS-based gate detection is measurably less sensitive to smooth mass/CoM
  shifts and to isolated battery droop than to the other disturbance types**, under
  the current `EstimatorParams`/`GateParams` tuning. Payload shifts up to
  `delta_M=0.5kg`/`delta_L=0.05m` never trigger detection even combined with a mild
  tilt; battery droop alone near equilibrium never binds the voltage ceiling at
  all. Both need a companion condition to be observable in a test (see
  `DECISIONS.md`). This is a real, reportable sensitivity limit of the current
  tuning, not a bug — but if the write-up claims the gate detects "disturbances"
  broadly, this nuance needs stating.
- **Literal χ²(3N) quantiles for the gate's τ1/τ2 are provably unachievable** for
  this system, at any confidence level (see `FAILED_APPROACHES.md`). `GateParams`
  uses empirically-calibrated thresholds instead. If the write-up needs to defend
  a literal reading of `CLAUDE.md` §10 against this deviation, the root cause (a
  slow ~10s θ-position closed-loop pole, itself caused by the missing §9
  speed-servo integral term) is the thing to explain.
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
- **Whether `front_threshold_speed_adjust` needs a corridor dead-end to be
  meaningfully validated end-to-end.** It's implemented and unit-tested
  correctly in isolation, but the current corridor (two infinite parallel
  walls, per §9's own "cut wall-following first if behind schedule" spirit)
  has nothing ahead to trigger it via a genuine head-on obstacle in the closed
  loop — only heading-drift-toward-a-side-wall exercises it indirectly.
- **Whether/how `sim/run.py` (step 6) should unify the three overlapping
  test-only closed-loop harnesses** that have now accumulated
  (`tests/test_estimator.py::_run_nominal_closed_loop`,
  `tests/test_gate.py::_closed_loop_with_gate`,
  `tests/test_wall_following.py::test_follows_wall_without_crashing`) — each
  independently hand-rolls plant-stepping/control/state-threading plumbing.
  Not yet a confirmed problem (each is still small and independently correct),
  but the duplication is growing and `run.py` is the natural point to resolve
  it, one way or another.
