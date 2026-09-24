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
- **`GroverOptimizer`'s value-qubit encoding cannot reliably resolve this
  project's real QUBO surrogate's optimum at a classically-simulable qubit
  count.** Confirmed (not suspected): the production 64-candidate quadratic
  surrogate's true minimum beats its closest rival by only ~0.12% of the
  objective's overall range (0.026 out of a 21.05-unit span). `GroverOptimizer`
  found the true surrogate optimum in 0/5 and 0/30 independent trials (two
  separate production runs) at `num_value_qubits=10`; raising `num_value_qubits`
  to 14 to get more resolution made a single trial take minutes (classical
  Aer-simulation cost of the value register). This is a genuine, reportable
  finding about applying value-qubit-based GAS to a continuous-valued,
  closely-spaced cost landscape, not a bug in the QUBO construction (verified
  independently: `QuadraticProgram.objective.evaluate()` matches the module's
  own `_surrogate_value()` exactly for all 64 bitstrings).
- **`battery_droop` and `payload_shift` are unsurvivable (100% fall rate)
  regardless of which controller is driving**, per the real Milestone-2 batch
  (`experiments/run_batch.py`'s default 7-scenario x 3-controller x 10-seed run):
  naive, tilt-threshold, and NIS-gate controllers all show 100% fall rate on
  these two scenarios, even though the NIS gate detects both quickly (see
  `analysis/metrics.py`'s `detection_delay` — 0 missed detections across all 60
  disturbance episodes). This is a natural extension of the pre-existing
  documented finding that these two disturbances are hard to detect at all under
  the current tuning (see below) — apparently they're also hard to *survive* once
  detected, since HALT keeps balancing but doesn't change the underlying
  mass/CoM or voltage mismatch the estimator is fighting. Not yet root-caused
  further (e.g. whether a different `EstimatorParams`/`GateParams` tuning would
  help, or whether this is a fundamental limit of NIS-gating without adapting the
  controller's own model) — a real, reportable limitation either way.
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

- ~~Whether Grover Adaptive Search shows any speedup over classical grid search
  on this project's QUBO~~ **Resolved, stale as of 2026-09-24.** No speedup, by
  design/expectation (classical exhaustive search over 64 candidates is a ~0.02ms
  `min()` call). More specifically: Grover's own single-run reliability on this
  project's real surrogate is poor (see "Confirmed problems" above), a separate
  and more interesting finding than the speed comparison itself.
- **Whether a different `EstimatorParams`/`GateParams` tuning would make
  `battery_droop`/`payload_shift` survivable, or whether the underlying model
  mismatch (mass/CoM shift, voltage ceiling) is fundamentally uncontrollable via
  gating alone** (see "Confirmed problems" above) — not yet investigated.
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
  meaningfully validated end-to-end.** Still open: `sim/run.py` (step 6) still
  uses the same two-infinite-parallel-walls corridor, so this is unchanged from
  step 5 — implemented and unit-tested correctly in isolation, but nothing ahead
  to trigger it via a genuine head-on obstacle in the closed loop.
- ~~Whether/how `sim/run.py` (step 6) should unify the three overlapping
  test-only closed-loop harnesses~~ **Resolved, stale as of 2026-09-24.**
  `sim/run.py::run_episode` is now the one real closed-loop implementation
  (parameterized by `ControllerKind`); the three test-only harnesses in
  `tests/test_estimator.py`/`tests/test_gate.py`/`tests/test_wall_following.py`
  remain as their own independent (smaller, faster, narrower-scope) unit-level
  checks and were not deleted or rewritten to call `run_episode` -- they test a
  simpler closed loop (balance-only or true-state-fed) deliberately, per the
  decisions already on record for why those simplifications existed.
