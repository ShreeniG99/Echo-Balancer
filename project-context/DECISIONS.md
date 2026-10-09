# Decisions — Echo Balancer

Real decisions found in the repo (`CLAUDE.md`, `docs/superpowers/plans/*.md`,
inline code comments), not invented ones. Dates are commit dates from `git log`
where known. "Alternatives considered" and "reason" are only filled in where the
source actually states them — no speculation.

---

### Decision: Grover Adaptive Search makes no speedup claim
- **What:** The quantum threshold-selection comparison (grid search vs.
  `GroverOptimizer`) will be reported honestly regardless of outcome — same
  optimum or not, oracle calls, wall-clock, qubit count all get reported plainly.
- **Reason (stated in `CLAUDE.md` §13 step 5):** "once the cost table is computed,
  the classical argmin is trivial; the contribution is the QUBO formulation of
  safe-switching-policy selection, not speed."
- **Date:** in the spec from project inception; not yet exercised (`quantum/` not built).
- **Still current:** Yes — this is a project-level constraint, not a tunable.

---

### Decision: back-EMF (`Kb`) must be zeroed too for the energy-conservation test
- **What:** `tests/test_plant.py`'s energy-conservation check (`CLAUDE.md` §11:
  "no input, no friction, f_m=0 ... conserved to <0.1% over 5s") overrides
  `Kb=0.0` in that test's own `PlantParams` instance only — `default_plant_params()`
  keeps the real hardware `Kb`.
- **Reason:** `β = n²·Kt·Kb/Rm + f_m` models back-EMF resistive damping through the
  motor windings, a real electrical dissipation effect distinct from `f_w`
  (wheel/floor friction) and `f_m` (body/motor mechanical friction). With only
  `f_w=f_m=0`, the derived Lagrangian gives
  `dE/dt = -2β(θ̇-ψ̇)² - (W²/2R²)β·φ̇²` — strictly negative whenever `β≠0`, so the
  literal "f_m=0" reading of the spec would make this test correctly (and
  misleadingly) fail.
- **Alternatives considered:** zeroing `Kt` instead of `Kb` (both appear only as a
  product in `β`, so either works) — `Kb` was chosen "because it's the more
  legible name for 'no motor damping' in the test." Modeling explicit motor
  coast/brake modes (a real TB6612FNG driver feature) was considered as a way to
  avoid needing this override at all, but explicitly left out as out-of-scope
  ("would add a small amount of scope... which the current instruction didn't ask for").
- **Date:** 2026-09-19 (`docs/superpowers/plans/2026-09-19-plant-model-step1.md`).
- **Still current:** Yes.

---

### Decision: recovery test uses per-plant-step feedback, not the 200Hz ZOH loop
- **What:** The ψ₀=10° nonlinear recovery test recomputes `u=-K·x` every 1ms plant
  step, not every 5ms control step with ZOH on voltage in between.
- **Reason:** The real 200Hz/1kHz multi-rate architecture (`CLAUDE.md` §7) is
  `sim/run.py`'s job (build-order step 6), which needs a real sensor/estimator
  pipeline to sample and hold — neither existed at the time of this test
  (build-order step 2). Recomputing feedback every 1ms is strictly more
  conservative for stability than the coarser 5ms ZOH loop, so the test doesn't
  overstate controller robustness — it just doesn't test the final architecture yet.
- **Date:** 2026-09-19 (`docs/superpowers/plans/2026-09-19-linearize-lqr-step2.md`).
- **Still current:** Yes, but **temporary by design** — must be revisited once
  `sim/run.py` exists, since it tests a different (simpler) control architecture
  than what will actually ship.

---

### Decision: LQR Q/R weights live in `sim/params.py` as `LQRBalanceParams`
- **What:** `Q=diag(1, 1e3, 1, 1)`, `R=1e2` (the only LQR design point `CLAUDE.md`
  §6.1 gives) is a frozen dataclass in `params.py`, not hardcoded in `control.py`.
- **Reason:** §3's "no magic numbers anywhere else" rule; also the only tuning the
  spec provides, so it doubles as the correct default.
- **Date:** 2026-09-19.
- **Still current:** Yes.

---

### Decision: `EstimatorParams` defaults are the empirically-tuned values in `sim/params.py`
- **What:** `Q_theta=Q_psi=1e-8`, `Q_theta_dot=Q_psi_dot=1e-6`,
  `P0_theta=P0_psi=P0_theta_dot=P0_psi_dot=1e-4`, `P0_bg=1e-6`.
- **Reason:** `CLAUDE.md` gives no target values for the KF's Q/R (unlike the LQR
  sanity check); these were found by iterating against the §11 NIS-consistency
  test on this repo's actual plant/control code before the implementation plan was
  written. See [FAILED_APPROACHES.md](FAILED_APPROACHES.md) for the two things
  that were tried and didn't work (large initial P, larger Q) — both inform why
  these specific numbers, not nearby ones, are correct.
- **Alternatives considered:** larger initial `P` (`diag(1e-2,...)`, "safely
  uncertain") — rejected, caused a startup-transient NIS spike to 200-600.
- **Date:** 2026-09-20 (`docs/superpowers/plans/2026-09-20-sensors-estimator-nis-step3.md`).
- **Still current:** Yes. **Do not retune without reading the plan doc's "Tuning
  notes" section first** — the intuitive direction for several of these
  parameters is backwards for this specific nominal-run scenario.

---

### Decision: NIS-consistency test acceptance band is χ²(3)'s own 95% interval, not the tighter i.i.d.-mean interval
- **What:** The §11 test accepts `mean(NIS) ∈ [chi2(3).ppf(0.025), chi2(3).ppf(0.975)] ≈ [0.216, 9.348]`
  and `fraction_above_95th_percentile ∈ [0.01, 0.10]`, rather than the much
  tighter `[chi2(3600).ppf(0.025)/N, chi2(3600).ppf(0.975)/N] ≈ [2.956, 3.044]`
  that a literal "95% interval for the mean of N i.i.d. χ²(3) samples" reading
  of the spec text would imply (N=12000 for a 60s run at 200Hz).
- **Reason:** Empirically tested directly — running the same closed loop with
  **zero** added process noise across 4 seeds gave sample means of
  `2.875, 2.831, 2.671, 3.066`, a spread ~10x wider than the tight interval
  predicts. This confirms successive NIS samples are correlated (through the
  KF's own smoothing and near-deterministic encoder quantization), so the
  textbook "variance shrinks as 1/N" assumption behind the tight interval does
  not hold for this system. See [FAILED_APPROACHES.md](FAILED_APPROACHES.md).
- **Alternatives considered:** the tight i.i.d.-mean interval (rejected, not
  achievable for this system, confirmed empirically — not a tuning failure).
  Averaging over many independent 60s Monte Carlo runs was noted as a possible
  legitimate way to actually shrink the interval later, but explicitly deferred
  as "a much larger undertaking."
- **Date:** 2026-09-20.
- **Still current:** Yes, but flagged in the plan doc itself as worth revisiting
  if a future contributor wants the stricter statistical guarantee.

---

### Decision: encoder quantizes per-wheel, then averages (not average-then-quantize)
- **What:** `sim/sensors.py::encoder` quantizes each wheel's `(θ_l-ψ)`/`(θ_r-ψ)`
  independently to `2π/CPR`, then averages the two quantized readings.
- **Reason:** Real hardware physically quantizes each wheel's encoder ticks before
  any averaging can happen — you can't un-quantize a tick count. The alternative
  (average the raw angles, then quantize once) is mathematically simpler and would
  make the result collapse to `quantize(θ-ψ)` with no yaw dependence at all (since
  `θ_l`,`θ_r`'s yaw offsets cancel exactly under pre-quantization averaging).
- **Alternatives considered:** average-then-quantize — explicitly flagged as "if
  you disagree with this reading of the spec, flag it," since the two readings
  currently agree exactly in all existing tests (no yaw is excited, `φ=0`
  throughout the NIS test) but will diverge once yaw is exercised.
- **Date:** 2026-09-20.
- **Still current:** Yes, but **untested under nonzero yaw** — see
  [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md).

---

### Decision: `qiskit`/`qiskit-optimization` not added to `pyproject.toml` yet
- **What:** The full stack list (`CLAUDE.md` §3) includes `qiskit` +
  `qiskit-optimization`, but they are deliberately not yet a dependency.
- **Reason:** "pinning `qiskit`/`qiskit-optimization` now without being able to
  verify `GroverOptimizer` imports (step 7) risks a pin we'll just redo later" —
  §3 itself says to "verify `GroverOptimizer` imports before writing quantum code."
- **Date:** 2026-09-19.
- **Still current:** Yes — still blocked on reaching build-order step 7.

---

### Decision: `GateParams.tau1`/`tau2` are empirically calibrated, not literal χ²(3N) quantiles
- **What:** `N=200` (1s window), `tau1=1300`, `tau2=1800`, `tau1_exit=1040`, `tau2_exit=1300`, `T_dwell=0.5s`.
- **Reason:** Literal `chi2(3N)` quantiles are provably unachievable for this
  system at any confidence level — see [FAILED_APPROACHES.md](FAILED_APPROACHES.md).
  Values were calibrated with ~35% margin above the observed maximum of ε_k across
  10 independent 60s seeded nominal runs (959.1), then verified against seed=42
  specifically for the checked-in test.
- **Alternatives considered:** literal χ²(3N) quantiles at any level (rejected,
  provably unachievable — see FAILED_APPROACHES.md); fixing the underlying slow
  θ-position pole via the §9 speed-servo integral term first (rejected as a
  materially larger undertaking than this build step).
- **Date:** 2026-09-21 (`docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md`).
- **Still current:** Yes. **Do not retune without reading the plan doc's "Design
  decision: τ1/τ2 cannot be literal χ²(3N) quantiles" section first.**

---

### Decision: NORMAL→HALT "skip" fires when the window is full, not on a separate sustained-duration timer
- **What:** `CLAUDE.md` §10's "No NORMAL → HALT skip unless ε_k > τ2 for a full
  window" is implemented as: allowed exactly when the N-sample window is fully
  populated (not mid-warm-up) and ε_k > τ2 — no separate persistence timer.
- **Reason:** ε_k, being an N-sample sum, already *is* "a full window" of evidence
  by construction; the simpler reading needs no extra state beyond what `step_gate`
  already tracks, and is a faithful literal reading of the clause.
- **Alternatives considered:** a separate sustained-duration timer distinct from
  window-fullness (rejected — more complex, not clearly required by the text).
- **Date:** 2026-09-21.
- **Still current:** Yes.

---

### Decision: push disturbance is a ψ̇ velocity kick, not a torque impulse requiring an inertia conversion
- **What:** `sim.disturbances.push_psi_dot_kick` adds a magnitude directly to ψ̇
  (rad/s) at onset, rather than computing an angular impulse and converting via
  `Δψ̇ = J/I_eff`.
- **Reason:** Exact via the impulse-momentum theorem for some implied `J`; avoids
  inventing an "effective inertia" constant with no clear source in the spec.
- **Date:** 2026-09-21.
- **Still current:** Yes.

---

### Decision: voltage clipping (`clip_voltage`, `V_batt`) lives in `sim/disturbances.py`, not `sim/plant.py`
- **What:** `CLAUDE.md` §5 describes `v = clip(u_cmd, ±V_batt)`, but §8 says
  "V_batt is a state of the disturbance model." Since V_batt only varies via the
  battery-droop disturbance, and `sim/plant.py` (already approved, step 1) takes
  already-clipped `v_l, v_r` directly with no signature change needed, clipping
  lives with the disturbance model that owns V_batt.
- **Reason:** Keeps `sim/plant.py` completely untouched by step 4.
- **Date:** 2026-09-21.
- **Still current:** Yes.

---

### Decision: battery-droop and payload-shift disturbance tests need a companion condition to be observable at all
- **What:** `DisturbanceParams.battery_droop_companion_push_magnitude` (0.05 rad/s,
  itself independently verified too small to trigger detection alone) and
  `payload_shift_test_psi0_deg` (2°) are test-harness setup values, not part of
  either disturbance's own physical profile per §8.

---

### Decision: `design_lqr_speed_servo` is a separate 5-state LQR, not a modification of `design_lqr_balance`
- **What:** A new function, state `[θ-θ_ref, ψ, θ̇-θ̇_ref, ψ̇, ∫(θ-θ_ref)dt]`,
  built by augmenting `linearize_planar`'s existing 4x4 `A`/`B` with one
  integrator row. `design_lqr_balance`/`LQRBalanceParams` (step 2) are
  completely unmodified — every test depending on them (recovery test, NIS
  test, gate tests) keeps exercising the exact same code path as before.
- **Reason:** First attempt fed a moving reference straight into the existing
  4-state balance LQR (`u = -K·[θ-θ_ref(t), ψ, θ̇-θ̇_ref, ψ̇]`). Result: 42%
  steady-state speed error (`θ̇_ref=1.0` settles at `θ̇≈1.42`) — pure state
  feedback has no integral action to drive a ramp-reference error to zero.
  Verified gain: `K=[-0.906,-53.30,-2.312,-5.011,-0.316]`,
  `Q=diag(1,1e3,1,1,10)`, `R=1e2`, all closed-loop poles stable
  (`-241.8, -0.394±0.389j, -7.66, -6.34`).
- **Caveat also worth knowing:** under a *ramping* reference (not constant),
  the closed-loop dynamics are NOT identical to the nominal `θ_ref=0` design —
  `A_planar[2,2]`/`A_planar[3,2]` introduce constant bias terms. Tracking error
  still converges to exactly zero at steady state (Type-1 servo / integral
  action rejecting the ramp-induced disturbance), but the integrator's own
  steady-state value is nonzero (`z_eq≠0`) under a ramp — don't assume the
  ramping and nominal cases behave identically beyond that tracking-error
  convergence.
- **Date:** 2026-09-21 (step 5).
- **Still current:** Yes.

---

### Decision: yaw control is proportional-only, permanently — do not add `Kd`
- **What:** `yaw_p_control` implements `u = Kp·(φ̇_ref - φ̇)` only.
  `YawControlParams` has no `Kd` field by design.
- **Reason:** The plant's actual yaw dynamics (`sim/linearize.py`, row/col 5)
  are `φ̈ ≈ -95.57·φ̇ + 106.16·(v_r-v_l)` — an open-loop pole at ≈-95.6 rad/s
  (~10ms time constant), uncomfortably fast relative to the 200Hz (5ms) control
  rate. Tested discrete PD: any nonzero `Kd` destabilizes the loop almost
  immediately (`Kd=0.01` borderline, `Kd=0.02` diverges to ~1e36 within 3s,
  `Kd=0.05` diverges outright). This is a genuine discrete-time sampling
  problem (large-gain derivative amplifying an already-fast plant pole at a
  fixed sample rate), not a tuning imprecision. `Kp=3.0` gives comfortable
  margin below the ~8-10 instability onset, at the cost of ~62%
  steady-state φ̇-tracking error (expected P-only behavior, not a bug) —
  compensated for by the outer wall-following loop reacting to actual measured
  distance error regardless of why φ̇ undershot.
- **If this needs revisiting:** requires either a faster control rate or a
  restructured yaw loop (e.g. state feedback using more than just φ̇) — not a
  `Kd` tweak on this design.
- **Date:** 2026-09-21 (step 5).
- **Still current:** Yes.

---

### Decision: the corridor is two infinite parallel walls, no corners or dead ends
- **What:** `sim/world.py::cast_ray` models walls at `y=0` and `y=width` only,
  unbounded in `x`. `ray_parallel_eps` (near-parallel-ray tolerance) lives in
  `CorridorParams`, not hardcoded (moved there after code review flagged the
  original hardcoded `1e-9` as a CLAUDE.md §3 violation).
- **Reason:** Simplest geometry sufficient to test wall-following; keeps
  `cast_ray` a one-line case split (no segment-endpoint intersection tests).
  Consistent with `CLAUDE.md`'s own "cut wall-following first if behind
  schedule" guidance suggesting a minimal implementation is appropriate — even
  though the user chose to build the *full* wall-following stack rather than
  cut it, the corridor geometry itself was still kept minimal.
- **Consequence:** `front_threshold_speed_adjust` can only be exercised
  end-to-end via heading drift toward a side wall, not a genuine head-on
  obstacle (nothing ahead to hit). It's implemented and unit-tested as a
  correct, general function regardless.
- **Date:** 2026-09-21 (step 5).
- **Still current:** Yes — a dead-end corridor is a natural extension, not
  required by CLAUDE.md §9's literal wording.

---

### Decision: `sim/world.py::pose_velocity` integrates at 1ms plant-substep resolution, not 5ms control-tick resolution
- **What:** In `tests/test_wall_following.py`'s closed loop, `pose_velocity` is
  called via `rk4_step` once per 1ms plant substep (using the just-updated
  `x[3]`/`x[2]` after each plant RK4 step), not once per 5ms control tick.
- **Reason:** `pose_velocity` is designed to plug into `rk4_step` exactly like
  `sim/plant.py::f` does, holding its input `(θ̇, φ)` constant across one step —
  the same ZOH pattern `plant.f` uses for `(v_l, v_r)`. But unlike `v_l`/`v_r`
  (a true DAC-held control input), `φ` is a genuine plant *state*, continuously
  evolving — freezing it is only a good approximation if the freeze window is
  short relative to how fast φ actually changes. A code reviewer flagged that
  the original plan (a hand-rolled position-update formula at 5ms resolution)
  left `pose_velocity` untested by its only specified consumer; this was fixed
  by integrating at the plant's own 1ms resolution instead — a 5x tighter
  freeze window, and it makes `pose_velocity` the function actually exercised.
  Re-verified numerically: results matched the old hand-rolled formula's
  pre-verified numbers (seed=0) to within ~0.0002 — negligible difference.
- **Date:** 2026-09-21 (step 5).
- **Still current:** Yes.
- **Reason:** Both disturbances, tested in isolation near equilibrium, never bind
  anything the gate can detect — battery droop's voltage ceiling is never
  approached by the tiny nominal command magnitudes, and payload shift's mass/CoM
  mismatch needs real acceleration (which near-perfect balance doesn't produce) to
  manifest. See [OPEN_PROBLEMS.md](OPEN_PROBLEMS.md) for the broader sensitivity
  limitation this points at.
- **Alternatives considered:** none — this was discovered empirically, not chosen
  among options.
- **Date:** 2026-09-21.
- **Still current:** Yes. **Do not feed these two fields into §12's cross-controller
  evaluation as if they were part of the canonical disturbance profile** — see
  `DisturbanceParams`' own docstring in `sim/params.py`.

---

### Decision: this session — Graphify installed project-scoped, git hooks not installed
- **What:** Graphify (`graphifyy` on PyPI) installed via `uv tool install`,
  registered with Claude Code via `graphify install --project --platform claude`
  (writes `.claude/skills/graphify/`, appends a `## graphify` section to root
  `CLAUDE.md`, adds a soft-nudge `PreToolUse` hook in `.claude/settings.json`).
  `graphify hook install` (automatic git post-commit/post-checkout rebuild +
  merge driver) was **not** run.
- **Reason:** The user's request explicitly asked for Claude Code
  integration (hooks/skills/CLAUDE.md), which this fulfills. Automatic git hooks
  are a standing, always-fires-on-commit change to normal git operations that
  wasn't explicitly requested — left for the user to opt into deliberately
  (command: `graphify hook install`) rather than installed silently.
- **Date:** 2026-09-20 (this session).
- **Still current:** Yes, pending the user's choice on git hooks.

---

### Decision: `sim/run.py` unifies the three closed-loop harnesses via an `EpisodeConfig.wall_following` flag, not a single shared control law
- **What:** `run_episode` uses `design_lqr_balance` (KF-fed) for every episode where `wall_following=False`, exactly reproducing `tests/test_gate.py`/`tests/test_estimator.py`'s already-calibrated algorithm. `wall_following=True` episodes use `design_lqr_speed_servo` (true-state-fed, matching step 5's precedent), required regardless since `design_lqr_balance` cannot track a nonzero speed reference.
- **Reason:** Measured that switching the balance/gate control law to `design_lqr_speed_servo` (either KF-fed or true-state-fed, at `theta_dot_ref=0` or nonzero) erodes the push-disturbance detection margin below `tau1=1300` in every variant tried (1344.87 for the existing `design_lqr_balance`, vs. 1111-1187 for every speed-servo variant) — `GateParams` was calibrated against `design_lqr_balance` specifically and does not transfer.
- **Alternatives considered:** a single shared `design_lqr_speed_servo` control law everywhere (rejected — breaks push/battery-droop detection, would require a full GateParams recalibration, a materially larger undertaking than this step).
- **Date:** 2026-09-22 (`docs/superpowers/plans/2026-09-22-run-batch-metrics-step6.md`).
- **Still current:** Yes.

---

### Decision: corridor episodes default to `corridor_theta_dot_ref_nominal=0.3` rad/s, not `tests/test_wall_following.py`'s `2.0`
- **What:** `RunParams.corridor_theta_dot_ref_nominal=0.3`.
- **Reason:** With the KF/gate actually watching a real wall-following (yaw-active) closed loop for the first time, `2.0` rad/s produces `max_epsilon=1605.56` over a nominal 60s run (exceeds `tau1=1300`, spurious CAUTIOUS). Swept `0.1/0.2/0.3/0.5`: `0.1` diverges/falls (`max_epsilon` reaches `~2.1e8` without a fall-check), `0.2`→689, `0.3`→909 (comparable to the balance-only nominal range of 823-951), `0.5`→1605 (unsafe). `0.3` was chosen as the best-margined safe value.
- **Alternatives considered:** `2.0` (rejected, unsafe once the gate is watching); `0.5` (rejected, exceeds `tau1`); `0.1`/lower (rejected — the relationship is not monotonic in speed, and `0.1` specifically destabilizes the closed loop entirely, not fully explained here).
- **Date:** 2026-09-22.
- **Still current:** Yes.
- **Update (same day):** This decision was validated against gate-triggering (epsilon vs `tau1`) at `seed=42` only. A full 10-seed batch run later surfaced a genuine ~20% nominal fall rate independent of gate choice at this speed (`naive`/`nis_gate` both 0.2, same falling seeds 3 and 5, near-identical fall times; `tilt_threshold` 0.5, its own additional falls correlated with its own mode-switching) — see `OPEN_PROBLEMS.md`. The epsilon-based safety claim above is still correct on its own terms; "safe" here should not be read as "never falls."

---

### Decision: `default_speed_servo_params_cautious()` divides every `SpeedServoParams` Q weight by 5
- **What:** CAUTIOUS mode's "softer Q" gain set (CLAUDE.md section 10).
- **Reason:** Reducing `Q_psi` alone (the dominant weight) barely changes `K` (`||K||` ratio 0.998) since it already dominates the other weights by 3 orders of magnitude. Dividing all five weights by 5 (mathematically equivalent to `R x5`) gives a genuinely gentler gain — verified stable, dominant pole `-0.263` vs nominal `-0.394` (~33% slower), `||K||` ratio 0.967.
- **Date:** 2026-09-22.
- **Still current:** Yes.

---

### Decision: HALT mode uses the nominal (not cautious) speed-servo gain
- **What:** Only CAUTIOUS uses `default_speed_servo_params_cautious()`; HALT sets `theta_dot_ref=0` but keeps the nominal gain.
- **Reason:** CLAUDE.md section 10 states "softer Q" only for CAUTIOUS, not HALT — a literal reading, consistent with prior steps' approach to ambiguous spec clauses (see the "NORMAL->HALT skip" and "push is a velocity kick" decisions above).
- **Date:** 2026-09-22.
- **Still current:** Yes.

---

### Decision: `experiments/run_batch.py`'s workers receive scenario data as an explicit parameter, not via module-level lookup
- **What:** `_run_one(controller, scenario_name, scenario, seed, wall_following)` takes the already-resolved `DISTURBANCE_SCENARIOS[scenario_name]` tuple as an explicit argument; `run_batch()` resolves it in the parent process and passes it through `delayed()`, rather than having `_run_one` look up the module-level `DISTURBANCE_SCENARIOS` dict itself.
- **Reason:** `joblib.Parallel`'s default `loky` backend needs process-based workers; on this Windows dev machine, `multiprocessing.get_all_start_methods()` returns only `['spawn']` (no fork/forkserver), so every worker process re-imports the module fresh from disk and never inherits the parent process's in-memory state. A worker-side lookup of `DISTURBANCE_SCENARIOS` therefore silently reads on-disk module state, invisible to anything (e.g. a test's `monkeypatch`) that modified it in the parent process. Caught by `tests/test_run_batch.py::test_run_batch_writes_readable_parquet` failing with 12000 rows instead of an expected 200 (the un-patched `nominal` duration of 60s was used instead of the patched 1.0s).
- **Alternatives considered:** none seriously — this is the correct general pattern (workers should never depend on mutable global state across a process boundary); dropping to `n_jobs=1`/threading to sidestep the issue was explicitly rejected as it would defeat the purpose of using joblib at all.
- **Date:** 2026-09-22.
- **Still current:** Yes. **If porting this codebase to a POSIX system where `fork` is available, do not "simplify" this back to a worker-side lookup** — the bug is latent on fork-based systems too (it just doesn't manifest, since fork *does* inherit parent memory), and would resurface the moment anyone runs on spawn-only Windows again.

## 2026-10-08 — Wokwi firmware lives in `firmware/` (user-approved scope addition)
Wokwi isn't in CLAUDE.md's stack/layout; the user explicitly approved adding a C++/PlatformIO `firmware/` dir. Scope = **the gate on ESP32** (KF + NIS + both gates), not balancing or wall-following. Reason: Wokwi has no physics, so the robot is a virtual linear plant inside the firmware (a still-sensor Wokwi setup trips the gate by itself because the KF predicts LQR corrections that never appear in the sensors — tried first, rejected). All numerics come from `sim/params.py` via `firmware/tools/gen_params_header.py`; `tests/test_firmware_core.py` pins C++ == Python.

## 2026-10-08 — Firmware fallback: softer gains in HALT + KF Q x100 outside NORMAL; target is ESP32-S3
Payload-shift run fell in the firmware. Root cause (verified numerically, *not* "LQR can't stabilise"): true-state feedback is stable on the payload plant; the nominal-model KF in the loop is what destabilises it, and the nominal K5 is also marginal in the sampled loop (rho 1.0082 vs 0.9993 for the cautious set). Fix, firmware only: HALT uses the cautious gain set (CLAUDE.md §10 only requires "keep balancing"); KF Q x `FallbackParams.q_inflation` (=100, `sim/params.py`) outside NORMAL. x10/x100 recover, x1000+ fall. `sim/run.py` NOT changed (still nominal gain in HALT, step-6 decision 6) — Python batch metrics unchanged. Board is ESP32-S3 (user's choice): GPIO 25-27 don't exist there, so pins are SDA8/SCL9, LEDs 4/5/6, buzzer 7, encoder 15/16.

## 2026-10-08 — Fallback ported to `sim/run.py` (default on)
`EpisodeConfig.fallback=True`: outside NORMAL the KF uses Q x `FallbackParams.q_inflation` (all episodes) and HALT uses the cautious K5 (wall-following). Supersedes step-6 "Design decision 6" (HALT on nominal gain) — that was a literal-reading choice never tested on payload shift. Verified on the nonlinear plant: payload shift seed 1 falls without, upright with (`tests/test_run.py::test_fallback_prevents_payload_shift_fall`). Cost: during the accel-noise fault, peak |psi| rises ~0.1 deg -> ~1 deg (inflated Q trusts the noisy accel more). `fallback=False` reproduces the old behaviour.

## 2026-10-08 — QUBO formulation: exact HUBO -> Rosenberg QUBO, rank-transformed, verified by brute force
Binary candidate index makes an arbitrary cost table a degree-n polynomial (HUBO), not a QUBO. We take the exact Mobius-transform HUBO of the *rank-transformed* table (monotone => same argmin; small integer coefficients => small Grover value register), reduce it to a QUBO with Rosenberg substitution using tight per-aux penalties, and brute-force-verify every QUBO decodes to argmin J (`quantum/formulation.py::verify_exact`). Infeasible codes (tau1 >= tau2) get rank = worst feasible + 1. Qubit budget: 6-bit (64-candidate) QUBO = 18 vars + ~16 value qubits ~ 34 qubits -> not simulable here, so Grover runs on the 5-bit (tau1 x tau2 x N) and 4-bit (tau1 x tau2) sub-grids at the default T_dwell/N; exhaustive search covers all 64. Rejected: one-hot over candidates (64 qubits); non-exact quadratic fit (minimum not guaranteed).

## 2026-10-08 — qiskit pinned: qiskit 2.5.2, qiskit-optimization 0.7.0, qiskit-aer 0.17.2
`GroverOptimizer` works on qiskit 2.x only with an Aer `SamplerV2` **plus a pass_manager** (without it the sampler job fails). Oracle calls are counted by wrapping `Grover.construct_circuit` (sum of Grover powers); `result.operation_counts` is per-round gate counts, not oracle calls.

## 2026-10-08 — Side-ultrasonic dropout rejection (`sim.sensors.hold_on_dropout`)
Root cause of nominal corridor falls (pre-existing since step 5/6: 60 s nominal corridor runs fell in 2/10 naive, 2/10 NIS, 5/10 tilt seeds): a 2% dropout returns max range (4 m), stepping the wall error by ~3.5 m; the wall-following D-term turns that into ~-35 rad/s yaw-rate ref and saturates the differential drive (+/-7.4 V), kicking the heading each time until the robot faces a wall and loses balance. Fix: hold the last valid side reading on a max-range reading (rng call order unchanged). After: 0/30 falls, max |phi| 20 deg, tilt gate's corridor false fallback 17.6% -> 0. `test_cautious_mode_scales_down_corridor_speed_reference` relied on these kicks to trip CAUTIOUS; rewritten to force CAUTIOUS with a GateParams override.

## 2026-10-09 — Threshold-selection cost: spec J is official; J_detect is a labelled extra (user's choice "C")
CLAUDE.md section 13's J (falls, false fallback, progress) stays the official selection cost. Because J does not reward detection and ties heavily once nothing falls, `J_detect = J + w_detect_variant*(1 - detection_rate)` (weight 1, `QuboParams.w_detect_variant`) is reported alongside, always labelled as NOT the spec J. Reason: changing the rule after seeing results invites "you picked the rule that gave a nicer answer". If the user later wants J_detect official: fix the weight first, update CLAUDE.md, re-run on fresh seeds, then report.
