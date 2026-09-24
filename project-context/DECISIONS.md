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

### Decision: user-approved scope change — 3D animation instead of the planned 2D side/top view
- **What:** `CLAUDE.md` §2 originally excluded "3D rendering" alongside CAD/physics
  engines/ROS/GUIs; the user explicitly asked for a 3D video demo instead. §2/§3/§4/§14
  were updated: "3D rendering" removed from the out-of-scope list (CAD/physics
  engines/ROS/GUIs/online learning/neural networks stay banned), `pyvista`+`vtk`
  (+`imageio`/`imageio-ffmpeg`) added to the stack, `analysis/animate.py`'s role
  changed from "2D side view + top view" to a 3D scene render.
- **Reason:** Direct user request ("I want the 3D video, not 2D. Demo has to be as
  impressive as possible"), explicitly deferred until after build-order steps 6/7
  were done (also the user's own instruction) so it didn't interrupt Milestone
  2/3 work.
- **Why this doesn't reopen the other bans:** `analysis/animate.py` computes no
  physics (it poses/colors meshes from `sim.run`'s already-simulated,
  already-logged DataFrame columns) and never opens an interactive window
  (`off_screen=True` throughout) — not a physics engine, not a GUI, not CAD.
- **Date:** 2026-09-24 (this session, after steps 6-8).
- **Still current:** Yes.

---

### Decision: `analysis/animate.py` self-manages a virtual X display (`_ensure_display`)
- **What:** Rather than requiring every caller (including `pytest`) to remember to
  wrap the whole process in `xvfb-run -a`, `render_episode` calls `_ensure_display()`
  first, which spawns `Xvfb :99 ...` directly via `subprocess.Popen` and sets
  `DISPLAY` if no display is already set.
- **Reason:** This container has no GPU/EGL/OSMesa (confirmed: VTK's off-screen
  render fails outright with "bad X server connection" and "libOSMesa not found"
  without a display); `pyvista.start_xvfb()` (the old built-in helper for exactly
  this) no longer exists in pyvista 0.49.0. `xvfb-run -a <cmd>` works but only if
  the *caller* remembers to use it — a self-managed fallback inside the module
  itself is more robust for a module other code (tests, `sim.run` users) may call
  without knowing about this environment's display quirks.
- **Alternatives considered:** requiring `xvfb-run -a` externally on every
  invocation — rejected as too easy to forget (and confirmed to actually break: an
  early version without `_ensure_display` needed `xvfb-run -a uv run pytest ...`
  for `tests/test_animate.py` to pass at all).
- **Date:** 2026-09-24.
- **Still current:** Yes. If a future environment *does* have a real
  GPU/EGL/OSMesa, `_ensure_display` is a no-op there too (checks `DISPLAY` first).

---

### Decision: `quantum/qubo.py`'s GroverOptimizer objective is a fitted quadratic surrogate, not the true cost table
- **What:** The true (re-simulated) 64-candidate cost table has no reason to be a
  degree<=2 polynomial in its 6 candidate-index bits — `GroverOptimizer`
  (`qiskit_optimization`) only accepts a linear+quadratic objective. `quantum/qubo.py`
  fits a least-squares quadratic surrogate (`_fit_quadratic_surrogate`, constant +
  linear + all pairwise terms) to the true table and hands GroverOptimizer that
  surrogate, then separately reports (a) whether Grover finds the surrogate's own
  optimum and (b) whether the surrogate's optimum matches the true table's.
- **Reason:** No other option exists for using GroverOptimizer on a black-box
  cost function of more than 2 variables without either (i) an exponential number
  of higher-order terms, or (ii) a fitted low-degree approximation. Reporting both
  checks separately (rather than one conflated "did it work") keeps the honesty
  CLAUDE.md §1/§13 asks for intact: a surrogate mismatch is a statement about the
  approximation's quality, a Grover mismatch is a statement about the search
  itself.
- **Date:** 2026-09-24 (step 7).
- **Still current:** Yes.

---

### Decision: `GroverOptimizer` is run multiple independent trials, best-of-N kept
- **What:** `run_qubo_pipeline(grover_trials=5)` runs `GroverOptimizer.solve()`
  five independent times and keeps the best `fval` found, reporting both the best
  result and how many of the N trials actually found the surrogate's true optimum
  (`grover_trial_hits`).
- **Reason:** Empirically verified during development (independent synthetic 6-bit
  QUBOs, not just this project's real surrogate) that a single `GroverOptimizer.solve()`
  call has a real, non-negligible failure probability — roughly 1-in-5 to 1-in-10
  observed at `num_value_qubits=10`, `num_iterations=8` across several synthetic
  and real trials, and the actual production run (real 64-candidate surrogate,
  margin 0.12% of range — see `OPEN_PROBLEMS.md`) got 0/5 and 0/30 hits across two
  separate runs. This is genuine probabilistic quantum-search behavior (GAS makes
  no single-shot guarantee), not a bug — treating one `.solve()` call as
  authoritative would misrepresent how the algorithm actually works.
- **Alternatives considered:** raising `num_value_qubits` to brute-force more
  precision — tried (12, 14); 14 value qubits made a single trial take minutes
  (classical statevector simulation cost grows with qubit count), making it
  impractical at this scale. Kept `num_value_qubits=10` and used repeated trials
  instead.
- **Date:** 2026-09-24 (step 7).
- **Still current:** Yes.

---

### Decision: `sim/run.py`'s balance/speed-servo control uses the KF estimate, not true state
- **What:** `run_episode`'s control loop computes `u = -K5 @ err5` from
  `kf.x_hat`, not `x_true` — resolving the "bypasses the Kalman filter for
  balance/speed state ... a documented, temporary simplification pending
  `run.py`" note left in step 5's `tests/test_wall_following.py`.
- **Reason:** This is what `run.py` was explicitly flagged as the place to fix
  (`CURRENT_STATE.md`'s step-5 entry, pre-step-6). Yaw control still uses the true
  `phi_dot` (`x_true[5]`) since the 5-state KF has no yaw state at all — nothing
  to switch there.
- **Date:** 2026-09-24 (step 6).
- **Still current:** Yes.

---

### Decision: a new `default_evaluation_gate_params()`, not a change to `default_gate_params()`
- **What:** `sim/run.py`'s closed loop (5-state speed-servo LQR, continuous
  forward motion via wall-following) uses a **separate** `GateParams` factory
  (`tau1=900, tau2=1400, tau1_exit=720, tau2_exit=1000`, same `N=200`/`T_dwell=0.5`)
  from the original `default_gate_params()` (`tau1=1300, tau2=1800, ...`), which
  is unchanged and still used by `tests/test_gate.py`'s own closed loop.
- **Reason:** These are genuinely different closed loops with different nominal
  NIS statistics, not two "reasonable" choices for the same system.
  `tests/test_gate.py`'s closed loop is balance-only (`design_lqr_balance`, robot
  near-stationary): its nominal-run epsilon max is ~959, calibrated against
  `tau1=1300`. `sim.run`'s closed loop (`design_lqr_speed_servo`, robot actually
  driving) has nominal-run epsilon max ~604 but the weakest disturbance response
  is ~1094 — using the *old* 1300/1800 thresholds here would make every
  disturbance in `experiments/run_batch.py`'s NIS_GATE controller silently
  undetectable (they'd all stay under `tau1=1300`). Overwriting
  `default_gate_params()`'s values instead would have broken
  `tests/test_gate.py::test_no_mode_change_on_nominal_60s_run` (its own documented
  max of 959.1 would exceed a lowered `tau1=900`).
- **Alternatives considered:** changing `default_gate_params()` in place —
  rejected, breaks an already-correct, already-passing test calibrated against a
  different (still-valid) closed loop; the two closed loops are both still used
  (unit-level gate/KF test vs. the real evaluation loop) and both need their own
  correct calibration.
- **Date:** 2026-09-24 (step 6).
- **Still current:** Yes. **Do not merge these two GateParams factories** without
  first checking which closed loop each caller actually drives.

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
