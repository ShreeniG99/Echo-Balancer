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
