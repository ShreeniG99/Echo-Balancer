# Failed Approaches — Echo Balancer

Approaches that were actually tried (or directly tested) and rejected. Kept
separate from [DECISIONS.md](DECISIONS.md), which covers choices made by
reasoning rather than by trying something and watching it fail. **Do not
rediscover these** — re-tuning in these directions has already been shown not
to work, for the stated reasons.

---

### Failed: large "safely uncertain" initial Kalman filter covariance `P0`
- **Approach tried:** `P0 = diag(1e-2, 1e-2, 1e-2, 1e-2, 1e-4)` for the balance KF
  — the intuitive "when in doubt, start uncertain" choice.
- **What happened:** A large, short-lived transient in the first ~10 control steps
  (~50ms): the filter's early overreaction to sensor noise fed large voltage
  commands into the plant, producing large `θ̈`, which corrupted the accelerometer
  reading (its `ψ_acc` model includes a `θ̈`-dependent term — see
  `sim/sensors.py::accelerometer`), producing NIS values of **200-600** during
  that transient window alone.
- **Why abandoned:** This is a textbook KF startup-transient artifact, not a
  genuine model-mismatch problem — the robot is known to start upright at rest,
  so a large initial uncertainty is actively wrong, not conservative.
- **What should not be repeated:** Don't widen `P0` to "be safe" when tuning the
  estimator. The fix was the opposite: shrink it to `P0≈1e-4`-ish (matching what's
  actually known about the start state), which dropped max NIS over the full 60s
  run from ~600 to ~20-25 (consistent with `chi2(3)`'s own tail). If a future
  scenario legitimately starts from an uncertain initial state (e.g. after a
  fall-and-recover), that needs its own justified `P0`, not a blanket increase.
- **Source:** `docs/superpowers/plans/2026-09-20-sensors-estimator-nis-step3.md`,
  "Tuning notes."

---

### Failed: increasing process noise `Q` to raise mean NIS toward χ²(3)'s mean of 3
- **Approach tried / hypothesis:** The naive expectation that "more process noise
  → filter trusts the model less → higher NIS" — so if mean NIS runs low, raise `Q`.
- **What happened:** The opposite. In this nominal, no-real-model-mismatch
  scenario, extra `Q` inflates the filter's own predicted uncertainty `S` beyond
  what's actually needed, so the same-size innovations produce a *smaller*
  `ν'S⁻¹ν`. Larger `Q` makes mean NIS **lower**, not higher.
- **Why abandoned:** The intuition doesn't hold for a nominal (no disturbance) run
  — it may hold once real model mismatch is present (post `sim/disturbances.py`),
  but that's a different regime.
- **What should not be repeated:** Don't chase a target mean NIS by turning the
  `Q` knob in the "obvious" direction without first checking which regime (nominal
  vs. disturbed) you're actually testing. The needed `Q` here is small — just
  enough for numerical health and headroom for real future disturbances, not a
  "trust the model less" buffer.
- **Source:** same plan doc as above, "Tuning notes."

---

### Failed (as a target, not as an implementation): the literal i.i.d.-mean χ²(3N) acceptance interval for the NIS test
- **Approach tried:** Treating `CLAUDE.md` §11's "mean NIS within the 95% interval
  for χ²(3) averaged over the run" as literally meaning the tight interval for the
  mean of N=12000 i.i.d. χ²(3) samples, `[chi2(3600).ppf(0.025)/N, chi2(3600).ppf(0.975)/N] ≈ [2.956, 3.044]`.
- **What happened:** Running the identical closed-loop simulation with **zero**
  added process noise (the theoretically "most correct" nominal setting) across 4
  different seeds gave sample means of `2.875, 2.831, 2.671, 3.066` — a spread of
  ~0.4, roughly **10x wider** than the ±0.044 the i.i.d. formula predicts.
- **Why abandoned:** This confirms the i.i.d. assumption is wrong for this
  system — successive NIS samples are correlated through the KF's own smoothing
  and the near-deterministic encoder-quantization pattern, so "variance of the
  mean shrinks as 1/N" does not apply here. It is not an achievable target with
  any amount of Q/R tuning; it's a mismatched statistical model, not a bug.
- **What should not be repeated:** Don't tighten this test's acceptance band to
  the literal i.i.d. formula and then "fix" the resulting failures by tuning
  Q/R — the failures would be the test being wrong, not the filter. If the
  tighter guarantee is genuinely wanted, the correct fix is averaging over many
  independent 60s Monte Carlo runs to legitimately shrink the interval, which is
  a materially larger undertaking than adjusting one test's bounds.
- **Source:** same plan doc, "Design decision: the NIS test's acceptance interval..."

---

### Failed (as a target, not as an implementation): literal χ²(3N) quantiles for the gate's τ1/τ2
- **Approach tried:** `CLAUDE.md` §10 says gate thresholds should be "χ²(3N) quantiles."
  First attempt: pick 0.95/0.999 quantiles of `chi2(3N)` at `N=40`.
- **What happened:** A **nominal, undisturbed** 60s run immediately left NORMAL and
  reached HALT (max ε_k = 229.7 against τ2=173.6). Sweeping `N` up to 200/800/1200
  made it *worse*, not better: at every `N` tried, the empirical nominal-run maximum
  of ε_k exceeded even `chi2(3N)`'s **0.999999** quantile (e.g. at `N=200`:
  observed max 959.1 vs. quantile 779.3 — `chi2(600).cdf(959.1) ≈ 1.0` to
  floating-point precision).
- **Why abandoned:** Same root cause as the NIS-test i.i.d. failure above, but here
  it's provable, not just empirically inconvenient: no quantile level, however
  extreme, reproduces a usable threshold, because the closed loop's slow ~10s
  θ-position pole (no speed-servo integral term yet) makes the NIS sequence's mean
  wander on a timescale the windowed-sum statistic can't average out at any window
  length actually useful for sub-second disturbance detection.
- **What should not be repeated:** Don't try a "smarter" quantile level or a bigger
  `N` to rescue the literal chi2(3N) reading — it cannot work while the slow-pole
  issue exists. `GateParams.tau1`/`tau2` are calibrated empirically (with margin)
  against real 60s nominal runs instead; see `DECISIONS.md`. Fixing this properly
  would mean adding the speed-servo integral term from §9 first — a materially
  larger undertaking than this build step.
- **Source:** `docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md`,
  "Design decision: τ1/τ2 cannot be literal χ²(3N) quantiles."

---

### Failed: firing the battery-droop companion push at the droop's own onset
- **Approach tried:** In the battery-droop gate test, apply the companion push
  (needed because battery droop alone never binds the voltage ceiling near
  equilibrium) at `battery_droop_onset` — the same instant the voltage ramp begins.
- **What happened:** `sim.disturbances.battery_droop_v_batt` *ramps* V_batt linearly
  from nominal to drooped over `battery_droop_duration` rather than stepping it.
  At `t = onset` exactly, `frac = 0`, so V_batt is still ~7.4V (nominal) — the push
  landed on an essentially undrooped battery and never actually got voltage-limited.
  The test failed (never entered CAUTIOUS within the detection window) when the
  full suite was actually run, despite an earlier exploratory script (which used an
  instant step rather than a ramp) suggesting it would work.
- **Why abandoned:** The fix is to fire the companion push at
  `battery_droop_onset + battery_droop_duration` (once the ramp has actually
  finished) and measure the detection window from that same reference.
- **What should not be repeated:** Don't trust a pre-plan exploratory script's
  timing numbers without re-deriving them against the actual committed function's
  behavior (a step-function approximation used during exploration silently diverged
  from the linear-ramp function that got implemented) — and don't skip actually
  running the full test suite before committing, even when a plan document already
  states specific "verified" latency figures.
- **Source:** commit `2a5f0f0` message; `docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md`, Task 5's `test_enters_cautious_after_battery_droop`.
