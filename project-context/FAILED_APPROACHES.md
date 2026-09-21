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
