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

---

### Feeding a moving reference into the existing 4-state balance LQR for speed tracking
- **What was tried:** Instead of designing a new integral-augmented LQR, reuse
  the approved `design_lqr_balance` and just feed it a moving reference:
  `u = -K·[θ-θ_ref(t), ψ, θ̇-θ̇_ref, ψ̇]` with `θ_ref(t)` ramping at the desired
  speed.
- **Why it failed:** 42% steady-state speed error (`θ̇_ref=1.0 rad/s` settles at
  `θ̇≈1.42`; `θ̇_ref=5.0` settles at `≈7.10`) — a textbook type-0-system
  steady-state error, since pure state feedback has no integral action to
  drive a ramp-reference error to zero.
- **What should not be repeated:** Don't try to retrofit reference-tracking
  onto a non-integral LQR by feeding it a moving setpoint — build a genuinely
  integral-augmented design instead (see [DECISIONS.md](DECISIONS.md),
  `design_lqr_speed_servo`).
- **Source:** `docs/superpowers/plans/2026-09-21-world-wallfollowing-step5.md`,
  "Design decision: a real 'speed servo' LQR is required."

---

### Discrete PD (nonzero `Kd`) for yaw-rate control
- **What was tried:** `u = Kp·(φ̇_ref-φ̇) + Kd·d(error)/dt` at `dt=5ms` (200Hz),
  for several `Kd` values from 0.001 up to 0.05.
- **Why it failed:** Any nonzero `Kd` destabilizes the loop almost immediately
  — `Kd=0.01` borderline, `Kd=0.02` diverges to `~1e36` within 3 seconds,
  `Kd=0.05` diverges outright. Root cause: the plant's yaw pole (~-95.6 rad/s,
  ~10ms time constant) is too fast relative to the 5ms sample period for
  derivative action to stay stable at any usable gain — a genuine discrete-time
  sampling problem, not an undertuned gain.
- **What should not be repeated:** Don't re-add a `Kd` field to
  `YawControlParams` expecting careful tuning to find a stable value — none
  exists at this control rate. A faster control rate or a restructured
  (state-feedback) yaw loop would be required first.
- **Source:** `docs/superpowers/plans/2026-09-21-world-wallfollowing-step5.md`,
  "Design decision: yaw control is proportional-only."

---

### Failed: using `design_lqr_speed_servo` (instead of `design_lqr_balance`) as a single shared control law for every `sim/run.py` episode
- **Approach tried:** Feed the KF estimate (or, separately, the true state) into `design_lqr_speed_servo` with `theta_dot_ref=0` for balance-in-place episodes, on the reasoning that a constant zero reference reduces exactly to the already-verified nominal design.
- **What happened:** Nominal-run epsilon behavior was fine in every variant (comparable to the existing `design_lqr_balance` range), but push-disturbance detection broke in *all four* tested variants (KF-fed/true-state-fed x `theta_dot_ref` 0/nonzero): max epsilon in the push-response window ranged 1111-1187, all below `tau1=1300`, versus `design_lqr_balance`'s own (already-thin) 1344.87.
- **Why abandoned:** `GateParams.tau1`/`tau2` were calibrated specifically against `design_lqr_balance`'s closed-loop response; the speed-servo gain's slightly faster damping of the psi_dot-kick response is enough to drop the windowed-sum epsilon peak below `tau1`, and there was no clean way to keep both control-law unification and the existing calibration.
- **What should not be repeated:** Don't unify `sim/run.py`'s control law onto `design_lqr_speed_servo` "for cleanliness" without re-verifying push/battery-droop detection against the new gain — the margin is thin enough that even a modest gain change tips it.
- **Source:** `docs/superpowers/plans/2026-09-22-run-batch-metrics-step6.md`, "Design decision 1."

---

### Failed: `tests/test_wall_following.py`'s `theta_dot_ref_nominal=2.0` as the corridor evaluation speed
- **Approach tried:** Reuse the existing wall-following demo's forward speed (`2.0` rad/s) for `sim/run.py`'s corridor episodes, now with the KF/gate actually observing the loop.
- **What happened:** A nominal (undisturbed) 60s run at `2.0` rad/s reaches `max_epsilon=1605.56`, exceeding `tau1=1300` and spuriously leaving NORMAL — a false-fallback purely from sustained forward motion plus active yaw correction, not any injected disturbance.
- **Why abandoned:** `2.0` rad/s was originally chosen in step 5 purely for a fast kinematic demo with no KF/gate in that loop at all; it was never validated against the gate's calibration. `0.3` rad/s (swept alongside `0.1/0.2/0.5`) keeps `max_epsilon` at `909.13`, comparable to the balance-only nominal range.
- **What should not be repeated:** Don't assume a control/kinematics-only demo's chosen speed transfers to a configuration that also includes the estimator/gate — sustained nonzero `theta_dot` was never exercised against `GateParams` before this step.
- **Source:** same plan doc, "Design decision 3."

---

### Failed (as a test design, not an implementation): assuming HALT mode makes `theta_dot` settle near zero within one `T_dwell` window
- **Approach tried:** Assert `theta_dot` stays near zero while `mode == "HALT"` in a wall-following episode, on the intuitive reading of CLAUDE.md section 10's "keep balancing in place."
- **What happened:** When HALT is triggered by a large initial-tilt recovery (e.g. `x0_psi_deg=15`), `theta_dot` does NOT settle near zero during the HALT window — it overshoots to roughly ±0.85 rad/s, *larger* in magnitude than the 0.3 rad/s NORMAL cruise speed. Root cause: `TiltGateParams.T_dwell=0.5s` is far shorter than the pitch-recovery pole's time constant (~2.5-4s), so HALT engages mid-recovery-transient, and freezing `theta_dot_ref=0` (a reference step) is itself a disturbance the LQR must react to. Verified this wasn't a red herring by deliberately reintroducing two plausible bugs (`theta_dot_ref=base_ref` instead of `0.0`; `K5=K5_cautious` instead of `K5_nominal`) — both produced *smaller* peak `|theta_dot|` (~0.30, ~0.56) than the correct code (~0.85), confirming a naive "near zero" bound would be non-discriminating (would reject correct code while being agnostic to at least these two specific bugs).
- **Why abandoned:** The near-zero assertion doesn't hold and isn't even the right thing to check; used instead: HALT is reached, no fall, no non-finite state, `theta_dot` stays boundedly finite, and mode eventually returns to NORMAL (evidence of stability, not the specific magnitude of the transient).
- **What should not be repeated:** Don't assume a gate mode's *steady-state* semantic description ("keep balancing in place") describes its behavior during the *transient* right after a mode transition, especially when the dwell time is short relative to the plant's own recovery time constants. If `T_dwell` or the pitch-recovery pole ever change, re-verify this finding — it's about their relative timescales, not a fixed property of either alone.
- **Source:** `docs/superpowers/plans/2026-09-22-run-batch-metrics-step6.md`, Task 5's code review discussion (`tests/test_run.py::test_halt_mode_zeroes_corridor_speed_reference`).
