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

### Instantaneous 0 -> full-speed step in `sim/run.py`'s theta_dot_ref
- **What was tried:** Setting `theta_dot_ref` directly to its target value
  (`theta_dot_ref_nominal=2.0` or the mode-derated value) every control tick, with
  no ramp — the same pattern `tests/test_wall_following.py` used successfully.
- **Why it failed:** An instantaneous 0->2 rad/s command at `t=0` produced a
  transient large enough to spuriously trip the NIS gate on an otherwise-nominal,
  undisturbed run (epsilon window sum reaching ~3500-4900 within the first second,
  vs. a calibrated `tau1` around 900-1300) — confirmed by a direct A/B: the exact
  same closed loop with `theta_dot_ref` held at 0 (stationary, matching step 3/4's
  original nominal-run calibration) produced max epsilon ~848, matching the
  expected calibration; only the abrupt step reintroduced the false trip.
- **Why it hadn't surfaced before:** `tests/test_wall_following.py`'s closed loop
  used true state for balance/speed feedback (a documented, temporary
  simplification), so it had no NIS/gate to trip. This was a genuinely new
  interaction, only possible once `sim/run.py` combined the KF-driven control
  loop with wall-following's speed reference for the first time.
- **What should not be repeated:** Don't step a speed reference instantly to its
  target inside a KF/NIS-gated control loop, even if a step worked fine in a
  simpler (non-gated, or true-state-fed) test harness. Use a bounded-acceleration
  ramp (`SpeedProfileParams.accel_limit`) instead.
- **Source:** `sim/run.py`'s development, this session (build-order step 6).

---

### `prev_wall_err` seeded at 0.0 in `sim/run.py` (classic PID cold-start derivative kick)
- **What was tried:** Initializing `wall_following_control`'s `prev_distance_error`
  state to `0.0` before the first ultrasonic sample, matching
  `tests/test_wall_following.py`'s own initialization.
- **Why it failed:** The very first real distance-error sample (the robot starts
  0.2m off-target) is compared against that `0.0` seed, so the PD controller's
  derivative term computes `(real_error - 0.0)/dt_ultrasonic` — the *entire*
  initial offset divided by one 50ms tick — producing a `phi_dot_ref` spike of
  several rad/s out of nothing, which (via `sim.plant.f`'s
  `phi_dot^2*sin(psi)*cos(psi)` yaw->pitch coupling term) perturbed pitch enough
  to spuriously trip the NIS gate.
- **Why seeding it from the true initial distance (not just "any nonzero value")
  mattered:** A first attempt seeded `prev_wall_err` from `corridor_width - y0`
  (the distance to the *far* wall) instead of `y0` itself (the distance to the
  *near*/right wall `cast_ray` at `heading-pi/2` actually measures) — this made
  the spike *worse* (phi_dot_ref jumped to 4.47 rad/s instead of 2.47), because
  the seed was for the wrong wall entirely. Fixed by seeding from
  `y0` (the robot's actual initial distance to the wall the controller measures).
- **What should not be repeated:** Don't assume a PD controller's `prev_error`
  seed value is a minor detail — for the very first sample after a large known
  initial offset, seed it from that offset (zero real derivative on sample one),
  not from `0.0` and not from a guess about which wall/quantity is being measured
  without re-deriving `cast_ray`'s actual geometry.
- **Source:** `sim/run.py`'s development, this session (build-order step 6).

---

### Ultrasonic dropout readings fed directly into `wall_following_control`
- **What was tried:** Feeding every ultrasonic sample (including HC-SR04-model
  dropouts, which return exactly `sensor_p.ultrasonic_range_max`, per CLAUDE.md
  §7's 2% dropout probability) straight into the wall-following PD controller,
  same as any other reading.
- **Why it failed:** A dropout is a single-sample jump from ~0.3-0.5m (typical
  in-corridor reading) to 4.0m (max range) — the derivative term reacts to this
  implausible jump with a multi-rad/s `phi_dot_ref` command (observed: 35.6
  rad/s), which again coupled into pitch via `sim.plant.f`'s yaw->pitch term and
  spuriously tripped the NIS gate on an otherwise-nominal run (epsilon spiking
  into the thousands within two ticks).
- **What should not be repeated:** Don't assume a sensor model's own documented
  fault behavior (dropout returning max range) is "just noise" a PD controller's
  derivative term can absorb — a dropout is identifiable (`reading ==
  sensor_p.ultrasonic_range_max` exactly, since the model adds no noise on a
  dropout) and should be filtered/held, not acted on as data. Fixed in
  `sim/run.py` by holding the previous `phi_dot_ref`/`prev_wall_err` on a
  detected dropout tick rather than computing a new control update from it.
- **Source:** `sim/run.py`'s development, this session (build-order step 6).

---

### Assuming a single `GroverOptimizer.solve()` call is authoritative
- **What was tried:** Calling `GroverOptimizer(...).solve(qp)` once and treating
  its `result.x`/`result.fval` as "the answer," the same way a classical
  exhaustive search's single result would be.
- **Why it failed:** On both a small synthetic 6-bit QUBO (with a well-separated
  optimum, gap 0.5 out of an 8-unit range) and this project's real 64-candidate
  surrogate, a single `.solve()` call missed the true optimum roughly 4 times out
  of 5 (synthetic case) to every time (production case, 0/5 and 0/30 across two
  separate production runs) at `num_value_qubits=10`, `num_iterations=8`. Raising
  `num_value_qubits` to 12-14 made single trials take up to minutes each (Aer's
  classical simulation cost grows with the value register's qubit count) without
  reliably fixing it, because the production surrogate's real bottleneck (see
  `OPEN_PROBLEMS.md`) is a razor-thin margin (~0.12% of the objective's range)
  between the best candidate and its closest rival, not just insufficient
  iterations.
- **What should not be repeated:** Don't report or act on a single Grover run as
  ground truth. Run several independent trials and keep the best, and report the
  hit rate honestly (see `DECISIONS.md`) — this is expected probabilistic
  behavior of Grover-style search, not a configuration bug to keep chasing.
- **Source:** `quantum/qubo.py`'s development, this session (build-order step 7).

---

### Passing string labels directly to `ax.bar(labels, values, ...)` with a NaN value present
- **What was tried:** `analysis.plots.plot_controller_comparison` called
  `ax.bar(labels, values.to_numpy(), color=colors)` with `labels` a list of 3
  controller name strings, where `values` included a `NaN` (Naive's
  `mean_detection_delay_s` is undefined -- it never leaves NORMAL, so there's
  nothing to time).
- **Why it failed:** The category whose bar height was `NaN` ("Naive")
  disappeared from the *rendered/saved* figure entirely -- no tick, no label,
  just blank space where its bar should have been. This was NOT reproducible
  by inspecting `ax.get_xticklabels()` immediately after the `ax.bar()` call
  (all 3 labels were present there); it only showed up in the actual saved
  PNG, confirmed by cropping and re-examining the output image directly.
- **What should not be repeated:** Don't rely on matplotlib's implicit
  categorical-axis tick generation (`ax.bar(string_labels, values)`) when any
  value can legitimately be `NaN` -- verify by rendering, not just by
  inspecting the Axes object's tick state right after the plotting call
  (deferred layout/draw logic can still change what's visible in the final
  figure). Fixed by using explicit numeric x-positions
  (`ax.bar(np.arange(n), values)`) plus explicit `ax.set_xticks`/
  `set_xticklabels`, which is unaffected by any bar's height.
- **Source:** `analysis/plots.py`'s development, this session (build-order
  step 8), caught by visually inspecting the rendered PNG before writing
  tests -- codified as
  `tests/test_plots.py::test_plot_controller_comparison_keeps_all_ticks_despite_nan`.

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
