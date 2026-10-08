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
- **Sustained nonzero `theta_dot` (forward motion) with active yaw correction significantly inflates the windowed NIS statistic epsilon_k, confirming the encoder-per-wheel-quantization-under-yaw suspicion this file previously flagged as untested.** A nominal (undisturbed) 60s wall-following run at `theta_dot_ref=2.0` rad/s reaches `max_epsilon=1605.56`, exceeding `tau1=1300` — a false-fallback from motion alone. Straight-line-only motion (no yaw) at the same speed only reaches `max_epsilon≈2465` at `theta_dot_ref=2.0` too, but yaw correction makes lower speeds unsafe as well (e.g. `0.5` rad/s: straight-line-only was comfortably safe in isolation, but with yaw active the safe ceiling drops well below that). `sim/run.py`'s corridor episodes use `theta_dot_ref_nominal=0.3` (verified: `max_epsilon=909.13`) specifically because of this. Root cause not fully characterized (plausible: `EstimatorParams.Q_theta=1e-8` is far too small to absorb the apparent process noise from per-wheel encoder quantization sweeping through many bins per second at speed) — fixing this (e.g. a `theta_dot`-dependent `Q_theta`, or an average-then-quantize encoder redesign) is out of scope for step 6.
- **A very low nonzero corridor speed (`theta_dot_ref_nominal=0.1` rad/s) destabilizes the closed loop entirely** (observed epsilon reaching `~2.1e8` without a fall-check present, almost certainly an undetected fall) — the relationship between corridor speed and stability/false-fallback is not monotonic. Not investigated further; `sim/run.py`'s fall condition (CLAUDE.md section 5) exists specifically so this terminates cleanly (`fallen=True`) instead of producing meaningless downstream numbers.
- **Push-type disturbances (psi_dot velocity kicks) are not reliably detected by the NIS gate once wall-following/corridor mode is active**: max epsilon in the push-response window reaches only ~1085.51 (under `tau1=1300`) for the shipped corridor configuration (`design_lqr_speed_servo`, true-state-fed, `theta_dot_ref_nominal=0.3`, seed=42 — verified in `tests/test_run.py::test_corridor_push_disturbance_is_a_known_miss_for_the_nis_gate`). This is a single verified data point, not a sweep across corridor architecture variants — no such sweep was run. This is inherited from an already-thin margin in the balance-only architecture (`design_lqr_balance`'s own push margin is only 3.5% over `tau1`, per the plan doc's "Design decision 1" — see `docs/superpowers/plans/2026-09-22-run-batch-metrics-step6.md`), not a new step-6 regression, but it means `experiments/run_batch.py`'s per-controller metrics table will likely show a real, reportable "missed" or delayed detection for push-type disturbances specifically for the NIS-gate controller — expected, not a bug to chase.
- **HALT mode's transient behavior does not match its steady-state description.** When HALT is triggered mid-recovery from a large initial disturbance, `theta_dot` overshoots well beyond the NORMAL-mode cruise speed before settling — because `TiltGateParams.T_dwell=0.5s` is much shorter than the pitch-recovery pole's time constant (~2.5-4s). See `FAILED_APPROACHES.md`'s "assuming HALT mode makes theta_dot settle near zero" entry for the full empirical validation (including the bug-injection check that ruled out a simpler "near zero" bound as non-discriminating).
- **A real end-to-end `run_batch(wall_following=True, scenario_names=["nominal"])` run (the full 10-seed default) shows a nontrivial fall rate on the corridor's own nominal (undisturbed) episodes, independent of gate choice.** Fall rate by controller: `naive` 0.2 (falls at seeds 3, 5), `nis_gate` 0.2 (falls at the *same* seeds 3, 5), `tilt_threshold` 0.5 (falls at seeds 2, 4, 5, 7, 8). This was not caught by any of step 6's own tests: they never ran the real 10-seed batch end-to-end, and Task 5's calibration test
  (`test_corridor_nominal_60s_stays_normal_at_the_calibrated_safe_speed`) only checked `seed=42` — one of the stable seeds — never fall rate across a wider seed population. Diagnosis so far (already run, not further investigated):
  - For `naive`/`nis_gate`, seeds 3 and 5 fall at nearly identical times regardless of controller (seed 3: `naive` at t=58.130s, `nis_gate` at t=58.200s; seed 5: `naive` at t=17.965s, `nis_gate` at t=17.955s) — strong evidence of a plant/sensor-noise-driven instability inherent to this (seed, `theta_dot_ref_nominal=0.3`) combination, independent of which gate/controller is used. The other 8 seeds (0,1,2,4,6,7,8,9) stay well-behaved for the full 60s with `naive` (max|psi| under ~4 degrees throughout, verified).
  - `tilt_threshold` falls on a different seed set (2, 4, 5, 7, 8 — not seed 3, unlike `naive`/`nis_gate`) — and at every one of those fall seeds its `mode` column is verified to actually transition through `{NORMAL, CAUTIOUS, HALT}` before falling, on ordinary sensor noise crossing its tilt thresholds, not any injected disturbance. This is consistent with (and reinforces) the "HALT mode's transient behavior does not match its steady-state description" entry directly above — a mode-switch reference-step disturbance landing during active recovery dynamics — and suggests the tilt-threshold gate's own mode-switching is itself destabilizing some otherwise-stable seeds.
  - **This is a confirmed gap in, not a contradiction of, "Design decision 3"** (`DECISIONS.md`'s "corridor episodes default to `corridor_theta_dot_ref_nominal=0.3`" entry): that decision's "best-margined safe value" claim was validated only against the gate-triggering criterion (epsilon staying under `tau1`) at `seed=42`, never against actual fall rate across a realistic seed population. The epsilon-based claim is still correct on its own terms; it just isn't sufficient evidence for "safe" in the fall-rate sense. See `DECISIONS.md` for the added caveat.

## Suspected problems (plausible, not verified)

- **Whether encoder quantize-then-average vs. average-then-quantize (specifically the per-wheel encoder quantization mechanism, as opposed to some other source of process noise) is the actual causal mechanism behind the sustained-motion false-fallback finding.** `tests/test_sensors.py::test_encoder_quantizes_per_wheel_before_averaging` already confirmed the per-wheel reading behaves as intended in a hand-constructed unit case, and step 6's `sim/run.py` corridor episodes are the first *real closed-loop* exercise of this under active yaw control — but encoder quantization is still only the leading suspected cause of the sustained-motion false-fallback finding above (see "Confirmed problems"), not a verified one; no isolation test has ruled out other process-noise sources.
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

## 2026-10-08 — Python sim vs firmware fallback (confirmed divergence)
Confirmed: `sim/run.py` HALT uses nominal K5 and no KF Q inflation; firmware uses cautious K5 + Q x100. Suspected (not tested): the Python nonlinear sim would show the same payload-shift benefit; porting the fallback to `sim/run.py` is needed before any paper claim that spans both. Also confirmed: 0.1 rad/s push is never detected in the firmware (0/10 seeds).

## 2026-10-08 — Nominal corridor run can fall after a false alarm (confirmed once; mechanism suspected)
Confirmed: gate candidate tau1=1200, tau2=1800, N=200, T_dwell=0.25 (cost-table index 21), corridor nominal, seed 2: false CAUTIOUS at 1.31 s (eps ~1290 > 1200), back to NORMAL, then at ~25 s the right ultrasonic saturates at 4.0 m (max range) for ~1 s, psi diverges and the robot falls at 25.66 s. Identical with fallback on or off. The default gate (index 54) does not fall on that seed.
Confirmed mechanism (traced phi/v_l/v_r): heading drifts to -41 deg by 22 s (turning toward the y=0 wall), reaches -90 deg at 24.6 s, the right ray is then parallel to the walls and reads max range (4.0 m), the wall-following error saturates the differential drive at v_l=+7.4, v_r=-7.4 V, leaving no voltage for balance -> psi diverges -> fall at 25.66 s. Root cause is in wall-following, not the gate: no handling of max-range/dropout readings and no cap on yaw authority relative to balance. RESOLVED same day: the heading drift itself came from ultrasonic dropouts (max-range readings) hitting the wall-following D-term; fixed with sim.sensors.hold_on_dropout (see DECISIONS.md). Not done: a cap on yaw authority relative to balance (diff_v can still saturate if a real large wall error occurs). Found by the QUBO cost table; the first cost-table version did not count falls in nominal episodes (fixed: fall_rate now spans the whole evaluation set).
