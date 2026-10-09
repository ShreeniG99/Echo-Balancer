# Graph Report - Echo-Balancer  (2026-10-09)

## Corpus Check
- 73 files · ~130,850 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 15 file(s) not represented in the graph (top: .csv 9, (none) 3, .ini 1)

## Summary
- 909 nodes · 2607 edges · 52 communities (41 shown, 11 thin omitted)
- Extraction: 85% EXTRACTED · 15% INFERRED · 0% AMBIGUOUS · INFERRED: 400 edges (avg confidence: 0.94)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `ba69402e`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- run.py
- _closed_loop_with_gate
- GateParams
- rk4_step
- params.py
- EstimatorParams
- echo-balancer
- numpy
- Decisions — Echo Balancer
- Echo Balancer — Simulation Spec
- design_lqr_balance
- Sim
- pandas
- run_episode
- Failed Approaches — Echo Balancer
- Wokwi guide, from zero (browser only, nothing to install)
- short_window_eval.py
- default_plant_params
- echo_core.hpp
- VirtualRobot
- Kalman
- .claude/CLAUDE.md
- Current State — Echo Balancer
- disturbances.py
- test_gate.py
- Model Handoff — Echo Balancer
- main.cpp
- Echo Balancer — results (simulation v0, NXTway-GS parameters)
- What currently works (verified: 122/122 tests passing, `uv run pytest -q`)
- step_tilt_gate
- pytest
- ControllerType
- run_batch.py
- diagram.json
- NisGate
- host_scenarios.cpp
- Faults
- Open Problems — Echo Balancer
- test_recovers_from_10_degree_pitch_in_nonlinear_sim
- heldout_fallback.py
- Item 8 — robot parameters
- animate.py
- Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan
- battery_droop_v_batt
- Echo Balancer — Section 14 Step 5: World, Wall-Following Implementation Plan
- Pre-verified design: this step required designing two new control loops from scratch

## God Nodes (most connected - your core abstractions)
1. `run_episode()` - 86 edges
2. `default_plant_params()` - 62 edges
3. `_closed_loop_with_gate()` - 40 edges
4. `design_lqr_balance()` - 39 edges
5. `ControllerType` - 39 edges
6. `EpisodeConfig` - 35 edges
7. `design_lqr_speed_servo()` - 34 edges
8. `default_sensor_params()` - 33 edges
9. `rk4_step()` - 32 edges
10. `Decisions — Echo Balancer` - 31 edges

## Surprising Connections (you probably didn't know these)
- `QUBO formulation (`quantum/formulation.py`)` --references--> `verify_exact()`  [INFERRED]
  docs/RESULTS.md → quantum/formulation.py
- `What does not exist yet (build order steps 7-8, not started)` --references--> `front_threshold_speed_adjust()`  [INFERRED]
  project-context/CURRENT_STATE.md → sim/control.py
- `Current problems` --references--> `front_threshold_speed_adjust()`  [INFERRED]
  project-context/MODEL_HANDOFF.md → sim/control.py
- `Design decision: voltage clipping (`clip_voltage`, `V_batt`) lands in `sim/disturbances.py`, not `sim/plant.py`` --references--> `clip_voltage()`  [INFERRED]
  docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md → sim/disturbances.py
- `Decision: push disturbance is a ψ̇ velocity kick, not a torque impulse requiring an inertia conversion` --references--> `push_psi_dot_kick()`  [INFERRED]
  project-context/DECISIONS.md → sim/disturbances.py

## Import Cycles
- None detected.

## Communities (52 total, 11 thin omitted)

### Community 0 - "run.py"
Cohesion: 0.08
Nodes (33): arr(), main(), discretize(), initial_covariance(), KalmanState, measurement_matrix(), measurement_noise(), predict() (+25 more)

### Community 1 - "_closed_loop_with_gate"
Cohesion: 0.14
Nodes (19): Design decision 1: `run_episode` does NOT unify onto one shared control law — `design_lqr_balance` for balance-only episodes, `design_lqr_speed_servo` for wall-following episodes, _push(), Decision: `sim/world.py::pose_velocity` integrates at 1ms plant-substep resolution, not 5ms control-tick resolution, push_psi_dot_kick(), default_disturbance_params(), DisturbanceParams, _assert_enters_cautious_within(), _closed_loop_with_gate() (+11 more)

### Community 2 - "GateParams"
Cohesion: 0.22
Nodes (8): Echo Balancer — Section 14 Step 4: Gate, Disturbances Implementation Plan, Task 1: `sim/params.py` — `GateParams` and `DisturbanceParams`, Task 2: `sim/gate.py` — Normal/Cautious/Halt state machine, Task 3: `sim/disturbances.py` — the five disturbance profiles, Task 4: Section 11 gate test — no mode change on a nominal 60s run, Task 5: Section 11 gate tests — enters CAUTIOUS under each disturbance (Milestone 1), Design decision 3 (CONFIRMED finding, promoted from `OPEN_PROBLEMS.md`'s "suspected"): sustained forward motion with active yaw control significantly inflates epsilon, and interacts with `sim/sensors.py::encoder`'s per-wheel-quantization-before-averaging in a way `OPEN_PROBLEMS.md` had flagged as untested, GateParams

### Community 3 - "rk4_step"
Cohesion: 0.14
Nodes (6): rk4_step(), simulate(), test_rk4_step_is_fourth_order_accurate(), error_at(), xdot(), test_zero_order_hold_freezes_u_across_rk4_stages()

### Community 4 - "params.py"
Cohesion: 0.05
Nodes (41): main(), run(), 2026-10-09 — Follow-up round, default_corridor_params(), default_gate_params(), default_yaw_control_params(), hardware_v1_param_set(), param_set() (+33 more)

### Community 5 - "EstimatorParams"
Cohesion: 0.05
Nodes (39): Design decision: the NIS test's acceptance interval is chi2(3)'s own 95% interval, not the tighter "mean of 12000 i.i.d. samples" interval, Echo Balancer — Section 14 Step 3: Sensors, Estimator, NIS Test Implementation Plan, Kalman filter design (implemented in Task 3), Pre-verified design: sensor models, KF tuning, and the NIS test's acceptance bounds, Self-review notes, Sensor models (implemented in Task 2), Task 1: `sim/params.py` — sensor and estimator parameters, Task 2: `sim/sensors.py` — gyro, accelerometer, encoder (+31 more)

### Community 9 - "numpy"
Cohesion: 0.07
Nodes (35): decode(), _episode(), evaluation_set(), gate_params_for(), main(), score(), simulate_rows(), table_from_rows() (+27 more)

### Community 10 - "Decisions — Echo Balancer"
Cohesion: 0.09
Nodes (23): 2026-10-08 — Fallback ported to `sim/run.py` (default on), 2026-10-08 — Firmware fallback: softer gains in HALT + KF Q x100 outside NORMAL; target is ESP32-S3, 2026-10-08 — qiskit pinned: qiskit 2.5.2, qiskit-optimization 0.7.0, qiskit-aer 0.17.2, 2026-10-08 — QUBO formulation: exact HUBO -> Rosenberg QUBO, rank-transformed, verified by brute force, 2026-10-08 — Wokwi firmware lives in `firmware/` (user-approved scope addition), 2026-10-09 — Threshold-selection cost: spec J is official; J_detect is a labelled extra (user's choice "C"), Decision: battery-droop and payload-shift disturbance tests need a companion condition to be observable at all, Decision: corridor episodes default to `corridor_theta_dot_ref_nominal=0.3` rad/s, not `tests/test_wall_following.py`'s `2.0` (+15 more)

### Community 11 - "Echo Balancer — Simulation Spec"
Cohesion: 0.10
Nodes (20): 10. Gate, 11. Required tests (tests/), 12. Evaluation, 13. Quantum (offline threshold selection), 14. Build order and milestones, 15. Key references, 16. Persistent project knowledge (read before scanning the repo), 1. The claim (what the project is) (+12 more)

### Community 12 - "design_lqr_balance"
Cohesion: 0.15
Nodes (17): Self-review notes, Current implementation details worth knowing, Decision: `design_lqr_speed_servo` is a separate 5-state LQR, not a modification of `design_lqr_balance`, Decision: `sim/run.py` unifies the three closed-loop harnesses via an `EpisodeConfig.wall_following` flag, not a single shared control law, Failed: using `design_lqr_speed_servo` (instead of `design_lqr_balance`) as a single shared control law for every `sim/run.py` episode, Feeding a moving reference into the existing 4-state balance LQR for speed tracking, design_lqr_balance(), design_lqr_speed_servo() (+9 more)

### Community 13 - "Sim"
Cohesion: 0.13
Nodes (14): Sim, epsilon, fallen, faults, kf, nis, nis_gate, robot (+6 more)

### Community 14 - "pandas"
Cohesion: 0.13
Nodes (27): detection_delay(), fall_rate(), false_fallback_fraction(), missed_fallback(), progress(), summarize_batch(), fig_detection(), fig_episode() (+19 more)

### Community 15 - "run_episode"
Cohesion: 0.12
Nodes (17): Task 5: `sim/run.py` — wall-following (corridor) path, 2026-10-08 — Side-ultrasonic dropout rejection (`sim.sensors.hold_on_dropout`), Confirmed problems / gaps, EpisodeConfig, run_episode(), hold_on_dropout(), test_balance_only_nominal_60s_matches_existing_gate_calibration(), test_cautious_mode_scales_down_corridor_speed_reference() (+9 more)

### Community 16 - "Failed Approaches — Echo Balancer"
Cohesion: 0.25
Nodes (8): Discrete PD (nonzero `Kd`) for yaw-rate control, Failed Approaches — Echo Balancer, Failed (as a target, not as an implementation): literal χ²(3N) quantiles for the gate's τ1/τ2, Failed (as a target, not as an implementation): the literal i.i.d.-mean χ²(3N) acceptance interval for the NIS test, Failed (as a test design, not an implementation): assuming HALT mode makes `theta_dot` settle near zero within one `T_dwell` window, Failed: increasing process noise `Q` to raise mean NIS toward χ²(3)'s mean of 3, Failed: large "safely uncertain" initial Kalman filter covariance `P0`, Failed: `tests/test_wall_following.py`'s `theta_dot_ref_nominal=2.0` as the corridor evaluation speed

### Community 17 - "Wokwi guide, from zero (browser only, nothing to install)"
Cohesion: 0.12
Nodes (14): Echo Balancer on Wokwi (ESP32 NIS gate), Hardware mapping / caveats, Injecting disturbances (CLAUDE.md section 8), Not in the firmware (yet), Run it, Verification (host, `tests/test_firmware_core.py`), A. Create the project, B. Add the code files (+6 more)

### Community 19 - "default_plant_params"
Cohesion: 0.05
Nodes (51): Design decision: Q/R weights are parameters, not magic numbers, Design decision: the recovery test uses per-step feedback, not a 200 Hz ZOH loop, Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan, Pre-verified math, Task 1: Add scipy dependency, Task 2: `sim/linearize.py` — analytic + numeric-verified Jacobian, Task 3: `sim/params.py` — LQR balance weights, Task 4: `sim/control.py` — balance LQR design, pole test, nonlinear recovery test (+43 more)

### Community 20 - "echo_core.hpp"
Cohesion: 0.16
Nodes (8): Mode, CAUTIOUS, HALT, mode_name(), NORMAL, TiltGate, mode, time_in_mode

### Community 21 - "VirtualRobot"
Cohesion: 0.21
Nodes (6): clip_motor(), Rng, s, VirtualRobot, rng, x

### Community 22 - "Kalman"
Cohesion: 0.24
Nodes (5): Kalman, P, q_scale, x, main()

### Community 24 - "Current State — Echo Balancer"
Cohesion: 0.18
Nodes (11): 1. Controller comparison (CLAUDE.md section 12), 2026-10-08 — Build-order steps 7 and 8 complete, 2026-10-08 — Wokwi firmware (extra, off the §14 build order), Current configuration, Current objective, Current State — Echo Balancer, Immediate next steps, Uncommitted / unusual repo state as of this session (+3 more)

### Community 25 - "disturbances.py"
Cohesion: 0.14
Nodes (13): _accel_noise_fault(), _gyro_bias_fault(), _surface_change(), Decision: voltage clipping (`clip_voltage`, `V_batt`) lives in `sim/disturbances.py`, not `sim/plant.py`, clip_voltage(), sensor_fault_accel_noise_params(), sensor_fault_gyro_bias_step(), surface_change_plant_params() (+5 more)

### Community 26 - "test_gate.py"
Cohesion: 0.25
Nodes (17): Self-review notes, GateMode, GateState, initial_gate_state(), step_gate(), default_short_window_gate_params(), test_halt_to_cautious_exit(), test_initial_gate_state() (+9 more)

### Community 27 - "Model Handoff — Echo Balancer"
Cohesion: 0.25
Nodes (8): Constraints the next model must respect, Current problems, Current state in one paragraph, Exact config/parameters in force right now, Important files, Model Handoff — Echo Balancer, What the next model should do, What this project is (one paragraph)

### Community 28 - "main.cpp"
Cohesion: 0.31
Nodes (6): calibrate(), handleSerial(), loop(), readMpu(), setup(), showMode()

### Community 29 - "Echo Balancer — results (simulation v0, NXTway-GS parameters)"
Cohesion: 0.15
Nodes (12): 2. Threshold selection: exhaustive search vs Grover Adaptive Search (CLAUDE.md section 13), 3. Firmware (ESP32-S3, Wokwi), 4. Held-out checks (data never used for tuning), 5. Bugs found during this evaluation (fixed), 6. Limitations (read before citing any number), 7. Not done / open, Cost table (10 seeds), Echo Balancer — results (simulation v0, NXTway-GS parameters) (+4 more)

### Community 30 - "What currently works (verified: 122/122 tests passing, `uv run pytest -q`)"
Cohesion: 0.17
Nodes (15): Self-review notes, Task 6: Closed-loop wall-following demonstration, What currently works (verified: 122/122 tests passing, `uv run pytest -q`), Decision: yaw control is proportional-only, permanently — do not add `Kd`, Architecture you need to know, Decisions you must not silently re-litigate, front_threshold_speed_adjust(), wall_following_control() (+7 more)

### Community 31 - "step_tilt_gate"
Cohesion: 0.15
Nodes (15): Design decision 2: wall-following episodes feed `design_lqr_speed_servo` from the TRUE state, not the KF estimate — matching step 5's existing precedent, Design decision 4: `TiltGateParams` calibration — the tilt-threshold baseline genuinely misses 3 of 6 disturbance types, by design, not by miscalibration, Design decision 5: CAUTIOUS mode's "softer Q" speed-servo gain — verified stable, chosen over reducing `Q_psi` alone, Design decision 6: mode's effect on control only applies to wall-following episodes, Design decision 7: reproducing CLAUDE.md section 5's fall condition is not optional — corridor episodes can genuinely diverge without it, Pre-verified design: five load-bearing findings from running this repo's actual code before writing this plan, Task 3: `sim/gate.py` — tilt-threshold baseline gate (`TiltGateState`, `initial_tilt_gate_state`, `step_tilt_gate`), initial_tilt_gate_state() (+7 more)

### Community 34 - "pytest"
Cohesion: 0.19
Nodes (14): Decision: the corridor is two infinite parallel walls, no corners or dead ends, CorridorParams, pose_dotf(), cast_ray(), pose_velocity(), pose_dotf(), test_cast_ray_clips_to_range_bounds(), test_cast_ray_diagonal() (+6 more)

### Community 35 - "ControllerType"
Cohesion: 0.16
Nodes (10): Task 4: `sim/run.py` — `ControllerType`, `EpisodeConfig`, `run_episode` (balance-only path + fall condition), What the previous model (this session) was doing, ControllerType, _first_non_normal_latency(), test_corridor_push_disturbance_is_a_known_miss_for_the_nis_gate(), test_naive_controller_stays_normal_even_under_a_disturbance_that_would_trigger_the_gate(), apply_push(), test_nis_gate_detects_each_disturbance_within_window() (+2 more)

### Community 36 - "run_batch.py"
Cohesion: 0.27
Nodes (7): main(), run_batch(), _run_one(), Decision: `experiments/run_batch.py`'s workers receive scenario data as an explicit parameter, not via module-level lookup, test_run_batch_balance_only_smoke_test(), test_run_batch_steps_df_groups_correctly_per_episode(), test_run_batch_writes_readable_parquet()

### Community 38 - "diagram.json"
Cohesion: 0.29
Nodes (6): author, connections, dependencies, editor, parts, version

### Community 39 - "NisGate"
Cohesion: 0.29
Nodes (7): NisGate, count, epsilon, head, mode, time_in_mode, window

### Community 41 - "Faults"
Cohesion: 0.33
Nodes (6): Faults, accel_noise_mult, accel_offset, enc_offset, gyro_bias, payload_shifted

### Community 42 - "Open Problems — Echo Balancer"
Cohesion: 0.29
Nodes (7): 2026-10-08 — Confirmed limits after build-order steps 7-8, 2026-10-08 — Nominal corridor run can fall after a false alarm (confirmed once; mechanism suspected), 2026-10-08 — Python sim vs firmware fallback (confirmed divergence), 2026-10-09 — From the follow-up round, Open Problems — Echo Balancer, Suspected problems (plausible, not verified), Unknowns (genuinely open, not yet investigated)

### Community 44 - "heldout_fallback.py"
Cohesion: 0.23
Nodes (8): main(), _one(), _payload_fn(), fn(), _payload_shift(), payload_shift_plant_params(), test_payload_shift_persists_after_onset(), apply_disturbance()

### Community 46 - "Item 8 — robot parameters"
Cohesion: 0.18
Nodes (10): A. Tell me which parts you actually bought (5 minutes, no tools), B. Measure with a kitchen scale and a ruler (30 minutes), C. Motor constants (from the datasheet, or measure: about 1 hour, needs a multimeter), D. Two short sensor logs (about 20 minutes, using a small sketch I'll provide), Decisions I need from you, Hardware checklist: what to measure and send back, Hardware I'd suggest adding (cheap, optional), Item 8 — robot parameters (+2 more)

### Community 47 - "animate.py"
Cohesion: 0.29
Nodes (3): animate(), main(), simulate()

### Community 48 - "Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan"
Cohesion: 0.25
Nodes (7): Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan, Task 1: `pyproject.toml` — add `pandas`, `pyarrow`, `joblib`, Task 2: `sim/params.py` — `RunParams`, `TiltGateParams`, `default_speed_servo_params_cautious`, Task 6: `analysis/metrics.py`, Task 7: `experiments/run_batch.py`, Task 8: Update `project-context/`, RunParams

### Community 49 - "battery_droop_v_batt"
Cohesion: 0.28
Nodes (7): _battery_droop(), Failed: firing the battery-droop companion push at the droop's own onset, Failed approaches — do not repeat, battery_droop_v_batt(), test_battery_droop_ramps_linearly_then_holds(), test_enters_cautious_after_battery_droop(), apply_disturbance()

### Community 50 - "Echo Balancer — Section 14 Step 5: World, Wall-Following Implementation Plan"
Cohesion: 0.29
Nodes (6): Echo Balancer — Section 14 Step 5: World, Wall-Following Implementation Plan, Task 1: `sim/params.py` — `CorridorParams`, `SpeedServoParams`, `YawControlParams`, `WallFollowParams`, Task 2: `sim/world.py` — corridor geometry, ray casting, pose kinematics, Task 4: `sim/control.py` — speed servo LQR, Task 5: `sim/control.py` — yaw control, wall-following, front-threshold slowdown, WallFollowParams

### Community 51 - "Pre-verified design: this step required designing two new control loops from scratch"
Cohesion: 0.33
Nodes (6): Design decision: a real "speed servo" LQR is required — reference-tracking on the existing 4-state balance LQR is not accurate enough, Design decision: Task 6 integrates position via `pose_velocity`/`rk4_step` at plant resolution (1ms), not a hand-rolled formula at control resolution (5ms), Design decision: the corridor is two infinite parallel walls, no corners or dead ends, Design decision: yaw control is proportional-only, not PD — the plant's yaw dynamics are too fast for derivative action at 200Hz, Pre-verified design: this step required designing two new control loops from scratch, Wall-following closed-loop verification (with real ultrasonic noise/dropout)

## Knowledge Gaps
- **36 isolated node(s):** `NORMAL`, `CAUTIOUS`, `HALT`, `x`, `P` (+31 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 353 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **11 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `What currently works (verified: 122/122 tests passing, `uv run pytest -q`)` connect `What currently works (verified: 122/122 tests passing, `uv run pytest -q`)` to `pytest`, `rk4_step`, `GateParams`, `ControllerType`, `Decisions — Echo Balancer`, `design_lqr_balance`, `pandas`, `run_episode`, `Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan`, `default_plant_params`, `Current State — Echo Balancer`, `disturbances.py`, `test_gate.py`, `step_tilt_gate`?**
  _High betweenness centrality (0.090) - this node is a cross-community bridge._
- **Are the 11 inferred relationships involving `run_episode()` (e.g. with `Design decision 1: `run_episode` does NOT unify onto one shared control law — `design_lqr_balance` for balance-only episodes, `design_lqr_speed_servo` for wall-following episodes` and `Design decision 2: wall-following episodes feed `design_lqr_speed_servo` from the TRUE state, not the KF estimate — matching step 5's existing precedent`) actually correct?**
  _`run_episode()` has 11 INFERRED edges - model-reasoned connections that need verification._
- **What connects `NORMAL`, `CAUTIOUS`, `HALT` to the rest of the system?**
  _36 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `run.py` be split into smaller, more focused modules?**
  _Cohesion score 0.07869742198100407 - nodes in this community are weakly interconnected._
- **Why does `run_episode()` connect `run_episode` to `run.py`, `_closed_loop_with_gate`, `GateParams`, `rk4_step`, `params.py`, `numpy`, `Decisions — Echo Balancer`, `design_lqr_balance`, `pandas`, `short_window_eval.py`, `default_plant_params`, `disturbances.py`, `test_gate.py`, `Model Handoff — Echo Balancer`, `What currently works (verified: 122/122 tests passing, `uv run pytest -q`)`, `step_tilt_gate`, `pytest`, `ControllerType`, `run_batch.py`, `heldout_fallback.py`, `animate.py`, `Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan`?**
  _High betweenness centrality (0.086) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `default_plant_params()` (e.g. with `Task 3: `sim/params.py` — LQR balance weights` and `Design decision: the energy-conservation test needs the back-EMF term zeroed too`) actually correct?**
  _`default_plant_params()` has 4 INFERRED edges - model-reasoned connections that need verification._
- **Should `_closed_loop_with_gate` be split into smaller, more focused modules?**
  _Cohesion score 0.14492753623188406 - nodes in this community are weakly interconnected._