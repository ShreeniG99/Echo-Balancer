# Graph Report - Echo-Balancer  (2026-10-09)

## Corpus Check
- 73 files · ~130,540 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 15 file(s) not represented in the graph (top: .csv 9, (none) 3, .ini 1)

## Summary
- 915 nodes · 2623 edges · 51 communities (42 shown, 9 thin omitted)
- Extraction: 85% EXTRACTED · 15% INFERRED · 0% AMBIGUOUS · INFERRED: 403 edges (avg confidence: 0.94)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `8df8a00a`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- run.py
- _closed_loop_with_gate
- Echo Balancer — Section 14 Step 4: Gate, Disturbances Implementation Plan
- rk4_step
- params.py
- Decisions — Echo Balancer
- echo-balancer
- test_qubo.py
- test_firmware_core.py
- Echo Balancer — Simulation Spec
- design_lqr_balance
- Sim
- pandas
- run_episode
- Failed Approaches — Echo Balancer
- Wokwi guide, from zero (browser only, nothing to install)
- Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan
- default_sensor_params
- echo_core.hpp
- VirtualRobot
- Kalman
- .claude/CLAUDE.md
- Current State — Echo Balancer
- run_batch.py
- test_gate.py
- Model Handoff — Echo Balancer
- main.cpp
- Echo Balancer — results (simulation v0, NXTway-GS parameters)
- numpy
- What currently works (verified: 122/122 tests passing, `uv run pytest -q`)
- pytest
- test_tilt_threshold_controller_detection
- run_batch
- diagram.json
- NisGate
- host_scenarios.cpp
- Faults
- Echo Balancer on Wokwi (ESP32 NIS gate)
- hold_on_dropout
- default_plant_params
- Item 8 — robot parameters
- ControllerType
- Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan
- push_psi_dot_kick
- 6. Parameters

## God Nodes (most connected - your core abstractions)
1. `run_episode()` - 87 edges
2. `default_plant_params()` - 62 edges
3. `_closed_loop_with_gate()` - 40 edges
4. `design_lqr_balance()` - 39 edges
5. `ControllerType` - 39 edges
6. `EpisodeConfig` - 36 edges
7. `design_lqr_speed_servo()` - 34 edges
8. `default_gate_params()` - 34 edges
9. `default_sensor_params()` - 33 edges
10. `Decisions — Echo Balancer` - 33 edges

## Surprising Connections (you probably didn't know these)
- `QUBO formulation (`quantum/formulation.py`)` --references--> `verify_exact()`  [INFERRED]
  docs/RESULTS.md → quantum/formulation.py
- `What does not exist yet (build order steps 7-8, not started)` --references--> `front_threshold_speed_adjust()`  [INFERRED]
  project-context/CURRENT_STATE.md → sim/control.py
- `Current problems` --references--> `front_threshold_speed_adjust()`  [INFERRED]
  project-context/MODEL_HANDOFF.md → sim/control.py
- `Design decision: voltage clipping (`clip_voltage`, `V_batt`) lands in `sim/disturbances.py`, not `sim/plant.py`` --references--> `clip_voltage()`  [INFERRED]
  docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md → sim/disturbances.py
- `Design decision: the NORMAL→HALT "skip" exception is read as "window not yet full", not "sustained over a separate timer"` --references--> `step_gate()`  [INFERRED]
  docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md → sim/gate.py

## Import Cycles
- None detected.

## Communities (51 total, 9 thin omitted)

### Community 0 - "run.py"
Cohesion: 0.13
Nodes (20): arr(), main(), discretize(), initial_covariance(), KalmanState, measurement_matrix(), measurement_noise(), predict() (+12 more)

### Community 1 - "_closed_loop_with_gate"
Cohesion: 0.28
Nodes (11): Design decision 1: `run_episode` does NOT unify onto one shared control law — `design_lqr_balance` for balance-only episodes, `design_lqr_speed_servo` for wall-following episodes, default_disturbance_params(), _assert_enters_cautious_within(), _closed_loop_with_gate(), test_enters_cautious_after_accel_noise_fault(), test_enters_cautious_after_gyro_bias_fault(), test_enters_cautious_after_payload_shift(), test_enters_cautious_after_push() (+3 more)

### Community 2 - "Echo Balancer — Section 14 Step 4: Gate, Disturbances Implementation Plan"
Cohesion: 0.22
Nodes (7): Echo Balancer — Section 14 Step 4: Gate, Disturbances Implementation Plan, Task 1: `sim/params.py` — `GateParams` and `DisturbanceParams`, Task 2: `sim/gate.py` — Normal/Cautious/Halt state machine, Task 3: `sim/disturbances.py` — the five disturbance profiles, Task 5: Section 11 gate tests — enters CAUTIOUS under each disturbance (Milestone 1), Decision: `sim/world.py::pose_velocity` integrates at 1ms plant-substep resolution, not 5ms control-tick resolution, DisturbanceParams

### Community 3 - "rk4_step"
Cohesion: 0.09
Nodes (16): Echo Balancer — Section 14 Step 1: Plant Model Implementation Plan, Open item to flag before/while executing, Self-review notes, Task 1: uv project scaffolding, Task 2: `sim/params.py` — plant parameters, Task 3: `sim/plant.py` — nonlinear dynamics, Task 4: `sim/integrate.py` — fixed-step RK4, Design decision: the NIS test's acceptance interval is chi2(3)'s own 95% interval, not the tighter "mean of 12000 i.i.d. samples" interval (+8 more)

### Community 4 - "params.py"
Cohesion: 0.05
Nodes (44): Design decision 5: CAUTIOUS mode's "softer Q" speed-servo gain — verified stable, chosen over reducing `Q_psi` alone, 2026-10-09 — Follow-up round, Decision: `default_speed_servo_params_cautious()` divides every `SpeedServoParams` Q weight by 5, Decision: HALT mode uses the nominal (not cautious) speed-servo gain, default_corridor_params(), default_run_params(), default_speed_servo_params_cautious(), default_tilt_gate_params() (+36 more)

### Community 5 - "Decisions — Echo Balancer"
Cohesion: 0.05
Nodes (40): Design decision: push is a ψ̇ velocity kick, not a torque impulse requiring an inertia conversion, Design decision: the NORMAL→HALT "skip" exception is read as "window not yet full", not "sustained over a separate timer", Design decision: voltage clipping (`clip_voltage`, `V_batt`) lands in `sim/disturbances.py`, not `sim/plant.py`, Design decision: τ1/τ2 cannot be literal χ²(3N) quantiles, at any quantile level, Disturbance magnitudes — five profiles, each independently verified, two with genuine surprises, Pre-verified design: this took by far the most empirical exploration of any step so far, Architecture, Constraints (+32 more)

### Community 9 - "test_qubo.py"
Cohesion: 0.07
Nodes (35): decode(), _episode(), evaluation_set(), gate_params_for(), main(), score(), simulate_rows(), table_from_rows() (+27 more)

### Community 10 - "test_firmware_core.py"
Cohesion: 0.16
Nodes (8): binary(), _scenario(), scenarios_bin(), test_gate_mode_changes_the_control(), test_generated_header_is_current(), test_nominal_60s_no_mode_change(), test_payload_shift_is_detected_and_survived(), test_sensor_faults_enter_non_normal_within_window()

### Community 11 - "Echo Balancer — Simulation Spec"
Cohesion: 0.12
Nodes (16): 11. Required tests (tests/), 12. Evaluation, 13. Quantum (offline threshold selection), 14. Build order and milestones, 15. Key references, 16. Persistent project knowledge (read before scanning the repo), 1. The claim (what the project is), 2. Scope (+8 more)

### Community 12 - "design_lqr_balance"
Cohesion: 0.12
Nodes (17): Self-review notes, Design decision: a real "speed servo" LQR is required — reference-tracking on the existing 4-state balance LQR is not accurate enough, Task 4: `sim/control.py` — speed servo LQR, Decision: `design_lqr_speed_servo` is a separate 5-state LQR, not a modification of `design_lqr_balance`, Failed: using `design_lqr_speed_servo` (instead of `design_lqr_balance`) as a single shared control law for every `sim/run.py` episode, Feeding a moving reference into the existing 4-state balance LQR for speed tracking, design_lqr_balance(), design_lqr_speed_servo() (+9 more)

### Community 13 - "Sim"
Cohesion: 0.13
Nodes (14): Sim, epsilon, fallen, faults, kf, nis, nis_gate, robot (+6 more)

### Community 14 - "pandas"
Cohesion: 0.10
Nodes (29): detection_delay(), fall_rate(), false_fallback_fraction(), missed_fallback(), progress(), summarize_batch(), main(), run() (+21 more)

### Community 15 - "run_episode"
Cohesion: 0.13
Nodes (18): Task 5: `sim/run.py` — wall-following (corridor) path, EpisodeConfig, run_episode(), _first_non_normal_latency(), test_balance_only_nominal_60s_matches_existing_gate_calibration(), test_corridor_episode_makes_forward_progress_and_stays_in_corridor(), test_corridor_push_missed_by_long_window_only_caught_by_short_window(), latency() (+10 more)

### Community 16 - "Failed Approaches — Echo Balancer"
Cohesion: 0.25
Nodes (8): Discrete PD (nonzero `Kd`) for yaw-rate control, Failed Approaches — Echo Balancer, Failed (as a target, not as an implementation): literal χ²(3N) quantiles for the gate's τ1/τ2, Failed (as a target, not as an implementation): the literal i.i.d.-mean χ²(3N) acceptance interval for the NIS test, Failed (as a test design, not an implementation): assuming HALT mode makes `theta_dot` settle near zero within one `T_dwell` window, Failed: increasing process noise `Q` to raise mean NIS toward χ²(3)'s mean of 3, Failed: large "safely uncertain" initial Kalman filter covariance `P0`, Failed: `tests/test_wall_following.py`'s `theta_dot_ref_nominal=2.0` as the corridor evaluation speed

### Community 17 - "Wokwi guide, from zero (browser only, nothing to install)"
Cohesion: 0.25
Nodes (8): A. Create the project, B. Add the code files, C. Run, D. Try the faults (this is the demo), E. Save evidence for the paper, F. Optional: run it locally (VS Code), If the S3 board part errors, Wokwi guide, from zero (browser only, nothing to install)

### Community 18 - "Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan"
Cohesion: 0.25
Nodes (7): Design decision: Q/R weights are parameters, not magic numbers, Design decision: the recovery test uses per-step feedback, not a 200 Hz ZOH loop, Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan, Pre-verified math, Task 1: Add scipy dependency, Task 2: `sim/linearize.py` — analytic + numeric-verified Jacobian, Task 4: `sim/control.py` — balance LQR design, pole test, nonlinear recovery test

### Community 19 - "default_sensor_params"
Cohesion: 0.06
Nodes (43): Echo Balancer — Section 14 Step 3: Sensors, Estimator, NIS Test Implementation Plan, Self-review notes, Task 1: `sim/params.py` — sensor and estimator parameters, Task 2: `sim/sensors.py` — gyro, accelerometer, encoder, Task 3: `sim/estimator.py` — Kalman filter core, Task 4: Section 11 NIS consistency test (nominal 60s closed-loop run), Echo Balancer — Section 14 Step 5: World, Wall-Following Implementation Plan, Task 2: `sim/world.py` — corridor geometry, ray casting, pose kinematics (+35 more)

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
Nodes (11): 10. Gate, 2026-10-08 — Build-order steps 7 and 8 complete, 2026-10-08 — Wokwi firmware (extra, off the §14 build order), 2026-10-09 — Decisions A and B, Current configuration, Current objective, Current State — Echo Balancer, Immediate next steps (+3 more)

### Community 25 - "run_batch.py"
Cohesion: 0.11
Nodes (20): _payload_fn(), fn(), _accel_noise_fault(), _gyro_bias_fault(), _payload_shift(), _surface_change(), Decision: voltage clipping (`clip_voltage`, `V_batt`) lives in `sim/disturbances.py`, not `sim/plant.py`, clip_voltage() (+12 more)

### Community 26 - "test_gate.py"
Cohesion: 0.18
Nodes (21): Self-review notes, Task 4: Section 11 gate test — no mode change on a nominal 60s run, 2026-10-09 — Short-window spike detector ON by default (user decision "A"), 2026-10-08 state (supersedes older sections), GateMode, GateState, initial_gate_state(), step_gate() (+13 more)

### Community 27 - "Model Handoff — Echo Balancer"
Cohesion: 0.25
Nodes (8): Constraints the next model must respect, Current problems, Exact config/parameters in force right now, Failed approaches — do not repeat, Important files, Model Handoff — Echo Balancer, What the next model should do, What this project is (one paragraph)

### Community 28 - "main.cpp"
Cohesion: 0.31
Nodes (6): calibrate(), handleSerial(), loop(), readMpu(), setup(), showMode()

### Community 29 - "Echo Balancer — results (simulation v0, NXTway-GS parameters)"
Cohesion: 0.14
Nodes (13): 1. Controller comparison (CLAUDE.md section 12), 2. Threshold selection: exhaustive search vs Grover Adaptive Search (CLAUDE.md section 13), 3. Firmware (ESP32-S3, Wokwi), 4. Held-out checks (data never used for tuning), 5. Bugs found during this evaluation (fixed), 6. Limitations (read before citing any number), 7. Not done / open, Cost table (10 seeds, default gate structure) (+5 more)

### Community 30 - "numpy"
Cohesion: 0.24
Nodes (9): Task 5: Section 11 plant tests (energy conservation, open-loop fall), f(), test_energy_conserved_no_input_no_friction(), xdot(), test_equilibrium_is_fixed_point(), test_falls_without_control(), xdot(), test_state_derivative_shape() (+1 more)

### Community 31 - "What currently works (verified: 122/122 tests passing, `uv run pytest -q`)"
Cohesion: 0.14
Nodes (20): Design decision 2: wall-following episodes feed `design_lqr_speed_servo` from the TRUE state, not the KF estimate — matching step 5's existing precedent, Design decision 4: `TiltGateParams` calibration — the tilt-threshold baseline genuinely misses 3 of 6 disturbance types, by design, not by miscalibration, Design decision 6: mode's effect on control only applies to wall-following episodes, Design decision 7: reproducing CLAUDE.md section 5's fall condition is not optional — corridor episodes can genuinely diverge without it, Pre-verified design: five load-bearing findings from running this repo's actual code before writing this plan, Task 3: `sim/gate.py` — tilt-threshold baseline gate (`TiltGateState`, `initial_tilt_gate_state`, `step_tilt_gate`), Current implementation details worth knowing, What currently works (verified: 122/122 tests passing, `uv run pytest -q`) (+12 more)

### Community 34 - "pytest"
Cohesion: 0.08
Nodes (34): Design decision: Task 6 integrates position via `pose_velocity`/`rk4_step` at plant resolution (1ms), not a hand-rolled formula at control resolution (5ms), Design decision: the corridor is two infinite parallel walls, no corners or dead ends, Design decision: yaw control is proportional-only, not PD — the plant's yaw dynamics are too fast for derivative action at 200Hz, Pre-verified design: this step required designing two new control loops from scratch, Self-review notes, Task 1: `sim/params.py` — `CorridorParams`, `SpeedServoParams`, `YawControlParams`, `WallFollowParams`, Task 5: `sim/control.py` — yaw control, wall-following, front-threshold slowdown, Wall-following closed-loop verification (with real ultrasonic noise/dropout) (+26 more)

### Community 35 - "test_tilt_threshold_controller_detection"
Cohesion: 0.33
Nodes (4): Task 4: `sim/run.py` — `ControllerType`, `EpisodeConfig`, `run_episode` (balance-only path + fall condition), What the previous model (this session) was doing, test_nis_gate_detects_each_disturbance_within_window(), test_tilt_threshold_controller_detection()

### Community 36 - "run_batch"
Cohesion: 0.29
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

### Community 42 - "Echo Balancer on Wokwi (ESP32 NIS gate)"
Cohesion: 0.25
Nodes (6): Echo Balancer on Wokwi (ESP32 NIS gate), Hardware mapping / caveats, Injecting disturbances (CLAUDE.md section 8), Run it, Short-window spike detector, Verification (host, `tests/test_firmware_core.py`)

### Community 43 - "hold_on_dropout"
Cohesion: 0.25
Nodes (5): 2026-10-08 — Side-ultrasonic dropout rejection (`sim.sensors.hold_on_dropout`), Confirmed problems / gaps, hold_on_dropout(), test_cautious_mode_scales_down_corridor_speed_reference(), test_corridor_nominal_60s_stays_normal_at_the_calibrated_safe_speed()

### Community 44 - "default_plant_params"
Cohesion: 0.13
Nodes (15): Task 3: `sim/params.py` — LQR balance weights, Design decision: the energy-conservation test needs the back-EMF term zeroed too, Kalman filter design (implemented in Task 3), Decision: back-EMF (`Kb`) must be zeroed too for the energy-conservation test, linearize(), linearize_planar(), default_plant_params(), PlantParams (+7 more)

### Community 46 - "Item 8 — robot parameters"
Cohesion: 0.18
Nodes (10): A. Tell me which parts you actually bought (5 minutes, no tools), B. Measure with a kitchen scale and a ruler (30 minutes), C. Motor constants (from the datasheet, or measure: about 1 hour, needs a multimeter), D. Two short sensor logs (about 20 minutes, using a small sketch I'll provide), Decisions I need from you, Hardware checklist: what to measure and send back, Hardware I'd suggest adding (cheap, optional), Item 8 — robot parameters (+2 more)

### Community 47 - "ControllerType"
Cohesion: 0.13
Nodes (8): animate(), main(), simulate(), main(), _one(), main(), _one(), ControllerType

### Community 48 - "Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan"
Cohesion: 0.22
Nodes (8): Design decision 3 (CONFIRMED finding, promoted from `OPEN_PROBLEMS.md`'s "suspected"): sustained forward motion with active yaw control significantly inflates epsilon, and interacts with `sim/sensors.py::encoder`'s per-wheel-quantization-before-averaging in a way `OPEN_PROBLEMS.md` had flagged as untested, Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan, Task 1: `pyproject.toml` — add `pandas`, `pyarrow`, `joblib`, Task 2: `sim/params.py` — `RunParams`, `TiltGateParams`, `default_speed_servo_params_cautious`, Task 6: `analysis/metrics.py`, Task 7: `experiments/run_batch.py`, Task 8: Update `project-context/`, RunParams

### Community 49 - "push_psi_dot_kick"
Cohesion: 0.17
Nodes (12): _battery_droop(), _push(), Decision: push disturbance is a ψ̇ velocity kick, not a torque impulse requiring an inertia conversion, Failed: firing the battery-droop companion push at the droop's own onset, battery_droop_v_batt(), push_psi_dot_kick(), test_battery_droop_ramps_linearly_then_holds(), test_push_psi_dot_kick_applies_once_at_onset() (+4 more)

### Community 50 - "6. Parameters"
Cohesion: 0.67
Nodes (3): 6.1 Simulation v0 (NXTway-GS values; replace with measured values when hardware arrives), 6.2 Our hardware (fill in when measured; keep both sets selectable), 6. Parameters

## Knowledge Gaps
- **36 isolated node(s):** `NORMAL`, `CAUTIOUS`, `HALT`, `x`, `P` (+31 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 358 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **9 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `run_episode()` connect `run_episode` to `run.py`, `_closed_loop_with_gate`, `rk4_step`, `params.py`, `Decisions — Echo Balancer`, `test_qubo.py`, `design_lqr_balance`, `pandas`, `default_sensor_params`, `run_batch.py`, `test_gate.py`, `numpy`, `What currently works (verified: 122/122 tests passing, `uv run pytest -q`)`, `pytest`, `test_tilt_threshold_controller_detection`, `run_batch`, `hold_on_dropout`, `ControllerType`, `Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan`?**
  _High betweenness centrality (0.087) - this node is a cross-community bridge._
- **Are the 11 inferred relationships involving `run_episode()` (e.g. with `Design decision 1: `run_episode` does NOT unify onto one shared control law — `design_lqr_balance` for balance-only episodes, `design_lqr_speed_servo` for wall-following episodes` and `Design decision 2: wall-following episodes feed `design_lqr_speed_servo` from the TRUE state, not the KF estimate — matching step 5's existing precedent`) actually correct?**
  _`run_episode()` has 11 INFERRED edges - model-reasoned connections that need verification._
- **What connects `NORMAL`, `CAUTIOUS`, `HALT` to the rest of the system?**
  _36 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `run.py` be split into smaller, more focused modules?**
  _Cohesion score 0.12846068660022147 - nodes in this community are weakly interconnected._
- **Why does `What currently works (verified: 122/122 tests passing, `uv run pytest -q`)` connect `What currently works (verified: 122/122 tests passing, `uv run pytest -q`)` to `pytest`, `rk4_step`, `params.py`, `design_lqr_balance`, `pandas`, `ControllerType`, `Echo Balancer — Section 14 Step 6: run.py, run_batch.py, metrics.py Implementation Plan`, `run_episode`, `default_sensor_params`, `Current State — Echo Balancer`, `run_batch.py`, `test_gate.py`?**
  _High betweenness centrality (0.084) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `default_plant_params()` (e.g. with `Task 3: `sim/params.py` — LQR balance weights` and `Design decision: the energy-conservation test needs the back-EMF term zeroed too`) actually correct?**
  _`default_plant_params()` has 4 INFERRED edges - model-reasoned connections that need verification._
- **Should `rk4_step` be split into smaller, more focused modules?**
  _Cohesion score 0.08866995073891626 - nodes in this community are weakly interconnected._