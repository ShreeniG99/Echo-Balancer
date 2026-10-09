# Graph Report - Echo-Balancer  (2026-10-09)

## Corpus Check
- 70 files · ~126,906 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 13 file(s) not represented in the graph (top: .csv 7, (none) 3, .ini 1)

## Summary
- 859 nodes · 2481 edges · 46 communities (35 shown, 11 thin omitted)
- Extraction: 85% EXTRACTED · 15% INFERRED · 0% AMBIGUOUS · INFERRED: 380 edges (avg confidence: 0.94)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `c1e6ee86`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- numpy
- Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan
- Echo Balancer — Section 14 Step 4: Gate, Disturbances Implementation Plan
- rk4_step
- test_params.py
- Echo Balancer — Claude Project Knowledge
- echo-balancer
- test_qubo.py
- Decisions — Echo Balancer
- Echo Balancer — Simulation Spec
- design_lqr_balance
- Sim
- pandas
- run_episode
- Failed Approaches — Echo Balancer
- Wokwi guide, from zero (browser only, nothing to install)
- plot_firmware_gate.py
- f
- echo_core.hpp
- VirtualRobot
- Kalman
- .claude/CLAUDE.md
- Current State — Echo Balancer
- run_batch.py
- default_speed_servo_params_cautious
- Model Handoff — Echo Balancer
- main.cpp
- Echo Balancer — results (simulation v0, NXTway-GS parameters)
- EstimatorParams
- Project Context — Echo Balancer
- Echo Balancer — Section 14 Step 3: Sensors, Estimator, NIS Test Implementation Plan
- default_plant_params
- Echo Balancer — Section 14 Step 1: Plant Model Implementation Plan
- diagram.json
- NisGate
- host_scenarios.cpp
- Faults
- Open Problems — Echo Balancer
- test_recovers_from_10_degree_pitch_in_nonlinear_sim
- test_speed_servo_tracks_constant_forward_speed_in_nonlinear_sim

## God Nodes (most connected - your core abstractions)
1. `run_episode()` - 84 edges
2. `default_plant_params()` - 61 edges
3. `_closed_loop_with_gate()` - 40 edges
4. `design_lqr_balance()` - 39 edges
5. `ControllerType` - 35 edges
6. `design_lqr_speed_servo()` - 34 edges
7. `default_sensor_params()` - 33 edges
8. `rk4_step()` - 32 edges
9. `EpisodeConfig` - 31 edges
10. `default_disturbance_params()` - 30 edges

## Surprising Connections (you probably didn't know these)
- `QUBO formulation (`quantum/formulation.py`)` --references--> `verify_exact()`  [INFERRED]
  docs/RESULTS.md → quantum/formulation.py
- `What does not exist yet (build order steps 7-8, not started)` --references--> `front_threshold_speed_adjust()`  [INFERRED]
  project-context/CURRENT_STATE.md → sim/control.py
- `Current problems` --references--> `front_threshold_speed_adjust()`  [INFERRED]
  project-context/MODEL_HANDOFF.md → sim/control.py
- `Design decision: the NORMAL→HALT "skip" exception is read as "window not yet full", not "sustained over a separate timer"` --references--> `step_gate()`  [INFERRED]
  docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md → sim/gate.py
- `Decision: NORMAL→HALT "skip" fires when the window is full, not on a separate sustained-duration timer` --references--> `step_gate()`  [INFERRED]
  project-context/DECISIONS.md → sim/gate.py

## Import Cycles
- None detected.

## Communities (46 total, 11 thin omitted)

### Community 0 - "numpy"
Cohesion: 0.06
Nodes (69): Self-review notes, Task 4: Section 11 gate test — no mode change on a nominal 60s run, Task 3: `sim/sensors.py` — ultrasonic sensor model, Wall-following closed-loop verification (with real ultrasonic noise/dropout), arr(), main(), Key components (built so far), discretize() (+61 more)

### Community 1 - "Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan"
Cohesion: 0.22
Nodes (8): Design decision: Q/R weights are parameters, not magic numbers, Design decision: the recovery test uses per-step feedback, not a 200 Hz ZOH loop, Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan, Pre-verified math, Task 1: Add scipy dependency, Task 2: `sim/linearize.py` — analytic + numeric-verified Jacobian, Task 3: `sim/params.py` — LQR balance weights, Task 4: `sim/control.py` — balance LQR design, pole test, nonlinear recovery test

### Community 2 - "Echo Balancer — Section 14 Step 4: Gate, Disturbances Implementation Plan"
Cohesion: 0.22
Nodes (7): Echo Balancer — Section 14 Step 4: Gate, Disturbances Implementation Plan, Task 1: `sim/params.py` — `GateParams` and `DisturbanceParams`, Task 2: `sim/gate.py` — Normal/Cautious/Halt state machine, Task 3: `sim/disturbances.py` — the five disturbance profiles, Task 5: Section 11 gate tests — enters CAUTIOUS under each disturbance (Milestone 1), Decision: `sim/world.py::pose_velocity` integrates at 1ms plant-substep resolution, not 5ms control-tick resolution, DisturbanceParams

### Community 3 - "rk4_step"
Cohesion: 0.13
Nodes (7): Self-review notes, rk4_step(), simulate(), test_rk4_step_is_fourth_order_accurate(), error_at(), xdot(), test_zero_order_hold_freezes_u_across_rk4_stages()

### Community 4 - "test_params.py"
Cohesion: 0.07
Nodes (29): default_corridor_params(), default_yaw_control_params(), test_corridor_params_is_frozen(), test_default_corridor_params(), test_default_corridor_params_ray_parallel_eps(), test_default_disturbance_params(), test_default_estimator_params(), test_default_gate_params() (+21 more)

### Community 5 - "Echo Balancer — Claude Project Knowledge"
Cohesion: 0.17
Nodes (11): Architecture, Constraints, Current problems, Current state, Current task, Echo Balancer — Claude Project Knowledge, Failed approaches (do not re-suggest these), Goal (+3 more)

### Community 9 - "test_qubo.py"
Cohesion: 0.07
Nodes (36): 2026-10-08 — QUBO formulation: exact HUBO -> Rosenberg QUBO, rank-transformed, verified by brute force, decode(), _episode(), evaluation_set(), gate_params_for(), main(), score(), simulate_rows() (+28 more)

### Community 10 - "Decisions — Echo Balancer"
Cohesion: 0.12
Nodes (17): 2026-10-08 — Fallback ported to `sim/run.py` (default on), 2026-10-08 — Firmware fallback: softer gains in HALT + KF Q x100 outside NORMAL; target is ESP32-S3, 2026-10-08 — qiskit pinned: qiskit 2.5.2, qiskit-optimization 0.7.0, qiskit-aer 0.17.2, 2026-10-08 — Wokwi firmware lives in `firmware/` (user-approved scope addition), Decision: back-EMF (`Kb`) must be zeroed too for the energy-conservation test, Decision: battery-droop and payload-shift disturbance tests need a companion condition to be observable at all, Decision: corridor episodes default to `corridor_theta_dot_ref_nominal=0.3` rad/s, not `tests/test_wall_following.py`'s `2.0`, Decision: encoder quantizes per-wheel, then averages (not average-then-quantize) (+9 more)

### Community 11 - "Echo Balancer — Simulation Spec"
Cohesion: 0.10
Nodes (20): 10. Gate, 11. Required tests (tests/), 12. Evaluation, 13. Quantum (offline threshold selection), 14. Build order and milestones, 15. Key references, 16. Persistent project knowledge (read before scanning the repo), 1. The claim (what the project is) (+12 more)

### Community 12 - "design_lqr_balance"
Cohesion: 0.06
Nodes (58): Self-review notes, Kalman filter design (implemented in Task 3), Design decision: a real "speed servo" LQR is required — reference-tracking on the existing 4-state balance LQR is not accurate enough, Design decision: Task 6 integrates position via `pose_velocity`/`rk4_step` at plant resolution (1ms), not a hand-rolled formula at control resolution (5ms), Design decision: the corridor is two infinite parallel walls, no corners or dead ends, Design decision: yaw control is proportional-only, not PD — the plant's yaw dynamics are too fast for derivative action at 200Hz, Echo Balancer — Section 14 Step 5: World, Wall-Following Implementation Plan, Pre-verified design: this step required designing two new control loops from scratch (+50 more)

### Community 13 - "Sim"
Cohesion: 0.13
Nodes (14): Sim, epsilon, fallen, faults, kf, nis, nis_gate, robot (+6 more)

### Community 14 - "pandas"
Cohesion: 0.13
Nodes (27): detection_delay(), fall_rate(), false_fallback_fraction(), missed_fallback(), progress(), summarize_batch(), fig_detection(), fig_episode() (+19 more)

### Community 15 - "run_episode"
Cohesion: 0.05
Nodes (53): animate(), main(), simulate(), Design decision 2: wall-following episodes feed `design_lqr_speed_servo` from the TRUE state, not the KF estimate — matching step 5's existing precedent, Design decision 3 (CONFIRMED finding, promoted from `OPEN_PROBLEMS.md`'s "suspected"): sustained forward motion with active yaw control significantly inflates epsilon, and interacts with `sim/sensors.py::encoder`'s per-wheel-quantization-before-averaging in a way `OPEN_PROBLEMS.md` had flagged as untested, Design decision 5: CAUTIOUS mode's "softer Q" speed-servo gain — verified stable, chosen over reducing `Q_psi` alone, Design decision 6: mode's effect on control only applies to wall-following episodes, Design decision 7: reproducing CLAUDE.md section 5's fall condition is not optional — corridor episodes can genuinely diverge without it (+45 more)

### Community 16 - "Failed Approaches — Echo Balancer"
Cohesion: 0.25
Nodes (8): Discrete PD (nonzero `Kd`) for yaw-rate control, Failed Approaches — Echo Balancer, Failed (as a target, not as an implementation): literal χ²(3N) quantiles for the gate's τ1/τ2, Failed (as a target, not as an implementation): the literal i.i.d.-mean χ²(3N) acceptance interval for the NIS test, Failed (as a test design, not an implementation): assuming HALT mode makes `theta_dot` settle near zero within one `T_dwell` window, Failed: increasing process noise `Q` to raise mean NIS toward χ²(3)'s mean of 3, Failed: large "safely uncertain" initial Kalman filter covariance `P0`, Failed: `tests/test_wall_following.py`'s `theta_dot_ref_nominal=2.0` as the corridor evaluation speed

### Community 17 - "Wokwi guide, from zero (browser only, nothing to install)"
Cohesion: 0.13
Nodes (13): Echo Balancer on Wokwi (ESP32 NIS gate), Hardware mapping / caveats, Injecting disturbances (CLAUDE.md section 8), Run it, Verification (host, `tests/test_firmware_core.py`), A. Create the project, B. Add the code files, C. Run (+5 more)

### Community 18 - "plot_firmware_gate.py"
Cohesion: 0.21
Nodes (4): main(), run(), binary(), scenarios_bin()

### Community 19 - "f"
Cohesion: 0.24
Nodes (9): Task 5: Section 11 plant tests (energy conservation, open-loop fall), f(), test_energy_conserved_no_input_no_friction(), xdot(), test_equilibrium_is_fixed_point(), test_falls_without_control(), xdot(), test_state_derivative_shape() (+1 more)

### Community 20 - "echo_core.hpp"
Cohesion: 0.18
Nodes (8): Mode, CAUTIOUS, HALT, mode_name(), NORMAL, TiltGate, mode, time_in_mode

### Community 21 - "VirtualRobot"
Cohesion: 0.19
Nodes (6): Design decision 4: `TiltGateParams` calibration — the tilt-threshold baseline genuinely misses 3 of 6 disturbance types, by design, not by miscalibration, Rng, s, VirtualRobot, rng, x

### Community 22 - "Kalman"
Cohesion: 0.19
Nodes (6): clip_motor(), Kalman, P, q_scale, x, main()

### Community 24 - "Current State — Echo Balancer"
Cohesion: 0.20
Nodes (10): 2026-10-08 — Build-order steps 7 and 8 complete, 2026-10-08 — Wokwi firmware (extra, off the §14 build order), Current configuration, Current objective, Current State — Echo Balancer, Immediate next steps, Uncommitted / unusual repo state as of this session, What does not exist yet (build order steps 7-8, not started) (+2 more)

### Community 25 - "run_batch.py"
Cohesion: 0.06
Nodes (46): Design decision: voltage clipping (`clip_voltage`, `V_batt`) lands in `sim/disturbances.py`, not `sim/plant.py`, _accel_noise_fault(), _battery_droop(), _gyro_bias_fault(), main(), _payload_shift(), _push(), run_batch() (+38 more)

### Community 26 - "default_speed_servo_params_cautious"
Cohesion: 0.40
Nodes (4): Decision: `default_speed_servo_params_cautious()` divides every `SpeedServoParams` Q weight by 5, Decision: HALT mode uses the nominal (not cautious) speed-servo gain, default_speed_servo_params_cautious(), test_default_speed_servo_params_cautious_has_softer_q_than_nominal()

### Community 27 - "Model Handoff — Echo Balancer"
Cohesion: 0.25
Nodes (8): Constraints the next model must respect, Current problems, Exact config/parameters in force right now, Failed approaches — do not repeat, Important files, Model Handoff — Echo Balancer, What the next model should do, What this project is (one paragraph)

### Community 28 - "main.cpp"
Cohesion: 0.31
Nodes (6): calibrate(), handleSerial(), loop(), readMpu(), setup(), showMode()

### Community 29 - "Echo Balancer — results (simulation v0, NXTway-GS parameters)"
Cohesion: 0.20
Nodes (9): 1. Controller comparison (CLAUDE.md section 12), 2. Threshold selection: exhaustive search vs Grover Adaptive Search (CLAUDE.md section 13), 3. Firmware (ESP32-S3, Wokwi), 4. Bugs found during this evaluation (fixed), 5. Not done / open, Cost table, Echo Balancer — results (simulation v0, NXTway-GS parameters), Grover vs exhaustive (+1 more)

### Community 30 - "EstimatorParams"
Cohesion: 0.22
Nodes (9): Self-review notes, Design decision: push is a ψ̇ velocity kick, not a torque impulse requiring an inertia conversion, Design decision: the NORMAL→HALT "skip" exception is read as "window not yet full", not "sustained over a separate timer", Design decision: τ1/τ2 cannot be literal χ²(3N) quantiles, at any quantile level, Disturbance magnitudes — five profiles, each independently verified, two with genuine surprises, Pre-verified design: this took by far the most empirical exploration of any step so far, Important decisions, Decision: `EstimatorParams` defaults are the empirically-tuned values in `sim/params.py` (+1 more)

### Community 31 - "Project Context — Echo Balancer"
Cohesion: 0.20
Nodes (10): 2026-10-08 additions, Architecture (planned, per `CLAUDE.md` §4), Constraints (from `CLAUDE.md`, non-negotiable unless the user changes the spec), External / official resources, Important files, Project Context — Echo Balancer, Project-specific terminology, Purpose / problem statement (+2 more)

### Community 34 - "Echo Balancer — Section 14 Step 3: Sensors, Estimator, NIS Test Implementation Plan"
Cohesion: 0.22
Nodes (8): Design decision: the NIS test's acceptance interval is chi2(3)'s own 95% interval, not the tighter "mean of 12000 i.i.d. samples" interval, Echo Balancer — Section 14 Step 3: Sensors, Estimator, NIS Test Implementation Plan, Pre-verified design: sensor models, KF tuning, and the NIS test's acceptance bounds, Sensor models (implemented in Task 2), Task 1: `sim/params.py` — sensor and estimator parameters, Task 2: `sim/sensors.py` — gyro, accelerometer, encoder, Task 3: `sim/estimator.py` — Kalman filter core, Task 4: Section 11 NIS consistency test (nominal 60s closed-loop run)

### Community 35 - "default_plant_params"
Cohesion: 0.33
Nodes (5): default_plant_params(), _numeric_jacobian(), test_numeric_jacobian_matches_analytic(), test_planar_eigenvalues_match_sanity_values(), test_planar_is_a_consistent_reduction_of_the_full_system()

### Community 36 - "Echo Balancer — Section 14 Step 1: Plant Model Implementation Plan"
Cohesion: 0.25
Nodes (7): Design decision: the energy-conservation test needs the back-EMF term zeroed too, Echo Balancer — Section 14 Step 1: Plant Model Implementation Plan, Open item to flag before/while executing, Task 1: uv project scaffolding, Task 2: `sim/params.py` — plant parameters, Task 3: `sim/plant.py` — nonlinear dynamics, Task 4: `sim/integrate.py` — fixed-step RK4

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
Cohesion: 0.33
Nodes (6): 2026-10-08 — Confirmed limits after build-order steps 7-8, 2026-10-08 — Nominal corridor run can fall after a false alarm (confirmed once; mechanism suspected), 2026-10-08 — Python sim vs firmware fallback (confirmed divergence), Open Problems — Echo Balancer, Suspected problems (plausible, not verified), Unknowns (genuinely open, not yet investigated)

## Knowledge Gaps
- **36 isolated node(s):** `NORMAL`, `CAUTIOUS`, `HALT`, `x`, `P` (+31 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 333 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **11 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `What currently works (verified: 122/122 tests passing, `uv run pytest -q`)` connect `design_lqr_balance` to `numpy`, `rk4_step`, `pandas`, `run_episode`, `VirtualRobot`, `Current State — Echo Balancer`, `run_batch.py`, `default_speed_servo_params_cautious`?**
  _High betweenness centrality (0.097) - this node is a cross-community bridge._
- **Are the 11 inferred relationships involving `run_episode()` (e.g. with `Design decision 1: `run_episode` does NOT unify onto one shared control law — `design_lqr_balance` for balance-only episodes, `design_lqr_speed_servo` for wall-following episodes` and `Design decision 2: wall-following episodes feed `design_lqr_speed_servo` from the TRUE state, not the KF estimate — matching step 5's existing precedent`) actually correct?**
  _`run_episode()` has 11 INFERRED edges - model-reasoned connections that need verification._
- **What connects `NORMAL`, `CAUTIOUS`, `HALT` to the rest of the system?**
  _36 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `numpy` be split into smaller, more focused modules?**
  _Cohesion score 0.05559593023255814 - nodes in this community are weakly interconnected._
- **Why does `run_episode()` connect `run_episode` to `numpy`, `rk4_step`, `test_params.py`, `default_plant_params`, `test_qubo.py`, `design_lqr_balance`, `pandas`, `f`, `run_batch.py`, `default_speed_servo_params_cautious`?**
  _High betweenness centrality (0.093) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `default_plant_params()` (e.g. with `Task 3: `sim/params.py` — LQR balance weights` and `Design decision: the energy-conservation test needs the back-EMF term zeroed too`) actually correct?**
  _`default_plant_params()` has 4 INFERRED edges - model-reasoned connections that need verification._
- **Should `rk4_step` be split into smaller, more focused modules?**
  _Cohesion score 0.1341991341991342 - nodes in this community are weakly interconnected._