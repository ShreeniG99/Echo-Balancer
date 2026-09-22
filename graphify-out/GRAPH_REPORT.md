# Graph Report - step6-run-batch-metrics  (2026-09-22)

## Corpus Check
- 57 files · ~74,357 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 5 file(s) not represented in the graph (top: (none) 4, .lock 1)

## Summary
- 650 nodes · 1753 edges · 34 communities (25 shown, 9 thin omitted)
- Extraction: 80% EXTRACTED · 20% INFERRED · 0% AMBIGUOUS · INFERRED: 352 edges (avg confidence: 0.94)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `0d29d835`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- run.py
- default_plant_params
- test_gate.py
- rk4_step
- test_params.py
- EstimatorParams
- echo-balancer
- What You Must Do When Invoked
- Decisions — Echo Balancer
- Echo Balancer — Simulation Spec
- cast_ray
- graphify reference: extra exports and benchmark
- What currently works (verified: 122/122 tests passing, `uv run pytest -q`)
- run_episode
- design_lqr_balance
- graphify reference: query, path, explain
- graphify reference: add a URL and watch a folder
- graphify reference: commit hook and native CLAUDE.md integration
- graphify reference: incremental update and cluster-only
- graphify reference: GitHub clone and cross-repo merge
- graphify reference: transcribe video and audio
- .claude/CLAUDE.md
- extraction-spec.md
- run_batch.py
- test_control.py
- front_threshold_speed_adjust
- ControllerType
- control.py
- default_disturbance_params
- sensor_fault_gyro_bias_step

## God Nodes (most connected - your core abstractions)
1. `run_episode()` - 74 edges
2. `default_plant_params()` - 52 edges
3. `_closed_loop_with_gate()` - 39 edges
4. `design_lqr_balance()` - 34 edges
5. `rk4_step()` - 32 edges
6. `design_lqr_speed_servo()` - 30 edges
7. `What currently works (verified: 122/122 tests passing, `uv run pytest -q`)` - 30 edges
8. `f()` - 29 edges
9. `ControllerType` - 28 edges
10. `Current implementation details worth knowing` - 28 edges

## Surprising Connections (you probably didn't know these)
- `What does not exist yet (build order steps 7-8, not started)` --references--> `front_threshold_speed_adjust()`  [INFERRED]
  project-context/CURRENT_STATE.md → sim/control.py
- `Design decision: voltage clipping (`clip_voltage`, `V_batt`) lands in `sim/disturbances.py`, not `sim/plant.py`` --references--> `clip_voltage()`  [INFERRED]
  docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md → sim/disturbances.py
- `Design decision: the NORMAL→HALT "skip" exception is read as "window not yet full", not "sustained over a separate timer"` --references--> `step_gate()`  [INFERRED]
  docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md → sim/gate.py
- `Decision: NORMAL→HALT "skip" fires when the window is full, not on a separate sustained-duration timer` --references--> `step_gate()`  [INFERRED]
  project-context/DECISIONS.md → sim/gate.py
- `Design decision: Q/R weights are parameters, not magic numbers` --references--> `LQRBalanceParams`  [INFERRED]
  docs/superpowers/plans/2026-09-19-linearize-lqr-step2.md → sim/params.py

## Import Cycles
- None detected.

## Communities (34 total, 9 thin omitted)

### Community 0 - "run.py"
Cohesion: 0.08
Nodes (63): dataclasses, Task 4: Section 11 NIS consistency test (nominal 60s closed-loop run), Task 3: `sim/sensors.py` — ultrasonic sensor model, numpy, Key components (built so far), Scheduled disturbance profiles (CLAUDE.md section 8). Each disturbance is a…, discretize(), initial_covariance() (+55 more)

### Community 1 - "default_plant_params"
Cohesion: 0.07
Nodes (36): Design decision: Q/R weights are parameters, not magic numbers, Design decision: the recovery test uses per-step feedback, not a 200 Hz ZOH loop, Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan, Pre-verified math, Self-review notes, Task 1: Add scipy dependency, Task 2: `sim/linearize.py` — analytic + numeric-verified Jacobian, Task 3: `sim/params.py` — LQR balance weights (+28 more)

### Community 2 - "test_gate.py"
Cohesion: 0.06
Nodes (72): Echo Balancer — Section 14 Step 4: Gate, Disturbances Implementation Plan, Self-review notes, Task 1: `sim/params.py` — `GateParams` and `DisturbanceParams`, Task 2: `sim/gate.py` — Normal/Cautious/Halt state machine, Task 3: `sim/disturbances.py` — the five disturbance profiles, Task 4: Section 11 gate test — no mode change on a nominal 60s run, Task 5: Section 11 gate tests — enters CAUTIOUS under each disturbance (Milestone 1), Design decision 1: `run_episode` does NOT unify onto one shared control law — `design_lqr_balance` for balance-only episodes, `design_lqr_speed_servo` for wall-following episodes (+64 more)

### Community 3 - "rk4_step"
Cohesion: 0.06
Nodes (42): Design decision: the energy-conservation test needs the back-EMF term zeroed too, Echo Balancer — Section 14 Step 1: Plant Model Implementation Plan, Open item to flag before/while executing, Self-review notes, Task 1: uv project scaffolding, Task 2: `sim/params.py` — plant parameters, Task 3: `sim/plant.py` — nonlinear dynamics, Task 4: `sim/integrate.py` — fixed-step RK4 (+34 more)

### Community 4 - "test_params.py"
Cohesion: 0.11
Nodes (25): math, default_corridor_params(), default_gate_params(), default_run_params(), default_tilt_gate_params(), Tests for sim.params module., CLAUDE.md section 10: tau1 < tau2, with hysteresis exits below their entries. A…, CLAUDE.md section 10: psi1 < psi2, with hysteresis exits below their entries. A… (+17 more)

### Community 5 - "EstimatorParams"
Cohesion: 0.06
Nodes (33): Design decision: the NIS test's acceptance interval is chi2(3)'s own 95% interval, not the tighter "mean of 12000 i.i.d. samples" interval, Echo Balancer — Section 14 Step 3: Sensors, Estimator, NIS Test Implementation Plan, Pre-verified design: sensor models, KF tuning, and the NIS test's acceptance bounds, Self-review notes, Sensor models (implemented in Task 2), Task 1: `sim/params.py` — sensor and estimator parameters, Task 2: `sim/sensors.py` — gyro, accelerometer, encoder, Task 3: `sim/estimator.py` — Kalman filter core (+25 more)

### Community 9 - "What You Must Do When Invoked"
Cohesion: 0.08
Nodes (24): For /graphify add and --watch, For /graphify query, For the commit hook and native CLAUDE.md integration, For --update and --cluster-only, /graphify, Honesty Rules, Interpreter guard for subcommands, Part A - Structural extraction for code files (+16 more)

### Community 10 - "Decisions — Echo Balancer"
Cohesion: 0.12
Nodes (19): Design decision 5: CAUTIOUS mode's "softer Q" speed-servo gain — verified stable, chosen over reducing `Q_psi` alone, Decision: back-EMF (`Kb`) must be zeroed too for the energy-conservation test, Decision: battery-droop and payload-shift disturbance tests need a companion condition to be observable at all, Decision: corridor episodes default to `corridor_theta_dot_ref_nominal=0.3` rad/s, not `tests/test_wall_following.py`'s `2.0`, Decision: `default_speed_servo_params_cautious()` divides every `SpeedServoParams` Q weight by 5, Decision: encoder quantizes per-wheel, then averages (not average-then-quantize), Decision: `GateParams.tau1`/`tau2` are empirically calibrated, not literal χ²(3N) quantiles, Decision: Grover Adaptive Search makes no speedup claim (+11 more)

### Community 11 - "Echo Balancer — Simulation Spec"
Cohesion: 0.06
Nodes (35): 10. Gate, 11. Required tests (tests/), 12. Evaluation, 13. Quantum (offline threshold selection), 14. Build order and milestones, 15. Key references, 16. Persistent project knowledge (read before scanning the repo), 1. The claim (what the project is) (+27 more)

### Community 12 - "cast_ray"
Cohesion: 0.18
Nodes (19): Decision: the corridor is two infinite parallel walls, no corners or dead ends, CorridorParams, 2D corridor geometry (CLAUDE.md section 4/9). Two infinite parallel walls at…, pose_dotf(), cast_ray(), pose_velocity(), ndarray, 2D corridor geometry, ray casting, and robot pose kinematics (CLAUDE.md section… (+11 more)

### Community 13 - "graphify reference: extra exports and benchmark"
Cohesion: 0.22
Nodes (8): graphify reference: extra exports and benchmark, Step 6b - Wiki (only if --wiki flag), Step 7 - Neo4j export (only if --neo4j or --neo4j-push flag), Step 7a - FalkorDB export (only if --falkordb or --falkordb-push flag), Step 7b - SVG export (only if --svg flag), Step 7c - GraphML export (only if --graphml flag), Step 7d - MCP server (only if --mcp flag), Step 8 - Token reduction benchmark (only if total_words > 5000)

### Community 14 - "What currently works (verified: 122/122 tests passing, `uv run pytest -q`)"
Cohesion: 0.17
Nodes (27): detection_delay(), fall_rate(), false_fallback_fraction(), missed_fallback(), progress(), DataFrame, Batch evaluation metrics (CLAUDE.md section 12). Operate on the (steps_df,…, Fraction of episodes that fell, grouped by controller. (+19 more)

### Community 15 - "run_episode"
Cohesion: 0.14
Nodes (27): Task 4: `sim/run.py` — `ControllerType`, `EpisodeConfig`, `run_episode` (balance-only path + fall condition), Task 5: `sim/run.py` — wall-following (corridor) path, Task 8: Update `project-context/`, scipy_stats, EpisodeConfig, DataFrame, One episode's setup. (config, seed) must reproduce an identical DataFrame…, run_episode() (+19 more)

### Community 16 - "design_lqr_balance"
Cohesion: 0.11
Nodes (27): Design decision: a real "speed servo" LQR is required — reference-tracking on the existing 4-state balance LQR is not accurate enough, Design decision: the corridor is two infinite parallel walls, no corners or dead ends, Echo Balancer — Section 14 Step 5: World, Wall-Following Implementation Plan, Pre-verified design: this step required designing two new control loops from scratch, Task 2: `sim/world.py` — corridor geometry, ray casting, pose kinematics, Task 4: `sim/control.py` — speed servo LQR, Wall-following closed-loop verification (with real ultrasonic noise/dropout), Decision: `design_lqr_speed_servo` is a separate 5-state LQR, not a modification of `design_lqr_balance` (+19 more)

### Community 17 - "graphify reference: query, path, explain"
Cohesion: 0.33
Nodes (5): For /graphify explain, For /graphify path, graphify reference: query, path, explain, Step 0 — Constrained query expansion (REQUIRED before traversal), Step 1 — Traversal

### Community 18 - "graphify reference: add a URL and watch a folder"
Cohesion: 0.50
Nodes (3): For /graphify add, For --watch, graphify reference: add a URL and watch a folder

### Community 19 - "graphify reference: commit hook and native CLAUDE.md integration"
Cohesion: 0.50
Nodes (3): For git commit hook, For native CLAUDE.md integration, graphify reference: commit hook and native CLAUDE.md integration

### Community 20 - "graphify reference: incremental update and cluster-only"
Cohesion: 0.50
Nodes (3): For --cluster-only, For --update (incremental re-extraction), graphify reference: incremental update and cluster-only

### Community 25 - "run_batch.py"
Cohesion: 0.11
Nodes (23): _accel_noise_fault(), _battery_droop(), _payload_shift(), _push(), Batch evaluation across the three CLAUDE.md section 12 controllers. Produces…, _surface_change(), joblib, pathlib (+15 more)

### Community 26 - "test_control.py"
Cohesion: 0.14
Nodes (19): pytest, CLAUDE.md section 9: "side ultrasonic distance error -> PD -> yaw-rate…, wall_following_control(), default_speed_servo_params(), default_wall_follow_params(), default_yaw_control_params(), test_front_threshold_speed_adjust_slows_down_below_threshold(), test_speed_servo_closed_loop_poles_stable() (+11 more)

### Community 27 - "front_threshold_speed_adjust"
Cohesion: 0.14
Nodes (16): Task 6: Closed-loop wall-following demonstration, Decision: voltage clipping (`clip_voltage`, `V_batt`) lives in `sim/disturbances.py`, not `sim/plant.py`, Architecture you need to know, Constraints the next model must respect, Current problems, Current state in one paragraph, Decisions you must not silently re-litigate, Exact config/parameters in force right now (+8 more)

### Community 28 - "ControllerType"
Cohesion: 0.20
Nodes (15): main(), DataFrame, run_batch(), _run_one(), Decision: `experiments/run_batch.py`'s workers receive scenario data as an explicit parameter, not via module-level lookup, What the next model should do, ControllerType, Enum (+7 more)

### Community 29 - "control.py"
Cohesion: 0.21
Nodes (14): Design decision: yaw control is proportional-only, not PD — the plant's yaw dynamics are too fast for derivative action at 200Hz, Self-review notes, Task 1: `sim/params.py` — `CorridorParams`, `SpeedServoParams`, `YawControlParams`, `WallFollowParams`, Task 5: `sim/control.py` — yaw control, wall-following, front-threshold slowdown, Decision: yaw control is proportional-only, permanently — do not add `Kd`, scipy_linalg, Controllers (CLAUDE.md section 9). design_lqr_balance: the balance LQR on the…, CLAUDE.md section 9: "Yaw: PD on phi_dot with differential voltage."… (+6 more)

### Community 30 - "default_disturbance_params"
Cohesion: 0.27
Nodes (12): Decision: push disturbance is a ψ̇ velocity kick, not a torque impulse requiring an inertia conversion, push_psi_dot_kick(), CLAUDE.md #3: an impulse torque on psi, modeled as an instantaneous psi_dot…, default_disturbance_params(), The tilt-threshold baseline's known, expected blind spot -- push's psi…, Documents, rather than hides, the plan doc's "Design decision 1" finding: push-…, test_corridor_push_disturbance_is_a_known_miss_for_the_nis_gate(), apply_push() (+4 more)

### Community 31 - "sensor_fault_gyro_bias_step"
Cohesion: 0.25
Nodes (8): _gyro_bias_fault(), parametrize, CLAUDE.md #4 (variant a): a persistent gyro bias step at onset., sensor_fault_gyro_bias_step(), test_sensor_fault_gyro_bias_step_applies_once_at_onset(), apply_disturbance(), Verified pre-plan against the balance-only architecture: the tilt-threshold…, test_tilt_threshold_controller_detection()

## Knowledge Gaps
- **118 isolated node(s):** `echo-balancer`, `graphify`, `Usage`, `What graphify is for`, `Step 0 - GitHub repos and multi-path merge (only if a URL or several paths)` (+113 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 264 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **9 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `run_episode()` connect `run_episode` to `run.py`, `default_plant_params`, `test_gate.py`, `rk4_step`, `test_params.py`, `Decisions — Echo Balancer`, `cast_ray`, `design_lqr_balance`, `run_batch.py`, `test_control.py`, `front_threshold_speed_adjust`, `ControllerType`, `control.py`, `default_disturbance_params`, `sensor_fault_gyro_bias_step`?**
  _High betweenness centrality (0.118) - this node is a cross-community bridge._
- **Why does `default_plant_params()` connect `default_plant_params` to `run.py`, `test_gate.py`, `rk4_step`, `test_params.py`, `Decisions — Echo Balancer`, `cast_ray`, `What currently works (verified: 122/122 tests passing, `uv run pytest -q`)`, `run_episode`, `run_batch.py`, `test_control.py`?**
  _High betweenness centrality (0.091) - this node is a cross-community bridge._
- **Why does `EstimatorParams` connect `EstimatorParams` to `run.py`, `test_gate.py`, `test_params.py`, `Echo Balancer — Simulation Spec`, `front_threshold_speed_adjust`?**
  _High betweenness centrality (0.057) - this node is a cross-community bridge._
- **Are the 11 inferred relationships involving `run_episode()` (e.g. with `Design decision 1: `run_episode` does NOT unify onto one shared control law — `design_lqr_balance` for balance-only episodes, `design_lqr_speed_servo` for wall-following episodes` and `Design decision 2: wall-following episodes feed `design_lqr_speed_servo` from the TRUE state, not the KF estimate — matching step 5's existing precedent`) actually correct?**
  _`run_episode()` has 11 INFERRED edges - model-reasoned connections that need verification._
- **Are the 4 inferred relationships involving `default_plant_params()` (e.g. with `Task 3: `sim/params.py` — LQR balance weights` and `Design decision: the energy-conservation test needs the back-EMF term zeroed too`) actually correct?**
  _`default_plant_params()` has 4 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `_closed_loop_with_gate()` (e.g. with `Self-review notes` and `Task 5: Section 11 gate tests — enters CAUTIOUS under each disturbance (Milestone 1)`) actually correct?**
  _`_closed_loop_with_gate()` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 19 inferred relationships involving `design_lqr_balance()` (e.g. with `Self-review notes` and `Task 4: Section 11 NIS consistency test (nominal 60s closed-loop run)`) actually correct?**
  _`design_lqr_balance()` has 19 INFERRED edges - model-reasoned connections that need verification._