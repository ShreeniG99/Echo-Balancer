# Graph Report - Echo Balancer  (2026-09-22)

## Corpus Check
- 48 files · ~56,313 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 4 file(s) not represented in the graph (top: (none) 3, .lock 1)

## Summary
- 506 nodes · 1223 edges · 22 communities (15 shown, 7 thin omitted)
- Extraction: 81% EXTRACTED · 19% INFERRED · 0% AMBIGUOUS · INFERRED: 232 edges (avg confidence: 0.94)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `d94f6149`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- test_estimator.py
- default_plant_params
- test_gate.py
- rk4_step
- test_params.py
- EstimatorParams
- echo-balancer
- What You Must Do When Invoked
- Decisions — Echo Balancer
- Echo Balancer — Simulation Spec
- design_lqr_balance
- graphify reference: extra exports and benchmark
- graphify reference: query, path, explain
- graphify reference: add a URL and watch a folder
- graphify reference: commit hook and native CLAUDE.md integration
- graphify reference: incremental update and cluster-only
- graphify reference: GitHub clone and cross-repo merge
- graphify reference: transcribe video and audio
- .claude/CLAUDE.md
- extraction-spec.md

## God Nodes (most connected - your core abstractions)
1. `default_plant_params()` - 46 edges
2. `_closed_loop_with_gate()` - 38 edges
3. `rk4_step()` - 31 edges
4. `design_lqr_balance()` - 28 edges
5. `f()` - 27 edges
6. `_run_nominal_closed_loop()` - 27 edges
7. `PlantParams` - 26 edges
8. `default_sensor_params()` - 25 edges
9. `design_lqr_speed_servo()` - 22 edges
10. `linearize_planar()` - 21 edges

## Surprising Connections (you probably didn't know these)
- `Current problems` --references--> `front_threshold_speed_adjust()`  [INFERRED]
  project-context/MODEL_HANDOFF.md → sim/control.py
- `Design decision: voltage clipping (`clip_voltage`, `V_batt`) lands in `sim/disturbances.py`, not `sim/plant.py`` --references--> `clip_voltage()`  [INFERRED]
  docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md → sim/disturbances.py
- `Design decision: the NORMAL→HALT "skip" exception is read as "window not yet full", not "sustained over a separate timer"` --references--> `step_gate()`  [INFERRED]
  docs/superpowers/plans/2026-09-21-gate-disturbances-step4.md → sim/gate.py
- `Decision: NORMAL→HALT "skip" fires when the window is full, not on a separate sustained-duration timer` --references--> `step_gate()`  [INFERRED]
  project-context/DECISIONS.md → sim/gate.py
- `Decision: LQR Q/R weights live in `sim/params.py` as `LQRBalanceParams`` --references--> `LQRBalanceParams`  [INFERRED]
  project-context/DECISIONS.md → sim/params.py

## Import Cycles
- None detected.

## Communities (22 total, 7 thin omitted)

### Community 0 - "test_estimator.py"
Cohesion: 0.07
Nodes (63): Echo Balancer — Section 14 Step 3: Sensors, Estimator, NIS Test Implementation Plan, Self-review notes, Task 1: `sim/params.py` — sensor and estimator parameters, Task 2: `sim/sensors.py` — gyro, accelerometer, encoder, Task 3: `sim/estimator.py` — Kalman filter core, Task 4: Section 11 NIS consistency test (nominal 60s closed-loop run), Task 3: `sim/sensors.py` — ultrasonic sensor model, Key components (built so far) (+55 more)

### Community 1 - "default_plant_params"
Cohesion: 0.05
Nodes (64): Design decision: Q/R weights are parameters, not magic numbers, Design decision: the recovery test uses per-step feedback, not a 200 Hz ZOH loop, Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan, Pre-verified math, Self-review notes, Task 1: Add scipy dependency, Task 2: `sim/linearize.py` — analytic + numeric-verified Jacobian, Task 3: `sim/params.py` — LQR balance weights (+56 more)

### Community 2 - "test_gate.py"
Cohesion: 0.06
Nodes (73): dataclasses, Echo Balancer — Section 14 Step 4: Gate, Disturbances Implementation Plan, Self-review notes, Task 1: `sim/params.py` — `GateParams` and `DisturbanceParams`, Task 2: `sim/gate.py` — Normal/Cautious/Halt state machine, Task 3: `sim/disturbances.py` — the five disturbance profiles, Task 4: Section 11 gate test — no mode change on a nominal 60s run, Task 5: Section 11 gate tests — enters CAUTIOUS under each disturbance (Milestone 1) (+65 more)

### Community 3 - "rk4_step"
Cohesion: 0.08
Nodes (29): Design decision: the NIS test's acceptance interval is chi2(3)'s own 95% interval, not the tighter "mean of 12000 i.i.d. samples" interval, Pre-verified design: sensor models, KF tuning, and the NIS test's acceptance bounds, Sensor models (implemented in Task 2), DynamicsFn, Input, InputFn, Constraints the next model must respect, Current problems (+21 more)

### Community 4 - "test_params.py"
Cohesion: 0.08
Nodes (32): math, default_corridor_params(), default_estimator_params(), default_gate_params(), default_wall_follow_params(), default_yaw_control_params(), test_front_threshold_speed_adjust_slows_down_below_threshold(), test_wall_following_control_formula_and_state_threading() (+24 more)

### Community 5 - "EstimatorParams"
Cohesion: 0.06
Nodes (33): Design decision: push is a ψ̇ velocity kick, not a torque impulse requiring an inertia conversion, Design decision: the NORMAL→HALT "skip" exception is read as "window not yet full", not "sustained over a separate timer", Design decision: voltage clipping (`clip_voltage`, `V_batt`) lands in `sim/disturbances.py`, not `sim/plant.py`, Design decision: τ1/τ2 cannot be literal χ²(3N) quantiles, at any quantile level, Disturbance magnitudes — five profiles, each independently verified, two with genuine surprises, Pre-verified design: this took by far the most empirical exploration of any step so far, Architecture, Constraints (+25 more)

### Community 9 - "What You Must Do When Invoked"
Cohesion: 0.08
Nodes (24): For /graphify add and --watch, For /graphify query, For the commit hook and native CLAUDE.md integration, For --update and --cluster-only, /graphify, Honesty Rules, Interpreter guard for subcommands, Part A - Structural extraction for code files (+16 more)

### Community 10 - "Decisions — Echo Balancer"
Cohesion: 0.08
Nodes (25): Current configuration, Current objective, Current State — Echo Balancer, Immediate next steps, Uncommitted / unusual repo state as of this session, What does not exist yet (build order steps 6-8, not started), Decision: back-EMF (`Kb`) must be zeroed too for the energy-conservation test, Decision: battery-droop and payload-shift disturbance tests need a companion condition to be observable at all (+17 more)

### Community 11 - "Echo Balancer — Simulation Spec"
Cohesion: 0.10
Nodes (20): 10. Gate, 11. Required tests (tests/), 12. Evaluation, 13. Quantum (offline threshold selection), 14. Build order and milestones, 15. Key references, 16. Persistent project knowledge (read before scanning the repo), 1. The claim (what the project is) (+12 more)

### Community 12 - "design_lqr_balance"
Cohesion: 0.08
Nodes (59): Design decision: Task 6 integrates position via `pose_velocity`/`rk4_step` at plant resolution (1ms), not a hand-rolled formula at control resolution (5ms), Design decision: the corridor is two infinite parallel walls, no corners or dead ends, Design decision: yaw control is proportional-only, not PD — the plant's yaw dynamics are too fast for derivative action at 200Hz, Echo Balancer — Section 14 Step 5: World, Wall-Following Implementation Plan, Pre-verified design: this step required designing two new control loops from scratch, Self-review notes, Task 1: `sim/params.py` — `CorridorParams`, `SpeedServoParams`, `YawControlParams`, `WallFollowParams`, Task 2: `sim/world.py` — corridor geometry, ray casting, pose kinematics (+51 more)

### Community 13 - "graphify reference: extra exports and benchmark"
Cohesion: 0.22
Nodes (8): graphify reference: extra exports and benchmark, Step 6b - Wiki (only if --wiki flag), Step 7 - Neo4j export (only if --neo4j or --neo4j-push flag), Step 7a - FalkorDB export (only if --falkordb or --falkordb-push flag), Step 7b - SVG export (only if --svg flag), Step 7c - GraphML export (only if --graphml flag), Step 7d - MCP server (only if --mcp flag), Step 8 - Token reduction benchmark (only if total_words > 5000)

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

## Knowledge Gaps
- **116 isolated node(s):** `echo-balancer`, `graphify`, `Usage`, `What graphify is for`, `Step 0 - GitHub repos and multi-path merge (only if a URL or several paths)` (+111 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 226 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **7 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `default_plant_params()` connect `default_plant_params` to `test_estimator.py`, `test_gate.py`, `test_params.py`, `Decisions — Echo Balancer`, `design_lqr_balance`?**
  _High betweenness centrality (0.086) - this node is a cross-community bridge._
- **Why does `EstimatorParams` connect `EstimatorParams` to `test_estimator.py`, `default_plant_params`, `test_gate.py`, `test_params.py`, `design_lqr_balance`?**
  _High betweenness centrality (0.077) - this node is a cross-community bridge._
- **Why does `rk4_step()` connect `rk4_step` to `test_estimator.py`, `default_plant_params`, `test_gate.py`, `EstimatorParams`, `design_lqr_balance`?**
  _High betweenness centrality (0.074) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `default_plant_params()` (e.g. with `Task 3: `sim/params.py` — LQR balance weights` and `Design decision: the energy-conservation test needs the back-EMF term zeroed too`) actually correct?**
  _`default_plant_params()` has 4 INFERRED edges - model-reasoned connections that need verification._
- **Are the 2 inferred relationships involving `_closed_loop_with_gate()` (e.g. with `Self-review notes` and `Task 5: Section 11 gate tests — enters CAUTIOUS under each disturbance (Milestone 1)`) actually correct?**
  _`_closed_loop_with_gate()` has 2 INFERRED edges - model-reasoned connections that need verification._
- **Are the 10 inferred relationships involving `rk4_step()` (e.g. with `Self-review notes` and `Task 4: Section 11 NIS consistency test (nominal 60s closed-loop run)`) actually correct?**
  _`rk4_step()` has 10 INFERRED edges - model-reasoned connections that need verification._
- **Are the 15 inferred relationships involving `design_lqr_balance()` (e.g. with `Self-review notes` and `Task 4: Section 11 NIS consistency test (nominal 60s closed-loop run)`) actually correct?**
  _`design_lqr_balance()` has 15 INFERRED edges - model-reasoned connections that need verification._