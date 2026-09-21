# Graph Report - Echo Balancer  (2026-09-20)

## Corpus Check
- 39 files · ~36,286 words
- Verdict: corpus is large enough that graph structure adds value.
- Unclassified: 4 file(s) not represented in the graph (top: (none) 3, .lock 1)

## Summary
- 328 nodes · 597 edges · 25 communities (18 shown, 7 thin omitted)
- Extraction: 87% EXTRACTED · 13% INFERRED · 0% AMBIGUOUS · INFERRED: 77 edges (avg confidence: 0.94)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `a7d84061`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- test_estimator.py
- PlantParams
- f
- rk4_step
- default_plant_params
- EstimatorParams
- echo-balancer
- What You Must Do When Invoked
- Current State — Echo Balancer
- Echo Balancer — Simulation Spec
- Model Handoff — Echo Balancer
- graphify reference: extra exports and benchmark
- Project Context — Echo Balancer
- Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan
- Decisions — Echo Balancer
- graphify reference: query, path, explain
- graphify reference: add a URL and watch a folder
- graphify reference: commit hook and native CLAUDE.md integration
- graphify reference: incremental update and cluster-only
- graphify reference: GitHub clone and cross-repo merge
- graphify reference: transcribe video and audio
- .claude/CLAUDE.md
- extraction-spec.md

## God Nodes (most connected - your core abstractions)
1. `default_plant_params()` - 33 edges
2. `_run_nominal_closed_loop()` - 24 edges
3. `PlantParams` - 20 edges
4. `f()` - 19 edges
5. `Echo Balancer — Simulation Spec` - 18 edges
6. `linearize_planar()` - 17 edges
7. `rk4_step()` - 16 edges
8. `simulate()` - 16 edges
9. `default_sensor_params()` - 16 edges
10. `design_lqr_balance()` - 14 edges

## Surprising Connections (you probably didn't know these)
- `What currently works (verified: 34/34 tests passing, `uv run pytest -q`)` --references--> `update()`  [INFERRED]
  project-context/CURRENT_STATE.md → sim/estimator.py
- `Unknowns (genuinely open, not yet investigated)` --references--> `SensorParams`  [INFERRED]
  project-context/OPEN_PROBLEMS.md → sim/params.py
- `What a new model must understand before touching code` --references--> `EstimatorParams`  [INFERRED]
  project-context/PROJECT_CONTEXT.md → sim/params.py
- `Task 4: Section 11 NIS consistency test (nominal 60s closed-loop run)` --references--> `design_lqr_balance()`  [INFERRED]
  docs/superpowers/plans/2026-09-20-sensors-estimator-nis-step3.md → sim/control.py
- `Key components (built so far)` --references--> `discretize()`  [INFERRED]
  project-context/PROJECT_CONTEXT.md → sim/estimator.py

## Import Cycles
- None detected.

## Communities (25 total, 7 thin omitted)

### Community 0 - "test_estimator.py"
Cohesion: 0.12
Nodes (31): scipy_stats, discretize(), initial_covariance(), KalmanState, measurement_matrix(), measurement_noise(), predict(), process_noise() (+23 more)

### Community 1 - "PlantParams"
Cohesion: 0.09
Nodes (36): Design decision: Q/R weights are parameters, not magic numbers, Self-review notes, Kalman filter design (implemented in Task 3), numpy, Current implementation details worth knowing, Decision: LQR Q/R weights live in `sim/params.py` as `LQRBalanceParams`, scipy_linalg, design_lqr_balance() (+28 more)

### Community 2 - "f"
Cohesion: 0.13
Nodes (21): dataclasses, Design decision: the energy-conservation test needs the back-EMF term zeroed too, Echo Balancer — Section 14 Step 1: Plant Model Implementation Plan, Open item to flag before/while executing, Task 1: uv project scaffolding, Task 2: `sim/params.py` — plant parameters, Task 3: `sim/plant.py` — nonlinear dynamics, Task 4: `sim/integrate.py` — fixed-step RK4 (+13 more)

### Community 3 - "rk4_step"
Cohesion: 0.13
Nodes (18): Self-review notes, DynamicsFn, Input, InputFn, ndarray, Fixed-step RK4 integration with zero-order-hold control (CLAUDE.md section 5)., One fixed-step RK4 update. `u` is held constant across the step (ZOH)., Integrate `f` for `n_steps` of size `dt`, sampling `u_fn(t, x)` once per step.… (+10 more)

### Community 4 - "default_plant_params"
Cohesion: 0.12
Nodes (32): Task 4: Section 11 NIS consistency test (nominal 60s closed-loop run), math, Key components (built so far), default_estimator_params(), default_plant_params(), default_sensor_params(), Physical parameters for the Echo Balancer plant. All numeric values from…, CLAUDE.md section 7 starting-point values. (+24 more)

### Community 5 - "EstimatorParams"
Cohesion: 0.08
Nodes (24): Design decision: the NIS test's acceptance interval is chi2(3)'s own 95% interval, not the tighter "mean of 12000 i.i.d. samples" interval, Echo Balancer — Section 14 Step 3: Sensors, Estimator, NIS Test Implementation Plan, Pre-verified design: sensor models, KF tuning, and the NIS test's acceptance bounds, Self-review notes, Sensor models (implemented in Task 2), Task 1: `sim/params.py` — sensor and estimator parameters, Task 2: `sim/sensors.py` — gyro, accelerometer, encoder, Task 3: `sim/estimator.py` — Kalman filter core (+16 more)

### Community 9 - "What You Must Do When Invoked"
Cohesion: 0.08
Nodes (24): For /graphify add and --watch, For /graphify query, For the commit hook and native CLAUDE.md integration, For --update and --cluster-only, /graphify, Honesty Rules, Interpreter guard for subcommands, Part A - Structural extraction for code files (+16 more)

### Community 10 - "Current State — Echo Balancer"
Cohesion: 0.13
Nodes (15): Current configuration, Current objective, Current State — Echo Balancer, Immediate next steps, Uncommitted / unusual repo state as of this session, What currently works (verified: 34/34 tests passing, `uv run pytest -q`), What does not exist yet (build order steps 4-8, not started), Failed Approaches — Echo Balancer (+7 more)

### Community 11 - "Echo Balancer — Simulation Spec"
Cohesion: 0.10
Nodes (20): 10. Gate, 11. Required tests (tests/), 12. Evaluation, 13. Quantum (offline threshold selection), 14. Build order and milestones, 15. Key references, 16. Persistent project knowledge (read before scanning the repo), 1. The claim (what the project is) (+12 more)

### Community 12 - "Model Handoff — Echo Balancer"
Cohesion: 0.18
Nodes (11): Architecture you need to know, Constraints the next model must respect, Current problems, Current state in one paragraph, Exact config/parameters in force right now, Failed approaches — do not repeat, Important files, Model Handoff — Echo Balancer (+3 more)

### Community 13 - "graphify reference: extra exports and benchmark"
Cohesion: 0.22
Nodes (8): graphify reference: extra exports and benchmark, Step 6b - Wiki (only if --wiki flag), Step 7 - Neo4j export (only if --neo4j or --neo4j-push flag), Step 7a - FalkorDB export (only if --falkordb or --falkordb-push flag), Step 7b - SVG export (only if --svg flag), Step 7c - GraphML export (only if --graphml flag), Step 7d - MCP server (only if --mcp flag), Step 8 - Token reduction benchmark (only if total_words > 5000)

### Community 14 - "Project Context — Echo Balancer"
Cohesion: 0.22
Nodes (9): Architecture (planned, per `CLAUDE.md` §4), Constraints (from `CLAUDE.md`, non-negotiable unless the user changes the spec), External / official resources, Important files, Project Context — Echo Balancer, Project-specific terminology, Purpose / problem statement, Technologies (+1 more)

### Community 15 - "Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan"
Cohesion: 0.25
Nodes (7): Design decision: the recovery test uses per-step feedback, not a 200 Hz ZOH loop, Echo Balancer — Section 14 Step 2: Linearize, LQR, Recovery Test Implementation Plan, Pre-verified math, Task 1: Add scipy dependency, Task 2: `sim/linearize.py` — analytic + numeric-verified Jacobian, Task 3: `sim/params.py` — LQR balance weights, Task 4: `sim/control.py` — balance LQR design, pole test, nonlinear recovery test

### Community 16 - "Decisions — Echo Balancer"
Cohesion: 0.25
Nodes (8): Decision: back-EMF (`Kb`) must be zeroed too for the energy-conservation test, Decision: encoder quantizes per-wheel, then averages (not average-then-quantize), Decision: Grover Adaptive Search makes no speedup claim, Decision: NIS-consistency test acceptance band is χ²(3)'s own 95% interval, not the tighter i.i.d.-mean interval, Decision: `qiskit`/`qiskit-optimization` not added to `pyproject.toml` yet, Decision: recovery test uses per-plant-step feedback, not the 200Hz ZOH loop, Decision: this session — Graphify installed project-scoped, git hooks not installed, Decisions — Echo Balancer

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
- **114 isolated node(s):** `echo-balancer`, `graphify`, `Usage`, `What graphify is for`, `Step 0 - GitHub repos and multi-path merge (only if a URL or several paths)` (+109 more)
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 187 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **7 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `EstimatorParams` connect `EstimatorParams` to `test_estimator.py`, `PlantParams`, `default_plant_params`, `Project Context — Echo Balancer`?**
  _High betweenness centrality (0.127) - this node is a cross-community bridge._
- **Why does `Project Context — Echo Balancer` connect `Project Context — Echo Balancer` to `Current State — Echo Balancer`, `default_plant_params`?**
  _High betweenness centrality (0.106) - this node is a cross-community bridge._
- **Why does `Echo Balancer — Simulation Spec` connect `Echo Balancer — Simulation Spec` to `Current State — Echo Balancer`?**
  _High betweenness centrality (0.090) - this node is a cross-community bridge._
- **Are the 4 inferred relationships involving `default_plant_params()` (e.g. with `Task 3: `sim/params.py` — LQR balance weights` and `Design decision: the energy-conservation test needs the back-EMF term zeroed too`) actually correct?**
  _`default_plant_params()` has 4 INFERRED edges - model-reasoned connections that need verification._
- **Are the 12 inferred relationships involving `PlantParams` (e.g. with `Task 3: `sim/params.py` — LQR balance weights` and `Echo Balancer — Section 14 Step 1: Plant Model Implementation Plan`) actually correct?**
  _`PlantParams` has 12 INFERRED edges - model-reasoned connections that need verification._
- **Are the 3 inferred relationships involving `f()` (e.g. with `Design decision: the energy-conservation test needs the back-EMF term zeroed too` and `Task 5: Section 11 plant tests (energy conservation, open-loop fall)`) actually correct?**
  _`f()` has 3 INFERRED edges - model-reasoned connections that need verification._
- **What connects `echo-balancer`, `graphify`, `Usage` to the rest of the system?**
  _114 weakly-connected nodes found - possible documentation gaps or missing edges._