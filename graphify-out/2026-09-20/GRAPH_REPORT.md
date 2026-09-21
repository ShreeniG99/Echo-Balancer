# Graph Report - Echo Balancer  (2026-09-20)

## Corpus Check
- cluster-only mode — file stats not available

## Summary
- 147 nodes · 364 edges · 9 communities (6 shown, 3 thin omitted)
- Extraction: 93% EXTRACTED · 7% INFERRED · 0% AMBIGUOUS · INFERRED: 24 edges (avg confidence: 0.91)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `a7d84061`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- test_estimator.py
- params.py
- default_plant_params
- simulate
- default_sensor_params
- test_params.py
- echo-balancer

## God Nodes (most connected - your core abstractions)
1. `default_plant_params()` - 29 edges
2. `_run_nominal_closed_loop()` - 23 edges
3. `f()` - 16 edges
4. `default_sensor_params()` - 16 edges
5. `PlantParams` - 15 edges
6. `linearize_planar()` - 15 edges
7. `simulate()` - 14 edges
8. `design_lqr_balance()` - 12 edges
9. `default_lqr_balance_params()` - 12 edges
10. `SensorParams` - 11 edges

## Surprising Connections (you probably didn't know these)
- `test_default_plant_params_derived_quantities()` --calls--> `default_plant_params()`  [EXTRACTED]
  tests/test_params.py → sim/params.py
- `test_plant_params_is_frozen()` --calls--> `default_plant_params()`  [EXTRACTED]
  tests/test_params.py → sim/params.py
- `test_default_sensor_params()` --calls--> `default_sensor_params()`  [EXTRACTED]
  tests/test_params.py → sim/params.py
- `test_sensor_params_is_frozen()` --calls--> `default_sensor_params()`  [EXTRACTED]
  tests/test_params.py → sim/params.py
- `test_measurement_matrix_values()` --calls--> `measurement_matrix()`  [EXTRACTED]
  tests/test_estimator.py → sim/estimator.py

## Import Cycles
- None detected.

## Communities (9 total, 3 thin omitted)

### Community 0 - "test_estimator.py"
Cohesion: 0.12
Nodes (30): scipy_stats, discretize(), initial_covariance(), KalmanState, measurement_matrix(), measurement_noise(), predict(), process_noise() (+22 more)

### Community 1 - "params.py"
Cohesion: 0.16
Nodes (20): dataclasses, numpy, scipy_linalg, design_lqr_balance(), ndarray, Balance controller (CLAUDE.md section 9). This step implements only the balance…, Balance-LQR state-feedback gain K (shape (4,)) on the planar model, such that u…, Discrete linear Kalman filter + NIS for the balance estimator (CLAUDE.md… (+12 more)

### Community 2 - "default_plant_params"
Cohesion: 0.14
Nodes (22): default_plant_params(), Return NXTway-GS values for simulation v0 (CLAUDE.md section 6.1)., f(), ndarray, Section 11: 'LQR closed-loop poles all Re < 0; recovers from psi0 = 10 deg in…, test_recovers_from_10_degree_pitch_in_nonlinear_sim(), _numeric_jacobian(), B's v_l and v_r columns must be identical in the planar rows: F_theta and F_psi… (+14 more)

### Community 3 - "simulate"
Cohesion: 0.14
Nodes (17): DynamicsFn, Input, InputFn, ndarray, Fixed-step RK4 integration with zero-order-hold control (CLAUDE.md section 5)., One fixed-step RK4 update. `u` is held constant across the step (ZOH)., Integrate `f` for `n_steps` of size `dt`, sampling `u_fn(t, x)` once per step.…, rk4_step() (+9 more)

### Community 4 - "default_sensor_params"
Cohesion: 0.20
Nodes (18): default_sensor_params(), CLAUDE.md section 7 starting-point values., Sensor noise model (CLAUDE.md section 7). All values are starting points, not…, SensorParams, accelerometer(), encoder(), gyro(), Sensor models (CLAUDE.md section 7). Gyro, accelerometer, and encoder -- the… (+10 more)

### Community 5 - "test_params.py"
Cohesion: 0.15
Nodes (17): math, default_estimator_params(), default_lqr_balance_params(), EstimatorParams, LQRBalanceParams, Balance-KF tuning for the 5-state model [theta, psi, theta_dot, psi_dot, b_g]:…, Balance-LQR state weights, on the planar model [theta, psi, theta_dot,…, CLAUDE.md section 6.1: Q=diag(1, 1e3, 1, 1), R=1e2. (+9 more)

## Knowledge Gaps
- **1 isolated node(s):** `echo-balancer`
  These have ≤1 connection - possible missing edges or undocumented components. (Counts symbols only; 60 node(s) total have ≤1 connection when file, concept and rationale nodes are included.)
- **3 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `default_plant_params()` connect `default_plant_params` to `test_estimator.py`, `params.py`, `default_sensor_params`, `test_params.py`?**
  _High betweenness centrality (0.185) - this node is a cross-community bridge._
- **Why does `_run_nominal_closed_loop()` connect `test_estimator.py` to `params.py`, `default_plant_params`, `simulate`, `default_sensor_params`, `test_params.py`?**
  _High betweenness centrality (0.129) - this node is a cross-community bridge._
- **Why does `rk4_step()` connect `simulate` to `test_estimator.py`?**
  _High betweenness centrality (0.101) - this node is a cross-community bridge._
- **Are the 7 inferred relationships involving `PlantParams` (e.g. with `design_lqr_balance()` and `discretize()`) actually correct?**
  _`PlantParams` has 7 INFERRED edges - model-reasoned connections that need verification._
- **What connects `echo-balancer` to the rest of the system?**
  _1 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `test_estimator.py` be split into smaller, more focused modules?**
  _Cohesion score 0.11742424242424243 - nodes in this community are weakly interconnected._
- **Should `default_plant_params` be split into smaller, more focused modules?**
  _Cohesion score 0.13538461538461538 - nodes in this community are weakly interconnected._