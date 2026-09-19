# Echo Balancer — Simulation Spec

Read this whole file before every task. It is the single source of truth. If code and this file disagree, stop and ask.

## 1. The claim (what the project is)

A two-wheeled self-balancing robot that monitors its own control confidence using the **Kalman-filter normalized innovation squared (NIS, χ²)** and switches between three modes: **Normal → Cautious → Halt**. Thresholds are selected offline; a classical exhaustive/grid search is compared against **Grover Adaptive Search (Qiskit `GroverOptimizer`)** on the same QUBO.

The contribution is the **gate + its evaluation**, not the robot. Grover makes **no speedup claim**. Report it honestly whichever way it lands.

## 2. Scope

In scope: planar+yaw plant, balance controller, sensor models, Kalman filter, NIS gate, disturbance injection, 2D corridor wall-following, batch evaluation, QUBO threshold selection, 2D animation.

**Out of scope, do not build:** cave arena, magnet-polarity detection, CAD, 3D rendering, physics engines (MuJoCo/Gazebo/CoppeliaSim), ROS, GUIs, online learning, neural networks.

## 3. Stack

Python ≥ 3.11, `uv` for environment. `numpy`, `scipy`, `python-control`, `pandas` + `pyarrow` (Parquet), `joblib`, `matplotlib` (+ ffmpeg for MP4), `pytest`, `qiskit` + `qiskit-optimization` (pin a compatible pair in `pyproject.toml`; verify `GroverOptimizer` imports before writing quantum code).

Rules:
- All parameters live in `sim/params.py` as frozen dataclasses. **No magic numbers anywhere else.**
- Every stochastic function takes an explicit `np.random.Generator`. Every run is reproducible from `(config, seed)`.
- Pure functions where possible; no global state.
- Type hints and SI units everywhere. Angles in radians internally; degrees only in plots/logs labelled `_deg`.
- Each module ships with its tests (section 11). A module is not done until its tests pass.
- Do not "fix" a failing physics test by loosening its tolerance. Find the bug.

## 4. Repository layout

```
sim/params.py        # all physical/sensor/controller/gate parameters
sim/plant.py         # nonlinear dynamics f(x, v_l, v_r, disturbances)
sim/integrate.py     # fixed-step RK4, zero-order-hold control
sim/linearize.py     # A, B at upright equilibrium (analytic + numeric check)
sim/sensors.py       # gyro, accelerometer, encoders, ultrasonic
sim/control.py       # LQR balance, speed/yaw servo, wall-following
sim/estimator.py     # Kalman filter + NIS
sim/gate.py          # Normal/Cautious/Halt state machine
sim/disturbances.py  # scheduled disturbance profiles
sim/world.py         # 2D corridor geometry + ray casting
sim/run.py           # one closed-loop episode -> DataFrame
experiments/run_batch.py
analysis/metrics.py
analysis/plots.py
analysis/animate.py
quantum/qubo.py      # cost table -> QUBO; grid vs GroverOptimizer
tests/
```

## 5. Plant model (base: Yamamoto, "NXTway-GS Model-Based Design", MathWorks, 2008)

Generalized coordinates: θ = mean wheel angle, ψ = body pitch (0 = upright, + = leaning forward), φ = yaw. θ_l, θ_r = left/right wheel angles; θ = (θ_l + θ_r)/2, φ = (R/W)(θ_r − θ_l).

State: `x = [θ, ψ, φ, θ̇, ψ̇, φ̇]`. Inputs: motor voltages `v_l, v_r`.

Equations of motion:

```
[(2m+M)R² + 2Jw + 2n²Jm] θ̈ + (MLR cosψ − 2n²Jm) ψ̈ − MLR ψ̇² sinψ = F_θ

(MLR cosψ − 2n²Jm) θ̈ + (ML² + Jψ + 2n²Jm) ψ̈ − MgL sinψ − ML² φ̇² sinψ cosψ = F_ψ

[½mW² + Jφ + (W²/2R²)(Jw + n²Jm) + ML² sin²ψ] φ̈ + 2ML² ψ̇ φ̇ sinψ cosψ = F_φ
```

Generalized forces:

```
F_θ = α(v_l + v_r) − 2(β + f_w) θ̇ + 2β ψ̇
F_ψ = −α(v_l + v_r) + 2β θ̇ − 2β ψ̇
F_φ = (W/2R) α (v_r − v_l) − (W²/2R²)(β + f_w) φ̇
α = n K_t / R_m
β = n² K_t K_b / R_m + f_m
```

Notes:
- The θ and ψ equations were checked against an independent Lagrange derivation (sympy). **Transcription status: verified 2026-09-19 against the actual NXTway-GS Model-Based Design PDF (File Exchange #19147, section 3). Equations (3.13)-(3.15) match the θ/ψ/φ equations above term-for-term, and (3.22)-(3.25) match the F_θ/F_ψ/F_φ/α/β forms above (the source writes β without the n² factor — see the next note, already accounted for). The section 6.1 parameter table (R_m, K_b, K_t, n, f_m, f_w) also matches the PDF's section 3.1 values exactly.**
- Yamamoto writes β = nK_tK_b/R_m + f_m. With n = 1 this is identical. For a geared motor where K_b, K_t are motor-side constants, back-EMF torque scales with n², so use n² (above).
- Solve the 2×2 coupled θ/ψ system with `np.linalg.solve` each step. Do not hand-invert.
- Voltage: `v = clip(u_cmd, −V_batt, +V_batt)`. V_batt is a state of the disturbance model (section 8), nominal 7.4 V (2S LiPo), full 8.4 V.
- Fall: `|ψ| > 45°` ends the episode (motors off).

## 6. Parameters

### 6.1 Simulation v0 (NXTway-GS values; replace with measured values when hardware arrives)

| Symbol | Value | Unit | Meaning |
|---|---|---|---|
| g | 9.81 | m/s² | gravity |
| m | 0.03 | kg | wheel mass (each) |
| R | 0.04 | m | wheel radius |
| Jw | mR²/2 | kg·m² | wheel inertia |
| M | 0.6 | kg | body mass |
| W | 0.14 | m | body width (wheel track) |
| D | 0.04 | m | body depth |
| H | 0.144 | m | body height |
| L | H/2 | m | axle → body CoM |
| Jψ | ML²/3 | kg·m² | body pitch inertia |
| Jφ | M(W²+D²)/12 | kg·m² | body yaw inertia |
| Jm | 1e-5 | kg·m² | motor rotor inertia |
| R_m | 6.69 | Ω | armature resistance |
| K_b | 0.468 | V·s/rad | back-EMF constant |
| K_t | 0.317 | N·m/A | torque constant |
| n | 1 | – | gear ratio |
| f_m | 0.0022 | – | body–motor friction |
| f_w | 0 | – | wheel–floor friction |

Sanity values for this set (upright, planar, linearized): open-loop eigenvalues ≈ {0, −241, +7.44, −6.52} rad/s. An LQR with Q = diag(1, 1e3, 1, 1), R = 1e2 on u = v_l+v_r gives K ≈ [−0.10, −49.3, −2.09, −4.55], all closed-loop poles in LHP. Use these as regression checks for `linearize.py`.

### 6.2 Our hardware (fill in when measured; keep both sets selectable)

ESP32, GY-87 (MPU6050 + HMC5883L), 2× encoder gear motors, TB6612FNG driver, 2S 1500 mAh LiPo. Every entry gets a tag: `REAL_SPEC` (datasheet), `MEASURED`, `ESTIMATED`, or `PLACEHOLDER`. The paper may only report results on `REAL_SPEC`/`MEASURED` values or must state otherwise.

## 7. Timing, sensors, noise

- Plant integration: fixed-step RK4, dt_plant = 1 ms.
- Control/estimation loop: 200 Hz (dt = 5 ms), zero-order hold on voltage.
- Gyro: `ψ̇_meas = ψ̇ + b_g + N(0, σ_g²)`, bias b_g random walk. Start σ_g ≈ 0.005 rad/s per sample, bias walk ≈ 1e-4 rad/s/√s.
- Accelerometer tilt: `ψ_acc = atan2(a_x, a_z)` computed from the true specific force (includes the body's linear acceleration, so it is corrupted during motion) + N(0, σ_a²), σ_a ≈ 0.02 rad.
- Encoders measure wheel angle **relative to the body**: `θ_enc = θ − ψ` (per wheel, then averaged), quantized to 2π/CPR. CPR = 360 placeholder.
- Ultrasonic (HC-SR04 model): ray cast in the 2D corridor, 20 Hz, range 0.02–4 m, σ ≈ 3 mm, dropout probability 2 % (returns max range).
- All σ values are starting points; list them in `params.py`.

## 8. Disturbances (each a time-scheduled profile with onset, duration, magnitude)

1. **Surface change:** f_w steps from 0 to a higher value (loose gravel).
2. **Battery droop:** V_batt ramps down (e.g. 8.4 → 6.0 V).
3. **Push:** impulse torque on ψ.
4. **Sensor fault:** gyro bias step, or accelerometer noise ×k.
5. **Payload/CoM shift:** M or L changes at onset (plant only; the estimator keeps the nominal model).

The estimator always uses the **nominal** model. The mismatch is what the innovation is meant to detect.

## 9. Controller and estimator

- **Balance:** LQR on the linearized planar model, states [θ, ψ, θ̇, ψ̇] plus integral of (θ − θ_ref) for speed servo. Yaw: PD on φ̇ with differential voltage.
- **Wall-following:** side ultrasonic distance error → PD → yaw-rate reference. Front ultrasonic below threshold → slow down.
- **Kalman filter:** discrete linear KF on the discretized nominal model (ZOH, dt = 5 ms). States [θ, ψ, θ̇, ψ̇, b_g]. Measurements y = [θ_enc, ψ̇_gyro, ψ_acc] (m = 3).
- **Innovation:** ν_k = y_k − C x̂_k|k−1, S_k = C P C^T + R. NIS_k = ν^T S^{-1} ν ~ χ²(3) when the model matches.
- **Windowed statistic:** ε_k = Σ over the last N samples of NIS ~ χ²(3N). The gate uses ε_k.

## 10. Gate

- Modes: NORMAL (full speed ref), CAUTIOUS (speed ref × 0.4, softer Q in the LQR gain set), HALT (speed ref = 0; **keep balancing in place**, flash/beep flag set). A balancing robot cannot just cut motors. Motors cut only on the fall condition.
- Transitions on ε_k with thresholds τ₁ < τ₂ (as χ²(3N) quantiles), hysteresis (exit threshold lower than entry), and minimum dwell time T_dwell.
- No NORMAL → HALT skip unless ε_k > τ₂ for a full window. Decide this once and keep it fixed.
- Gate parameters to be selected: τ₁, τ₂, N, T_dwell.

## 11. Required tests (tests/)

- Plant, no input, no friction, f_m = 0: total energy conserved to < 0.1 % over 5 s.
- Plant released from ψ = 1°, no control: |ψ| grows, robot falls.
- Numeric Jacobian of `plant.f` at upright equals analytic A, B (rtol 1e-6).
- Linear eigenvalues match section 6.1 sanity values.
- LQR closed-loop poles all Re < 0; recovers from ψ₀ = 10° in the nonlinear sim.
- KF, nominal plant, no disturbances, 60 s: mean NIS within the 95 % interval for χ²(3) averaged over the run; ≈ 5 % of samples above the χ²(3) 95 % quantile.
- Gate: no mode change on a nominal 60 s run (seeded); enters CAUTIOUS within X s of each disturbance at a stated magnitude.
- Same (config, seed) ⇒ identical DataFrame.

## 12. Evaluation

Controllers compared on identical disturbance seeds:
1. **Naive:** always NORMAL.
2. **Tilt-threshold gate:** mode switches on |ψ| thresholds only (the plain safety-filter baseline).
3. **NIS gate (ours).**

Metrics per run (logged to Parquet, one row per episode + a per-step log):
- **Fall rate.**
- **False-fallback:** fraction of disturbance-free time spent outside NORMAL; fallback events per minute in nominal runs.
- **Missed-fallback:** falls where the gate was still NORMAL 0.5 s before the fall.
- **Detection delay:** disturbance onset → first non-NORMAL.
- **Progress:** distance travelled along the corridor (cost of caution).
- **Threshold sensitivity:** metrics vs τ₁, τ₂ sweeps.

## 13. Quantum (offline threshold selection)

1. Discretize candidates: τ₁, τ₂, N, T_dwell on small grids (total ≤ 2¹⁶ combinations).
2. For each candidate, **re-simulate** the evaluation set (the gate changes the robot's behaviour, so log replay is not valid). Compute cost J = w_f·fall_rate + w_s·false_fallback + w_p·(1 − progress).
3. Encode the candidate index in binary variables; build the QUBO whose minimum is the min-J candidate, with penalty terms for infeasible codes (τ₁ ≥ τ₂).
4. Solve with exhaustive search and with `GroverOptimizer` (Aer simulator). Report: same optimum or not, number of oracle calls, wall-clock time, qubit count.
5. State plainly in the write-up: once the cost table is computed, the classical argmin is trivial; the contribution is the QUBO formulation of safe-switching-policy selection, not speed.

## 14. Build order and milestones

1. Verify section 5 against the NXTway-GS PDF → `params.py` → `plant.py` → `integrate.py` → plant tests.
2. `linearize.py` → LQR → recovery test.
3. `sensors.py` → `estimator.py` → NIS test. (Expect Q/R tuning to take the longest.)
4. `gate.py` → `disturbances.py`. **Milestone 1: gate switches under one disturbance.**
5. `world.py` → wall-following.
6. `run.py` → `run_batch.py` → `metrics.py`. **Milestone 2: metrics table for all three controllers.**
7. `qubo.py`. **Milestone 3: grid vs Grover.**
8. `plots.py`, `animate.py` (2D side view + top view, background colour = gate mode).

If behind schedule after milestone 1: cut wall-following first.

## 15. Key references

- Y. Yamamoto, *NXTway-GS Model-Based Design*, MathWorks, 2008 (File Exchange #19147): plant, parameters.
- S. Kim, S. Kwon, "Dynamic Modeling of a Two-wheeled Inverted Pendulum Balancing Mobile Robot," IJCAS 13(4), 2015: derivation cross-check (parameters are for a 41 kg robot, do not use).
- R. K. Mehra, J. Peschon, "An innovations approach to fault detection and diagnosis in dynamic systems," *Automatica*, 1971: NIS/χ² foundation.
- A. Shojaei, "Conformal Recovery-Deadline Certificates for Runtime Assurance of Adapting Controllers," arXiv 2606.25371: closest recent work (pendulum, fallback timing).
- "Cost-Aware Adaptive Conformal Inference for Runtime Assurance in Dynamic Environments," arXiv 2605.24463: risk-scaled caution.
- Gilliam, Woerner, Gonciulea, "Grover Adaptive Search for Constrained Polynomial Binary Optimization," *Quantum* 5:428, 2021: GAS / `GroverOptimizer`.
