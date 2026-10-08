# Echo Balancer — results (simulation v0, NXTway-GS parameters)

All numbers come from code in this repository and can be reproduced from `(config, seed)`.
The parameters are the NXTway-GS **simulation v0** set (CLAUDE.md 6.1), not measured hardware (6.2).
Per CLAUDE.md 6.2, any paper reporting these numbers must say so.

Reproduce:
```
PYTHONPATH=. uv run python -m experiments.run_batch   # 3 controllers x 7 scenarios x 10 seeds (~3 min, 4 cores)
PYTHONPATH=. uv run python -m quantum.cost_table      # 60 candidates re-simulated (~15 min)
PYTHONPATH=. uv run python -m quantum.qubo            # exhaustive vs GroverOptimizer
PYTHONPATH=. uv run python -m analysis.plots          # figures + metrics CSVs
PYTHONPATH=. uv run python -m analysis.animate        # docs/figures/animation_*.mp4
```

## 1. Controller comparison (CLAUDE.md section 12)

Setup:
- Balance-only episodes: 7 scenarios × 10 seeds, 20 s each (nominal: 60 s).
- Corridor (wall-following) nominal: 10 seeds × 60 s.
- Default NIS gate: τ1 = 1300, τ2 = 1800, N = 200, T_dwell = 0.5 s.
- Source: `docs/results/metrics_per_scenario.csv`, `docs/results/metrics_summary.csv`, `docs/figures/detection_and_falls.png`.

| Scenario | Naive: falls | Tilt gate: detected / falls | **NIS gate: detected / falls** | NIS median delay |
|---|---|---|---|---|
| accel noise ×5 | 0/10 | **0**/10 / 0 | **10/10** / 0 | 0.14 s |
| gyro bias step | 0/10 | 10/10 / 0 | 10/10 / 0 | 0.47 s (tilt: 0.78 s) |
| payload shift | **5/10** | 10/10 / **0** | 10/10 / **0** | 0.36 s |
| surface change | 0/10 | 2/10 / 0 | 8/10 / 0 | 1.5 s |
| push (0.1 rad/s) | 0/10 | 0/10 / 0 | 3/10 / 0 | 0.14 s |
| battery droop | 0/10 | 0/10 / 0 | **0/10** / 0 | – |
| nominal (false fallback) | – | 0 % of time | 0 % of time | – |
| corridor nominal: falls / progress | 0/10, 0.72 m | 0/10, 0.72 m | 0/10, 0.72 m | – |

Findings:
- **The NIS gate catches sensor faults that the tilt baseline cannot see.**
  - Accel noise: 10/10 for the NIS gate vs 0/10 for the tilt gate. The robot's tilt barely changes, so a tilt threshold never fires.
  - Gyro bias: the NIS gate is faster, 0.47 s vs 0.78 s.
- **Detecting a fault only helps if the mode changes what the robot does.**
  - The payload shift destabilises the estimate-based loop: the Kalman filter keeps the nominal model by design, while true-state feedback stays stable.
  - Fix: outside NORMAL, the filter's process noise is inflated ×100 and HALT uses the softer gain set (`FallbackParams`).
  - Payload falls drop from 5/10 to 0/10 for both gates. Naive never leaves NORMAL, so it still falls 5/10.
  - Cost: during the accel-noise fault, peak tilt rises from about 0.1° to about 1°.
- **Limits.**
  - Battery droop is never detected.
  - A 0.1 rad/s push is detected 3/10 times; ε peaks around 1150 against τ1 = 1300.
  - Neither caused a fall here.

## 2. Threshold selection: exhaustive search vs Grover Adaptive Search (CLAUDE.md section 13)

### Cost table
- **Grid:** τ1/N ∈ {3.5, 4.5, 5.0, 6.5}, τ2/N ∈ {5.5, 7, 9, 14}, N ∈ {100, 200}, T_dwell ∈ {0.25, 0.5} s. That is 64 codes; 4 are infeasible (τ1 ≥ τ2).
- **Re-simulation:** each of the 60 feasible candidates is re-simulated on 6 disturbance scenarios + nominal balance + nominal corridor, × 3 seeds. That is 1,443 episodes in about 15 min on 4 cores.
- **Cost:** J = 10·fall_rate + 1·false_fallback + 1·(1 − progress), as specified.
- **Source:** `docs/results/qubo_cost_table.csv`, `docs/figures/threshold_sensitivity.png`.

What the table shows:
- **Falls and progress are flat.** fall_rate = 0 and progress = 1.0 for every candidate. With the fallback and the dropout fix, no threshold in the grid causes or prevents a fall, and corridor runs never false-alarm (their ε level is about 3).
- **J therefore reduces to the false-fallback fraction.**
  - Thresholds below the nominal ε range false-alarm: up to 10 % of nominal time at τ1/N = 3.5.
  - **28 of 60 candidates tie at J = 0.** They are every candidate with τ1/N ≥ 5.0, including the current default.
- **The spec J does not reward detection.** The tied optima detect only 72–94 % of disturbance episodes, while τ1/N ≤ 4.5 detects 100 %.
  - Because of that, a clearly labelled variant is also reported: **J_detect = J + 1·(1 − detection_rate)**. It is *not* the spec J.
  - Its optimum is τ1/N = 4.5 at N = 100 (4 tied candidates): detection 100 %, false fallback 1.3 %.
- **Grid history.** The first grid used τ1/N ≥ 5.0 (`docs/results/qubo_cost_table_grid_v1.csv`). After the dropout fix, every v1 candidate scored J = 0, because no level reached the nominal ε range: mean NIS 2.89, worst 60 s window 4.0–4.7 at N = 200. The grid was widened downward into that range.

### QUBO formulation (`quantum/formulation.py`)
1. **Binary index.** The candidate index is encoded in 6 binary variables, little-endian. Bits 0–1 select the τ1 level, bits 2–3 the τ2 level, bit 4 N, bit 5 T_dwell.
2. **Rank transform.** J is mapped to dense integer ranks. The transform is monotone, so the argmin is unchanged, and the coefficients stay small integers so Grover's value register stays small. Infeasible codes get rank = worst feasible + 1, which is the penalty term.
3. **Exact HUBO.** The Möbius transform of the rank table gives the exact multilinear polynomial. For a binary index this is generally degree n, not quadratic.
4. **Exact reduction to a QUBO.** Rosenberg substitution (y = x_i·x_j plus a penalty) does the reduction, with a tight per-auxiliary penalty.
5. **Check.** Every QUBO is brute-force verified to decode to an argmin of J (`verify_exact`), and tested on random 3–6-bit tables (`tests/test_qubo.py`).

### Grover vs exhaustive
- **Setup:** `GroverOptimizer` (qiskit 2.5.2, qiskit-optimization 0.7.0, qiskit-aer 0.17.2), Aer `SamplerV2` + preset pass manager, 5 independent seeds per instance, `num_iterations` = 8.
- **Oracle calls:** the sum of Grover powers over all sampled circuits.
- **Classical evaluations:** table lookups over the feasible codes.
- **Source:** `docs/results/qubo_summary.csv`, `docs/results/qubo_report.json`.

| Instance | Cost | Candidates (feasible) | Optimal set | QUBO vars + value = **qubits** | Exhaustive: evaluations, time | Grover: same optimum | Grover: oracle calls (mean, range) | Grover wall-clock (mean) |
|---|---|---|---|---|---|---|---|---|
| full grid, 6 bits | spec J | 64 (60) | 28 | 9 + 10 = **19** | 60 lookups, <1 ms | **5/5** | 17.8 (9–23) | 8.2 s |
| τ1×τ2×N, 5 bits (T_dwell = 0.5) | spec J | 32 (30) | 14 | 7 + 9 = **16** | 30, <1 ms | **5/5** | 13.8 (8–20) | 1.3 s |
| τ1×τ2, 4 bits (N = 200, T_dwell = 0.5) | spec J | 16 (15) | 7 | 6 + 9 = **15** | 15, <1 ms | **5/5** | 16.2 (12–22) | 1.0 s |
| full grid, 6 bits | J_detect (variant) | 64 (60) | **4** | 10 + 10 = **20** | 60, <1 ms | **2/5** | 19.4 (7–25) | 19.0 s |
| τ1×τ2×N, 5 bits | J_detect (variant) | 32 (30) | **4** | 8 + 9 = **17** | 30, <1 ms | **4/5** | 18.6 (7–26) | 3.2 s |

QUBO exactness: every QUBO was brute-force verified to decode to an optimal candidate (all 5 instances).
Grover misses on J_detect were near-misses. It returned the 2nd- or 3rd-best of 10 cost levels: J_detect 0.0135 or 0.0499 against an optimum of 0.0115 on the full grid, and 0.0560 against 0.0135 on the 5-bit instance.
Qubit counts are for the structured real tables. A random 6-bit cost table needs about 18 QUBO variables and about 34 qubits, which is not simulable here.

Reading this table honestly:
- **Same optimum: yes on the spec J, not reliably on the variant.** On the spec J, Grover reached an optimal candidate in 15/15 runs. Those instances are easy, because 7–28 of the codes are optimal and adaptive search terminates quickly when many states are marked.
- **With a small optimal set, `GroverOptimizer` misses.** On the detection-weighted variant, where only 4 of 64 codes are optimal, it succeeded in 6/10 runs. Its stopping rule (8 iterations without improvement) ends some runs early. Exhaustive search is exact by construction.
- **The classical argmin is trivial.** Once the cost table exists it takes microseconds. The expensive part is building the table: about 15 min of re-simulation, needed by *both* methods.
- **No speedup is claimed or observed.** Grover here runs on a classical simulator, its wall-clock time is seconds against microseconds, and its oracle calls are not comparable to free table lookups.
- **The contribution is the formulation.** Safe-switching-policy selection is written as an exact QUBO over a binary-encoded candidate index, with infeasible threshold orderings penalised, and verified exact.

## 3. Firmware (ESP32-S3, Wokwi)

`firmware/` runs the same Kalman filter, NIS and both gates at 200 Hz on an ESP32-S3 against a virtual linear robot. Every constant is generated from `sim/params.py`.

Host checks (`tests/test_firmware_core.py`):
- **Parity with Python:** C++ matches the Python filter and gates to rtol 1e-6, with identical modes.
- **Nominal:** no mode change over 60 s; mean NIS about 3.0.
- **Payload shift:** detected and survived on 10/10 seeds with the same fallback.

The user confirmed the Wokwi build runs. Details: `firmware/README.md`, `firmware/WOKWI_GUIDE.md`.

## 4. Bugs found during this evaluation (fixed)

- **Nominal corridor runs fell in 2/10 (naive) to 5/10 (tilt gate) seeds.**
  - Cause: an ultrasonic dropout returns the 4 m maximum. That steps the wall error by about 3.5 m, the wall-following D-term saturates the motors at ±7.4 V, and the heading kicks accumulate until the robot faces a wall.
  - Fixed by holding the last valid side reading (`sim.sensors.hold_on_dropout`).
  - It also removed the tilt gate's 17.6 % corridor false-fallback rate.
- **The first cost table counted falls only in disturbance episodes.** It now counts them over the whole evaluation set.

## 5. Not done / open
- Yaw authority is not capped relative to balance.
- Battery droop and small pushes are not detected.
- Hardware parameters (CLAUDE.md 6.2) are still placeholders.
- See `project-context/OPEN_PROBLEMS.md`.
