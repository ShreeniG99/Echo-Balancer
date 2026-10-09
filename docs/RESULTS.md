# Echo Balancer — results (simulation v0, NXTway-GS parameters)

All numbers come from code in this repository and can be reproduced from `(config, seed)`.
The parameters are the NXTway-GS **simulation v0** set (CLAUDE.md 6.1), not measured hardware (6.2).
Per CLAUDE.md 6.2, any paper reporting these numbers must say so.

**Default NIS gate (since 2026-10-09):**
- Long window: N = 200, τ1 = 1300, τ2 = 1800, T_dwell = 0.5 s.
- Short-window spike detector: 10 samples, threshold 125. It triggers CAUTIOUS only.
- Fallback outside NORMAL: KF Q ×100, and HALT uses the softer gains.

The pre-2026-10-09 gate is `long_window_only_gate_params()`.

Reproduce:
```
PYTHONPATH=. uv run python -m experiments.run_batch          # 3 controllers x 7 scenarios x 10 seeds (~4 min, 4 cores)
PYTHONPATH=. uv run python -m quantum.cost_table             # 60 candidates x 10 seeds re-simulated (~85 min)
PYTHONPATH=. uv run python -m quantum.qubo                   # exhaustive vs GroverOptimizer
PYTHONPATH=. uv run python -m experiments.short_window_eval  # default vs long-window-only, held-out seeds
PYTHONPATH=. uv run python -m experiments.heldout_fallback   # fallback on unseen seeds / payloads
PYTHONPATH=. uv run python -m analysis.plots                 # figures + metrics CSVs
PYTHONPATH=. uv run python -m analysis.animate               # docs/figures/animation_*.mp4
```

## 1. Controller comparison (CLAUDE.md section 12)

Setup:
- Balance-only episodes: 7 scenarios × 10 seeds (0–9), 20 s each (nominal: 60 s).
- Corridor (wall-following) nominal: 10 seeds × 60 s.
- Source: `docs/results/metrics_per_scenario.csv`, `docs/results/metrics_summary.csv`, `docs/figures/detection_and_falls.png`.
- The long-window-only column is the same batch before the short window became the default.

| Scenario | Naive: falls | Tilt gate: detected (median delay) / falls | NIS gate, long window only: detected (median delay) | **NIS gate, default: detected (median delay) / falls** |
|---|---|---|---|---|
| accel noise ×5 | 0/10 | **0/10** / 0 | 10/10 (0.14 s) | **10/10 (0.03 s)** / 0 |
| gyro bias step | 0/10 | 10/10 (0.78 s) / 0 | 10/10 (0.47 s) | **10/10 (0.005 s)** / 0 |
| payload shift | **5/10** | 10/10 (0.38 s) / **0** | 10/10 (0.36 s) | **10/10 (0.23 s)** / **0** |
| surface change | 0/10 | 2/10 (5.2 s) / 0 | 8/10 (1.5 s) | 8/10 (1.3 s) / 0 |
| push (0.1 rad/s) | 0/10 | 0/10 / 0 | 3/10 (0.14 s) | **10/10 (0.00 s)** / 0 |
| battery droop (+ companion push) | 0/10 | 0/10 / 0 | **0/10** | **10/10 (0.005 s)** / 0 |
| nominal (false fallback) | – | 0 % of time | 0 % | **0 %** |
| corridor nominal: falls / progress | 0/10, 0.72 m | 0/10, 0.72 m | 0/10, 0.72 m | 0/10, 0.72 m |

Findings:
- **The NIS gate catches faults that the tilt baseline cannot see.**
  - Accel noise: 10/10 vs 0/10.
  - Push: 10/10 vs 0/10.
  - Battery droop (via its companion push): 10/10 vs 0/10.
  - Gyro bias is caught about 150× faster (0.005 s vs 0.78 s).
- **The short window matters.**
  - The 1 s window alone missed most pushes (3/10) and every battery-droop event (0/10). Their ~50 ms innovation spike is diluted below τ1.
  - The 50 ms window catches all of them, with no extra false alarms in any nominal run.
- **Detecting a fault only helps if the mode changes what the robot does.**
  - The payload shift destabilises the estimate-based loop, because the KF keeps the nominal model by design.
  - The fallback (Q inflated ×100, softer HALT gains) takes payload falls from 5/10 to 0/10.
  - Naive never leaves NORMAL, so it still falls 5/10.
  - Cost: during the accel-noise fault, peak tilt rises from about 0.1° to about 1°.
- **Limits.**
  - Surface change is still missed in 2/10.
  - "Battery droop" is only caught through the small push the scenario includes in order to be observable. A pure voltage sag is invisible to this filter.

## 2. Threshold selection: exhaustive search vs Grover Adaptive Search (CLAUDE.md section 13)

**Which cost is official.** The spec J (CLAUDE.md §13) is the official selection cost. J_detect is reported next to it as a clearly labelled extra and is *not* the spec. See `project-context/DECISIONS.md`, 2026-10-09.

### Cost table (10 seeds, default gate structure)
- **Grid:** τ1/N ∈ {3.5, 4.5, 5.0, 6.5}, τ2/N ∈ {5.5, 7, 9, 14}, N ∈ {100, 200}, T_dwell ∈ {0.25, 0.5} s. That is 64 codes; 4 are infeasible (τ1 ≥ τ2).
- **The short window is part of every candidate,** since it is part of the gate, not a grid variable.
- **Re-simulation:** each of the 60 feasible candidates is re-simulated on 6 disturbance scenarios, nominal balance (60 s) and nominal corridor (60 s), with 10 seeds each. That is 4,810 episodes in 87 min on 4 cores.
- **Cost:** J = 10·fall_rate + 1·false_fallback + 1·(1 − progress), as specified.
- **Source:** `docs/results/qubo_cost_table.csv`, `docs/figures/threshold_sensitivity.png`.
- **Earlier tables kept:**
  - `qubo_cost_table_3seeds.csv`: 3 seeds, long window only.
  - `qubo_cost_table_grid_v1.csv`: the flat first grid.
  - The long-window-only 10-seed table is in git history (`master` at d31feec).

What the table shows:
- **Progress is flat.** It is 1.0 for every candidate, because nominal corridor runs never false-alarm.
- **Falls appear only at the most sensitive setting.** τ1/N = 3.5 with N = 100 (8 candidates, 640 episodes) had 5 payload-shift falls; every other candidate had none.
  - Mechanism, checked on candidate 40, seed 7: false alarms before the shift make the gate flip between NORMAL and CAUTIOUS. It drops back to NORMAL mid-disturbance, snapping the KF back to its nominal noise, then trips HALT, and the robot falls.
  - With the fallback off, that seed survives.
  - So the fallback is only safe with a gate that doesn't chatter, and the cost table penalises the settings that do.
- **J is dominated by false fallbacks.** They reach up to 10 % of nominal time at τ1/N = 3.5. The short window adds none.
  - **20 of 60 candidates tie at J = 0:** τ1/N ≥ 5.0 at N = 200, and τ1/N = 6.5 at N = 100. The current default is one of them.
- **Detection is now high everywhere.** It is 97–100 % across all 60 candidates; with the long window alone it was 68–100 %.
  - So the spec J's blind spot (it doesn't reward detection) no longer hides lazy alarms. Every tied optimum detects at least 96.7 %.
  - The default detects 96.7 %. Its misses are 2 surface-change seeds.
- **The spec J and the variant now agree.**
  - The labelled variant **J_detect = J + 1·(1 − detection_rate)** has 8 optima: τ1/N = 5.0, N = 200, any τ2 and T_dwell. They give detection 100 % and false fallback 0.
  - All 8 are inside the spec J's optimal set.
  - With the long window alone, the two costs disagreed: J_detect then preferred τ1/N = 4.5 at N = 100.
- **Grid history.** The first grid used τ1/N ≥ 5.0. After the dropout fix, every candidate scored J = 0, so the grid was widened downward into the nominal ε range (mean NIS 2.89, worst 60 s window 4.0–4.7 at N = 200). This choice was made after seeing the data.

### QUBO formulation (`quantum/formulation.py`)
1. **Binary index.** The candidate index is encoded in 6 binary variables, little-endian. Bits 0–1 select the τ1 level, bits 2–3 the τ2 level, bit 4 N, bit 5 T_dwell.
2. **Rank transform.** J is mapped to dense integer ranks. The transform is monotone, so the argmin is unchanged, and the coefficients stay small integers so Grover's value register stays small. Infeasible codes get rank = worst feasible + 1, which is the penalty term.
3. **Exact HUBO.** The Möbius transform of the rank table gives the exact multilinear polynomial. For a binary index this is generally degree n, not quadratic.
4. **Exact reduction to a QUBO.** Rosenberg substitution (y = x_i·x_j plus a penalty) does the reduction, with a tight per-auxiliary penalty.
5. **Check.** Every QUBO is brute-force verified to decode to an argmin of J (`verify_exact`), and tested on random 3–6-bit tables (`tests/test_qubo.py`).

### Grover vs exhaustive
- **Setup:** `GroverOptimizer` (qiskit 2.5.2, qiskit-optimization 0.7.0, qiskit-aer 0.17.2), Aer `SamplerV2` + preset pass manager, 5 independent seeds per instance, `num_iterations` = 8.
- **Qubit limit:** Grover is skipped above `QuboParams.max_simulated_qubits` = 24.
- **Source:** `docs/results/qubo_summary.csv`, `docs/results/qubo_report.json`.

| Instance | Cost | Candidates (feasible) | Optimal set | QUBO vars + value = **qubits** | Exhaustive | Grover: same optimum | Grover oracle calls (mean) | Grover wall-clock (mean) |
|---|---|---|---|---|---|---|---|---|
| full grid, 6 bits | spec J | 64 (60) | 20 | 18 + 11 = **29** | 60 lookups, <1 ms | not run (29 qubits) | – | – |
| τ1×τ2×N, 5 bits (T_dwell = 0.5) | spec J | 32 (30) | 10 | 7 + 9 = **16** | 30, <1 ms | **5/5** | 18.0 | 2.3 s |
| τ1×τ2, 4 bits (N = 200, T_dwell = 0.5) | spec J | 16 (15) | 7 | 6 + 9 = **15** | 15, <1 ms | **5/5** | 16.0 | 1.4 s |
| full grid, 6 bits | J_detect (variant) | 64 (60) | 8 | 18 + 11 = **29** | 60, <1 ms | not run (29 qubits) | – | – |
| τ1×τ2×N, 5 bits | J_detect (variant) | 32 (30) | 4 | 8 + 9 = **17** | 30, <1 ms | **5/5** | 19.8 | 4.4 s |
| τ1×τ2, 4 bits | J_detect (variant) | 16 (15) | 4 | 6 + 8 = **14** | 15, <1 ms | **5/5** | 21.0 | 1.4 s |

- **Exactness:** every QUBO, including the two full-grid ones too big to simulate, was brute-force verified to decode to an optimal candidate.
- **Qubit count depends on the table's structure.** The 3-seed table was regular enough that the full grid needed only 19–20 qubits. The 10-seed tables need 29.
- **An earlier, smaller result.** On the 3-seed full J_detect grid (20 qubits), Grover hit the optimum in only 2/5 runs (`qubo_summary_3seeds.csv`); the misses were 2nd/3rd-best.

Reading this honestly:
- **Same optimum: yes on every instance that could be simulated** (20/20 runs).
  - These instances are small (14–17 qubits) with 4–10 optimal codes, so adaptive search finds one quickly.
  - On the largest instance it could ever run (20 qubits, 4 optima), it missed 3/5 times.
- **The classical argmin is trivial.** Once the cost table exists it takes microseconds. The expensive part is building the table, about 85 min of re-simulation, which *both* methods need.
- **No speedup is claimed or observed.** Grover here runs on a classical simulator. Its wall-clock time is seconds against microseconds, and the full 64-candidate problem does not fit the simulator.
- **The contribution is the formulation.** Safe-switching-policy selection is written as an exact QUBO over a binary-encoded candidate index, with infeasible threshold orderings penalised, and verified exact.

## 3. Firmware (ESP32-S3, Wokwi)

`firmware/` runs the same Kalman filter, NIS, gate (now including the short-window detector) and tilt gate at 200 Hz on an ESP32-S3, against a virtual linear robot. Every constant is generated from `sim/params.py`.

Host checks (`tests/test_firmware_core.py`):
- **Parity with Python:** C++ matches the Python filter and gates to rtol 1e-6, with identical modes. This now covers the short window.
- **Nominal:** no mode change over 60 s; mean NIS about 3.0.
- **Payload shift:** detected and survived on 10/10 seeds with the same fallback.

The user confirmed the Wokwi build runs, before the short window was added. In Wokwi, the `p` (push) key should now turn the LED yellow. Details: `firmware/README.md`, `firmware/WOKWI_GUIDE.md`.

## 4. Held-out checks (data never used for tuning)

### Fallback on unseen seeds and payloads
- **Why:** the ×100 setting was chosen on the default payload shift with seeds 0–9, so the section 1 payload result is in-sample.
- **Setup:** seeds 100–119, three payload sizes, default gate. Source: `experiments/heldout_fallback.py`, `docs/results/fallback_heldout.csv`.

| Payload (ΔM, ΔL) | Falls, fallback off | **Falls, fallback on** | Detected (median delay) | Median worst tilt, fallback on |
|---|---|---|---|---|
| +0.5 kg, +5 cm (lighter, unseen) | 0/20 | 0/20 | 0/20 | 2.0° (the starting tilt only) |
| +1.0 kg, +10 cm (tuning case, unseen seeds) | 13/20 | **0/20** | 20/20 (0.28 s) | 8.4° |
| +1.5 kg, +15 cm (heavier, unseen) | 20/20 | **0/20** | 20/20 (0.18 s) | 10.9° |

The fallback generalises to the heavier, never-seen payload. The light payload is never detected, but it is harmless.

### Short-window detector on unseen seeds
- **Calibration:** on seeds 0–9.
- **Evaluation:** seeds 100–109, default gate against the long-window-only gate (`experiments/short_window_eval.py`, `docs/results/short_window_heldout.csv`).

| Held-out scenario | Long window only: caught, median delay | **Default (with short window): caught, median delay** |
|---|---|---|
| push | 7/10, 0.72 s | **10/10, 0.00 s** |
| battery droop (+ companion push) | **0/10** | **10/10, 0.008 s** |
| gyro bias | 10/10, 0.49 s | 10/10, 0.005 s |
| payload shift | 10/10, 0.40 s | 10/10, 0.26 s |
| surface change | 8/10, 1.87 s | 9/10, 1.64 s |
| accel noise | 10/10, 0.13 s | 10/10, 0.015 s |
| nominal balance + corridor (false alarms, 10 × 60 s each) | 0 | **0** |
| falls (all scenarios) | 0 | 0 |

**Caveats:**
- Push detection with the long window alone is very seed-dependent: 3/10 on seeds 0–9 and 7/10 on seeds 100–109.
- With only 10 held-out seeds, a false-alarm rate below about 1 in 1,200 s cannot be ruled out.

## 5. Bugs found during this evaluation (fixed)

- **Nominal corridor runs fell in 2/10 (naive) to 5/10 (tilt gate) seeds.**
  - Cause: an ultrasonic dropout returns the 4 m maximum. That steps the wall error by about 3.5 m, the wall-following D-term saturates the motors at ±7.4 V, and the heading kicks accumulate until the robot faces a wall.
  - Fixed by holding the last valid side reading (`sim.sensors.hold_on_dropout`).
  - It also removed the tilt gate's 17.6 % corridor false-fallback rate.
- **The first cost table counted falls only in disturbance episodes.** It now counts them over the whole evaluation set.
- **`quantum/qubo.py` crashed on instances too large for the simulator.** It now reports them as not run.

## 6. Limitations (read before citing any number)
- **Simulation only.** NXTway-GS parameters; our robot's values are still placeholders (`sim.params.hardware_v1_param_set`, `docs/HARDWARE_CHECKLIST.md`).
- **In-sample tuning, partly corrected.**
  - The fallback's ×100 and the short-window threshold were tuned on seeds 0–9.
  - Both were re-checked on held-out seeds (section 4), but only for the scenarios defined here.
  - The τ1/τ2 defaults were calibrated in step 4 on the same nominal runs used for evaluation.
- **Small samples.** 10 seeds per candidate and per scenario; 10–20 held-out seeds.
- **The grid was changed after seeing results** (v1 was flat). Both versions are kept.
- **The spec J does not reward detection.** With the short window, detection is 97–100 % for every candidate, so this no longer changes the answer, but it is still a property of the spec.
- **Fallback plus a chattering gate can cause a fall** (section 2). It is safe only with thresholds above the nominal ε range.
- **Battery droop as a pure voltage sag is not detectable** by any NIS window in this model.
- **The Wokwi build has not been re-run** since the short window was added to the firmware. Only the host C++ tests cover it.

## 7. Not done / open
- Yaw authority is not capped relative to balance.
- Ramping the fallback's Q inflation instead of switching it may remove the chatter-induced fall; untested.
- Surface change is missed in 2/10 seeds by the default gate.
- Hardware parameters and real-robot firmware: see `docs/HARDWARE_CHECKLIST.md`.
- See `project-context/OPEN_PROBLEMS.md`.
