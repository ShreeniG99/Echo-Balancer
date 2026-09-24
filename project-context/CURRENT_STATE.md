# Current State — Echo Balancer

Last verified: 2026-09-24, against branch `claude/amazing-thompson-o1hp87` at
commit `197e88f` (`git log --oneline -1`). **Re-verify against `git log` if this
looks stale — this file is a snapshot, not a live view.**

## What currently works (verified: 113/113 tests passing, `uv run pytest -q`)

**All 8 build-order steps (`CLAUDE.md` §14) are now complete.** Steps 1-5
(plant/LQR/estimator/gate/wall-following) were done in prior sessions — see
git history before this session for their detail; this file focuses on
steps 6-8, done in this session.

1-5. Unchanged from the previous snapshot: nonlinear plant, RK4 integration,
   analytic linearization, balance LQR, sensor noise models, 5-state Kalman
   filter with NIS, Normal/Cautious/Halt gate with all 5 §8 disturbance
   profiles, `sim/world.py` corridor + ray casting, full §9 wall-following
   stack (speed-servo LQR, yaw control, wall-following, front-threshold
   slowdown).

6. **`sim/run.py` + `experiments/run_batch.py` + `analysis/metrics.py` —
   Milestone 2 achieved.**
   - `sim/run.py::run_episode(config, seed) -> DataFrame`: the one real
     closed-loop implementation, unifying the plumbing that had been
     duplicated across `tests/test_estimator.py`, `tests/test_gate.py`, and
     `tests/test_wall_following.py`. Parameterized by `ControllerKind`
     (`NAIVE`/`TILT_GATE`/`NIS_GATE`, CLAUDE.md §12's three controllers).
     Balance/speed-servo control now runs on the KF estimate, not true state
     (resolving step 5's documented "pending run.py" simplification). The KF
     always runs regardless of controller, so NIS/epsilon are logged for all
     three, enabling apples-to-apples detection-delay comparison even for
     NAIVE/TILT_GATE.
   - Two real implementation bugs found and fixed during integration (not
     physics, not calibration — see `FAILED_APPROACHES.md`): an instantaneous
     speed-reference step spuriously tripping the NIS gate (fixed with
     `SpeedProfileParams`'s bounded-acceleration ramp), and ultrasonic dropout
     readings (HC-SR04 model, 2% probability, returns exactly `range_max`)
     feeding a huge derivative-term spike into wall-following (fixed by
     holding the previous control output on a detected dropout tick).
   - `GateParams` needed re-calibration for this closed loop specifically —
     see `sim.params.default_evaluation_gate_params` (tau1=900, tau2=1400) vs.
     the original `default_gate_params` (tau1=1300, tau2=1800, still correct
     for `tests/test_gate.py`'s own different, balance-only closed loop). See
     `DECISIONS.md` for why these must stay two separate functions.
   - `experiments/run_batch.py`: 7 scenarios (nominal + the 6 §8 disturbances)
     x 3 controllers x 10 seeds = 210 episodes, parallelized with `joblib`,
     logged to `experiments/results/{episodes,steps}.parquet` (gitignored,
     regenerable, ~150MB for steps.parquet).
   - `analysis/metrics.py`: fall rate, false-fallback, missed-fallback,
     detection delay, progress — computed per controller from each
     controller's own logged `mode` column.
   - **Real result, worth carrying into any write-up:** the NIS gate has
     **zero missed detections** across all 60 disturbance episodes (mean
     delay 0.17s) vs. the tilt-threshold baseline's 50% miss rate, cutting
     fall rate from 42.9% (naive) to 28.6%, at a real but modest progress
     cost (1.62m vs. 1.93m mean). `battery_droop`/`payload_shift` are
     unsurvivable (100% fall rate) regardless of controller — see
     `OPEN_PROBLEMS.md`.

7. **`quantum/qubo.py` — Milestone 3 achieved.**
   - `qiskit`==2.5.2 / `qiskit-optimization`==0.7.0 / `qiskit-aer`==0.17.2
     installed and `GroverOptimizer` import-verified before writing any
     quantum code, per §3's explicit instruction.
   - 4x4x2x2 grid over tau1/tau2/N/T_dwell (64 candidates, 6 bits, well
     under §13's "<=2^16" bound). Each feasible candidate (tau1<tau2) is
     re-simulated (NIS_GATE only, a small/fast scenario-seed set distinct
     from step 6's Milestone-2 batch — see `QuboSearchParams`); infeasible
     candidates get a fixed penalty without simulation.
   - Since `GroverOptimizer` only accepts a linear+quadratic objective but
     the true cost table has no reason to be quadratic in 6 bits, a
     least-squares quadratic surrogate is fit to the true table and handed to
     `GroverOptimizer` — reported explicitly (surrogate R^2, and separately
     whether the surrogate's optimum matches the true table's).
   - `GroverOptimizer` is run `grover_trials` (default 5) independent times,
     best kept — a single run is not authoritative (verified: real
     probabilistic failure behavior, see `DECISIONS.md`/`FAILED_APPROACHES.md`).
   - **Real result:** classical exhaustive search over 64 candidates is a
     ~0.02ms `min()` call (no speedup claim, as designed). More interestingly:
     the production surrogate's true optimum beats its closest rival by only
     ~0.12% of the objective's range, a genuine value-qubit-resolution
     bottleneck for `GroverOptimizer` at a classically-simulable qubit count
     (0/5 and 0/30 hit rates across two production runs) — see
     `OPEN_PROBLEMS.md`.

8. **`analysis/animate.py` — 3D video demo, per the user's explicit
   post-Milestone-3 scope-change request** (see `DECISIONS.md`; `CLAUDE.md`
   §2 updated accordingly).
   - Renders any `sim.run.run_episode` DataFrame to MP4 via PyVista/VTK,
     entirely off-screen: robot body (box) + two wheels (cylinders) posed
     each frame from `[x, y, phi, psi]`, two corridor walls + floor, a path
     trail, a chase camera, and a status light colored by the logged `mode`
     (green=NORMAL, gold=CAUTIOUS, crimson=HALT), plus an on-screen HUD
     (t/mode/epsilon).
   - This container has no GPU/EGL/OSMesa; `render_episode` self-manages a
     virtual X display (`_ensure_display`, spawns `Xvfb` directly) rather
     than requiring every caller to remember `xvfb-run -a`.
   - `analysis/results/echo_balancer_demo.mp4`: a 35s demo episode (seed=3,
     NIS_GATE controller) with a scripted two-act disturbance (a push at
     t=8s triggering a brief CAUTIOUS blip; a surface-change at t=20-30s
     pushing epsilon to ~7.8M and triggering a sustained HALT, followed by
     full recovery) — sent to the user, not committed to git (demo output,
     regenerable via `analysis/animate.py`'s `__main__`).
   - `plots.py` (the other file `CLAUDE.md` §4/§14 step 8 names) was **not**
     built this session — only `animate.py` was in scope for the video
     request. Still open.

Full test run (this session, in isolation — see note on flakiness below):
`113 passed` (~2m45s). Test files added this session: `tests/test_run.py` (7),
`tests/test_run_batch.py` (2), `tests/test_metrics.py` (10), `tests/test_qubo.py`
(6), `tests/test_animate.py` (6) — 31 new tests on top of the 82 that existed
at the start of this session (2 params/gate tests were consolidated into the
new suites, net +31 to 113... re-check exact arithmetic against a fresh
`pytest --collect-only` if this matters, this count wasn't re-verified digit
by digit at write time).

**A note on test flakiness:** one full-suite run this session had
`tests/test_animate.py::test_render_episode_writes_a_nonempty_mp4` fail while
a separate, unrelated heavy `quantum.qubo` background job was consuming all 4
CPU cores concurrently; re-running the full suite in isolation (no concurrent
heavy job) passed cleanly (113/113, twice). Treat a single animate-test
failure under heavy concurrent load as resource contention, not a real bug,
but re-verify in isolation if it recurs.

## What does not exist yet

- `analysis/plots.py` — CLAUDE.md §4/§14 step 8 names this alongside
  `animate.py`; not built (the user's request was specifically for the video).
- Hardware parameter set (`CLAUDE.md` §6.2, `REAL_SPEC`/`MEASURED`/`ESTIMATED`/
  `PLACEHOLDER` tags) — no hardware has arrived yet; only the NXTway-GS §6.1
  values are in use.
- Corridor corners/dead-ends in `sim/world.py` — still two infinite parallel
  walls (unchanged since step 5); `front_threshold_speed_adjust` still has
  nothing to meaningfully trigger it head-on in the closed loop.
- `quantum/qubo.py`'s `threshold_sensitivity()` sweep helper exists but is not
  run by default (documented as more expensive than the base table; call it
  directly if a tau1/tau2 sweep table is wanted for the write-up).

## Current implementation details worth knowing

- **`sim/params.py`** additions this session: `CautiousParams`/
  `default_cautious_speed_servo_params` (CAUTIOUS-mode speed/gain derating),
  `TiltGateParams`/`default_tilt_gate_params` (the tilt-threshold baseline
  controller), `SpeedProfileParams`/`default_speed_profile_params` (the
  bounded-acceleration speed ramp fix), `default_evaluation_gate_params`
  (the recalibrated GateParams for `sim.run`'s closed loop — see
  `DECISIONS.md`), `QuboSearchParams`/`default_qubo_search_params` (the
  tau1/tau2/N/T_dwell grid + cost weights for step 7).
- **`sim/gate.py`** additions: `TiltGateState`/`initial_tilt_gate_state`/
  `step_tilt_gate` (mirrors `step_gate`'s hysteresis+dwell structure, acting
  on instantaneous `|psi|` instead of a windowed NIS sum).
- **`sim/run.py`**: `ControllerKind` enum, `EpisodeConfig` (frozen dataclass,
  `.resolved()` fills in `None` fields with CLAUDE.md defaults), `run_episode`.
- **`experiments/run_batch.py`**: `ScenarioSpec`, `default_scenarios(nominal_T,
  disturbance_T)` (overridable so `quantum/qubo.py` can reuse the same six
  disturbance closures at a shorter episode length instead of duplicating
  them), `run_batch`.
- **`analysis/metrics.py`**: `fall_rate`, `false_fallback`,
  `missed_fallback_rate`, `detection_delay`, `progress`, `build_metrics_table`,
  `threshold_sensitivity`.
- **`quantum/qubo.py`**: `Candidate`/`all_candidates` (bit encoding),
  `cost_table` (the true re-simulated table, parallelized), `run_qubo_pipeline`,
  `format_report`.
- **`analysis/animate.py`**: `_body_frame`/`_pose_transform` (pure geometry,
  well-tested in isolation), `render_episode`, `_ensure_display`.
- Test suite: 17 test modules under `tests/`. `test_run.py`, `test_run_batch.py`,
  `test_metrics.py`, `test_qubo.py`, `test_animate.py` are new this session.

## Current configuration

- Python 3.11 (venv), managed by `uv` (`uv.lock` checked in).
- `pyproject.toml` deps added this session: `pandas`, `pyarrow`, `joblib`,
  `pyvista`, `vtk` (transitive), `imageio`, `imageio-ffmpeg`, `qiskit`,
  `qiskit-optimization`, `qiskit-aer`. `python-control` (named in CLAUDE.md §3)
  is still **not** a dependency — nothing in the codebase imports it; `scipy`'s
  `solve_continuous_are` covers the LQR design needs so far.
- Git remote: `https://github.com/ShreeniG99/Echo-Balancer.git`, working
  branch this session: `claude/amazing-thompson-o1hp87`.
- `.gitignore` addition this session: `/experiments/results/` (regenerable
  batch output, ~150MB for the default run — not source).
- `nxtway_gs/` and `license.txt` are gitignored (third-party reference
  material). `graphify-out/cost.json` and `graphify-out/cache/` are also
  gitignored.
- **Environment note:** this container has no GPU/EGL/OSMesa. PyVista/VTK
  off-screen rendering needs `xvfb-run -a <cmd>` or `analysis.animate`'s own
  `_ensure_display()` (used automatically by `render_episode`).

## Current objective

All of `CLAUDE.md` §14's numbered build-order steps are done. Remaining
plausible next steps, none currently in progress: `analysis/plots.py`
(§4/§14 step 8's other half), a hardware parameter pass once real components
arrive, `quantum.qubo.threshold_sensitivity()`'s sweep table if wanted for a
write-up, or a deeper investigation into why `battery_droop`/`payload_shift`
are unsurvivable regardless of controller (see `OPEN_PROBLEMS.md`).

## Immediate next steps

None currently assigned — this is a natural stopping point (build order
complete). If continuing:
1. Ask the user whether `analysis/plots.py` (static plots, distinct from the
   video) is wanted, or whether the paper/write-up is the next priority.
2. If writing up results, the real numbers to cite are in this file's step 6/7
   sections above (fall rate/detection-delay comparison; Grover's margin/range
   finding) — re-run `experiments/run_batch.py`/`quantum/qubo.py` fresh rather
   than trusting these numbers verbatim if any upstream code changes.

## Uncommitted / unusual repo state as of this session

- `research/echo_balancer_research.pdf` and `research/"echo_balancer_detailed (1).pdf"`
  remain untracked and ungitignored — purpose/provenance still unconfirmed with
  the user, unchanged since prior sessions.
- `analysis/results/echo_balancer_demo.mp4` exists locally (sent to the user
  via file delivery); `/analysis/results/` was added to `.gitignore` this
  session, following the same precedent as `/experiments/results/` (large,
  regenerable output, not source).
- Everything else described above is committed (through commit `197e88f` on
  `claude/amazing-thompson-o1hp87`, pushed to origin).
