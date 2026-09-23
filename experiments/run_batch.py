"""Batch evaluation across the three CLAUDE.md section 12 controllers.
Produces two Parquet outputs per batch under experiments/results/:
<name>.parquet (one row per episode, summary metadata) and
steps_<name>.parquet (per-control-tick log, tagged by episode_id).

Two separate batches -- see docs/superpowers/plans/2026-09-22-run-batch-
metrics-step6.md "Design decision 1/3":
- balance-only (wall_following=False): all 7 scenarios (nominal + 6
  disturbances) -- the core NIS-detection metrics, reproducing the
  already-calibrated step 3/4 architecture exactly.
- corridor (wall_following=True, corridor_theta_dot_ref_nominal=0.3): the
  nominal scenario only -- the Progress metric.
"""
from pathlib import Path

import pandas as pd
from joblib import Parallel, delayed

from sim.disturbances import (
    battery_droop_v_batt,
    payload_shift_plant_params,
    push_psi_dot_kick,
    sensor_fault_accel_noise_params,
    sensor_fault_gyro_bias_step,
    surface_change_plant_params,
)
from sim.params import default_disturbance_params
from sim.run import ControllerType, EpisodeConfig, run_episode

RESULTS_DIR = Path(__file__).resolve().parent / "results"
CONTROLLERS = [ControllerType.NAIVE, ControllerType.TILT_THRESHOLD, ControllerType.NIS_GATE]
SEEDS = list(range(10))
DP = default_disturbance_params()


def _push(t, x, bg, p, sp):
    x = x.copy()
    x[4] = push_psi_dot_kick(x[4], t, DP.push_onset, 0.005, DP.push_magnitude)
    return x, bg, p, sp, DP.battery_droop_v_nominal


def _surface_change(t, x, bg, p, sp):
    return x, bg, surface_change_plant_params(
        t, DP.surface_change_onset, DP.surface_change_duration, DP.surface_change_fw, p
    ), sp, DP.battery_droop_v_nominal


def _gyro_bias_fault(t, x, bg, p, sp):
    return x, sensor_fault_gyro_bias_step(
        bg, t, DP.gyro_bias_fault_onset, 0.005, DP.gyro_bias_fault_magnitude
    ), p, sp, DP.battery_droop_v_nominal


def _accel_noise_fault(t, x, bg, p, sp):
    return x, bg, p, sensor_fault_accel_noise_params(
        t, DP.accel_noise_fault_onset, DP.accel_noise_fault_duration, DP.accel_noise_fault_multiplier, sp
    ), DP.battery_droop_v_nominal


_BATTERY_PUSH_ONSET = DP.battery_droop_onset + DP.battery_droop_duration


def _battery_droop(t, x, bg, p, sp):
    v = battery_droop_v_batt(
        t, DP.battery_droop_onset, DP.battery_droop_duration, DP.battery_droop_v_nominal, DP.battery_droop_v_drooped
    )
    x = x.copy()
    x[4] = push_psi_dot_kick(x[4], t, _BATTERY_PUSH_ONSET, 0.005, DP.battery_droop_companion_push_magnitude)
    return x, bg, p, sp, v


def _payload_shift(t, x, bg, p, sp):
    return x, bg, payload_shift_plant_params(
        t, DP.payload_shift_onset, DP.payload_shift_delta_M, DP.payload_shift_delta_L, p
    ), sp, DP.battery_droop_v_nominal


# scenario name -> (disturbance_fn or None, T, disturbance_onset or None, x0_psi_deg)
DISTURBANCE_SCENARIOS = {
    "nominal": (None, 60.0, None, 0.0),
    "push": (_push, 20.0, DP.push_onset, 0.0),
    "surface_change": (_surface_change, 20.0, DP.surface_change_onset, 0.0),
    "gyro_bias_fault": (_gyro_bias_fault, 20.0, DP.gyro_bias_fault_onset, 0.0),
    "accel_noise_fault": (_accel_noise_fault, 20.0, DP.accel_noise_fault_onset, 0.0),
    "battery_droop": (_battery_droop, 20.0, _BATTERY_PUSH_ONSET, 0.0),
    "payload_shift": (_payload_shift, 20.0, DP.payload_shift_onset, DP.payload_shift_test_psi0_deg),
}


def _run_one(controller, scenario_name, scenario, seed, wall_following):
    # `scenario` (the DISTURBANCE_SCENARIOS[scenario_name] tuple) is resolved
    # by the caller in the parent process and passed explicitly rather than
    # looked up here from the module-level DISTURBANCE_SCENARIOS global.
    # joblib's default backend (loky) runs each job in a separate OS
    # process; on native Windows, "spawn" is the *only* available start
    # method (no fork/forkserver -- see multiprocessing.get_all_start_
    # methods()), so a worker process always re-imports this module fresh
    # from disk instead of inheriting the parent process's in-memory state.
    # A lookup here would silently see the on-disk DISTURBANCE_SCENARIOS,
    # not any value a caller (e.g. a test via monkeypatch) set in the
    # parent process. Passing the resolved tuple through `delayed()`
    # sidesteps that: joblib pickles the actual argument value in the
    # parent and ships it to the worker explicitly.
    fn, T, onset, x0_psi_deg = scenario
    config = EpisodeConfig(
        controller=controller, T=T, wall_following=wall_following,
        x0_psi_deg=x0_psi_deg, disturbance=fn, disturbance_name=scenario_name,
    )
    steps_df = run_episode(config, seed).copy()
    episode_id = f"{controller.value}_{scenario_name}_{seed}_{'corridor' if wall_following else 'balance'}"
    steps_df["episode_id"] = episode_id
    summary = {
        "episode_id": episode_id,
        "controller": controller.value,
        "scenario": scenario_name,
        "seed": seed,
        "wall_following": wall_following,
        "disturbance_onset": onset,
        "fell": bool(steps_df["fallen"].any()),
    }
    return steps_df, summary


def run_batch(wall_following: bool, scenario_names: list[str]) -> tuple[pd.DataFrame, pd.DataFrame]:
    jobs = [
        (controller, scenario_name, DISTURBANCE_SCENARIOS[scenario_name], seed)
        for controller in CONTROLLERS
        for scenario_name in scenario_names
        for seed in SEEDS
    ]
    results = Parallel(n_jobs=-1)(
        delayed(_run_one)(controller, scenario_name, scenario, seed, wall_following)
        for controller, scenario_name, scenario, seed in jobs
    )
    steps_dfs, summaries = zip(*results)
    steps_df = pd.concat(steps_dfs, ignore_index=True)
    episodes_df = pd.DataFrame(summaries)
    return steps_df, episodes_df


def main():
    RESULTS_DIR.mkdir(parents=True, exist_ok=True)

    balance_scenarios = list(DISTURBANCE_SCENARIOS.keys())
    steps_bal, episodes_bal = run_batch(wall_following=False, scenario_names=balance_scenarios)
    steps_bal.to_parquet(RESULTS_DIR / "steps_balance.parquet")
    episodes_bal.to_parquet(RESULTS_DIR / "episodes_balance.parquet")

    steps_cor, episodes_cor = run_batch(wall_following=True, scenario_names=["nominal"])
    steps_cor.to_parquet(RESULTS_DIR / "steps_corridor.parquet")
    episodes_cor.to_parquet(RESULTS_DIR / "episodes_corridor.parquet")


if __name__ == "__main__":
    main()
