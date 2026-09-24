"""Batch evaluation across the three CLAUDE.md section 12 controllers
(naive, tilt-threshold gate, NIS gate), compared on identical seeded
disturbance scenarios (build-order step 6, Milestone 2).

Scenarios: one nominal (disturbance-free, T=60s, matching the section 11
calibration length) plus one scenario per CLAUDE.md section 8 disturbance
profile (T=30s, onset=5s, giving the 10s-duration surface-change profile
and the gate's recovery both room to play out) -- 7 scenarios in total,
run at 10 seeds each per controller (210 episodes). Runs are parallelized
across CPU cores with joblib (CLAUDE.md section 3).

Writes two Parquet files under out_dir: episodes.parquet (one row per
episode: controller, scenario, seed, + episode-level summary) and
steps.parquet (one row per control tick across every episode, tagged with
controller/scenario/seed) -- CLAUDE.md section 12: "logged to Parquet, one
row per episode + a per-step log."
"""

from dataclasses import dataclass
from pathlib import Path
from typing import Optional

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
from sim.run import ControllerKind, DisturbanceFn, EpisodeConfig, run_episode

DEFAULT_CONTROLLERS = (ControllerKind.NAIVE, ControllerKind.TILT_GATE, ControllerKind.NIS_GATE)
DEFAULT_SEEDS = tuple(range(10))
NOMINAL_T = 60.0
DISTURBANCE_T = 30.0


@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    disturbance: Optional[DisturbanceFn]
    onset: Optional[float]  # None for the nominal scenario (no detection-delay metric applies)
    T: float
    x0_psi_deg: float = 0.0


def default_scenarios(nominal_T: float = NOMINAL_T, disturbance_T: float = DISTURBANCE_T) -> list[ScenarioSpec]:
    """The nominal scenario plus one per CLAUDE.md section 8 disturbance,
    with the same onset/magnitude choices tests/test_gate.py's per-
    disturbance closures use (see that file for why, e.g., battery droop
    needs a companion push fired after its ramp completes, or payload
    shift needs a concurrent mild tilt).

    nominal_T/disturbance_T default to this module's own Milestone-2
    lengths, but are overridable so other callers (quantum/qubo.py's
    offline threshold search, which needs much shorter episodes to stay
    computationally bounded across its whole candidate grid) can reuse
    these same scenario definitions without duplicating the six
    disturbance closures.
    """
    dp = default_disturbance_params()
    battery_push_onset = dp.battery_droop_onset + dp.battery_droop_duration

    def push(t, x, b, p, sp):
        x = x.copy()
        x[4] = push_psi_dot_kick(x[4], t, dp.push_onset, 0.005, dp.push_magnitude)
        return x, b, p, sp, dp.battery_droop_v_nominal

    def surface_change(t, x, b, p, sp):
        p_now = surface_change_plant_params(t, dp.surface_change_onset, dp.surface_change_duration, dp.surface_change_fw, p)
        return x, b, p_now, sp, dp.battery_droop_v_nominal

    def gyro_bias_fault(t, x, b, p, sp):
        b_now = sensor_fault_gyro_bias_step(b, t, dp.gyro_bias_fault_onset, 0.005, dp.gyro_bias_fault_magnitude)
        return x, b_now, p, sp, dp.battery_droop_v_nominal

    def accel_noise_fault(t, x, b, p, sp):
        sp_now = sensor_fault_accel_noise_params(
            t, dp.accel_noise_fault_onset, dp.accel_noise_fault_duration, dp.accel_noise_fault_multiplier, sp
        )
        return x, b, p, sp_now, dp.battery_droop_v_nominal

    def payload_shift(t, x, b, p, sp):
        p_now = payload_shift_plant_params(t, dp.payload_shift_onset, dp.payload_shift_delta_M, dp.payload_shift_delta_L, p)
        return x, b, p_now, sp, dp.battery_droop_v_nominal

    def battery_droop(t, x, b, p, sp):
        v_batt = battery_droop_v_batt(
            t, dp.battery_droop_onset, dp.battery_droop_duration, dp.battery_droop_v_nominal, dp.battery_droop_v_drooped
        )
        x = x.copy()
        x[4] = push_psi_dot_kick(x[4], t, battery_push_onset, 0.005, dp.battery_droop_companion_push_magnitude)
        return x, b, p, sp, v_batt

    return [
        ScenarioSpec(name="nominal", disturbance=None, onset=None, T=nominal_T),
        ScenarioSpec(name="push", disturbance=push, onset=dp.push_onset, T=disturbance_T),
        ScenarioSpec(name="surface_change", disturbance=surface_change, onset=dp.surface_change_onset, T=disturbance_T),
        ScenarioSpec(name="gyro_bias_fault", disturbance=gyro_bias_fault, onset=dp.gyro_bias_fault_onset, T=disturbance_T),
        ScenarioSpec(name="accel_noise_fault", disturbance=accel_noise_fault, onset=dp.accel_noise_fault_onset, T=disturbance_T),
        ScenarioSpec(
            name="payload_shift", disturbance=payload_shift, onset=dp.payload_shift_onset, T=disturbance_T,
            x0_psi_deg=dp.payload_shift_test_psi0_deg,
        ),
        ScenarioSpec(name="battery_droop", disturbance=battery_droop, onset=battery_push_onset, T=disturbance_T),
    ]


def _run_one(controller: ControllerKind, scenario: ScenarioSpec, seed: int) -> pd.DataFrame:
    cfg = EpisodeConfig(
        controller=controller, T=scenario.T, disturbance=scenario.disturbance, x0_psi_deg=scenario.x0_psi_deg
    )
    df = run_episode(cfg, seed=seed)
    df = df.copy()
    df.insert(0, "seed", seed)
    df.insert(0, "scenario", scenario.name)
    df.insert(0, "controller", controller.value)
    return df


def run_batch(
    controllers=DEFAULT_CONTROLLERS,
    scenarios=None,
    seeds=DEFAULT_SEEDS,
    out_dir: Path = Path("experiments/results"),
    n_jobs: int = -1,
) -> tuple[pd.DataFrame, pd.DataFrame]:
    """Runs every (controller, scenario, seed) episode, in parallel, and
    writes episodes.parquet / steps.parquet under out_dir. Returns
    (episodes_df, steps_df).
    """
    scenarios = default_scenarios() if scenarios is None else scenarios
    jobs = [
        (controller, scenario, seed)
        for controller in controllers
        for scenario in scenarios
        for seed in seeds
    ]

    step_dfs = Parallel(n_jobs=n_jobs)(delayed(_run_one)(c, s, seed) for c, s, seed in jobs)

    episode_rows = []
    for (controller, scenario, seed), df in zip(jobs, step_dfs):
        episode_rows.append(dict(
            controller=controller.value,
            scenario=scenario.name,
            seed=seed,
            onset=scenario.onset if scenario.onset is not None else float("nan"),
            T=scenario.T,
            fell=df.attrs["fell"],
            fall_time=df.attrs["fall_time"] if df.attrs["fall_time"] is not None else float("nan"),
            progress=df.attrs["progress"],
            n_steps=len(df),
        ))

    episodes_df = pd.DataFrame(episode_rows)
    steps_df = pd.concat(step_dfs, ignore_index=True)

    out_dir.mkdir(parents=True, exist_ok=True)
    episodes_df.to_parquet(out_dir / "episodes.parquet", index=False)
    steps_df.to_parquet(out_dir / "steps.parquet", index=False)

    return episodes_df, steps_df


if __name__ == "__main__":
    episodes_df, steps_df = run_batch()
    print(f"{len(episodes_df)} episodes, {len(steps_df)} step-rows written to experiments/results/")
