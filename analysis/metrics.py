"""Cross-controller evaluation metrics (CLAUDE.md section 12, Milestone 2).

Consumes the (episodes_df, steps_df) pair experiments/run_batch.py
produces. Every metric here is computed per controller from each
controller's own `mode` column -- NAIVE's is always NORMAL by
construction (sim.run.run_episode), TILT_GATE's comes from
sim.gate.step_tilt_gate on the KF-estimated psi, NIS_GATE's from
sim.gate.step_gate on the NIS -- so the same functions give a like-for-
like comparison across all three without any controller-specific casing.
"""

import numpy as np
import pandas as pd


def fall_rate(episodes_df: pd.DataFrame) -> pd.Series:
    """CLAUDE.md section 12: 'Fall rate.'"""
    return episodes_df.groupby("controller")["fell"].mean().rename("fall_rate")


def false_fallback(episodes_df: pd.DataFrame, steps_df: pd.DataFrame, dt_control: float = 0.005) -> pd.DataFrame:
    """CLAUDE.md section 12: 'False-fallback: fraction of disturbance-free
    time spent outside NORMAL; fallback events per minute in nominal
    runs.' Computed only on the 'nominal' scenario's episodes -- disturbed
    episodes leaving NORMAL is the point, not a false fallback.
    """
    nominal_steps = steps_df[steps_df["scenario"] == "nominal"]
    rows = {}
    for controller, group in nominal_steps.groupby("controller"):
        fracs, events_per_min = [], []
        for _seed, ep in group.groupby("seed"):
            ep = ep.sort_values("t")
            fracs.append(float((ep["mode"] != "NORMAL").mean()))
            is_non_normal = (ep["mode"] != "NORMAL").to_numpy()
            n_entries = int(np.sum(is_non_normal[1:] & ~is_non_normal[:-1]))
            duration_min = len(ep) * dt_control / 60.0
            events_per_min.append(n_entries / duration_min if duration_min > 0 else float("nan"))
        rows[controller] = dict(
            false_fallback_frac=float(np.mean(fracs)),
            fallback_events_per_min=float(np.mean(events_per_min)),
        )
    return pd.DataFrame(rows).T


def missed_fallback_rate(episodes_df: pd.DataFrame, steps_df: pd.DataFrame, warning_s: float = 0.5) -> pd.Series:
    """CLAUDE.md section 12: 'Missed-fallback: falls where the gate was
    still NORMAL 0.5s before the fall.' NaN for a controller with no falls
    at all (rate is undefined, not zero)."""
    rows = {}
    for controller in episodes_df["controller"].unique():
        fallen = episodes_df[(episodes_df["controller"] == controller) & episodes_df["fell"]]
        if len(fallen) == 0:
            rows[controller] = float("nan")
            continue
        missed = 0
        for _, ep_row in fallen.iterrows():
            steps = _episode_steps(steps_df, ep_row).sort_values("t")
            before = steps[steps["t"] <= ep_row["fall_time"] - warning_s]
            if len(before) == 0 or before["mode"].iloc[-1] == "NORMAL":
                missed += 1
        rows[controller] = missed / len(fallen)
    return pd.Series(rows, name="missed_fallback_rate")


def detection_delay(episodes_df: pd.DataFrame, steps_df: pd.DataFrame) -> pd.DataFrame:
    """CLAUDE.md section 12: 'Detection delay: disturbance onset -> first
    non-NORMAL.' Averaged over disturbance scenarios only ('nominal' has
    no onset); episodes that never left NORMAL are counted as
    missed_detections and excluded from the mean, not treated as a
    zero/huge delay.
    """
    disturbance_eps = episodes_df[episodes_df["scenario"] != "nominal"]
    rows = {}
    for controller in episodes_df["controller"].unique():
        eps = disturbance_eps[disturbance_eps["controller"] == controller]
        delays, misses = [], 0
        for _, ep_row in eps.iterrows():
            steps = _episode_steps(steps_df, ep_row).sort_values("t")
            after_onset = steps[steps["t"] >= ep_row["onset"]]
            non_normal = after_onset[after_onset["mode"] != "NORMAL"]
            if len(non_normal) == 0:
                misses += 1
            else:
                delays.append(float(non_normal["t"].iloc[0] - ep_row["onset"]))
        rows[controller] = dict(
            mean_detection_delay_s=float(np.mean(delays)) if delays else float("nan"),
            missed_detections=misses,
            n_disturbance_episodes=len(eps),
        )
    return pd.DataFrame(rows).T


def progress(episodes_df: pd.DataFrame) -> pd.Series:
    """CLAUDE.md section 12: 'Progress: distance travelled along the
    corridor (cost of caution).'"""
    return episodes_df.groupby("controller")["progress"].mean().rename("mean_progress_m")


def _episode_steps(steps_df: pd.DataFrame, ep_row: pd.Series) -> pd.DataFrame:
    return steps_df[
        (steps_df["controller"] == ep_row["controller"])
        & (steps_df["scenario"] == ep_row["scenario"])
        & (steps_df["seed"] == ep_row["seed"])
    ]


def build_metrics_table(episodes_df: pd.DataFrame, steps_df: pd.DataFrame) -> pd.DataFrame:
    """The Milestone 2 cross-controller comparison table: one row per
    controller (naive, tilt_gate, nis_gate)."""
    table = fall_rate(episodes_df).to_frame()
    table = table.join(false_fallback(episodes_df, steps_df))
    table["missed_fallback_rate"] = missed_fallback_rate(episodes_df, steps_df)
    table = table.join(detection_delay(episodes_df, steps_df))
    table = table.join(progress(episodes_df))
    return table


def threshold_sensitivity(
    scenarios=None, seeds=range(5), gate_params_grid=None,
) -> pd.DataFrame:
    """CLAUDE.md section 12: 'Threshold sensitivity: metrics vs tau1, tau2
    sweeps.' Re-simulates the NIS_GATE controller only (the other two
    controllers have no tau1/tau2) across a small grid of GateParams,
    re-using experiments.run_batch's scenario/seed machinery. Not run by
    default from run_batch's __main__ (a full grid is much more expensive
    than the base three-controller table); call this directly when a
    sensitivity sweep is actually wanted.
    """
    from dataclasses import replace

    from sim.params import default_evaluation_gate_params
    from sim.run import ControllerKind, EpisodeConfig, run_episode
    from experiments.run_batch import default_scenarios

    scenarios = default_scenarios() if scenarios is None else scenarios
    base_gp = default_evaluation_gate_params()
    if gate_params_grid is None:
        gate_params_grid = [
            replace(base_gp, tau1=base_gp.tau1 * f1, tau2=base_gp.tau2 * f2)
            for f1 in (0.75, 1.0, 1.25)
            for f2 in (0.75, 1.0, 1.25)
            if f1 * base_gp.tau1 < f2 * base_gp.tau2
        ]

    rows = []
    for gp in gate_params_grid:
        episode_rows, step_dfs = [], []
        for scenario in scenarios:
            for seed in seeds:
                cfg = EpisodeConfig(
                    controller=ControllerKind.NIS_GATE, T=scenario.T, disturbance=scenario.disturbance,
                    x0_psi_deg=scenario.x0_psi_deg, gate_p=gp,
                )
                df = run_episode(cfg, seed=seed)
                df = df.copy()
                df["controller"] = ControllerKind.NIS_GATE.value
                df["scenario"] = scenario.name
                df["seed"] = seed
                step_dfs.append(df)
                episode_rows.append(dict(
                    controller=ControllerKind.NIS_GATE.value, scenario=scenario.name, seed=seed,
                    onset=scenario.onset if scenario.onset is not None else float("nan"),
                    fell=df.attrs["fell"], fall_time=df.attrs["fall_time"] or float("nan"),
                    progress=df.attrs["progress"],
                ))
        episodes_df = pd.DataFrame(episode_rows)
        steps_df = pd.concat(step_dfs, ignore_index=True)
        table = build_metrics_table(episodes_df, steps_df)
        row = table.loc["nis_gate"].to_dict()
        row.update(tau1=gp.tau1, tau2=gp.tau2)
        rows.append(row)
    return pd.DataFrame(rows)


if __name__ == "__main__":
    from pathlib import Path

    results_dir = Path("experiments/results")
    episodes_path, steps_path = results_dir / "episodes.parquet", results_dir / "steps.parquet"
    if not episodes_path.exists() or not steps_path.exists():
        from experiments.run_batch import run_batch

        episodes_df, steps_df = run_batch()
    else:
        episodes_df = pd.read_parquet(episodes_path)
        steps_df = pd.read_parquet(steps_path)

    table = build_metrics_table(episodes_df, steps_df)
    pd.set_option("display.width", 120)
    print(table)
