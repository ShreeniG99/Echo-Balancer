"""Batch evaluation metrics (CLAUDE.md section 12). Operate on the
(steps_df, episodes_df) pair experiments/run_batch.py produces:
steps_df has one row per control tick (columns include at least
"episode_id", "t", "mode", "theta", "fallen"); episodes_df has one row per
episode ("episode_id", "controller", "scenario", "disturbance_onset"
(NaN for the nominal scenario), "fell").

Threshold-sensitivity sweeps (CLAUDE.md section 12's last metric) are not
computed here -- they're what quantum/qubo.py's grid search re-simulates
against (section 13), a later build-order step.
"""

import numpy as np
import pandas as pd

from sim.params import default_plant_params


def fall_rate(episodes_df: pd.DataFrame) -> pd.Series:
    """Fraction of episodes that fell, grouped by controller."""
    return episodes_df.groupby("controller")["fell"].mean()


def false_fallback_fraction(steps_df: pd.DataFrame, episodes_df: pd.DataFrame) -> pd.Series:
    """Fraction of disturbance-free ("nominal" scenario) time spent
    outside NORMAL, grouped by controller."""
    nominal_episodes = episodes_df.loc[episodes_df["scenario"] == "nominal", ["episode_id", "controller"]]
    merged = steps_df.merge(nominal_episodes, on="episode_id", how="inner")
    return merged.groupby("controller")["mode"].apply(lambda modes: (modes != "NORMAL").mean())


def missed_fallback(steps_df: pd.DataFrame, episodes_df: pd.DataFrame) -> pd.DataFrame:
    """Falls where the gate was still NORMAL 0.5s before the fall, one row
    per fallen episode."""
    rows = []
    for _, ep in episodes_df[episodes_df["fell"]].iterrows():
        ep_steps = steps_df[steps_df["episode_id"] == ep["episode_id"]].sort_values("t")
        fall_t = ep_steps.loc[ep_steps["fallen"], "t"].iloc[0]
        before = ep_steps[ep_steps["t"] <= fall_t - 0.5]
        was_normal = before.empty or before["mode"].iloc[-1] == "NORMAL"
        rows.append({"episode_id": ep["episode_id"], "controller": ep["controller"], "missed_fallback": bool(was_normal)})
    return pd.DataFrame(rows)


def detection_delay(steps_df: pd.DataFrame, episodes_df: pd.DataFrame) -> pd.DataFrame:
    """Disturbance onset -> first non-NORMAL tick, one row per episode
    with a known disturbance onset (NaN if never detected)."""
    rows = []
    disturbed = episodes_df[episodes_df["disturbance_onset"].notna()]
    for _, ep in disturbed.iterrows():
        ep_steps = steps_df[steps_df["episode_id"] == ep["episode_id"]].sort_values("t")
        after_onset = ep_steps[ep_steps["t"] >= ep["disturbance_onset"]]
        non_normal = after_onset[after_onset["mode"] != "NORMAL"]
        delay = (non_normal["t"].iloc[0] - ep["disturbance_onset"]) if not non_normal.empty else np.nan
        rows.append({
            "episode_id": ep["episode_id"], "controller": ep["controller"],
            "scenario": ep["scenario"], "detection_delay_s": delay,
        })
    return pd.DataFrame(rows)


def progress(steps_df: pd.DataFrame, episodes_df: pd.DataFrame) -> pd.DataFrame:
    """Distance travelled (R * delta-theta) per episode -- CLAUDE.md
    section 12: "distance travelled along the corridor (cost of
    caution)." """
    R = default_plant_params().R
    rows = []
    for episode_id, ep_steps in steps_df.groupby("episode_id"):
        ep_steps = ep_steps.sort_values("t")
        dist = R * (ep_steps["theta"].iloc[-1] - ep_steps["theta"].iloc[0])
        rows.append({"episode_id": episode_id, "progress_m": dist})
    result = pd.DataFrame(rows)
    return result.merge(episodes_df[["episode_id", "controller", "scenario"]], on="episode_id")


def summarize_batch(steps_df: pd.DataFrame, episodes_df: pd.DataFrame) -> pd.DataFrame:
    """One row per controller: fall rate, false-fallback fraction, mean
    detection delay, mean progress -- CLAUDE.md section 12's metrics
    table."""
    fr = fall_rate(episodes_df)
    ff = false_fallback_fraction(steps_df, episodes_df)
    dd = detection_delay(steps_df, episodes_df).groupby("controller")["detection_delay_s"].mean()
    pr = progress(steps_df, episodes_df).groupby("controller")["progress_m"].mean()
    return pd.DataFrame({
        "fall_rate": fr,
        "false_fallback_fraction": ff,
        "mean_detection_delay_s": dd,
        "mean_progress_m": pr,
    })
