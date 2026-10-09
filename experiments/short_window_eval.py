"""Held-out evaluation of the opt-in short-window spike detector
(sim.params.default_short_window_gate_params) against the default NIS gate.

The short-window threshold was calibrated on run_batch seeds 0-9, so this
uses seeds 100-109 only. Every section 8 scenario is run (balance-only), plus
nominal corridor runs (motion can create innovation spikes).

Usage: PYTHONPATH=. uv run python -m experiments.short_window_eval
Writes docs/results/short_window_heldout.csv (one row per episode) and prints a summary.
"""

from pathlib import Path

import numpy as np
import pandas as pd
from joblib import Parallel, delayed

from experiments.run_batch import DISTURBANCE_SCENARIOS
from sim.params import default_gate_params, default_short_window_gate_params
from sim.run import ControllerType, EpisodeConfig, run_episode

OUT = Path(__file__).resolve().parents[1] / "docs" / "results" / "short_window_heldout.csv"
HELDOUT_SEEDS = tuple(range(100, 110))
GATES = {"default": default_gate_params(), "short_window": default_short_window_gate_params()}


def _one(gate_name: str, scenario: str, wall_following: bool, seed: int) -> dict:
    fn, T, onset, psi0 = DISTURBANCE_SCENARIOS[scenario]
    df = run_episode(EpisodeConfig(ControllerType.NIS_GATE, T, wall_following=wall_following, x0_psi_deg=psi0,
                                   disturbance=fn, gate_params=GATES[gate_name]), seed)
    non_normal = df["mode"] != "NORMAL"
    entries = int(((df["mode"] != "NORMAL") & (df["mode"].shift(fill_value="NORMAL") == "NORMAL")).sum())
    row = {
        "gate": gate_name, "scenario": scenario, "wall_following": wall_following, "seed": seed,
        "fell": bool(df["fallen"].any()), "frac_non_normal": float(non_normal.mean()),
        "fallback_entries": entries, "duration_s": float(df["t"].iloc[-1] + 0.005),
    }
    if onset is not None:
        after = df[df["t"] >= onset]
        trip = after.loc[after["mode"] != "NORMAL", "t"]
        row["detect_delay_s"] = float(trip.iloc[0] - onset) if len(trip) else np.nan
        row["false_alarm_before_onset"] = bool(non_normal[df["t"] < onset].any())
    return row


def main() -> None:
    jobs = [(g, sc, False, s) for g in GATES for sc in DISTURBANCE_SCENARIOS for s in HELDOUT_SEEDS]
    jobs += [(g, "nominal", True, s) for g in GATES for s in HELDOUT_SEEDS]
    rows = pd.DataFrame(Parallel(n_jobs=-1)(delayed(_one)(*j) for j in jobs))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    rows.to_csv(OUT, index=False, float_format="%.4g")
    rows["setting"] = np.where(rows["wall_following"], rows["scenario"] + " (corridor)", rows["scenario"])
    summary = rows.groupby(["setting", "gate"]).agg(
        falls=("fell", "sum"), episodes=("fell", "size"),
        detected=("detect_delay_s", lambda s: int(s.notna().sum())),
        median_delay_s=("detect_delay_s", "median"),
        pre_onset_false_alarms=("false_alarm_before_onset", lambda s: int(s.fillna(False).sum())),
        frac_time_non_normal=("frac_non_normal", "mean"),
        fallback_entries_per_min=("fallback_entries", lambda s: float(s.sum()) / (rows.loc[s.index, "duration_s"].sum() / 60)),
    )
    print(summary.round(4).to_string())


if __name__ == "__main__":
    main()
