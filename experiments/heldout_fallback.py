"""Held-out check of the fallback (FallbackParams): its q_inflation was chosen
on the default payload-shift scenario with seeds 0-9, so that result is
in-sample. This re-tests it on seeds and payload sizes never used to tune it.

Usage: PYTHONPATH=. uv run python -m experiments.heldout_fallback
Writes docs/results/fallback_heldout.csv (one row per episode) and prints a summary.
"""

from pathlib import Path

import numpy as np
import pandas as pd
from joblib import Parallel, delayed

from sim.disturbances import payload_shift_plant_params
from sim.params import default_disturbance_params
from sim.run import ControllerType, EpisodeConfig, run_episode

OUT = Path(__file__).resolve().parents[1] / "docs" / "results" / "fallback_heldout.csv"
HELDOUT_SEEDS = tuple(range(100, 120))  # never used by run_batch (0-9) or the cost table (0-9)
DP = default_disturbance_params()
# (delta_M kg, delta_L m): lighter, the tuning case, heavier
PAYLOADS = ((0.5, 0.05), (DP.payload_shift_delta_M, DP.payload_shift_delta_L), (1.5, 0.15))
T = 20.0


def _payload_fn(dM: float, dL: float):
    def fn(t, x, bg, p, sp):
        return x, bg, payload_shift_plant_params(t, DP.payload_shift_onset, dM, dL, p), sp, DP.battery_droop_v_nominal
    return fn


def _one(dM: float, dL: float, fallback: bool, seed: int) -> dict:
    cfg = EpisodeConfig(ControllerType.NIS_GATE, T, x0_psi_deg=DP.payload_shift_test_psi0_deg,
                        disturbance=_payload_fn(dM, dL), fallback=fallback)
    df = run_episode(cfg, seed)
    after = df[df["t"] >= DP.payload_shift_onset]
    trip = after.loc[after["mode"] != "NORMAL", "t"]
    return {
        "delta_M": dM, "delta_L": dL, "fallback": fallback, "seed": seed,
        "fell": bool(df["fallen"].any()),
        "max_abs_psi_deg": float(np.degrees(df["psi"].abs().max())),
        "detect_delay_s": float(trip.iloc[0] - DP.payload_shift_onset) if len(trip) else np.nan,
    }


def main() -> None:
    jobs = [(dM, dL, fb, s) for dM, dL in PAYLOADS for fb in (False, True) for s in HELDOUT_SEEDS]
    rows = pd.DataFrame(Parallel(n_jobs=-1)(delayed(_one)(*j) for j in jobs))
    OUT.parent.mkdir(parents=True, exist_ok=True)
    rows.to_csv(OUT, index=False, float_format="%.4g")
    summary = rows.groupby(["delta_M", "delta_L", "fallback"]).agg(
        falls=("fell", "sum"), episodes=("fell", "size"),
        median_max_psi_deg=("max_abs_psi_deg", "median"),
        detected=("detect_delay_s", lambda s: int(s.notna().sum())),
        median_delay_s=("detect_delay_s", "median"),
    )
    print(summary.to_string())


if __name__ == "__main__":
    main()
