"""Cost table for gate-threshold selection (CLAUDE.md section 13, steps 1-2).

Every feasible candidate is RE-SIMULATED on the evaluation set (the gate
changes the robot's behaviour, so log replay is not valid):
  - each section 8 disturbance scenario x seeds (balance-only)  -> fall rate
  - nominal balance + nominal corridor x seeds                  -> false fallback
  - nominal corridor x seeds                                    -> progress
and scored with J = w_fall*fall_rate + w_false*false_fallback + w_progress*(1-progress).

Usage: PYTHONPATH=. uv run python -m quantum.cost_table   (~20-25 min on 4 cores)
"""

from itertools import product
from pathlib import Path

import numpy as np
import pandas as pd
from joblib import Parallel, delayed

from experiments.run_batch import DISTURBANCE_SCENARIOS
from sim.params import GateParams, QuboParams, default_gate_params, default_plant_params, default_qubo_params
from sim.run import ControllerType, EpisodeConfig, run_episode

RESULTS = Path(__file__).resolve().parents[1] / "experiments" / "results"
TABLE_PATH = RESULTS / "qubo_cost_table.parquet"
EPISODES_PATH = RESULTS / "qubo_cost_episodes.parquet"  # per-episode rows: re-score without re-simulating
N_BITS = 6


def decode(idx: int, qp: QuboParams) -> dict:
    """Index bits: 0-1 tau1 level, 2-3 tau2 level, 4 N, 5 T_dwell."""
    i1, i2, iN, iT = idx & 3, (idx >> 2) & 3, (idx >> 4) & 1, (idx >> 5) & 1
    l1, l2, N = qp.tau1_levels[i1], qp.tau2_levels[i2], qp.N_values[iN]
    return {
        "index": idx, "tau1_level": l1, "tau2_level": l2, "N": N, "T_dwell": qp.T_dwell_values[iT],
        "tau1": l1 * N, "tau2": l2 * N, "feasible": l1 < l2,
    }


def gate_params_for(c: dict, qp: QuboParams) -> GateParams:
    """Candidate long-window thresholds; the short-window detector is part of the gate (not a grid
    variable) and is carried over unchanged from default_gate_params()."""
    d = default_gate_params()
    return GateParams(N=c["N"], tau1=c["tau1"], tau2=c["tau2"], tau1_exit=qp.tau1_exit_ratio * c["tau1"],
                      tau2_exit=c["tau1"], T_dwell=c["T_dwell"], short_N=d.short_N, short_tau=d.short_tau)


def evaluation_set(qp: QuboParams) -> list[tuple[str, bool, float, float, object]]:
    """(scenario, wall_following, T, x0_psi_deg, disturbance_fn) for each episode type."""
    eps = [("nominal", False, qp.nominal_T, 0.0, None), ("nominal", True, qp.nominal_T, 0.0, None)]
    for name, (fn, T, _onset, psi0) in DISTURBANCE_SCENARIOS.items():
        if name != "nominal":
            eps.append((name, False, T, psi0, fn))
    return eps


def _episode(controller, gate_p, scen, seed) -> dict:
    name, wf, T, psi0, fn = scen
    df = run_episode(EpisodeConfig(controller, T, wall_following=wf, x0_psi_deg=psi0, disturbance=fn,
                                   disturbance_name=name, gate_params=gate_p), seed)
    return {
        "scenario": name, "wall_following": wf, "seed": seed, "fell": bool(df["fallen"].any()),
        "frac_non_normal": float((df["mode"] != "NORMAL").mean()),
        "distance_m": default_plant_params().R * float(df["theta"].iloc[-1] - df["theta"].iloc[0]),
        "detected": bool((df["mode"] != "NORMAL").any()),
    }


def score(rows: pd.DataFrame, naive_distance: float, qp: QuboParams) -> dict:
    """fall_rate counts falls over the WHOLE evaluation set (nominal episodes included: a gate
    action can itself destabilise a nominal corridor run)."""
    dist = rows[rows["scenario"] != "nominal"]
    nominal = rows[rows["scenario"] == "nominal"]
    corridor = nominal[nominal["wall_following"]]
    fall_rate = float(rows["fell"].mean())
    false_fb = float(nominal["frac_non_normal"].mean())
    progress = float(np.clip(corridor["distance_m"].mean() / naive_distance, 0.0, 1.0))
    return {
        "fall_rate": fall_rate, "false_fallback": false_fb, "progress": progress,
        "detection_rate": float(dist["detected"].mean()),
        "J": qp.w_fall * fall_rate + qp.w_false * false_fb + qp.w_progress * (1.0 - progress),
    }


def simulate_rows(qp: QuboParams | None = None, n_jobs: int = -1) -> pd.DataFrame:
    """Re-simulate every feasible candidate (plus the naive corridor reference) -> one row per episode."""
    qp = qp or default_qubo_params()
    cands = [decode(i, qp) for i in range(2**N_BITS)]
    feas = [c for c in cands if c["feasible"]]
    scen = evaluation_set(qp)
    corridor = [s for s in scen if s[1]]
    jobs = [(ControllerType.NAIVE, None, s, seed, -1) for s in corridor for seed in qp.seeds]
    jobs += [(ControllerType.NIS_GATE, gate_params_for(c, qp), s, seed, c["index"])
             for c in feas for s, seed in product(scen, qp.seeds)]
    out = Parallel(n_jobs=n_jobs, verbose=5)(delayed(_episode)(ctl, gp, s, seed) for ctl, gp, s, seed, _ in jobs)
    rows = pd.DataFrame(out)
    rows["index"] = [j[4] for j in jobs]
    return rows


def table_from_rows(rows: pd.DataFrame, qp: QuboParams | None = None) -> pd.DataFrame:
    qp = qp or default_qubo_params()
    cands = [decode(i, qp) for i in range(2**N_BITS)]
    naive_distance = rows.loc[rows["index"] == -1, "distance_m"].mean()
    table = []
    for c in cands:
        if c["feasible"]:
            table.append({**c, **score(rows[rows["index"] == c["index"]], naive_distance, qp)})
        else:
            table.append({**c, "fall_rate": np.nan, "false_fallback": np.nan, "progress": np.nan,
                          "detection_rate": np.nan, "J": np.nan})
    return pd.DataFrame(table)


def main() -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    rows = simulate_rows()
    rows.to_parquet(EPISODES_PATH)
    table = table_from_rows(rows)
    table.to_parquet(TABLE_PATH)
    out = Path(__file__).resolve().parents[1] / "docs" / "results"
    out.mkdir(parents=True, exist_ok=True)
    table.to_csv(out / "qubo_cost_table.csv", index=False, float_format="%.6g")
    print(table.sort_values("J").head(10).to_string())


if __name__ == "__main__":
    main()
