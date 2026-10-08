"""Paper figures + metrics tables (CLAUDE.md section 14 step 8, section 12 metrics).

Usage (after experiments/run_batch.py and, for the sensitivity figure, quantum/cost_table.py):
    PYTHONPATH=. uv run python -m analysis.plots
Writes docs/figures/*.png and docs/results/metrics_*.csv.
"""

from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from analysis.metrics import detection_delay, missed_fallback, summarize_batch
from sim.params import default_gate_params, default_qubo_params

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "experiments" / "results"
FIGS = ROOT / "docs" / "figures"
TABLES = ROOT / "docs" / "results"

# Validated categorical order (dataviz palette: CVD-safe adjacent pairs); fixed per controller.
CONTROLLER_COLORS = {"naive": "#2a78d6", "tilt_threshold": "#eb6834", "nis_gate": "#1baf7a"}
CONTROLLER_LABELS = {"naive": "Naive (always NORMAL)", "tilt_threshold": "Tilt-threshold gate", "nis_gate": "NIS gate (ours)"}
# Gate modes are states -> reserved status colours, always shown with a text label.
MODE_COLORS = {"NORMAL": "#0ca30c", "CAUTIOUS": "#fab219", "HALT": "#d03b3b"}
INK, MUTED, GRID = "#0b0b0b", "#52514e", "#e4e3df"


def _style(ax) -> None:
    ax.spines[["top", "right"]].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(MUTED)
    ax.tick_params(colors=MUTED, labelsize=8)
    ax.yaxis.grid(True, color=GRID, lw=0.8)
    ax.set_axisbelow(True)


def load_batch(tag: str = "balance") -> tuple[pd.DataFrame, pd.DataFrame]:
    return pd.read_parquet(RESULTS / f"steps_{tag}.parquet"), pd.read_parquet(RESULTS / f"episodes_{tag}.parquet")


def per_scenario_table(steps: pd.DataFrame, episodes: pd.DataFrame) -> pd.DataFrame:
    """controller x scenario: falls, detection rate, median detection delay, missed fallbacks."""
    dd = detection_delay(steps, episodes)
    mf = missed_fallback(steps, episodes)
    rows = []
    for (ctl, sc), ep in episodes.groupby(["controller", "scenario"]):
        d = dd[(dd["controller"] == ctl) & (dd["scenario"] == sc)]["detection_delay_s"]
        m = mf[mf["episode_id"].isin(ep["episode_id"])]
        rows.append({
            "controller": ctl, "scenario": sc, "episodes": len(ep), "falls": int(ep["fell"].sum()),
            "detection_rate": float(d.notna().mean()) if len(d) else np.nan,
            "median_detection_delay_s": float(d.median()) if d.notna().any() else np.nan,
            "missed_fallbacks": int(m["missed_fallback"].sum()) if len(m) else 0,
        })
    return pd.DataFrame(rows)


def fig_detection(table: pd.DataFrame, path: Path) -> None:
    scen = [s for s in table["scenario"].unique() if s != "nominal"]
    ctls = list(CONTROLLER_COLORS)
    fig, axes = plt.subplots(1, 2, figsize=(10, 3.6))
    w = 0.26
    x = np.arange(len(scen))
    for k, (col, title, fmt) in enumerate([("detection_rate", "Detection rate (left NORMAL after onset)", "{:.0%}"),
                                           ("falls", "Falls (of 10 seeds)", "{:.0f}")]):
        ax = axes[k]
        for j, c in enumerate(ctls):
            vals = [table[(table.controller == c) & (table.scenario == s)][col].iloc[0] for s in scen]
            vals = np.nan_to_num(np.array(vals, dtype=float))
            bars = ax.bar(x + (j - 1) * w, vals, w - 0.02, color=CONTROLLER_COLORS[c], label=CONTROLLER_LABELS[c])
            for b, v in zip(bars, vals):
                if v > 0 and (c == "nis_gate" or col == "falls"):  # selective direct labels
                    ax.text(b.get_x() + b.get_width() / 2, v, fmt.format(v), ha="center", va="bottom", fontsize=6, color=INK)
        ax.set_xticks(x, [s.replace("_", "\n") for s in scen], fontsize=7)
        ax.set_title(title, loc="left", fontsize=9, color=INK)
        _style(ax)
    axes[1].set_ylim(0, 10.5)
    axes[0].set_ylim(0, 1.12)
    axes[0].legend(frameon=False, fontsize=7, loc="upper left", ncol=3, bbox_to_anchor=(0, -0.22))
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def fig_sensitivity(cost: pd.DataFrame, path: Path) -> None:
    """Threshold sensitivity (section 12): metrics over tau1 x tau2 levels, one row per (N, T_dwell)."""
    qp = default_qubo_params()
    metrics = [("J", "Cost J (lower is better)"), ("false_fallback", "False-fallback fraction"),
               ("fall_rate", "Fall rate"), ("detection_rate", "Detection rate")]
    combos = [(N, T) for N in qp.N_values for T in qp.T_dwell_values]
    fig, axes = plt.subplots(len(combos), len(metrics), figsize=(12, 2.6 * len(combos)))
    best = cost.loc[cost["J"].idxmin()]
    for r, (N, T) in enumerate(combos):
        sub = cost[(cost.N == N) & (cost.T_dwell == T)]
        for c, (m, title) in enumerate(metrics):
            ax = axes[r, c]
            grid = sub.pivot(index="tau2_level", columns="tau1_level", values=m).sort_index(ascending=False)
            vmax = np.nanmax(cost[m]) if np.nanmax(cost[m]) > 0 else 1.0
            ax.imshow(grid.to_numpy(dtype=float), cmap="Blues", vmin=0, vmax=vmax, aspect="auto")
            for (i, j), v in np.ndenumerate(grid.to_numpy(dtype=float)):
                txt = "infeasible" if np.isnan(v) else f"{v:.2f}"
                dark = (not np.isnan(v)) and v > 0.6 * vmax
                ax.text(j, i, txt, ha="center", va="center", fontsize=6.5 if not np.isnan(v) else 5.5,
                        color="white" if dark else (MUTED if np.isnan(v) else INK))
                if m == "J" and not np.isnan(v) and (N, T, grid.index[i], grid.columns[j]) == (
                        best.N, best.T_dwell, best.tau2_level, best.tau1_level):
                    ax.add_patch(plt.Rectangle((j - 0.5, i - 0.5), 1, 1, fill=False, ec=INK, lw=2))
            ax.set_xticks(range(len(grid.columns)), [f"{v:g}" for v in grid.columns], fontsize=7)
            ax.set_yticks(range(len(grid.index)), [f"{v:g}" for v in grid.index], fontsize=7)
            if r == len(combos) - 1:
                ax.set_xlabel("tau1 / N (CAUTIOUS entry, mean NIS)", fontsize=7, color=MUTED)
            if c == 0:
                ax.set_ylabel(f"N={N}, T_dwell={T:g}s\ntau2 / N", fontsize=7, color=MUTED)
            if r == 0:
                ax.set_title(title, loc="left", fontsize=9, color=INK)
    fig.suptitle("Threshold sensitivity (re-simulated per candidate; outlined = min-J candidate)", fontsize=10, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def fig_episode(steps: pd.DataFrame, episodes: pd.DataFrame, scenario: str, path: Path) -> None:
    """Per controller: psi (left) and epsilon (right, own axis -- never a dual axis), mode strip on both."""
    gp = default_gate_params()
    fig, axes = plt.subplots(3, 2, figsize=(11, 6.5), sharex=True)
    for row, ctl in zip(axes, CONTROLLER_COLORS):
        ep = episodes[(episodes.controller == ctl) & (episodes.scenario == scenario) & (episodes.seed == 0)].iloc[0]
        d = steps[steps.episode_id == ep.episode_id].sort_values("t")
        row[0].plot(d.t, np.degrees(d.psi), color=CONTROLLER_COLORS[ctl], lw=1.6)
        row[0].set_ylabel("psi_deg", fontsize=8, color=MUTED)
        row[1].plot(d.t, d.epsilon, color=CONTROLLER_COLORS[ctl], lw=1.2)
        row[1].axhline(gp.tau1, color=MODE_COLORS["CAUTIOUS"], ls="--", lw=1)
        row[1].axhline(gp.tau2, color=MODE_COLORS["HALT"], ls="--", lw=1)
        row[1].set_yscale("log")
        row[1].set_ylabel("epsilon", fontsize=8, color=MUTED)
        lo, hi = np.degrees(d.psi).min(), np.degrees(d.psi).max()
        row[0].set_ylim(lo - 0.25 * (hi - lo + 1e-3), hi + 0.05 * (hi - lo + 1e-3))
        e = d.epsilon[d.epsilon > 0]
        row[1].set_ylim(max(e.min(), 1.0) / 8, e.max() * 2)
        t, modes = d.t.to_numpy(), d["mode"].to_numpy()
        for ax in row:
            i = 0
            while i < len(t) - 1:
                j = i
                while j < len(t) - 1 and modes[j] == modes[i]:
                    j += 1
                ax.axvspan(t[i], t[j], ymin=0, ymax=0.05, color=MODE_COLORS[modes[i]], lw=0)
                if t[j] - t[i] > 1.5:
                    ax.text((t[i] + t[j]) / 2, 0.065, modes[i], transform=ax.get_xaxis_transform(), fontsize=6,
                            ha="center", color=INK)
                i = j if j > i else i + 1
            if not np.isnan(ep.disturbance_onset):
                ax.axvline(ep.disturbance_onset, color=INK, ls=":", lw=1)
            _style(ax)
        row[0].set_title(CONTROLLER_LABELS[ctl] + ("  (FELL)" if ep.fell else ""), loc="left", fontsize=9, color=INK)
    for ax in axes[-1]:
        ax.set_xlabel("time (s); dotted = onset; dashed = tau1 / tau2; strip = gate mode", fontsize=7, color=MUTED)
    fig.suptitle(f"Scenario: {scenario} (seed 0)", fontsize=10, x=0.01, ha="left")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main() -> None:
    FIGS.mkdir(parents=True, exist_ok=True)
    TABLES.mkdir(parents=True, exist_ok=True)
    steps, episodes = load_batch("balance")
    summary = summarize_batch(steps, episodes)
    cs, ce = load_batch("corridor")
    summary["corridor_progress_m"] = summarize_batch(cs, ce)["mean_progress_m"]
    summary["corridor_false_fallback_fraction"] = summarize_batch(cs, ce)["false_fallback_fraction"]
    summary.to_csv(TABLES / "metrics_summary.csv", float_format="%.4g")
    table = per_scenario_table(steps, episodes)
    table.to_csv(TABLES / "metrics_per_scenario.csv", index=False, float_format="%.4g")
    print(summary.to_string())
    print(table.to_string())
    fig_detection(table, FIGS / "detection_and_falls.png")
    for sc in ("payload_shift", "accel_noise_fault", "push"):
        fig_episode(steps, episodes, sc, FIGS / f"episode_{sc}.png")
    cost_path = RESULTS / "qubo_cost_table.parquet"
    if cost_path.exists():
        fig_sensitivity(pd.read_parquet(cost_path), FIGS / "threshold_sensitivity.png")


if __name__ == "__main__":
    main()
