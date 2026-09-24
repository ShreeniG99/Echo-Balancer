"""Static plots (CLAUDE.md section 4/14 step 8, alongside analysis/animate.py).

Three plots, each consuming data other modules already produce (no new
simulation logic here):

- `plot_episode_timeline`: a single sim.run.run_episode DataFrame's psi and
  epsilon over time, background-shaded by gate mode.
- `plot_controller_comparison`: analysis.metrics.build_metrics_table's
  cross-controller comparison as small-multiple bar charts.
- `plot_threshold_sensitivity`: analysis.metrics.threshold_sensitivity's
  tau1/tau2 sweep as a fall-rate heatmap (CLAUDE.md section 12: "Threshold
  sensitivity: metrics vs tau1, tau2 sweeps").

Color choices follow a status/categorical split, not arbitrary matplotlib
defaults: gate mode is a status semantic (NORMAL=good, CAUTIOUS=warning,
HALT=critical), the three controllers are a fixed categorical order (same
order in every chart), and the sensitivity heatmap uses a single sequential
hue (magnitude, not identity). Every mode band is also text-labeled, since a
status color must never carry meaning alone.
"""

from pathlib import Path
from typing import Optional

import matplotlib

matplotlib.use("Agg")  # headless: this project never opens an interactive plot window
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from sim.params import GateParams

MODE_STATUS_COLORS = {"NORMAL": "#0ca30c", "CAUTIOUS": "#fab219", "HALT": "#d03b3b"}
CONTROLLER_COLORS = {"naive": "#2a78d6", "tilt_gate": "#eb6834", "nis_gate": "#1baf7a"}
CONTROLLER_ORDER = ["naive", "tilt_gate", "nis_gate"]
CONTROLLER_LABELS = {"naive": "Naive", "tilt_gate": "Tilt-threshold", "nis_gate": "NIS gate (ours)"}
SEQUENTIAL_CMAP = "Blues"


def _mode_spans(df: pd.DataFrame) -> list[tuple[float, float, str]]:
    """Contiguous [t_start, t_end, mode] runs, for axvspan shading."""
    spans = []
    start_idx = 0
    modes = df["mode"].to_numpy()
    ts = df["t"].to_numpy()
    for i in range(1, len(modes) + 1):
        if i == len(modes) or modes[i] != modes[start_idx]:
            t_end = ts[i] if i < len(modes) else ts[-1] + (ts[-1] - ts[-2] if len(ts) > 1 else 0.0)
            spans.append((ts[start_idx], t_end, modes[start_idx]))
            start_idx = i
    return spans


def plot_episode_timeline(
    df: pd.DataFrame, out_path: Path, gate_p: Optional[GateParams] = None, figsize: tuple[float, float] = (10, 6),
) -> Path:
    """psi (deg) and epsilon over time, mode-shaded, for one episode."""
    fig, (ax_psi, ax_eps) = plt.subplots(2, 1, sharex=True, figsize=figsize)

    spans = _mode_spans(df)
    seen_modes = []
    for t_start, t_end, mode in spans:
        color = MODE_STATUS_COLORS.get(mode, "#c3c2b7")
        for ax in (ax_psi, ax_eps):
            ax.axvspan(t_start, t_end, color=color, alpha=0.18, linewidth=0)
        if mode not in seen_modes:
            seen_modes.append(mode)

    ax_psi.plot(df["t"], np.degrees(df["psi"]), color="#0b0b0b", linewidth=1.5)
    ax_psi.set_ylabel("psi (deg)")
    ax_psi.axhline(45.0, color="#d03b3b", linewidth=1.0, linestyle="--", alpha=0.6)
    ax_psi.axhline(-45.0, color="#d03b3b", linewidth=1.0, linestyle="--", alpha=0.6)
    ax_psi.grid(True, alpha=0.15)

    if "epsilon" in df.columns and df["epsilon"].notna().any():
        ax_eps.plot(df["t"], df["epsilon"], color="#0b0b0b", linewidth=1.5)
        ax_eps.set_yscale("log")
        if gate_p is not None:
            ax_eps.axhline(gate_p.tau1, color="#fab219", linewidth=1.0, linestyle="--", label=f"tau1={gate_p.tau1:g}")
            ax_eps.axhline(gate_p.tau2, color="#d03b3b", linewidth=1.0, linestyle="--", label=f"tau2={gate_p.tau2:g}")
            ax_eps.legend(loc="upper right", fontsize=8, framealpha=0.8)
    ax_eps.set_ylabel("epsilon (windowed NIS)")
    ax_eps.set_xlabel("t (s)")
    ax_eps.grid(True, alpha=0.15)

    # Status color never carries meaning alone: a labeled legend, not just shading.
    handles = [plt.Rectangle((0, 0), 1, 1, color=MODE_STATUS_COLORS.get(m, "#c3c2b7"), alpha=0.4) for m in seen_modes]
    ax_psi.legend(handles, seen_modes, loc="upper right", fontsize=8, framealpha=0.8, title="mode")

    fig.suptitle("Episode timeline")
    fig.tight_layout()
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_controller_comparison(metrics_table: pd.DataFrame, out_path: Path, figsize: tuple[float, float] = (11, 3.5)) -> Path:
    """Small-multiple bar charts (CLAUDE.md section 12's metrics), one panel
    per metric, each with one bar per controller in a fixed color/order --
    never combined onto one axis (different units/scales: a fraction, a
    delay in seconds, a distance in meters)."""
    controllers = [c for c in CONTROLLER_ORDER if c in metrics_table.index]

    panels = [
        ("fall_rate", "Fall rate", "{:.0%}"),
        ("mean_detection_delay_s", "Mean detection delay (s)", "{:.2f}"),
        ("mean_progress_m", "Mean progress (m)", "{:.2f}"),
    ]
    panels = [(col, title, fmt) for col, title, fmt in panels if col in metrics_table.columns]

    fig, axes = plt.subplots(1, len(panels), figsize=figsize)
    if len(panels) == 1:
        axes = [axes]

    for ax, (col, title, fmt) in zip(axes, panels):
        values = metrics_table.loc[controllers, col]
        colors = [CONTROLLER_COLORS.get(c, "#c3c2b7") for c in controllers]
        labels = [CONTROLLER_LABELS.get(c, c) for c in controllers]
        x_positions = np.arange(len(controllers))
        # Explicit numeric x-positions + explicit tick labels, not a string-keyed
        # ax.bar(labels, ...) call: matplotlib's categorical-axis tick inference
        # was observed (this module's development) to silently drop a category
        # whose bar height is NaN once the figure is actually drawn/saved (not
        # immediately after ax.bar() returns) -- e.g. Naive's undefined
        # detection delay -- even though the Rectangle itself is still created.
        bars = ax.bar(x_positions, values.to_numpy(), color=colors)
        ax.set_xticks(x_positions)
        ax.set_xticklabels(labels)
        ax.set_title(title, fontsize=10)
        ax.tick_params(axis="x", rotation=20, labelsize=8)
        ax.grid(True, axis="y", alpha=0.15)
        for bar, v in zip(bars, values.to_numpy()):
            if pd.notna(v):
                ax.annotate(fmt.format(v), (bar.get_x() + bar.get_width() / 2, bar.get_height()),
                            ha="center", va="bottom", fontsize=8)

    fig.suptitle("Cross-controller comparison")
    fig.tight_layout()
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


def plot_threshold_sensitivity(
    sensitivity_df: pd.DataFrame, out_path: Path, metric: str = "fall_rate", figsize: tuple[float, float] = (6, 5),
) -> Path:
    """CLAUDE.md section 12: 'Threshold sensitivity: metrics vs tau1, tau2
    sweeps' -- a fall-rate heatmap over the (tau1, tau2) grid from
    analysis.metrics.threshold_sensitivity. A single sequential hue
    (magnitude), not a categorical/rainbow scale.
    """
    pivot = sensitivity_df.pivot_table(index="tau2", columns="tau1", values=metric)
    pivot = pivot.sort_index(ascending=False)  # tau2 increasing upward

    fig, ax = plt.subplots(figsize=figsize)
    im = ax.imshow(pivot.to_numpy(), cmap=SEQUENTIAL_CMAP, aspect="auto")
    ax.set_xticks(range(len(pivot.columns)))
    ax.set_xticklabels([f"{v:g}" for v in pivot.columns])
    ax.set_yticks(range(len(pivot.index)))
    ax.set_yticklabels([f"{v:g}" for v in pivot.index])
    ax.set_xlabel("tau1")
    ax.set_ylabel("tau2")
    ax.set_title(f"{metric} vs. (tau1, tau2)")

    for i in range(pivot.shape[0]):
        for j in range(pivot.shape[1]):
            v = pivot.to_numpy()[i, j]
            if not np.isnan(v):
                ax.text(j, i, f"{v:.2f}", ha="center", va="center", fontsize=8, color="#0b0b0b")

    fig.colorbar(im, ax=ax, label=metric)
    fig.tight_layout()
    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
    return out_path


if __name__ == "__main__":
    import pandas as pd

    from sim.params import default_evaluation_gate_params
    from sim.run import ControllerKind, EpisodeConfig, run_episode

    out_dir = Path("analysis/results")

    cfg = EpisodeConfig(controller=ControllerKind.NIS_GATE, T=20.0)
    df = run_episode(cfg, seed=0)
    plot_episode_timeline(df, out_dir / "episode_timeline.png", gate_p=default_evaluation_gate_params())
    print("wrote", out_dir / "episode_timeline.png")

    episodes_path = Path("experiments/results/episodes.parquet")
    steps_path = Path("experiments/results/steps.parquet")
    if episodes_path.exists() and steps_path.exists():
        from analysis.metrics import build_metrics_table

        episodes_df = pd.read_parquet(episodes_path)
        steps_df = pd.read_parquet(steps_path)
        table = build_metrics_table(episodes_df, steps_df)
        plot_controller_comparison(table, out_dir / "controller_comparison.png")
        print("wrote", out_dir / "controller_comparison.png")
    else:
        print("skipping controller comparison: run `python -m experiments.run_batch` first")
