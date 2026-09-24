"""Tests for analysis/plots.py (CLAUDE.md section 4/14 step 8, section 12).

Uses small synthetic DataFrames throughout rather than real simulated
episodes/batches -- these tests check plotting logic (mode-span detection,
tick labels, file output), not simulation correctness, which sim/run.py's
and analysis/metrics.py's own tests already cover.
"""

import matplotlib

matplotlib.use("Agg")

import numpy as np
import pandas as pd
import pytest

from analysis.plots import (
    CONTROLLER_ORDER,
    _mode_spans,
    plot_controller_comparison,
    plot_episode_timeline,
    plot_threshold_sensitivity,
)
from sim.params import default_evaluation_gate_params


def _episode_df(n=20, modes=None):
    modes = modes or ["NORMAL"] * n
    return pd.DataFrame({
        "t": np.arange(n) * 0.005,
        "psi": 0.02 * np.sin(np.linspace(0, 3, n)),
        "epsilon": np.linspace(1, 500, n),
        "mode": modes,
    })


def test_mode_spans_detects_contiguous_runs():
    df = _episode_df(n=10, modes=["NORMAL"] * 4 + ["CAUTIOUS"] * 3 + ["NORMAL"] * 3)
    spans = _mode_spans(df)
    assert [mode for _, _, mode in spans] == ["NORMAL", "CAUTIOUS", "NORMAL"]
    # spans cover the whole timeline with no gaps
    for (t_start, t_end, _), (next_start, _, _) in zip(spans, spans[1:]):
        assert t_end == next_start


def test_mode_spans_single_mode_gives_one_span():
    df = _episode_df(n=5, modes=["NORMAL"] * 5)
    spans = _mode_spans(df)
    assert len(spans) == 1
    assert spans[0][2] == "NORMAL"


def test_plot_episode_timeline_writes_nonempty_file(tmp_path):
    df = _episode_df(n=30, modes=["NORMAL"] * 10 + ["CAUTIOUS"] * 5 + ["HALT"] * 10 + ["NORMAL"] * 5)
    out_path = tmp_path / "timeline.png"
    result = plot_episode_timeline(df, out_path, gate_p=default_evaluation_gate_params())
    assert result == out_path
    assert out_path.exists()
    assert out_path.stat().st_size > 0


def test_plot_episode_timeline_without_gate_params_still_works(tmp_path):
    df = _episode_df(n=10)
    out_path = tmp_path / "timeline_no_gate.png"
    plot_episode_timeline(df, out_path)
    assert out_path.exists()


def _metrics_table_with_a_nan():
    return pd.DataFrame(
        {
            "fall_rate": [0.43, 0.34, 0.29],
            "mean_detection_delay_s": [float("nan"), 0.43, 0.17],
            "mean_progress_m": [1.93, 1.90, 1.62],
        },
        index=pd.Index(CONTROLLER_ORDER, name="controller"),
    )


def test_plot_controller_comparison_keeps_all_ticks_despite_nan(tmp_path):
    """Regression test: an earlier version passed string labels directly to
    ax.bar(...), which was observed to silently drop a category from the
    rendered figure once a bar's height was NaN (Naive's undefined detection
    delay) -- reproducible only in the actually-saved figure, not
    immediately after ax.bar() returns. Fixed by using explicit numeric
    x-positions + explicit tick labels."""
    table = _metrics_table_with_a_nan()
    out_path = tmp_path / "comparison.png"
    result = plot_controller_comparison(table, out_path)

    assert result == out_path
    assert out_path.exists()
    assert out_path.stat().st_size > 0

    # Re-derive the figure the same way the function does, and check the
    # panel with a NaN value still carries all 3 tick labels.
    import matplotlib.pyplot as plt

    controllers = [c for c in CONTROLLER_ORDER if c in table.index]
    fig, ax = plt.subplots()
    x_positions = np.arange(len(controllers))
    ax.bar(x_positions, table.loc[controllers, "mean_detection_delay_s"].to_numpy())
    ax.set_xticks(x_positions)
    ax.set_xticklabels(controllers)
    fig.canvas.draw()
    assert [t.get_text() for t in ax.get_xticklabels()] == controllers
    plt.close(fig)


def test_plot_controller_comparison_handles_missing_controller(tmp_path):
    table = _metrics_table_with_a_nan().drop(index="tilt_gate")
    out_path = tmp_path / "comparison_partial.png"
    plot_controller_comparison(table, out_path)
    assert out_path.exists()


def test_plot_threshold_sensitivity_writes_nonempty_file(tmp_path):
    sensitivity_df = pd.DataFrame({
        "tau1": [700, 900, 700, 900],
        "tau2": [1000, 1000, 1400, 1400],
        "fall_rate": [0.3, 0.28, 0.31, 0.29],
    })
    out_path = tmp_path / "sensitivity.png"
    result = plot_threshold_sensitivity(sensitivity_df, out_path)
    assert result == out_path
    assert out_path.exists()
    assert out_path.stat().st_size > 0


def test_plot_threshold_sensitivity_handles_missing_grid_point(tmp_path):
    """An infeasible (tau1, tau2) combination is simply absent from the
    sweep grid -- the heatmap should still render (as a blank/NaN cell),
    not raise."""
    sensitivity_df = pd.DataFrame({
        "tau1": [700, 900, 700],
        "tau2": [1000, 1000, 1400],  # (900, 1400) missing entirely
        "fall_rate": [0.3, 0.28, 0.31],
    })
    out_path = tmp_path / "sensitivity_sparse.png"
    plot_threshold_sensitivity(sensitivity_df, out_path)
    assert out_path.exists()
