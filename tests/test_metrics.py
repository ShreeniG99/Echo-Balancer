"""Tests for analysis/metrics.py (CLAUDE.md section 12)."""

import pandas as pd

from analysis.metrics import (
    build_metrics_table,
    detection_delay,
    fall_rate,
    false_fallback,
    missed_fallback_rate,
    progress,
)


def _episode(controller, scenario, seed, onset, fell, fall_time, prog):
    return dict(
        controller=controller, scenario=scenario, seed=seed, onset=onset,
        fell=fell, fall_time=fall_time, progress=prog,
    )


def _steps(controller, scenario, seed, ts, modes):
    return pd.DataFrame(dict(
        controller=controller, scenario=scenario, seed=seed, t=ts, mode=modes,
    ))


def test_fall_rate():
    episodes = pd.DataFrame([
        _episode("naive", "push", 0, 5.0, True, 6.0, 1.0),
        _episode("naive", "push", 1, 5.0, False, None, 2.0),
        _episode("nis_gate", "push", 0, 5.0, False, None, 1.5),
    ])
    rates = fall_rate(episodes)
    assert rates["naive"] == 0.5
    assert rates["nis_gate"] == 0.0


def test_false_fallback_only_uses_nominal_scenario():
    steps = pd.concat([
        _steps("nis_gate", "nominal", 0, [0.0, 0.1, 0.2, 0.3], ["NORMAL", "NORMAL", "CAUTIOUS", "NORMAL"]),
        _steps("nis_gate", "push", 0, [0.0, 0.1], ["NORMAL", "CAUTIOUS"]),  # must be excluded
    ])
    result = false_fallback(pd.DataFrame(), steps, dt_control=0.1)
    assert result.loc["nis_gate", "false_fallback_frac"] == 0.25  # 1 of 4 nominal samples non-NORMAL
    assert result.loc["nis_gate", "fallback_events_per_min"] > 0


def test_missed_fallback_rate_true_when_still_normal_before_fall():
    episodes = pd.DataFrame([_episode("naive", "surface_change", 0, 5.0, True, 10.0, 3.0)])
    steps = _steps("naive", "surface_change", 0, [9.0, 9.4, 9.6, 10.0], ["NORMAL", "NORMAL", "NORMAL", "NORMAL"])
    result = missed_fallback_rate(episodes, steps, warning_s=0.5)
    assert result["naive"] == 1.0  # still NORMAL at t=9.5 (fall_time - 0.5) -> missed


def test_missed_fallback_rate_false_when_gate_had_already_reacted():
    episodes = pd.DataFrame([_episode("nis_gate", "surface_change", 0, 5.0, True, 10.0, 3.0)])
    steps = _steps("nis_gate", "surface_change", 0, [9.0, 9.4, 9.6, 10.0], ["NORMAL", "HALT", "HALT", "HALT"])
    result = missed_fallback_rate(episodes, steps, warning_s=0.5)
    assert result["nis_gate"] == 0.0


def test_missed_fallback_rate_nan_when_no_falls():
    episodes = pd.DataFrame([_episode("nis_gate", "push", 0, 5.0, False, None, 3.0)])
    result = missed_fallback_rate(episodes, pd.DataFrame(columns=["controller", "scenario", "seed", "t", "mode"]))
    assert pd.isna(result["nis_gate"])


def test_detection_delay_excludes_nominal_and_counts_misses():
    episodes = pd.DataFrame([
        _episode("nis_gate", "nominal", 0, None, False, None, 3.0),
        _episode("nis_gate", "push", 0, 5.0, False, None, 3.0),
        _episode("nis_gate", "push", 1, 5.0, False, None, 3.0),
    ])
    steps = pd.concat([
        _steps("nis_gate", "nominal", 0, [0.0, 1.0], ["NORMAL", "NORMAL"]),
        _steps("nis_gate", "push", 0, [4.9, 5.0, 5.2], ["NORMAL", "NORMAL", "CAUTIOUS"]),
        _steps("nis_gate", "push", 1, [4.9, 5.0, 5.2], ["NORMAL", "NORMAL", "NORMAL"]),  # never detected
    ])
    result = detection_delay(episodes, steps)
    row = result.loc["nis_gate"]
    assert row["n_disturbance_episodes"] == 2  # nominal excluded
    assert row["missed_detections"] == 1
    assert abs(row["mean_detection_delay_s"] - 0.2) < 1e-9


def test_progress_averages_per_controller():
    episodes = pd.DataFrame([
        _episode("naive", "nominal", 0, None, False, None, 4.0),
        _episode("naive", "nominal", 1, None, False, None, 6.0),
    ])
    result = progress(episodes)
    assert result["naive"] == 5.0


def test_build_metrics_table_end_to_end_small_batch():
    """Wires run_batch's real output into build_metrics_table on a tiny
    (fast) subset -- confirms the two modules' DataFrame schemas actually
    match, not just each function's math in isolation."""
    from experiments.run_batch import ControllerKind, default_scenarios, run_batch

    scenarios = default_scenarios()
    episodes_df, steps_df = run_batch(
        controllers=(ControllerKind.NAIVE, ControllerKind.NIS_GATE),
        scenarios=[scenarios[0], scenarios[1]],  # nominal + push
        seeds=(0, 1),
        out_dir=_tmp_out_dir(),
        n_jobs=1,
    )
    table = build_metrics_table(episodes_df, steps_df)
    assert set(table.index) == {"naive", "nis_gate"}
    assert {
        "fall_rate", "false_fallback_frac", "fallback_events_per_min",
        "missed_fallback_rate", "mean_detection_delay_s", "missed_detections",
        "n_disturbance_episodes", "mean_progress_m",
    } <= set(table.columns)


def _tmp_out_dir():
    import tempfile
    from pathlib import Path

    return Path(tempfile.mkdtemp())
