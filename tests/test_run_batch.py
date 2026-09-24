"""Tests for experiments/run_batch.py (CLAUDE.md section 12, Milestone 2)."""

import tempfile
from pathlib import Path

from experiments.run_batch import ControllerKind, default_scenarios, run_batch


def test_default_scenarios_covers_nominal_and_all_six_disturbances():
    names = [s.name for s in default_scenarios()]
    assert names == [
        "nominal", "push", "surface_change", "gyro_bias_fault",
        "accel_noise_fault", "payload_shift", "battery_droop",
    ]


def test_run_batch_produces_one_episode_row_per_combination_and_writes_parquet():
    scenarios = default_scenarios()[:2]  # nominal + push
    controllers = (ControllerKind.NAIVE, ControllerKind.NIS_GATE)
    seeds = (0, 1)

    with tempfile.TemporaryDirectory() as tmp:
        out_dir = Path(tmp)
        episodes_df, steps_df = run_batch(
            controllers=controllers, scenarios=scenarios, seeds=seeds, out_dir=out_dir, n_jobs=1,
        )

        assert len(episodes_df) == len(controllers) * len(scenarios) * len(seeds)
        assert set(episodes_df["controller"]) == {"naive", "nis_gate"}
        assert set(episodes_df["scenario"]) == {"nominal", "push"}
        assert (out_dir / "episodes.parquet").exists()
        assert (out_dir / "steps.parquet").exists()

        # steps_df has real per-tick rows tagged with the same combination keys
        assert len(steps_df) == episodes_df["n_steps"].sum()
        for col in ("controller", "scenario", "seed"):
            assert col in steps_df.columns
