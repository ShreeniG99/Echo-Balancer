import pandas as pd

from experiments.run_batch import DISTURBANCE_SCENARIOS, run_batch
from sim.run import ControllerType


def test_run_batch_balance_only_smoke_test(tmp_path, monkeypatch):
    """Short T and few seeds -- this is a wiring/schema smoke test, not a
    calibration check (those live in tests/test_run.py)."""
    monkeypatch.setattr("experiments.run_batch.SEEDS", [0, 1])
    monkeypatch.setattr("experiments.run_batch.CONTROLLERS", [ControllerType.NAIVE, ControllerType.NIS_GATE])
    # Shrink every scenario's duration so the smoke test runs fast.
    short_scenarios = {
        name: (fn, 1.0, onset, x0) for name, (fn, _T, onset, x0) in DISTURBANCE_SCENARIOS.items()
    }
    monkeypatch.setattr("experiments.run_batch.DISTURBANCE_SCENARIOS", short_scenarios)

    steps_df, episodes_df = run_batch(wall_following=False, scenario_names=["nominal", "push"])

    assert isinstance(steps_df, pd.DataFrame)
    assert isinstance(episodes_df, pd.DataFrame)
    assert len(episodes_df) == 2 * 2 * 2  # 2 controllers x 2 scenarios x 2 seeds
    assert set(episodes_df["episode_id"]) == set(steps_df["episode_id"].unique())
    assert {"episode_id", "controller", "scenario", "seed", "wall_following", "disturbance_onset", "fell"} <= set(episodes_df.columns)


def test_run_batch_writes_readable_parquet(tmp_path, monkeypatch):
    monkeypatch.setattr("experiments.run_batch.SEEDS", [0])
    monkeypatch.setattr("experiments.run_batch.CONTROLLERS", [ControllerType.NAIVE])
    short_scenarios = {"nominal": (None, 1.0, None, 0.0)}
    monkeypatch.setattr("experiments.run_batch.DISTURBANCE_SCENARIOS", short_scenarios)
    monkeypatch.setattr("experiments.run_batch.RESULTS_DIR", tmp_path)

    from experiments.run_batch import main
    main()

    steps = pd.read_parquet(tmp_path / "steps_balance.parquet")
    episodes = pd.read_parquet(tmp_path / "episodes_balance.parquet")
    assert len(episodes) == 1
    assert len(steps) == int(round(1.0 / 0.005))
    steps_cor = pd.read_parquet(tmp_path / "steps_corridor.parquet")
    episodes_cor = pd.read_parquet(tmp_path / "episodes_corridor.parquet")
    assert len(episodes_cor) == 1
