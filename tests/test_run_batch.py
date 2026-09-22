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
    # Regression guard for the cross-process scenario-passing fix in
    # experiments/run_batch.py's _run_one/run_batch: this monkeypatches
    # DISTURBANCE_SCENARIOS["nominal"] duration down to T=1.0s, but the
    # on-disk (un-monkeypatched) module-level default is T=60.0s. If a
    # worker process ever went back to looking up scenario duration from
    # its own fresh (re-imported) copy of DISTURBANCE_SCENARIOS instead of
    # using the parent-resolved tuple passed through delayed(), this row
    # count would silently become int(round(60.0 / 0.005)) == 12000 instead
    # of 200 -- i.e. this assertion is deliberately sensitive to exactly
    # that bug, not just an incidental sanity check on row count.
    assert len(steps) == int(round(1.0 / 0.005))
    steps_cor = pd.read_parquet(tmp_path / "steps_corridor.parquet")
    episodes_cor = pd.read_parquet(tmp_path / "episodes_corridor.parquet")
    assert len(episodes_cor) == 1


def test_run_batch_steps_df_groups_correctly_per_episode(monkeypatch):
    """Regression guard for the cross-process scenario-passing fix: with
    heterogeneous episode durations in flight simultaneously (via joblib's
    parallel workers), each episode's rows in the concatenated steps_df
    must have the row count implied by ITS OWN scenario duration, not
    another job's -- this is exactly the kind of mixup the original
    (buggy) design, where each worker independently re-derived its
    scenario from possibly-stale module state, could produce."""
    monkeypatch.setattr("experiments.run_batch.SEEDS", [0])
    monkeypatch.setattr("experiments.run_batch.CONTROLLERS", [ControllerType.NAIVE, ControllerType.NIS_GATE])
    mixed_scenarios = {
        "nominal": (None, 1.0, None, 0.0),
        "push": (DISTURBANCE_SCENARIOS["push"][0], 2.0, DISTURBANCE_SCENARIOS["push"][1], 0.0),
    }
    monkeypatch.setattr("experiments.run_batch.DISTURBANCE_SCENARIOS", mixed_scenarios)

    steps_df, episodes_df = run_batch(wall_following=False, scenario_names=["nominal", "push"])

    for _, ep in episodes_df.iterrows():
        ep_steps = steps_df[steps_df["episode_id"] == ep["episode_id"]]
        expected_T, expected_rows = (1.0, 200) if ep["scenario"] == "nominal" else (2.0, 400)
        assert len(ep_steps) == expected_rows, f"{ep['episode_id']}: expected {expected_rows} rows for T={expected_T}, got {len(ep_steps)}"
        # every row in this episode's slice really belongs to it (no cross-episode contamination)
        assert (ep_steps["episode_id"] == ep["episode_id"]).all()
