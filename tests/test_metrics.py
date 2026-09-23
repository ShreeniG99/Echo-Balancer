import numpy as np
import pandas as pd
import pytest

from analysis.metrics import (
    detection_delay,
    fall_rate,
    false_fallback_fraction,
    missed_fallback,
    progress,
    summarize_batch,
)


def _episode(episode_id, controller, scenario, fell, disturbance_onset=np.nan):
    return {
        "episode_id": episode_id, "controller": controller, "scenario": scenario,
        "seed": 0, "wall_following": False, "disturbance_onset": disturbance_onset, "fell": fell,
    }


def _step(episode_id, t, mode, theta=0.0, fallen=False):
    return {"episode_id": episode_id, "t": t, "mode": mode, "theta": theta, "fallen": fallen}


def test_fall_rate_computes_fraction_per_controller():
    episodes_df = pd.DataFrame([
        _episode("e1", "naive", "nominal", fell=False),
        _episode("e2", "naive", "push", fell=True),
        _episode("e3", "nis_gate", "nominal", fell=False),
        _episode("e4", "nis_gate", "push", fell=False),
    ])

    result = fall_rate(episodes_df)

    assert result["naive"] == 0.5
    assert result["nis_gate"] == 0.0


def test_false_fallback_fraction_uses_only_nominal_scenario():
    episodes_df = pd.DataFrame([
        _episode("e1", "nis_gate", "nominal", fell=False),
        _episode("e2", "nis_gate", "push", fell=False),  # excluded: not nominal
    ])
    steps_df = pd.DataFrame([
        _step("e1", 0.0, "NORMAL"), _step("e1", 0.005, "NORMAL"),
        _step("e1", 0.010, "CAUTIOUS"), _step("e1", 0.015, "NORMAL"),
        _step("e2", 0.0, "HALT"), _step("e2", 0.005, "HALT"),  # ignored: not the nominal scenario
    ])

    result = false_fallback_fraction(steps_df, episodes_df)

    assert result["nis_gate"] == 0.25  # 1 of 4 nominal-episode ticks was non-NORMAL


def test_detection_delay_measures_onset_to_first_non_normal():
    episodes_df = pd.DataFrame([_episode("e1", "nis_gate", "push", fell=False, disturbance_onset=5.0)])
    steps_df = pd.DataFrame([
        _step("e1", 4.995, "NORMAL"), _step("e1", 5.000, "NORMAL"),
        _step("e1", 5.005, "NORMAL"), _step("e1", 5.010, "CAUTIOUS"),
    ])

    result = detection_delay(steps_df, episodes_df)

    assert result.loc[result["episode_id"] == "e1", "detection_delay_s"].iloc[0] == pytest.approx(0.010, abs=1e-9)


def test_detection_delay_is_nan_when_never_detected():
    episodes_df = pd.DataFrame([_episode("e1", "tilt_threshold", "push", fell=False, disturbance_onset=5.0)])
    steps_df = pd.DataFrame([_step("e1", 5.0, "NORMAL"), _step("e1", 6.0, "NORMAL")])

    result = detection_delay(steps_df, episodes_df)

    assert np.isnan(result.loc[result["episode_id"] == "e1", "detection_delay_s"].iloc[0])


def test_progress_is_wheel_radius_times_delta_theta():
    episodes_df = pd.DataFrame([_episode("e1", "naive", "nominal", fell=False)])
    steps_df = pd.DataFrame([
        _step("e1", 0.0, "NORMAL", theta=0.0),
        _step("e1", 1.0, "NORMAL", theta=2.5),
    ])

    result = progress(steps_df, episodes_df)

    from sim.params import default_plant_params
    expected = default_plant_params().R * 2.5
    assert result.loc[result["episode_id"] == "e1", "progress_m"].iloc[0] == pytest.approx(expected, abs=1e-9)


def test_missed_fallback_true_when_still_normal_half_a_second_before_the_fall():
    episodes_df = pd.DataFrame([_episode("e1", "nis_gate", "push", fell=True, disturbance_onset=5.0)])
    steps_df = pd.DataFrame([
        _step("e1", 4.0, "NORMAL"),
        _step("e1", 4.5, "NORMAL"),   # exactly 0.5s before the fall -- still NORMAL
        _step("e1", 4.9, "NORMAL"),
        _step("e1", 5.0, "NORMAL", fallen=True),  # the fall itself
    ])

    result = missed_fallback(steps_df, episodes_df)

    assert bool(result.loc[result["episode_id"] == "e1", "missed_fallback"].iloc[0]) is True


def test_missed_fallback_false_when_already_non_normal_half_a_second_before_the_fall():
    episodes_df = pd.DataFrame([_episode("e1", "nis_gate", "push", fell=True, disturbance_onset=5.0)])
    steps_df = pd.DataFrame([
        _step("e1", 4.0, "NORMAL"),
        _step("e1", 4.5, "CAUTIOUS"),  # already caught 0.5s before the fall
        _step("e1", 4.9, "CAUTIOUS"),
        _step("e1", 5.0, "CAUTIOUS", fallen=True),
    ])

    result = missed_fallback(steps_df, episodes_df)

    assert bool(result.loc[result["episode_id"] == "e1", "missed_fallback"].iloc[0]) is False


def test_missed_fallback_only_includes_fallen_episodes():
    episodes_df = pd.DataFrame([
        _episode("e1", "nis_gate", "push", fell=True, disturbance_onset=5.0),
        _episode("e2", "nis_gate", "nominal", fell=False),
    ])
    steps_df = pd.DataFrame([
        _step("e1", 5.0, "NORMAL", fallen=True),
        _step("e2", 5.0, "NORMAL"),
    ])

    result = missed_fallback(steps_df, episodes_df)

    assert set(result["episode_id"]) == {"e1"}


def test_missed_fallback_empty_but_correctly_columned_when_no_fallen_episodes():
    # No episode fell -- the loop body never runs. Regression test: this used
    # to return pd.DataFrame([]), a zero-column frame that crashes any
    # downstream .groupby("controller").
    episodes_df = pd.DataFrame([
        _episode("e1", "nis_gate", "nominal", fell=False),
        _episode("e2", "naive", "nominal", fell=False),
    ])
    steps_df = pd.DataFrame([
        _step("e1", 0.0, "NORMAL"),
        _step("e2", 0.0, "NORMAL"),
    ])

    result = missed_fallback(steps_df, episodes_df)

    assert list(result.columns) == ["episode_id", "controller", "missed_fallback"]
    assert len(result) == 0


def test_detection_delay_empty_but_correctly_columned_when_no_disturbance_onset():
    # Every episode is the "nominal" scenario (disturbance_onset is NaN for
    # all of them) -- the loop body never runs. Regression test: this used
    # to return pd.DataFrame([]), a zero-column frame that crashes any
    # downstream .groupby("controller").
    episodes_df = pd.DataFrame([
        _episode("e1", "nis_gate", "nominal", fell=False),
        _episode("e2", "naive", "nominal", fell=False),
    ])
    steps_df = pd.DataFrame([
        _step("e1", 0.0, "NORMAL"),
        _step("e2", 0.0, "NORMAL"),
    ])

    result = detection_delay(steps_df, episodes_df)

    assert list(result.columns) == ["episode_id", "controller", "scenario", "detection_delay_s"]
    assert len(result) == 0


def test_progress_empty_but_correctly_columned_when_no_steps():
    # steps_df has zero rows -- the groupby("episode_id") loop body never
    # runs. Regression test: this used to return pd.DataFrame([]), a
    # zero-column frame whose .merge(episodes_df[...], on="episode_id")
    # raises KeyError: 'episode_id'.
    episodes_df = pd.DataFrame([
        _episode("e1", "nis_gate", "nominal", fell=False),
        _episode("e2", "naive", "nominal", fell=False),
    ])
    steps_df = pd.DataFrame(columns=["episode_id", "t", "theta"])

    result = progress(steps_df, episodes_df)

    assert list(result.columns) == ["episode_id", "progress_m", "controller", "scenario"]
    assert len(result) == 0


def test_summarize_batch_handles_corridor_nominal_only_shape():
    # Reproduces experiments/run_batch.py's own
    # run_batch(wall_following=True, scenario_names=["nominal"]) shape: every
    # episode is the nominal scenario (zero disturbed episodes, so
    # detection_delay is empty) and, in this particular batch, zero fallen
    # episodes too (so missed_fallback would also be empty). Before the fix,
    # summarize_batch crashed with KeyError: 'controller' because
    # detection_delay(...).groupby("controller") ran on a zero-column frame.
    episodes_df = pd.DataFrame([
        _episode("e1", "naive", "nominal", fell=False),
        _episode("e2", "nis_gate", "nominal", fell=False),
        _episode("e3", "tilt_threshold", "nominal", fell=False),
    ])
    steps_df = pd.DataFrame([
        _step("e1", 0.0, "NORMAL", theta=0.0), _step("e1", 1.0, "NORMAL", theta=1.0),
        _step("e2", 0.0, "NORMAL", theta=0.0), _step("e2", 1.0, "CAUTIOUS", theta=0.8),
        _step("e3", 0.0, "NORMAL", theta=0.0), _step("e3", 1.0, "NORMAL", theta=1.2),
    ])

    result = summarize_batch(steps_df, episodes_df)  # must not raise

    assert (result["fall_rate"] == 0.0).all()
    assert result.loc["naive", "false_fallback_fraction"] == pytest.approx(0.0)
    assert result.loc["nis_gate", "false_fallback_fraction"] == pytest.approx(0.5)
    assert result.loc["tilt_threshold", "false_fallback_fraction"] == pytest.approx(0.0)
    # No episode had a known disturbance onset, so there is nothing to
    # average -- every controller's mean detection delay is NaN, not a
    # missing row or a crash.
    assert result["mean_detection_delay_s"].isna().all()
    # mean_progress_m is still computable from progress() alone.
    from sim.params import default_plant_params
    R = default_plant_params().R
    assert result.loc["naive", "mean_progress_m"] == pytest.approx(R * 1.0)
    assert result.loc["nis_gate", "mean_progress_m"] == pytest.approx(R * 0.8)
    assert result.loc["tilt_threshold", "mean_progress_m"] == pytest.approx(R * 1.2)
