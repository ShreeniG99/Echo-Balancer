import numpy as np
import pandas as pd
import pytest

from analysis.metrics import (
    detection_delay,
    fall_rate,
    false_fallback_fraction,
    missed_fallback,
    progress,
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
