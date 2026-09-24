"""Tests for analysis/animate.py (CLAUDE.md section 14 step 8)."""

import numpy as np
import pandas as pd
import pytest

from analysis.animate import _body_frame, _pose_transform


def test_body_frame_is_orthonormal_for_various_angles():
    for phi in [0.0, 0.3, -1.2, np.pi]:
        for psi in [0.0, 0.1, -0.3]:
            forward, lateral, up = _body_frame(phi, psi)
            for v in (forward, lateral, up):
                assert abs(np.linalg.norm(v) - 1.0) < 1e-9
            assert abs(np.dot(forward, lateral)) < 1e-9
            assert abs(np.dot(forward, up)) < 1e-9
            assert abs(np.dot(lateral, up)) < 1e-9


def test_body_frame_at_zero_pose_matches_world_axes():
    forward, lateral, up = _body_frame(phi=0.0, psi=0.0)
    assert np.allclose(forward, [1.0, 0.0, 0.0])
    assert np.allclose(lateral, [0.0, 1.0, 0.0])
    assert np.allclose(up, [0.0, 0.0, 1.0])


def test_positive_psi_leans_up_vector_toward_forward_travel():
    """psi > 0 means 'leaning forward' (CLAUDE.md section 5) -- the body's
    up vector should tilt toward the direction of travel, not away from
    it."""
    forward_yaw_only = np.array([1.0, 0.0, 0.0])
    _, _, up = _body_frame(phi=0.0, psi=0.2)
    assert np.dot(up, forward_yaw_only) > 0


def test_pose_transform_places_center_and_preserves_rotation_columns():
    forward, lateral, up = _body_frame(phi=0.4, psi=-0.1)
    center = np.array([1.0, 2.0, 0.3])
    M = _pose_transform(center, forward, lateral, up)

    assert np.allclose(M[:3, 3], center)
    assert np.allclose(M[:3, 0], forward)
    assert np.allclose(M[:3, 1], lateral)
    assert np.allclose(M[:3, 2], up)
    assert np.allclose(M[3, :], [0, 0, 0, 1])
    # a valid rotation submatrix is orthonormal (det = +1)
    assert abs(np.linalg.det(M[:3, :3]) - 1.0) < 1e-9


def test_render_episode_writes_a_nonempty_mp4(tmp_path):
    from analysis.animate import render_episode

    n = 15
    df = pd.DataFrame({
        "t": np.arange(n) * 0.005,
        "x": np.linspace(0, 0.3, n),
        "y": 0.5 + 0.05 * np.sin(np.linspace(0, 3, n)),
        "phi": 0.05 * np.sin(np.linspace(0, 3, n)),
        "psi": 0.02 * np.sin(np.linspace(0, 6, n)),
        "mode": ["NORMAL"] * 5 + ["CAUTIOUS"] * 5 + ["HALT"] * 5,
    })

    out_path = tmp_path / "episode.mp4"
    result_path = render_episode(df, out_path, fps=10)

    assert result_path == out_path
    assert out_path.exists()
    assert out_path.stat().st_size > 0


def test_render_episode_raises_on_empty_dataframe(tmp_path):
    from analysis.animate import render_episode

    empty = pd.DataFrame(columns=["t", "x", "y", "phi", "psi", "mode"])
    with pytest.raises(ValueError):
        render_episode(empty, tmp_path / "out.mp4")
