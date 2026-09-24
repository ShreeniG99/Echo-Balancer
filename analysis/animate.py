"""3D animation of a logged episode -> MP4 (CLAUDE.md section 14 step 8;
see section 2's scope change from the original 2D side/top view plan).

Renders sim.run.run_episode's DataFrame with PyVista/VTK, entirely
off-screen: a robot (body box + two wheel cylinders) posed each frame from
the logged [x, y, phi, psi] state, two corridor walls + floor, a chase
camera following the robot down the corridor, and a status light above
the robot colored by the logged gate `mode` (green=NORMAL, gold=CAUTIOUS,
crimson=HALT). No physics is computed here -- everything drawn is a
value sim.run already simulated and logged; this module only poses and
renders it (CLAUDE.md section 2: not a physics engine, not a GUI, not
CAD).

Geometry/orientation derivation: at psi=phi=0 the robot faces +x with the
corridor's lateral axis along +y and up along +z. `_body_frame` builds
the robot's orthonormal (forward, lateral, up) frame for any (phi, psi):
- `lateral` is yaw-only (psi is a pitch about the lateral axis, so the
  lateral axis itself is unaffected by pitch): l = (-sin(phi), cos(phi), 0).
- `forward`/`up` are `world forward`/`world up` each rotated by psi within
  their own (forward, up) plane (a pitch rotation about `lateral`):
  up = cos(psi)*world_up + sin(psi)*forward_yaw_only,
  forward = -sin(psi)*world_up + cos(psi)*forward_yaw_only.
These three vectors are mutually orthonormal by construction (a pure
rotation of an orthonormal frame), so they form a valid rotation matrix
for `_pose_transform`.
"""

import os
import shutil
import subprocess
import time
from pathlib import Path
from typing import Optional

import numpy as np
import pandas as pd

from sim.params import CorridorParams, PlantParams, default_corridor_params, default_plant_params

MODE_COLORS = {"NORMAL": "limegreen", "CAUTIOUS": "gold", "HALT": "crimson"}
DEFAULT_DT_CONTROL = 0.005  # sim.run.run_episode's fixed control-loop dt


def _ensure_display() -> None:
    """Starts a virtual X server if no display is already set (this
    project's own dev container has no GPU/EGL/OSMesa -- CLAUDE.md
    section 3). Mirrors `xvfb-run -a` from inside Python so callers
    (including pytest) don't need to remember to wrap the whole process;
    a no-op if DISPLAY is already set (a real display, or an Xvfb this
    process already started) or if Xvfb isn't installed at all.
    """
    if os.environ.get("DISPLAY"):
        return
    if shutil.which("Xvfb") is None:
        return
    subprocess.Popen(
        ["Xvfb", ":99", "-screen", "0", "1280x720x24"],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
    )
    os.environ["DISPLAY"] = ":99"
    time.sleep(1.0)


def _body_frame(phi: float, psi: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Returns (forward, lateral, up) unit vectors for the robot's body
    frame -- see module docstring for the derivation."""
    forward_yaw_only = np.array([np.cos(phi), np.sin(phi), 0.0])
    lateral = np.array([-np.sin(phi), np.cos(phi), 0.0])
    world_up = np.array([0.0, 0.0, 1.0])
    up = np.cos(psi) * world_up + np.sin(psi) * forward_yaw_only
    forward = -np.sin(psi) * world_up + np.cos(psi) * forward_yaw_only
    return forward, lateral, up


def _pose_transform(center: np.ndarray, forward: np.ndarray, lateral: np.ndarray, up: np.ndarray) -> np.ndarray:
    """4x4 homogeneous transform placing a mesh built in its own local
    (x=forward, y=lateral, z=up) frame at `center` with this orientation."""
    M = np.eye(4)
    M[:3, 0] = forward
    M[:3, 1] = lateral
    M[:3, 2] = up
    M[:3, 3] = center
    return M


def render_episode(
    df: pd.DataFrame,
    out_path: Path,
    plant_p: Optional[PlantParams] = None,
    corridor_p: Optional[CorridorParams] = None,
    fps: int = 30,
    dt_control: float = DEFAULT_DT_CONTROL,
    window_size: tuple[int, int] = (1280, 720),
) -> Path:
    """Renders `df` (a sim.run.run_episode DataFrame -- needs at least
    t, x, y, phi, psi, mode) to an MP4 at `out_path`, sampling one frame
    every `round(1/(fps*dt_control))` control ticks so the output plays
    back at approximately real-time speed. Returns out_path.
    """
    _ensure_display()
    import pyvista as pv  # imported after _ensure_display sets DISPLAY

    plant_p = default_plant_params() if plant_p is None else plant_p
    corridor_p = default_corridor_params() if corridor_p is None else corridor_p

    frame_stride = max(1, round(1.0 / (fps * dt_control)))
    frames = df.iloc[::frame_stride].reset_index(drop=True)
    if len(frames) == 0:
        raise ValueError("df has no rows to render")

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    plotter = pv.Plotter(off_screen=True, window_size=list(window_size))
    plotter.set_background("black")

    x_min, x_max = float(df["x"].min()) - 0.3, float(df["x"].max()) + 0.5
    length = max(x_max - x_min, 1.0)
    x_mid = (x_min + x_max) / 2

    floor = pv.Plane(center=(x_mid, corridor_p.width / 2, 0.0), direction=(0, 0, 1), i_size=length, j_size=corridor_p.width)
    wall_height = 0.4
    wall_near = pv.Plane(center=(x_mid, 0.0, wall_height / 2), direction=(0, 1, 0), i_size=length, j_size=wall_height)
    wall_far = pv.Plane(
        center=(x_mid, corridor_p.width, wall_height / 2), direction=(0, 1, 0), i_size=length, j_size=wall_height
    )
    plotter.add_mesh(floor, color="dimgray")
    plotter.add_mesh(wall_near, color="steelblue", opacity=0.45)
    plotter.add_mesh(wall_far, color="steelblue", opacity=0.45)

    # Meshes are built in their own local (x=forward, y=lateral, z=up)
    # frame, centered at the origin, so _pose_transform's rotation columns
    # place them correctly however the robot is posed each frame.
    body_mesh = pv.Cube(x_length=plant_p.D, y_length=plant_p.W * 0.9, z_length=plant_p.H)
    wheel_mesh = pv.Cylinder(radius=plant_p.R, height=0.018, direction=(0, 1, 0), resolution=24)
    light_mesh = pv.Sphere(radius=0.025)

    body_actor = plotter.add_mesh(body_mesh, color="orange", specular=0.6)
    wheel_l_actor = plotter.add_mesh(wheel_mesh.copy(), color="black")
    wheel_r_actor = plotter.add_mesh(wheel_mesh.copy(), color="black")
    light_actor = plotter.add_mesh(light_mesh, color=MODE_COLORS["NORMAL"])

    plotter.enable_shadows()
    plotter.add_light(pv.Light(position=(x_mid, corridor_p.width / 2, 3.0), light_type="scene light", intensity=0.9))

    plotter.open_movie(str(out_path), framerate=fps)

    half_track = plant_p.W / 2
    light_height = plant_p.H * 0.75

    for _, row in frames.iterrows():
        forward, lateral, up = _body_frame(row["phi"], row["psi"])
        axle_center = np.array([row["x"], row["y"], plant_p.R])
        body_center = axle_center + plant_p.L * up

        body_actor.user_matrix = _pose_transform(body_center, forward, lateral, up)
        wheel_l_actor.user_matrix = _pose_transform(axle_center - half_track * lateral, forward, lateral, up)
        wheel_r_actor.user_matrix = _pose_transform(axle_center + half_track * lateral, forward, lateral, up)
        light_actor.user_matrix = _pose_transform(body_center + light_height * up, forward, lateral, up)
        light_actor.prop.color = MODE_COLORS.get(row["mode"], "white")

        cam_pos = axle_center - 0.6 * forward + 0.35 * up
        focal_pt = axle_center + 0.5 * forward
        plotter.camera_position = [tuple(cam_pos), tuple(focal_pt), (0.0, 0.0, 1.0)]

        plotter.add_text(
            f"t={row['t']:.2f}s   mode={row['mode']}", position="upper_left", font_size=14, color="white", name="hud",
        )

        plotter.write_frame()

    plotter.close()
    return out_path


if __name__ == "__main__":
    import sys

    from sim.run import ControllerKind, EpisodeConfig, run_episode

    cfg = EpisodeConfig(controller=ControllerKind.NIS_GATE, T=20.0)
    df = run_episode(cfg, seed=0)
    out = render_episode(df, Path(sys.argv[1] if len(sys.argv) > 1 else "analysis/results/demo.mp4"))
    print(f"wrote {out}")
