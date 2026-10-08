"""2D animation of one episode: side view (wheel + body pitch) and top view
(corridor), background tinted by the gate mode (CLAUDE.md section 14 step 8).

Usage: PYTHONPATH=. uv run python -m analysis.animate [scenario] [controller]
  default: gyro_bias_fault, nis_gate, corridor (wall-following) episode, seed 0.
Writes docs/figures/animation_<scenario>_<controller>.mp4 (GIF if ffmpeg is missing).
"""

import shutil
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from matplotlib import animation

from experiments.run_batch import DISTURBANCE_SCENARIOS
from sim.params import default_corridor_params, default_plant_params
from sim.run import ControllerType, EpisodeConfig, run_episode

ROOT = Path(__file__).resolve().parents[1]
FIGS = ROOT / "docs" / "figures"
FPS = 25
MODE_COLORS = {"NORMAL": "#0ca30c", "CAUTIOUS": "#fab219", "HALT": "#d03b3b"}  # reserved status colours
TINT = 0.18  # background alpha of the mode colour
INK, MUTED = "#0b0b0b", "#52514e"


def simulate(scenario: str, controller: ControllerType, seed: int = 0) -> pd.DataFrame:
    fn, T, _onset, psi0 = DISTURBANCE_SCENARIOS[scenario]
    return run_episode(EpisodeConfig(controller, T, wall_following=True, x0_psi_deg=psi0, disturbance=fn,
                                     disturbance_name=scenario), seed)


def animate(df: pd.DataFrame, title: str, path: Path) -> Path:
    p, cp = default_plant_params(), default_corridor_params()
    dt = float(df.t.iloc[1] - df.t.iloc[0])
    step = max(1, int(round(1.0 / (FPS * dt))))
    frames = df.iloc[::step].reset_index(drop=True)

    fig, (ax_side, ax_top) = plt.subplots(2, 1, figsize=(8, 6.2), gridspec_kw={"height_ratios": [1.1, 1]})
    fig.suptitle(title, fontsize=10, x=0.01, ha="left", color=INK)
    # side view: x along the floor = R*theta, body of length 2L pivoting at the axle
    ax_side.set_aspect("equal")
    ax_side.set_ylim(-0.02, 2.4 * p.L + 0.06)
    ax_side.axhline(0, color=MUTED, lw=1)
    ax_side.set_title("side view (pitch psi)", loc="left", fontsize=8, color=MUTED)
    wheel = plt.Circle((0, p.R), p.R, fill=False, lw=2, color=INK)
    ax_side.add_patch(wheel)
    (body,) = ax_side.plot([], [], lw=6, color="#2a78d6", solid_capstyle="round")
    (spoke,) = ax_side.plot([], [], lw=1.2, color=INK)
    txt = ax_side.text(0.01, 0.95, "", transform=ax_side.transAxes, va="top", fontsize=8, color=INK, family="monospace")
    # top view: corridor walls at y = 0 and y = width
    xmax = max(float(np.nanmax(df.pos_x)) + 0.2, 1.0)
    ax_top.set_xlim(-0.2, xmax)
    ax_top.set_ylim(-0.1, cp.width + 0.1)
    ax_top.set_aspect("equal")
    for yw in (0.0, cp.width):
        ax_top.axhline(yw, color=INK, lw=3)
    ax_top.set_title("top view (corridor, wall-following)", loc="left", fontsize=8, color=MUTED)
    (trail,) = ax_top.plot([], [], lw=1, color=MUTED)
    (robot,) = ax_top.plot([], [], marker="o", ms=7, color="#2a78d6")
    (head,) = ax_top.plot([], [], lw=2, color="#2a78d6")
    for ax in (ax_side, ax_top):
        ax.tick_params(colors=MUTED, labelsize=7)
        ax.spines[["top", "right"]].set_visible(False)

    def draw(i: int):
        r = frames.iloc[i]
        xw = p.R * r.theta
        cx, cy = xw, p.R
        wheel.center = (cx, cy)
        top = (cx + 2 * p.L * np.sin(r.psi), cy + 2 * p.L * np.cos(r.psi))
        body.set_data([cx, top[0]], [cy, top[1]])
        spoke.set_data([cx, cx + p.R * np.sin(r.theta)], [cy, cy + p.R * np.cos(r.theta)])
        ax_side.set_xlim(cx - 0.35, cx + 0.35)
        txt.set_text(f"t={r.t:5.2f}s  mode={r['mode']:<8} psi_deg={np.degrees(r.psi):+6.2f}  eps={r.epsilon:8.0f}")
        upto = frames.iloc[: i + 1]
        trail.set_data(upto.pos_x, upto.pos_y)
        robot.set_data([r.pos_x], [r.pos_y])
        head.set_data([r.pos_x, r.pos_x + 0.12 * np.cos(r.phi)], [r.pos_y, r.pos_y + 0.12 * np.sin(r.phi)])
        col = MODE_COLORS[r["mode"]]
        for ax in (ax_side, ax_top):
            ax.set_facecolor(matplotlib.colors.to_rgba(col, TINT))
        return wheel, body, spoke, txt, trail, robot, head

    anim = animation.FuncAnimation(fig, draw, frames=len(frames), interval=1000 / FPS, blit=False)
    path.parent.mkdir(parents=True, exist_ok=True)
    if shutil.which("ffmpeg"):
        anim.save(path, writer=animation.FFMpegWriter(fps=FPS, bitrate=1800))
    else:
        path = path.with_suffix(".gif")
        anim.save(path, writer=animation.PillowWriter(fps=FPS))
    plt.close(fig)
    return path


def main() -> None:
    scenario = sys.argv[1] if len(sys.argv) > 1 else "gyro_bias_fault"
    controller = ControllerType(sys.argv[2]) if len(sys.argv) > 2 else ControllerType.NIS_GATE
    df = simulate(scenario, controller)
    out = animate(df, f"Echo Balancer: {scenario}, {controller.value} (background = gate mode)",
                  FIGS / f"animation_{scenario}_{controller.value}.mp4")
    print(f"wrote {out}")


if __name__ == "__main__":
    main()
