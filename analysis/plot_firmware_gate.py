"""NIS gate vs tilt-threshold gate on the firmware core, same seeded faults.
Usage: uv run python analysis/plot_firmware_gate.py  -> docs/firmware_gate_comparison.png
Compiles firmware/test/host_scenarios.cpp with g++ and runs it."""

import subprocess
import tempfile
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from sim.params import default_disturbance_params, default_gate_params

ROOT = Path(__file__).resolve().parents[1]
SCENARIOS = [("gyro_bias", "Gyro bias step"), ("accel_noise", "Accel noise x5"),
             ("push", "Push (psi_dot kick)"), ("payload", "Payload shift")]
SEED, SECS = 1, 15.0
MODE_COLORS = ["#2e7d32", "#f9a825", "#c62828"]


def run(exe: Path, name: str, onset: float) -> np.ndarray:
    out = subprocess.run([str(exe), name, str(SEED), str(SECS), str(onset)], capture_output=True, text=True, check=True).stdout
    return np.array([[float(v) for v in line.split()] for line in out.splitlines()])


def main() -> None:
    onset = default_disturbance_params().gyro_bias_fault_onset
    gp = default_gate_params()
    with tempfile.TemporaryDirectory() as tmp:
        exe = Path(tmp) / "hs"
        subprocess.run(["g++", "-std=c++17", "-O2", "-o", str(exe), str(ROOT / "firmware/test/host_scenarios.cpp")], check=True)
        fig, axes = plt.subplots(len(SCENARIOS), 1, figsize=(9, 9), sharex=True)
        for ax, (name, title) in zip(axes, SCENARIOS):
            d = run(exe, name, onset)
            t = d[:, 0]
            ax.plot(t, d[:, 2], color="#1f3a5f", lw=1, label="epsilon (windowed NIS)")
            ax.axhline(gp.tau1, color="#f9a825", ls="--", lw=1)
            ax.axhline(gp.tau2, color="#c62828", ls="--", lw=1)
            ax.set_yscale("log"); ax.set_ylim(bottom=60)
            ax.axvline(onset, color="k", ls=":", lw=1)
            for row, (col, label) in enumerate([(3, "NIS gate"), (4, "tilt gate")]):
                for i in range(len(t) - 1):
                    ax.axvspan(t[i], t[i + 1], ymin=0.02 + 0.05 * row, ymax=0.06 + 0.05 * row,
                               color=MODE_COLORS[int(d[i, col])], lw=0)
                ax.text(t[-1] + 0.1, 0.02 + 0.05 * row, label, transform=ax.get_xaxis_transform(), fontsize=7, va="bottom")
            ax.set_ylabel("epsilon")
            ax.set_title(title, loc="left", fontsize=9)
        axes[-1].set_xlabel("time (s); dotted = fault onset; dashed = tau1 / tau2; strips: green NORMAL, amber CAUTIOUS, red HALT")
        fig.tight_layout()
        out = ROOT / "docs" / "firmware_gate_comparison.png"
        fig.savefig(out, dpi=140)
        print(f"wrote {out}")


if __name__ == "__main__":
    main()
