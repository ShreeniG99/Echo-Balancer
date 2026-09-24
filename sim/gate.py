"""Normal/Cautious/Halt gate state machine (CLAUDE.md section 10).

Watches the windowed NIS statistic epsilon_k = sum of the last N NIS
values (CLAUDE.md section 9) and switches between three modes on
thresholds tau1 < tau2, with hysteresis (tau1_exit < tau1, tau2_exit <
tau2) and a minimum dwell time before any transition. See GateParams'
docstring in sim/params.py for why tau1/tau2 are empirically calibrated
rather than literal chi2(3N) quantiles.
"""

from dataclasses import dataclass
from enum import Enum

import numpy as np

from sim.params import GateParams, TiltGateParams


class GateMode(Enum):
    NORMAL = "NORMAL"
    CAUTIOUS = "CAUTIOUS"
    HALT = "HALT"


@dataclass(frozen=True)
class GateState:
    mode: GateMode
    nis_window: tuple[float, ...]
    time_in_mode: float


def initial_gate_state() -> GateState:
    return GateState(mode=GateMode.NORMAL, nis_window=(), time_in_mode=0.0)


def step_gate(state: GateState, nis_k: float, dt: float, gate_p: GateParams) -> tuple[GateState, float]:
    """Advance the gate by one control-loop sample. Returns (new_state, epsilon_k).

    epsilon_k is returned even when the window isn't yet full (sum of
    whatever has accumulated so far), but mode transitions only ever
    happen once the window is full -- CLAUDE.md's "No NORMAL -> HALT skip
    unless epsilon_k > tau2 for a full window" is read here as: a direct
    NORMAL -> HALT transition is allowed exactly when the window is fully
    populated and epsilon_k > tau2 (epsilon_k, being an N-sample sum, IS
    "a full window" of evidence by construction; the exception is about
    not acting on an under-filled, still-warming-up window).
    """
    window = state.nis_window + (nis_k,)
    if len(window) > gate_p.N:
        window = window[-gate_p.N:]
    window_full = len(window) == gate_p.N
    epsilon = sum(window)

    time_in_mode = state.time_in_mode + dt
    can_transition = time_in_mode >= gate_p.T_dwell

    new_mode = state.mode
    if can_transition and window_full:
        if state.mode is GateMode.NORMAL:
            if epsilon > gate_p.tau2:
                new_mode = GateMode.HALT
            elif epsilon > gate_p.tau1:
                new_mode = GateMode.CAUTIOUS
        elif state.mode is GateMode.CAUTIOUS:
            if epsilon > gate_p.tau2:
                new_mode = GateMode.HALT
            elif epsilon < gate_p.tau1_exit:
                new_mode = GateMode.NORMAL
        elif state.mode is GateMode.HALT:
            if epsilon < gate_p.tau2_exit:
                new_mode = GateMode.CAUTIOUS

    if new_mode != state.mode:
        time_in_mode = 0.0

    return GateState(mode=new_mode, nis_window=window, time_in_mode=time_in_mode), epsilon


@dataclass(frozen=True)
class TiltGateState:
    mode: GateMode
    time_in_mode: float


def initial_tilt_gate_state() -> TiltGateState:
    return TiltGateState(mode=GateMode.NORMAL, time_in_mode=0.0)


def step_tilt_gate(state: TiltGateState, psi: float, dt: float, tilt_p: TiltGateParams) -> TiltGateState:
    """Tilt-threshold gate baseline (CLAUDE.md section 12): mode switches
    on |psi| thresholds only, no windowing -- same hysteresis + minimum-
    dwell structure as step_gate, but acting directly on the instantaneous
    tilt magnitude rather than a windowed NIS statistic.
    """
    abs_psi_deg = abs(np.degrees(psi))
    time_in_mode = state.time_in_mode + dt
    can_transition = time_in_mode >= tilt_p.T_dwell

    new_mode = state.mode
    if can_transition:
        if state.mode is GateMode.NORMAL:
            if abs_psi_deg > tilt_p.tau2_deg:
                new_mode = GateMode.HALT
            elif abs_psi_deg > tilt_p.tau1_deg:
                new_mode = GateMode.CAUTIOUS
        elif state.mode is GateMode.CAUTIOUS:
            if abs_psi_deg > tilt_p.tau2_deg:
                new_mode = GateMode.HALT
            elif abs_psi_deg < tilt_p.tau1_exit_deg:
                new_mode = GateMode.NORMAL
        elif state.mode is GateMode.HALT:
            if abs_psi_deg < tilt_p.tau2_exit_deg:
                new_mode = GateMode.CAUTIOUS

    if new_mode != state.mode:
        time_in_mode = 0.0

    return TiltGateState(mode=new_mode, time_in_mode=time_in_mode)
