from sim.gate import GateMode, GateState, initial_gate_state, step_gate
from sim.params import GateParams


def _toy_gate_params() -> GateParams:
    """Small N so test sequences are easy to hand-verify; not the real
    N=200 config (that's exercised by the closed-loop tests later in this
    file)."""
    return GateParams(N=3, tau1=10.0, tau2=20.0, tau1_exit=4.0, tau2_exit=10.0, T_dwell=2.0)


def test_initial_gate_state():
    state = initial_gate_state()

    assert state.mode is GateMode.NORMAL
    assert state.nis_window == ()
    assert state.time_in_mode == 0.0


def test_no_transition_while_window_not_full():
    gp = _toy_gate_params()
    state = initial_gate_state()

    for nis in [100.0, 100.0]:  # only 2 samples; N=3, window never fills
        state, _epsilon = step_gate(state, nis, dt=1.0, gate_p=gp)
        assert state.mode is GateMode.NORMAL


def test_normal_to_cautious_and_back_with_dwell():
    gp = _toy_gate_params()
    state = initial_gate_state()

    # (nis, expected_mode, expected_epsilon) per step, computed by hand-running
    # the exact state machine below -- see plan Task 2 for the derivation.
    expected = [
        (1.0, GateMode.NORMAL, 1.0),
        (1.0, GateMode.NORMAL, 2.0),
        (1.0, GateMode.NORMAL, 3.0),
        (15.0, GateMode.CAUTIOUS, 17.0),  # window=(1,1,15) full, eps=17>tau1=10
        (1.0, GateMode.CAUTIOUS, 17.0),   # window=(1,15,1), dwell blocks exit anyway (eps=17 > tau1_exit=4)
        (1.0, GateMode.CAUTIOUS, 17.0),   # window=(15,1,1)
        (1.0, GateMode.NORMAL, 3.0),      # window=(1,1,1), eps=3<tau1_exit=4, dwell (2.0s) satisfied
        (1.0, GateMode.NORMAL, 3.0),
    ]
    for nis, expected_mode, expected_eps in expected:
        state, eps = step_gate(state, nis, dt=1.0, gate_p=gp)
        assert state.mode is expected_mode
        assert eps == expected_eps


def test_normal_to_halt_skip_on_full_window():
    gp = _toy_gate_params()
    state = initial_gate_state()

    for nis in [1.0, 1.0, 1.0]:
        state, _eps = step_gate(state, nis, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.NORMAL

    state, eps = step_gate(state, 25.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.HALT
    assert eps == 27.0


def test_halt_to_cautious_exit():
    gp = _toy_gate_params()
    state = GateState(mode=GateMode.HALT, nis_window=(100.0, 100.0, 100.0), time_in_mode=5.0)

    state, eps = step_gate(state, 1.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.HALT
    assert eps == 201.0

    state, eps = step_gate(state, 1.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.HALT
    assert eps == 102.0

    state, eps = step_gate(state, 1.0, dt=1.0, gate_p=gp)
    assert state.mode is GateMode.CAUTIOUS
    assert eps == 3.0
