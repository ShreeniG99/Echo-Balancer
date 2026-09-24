"""Tests for quantum/qubo.py (CLAUDE.md section 13, Milestone 3)."""

from itertools import combinations, product

import numpy as np

from quantum.qubo import (
    N_BITS,
    _build_quadratic_program,
    _fit_quadratic_surrogate,
    _surrogate_value,
    all_candidates,
    run_qubo_pipeline,
)
from sim.params import QuboSearchParams, default_qubo_search_params


def test_all_candidates_covers_the_full_grid_with_unique_bit_strings():
    qp = default_qubo_search_params()
    candidates = all_candidates(qp)
    assert len(candidates) == 2 ** N_BITS
    assert len({c.bits for c in candidates}) == len(candidates)

    expected = len(qp.tau1_grid) * len(qp.tau2_grid) * len(qp.N_grid) * len(qp.T_dwell_grid)
    assert len(candidates) == expected


def test_feasibility_matches_tau1_less_than_tau2():
    qp = default_qubo_search_params()
    candidates = all_candidates(qp)
    for c in candidates:
        assert c.feasible == (c.tau1 < c.tau2)
    # the default grids are chosen so both cases actually occur -- a
    # penalty term with nothing to penalize would be a vacuous check
    assert any(c.feasible for c in candidates)
    assert any(not c.feasible for c in candidates)


def test_candidate_to_gate_params_preserves_hysteresis_ratio():
    from sim.params import default_evaluation_gate_params

    qp = default_qubo_search_params()
    base = default_evaluation_gate_params()
    candidate = all_candidates(qp)[0]
    gate_p = candidate.to_gate_params(base)

    assert gate_p.tau1 == candidate.tau1
    assert gate_p.tau2 == candidate.tau2
    assert gate_p.N == candidate.N
    assert gate_p.T_dwell == candidate.T_dwell
    assert abs(gate_p.tau1_exit / gate_p.tau1 - base.tau1_exit / base.tau1) < 1e-12
    assert abs(gate_p.tau2_exit / gate_p.tau2 - base.tau2_exit / base.tau2) < 1e-12


def test_quadratic_surrogate_recovers_exact_synthetic_quadratic():
    """Sanity check for the regression machinery itself, independent of
    any simulated data: fed an EXACTLY quadratic target, the least-squares
    fit must recover it (near-)exactly (R^2 ~= 1, ~0 pointwise error)."""
    rng = np.random.default_rng(0)
    true_const = 3.0
    true_linear = {i: float(rng.uniform(-2, 2)) for i in range(N_BITS)}
    true_quad = {(i, j): float(rng.uniform(-1, 1)) for i, j in combinations(range(N_BITS), 2)}

    def true_val(bits):
        v = true_const
        v += sum(true_linear[i] * bits[i] for i in range(N_BITS))
        v += sum(c * bits[i] * bits[j] for (i, j), c in true_quad.items())
        return v

    table = {bits: (None, true_val(bits)) for bits in product((0, 1), repeat=N_BITS)}
    const, linear, quadratic, r_squared = _fit_quadratic_surrogate(table)

    assert r_squared > 0.999999
    for bits in list(product((0, 1), repeat=N_BITS))[:20]:
        assert abs(_surrogate_value(bits, const, linear, quadratic) - true_val(bits)) < 1e-8


def test_build_quadratic_program_has_n_bits_binary_variables():
    qprog = _build_quadratic_program(1.0, {f"x{i}": 0.0 for i in range(N_BITS)}, {})
    assert qprog.get_num_binary_vars() == N_BITS
    assert qprog.get_num_vars() == N_BITS


def test_run_qubo_pipeline_end_to_end_tiny_config():
    """Deliberately tiny (1 seed, ~2s episodes, few Grover trials) purely
    to exercise the full wiring (re-simulation -> surrogate fit -> QUBO ->
    GroverOptimizer) fast -- not meant to produce a meaningful surrogate
    fit or reliable Grover convergence, see the other tests for those."""
    tiny_qp = QuboSearchParams(seeds=(0,), nominal_T=2.0, disturbance_T=2.0)
    result = run_qubo_pipeline(qp=tiny_qp, num_value_qubits=8, num_iterations=6, grover_trials=2, n_jobs=2)

    assert len(result.true_table) == 2 ** N_BITS
    assert len(result.grover_best_bits) == N_BITS
    assert result.n_input_qubits == N_BITS
    assert result.grover_trials == 2
    assert 0 <= result.grover_trial_hits <= 2
    assert result.oracle_calls > 0
    assert isinstance(result.grover_matches_surrogate_optimum, bool)
    assert isinstance(result.surrogate_matches_true_optimum, bool)
