"""QUBO formulation for gate-threshold selection (CLAUDE.md section 13)."""

import importlib.util

import numpy as np
import pytest

from quantum.cost_table import decode
from quantum.formulation import (
    bits_to_index, evaluate, quadratize, rank_transform, table_to_hubo, verify_exact,
)
from quantum.qubo import build_qubo, penalised_ranks
from sim.params import default_gate_params, default_qubo_params


@pytest.mark.parametrize("n", [2, 3, 4, 5])
def test_hubo_reproduces_table_exactly(n):
    J = np.random.default_rng(n).normal(size=2**n)
    h = table_to_hubo(J)
    for c in range(2**n):
        x = np.array([(c >> i) & 1 for i in range(n)])
        assert evaluate(h, x) == pytest.approx(J[c])


@pytest.mark.parametrize("n", [3, 4, 5, 6])
@pytest.mark.parametrize("seed", range(4))
def test_quadratized_qubo_minimum_is_table_argmin(n, seed):
    J = np.random.default_rng(100 * n + seed).random(2**n)
    q, n_vars, _ = quadratize(table_to_hubo(rank_transform(J)), n)
    assert max(len(m) for m in q) <= 2
    idx, _ = verify_exact(q, n_vars, J, n)
    assert idx == int(np.argmin(J))


def test_rank_transform_keeps_argmin_and_ties():
    J = np.array([3.0, 1.0, 1.0, 7.0])
    r = rank_transform(J)
    assert list(r) == [1, 0, 0, 2]


def test_infeasible_codes_never_win():
    J = np.array([np.nan, 5.0, np.nan, 2.0, 9.0, np.nan, 4.0, 3.0])
    r = penalised_ranks(J)
    assert r[np.isnan(J)].min() > r[~np.isnan(J)].max()
    built = build_qubo(J)
    assert built["brute_force_qubo_index"] == 3


def test_default_gate_is_a_candidate():
    qp, gd = default_qubo_params(), default_gate_params()
    hits = [decode(i, qp) for i in range(64)]
    hits = [c for c in hits if (c["tau1"], c["tau2"], c["N"], c["T_dwell"]) == (gd.tau1, gd.tau2, gd.N, gd.T_dwell)]
    assert len(hits) == 1 and hits[0]["feasible"]
    assert sum(decode(i, qp)["feasible"] for i in range(64)) == 52


def test_bits_to_index_little_endian():
    assert bits_to_index(np.array([1, 0, 1, 1, 0]), 4) == 0b1101


@pytest.mark.skipif(importlib.util.find_spec("qiskit_optimization") is None, reason="qiskit not installed")
def test_grover_optimizer_finds_optimum_on_tiny_instance():
    from quantum.qubo import run_grover

    J = np.array([4.0, 2.0, np.nan, 7.0, 1.0, 6.0, 3.0, np.nan])
    built = build_qubo(J)
    r = run_grover(built, seed=0, iterations=6)
    assert r["index"] == 4
    assert r["oracle_calls"] >= 0
