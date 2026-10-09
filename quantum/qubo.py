"""Gate-threshold selection: exhaustive search vs Grover Adaptive Search on the
same QUBO (CLAUDE.md section 13, steps 3-5).

Once the cost table exists the classical argmin is trivial; the point is the
QUBO formulation of safe-switching-policy selection, NOT speed. Nothing here
claims a speedup.

Encoding: candidate index in binary (quantum/cost_table.py: bits 0-1 tau1
level, 2-3 tau2 level, 4 N, 5 T_dwell). The cost table is mapped through a
rank transform (monotone: argmin unchanged; integer coefficients keep
Grover's value register small); infeasible codes (tau1 >= tau2) get rank =
1 + worst feasible rank, i.e. a penalty. The resulting exact multilinear
polynomial (a HUBO) is reduced to a QUBO with Rosenberg substitution, and
every QUBO is brute-force verified to decode to the true argmin.

Qubit budget: a *random* 6-bit table needs 18 QUBO variables + ~16 value
qubits (~34, not simulable here), but the real cost tables are structured
(J depends mostly on the tau1 level and N), so their HUBOs have low degree
and the full 6-bit QUBO needs only 9 variables + 10 value qubits = 19 qubits.
Grover therefore runs on the full 64-candidate problem and, for comparison,
on the 5-bit (tau1 x tau2 x N) and 4-bit (tau1 x tau2) sub-grids at the
default gate's T_dwell / N. Optimal sets are often large (ties), which makes
the search easy -- the report lists each instance's optimal-set size.

Usage: PYTHONPATH=. uv run python -m quantum.qubo   (needs the cost table)
"""

import json
import time
import warnings
from pathlib import Path

import numpy as np
import pandas as pd

from quantum.cost_table import N_BITS, TABLE_PATH
from quantum.formulation import (
    bits_to_index, quadratize, rank_transform, table_to_hubo, value_range, verify_exact,
)
from sim.params import default_gate_params, default_qubo_params

OUT_DIR = Path(__file__).resolve().parents[1] / "docs" / "results"


def penalised_ranks(J: np.ndarray) -> np.ndarray:
    """Rank feasible costs; infeasible (NaN) codes get a rank above every feasible one."""
    feas = ~np.isnan(J)
    r = np.empty_like(J)
    r[feas] = rank_transform(J[feas])
    r[~feas] = r[feas].max() + 1.0
    return r


def build_qubo(J: np.ndarray) -> dict:
    n = int(round(np.log2(J.size)))
    ranks = penalised_ranks(J)
    qubo, n_vars, subs = quadratize(table_to_hubo(ranks), n)
    t = time.perf_counter()
    idx, _ = verify_exact(qubo, n_vars, ranks, n)
    return {
        "n_index_bits": n, "qubo": qubo, "n_vars": n_vars, "n_aux": len(subs),
        "value_qubits": int(np.ceil(np.log2(value_range(qubo) + 1))) + 1,
        "brute_force_qubo_index": idx, "brute_force_qubo_s": time.perf_counter() - t,
    }


def to_quadratic_program(qubo: dict, n_vars: int):
    from qiskit_optimization import QuadraticProgram

    qp = QuadraticProgram("gate_threshold_selection")
    for i in range(n_vars):
        qp.binary_var(f"x{i}")
    qp.minimize(
        constant=qubo.get((), 0.0),
        linear={f"x{m[0]}": c for m, c in qubo.items() if len(m) == 1},
        quadratic={(f"x{m[0]}", f"x{m[1]}"): c for m, c in qubo.items() if len(m) == 2},
    )
    return qp


def run_grover(built: dict, seed: int, iterations: int) -> dict:
    from qiskit.transpiler.preset_passmanagers import generate_preset_pass_manager
    from qiskit_aer import AerSimulator
    from qiskit_aer.primitives import SamplerV2
    from qiskit_optimization.algorithms import GroverOptimizer
    from qiskit_optimization.algorithms import grover_optimizer as go_module

    powers: list[int] = []

    class CountingGrover(go_module.Grover):  # records each circuit's Grover power = oracle applications
        def construct_circuit(self, problem, power=None, measurement=False):
            powers.append(int(power or 0))
            return super().construct_circuit(problem=problem, power=power, measurement=measurement)

    qp = to_quadratic_program(built["qubo"], built["n_vars"])
    pm = generate_preset_pass_manager(optimization_level=1, backend=AerSimulator(seed_simulator=seed))
    opt = GroverOptimizer(num_value_qubits=built["value_qubits"], num_iterations=iterations,
                          sampler=SamplerV2(seed=seed), pass_manager=pm)
    t = time.perf_counter()
    original = go_module.Grover
    go_module.Grover = CountingGrover
    try:
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            res = opt.solve(qp)
    finally:
        go_module.Grover = original
    wall = time.perf_counter() - t
    return {
        "seed": seed, "index": bits_to_index(res.x, built["n_index_bits"]), "wall_s": wall,
        "circuits_sampled": len(powers),   # one measured circuit per adaptive-search step
        "oracle_calls": int(sum(powers)),  # Grover-operator (oracle) applications, summed
    }


def sub_table(table: pd.DataFrame, fixed_bits: dict[int, int], col: str = "J") -> tuple[np.ndarray, np.ndarray]:
    """Slice the full table at fixed high bits -> (cost sub-table, full indices)."""
    keep = [i for i in range(2**N_BITS) if all(((i >> b) & 1) == v for b, v in fixed_bits.items())]
    sub = table.set_index("index").loc[keep]
    return sub[col].to_numpy(dtype=float), np.array(keep)


def with_detection_variant(table: pd.DataFrame) -> pd.DataFrame:
    """Adds J_detect = J + w_detect_variant * (1 - detection_rate). NOT the spec J (see QuboParams)."""
    t = table.copy()
    t["J_detect"] = t["J"] + default_qubo_params().w_detect_variant * (1.0 - t["detection_rate"])
    return t


def solve_instance(name: str, table: pd.DataFrame, fixed_bits: dict[int, int], run_q: bool, col: str = "J") -> dict:
    qp = default_qubo_params()
    J, full_idx = sub_table(table, fixed_bits, col)
    t = time.perf_counter()
    feas = ~np.isnan(J)
    classical_local = int(np.nanargmin(J))
    classical_s = time.perf_counter() - t
    built = build_qubo(J)
    out = {
        "instance": name, "cost": col, "candidates": int(J.size), "feasible": int(feas.sum()),
        "distinct_costs": int(np.unique(J[feas]).size),
        "optimal_set": [int(i) for i in full_idx[np.isclose(J, np.nanmin(J))]],
        "classical_optimum_index": int(full_idx[classical_local]), "classical_J": float(J[classical_local]),
        "classical_evaluations": int(feas.sum()), "classical_s": classical_s,
        "qubo_vars": built["n_vars"], "qubo_aux_vars": built["n_aux"], "value_qubits": built["value_qubits"],
        "total_qubits": built["n_vars"] + built["value_qubits"],
        "qubo_brute_force_index": int(full_idx[built["brute_force_qubo_index"]]),
        "qubo_exact": bool(np.isclose(J[built["brute_force_qubo_index"]], J[classical_local])),
        "grover_runs": [],
    }
    if run_q:
        for seed in qp.grover_seeds:
            r = run_grover(built, seed, qp.grover_iterations)
            r["index"] = int(full_idx[r["index"]])
            r["J"] = float(table.set_index("index").loc[r["index"], col])
            r["same_optimum"] = bool(np.isclose(r["J"], out["classical_J"]))
            out["grover_runs"].append(r)
            print(f"  {name} grover seed {seed}: index {r['index']} same_optimum={r['same_optimum']} "
                  f"oracle_calls={r['oracle_calls']} {r['wall_s']:.0f}s", flush=True)
    return out


def main() -> None:
    table = with_detection_variant(pd.read_parquet(TABLE_PATH))
    qp, gd = default_qubo_params(), default_gate_params()
    nbit = qp.N_values.index(gd.N)
    tbit = qp.T_dwell_values.index(gd.T_dwell)
    instances = [  # (name, fixed index bits, run Grover?, cost column)
        ("full_6bit", {}, True, "J"),
        ("tau1_tau2_N_5bit", {5: tbit}, True, "J"),
        ("tau1_tau2_4bit", {4: nbit, 5: tbit}, True, "J"),
        ("full_6bit_detect", {}, True, "J_detect"),
        ("tau1_tau2_N_5bit_detect", {5: tbit}, True, "J_detect"),
    ]
    report = []
    for name, fixed, run_q, col in instances:
        print(f"instance {name}", flush=True)
        report.append(solve_instance(name, table, fixed, run_q, col))
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    (OUT_DIR / "qubo_report.json").write_text(json.dumps(report, indent=2))
    rows = []
    for r in report:
        g = r["grover_runs"]
        rows.append({
            "instance": r["instance"], "cost": r["cost"], "candidates": r["candidates"], "feasible": r["feasible"],
            "distinct_costs": r["distinct_costs"], "optimal_set_size": len(r["optimal_set"]),
            "classical_optimum": r["classical_optimum_index"], "QUBO_exact": r["qubo_exact"],
            "qubo_vars": r["qubo_vars"], "total_qubits": r["total_qubits"],
            "grover_same_optimum": f"{sum(x['same_optimum'] for x in g)}/{len(g)}" if g else "not run (qubits)",
            "grover_mean_oracle_calls": np.mean([x["oracle_calls"] for x in g]) if g else np.nan,
            "grover_mean_wall_s": np.mean([x["wall_s"] for x in g]) if g else np.nan,
            "classical_evaluations": r["classical_evaluations"],
        })
    summary = pd.DataFrame(rows)
    summary.to_csv(OUT_DIR / "qubo_summary.csv", index=False, float_format="%.4g")
    print(summary.to_string())


if __name__ == "__main__":
    main()
