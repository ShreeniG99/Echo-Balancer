"""Offline gate-threshold selection: classical exhaustive search vs.
Grover Adaptive Search on a QUBO (CLAUDE.md section 13, Milestone 3).

The candidate space is tau1/tau2/N/T_dwell on the grids in
sim.params.QuboSearchParams (4x4x2x2 = 64 candidates, encoded as 6 binary
variables -- see _decode/_encode). Because the gate changes the robot's
own behaviour, log replay is invalid; each candidate is re-simulated
(NIS_GATE only, tau1/tau2/N/T_dwell being meaningless to the other two
controllers) across a small scenario/seed set, and its cost
J = w_fall*fall_rate + w_false_fallback*false_fallback_frac +
w_progress*(1 - mean_progress_m) is computed from that re-simulation. A
tau1 >= tau2 candidate is never simulated at all -- it's assigned
QuboSearchParams.infeasible_penalty directly, which IS this step's
"penalty term for infeasible codes" (CLAUDE.md section 13 step 3): it
enters the true-cost table the same way every other candidate's cost
does, rather than as a separate analytic constraint term.

GroverOptimizer (qiskit_optimization) only accepts a linear+quadratic
(degree <= 2) objective over its decision variables. The true 64-entry
cost table has no reason to decompose that way -- it's the output of a
real closed-loop simulation, not a hand-built quadratic function -- so it
cannot be handed to GroverOptimizer directly. This module instead fits a
least-squares quadratic surrogate to the true table (constant + linear +
pairwise terms over the 6 bits, via _fit_quadratic_surrogate) and builds
the QUBO from that surrogate. This is a real, load-bearing approximation,
not a formality: report_pipeline_results reports both (a) whether Grover
finds the surrogate's true minimizer (a check on Grover itself) and (b)
whether the surrogate's minimizer matches the TRUE table's minimizer (a
check on whether the quadratic approximation preserved the right answer)
-- CLAUDE.md's "report it honestly whichever way it lands" applies to
both.

CLAUDE.md section 13 step 5: once the cost table is computed, the
classical argmin is trivial (a 64-entry min() call); the contribution
here is the QUBO formulation of safe-switching-policy selection, not
speed. No speedup claim is made.
"""

import time
from dataclasses import dataclass, replace
from itertools import combinations, product
from typing import Optional

import numpy as np
from joblib import Parallel, delayed
from qiskit_aer.primitives import Sampler
from qiskit_optimization import QuadraticProgram
from qiskit_optimization.algorithms import GroverOptimizer

from analysis.metrics import build_metrics_table
from experiments.run_batch import default_scenarios
from sim.params import GateParams, QuboSearchParams, default_qubo_search_params
from sim.run import ControllerKind, EpisodeConfig, run_episode

N_BITS = 6  # 2 (tau1) + 2 (tau2) + 1 (N) + 1 (T_dwell) -- see module docstring


@dataclass(frozen=True)
class Candidate:
    bits: tuple[int, ...]  # length N_BITS, little-endian per field
    tau1: float
    tau2: float
    N: int
    T_dwell: float

    @property
    def feasible(self) -> bool:
        return self.tau1 < self.tau2

    def to_gate_params(self, base: GateParams) -> GateParams:
        """tau1_exit/tau2_exit keep base's hysteresis ratios (0.8, ~0.714)
        rather than being independently searched -- CLAUDE.md section 13
        names exactly tau1/tau2/N/T_dwell as the parameters to select."""
        ratio1 = base.tau1_exit / base.tau1
        ratio2 = base.tau2_exit / base.tau2
        return GateParams(
            N=self.N, tau1=self.tau1, tau2=self.tau2,
            tau1_exit=ratio1 * self.tau1, tau2_exit=ratio2 * self.tau2,
            T_dwell=self.T_dwell,
        )


def all_candidates(qp: QuboSearchParams) -> list[Candidate]:
    """All 2**N_BITS candidates, in bit-index order (bits[i] is the i-th
    binary variable x_i of the eventual QUBO)."""
    candidates = []
    for bits in product((0, 1), repeat=N_BITS):
        tau1_idx = bits[0] + 2 * bits[1]
        tau2_idx = bits[2] + 2 * bits[3]
        N_idx = bits[4]
        T_dwell_idx = bits[5]
        candidates.append(Candidate(
            bits=bits,
            tau1=qp.tau1_grid[tau1_idx], tau2=qp.tau2_grid[tau2_idx],
            N=qp.N_grid[N_idx], T_dwell=qp.T_dwell_grid[T_dwell_idx],
        ))
    return candidates


def _evaluate_candidate(candidate: Candidate, qp: QuboSearchParams, base_gate_p: GateParams) -> float:
    """Re-simulates this candidate's GateParams across qp's scenario/seed
    set (NIS_GATE only) and returns its cost J. Never called for an
    infeasible candidate -- see cost_table."""
    gate_p = candidate.to_gate_params(base_gate_p)
    scenarios = default_scenarios(nominal_T=qp.nominal_T, disturbance_T=qp.disturbance_T)

    episode_rows, step_dfs = [], []
    for scenario in scenarios:
        for seed in qp.seeds:
            cfg = EpisodeConfig(
                controller=ControllerKind.NIS_GATE, T=scenario.T, disturbance=scenario.disturbance,
                x0_psi_deg=scenario.x0_psi_deg, gate_p=gate_p,
            )
            df = run_episode(cfg, seed=seed)
            df = df.copy()
            df["controller"] = ControllerKind.NIS_GATE.value
            df["scenario"] = scenario.name
            df["seed"] = seed
            step_dfs.append(df)
            episode_rows.append(dict(
                controller=ControllerKind.NIS_GATE.value, scenario=scenario.name, seed=seed,
                onset=scenario.onset if scenario.onset is not None else float("nan"),
                fell=df.attrs["fell"], fall_time=df.attrs["fall_time"] or float("nan"),
                progress=df.attrs["progress"],
            ))

    import pandas as pd

    episodes_df = pd.DataFrame(episode_rows)
    steps_df = pd.concat(step_dfs, ignore_index=True)
    table = build_metrics_table(episodes_df, steps_df).loc["nis_gate"]

    return (
        qp.w_fall * table["fall_rate"]
        + qp.w_false_fallback * table["false_fallback_frac"]
        + qp.w_progress * (1.0 - table["mean_progress_m"])
    )


def cost_table(
    qp: Optional[QuboSearchParams] = None, base_gate_p: Optional[GateParams] = None, n_jobs: int = -1,
) -> dict[tuple[int, ...], tuple[Candidate, float]]:
    """The true (re-simulated) cost of every candidate, keyed by its bit
    tuple. Infeasible candidates get qp.infeasible_penalty without ever
    being simulated -- this dict IS the classical exhaustive/grid search's
    result table (its argmin is the classical baseline, CLAUDE.md section
    13 step 4)."""
    from sim.params import default_evaluation_gate_params

    qp = default_qubo_search_params() if qp is None else qp
    base_gate_p = default_evaluation_gate_params() if base_gate_p is None else base_gate_p
    candidates = all_candidates(qp)

    feasible = [c for c in candidates if c.feasible]
    costs = Parallel(n_jobs=n_jobs)(delayed(_evaluate_candidate)(c, qp, base_gate_p) for c in feasible)

    table = {c.bits: (c, cost) for c, cost in zip(feasible, costs)}
    for c in candidates:
        if not c.feasible:
            table[c.bits] = (c, qp.infeasible_penalty)
    return table


def _fit_quadratic_surrogate(table: dict[tuple[int, ...], tuple["Candidate", float]]) -> tuple[float, dict, dict, float]:
    """Least-squares quadratic (constant + linear + pairwise) fit to the
    true cost table over the N_BITS binary variables. Returns
    (constant, linear_dict, quadratic_dict, r_squared).
    """
    pairs = list(combinations(range(N_BITS), 2))
    bits_list = list(table.keys())
    y = np.array([table[b][1] for b in bits_list])

    X = np.ones((len(bits_list), 1 + N_BITS + len(pairs)))
    for row, bits in enumerate(bits_list):
        X[row, 1:1 + N_BITS] = bits
        for k, (i, j) in enumerate(pairs):
            X[row, 1 + N_BITS + k] = bits[i] * bits[j]

    coeffs, _, _, _ = np.linalg.lstsq(X, y, rcond=None)
    y_hat = X @ coeffs
    ss_res = float(np.sum((y - y_hat) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r_squared = 1.0 - ss_res / ss_tot if ss_tot > 0 else 1.0

    constant = float(coeffs[0])
    linear = {f"x{i}": float(coeffs[1 + i]) for i in range(N_BITS)}
    quadratic = {(f"x{i}", f"x{j}"): float(coeffs[1 + N_BITS + k]) for k, (i, j) in enumerate(pairs)}
    return constant, linear, quadratic, r_squared


def _build_quadratic_program(constant: float, linear: dict, quadratic: dict) -> QuadraticProgram:
    qp = QuadraticProgram()
    for i in range(N_BITS):
        qp.binary_var(f"x{i}")
    qp.minimize(constant=constant, linear=linear, quadratic=quadratic)
    return qp


def _surrogate_value(bits: tuple[int, ...], constant: float, linear: dict, quadratic: dict) -> float:
    val = constant
    for i in range(N_BITS):
        val += linear[f"x{i}"] * bits[i]
    for (xi, xj), coeff in quadratic.items():
        i, j = int(xi[1:]), int(xj[1:])
        val += coeff * bits[i] * bits[j]
    return val


@dataclass(frozen=True)
class QuboRunResult:
    true_table: dict
    surrogate_constant: float
    surrogate_linear: dict
    surrogate_quadratic: dict
    surrogate_r_squared: float
    true_best_bits: tuple[int, ...]
    surrogate_exhaustive_best_bits: tuple[int, ...]
    grover_best_bits: tuple[int, ...]
    grover_fval: float
    grover_trials: int
    grover_trial_hits: int
    classical_wall_time_s: float
    grover_wall_time_s: float
    n_input_qubits: int
    n_output_qubits: int
    oracle_calls: int
    grover_matches_surrogate_optimum: bool
    surrogate_matches_true_optimum: bool
    surrogate_optimum_gap: float
    surrogate_value_range: float


def run_qubo_pipeline(
    qp: Optional[QuboSearchParams] = None,
    num_value_qubits: int = 10,
    num_iterations: int = 8,
    grover_trials: int = 5,
    n_jobs: int = -1,
) -> QuboRunResult:
    """grover_trials: GroverOptimizer (like Grover's algorithm generally)
    is a probabilistic quantum search -- a single run has a real,
    non-negligible chance of returning a non-optimal sample even with
    enough iterations/value qubits to represent the true optimum (verified
    empirically during this module's development: a single 6-input-qubit
    run found the true minimizer roughly 1 time in 5 at num_value_qubits=10,
    num_iterations=8). Running it grover_trials times independently and
    keeping the best fval found is the standard way to use it, not a way
    of avoiding the algorithm's real behavior -- grover_trial_hits below
    reports exactly how many of those independent trials found the true
    surrogate optimum, so this doesn't paper over that probabilistic
    behavior.
    """
    qp = default_qubo_search_params() if qp is None else qp

    table = cost_table(qp, n_jobs=n_jobs)

    t0 = time.time()
    true_best_bits = min(table, key=lambda b: table[b][1])
    classical_wall_time_s = time.time() - t0

    constant, linear, quadratic, r_squared = _fit_quadratic_surrogate(table)

    surrogate_values = sorted(_surrogate_value(b, constant, linear, quadratic) for b in table.keys())
    surrogate_exhaustive_best_bits = min(
        table.keys(), key=lambda b: _surrogate_value(b, constant, linear, quadratic)
    )
    # How hard this surrogate actually is for a value-qubit-based search: the
    # margin between the best and second-best candidate, relative to the
    # objective's overall span. A small ratio here means GroverOptimizer needs
    # correspondingly more value qubits (num_value_qubits) than the "range"
    # alone would suggest, just to tell the true optimum apart from its
    # closest rival -- see run_qubo_pipeline's docstring and format_report.
    surrogate_optimum_gap = surrogate_values[1] - surrogate_values[0]
    surrogate_value_range = surrogate_values[-1] - surrogate_values[0]

    qprog = _build_quadratic_program(constant, linear, quadratic)
    t0 = time.time()
    best_result, best_bits = None, None
    trial_hits = 0
    total_oracle_calls = 0
    for _ in range(grover_trials):
        grover = GroverOptimizer(num_value_qubits=num_value_qubits, num_iterations=num_iterations, sampler=Sampler())
        result = grover.solve(qprog)
        bits = tuple(int(round(v)) for v in result.x)
        # Total Grover-oracle-family operations across every adaptive-search
        # round of this trial (each round's own circuit applies the oracle
        # 'num_iterations' times; GAS runs several rounds until it stops
        # improving) -- summed from result.operation_counts, which records
        # op counts per round.
        total_oracle_calls += sum(
            counts.get("Q", 0) + counts.get("Q(x)", 0) for counts in result.operation_counts.values()
        )
        if bits == surrogate_exhaustive_best_bits:
            trial_hits += 1
        if best_result is None or result.fval < best_result.fval:
            best_result, best_bits = result, bits
    grover_wall_time_s = time.time() - t0

    return QuboRunResult(
        true_table=table,
        surrogate_constant=constant, surrogate_linear=linear, surrogate_quadratic=quadratic,
        surrogate_r_squared=r_squared,
        true_best_bits=true_best_bits,
        surrogate_exhaustive_best_bits=surrogate_exhaustive_best_bits,
        grover_best_bits=best_bits,
        grover_fval=float(best_result.fval),
        grover_trials=grover_trials,
        grover_trial_hits=trial_hits,
        classical_wall_time_s=classical_wall_time_s,
        grover_wall_time_s=grover_wall_time_s,
        n_input_qubits=best_result.n_input_qubits,
        n_output_qubits=best_result.n_output_qubits,
        oracle_calls=total_oracle_calls,
        grover_matches_surrogate_optimum=(best_bits == surrogate_exhaustive_best_bits),
        surrogate_matches_true_optimum=(surrogate_exhaustive_best_bits == true_best_bits),
        surrogate_optimum_gap=surrogate_optimum_gap,
        surrogate_value_range=surrogate_value_range,
    )


def format_report(r: QuboRunResult) -> str:
    true_best = r.true_table[r.true_best_bits][0]
    lines = [
        "=== QUBO threshold search: classical exhaustive vs. Grover Adaptive Search ===",
        f"Candidates evaluated: {len(r.true_table)} (2^{N_BITS})",
        f"Surrogate quadratic fit R^2 (vs. true re-simulated cost table): {r.surrogate_r_squared:.4f}",
        "",
        f"Classical exhaustive best (true cost table): tau1={true_best.tau1}, tau2={true_best.tau2}, "
        f"N={true_best.N}, T_dwell={true_best.T_dwell} -> J={r.true_table[r.true_best_bits][1]:.4f}",
        f"Classical exhaustive wall-clock: {r.classical_wall_time_s * 1000:.3f} ms",
        "",
        f"Grover Adaptive Search best-of-{r.grover_trials} (on the quadratic surrogate): "
        f"bits={r.grover_best_bits}, fval={r.grover_fval:.4f}",
        f"  ({r.grover_trial_hits}/{r.grover_trials} independent trials found the true surrogate "
        "optimum -- GAS is a probabilistic search, not a guaranteed single-shot result)",
        f"Grover wall-clock ({r.grover_trials} trials total): {r.grover_wall_time_s:.3f} s",
        f"Qubits used per trial: {r.n_input_qubits} input + {r.n_output_qubits} value = {r.n_input_qubits + r.n_output_qubits}",
        f"Oracle-family operations across all trials/rounds: {r.oracle_calls}",
        "",
        f"Grover found the surrogate's true minimizer: {r.grover_matches_surrogate_optimum}",
        f"Surrogate's minimizer matches the TRUE (re-simulated) table's minimizer: {r.surrogate_matches_true_optimum}",
        "",
        f"Surrogate optimum's margin over its closest rival: {r.surrogate_optimum_gap:.4f} "
        f"(objective spans {r.surrogate_value_range:.4f} overall, a "
        f"{100 * r.surrogate_optimum_gap / r.surrogate_value_range:.2f}% margin)",
    ]
    if r.surrogate_optimum_gap / r.surrogate_value_range < 0.01:
        lines.append(
            "  This margin is small relative to the objective's range -- resolving it reliably would need"
        )
        lines.append(
            "  more value qubits than are classically simulable at reasonable cost here (num_value_qubits=14"
        )
        lines.append(
            "  was observed, during development, to take minutes per Grover trial). A low Grover hit rate above"
        )
        lines.append(
            "  is consistent with this genuine value-qubit resolution limit, not necessarily a search failure."
        )
    lines += [
        "",
        "No speedup claim: this compares a 64-entry classical min() against a full",
        "Grover-Adaptive-Search circuit run on a simulator, at a scale chosen for",
        "tractability, not to demonstrate an advantage either way.",
    ]
    return "\n".join(lines)


if __name__ == "__main__":
    result = run_qubo_pipeline()
    print(format_report(result))
