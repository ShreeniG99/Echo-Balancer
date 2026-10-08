"""Exact cost-table -> binary polynomial -> QUBO encodings (CLAUDE.md section 13).

A candidate index c in [0, 2^n) is encoded in n binary variables
(little-endian: bit i of c is x_i). Any cost table J[c] is *exactly* a
multilinear polynomial in x (the Mobius transform of the table), but in
general of degree up to n -- a HUBO, not a QUBO. `quadratize` reduces it
exactly to a QUBO with Rosenberg substitution (auxiliary y = x_i x_j plus a
penalty that is 0 iff y = x_i x_j and >= M otherwise), so the QUBO minimum
over (x, y) equals min_c J[c] and its x-part is the argmin. Pure functions,
no qiskit dependency.
"""

from itertools import combinations

import numpy as np

Monomial = tuple[int, ...]  # sorted variable indices; () is the constant term
Poly = dict[Monomial, float]


def table_to_hubo(J: np.ndarray, tol: float = 0.0) -> Poly:
    """Exact multilinear polynomial p with p(bits(c)) == J[c] for every c."""
    J = np.asarray(J, dtype=float)
    n = int(round(np.log2(J.size)))
    if 2**n != J.size:
        raise ValueError("cost table length must be a power of two")
    a = J.copy()
    for i in range(n):  # Mobius transform over the subset lattice
        bit = 1 << i
        for c in range(J.size):
            if c & bit:
                a[c] -= a[c ^ bit]
    return {tuple(i for i in range(n) if c >> i & 1): float(a[c]) for c in range(J.size) if abs(a[c]) > tol}


def evaluate(poly: Poly, x: np.ndarray) -> float:
    return float(sum(coef * np.prod([x[i] for i in mono]) for mono, coef in poly.items()))


def quadratize(hubo: Poly, n_vars: int, penalty: float | None = None) -> tuple[Poly, int, list[tuple[int, int, int]]]:
    """Rosenberg reduction to degree <= 2. Returns (qubo, total_vars, subs)
    with subs = [(aux, i, j)] meaning aux == x_i * x_j at any minimum.

    penalty: one global penalty weight; by default each aux gets its own
    (1 + sum|coef| of the objective terms that contain it), which keeps
    coefficients small. Use verify_exact to prove the minimum is preserved.
    """
    M = penalty if penalty is not None else 1.0 + sum(abs(c) for c in hubo.values())
    terms: dict[Monomial, float] = dict(hubo)
    subs: list[tuple[int, int, int]] = []
    nxt = n_vars
    while True:
        high = [m for m in terms if len(m) > 2]
        if not high:
            break
        counts: dict[tuple[int, int], int] = {}
        for m in high:
            for p in combinations(m, 2):
                counts[p] = counts.get(p, 0) + 1
        i, j = max(sorted(counts), key=lambda p: counts[p])  # deterministic tie-break
        y = nxt
        nxt += 1
        subs.append((y, i, j))
        new: dict[Monomial, float] = {}
        for m, c in terms.items():
            if len(m) > 2 and i in m and j in m:
                m = tuple(sorted([v for v in m if v not in (i, j)] + [y]))
            new[m] = new.get(m, 0.0) + c
        terms = new
    for y, i, j in subs:  # M_y * (x_i x_j - 2 x_i y - 2 x_j y + 3 y)
        # Tight per-aux penalty: 1 + sum|coef| of objective terms containing y bounds what a wrong
        # y can gain. Callers verify minima by brute force (see verify_exact), so this is checked.
        My = M if penalty is not None else 1.0 + sum(abs(c) for m, c in terms.items() if y in m)
        for m, c in (((i, j), My), ((i, y), -2 * My), ((j, y), -2 * My), ((y,), 3 * My)):
            terms[m] = terms.get(m, 0.0) + c
    return {m: c for m, c in terms.items() if c != 0.0}, nxt, subs


def brute_force_min(poly: Poly, n_vars: int) -> tuple[np.ndarray, float]:
    """Exhaustive minimum over all 2^n_vars assignments (vectorised; n_vars <= ~24)."""
    idx = np.arange(2**n_vars, dtype=np.int64)
    X = ((idx[:, None] >> np.arange(n_vars)) & 1).astype(np.int8)
    vals = np.zeros(idx.size)
    for mono, coef in poly.items():
        vals += coef * (X[:, list(mono)].all(axis=1) if mono else 1.0)
    k = int(np.argmin(vals))
    return X[k].astype(int), float(vals[k])


def bits_to_index(x: np.ndarray, n: int) -> int:
    return int(sum(int(round(x[i])) << i for i in range(n)))


def rank_transform(J: np.ndarray) -> np.ndarray:
    """Dense integer ranks (ties share a rank). Monotone, so argmin is unchanged,
    while QUBO coefficients (and Grover's value register) stay small."""
    _, inv = np.unique(np.asarray(J), return_inverse=True)
    return inv.astype(float)


def verify_exact(qubo: Poly, n_vars: int, J: np.ndarray, n_index: int) -> tuple[int, float]:
    """Brute-force the QUBO; assert its minimiser decodes to argmin J. Returns (index, min value)."""
    x, v = brute_force_min(qubo, n_vars)
    idx = bits_to_index(x, n_index)
    if not np.isclose(J[idx], np.min(J)):
        raise AssertionError(f"QUBO minimum decodes to {idx}, not an argmin of J")
    return idx, v


def value_range(qubo: Poly) -> float:
    """Upper bound on |QUBO(x)| over all x (sum of |coef|)."""
    return float(sum(abs(c) for c in qubo.values()))
