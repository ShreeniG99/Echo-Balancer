import numpy as np

from sim.linearize import linearize, linearize_planar
from sim.params import default_plant_params
from sim.plant import f as plant_f


def _numeric_jacobian(p, eps=1e-6):
    """Central-difference Jacobian of plant_f at x=0, u=(0,0)."""
    x0 = np.zeros(6)
    n, m = 6, 2
    A = np.zeros((n, n))
    B = np.zeros((n, m))
    for i in range(n):
        dx = np.zeros(n)
        dx[i] = eps
        f_plus = plant_f(x0 + dx, 0.0, 0.0, p)
        f_minus = plant_f(x0 - dx, 0.0, 0.0, p)
        A[:, i] = (f_plus - f_minus) / (2 * eps)
    for j in range(m):
        u_plus = [eps if k == j else 0.0 for k in range(m)]
        u_minus = [-eps if k == j else 0.0 for k in range(m)]
        f_plus = plant_f(x0, u_plus[0], u_plus[1], p)
        f_minus = plant_f(x0, u_minus[0], u_minus[1], p)
        B[:, j] = (f_plus - f_minus) / (2 * eps)
    return A, B


def test_numeric_jacobian_matches_analytic():
    p = default_plant_params()
    A_analytic, B_analytic = linearize(p)
    A_numeric, B_numeric = _numeric_jacobian(p)

    np.testing.assert_allclose(A_analytic, A_numeric, rtol=1e-6, atol=1e-8)
    np.testing.assert_allclose(B_analytic, B_numeric, rtol=1e-6, atol=1e-8)


def test_planar_eigenvalues_match_sanity_values():
    p = default_plant_params()
    A_planar, _ = linearize_planar(p)
    eigs = np.sort(np.linalg.eigvals(A_planar).real)

    # CLAUDE.md section 6.1: open-loop eigenvalues ~= {0, -241, +7.44, -6.52} rad/s.
    target = np.sort(np.array([0.0, -241.0, 7.44, -6.52]))
    np.testing.assert_allclose(eigs, target, rtol=1e-2, atol=0.05)


def test_planar_is_a_consistent_reduction_of_the_full_system():
    """B's v_l and v_r columns must be identical in the planar rows: F_theta
    and F_psi depend only on (v_l + v_r), never on v_l and v_r individually."""
    p = default_plant_params()
    A, B = linearize(p)
    planar_rows = [0, 1, 3, 4]
    np.testing.assert_allclose(B[planar_rows, 0], B[planar_rows, 1])
