"""Fixed-step RK4 integration with zero-order-hold control (CLAUDE.md section 5)."""

from typing import Callable, Tuple

import numpy as np

State = np.ndarray
Input = Tuple[float, ...]
DynamicsFn = Callable[[State, Input], State]
InputFn = Callable[[float, State], Input]


def rk4_step(f: DynamicsFn, x: State, u: Input, dt: float) -> State:
    """One fixed-step RK4 update. `u` is held constant across the step (ZOH)."""
    k1 = f(x, u)
    k2 = f(x + dt / 2 * k1, u)
    k3 = f(x + dt / 2 * k2, u)
    k4 = f(x + dt * k3, u)
    return x + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)


def simulate(
    f: DynamicsFn,
    x0: State,
    u_fn: InputFn,
    dt: float,
    n_steps: int,
) -> np.ndarray:
    """Integrate `f` for `n_steps` of size `dt`, sampling `u_fn(t, x)` once per step.

    Returns an (n_steps + 1, len(x0)) array of states, including x0.
    """
    xs = np.zeros((n_steps + 1, x0.shape[0]))
    xs[0] = x0
    x = x0.copy()
    t = 0.0
    for i in range(n_steps):
        u = u_fn(t, x)
        x = rk4_step(f, x, u, dt)
        xs[i + 1] = x
        t += dt
    return xs
