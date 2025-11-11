<!-- # Math Tools for ENME480

Quick reference for the linear algebra and geometry you will touch every week. Pair these snippets with the detailed derivations in [kinematics-reference.md](kinematics-reference.md).

## Coordinate transforms
- Homogeneous transform:

```math
^{A}T_{B} =
\begin{bmatrix}
R & p \\
0 & 1
\end{bmatrix}
```

- Compose transforms with matrix multiplication; invert by transposing the rotation and negating the translated point: `T_AB.inverse() = [[Rᵀ, -Rᵀ p], [0, 1]]`.

## DH parameters
- Standard DH table columns: `aᵢ`, `αᵢ`, `dᵢ`, `θᵢ`.
- Use the UR3e table provided in [labs/week-05.md](labs/week-05.md).
- Build transforms programmatically with helper functions (see snippet below).

```python
import numpy as np

def dh_transform(a, alpha, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,       sa,      ca,    d],
        [0,        0,       0,    1],
    ])
```

## Jacobians
- Compute the geometric Jacobian by stacking `z × (pᴇ - pᵢ)` for revolute joints and `z` for prismatic joints.
- Numerical stability: avoid near-singular poses by checking `np.linalg.cond(J)` and backing away from zero determinant configurations.

## Linear algebra reminders
- **Matrix inverse:** avoid explicit inverses in code; use `numpy.linalg.solve`.
- **Least squares:** `np.linalg.lstsq(J, dx, rcond=None)` is the fastest way to compute a damped pseudo-inverse for IK tweaks.
- **Eigenvalues:** use `np.linalg.eig` for stability analysis of control matrices.

## Useful calculators & notebooks
- [Symbolab Matrix Calculator](https://www.symbolab.com/solver/matrix-calculator) for quick checks.
- [Wolfram Alpha](https://www.wolframalpha.com/input?i=dh+matrix) for DH sanity checks.
- [NumPy docs](https://numpy.org/doc/stable/reference/routines.linalg.html) for full linear algebra API.

Need concrete examples? Check [code-examples.md](code-examples.md) for ready-to-run Python snippets. -->
