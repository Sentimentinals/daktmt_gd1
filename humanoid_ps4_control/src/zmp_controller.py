"""
ZMP preview controller based on Kajita's LIPM formulation.

The controller tracks a desired ZMP sequence and outputs the lateral CoM
position. It uses backward Riccati iteration, which avoids requiring scipy for
the discrete algebraic Riccati equation.
"""

from __future__ import annotations

import numpy as np


class ZMPPreviewController:
    """
    Preview LQR for one LIPM axis.

    State : [x, xdot, xddot] in mm, mm/s, mm/s^2
    Input : jerk u in mm/s^3
    ZMP   : p = x - (zc / g) * xddot
    """

    def __init__(
        self,
        dt: float,
        zc: float,
        g: float = 9800.0,
        Qe: float = 1.0,
        R: float = 1e-6,
        preview_steps: int = 24,
        riccati_iters: int = 3000,
    ) -> None:
        self.dt = dt
        self.zc = zc
        self.g = g
        self.preview_steps = preview_steps
        self._zmp_accel_coeff = zc / g

        A = np.array(
            [
                [1.0, dt, 0.5 * dt**2],
                [0.0, 1.0, dt],
                [0.0, 0.0, 1.0],
            ]
        )
        B = np.array([[dt**3 / 6.0], [0.5 * dt**2], [dt]])
        C = np.array([[1.0, 0.0, -zc / g]])

        self._A = A
        self._B_vec = B.ravel()

        # Augmented state: [integrated_zmp_error, x, xdot, xddot].
        state_size = 4
        Aa = np.zeros((state_size, state_size))
        Aa[0, 0] = 1.0
        Aa[0, 1:] = (C @ A).ravel()
        Aa[1:, 1:] = A

        Ba = np.zeros((state_size, 1))
        Ba[0, 0] = (C @ B).item()
        Ba[1:] = B

        Qa = np.diag([Qe, 0.0, 0.0, 0.0])
        P = Qa.copy()

        for _ in range(riccati_iters):
            AtPA = Aa.T @ P @ Aa
            AtPB = Aa.T @ P @ Ba
            BtPB = (Ba.T @ P @ Ba).item()
            P_next = Qa + AtPA - AtPB @ AtPB.T / (R + BtPB)
            if np.max(np.abs(P_next - P)) < 1e-10:
                P = P_next
                break
            P = P_next

        BtPB = (Ba.T @ P @ Ba).item()
        K = (Ba.T @ P @ Aa) / (R + BtPB)
        self.Ki = float(K[0, 0])
        self.Kx = K[0, 1:].ravel()

        Ac = Aa - Ba @ K
        G_list: list[float] = []
        e1 = np.zeros((state_size, 1))
        e1[0] = 1.0
        # Preview gain recursion from the LQ preview controller:
        # Gp(j) = inv(R + B^T P B) B^T (Ac^T)^(j-1) P I.
        preview_vec = P @ e1
        for _ in range(preview_steps):
            G_list.append(((Ba.T @ preview_vec) / (R + BtPB)).item())
            preview_vec = Ac.T @ preview_vec
        self.G = np.array(G_list)

        self._x = np.zeros(3)
        self._ei = 0.0

    def reset(self, x0: float = 0.0) -> None:
        self._x[:] = 0.0
        self._x[0] = x0
        self._ei = 0.0

    def is_settled(
        self,
        position_tol: float = 0.2,
        velocity_tol: float = 1.0,
        acceleration_tol: float = 20.0,
        integral_tol: float = 1.0,
    ) -> bool:
        return (
            abs(float(self._x[0])) <= position_tol
            and abs(float(self._x[1])) <= velocity_tol
            and abs(float(self._x[2])) <= acceleration_tol
            and abs(self._ei) <= integral_tol
        )

    def step(self, zmp_ref_now: float, zmp_preview: list[float]) -> float:
        """Advance one controller tick and return the new CoM position."""
        zmp_cur = self._x[0] - self._zmp_accel_coeff * self._x[2]

        # Kajita convention: e(k) = p(k) - p_ref(k).
        self._ei += zmp_cur - zmp_ref_now

        preview_sum = float(np.dot(self.G, zmp_preview[: self.preview_steps]))
        u = -self.Ki * self._ei - float(self.Kx @ self._x) - preview_sum
        self._x = self._A @ self._x + self._B_vec * u
        return float(self._x[0])
