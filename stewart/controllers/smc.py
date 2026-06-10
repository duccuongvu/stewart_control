#
# Created on Wed Jun 10 2026
#
# Copyright (c) 2026 Duc-Cuong Vu - vdcuong2002@gmail.com
#

import numpy as np


class SlidingModeController:
    def __init__(self, model, c0=0.0, c1=8.0, c2=8.0, c3=8.0,
                 boundary_layer=0.0, dt=0.002, vel_filter_alpha=0.2):
        self.model = model
        self.c0, self.c1, self.c2, self.c3 = c0, c1, c2, c3
        self.phi = boundary_layer
        self.dt = dt
        self.alpha = vel_filter_alpha
        self.reset()

    def reset(self):
        self.integral = np.zeros((6, 1))
        self._prev_q = None
        self._prev_dq = np.zeros((6, 1))

    def _saturate(self, s):
        if self.phi > 0.0:
            return np.tanh(s / self.phi)
        return np.sign(s)

    def compute(self, qr, q, dq=None, dqr=None, ddqr=None):
        """Return the 6 leg forces. All pose/vel args are length-6 vectors."""
        qr = np.asarray(qr, dtype=float).reshape(6, 1)
        q = np.asarray(q, dtype=float).reshape(6, 1)
        dqr = np.zeros((6, 1)) if dqr is None else np.asarray(dqr, float).reshape(6, 1)
        ddqr = np.zeros((6, 1)) if ddqr is None else np.asarray(ddqr, float).reshape(6, 1)

        if dq is None:
            if self._prev_q is None:
                self._prev_q = q
            raw = (q - self._prev_q) / self.dt
            dq = (1 - self.alpha) * self._prev_dq + self.alpha * raw
            self._prev_q = q
            self._prev_dq = dq
        else:
            dq = np.asarray(dq, dtype=float).reshape(6, 1)

        e = qr - q
        de = dqr - dq
        self.integral += e * self.dt
        s = de + self.c1 * e + self.c0 * self.integral

        qf = q.flatten()
        M = self.model.mass_matrix(qf)
        J = self.model.jacobian(qf)
        Minv = np.linalg.inv(M)
        G = Minv @ J.T
        f = Minv @ (-self.model.gravity_wrench())

        bracket = (ddqr - f + self.c0 * e + self.c1 * de
                   + self.c2 * self._saturate(s) + self.c3 * s)
        F = np.linalg.solve(G, bracket)
        return F.flatten()
