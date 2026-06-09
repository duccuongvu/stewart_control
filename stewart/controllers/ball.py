"""Outer-loop ball controller: ball position error -> platform tilt reference."""

import numpy as np


class PID:
    def __init__(self, kp, ki, kd, dt, int_limit=200.0, deriv_alpha=0.2):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.dt = dt
        self.int_limit = int_limit
        self.alpha = deriv_alpha
        self.reset()

    def reset(self):
        self.integral = 0.0
        self._prev_e = None
        self._prev_de = 0.0

    def step(self, e, de=None):
        if self._prev_e is None:
            self._prev_e = e
        if de is None:
            de = (e - self._prev_e) / self.dt
        de = (1 - self.alpha) * self._prev_de + self.alpha * de
        self.integral = np.clip(self.integral + e * self.dt, -self.int_limit, self.int_limit)
        self._prev_e = e
        self._prev_de = de
        return self.kp * e + self.ki * self.integral + self.kd * de


class BallBalancingController:
    """Map ball (x, y) position/velocity error on the tray to (roll, pitch) refs.

    Tilting about +x (roll) moves the ball along -y, hence the y channel is
    sign-flipped relative to the x channel.
    """

    def __init__(self, kp, ki, kd, dt):
        self.pid_x = PID(kp, ki, kd, dt)   # -> pitch (tilt about y, moves ball in x)
        self.pid_y = PID(kp, ki, kd, dt)   # -> roll  (tilt about x, moves ball in y)

    def reset(self):
        self.pid_x.reset()
        self.pid_y.reset()

    def step(self, pos_err_xy, vel_err_xy):
        """Return ``(roll_ref, pitch_ref)`` from 2-vectors of position/velocity error."""
        pitch_ref = self.pid_x.step(pos_err_xy[0], vel_err_xy[0])
        roll_ref = self.pid_y.step(-pos_err_xy[1], -vel_err_xy[1])
        return roll_ref, pitch_ref
