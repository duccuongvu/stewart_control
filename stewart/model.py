"""Analytic rigid-body model of the Stewart platform.

Provides the quantities the sliding-mode controller needs in task space:
inverse kinematics (leg vectors/lengths), the 6x6 Jacobian d(leg length)/d(pose),
the task-space mass/inertia matrix of the moving platform, and the gravity wrench.

State convention: ``q = [Px, Py, Pz, roll, pitch, yaw]`` with ZYX Euler angles,
matching ``utils.quat_to_euler`` ('xyz' extrinsic == 'ZYX' intrinsic here).
"""

import numpy as np
from scipy.spatial.transform import Rotation as R

from .utils import skew


def _euler_rate_to_omega(roll, pitch):
    """Map ZYX Euler-angle rates to platform angular velocity (the ``T`` matrix)."""
    sa, ca = np.sin(roll), np.cos(roll)
    sb, cb = np.sin(pitch), np.cos(pitch)
    return np.array([
        [1.0, 0.0, -sb],
        [0.0, ca, cb * sa],
        [0.0, -sa, cb * ca],
    ])


class StewartModel:
    def __init__(self, mass, inertia, base_anchors, top_anchors, gravity=9.81):
        self.mu = float(mass)
        self.Ix, self.Iy, self.Iz = (float(v) for v in inertia)
        self.B = np.asarray(base_anchors, dtype=float)   # (6, 3) base frame
        self.T = np.asarray(top_anchors, dtype=float)     # (6, 3) platform frame
        self.g = float(gravity)

    def inverse_kinematics(self, q):
        """Return (leg_vectors (6,3), leg_lengths (6,), rotation matrix (3,3))."""
        Px, Py, Pz, roll, pitch, yaw = q
        Rm = R.from_euler("ZYX", [yaw, pitch, roll]).as_matrix()
        top_world = np.array([Px, Py, Pz]) + (Rm @ self.T.T).T   # (6, 3)
        leg_vectors = top_world - self.B                          # (6, 3)
        leg_lengths = np.linalg.norm(leg_vectors, axis=1)
        return leg_vectors, leg_lengths, Rm

    def jacobian(self, q):
        """6x6 Jacobian mapping platform twist [v; euler_rate] to leg-length rates."""
        _, _, _, roll, pitch, _ = q
        leg_vectors, leg_lengths, Rm = self.inverse_kinematics(q)
        units = leg_vectors / leg_lengths[:, None]                # (6, 3)
        T_omega = _euler_rate_to_omega(roll, pitch)
        J = np.empty((6, 6))
        for i in range(6):
            Ji_rot = -Rm @ skew(self.T[i]) @ T_omega              # (3, 3)
            Ji = np.hstack([np.eye(3), Ji_rot])                   # (3, 6)
            J[i] = units[i] @ Ji
        return J

    def mass_matrix(self, q):
        """6x6 task-space mass/inertia matrix of the moving platform."""
        roll, yaw = q[3], q[5]
        ca, sa = np.cos(roll), np.sin(roll)
        cg, sg = np.cos(yaw), np.sin(yaw)

        Ix_minus_Iy = self.Ix - self.Iy
        Iz_sa2 = self.Iz * sa ** 2
        cross = ca * cg * sg * Ix_minus_Iy
        Ixx = self.Ix * cg ** 2 + self.Iy * sg ** 2
        Iyy = Iz_sa2 + ca ** 2 * (self.Ix * sg ** 2 + self.Iy * cg ** 2)
        Iyz = -Iz_sa2

        M = np.zeros((6, 6))
        M[0, 0] = M[1, 1] = M[2, 2] = self.mu
        M[3, 3] = Ixx
        M[3, 4] = M[4, 3] = cross
        M[4, 4] = Iyy
        M[4, 5] = M[5, 4] = Iyz
        M[5, 5] = self.Iz
        return M

    def gravity_wrench(self):
        """Gravity generalized force on the platform, [Fx Fy Fz Tx Ty Tz]^T (6,1)."""
        return np.array([0.0, 0.0, self.g * self.mu, 0.0, 0.0, 0.0]).reshape(6, 1)
