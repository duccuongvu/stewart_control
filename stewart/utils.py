"""Small numeric helpers shared across the package."""

import numpy as np
from scipy.spatial.transform import Rotation


def quat_to_euler(quat_mujoco, degrees=False):
    """Convert a MuJoCo quaternion ``[w, x, y, z]`` to XYZ Euler ``[roll, pitch, yaw]``.

    SciPy expects ``[x, y, z, w]`` order, so the components are reordered first.
    """
    w, x, y, z = quat_mujoco
    return Rotation.from_quat([x, y, z, w]).as_euler("xyz", degrees=degrees)


def skew(v):
    """Skew-symmetric matrix of a 3-vector."""
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])
