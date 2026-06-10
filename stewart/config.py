#
# Created on Wed Jun 10 2026
#
# Copyright (c) 2026 Duc-Cuong Vu - vdcuong2002@gmail.com
#

import numpy as np

# Base anchor points B_i in the fixed/base frame (cardan1_1..6 positions).
BASE_ANCHORS = np.array([
    [0.48295431, -0.12941450, 0.08690238],
    [0.12940089, 0.48295792, 0.08690238],
    [-0.12941813, 0.48295792, 0.08690238],
    [-0.48297152, -0.12941450, 0.08690238],
    [-0.35356201, -0.35355835, 0.08690238],
    [0.35354476, -0.35355835, 0.08690238],
])

# Top anchor points T_i in the moving-platform frame (site_upper_1..6 positions).
TOP_ANCHORS = np.array([
    [0.3380740, 0.0905866, -0.0419024],
    [0.2474874, 0.2474874, -0.0419024],
    [-0.2474874, 0.2474874, -0.0419024],
    [-0.3380740, 0.0905866, -0.0419024],
    [-0.0905867, -0.3380740, -0.0419024],
    [0.0905867, -0.3380740, -0.0419024],
])

# Upper-platform inertial properties (body frame), from MuJoCo inertiafromgeom.
UPPER_MASS = 20.0
UPPER_INERTIA = np.array([0.79079, 0.79079, 1.57816])  # Ixx, Iyy, Izz
GRAVITY = 9.81

# Nominal (home) platform pose: [x, y, z, roll, pitch, yaw].
HOME_POSE = np.array([0.0, 0.0, 1.1, 0.0, 0.0, 0.0])

# Default sliding-mode gains (see stewart/controllers/smc.py).
SMC_GAINS = dict(c0=2.0, c1=10.0, c2=6.0, c3=12.0, boundary_layer=0.02)
