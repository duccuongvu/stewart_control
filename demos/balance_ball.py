"""Ball-balancing demo.

The ball rolls on the tray; an outer PID maps its position error to a tilt
reference, and the sliding-mode controller realises that tilt by driving the
six legs. The target walks around a square so the platform actively chases it.

Run:
    python -m demos.balance_ball                  # live viewer
    python -m demos.balance_ball --video          # render outputs/balance.mp4
"""

import numpy as np

from stewart import config
from demos._common import build, parse_args, resolve_video

# Ball xy targets on the tray, switched every DWELL seconds.
TARGETS = np.array([
    [0.12, 0.12],
    [-0.12, 0.12],
    [-0.12, -0.12],
    [0.12, -0.12],
    [0.0, 0.0],
])
DWELL = 3.0
BALL_GAINS = dict(kp=0.8, ki=0.0, kd=1.1)


def main():
    args = parse_args(__doc__, default_duration=20.0)
    sim, smc, ball = build("balance", BALL_GAINS)
    z_ref = config.HOME_POSE[2]

    def control(sim, t):
        ball_pos, ball_vel = sim.ball_state()
        target = TARGETS[int(t // DWELL) % len(TARGETS)]
        pos_err = target - ball_pos[:2]
        vel_err = -ball_vel[:2]
        roll_ref, pitch_ref = ball.step(pos_err, vel_err)

        qr = np.array([0.0, 0.0, z_ref, roll_ref, pitch_ref, 0.0])
        u = smc.compute(qr, sim.upper_pose(), dq=sim.upper_twist())
        sim.set_leg_forces(u)

    sim.run(control, args.duration,
            video_path=resolve_video(args, "balance.mp4"), fps=args.fps)


if __name__ == "__main__":
    main()
