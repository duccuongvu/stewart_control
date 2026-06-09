"""Ball-bouncing demo.

An elastic ball is dropped onto the tray from above. The outer PID keeps the
ball over the tray centre (so it bounces on the spot instead of skipping off the
edge) while the sliding-mode controller holds the platform level. The ball
bounces several times with decaying height and settles centred.

Run:
    python -m demos.bounce_ball                  # live viewer
    python -m demos.bounce_ball --video          # render outputs/bounce.mp4
"""

import numpy as np

from stewart import config
from demos._common import build, parse_args, resolve_video

BALL_GAINS = dict(kp=0.6, ki=0.0, kd=0.45)


def main():
    args = parse_args(__doc__, default_duration=16.0)
    sim, smc, ball = build("scene_bounce.xml", BALL_GAINS)
    home_z = config.HOME_POSE[2]

    def control(sim, t):
        ball_pos, ball_vel = sim.ball_state()
        # Keep the ball over the tray centre.
        roll_ref, pitch_ref = ball.step(-ball_pos[:2], -ball_vel[:2])
        qr = np.array([0.0, 0.0, home_z, roll_ref, pitch_ref, 0.0])
        u = smc.compute(qr, sim.upper_pose(), dq=sim.upper_twist())
        sim.set_leg_forces(u)

    sim.run(control, args.duration,
            video_path=resolve_video(args, "bounce.mp4"), fps=args.fps)


if __name__ == "__main__":
    main()
