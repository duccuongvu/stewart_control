"""Ball-bouncing demo.

An elastic ball is dropped onto the tray. The controller does three things:

* outer PID steers the ball toward a target that walks around a square (the same
  idea as the balancing demo, but here the ball is bouncing while it travels);
* a vertical reference *pump* sustains the bounce: the platform's z reference
  integrates the ball's vertical velocity, so the tray rises to meet a falling
  ball and drops away as it rebounds, re-injecting energy on every contact;
* the sliding-mode controller realises that moving pose with the six legs.

The ±0.15 m clip on the pumped reference bounds the energy, so the bounce
sustains indefinitely. Tilt gains are kept gentle: an aggressive tilt during a
contact injects sideways energy and the ball skips off the tray.

Run:
    python -m demos.bounce_ball                  # live viewer
    python -m demos.bounce_ball --video          # render outputs/bounce.mp4
"""

import numpy as np

from demos._common import build, parse_args, resolve_video

# Ball xy targets on the tray, switched every DWELL seconds.
TARGETS = np.array([
    [0.08, 0.08],
    [-0.08, 0.08],
    [-0.08, -0.08],
    [0.08, -0.08],
    [0.0, 0.0],
])
DWELL = 4.0
BALL_GAINS = dict(kp=0.2, ki=0.0, kd=0.15)
BASE_Z = 1.15            # platform height the bounce oscillates around (m)
Z_PUMP_GAIN = 0.01       # how strongly the z reference chases the ball velocity
Z_PUMP_LIMIT = 0.15      # clip on the pumped reference (m); bounds bounce energy


def main():
    args = parse_args(__doc__, default_duration=24.0)
    sim, smc, ball = build("bounce", BALL_GAINS)
    z_ref = 0.0

    def control(sim, t):
        nonlocal z_ref
        ball_pos, ball_vel = sim.ball_state()
        target = TARGETS[int(t // DWELL) % len(TARGETS)]
        roll_ref, pitch_ref = ball.step(target - ball_pos[:2], -ball_vel[:2])
        z_ref = float(np.clip(z_ref - Z_PUMP_GAIN * ball_vel[2],
                              -Z_PUMP_LIMIT, Z_PUMP_LIMIT))
        qr = np.array([0.0, 0.0, BASE_Z + z_ref, roll_ref, pitch_ref, 0.0])
        u = smc.compute(qr, sim.upper_pose(), dq=sim.upper_twist())
        sim.set_leg_forces(u)

    sim.run(control, args.duration,
            video_path=resolve_video(args, "bounce.mp4"), fps=args.fps)


if __name__ == "__main__":
    main()
