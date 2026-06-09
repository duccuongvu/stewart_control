# Stewart Platform — Ball Balancing & Bouncing

A 6-DOF Stewart platform (hexapod) in [MuJoCo](https://mujoco.org/) that
balances and bounces a ball on a tray, controlled by a task-space **sliding-mode
controller** (SMC) on the platform pose plus an outer PID that converts ball
position error into a tray tilt.

<p align="center">
  <img src="model/stewart_platform.png" width="420">
</p>

## Demos

| Ball balancing | Ball bouncing |
| --- | --- |
| ▶ [outputs/balance.mp4](outputs/balance.mp4) | ▶ [outputs/bounce.mp4](outputs/bounce.mp4) |

*Balancing*: the ball rolls to a square of targets. *Bouncing*: the ball bounces
while travelling to the same targets, sustained by a pumped vertical reference.

<!-- These tags play in IDEs / local viewers; GitHub shows the links above. -->
<p align="center">
  <table>
    <tr>
      <td style="padding-right:20px;">
        <img src="doc/balancing.gif" height="420">
      </td>
      <td>
        <img src="doc/bouncing.gif" height="420">
      </td>
    </tr>
  </table>
</p>
## Install

The MuJoCo platform model is a git **submodule** (`model/stewart_platform_mujoco`,
from [duccuongvu/stewart_platform_mujoco](https://github.com/duccuongvu/stewart_platform_mujoco))
and is never modified — the tray, ball and scene are added in memory at load time.

```bash
git submodule update --init          # fetch the platform model (or clone with --recurse-submodules)

python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

No compilation step — the controller model is pure NumPy/SciPy.

## Run

From the repo root:

```bash
python -m demos.balance_ball          # ball balancing, live viewer
python -m demos.bounce_ball           # ball bouncing, live viewer

python -m demos.balance_ball --video  # render to outputs/balance.mp4 (headless)
python -m demos.bounce_ball  --video  # render to outputs/bounce.mp4
```

Flags: `--duration <seconds>`, `--video [path]`, `--fps <n>`. Headless machines
should use `--video` (offscreen render); set `MUJOCO_GL=egl` if needed.

## How it works

```
ball (x,y) error ──▶ BallBalancingController (PID) ──▶ (roll, pitch) reference
                                                              │
upper-platform pose/twist ──▶ SlidingModeController (SMC) ◀───┘
                                       │
                                       ▼
                              6 leg forces ──▶ MuJoCo
```

- **Inner loop — `SlidingModeController`**: task-space computed-force SMC. Uses the
  platform Jacobian, task-space mass matrix and gravity wrench (from
  `StewartModel`) to produce the six leg forces that track a desired platform
  pose `[x, y, z, roll, pitch, yaw]`.
- **Outer loop — `BallBalancingController`**: two PIDs map ball position/velocity
  error on the tray to a `(roll, pitch)` reference for the platform.

## Layout

```
model/
  stewart_platform_mujoco/   git submodule: the pristine MuJoCo platform model (never edited)
  stewart_platform.png       hero render
stewart/    python package: model, controllers (smc, ball), sim harness, scene builder, config
demos/      runnable entry points: balance_ball.py, bounce_ball.py
outputs/    rendered videos
```

`stewart/scene.py` loads the submodule model and, via MuJoCo's `MjSpec` API, adds
the ball tray (a child of the `upper` body), the ball, the ball sensors and the
visual scene in memory — so the submodule files stay untouched. The platform
geometry and the upper-platform mass/inertia used for control design are recorded
in `stewart/config.py`, taken directly from that model.

## License

MIT (see `model/stewart_platform_mujoco/LICENSE`). Platform model by
[Duc Cuong Vu](https://github.com/duccuongvu) and
[Viet Khanh Nguyen](https://github.com/vietkhanh-nguyen).
