# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A 6-DOF Stewart platform in MuJoCo that **balances and bounces a ball on a tray**, controlled by a task-space **sliding-mode controller** (SMC). The control model is pure NumPy/SciPy — there is no build/compile step.

## Setup & run

```bash
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
```

Run demos as modules **from the repo root** (so `stewart` and `demos` import, and `model/` is found relative to the package):

```bash
python -m demos.balance_ball           # live passive viewer
python -m demos.bounce_ball
python -m demos.balance_ball --video   # offscreen render -> outputs/balance.mp4
```

Flags (see `demos/_common.py`): `--duration <s>`, `--video [path]`, `--fps <n>`.
Headless machines must use `--video`; offscreen rendering uses the framebuffer size set by `<global offwidth/offheight>` in the scene files (currently 1280×960). MP4 writing needs `imageio-ffmpeg` (in requirements). There is no test suite; verification is done by stepping a scene headless and checking the platform stabilises and the ball stays on the tray.

## Architecture

Two nested control loops drive the platform; see the diagram in `README.md`.

- **Inner loop — `stewart/controllers/smc.py` (`SlidingModeController`)**: task-space computed-force SMC. Given a desired platform pose `qr = [x,y,z,roll,pitch,yaw]` and the measured pose/twist, it outputs the **6 leg forces**. The law is `F = (M⁻¹Jᵀ)⁻¹ (q̈r − f + c0·e + c1·ė + c2·sat(s) + c3·s)` where `M`, `J`, `f=−M⁻¹·g` come from `StewartModel`. `boundary_layer>0` switches the `sign` term to `tanh(s/φ)` to cut chattering.
- **Outer loop — `stewart/controllers/ball.py` (`BallBalancingController`)**: two `PID`s map ball (x,y) position/velocity error on the tray to a `(roll, pitch)` tilt reference. The **y channel is sign-flipped** (tilting +roll moves the ball −y).
- **Model — `stewart/model.py` (`StewartModel`)**: analytic Jacobian `∂(leg length)/∂pose`, task-space mass matrix, and gravity wrench. Pure NumPy. State uses ZYX Euler; this matches `utils.quat_to_euler` (`'xyz'` extrinsic == `'ZYX'` intrinsic).
- **Harness — `stewart/sim.py` (`StewartSim`)**: loads a scene, exposes typed sensor reads (`upper_pose`, `upper_twist`, `ball_state`) and `set_leg_forces`, and runs a `control_fn(sim, t)` loop either in a live `mujoco.viewer` or rendered offscreen to MP4. Control is called once per `mj_step`.
- **Constants — `stewart/config.py`**: platform geometry (`BASE_ANCHORS`, `TOP_ANCHORS`) and upper-platform `UPPER_MASS`/`UPPER_INERTIA`, all taken directly from the MJCF. `SMC_GAINS` holds the tuned defaults.

A demo (`demos/balance_ball.py`, `demos/bounce_ball.py`) is just: build via `demos/_common.build(scene, ball_gains)`, define a `control(sim, t)` that reads ball state → outer PID → `qr` → SMC → `set_leg_forces`, then `sim.run(...)`.

## MuJoCo model (`model/`)

- `platform.xml` — the robot: base, 6 legs, moving `upper` platform, and a `tray` (cylinder, `class="tray"`, the only geoms with `contype/conaffinity=1` so it collides with the ball). Defines force actuators `leg1_ctrl..leg6_ctrl` (the actuated prismatic joints `joint_lu_1..6`), optional `upper_f_*/upper_t_*` disturbance-wrench actuators, and the `upper_position/quaternion/linvel/angvel` + leg-length sensors. Legs close the parallel loop via `<equality><weld>` from each leg tip site to a `site_upper_i`.
- `scene_balance.xml` / `scene_bounce.xml` — `<include platform.xml>` + floor/lights/skybox + a `track` camera + the **ball** body and `ball_position`/`ball_velocity` sensors (relative to the tray's `plane_site`). They differ only in ball drop height and contact `solref`:
  - **balance**: damped contact (`solref="0.006 0.9"`) → the ball rolls and is balanced.
  - **bounce**: elastic contact (`solref="0.01 0.15"`) dropped from `z=2.1` → decaying bounces kept centred. Note: lower dampratio (<0.15) injects energy on tilted contacts and the ball flies off; the heavy/slow platform cannot sustain an active paddle, so the bounce is intentionally passive (decaying).

## Conventions to preserve

- MuJoCo quaternions are `[w,x,y,z]`; `utils.quat_to_euler` reorders to SciPy `[x,y,z,w]` then `as_euler('xyz')`.
- `StewartSim.set_leg_forces` writes `leg{i+1}_ctrl`; leg force limit is ±400 N (typical demand ~90–130 N).
- The control model lumps everything into the 20 kg `upper` body; the unmodeled leg mass shows up as a ~3 cm steady-state z offset that the integral term (`c0`) slowly removes — this is expected, not a bug, and does not affect roll/pitch tracking.
