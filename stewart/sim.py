#
# Created on Wed Jun 10 2026
#
# Copyright (c) 2026 Duc-Cuong Vu - vdcuong2002@gmail.com
#

import time

import mujoco as mj
import numpy as np

from .utils import quat_to_euler


class StewartSim:
    def __init__(self, model, timestep=None):
        # Accept a prebuilt MjModel (see stewart.scene.build) or a scene path.
        self.model = model if isinstance(model, mj.MjModel) else mj.MjModel.from_xml_path(str(model))
        if timestep is not None:
            self.model.opt.timestep = timestep
        self.data = mj.MjData(self.model)
        self.dt = self.model.opt.timestep

    # ------------------------------------------------------------------ reads
    def upper_pose(self):
        """Platform pose [x, y, z, roll, pitch, yaw] relative to the base."""
        pos = self.data.sensor("upper_position").data
        euler = quat_to_euler(self.data.sensor("upper_quaternion").data)
        return np.concatenate([pos, euler])

    def upper_twist(self):
        """Platform twist [vx, vy, vz, wx, wy, wz] relative to the base."""
        lin = self.data.sensor("upper_linvel").data
        ang = self.data.sensor("upper_angvel").data
        return np.concatenate([lin, ang])

    def ball_state(self):
        """Ball (position, velocity) 3-vectors relative to the tray center."""
        pos = np.array(self.data.sensor("ball_position").data)
        vel = np.array(self.data.sensor("ball_velocity").data)
        return pos, vel

    # ----------------------------------------------------------------- writes
    def set_leg_forces(self, forces):
        for i in range(6):
            self.data.actuator(f"leg{i + 1}_ctrl").ctrl = forces[i]

    # ------------------------------------------------------------------- loops
    def run(self, control_fn, duration, video_path=None, fps=60,
            width=960, height=720, camera="track", realtime=True):
        mj.mj_resetData(self.model, self.data)
        mj.mj_forward(self.model, self.data)
        if video_path is not None:
            self._run_offscreen(control_fn, duration, video_path, fps, width, height, camera)
        else:
            self._run_viewer(control_fn, duration, realtime)

    def _run_offscreen(self, control_fn, duration, video_path, fps, width, height, camera):
        import imageio

        frames = []
        with mj.Renderer(self.model, height, width) as renderer:
            while self.data.time < duration:
                control_fn(self, self.data.time)
                mj.mj_step(self.model, self.data)
                if len(frames) < self.data.time * fps:
                    renderer.update_scene(self.data, camera=camera)
                    frames.append(renderer.render())
        imageio.mimsave(video_path, frames, fps=fps)
        print(f"wrote {video_path} ({len(frames)} frames @ {fps} fps)")

    def _run_viewer(self, control_fn, duration, realtime):
        import mujoco.viewer

        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            while viewer.is_running() and self.data.time < duration:
                t0 = time.time()
                control_fn(self, self.data.time)
                mj.mj_step(self.model, self.data)
                viewer.sync()
                if realtime:
                    sleep = self.dt - (time.time() - t0)
                    if sleep > 0:
                        time.sleep(sleep)
