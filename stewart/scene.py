#
# Created on Wed Jun 10 2026
#
# Copyright (c) 2026 Duc-Cuong Vu - vdcuong2002@gmail.com
#

from pathlib import Path

import mujoco as mj

# model/stewart_platform_mujoco/stewart_platform.xml (the submodule entry point).
SUBMODULE_XML = (Path(__file__).resolve().parent.parent
                 / "model" / "stewart_platform_mujoco" / "stewart_platform.xml")

# Tray (ball support) — identical for both demos.
_TRAY = dict(size=[0.3, 0.01, 0], mass=0.5, rgba=[0.85, 0.85, 0.88, 1],
             solimp=[0.95, 0.95, 0.001, 0.5, 2], solref=[0.006, 0.9])

# Per-demo ball: damped contact for balancing, elastic for bouncing.
BALL = {
    "balance": dict(pos=[0.12, 0.10, 1.22], size=0.025, mass=0.05, rgba=[0.85, 0.1, 0.1, 1],
                    solref=[0.006, 0.9], friction=[0.6, 0.01, 0.001]),
    "bounce": dict(pos=[0.05, 0.04, 1.6], size=0.03, mass=0.05, rgba=[0.95, 0.55, 0.1, 1],
                   solref=[0.01, 0.15], friction=[0.4, 0.01, 0.001]),
}


def build(kind, submodule_xml=SUBMODULE_XML):
    """Return a compiled ``MjModel`` for ``kind`` in {"balance", "bounce"}."""
    if kind not in BALL:
        raise ValueError(f"unknown demo kind {kind!r}; expected one of {list(BALL)}")
    if not Path(submodule_xml).exists():
        raise FileNotFoundError(
            f"platform model not found at {submodule_xml}.\n"
            "Initialise the submodule:  git submodule update --init")

    spec = mj.MjSpec.from_file(str(submodule_xml))

    # --- tray: a real child of the moving upper platform ---
    upper = next(b for b in spec.bodies if b.name == "upper")
    tray = upper.add_body(name="tray", pos=[0, 0, 0.01])
    g = tray.add_geom()
    g.name = "tray"
    g.type = mj.mjtGeom.mjGEOM_CYLINDER
    g.size = _TRAY["size"]
    g.rgba = _TRAY["rgba"]
    g.mass = _TRAY["mass"]
    g.contype = g.conaffinity = 1
    g.solimp = _TRAY["solimp"]
    g.solref = _TRAY["solref"]
    tray.add_site(name="plane_site")

    # --- ball ---
    b = BALL[kind]
    ball = spec.worldbody.add_body(name="ball", pos=b["pos"])
    ball.add_freejoint()
    gb = ball.add_geom()
    gb.name = "ball"
    gb.type = mj.mjtGeom.mjGEOM_SPHERE
    gb.size = [b["size"], 0, 0]
    gb.rgba = b["rgba"]
    gb.mass = b["mass"]
    gb.priority = 1            # ball's contact params win over the tray's
    gb.contype = gb.conaffinity = 1
    gb.solimp = [0.95, 0.95, 0.01, 0.5, 2]
    gb.solref = b["solref"]
    gb.friction = b["friction"]
    ball.add_site(name="ball_site")

    # --- ball sensors (relative to the tray centre) ---
    for name, stype in (("ball_position", mj.mjtSensor.mjSENS_FRAMEPOS),
                        ("ball_velocity", mj.mjtSensor.mjSENS_FRAMELINVEL)):
        s = spec.add_sensor()
        s.name = name
        s.type = stype
        s.objtype = mj.mjtObj.mjOBJ_SITE
        s.objname = "ball_site"
        s.reftype = mj.mjtObj.mjOBJ_SITE
        s.refname = "plane_site"

    # --- visual scene: floor, light, tracking camera, skybox ---
    floor = spec.worldbody.add_geom()
    floor.name = "floor"
    floor.type = mj.mjtGeom.mjGEOM_PLANE
    floor.size = [0, 0, 0.05]
    floor.rgba = [0.3, 0.32, 0.35, 1]

    light = spec.worldbody.add_light()
    light.type = mj.mjtLightType.mjLIGHT_DIRECTIONAL
    light.pos = [0, 0, 3.5]
    light.dir = [0, 0, -1]

    cam = spec.worldbody.add_camera()
    cam.name = "track"
    cam.pos = [1.7, -1.7, 2.2]
    cam.mode = mj.mjtCamLight.mjCAMLIGHT_TARGETBODY
    cam.targetbody = "upper"

    sky = spec.add_texture()
    sky.name = "skybox"
    sky.type = mj.mjtTexture.mjTEXTURE_SKYBOX
    sky.builtin = mj.mjtBuiltin.mjBUILTIN_GRADIENT
    sky.rgb1 = [0.3, 0.5, 0.7]
    sky.rgb2 = [0, 0, 0]
    sky.width = 512
    sky.height = 3072

    spec.visual.global_.offwidth = 1280
    spec.visual.global_.offheight = 960

    return spec.compile()
