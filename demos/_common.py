"""Shared wiring for the demos: build the model + controllers + simulation."""

import argparse
from pathlib import Path

from stewart import StewartModel, StewartSim, SlidingModeController, BallBalancingController
from stewart import config, scene

OUTPUT_DIR = Path(__file__).resolve().parent.parent / "outputs"


def build(kind, ball_gains):
    """Return (sim, smc, ball_ctrl) for demo ``kind`` in {"balance", "bounce"}."""
    sim = StewartSim(scene.build(kind))
    model = StewartModel(config.UPPER_MASS, config.UPPER_INERTIA,
                         config.BASE_ANCHORS, config.TOP_ANCHORS, config.GRAVITY)
    smc = SlidingModeController(model, dt=sim.dt, **config.SMC_GAINS)
    ball = BallBalancingController(dt=sim.dt, **ball_gains)
    return sim, smc, ball


def parse_args(description, default_duration):
    p = argparse.ArgumentParser(description=description)
    p.add_argument("--duration", type=float, default=default_duration,
                   help="simulation time in seconds")
    p.add_argument("--video", nargs="?", const="auto", default=None,
                   help="render offscreen to an MP4 instead of the live viewer; "
                        "optionally give a path (default outputs/<demo>.mp4)")
    p.add_argument("--fps", type=int, default=60, help="video frame rate")
    return p.parse_args()


def resolve_video(args, default_name):
    if args.video is None:
        return None
    OUTPUT_DIR.mkdir(exist_ok=True)
    if args.video == "auto":
        return str(OUTPUT_DIR / default_name)
    return args.video
