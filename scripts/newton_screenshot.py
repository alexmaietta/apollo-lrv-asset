"""Headless Newton GL-viewer and camera-sensor screenshots of the LRV example (for the README).

Run: .venv/bin/python scripts/newton_screenshot.py [--out docs/images]
(Use a Newton with examples/newton/newton_viewer_materials.patch applied for cut-outs and double-sided meshes.)
"""
import argparse
import math
import os

import numpy as np
import warp as wp
from PIL import Image

import sys

import newton

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "examples", "newton"))
from example_robot_lrv import Example  # noqa: E402

VIEWS = {  # name: (camera position, look-at target)
    "newton_viewer": ((4.2, -3.4, 1.9), (0.2, 0.0, 0.6)),
    "newton_viewer_hga": ((2.3, 1.6, 1.0), (1.4, 0.4, 1.4)),
}


def look(viewer, pos, target):
    d = np.subtract(target, pos)
    d /= np.linalg.norm(d)
    viewer.set_camera(wp.vec3(*pos), math.degrees(math.asin(d[2])), math.degrees(math.atan2(d[1], d[0])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="docs/images")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    args = Example.create_parser().parse_args(["--viewer", "gl", "--headless", "--speed", "0"])
    viewer = newton.viewer.ViewerGL(headless=True, width=1600, height=900)
    ex = Example(viewer, args)
    for _ in range(30):
        ex.step()
    for name, (pos, tgt) in VIEWS.items():
        look(viewer, pos, tgt)
        ex.render()
        Image.fromarray(viewer.get_frame().numpy()).save(os.path.join(a.out, f"{name}.png"))
        print("wrote", name)
    # TV camera sensor frame: pan back over the crew station (+197 deg is inside the +214 deg stop)
    ex.poses["tv_pan"], ex.poses["tv_camera"] = math.radians(197), math.radians(-12)
    for _ in range(420):
        ex.step()
    ex.render_cameras()
    img = ex.cam_color.numpy().view(np.uint8).reshape(-1, ex.cam_h, ex.cam_w, 4)[0, ..., :3]
    Image.fromarray(img).save(os.path.join(a.out, "newton_tv_camera.png"))
    print("wrote newton_tv_camera")


if __name__ == "__main__":
    main()
