"""Apply a recorded Newton run (body_q per frame) to the Blender model and render frames.

Run: blender -b build/lrv.blend -P scripts/playback.py -- --run build/sim/run.npz
       [--frames 450,800] [--every 0] [--tag sim] [--follow] [--samples 48]
"""
import os
import sys

import bpy
import numpy as np
from mathutils import Quaternion, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render as R  # noqa: E402


def main():
    a = sys.argv[sys.argv.index("--") + 1:]
    get = lambda k, d: a[a.index(k) + 1] if k in a else d  # noqa: E731
    run = np.load(get("--run", "build/sim/run.npz"))
    body_q, names = run["body_q"], [str(n) for n in run["bodies"]]
    frames = [int(f) for f in get("--frames", "0").split(",")]
    every = int(get("--every", "0"))
    if every:
        frames = list(range(0, len(body_q), every))
    tag = get("--tag", "sim")

    empties = {o["link"]: o for o in bpy.data.objects if o.type == "EMPTY" and "link" in o}
    for e in empties.values():
        e.parent = None
        e.rotation_mode = "QUATERNION"

    o = R.args()
    o["samples"] = int(get("--samples", "48"))
    R.setup_render(o)
    R.setup_world(False)
    os.makedirs("renders", exist_ok=True)
    for f in frames:
        q = body_q[f]
        for i, n in enumerate(names):
            if n in empties:
                x, y, z, qx, qy, qz, qw = q[i]
                empties[n].location = (x, y, z)
                empties[n].rotation_quaternion = Quaternion((qw, qx, qy, qz))
        bpy.context.view_layer.update()
        c = Vector(q[names.index("chassis")][:3])
        R.camera("cam", c + Vector((4.2, 3.4, 2.4)), c + Vector((0, 0, 0.5)), 40)
        bpy.context.scene.render.filepath = os.path.abspath(f"renders/{tag}_{f:05d}.png")
        bpy.ops.render.render(write_still=True)
        print("wrote", bpy.context.scene.render.filepath)


if __name__ == "__main__":
    main()
