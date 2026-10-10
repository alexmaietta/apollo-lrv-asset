"""Render review views of a built LRV .blend.

Run: blender -b build/lrv.blend -P scripts/render.py -- [--tag pass1] [--views 34,side] [--samples 64]
       [--res 1280x720] [--studio] [--cam x,y,z --target x,y,z --lens 50]   (custom view "custom")
"""
import math
import os
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import materials as M  # noqa: E402

# name: (camera location, look-at target, focal length mm)
VIEWS = {
    "34": ((4.6, 3.6, 2.3), (0, 0, 0.55), 40),
    "34rear": ((-4.4, -3.8, 2.6), (0, 0, 0.55), 40),
    "side": ((0, 7.5, 0.75), (0, 0, 0.6), 50),
    "front": ((7.0, 0, 0.9), (0, 0, 0.55), 50),
    "top": ((0, 0, 9.0), (0, 0, 0.4), 45),
    "wheel": ((2.3, 1.95, 0.75), (1.143, 0.93, 0.42), 50),
    "susp": ((1.2, 2.2, 0.15), (1.143, 0.6, 0.42), 30),
}


def args():
    a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    get = lambda k, d: a[a.index(k) + 1] if k in a else d  # noqa: E731
    return {
        "tag": get("--tag", "dev"),
        "views": get("--views", "34,side,front,top,wheel").split(","),
        "samples": int(get("--samples", "64")),
        "res": tuple(int(x) for x in get("--res", "1280x720").split("x")),
        "studio": "--studio" in a,
        "museum": "--museum" in a,
        "out": get("--outdir", "renders"),
        "cam": get("--cam", None),
        "target": get("--target", "0,0,0.5"),
        "lens": float(get("--lens", "50")),
    }


def setup_render(o):
    sc = bpy.context.scene
    sc.render.engine = "CYCLES"
    prefs = bpy.context.preferences.addons["cycles"].preferences
    prefs.compute_device_type = "OPTIX"
    prefs.get_devices()
    for d in prefs.devices:
        d.use = d.type == "OPTIX"
    sc.cycles.device = "GPU"
    sc.cycles.samples = o["samples"]
    sc.cycles.use_denoising = True
    sc.render.resolution_x, sc.render.resolution_y = o["res"]
    sc.render.image_settings.file_format = "PNG"
    sc.view_settings.view_transform = "AgX"
    sc.view_settings.look = "AgX - Medium High Contrast"


def setup_museum():
    """Smithsonian-style: black void, big soft key from front-above, fills, dark floor (NASM QTU photos)."""
    w = bpy.data.worlds.new("world")
    bpy.context.scene.world = w
    try:
        w.use_nodes = True
    except AttributeError:
        pass
    w.node_tree.nodes["Background"].inputs["Color"].default_value = (0.0, 0.0, 0.0, 1)
    for name, loc, energy, size in (("key", (5, -5, 7), 1100, 6.0), ("fill", (-5, -6, 3), 350, 6.0),
                                    ("rim", (-6, 5, 5), 500, 4.0), ("front", (7, 2, 2), 300, 4.0)):
        L = bpy.data.objects.new(name, bpy.data.lights.new(name, "AREA"))
        L.data.energy, L.data.size = energy, size
        L.location = loc
        L.rotation_euler = (Vector((0, 0, 0.5)) - Vector(loc)).to_track_quat("-Z", "Y").to_euler()
        bpy.context.scene.collection.objects.link(L)
    # no floor: the NASM photos are cut out against black


def setup_world(studio):
    w = bpy.data.worlds.new("world")
    bpy.context.scene.world = w
    try:
        w.use_nodes = True
    except AttributeError:
        pass
    bg = w.node_tree.nodes["Background"]
    if studio:
        bg.inputs["Color"].default_value = (0.35, 0.37, 0.40, 1)
        bg.inputs["Strength"].default_value = 0.8
    else:
        # black sky; small ambient term standing in for regolith bounce light
        bg.inputs["Color"].default_value = (0.02, 0.02, 0.022, 1)
        bg.inputs["Strength"].default_value = 1.0

    sun = bpy.data.objects.new("sun", bpy.data.lights.new("sun", "SUN"))
    sun.data.energy = 6.0 if not studio else 3.0
    sun.data.angle = math.radians(0.53)
    sun.rotation_euler = (math.radians(65), 0, math.radians(-35))  # ~25 deg elevation
    bpy.context.scene.collection.objects.link(sun)

    if not studio:
        ground = bpy.data.meshes.new("ground")
        s = 40
        ground.from_pydata([(-s, -s, 0), (s, -s, 0), (s, s, 0), (-s, s, 0)], [], [(0, 1, 2, 3)])
        ground.materials.append(M.get("regolith"))
        bpy.context.scene.collection.objects.link(bpy.data.objects.new("ground", ground))
        # fill from the sunlit ground on the shadow side
        fill = bpy.data.objects.new("fill", bpy.data.lights.new("fill", "SUN"))
        fill.data.energy = 0.6
        fill.data.use_shadow = False
        fill.rotation_euler = (math.radians(-60), 0, math.radians(150))
        bpy.context.scene.collection.objects.link(fill)


def camera(name, loc, target, lens):
    cam = bpy.data.objects.get("cam") or bpy.data.objects.new("cam", bpy.data.cameras.new("cam"))
    if cam.name not in bpy.context.scene.collection.objects:
        bpy.context.scene.collection.objects.link(cam)
    cam.location = loc
    d = Vector(target) - Vector(loc)
    cam.rotation_euler = d.to_track_quat("-Z", "Y").to_euler()
    cam.data.lens = lens
    cam.data.clip_start = 0.01
    bpy.context.scene.camera = cam
    return cam


def main():
    o = args()
    setup_render(o)
    if o["museum"]:
        setup_museum()
    else:
        setup_world(o["studio"])
    os.makedirs(o["out"], exist_ok=True)
    if o["cam"]:
        VIEWS["custom"] = (tuple(map(float, o["cam"].split(","))), tuple(map(float, o["target"].split(","))), o["lens"])
        o["views"] = ["custom"]
    for v in o["views"]:
        loc, tgt, lens = VIEWS[v]
        camera(v, loc, tgt, lens)
        path = os.path.abspath(os.path.join(o["out"], f"{o['tag']}_{v}.png"))
        bpy.context.scene.render.filepath = path
        bpy.ops.render.render(write_still=True)
        print("wrote", path)


if __name__ == "__main__":
    main()
