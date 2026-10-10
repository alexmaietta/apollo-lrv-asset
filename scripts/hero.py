"""Cycles hero renders for the README gallery.

Run: blender -b build/lrv.blend -P scripts/hero.py -- [--only side,wheel] [--samples 256] [--res 1600x1200]
       [--out docs/images]

Shots framed like the Smithsonian NASM walkaround / close-up photographs (museum lighting).
"""
import os
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render as R  # noqa: E402

# name: (lighting, camera, target, lens mm, reference photo or None[, variant overrides])
SHOTS = {
    "front_three_quarter": ("museum", (4.2, -4.6, 2.2), (0.15, 0.0, 0.85), 56, "NASM-NASM2020-00046.jpg"),
    "side": ("museum", (0.05, -7.8, 1.15), (0.05, 0.0, 0.95), 72, "NASM-NASM2020-00047.jpg"),
    "rear_three_quarter": ("museum", (-4.4, -3.9, 2.3), (0.0, 0.0, 0.9), 56, "NASM-NASM2020-00048.jpg"),
    "rear_left": ("museum", (-4.4, 3.8, 2.0), (-0.2, 0.0, 0.9), 56, "NASM-NASM2020-00049.jpg"),
    "wheel_fender": ("museum", (2.25, 1.75, 0.85), (1.25, 0.9, 0.55), 40, "NASM-NASM2020-00054.jpg"),  # left front (maroon)
    "console_t_handle": ("museum", (-0.48, -0.62, 1.15), (0.16, 0.0, 0.9), 40, "NASM-NASM2020-00055.jpg"),
    "seats": ("museum", (0.30, -1.30, 1.18), (-0.45, -0.08, 0.74), 30, "NASM-NASM2020-00056.jpg"),
    "lcru": ("museum", (2.8, -0.8, 0.85), (1.45, 0.0, 0.6), 45, "NASM-NASM2020-00053.jpg"),
    "hga": ("museum", (3.5, 2.9, 2.4), (1.5, 1.0, 1.55), 42, "NASM-NASM2020-00051.jpg"),
    # Apollo 17 surface configuration with the EVA-2 map-and-clamp right-rear fender repair (as17-137-20979)
    # README banner: Apollo 17 surface configuration, low front three-quarter, wide frame
    "hero": ("museum", (5.6, -3.6, 1.05), (0.15, 0.1, 1.0), 36, None,
             {"Mission": "a17", "LCRU": "radiator_open", "_res": (2400, 1200)}),
    # flight configurations, compared with the Hasselblad frames
    "a17_surface": ("museum", (2.0, -5.4, 1.15), (0.1, 0.0, 0.8), 45, "as17-147-22526.jpg",
                    {"Mission": "a17", "LCRU": "radiator_open"}),
    "a15": ("museum", (-0.9, 5.4, 1.3), (0.0, 0.0, 0.85), 45, "as15-86-11603.jpg",
            {"Mission": "a15", "LCRU": "radiator_open"}),
    "a17_repair": ("museum", (-1.95, -1.7, 0.92), (-1.22, -0.78, 0.55), 28, "as17-137-20979.jpg",
                   {"Mission": "a17_repair", "LCRU": "radiator_open"}),
}


HERO_VARIANTS = {"Mission": "nasm", "LCRU": "closed"}   # the gallery is compared against the Smithsonian vehicle


def show_default_variants(overrides=None):
    """The .blend holds every variant's parts; render only those in the selected variants."""
    import params as P
    for ob in bpy.data.objects:
        if "variant" in ob:
            set_name, opts = ob["variant"].split(":", 1)
            default = {**P.VARIANT_DEFAULTS, **HERO_VARIANTS, **(overrides or {})}.get(set_name)
            ob.hide_render = default is not None and default not in [o.strip() for o in opts.split(",")]


def main():
    a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    get = lambda k, d: a[a.index(k) + 1] if k in a else d  # noqa: E731
    only = set(filter(None, get("--only", "").split(",")))
    out = os.path.abspath(get("--out", "docs/images"))
    os.makedirs(out, exist_ok=True)
    o = R.args()
    o["samples"] = int(get("--samples", "256"))
    o["res"] = tuple(int(x) for x in get("--res", "1600x1200").split("x"))
    R.setup_render(o)
    show_default_variants()
    sc = bpy.context.scene
    sc.cycles.use_adaptive_sampling = True
    sc.render.film_transparent = False
    lighting = None
    for name, (light, cam, tgt, lens, _ref, *extra) in SHOTS.items():
        if only and name not in only:
            continue
        opts = dict(extra[0]) if extra else {}
        sc.render.resolution_x, sc.render.resolution_y = opts.pop("_res", o["res"])
        show_default_variants(opts)
        if light != lighting:
            for ob in [o for o in bpy.data.objects if o.type == "LIGHT" or o.name == "ground"]:
                bpy.data.objects.remove(ob)
            R.setup_museum()
            lighting = light
        R.camera(name, Vector(cam), Vector(tgt), lens)
        sc.render.filepath = os.path.join(out, f"{name}.png")
        bpy.ops.render.render(write_still=True)
        print("wrote", sc.render.filepath)


if __name__ == "__main__":
    main()
