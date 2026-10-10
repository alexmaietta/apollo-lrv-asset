"""Self-contained Blender asset: pack every texture into the built .blend and save it to assets/.

Run: blender -b build/lrv.blend -P scripts/export_blend.py -- [--out assets/blender/lrv.blend]

All variant parts stay in the file; each keeps its "variant" custom property ("Set:option,...") and only the
default selections (params.VARIANT_DEFAULTS) are visible, so other configurations can be shown by toggling them.
"""
import os
import sys

import bpy

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P  # noqa: E402


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = os.path.abspath(argv[argv.index("--out") + 1] if "--out" in argv else "assets/blender/lrv.blend")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    hidden = 0
    for ob in bpy.data.objects:
        if "variant" in ob:
            set_name, opts = ob["variant"].split(":", 1)
            default = P.VARIANT_DEFAULTS.get(set_name)
            off = default is not None and default not in [o.strip() for o in opts.split(",")]
            ob.hide_viewport = ob.hide_render = off
            ob.hide_set(off)
            hidden += off
    bpy.ops.file.pack_all()
    bpy.context.preferences.filepaths.save_version = 0   # no lrv.blend1 backup next to the asset
    bpy.ops.wm.save_as_mainfile(filepath=out, compress=True, copy=True)
    print(f"saved {out}: {len(bpy.data.objects)} objects, {len(bpy.data.images)} packed images, "
          f"{hidden} non-default variant parts hidden")


if __name__ == "__main__":
    main()
