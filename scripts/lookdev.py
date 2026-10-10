"""Look development: weathering-aware materials and a physically placed dust mask.

Run inside Blender on a built model (also imported by bake.py):
  blender -b build/lrv.blend -P scripts/lookdev.py -- [--dust 1.0] [--out build/lrv_look.blend]

1. Every mesh gets a per-vertex "dust" attribute (rest pose, world space):
     - wheel spray: surfaces near a wheel, facing it, get coated (fender undersides most)
     - settling: up-facing surfaces collect a light layer
     - low surfaces near the ground take spray from all four wheels
     - occlusion: crevices and inner corners collect grime (per pixel, AO node in the shader)
   The tires themselves are packed with regolith (the mesh fills in).
2. Every material gets a weathering stage between its inputs and the Principled BSDF:
   per-material surface detail (brushed metal, paint orange peel, foil crinkle, cloth weave),
   edge wear, and a dust layer driven by the vertex attribute and a scene-wide amount.
"""
import math
import os
import sys

import bpy
from mathutils import Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P  # noqa: E402

REGOLITH = (0.40, 0.39, 0.37)  # Taurus-Littrow fines, light neutral grey under direct sun
DUST_ATTR = "dust"


# ---------------------------------------------------------------- dust mask

def dust_value(p, n, wheel_centres):
    """0..1 dust coverage for a surface point p with normal n (world space)."""
    up = max(n.z, 0.0)
    settle = 0.16 * up ** 2
    spray = 0.0
    for c in wheel_centres:
        d = p - c
        dist = d.length
        if dist < 0.9:
            facing = max(-n.dot(d.normalized()), 0.0)        # surface looks toward the wheel
            # rooster tail: the tire throws fines forward/back and low, so surfaces below the
            # axle line and those facing the tread get most of it
            below = max(0.0, min(1.0, (c.z + 0.1 - p.z) / 0.35))
            spray = max(spray, (1 - dist / 0.9) ** 1.4 * (0.08 + 0.6 * facing + 0.35 * below))
    low = max(0.0, 1.0 - p.z / 0.5) * 0.35                    # ground-level spray band
    return min(1.0, settle + spray + low)


def compute_dust():
    """Write the 'dust' point attribute on every mesh (rest pose): the smooth physical fields.

    Occlusion (crevice grime) is added per pixel in the shader with Cycles' AO node.
    """
    wheels = [Vector(P.wheel_center(c)) for c in P.CORNERS]
    n_done = 0
    for ob in bpy.context.scene.objects:
        if ob.type != "MESH" or ob.name == "ground":
            continue
        me = ob.data
        mw = ob.matrix_world
        nm = mw.to_3x3().inverted().transposed()
        attr = me.attributes.get(DUST_ATTR) or me.attributes.new(DUST_ATTR, "FLOAT", "POINT")
        is_tire = "tire_mesh" in ob.name
        vals = []
        for v in me.vertices:
            n = (nm @ v.normal).normalized()
            if is_tire:
                vals.append(0.85 + 0.15 * max(-n.z, 0.0))
            else:
                vals.append(dust_value(mw @ v.co, n, wheels))
        attr.data.foreach_set("value", vals)
        n_done += len(vals)
    return n_done


# ---------------------------------------------------------------- weathering shader

RECIPES = [  # (name prefix/substring, recipe)
    ("wire_mesh", "wire"), ("titanium", "metal_raw"), ("aluminium_spun", "metal_spun"),
    ("aluminium", "metal_brushed"), ("steel", "metal_raw"), ("chrome", "metal_polished"),
    ("drive_housing", "metal_cast"), ("floor_panel", "metal_brushed"),
    ("fender", "fibreglass"), ("paint", "paint"), ("decal", "decal"),
    ("kapton", "foil"), ("gold", "foil"), ("foil", "foil"), ("beta", "cloth"), ("cloth", "cloth"),
    ("webbing", "cloth"), ("blanket", "cloth"), ("damper", "paint"), ("console", "paint"),
]


def recipe_for(name):
    for key, r in RECIPES:
        if key in name:
            return r
    return "generic"


class _G:
    """Tiny node-graph helper."""

    def __init__(self, nt):
        self.nt, self.N = nt, nt.nodes

    def node(self, kind, **props):
        n = self.N.new(kind)
        for k, v in props.items():
            setattr(n, k, v)
        return n

    def math(self, op, a, b=None, clamp=False):
        n = self.node("ShaderNodeMath", operation=op, use_clamp=clamp)
        self.feed(n.inputs[0], a)
        if b is not None:
            self.feed(n.inputs[1], b)
        return n.outputs[0]

    def feed(self, sock, src):
        if isinstance(src, (int, float)):
            sock.default_value = src
        else:
            self.nt.links.new(src, sock)

    def mix_col(self, fac, a, b):
        n = self.node("ShaderNodeMix", data_type="RGBA", blend_type="MIX")
        self.feed(n.inputs["Factor"], fac)
        self._col(n.inputs[6], a)
        self._col(n.inputs[7], b)
        return n.outputs[2]

    def mix_f(self, fac, a, b):
        n = self.node("ShaderNodeMix", data_type="FLOAT")
        self.feed(n.inputs["Factor"], fac)
        self.feed(n.inputs[2], a)
        self.feed(n.inputs[3], b)
        return n.outputs[0]

    def _col(self, sock, c):
        if isinstance(c, tuple):
            sock.default_value = (*c[:3], 1.0)
        else:
            self.nt.links.new(c, sock)

    def noise(self, scale, detail=6.0, rough=0.55, vec=None, dim="3D"):
        n = self.node("ShaderNodeTexNoise", noise_dimensions=dim)
        n.inputs["Scale"].default_value = scale
        n.inputs["Detail"].default_value = detail
        n.inputs["Roughness"].default_value = rough
        if vec is not None:
            self.nt.links.new(vec, n.inputs["Vector"])
        return n.outputs["Fac"]

    def ramp(self, src, lo, hi):
        n = self.node("ShaderNodeMapRange", clamp=True)
        self.feed(n.inputs["Value"], src)
        n.inputs["From Min"].default_value = lo
        n.inputs["From Max"].default_value = hi
        return n.outputs["Result"]


def _input_src(bsdf, name):
    """Current source of a Principled input: a linked socket or its constant."""
    s = bsdf.inputs[name]
    if s.is_linked:
        return s.links[0].from_socket
    v = s.default_value
    return tuple(v) if hasattr(v, "__len__") else float(v)


def weather(mat, dust_amount=1.0, wear_amount=1.0):
    """Insert surface detail + edge wear + dust between a material's inputs and its BSDF."""
    if mat.get("weathered"):
        return
    nt = mat.node_tree
    bsdf = next((n for n in nt.nodes if n.type == "BSDF_PRINCIPLED"), None)
    if bsdf is None:
        return
    g = _G(nt)
    r = recipe_for(mat.name)
    col, rough, metal = _input_src(bsdf, "Base Color"), _input_src(bsdf, "Roughness"), _input_src(bsdf, "Metallic")
    tc = g.node("ShaderNodeTexCoord")
    obj = tc.outputs["Object"]
    bump_h = None

    # --- per-material surface character
    if r == "metal_brushed":
        sv = g.node("ShaderNodeMapping", vector_type="POINT")
        sv.inputs["Scale"].default_value = (1.0, 60.0, 60.0)  # streaks along X (extrusion/brush direction)
        g.nt.links.new(obj, sv.inputs["Vector"])
        streak = g.noise(40.0, 2.0, 0.5, sv.outputs[0])
        rough = g.math("ADD", g.math("MULTIPLY", streak, 0.12), g.math("SUBTRACT", rough, 0.06))
        col = g.mix_col(g.math("MULTIPLY", streak, 0.25), col, (0.62, 0.62, 0.63))
        bump_h = g.math("MULTIPLY", streak, 0.15)
    elif r == "metal_spun":
        rough = g.math("ADD", g.math("MULTIPLY", g.noise(300.0, 2.0), 0.08), rough)
    elif r in ("metal_raw", "metal_cast"):
        n = g.noise(25.0 if r == "metal_raw" else 90.0, 8.0, 0.6)
        rough = g.math("ADD", g.math("MULTIPLY", n, 0.18), g.math("SUBTRACT", rough, 0.06))
        bump_h = g.math("MULTIPLY", n, 0.3 if r == "metal_cast" else 0.08)
    elif r == "fibreglass":
        peel = g.noise(220.0, 3.0, 0.5)
        blotch = g.noise(4.0, 4.0, 0.6)
        col = g.mix_col(g.math("MULTIPLY", blotch, 0.35), col, (0.36, 0.10, 0.05))
        rough = g.math("ADD", rough, g.math("MULTIPLY", peel, 0.1))
        bump_h = g.math("MULTIPLY", peel, 0.25)
    elif r == "foil":
        cr = g.node("ShaderNodeTexVoronoi", feature="F2")
        cr.inputs["Scale"].default_value = 30.0
        g.nt.links.new(obj, cr.inputs["Vector"])
        bump_h = g.math("MULTIPLY", cr.outputs["Distance"], 1.0)
        rough = g.math("ADD", rough, g.math("MULTIPLY", g.noise(60.0), 0.15))
    elif r == "cloth":
        w = g.node("ShaderNodeTexWave", wave_type="BANDS", bands_direction="X")
        w.inputs["Scale"].default_value = 400.0
        w2 = g.node("ShaderNodeTexWave", wave_type="BANDS", bands_direction="Y")
        w2.inputs["Scale"].default_value = 400.0
        bump_h = g.math("MULTIPLY", g.math("MAXIMUM", w.outputs["Fac"], w2.outputs["Fac"]), 0.4)
        rough = g.math("MAXIMUM", rough, 0.75)
        col = g.mix_col(g.math("MULTIPLY", g.noise(8.0), 0.15), col, (0.55, 0.52, 0.48))
    elif r == "paint":
        bump_h = g.math("MULTIPLY", g.noise(250.0, 2.0), 0.12)
        rough = g.math("ADD", rough, g.math("MULTIPLY", g.noise(6.0), 0.1))

    # --- edge wear (pointiness = convexity; Cycles only)
    geo = g.node("ShaderNodeNewGeometry")
    edge = g.ramp(geo.outputs["Pointiness"], 0.52, 0.62)
    wear = g.math("MULTIPLY", g.math("MULTIPLY", edge, g.ramp(g.noise(30.0, 6.0), 0.45, 0.7)), wear_amount)
    if r in ("fibreglass", "paint"):
        under = (0.72, 0.68, 0.58) if r == "fibreglass" else (0.75, 0.75, 0.74)
        col = g.mix_col(g.math("MULTIPLY", wear, 0.8), col, under)
        if r == "paint" or r == "fibreglass":
            rough = g.mix_f(wear, rough, 0.6)
    elif r.startswith("metal"):
        rough = g.mix_f(g.math("MULTIPLY", wear, 0.7), rough, 0.15)  # burnished edges

    # --- dust: vertex mask x breakup noise x scene amount
    attr = g.node("ShaderNodeAttribute", attribute_type="GEOMETRY", attribute_name=DUST_ATTR)
    ao = g.node("ShaderNodeAmbientOcclusion", samples=16)
    ao.inputs["Distance"].default_value = 0.12
    occl = g.math("SUBTRACT", 1.0, ao.outputs["AO"])
    field = g.math("ADD", attr.outputs["Fac"], g.math("MULTIPLY", occl, 0.5), clamp=True)
    # powder: a crisp patch boundary (thresholded fractal noise compared against the mask),
    # plus fine grain so thin layers read as a speckled film rather than a tint
    brk = g.noise(9.0, 12.0, 0.65)
    grain = g.noise(900.0, 2.0)
    jitter = g.math("ADD", g.math("MULTIPLY", g.math("SUBTRACT", brk, 0.5), 0.7),
                    g.math("MULTIPLY", g.math("SUBTRACT", grain, 0.5), 0.12))
    cover = g.math("ADD", field, jitter)
    dust = g.math("MULTIPLY", g.ramp(cover, 0.46, 0.56), dust_amount, clamp=True)
    film = g.math("MULTIPLY", g.ramp(g.math("MULTIPLY", field, grain), 0.18, 0.5), 0.35)
    dust = g.math("MAXIMUM", dust, g.math("MULTIPLY", film, dust_amount), clamp=True)
    dcol = g.mix_col(g.math("MULTIPLY", grain, 0.6), REGOLITH, (0.30, 0.295, 0.28))
    if r == "wire":
        dust = g.math("MULTIPLY", dust, 0.85)
    col = g.mix_col(dust, col, dcol)
    rough = g.mix_f(dust, rough, 0.96)
    metal = g.mix_f(dust, metal, 0.0)
    dust_bump = g.math("MULTIPLY", g.math("MULTIPLY", grain, dust), 0.6)
    bump_h = dust_bump if bump_h is None else g.math("ADD", bump_h, dust_bump)

    g.feed(bsdf.inputs["Base Color"], col) if not isinstance(col, tuple) else None
    if isinstance(rough, float):
        bsdf.inputs["Roughness"].default_value = rough
    else:
        g.nt.links.new(rough, bsdf.inputs["Roughness"])
    if isinstance(metal, float):
        bsdf.inputs["Metallic"].default_value = metal
    else:
        g.nt.links.new(metal, bsdf.inputs["Metallic"])
    if bump_h is not None and not bsdf.inputs["Normal"].is_linked:
        bump = g.node("ShaderNodeBump")
        bump.inputs["Strength"].default_value = 0.35
        bump.inputs["Distance"].default_value = 0.0004
        g.nt.links.new(bump_h, bump.inputs["Height"])
        g.nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    mat["weathered"] = r


def apply(dust_amount=1.0, wear_amount=1.0, compute=True):
    if compute:
        n = compute_dust()
        print(f"dust mask: {n} vertices")
    for m in bpy.data.materials:
        if m.node_tree and m.name not in ("regolith",):
            weather(m, dust_amount, wear_amount)


def main():
    a = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    get = lambda k, d: a[a.index(k) + 1] if k in a else d  # noqa: E731
    apply(float(get("--dust", "0.0")), float(get("--wear", "1.0")))
    out = get("--out", "build/lrv_look.blend")
    bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(out))
    print("saved", out)


if __name__ == "__main__":
    main()
