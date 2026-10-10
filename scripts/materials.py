"""Physically based materials for the LRV (Cycles / EEVEE node trees)."""
import math
import os

import bpy

import params as P

TEX_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "textures", "gen")


def texture_path(name):
    path = os.path.abspath(os.path.join(TEX_DIR, name))
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} missing: run .venv/bin/python scripts/textures.py first")
    return path

_cache = {}
_tiles = None


def tile_size(set_name):
    global _tiles
    if _tiles is None:
        import json
        with open(texture_path("tiles.json")) as f:
            _tiles = json.load(f)
    return _tiles[set_name]


def _image(nt, set_name, kind, vec, colour=False):
    path = texture_path(f"{set_name}/{os.path.basename(set_name)}_{kind}.png")
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = bpy.data.images.load(path, check_existing=True)
    tex.image.colorspace_settings.name = "sRGB" if colour else "Non-Color"
    tex.interpolation = "Smart"
    nt.links.new(vec, tex.inputs["Vector"])
    return tex


def _pbr(name, set_name, metallic, normal_strength=1.0, alpha=False):
    """Material driven by a tileable texture set; UVs are in metres (see geo.ensure_uvs)."""
    m, bsdf = _principled(name, (0.5, 0.5, 0.5), metallic, 0.5)
    nt = m.node_tree
    uv = nt.nodes.new("ShaderNodeUVMap")
    uv.uv_map = "UVMap"
    mp = nt.nodes.new("ShaderNodeMapping")
    s = 1.0 / tile_size(set_name)
    mp.inputs["Scale"].default_value = (s, s, 1.0)
    nt.links.new(uv.outputs["UV"], mp.inputs["Vector"])
    vec = mp.outputs["Vector"]
    nt.links.new(_image(nt, set_name, "basecolor", vec, True).outputs["Color"], bsdf.inputs["Base Color"])
    nt.links.new(_image(nt, set_name, "roughness", vec).outputs["Color"], bsdf.inputs["Roughness"])
    if os.path.exists(os.path.join(TEX_DIR, set_name, f"{os.path.basename(set_name)}_metallic.png")):
        nt.links.new(_image(nt, set_name, "metallic", vec).outputs["Color"], bsdf.inputs["Metallic"])
    nm = nt.nodes.new("ShaderNodeNormalMap")
    nm.uv_map = "UVMap"
    nm.inputs["Strength"].default_value = normal_strength
    nt.links.new(_image(nt, set_name, "normal", vec).outputs["Color"], nm.inputs["Color"])
    nt.links.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
    if alpha:
        nt.links.new(_image(nt, set_name, "opacity", vec).outputs["Color"], bsdf.inputs["Alpha"])
        try:
            m.surface_render_method = "DITHERED"
        except AttributeError:
            pass
        m.use_backface_culling = False
    return m


PBR = {  # material -> (texture set, metallic, normal strength, alpha)
    "wire_mesh": ("wire_weave", 0.55, 1.0, True),   # weathered zinc coat: not a mirror
    "fender": ("fiberglass_fender", 0.0, 0.6, False),
    "fender_ext": ("fiberglass_fender_ext", 0.0, 0.6, False),
    "aluminium": ("aluminium", 1.0, 0.4, False),
    "aluminium_spun": ("aluminium_spun", 1.0, 0.3, False),
    "aluminium_disc": ("aluminium_disc", 1.0, 0.3, False),
    "steel": ("steel", 1.0, 0.4, False),
    "titanium": ("titanium", 1.0, 0.5, False),
    "drive_housing": ("drive_housing", 1.0, 0.5, False),
    "floor_panel": ("floor_panel", 1.0, 0.4, False),
    "white_paint": ("white_paint", 0.0, 0.3, False),
    "cream_paint": ("cream_paint", 0.0, 0.3, False),
    "black_paint": ("black_paint", 0.0, 0.3, False),
    "yellow_paint": ("yellow_paint", 0.0, 0.3, False),
}


def _principled(name, color, metallic=0.0, rough=0.5, **inputs):
    m = bpy.data.materials.new(name)
    try:
        m.use_nodes = True
    except AttributeError:
        pass
    bsdf = m.node_tree.nodes["Principled BSDF"]
    bsdf.inputs["Base Color"].default_value = (*color, 1.0)
    bsdf.inputs["Metallic"].default_value = metallic
    bsdf.inputs["Roughness"].default_value = rough
    for k, v in inputs.items():
        bsdf.inputs[k].default_value = v
    m.diffuse_color = (*color, 1.0)
    return m, bsdf


def _noise_roughness(m, bsdf, base, amount, scale=40.0):
    nt = m.node_tree
    tex = nt.nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = scale
    ramp = nt.nodes.new("ShaderNodeMapRange")
    ramp.inputs["To Min"].default_value = base - amount
    ramp.inputs["To Max"].default_value = base + amount
    nt.links.new(tex.outputs["Fac"], ramp.inputs["Value"])
    nt.links.new(ramp.outputs["Result"], bsdf.inputs["Roughness"])


def _wire_mesh():
    """Zinc-coated piano wire woven on the bias: two diagonal wire families as alpha."""
    m, bsdf = _principled("wire_mesh", (0.55, 0.55, 0.53), 1.0, 0.52)
    nt = m.node_tree
    N = nt.nodes
    L = nt.links.new
    uv = N.new("ShaderNodeUVMap")
    sep = N.new("ShaderNodeSeparateXYZ")
    L(uv.outputs["UV"], sep.inputs["Vector"])
    frac_w = P.WIRE_DIAMETER * 1.6 / P.WIRE_PITCH  # a little fat so it reads at render scale

    def family(cu, cv):
        mu = N.new("ShaderNodeMath"); mu.operation = "MULTIPLY"; mu.inputs[1].default_value = cu
        L(sep.outputs["X"], mu.inputs[0])
        s = N.new("ShaderNodeMath"); s.operation = "MULTIPLY_ADD"; s.inputs[1].default_value = cv
        L(sep.outputs["Y"], s.inputs[0]); L(mu.outputs[0], s.inputs[2])
        d = N.new("ShaderNodeMath"); d.operation = "DIVIDE"; d.inputs[1].default_value = P.WIRE_PITCH
        L(s.outputs[0], d.inputs[0])
        fr = N.new("ShaderNodeMath"); fr.operation = "FRACT"
        L(d.outputs[0], fr.inputs[0])
        sub = N.new("ShaderNodeMath"); sub.operation = "SUBTRACT"; sub.inputs[1].default_value = 0.5
        L(fr.outputs[0], sub.inputs[0])
        ab = N.new("ShaderNodeMath"); ab.operation = "ABSOLUTE"
        L(sub.outputs[0], ab.inputs[0])
        gt = N.new("ShaderNodeMath"); gt.operation = "GREATER_THAN"; gt.inputs[1].default_value = 0.5 - frac_w / 2
        L(ab.outputs[0], gt.inputs[0])
        return gt

    mx = N.new("ShaderNodeMath"); mx.operation = "MAXIMUM"
    b = P.WIRE_BIAS
    L(family(math.cos(b), math.sin(b)).outputs[0], mx.inputs[0])
    L(family(-math.sin(b), math.cos(b)).outputs[0], mx.inputs[1])
    L(mx.outputs[0], bsdf.inputs["Alpha"])
    try:
        m.surface_render_method = "DITHERED"
    except AttributeError:
        pass
    m.use_backface_culling = False
    return m


def _regolith():
    m, bsdf = _principled("regolith", (0.16, 0.155, 0.15), 0.0, 0.97)
    nt = m.node_tree
    tex = nt.nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = 3.0
    tex.inputs["Detail"].default_value = 12.0
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.4
    nt.links.new(tex.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    return m


def _decal(name, image, rough=0.4):
    """Printed decal: image base colour on a thin film, slight sheen."""
    m, bsdf = _principled(name, (1, 1, 1), 0.0, rough)
    nt = m.node_tree
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = bpy.data.images.load(texture_path(image), check_existing=True)
    tex.interpolation = "Cubic"
    tex.extension = "CLIP"
    nt.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    return m


def _build(name):
    if name == "flag_decal":
        return _decal(name, "flag_us.png")
    if name in PBR:
        return _pbr(name, *PBR[name])
    if name == "wire_mesh":
        return _wire_mesh()
    if name == "regolith":
        return _regolith()
    spec = {
        # 2219 aluminium tube, bare/lightly anodised
        "aluminium": ((0.80, 0.80, 0.80), 1.0, 0.32),
        "aluminium_spun": ((0.85, 0.85, 0.84), 1.0, 0.22),
        "aluminium_disc": ((0.66, 0.66, 0.64), 1.0, 0.42),
        "cream_paint": ((0.86, 0.83, 0.72), 0.0, 0.5),
        "titanium": ((0.60, 0.58, 0.55), 1.0, 0.30),
        "steel": ((0.55, 0.55, 0.56), 1.0, 0.35),
        "chrome": ((0.9, 0.9, 0.9), 1.0, 0.08),
        "drive_housing": ((0.70, 0.70, 0.70), 1.0, 0.45),
        "damper": ((0.30, 0.30, 0.31), 0.6, 0.45),
        "floor_panel": ((0.72, 0.72, 0.71), 1.0, 0.45),
        "white_paint": ((0.82, 0.82, 0.80), 0.0, 0.45),
        "black_paint": ((0.02, 0.02, 0.02), 0.0, 0.5),
        "yellow_paint": ((0.85, 0.65, 0.05), 0.0, 0.45),
        "brass": ((0.62, 0.46, 0.22), 1.0, 0.35),       # rivet heads, NASM2020-00054
        "flap_grey": ((0.40, 0.44, 0.48), 0.0, 0.5),    # front fender flaps, Smithsonian vehicle (00048/00049)
        # fibreglass fenders: LRV-3 red-copper (as17-147-22526), extensions a shade darker
        "fender": (P.FENDER_COLOR[P.MISSION][0], 0.35, 0.35),
        "fender_ext": (P.FENDER_COLOR[P.MISSION][1], 0.35, 0.38),
    }[name]
    m, bsdf = _principled(name, *spec)
    if name in ("aluminium", "drive_housing", "floor_panel"):
        _noise_roughness(m, bsdf, spec[2], 0.08)
    return m


AUTO_RULES = [  # (substring, recipe) for materials that arrive as plain colours
    ("tge_", "dusty_cloth"), ("webbing", "webbing"), ("belt", "webbing"), ("harness", "webbing"), ("velcro", "cloth"),
    ("beta", "beta"), ("seat_cover", "cloth"), ("bag", "cloth"), ("tape", "cloth"), ("cover", "cloth"),
    ("kapton", "foil"), ("foil", "foil"), ("hga_mesh", None), ("mirror", None), ("decal", None),
    ("console_panel", None), ("cw_flag", None), ("fiberglass", "fiberglass"), ("anodized", "plastic"),
    ("knob", "plastic"), ("toggle", "plastic"), ("grip", "plastic"), ("bezel", "plastic"), ("cable", "plastic"),
    ("teflon", "plastic"), ("foam", "cloth"), ("housing", "paint"), ("battery", "cast"), ("panel", "paint"),
    ("strap", "brushed"), ("frame", "paint"), ("honeycomb", "cast"), ("armrest", "plastic"),
    ("handhold", "plastic"), ("bellows", "plastic"), ("dial", "paint"), ("sunshade", "paint"),
    ("collar", "plastic"), ("edge_tape", "cloth"),
]


def _auto_recipe(name):
    for key, r in AUTO_RULES:
        if key in name:
            return r
    return "paint"


def _has_images(m):
    return any(n.type == "TEX_IMAGE" for n in m.node_tree.nodes)


def upgrade_all():
    """Give every plain-colour material a generated, coloured tileable texture set (cached on disk)."""
    import textures as T
    auto_dir = os.path.join(TEX_DIR, "auto")
    done = 0
    for m in list(bpy.data.materials):
        if not m.node_tree or m.name in PBR or m.name == "regolith" or _has_images(m):
            continue
        recipe = _auto_recipe(m.name)
        if recipe is None:
            continue
        bsdf = next((n for n in m.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
        if bsdf is None or bsdf.inputs["Base Color"].is_linked:
            continue
        col = tuple(bsdf.inputs["Base Color"].default_value)[:3]
        rough = float(bsdf.inputs["Roughness"].default_value)
        metal = float(bsdf.inputs["Metallic"].default_value)
        key = f"{m.name}_{recipe}_{col[0]:.3f}_{col[1]:.3f}_{col[2]:.3f}_{rough:.2f}_v{T.AUTO_VERSION.get(recipe, 1)}"
        set_dir = os.path.join(auto_dir, m.name)
        stamp = os.path.join(set_dir, "spec.txt")
        if not (os.path.exists(stamp) and open(stamp).read() == key):
            T.make_auto_set(auto_dir, m.name, recipe, col, rough)
            with open(stamp, "w") as f:
                f.write(key)
        tile = float(open(os.path.join(set_dir, "tile.txt")).read())
        _rewire(m, f"auto/{m.name}", tile, metal)
        done += 1
    return done


def _rewire(m, set_name, tile, metallic):
    """Replace a plain material's inputs with the texture set (keeps alpha / other inputs)."""
    nt = m.node_tree
    bsdf = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
    uv = nt.nodes.new("ShaderNodeUVMap")
    uv.uv_map = "UVMap"
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (1 / tile, 1 / tile, 1.0)
    nt.links.new(uv.outputs["UV"], mp.inputs["Vector"])
    vec = mp.outputs["Vector"]
    nt.links.new(_image(nt, set_name, "basecolor", vec, True).outputs["Color"], bsdf.inputs["Base Color"])
    nt.links.new(_image(nt, set_name, "roughness", vec).outputs["Color"], bsdf.inputs["Roughness"])
    bsdf.inputs["Metallic"].default_value = metallic
    nm = nt.nodes.new("ShaderNodeNormalMap")
    nm.uv_map = "UVMap"
    nm.inputs["Strength"].default_value = 0.6
    nt.links.new(_image(nt, set_name, "normal", vec).outputs["Color"], nm.inputs["Color"])
    nt.links.new(nm.outputs["Normal"], bsdf.inputs["Normal"])


def cutout(mat, set_name, metallic=0.5):
    """Re-drive an existing material from a texture set with an opacity cut-out (metre UVs)."""
    nt = mat.node_tree
    for n in [n for n in nt.nodes if n.type not in ("BSDF_PRINCIPLED", "OUTPUT_MATERIAL")]:
        nt.nodes.remove(n)
    bsdf = next(n for n in nt.nodes if n.type == "BSDF_PRINCIPLED")
    tile = tile_size(set_name)
    uv = nt.nodes.new("ShaderNodeUVMap")
    uv.uv_map = "UVMap"
    mp = nt.nodes.new("ShaderNodeMapping")
    mp.inputs["Scale"].default_value = (1 / tile, 1 / tile, 1.0)
    nt.links.new(uv.outputs["UV"], mp.inputs["Vector"])
    vec = mp.outputs["Vector"]
    nt.links.new(_image(nt, set_name, "basecolor", vec, True).outputs["Color"], bsdf.inputs["Base Color"])
    nt.links.new(_image(nt, set_name, "roughness", vec).outputs["Color"], bsdf.inputs["Roughness"])
    nt.links.new(_image(nt, set_name, "opacity", vec).outputs["Color"], bsdf.inputs["Alpha"])
    bsdf.inputs["Metallic"].default_value = metallic
    nm = nt.nodes.new("ShaderNodeNormalMap")
    nm.uv_map = "UVMap"
    nm.inputs["Strength"].default_value = 0.5
    nt.links.new(_image(nt, set_name, "normal", vec).outputs["Color"], nm.inputs["Color"])
    nt.links.new(nm.outputs["Normal"], bsdf.inputs["Normal"])
    try:
        mat.surface_render_method = "DITHERED"
    except AttributeError:
        pass
    mat.use_backface_culling = False
    return mat


def define(name, color, metallic=0.0, rough=0.5, **inputs):
    """Register a simple Principled material from a component module (idempotent)."""
    m = _cache.get(name) or bpy.data.materials.get(name)
    if m is None:
        m, _ = _principled(name, color, metallic, rough, **inputs)
    _cache[name] = m
    return m


def get(name):
    m = _cache.get(name) or bpy.data.materials.get(name)
    if m is None:
        m = _build(name)
    _cache[name] = m
    return m
