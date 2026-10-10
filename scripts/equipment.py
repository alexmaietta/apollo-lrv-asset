"""Apollo 17 LRV-3 communications, power and payload equipment.

build(ch) adds, in handbook coordinates (P.sta / P.bl / P.wl, inches):

Forward chassis (vehicle hardware, meshes on the chassis; build.py moves them onto chassis_fwd):
  batteries No.1 (left) / No.2 (right), drive control electronics (DCE), signal processing
  unit (SPU), directional gyro unit (DGU), fused-silica radiator mirrors, multilayer insulation
  blanket (beta cloth), and the three hinged dust covers (links, revolute):
    battery No.1 + DCE cover, SPU cover (slaved to No.1), battery No.2 cover.
Payload (separate links, joint "payload": True):
  LCRU on the front fittings, high-gain antenna (mast -> telescope -> ball joint az/el -> hinge
  plate -> umbrella reflector), low-gain antenna on the left inboard handhold (az/el), GCTA TV
  camera on its TCU at the right-front fitting (pan/tilt), Apollo 17 geology pallet on the aft
  chassis with tongs, scoop, hammer, rake, extension handle, gnomon, sample bag dispenser,
  SEP receiver + tri-loop antenna, traverse gravimeter. Aft-chassis pallet-stop decal.

Sources (PDF pages): A17 Sys HB DWG 3.1 (p.21) mount fittings and forward-chassis layout;
Ops HB Fig 1-16 battery 9.78 x 10.66 x 7.94 in (p.39), Figs 1-31..1-35 thermal control and dust
covers (p.65-69); HSI-481184 LCRU handbook 1.3.1 (22x16x6 in, 55 lb, 196 sq in mirrors), 1.3.3
LGA (9.32 in, 5.13 in reflector, 1.54 in feed, 2.45 lb), 1.3.4 HGA (36 in rib-mesh, 10 lb),
Figs 4-1..4-9; GCTA final report Table 1-2 (pan +214/-134, tilt +85/-45, 18.24 lb, 455 cu in,
clutch 18 in-lb); A17 press kit Fig 3.6-5 (payload stowage, p.119), traverse gravimeter (p.77);
A17 MR App A.3.2 (aft decal). Shapes from NASM QTU photos 00046-00057 and as17-146-22367,
as17-146-22296, as17-137-20979, as16-117-18754.
"""
import math
import random

import bpy
from mathutils import Matrix, Vector

import materials as M
import params as P
from geo import add_mesh, box_geo, link, sensor_camera, tube_geo

IN = P.IN
TAU = 2 * math.pi
UP = Vector((0, 0, 1))


def W(sta, bl, wl):
    """Handbook station / buttock line / waterline (in) -> world point (m)."""
    return Vector((P.sta(sta), P.bl(bl), P.wl(wl)))


def rad(d):
    return math.radians(d)


# ---------------------------------------------------------------- materials

def _knit(mat, pitch=0.0032, thread=0.22):
    """Turn a plain material into an open knitted wire mesh: threads opaque, holes transparent
    (alpha from two thread families on the UVs, metres). Idempotent."""
    nt = mat.node_tree
    if any(n.type == "UVMAP" for n in nt.nodes):
        return mat
    N, L = nt.nodes, nt.links.new
    bsdf = N["Principled BSDF"]
    uv = N.new("ShaderNodeUVMap")
    uv.uv_map = "UVMap"
    sep = N.new("ShaderNodeSeparateXYZ")
    L(uv.outputs["UV"], sep.inputs["Vector"])
    fams = []
    for comp, shift in (("X", 0.0), ("Y", 0.5)):
        dv = N.new("ShaderNodeMath"); dv.operation = "DIVIDE"; dv.inputs[1].default_value = pitch
        L(sep.outputs[comp], dv.inputs[0])
        fr = N.new("ShaderNodeMath"); fr.operation = "FRACT"
        L(dv.outputs[0], fr.inputs[0])
        sb = N.new("ShaderNodeMath"); sb.operation = "SUBTRACT"; sb.inputs[1].default_value = 0.5
        L(fr.outputs[0], sb.inputs[0])
        ab = N.new("ShaderNodeMath"); ab.operation = "ABSOLUTE"
        L(sb.outputs[0], ab.inputs[0])
        gt = N.new("ShaderNodeMath"); gt.operation = "GREATER_THAN"; gt.inputs[1].default_value = 0.5 - thread / 2
        L(ab.outputs[0], gt.inputs[0])
        fams.append(gt)
    mx = N.new("ShaderNodeMath"); mx.operation = "MAXIMUM"
    L(fams[0].outputs[0], mx.inputs[0]); L(fams[1].outputs[0], mx.inputs[1])
    mr = N.new("ShaderNodeMapRange")
    mr.inputs["To Min"].default_value = 0.06
    mr.inputs["To Max"].default_value = 1.0
    L(mx.outputs[0], mr.inputs["Value"])
    L(mr.outputs["Result"], bsdf.inputs["Alpha"])
    try:
        mat.surface_render_method = "DITHERED"
    except AttributeError:
        pass
    return mat


def _mats():
    d = M.define
    return {
        "al": M.get("aluminium"),
        "al_spun": M.get("aluminium_spun"),
        "steel": M.get("steel"),
        "chrome": M.get("chrome"),
        "white": M.get("white_paint"),
        "black": M.get("black_paint"),
        # multilayer insulation outer layer: Teflon-coated beta glass cloth, off-white
        "beta": d("eq_beta_cloth", (0.80, 0.79, 0.74), 0.0, 0.85),
        # aluminised Kapton H film (gold) and its darker copper-orange variant (SEP receiver)
        # colour sampled from the Smithsonian LCRU blanket (NASM2020-00053): saturated amber-gold, hue 1 : 0.48 : 0.03
        "kapton": d("eq_gold_kapton", (0.90, 0.44, 0.03), 0.9, 0.3),
        "kapton_cu": d("eq_copper_kapton", (0.80, 0.33, 0.12), 1.0, 0.22),
        # fused-silica second-surface mirrors (silvered), and their frames
        "mirror": d("eq_osr_mirror", (0.93, 0.94, 0.96), 1.0, 0.03),
        "panel": d("eq_lcru_panel_plate", (0.22, 0.235, 0.28), 0.3, 0.5),   # grey-blue faceplate (00053)
        "knob": d("eq_knob_bronze", (0.075, 0.07, 0.065), 0.4, 0.4),
        "velcro": d("eq_velcro", (0.80, 0.76, 0.66), 0.0, 0.9),
        "wtape": d("eq_white_tape", (0.88, 0.87, 0.82), 0.0, 0.6),
        # pallet structure: light, chemically-filmed aluminium sheet and tube (QTU 00057)
        "pal": d("eq_pallet_frame", (0.76, 0.77, 0.78), 0.55, 0.38),
        "pal_sheet": d("eq_pallet_sheet", (0.70, 0.71, 0.72), 0.5, 0.45),
        "green": d("eq_green_anodized", (0.10, 0.48, 0.38), 0.4, 0.4),   # pip-pin T-levers (00057)
        "ytab": d("eq_yellow_tab_webbing", (0.80, 0.62, 0.25), 0.0, 0.85),      # X-stitched strap tabs
        "anodized": d("eq_black_anodized", (0.02, 0.02, 0.022), 0.4, 0.35),
        "battery": d("eq_battery_case", (0.42, 0.43, 0.44), 0.7, 0.5),
        "fiberglass": d("eq_fiberglass", (0.72, 0.62, 0.38), 0.0, 0.55),
        "copper": d("eq_copper_strap", (0.72, 0.38, 0.20), 1.0, 0.3),
        # gold-tinted knitted wire mesh of the HGA reflector: partly transparent
        # dark bronze knitted wire mesh, ~45 % thread coverage (QTU 00051/00052)
        # knitted reflector mesh as an image-based cut-out so it survives USD / Newton / Isaac
        "hga_mesh": M.cutout(d("eq_hga_mesh", (0.20, 0.15, 0.10), 0.5, 0.55), "hga_knit"),
        "red": d("eq_red_anodized", (0.62, 0.06, 0.04), 0.3, 0.4),
        "blue": d("eq_blue_anodized", (0.08, 0.22, 0.60), 0.3, 0.4),
        # traverse gravimeter thermal cover, light dusty sky-blue fabric; darker blue lid (as17-137-20979,
        # calibrated against the white mounting frame beside it)
        "tge_cover": d("eq_tge_cover", (0.29, 0.41, 0.48), 0.0, 0.85),
        "tge_lid": d("eq_tge_lid_cover", (0.06, 0.19, 0.46), 0.0, 0.8),
        "hood": d("eq_sunshade_gray", (0.62, 0.62, 0.60), 0.0, 0.55),
        "webbing": d("eq_webbing", (0.45, 0.30, 0.16), 0.0, 0.9),
        "gray": d("eq_gray_anodized", (0.25, 0.26, 0.27), 0.5, 0.45),
        "dial": d("eq_dial_face", (0.85, 0.85, 0.82), 0.0, 0.3),
        "decal": d("eq_decal_yellow", (0.85, 0.70, 0.08), 0.0, 0.5),
        "cable": d("eq_cable_white", (0.78, 0.77, 0.72), 0.0, 0.6),
        "teflon": d("eq_teflon_white", (0.90, 0.90, 0.88), 0.0, 0.35),
        "tape": d("eq_binding_tape", (0.82, 0.74, 0.55), 0.0, 0.7),
    }


# ---------------------------------------------------------------- geometry helpers

def perp(d):
    d = Vector(d).normalized()
    hint = UP if abs(d.z) < 0.9 else Vector((1, 0, 0))
    a = hint.cross(d).normalized()
    b = d.cross(a).normalized()
    return a, b


def axis_xform(origin, d, a=None):
    """Matrix taking local (x, y, z) to world with local Y along d (revolve_geo convention)."""
    d = Vector(d).normalized()
    if a is None:
        a, b = perp(d)
    else:
        a = (Vector(a) - d * Vector(a).dot(d)).normalized()
        b = d.cross(a)
    m = Matrix((a, d, b)).transposed().to_4x4()
    m.translation = Vector(origin)
    return m


def cyl(v, f, p0, p1, r, sides=16, caps=True):
    return tube_geo(Vector(p0), Vector(p1), r, sides, caps, v, f)


def bx(v, f, c, size):
    return box_geo(Vector(c), size, v, f)


def obox(v, f, c, ax, ay, az, size):
    """Oriented box: centre c, unit axes ax/ay/az, size (sx, sy, sz)."""
    c, ax, ay, az = Vector(c), Vector(ax), Vector(ay), Vector(az)
    base = len(v)
    for z in (-1, 1):
        for y in (-1, 1):
            for x in (-1, 1):
                v.append(c + ax * (x * size[0] / 2) + ay * (y * size[1] / 2) + az * (z * size[2] / 2))
    for q in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        f.append(tuple(base + i for i in q))
    return v, f


def revolve(v, f, profile, xform, segs=24, closed=False):
    """Revolve (radius, axial) profile about local Y (see axis_xform). Optionally cap ends."""
    n = len(profile)
    base = len(v)
    for s in range(segs):
        t = TAU * s / segs
        for r, a in profile:
            v.append(xform @ Vector((r * math.sin(t), a, r * math.cos(t))))
    for s in range(segs):
        s1 = (s + 1) % segs
        for i in range(n - 1):
            f.append((base + s * n + i, base + s * n + i + 1, base + s1 * n + i + 1, base + s1 * n + i))
    if closed:
        f.append(tuple(base + s * n for s in range(segs))[::-1])
        f.append(tuple(base + s * n + n - 1 for s in range(segs)))
    return v, f


def sphere(v, f, c, r, segs=16, rings=8):
    prof = [(r * math.sin(math.pi * i / rings) + 1e-5, -r * math.cos(math.pi * i / rings)) for i in range(rings + 1)]
    return revolve(v, f, prof, axis_xform(c, UP), segs)


def polyline_tubes(v, f, pts, r, sides=8):
    """Continuous swept tube along a polyline (mitred rings, capped ends)."""
    pts = [Vector(p) for p in pts]
    n = len(pts)
    base = len(v)
    a_prev = None
    for i, p in enumerate(pts):
        t0 = (pts[min(i + 1, n - 1)] - pts[max(i - 1, 0)]).normalized()
        if a_prev is None:
            a, _ = perp(t0)
        else:  # parallel-transport the frame to avoid twisting
            a = (a_prev - t0 * a_prev.dot(t0)).normalized()
        b = t0.cross(a)
        a_prev = a
        for k in range(sides):
            th = TAU * k / sides
            v.append(p + (a * math.cos(th) + b * math.sin(th)) * r)
    for i in range(n - 1):
        for k in range(sides):
            k1 = (k + 1) % sides
            o, o1 = base + i * sides, base + (i + 1) * sides
            f.append((o + k, o + k1, o1 + k1, o1 + k))
    f.append(tuple(base + k for k in reversed(range(sides))))
    f.append(tuple(base + (n - 1) * sides + k for k in range(sides)))
    return v, f


def cable_pts(p0, p1, sag, n=10, side=None):
    """Hanging cable between two points (parabolic sag in -Z, optional lateral bow)."""
    p0, p1 = Vector(p0), Vector(p1)
    out = []
    for i in range(n + 1):
        t = i / n
        p = p0.lerp(p1, t) - UP * (4 * sag * t * (1 - t))
        if side is not None:
            p += Vector(side) * (4 * t * (1 - t))
        out.append(p)
    return out


def crinkle_box(v, f, c, size, amp=0.0, pillow=0.0, cell=1.0 * IN, seed=0, skip=(), wave=0.0, fold=0.0):
    """Blanket-wrapped box: each face is a grid displaced along its normal (edges stay put).

    amp: random per-vertex crinkle; pillow: quilted bulge; wave: low-frequency wrinkles;
    fold: sharp crease ridges (crumpled Kapton film), metres.
    skip: faces to omit, e.g. ("-z",).
    """
    rng = random.Random(seed)
    c = Vector(c)
    h = [s / 2 for s in size]
    E = [Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1))]
    for k in range(3):
        for s in (-1, 1):
            if f"{'+' if s > 0 else '-'}{'xyz'[k]}" in skip:
                continue
            n = E[k] * s
            iu, iv = (k + 1) % 3, (k + 2) % 3
            u, w = E[iu], E[iv]
            if s < 0:
                u, w, iu, iv = w, u, iv, iu
            nu = max(2, round(size[iu] / cell))
            nv = max(2, round(size[iv] / cell))
            ph = [rng.uniform(0, TAU) for _ in range(4)]
            creases = [(rng.uniform(0, TAU), rng.uniform(1.2, 3.5) / IN, rng.uniform(0, TAU), rng.uniform(0.4, 1.0))
                       for _ in range(5)]
            base = len(v)
            for i in range(nu + 1):
                for j in range(nv + 1):
                    a, b = i / nu, j / nv
                    env = (math.sin(math.pi * a) * math.sin(math.pi * b)) ** 0.5
                    wv = wave * (math.sin(5.1 * a + ph[0]) * math.sin(4.3 * b + ph[1])
                                 + 0.6 * math.sin(11.0 * a + 7.0 * b + ph[2]))
                    pu, pv_ = h[iu] * (2 * a - 1), h[iv] * (2 * b - 1)
                    cr = 0.0
                    if fold:
                        for ang, fr, phs, wt in creases:
                            cr += wt * (abs(math.sin((pu * math.cos(ang) + pv_ * math.sin(ang)) * fr + phs)) - 0.6)
                    dn = env * (pillow + amp * rng.uniform(-1, 1) + wv + fold * cr)
                    p = c + n * h[k] + u * pu + w * pv_ + n * dn
                    v.append(p)
            for i in range(nu):
                for j in range(nv):
                    o = base + i * (nv + 1) + j
                    f.append((o, o + nv + 1, o + nv + 2, o + 1))
    return v, f


def crinkle_cyl(v, f, p0, p1, r, amp, seed=0, segs=20, nlen=6):
    """Foil-wrapped cylinder (open ends), radial crinkle."""
    rng = random.Random(seed)
    p0, p1 = Vector(p0), Vector(p1)
    d = (p1 - p0)
    a, b = perp(d)
    base = len(v)
    for i in range(nlen + 1):
        t = i / nlen
        for s in range(segs):
            th = TAU * s / segs
            rr = r + amp * rng.uniform(-1, 1) * (1 if 0 < i < nlen else 0.3)
            v.append(p0 + d * t + (a * math.cos(th) + b * math.sin(th)) * rr)
    for i in range(nlen):
        for s in range(segs):
            s1 = (s + 1) % segs
            o, o1 = base + i * segs, base + (i + 1) * segs
            f.append((o + s, o + s1, o1 + s1, o1 + s))
    return v, f


def _axis_samples(h, r, cell, nr=4):
    """1-D sample positions over [-h, h]: dense across the rounded edges, ~cell in between."""
    r = min(r, h * 0.95)
    ends = [-h + r * i / nr for i in range(nr)]
    inner = max(1, round((2 * (h - r)) / cell))
    mid = [-(h - r) + 2 * (h - r) * i / inner for i in range(inner + 1)]
    return ends + mid + [-t for t in reversed(ends)]


def _tent(t, h, seams):
    """Quilting envelope along one axis: 0 at the box edges and at seam lines, 1 mid-panel."""
    cuts = [-h] + sorted(seams) + [h]
    for a, b in zip(cuts, cuts[1:]):
        if a <= t <= b:
            return max(0.0, math.sin(math.pi * (t - a) / max(b - a, 1e-9))) ** 0.55
    return 0.0


def pillow_box(v, f, c, size, r=0.5 * IN, cell=0.35 * IN, seams=((), (), ()), puff=0.3 * IN,
               wrinkle=0.0, nwrinkle=3, sag=0.0, seed=0, skip=(), holes=(), inset=0.0):
    """Multilayer-insulation blanket over a box: rounded edges, quilted panels that puff out between
    seam lines, a few large folds, and an optional sagging bottom.

    seams: per axis, offsets (m, from the centre) of stitched seam lines that pinch the blanket.
    holes: cut-outs as (face, (lo, hi) along the face's first in-plane axis, (lo, hi) along the
    second), axes in xyz order, e.g. ("+x", (y0, y1), (z0, z1)); inside them the blanket is tucked
    back by `inset` (down to the case) so a panel can sit flush.
    Returns a callable surf(point_on_box) -> (point on the un-puffed rounded surface, normal), for
    placing tapes, velcro and piping.
    """
    rng = random.Random(seed)
    c = Vector(c)
    h = [s / 2 for s in size]
    r = min(r, *[x * 0.9 for x in h])
    core = [x - r for x in h]
    E = [Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1))]

    def round_pt(p):
        q = Vector([max(-core[i], min(core[i], p[i])) for i in range(3)])
        d = p - q
        if d.length < 1e-9:
            return c + p, None
        n = d.normalized()
        return c + q + n * r, n

    for k in range(3):
        for s in (-1, 1):
            if f"{'+' if s > 0 else '-'}{'xyz'[k]}" in skip:
                continue
            iu, iv = (k + 1) % 3, (k + 2) % 3
            if s < 0:
                iu, iv = iv, iu
            us = _axis_samples(h[iu], r, cell)
            vs = _axis_samples(h[iv], r, cell)
            folds = []
            for _ in range(nwrinkle):
                ang = rng.uniform(0, math.pi)
                folds.append((Vector((rng.uniform(-h[iu], h[iu]) * 0.6, rng.uniform(-h[iv], h[iv]) * 0.6)),
                              Vector((math.cos(ang), math.sin(ang))), rng.uniform(0.6, 1.4) * IN,
                              rng.choice((-1, 1)) * rng.uniform(0.5, 1.0)))
            fname = f"{'+' if s > 0 else '-'}{'xyz'[k]}"
            fholes = []
            for hf, r1, r2 in holes:
                if hf == fname:
                    lo_hi = {min(iu, iv): r1, max(iu, iv): r2}
                    fholes.append((lo_hi[iu], lo_hi[iv]))
            base = len(v)
            for a in us:
                for b in vs:
                    p = Vector((0.0, 0.0, 0.0))
                    p[k] = s * h[k]
                    p[iu], p[iv] = a, b
                    pt, n = round_pt(p)
                    if n is None:
                        n = E[k] * s
                    env = _tent(a, h[iu], seams[iu]) * _tent(b, h[iv], seams[iv])
                    dn = puff * env
                    if wrinkle:
                        q2 = Vector((a, b))
                        for o, dvec, wd, sg in folds:
                            dist = abs((q2 - o).x * dvec.y - (q2 - o).y * dvec.x)
                            along = (q2 - o).dot(dvec)
                            fade = math.exp(-(along / (max(h[iu], h[iv]) * 0.8)) ** 2)
                            dn += wrinkle * sg * fade * math.exp(-(dist / wd) ** 2) * env ** 0.5
                    if sag and k == 2 and s < 0:
                        dn += sag * env
                    for (u0, u1), (w0, w1) in fholes:
                        du = max(u0 - a, a - u1, 0.0)
                        dw = max(w0 - b, b - w1, 0.0)
                        g = min(1.0, math.hypot(du, dw) / (0.9 * IN))
                        g = g * g * (3 - 2 * g)
                        dn = dn * g - inset * (1 - g)
                    v.append(pt + n * dn)
            nu, nv = len(us), len(vs)
            for i in range(nu - 1):
                for j in range(nv - 1):
                    o = base + i * nv + j
                    f.append((o, o + nv, o + nv + 1, o + 1))

    def surf(p):
        """Puffed blanket surface point and normal nearest a point given on/near the box."""
        p = Vector(p) - c
        k = max(range(3), key=lambda i: abs(p[i]) / h[i])
        iu, iv = (k + 1) % 3, (k + 2) % 3
        pt, n = round_pt(p)
        if n is None:
            n = E[k] * (1 if p[k] > 0 else -1)
        env = _tent(p[iu], h[iu], seams[iu]) * _tent(p[iv], h[iv], seams[iv])
        return pt + n * puff * env, n
    return surf


def pillow_piping(v, f, c, size, r, rad, seams=((), (), ()), edges=True, axes=(0, 1, 2), skip_faces=(),
                  lift=0.0, n=24, top=None):
    """Rolled-edge piping tubes along the rounded box edges and over the seam lines.
    top: None = all lines; True = only lines lying on the +z face; False = all others."""
    c = Vector(c)
    h = [s / 2 for s in size]
    r = min(r, *[x * 0.9 for x in h])
    core = [x - r for x in h]

    def on(p):
        q = Vector([max(-core[i], min(core[i], p[i])) for i in range(3)])
        d = p - q
        nn = d.normalized() if d.length > 1e-9 else Vector((0, 0, 1))
        return c + q + nn * (r + lift)

    lines = []
    if edges:
        for k in axes:  # edges parallel to axis k
            i, j = (k + 1) % 3, (k + 2) % 3
            for si in (-1, 1):
                for sj in (-1, 1):
                    if (f"{'+' if si > 0 else '-'}{'xyz'[i]}" in skip_faces or
                            f"{'+' if sj > 0 else '-'}{'xyz'[j]}" in skip_faces):
                        continue
                    a = Vector((0, 0, 0)); b = Vector((0, 0, 0))
                    a[k], b[k] = -h[k], h[k]
                    a[i] = b[i] = si * h[i]
                    a[j] = b[j] = sj * h[j]
                    lines.append((a, b))
    for k in range(3):  # seams across the faces perpendicular to axis k ... run around the box
        for t in seams[k]:
            i, j = (k + 1) % 3, (k + 2) % 3
            for fi, fs in ((i, -1), (i, 1), (j, -1), (j, 1)):
                if f"{'+' if fs > 0 else '-'}{'xyz'[fi]}" in skip_faces:
                    continue
                other = j if fi == i else i
                a = Vector((0, 0, 0)); b = Vector((0, 0, 0))
                a[k] = b[k] = t
                a[fi] = b[fi] = fs * h[fi]
                a[other], b[other] = -h[other], h[other]
                lines.append((a, b))
    for a, b in lines:
        if top is not None and (abs(a[2] - h[2]) < 1e-9 and abs(b[2] - h[2]) < 1e-9) != top:
            continue
        pts = [on(a.lerp(b, q / n)) for q in range(n + 1)]
        polyline_tubes(v, f, pts, rad, 8)
    return v, f


def surface_strip(v, f, surf, p0, p1, w, t=0.02 * IN, lift=0.03 * IN, n=8):
    """Thin tape strip laid over the rounded blanket surface between two box-surface points."""
    p0, p1 = Vector(p0), Vector(p1)
    base = len(v)
    for q in range(n + 1):
        p = p0.lerp(p1, q / n)
        pt, nn = surf(p)
        if nn is None:
            nn = UP
        d = (p1 - p0).normalized()
        side = nn.cross(d).normalized()
        for sd in (-1, 1):
            for tt in (0.0, t):
                v.append(pt + nn * (lift + tt) + side * (sd * w / 2))
    for q in range(n):
        o, o1 = base + q * 4, base + (q + 1) * 4
        f.append((o + 1, o1 + 1, o1 + 3, o + 3))   # top
        f.append((o, o + 2, o1 + 2, o1))           # bottom
        f.append((o, o1, o1 + 1, o + 1))
        f.append((o + 2, o + 3, o1 + 3, o1 + 2))
    f.append((base, base + 1, base + 3, base + 2))
    o = base + n * 4
    f.append((o, o + 2, o + 3, o + 1))
    return v, f


def mirror_tiles(v, f, x0, x1, y0, y1, z, nx, ny, gap=0.08 * IN, t=0.06 * IN):
    """Grid of second-surface mirror tiles on a horizontal radiator plate (world metres)."""
    dx, dy = (x1 - x0) / nx, (y1 - y0) / ny
    for i in range(nx):
        for j in range(ny):
            bx(v, f, (x0 + dx * (i + 0.5), y0 + dy * (j + 0.5), z + t / 2), (abs(dx) - gap, abs(dy) - gap, t))
    return v, f


def prism(v, f, poly, z0, z1):
    """Extrude a CCW polygon (list of (x, y) world metres) between z0 and z1."""
    n = len(poly)
    base = len(v)
    for z in (z0, z1):
        for x, y in poly:
            v.append(Vector((x, y, z)))
    f.append(tuple(base + i for i in reversed(range(n))))
    f.append(tuple(base + n + i for i in range(n)))
    for i in range(n):
        j = (i + 1) % n
        f.append((base + i, base + j, base + n + j, base + n + i))
    return v, f


def mesh(name, v, f, mat, parent, smooth=False, uv=None, variant=None):
    """Add a mesh with UVs: box projection in metres by default (for the tileable texture sets
    materials.upgrade_all() attaches), or uv(world_point) -> (u, v) per vertex.
    variant: "Set:option" tag forwarded to geo.add_mesh (part exists only in that option)."""
    ob = add_mesh(f"eq_{name}", v, f, mat, parent, smooth=smooth, variant=variant)
    me = ob.data
    layer = me.uv_layers.new(name="UVMap") if not me.uv_layers else me.uv_layers[0]
    origin = parent.matrix_world.translation
    for poly in me.polygons:
        n = poly.normal
        k = max(range(3), key=lambda i: abs(n[i]))
        a, b = [(1, 2), (0, 2), (0, 1)][k]
        for li in poly.loop_indices:
            co = me.vertices[me.loops[li].vertex_index].co
            if uv is not None:
                layer.data[li].uv = uv(co + origin)
            else:
                layer.data[li].uv = (co[a], co[b])
    return ob


# ---------------------------------------------------------------- forward chassis electronics

# Equipment plan (STA, BL in inches). Batteries per Ops HB Fig 1-16: body 10.00 x 9.12 x 7.94
# with a 10.66 x 9.78 top flange; they sit on fiberglass mounts on the frame top (WL 102.95).
FRAME_TOP = 100.0 + P.FRAME_H_IN
BAT_STA = (21.4, 31.4)                  # fore-aft 10.0 body
BAT_BL = (7.9, 17.0)                    # lateral 9.12 body (left battery; right mirrored)
BAT_Z = (FRAME_TOP + 0.05, FRAME_TOP + 0.05 + 7.94)
RAD_TOP = BAT_Z[1] + 0.18               # radiator plate + mirrors
COVER_Z = (RAD_TOP + 0.05, RAD_TOP + 0.65)
DCE = ((32.6, 43.6), (-2.2, 17.2))      # DCE + thermal control unit, behind battery 1 (Fig 1-31)
SPU = ((21.6, 31.0), (-6.4, 6.4))       # SPU between the batteries, strapped to battery 1 (Fig 1-33)
DGU = ((33.6, 40.0), (-15.5, -8.5))     # gyro behind battery 2 (Fig 1-34)


def _rect(sta, bl):
    """World x/y bounds from station and buttock-line ranges."""
    xs = sorted((P.sta(sta[0]), P.sta(sta[1])))
    ys = sorted((P.bl(bl[0]), P.bl(bl[1])))
    return xs, ys


def build_forward_electronics(ch, m):
    v, f = [], []  # battery cases
    fl, ff = [], []  # flange bolts / fittings (steel)
    for side in (1, -1):
        bl = (side * BAT_BL[0], side * BAT_BL[1])
        (x0, x1), (y0, y1) = _rect(BAT_STA, bl)
        z0, z1 = P.wl(BAT_Z[0]), P.wl(BAT_Z[1])
        bx(v, f, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0 - 0.3 * IN))
        # top flange 10.66 x 9.78 with bolt heads (Fig 1-16)
        cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
        bx(v, f, (cx, cy, z1 - 0.15 * IN), (10.66 * IN, 9.78 * IN, 0.3 * IN))
        for i in range(8):
            for sx in (-1, 1):
                cyl(fl, ff, (cx + (i - 3.5) * 1.3 * IN, cy + sx * 4.65 * IN, z1),
                    (cx + (i - 3.5) * 1.3 * IN, cy + sx * 4.65 * IN, z1 + 0.08 * IN), 0.11 * IN, 6)
        # connectors J1/J2 and relief valve on the aft face, fiberglass mounts at the sides
        for dz, r in ((2.0, 0.7), (-0.6, 0.35), (-2.0, 0.55)):
            cyl(fl, ff, (x0, cy, (z0 + z1) / 2 + dz * IN), (x0 - 0.6 * IN, cy, (z0 + z1) / 2 + dz * IN), r * IN, 12)
        mesh(f"battery{1 if side > 0 else 2}_case", v, f, m["battery"], ch)
        v, f = [], []
        # radiator: fused-silica second-surface mirrors on an aluminium plate
        rv, rf = [], []
        bx(rv, rf, (cx, cy, P.wl(BAT_Z[1]) + 0.05 * IN), (10.3 * IN, 9.4 * IN, 0.1 * IN))
        mv, mf = mirror_tiles([], [], cx - 4.9 * IN, cx + 4.9 * IN, cy - 4.4 * IN, cy + 4.4 * IN,
                              P.wl(BAT_Z[1]) + 0.1 * IN, 6, 5)
        mesh(f"battery{1 if side > 0 else 2}_radiator", rv, rf, m["al"], ch)
        mesh(f"battery{1 if side > 0 else 2}_mirrors", mv, mf, m["mirror"], ch)
        # fiberglass mounts (Fig 1-35), bi-metallic dust-cover closing actuator on the outboard face
        gv, gf = [], []
        for sx in (x0 + 1.5 * IN, x1 - 1.5 * IN):
            for yy in (y0 - 0.35 * IN, y1 + 0.35 * IN):
                obox(gv, gf, (sx, yy, z0 + 1.2 * IN), (1, 0, 0), (0, 1, 0), (0, 0, 1), (1.4 * IN, 0.7 * IN, 2.4 * IN))
        mesh(f"battery{1 if side > 0 else 2}_mounts", gv, gf, m["fiberglass"], ch)
        yo = y1 if side > 0 else y0
        cyl(fl, ff, (cx + 1.5 * IN, yo, z1 - 1.5 * IN), (cx + 1.5 * IN, yo + side * 0.5 * IN, z1 - 1.5 * IN), 0.9 * IN, 16)
    mesh("battery_hardware", fl, ff, m["steel"], ch, smooth=True)

    # DCE, SPU and gyro boxes with their radiators
    for name, (sta, bl), ztop, nx, ny in (("dce", DCE, BAT_Z[1], 7, 10), ("spu", SPU, BAT_Z[1], 5, 6),
                                          ("dgu", DGU, FRAME_TOP + 5.6, 0, 0)):
        (x0, x1), (y0, y1) = _rect(sta, bl)
        z0, z1 = P.wl(FRAME_TOP + 0.05), P.wl(ztop)
        v, f = [], []
        bx(v, f, ((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0))
        if name == "dce":  # fusible-mass (octadecane wax) tank under the radiator
            bx(v, f, ((x0 + x1) / 2, (y0 + y1) / 2, z1 - 0.9 * IN), (x1 - x0 + 0.3 * IN, y1 - y0 + 0.3 * IN, 1.5 * IN))
        mesh(f"{name}_case", v, f, m["gray"] if name != "dgu" else m["white"], ch)
        if nx:
            mv, mf = mirror_tiles([], [], x0 + 0.3 * IN, x1 - 0.3 * IN, y0 + 0.3 * IN, y1 - 0.3 * IN, z1, nx, ny)
            mesh(f"{name}_mirrors", mv, mf, m["mirror"], ch)
    # thermal straps: SPU -> battery 1, gyro -> battery 2 (Figs 1-33, 1-34)
    v, f = [], []
    bx(v, f, W(26.4, 7.15, FRAME_TOP + 5.5), (5.0 * IN, 1.6 * IN, 0.12 * IN))
    bx(v, f, W(33.0, -12.0, FRAME_TOP + 3.5), (1.6 * IN, 4.0 * IN, 0.12 * IN))
    mesh("thermal_straps", v, f, m["copper"], ch)

    # multilayer insulation blanket (Ops HB Fig 1-36, QTU 00046/00048): each equipment group sits
    # in its own pillowy beta-cloth envelope with cream binding down the corners and along the
    # bottom; the gyro corner has a fixed, lower top. Radiators are left open under the covers.
    zb, zt = P.wl(100.7), P.wl(COVER_Z[0] - 0.05)
    v, f = [], []
    pv, pf = [], []
    blocks = (((20.9, 32.5), (7.2, 18.3), zt, ("+z", "-z"), 41),
              ((20.9, 32.5), (-18.3, -7.2), zt, ("+z", "-z"), 42),
              ((20.9, 32.5), (-7.2, 7.2), zt, ("+z", "-z"), 43),
              ((32.5, 44.0), (-2.5, 18.3), zt, ("+z", "-z"), 44),
              ((32.5, 44.0), (-18.3, -2.5), P.wl(FRAME_TOP + 7.0), ("-z",), 45))
    for sta, bl, top, skip, seed in blocks:
        (x0, x1), (y0, y1) = _rect(sta, bl)
        c = Vector(((x0 + x1) / 2, (y0 + y1) / 2, (zb + top) / 2))
        size = (x1 - x0, y1 - y0, top - zb)
        pillow_box(v, f, c, size, r=0.45 * IN, cell=0.45 * IN, puff=0.4 * IN, wrinkle=0.35 * IN, nwrinkle=4,
                   seed=seed, skip=skip)
        pillow_piping(pv, pf, c, size, 0.45 * IN, 0.12 * IN, skip_faces=("+z",) if "+z" in skip else (),
                      lift=0.03 * IN)
    mesh("fwd_blanket", v, f, m["beta"], ch, smooth=True)
    mesh("fwd_blanket_binding", pv, pf, m["tape"], ch, smooth=True)


def _cover_slabs(m, rects, z0, z1, seed, link_ob, name):
    """Dust cover: fiberglass panel under a quilted multilayer blanket: beta-cloth top with cream
    edge binding and a taped cross (QTU 00046), Kapton H underside (Creel 2007 p.22)."""
    tv, tf, bv, bf, ev, ef, sv, sf = [], [], [], [], [], [], [], []
    z1 = z0 + 1.0 * IN
    for i, (sta, bl) in enumerate(rects):
        (x0, x1), (y0, y1) = _rect(sta, bl)
        c = Vector(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2 + 0.05 * IN))
        size = (x1 - x0, y1 - y0, z1 - z0 - 0.1 * IN)
        surf = pillow_box(tv, tf, c, size, r=0.35 * IN, cell=0.45 * IN, puff=0.3 * IN, wrinkle=0.18 * IN,
                          nwrinkle=3, seed=seed + i, skip=("-z",))
        pillow_piping(ev, ef, c, size, 0.35 * IN, 0.13 * IN, skip_faces=("-z",), lift=0.02 * IN)
        hx, hy, hz = size[0] / 2, size[1] / 2, size[2] / 2
        surface_strip(sv, sf, surf, c + Vector((-hx, 0, hz)), c + Vector((hx, 0, hz)), 1.2 * IN, n=30)
        surface_strip(sv, sf, surf, c + Vector((0, -hy, hz)), c + Vector((0, hy, hz)), 1.2 * IN, n=30)
        bx(bv, bf, (c.x, c.y, z0 + 0.05 * IN), (x1 - x0 - 0.1 * IN, y1 - y0 - 0.1 * IN, 0.1 * IN))
    mesh(f"{name}_top", tv, tf, m["beta"], link_ob, smooth=True)
    mesh(f"{name}_binding", ev, ef, m["tape"], link_ob, smooth=True)
    mesh(f"{name}_tape", sv, sf, m["wtape"], link_ob)
    mesh(f"{name}_inner", bv, bf, m["kapton"], link_ob)


def build_dust_covers(ch, m):
    """Three manually opened, bimetal-closed dust covers (Ops HB 1.8.2, Figs 1-31..1-35).

    Hinged along the outboard edge (opening strap on the inboard edge, Fig 1-33); rest pose closed
    (driving configuration); the overcentre latch holds them ~95 deg open while parked.
    """
    z0, z1 = P.wl(COVER_Z[0]), P.wl(COVER_Z[1])
    zh = z1 - 0.1 * IN
    drive = {"kind": "position", "stiffness": 8.0, "damping": 0.8, "max_force": 4.0, "target": 0.0}
    specs = (
        # name, hinge BL, axis, rects (sta, bl), mass
        ("bat1_cover", 18.0, (-1.0, 0.0, 0.0), [((21.0, 32.4), (7.3, 18.0)), ((32.4, 43.8), (-2.5, 18.0))], 0.9),
        ("spu_cover", 6.9, (-1.0, 0.0, 0.0), [((21.2, 31.4), (-6.9, 6.9))], 0.35),
        ("bat2_cover", -18.0, (1.0, 0.0, 0.0), [((21.0, 32.4), (-18.0, -7.3))], 0.45),
    )
    for idx, (name, hbl, axis, rects, mass) in enumerate(specs):
        sta_lo = min(r[0][0] for r in rects)
        sta_hi = max(r[0][1] for r in rects)
        hinge = W((sta_lo + sta_hi) / 2, hbl, COVER_Z[1] - 0.1)
        lk = link(name, hinge, ch, {
            "type": "revolute", "axis": axis, "limit": (0.0, rad(100)), "role": "dust_cover",
            "drive": drive}, mass=mass)
        _cover_slabs(m, rects, z0, z1, 90 + 7 * idx, lk, name)
        # piano hinge along the edge, aluminium support/connecting link, opening strap
        v, f = [], []
        cyl(v, f, (P.sta(sta_lo + 0.5), hinge.y, zh), (P.sta(sta_hi - 0.5), hinge.y, zh), 0.14 * IN, 10)
        s = -1 if axis[0] < 0 else 1  # inboard direction in y
        inb = P.bl(hbl) + s * P.bl(abs(rects[0][1][1] - rects[0][1][0]))
        mesh(f"{name}_hinge", v, f, m["al"], lk, smooth=True)
        v, f = [], []
        sx = P.sta((rects[0][0][0] + rects[0][0][1]) / 2)
        bx(v, f, (sx, inb + s * 0.6 * IN, z1 - 0.4 * IN), (1.4 * IN, 1.4 * IN, 0.05 * IN))
        bx(v, f, (sx, inb + s * 1.3 * IN, z1 - 1.6 * IN), (1.4 * IN, 0.05 * IN, 2.6 * IN))
        mesh(f"{name}_strap", v, f, m["beta"], lk)


# ---------------------------------------------------------------- mounting fittings

FIT_STA = 13.6         # fitting bores, ~2 in forward of the STA 15.50 face (DWG 3.1 plan view)
FIT_WL = 101.5
HGA_BL, TV_BL, LCRU_BL = 14.25, -14.25, 9.85


def build_fittings(ch, m):
    v, f = [], []
    for bl in (HGA_BL, TV_BL):
        c = W(FIT_STA, bl, FIT_WL)
        revolve(v, f, [(0.38 * IN, -1.2 * IN), (0.62 * IN, -1.2 * IN), (0.62 * IN, 1.2 * IN), (0.38 * IN, 1.2 * IN)],
                axis_xform(c, UP), 16, closed=False)
        bx(v, f, W(FIT_STA + 1.2, bl, FIT_WL), (1.6 * IN, 0.5 * IN, 1.8 * IN))
    for s in (1, -1):  # LCRU clevis fittings
        bx(v, f, W(FIT_STA + 0.9, s * LCRU_BL, FIT_WL + 0.6), (2.2 * IN, 0.45 * IN, 1.4 * IN))
    mesh("mount_fittings", v, f, m["al"], ch, smooth=False)


# ---------------------------------------------------------------- LCRU

LCRU_STA = (4.6, 20.6)          # 16 in fore-aft (QTU side view 00047: front ~21 in ahead of the axle)
LCRU_BL = 11.0                  # 22 in wide
LCRU_Z = (103.7, 109.7)         # 6 in tall, above the frame top
BLANKET_T = 0.35                # MLI blanket thickness over the case, in

# Control panel (NASM-NASM2020-00053, handbook Fig 4-1): 12.0 x 4.6 in faceplate on the front face,
# toward the HGA (vehicle left) end. (u, v) in inches from its top-left corner as seen from the front;
# must match scripts/eq_textures.py.
PANEL_W, PANEL_H = 12.0, 4.6
PANEL_BL = (-3.4, 8.6)
PANEL_PROUD = BLANKET_T + 0.12  # faceplate sits flush in a cut-out of the blanket
MONITOR_SWITCH = (2.75, 1.75)
METER = (1.0, 3.45)
METER_R = 0.56
MODE_KNOB = (6.35, 2.95)
POWER_SWITCH = (9.55, 1.45)
BREAKER = (11.0, 1.25)
CONNECTOR = (10.7, 3.25)
SCREWS = ((4.35, 0.75), (7.9, 0.75), (7.3, 4.25), (0.35, 0.4), (11.65, 4.25), (11.65, 0.4))


def _tex_material(name, fname, fallback=(0.5, 0.5, 0.5), metallic=0.0, rough=0.5):
    """Material whose base colour is a generated texture in textures/gen (scripts/eq_textures.py)."""
    mat = bpy.data.materials.get(name)
    if mat:
        return mat
    mat = M.define(name, fallback, metallic, rough)
    try:
        path = M.texture_path(fname)
    except FileNotFoundError:
        print(f"{fname} missing: run .venv/bin/python scripts/eq_textures.py")
        return mat
    nt = mat.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = bpy.data.images.load(path, check_existing=True)
    tex.interpolation = "Cubic"
    tex.extension = "EXTEND"
    nt.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    return mat


# placard texture -> (fallback colour, metallic, roughness); see eq_textures.PLACARDS
PLACARD_MATS = {
    "eq_label_hga": ((0.85, 0.80, 0.70), 0.0, 0.8),
    "eq_label_lga": ((0.85, 0.80, 0.70), 0.0, 0.8),
    "eq_placard_collection_bag": ((0.02, 0.02, 0.02), 0.0, 0.45),
    "eq_placard_tongs": ((0.02, 0.02, 0.02), 0.0, 0.45),
    "eq_placard_extension_handle": ((0.02, 0.02, 0.02), 0.0, 0.45),
    "eq_placard_gnomon": ((0.02, 0.02, 0.02), 0.0, 0.45),
    "eq_placard_dust": ((0.02, 0.02, 0.02), 0.0, 0.45),
    "eq_partno_seb39105285": ((0.70, 0.71, 0.70), 0.5, 0.45),
}


def placard(name, tex, c, right, up, w, h, parent, lift=0.03 * IN, variant=None):
    """Thin textured placard quad (text along `right`, face normal right x up), w x h in inches."""
    c = Vector(c)
    right, up = Vector(right).normalized(), Vector(up).normalized()
    nrm = right.cross(up)
    c = c + nrm * lift
    hw, hh = w * IN / 2, h * IN / 2
    v = [c - right * hw - up * hh, c + right * hw - up * hh, c + right * hw + up * hh, c - right * hw + up * hh]
    col, metal, rough = PLACARD_MATS[tex]
    mat = _tex_material(tex, f"{tex}.png", col, metal, rough)
    return mesh(name, v, [(0, 1, 2, 3)], mat, parent, variant=variant,
                uv=lambda p: ((p - c).dot(right) / (2 * hw) + 0.5, (p - c).dot(up) / (2 * hh) + 0.5))


def _panel_material(m):
    mat = bpy.data.materials.get("eq_lcru_panel_face")
    if mat:
        return mat
    mat = M.define("eq_lcru_panel_face", (0.24, 0.25, 0.30), 0.3, 0.5)
    try:
        path = M.texture_path("eq_lcru_panel.png")
    except FileNotFoundError:
        print("eq_lcru_panel.png missing: run .venv/bin/python scripts/eq_textures.py")
        return mat
    nt = mat.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    tex = nt.nodes.new("ShaderNodeTexImage")
    tex.image = bpy.data.images.load(path, check_existing=True)
    tex.interpolation = "Cubic"
    tex.extension = "EXTEND"
    nt.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
    return mat


def build_lcru(ch, m):
    xs, ys = _rect(LCRU_STA, (-LCRU_BL, LCRU_BL))
    z0, z1 = P.wl(LCRU_Z[0]), P.wl(LCRU_Z[1])
    cx, cz = (xs[0] + xs[1]) / 2, (z0 + z1) / 2
    lk = link("lcru", (cx, 0.0, z0), ch, {"type": "fixed", "role": "lcru", "payload": True},
              mass=55.0 * 0.4536)  # 55 lb Earth weight incl. battery (HSI-481184 1.3.1)
    xf = xs[1]  # case front face (x max)
    t = BLANKET_T * IN
    c = Vector((cx, 0.0, cz))
    size = (xs[1] - xs[0] + 2 * t, 2 * LCRU_BL * IN + 2 * t, z1 - z0 + 2 * t)
    # Kapton blanket (QTU 00053, 00046, 00050): quilted panels, two stitched ridges running across
    # the top, vertical seams near the ends, crimped rolled edges, a sagging underside.
    seams = ((-2.6 * IN, 2.6 * IN), (-7.6 * IN, 7.6 * IN), ())
    r = 0.6 * IN
    # The top panel exists only with the blanket closed (museum QTU, variant LCRU:closed); in
    # lunar operation it is opened off the radiator (variant LCRU:radiator_open, as17-146-22367).
    CL, OP = "LCRU:closed", "LCRU:radiator_open"
    kw = dict(r=r, cell=0.35 * IN, seams=seams, puff=0.34 * IN, wrinkle=0.3 * IN, nwrinkle=4, sag=0.35 * IN,
              seed=21, inset=0.3 * IN,
              holes=(("+x", (P.bl(PANEL_BL[0]) - 0.15 * IN, P.bl(PANEL_BL[1]) + 0.15 * IN),
                      (-(PANEL_H / 2 + 0.15) * IN, (PANEL_H / 2 + 0.15) * IN)),))
    v, f = [], []
    surf = pillow_box(v, f, c, size, skip=("+z",), **kw)
    mesh("lcru_blanket", v, f, m["kapton"], lk, smooth=True)
    v, f = [], []
    pillow_box(v, f, c, size, skip=("-x", "+x", "-y", "+y", "-z"), **kw)
    mesh("lcru_blanket_top", v, f, m["kapton"], lk, smooth=True, variant=CL)
    for top, var, sfx in ((False, None, ""), (True, CL, "_top")):
        v, f = [], []
        pillow_piping(v, f, c, size, r, 0.13 * IN, lift=0.02 * IN, top=top)
        mesh(f"lcru_blanket_edges{sfx}", v, f, m["kapton"], lk, smooth=True, variant=var)
        v, f = [], []
        pillow_piping(v, f, c, size, r, 0.1 * IN, seams=seams, edges=False, lift=0.0, top=top)
        mesh(f"lcru_seam_tape{sfx}", v, f, m["wtape"], lk, smooth=True, variant=var)
    # beige beta-cloth binding along the top-front edge (00053), white tape tabs over the top
    # edges (closed only); velcro closure strips on the ends and the front
    hx, hy, hz = [s_ / 2 for s_ in size]
    v, f = [], []
    surface_strip(v, f, surf, c + Vector((hx - 0.9 * IN, -hy + 0.3 * IN, hz)),
                  c + Vector((hx - 0.9 * IN, hy - 0.3 * IN, hz)), 1.1 * IN, n=40)
    for y in (-9.6, -4.0, 1.5, 9.0):
        surface_strip(v, f, surf, c + Vector((hx - 1.6 * IN, y * IN, hz)), c + Vector((hx, y * IN, hz)), 0.9 * IN, n=6)
        surface_strip(v, f, surf, c + Vector((hx, y * IN, hz - 0.1 * IN)),
                      c + Vector((hx, y * IN, hz - 1.3 * IN)), 0.9 * IN, n=6)
    for x in (-5.5, 0.0, 5.5):
        surface_strip(v, f, surf, c + Vector((x * IN, -hy + 1.2 * IN, hz)), c + Vector((x * IN, -hy, hz)), 0.8 * IN, n=6)
    mesh("lcru_tape_tabs", v, f, m["wtape"], lk, variant=CL)
    _lcru_radiator_open(m, lk, xs, xf, z0, z1, cx, t, OP)
    v, f = [], []
    for sy in (-1, 1):
        for x in (-5.0, -3.4, 4.2):
            surface_strip(v, f, surf, c + Vector((x * IN, sy * hy, hz - 1.0 * IN)),
                          c + Vector((x * IN, sy * hy, -hz + 1.6 * IN)), 0.55 * IN, n=10)
    for y in (-10.4, -6.8, -4.6):
        surface_strip(v, f, surf, c + Vector((hx, y * IN, -hz + 3.4 * IN)), c + Vector((hx, y * IN, -hz + 0.6 * IN)),
                      0.55 * IN, n=8)
    mesh("lcru_velcro", v, f, m["velcro"], lk)
    # aluminium lifting / tie-down tabs at the top corners (00053)
    v, f = [], []
    for y in (-(LCRU_BL - 0.4), LCRU_BL - 3.2):
        p = Vector((xf - 0.3 * IN, y * IN, z1 + t + 0.75 * IN))
        obox(v, f, p, (1, 0, 0), (0, 1, 0), (0, 0, 1), (0.1 * IN, 0.75 * IN, 1.5 * IN))
    mesh("lcru_tabs", v, f, m["al"], lk)
    v, f = [], []
    for y in (-(LCRU_BL - 0.4), LCRU_BL - 3.2):
        p = Vector((xf - 0.3 * IN, y * IN, z1 + t + 1.15 * IN))
        cyl(v, f, p - Vector((0.07 * IN, 0, 0)), p + Vector((0.07 * IN, 0, 0)), 0.14 * IN, 10)
    mesh("lcru_tab_holes", v, f, m["black"], lk)

    _lcru_panel(m, lk, xf, z1)
    LCRU_SURF[:] = [surf]
    _lcru_details(m, lk, xs, xf, z0, z1, cx, cz, t)
    return lk


def _lcru_radiator_open(m, lk, xs, xf, z0, z1, cx, t, var):
    """Lunar-operations configuration: top blanket opened off the 196 sq in (14 x 14 in) mirror
    radiator, flaps rolled back along the sides and rear, front flap hanging over the right half of
    the front face (HSI-481184 1.3.1.1 'total exposure'; as17-146-22367)."""
    zl = z1 + t - 0.2 * IN
    v, f = [], []
    bx(v, f, (cx, 0.0, zl), (xs[1] - xs[0] + 2 * t - 0.1 * IN, 2 * LCRU_BL * IN + 2 * t - 0.1 * IN, 0.2 * IN))
    mesh("lcru_lid", v, f, m["al"], lk, variant=var)
    mv, mf = mirror_tiles([], [], cx - 6.0 * IN, cx + 8.0 * IN, -7.0 * IN, 7.0 * IN, zl + 0.1 * IN, 8, 8)
    mesh("lcru_mirrors", mv, mf, m["mirror"], lk, variant=var)
    v, f = [], []
    for sy in (1, -1):   # side panels rolled back
        pillow_box(v, f, (cx, sy * 9.4 * IN, zl + 0.55 * IN), (xs[1] - xs[0] + 0.4 * IN, 3.0 * IN, 0.9 * IN),
                   r=0.38 * IN, cell=0.3 * IN, puff=0.12 * IN, wrinkle=0.15 * IN, nwrinkle=4, seed=23 + sy,
                   skip=("-z",))
    pillow_box(v, f, (xs[0] + 1.0 * IN, 0.0, zl + 0.6 * IN), (2.0 * IN, 15.8 * IN, 1.0 * IN),
               r=0.42 * IN, cell=0.3 * IN, puff=0.12 * IN, wrinkle=0.15 * IN, nwrinkle=3, seed=26, skip=("-z",))
    mesh("lcru_top_flaps", v, f, m["kapton"], lk, smooth=True, variant=var)
    # front flap, folded down over the case front and hanging below it
    v, f = [], []
    rng = random.Random(29)
    nu, nv = 22, 18
    yl, yr = P.bl(-0.6), P.bl(-11.2)
    x0f = xf + t + 1.15 * IN
    for i in range(nu + 1):
        for j in range(nv + 1):
            a, b = i / nu, j / nv                 # a across, b down the flap
            y = yl + (yr - yl) * a
            if b < 0.12:                          # roll over the top-front edge
                th = (b / 0.12) * math.pi / 2
                x = xf + t - 0.6 * IN + (x0f - xf - t + 0.6 * IN) * math.sin(th)
                z = zl + 0.15 * IN + 0.55 * IN * math.cos(th)
            else:
                bb = (b - 0.12) / 0.88
                z = zl + 0.15 * IN - bb * (z1 - z0 + 5.0 * IN)
                x = x0f + 0.6 * IN * bb * bb + 0.08 * IN * math.sin(9 * a + 4 * bb) \
                    + 0.04 * IN * rng.uniform(-1, 1) + 0.16 * IN * abs(math.sin(5.5 * a + 1.3 * bb))
            v.append(Vector((x, y, z)))
    for i in range(nu):
        for j in range(nv):
            o = i * (nv + 1) + j
            f.append((o, o + 1, o + nv + 2, o + nv + 1))
    ob = mesh("lcru_front_flap", v, f, m["kapton"], lk, smooth=True, variant=var)
    ob.modifiers.new("thick", "SOLIDIFY").thickness = 0.12 * IN
    v, f = [], []
    for y in (yl, yr):                             # white beta-cloth binding down both edges
        edge = [Vector((x0f + 0.1 * IN + 0.6 * IN * (q / 12) ** 2, y, zl + 0.15 * IN - (q / 12) * (z1 - z0 + 5.0 * IN)))
                for q in range(13)]
        polyline_tubes(v, f, edge, 0.14 * IN, 8)
    mesh("lcru_front_flap_binding", v, f, m["beta"], lk, smooth=True, variant=var)


def _lcru_panel(m, lk, xf, z1):
    xp = xf + PANEL_PROUD * IN
    y0 = P.bl(PANEL_BL[0])
    ztop = P.wl((LCRU_Z[0] + LCRU_Z[1]) / 2 + PANEL_H / 2)

    def at(u, vv, dx=0.0):
        return Vector((xp + dx, y0 + u * IN, ztop - vv * IN))

    def uvf(p):
        return ((p.y - y0) / (PANEL_W * IN), 1.0 - (ztop - p.z) / (PANEL_H * IN))
    face = _panel_material(m)
    # plate body (grey-blue sides) and its textured front face
    v, f = [], []
    bx(v, f, at(PANEL_W / 2, PANEL_H / 2, -PANEL_PROUD * IN / 2 - 0.2 * IN),
       ((PANEL_PROUD + 0.4) * IN, PANEL_W * IN, PANEL_H * IN))
    mesh("lcru_panel", v, f, m["panel"], lk)
    v = [at(0, PANEL_H, 0.005 * IN), at(PANEL_W, PANEL_H, 0.005 * IN), at(PANEL_W, 0, 0.005 * IN),
         at(0, 0, 0.005 * IN)]
    mesh("lcru_panel_face", v, [(0, 1, 2, 3)], face, lk, uv=uvf)
    # rad-temp meter: square chrome bezel, round window ring, dial face (texture-mapped disc)
    mu, mv = METER
    v, f = [], []
    bx(v, f, at(mu, mv, 0.06 * IN), (0.12 * IN, 1.45 * IN, 1.45 * IN))
    revolve(v, f, [(METER_R * IN, 0.0), (METER_R * IN + 0.1 * IN, 0.02 * IN), (METER_R * IN + 0.1 * IN, 0.1 * IN),
                   (METER_R * IN, 0.12 * IN)], axis_xform(at(mu, mv, 0.12 * IN), (1, 0, 0)), 32)
    mesh("lcru_meter_bezel", v, f, m["chrome"], lk, smooth=True)
    v, f = [], []
    for du in (-0.55, 0.55):
        for dv in (-0.55, 0.55):
            sphere(v, f, at(mu + du, mv + dv, 0.13 * IN), 0.08 * IN, 8, 4)
    for (u, vv) in SCREWS:
        sphere(v, f, at(u, vv, 0.0), 0.1 * IN, 10, 4)
    mesh("lcru_panel_screws", v, f, m["steel"], lk, smooth=True)
    v, f = [], []
    revolve(v, f, [(0.0, 0.0), (METER_R * IN, 0.0)], axis_xform(at(mu, mv, 0.13 * IN), (1, 0, 0)), 32)
    mesh("lcru_meter_dial", v, f, face, lk, uv=uvf)

    # toggle switches: black recessed collar, chrome bushing nut, lever
    v, f = [], []
    lv, lf = [], []
    for (u, vv), lever, ang in ((MONITOR_SWITCH, "paddle", (0.0, 0.35, 0.9)), (POWER_SWITCH, "bat", (0.0, 0.5, -0.85))):
        p = at(u, vv)
        revolve(v, f, [(0.22 * IN, 0.12 * IN), (0.36 * IN, 0.12 * IN), (0.42 * IN, 0.0), (0.46 * IN, -0.05 * IN)],
                axis_xform(p, (1, 0, 0)), 20)
        cyl(lv, lf, p - Vector((0.05 * IN, 0, 0)), p + Vector((0.3 * IN, 0, 0)), 0.17 * IN, 6)   # hex nut
        d = Vector(ang).normalized()
        d = (Vector((1, 0, 0)) * 0.7 + d * 0.7).normalized()
        b0 = p + Vector((0.3 * IN, 0, 0))
        if lever == "paddle":   # ribbed flat paddle (MONITOR selector)
            a2 = d.cross(Vector((1, 0, 0))).normalized()
            obox(lv, lf, b0 + d * 0.32 * IN, d, a2, d.cross(a2), (0.62 * IN, 0.34 * IN, 0.14 * IN))
            for k in range(5):
                obox(lv, lf, b0 + d * (0.12 + 0.1 * k) * IN + d.cross(a2) * 0.08 * IN, d, a2, d.cross(a2),
                     (0.03 * IN, 0.34 * IN, 0.04 * IN))
        else:
            cyl(lv, lf, b0, b0 + d * 0.6 * IN, 0.07 * IN, 10)
            sphere(lv, lf, b0 + d * 0.62 * IN, 0.1 * IN, 10, 5)
    mesh("lcru_switch_collars", v, f, m["anodized"], lk, smooth=True)
    mesh("lcru_switch_levers", lv, lf, m["chrome"], lk, smooth=False)
    # circuit breaker: black cap on a boss
    v, f = [], []
    p = at(*BREAKER)
    cyl(v, f, p, p + Vector((0.2 * IN, 0, 0)), 0.36 * IN, 20)
    cyl(v, f, p + Vector((0.2 * IN, 0, 0)), p + Vector((0.55 * IN, 0, 0)), 0.26 * IN, 20)
    mesh("lcru_breaker", v, f, m["anodized"], lk, smooth=True)
    # S-BD MODE rotary selector: large faceted pointer knob, dark bronze (00053)
    ku, kv = MODE_KNOB
    poly = [(0.0, 1.0), (0.55, 0.55), (0.72, -0.2), (0.38, -0.78), (-0.38, -0.78), (-0.72, -0.2), (-0.55, 0.55)]
    rot = math.radians(-14)
    base = len(v)
    v, f = [], []
    rings = []
    for dx, sc in ((0.02, 1.25), (0.5, 1.25), (0.85, 1.0)):
        ring = []
        for (pu, pv) in poly:
            uu = pu * math.cos(rot) - pv * math.sin(rot)
            vv = pu * math.sin(rot) + pv * math.cos(rot)
            ring.append(at(ku + uu * sc, kv - vv * sc, dx * IN))
        rings.append(ring)
    n = len(poly)
    for ring in rings:
        v.extend(ring)
    for i in range(len(rings) - 1):
        for j in range(n):
            j1 = (j + 1) % n
            f.append((i * n + j, i * n + j1, (i + 1) * n + j1, (i + 1) * n + j))
    f.append(tuple((len(rings) - 1) * n + j for j in range(n)))
    f.append(tuple(reversed(range(n))))
    mesh("lcru_mode_knob", v, f, m["knob"], lk)
    v, f = [], []
    tip = Vector((-math.sin(rot) * 0.55, math.cos(rot) * 0.55))
    obox(v, f, at(ku + tip.x * 1.0, kv - tip.y * 1.0, 0.86 * IN),
         Vector((0, tip.x, -tip.y)).normalized(), Vector((0, tip.y, tip.x)).normalized(), (1, 0, 0),
         (0.4 * IN, 0.04 * IN, 0.01 * IN))
    mesh("lcru_knob_index", v, f, m["white"], lk)
    # zero-g power / GCTA connector: machined aluminium receptacle with a cut-away guard
    p = at(*CONNECTOR)
    LCRU_PORTS["tv"] = p + Vector((0.6 * IN, 0, 0))
    v, f = [], []
    revolve(v, f, [(0.0, 0.02 * IN), (0.5 * IN, 0.02 * IN), (0.5 * IN, 0.35 * IN), (0.62 * IN, 0.4 * IN),
                   (0.62 * IN, 0.75 * IN), (0.78 * IN, 0.8 * IN), (0.78 * IN, 0.95 * IN), (0.66 * IN, 0.98 * IN),
                   (0.66 * IN, 0.6 * IN), (0.0, 0.6 * IN)], axis_xform(p, (1, 0, 0)), 32)
    mesh("lcru_gcta_connector", v, f, m["al_spun"], lk, smooth=True)


def _lcru_details(m, lk, xs, xf, z0, z1, cx, cz, t):
    # S-band ports: HGA (red band) and LGA (blue band) low on the front face, right of the panel,
    # cables hanging down (QTU 00046)
    v, f = [], []
    rv_, rf_ = [], []
    bv, bf = [], []
    ports = {}
    for name, y, mat_list in (("hga", -6.9, (rv_, rf_)), ("lga", -5.3, (bv, bf))):
        p = Vector((xf + t + 0.15 * IN, y * IN, z0 + 1.6 * IN))
        cyl(v, f, p - Vector((0.3 * IN, 0, 0)), p + Vector((0.55 * IN, 0, 0)), 0.3 * IN, 14)
        cyl(mat_list[0], mat_list[1], p + Vector((0.12 * IN, 0, 0)), p + Vector((0.35 * IN, 0, 0)), 0.33 * IN, 14)
        ports[name] = p + Vector((0.55 * IN, 0, 0))
    # LGA castellated hand-carry receptacle, top right-front corner (HSI-481184 1.3.3)
    rc = Vector((xf - 1.6 * IN, P.bl(-9.0), z1 + t))
    revolve(v, f, [(0.5 * IN, 0), (0.7 * IN, 0), (0.7 * IN, 1.0 * IN), (0.5 * IN, 1.0 * IN)], axis_xform(rc, UP), 16)
    mesh("lcru_connectors", v, f, m["chrome"], lk, smooth=True)
    mesh("lcru_hga_port_band", rv_, rf_, m["red"], lk, smooth=True)
    mesh("lcru_lga_port_band", bv, bf, m["blue"], lk, smooth=True)
    for name in ("hga", "lga"):  # printed cloth tags above the ports (NASM-NASM2020-00046)
        p0 = ports[name] + Vector((0.0, 0.0, 1.05 * IN))
        pt, nn = LCRU_SURF[0](Vector((xf + t, p0.y, p0.z)))
        placard(f"lcru_label_{name}", f"eq_label_{name}", pt, (0, 1, 0), (0, 0, 1), 1.0, 0.42, lk, lift=0.06 * IN)
    LCRU_PORTS.update(ports)
    # carrying handle on the right end (QTU 00053), white, in a recess of the blanket
    v, f = [], []
    ye = -(LCRU_BL * IN + t)
    hp = [Vector((cx - 2.6 * IN, ye + 0.2 * IN, cz)), Vector((cx - 2.6 * IN, ye - 1.5 * IN, cz)),
          Vector((cx + 2.6 * IN, ye - 1.5 * IN, cz)), Vector((cx + 2.6 * IN, ye + 0.2 * IN, cz))]
    polyline_tubes(v, f, hp, 0.25 * IN, 12)
    mesh("lcru_handle", v, f, m["teflon"], lk, smooth=True)
    # VHF omni antenna: flat spring-steel blade on the top, near the front (QTU 00053)
    v, f = [], []
    ab = Vector((xf - 1.6 * IN, P.bl(-2.4), z1 + t))
    bx(v, f, ab + Vector((0, 0, 6.5 * IN)), (0.05 * IN, 0.55 * IN, 13.0 * IN))
    cyl(v, f, ab - UP * 0.2 * IN, ab + Vector((0, 0, 0.5 * IN)), 0.3 * IN, 12)
    mesh("lcru_vhf_antenna", v, f, m["steel"], lk)
    # mounting clevises under the case onto the chassis fittings (BL +/-9.85)
    v, f = [], []
    for s in (1, -1):
        bx(v, f, W(FIT_STA, s * LCRU_BL, (FIT_WL + LCRU_Z[0]) / 2 + 0.2), (1.2 * IN, 0.6 * IN, (LCRU_Z[0] - FIT_WL + 0.4) * IN))
        cyl(v, f, W(FIT_STA, s * (LCRU_BL - 0.5), FIT_WL + 0.6), W(FIT_STA, s * (LCRU_BL + 0.6), FIT_WL + 0.6), 0.2 * IN, 8)
    # latch bracket hanging over the front face beside the panel (00053)
    bv_, bf_ = [], []
    yb = P.bl(PANEL_BL[0] - 0.9)
    obox(bv_, bf_, Vector((xf + t + 0.3 * IN, yb, z1 + 0.2 * IN)), (1, 0, 0), (0, 1, 0), (0, 0, 1),
         (0.12 * IN, 0.75 * IN, 2.6 * IN))
    obox(bv_, bf_, Vector((xf + t - 0.15 * IN, yb, z1 + t + 1.05 * IN)), (1, 0, 0), (0, 1, 0), (0, 0, 1),
         (1.0 * IN, 0.75 * IN, 0.12 * IN))
    obox(bv_, bf_, Vector((xf + t + 0.3 * IN, yb + 0.25 * IN, z1 - 1.4 * IN)), (1, 0, 0), (0, 1, 0), (0, 0, 1),
         (0.12 * IN, 1.2 * IN, 0.7 * IN))
    mesh("lcru_latch_bracket", bv_, bf_, m["al"], lk)
    mesh("lcru_mounts", v, f, m["al"], lk)


LCRU_PORTS = {}
LCRU_SURF = []


# ---------------------------------------------------------------- high-gain antenna

HGA_D = 36.0          # reflector diameter (HSI-481184 1.3.4)
HGA_F = 11.0          # focal length, f/D ~0.3 (depth ~7.4 in, scaled from QTU 00051/00052)
HGA_RIBS = 12         # counted on QTU 00050
HGA_EL0 = 50.0        # rest elevation (Earth ~50 deg high at Taurus-Littrow)
BALL_WL = 143.0       # ball joint, mast fully extended (~61 in above ground: sight at eye level)
HGA_DRIVE = {"kind": "position", "stiffness": 300.0, "damping": 30.0, "max_force": 30.0, "target": 0.0}


def build_hga(ch, m, lcru):
    base = W(FIT_STA, HGA_BL, FIT_WL)
    mast = link("hga_mast", base, ch, {"type": "fixed", "role": "hga_mast", "payload": True}, mass=1.2)
    # lower mast: tapered shaft into the fitting, bayonet collar, tube, extension lock wheel
    v, f = [], []
    revolve(v, f, [(0.2 * IN, -2.0 * IN), (0.36 * IN, 0.0), (0.62 * IN, 0.0), (0.62 * IN, 26.5 * IN),
                   (0.5 * IN, 26.5 * IN)], axis_xform(base, UP), 16)
    mesh("hga_mast_tube", v, f, m["al_spun"], mast, smooth=True)
    v, f = [], []
    for z0, z1, r in ((1.4, 3.0, 0.82), (25.0, 27.0, 0.82), (14.0, 14.6, 0.72)):
        cyl(v, f, base + UP * (z0 * IN), base + UP * (z1 * IN), r * IN, 20)
    # extension lock: side clamp block with a four-lobed star knob (QTU 00052)
    kc = base + UP * (26.0 * IN)
    bx(v, f, kc + Vector((0, -0.95, 0)) * IN, (1.0 * IN, 0.8 * IN, 1.6 * IN))
    cyl(v, f, kc + Vector((0, -1.3, 0)) * IN, kc + Vector((0, -2.1, 0)) * IN, 0.22 * IN, 10)
    for i in range(4):
        t = TAU * i / 4 + 0.4
        p = kc + Vector((math.cos(t) * 0.5, -2.3, math.sin(t) * 0.5)) * IN
        cyl(v, f, p - Vector((0, 0.25, 0)) * IN, p + Vector((0, 0.25, 0)) * IN, 0.36 * IN, 12)
    cyl(v, f, kc + Vector((0, -2.05, 0)) * IN, kc + Vector((0, -2.55, 0)) * IN, 0.55 * IN, 16)
    mesh("hga_mast_collars", v, f, m["al"], mast, smooth=True)
    v, f = [], []
    cyl(v, f, base + UP * (1.0 * IN) + Vector((0.6 * IN, 0, 0)), base + UP * (2.0 * IN) + Vector((0.63 * IN, 0, 0)), 0.1 * IN, 6)
    mesh("hga_alignment_mark", v, f, m["red"], mast)
    # RF cable from the LCRU HGA port (red) up the mast, clipped at the lock wheel
    hc = LCRU_PORTS["hga"]
    pts = cable_pts(hc, base + UP * (8 * IN) + Vector((0.7 * IN, 0, 0)), 8.0 * IN, 16, side=(0.05, 0.0, 0.0))
    pts += [base + UP * (z * IN) + Vector((0.75 * IN, 0.2 * IN, 0)) for z in (12, 20, 25.5)]
    v, f = [], []
    polyline_tubes(v, f, pts, 0.16 * IN, 8)
    mesh("hga_cable", v, f, m["cable"], mast, smooth=True)

    # upper mast (telescoping, friction-locked by the lock wheel; rest = fully extended)
    top = W(FIT_STA, HGA_BL, BALL_WL)
    ext = link("hga_mast_ext", base + UP * (26.8 * IN), mast, {
        "type": "prismatic", "axis": (0.0, 0.0, 1.0), "limit": (-0.30, 0.0), "role": "hga_telescope",
        "payload": True,
        "drive": {"kind": "position", "stiffness": 5.0e4, "damping": 500.0, "max_force": 300.0, "target": 0.0}},
        mass=0.6)
    v, f = [], []
    cyl(v, f, base + UP * (14.0 * IN), top - UP * (1.6 * IN), 0.5 * IN, 16)
    cyl(v, f, top - UP * (2.4 * IN), top - UP * (0.7 * IN), 0.75 * IN, 16)   # ball socket cup
    mesh("hga_mast_upper", v, f, m["al_spun"], ext, smooth=True)
    v, f = [], []
    cyl(v, f, base + UP * (27.0 * IN), base + UP * (27.5 * IN), 0.53 * IN, 16)
    mesh("hga_ext_indicator", v, f, m["red"], ext, smooth=True)
    v, f = [], []  # coiled strain-relief spring on the RF cable below the ball joint (QTU 00051)
    hel = []
    for i in range(121):
        t = TAU * 7 * i / 120
        hel.append(top - UP * ((7.5 - 5.0 * i / 120) * IN) + Vector((0.95 + 0.32 * math.cos(t), 0.32 * math.sin(t), 0)) * IN)
    polyline_tubes(v, f, hel, 0.06 * IN, 6)
    cyl(v, f, top - UP * 8.5 * IN + Vector((0.95, 0, 0)) * IN, top - UP * 1.6 * IN + Vector((0.95, 0, 0)) * IN, 0.14 * IN, 8)
    mesh("hga_cable_spring", v, f, m["steel"], ext, smooth=True)

    # ball joint: azimuth about the mast, then elevation (lock bar + friction knob)
    az = link("hga_az", top, ext, {
        "type": "revolute", "axis": (0.0, 0.0, 1.0), "limit": (-math.pi, math.pi), "role": "hga_azimuth",
        "payload": True, "drive": dict(HGA_DRIVE)}, mass=0.35)
    v, f = [], []
    sphere(v, f, top, 0.9 * IN)
    cyl(v, f, top - UP * 0.9 * IN, top - UP * 0.5 * IN, 1.0 * IN, 16)
    mesh("hga_ball", v, f, m["al"], az, smooth=True)
    v, f = [], []
    kn = top + Vector((0, -1.0, 0)) * IN - UP * 0.6 * IN  # friction knob on the right
    cyl(v, f, kn, kn + Vector((0, -1.3, 0)) * IN, 0.2 * IN, 10)
    revolve(v, f, [(0.0, 0.0), (0.75 * IN, 0.0), (0.75 * IN, 0.45 * IN), (0.0, 0.45 * IN)],
            axis_xform(kn + Vector((0, -1.3, 0)) * IN, (0, -1, 0)), 8)
    lb = top + Vector((-1.0, 0.0, 0.0)) * IN  # lock bar lever
    cyl(v, f, lb, lb + Vector((-2.4, 0.4, -1.4)) * IN, 0.14 * IN, 8)
    mesh("hga_lock_controls", v, f, m["al"], az, smooth=False)

    d = Vector((math.cos(rad(HGA_EL0)), 0.0, math.sin(rad(HGA_EL0))))
    el = link("hga_el", top, az, {
        "type": "revolute", "axis": (0.0, -1.0, 0.0), "limit": (-rad(HGA_EL0), rad(90 - HGA_EL0)),
        "role": "hga_elevation", "payload": True, "drive": dict(HGA_DRIVE)}, mass=0.5)
    H = top + d * (2.6 * IN)            # hinge plate
    v, f = [], []
    cyl(v, f, top + d * (0.6 * IN), H, 0.55 * IN, 14)
    obox(v, f, H, d, Vector((0, 1, 0)), d.cross(Vector((0, 1, 0))), (0.5 * IN, 2.6 * IN, 1.8 * IN))
    mesh("hga_hinge_lower", v, f, m["al"], el)
    # pointing handle: socketed in the positioning mechanism, angled down-aft-left (Fig 4-8)
    hdir = (-d + Vector((0.0, 0.45, -0.55))).normalized()
    v, f = [], []
    hs = top + d * (1.0 * IN) + Vector((0, 0.6, 0)) * IN
    cyl(v, f, hs, hs + hdir * (26.0 * IN), 0.3 * IN, 12)
    cyl(v, f, hs + hdir * (25.0 * IN), hs + hdir * (26.5 * IN), 0.38 * IN, 12)
    mesh("hga_pointing_handle", v, f, m["al_spun"], el, smooth=True)
    v, f = [], []
    cyl(v, f, hs + hdir * (1.5 * IN), hs + hdir * (1.5 * IN) + Vector((0, 0.9, -0.3)) * IN, 0.12 * IN, 8)
    mesh("hga_handle_toggle", v, f, m["red"], el)

    fold = link("hga_dish", H, el, {
        "type": "revolute", "axis": (0.0, 1.0, 0.0), "limit": (0.0, math.pi), "role": "hga_fold",
        "payload": True,
        "drive": {"kind": "position", "stiffness": 500.0, "damping": 50.0, "max_force": 60.0, "target": 0.0}},
        mass=1.9)
    _hga_reflector(m, fold, H, d)
    return mast


def _hga_reflector(m, lk, H, d):
    a = Vector((0.0, 1.0, 0.0))             # reflector lateral axis
    b = d.cross(a)                          # reflector 'up' axis
    R = HGA_D / 2 * IN
    f_ = HGA_F * IN
    V = H + d * (6.2 * IN)                  # reflector vertex (hub face)
    r0 = 1.7 * IN
    # hinge plate upper half + latch, deployment collar, hub
    v, f = [], []
    obox(v, f, H + d * (0.55 * IN), d, a, b, (0.5 * IN, 2.6 * IN, 1.8 * IN))
    cyl(v, f, H + d * 0.8 * IN, V - d * 1.2 * IN, 0.5 * IN, 14)
    cyl(v, f, H + d * 2.2 * IN, H + d * 3.4 * IN, 1.25 * IN, 20)       # deployment collar
    cyl(v, f, V - d * 1.4 * IN, V + d * 0.3 * IN, 1.8 * IN, 24)        # rib hub
    for k in range(HGA_RIBS):                                          # rib root clevises
        t = TAU * k / HGA_RIBS
        u = a * math.cos(t) + b * math.sin(t)
        obox(v, f, V - d * 0.5 * IN + u * 1.9 * IN, d, u, d.cross(u), (1.2 * IN, 0.8 * IN, 0.35 * IN))
    mesh("hga_hub", v, f, m["al"], lk)
    # 12 ribs (riveted aluminium channels) on the paraboloid, deployment struts from the collar
    v, f = [], []
    sv, sf = [], []
    rv, rf = [], []
    tips = []
    for k in range(HGA_RIBS):
        t = TAU * k / HGA_RIBS
        u = a * math.cos(t) + b * math.sin(t)
        tang = d.cross(u).normalized()
        pts = []
        for i in range(13):
            r = r0 + (R - r0) * i / 12
            pts.append(V + d * (r * r / (4 * f_) - 0.2 * IN) + u * r)
        for p0, p1 in zip(pts, pts[1:]):   # flat riveted channel, 0.45 x 0.22 in (QTU 00051)
            dn = (p1 - p0).normalized()
            nr_ = dn.cross(tang).normalized()
            obox(v, f, (p0 + p1) / 2, dn, tang, nr_, ((p1 - p0).length + 0.02 * IN, 0.45 * IN, 0.22 * IN))
            for q in (0.25, 0.75):
                sphere(rv, rf, p0.lerp(p1, q) - nr_ * 0.11 * IN, 0.035 * IN, 6, 3)
        tips.append(pts[-1])
        r = 7.0 * IN
        cyl(sv, sf, H + d * 2.8 * IN + u * 1.2 * IN, V + d * (r * r / (4 * f_) - 0.4 * IN) + u * r, 0.1 * IN, 6)
    mesh("hga_ribs", v, f, m["al"], lk, smooth=True)
    mesh("hga_struts", sv, sf, m["al"], lk, smooth=True)
    mesh("hga_rib_rivets", rv, rf, m["al"], lk, smooth=True)
    v, f = [], []
    for p in tips:
        sphere(v, f, p, 0.3 * IN, 10, 6)
    mesh("hga_rib_tips", v, f, m["teflon"], lk, smooth=True)
    # knitted mesh: scalloped between ribs (pulled in towards the chords at the rim)
    nr, per = 14, 8
    ns = HGA_RIBS * per
    v, f = [], []
    for j in range(ns):
        t = TAU * j / ns
        u = a * math.cos(t) + b * math.sin(t)
        sc = abs(math.sin(HGA_RIBS / 2 * t))
        for i in range(nr + 1):
            r = r0 + (R - r0) * i / nr
            rr = r * (1 - 0.045 * (r / R) ** 2 * sc)
            v.append(V + d * (rr * rr / (4 * f_) - 0.05 * IN * sc) + u * rr)
    for j in range(ns):
        j1 = (j + 1) % ns
        for i in range(nr):
            f.append((j * (nr + 1) + i, j1 * (nr + 1) + i, j1 * (nr + 1) + i + 1, j * (nr + 1) + i + 1))
    mesh("hga_reflector_mesh", v, f, m["hga_mesh"], lk, smooth=True,
         uv=lambda p: ((p - V).dot(a), (p - V).dot(b)))   # planar along the boresight, metres
    v, f = [], []   # rim edge cord between the rib tips
    rim = []
    for j in range(ns + 1):
        t = TAU * (j % ns) / ns
        u = a * math.cos(t) + b * math.sin(t)
        sc = abs(math.sin(HGA_RIBS / 2 * t))
        rr = R * (1 - 0.045 * sc)
        rim.append(V + d * (rr * rr / (4 * f_) - 0.05 * IN * sc) + u * rr)
    polyline_tubes(v, f, rim, 0.05 * IN, 6)
    mesh("hga_rim_cord", v, f, m["webbing"], lk, smooth=True)
    # cup-helical feed on its support tube; gold scalloped radome disc at the outer end
    v, f = [], []
    # feed support tube ~18 in long (scaled off QTU 00051/00052), cup can at the outer end
    cyl(v, f, V, V + d * (17.8 * IN), 0.42 * IN, 16)
    revolve(v, f, [(0.42 * IN, 0.0), (1.75 * IN, 0.15 * IN), (1.8 * IN, 0.3 * IN), (1.8 * IN, 2.4 * IN),
                   (1.65 * IN, 2.4 * IN), (1.65 * IN, 0.45 * IN), (0.42 * IN, 0.35 * IN)],
            axis_xform(V + d * (17.6 * IN), d), 28)
    mesh("hga_feed", v, f, m["al_spun"], lk, smooth=True)
    v, f = [], []
    nl = 8
    prof = []
    for j in range(nl * 6):
        t = TAU * j / (nl * 6)
        rr = (2.0 + 0.45 * math.cos(nl * t)) * IN
        prof.append(rr)
    c0 = V + d * (20.05 * IN)
    base = len(v)
    v.append(c0)
    for j, rr in enumerate(prof):
        t = TAU * j / len(prof)
        v.append(c0 + (a * math.cos(t) + b * math.sin(t)) * rr)
    for j in range(len(prof)):
        f.append((base, base + 1 + j, base + 1 + (j + 1) % len(prof)))
    mesh("hga_feed_disc", v, f, m["kapton_cu"], lk)
    # optical boresight sight: black hood cylinder alongside the hub (Fig 4-9)
    v, f = [], []
    sc = H + d * (3.0 * IN) - a * (3.0 * IN) - b * (0.6 * IN)
    cyl(v, f, sc - d * 2.0 * IN, sc + d * 2.4 * IN, 1.1 * IN, 20)
    mesh("hga_sight", v, f, m["anodized"], lk, smooth=True)
    v, f = [], []
    cyl(v, f, sc + d * 2.4 * IN, sc + d * 3.0 * IN, 0.75 * IN, 16)
    obox(v, f, H + d * 3.0 * IN - a * 1.6 * IN, d, a, b, (1.0 * IN, 2.2 * IN, 0.4 * IN))
    mesh("hga_sight_mount", v, f, m["al"], lk, smooth=True)


# ---------------------------------------------------------------- low-gain antenna

LGA_POS = (47.6, 8.6)    # left inboard handhold socket (HSI-481184 1.3.3), position estimated
LGA_EL0 = 50.0


def build_lga(ch, m):
    base = W(LGA_POS[0], LGA_POS[1], FRAME_TOP + 1.0)
    v, f = [], []
    revolve(v, f, [(0.45 * IN, -1.0 * IN), (0.7 * IN, -1.0 * IN), (0.7 * IN, 3.0 * IN), (0.45 * IN, 3.0 * IN)],
            axis_xform(base, UP), 14)
    mesh("lga_socket", v, f, m["al"], ch, smooth=True)
    az = link("lga_staff", base, ch, {
        "type": "revolute", "axis": (0.0, 0.0, 1.0), "limit": (-math.pi, math.pi), "role": "lga_azimuth",
        "payload": True, "drive": {"kind": "position", "stiffness": 60.0, "damping": 6.0, "max_force": 8.0,
                                   "target": 0.0}}, mass=0.6)
    top = base + UP * (37.0 * IN)
    v, f = [], []
    cyl(v, f, base - UP * 0.8 * IN, top, 0.38 * IN, 14)
    mesh("lga_staff", v, f, m["al_spun"], az, smooth=True)
    v, f = [], []
    cyl(v, f, base + UP * 24.0 * IN, base + UP * 29.5 * IN, 0.6 * IN, 16)     # azimuth grip
    for i in range(10):
        cyl(v, f, base + UP * (24.3 + 0.5 * i) * IN, base + UP * (24.5 + 0.5 * i) * IN, 0.66 * IN, 16)
    cyl(v, f, base + UP * 33.0 * IN, base + UP * 34.2 * IN, 0.55 * IN, 12)   # spline collar
    mesh("lga_grip", v, f, m["gray"], az, smooth=True)
    pv = top + UP * 0.9 * IN
    v, f = [], []
    obox(v, f, top + UP * 0.45 * IN, (1, 0, 0), (0, 1, 0), (0, 0, 1), (0.8 * IN, 1.2 * IN, 0.9 * IN))
    mesh("lga_pivot_yoke", v, f, m["al"], az)

    d = Vector((math.cos(rad(LGA_EL0)), 0.0, math.sin(rad(LGA_EL0))))
    el = link("lga_antenna", pv, az, {
        "type": "revolute", "axis": (0.0, -1.0, 0.0), "limit": (-rad(LGA_EL0), rad(90 - LGA_EL0)),
        "role": "lga_elevation", "payload": True,
        "drive": {"kind": "position", "stiffness": 60.0, "damping": 6.0, "max_force": 8.0, "target": 0.0}},
        mass=0.5)
    v, f = [], []
    cyl(v, f, pv - Vector((0, 0.7, 0)) * IN, pv + Vector((0, 0.7, 0)) * IN, 0.8 * IN, 20)  # elevation rotor
    cyl(v, f, pv, pv + d * 1.6 * IN, 0.3 * IN, 12)                                  # release button collar
    mesh("lga_rotor", v, f, m["al"], el, smooth=True)
    v, f = [], []
    for i in range(10):  # red elevation scale ticks (0..90 in 10 deg steps)
        t = rad(10 * i)
        p = pv + Vector((math.cos(t) * 0.7, 0.72, math.sin(t) * 0.7)) * IN
        bx(v, f, p, (0.05 * IN, 0.02 * IN, 0.05 * IN))
    mesh("lga_scale", v, f, m["red"], el)
    # perforated reflector cup (5.13 in dia, white), helix feed radome (1.54 in), total 9.32 in
    c0 = pv + d * 1.6 * IN
    rr, L = 5.13 / 2 * IN, 3.0 * IN
    a, b = perp(d)
    v, f = [], []
    ns, nl = 24, 6
    o0 = len(v)
    for i in range(nl + 1):
        for s in range(ns):
            t = TAU * s / ns
            v.append(c0 + d * (L * i / nl) + (a * math.cos(t) + b * math.sin(t)) * rr)
    for i in range(nl):
        for s in range(ns):
            if 0 < i < nl - 1 and (s + i) % 2 == 0 and i % 2 == 1:
                continue  # perforation holes
            s1 = (s + 1) % ns
            f.append((o0 + i * ns + s, o0 + i * ns + s1, o0 + (i + 1) * ns + s1, o0 + (i + 1) * ns + s))
    cyl(v, f, c0, c0 + d * 0.15 * IN, rr, ns)                     # cup base
    revolve(v, f, [(0.4 * IN, 0.0), (0.77 * IN, 0.3 * IN), (0.77 * IN, 9.0 * IN), (0.6 * IN, 9.32 * IN),
                   (0.0, 9.32 * IN)], axis_xform(c0, d), 18)
    mesh("lga_reflector_feed", v, f, m["white"], el, smooth=False)
    v, f = [], []  # blue-coded RF cable from the antenna down the staff
    pts = [c0 - a * 0.9 * IN, pv - Vector((0.6, 0.0, 1.5)) * IN, base + UP * 30 * IN + Vector((0.45, 0, 0)) * IN]
    polyline_tubes(v, f, pts, 0.14 * IN, 8)
    mesh("lga_cable", v, f, m["cable"], el, smooth=True)


# ---------------------------------------------------------------- GCTA TV camera + TCU

TCU_Z = (116.0, 122.5)
TILT_WL = 132.0
TV_DRIVE = {"kind": "position", "stiffness": 50.0, "damping": 5.0,
            "max_force": 2.0}   # 18 in-lb (2.0 N*m) slip clutch, GCTA report V-B


# GCTA TV camera and its control unit flew on A15-A17; the Smithsonian display vehicle has none (NASM2020-00046,
# 00050, 00051 show the bare HGA mast). The links, joints and camera sensor stay so every variant shares one
# articulation; only the visuals are switched.
FLIGHT_TV = "Mission:a15,a17,a17_repair"


def build_tv(ch, m):
    base = W(FIT_STA, TV_BL, FIT_WL)
    mount = link("tcu", base, ch, {"type": "fixed", "role": "tv_mount", "payload": True}, mass=3.8)
    # short staff with swing-away hinge (lets the LCRU come off without removing the GCTA)
    v, f = [], []
    revolve(v, f, [(0.2 * IN, -2.0 * IN), (0.36 * IN, 0.0), (0.6 * IN, 0.0), (0.6 * IN, (TCU_Z[0] - FIT_WL) * IN)],
            axis_xform(base, UP), 16)
    hz = base + UP * (4.0 * IN)
    obox(v, f, hz, (1, 0, 0), (0, 1, 0), (0, 0, 1), (1.6 * IN, 1.0 * IN, 1.2 * IN))
    mesh("tv_staff", v, f, m["al_spun"], mount, smooth=False, variant=FLIGHT_TV)
    v, f = [], []
    cyl(v, f, hz + Vector((0, -0.7, 0)) * IN, hz + Vector((0, 0.7, 0)) * IN, 0.25 * IN, 10)
    mesh("tv_staff_hinge_pin", v, f, m["steel"], mount, smooth=True, variant=FLIGHT_TV)
    # TCU electronics box: gold Kapton blanket, bare aluminium connector face (as16-117-18754)
    c = W(FIT_STA, TV_BL, (TCU_Z[0] + TCU_Z[1]) / 2)
    sz = (7.0 * IN, 6.0 * IN, (TCU_Z[1] - TCU_Z[0]) * IN)
    v, f = [], []
    crinkle_box(v, f, c, sz, amp=0.012 * IN, pillow=0.1 * IN, cell=0.4 * IN, seed=31, skip=("+x",), fold=0.05 * IN)
    mesh("tcu_blanket", v, f, m["kapton"], mount, variant=FLIGHT_TV)
    v, f = [], []
    bx(v, f, c + Vector((sz[0] / 2 + 0.05 * IN, 0, 0)), (0.12 * IN, sz[1], sz[2]))
    mesh("tcu_face", v, f, m["al"], mount, variant=FLIGHT_TV)
    v, f = [], []
    for dy, dz in ((-1.6, 1.2), (0.3, 1.2), (1.8, -1.0)):
        p = c + Vector((sz[0] / 2 + 0.1 * IN, dy * IN, dz * IN))
        cyl(v, f, p, p + Vector((0.8 * IN, 0, 0)), 0.42 * IN, 14)
    mesh("tcu_connectors", v, f, m["chrome"], mount, smooth=True, variant=FLIGHT_TV)
    # TV video cable TCU -> LCRU (TV/EXT PWR connector)
    v, f = [], []
    p0 = c + Vector((sz[0] / 2 + 0.9 * IN, -1.6 * IN, 1.2 * IN))
    p1 = LCRU_PORTS["tv"]
    polyline_tubes(v, f, cable_pts(p0, p1, 4.0 * IN, 12, side=(0.06, 0.0, 0.0)), 0.16 * IN, 8)
    mesh("tv_cable", v, f, m["cable"], mount, smooth=True, variant=FLIGHT_TV)

    # pan: azimuth drive column on top of the TCU
    pax = W(FIT_STA, TV_BL, TCU_Z[1])
    pan = link("tv_pan", pax, mount, {
        "type": "revolute", "axis": (0.0, 0.0, -1.0), "limit": (-rad(134), rad(214)), "role": "tv_pan",
        "payload": True, "drive": dict(TV_DRIVE, target=0.0)}, mass=0.9)
    v, f = [], []
    crinkle_cyl(v, f, pax, pax + UP * (6.8 * IN), 2.0 * IN, 0.05 * IN, seed=33, segs=36, nlen=16)
    mesh("tv_pan_column", v, f, m["kapton"], pan, smooth=False, variant=FLIGHT_TV)
    v, f = [], []
    cyl(v, f, pax + UP * 6.8 * IN, pax + UP * 7.3 * IN, 2.1 * IN, 22)
    # elevation drive bracket and motor housing on the left side of the camera (as16-117-18754)
    tax = pax + UP * ((TILT_WL - TCU_Z[1]) * IN)
    obox(v, f, pax + UP * 8.0 * IN + Vector((0, 2.9, 0)) * IN, (1, 0, 0), (0, 1, 0), (0, 0, 1),
         (2.6 * IN, 0.3 * IN, 3.4 * IN))
    mesh("tv_yoke", v, f, m["al"], pan, variant=FLIGHT_TV)
    v, f = [], []
    cyl(v, f, tax + Vector((0, 3.0, 0)) * IN, tax + Vector((0, 4.6, 0)) * IN, 1.15 * IN, 20)
    mesh("tv_tilt_motor", v, f, m["gray"], pan, smooth=True, variant=FLIGHT_TV)

    tilt = link("tv_camera", tax, pan, {
        "type": "revolute", "axis": (0.0, -1.0, 0.0), "limit": (-rad(45), rad(85)), "role": "tv_tilt",
        "payload": True, "drive": dict(TV_DRIVE, target=0.0)}, mass=3.6)
    _tv_camera(m, tilt, tax)
    return mount


def _tv_camera(m, lk, tax):
    """RCA colour TV camera (CTV): 455 cu in body ~15 x 5.5 x 5.5 in, zoom lens, sunshade."""
    L, Wd, Hh = 15.2 * IN, 5.6 * IN, 5.0 * IN     # 426 cu in body + lens/hood ~ 455 (Table 1-2)
    cc = tax + Vector((1.3 * IN, 0.0, 2.9 * IN))
    v, f = [], []
    crinkle_box(v, f, cc, (L, Wd, Hh), amp=0.012 * IN, pillow=0.12 * IN, cell=0.4 * IN, seed=41, skip=("+x",), fold=0.05 * IN)
    mesh("tv_camera_blanket", v, f, m["kapton"], lk, variant=FLIGHT_TV)
    v, f = [], []
    # front lens plate; the rear is blanket-wrapped with the connector poking through (18754)
    bx(v, f, cc + Vector(((L / 2 + 0.05 * IN), 0, 0)), (0.12 * IN, Wd + 0.1 * IN, Hh + 0.1 * IN))
    # cradle under the camera, side plate to the tilt motor, white tape top strip
    bx(v, f, cc - Vector((0, 0, Hh / 2 + 0.15 * IN)), (6.0 * IN, Wd + 0.4 * IN, 0.25 * IN))
    bx(v, f, tax + Vector((0.6 * IN, 2.95 * IN, 1.4 * IN)), (3.4 * IN, 0.25 * IN, 3.6 * IN))
    mesh("tv_camera_plates", v, f, m["al"], lk, variant=FLIGHT_TV)
    v, f = [], []
    for du, dz in ((-0.9, 2.6), (0.4, 2.6), (1.7, 2.6), (-0.9, 0.4), (1.7, 0.4)):
        p = tax + Vector((du * IN, 3.08 * IN, dz * IN))
        cyl(v, f, p, p + Vector((0, 0.18 * IN, 0)), 0.14 * IN, 6)
    mesh("tv_camera_bolts", v, f, m["steel"], lk, variant=FLIGHT_TV)
    v, f = [], []
    bx(v, f, cc + Vector((-1.0 * IN, 0, Hh / 2 + 0.12 * IN)), (L - 3.0 * IN, 1.2 * IN, 0.05 * IN))
    mesh("tv_camera_tape", v, f, m["beta"], lk, variant=FLIGHT_TV)
    # rear connector + cable
    v, f = [], []
    rp = cc + Vector((-(L / 2 + 0.1 * IN), -1.2 * IN, 1.5 * IN))
    revolve(v, f, [(0.45 * IN, 0), (0.45 * IN, 0.7 * IN), (0.3 * IN, 0.9 * IN), (0.25 * IN, 1.4 * IN)],
            axis_xform(rp, (-1, 0, 0)), 14)
    mesh("tv_camera_connector", v, f, m["chrome"], lk, smooth=True, variant=FLIGHT_TV)
    v, f = [], []
    pts = cable_pts(rp + Vector((-1.4 * IN, 0, 0)), tax + Vector((-2.0, -2.2, -8.5)) * IN, 2.0 * IN, 8,
                    side=(-0.05, -0.02, 0.0))
    polyline_tubes(v, f, pts, 0.16 * IN, 8)
    mesh("tv_camera_cable", v, f, m["cable"], lk, smooth=True, variant=FLIGHT_TV)
    # zoom lens barrel and the flared, bent-sheet sunshade (as16-117-18754, press kit F5)
    lf = cc + Vector((L / 2 + 0.1 * IN, 0, 0.4 * IN))
    v, f = [], []
    revolve(v, f, [(1.55 * IN, 0.0), (1.55 * IN, 1.4 * IN), (1.35 * IN, 1.5 * IN), (1.35 * IN, 2.2 * IN)],
            axis_xform(lf, (1, 0, 0)), 24)
    mesh("tv_lens_barrel", v, f, m["anodized"], lk, smooth=True, variant=FLIGHT_TV)
    # optical axis: out of the lens front along +X, image up = +Z (rest pose, tilt 0)
    sensor_camera("tv_camera_optics", lk, lf + Vector((2.2 * IN, 0, 0)), (1, 0, 0), (0, 0, 1), P.TV_CAMERA)
    v, f = [], []
    rings = []
    for i in range(7):
        t = i / 6
        x = lf.x + (2.0 + 4.6 * t) * IN
        hw = (1.8 + 1.6 * t ** 1.3) * IN
        zb = lf.z - (1.8 + 0.9 * t) * IN
        zt = lf.z + (1.8 + 0.6 * t + 2.0 * t ** 2.2) * IN       # top visor curls up
        rows = []
        for k in range(9):  # U section: left wall, curved top, right wall (open bottom)
            s = k / 8
            ang = math.pi * s
            y = -hw * math.cos(ang)
            z = zb + (zt - zb) * (math.sin(ang) ** 0.35 if 0 < s < 1 else 0.0)
            rows.append(Vector((x, lf.y + y, z)))
        rings.append(rows)
    for rows in rings:
        v.extend(rows)
    for i in range(6):
        for k in range(8):
            o, o1 = i * 9 + k, (i + 1) * 9 + k
            f.append((o, o1, o1 + 1, o + 1))
    ob = mesh("tv_sunshade", v, f, m["hood"], lk, smooth=True, variant=FLIGHT_TV)
    ob.modifiers.new("thick", "SOLIDIFY").thickness = 0.03 * IN


# ---------------------------------------------------------------- aft payload (Apollo 17)

PAL_STA = (100.6, 122.6)
PAL_BL = 17.4
PAL_Z = (FRAME_TOP + 0.1, FRAME_TOP + 26.0)    # ~26 in tall (QTU 00047)


def _sq(v, f, p0, p1, s=1.0 * IN):
    """Square aluminium tube member between two world points."""
    p0, p1 = Vector(p0), Vector(p1)
    d = (p1 - p0).normalized()
    a, b = perp(d)
    obox(v, f, (p0 + p1) / 2, a, d, b, (s, (p1 - p0).length, s))


def build_pallet(ch, m):
    (x0, x1), (y0, y1) = _rect(PAL_STA, (-PAL_BL, PAL_BL))
    z0, z1 = P.wl(PAL_Z[0]), P.wl(PAL_Z[1])
    cx = (x0 + x1) / 2
    pal = link("pallet", (cx, 0.0, z0), ch, {"type": "fixed", "role": "payload_pallet", "payload": True},
               mass=11.0)
    # frame: corner posts, intermediate posts, three belts of rails, rear-face grid (QTU 00057)
    v, f = [], []
    ys = [y0, P.bl(-5.5), P.bl(5.5), y1]
    zs = [z0 + 0.5 * IN, P.wl(PAL_Z[0] + 11.0), z1 - 0.5 * IN]
    for x in (x0, x1):
        for y in ys:
            _sq(v, f, (x, y, z0), (x, y, z1))
        for z in zs:
            _sq(v, f, (x, y0, z), (x, y1, z))
    for y in (y0, y1):
        for z in zs:
            _sq(v, f, (x0, y, z), (x1, y, z))
        _sq(v, f, (x0, y, z0 + 1.0 * IN), (x1, y, z1 - 1.0 * IN), 0.7 * IN)   # side diagonal
    for z in (P.wl(PAL_Z[0] + 5.5), P.wl(PAL_Z[0] + 17.5)):
        _sq(v, f, (x0 - 0.2 * IN, y0, z), (x0 - 0.2 * IN, y1, z), 0.8 * IN)
    for y in (P.bl(-11.0), P.bl(0.0), P.bl(11.0)):
        _sq(v, f, (x0 - 0.2 * IN, y, z0), (x0 - 0.2 * IN, y, z1), 0.8 * IN)
    for y in (P.bl(-8.0), P.bl(-3.0), P.bl(3.0), P.bl(8.0)):   # secondary uprights of the rear grid
        _sq(v, f, (x0 - 0.35 * IN, y, z0 + 1.0 * IN), (x0 - 0.35 * IN, y, z1 - 1.0 * IN), 0.55 * IN)
    for z in (P.wl(PAL_Z[0] + 11.0), P.wl(PAL_Z[0] + 22.0)):
        _sq(v, f, (x0 - 0.35 * IN, y0, z), (x0 - 0.35 * IN, P.bl(-5.5), z), 0.55 * IN)
    mesh("pallet_frame", v, f, m["pal"], pal)
    # right-hand module: riveted sheet-metal box (bag magazine / reel housing), QTU 00057
    v, f = [], []
    rv, rf = [], []
    ym0, ym1 = y0 + 0.3 * IN, P.bl(-5.8)
    bx(v, f, (x0 + 4.0 * IN, (ym0 + ym1) / 2, (z0 + z1) / 2), (7.5 * IN, ym1 - ym0, z1 - z0 - 0.6 * IN))
    bx(v, f, (x0 - 0.2 * IN, (ym0 + ym1) / 2, z1 - 3.6 * IN), (1.4 * IN, ym1 - ym0 - 1.0 * IN, 5.0 * IN))
    bx(v, f, (x0 - 0.95 * IN, (ym0 + ym1) / 2, z1 - 3.6 * IN), (0.1 * IN, ym1 - ym0 - 2.6 * IN, 3.4 * IN))
    for zz in (8.0, 13.5):
        bx(v, f, (x0 + 0.1 * IN, (ym0 + ym1) / 2, P.wl(PAL_Z[0] + zz)), (0.5 * IN, ym1 - ym0 - 0.6 * IN, 0.35 * IN))
    for yy in (ym0 + 0.8 * IN, ym1 - 0.8 * IN):
        for k in range(18):
            zz = z0 + (k + 0.5) * (z1 - z0) / 18
            cyl(rv, rf, (x0 + 0.25 * IN, yy, zz), (x0 + 0.1 * IN, yy, zz), 0.07 * IN, 6)
    mesh("pallet_module", v, f, m["pal_sheet"], pal)
    mesh("pallet_module_rivets", rv, rf, m["steel"], pal)
    # raised tool rack along the top: stand-offs, two rails, stowed handle sleeves; white
    # ball caps on the corner posts; U grab bar at the right-front corner
    v, f = [], []
    for x in (x0 + 0.8 * IN, x1 - 0.8 * IN):
        for y in (y0 + 2.0 * IN, 0.0, y1 - 2.0 * IN):
            cyl(v, f, (x, y, z1), (x, y, z1 + 1.6 * IN), 0.2 * IN, 8)
        cyl(v, f, (x, y0 + 1.0 * IN, z1 + 1.6 * IN), (x, y1 - 1.0 * IN, z1 + 1.6 * IN), 0.3 * IN, 12)
    ub = [Vector((x1 - 1.0 * IN, y0, z1 - 2.0 * IN)), Vector((x1 - 1.0 * IN, y0, z1 + 6.0 * IN)),
          Vector((x0 + 2.0 * IN, y0, z1 + 6.0 * IN)), Vector((x0 + 2.0 * IN, y0, z1 - 2.0 * IN))]
    polyline_tubes(v, f, ub, 0.35 * IN, 12)
    mesh("pallet_tool_rack", v, f, m["pal"], pal, smooth=True)
    v, f = [], []
    for (yy, ln) in ((P.bl(2.0), 9.0), (P.bl(-1.5), 7.0)):
        cyl(v, f, (x1 - 0.8 * IN, yy, z1 + 2.25 * IN), (x1 - 0.8 * IN, yy + ln * IN, z1 + 2.25 * IN), 0.38 * IN, 12)
    mesh("pallet_handle_sleeves", v, f, m["webbing"], pal, smooth=True)
    v, f = [], []
    for x in (x0, x1):
        for y in (y0, y1):
            sphere(v, f, (x, y, z1 + 0.5 * IN), 0.55 * IN, 14, 7)
    mesh("pallet_post_caps", v, f, m["teflon"], pal, smooth=True)
    v, f = [], []
    for (yy, zz) in ((-12.5, 19.0), (-3.0, 20.0), (3.0, 20.0), (12.5, 19.0), (6.0, 9.0)):
        obox(v, f, Vector((x0 - 0.95 * IN, P.bl(yy) + 0.9 * IN, P.wl(PAL_Z[0] + zz))), (1, 0, 0), (0, 1, 0), (0, 0, 1),
             (0.1 * IN, 1.0 * IN, 1.4 * IN))
    mesh("pallet_pull_tabs", v, f, m["green"], pal)
    # sheet-metal bag/tool compartments inside the frame, top deck under the SEP receiver,
    # end plates (QTU 00057: dense riveted sheet structure behind the tube grid)
    v, f = [], []
    bx(v, f, (cx + 1.0 * IN, 0.0, (z0 + z1) / 2 - 1.0 * IN), (x1 - x0 - 5.0 * IN, y1 - y0 - 3.0 * IN, z1 - z0 - 4.0 * IN))
    bx(v, f, (cx, P.bl(8.0), z1 - 0.1 * IN), (x1 - x0, 15.0 * IN, 0.12 * IN))
    for y in (y0 + 0.55 * IN, y1 - 0.55 * IN):
        bx(v, f, (cx, y, (z0 + z1) / 2), (x1 - x0 - 1.0 * IN, 0.08 * IN, z1 - z0 - 1.0 * IN))
    for k in range(3):  # bag compartment dividers seen through the rear grid
        y = P.bl(-8.0 + 8.0 * k)
        bx(v, f, (x0 + 2.0 * IN, y, (z0 + z1) / 2), (0.1 * IN, 7.0 * IN, z1 - z0 - 3.0 * IN))
    mesh("pallet_sheet", v, f, m["pal_sheet"], pal)
    # perforated lightening plates, rivet rows
    v, f = [], []
    rng = random.Random(51)
    for (yc, zc, wy, hz) in ((P.bl(-8.2), P.wl(PAL_Z[0] + 8.0), 4.6, 9.0), (P.bl(8.2), P.wl(PAL_Z[0] + 8.0), 4.6, 9.0),
                             (P.bl(0.0), P.wl(PAL_Z[0] + 3.5), 9.0, 4.0)):
        nh, nv = int(wy / 1.5), int(hz / 1.5)
        cw, chh = wy * IN / nh, hz * IN / nv
        for i in range(nh):
            for j in range(nv):
                yy = yc - wy * IN / 2 + cw * (i + 0.5)
                zz = zc - hz * IN / 2 + chh * (j + 0.5)
                # plate cell as a frame around a hole (four thin boxes)
                hw = cw * 0.32
                for dy, dz, sy, sz in ((0, chh / 2 - (chh / 2 - hw) / 2, cw, chh / 2 - hw),
                                       (0, -(chh / 2 - (chh / 2 - hw) / 2), cw, chh / 2 - hw),
                                       (cw / 2 - (cw / 2 - hw) / 2, 0, cw / 2 - hw, 2 * hw),
                                       (-(cw / 2 - (cw / 2 - hw) / 2), 0, cw / 2 - hw, 2 * hw)):
                    bx(v, f, (x0 - 0.75 * IN, yy + dy, zz + dz), (0.06 * IN, sy, sz))
    mesh("pallet_plates", v, f, m["pal"], pal)
    v, f = [], []
    for y in ys:
        for k in range(16):
            z = z0 + (k + 0.5) * (z1 - z0) / 16
            cyl(v, f, (x0 - 0.5 * IN, y, z), (x0 - 0.62 * IN, y, z), 0.08 * IN, 6)
    mesh("pallet_rivets", v, f, m["steel"], pal)
    # webbing straps and green/yellow pull tabs
    v, f = [], []
    for y in (P.bl(-12.5), P.bl(-3.0), P.bl(3.0), P.bl(12.5)):
        bx(v, f, (x0 - 0.9 * IN, y, P.wl(PAL_Z[0] + 18.0)), (0.05 * IN, 1.0 * IN, 9.0 * IN))
        bx(v, f, (x0 - 0.95 * IN, y, P.wl(PAL_Z[0] + 13.0)), (0.08 * IN, 1.4 * IN, 1.6 * IN))
    mesh("pallet_straps", v, f, m["webbing"], pal)
    # riveted tool-carrier column on the right end with the sample bag dispenser on top (22296)
    v, f = [], []
    colc = Vector((cx + 2.0 * IN, P.bl(-12.8), (z1 + P.wl(PAL_Z[1] + 9.0)) / 2))
    bx(v, f, colc, (4.0 * IN, 3.2 * IN, P.wl(PAL_Z[1] + 9.0) - z1))
    mesh("pallet_column", v, f, m["pal"], pal)
    v, f = [], []
    for k in range(10):
        zc = z1 + (k + 0.5) * 0.9 * IN
        cyl(v, f, (colc.x - 1.4 * IN, colc.y - 1.6 * IN, zc), (colc.x - 1.4 * IN, colc.y - 1.72 * IN, zc), 0.1 * IN, 6)
        cyl(v, f, (colc.x + 1.4 * IN, colc.y - 1.6 * IN, zc), (colc.x + 1.4 * IN, colc.y - 1.72 * IN, zc), 0.1 * IN, 6)
    mesh("pallet_column_rivets", v, f, m["steel"], pal)
    dz0 = P.wl(PAL_Z[1] + 9.0)
    v, f = [], []
    for dy in (-2.6, 2.6):  # twin bag dispenser magazines with a carry bail
        bx(v, f, (colc.x, colc.y + dy * IN, dz0 + 3.6 * IN), (5.6 * IN, 4.8 * IN, 7.2 * IN))
    bx(v, f, (colc.x - 3.0 * IN, colc.y, dz0 + 2.8 * IN), (0.6 * IN, 10.6 * IN, 2.0 * IN))
    mesh("bag_dispenser", v, f, m["gray"], pal)
    v, f = [], []
    polyline_tubes(v, f, [Vector((colc.x - 3.4 * IN, colc.y - 4.2 * IN, dz0 + 2.0 * IN)),
                          Vector((colc.x - 4.6 * IN, colc.y - 4.2 * IN, dz0 + 3.4 * IN)),
                          Vector((colc.x - 4.6 * IN, colc.y + 4.2 * IN, dz0 + 3.4 * IN)),
                          Vector((colc.x - 3.4 * IN, colc.y + 4.2 * IN, dz0 + 2.0 * IN))], 0.3 * IN, 10)
    mesh("bag_dispenser_bail", v, f, m["anodized"], pal, smooth=True)
    # stowed sample collection bag (beta cloth) hanging on the left-rear post
    v, f = [], []
    crinkle_box(v, f, (x0 - 3.0 * IN, P.bl(12.0), P.wl(PAL_Z[0] + 14.0)), (4.5 * IN, 7.5 * IN, 13.0 * IN),
                amp=0.05 * IN, pillow=0.5 * IN, cell=0.6 * IN, seed=55, wave=0.35 * IN, fold=0.08 * IN)
    mesh("sample_collection_bag", v, f, m["beta"], pal, smooth=True)

    _tools(m, pal, x0, x1, y0, y1, z0, z1, colc, dz0)
    _pallet_details(m, pal, x0, x1, y0, y1, z0, z1)
    _sep(m, pal, x0, x1, y0, y1, z1)
    _tge(m, pal, x0, z0)
    return pal


def _pallet_details(m, pal, x0, x1, y0, y1, z0, z1):
    """QTU-level hardware on the pallet's rear face (NASM-NASM2020-00057 close-ups): riveted gusset
    plates at the frame joints, stainless tie-down tongues under the bottom rail, a bag-hook rod
    with formed clips, the tall tool bracket (part no. SEB39105285-301), green pip-pin T-levers,
    yellow X-stitched pull tabs on the straps, the funnel tool holder, and stowage placards."""
    X = Vector((1, 0, 0)); Y = Vector((0, 1, 0)); Z = UP
    xr = x0 - 0.75 * IN                      # plane just behind the rear grid
    rails = [z0 + 0.5 * IN, P.wl(PAL_Z[0] + 5.5), P.wl(PAL_Z[0] + 11.0), P.wl(PAL_Z[0] + 17.5), z1 - 0.5 * IN]
    posts = [y0, P.bl(-5.5), P.bl(-3.0), P.bl(3.0), P.bl(5.5), P.bl(8.0), y1]
    gv, gf, rv, rf = [], [], [], []
    for y in posts:
        for z in rails:
            obox(gv, gf, Vector((xr, y, z)), X, Y, Z, (0.05 * IN, 1.5 * IN, 1.5 * IN))
            for dy, dz in ((-0.5, -0.5), (0.5, -0.5), (-0.5, 0.5), (0.5, 0.5)):
                sphere(rv, rf, Vector((xr - 0.03 * IN, y + dy * IN, z + dz * IN)), 0.07 * IN, 6, 3)
    mesh("pallet_gussets", gv, gf, m["pal"], pal)
    mesh("pallet_gusset_rivets", rv, rf, m["al"], pal, smooth=True)
    # stainless tie-down tongues (three holes) below the bottom rail
    v, f, hv, hf = [], [], [], []
    for yy in (-11.0, 1.0, 11.5):
        p = Vector((x0 - 0.7 * IN, P.bl(yy), z0 - 0.3 * IN))
        obox(v, f, p, X, Y, Z, (0.06 * IN, 1.5 * IN, 1.9 * IN))
        for dy, dz in ((-0.35, -0.35), (0.35, -0.35), (0.0, 0.4)):
            cyl(hv, hf, p + Vector((-0.04 * IN, dy * IN, dz * IN)), p + Vector((0.04 * IN, dy * IN, dz * IN)),
                0.2 * IN, 12)
    mesh("pallet_tiedowns", v, f, m["steel"], pal)
    mesh("pallet_tiedown_holes", hv, hf, m["black"], pal)
    # bag-hook rod with formed U clips (upper rear, left two thirds)
    v, f = [], []
    zr = P.wl(PAL_Z[0] + 20.5)
    cyl(v, f, (x0 - 1.5 * IN, P.bl(-4.5), zr), (x0 - 1.5 * IN, P.bl(16.0), zr), 0.16 * IN, 10)
    for yy in (-2.0, 4.0, 9.5, 14.5):
        yc = P.bl(yy)
        for sx in (-0.45, 0.45):
            obox(v, f, Vector((x0 - 1.5 * IN, yc + sx * IN, zr + 0.5 * IN)), X, Y, Z, (0.9 * IN, 0.05 * IN, 1.6 * IN))
        obox(v, f, Vector((x0 - 1.0 * IN, yc, zr + 1.2 * IN)), X, Y, Z, (0.05 * IN, 0.95 * IN, 0.9 * IN))
    mesh("pallet_bag_hooks", v, f, m["pal_sheet"], pal)
    # tall tool bracket: channel with a top flange plate, part number engraved along it
    v, f = [], []
    yb = P.bl(4.5)
    zb0, zb1 = P.wl(PAL_Z[0] + 11.5), P.wl(PAL_Z[0] + 21.5)
    obox(v, f, Vector((x0 - 1.2 * IN, yb, (zb0 + zb1) / 2)), X, Y, Z, (0.6 * IN, 1.0 * IN, zb1 - zb0))
    obox(v, f, Vector((x0 - 1.6 * IN, yb, zb1 + 0.06 * IN)), X, Y, Z, (1.8 * IN, 2.4 * IN, 0.12 * IN))
    obox(v, f, Vector((x0 - 1.55 * IN, yb, zb0 + 0.9 * IN)), X, Y, Z, (0.3 * IN, 1.4 * IN, 1.8 * IN))
    mesh("pallet_tool_bracket", v, f, m["pal_sheet"], pal)
    placard("pallet_partno", "eq_partno_seb39105285", Vector((x0 - 1.5 * IN, yb, (zb0 + zb1) / 2 + 1.0 * IN)),
            Z, Y, 2.2, 0.24, pal, lift=0.01 * IN)
    # green pip-pin T-levers
    v, f = [], []
    for yy, zz in ((-1.2, 21.3), (15.0, 14.2)):
        p = Vector((x0 - 1.0 * IN, P.bl(yy), P.wl(PAL_Z[0] + zz)))
        obox(v, f, p + Vector((-0.4, 0.0, -0.9)) * IN, X, Y, Z, (0.2 * IN, 0.5 * IN, 2.0 * IN))
        obox(v, f, p + Vector((-0.5, 0.0, -2.1)) * IN, X, Y, Z, (0.1 * IN, 1.1 * IN, 0.9 * IN))
    mesh("pallet_pip_levers", v, f, m["green"], pal)
    # yellow X-stitched pull tabs at the strap ends
    v, f = [], []
    for y in (P.bl(-12.5), P.bl(-3.0), P.bl(3.0), P.bl(12.5)):
        bx(v, f, (x0 - 0.97 * IN, y, P.wl(PAL_Z[0] + 12.6)), (0.08 * IN, 1.1 * IN, 1.6 * IN))
    mesh("pallet_strap_tabs", v, f, m["ytab"], pal)
    # funnel-shaped tool holder at the lower right of the rear face
    v, f = [], []
    fc = Vector((x0 - 2.2 * IN, P.bl(-3.8), P.wl(PAL_Z[0] + 2.0)))
    revolve(v, f, [(0.5 * IN, 0.0), (0.55 * IN, 1.6 * IN), (1.3 * IN, 2.6 * IN), (1.35 * IN, 3.2 * IN)],
            axis_xform(fc, UP), 24)
    obox(v, f, fc + Vector((0.9, 0.0, 2.6)) * IN, X, Y, Z, (1.6 * IN, 0.8 * IN, 0.1 * IN))
    ob = mesh("pallet_funnel", v, f, m["pal_sheet"], pal, smooth=True)
    ob.modifiers.new("thick", "SOLIDIFY").thickness = 0.04 * IN
    # stowage placards (legible in 00057)
    back = (0, -1, 0)
    placard("pallet_placard_collection_bag", "eq_placard_collection_bag",
            Vector((x0 - 0.95 * IN, P.bl(6.6), P.wl(PAL_Z[0] + 6.5))), Z, (0, 1, 0), 2.6, 0.42, pal, lift=0.0)
    placard("pallet_placard_tongs", "eq_placard_tongs",
            Vector((x0 - 0.52 * IN, y0, P.wl(PAL_Z[0] + 21.0))), Z, (0, 1, 0), 1.1, 0.36, pal, lift=0.0)
    placard("pallet_placard_extension_handle", "eq_placard_extension_handle",
            Vector((x0 - 0.52 * IN, y0, P.wl(PAL_Z[0] + 15.0))), Z, (0, 1, 0), 3.0, 0.36, pal, lift=0.0)
    placard("pallet_placard_gnomon", "eq_placard_gnomon",
            Vector((x0 - 0.82 * IN, P.bl(1.5), P.wl(PAL_Z[0] + 4.8))), back, Z, 1.2, 0.34, pal, lift=0.0)
    placard("pallet_placard_dust", "eq_placard_dust",
            Vector((x0 - 0.52 * IN, P.bl(-1.0), z1 - 0.5 * IN)), back, Z, 0.9, 0.34, pal, lift=0.0)


def _tool(name, origin, parent, mass):
    return link(name, origin, parent, {"type": "fixed", "role": "tool", "payload": True}, mass=mass)


def _tools(m, pal, x0, x1, y0, y1, z0, z1, colc, dz0):
    # 32-inch tongs x2, stowed vertically in the dispenser holders with red grips (22296)
    for i, dy in enumerate((-1.6, 1.4)):
        p = Vector((colc.x + 1.6 * IN, colc.y + dy * IN, dz0 - 18.0 * IN))
        lk = _tool(f"tongs_{i + 1}", p, pal, 0.45)
        v, f = [], []
        cyl(v, f, p, p + UP * 31.0 * IN, 0.3 * IN, 10)
        cyl(v, f, p - UP * 1.0 * IN + Vector((0, 0.4, 0)) * IN, p + Vector((0, 0.4, 0)) * IN, 0.12 * IN, 6)
        cyl(v, f, p - UP * 1.0 * IN - Vector((0, 0.4, 0)) * IN, p - Vector((0, 0.4, 0)) * IN, 0.12 * IN, 6)
        mesh(f"tongs_{i + 1}", v, f, m["al_spun"], lk, smooth=True)
        v, f = [], []
        g = p + UP * 31.0 * IN
        polyline_tubes(v, f, [g, g + UP * 1.6 * IN, g + UP * 2.4 * IN + Vector((0.8, 0, 0)) * IN,
                              g + UP * 1.6 * IN + Vector((1.6, 0, 0)) * IN], 0.22 * IN, 8)
        mesh(f"tongs_{i + 1}_grip", v, f, m["red"], lk, smooth=True)
    # lunar rake: tall staff at the right-rear corner, basket head on top (22296)
    rb = Vector((x0 - 2.2 * IN, y0 - 0.8 * IN, z0 + 2.0 * IN))
    lk = _tool("rake", rb, pal, 0.9)
    v, f = [], []
    rt = rb + UP * 48.0 * IN
    cyl(v, f, rb, rt, 0.45 * IN, 12)
    for z in (14.0, 22.0, 30.0):
        cyl(v, f, rb + UP * z * IN, rb + UP * (z + 1.2) * IN, 0.6 * IN, 12)
    mesh("rake_handle", v, f, m["al_spun"], lk, smooth=True)
    v, f = [], []
    hc = rt + UP * 2.5 * IN
    hx = Vector((-1.0, -0.35, 0.0)).normalized()
    hy = UP.cross(hx).normalized()
    for k in range(13):  # curved tines (basket)
        s = (k / 12 - 0.5) * 9.0 * IN
        pts = []
        for j in range(7):
            t = math.pi * j / 6
            pts.append(hc + hy * s + hx * (3.2 * IN * math.sin(t)) + UP * (3.2 * IN * -math.cos(t)))
        polyline_tubes(v, f, pts, 0.06 * IN, 5)
    for j in (0, 3, 6):
        t = math.pi * j / 6
        p = hc + hx * (3.2 * IN * math.sin(t)) + UP * (3.2 * IN * -math.cos(t))
        cyl(v, f, p - hy * 4.6 * IN, p + hy * 4.6 * IN, 0.1 * IN, 6)
    mesh("rake_head", v, f, m["steel"], lk, smooth=True)
    v, f = [], []
    obox(v, f, hc + hx * 1.4 * IN + UP * 3.4 * IN, hx, hy, UP, (3.6 * IN, 9.6 * IN, 0.06 * IN))
    mesh("rake_shield", v, f, m["al"], lk)
    # extension handle laid across the pallet top, hammer in its holster, adjustable scoop
    p = Vector((x1 - 2.0 * IN, y0 + 3.0 * IN, z1 + 0.9 * IN))
    lk = _tool("extension_handle", p, pal, 0.7)
    v, f = [], []
    cyl(v, f, p, p + Vector((0, 26.0, 0)) * IN, 0.4 * IN, 12)
    cyl(v, f, p + Vector((0, 26.0, 0)) * IN, p + Vector((0, 26.0, 0)) * IN + Vector((-5.0, 0, 0)) * IN, 0.35 * IN, 12)
    mesh("extension_handle", v, f, m["al_spun"], lk, smooth=True)
    v, f = [], []
    cyl(v, f, p - Vector((0, 0.5, 0)) * IN, p + Vector((0, 1.0, 0)) * IN, 0.5 * IN, 12)
    mesh("extension_handle_lock", v, f, m["anodized"], lk, smooth=True)
    p = Vector((x0 - 1.4 * IN, P.bl(-4.0), z0 + 10.0 * IN))
    lk = _tool("hammer", p, pal, 0.65)
    v, f = [], []
    cyl(v, f, p, p + UP * 15.0 * IN, 0.4 * IN, 12)
    mesh("hammer_handle", v, f, m["al_spun"], lk, smooth=True)
    v, f = [], []
    obox(v, f, p + UP * 15.6 * IN, (1, 0, 0), (0, 1, 0), (0, 0, 1), (1.2 * IN, 5.0 * IN, 1.2 * IN))
    mesh("hammer_head", v, f, m["steel"], lk)
    v, f = [], []
    bx(v, f, p + UP * 8.0 * IN + Vector((0.2, 0, 0)) * IN, (1.2 * IN, 1.6 * IN, 6.0 * IN))
    mesh("hammer_holster", v, f, m["beta"], lk)
    p = Vector((x0 - 1.4 * IN, P.bl(4.5), z0 + 8.0 * IN))
    lk = _tool("scoop", p, pal, 0.6)
    v, f = [], []
    cyl(v, f, p, p + UP * 14.0 * IN, 0.4 * IN, 12)
    mesh("scoop_handle", v, f, m["al_spun"], lk, smooth=True)
    v, f = [], []
    sc = p + UP * 16.5 * IN
    crinkle_box(v, f, sc, (1.4 * IN, 3.0 * IN, 4.6 * IN), pillow=-0.2 * IN, cell=0.5 * IN, seed=61, skip=("-x",))
    mesh("scoop_pan", v, f, m["steel"], lk)
    # gnomon (weighted staff, colour chart legs) in its holder on the left end
    p = Vector((x1 + 1.0 * IN, y1 - 1.5 * IN, z0 + 6.0 * IN))
    lk = _tool("gnomon", p, pal, 0.35)
    v, f = [], []
    cyl(v, f, p, p + UP * 17.0 * IN, 0.2 * IN, 8)
    for k in range(3):
        t = TAU * k / 3
        cyl(v, f, p + UP * 2.0 * IN, p + Vector((math.cos(t) * 0.8, math.sin(t) * 0.8, 6.0)) * IN, 0.12 * IN, 6)
    mesh("gnomon", v, f, m["black"], lk, smooth=True)
    v, f = [], []
    for k in range(6):  # grey-scale / colour bands on the staff
        cyl(v, f, p + UP * (8.0 + 1.4 * k) * IN, p + UP * (8.7 + 1.4 * k) * IN, 0.22 * IN, 8)
    mesh("gnomon_bands", v, f, m["white"], lk, smooth=True)


# Apollo 17-only payload (SEP experiment, traverse gravimeter) and the LRV-3 aft-chassis decal: absent on the
# A15 vehicle and on the Smithsonian display vehicle (Mission variant).
A17_ONLY = "Mission:a17,a17_repair"


def _sep(m, pal, x0, x1, y0, y1, z1):
    """SEP receiver (copper-gold Kapton blanket, white radiator top) and its tri-loop antenna."""
    c = Vector((P.sta(110.5), P.bl(9.0), z1 + 5.6 * IN))
    lk = link("sep_receiver", c - UP * 5.5 * IN, pal, {"type": "fixed", "role": "sep_receiver", "payload": True},
              mass=8.0)
    v, f = [], []
    crinkle_box(v, f, c, (10.0 * IN, 11.0 * IN, 11.0 * IN), amp=0.02 * IN, pillow=0.15 * IN, cell=0.45 * IN,
                seed=71, wave=0.15 * IN, fold=0.09 * IN, skip=("+z",))
    mesh("sep_receiver_blanket", v, f, m["kapton_cu"], lk, variant=A17_ONLY)
    v, f = [], []
    bx(v, f, c + UP * 5.55 * IN, (9.6 * IN, 10.6 * IN, 0.2 * IN))
    mesh("sep_receiver_radiator", v, f, m["teflon"], lk, variant=A17_ONLY)
    v, f = [], []
    polyline_tubes(v, f, [c + Vector((-2.5, -3.5, 5.7)) * IN, c + Vector((-2.5, -3.5, 7.2)) * IN,
                          c + Vector((2.5, -3.5, 7.2)) * IN, c + Vector((2.5, -3.5, 5.7)) * IN], 0.25 * IN, 10)
    mesh("sep_receiver_handle", v, f, m["al"], lk, smooth=True, variant=A17_ONLY)

    # antenna mast on the left-rear of the pallet, octahedral tri-loop (three orthogonal loops)
    mb = Vector((x0 - 1.5 * IN, P.bl(2.0), z1))
    ant = link("sep_antenna", mb, pal, {"type": "fixed", "role": "sep_antenna", "payload": True}, mass=0.9)
    mt = mb + UP * 30.0 * IN
    v, f = [], []
    cyl(v, f, mb, mb + UP * 14.0 * IN, 0.45 * IN, 12)
    cyl(v, f, mb + UP * 13.0 * IN, mt + UP * 20.0 * IN, 0.3 * IN, 12)
    cyl(v, f, mb + UP * 13.6 * IN, mb + UP * 14.6 * IN, 0.55 * IN, 12)
    mesh("sep_antenna_mast", v, f, m["al_spun"], ant, smooth=True, variant=A17_ONLY)
    ac = mt + UP * 10.0 * IN
    rr = 10.0 * IN
    vx = [ac + Vector(e) * rr for e in ((1, 0, 0), (-1, 0, 0), (0, 1, 0), (0, -1, 0), (0, 0, 1), (0, 0, -1))]
    edges = [(0, 2), (2, 1), (1, 3), (3, 0), (0, 4), (4, 1), (1, 5), (5, 0), (2, 4), (4, 3), (3, 5), (5, 2)]
    v, f = [], []
    for i, j in edges:
        cyl(v, f, vx[i], vx[j], 0.06 * IN, 6)
    for i in (0, 1, 2, 3):  # spreader arms
        cyl(v, f, ac, vx[i], 0.12 * IN, 6)
    mesh("sep_antenna_loops", v, f, m["copper"], ant, smooth=True, variant=A17_ONLY)
    v, f = [], []
    pts = [vx[4], ac + Vector((2.0, 2.5, 4.0)) * IN, ac + Vector((3.0, 2.0, -6.0)) * IN,
           mb + UP * 20.0 * IN + Vector((0.6, 0.0, 0.0)) * IN, mb + UP * 4.0 * IN + Vector((0.6, 0.0, 0.0)) * IN,
           c + Vector((-5.2, -2.0, 3.0)) * IN]
    polyline_tubes(v, f, pts, 0.12 * IN, 6)
    mesh("sep_antenna_cable", v, f, m["kapton"], ant, smooth=True, variant=A17_ONLY)


def _tge(m, pal, x0, z0):
    """Traverse gravimeter (S-199) on the right rear of the pallet, blue thermal cover (as17-137-20979).

    The broad face (9.5 x 17 in) faces outboard (-Y, vehicle right) with the levelling-gimbal hub in a dished
    recess slightly forward of centre; darker blue lid cap on top, stitched seam round the middle.
    """
    c = Vector((x0 - 5.65 * IN, P.bl(-10.25), z0 + 10.5 * IN))
    lk = link("traverse_gravimeter", c - UP * 9.0 * IN, pal, {"type": "fixed", "role": "traverse_gravimeter",
                                                               "payload": True}, mass=12.7)
    v, f = [], []
    crinkle_box(v, f, c, (9.5 * IN, 7.0 * IN, 17.0 * IN), amp=0.05 * IN, pillow=0.35 * IN, cell=1.0 * IN,
                seed=81, wave=0.15 * IN)
    mesh("tge_cover", v, f, m["tge_cover"], lk, smooth=True, variant=A17_ONLY)
    fy = c.y - 3.5 * IN                      # outboard face
    pc = Vector((c.x + 1.0 * IN, fy, c.z + 0.5 * IN))   # hub centre on the face, a little forward of centre
    v, f = [], []
    cyl(v, f, pc + Vector((0, 0.45, 0)) * IN, pc + Vector((0, -0.75, 0)) * IN, 2.2 * IN, 24)
    cyl(v, f, pc + Vector((0, -0.75, 0)) * IN, pc + Vector((0, -1.05, 0)) * IN, 1.3 * IN, 24)
    mesh("tge_hub", v, f, m["tge_cover"], lk, smooth=True, variant=A17_ONLY)
    # dished recess round the hub: a shallow cone from the face into the cover down to the hub base
    v, f = [], []
    n = 32
    for k in range(n):
        for r_, dy in ((3.6, 0.0), (2.3, 0.45)):
            a_ = 2 * math.pi * k / n
            v.append(Vector((pc.x + r_ * IN * math.cos(a_), fy + dy * IN, pc.z + r_ * IN * math.sin(a_))))
    for k in range(n):
        a0, a1 = 2 * k, 2 * ((k + 1) % n)
        f.append((a0, a0 + 1, a1 + 1, a1))
    mesh("tge_recess", v, f, m["tge_cover"], lk, smooth=True, variant=A17_ONLY)
    v, f = [], []
    crinkle_box(v, f, c + Vector((0.0, 0.0, 9.6)) * IN, (10.1 * IN, 7.6 * IN, 2.6 * IN), amp=0.04 * IN,
                pillow=0.25 * IN, cell=1.0 * IN, seed=82, wave=0.1 * IN)
    mesh("tge_lid", v, f, m["tge_lid"], lk, smooth=True, variant=A17_ONLY)
    v, f = [], []
    hx, hy, zs = 4.8 * IN, 3.55 * IN, c.z + 1.5 * IN
    polyline_tubes(v, f, [Vector((c.x + sx_ * hx, c.y + sy_ * hy, zs)) for sx_, sy_ in
                          ((-1, -1), (-1, 1), (1, 1), (1, -1), (-1, -1))], 0.09 * IN, 6)
    mesh("tge_seam", v, f, m["tge_cover"], lk, smooth=True, variant=A17_ONLY)
    v, f = [], []
    polyline_tubes(v, f, [c + Vector((-3.0, 1.5, 10.5)) * IN, c + Vector((-3.0, 1.5, 12.2)) * IN,
                          c + Vector((3.0, 1.5, 12.2)) * IN, c + Vector((3.0, 1.5, 10.5)) * IN], 0.25 * IN, 10)
    for dx in (-3.5, 3.5):  # feet
        cyl(v, f, c + Vector((dx, 0, -8.5)) * IN, c + Vector((dx, 0, -9.0)) * IN, 0.8 * IN, 12)
    mesh("tge_handle_feet", v, f, m["al"], lk, smooth=True, variant=A17_ONLY)


def build_decal(ch, m):
    """Pallet-stop tether decal on the aft chassis end member (A17 MR App A.3.2).

    Artwork unknown: modelled as a yellow placard with a black arrow pointing at the tether hole.
    """
    z = P.wl(FRAME_TOP) + 0.01 * IN
    c = W(124.0, -3.5, FRAME_TOP)
    v, f = [], []
    bx(v, f, (c.x, c.y, z + 0.01 * IN), (1.2 * IN, 2.6 * IN, 0.02 * IN))
    mesh("aft_decal", v, f, m["decal"], ch, variant=A17_ONLY)
    v, f = [], []
    poly = [(c.x - 0.12 * IN, c.y + 0.9 * IN), (c.x - 0.12 * IN, c.y - 0.4 * IN), (c.x - 0.35 * IN, c.y - 0.4 * IN),
            (c.x, c.y - 1.05 * IN), (c.x + 0.35 * IN, c.y - 0.4 * IN), (c.x + 0.12 * IN, c.y - 0.4 * IN),
            (c.x + 0.12 * IN, c.y + 0.9 * IN)]
    prism(v, f, poly, z + 0.02 * IN, z + 0.03 * IN)
    mesh("aft_decal_arrow", v, f, m["black"], ch)
    v, f = [], []
    cyl(v, f, W(124.0, -5.6, FRAME_TOP + 0.01), W(124.0, -5.6, FRAME_TOP - 0.3), 0.18 * IN, 12)
    mesh("pallet_tether_hole", v, f, m["black"], ch)


# ---------------------------------------------------------------- entry point

def build(ch):
    m = _mats()
    build_forward_electronics(ch, m)
    build_dust_covers(ch, m)
    build_fittings(ch, m)
    lcru = build_lcru(ch, m)
    build_hga(ch, m, lcru)
    build_lga(ch, m)
    build_tv(ch, m)
    build_pallet(ch, m)
    build_decal(ch, m)
