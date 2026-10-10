"""Crew station: seats, footrests, handholds, Control and Display Console, hand controller.

build(ch) is called by build.py with the chassis link. Positions are in handbook inches
(STA aft-positive, BL left-positive, WL with 100 = chassis bottom) via params.sta/bl/wl.

Sources:
  A17 Systems Handbook (LRVSysHndbkA17.pdf)
    DWG 5.1 crew station (PDF p.45): seat pan 20.50 x 17.50, seat base 19.10, back 4.7 aft of
      the base, outer back post 18.14 / inner 15.33 above the pan, 21.205 seat outer frame to
      console centreline, footrest 24 wide, +-1.5 in / 15 deg adjustment (LRV-3 fully forward),
      toe restraints (wingtips) inboard only, inboard/outboard handholds, armrest, seat belts.
    DWG 5.2 console (PDF p.46): 10.00 x 12.00 panel, full switch layout; hand controller pivot
      18.00 forward of / 11.00 above the SRP; console plan view (handholds, LGA/DAC sockets,
      tee-handle latch, C&W flag); contingency park brake release ring.
    DWG 5.4 hand controller (PDF p.48): palm (throttle) pivot +-14 deg, brake pivot 30 deg,
      3 in / 4 in brake displacement, steering axis +-14 deg, reverse inhibit switch.
  Ops Handbook LS006-002-2H p.1-20..1-23: dead bands, soft/hard stops, torques.
  Photos: 7021041.jpg / 7021092.jpg (LRV-2 flight console: black faceplate, white legends),
    NASM QTU NASM2020-00055 (console + T-handle), -00056 / -00047 (seats, webbing).
"""
import json
import math
import os
import subprocess
import sys

import bmesh
import bpy
from mathutils import Vector

import materials as M
import params as P
from geo import _frame, add_mesh, link

IN = P.IN
HERE = os.path.dirname(os.path.abspath(__file__))
TEX_DIR = os.path.abspath(os.path.join(HERE, "..", "textures", "gen"))

X, Y, Z = Vector((1, 0, 0)), Vector((0, 1, 0)), Vector((0, 0, 1))


def H(s, b, w):
    """Handbook (STA, BL, WL) inches -> world metres."""
    return Vector((P.sta(s), P.bl(b), P.wl(w)))


# ---------------------------------------------------------------- crew-station layout (inches)
SEAT_BL_IN, SEAT_BL_OUT = 3.5, 24.0      # 20.50 pan width (DWG 5.1 plan); 21.205 frame-to-CL front view
SEAT_STA_F, SEAT_STA_R = 72.6, 90.1      # 17.50 pan depth; SRP 18.00 aft of the hand controller
SEAT_WL = 110.6                          # pan tube centreline: ~7.2 in above chassis top (DWG 5.1/5.2)
SEAT_TUBE_R = 0.5                        # 1 in aluminium tube
BACK_LEAN = math.radians(18)             # PLSS support lean, measured off QTU side view 00047 (~18-21 deg)
BACK_POST_BL = (3.0, 24.5)
BACK_H_IN, BACK_H_OUT = 15.33, 18.14     # post heights above the pan (DWG 5.1 front view)
BACK_FOLD = math.pi / 2 + BACK_LEAN         # back folds forward flat onto the pan for stowage

HC_STA = SEAT_STA_F + 14.9 - 18.0        # SRP 14.9 aft of the seat-base front (step 4), HC 18.00 fwd
SRP_WL = 110.2
HC_PALM_WL = 119.6                       # palm pivot mid-stem: QTU 00055 scale (stem 4.4 in collar->bar,
                                         # 1.3 in bar); DWG 5.2 gives 11.00 above an SRP we can only estimate
HC_BRAKE_R = 7.7                         # 4.0 in limit displacement at 30 deg -> 7.7 in below the palm
HC_BRAKE_WL = HC_PALM_WL - HC_BRAKE_R
HC_ROLL_WL = HC_BRAKE_WL - 1.0
HOUSING = (61.3, 76.0, 3.3, 108.0, 116.0)  # sta0, sta1, half width, wl0, wl1 (DWG 5.2 top/front view)

PANEL_TILT = math.radians(30)            # faces aft and up (DWG 5.2 side view / QTU photos)
PANEL_TOP = (55.3, 127.3)                # STA, WL of the faceplate's top edge (44.8 in overall height)
PANEL_W, PANEL_H = 10.0, 12.0
CONSOLE_FRONT_STA = 47.3                 # console box 14 in long in plan (DWG 5.2 top view)

FOOT_HINGE = (48.8, 100.6)               # DWG 5.1 footrest side view (fully forward)
FOOT_TOP = (42.7, 108.1)
FOOT_BL = (1.8, 25.8)                    # 24 in wide

# Joint drives. Torques from Ops HB p.1-20..1-23: throttle breakout 6 in-lb, steering 7 in-lb
# rising linearly to the 9 deg soft stop (+5 in-lb step) and the 14 deg hard stop; brake
# ~30 lb at the grip for the 3 in park-latch pull (Fig 1-14).
HC_STEER_LIMIT = math.radians(14.0)
HC_THROTTLE_LIMIT = math.radians(14.0)
HC_BRAKE_LIMIT = math.radians(30.0)
HC_STEER_DRIVE = {"kind": "position", "stiffness": 11.0, "damping": 0.3, "max_force": 5.0, "target": 0.0}
HC_THROTTLE_DRIVE = {"kind": "position", "stiffness": 3.0, "damping": 0.5, "max_force": 2.0, "target": 0.0}
HC_BRAKE_DRIVE = {"kind": "position", "stiffness": 90.0, "damping": 2.0, "max_force": 60.0, "target": 0.0}
SWITCH_DRIVE = {"kind": "position", "stiffness": 0.05, "damping": 0.001, "max_force": 0.2, "target": 0.0}
SEAT_DRIVE = {"kind": "position", "stiffness": 2000.0, "damping": 40.0, "max_force": 400.0, "target": 0.0}

MASS = {"seat_back": 1.6, "hc_steer": 0.25, "hc_brake": 0.35, "hc_throttle": 0.3, "hc_reverse_switch": 0.01}


# ---------------------------------------------------------------- materials

def _mat(name):
    specs = {
        # seat colours sampled from NASM-NASM2020-00056 against the white under-seat bag (= 0.82)
        "crew_webbing_olive": ((0.11, 0.095, 0.05), 0.0, 0.9),
        "crew_webbing_white": ((0.66, 0.64, 0.54), 0.0, 0.85),
        "crew_velcro_tan": ((0.62, 0.40, 0.15), 0.0, 0.85),
        "crew_velcro_hook": ((0.62, 0.50, 0.32), 0.0, 0.95),
        "crew_velcro_white": ((0.74, 0.71, 0.64), 0.0, 0.95),
        "crew_velcro_dark": ((0.46, 0.26, 0.08), 0.0, 0.8),
        "crew_pan_strap": ((0.48, 0.31, 0.10), 0.0, 0.75),
        "crew_seat_cover": ((0.78, 0.76, 0.64), 0.0, 0.85),
        "crew_edge_tape": ((0.30, 0.075, 0.02), 0.0, 0.38),
        "crew_foam": ((0.56, 0.36, 0.30), 0.0, 0.9),
        "crew_belt": ((0.45, 0.12, 0.02), 0.0, 0.7),
        "crew_strap_brown": ((0.25, 0.08, 0.03), 0.0, 0.75),
        "crew_bag": ((0.82, 0.82, 0.80), 0.0, 0.8),
        "crew_frame_white": ((0.80, 0.80, 0.78), 0.0, 0.45),
        "crew_gold": ((0.62, 0.45, 0.15), 1.0, 0.35),
        "crew_honeycomb": ((0.55, 0.55, 0.53), 0.9, 0.55),
        "crew_housing": ((0.83, 0.83, 0.80), 0.0, 0.42),
        "crew_bezel": ((0.03, 0.03, 0.035), 0.3, 0.35),
        "crew_meter_frame": ((0.10, 0.12, 0.16), 0.2, 0.4),
        "crew_toggle": ((0.80, 0.80, 0.78), 0.6, 0.35),
        "crew_switch_collar": ((0.70, 0.30, 0.10), 0.8, 0.35),
        "crew_cb_knob": ((0.015, 0.015, 0.015), 0.0, 0.3),
        "crew_knob_white": ((0.85, 0.85, 0.82), 0.0, 0.3),
        # sampled from NASM-NASM2020-00055 (sRGB -> linear, scaled so the housing reads 0.83)
        "crew_grip": ((0.50, 0.53, 0.41), 0.0, 0.32),
        "crew_grip_dark": ((0.16, 0.17, 0.13), 0.0, 0.6),
        "crew_boot": ((0.55, 0.54, 0.51), 0.4, 0.5),
        "crew_armrest": ((0.13, 0.045, 0.026), 0.0, 0.45),
        "crew_handhold": ((0.13, 0.13, 0.14), 0.6, 0.4),
        "crew_harness": ((0.68, 0.57, 0.34), 0.0, 0.55),
        "crew_tape": ((0.78, 0.71, 0.52), 0.0, 0.3),
        "crew_lacing": ((0.86, 0.85, 0.80), 0.0, 0.8),
        "crew_clamp": ((0.68, 0.53, 0.22), 0.0, 0.3),
        "crew_seam": ((0.36, 0.36, 0.34), 0.0, 0.6),
        "crew_blue": ((0.10, 0.30, 0.60), 0.6, 0.35),
    }
    if name in specs:
        new = bpy.data.materials.get(name) is None
        m = M.define(name, *specs[name])
        if new and name in ("crew_grip", "crew_armrest", "crew_boot", "crew_seat_cover", "crew_webbing_olive", "crew_webbing_white"):
            _detail(m, name)
        return m
    if name in ("crew_console_panel", "crew_cw_flag") or name.startswith("crew_decal_"):
        return _tex_mat(name)
    return M.get(name)


def _detail(m, name):
    """Small procedural surface detail: clear coat on the moulded grip, a fine woven-leather
    bump on the armrest pad, crinkle bump on the aluminised boot fabric."""
    nt = m.node_tree
    bsdf = nt.nodes["Principled BSDF"]
    if name == "crew_grip":
        for k, val in (("Coat Weight", 0.35), ("Coat Roughness", 0.15)):
            if k in bsdf.inputs:
                bsdf.inputs[k].default_value = val
        return
    tex = nt.nodes.new("ShaderNodeTexNoise")
    tex.inputs["Scale"].default_value = {"crew_armrest": 900.0, "crew_boot": 140.0}.get(name, 1500.0)
    tex.inputs["Detail"].default_value = 4.0
    bump = nt.nodes.new("ShaderNodeBump")
    bump.inputs["Strength"].default_value = 0.25 if name == "crew_armrest" else 0.4
    bump.inputs["Distance"].default_value = 0.0005
    nt.links.new(tex.outputs["Fac"], bump.inputs["Height"])
    nt.links.new(bump.outputs["Normal"], bsdf.inputs["Normal"])
    if name == "crew_armrest" and "Sheen Weight" in bsdf.inputs:
        bsdf.inputs["Sheen Weight"].default_value = 0.3


def _tex_path(fname):
    p = os.path.join(TEX_DIR, fname)
    if not os.path.exists(p):  # Blender's Python has no PIL: generate with the project venv
        py = os.path.join(HERE, "..", ".venv", "bin", "python")
        try:
            subprocess.run([py if os.path.exists(py) else "python3", os.path.join(HERE, "crew_textures.py")], check=True)
        except (OSError, subprocess.CalledProcessError) as err:
            print(f"crew: could not generate {fname}: {err}")
    return p if os.path.exists(p) else None


def _tex_mat(name):
    m = bpy.data.materials.get(name)
    if m:
        return m
    fname, rough, fallback = {"crew_console_panel": ("console_panel.png", 0.5, (0.02, 0.02, 0.02)),
                              "crew_cw_flag": ("console_cwflag.png", 0.35, (0.75, 0.58, 0.2)),
                              "crew_decal_hc": ("crew_hc_instructions.png", 0.45, (0.8, 0.8, 0.75)),
                              "crew_decal_armrest": ("crew_armrest_pn.png", 0.35, (0.5, 0.5, 0.48)),
                              "crew_decal_seat29": ("crew_seat_pn_29.png", 0.85, (0.62, 0.40, 0.15)),
                              "crew_decal_seat30": ("crew_seat_pn_30.png", 0.85, (0.62, 0.40, 0.15)),
                              "crew_decal_belt": ("crew_belt_placard.png", 0.35, (0.6, 0.6, 0.58))}[name]
    m = M.define(name, fallback, 0.0, rough)
    path = _tex_path(fname)
    if path:
        nt = m.node_tree
        bsdf = nt.nodes["Principled BSDF"]
        tex = nt.nodes.new("ShaderNodeTexImage")
        tex.image = bpy.data.images.load(path, check_existing=True)
        tex.interpolation = "Cubic"
        tex.extension = "EXTEND"
        nt.links.new(tex.outputs["Color"], bsdf.inputs["Base Color"])
        if name == "crew_cw_flag":
            bsdf.inputs["Metallic"].default_value = 0.8
    return m


# ---------------------------------------------------------------- geometry accumulation

class Parts:
    """Geometry buckets keyed by (material, smooth, uv-projector); flushed to one mesh each."""

    def __init__(self, prefix, parent):
        self.prefix, self.parent, self.b = prefix, parent, {}

    def get(self, mat, smooth=False, uv=None, variant=None):
        return self.b.setdefault((mat, smooth, uv, variant), ([], []))

    def flush(self, bevel=None):
        for (mat, smooth, uv, variant), (v, f) in self.b.items():
            if not f:
                continue
            uvs = [UV_FUNCS[uv](p) for p in v] if uv else None
            vs = f"_{variant.split(':')[1]}" if variant else ""
            ob = add_mesh(f"{self.prefix}_{mat.replace('crew_', '')}{'_s' if smooth else ''}{vs}", v, f,
                          _mat(mat), self.parent, uvs=uvs, smooth=smooth, variant=variant)
            bm = bmesh.new()
            bm.from_mesh(ob.data)
            bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
            bm.to_mesh(ob.data)
            bm.free()
            if not uv:
                _box_uvs(ob.data)
            if bevel and mat in bevel:
                mod = ob.modifiers.new("bevel", "BEVEL")
                mod.width = bevel[mat]
                mod.segments = 2
                mod.limit_method = "ANGLE"
        self.b = {}


UV_FUNCS = {}


def _box_uvs(me):
    """Cube-projected UVs in metres (per face, dominant normal axis) so the tiling texture sets
    from materials.upgrade_all() land at real-world scale on every crew mesh."""
    uv = me.uv_layers.new(name="UVMap")
    co = [v.co for v in me.vertices]
    for poly in me.polygons:
        n = poly.normal
        ax = max(range(3), key=lambda i: abs(n[i]))
        u_i, v_i = [(1, 2), (0, 2), (0, 1)][ax]
        sgn = 1.0 if n[ax] >= 0 else -1.0
        for li in poly.loop_indices:
            p = co[me.loops[li].vertex_index]
            uv.data[li].uv = (sgn * p[u_i], p[v_i])


def obox(acc, c, ax, ay, az, sx, sy, sz):
    """Oriented box: centre c, unit axes, full sizes (metres)."""
    v, f = acc
    b = len(v)
    for k in (-1, 1):
        for j in (-1, 1):
            for i in (-1, 1):
                v.append(c + ax * (i * sx / 2) + ay * (j * sy / 2) + az * (k * sz / 2))
    for q in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        f.append(tuple(b + i for i in q))


def abox(acc, lo, hi):
    """Axis-aligned box between two world corners."""
    lo, hi = Vector(lo), Vector(hi)
    obox(acc, (lo + hi) / 2, X, Y, Z, *(hi - lo))


def rbox(acc, c, ax, ay, az, size, r, n=3):
    """Rounded (pillow) box: lattice on the cube surface projected onto rounded corners."""
    v, f = acc
    half = [s / 2 for s in size]
    r = min(r, *[h * 0.99 for h in half])

    def samples(h):
        a = [-(h - r) - r * math.cos(math.pi / 2 * j / n) for j in range(n + 1)]
        return a + [-x for x in reversed(a)]

    S = [samples(h) for h in half]
    N = [len(s) for s in S]
    idx = {}

    def vid(i, j, k):
        key = (i, j, k)
        if key not in idx:
            p = [S[0][i], S[1][j], S[2][k]]
            inner = [max(-(h - r), min(h - r, x)) for x, h in zip(p, half)]
            d = Vector(p) - Vector(inner)
            q = Vector(inner) + (d.normalized() * r if d.length > 1e-12 else d)
            idx[key] = len(v)
            v.append(c + ax * q.x + ay * q.y + az * q.z)
        return idx[key]

    for axis in range(3):
        o = [a for a in range(3) if a != axis]
        for side in (0, N[axis] - 1):
            for a in range(N[o[0]] - 1):
                for b2 in range(N[o[1]] - 1):
                    quad = []
                    for da, db in ((0, 0), (1, 0), (1, 1), (0, 1)):
                        ijk = [0, 0, 0]
                        ijk[axis], ijk[o[0]], ijk[o[1]] = side, a + da, b2 + db
                        quad.append(vid(*ijk))
                    f.append(tuple(quad))


def fillet(pts, rad, seg=6, closed=False):
    """Polyline with rounded corners (quadratic Bezier fillets)."""
    pts = [Vector(p) for p in pts]
    n = len(pts)
    out = []
    rng = range(n) if closed else range(1, n - 1)
    if not closed:
        out.append(pts[0])
    for i in rng:
        a, b, c = pts[i - 1], pts[i], pts[(i + 1) % n]
        d1, d2 = (a - b), (c - b)
        dist = min(rad, d1.length / 2, d2.length / 2)
        p1, p2 = b + d1.normalized() * dist, b + d2.normalized() * dist
        for k in range(seg + 1):
            t = k / seg
            out.append((1 - t) ** 2 * p1 + 2 * (1 - t) * t * b + t * t * p2)
    if not closed:
        out.append(pts[-1])
    else:
        out.append(out[0].copy())
    return out


def sweep(acc, pts, r, sides=12, caps=True):
    """Circular tube along a polyline with parallel-transported frames. r may be a list."""
    v, f = acc
    pts = [Vector(p) for p in pts]
    rs = r if isinstance(r, (list, tuple)) else [r] * len(pts)
    n = len(pts)
    t_prev = (pts[1] - pts[0]).normalized()
    a, _ = _frame(t_prev)
    base = len(v)
    for i, p in enumerate(pts):
        t = (pts[min(i + 1, n - 1)] - pts[max(i - 1, 0)]).normalized()
        a = (a - t * a.dot(t)).normalized()
        b = t.cross(a)
        for k in range(sides):
            th = 2 * math.pi * k / sides
            v.append(p + rs[i] * (math.cos(th) * a + math.sin(th) * b))
    for i in range(n - 1):
        for k in range(sides):
            k2 = (k + 1) % sides
            f.append((base + i * sides + k, base + i * sides + k2, base + (i + 1) * sides + k2, base + (i + 1) * sides + k))
    if caps:
        f.append(tuple(base + k for k in reversed(range(sides))))
        f.append(tuple(base + (n - 1) * sides + k for k in range(sides)))


def tube(acc, p0, p1, r, sides=12):
    sweep(acc, [p0, p1], r, sides)


def lathe(acc, base, axis, prof, sides=24, closed=False):
    """Solid of revolution: prof = [(radius, height along axis)], capped at both ends, or
    with closed=True the profile is a closed loop (a ring such as a bezel)."""
    v, f = acc
    axis = Vector(axis).normalized()
    a, b = _frame(axis)
    base = Vector(base)
    b0 = len(v)
    for r, h in prof:
        for k in range(sides):
            th = 2 * math.pi * k / sides
            v.append(base + axis * h + r * (math.cos(th) * a + math.sin(th) * b))
    n = len(prof)
    for i in range(n if closed else n - 1):
        i2 = (i + 1) % n
        for k in range(sides):
            k2 = (k + 1) % sides
            f.append((b0 + i * sides + k, b0 + i * sides + k2, b0 + i2 * sides + k2, b0 + i2 * sides + k))
    if not closed:
        f.append(tuple(b0 + k for k in reversed(range(sides))))
        f.append(tuple(b0 + (n - 1) * sides + k for k in range(sides)))


def strip(acc, pts, wdir, width, thick):
    """Flat webbing ribbon along a polyline; wdir = width direction (Vector or list)."""
    v, f = acc
    pts = [Vector(p) for p in pts]
    n = len(pts)
    base = len(v)
    for i, p in enumerate(pts):
        t = (pts[min(i + 1, n - 1)] - pts[max(i - 1, 0)]).normalized()
        w = Vector(wdir[i] if isinstance(wdir, list) else wdir)
        nn = t.cross(w).normalized()
        w = nn.cross(t).normalized()
        for sw, sn in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            v.append(p + w * (sw * width / 2) + nn * (sn * thick / 2))
    for i in range(n - 1):
        for k in range(4):
            k2 = (k + 1) % 4
            f.append((base + 4 * i + k, base + 4 * i + k2, base + 4 * (i + 1) + k2, base + 4 * (i + 1) + k))
    f.append(tuple(base + k for k in (3, 2, 1, 0)))
    f.append(tuple(base + 4 * (n - 1) + k for k in range(4)))


def prism(acc, poly, d, depth):
    """Extrude a planar polygon (world points) along unit d by depth."""
    v, f = acc
    b = len(v)
    n = len(poly)
    for p in poly:
        v.append(Vector(p))
    for p in poly:
        v.append(Vector(p) + d * depth)
    f.append(tuple(b + i for i in range(n)))
    f.append(tuple(b + n + i for i in reversed(range(n))))
    for i in range(n):
        j = (i + 1) % n
        f.append((b + i, b + j, b + n + j, b + n + i))


# ---------------------------------------------------------------- seats

def _pan_outline(sf, sr, width, rf=2.0, ra=2.6, rb=3.0, n=6):
    """Seat-pan cover outline (STA, distance from the inboard tube): rounded notches at the front
    corners expose the white tube bends; rectangular notches at the rear corners clear the hinges
    (QTU NASM-NASM2020-00056)."""
    A0, A1, B0, B1 = sf, sr, 0.0, width
    pts = []

    def arc(ca, cb, a0, a1):
        for k in range(n + 1):
            t = a0 + (a1 - a0) * k / n
            pts.append((ca + rf * math.cos(t), cb + rf * math.sin(t)))

    arc(A0, B0, math.pi / 2, 0.0)                      # front-inboard notch
    arc(A0, B1, math.pi, math.pi / 2)                  # front-outboard notch (concave)
    pts += [(A1 - ra, B1), (A1 - ra, B1 - rb), (A1, B1 - rb), (A1, B0 + rb), (A1 - ra, B0 + rb), (A1 - ra, B0)]
    return pts


def buckle(g, c, ax, ay, az, variant, s, closed=False):
    """Seat-belt buckle (DWG 5.1 detail): steel body, chrome release lever, belt bar; when closed the
    outboard half's hook tongue is engaged and the lever lies flat."""
    obox(g.get("steel", variant=variant), c, ax, ay, az, 2.6 * IN, 1.9 * IN, 0.28 * IN)
    lever_tilt = 0.0 if closed else 0.35
    lz = (az * math.cos(lever_tilt) + ax * math.sin(lever_tilt)).normalized()
    obox(g.get("chrome", variant=variant), c + az * 0.24 * IN + ax * (0.2 * IN), ax, ay, lz, 1.9 * IN, 1.0 * IN, 0.16 * IN)
    tube(g.get("steel", True, None, variant), c - ax * 1.2 * IN - ay * 0.8 * IN, c - ax * 1.2 * IN + ay * 0.8 * IN, 0.1 * IN, 8)
    if closed:
        obox(g.get("steel", variant=variant), c + ax * 1.5 * IN, ax, ay, az, 0.9 * IN, 1.6 * IN, 0.12 * IN)


def hook_plate(g, c, nrm, along, variant):
    """Outboard-half adjuster/hook plate with its marking placard (crew_belt_placard.png)."""
    up = nrm.cross(along).normalized()
    obox(g.get("steel", variant=variant), c, along, up, nrm, 1.9 * IN, 1.2 * IN, 0.2 * IN)
    decal(g.parent, f"{g.prefix}_belt_placard_{variant.split(':')[1]}", c + nrm * 0.11 * IN, along, up,
          1.6 * IN, 0.9 * IN, "crew_decal_belt", variant)


def decal(parent, name, c, au, av, w, h, mat, variant=None):
    """Thin textured placard quad (UV 0..1), facing au x av."""
    au, av = Vector(au).normalized(), Vector(av).normalized()
    vs = [c - au * w / 2 - av * h / 2, c + au * w / 2 - av * h / 2, c + au * w / 2 + av * h / 2, c - au * w / 2 + av * h / 2]
    return add_mesh(name, vs, [(0, 1, 2, 3)], _mat(mat), parent, uvs=[(0, 0), (1, 0), (1, 1), (0, 1)], variant=variant)


def seat(ch, s):
    """Seat base, cover, bag, hinge hardware, belt, handhold (on the chassis) and the folding
    seat back / PLSS support (own link). Matched to QTU photos -00056 / -00047 and, for the
    LRV-3 flight strap layout (3 olive straps), AS17-147-22526."""
    side = "l" if s > 0 else "r"
    bi, bo = s * SEAT_BL_IN, s * SEAT_BL_OUT
    bc = (bi + bo) / 2
    width = SEAT_BL_OUT - SEAT_BL_IN
    sf, sr, w = SEAT_STA_F, SEAT_STA_R, SEAT_WL
    tr = SEAT_TUBE_R * IN
    g = Parts(f"seat_{side}", ch)

    def P2(a, b, z):            # (STA, distance from the inboard tube, WL)
        return H(a, bi + s * b, z)

    # seat base: white-painted tube loop with bent corners
    loop = fillet([H(sf, bi, w), H(sf, bo, w), H(sr, bo, w), H(sr, bi, w)], 1.8 * IN, 8, closed=True)
    sweep(g.get("crew_frame_white", True), loop, tr, 14, caps=False)
    # cover binding wraps the side and front tubes (corners and rear tube stay bare)
    for b in (bi, bo):
        tube(g.get("crew_edge_tape", True), H(sf + 2.2, b, w), H(sr - 2.8, b, w), tr * 1.18, 14)
    tube(g.get("crew_edge_tape", True), H(sf, bi + s * 2.2, w), H(sf, bo - s * 2.2, w), tr * 1.18, 14)
    # woven cover with notched corners and a glossy rust binding round the edge
    outline = _pan_outline(sf, sr, width)
    poly = [P2(a, b, w + 0.47) for a, b in outline]
    prism(g.get("crew_seat_cover", False), poly, -Z, 0.07 * IN)
    sweep(g.get("crew_edge_tape", True), poly + [poly[0]], 0.12 * IN, 8, caps=False)
    # the binding is a ~1 in band of Teflon-coated glass tape folded over the cover edge
    inner = [P2(a, b, w + 0.49) for a, b in _pan_outline(sf + 1.0, sr - 1.0, width - 2.0, rf=2.0, ra=1.6, rb=2.0)]
    inner = [p + Y * (s * 1.0 * IN) for p in inner]
    for (p0, p1), (q0, q1) in zip(zip(poly, poly[1:] + poly[:1]), zip(inner, inner[1:] + inner[:1])):
        v_, f_ = g.get("crew_edge_tape", False)
        b0 = len(v_)
        v_ += [p0, p1, q1, q0, p0 + Z * 0.02 * IN, p1 + Z * 0.02 * IN, q1 + Z * 0.02 * IN, q0 + Z * 0.02 * IN]
        f_ += [(b0 + 4, b0 + 5, b0 + 6, b0 + 7), (b0 + 3, b0 + 2, b0 + 1, b0)]
    # tan strap running aft from the front tube with a velcro tab (inboard half)
    sb_ = width * 0.62
    strip(g.get("crew_pan_strap"), [P2(sf - 0.7, sb_, w - 0.7), P2(sf - 0.2, sb_, w + 0.6), P2(sf + 1.0, sb_, w + 0.6),
                                     P2(sf + 8.5, sb_, w + 0.6)], Y, 3.0 * IN, 0.06 * IN)
    obox(g.get("crew_velcro_dark"), P2(sf + 8.6, sb_ - 0.6, w + 0.66), X, Y, Z, 2.2 * IN, 2.4 * IN, 0.06 * IN)
    # pink foam block on the rear tube, -Y half
    fb = (SEAT_BL_IN + SEAT_BL_OUT) / 2 * s - 5.25
    rbox(g.get("crew_foam", True), H(sr - 0.3, fb, w + 1.0), X, Y, Z, (1.7 * IN, 9.0 * IN, 1.5 * IN), 0.2 * IN, 2)

    # covered stowage bag hanging beneath the seat (DWG 5.1 note 2): white Beta cloth, tapered
    rings = []
    for z, sc in ((110.15, 1.0), (109.7, 1.0), (104.0, 0.97), (101.9, 0.94), (101.35, 0.86)):
        rings.append((H((sf + sr) / 2 + 0.9, bc, z), X, Y, 14.6 / 2 * sc * IN, (width - 2.2) / 2 * sc * IN, 0.18))
    sloft(g.get("crew_bag", True), rings, 32)
    for z in (109.0, 102.6):     # stitched seams
        sweep(g.get("crew_lacing", True), [H((sf + sr) / 2 + 0.9, bc, z) + X * (14.62 / 2 * IN * math.cos(t)) +
                                           Y * ((width - 2.18) / 2 * IN * math.sin(t)) for t in
                                           [2 * math.pi * k / 48 for k in range(49)]], 0.02 * IN, 4, caps=False)

    # front seat-support legs into chassis sockets, rear diagonal links
    for b in (0.7, width - 0.7):
        tube(g.get("aluminium", True), P2(sf + 3.5, b, 102.95), P2(sf + 3.5, b, w - 0.4), 0.4 * IN, 14)
        obox(g.get("aluminium"), P2(sf + 3.5, b, w - 0.35), X, Y, Z, 1.1 * IN, 1.1 * IN, 0.7 * IN)
        abox(g.get("aluminium"), P2(sf + 4.1, b, 102.95) - Y * 0.5 * IN, P2(sf + 2.9, b, 103.6) + Y * 0.5 * IN)
        tube(g.get("aluminium", True), P2(sr - 4.0, b, w), P2(sr + 3.0, b, 103.3), 0.36 * IN)
        abox(g.get("aluminium"), P2(sr + 3.6, b, 102.95) - Y * 0.5 * IN, P2(sr + 2.4, b, 103.6) + Y * 0.5 * IN)

    # PLSS-support lower stubs, hinge clevises, gold-anodised support links with blue-locked bolts
    d = Vector((-math.sin(BACK_LEAN), 0, math.cos(BACK_LEAN)))
    for b in BACK_POST_BL:
        top = H(sr, s * b, w)
        bot = top - d * (7.3 * IN / math.cos(BACK_LEAN))
        tube(g.get("aluminium", True), bot, top - d * 0.6 * IN, tr, 14)
        obox(g.get("aluminium"), bot + Z * 0.1 * IN, X, Y, Z, 1.3 * IN, 1.1 * IN, 1.2 * IN)
        obox(g.get("aluminium"), top, X, Y, Z, 1.3 * IN, 1.25 * IN, 1.4 * IN)        # hinge clevis
        tube(g.get("steel", True), H(sr, s * b - 0.75, w), H(sr, s * b + 0.75, w), 0.2 * IN, 10)
        lo = bot + X * 1.6 * IN + Z * 1.0 * IN
        hi = H(sr - 2.6, s * b, w - 0.5)
        lb = (hi - lo)
        side_off = Y * (s * 0.7 * IN if b == BACK_POST_BL[1] else -s * 0.7 * IN)
        obox(g.get("crew_gold"), (lo + hi) / 2 + side_off, lb.normalized(), Y, lb.normalized().cross(Y),
             lb.length + 0.8 * IN, 0.12 * IN, 0.55 * IN)
        for p in (lo, hi):
            lathe(g.get("crew_blue", True), p + side_off * 1.12, Y * (1 if side_off.y > 0 else -1),
                  [(0.16 * IN, 0), (0.16 * IN, 0.1 * IN)], 6)

    # outboard handhold (inverted U on the chassis side) and outboard toe hold
    ob = s * 27.3
    sweep(g.get("aluminium", True), fillet([H(75.5, ob, 102.95), H(75.5, ob, 107.8), H(80.5, ob, 107.8),
                                            H(80.5, ob, 102.95)], 1.4 * IN, 6), 0.45 * IN, 14)
    sweep(g.get("aluminium", True), fillet([H(48.5, s * 26.4, 100.6), H(49.5, s * 26.4, 104.0),
                                            H(52.5, s * 26.4, 104.0), H(53.0, s * 26.4, 100.6)], 1.0 * IN, 5), 0.35 * IN)

    # seat belts (herringbone, rust-orange) as a variant set. Anchors from DWG 5.1 seat-belt detail:
    # inboard half on the hand-controller housing side, outboard half on the outboard handhold.
    # "stowed": unbuckled as on the QTU, inboard half lying on the pan, outboard half hanging.
    VS = "SeatBelts:stowed"
    inb = fillet([H(77.0, s * 3.45, 113.6), H(77.4, s * 3.9, 113.4), H(78.2, s * 4.3, w + 0.62),
                  H(79.5, s * 7.5, w + 0.62)], 0.8 * IN, 4)
    strip(g.get("crew_belt", variant=VS), inb, X, 1.75 * IN, 0.09 * IN)
    buckle(g, H(80.6, s * 8.6, w + 0.75), X, Y, Z, VS, s)
    outb = fillet([H(78.0, s * 27.3, 107.9), H(78.2, s * 27.9, 107.2), H(78.6, s * 28.2, 104.5),
                   H(79.4, s * 28.1, 102.2)], 0.8 * IN, 4)
    strip(g.get("crew_belt", variant=VS), outb, X, 1.75 * IN, 0.09 * IN)
    hook_plate(g, H(79.5, s * 28.15, 101.8), Y * s, X, VS)
    # "buckled": worn across the lap of a seated, suited crewman (DWG 5.1 front view; AS17-147-22526),
    # arching over the suit thighs ~7.5 in above the pan, buckle just inboard of the seat centre
    VB = "SeatBelts:buckled"
    lap = [H(77.0, s * 3.45, 113.6), H(78.6, s * 5.2, 116.6), H(80.6, s * 8.0, 118.5), H(81.4, s * 11.6, 119.0),
           H(81.4, s * 16.0, 118.9), H(80.8, s * 20.0, 117.8), H(79.8, s * 23.6, 114.8), H(78.6, s * 26.0, 110.8),
           H(78.0, s * 27.3, 107.9)]
    lap = fillet(lap, 1.6 * IN, 5)
    strip(g.get("crew_belt", variant=VB), lap, X, 1.75 * IN, 0.09 * IN)
    p0 = H(81.4, s * 12.6, 119.05)
    t = (H(81.4, s * 16.0, 118.9) - H(81.4, s * 11.6, 119.0)).normalized()
    up = Vector((0, 0, 1))
    buckle(g, p0 + up * 0.12 * IN, t, up.cross(t).normalized(), up, VB, s, closed=True)
    hook_plate(g, H(78.4, s * 27.0, 108.6), Y * s, X, VB)
    g.flush()

    # ---- seat back / PLSS support: revolute fold joint at the pan-rear hinge
    hinge = H(sr, bc, w)
    back = link(f"seat_back_{side}", hinge, ch, {
        "type": "revolute", "axis": (0.0, 1.0, 0.0), "limit": (0.0, BACK_FOLD), "role": "seat_fold",
        "drive": dict(SEAT_DRIVE)}, mass=MASS["seat_back"])
    gb = Parts(f"seat_back_{side}", back)
    nrm = Vector((math.cos(BACK_LEAN), 0, math.sin(BACK_LEAN)))   # faces forward (toward the occupant)

    def bp(b, h, off=0.0):
        return H(sr, b, w) + d * (h * IN) + nrm * (off * IN)

    pin, pout = s * BACK_POST_BL[0], s * BACK_POST_BL[1]
    l_in = BACK_H_IN / math.cos(BACK_LEAN) + 0.4
    l_out = BACK_H_OUT / math.cos(BACK_LEAN) + 0.4
    al = gb.get("aluminium", True)
    tube(al, bp(pin, -0.6), bp(pin, l_in), tr, 14)
    tube(al, bp(pout, -0.6), bp(pout, l_out), tr, 14)
    for b, l in ((pin, l_in), (pout, l_out)):     # white Teflon caps, domed
        lathe(gb.get("crew_knob_white", True), bp(b, l - 0.1), d, [(tr * 1.06, 0), (tr * 1.06, 0.35 * IN),
                                                                   (tr * 0.8, 0.55 * IN), (tr * 0.35, 0.65 * IN)], 16)
    top_in, top_out = l_in - 0.8, l_out - 0.8
    tube(al, bp(pin, top_in), bp(pout, top_out), 0.45 * IN, 14)
    tube(al, bp(pin, 2.4), bp(pout, 2.4), 0.42 * IN, 14)
    for b, h in ((pin, top_in), (pout, top_out), (pin, 2.4), (pout, 2.4)):   # weld fillets at the tee joints
        sweep(al, ring_pts(bp(b, h), d, tr * 1.05, 14), 0.07 * IN, 6, caps=False)

    def top_at(b):
        f = (b - pin) / (pout - pin)
        return top_in + f * (top_out - top_in)

    # horizontal olive herringbone straps (3 on the LRV-3 flight seats), wrapped round both posts
    hs = (5.8, 9.9, 14.0)
    for h in hs:
        strip(gb.get("crew_webbing_olive"), [bp(pin, h), bp(pout, h)], d, 2.0 * IN, 0.07 * IN)
        for b in (pin, pout):
            lathe(gb.get("crew_webbing_olive", True), bp(b, h - 1.0), d, [(tr + 0.07 * IN, 0), (tr + 0.07 * IN, 2.0 * IN)], 14)
            lathe(gb.get("crew_edge_tape", True), bp(b, h - 1.0), d, [(tr + 0.09 * IN, 0), (tr + 0.09 * IN, 0.12 * IN)], 14)
    # two vertical white straps in front of the olive ones, looped round the bottom cross tube and
    # over the top tube, carrying tan velcro patch covers with hook tabs and stitched channels
    for frac in (0.33, 0.67):
        b = pin + (pout - pin) * frac
        tt = top_at(b)
        pts = [bp(b, 3.6, -0.5), bp(b, 2.4, -0.5), bp(b, 1.85, 0.0), bp(b, 2.4, 0.5), bp(b, 3.3, 0.2),
               bp(b, tt - 0.6, 0.2), bp(b, tt, 0.55), bp(b, tt + 0.5, 0.0), bp(b, tt, -0.55), bp(b, tt - 2.5, -0.6)]
        strip(gb.get("crew_webbing_white"), pts, Y, 3.0 * IN, 0.06 * IN)
        p_top, p_bot = tt - 0.7, 7.4
        c = bp(b, (p_top + p_bot) / 2, 0.32)
        obox(gb.get("crew_velcro_tan"), c, Y, d, nrm, 5.5 * IN, (p_top - p_bot) * IN, 0.1 * IN)
        for du in (-1.3, 0.0, 1.3):
            obox(gb.get("crew_lacing"), bp(b + du, (p_top + p_bot) / 2 - 0.2, 0.385), Y, d, nrm,
                 0.03 * IN, (p_top - p_bot - 0.8) * IN, 0.02 * IN)
        obox(gb.get("crew_velcro_hook"), bp(b + 1.75, p_bot - 1.0, 0.3), Y, d, nrm, 1.8 * IN, 2.4 * IN, 0.1 * IN)
        dash = 29 if frac < 0.5 else 30          # inked part numbers (QTU 00056: 209-31006-29 / -30)
        decal(back, f"seat_back_{side}_pn{dash}", bp(b + (-2.15 if frac < 0.5 else 2.15), (p_top + p_bot) / 2, 0.385),
              d, -Y, 2.6 * IN, 0.32 * IN, f"crew_decal_seat{dash}")
        strip(gb.get("crew_velcro_tan"), [bp(b, p_top - 0.3, 0.32), bp(b, tt, 0.65), bp(b, tt + 0.55, 0.0), bp(b, tt - 0.5, -0.6)],
              Y, 5.5 * IN, 0.08 * IN)
    # velcro loop sleeves on the top tube, brown snap straps on the bottom cross tube
    for f0, f1 in ((0.04, 0.18), (0.44, 0.56)):
        b0, b1 = pin + (pout - pin) * f0, pin + (pout - pin) * f1
        tube(gb.get("crew_velcro_white", True), bp(b0, top_at(b0)), bp(b1, top_at(b1)), 0.58 * IN, 12)
    for f in (0.12, 0.88):
        b = pin + (pout - pin) * f
        strip(gb.get("crew_strap_brown"), [bp(b, 2.4, 0.5), bp(b, 1.8, 0.55), bp(b, -1.8, 0.6)], Y, 1.25 * IN, 0.06 * IN)
        for h in (-1.4, -0.6, 0.2):
            lathe(gb.get("chrome", True), bp(b, h, 0.6), nrm, [(0.2 * IN, 0), (0.2 * IN, 0.06 * IN), (0.1 * IN, 0.1 * IN)], 12)
    gb.flush()


# ---------------------------------------------------------------- footrests

def footrests(ch):
    g = Parts("footrest", ch)
    h0, t0 = H(FOOT_HINGE[0], 0, FOOT_HINGE[1]), H(FOOT_TOP[0], 0, FOOT_TOP[1])
    along = (t0 - h0).normalized()
    nrm = along.cross(Y)                     # aft and up: the foot side
    length = (t0 - h0).length
    th = 0.6 * IN
    for s in (1, -1):
        b0, b1 = s * FOOT_BL[0], s * FOOT_BL[1]
        bc = (b0 + b1) / 2
        c = (h0 + t0) / 2 + Vector((0, P.bl(bc), 0)) + nrm * (th / 2)
        obox(g.get("crew_honeycomb"), c, along, Y, nrm, length, 24.0 * IN, th)
        # edge channels
        for b in (b0, b1):
            obox(g.get("aluminium"), (h0 + t0) / 2 + Vector((0, P.bl(b), 0)) + nrm * (th / 2), along, Y, nrm,
                 length, 0.25 * IN, th * 1.25)
        tube(g.get("steel", True), h0 + Vector((0, P.bl(b0), 0)), h0 + Vector((0, P.bl(b1), 0)), 0.32 * IN)
        # Velcro hold-down strap from the top edge to the cross member front face
        tm = t0 + Vector((0, P.bl(bc), 0))
        strip(g.get("crew_webbing_white"), [tm + nrm * th, tm + along * 0.25 * IN - nrm * 0.1 * IN,
                                            H(44.0, bc, 103.5), H(44.1, bc, 101.0)], Y, 1.5 * IN, 0.06 * IN)
        # toe restraint (wingtip) on the inboard edge only
        base = h0 + along * (length - 3.8 * IN) + Vector((0, P.bl(b0) - s * 0.06 * IN, 0))
        poly = [base]
        for k in range(9):
            a = math.pi / 2 * k / 8
            poly.append(base + along * (3.7 * IN * math.cos(a)) + nrm * (3.7 * IN * math.sin(a) + th))
        prism(g.get("aluminium"), poly, Y * s, 0.12 * IN)
    g.flush()


# ---------------------------------------------------------------- console

def _panel_frame():
    th = PANEL_TILT
    top = H(PANEL_TOP[0], 0, PANEL_TOP[1])
    ru = Vector((0, -1, 0))                               # +u: toward the crew's right
    dv = Vector((-math.sin(th), 0, -math.cos(th)))        # +v: down the faceplate
    n = Vector((-math.cos(th), 0, math.sin(th)))          # outward: aft and up

    def pp(u, v, h=0.0):
        return top + ((u - PANEL_W / 2) * ru + v * dv + h * n) * IN

    def uv(p):
        rel = (p - top) / IN
        u, v = PANEL_W / 2 + rel.dot(ru), rel.dot(dv)
        return ((u + 2.6) / 12.6, 1.0 - v / 12.0)          # crew_textures U0..U1, V0..V1

    return pp, ru, -dv, n, uv


def toggle(g, pp, ax, ay, n, u, v, lever_up=True, collar="crew_switch_collar"):
    """Lever-lock toggle: hex nut, coloured lock collar, flat ribbed paddle (7021041)."""
    lathe(g.get("steel", True), pp(u, v, 0.0), n, [(0.17 * IN, 0), (0.17 * IN, 0.07 * IN)], 6)
    lathe(g.get(collar, True), pp(u, v, 0.07), n, [(0.15 * IN, 0), (0.15 * IN, 0.12 * IN), (0.1 * IN, 0.16 * IN)], 16)
    tilt = math.radians(18) * (1 if lever_up else -1)
    dirn = (n * math.cos(tilt) + ay * math.sin(tilt)).normalized()
    side = ax
    up = dirn.cross(side).normalized()
    c = pp(u, v, 0.2) + dirn * 0.2 * IN
    obox(g.get("crew_toggle"), c, side, up, dirn, 0.28 * IN, 0.17 * IN, 0.4 * IN)
    for k in range(3):   # ribs
        obox(g.get("crew_toggle"), c + up * (0.09 * IN) + dirn * ((k - 1) * 0.11 * IN), side, up, dirn,
             0.29 * IN, 0.02 * IN, 0.04 * IN)


def breaker(g, pp, n, u, v):
    """Push-pull circuit breaker: bright collar, black button."""
    lathe(g.get("aluminium_spun", True), pp(u, v, 0), n, [(0.2 * IN, 0), (0.2 * IN, 0.06 * IN), (0.16 * IN, 0.12 * IN)], 16)
    lathe(g.get("crew_cb_knob", True), pp(u, v, 0.1), n, [(0.14 * IN, 0), (0.14 * IN, 0.3 * IN), (0.11 * IN, 0.36 * IN)], 16)


def guard(g, pp, u, v0, v1, inward):
    """Bent-wire switch guard: out from the panel, along v, back in; ends turned toward the switch."""
    pts = [pp(u + inward * 0.12, v0, 0.0), pp(u, v0, 0.0), pp(u, v0, 0.55), pp(u, v1, 0.55), pp(u, v1, 0.0),
           pp(u + inward * 0.12, v1, 0.0)]
    sweep(g.get("steel", True), fillet(pts, 0.12 * IN, 3), 0.035 * IN, 8)


def screw(g, pp, n, u, v):
    lathe(g.get("steel", True), pp(u, v, 0), n, [(0.07 * IN, 0), (0.07 * IN, 0.04 * IN), (0.04 * IN, 0.06 * IN)], 10)


def console(ch):
    pp, ax, ay, n, uvf = _panel_frame()
    UV_FUNCS["panel"] = uvf
    g = Parts("console", ch)
    T = "crew_console_panel"

    # faceplate (UV-mapped legends) on the housing's aft face
    obox(g.get(T, False, "panel"), pp(5, 6, -0.045), ax, ay, n, PANEL_W * IN, PANEL_H * IN, 0.09 * IN)

    # housing: side profile extruded across the console width
    hw = 4.95
    prof = [pp(5, 0, -0.09), H(CONSOLE_FRONT_STA + 1.2, 0, 125.9), H(CONSOLE_FRONT_STA, 0, 125.0),
            H(CONSOLE_FRONT_STA + 0.3, 0, 118.4), pp(5, 12, -0.09)]
    prof = [p + Vector((0, P.bl(-hw), 0)) for p in prof]
    prism(g.get("crew_housing"), prof, Y, 2 * hw * IN)
    # hinge strip and C&W flag on the top, stowed flat (DWG 5.2 C&W flag detail)
    tube(g.get("aluminium", True), pp(2.0, -0.05, -0.15), pp(8.0, -0.05, -0.15), 0.1 * IN)
    a, b = pp(5, 0, -0.09), H(CONSOLE_FRONT_STA + 1.2, 0, 125.9)
    topdir = (b - a).normalized()
    topn = topdir.cross(Y)
    if topn.z < 0:
        topn = -topn
    fc = a + topdir * 1.0 * IN + topn * 0.06 * IN
    flag = g.get("crew_cw_flag", False, "flag")
    obox(flag, fc, -Y, topdir, topn, 5.8 * IN, 1.6 * IN, 0.06 * IN)
    UV_FUNCS["flag"] = lambda p, c=fc, d=topdir: (0.5 + (p - c).dot(-Y) / (5.8 * IN), 0.5 + (p - c).dot(d) / (1.6 * IN))
    for k in (-1, 1):   # flag magnets
        obox(g.get("crew_bezel"), fc + Y * (k * 0.6 * IN) + topn * 0.08 * IN, Y, topdir, topn, 0.5 * IN, 0.6 * IN, 0.12 * IN)

    # heading indicator: bezel ring, recessed dial face (texture), lubber index
    hu, hv = 2.26, 2.80
    lathe(g.get("crew_bezel", True), pp(hu, hv, 0), n, [(1.62 * IN, 0.0), (1.85 * IN, 0.0), (1.85 * IN, 0.25 * IN),
                                                         (1.75 * IN, 0.35 * IN), (1.62 * IN, 0.35 * IN)], 48, closed=True)
    disc = g.get(T, False, "panel")
    lathe(disc, pp(hu, hv, 0.0), n, [(1.63 * IN, 0.0), (1.63 * IN, 0.08 * IN)], 48)
    # speed indicator: square bezel, 4 screws, dial
    su, sv = 7.53, 1.81
    obox(g.get("crew_bezel"), pp(su, sv, 0.12), ax, ay, n, 1.65 * IN, 1.65 * IN, 0.24 * IN)
    lathe(g.get(T, False, "panel"), pp(su, sv, 0.2), n, [(0.67 * IN, 0), (0.67 * IN, 0.06 * IN)], 32)
    for du in (-0.62, 0.62):
        for dv in (-0.62, 0.62):
            lathe(g.get("steel", True), pp(su + du, sv + dv, 0.24), n, [(0.07 * IN, 0), (0.07 * IN, 0.04 * IN)], 10)

    # sun shadow device (deployed, 7021041 / DWG 5.2): two bearing blocks with blue bushings carry a
    # cross shaft; a clamp hub on the shaft holds the gnomon rod, which lies just proud of the panel and
    # casts its shadow on the raised 15-0-15 deg scale below
    for mu in (4.53, 6.09):
        obox(g.get("steel"), pp(mu, 0.62, 0.15), ax, ay, n, 0.32 * IN, 0.25 * IN, 0.3 * IN)
        lathe(g.get("crew_blue", True), pp(mu + (0.16 if mu < 5 else -0.16), 0.62, 0.15), ax * (1 if mu < 5 else -1),
              [(0.09 * IN, 0), (0.09 * IN, 0.12 * IN)], 10)
        for dv in (-0.08, 0.08):
            screw(g, pp, n, mu, 0.62 + dv)
    tube(g.get("steel", True), pp(4.7, 0.62, 0.15), pp(5.92, 0.62, 0.15), 0.045 * IN, 10)
    obox(g.get("crew_bezel"), pp(5.1, 0.62, 0.2), ax, ay, n, 0.34 * IN, 0.3 * IN, 0.24 * IN)
    lathe(g.get("steel", True), pp(5.1 + 0.17, 0.62, 0.22), ax, [(0.05 * IN, 0), (0.05 * IN, 0.1 * IN)], 6)
    obox(g.get(T, False, "panel"), pp(5.105, 1.73, 0.17), ax, ay, n, 1.97 * IN, 0.74 * IN, 0.06 * IN)
    for su2 in (4.25, 5.95):
        tube(g.get("steel", True), pp(su2, 1.73, 0), pp(su2, 1.73, 0.15), 0.04 * IN, 8)
        screw(g, pp, n, su2, 1.73 + 0.0)
    tube(g.get("crew_knob_white", True), pp(5.1, 2.12, 0.27), pp(5.1, -0.95, 0.22), 0.035 * IN, 8)
    obox(g.get("crew_knob_white"), pp(5.1, 2.12, 0.27), ax, ay, n, 0.12 * IN, 0.1 * IN, 0.04 * IN)

    # NAV POWER breaker, GYRO TORQUING lever (white knob), SYSTEM RESET push button
    breaker(g, pp, n, 9.01, 1.75)
    lathe(g.get("steel", True), pp(7.0, 3.91, 0), n, [(0.22 * IN, 0), (0.22 * IN, 0.1 * IN), (0.12 * IN, 0.2 * IN)], 12)
    tube(g.get("chrome", True), pp(7.0, 3.91, 0.15), pp(7.0, 3.91, 0.45), 0.06 * IN, 10)
    lathe(g.get("crew_knob_white", True), pp(7.0, 3.91, 0.45), n, [(0.2 * IN, 0), (0.22 * IN, 0.15 * IN), (0.2 * IN, 0.4 * IN), (0.1 * IN, 0.45 * IN)], 16)
    lathe(g.get("aluminium_spun", True), pp(9.01, 3.74, 0), n, [(0.3 * IN, 0), (0.3 * IN, 0.1 * IN), (0.26 * IN, 0.14 * IN)], 20)
    lathe(g.get("crew_knob_white", True), pp(9.01, 3.74, 0.1), n, [(0.23 * IN, 0), (0.23 * IN, 0.22 * IN), (0.15 * IN, 0.3 * IN)], 20)

    # power/temperature meters: raised frames around recessed faces (texture)
    for u0, u1 in ((2.92, 3.95), (3.99, 4.98), (6.46, 7.49), (7.53, 8.52)):
        v0, v1 = 5.39, 7.74
        fr = g.get("crew_meter_frame")
        cu, cv = (u0 + u1) / 2, (v0 + v1) / 2
        obox(fr, pp(cu, v0 + 0.06, 0.12), ax, ay, n, (u1 - u0) * IN, 0.12 * IN, 0.24 * IN)
        obox(fr, pp(cu, v1 - 0.06, 0.12), ax, ay, n, (u1 - u0) * IN, 0.12 * IN, 0.24 * IN)
        obox(fr, pp(u0 + 0.06, cv, 0.12), ax, ay, n, 0.12 * IN, (v1 - v0) * IN, 0.24 * IN)
        obox(fr, pp(u1 - 0.06, cv, 0.12), ax, ay, n, 0.12 * IN, (v1 - v0) * IN, 0.24 * IN)
        obox(g.get(T, False, "panel"), pp(cu, cv, 0.04), ax, ay, n, (u1 - u0 - 0.2) * IN, (v1 - v0 - 0.2) * IN, 0.02 * IN)

    # circuit breakers (DWG 5.2 / 7021041)
    for u, v in ((1.60, 5.43), (1.60, 6.38), (1.60, 7.24), (1.60, 8.11), (1.60, 9.01),   # AUX, BUS A-D
                 (1.15, 9.96), (1.15, 10.70),                                          # +-15 VDC
                 (4.16, 8.89), (4.16, 10.29),                                          # steering fwd/rear
                 (4.90, 8.89), (5.68, 8.89), (4.90, 10.29), (5.68, 10.29)):            # drive power LF RF LR RR
        breaker(g, pp, n, u, v)
    # toggle switches
    for u, v in ((2.35, 10.33),                                    # +-15 VDC PRIM/OFF/SEC
                 (5.72, 5.80), (5.72, 7.33),                       # motor temp select, battery volts/amps
                 (9.14, 7.24),                                     # PWM select
                 (3.29, 8.93), (3.29, 10.37),                      # steering fwd/rear
                 (6.54, 8.89), (7.37, 8.89), (6.54, 10.29), (7.37, 10.29),   # drive power buses
                 (8.27, 8.89), (9.14, 8.89), (8.27, 10.29), (9.14, 10.29)):  # drive enable
        toggle(g, pp, ax, ay, n, u, v)
    # wire guards
    for u, v0, v1, inward in ((1.92, 9.75, 10.95, 1), (2.75, 9.75, 10.95, -1),
                              (8.77, 6.90, 8.00, 1), (9.71, 6.90, 8.00, -1),
                              (8.00, 8.35, 11.25, 1), (9.68, 8.35, 11.25, -1)):
        guard(g, pp, u, v0, v1, inward)
    for u, v in ((0.66, 5.39), (9.38, 5.31), (0.45, 10.45), (9.55, 10.6), (5.0, 4.57), (5.0, 11.4),
                 (0.3, 0.3), (9.7, 0.3), (0.3, 11.7), (9.7, 11.7)):
        screw(g, pp, n, u, v)

    # vehicle attitude indicator on the console's left side (7021041, DWG 5.2)
    vu, vv = -1.1, 2.55
    obox(g.get("crew_housing"), pp(vu, vv, -0.65), ax, ay, n, 1.8 * IN, 2.7 * IN, 1.3 * IN)
    obox(g.get(T, False, "panel"), pp(vu, vv, 0.03), ax, ay, n, 1.8 * IN, 2.7 * IN, 0.04 * IN)
    obox(g.get("aluminium"), pp(-0.1, vv, -0.65), ax, ay, n, 0.2 * IN, 1.6 * IN, 0.6 * IN)
    tube(g.get("steel", True), pp(-1.7, 1.05, -0.3), pp(-1.7, 0.75, -0.3), 0.05 * IN, 8)   # lock pin

    console_back(g, pp)
    hc_surround(g)
    g.flush(bevel={"crew_housing": 0.004, "crew_bezel": 0.0015, T: 0.001})


# ---------------------------------------------------------------- hand controller and surroundings
# Geometry and colours matched to the Smithsonian QTU close-up NASM-NASM2020-00055 (T-grip,
# bellows boot, armrest, harness, housing) and DWG 5.2 / 5.4 for dimensions.

def star_prism(acc, c, axis, teeth, r_out, r_in, length, flat=0.35):
    """Fluted/knurled cylinder: star cross-section (teeth ridges) extruded along axis, centred at c."""
    axis = Vector(axis).normalized()
    e1, e2 = _frame(axis)
    poly = []
    for k in range(teeth):
        for j, (fr, rr) in enumerate(((0.0, r_out), (flat, r_out), (0.5, r_in), (0.5 + flat * 0.6, r_in))):
            t = 2 * math.pi * (k + fr) / teeth
            poly.append(c - axis * (length / 2) + rr * (math.cos(t) * e1 + math.sin(t) * e2))
    prism(acc, poly, axis, length)


def sloft(acc, rings, n=24):
    """Loft of superellipse rings: (centre, ax, ay, half_x, half_y, p); p=1 ellipse, smaller = squarer."""
    v, f = acc
    base = len(v)
    for c, ax, ay, hx, hy, p in rings:
        for k in range(n):
            t = 2 * math.pi * k / n
            ct, st = math.cos(t), math.sin(t)
            v.append(c + ax * (math.copysign(abs(ct) ** p, ct) * hx) + ay * (math.copysign(abs(st) ** p, st) * hy))
    m = len(rings)
    for i in range(m - 1):
        for k in range(n):
            k2 = (k + 1) % n
            f.append((base + i * n + k, base + i * n + k2, base + (i + 1) * n + k2, base + (i + 1) * n + k))
    f.append(tuple(base + k for k in reversed(range(n))))
    f.append(tuple(base + (m - 1) * n + k for k in range(n)))


def ring_pts(c, axis, r, n=16):
    e1, e2 = _frame(Vector(axis).normalized())
    return [c + r * (math.cos(2 * math.pi * k / n) * e1 + math.sin(2 * math.pi * k / n) * e2) for k in range(n + 1)]


class Path:
    """Arc-length parametrised polyline."""

    def __init__(self, pts):
        self.p = [Vector(x) for x in pts]
        self.s = [0.0]
        for a, b in zip(self.p, self.p[1:]):
            self.s.append(self.s[-1] + (b - a).length)
        self.length = self.s[-1]

    def at(self, s):
        s = min(max(s, 0.0), self.length)
        for i in range(len(self.p) - 1):
            if self.s[i + 1] >= s:
                seg = max(self.s[i + 1] - self.s[i], 1e-9)
                t = (s - self.s[i]) / seg
                return self.p[i].lerp(self.p[i + 1], t), (self.p[i + 1] - self.p[i]).normalized()
        return self.p[-1], (self.p[-1] - self.p[-2]).normalized()


def harness(g, pts, r=0.27, offset=Vector((0.25, 0, -0.45))):
    """Two taped cable bundles with tape wraps every ~1.4 in and lacing-cord ties (QTU 00055)."""
    pts = fillet(pts, 1.1 * IN, 6)
    off = offset * IN
    paths = [Path(pts), Path([p + off for p in pts])]
    for P_, rr in zip(paths, (r, r * 0.9)):
        sweep(g.get("crew_harness", True), P_.p, rr * IN, 10)
        s = 0.5 * IN
        k = 0
        while s < P_.length - 0.4 * IN:
            a, _ = P_.at(s - 0.22 * IN)
            b, _ = P_.at(s + 0.22 * IN)
            tube(g.get("crew_tape", True), a, b, rr * 1.13 * IN, 10)
            s += 1.4 * IN
            k += 1
    s = 1.2 * IN
    while s < paths[0].length - 0.5 * IN:   # lacing ties around the pair
        a, t = paths[0].at(s)
        b, _ = paths[1].at(s)
        c = (a + b) / 2
        sweep(g.get("crew_lacing", True), ring_pts(c, t, (r + 0.32) * IN, 14), 0.025 * IN, 6, caps=False)
        s += 2.8 * IN
    return paths


def p_clamp(g, path0, path1, s, anchor):
    """Nylon P-clamp around both bundles, bolted to the housing at `anchor`."""
    a, t = path0.at(s)
    b, _ = path1.at(s)
    c = (a + b) / 2
    sweep(g.get("crew_clamp", True), ring_pts(c, t, 0.62 * IN, 18), 0.07 * IN, 8, caps=False)
    up = (anchor - c)
    tube(g.get("crew_clamp", True), c + up.normalized() * 0.55 * IN, anchor, 0.1 * IN, 8)
    obox(g.get("crew_clamp"), anchor, t, Y, Z, 0.45 * IN, 0.12 * IN, 0.55 * IN)
    lathe(g.get("steel", True), anchor, Y if anchor.y > 0 else -Y, [(0.15 * IN, 0.0), (0.15 * IN, 0.14 * IN)], 6)


def rivet(g, c, nrm):
    lathe(g.get("crew_housing", True), c, nrm, [(0.1 * IN, 0.0), (0.09 * IN, 0.04 * IN), (0.05 * IN, 0.075 * IN), (0.01 * IN, 0.085 * IN)], 8)


def console_back(g, pp):
    """Console underside/back (QTU 00046, DWG 5.2): three bayonet connectors in a row on the sloping
    underside near the front, backshells and a taped bundle running aft into the housing harness;
    screwed access cover on the top; U-frame support under the hand-controller housing (7021092)."""
    fb = Vector(H(CONSOLE_FRONT_STA + 0.3, 0, 118.4))
    pb = pp(5, 12, -0.09)
    along = (pb - fb).normalized()                           # front -> aft along the underside
    down = along.cross(Y)
    if down.z > 0:
        down = -down
    bundle = []
    for i, bl in enumerate((-3.0, -0.7, 1.6)):
        c = fb + along * 2.6 * IN + Y * (bl * IN)
        obox(g.get("aluminium"), c + down * 0.05 * IN, along, Y, -down, 1.3 * IN, 1.3 * IN, 0.1 * IN)
        for du in (-0.5, 0.5):
            for dv in (-0.5, 0.5):
                lathe(g.get("steel", True), c + down * 0.1 * IN + along * du * IN + Y * dv * IN, down,
                      [(0.06 * IN, 0), (0.06 * IN, 0.04 * IN)], 6)
        lathe(g.get("aluminium_spun", True), c, down, [(0.5 * IN, 0), (0.5 * IN, 0.5 * IN)], 18)
        star_prism(g.get("aluminium_spun"), c + down * 0.75 * IN, down, 24, 0.62 * IN, 0.58 * IN, 0.45 * IN, 0.4)
        bend = down.lerp(along, 0.55).normalized()
        b0 = c + down * 0.98 * IN
        lathe(g.get("steel", True), b0, bend, [(0.45 * IN, 0), (0.42 * IN, 0.5 * IN), (0.3 * IN, 0.9 * IN)], 16)
        lathe(g.get("crew_cb_knob", True), b0 + bend * 0.85 * IN, bend, [(0.3 * IN, 0), (0.22 * IN, 0.6 * IN)], 12)  # strain relief
        tail = b0 + bend * 1.4 * IN
        pts = fillet([tail, tail + bend * 0.8 * IN + along * 0.6 * IN,
                      H(57.0, (bl * 0.35) - 0.2, 116.0), H(60.5, 0.0, 115.0)], 1.0 * IN, 5)
        sweep(g.get("crew_harness", True), pts, 0.2 * IN, 10)
        bundle.append(pts)
    p0 = Path(bundle[1])
    s_ = 2.0 * IN
    while s_ < p0.length - 0.5 * IN:
        a, t = p0.at(s_)
        sweep(g.get("crew_lacing", True), ring_pts(a, t, 0.75 * IN, 16), 0.025 * IN, 6, caps=False)
        s_ += 2.2 * IN
    obox(g.get("crew_clamp"), H(59.6, 0, 115.6), X, Y, Z, 0.6 * IN, 1.9 * IN, 0.5 * IN)

    # top access cover: raised screwed plate on the console top, hinge strip on the forward edge
    a = pp(5, 0, -0.09)
    b = Vector(H(CONSOLE_FRONT_STA + 1.2, 0, 125.9))
    tdir = (b - a).normalized()
    tn = tdir.cross(Y)
    if tn.z < 0:
        tn = -tn
    c = a + tdir * ((b - a).length * 0.52) + tn * 0.02 * IN
    obox(g.get("crew_housing"), c, tdir, Y, tn, 5.4 * IN, 7.6 * IN, 0.05 * IN)
    for du in (-2.4, 0.0, 2.4):
        for dv in (-3.5, 3.5):
            lathe(g.get("steel", True), c + tdir * du * IN + Y * dv * IN + tn * 0.025 * IN, tn,
                  [(0.08 * IN, 0), (0.08 * IN, 0.03 * IN), (0.04 * IN, 0.05 * IN)], 8)
    tube(g.get("aluminium", True), c + tdir * 2.75 * IN - Y * 3.6 * IN + tn * 0.06 * IN,
         c + tdir * 2.75 * IN + Y * 3.6 * IN + tn * 0.06 * IN, 0.08 * IN, 10)
    lathe(g.get("steel", True), c + tn * 0.02 * IN, tn, [(0.18 * IN, 0), (0.18 * IN, 0.08 * IN), (0.1 * IN, 0.14 * IN)], 12)

    # support U-frame under the hand-controller housing (7021092)
    w0 = HOUSING[3]
    for k in (1, -1):
        tube(g.get("aluminium", True), H(68.5, k * 4.6, 101.6), H(68.5, k * 4.6, w0 + 0.1), 0.45 * IN, 14)
        obox(g.get("aluminium"), H(68.5, k * 4.6, w0 - 0.2), X, Y, Z, 1.2 * IN, 1.2 * IN, 0.6 * IN)
    sweep(g.get("aluminium", True), fillet([H(68.5, 6.4, 100.4), H(68.5, 6.4, 101.6), H(68.5, -6.4, 101.6),
                                            H(68.5, -6.4, 100.4)], 0.9 * IN, 5), 0.45 * IN, 14)
    for k in (1, -1):
        lathe(g.get("aluminium", True), H(68.5, k * 6.4, 100.3), Z, [(0.9 * IN, 0), (0.9 * IN, 0.15 * IN), (0.5 * IN, 0.4 * IN)], 16)


def hc_surround(g):
    """White hand-controller housing, boot opening, armrest + bracket, harnesses, release ring, post."""
    s0, s1, hh, w0, w1 = HOUSING
    rbox(g.get("crew_housing", True), H((s0 + s1) / 2, 0, (w0 + w1) / 2), X, Y, Z,
         ((s1 - s0) * IN, 2 * hh * IN, (w1 - w0) * IN), 0.3 * IN, 2)
    # "LRV operation instructions" decal on the housing top forward of the grip (DWG 5.2 top view)
    decal(g.parent, "console_hc_instructions", H(62.9, 0, w1 + 0.012), -Y, X, 3.4 * IN, 1.5 * IN, "crew_decal_hc")
    # boot opening: raised fabric bead on a flat flange
    lathe(g.get("crew_housing", True), H(HC_STA, 0, w1 - 0.05), Z, [(2.25 * IN, 0), (2.25 * IN, 0.12 * IN), (2.05 * IN, 0.15 * IN)], 40)
    sweep(g.get("crew_webbing_white", True), ring_pts(H(HC_STA, 0, w1 + 0.2), Z, 2.0 * IN, 40), 0.13 * IN, 8, caps=False)

    # access-panel seams and dome rivets on both sides
    for k in (1, -1):
        y = k * (hh + 0.005)
        ny = Y * k
        for (a0, z0, a1, z1) in ((65.6, w0 + 0.4, 72.6, w0 + 0.4), (65.6, w0 + 3.6, 72.6, w0 + 3.6),
                                 (65.6, w0 + 0.4, 65.6, w0 + 3.6), (72.6, w0 + 0.4, 72.6, w0 + 3.6),
                                 (74.4, w0 + 0.3, 74.4, w1 - 0.4)):
            p0, p1 = H(a0, y, z0), H(a1, y, z1)
            obox(g.get("crew_seam"), (p0 + p1) / 2, X, Y, Z, abs(p1.x - p0.x) + 0.035 * IN, 0.02 * IN,
                 abs(p1.z - p0.z) + 0.035 * IN)
        for i in range(6):
            rivet(g, H(74.75, y, w0 + 0.8 + i * 1.1), ny)
        for i in range(8):
            rivet(g, H(62.2 + i * 1.6, y, w0 + 0.25), ny)
        for z in (w0 + 0.9, w0 + 3.1):
            for a in (66.0, 72.2):
                lathe(g.get("steel", True), H(a, y, z), ny, [(0.11 * IN, 0), (0.11 * IN, 0.05 * IN), (0.06 * IN, 0.08 * IN)], 8)
        # tee-handle latch studs: steel tab, stud, white nylon cap (00055)
        for a, z in ((73.6, 113.9), (74.9, 109.3)):
            obox(g.get("steel"), H(a, k * (hh + 0.03), z), X, Y, Z, 0.9 * IN, 0.05 * IN, 0.6 * IN)
            tube(g.get("steel", True), H(a, k * hh, z), H(a, k * (hh + 0.45), z), 0.13 * IN, 10)
            lathe(g.get("crew_knob_white", True), H(a, k * (hh + 0.4), z), ny, [(0.17 * IN, 0), (0.17 * IN, 0.2 * IN), (0.08 * IN, 0.3 * IN)], 12)

    # armrest bracket (stainless side plates, part-number placard) and leather saddle pad
    for k in (1, -1):
        y = k * (hh + 0.06)
        poly = [H(71.4, y, 115.7), H(75.6, y, 115.7), H(77.6, y, 116.3), H(77.6, y, 117.0), H(71.4, y, 117.0)]
        prism(g.get("steel"), poly, Y * k, 0.07 * IN)
        for a, z, hexa in ((72.0, 116.35, False), (74.6, 116.4, True)):
            c = H(a, y + k * 0.07, z)
            if hexa:
                lathe(g.get("steel", True), c, Y * k, [(0.22 * IN, 0), (0.22 * IN, 0.16 * IN)], 6)
            else:
                lathe(g.get("aluminium_spun", True), c, Y * k, [(0.2 * IN, 0), (0.18 * IN, 0.06 * IN), (0.1 * IN, 0.1 * IN)], 14)
        if k < 0:   # stamped part number on the bracket (QTU 00055 "209-31725-2")
            decal(g.parent, "console_armrest_pn", H(73.3, y + k * 0.075, 116.15), X, Z, 1.6 * IN, 0.35 * IN,
                  "crew_decal_armrest")
    for a in (72.3, 76.6):   # cross straps under the pad between the side plates
        obox(g.get("steel"), H(a, 0, 116.95), X, Y, Z, 0.6 * IN, 2 * (hh + 0.1) * IN, 0.08 * IN)
    rings = []
    for i in range(15):
        sfr = i / 14
        x = 72.6 + 6.6 * sfr
        wdt = (3.8 + 0.9 * sfr) * (0.75 if i in (0, 14) else (0.93 if i in (1, 13) else 1.0))
        hgt = (0.85 + 1.0 * sfr ** 4) * (0.55 if i in (0, 14) else (0.88 if i in (1, 13) else 1.0))
        zc = 117.5 + 0.12 * math.sin(math.pi * sfr) - 0.6 * sfr ** 4
        rings.append((H(x, 0, zc), Y, Z, wdt / 2 * IN, hgt / 2 * IN, 0.45))
    rings[0] = (rings[0][0] + Vector((0.12 * IN, 0, 0)),) + rings[0][1:]
    rings[-1] = (rings[-1][0] - Vector((0.12 * IN, 0, 0)),) + rings[-1][1:]
    sloft(g.get("crew_armrest", True), rings, 28)

    # post to the floor
    tube(g.get("aluminium", True), H(70.5, 0, 100.3), H(70.5, 0, w0 + 0.2), 1.1 * IN, 20)
    lathe(g.get("aluminium", True), H(70.5, 0, 100.3), Z, [(2.4 * IN, 0), (2.4 * IN, 0.25 * IN), (1.3 * IN, 0.9 * IN)], 24)

    # contingency park-brake release ring on the aft face, with its lanyard wire
    ring_c = H(s1 + 0.25, 1.4, w0 + 1.6)
    ring = [ring_c + (Y * math.cos(t) + Z * math.sin(t)) * 0.6 * IN for t in
            [2 * math.pi * k / 24 for k in range(25)]]
    sweep(g.get("aluminium_spun", True), ring, 0.07 * IN, 8, caps=False)
    sweep(g.get("steel", True), [H(s1, 1.4, w0 + 4.0), H(s1 + 0.15, 1.4, w0 + 3.0), H(s1 + 0.25, 1.4, w0 + 2.2)], 0.025 * IN, 6)

    # taped harnesses along both sides, P-clamps, dropping down behind the armrest bracket to the floor
    for k in (1, -1):
        y = k * (hh + 0.33)
        pts = [H(59.0, k * 3.0, 117.2), H(61.6, y, 114.6), H(73.0, y, 114.2), H(75.6, y, 113.6),
               H(76.6, k * 2.0, 111.5), H(76.6, k * 2.0, w0 - 0.2), H(71.8, k * 1.6, w0 - 1.2),
               H(71.8, k * 1.6, 101.0), H(73.5, k * 1.8, 100.5)]
        p0, p1 = harness(g, pts)
        for s_in, a in ((6.0, 66.0), (11.5, 71.6)):
            pa, _ = p0.at(s_in * IN)
            p_clamp(g, p0, p1, s_in * IN, Vector((pa.x, k * hh * IN, pa.z + 0.9 * IN)))

    # inboard handholds: dark steel tubes rising beside the console. The left one carries the LGA
    # staff socket (Ops HB 2.14 / Fig 4-4: "push button on end of handhold" releases the staff);
    # the right one has the grey-green grip sleeve (00055).
    for k in (1, -1):
        pts = fillet([H(68.5, k * hh, 114.6), H(66.5, k * 5.8, 114.8), H(62.6, k * 7.1, 117.0),
                      H(61.4, k * 7.1, 120.0), H(61.0, k * 7.1, 124.6)], 1.5 * IN, 6)
        sweep(g.get("crew_handhold", True), pts, 0.45 * IN, 14)
        top = H(61.0, k * 7.1, 124.6)
        sweep(g.get("crew_handhold", True), ring_pts(H(61.35, k * 7.1, 120.4), Z, 0.47 * IN, 16), 0.06 * IN, 6, caps=False)
        d = Vector((-1.0, -0.8 * k, 0)).normalized()     # push button: toward the crew and inboard
        if k < 0:
            # right handhold: long grey-green grip to the top, short stub with a brushed cap at its base
            lathe(g.get("crew_grip", True), H(61.38, k * 7.1, 119.9), Vector((-0.06, 0, 1)),
                  [(0.6 * IN, 0), (0.64 * IN, 0.3 * IN)] + [(0.66 * IN if j % 2 else 0.63 * IN, (0.6 + 0.5 * j) * IN) for j in range(8)]
                  + [(0.64 * IN, 5.0 * IN), (0.5 * IN, 5.3 * IN), (0.2 * IN, 5.42 * IN)], 18)
            c0 = H(61.4, k * 7.1, 119.5)
            tube(g.get("crew_handhold", True), c0, c0 + d * 0.9 * IN, 0.38 * IN, 14)
            lathe(g.get("aluminium_spun", True), c0 + d * 0.85 * IN, d, [(0.5 * IN, 0), (0.5 * IN, 0.45 * IN), (0.3 * IN, 0.55 * IN)], 20)
            continue
        # left handhold: LGA staff socket with a flat flange and the staff-release push button (00055)
        lathe(g.get("crew_handhold", True), top, Z, [(0.68 * IN, 0), (0.68 * IN, 1.35 * IN)], 20)
        rbox(g.get("aluminium_spun", True), top + Z * 1.5 * IN + X * 0.15 * IN, X, Y, Z, (2.0 * IN, 1.45 * IN, 0.32 * IN), 0.15 * IN, 2)
        lathe(g.get("crew_bezel", True), top + Z * 1.62 * IN, Z, [(0.45 * IN, 0), (0.45 * IN, 0.06 * IN)], 16)
        c0 = top + Z * 0.7 * IN
        tube(g.get("crew_handhold", True), c0, c0 + d * 1.0 * IN, 0.42 * IN, 16)
        lathe(g.get("aluminium_spun", True), c0 + d * 0.95 * IN, d, [(0.6 * IN, 0), (0.62 * IN, 0.55 * IN),
                                                                     (0.55 * IN, 0.75 * IN), (0.3 * IN, 0.85 * IN)], 24)
        sweep(g.get("steel", True), ring_pts(top - Z * 0.05 * IN, Z, 0.47 * IN, 16), 0.07 * IN, 6, caps=False)   # weld bead


def hand_controller(ch):
    w1 = HOUSING[4]
    steer = link("hc_steer", H(HC_STA, 0, HC_ROLL_WL), ch, {
        "type": "revolute", "axis": (1.0, 0.0, 0.0), "limit": (-HC_STEER_LIMIT, HC_STEER_LIMIT),
        "role": "hand_controller_steer", "drive": dict(HC_STEER_DRIVE)}, mass=MASS["hc_steer"])
    g = Parts("hc_steer", steer)
    obox(g.get("aluminium"), H(HC_STA, 0, HC_ROLL_WL + 0.6), X, Y, Z, 2.4 * IN, 2.0 * IN, 2.4 * IN)
    tube(g.get("steel", True), H(HC_STA + 2.5, 0, HC_ROLL_WL), H(HC_STA - 2.5, 0, HC_ROLL_WL), 0.25 * IN)
    # crinkled aluminised-fabric bellows boot: revolved profile with noisy radial folds
    v, f = g.get("crew_boot", True)
    nth, nz = 72, 16
    base = len(v)
    # (t, radius, height above the housing top): sags into the opening, then rises to the stem collar
    prof = [(0.0, 2.0, 0.12), (0.18, 1.82, -0.12), (0.38, 1.58, -0.22), (0.58, 1.38, 0.02),
            (0.8, 1.27, 0.55), (1.0, 1.26, 1.15)]
    for j in range(nz + 1):
        t = j / nz
        for i in range(len(prof) - 1):
            if prof[i + 1][0] >= t:
                u = (t - prof[i][0]) / (prof[i + 1][0] - prof[i][0])
                r0 = prof[i][1] + u * (prof[i + 1][1] - prof[i][1])
                z0 = prof[i][2] + u * (prof[i + 1][2] - prof[i][2])
                break
        amp = math.sin(math.pi * t) * (0.0 if j in (0, nz) else 1.0)
        for i in range(nth):
            th = 2 * math.pi * i / nth
            fold = (0.06 * math.sin(5 * th + 4 * t) + 0.045 * math.sin(11 * th - 7 * t + 1.3)
                    + 0.02 * math.sin(19 * th + 9 * t + 0.4) + 0.008 * math.sin(37 * th - 15 * t))
            r = r0 * (1 + amp * fold)
            z = w1 + z0 + amp * (0.12 * math.sin(7 * th + 3.0) + 0.05 * math.sin(15 * th - 1.0))
            v.append(H(HC_STA, 0, z) + Vector((math.cos(th), math.sin(th), 0)) * r * IN)
    for j in range(nz):
        for i in range(nth):
            i2 = (i + 1) % nth
            f.append((base + j * nth + i, base + j * nth + i2, base + (j + 1) * nth + i2, base + (j + 1) * nth + i))
    f.append(tuple(base + i for i in reversed(range(nth))))
    f.append(tuple(base + nz * nth + i for i in range(nth)))
    # stitched fabric collar gripping the stem base
    lathe(g.get("crew_webbing_white", True), H(HC_STA, 0, w1 + 1.15), Z,
          [(1.3 * IN, 0), (1.32 * IN, 0.1 * IN), (1.32 * IN, 0.38 * IN), (1.26 * IN, 0.45 * IN)], 32)
    g.flush()

    brake = link("hc_brake", H(HC_STA, 0, HC_BRAKE_WL), steer, {
        "type": "revolute", "axis": (0.0, -1.0, 0.0), "limit": (0.0, HC_BRAKE_LIMIT),
        "role": "hand_controller_brake", "drive": dict(HC_BRAKE_DRIVE)}, mass=MASS["hc_brake"])
    g = Parts("hc_brake", brake)
    # bulbous moulded stem base: round collar, flare, neck up to the palm pivot
    secs = [(w1 + 1.25, 2.25, 2.25, 1.0), (w1 + 1.4, 2.55, 2.55, 1.0), (w1 + 1.62, 2.62, 2.62, 1.0),
            (w1 + 1.85, 2.5, 2.46, 0.95), (118.45, 2.25, 2.14, 0.85), (119.05, 1.88, 1.74, 0.7),
            (HC_PALM_WL, 1.7, 1.56, 0.55)]
    sloft(g.get("crew_grip", True), [(H(HC_STA, 0, z), X, Y, dx / 2 * IN, dy / 2 * IN, p) for z, dx, dy, p in secs], 28)
    for b in (-0.55, 0.0, 0.55):   # moulded oval drain holes in the flare
        c = H(HC_STA + 1.12 - abs(b) * 0.12, b, 118.2 + abs(b) * 0.1)
        rbox(g.get("crew_grip_dark", True), c, X, Y, Z, (0.14 * IN, 0.22 * IN, 0.48 * IN), 0.06 * IN, 2)
    tube(g.get("steel", True), H(HC_STA, 1.0, HC_BRAKE_WL), H(HC_STA, -1.0, HC_BRAKE_WL), 0.3 * IN)
    obox(g.get("aluminium"), H(HC_STA, 0, (HC_BRAKE_WL + w1) / 2 + 0.3), X, Y, Z, 1.2 * IN, 0.8 * IN, (w1 - HC_BRAKE_WL + 0.6) * IN)
    g.flush()

    thr = link("hc_throttle", H(HC_STA, 0, HC_PALM_WL), brake, {
        "type": "revolute", "axis": (0.0, 1.0, 0.0), "limit": (-HC_THROTTLE_LIMIT, HC_THROTTLE_LIMIT),
        "role": "hand_controller_throttle", "drive": dict(HC_THROTTLE_DRIVE)}, mass=MASS["hc_throttle"])
    g = Parts("hc_throttle", thr)
    tw = HC_PALM_WL + 2.15          # T-bar centreline, 4.4 in above the stem collar
    secs = [(HC_PALM_WL, 1.7, 1.56, 0.55), (HC_PALM_WL + 0.5, 1.88, 1.62, 0.4), (HC_PALM_WL + 0.9, 2.05, 1.68, 0.28),
            (tw - 0.62, 2.05, 1.68, 0.28), (tw - 0.42, 1.8, 1.6, 0.35), (tw - 0.2, 1.3, 1.4, 0.5)]
    sloft(g.get("crew_grip", True), [(H(HC_STA, 0, z), X, Y, dx / 2 * IN, dy / 2 * IN, p) for z, dx, dy, p in secs], 28)
    # T bar: 1.3 in fluted crossbar between rounded end caps (00055)
    star_prism(g.get("crew_grip", False), H(HC_STA, 0, tw), Y, 30, 0.66 * IN, 0.61 * IN, 4.1 * IN, 0.25)
    for k in (1, -1):
        rbox(g.get("crew_grip", True), H(HC_STA - 0.05, k * 2.72, tw - 0.12), X, Y, Z,
             (1.5 * IN, 1.5 * IN, 1.8 * IN), 0.62 * IN, 4)
    lathe(g.get("crew_grip_dark", True), H(HC_STA + 0.1, -3.43, tw - 0.25), Y, [(0.24 * IN, 0), (0.24 * IN, 0.05 * IN)], 14)
    # black "R" index line beside the reverse-inhibit thumbwheel
    obox(g.get("crew_cb_knob"), H(HC_STA + 1.03, -0.25, tw - 0.95), X, Y, Z, 0.02 * IN, 0.9 * IN, 0.06 * IN)
    g.flush()

    # reverse inhibit switch: knurled aluminium thumbwheel at the aft face, lateral axis (00055)
    sw_c = H(HC_STA + 0.85, 0.3, tw - 0.95)
    sw = link("hc_reverse_switch", sw_c, thr, {
        "type": "revolute", "axis": (0.0, 1.0, 0.0), "limit": (0.0, math.radians(25)),
        "role": "reverse_inhibit", "drive": dict(SWITCH_DRIVE)}, mass=MASS["hc_reverse_switch"])
    g = Parts("hc_reverse_switch", sw)
    star_prism(g.get("aluminium_spun"), sw_c + Y * 0.12 * IN, Y, 16, 0.52 * IN, 0.46 * IN, 0.42 * IN, 0.45)
    lathe(g.get("aluminium_spun", True), sw_c - Y * 0.2 * IN, Y, [(0.3 * IN, 0), (0.3 * IN, 0.1 * IN)], 16)
    g.flush()


# ---------------------------------------------------------------- entry point

def build(ch):
    for s in (1, -1):
        seat(ch, s)
    footrests(ch)
    console(ch)
    hand_controller(ch)
    print(f"crew: hand controller at STA {HC_STA:.1f}, palm pivot WL {HC_PALM_WL:.1f}")


if __name__ == "__main__":
    print(json.dumps({"hc_sta": HC_STA, "palm_wl": HC_PALM_WL}), file=sys.stderr)
