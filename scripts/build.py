"""Build the LRV model in Blender.

Run: blender -b --factory-startup -P scripts/build.py -- [--out build/lrv.blend]

Every moving part is a link Empty named "link:<name>" placed at its joint origin
(unrotated, in the rest pose). Visual meshes are parented to their link Empty, so
the exporters can write one mesh per link in link-local coordinates.
"""
import math
import os
import sys

import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P  # noqa: E402
import materials as M  # noqa: E402
from geo import (add_mesh, box_beam_geo, box_geo, ensure_uvs, link, loop_joint, pipe, reset, revolve_geo,
                 rivet_geo, tag_variant, tube_geo, tubes)  # noqa: E402
from wheel import build_wheel  # noqa: E402


# ---------------------------------------------------------------- upright / knuckle / drive

def build_upright(corner, parent):
    """Kingpin carrier between the outer ends of the two A-arms (moves with suspension, doesn't steer)."""
    sx, sy = P.CORNERS[corner]
    wc = Vector(P.wheel_center(corner))
    kp = Vector((wc.x, sy * P.KINGPIN_Y, wc.z))
    lo = kp + Vector((0, 0, P.KNUCKLE_LOWER_DZ))
    hi = kp + Vector((0, 0, P.KNUCKLE_UPPER_DZ))
    v, f = tube_geo(lo - Vector((0, 0, 0.012)), hi + Vector((0, 0, 0.012)), 0.024, 16)
    for p in (lo, hi):  # clevis bosses for the arm pins (fore-aft axis)
        tube_geo(p - Vector((0.03, 0, 0)), p + Vector((0.03, 0, 0)), 0.016, 12, True, v, f)
    add_mesh(f"{corner}_upright", v, f, M.get("aluminium"), parent, smooth=True)


def _drive_frame(corner):
    """Wheel-axis frame for the traction drive: local (x, a, z) -> world, a = axial (+ outboard)."""
    sx, sy = P.CORNERS[corner]
    wc = Vector(P.wheel_center(corner))
    xform = Matrix.Translation(wc) @ Matrix.Diagonal((1, sy, 1, 1))
    return xform, sy < 0, (lambda x, a, z: xform @ Vector((x, a, z)))


def _merge(dst, src):
    v, f = dst
    o = len(v)
    v.extend(src[0])
    f.extend(tuple(i + o for i in q) for q in src[1])


def build_knuckle(corner, parent):
    """Steering knuckle link: traction drive housing, brake, steering arm. Steers, doesn't spin.

    A17 Systems Handbook Drawing 4.4 (traction drive: harmonic drive inside the hub, band brake at the
    wheel-hub interface, 3.25 in DC motor running inboard past the suspension-arm studs, ribbed end cap,
    odometer pickup) and Drawing 4.5 (brake arm under a rubber dust boot, cable clevis, return spring).
    The rotating hub flange is on the wheel link (wheel.py). Ref photo 7020254.jpg (inboard side).
    """
    sx, sy = P.CORNERS[corner]
    wc = Vector(P.wheel_center(corner))
    kp = Vector((wc.x, sy * P.KINGPIN_Y, wc.z))
    xform, flip, W = _drive_frame(corner)
    cx = -sx                                          # local x toward the vehicle centre
    rm, end = P.DRIVE_MOTOR_R, P.DRIVE_INBOARD_END
    akp = -(P.TRACK / 2 - P.KINGPIN_Y)                # axial position of the kingpin / arm studs

    # brake housing + dust shield at the wheel-hub interface (bare metal)
    rb = P.DRIVE_BRAKE_R
    brake = [(0.034, -0.012), (rb - 0.002, -0.012), (rb, -0.014), (rb, -0.030), (rb - 0.006, -0.0345),
             (0.034, -0.0345)]
    bv, bf, _ = revolve_geo(brake, 48, xform, flip)
    hw_v, hw_f = list(bv), list(bf)
    for i in range(7):                                # 7 bolts on the inboard face (Dwg 4.5 rear view)
        t = math.tau * (i + 0.5) / 7
        d = Vector((math.sin(t), 0, math.cos(t)))
        p = d * (rb - 0.012)
        tube_geo(W(p.x, -0.034, p.z), W(p.x, -0.0385, p.z), 0.0042, 6, True, hw_v, hw_f)
    for t in (math.radians(-22), math.radians(22)):  # two hex plugs flanking the brake arm
        tube_geo(W(rb * 0.82 * math.sin(t), -0.034, rb * 0.82 * math.cos(t)),
                 W(rb * 0.82 * math.sin(t), -0.040, rb * 0.82 * math.cos(t)), 0.0065, 6, True, hw_v, hw_f)
    add_mesh(f"{corner}_brake_housing", hw_v, hw_f, M.get("drive_housing"), parent, smooth=False)

    # motor housing: neck behind the brake, 3.25 in motor case, end flange and ribbed end cap (white)
    motor = [(0.036, -0.0345), (0.031, -0.040), (0.030, -0.058), (rm, -0.066),
             (rm, -(end - 0.017)), (rm + 0.005, -(end - 0.015)), (rm + 0.005, -(end - 0.009)), (0.036, -(end - 0.007)),
             (0.031, -end), (0.0, -end)]
    mv, mf, _ = revolve_geo(motor, 48, xform, flip)
    mot = (list(mv), list(mf))
    for k in range(6):                                # radial ribs on the end cap (Dwg 4.4 rear view)
        t = math.tau * k / 6
        d, tg = Vector((math.sin(t), 0, math.cos(t))), Vector((math.cos(t), 0, -math.sin(t)))
        p0, p1 = d * 0.010, d * 0.033
        v, f = box_beam_geo(W(*(p0 + Vector((0, -end - 0.002, 0)))), W(*(p1 + Vector((0, -end - 0.002, 0)))),
                            0.004, 0.005)
        _merge(mot, (v, f))
    v, f, _ = revolve_geo([(0.018, -end), (0.018, -end - 0.004), (0.022, -end - 0.004), (0.022, -end)], 32, xform, flip)
    _merge(mot, (v, f))
    # four-lug mounting flange a little ahead of the end cap
    lug = []
    for k in range(16):
        t = math.tau * k / 16
        rr = 0.05 if k % 4 == 2 else 0.044
        lug.append(W(rr * math.sin(t), -(end - 0.035), rr * math.cos(t)))
    v, f = [], []
    plate_geo(lug, xform.to_3x3() @ Vector((0, -1, 0)), 0.005, v, f)
    _merge(mot, (v, f))
    # suspension-arm stud boss through the housing at the kingpin
    v, f = tube_geo(kp + Vector((0, 0, -rm - 0.012)), kp + Vector((0, 0, rm + 0.012)), 0.021, 16)
    _merge(mot, (v, f))
    # odometer pickup on the upper aft quarter
    t = math.radians(-50 * sx)
    d = Vector((math.sin(t), 0, math.cos(t)))
    v, f = tube_geo(W(*(d * rm + Vector((0, -(end - 0.05), 0)))), W(*(d * (rm + 0.022) + Vector((0, -(end - 0.05), 0)))),
                    0.009, 12)
    _merge(mot, (v, f))
    add_mesh(f"{corner}_drive_motor", *mot, M.get("white_paint"), parent, smooth=True)

    # studs, screws, brake arm hardware (steel)
    sv, sf = [], []
    for zs in (-1, 1):
        tube_geo(kp + Vector((0, 0, zs * (rm + 0.012))), kp + Vector((0, 0, zs * (rm + 0.034))), 0.008, 10, True, sv, sf)
    for k in (2, 6, 10, 14):
        t = math.tau * k / 16
        tube_geo(W(0.046 * math.sin(t), -(end - 0.035) - 0.005, 0.046 * math.cos(t)),
                 W(0.046 * math.sin(t), -(end - 0.035) - 0.008, 0.046 * math.cos(t)), 0.0035, 8, True, sv, sf)
    # brake arm: from under the dust boot at 12 o'clock, inboard along the housing, cranked down to the
    # cable clevis; cable with return spring to a bracket on the motor (Dwg 4.5)
    zb = rb + 0.004
    pts = [W(0, -0.022, zb), W(0, -0.072, zb + 0.003), W(cx * 0.03, -0.098, rm + 0.022)]
    for p0, p1 in zip(pts, pts[1:]):
        box_beam_geo(p0, p1, 0.007, 0.004, sv, sf)
    clevis = pts[-1]
    tube_geo(clevis + (W(0, 0, 1) - W(0, 0, 0)) * -0.004, clevis + (W(0, 0, 1) - W(0, 0, 0)) * 0.004, 0.005, 10,
             True, sv, sf)
    brk = W(cx * 0.03, -0.150, rm + 0.022)
    box_beam_geo(brk + (W(0, 0, 0) - W(0, 0, 1)) * 0.024, brk + (W(0, 0, 1) - W(0, 0, 0)) * 0.006, 0.012, 0.012, sv, sf)
    coil = []
    for i in range(97):                                # return spring, 8 turns
        u = i / 96
        t = math.tau * 8 * u
        coil.append(W(cx * 0.03 + 0.006 * math.cos(t), -0.104 - 0.040 * u, rm + 0.022 + 0.006 * math.sin(t)))
    add_mesh(f"{corner}_brake_hardware", sv, sf, M.get("steel"), parent, smooth=False)
    pipe(f"{corner}_brake_spring", coil, 0.0012, M.get("steel"), parent, sides=6)
    cable = [clevis, W(cx * 0.03, -0.150, rm + 0.022), W(cx * 0.035, -0.20, rm + 0.035),
             W(cx * 0.05, -0.245, rm + 0.07), W(cx * 0.07, -0.27, rm + 0.11)]
    pipe(f"{corner}_brake_cable", cable, 0.0028, M.get("damper"), parent, sides=8)
    v, f = box_geo(W(0, -0.026, rb + 0.006), (0.024, 0.02, 0.012))
    add_mesh(f"{corner}_brake_boot", v, f, M.define("rubber_boot", (0.035, 0.033, 0.03), 0.0, 0.75), parent)

    # steering arm (DWG 4.6 plan view): hangs off the lower arm stud and trails toward the vehicle centre;
    # the tie-rod ball stud at its end is the anchor of the tie-rod loop joint
    arm_end = Vector(P.steer_geometry(corner)[2])
    lo = Vector((0, 0, -0.017))                       # arm runs just under the ball, stud points up into it
    root = Vector((kp.x, kp.y, arm_end.z)) + lo
    v, f = box_beam_geo(root, arm_end + lo, 0.016, 0.012)
    tube_geo(root + Vector((0, 0, -0.008)), kp + Vector((0, 0, -rm - 0.03)), 0.012, 10, True, v, f)
    tube_geo(arm_end + lo + Vector((0, 0, -0.007)), arm_end + lo + Vector((0, 0, 0.006)), 0.012, 12, True, v, f)
    add_mesh(f"{corner}_knuckle", v, f, M.get("drive_housing"), parent, smooth=True)
    v, f = tube_geo(arm_end + lo, arm_end + Vector((0, 0, 0.014)), 0.006, 8)  # ball stud + nut
    add_mesh(f"{corner}_steer_arm_stud", v, f, M.get("steel"), parent, smooth=True)
    return arm_end


# ---------------------------------------------------------------- suspension

def a_arm(name, pivot_y, pivot_z, apex, x, sy, parent, r=0.012):
    b0 = Vector((x - P.ARM_BASE / 2, sy * pivot_y, pivot_z))
    b1 = Vector((x + P.ARM_BASE / 2, sy * pivot_y, pivot_z))
    v, f = tube_geo(b0, apex, r, 10)
    tube_geo(b1, apex, r, 10, True, v, f)
    tube_geo(b0, b1, 0.02, 12, True, v, f)  # pivot sleeve on the torsion bar
    return add_mesh(name, v, f, M.get("aluminium"), parent, smooth=True)


def build_suspension(corner, chassis):
    """Double wishbone per corner, as a physical four-bar:

    chassis -> lower_arm (torsion bar spring) -> upright -> steer (kingpin, passive) -> wheel
    chassis -> upper_arm, closed to the upright by a loop joint.
    chassis -> damper_cyl (trunnion) -> damper_rod (sliding, viscous), closed to the upper arm.
    """
    sx, sy = P.CORNERS[corner]
    wc = Vector(P.wheel_center(corner))
    kx = wc.x
    X = (1.0, 0.0, 0.0)
    lo_pivot = Vector((kx, sy * P.LOWER_PIVOT_Y, wc.z + P.LOWER_PIVOT_DZ))
    up_pivot = Vector((kx, sy * P.UPPER_PIVOT_Y, wc.z + P.UPPER_PIVOT_DZ))
    kp_lo = Vector((kx, sy * P.KINGPIN_Y, wc.z + P.KNUCKLE_LOWER_DZ))
    kp_hi = Vector((kx, sy * P.KINGPIN_Y, wc.z + P.KNUCKLE_UPPER_DZ))

    # +rotation about X lifts the outer end on the left side, lowers it on the right
    jl, rb = P.arm_angle(P.JOUNCE), P.arm_angle(P.REBOUND)
    limit = (-rb, jl) if sy > 0 else (-jl, rb)
    lower = link(f"{corner}_lower_arm", lo_pivot, chassis, {
        "type": "revolute", "axis": X, "limit": limit, "role": "suspension",
        "drive": {"kind": "spring", "stiffness": P.TORSION_K_LOWER, "damping": 0.0,
                  "target": -sy * P.SPRING_PRELOAD_ANGLE}})
    a_arm(f"{corner}_lower_arm", P.LOWER_PIVOT_Y, lo_pivot.z, kp_lo, kx, sy, lower, 0.013)
    upper = link(f"{corner}_upper_arm", up_pivot, chassis, {
        "type": "revolute", "axis": X, "limit": limit, "role": "suspension_upper",
        "drive": {"kind": "spring", "stiffness": P.TORSION_K_UPPER, "damping": 0.0,
                  "target": -sy * P.SPRING_PRELOAD_ANGLE}})
    a_arm(f"{corner}_upper_arm", P.UPPER_PIVOT_Y, up_pivot.z, kp_hi, kx, sy, upper, 0.010)

    upright = link(f"{corner}_upright", kp_lo, lower, {"type": "revolute", "axis": X, "role": "passive"})
    build_upright(corner, upright)
    loop_joint(f"{corner}_upper_ball", f"{corner}_upper_arm", f"{corner}_upright", kp_hi, X)

    # linear damper: cylinder trunnion on the chassis, rod clevis on the upper arm
    d_arm = up_pivot.lerp(kp_hi, 0.62) + Vector((sx * 0.03, 0, 0))
    d_chs = Vector((kx + sx * 0.03, sy * (P.LOWER_PIVOT_Y - 0.12), wc.z - 0.01))
    axis = (d_arm - d_chs).normalized()
    mid = d_chs.lerp(d_arm, 0.55)
    cyl = link(f"{corner}_damper_cyl", d_chs, chassis, {"type": "revolute", "axis": X, "role": "passive"})
    tubes(f"{corner}_damper_cyl", [(d_chs, mid)], 0.024, M.get("damper"), cyl)
    rod = link(f"{corner}_damper_rod", mid, cyl, {
        "type": "prismatic", "axis": tuple(axis), "limit": P.DAMPER_STROKE, "role": "damper",
        "drive": {"kind": "damper", "stiffness": 0.0, "damping": P.DAMPER_C, "target": 0.0}})
    tubes(f"{corner}_damper_rod", [(mid - axis * 0.06, d_arm)], 0.010, M.get("chrome"), rod)
    loop_joint(f"{corner}_damper_clevis", f"{corner}_upper_arm", f"{corner}_damper_rod", d_arm, X)

    # kingpin: passive, steered by the sector through the tie rod (build_steering_linkage)
    steer = link(f"{corner}_steer", Vector((kx, sy * P.KINGPIN_Y, wc.z)), upright, {
        "type": "revolute", "axis": (0.0, 0.0, 1.0), "limit": (-P.STEER_KINGPIN_STOP, P.STEER_KINGPIN_STOP),
        "role": "passive"})
    arm_end = build_knuckle(corner, steer)
    STEER[corner] = steer
    wheel = link(f"{corner}_wheel", wc, steer, {
        "type": "revolute", "axis": (0.0, 1.0, 0.0), "role": "wheel",
        "drive": {"kind": "velocity", "stiffness": 0.0, "damping": P.WHEEL_DRIVE_D,
                  "max_force": P.WHEEL_MAX_TORQUE, "target": 0.0}})
    build_wheel(corner, wheel)
    return arm_end


# ---------------------------------------------------------------- chassis

SECTIONS = {}  # "fwd" / "center" / "aft" chassis links
STEER = {}     # corner -> steering knuckle link


def beaded_panel_geo(x0, x1, y0, y1, z, verts, faces, pitch=2.30 * P.IN, flat=1.50 * P.IN, depth=0.32 * P.IN):
    """Floor panel with lateral stiffening beads (Drawing 3.1 'typical bead section'), beads hang down."""
    xs = []
    n = int(abs(x1 - x0) / pitch)
    step = (x1 - x0) / max(n, 1)
    for k in range(n):
        a = x0 + k * step
        bw = step - flat * (step / pitch)
        for t in (0.0, flat * step / pitch / 2):
            xs.append((a + t, 0.0))
        for i in range(1, 6):
            u = i / 6
            xs.append((a + flat * step / pitch / 2 + u * bw, -depth * math.sin(math.pi * u)))
        xs.append((a + step - flat * step / pitch / 2, 0.0))
    xs.append((x1, 0.0))
    base = len(verts)
    for x, dz in xs:
        verts.append((x, y0, z + dz))
        verts.append((x, y1, z + dz))
    for i in range(len(xs) - 1):
        o = base + 2 * i
        faces.append((o, o + 2, o + 3, o + 1))


def _web_with_holes(o, ax, up, L, h, holes, out_n, verts, faces, n_seg=24, lip=0.004):
    """Flat rectangular web o + ax*s + up*z (s in [0, L], z in [-h/2, h/2]) with real round holes.

    holes: [(s, r)] along the web. Each hole sits in a square cell of side h whose boundary vertices
    are shared with the n-gons filling the gaps, so the surface is closed (no T-junctions). A short
    flanged lip is turned inward (-out_n) round each hole, as on pressed lightening holes.
    """
    a = h / 2
    cells = sorted((s, min(r, a - 0.006)) for s, r in holes if a <= s <= L - a)
    pt = lambda s, z: o + ax * s + up * z  # noqa: E731
    flip = ax.cross(up).dot(out_n) < 0

    def add_poly(idx):
        faces.append(tuple(reversed(idx)) if flip else tuple(idx))

    prev_right = [len(verts), len(verts) + 1]          # beam start: bottom, top corners
    verts.extend([pt(0.0, -a), pt(0.0, a)])
    q = n_seg // 4
    for c, r in cells:
        base = len(verts)
        ring = []
        for i in range(n_seg):                         # CCW in (s, z) from the bottom-right corner
            t = -math.pi / 4 + 2 * math.pi * i / n_seg
            k = a / max(abs(math.cos(t)), abs(math.sin(t)))
            verts.append(pt(c + k * math.cos(t), k * math.sin(t)))
            ring.append((c + r * math.cos(t), r * math.sin(t)))
        for s, z in ring:
            verts.append(pt(s, z))
        for s, z in ring:
            verts.append(pt(s, z) - out_n * lip)
        for i in range(n_seg):
            j = (i + 1) % n_seg
            add_poly((base + i, base + j, base + n_seg + j, base + n_seg + i))
            add_poly((base + n_seg + i, base + n_seg + j, base + 2 * n_seg + j, base + 2 * n_seg + i))
        left = [base + (2 * q + k) % n_seg for k in range(q + 1)]       # top-left -> bottom-left
        right = [base + k for k in range(q + 1)]                         # bottom-right -> top-right
        add_poly(list(reversed(prev_right + left)))   # gap: down this cell's left edge, up the previous right
        prev_right = right
    end = len(verts)
    verts.extend([pt(L, a), pt(L, -a)])
    add_poly(list(reversed(prev_right + [end, end + 1])))


def holed_beam_geo(p0, p1, w, h, holes, verts, faces):
    """box_beam_geo with matching lightening holes through both vertical webs (see-through)."""
    p0, p1 = Vector(p0), Vector(p1)
    L = (p1 - p0).length
    d = (p1 - p0).normalized()
    side = d.cross(Vector((0, 0, 1))).normalized()
    up = side.cross(d).normalized()
    for sg in (-1, 1):
        _web_with_holes(p0 + side * (sg * w / 2), d, up, L, h, holes, side * sg, verts, faces)
    for sg in (-1, 1):                                 # flanges (top / bottom) and end caps
        c0, c1 = p0 + up * (sg * h / 2), p1 + up * (sg * h / 2)
        base = len(verts)
        verts.extend([c0 - side * w / 2, c0 + side * w / 2, c1 + side * w / 2, c1 - side * w / 2])
        faces.append((base, base + 1, base + 2, base + 3) if sg > 0 else (base + 3, base + 2, base + 1, base))
    for p, sg in ((p0, -1), (p1, 1)):
        base = len(verts)
        verts.extend([p + side * (sw * w / 2) + up * (sh * h / 2) for sw, sh in ((-1, -1), (1, -1), (1, 1), (-1, 1))])
        faces.append((base, base + 1, base + 2, base + 3) if sg < 0 else (base + 3, base + 2, base + 1, base))
    return verts, faces


def flange_rivets(p0, p1, w, h, rv, rf, skip=(), both=True, pitch=None, inset=0.0055):
    """Rivet rows along the top and bottom of the vertical webs of a box member (QTU photos)."""
    p0, p1 = Vector(p0), Vector(p1)
    L = (p1 - p0).length
    d = (p1 - p0).normalized()
    side = d.cross(Vector((0, 0, 1))).normalized()
    pitch = pitch or P.FRAME_RIVET_PITCH
    n = int((L - 0.02) / pitch)
    for sg in ((-1, 1) if both else (1,)):
        for k in range(n + 1):
            s = 0.01 + (L - 0.02) * k / max(n, 1)
            if any(a <= s <= b for a, b in skip):
                continue
            for zs in (-1, 1):
                p = p0 + d * s + side * (sg * (w / 2 + 0.0002)) + Vector((0, 0, zs * (h / 2 - inset)))
                rivet_geo(p, side * sg, P.FRAME_RIVET_R, 0.0013, rv, rf, sides=8)


def plate_geo(pts, n, t, verts, faces):
    """Thin plate (prism) from a convex outline pts (world, CCW about n), thickness t along n."""
    n = Vector(n).normalized()
    base = len(verts)
    k = len(pts)
    verts.extend([Vector(p) for p in pts])
    verts.extend([Vector(p) + n * t for p in pts])
    faces.append(tuple(base + i for i in reversed(range(k))))
    faces.append(tuple(base + k + i for i in range(k)))
    for i in range(k):
        j = (i + 1) % k
        faces.append((base + i, base + j, base + k + j, base + k + i))


def gusset(corner, ex, ey, zt, legs, rv, rf, v, f, t=0.0016):
    """Riveted triangular corner plate on the top flanges where two members meet."""
    c = Vector(corner)
    ex, ey = Vector(ex).normalized(), Vector(ey).normalized()
    pts = [c, c + ex * legs, c + ey * legs]
    if ex.cross(ey).z < 0:
        pts = [pts[0], pts[2], pts[1]]
    plate_geo([Vector((p.x, p.y, zt)) for p in pts], (0, 0, 1), t, v, f)
    for a, b in ((0.25, 0.12), (0.12, 0.25), (0.55, 0.12), (0.12, 0.55), (0.3, 0.3)):
        p = c + ex * (legs * a) + ey * (legs * b)
        rivet_geo(Vector((p.x, p.y, zt + t)), (0, 0, 1), P.FRAME_RIVET_R, 0.0013, rv, rf, sides=8)


def doubler(c, along, n, w, h, rv, rf, v, f, t=0.0016, cols=2, rows=3):
    """Riveted splice / corner doubler plate on a vertical web (7021092 corner fittings)."""
    c, along, n = Vector(c), Vector(along).normalized(), Vector(n).normalized()
    up = Vector((0, 0, 1))
    pts = [c - along * w / 2 - up * h / 2, c + along * w / 2 - up * h / 2,
           c + along * w / 2 + up * h / 2, c - along * w / 2 + up * h / 2]
    if along.cross(up).dot(n) < 0:
        pts.reverse()
    plate_geo(pts, n, t, v, f)
    for i in range(cols):
        for j in range(rows):
            p = c + along * (w * (i + 0.5) / cols - w / 2) * 0.8 + up * (h * (j + 0.5) / rows - h / 2) * 0.8
            rivet_geo(p + n * t, n, P.FRAME_RIVET_R, 0.0013, rv, rf, sides=8)


def build_chassis():
    """Three-section 2219 aluminium frame per A17 Systems Handbook Drawing 3.1.

    Forward (STA 15.50-44.30) and aft (STA 97.30-125.00) sections hinge up over the
    centre section for LM stowage; the hinges are joints held deployed by lock pins.
    """
    ch = link("chassis", (0, 0, 0))
    SECTIONS["center"] = ch
    z0, z1 = P.wl(100.0), P.wl(100.0 + P.FRAME_H_IN)
    zc = (z0 + z1) / 2
    w, h = P.FRAME_W_IN * P.IN, P.FRAME_H_IN * P.IN
    al = M.get("aluminium")

    # centre section: outer frame, centre frame member, beaded floor panels underneath
    xf, xa = P.sta(P.STA_FWD_HINGE), P.sta(P.STA_AFT_HINGE)
    yo = P.bl(P.CENTER_WIDTH_IN / 2) - w / 2
    v, f = [], []
    for y in (-yo, 0.0, yo):
        box_beam_geo((xf, y, zc), (xa, y, zc), w, h, v, f)
    for x in (xf - w / 2, xa + w / 2):
        box_beam_geo((x, -yo - w / 2, zc), (x, yo + w / 2, zc), w, h, v, f)
    add_mesh("chassis_center_frame", v, f, al, ch)
    # rivet rows along the side-member flanges, riveted corner plates where the cross members meet
    v, f, rv, rf = [], [], [], []
    for y in (-yo, yo):
        flange_rivets((xf, y, zc), (xa, y, zc), w, h, rv, rf)
    for x, ex in ((xf - w / 2, -1), (xa + w / 2, 1)):
        for sy in (-1, 1):
            gusset((x - ex * w / 2, sy * (yo - w / 2), 0), (ex, 0, 0), (0, -sy, 0), z1, 0.085, rv, rf, v, f)
            gusset((x - ex * w / 2, sy * w / 2, 0), (ex, 0, 0), (0, sy, 0), z1, 0.07, rv, rf, v, f)
            gusset((x - ex * w / 2, -sy * w / 2, 0), (ex, 0, 0), (0, -sy, 0), z1, 0.07, rv, rf, v, f)
    add_mesh("chassis_center_gussets", v, f, al, ch)
    add_mesh("chassis_center_rivets", rv, rf, al, ch, smooth=True)
    # black alignment marks along the outboard faces of the side members (QTU NASM2020-00047)
    v, f = [], []
    for s in (-1, 1):
        x = xf - 0.06
        k = 0
        while x > xa + 0.05:
            box_geo((x, s * (yo + w / 2 + 0.0004), zc - h * 0.12), (P.MARK_W, 0.0008, h * 0.62), v, f)
            x -= P.MARK_PITCH[k % len(P.MARK_PITCH)]
            k += 1
    add_mesh("chassis_marks", v, f, M.get("black_paint"), ch)
    v, f = [], []
    for s in (-1, 1):
        beaded_panel_geo(xf - w, xa + w, s * w / 2, s * (yo - w / 2), z0 - 0.0005, v, f)
    add_mesh("floor_center", v, f, M.get("floor_panel"), ch, smooth=True)

    # hinged end sections
    for name, sx, s_hinge, s_end, lim in (("fwd", 1, P.STA_FWD_HINGE, P.STA_FWD_END, (-math.pi, 0.0)),
                                           ("aft", -1, P.STA_AFT_HINGE, P.STA_AFT_END, (0.0, math.pi))):
        xh, xe = P.sta(s_hinge), P.sta(s_end)
        sec = link(f"chassis_{name}", (xh, 0.0, z1), ch, {
            "type": "revolute", "axis": (0.0, 1.0, 0.0), "limit": lim, "role": "chassis_fold",
            "drive": {"kind": "position", "stiffness": P.FOLD_LOCK_K, "damping": P.FOLD_LOCK_D, "target": 0.0}},
            mass=P.MASS[f"chassis_{name}"])
        SECTIONS[name] = sec
        ys = P.bl(P.END_SIDE_BL_IN)
        yi = P.bl(P.END_INNER_BL_IN)
        v, f = [], []
        xm = (xh + xe) / 2
        for y in (-ys, ys):
            box_beam_geo((xh - sx * 0.01, y, zc), (xe, y, zc), w, h, v, f)
        # inner longerons: lightening holes in the bays either side of the mid cross member
        ln = abs(xe - xh) + 0.01
        bays = [(0.25 * ln, 0.05 * ln), (0.75 * ln, 0.05 * ln)]
        lholes = [(c + o, P.LONGERON_HOLE_R) for c, _ in bays for o in (-0.085, 0.0, 0.085)]
        for y in (-yi, yi):
            holed_beam_geo((xh - sx * 0.01, y, zc), (xe, y, zc), w, h, lholes, v, f)
        # end and mid cross members: round lightening holes, riveted flanges (7021092.jpg)
        span = 2 * (ys + w / 2)
        xholes = [(span / 2 + c, r) for c, r in P.CROSS_MEMBER_HOLES]
        for x in (xe + sx * w / 2, xm):
            holed_beam_geo((x, -ys - w / 2, zc), (x, ys + w / 2, zc), w, h, xholes, v, f)
        box_beam_geo((xh - sx * (w / 2 + 0.01), -ys - w / 2, zc), (xh - sx * (w / 2 + 0.01), ys + w / 2, zc), w, h, v, f)
        rv, rf = [], []
        for x in (xe + sx * w / 2, xm):
            flange_rivets((x, -ys - w / 2, zc), (x, ys + w / 2, zc), w, h, rv, rf)
            for y in (-yi, yi):        # riveted doublers where the longerons butt in
                for sg in (-1, 1):
                    doubler((x + sg * (w / 2 + 0.0002), y, zc - 0.004), (0, 1, 0), (sg, 0, 0), 0.06, h * 0.72,
                            rv, rf, v, f)
            for sy in (-1, 1):         # corner fittings at the side members
                doubler((x + sx * (w / 2 + 0.0002), sy * (ys - 0.045), zc - 0.004), (0, 1, 0), (sx, 0, 0), 0.07,
                        h * 0.72, rv, rf, v, f, cols=3)
        for y in (-ys, ys, -yi, yi):
            flange_rivets((xh - sx * 0.01, y, zc), (xe, y, zc), w, h, rv, rf, pitch=1.4 * P.IN)
        for sy in (-1, 1):
            for yy, ey in ((ys - w / 2, -sy), (yi + w / 2, sy), (yi - w / 2, -sy)):
                gusset((xe + sx * w / 2 - sx * w / 2, sy * yy, 0), (-sx, 0, 0), (0, ey, 0), z1, 0.07, rv, rf, v, f)
        add_mesh(f"chassis_{name}_rivets", rv, rf, al, sec, smooth=True)
        # suspension support fittings: clevis plates carrying both arm pivots
        xw = sx * P.WHEELBASE / 2
        for sy in (-1, 1):
            for xo in (xw - P.ARM_BASE / 2, xw + P.ARM_BASE / 2):
                yl = sy * (P.LOWER_PIVOT_Y + 0.004)
                box_geo((xo, sy * (ys + w / 2 + 0.012), (z0 + z1) / 2 - 0.02), (0.05, 0.03, h + 0.07), v, f)
                box_geo((xo, yl, P.WHEEL_RADIUS + P.LOWER_PIVOT_DZ), (0.05, 0.03, 0.04), v, f)
                box_geo((xo, sy * (P.UPPER_PIVOT_Y + 0.004), P.WHEEL_RADIUS + P.UPPER_PIVOT_DZ), (0.05, 0.03, 0.04), v, f)
        add_mesh(f"chassis_{name}_frame", v, f, al, sec)
        # hinge fittings at the top corners, yellow-painted lock pins
        v, f, pv, pf = [], [], [], []
        for y in (-ys, ys):
            box_geo((xh, y, z1 - 0.01), (0.07, w + 0.012, 0.05), v, f)
            tube_geo((xh, y - w / 2 - 0.01, z1 + 0.005), (xh, y + w / 2 + 0.01, z1 + 0.005), 0.009, 12, True, v, f)
            tube_geo((xh, y, z0 - 0.012), (xh, y, z0 + 0.004), 0.006, 10, True, pv, pf)
        add_mesh(f"chassis_{name}_hinges", v, f, M.get("steel"), sec, smooth=True)
        add_mesh(f"chassis_{name}_lockpins", pv, pf, M.get("yellow_paint"), sec, smooth=True)
        v, f = [], []
        beaded_panel_geo(xh - sx * w, xe + sx * w, -ys + w / 2, ys - w / 2, z0 - 0.0005, v, f)
        add_mesh(f"floor_{name}", v, f, M.get("floor_panel"), sec, smooth=True)

        # torsion bars inside the side members, inboard from the arm pivots
        tb = []
        for sy in (-1, 1):
            for py, dz, ln in ((P.LOWER_PIVOT_Y, P.LOWER_PIVOT_DZ, P.TORSION_BAR_LEN),
                               (P.UPPER_PIVOT_Y, P.UPPER_PIVOT_DZ, P.TORSION_BAR_UPPER_LEN)):
                z = P.WHEEL_RADIUS + dz
                tb.append(((xw + sx * P.ARM_BASE / 2, sy * py, z), (xw + sx * P.ARM_BASE / 2 - sx * ln, sy * py, z)))
        tubes(f"torsion_bars_{name}", tb, 0.011, M.get("steel"), sec)
    return ch


def assign_to_sections(ch):
    """Move anything parented straight to the centre link onto the section it physically sits on."""
    xf, xa = P.sta(P.STA_FWD_HINGE), P.sta(P.STA_AFT_HINGE)
    for ob in list(ch.children):
        if ob in (SECTIONS["fwd"], SECTIONS["aft"]):
            continue
        if ob.type == "MESH":
            pts = [ob.matrix_world @ Vector(c) for c in ob.bound_box]
            x = sum(p.x for p in pts) / 8
        else:
            x = ob.matrix_world.translation.x
        target = SECTIONS["fwd"] if x > xf else SECTIONS["aft"] if x < xa else None
        if target is None:
            continue
        mw = ob.matrix_world.copy()
        ob.parent = target
        ob.matrix_parent_inverse = target.matrix_world.inverted()
        ob.matrix_world = mw


def build_steering_linkage(arm_ends):
    """Double Ackermann, A17 Systems Handbook DWG 4.6: per axle the steering motor + gear train turns a
    triangular sector on the centre line, and two ball-jointed tie rods run out to the knuckle arms.

    chassis section -> <end>_steer_sector (revolute, position servo: the steering motor, role "steer")
    sector -> <corner>_tie_rod (spherical at the sector stud), closed to the knuckle arm by a spherical loop
    joint. The kingpins are passive and follow the sector through the real linkage (params.steer_linkage).
    Both sectors take the same command, + = left turn: the rear sector's axis is -Z so the rear wheels counter-steer.
    """
    lim = P.steer_sector_limit()
    for name, sx in (("fwd", 1), ("aft", -1)):
        sec = SECTIONS[name]
        corners = [c for c, (cx, _) in P.CORNERS.items() if cx == sx]
        piv = Vector(P.steer_geometry(corners[0])[0])
        z = piv.z
        sector = link(f"{name}_steer_sector", piv, sec, {
            "type": "revolute", "axis": (0.0, 0.0, float(sx)), "limit": (-lim, lim), "role": "steer",
            "drive": {"kind": "position", "stiffness": P.SECTOR_K, "damping": P.SECTOR_D,
                      "max_force": P.SECTOR_MAX_TORQUE, "target": 0.0}})
        # sector plate: apex boss at the pivot, wide end on an arc about it carrying the gear teeth
        u = Vector((-sx, 0, 0))                      # pivot -> wide end (toward the vehicle centre)
        lat = Vector((0, 1, 0))
        r_gear, half = 0.160, math.radians(28)
        z0, t = z - 0.022, 0.008                     # plate under the stud plane
        outline = []
        for k in range(9):                           # apex rounding
            a = math.pi / 2 + math.pi * k / 8
            outline.append(piv + 0.028 * (math.cos(a) * u + math.sin(a) * lat))
        for k in range(13):                          # wide-end arc
            a = -half + 2 * half * k / 12
            outline.append(piv + r_gear * (math.cos(a) * u + math.sin(a) * lat))
        outline = [p + Vector((0, 0, z0 - p.z)) for p in outline]
        if sx > 0:
            outline.reverse()                        # keep CCW about +Z
        v, f = [], []
        plate_geo(outline, (0, 0, 1), t, v, f)
        tube_geo(piv + Vector((0, 0, z0 - z - 0.006)), piv + Vector((0, 0, 0.008)), 0.024, 16, True, v, f)
        add_mesh(f"{name}_steer_sector", v, f, M.get("aluminium"), sector)
        v, f = [], []
        for k in range(29):                          # spur-gear teeth on the wide end
            a = -half * 0.95 + 1.9 * half * k / 28
            c = piv + (r_gear + 0.005) * (math.cos(a) * u + math.sin(a) * lat) + Vector((0, 0, z0 - z + t / 2))
            box_beam_geo(c - 0.005 * (math.cos(a) * u + math.sin(a) * lat),
                         c + 0.005 * (math.cos(a) * u + math.sin(a) * lat), 0.004, t, v, f)
        for c in corners:                            # tie-rod ball studs at the wide-end corners
            stud = Vector(P.steer_geometry(c)[1])
            tube_geo(stud + Vector((0, 0, z0 + t - z)), stud + Vector((0, 0, 0.004)), 0.007, 10, True, v, f)
        add_mesh(f"{name}_steer_sector_gear", v, f, M.get("steel"), sector, smooth=True)

        # steering motor + gear train housing under the frame, inboard of the sector's wide end (its output
        # pinion meshes with the teeth); the pivot shaft hangs from a bracket off the mid cross member
        zt = P.wl(100.0)
        xa, xb = P.STEER_SECTOR_X - r_gear - 0.125, P.STEER_SECTOR_X - r_gear - 0.012
        v, f = box_geo((sx * (xa + xb) / 2, 0, (z0 - 0.012 + zt) / 2), (xb - xa, 0.11, zt - (z0 - 0.012)))
        tube_geo(Vector((sx * xa, 0, z0 + 0.012)), Vector((sx * (xa - 0.12), 0, z0 + 0.012)), 0.026, 16, True, v, f)
        box_geo((sx * (P.STEER_SECTOR_X - 0.03), 0, zt - 0.0035), (0.15, 0.06, 0.007), v, f)
        add_mesh(f"{name}_steer_drive", v, f, M.get("drive_housing"), sec, smooth=False)

        for c in corners:
            _, cy = P.CORNERS[c]
            stud = Vector(P.steer_geometry(c)[1])
            end = arm_ends[c]
            rod = link(f"{c}_tie_rod", stud, sector, {"type": "spherical", "role": "passive"})
            d = (end - stud).normalized()
            v, f = [], []
            for p in (stud, end):                    # ball-socket housings round the studs
                tube_geo(p + Vector((0, 0, -0.010)), p + Vector((0, 0, 0.008)), 0.014, 12, True, v, f)
            tube_geo(stud + d * 0.012, end - d * 0.012, 0.0075, 10, True, v, f)
            mid = stud.lerp(end, 0.22)                # length-adjuster sleeve near the inboard end
            tube_geo(mid - d * 0.04, mid + d * 0.04, 0.0105, 6, True, v, f)
            add_mesh(f"{c}_tie_rod", v, f, M.get("steel"), rod, smooth=True)
            loop_joint(f"{c}_tie_rod_ball", f"{c}_steer", f"{c}_tie_rod", end, (1.0, 0.0, 0.0), jtype="spherical")


# ---------------------------------------------------------------- fenders

def fender_section(hw, skirt, rc=0.035, n_skirt=4, n_corner=5, n_crown=10):
    """U-channel cross-section (y offset, radial offset): inboard skirt, crown, outboard skirt.

    Refs 7020254.jpg / 7021092.jpg: the fibreglass fender is a deep channel whose side walls
    come down both faces of the tire. Index 0 is the -Y side.
    """
    pts = []
    for k in range(n_skirt):                                   # -Y skirt, bottom up
        pts.append((-hw, -skirt + (skirt - rc) * k / n_skirt))
    for k in range(n_corner + 1):                              # rounded corner
        a = math.pi / 2 * k / n_corner
        pts.append((-hw + rc - rc * math.cos(a), -rc + rc * math.sin(a) + fender_crown(-hw + rc, hw) * math.sin(a)))
    for k in range(1, n_crown):
        y = -hw + rc + (2 * hw - 2 * rc) * k / n_crown
        pts.append((y, fender_crown(y, hw)))
    for y, r in reversed(pts[:n_skirt + n_corner + 1]):        # mirror
        pts.append((-y, r))
    return pts


def fender_crown(y, hw):
    return P.FENDER_CROWN * (1 - (y / hw) ** 2)


def fender_geo(corner, a_start, a_end, inset=0.0):
    """Channel over the wheel. Angles measured from top-dead-centre toward the vehicle end."""
    sx, sy = P.CORNERS[corner]
    wc = Vector(P.wheel_center(corner))
    R = P.WHEEL_RADIUS + P.FENDER_CLEARANCE + inset
    cy = sy * (P.TRACK / 2 - 0.01)
    sec = fender_section(P.FENDER_WIDTH / 2 + inset * 0.6, P.FENDER_SKIRT)
    na, nw = 24, len(sec) - 1
    verts, faces = [], []
    for i in range(na + 1):
        t = a_start + (a_end - a_start) * i / na
        for dy, dr in sec:
            r = R + dr
            verts.append(Vector((wc.x + sx * r * math.sin(t), cy + dy, wc.z + r * math.cos(t))))
    for i in range(na):
        for j in range(nw):
            q = (i * (nw + 1) + j, i * (nw + 1) + j + 1, (i + 1) * (nw + 1) + j + 1, (i + 1) * (nw + 1) + j)
            faces.append(q[::-1] if sx > 0 else q)  # outward-facing normals
    return verts, faces, na, nw


def fender_bead(verts, na, nw):
    """Rolled edge around the fibreglass shell: closed loop along the boundary of the grid."""
    idx = lambda i, j: i * (nw + 1) + j  # noqa: E731
    loop = [idx(i, 0) for i in range(na + 1)] + [idx(na, j) for j in range(1, nw + 1)]
    loop += [idx(i, nw) for i in range(na - 1, -1, -1)] + [idx(0, j) for j in range(nw - 1, 0, -1)]
    return [verts[a] for a in loop]


def build_fenders():
    """Fibreglass fenders ride on the steering knuckles (they steer and fold with the wheel).

    The extension slides around the wheel arc on rails: a revolute joint about the axle,
    rest pose = deployed, retracted toward top-dead-centre for stowage.
    """
    for corner, (sx, sy) in P.CORNERS.items():
        steer = STEER[corner]
        wc = Vector(P.wheel_center(corner))
        v, f, na, nw = fender_geo(corner, *P.FENDER_FIXED)
        o = add_mesh(f"{corner}_fender", v, f, M.get("fender"), steer, smooth=True)
        _shell(o)
        pipe(f"{corner}_fender_bead", fender_bead(v, na, nw), 0.007, M.get("fender"), steer, closed=True)
        # support strut from the fender's inboard edge down to the traction drive
        t = (P.FENDER_FIXED[0] + P.FENDER_FIXED[1]) / 2
        r = P.WHEEL_RADIUS + P.FENDER_CLEARANCE
        r_sk = r - P.FENDER_SKIRT + 0.02
        top = Vector((wc.x + sx * r_sk * math.sin(t), sy * (P.TRACK / 2 - 0.01 - P.FENDER_WIDTH / 2),
                      wc.z + r_sk * math.cos(t)))
        foot = Vector((wc.x, sy * (P.KINGPIN_Y - 0.02), wc.z + 0.075))  # bracket on the kingpin housing
        tubes(f"{corner}_fender_strut", [(top, foot)], 0.008, M.get("aluminium"), steer)

        retract = P.FENDER_EXT[1] - P.FENDER_EXT[0]  # slides back fully over the fixed fender
        lim = (-retract, 0.0) if sx > 0 else (0.0, retract)
        ext = link(f"{corner}_fender_ext", wc, steer, {
            "type": "revolute", "axis": (0.0, 1.0, 0.0), "limit": lim, "role": "fender_extension",
            "drive": {"kind": "position", "stiffness": 50.0, "damping": 2.0, "target": 0.0}},
            mass=P.MASS["fender_ext"])
        v, f, na, nw = fender_geo(corner, *P.FENDER_EXT, inset=0.008)
        o = add_mesh(f"{corner}_fender_ext", v, f, M.get("fender"), ext, smooth=True)
        _shell(o)
        pipe(f"{corner}_fender_ext_bead", fender_bead(v, na, nw), 0.007, M.get("fender"), ext, closed=True)
        if corner in P.FENDER_EXT_MAROON:   # Smithsonian vehicle: maroon extension replaces the ochre one
            o = add_mesh(f"{corner}_fender_ext_nasm", v, f, M.get("fender_ext"), ext, smooth=True, variant="Mission:nasm")
            _shell(o)
            b = pipe(f"{corner}_fender_ext_bead_nasm", fender_bead(v, na, nw), 0.007, M.get("fender_ext"), ext, closed=True)
            tag_variant(b, "Mission:nasm")
            for n in (f"{corner}_fender_ext", f"{corner}_fender_ext_bead"):
                tag_variant(bpy.data.objects[n], "Mission:a15,a17,a17_repair")
        fender_hardware(corner, steer, ext)
        fender_stops(corner, steer)
        if sx > 0:
            flag_decal(corner, ext)
            fender_flap(corner, steer)
        else:
            flag_decal(corner, ext, variant=f"Mission:{P.REAR_FLAG_MISSIONS}")
        if corner == "rr":   # lost on A17 EVA-1: absent in the map-repair configuration (Mission:a17_repair)
            for n in ("rr_fender_ext", "rr_fender_ext_bead", "rr_fender_ext_rivets"):
                tag_variant(bpy.data.objects[n], "Mission:a15,a17,nasm")
    build_fender_repair()


def obox_geo(c, ex, ey, ez, size, verts, faces):
    """Oriented box centred at c with (orthonormal) axes ex, ey, ez and full extents size."""
    c = Vector(c)
    ax = [Vector(ex) * size[0] / 2, Vector(ey) * size[1] / 2, Vector(ez) * size[2] / 2]
    if ax[0].cross(ax[1]).dot(ax[2]) < 0:
        ax[2] = -ax[2]
    base = len(verts)
    for z in (-1, 1):
        for y in (-1, 1):
            for x in (-1, 1):
                verts.append(c + ax[0] * x + ax[1] * y + ax[2] * z)
    for q in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        faces.append(tuple(base + i for i in q))
    return verts, faces


def fender_stops(corner, steer):
    """Fender extension stops, an LRV-3 addition (A17 Mission Report App. A.3.2) after the A15/A16 losses.

    No drawing located: modelled as riveted aluminium blocks at the deployed end of the slide rails on
    both skirts, which the extension's slider shoes bear against. Tagged to the A17 options only.
    """
    sx, sy = P.CORNERS[corner]
    t = P.FENDER_FIXED[1] - 0.045
    v, f, rv, rf = [], [], [], []
    L, Wd, H = P.FENDER_STOP_SIZE
    for s_ in (0.04, 0.96):
        p, n = fender_point(corner, t, s_, lift=0.0045)
        p2, _ = fender_point(corner, t + 0.01, s_, lift=0.0045)
        et = (p2 - p).normalized()
        er = n.cross(et).normalized()
        obox_geo(p + n * (Wd / 2), et, n, er, (L, Wd, H), v, f)                # stop block
        obox_geo(p + n * 0.0008, et, n, er, (L + 0.026, 0.0016, H * 0.8), v, f)   # riveted base plate
        for k in (-1, 1):
            rivet_geo(p + n * 0.0016 + et * (k * (L / 2 + 0.007)), n, 0.0032, 0.0018, rv, rf, sides=8)
    tag = "Mission:a17,a17_repair"
    add_mesh(f"{corner}_fender_stops", v, f, M.get("aluminium"), steer, variant=tag)
    add_mesh(f"{corner}_fender_stop_rivets", rv, rf, M.get("brass"), steer, smooth=True, variant=tag)


# ---------------------------------------------------------------- Apollo 17 right-rear fender repair

REPAIR_TEX = "a17_repair/maps_stylised_v3.png"


def _repair_map_texture():
    """Generate (once) a STYLISED laminated-map texture: 2 x 2 photomap sheets with contours, grid,
    craters, a traverse line and margin print blocks. It is not a reproduction of the actual A17
    traverse maps; it only gives the patch a plausible map-paper look. Built with numpy inside Blender
    (PIL is not available there), cached in textures/gen/."""
    import numpy as np
    path = os.path.abspath(os.path.join(M.TEX_DIR, REPAIR_TEX))
    if os.path.exists(path):
        return path
    os.makedirs(os.path.dirname(path), exist_ok=True)
    rng = np.random.default_rng(17)
    MW, MH = 768, 1024                       # one map (u across the fender, v along it)

    def upsample(a, h, w):
        ys, xs = np.linspace(0, a.shape[0] - 1, h), np.linspace(0, a.shape[1] - 1, w)
        y0, x0 = np.floor(ys).astype(int), np.floor(xs).astype(int)
        y1, x1 = np.minimum(y0 + 1, a.shape[0] - 1), np.minimum(x0 + 1, a.shape[1] - 1)
        fy, fx = (ys - y0)[:, None], (xs - x0)[None, :]
        fy, fx = fy * fy * (3 - 2 * fy), fx * fx * (3 - 2 * fx)
        top = a[y0][:, x0] * (1 - fx) + a[y0][:, x1] * fx
        bot = a[y1][:, x0] * (1 - fx) + a[y1][:, x1] * fx
        return top * (1 - fy) + bot * fy

    def fbm(h, w, oct=6):
        out = np.zeros((h, w))
        for o in range(oct):
            n = 3 * 2 ** o
            out += upsample(rng.random((n, int(n * w / h) + 1)), h, w) / 2 ** o
        return out / 2

    yy, xx = np.mgrid[0:MH, 0:MW].astype(float)
    maps = []
    for m in range(4):
        hgt = fbm(MH, MW)
        photo = 0.70 + 0.32 * (hgt - hgt.mean()) + 0.08 * (fbm(MH, MW, 4) - 0.5)
        for _ in range(45):                   # shaded craters, sun from the left
            cx, cy, r = rng.uniform(0, MW), rng.uniform(0, MH), rng.uniform(6, 60) * rng.random() ** 1.5 + 4
            d = np.hypot(xx - cx, yy - cy) / r
            ins = d < 1
            photo[ins] += -0.22 * ((xx - cx) / r)[ins] * np.sqrt(1 - d[ins] ** 2) * -1 - 0.06
            rim = (d > 0.95) & (d < 1.15)
            photo[rim] += 0.08
        photo = np.clip(photo, 0, 1)
        rgb = np.stack([photo * 0.96, photo * 0.95, photo * 0.92], -1)
        cval = hgt * 14                        # contours: brown, every 5th heavier, constant pixel width
        gy, gx = np.gradient(cval)
        gr = np.hypot(gx, gy) + 1e-6
        c, c5 = cval % 1.0, (cval / 5) % 1.0
        line = np.minimum(c, 1 - c) < 0.9 * gr
        heavy = np.minimum(c5, 1 - c5) < 1.6 * gr / 5
        rgb[line] = rgb[line] * 0.45 + np.array([0.55, 0.32, 0.15]) * 0.55
        rgb[heavy] = np.array([0.45, 0.24, 0.10])
        g = 96                                # black grid with fine ticks
        grid = (np.abs((xx % g) - g / 2) > g / 2 - 1.2) | (np.abs((yy % g) - g / 2) > g / 2 - 1.2)
        rgb[grid] = rgb[grid] * 0.25
        pts = [(rng.uniform(80, MW - 80), y) for y in np.linspace(60, MH - 60, 6)]
        for (x0_, y0_), (x1_, y1_) in zip(pts, pts[1:]):   # traverse route + station circles
            dx, dy = x1_ - x0_, y1_ - y0_
            t = np.clip(((xx - x0_) * dx + (yy - y0_) * dy) / (dx * dx + dy * dy), 0, 1)
            dist = np.hypot(xx - x0_ - t * dx, yy - y0_ - t * dy)
            rgb[dist < 3.0] = np.array([0.85, 0.12, 0.08])
            st = np.abs(np.hypot(xx - x1_, yy - y1_) - 14) < 2.2
            rgb[st] = np.array([0.1, 0.1, 0.12])
        border = 34                           # white margin with blocky "print" and a title bar
        marg = (xx < border) | (xx > MW - border) | (yy < border * 2.2) | (yy > MH - border)
        rgb[marg] = np.array([0.93, 0.92, 0.88])
        rgb[(yy > MH - border + 8) & (yy < MH - 10) & (xx > 60) & (xx < 380)] = 0.1
        for row in range(3):
            y0_ = 12 + row * 22
            x = 40.0
            while x < MW - 60:
                wlen = rng.uniform(10, 46)
                rgb[(yy > y0_) & (yy < y0_ + 8) & (xx > x) & (xx < x + wlen)] = 0.15
                x += wlen + rng.uniform(6, 12)
        frame = ((np.abs(xx - border) < 1.5) | (np.abs(xx - (MW - border)) < 1.5) | (np.abs(yy - border * 2.2) < 1.5)
                 | (np.abs(yy - (MH - border)) < 1.5))
        rgb[frame] = 0.05
        maps.append(rgb)
    img = np.concatenate([np.concatenate(maps[:2], 1), np.concatenate(maps[2:], 1)], 0)   # rows = v
    sheen = 0.03 * (fbm(img.shape[0], img.shape[1], 3) - 0.5)[..., None]                   # lamination
    img = np.clip(img + sheen, 0, 1)
    H, Wt = img.shape[:2]
    rgba = np.concatenate([img, np.ones((H, Wt, 1))], -1).astype(np.float32)
    im = bpy.data.images.new("a17_repair_maps", Wt, H, alpha=False)
    im.pixels.foreach_set(rgba.ravel())       # row 0 = bottom = v 0 (leading edge)
    im.filepath_raw = path
    im.file_format = "PNG"
    im.save()
    bpy.data.images.remove(im)
    return path


def _repair_sheet(corner):
    """Parametric surface of the map patch: (u across, v along, lift) -> (point, normal).

    The stiff laminated sheet follows the fixed fender's crown from just forward of top-dead-centre to the
    aft edge, then runs on straight (drooping slightly) where the extension used to be; its side flaps
    are folded down over both skirts (as17-137-20979, as17-140-21494).
    """
    sx, sy = P.CORNERS[corner]
    wc = Vector(P.wheel_center(corner))
    gap = 0.011
    hw2 = P.FENDER_WIDTH / 2 + 0.013
    sec = fender_section(hw2, P.REPAIR_FLAP, rc=0.03)
    arc = [0.0]
    for (y0, r0), (y1, r1) in zip(sec, sec[1:]):
        arc.append(arc[-1] + math.hypot(y1 - y0, r1 - r0))
    R = P.WHEEL_RADIUS + P.FENDER_CLEARANCE + gap
    t0, t1 = P.REPAIR_SHEET_START, P.FENDER_FIXED[1]
    La, Lt = (t1 - t0) * R, P.REPAIR_TAIL_LEN
    cy = sy * (P.TRACK / 2 - 0.01)

    def at(u, v, lift=0.0, drop=0.0):
        a = min(max(u, 0.0), 1.0) * arc[-1]
        i = min(max(next((k for k in range(len(arc) - 1) if arc[k + 1] >= a), len(arc) - 2), 0), len(sec) - 2)
        k = (a - arc[i]) / max(arc[i + 1] - arc[i], 1e-9)
        (y0, r0), (y1, r1) = sec[i], sec[i + 1]
        dy, dr = y0 + (y1 - y0) * k, r0 + (r1 - r0) * k
        tan = Vector((y1 - y0, r1 - r0)).normalized()
        ny, nr = -tan.y, tan.x
        if nr < 0 or (abs(nr) < 1e-6 and dy * ny < 0):
            ny, nr = -ny, -nr
        s = v * (La + Lt)
        t = t0 + min(s, La) / R
        r = R + dr + nr * lift - drop
        rad = Vector((sx * math.sin(t), 0, math.cos(t)))
        p = Vector((wc.x, cy + dy + ny * lift, wc.z)) + rad * r
        n = Vector((sx * nr * math.sin(t), ny, nr * math.cos(t))).normalized()
        if s > La:                                       # straight tail, sagging a little
            e = s - La
            tg = Vector((sx * math.cos(t1), 0, -math.sin(t1)))
            sag = 0.35 * e * e
            p += tg * e - rad * sag
            n = (Matrix.Rotation(-sx * math.atan(0.7 * e), 3, "Y") @ n).normalized()
        return p, n
    return at, La / (La + Lt)


def build_fender_repair():
    """Apollo 17 EVA-2 right-rear fender repair (Mission variant "a17_repair").

    A17 Mission Report sec. 9.1 / Fig 9-1 and sec. 10: the right-rear extension was knocked off on EVA-1
    and lost; before EVA-2 the crew taped four laminated lunar surface maps together and clamped them to
    the fixed fender with two utility clamps from the LM optical alignment telescope. Map printing is a
    stylised generated texture, not the real maps.
    """
    corner, var = "rr", "Mission:a17_repair"
    sx, sy = P.CORNERS[corner]
    steer = STEER[corner]
    at, v_aft = _repair_sheet(corner)
    nu, nv = 40, 64
    verts, faces, uvs = [], [], []
    for j in range(nv + 1):
        for i in range(nu + 1):
            p, _ = at(i / nu, j / nv)
            verts.append(p)
            uvs.append((i / nu, j / nv))
    for j in range(nv):
        for i in range(nu):
            a = j * (nu + 1) + i
            q = (a, a + 1, a + nu + 2, a + nu + 1)
            faces.append(q if sx * sy > 0 else q[::-1])
    _repair_map_texture()
    mat = bpy.data.materials.get("a17_repair_maps") or M._decal("a17_repair_maps", REPAIR_TEX, rough=0.28)
    bsdf = next(n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED")
    for k, val in (("Coat Weight", 0.6), ("Coat Roughness", 0.12)):   # laminate film
        if k in bsdf.inputs:
            bsdf.inputs[k].default_value = val
    o = add_mesh("rr_fender_repair_maps", verts, faces, mat, steer, uvs=uvs, smooth=True, variant=var)
    _shell(o, 0.0022)

    # grey tape: along the map seams, and the leading edge taped down onto the fender crown
    tv, tf = [], []

    def strip(u0, u1, v0, v1, nu_=24, nv_=24, drop_front=False):
        base = len(tv)
        for j in range(nv_ + 1):
            vv = v0 + (v1 - v0) * j / nv_
            for i in range(nu_ + 1):
                uu = u0 + (u1 - u0) * i / nu_
                drop = 0.0
                if drop_front and vv < 0:
                    drop = min(1.0, -vv / 0.03) * 0.0085      # tape folds down onto the fender surface
                p, _ = at(uu, max(vv, 0.0) if not drop_front else vv, 0.0018, drop)
                tv.append(p)
        for j in range(nv_):
            for i in range(nu_):
                a = base + j * (nu_ + 1) + i
                q = (a, a + 1, a + nu_ + 2, a + nu_ + 1)
                tf.append(q if sx * sy > 0 else q[::-1])
    strip(0.0, 1.0, 0.475, 0.525, 40, 2)
    strip(0.47, 0.53, 0.0, 1.0, 2, 64)
    strip(0.12, 0.88, -0.06, 0.035, 30, 6, drop_front=True)
    strip(0.0, 1.0, v_aft - 0.03, v_aft + 0.02, 40, 2)
    tape = M.define("repair_tape", (0.50, 0.50, 0.48), 0.0, 0.65)
    o = add_mesh("rr_fender_repair_tape", tv, tf, tape, steer, smooth=True, variant=var)
    _shell(o, 0.0006)

    # two utility clamps (LM optical alignment telescope), one on each skirt near the fender's aft edge,
    # gripping the folded map flap against the skirt; T-handle screw outboard (as17-137-20979)
    wc = Vector(P.wheel_center(corner))
    t = P.REPAIR_CLAMP_ANGLE
    rad = Vector((sx * math.sin(t), 0, math.cos(t)))
    tg = Vector((sx * math.cos(t), 0, -math.sin(t)))
    R0 = P.WHEEL_RADIUS + P.FENDER_CLEARANCE
    cy = sy * (P.TRACK / 2 - 0.01)
    cv, cf, sv, sf = [], [], [], []
    for side in (-1, 1):
        ny = Vector((0, side, 0))
        y_sk = cy + side * P.FENDER_WIDTH / 2                # skirt mid-plane
        y_in = y_sk - side * 0.012                           # fixed jaw inside the skirt
        y_out = y_sk + side * 0.034                          # screw boss outside the map flap
        P_ = lambda y, d: Vector((wc.x, y, wc.z)) + rad * (R0 + d)  # noqa: E731
        d_jaw, d_sp = -0.035, -0.072                         # jaw line / spine below the skirt edge
        for a, b in ((P_(y_in, d_jaw), P_(y_in, d_sp)), (P_(y_in, d_sp), P_(y_out, d_sp)),
                     (P_(y_out, d_sp), P_(y_out, d_jaw))):
            box_beam_geo(a, b, 0.016, 0.009, cv, cf) if abs((a - b).z) < abs((a - b).y) else                 obox_geo((a + b) / 2, (b - a).normalized(), tg, (b - a).normalized().cross(tg),
                         ((b - a).length + 0.009, 0.016, 0.009), cv, cf)
        obox_geo(P_(y_in + side * 0.0035, d_jaw + 0.006), ny, tg, rad, (0.004, 0.02, 0.016), cv, cf)  # fixed pad
        # screw from the boss through to the swivel pad on the map flap; T-handle on the outer end
        pad_y = cy + side * (P.FENDER_WIDTH / 2 + 0.013 + 0.004)
        tube_geo(P_(y_out + side * 0.03, d_jaw + 0.004), P_(pad_y + side * 0.002, d_jaw + 0.004), 0.0035, 10, True, sv, sf)
        tube_geo(P_(pad_y, d_jaw + 0.004), P_(pad_y + side * 0.004, d_jaw + 0.004), 0.009, 14, True, sv, sf)
        hc = P_(y_out + side * 0.03, d_jaw + 0.004)
        tube_geo(hc - tg * 0.028, hc + tg * 0.028, 0.0035, 10, True, sv, sf)
        for e in (-1, 1):
            tube_geo(hc + tg * (e * 0.028), hc + tg * (e * 0.034), 0.0055, 10, True, sv, sf)
    add_mesh("rr_fender_repair_clamps", cv, cf, M.get("aluminium"), steer, variant=var)
    add_mesh("rr_fender_repair_clamp_screws", sv, sf, M.get("steel"), steer, smooth=True, variant=var)


def _shell(ob, t=0.004):
    """Fibreglass thickness centred on the surface, so winding direction doesn't matter."""
    s = ob.modifiers.new("thick", "SOLIDIFY")
    s.thickness = t
    s.offset = 0.0


def fender_point(corner, t, s, inset=0.0, lift=0.0):
    """World point (and outward normal) on a fender: angle t, s in [0,1] across the U-section."""
    sx, sy = P.CORNERS[corner]
    wc = Vector(P.wheel_center(corner))
    sec = fender_section(P.FENDER_WIDTH / 2 + inset * 0.6, P.FENDER_SKIRT)
    f = s * (len(sec) - 1)
    i = min(int(f), len(sec) - 2)
    (y0, r0), (y1, r1) = sec[i], sec[i + 1]
    k = f - i
    dy, dr = y0 + (y1 - y0) * k, r0 + (r1 - r0) * k
    tan = Vector((y1 - y0, r1 - r0)).normalized()
    ny, nr = -tan.y, tan.x                      # section normal (outward = +r on the crown)
    if nr < 0 or (abs(nr) < 1e-6 and dy * ny < 0):
        ny, nr = -ny, -nr
    R = P.WHEEL_RADIUS + P.FENDER_CLEARANCE + inset
    r = R + dr + nr * lift
    cy = sy * (P.TRACK / 2 - 0.01)
    p = Vector((wc.x + sx * r * math.sin(t), cy + dy + ny * lift, wc.z + r * math.cos(t)))
    n = Vector((sx * nr * math.sin(t), ny, nr * math.cos(t))).normalized()
    return p, n


def fender_hardware(corner, steer, ext):
    """Riveted aluminium guide rail on the outboard skirt + brass rivet rows (NASM2020-00054)."""
    sx, sy = P.CORNERS[corner]
    a0, a1 = P.FENDER_FIXED
    out_s = 0.04 if sy < 0 else 0.96           # outboard skirt side of the section
    rail = []
    n_pts = 40
    for i in range(n_pts + 1):
        t = a0 + (a1 - a0) * i / n_pts
        p, n = fender_point(corner, t, out_s, lift=0.003)
        rail.append((p, n))
    v, f = [], []
    for (p0, n0), (p1, n1) in zip(rail, rail[1:]):
        w = (p1 - p0).cross(n0).normalized() * 0.011
        base = len(v)
        for q in (p0 - w, p0 + w, p1 + w, p1 - w):
            v.append(q)
        for q in (p0 - w, p0 + w, p1 + w, p1 - w):
            v.append(q - n0 * 0.0015)
        f.extend([(base, base + 1, base + 2, base + 3), (base + 7, base + 6, base + 5, base + 4),
                  (base, base + 4, base + 5, base + 1), (base + 3, base + 2, base + 6, base + 7)])
    add_mesh(f"{corner}_fender_rail", v, f, M.get("aluminium"), steer)
    v, f = [], []
    for p, n in rail[1:-1:3]:
        rivet_geo(p + n * 0.0002, n, 0.0035, 0.002, v, f, sides=8)
    add_mesh(f"{corner}_fender_rail_rivets", v, f, M.get("brass"), steer, smooth=True)

    # extension: rivet row along its overlap edge and down both skirts
    e0, e1 = P.FENDER_EXT
    v, f = [], []
    for k in range(13):
        p, n = fender_point(corner, e0 + 0.06, k / 12, inset=0.008, lift=0.002)
        rivet_geo(p, n, 0.0045, 0.0025, v, f, sides=8)
    for s in (0.02, 0.98):
        for k in range(1, 6):
            p, n = fender_point(corner, e0 + 0.06 + (e1 - e0 - 0.12) * k / 6, s, inset=0.008, lift=0.002)
            rivet_geo(p, n, 0.0045, 0.0025, v, f, sides=8)
    add_mesh(f"{corner}_fender_ext_rivets", v, f, M.get("brass"), ext, smooth=True)


def fender_flap(corner, steer):
    """Light blue-grey flap hanging from the aft end of a front fixed fender (Smithsonian vehicle only,
    NASM2020-00048 / 00049): a flat plate continuing the fender surface downward, full fender width."""
    t0 = P.FENDER_FIXED[0]
    drop, thick = P.FLAP_SIZE
    v, f = [], []
    ns = 8
    for i in range(ns + 1):
        p, n = fender_point(corner, t0, i / ns, lift=0.001)
        p2, _ = fender_point(corner, t0 + 0.02 * (1 if P.CORNERS[corner][0] < 0 else -1), i / ns, lift=0.001)
        down = (p2 - p).normalized()
        if down.z > 0:
            down = -down
        for k, off in enumerate((0.0, -thick)):
            v += [p + n * off, p + down * drop + n * off]
    for i in range(ns):
        a, b = 4 * i, 4 * (i + 1)
        f += [(a, a + 1, b + 1, b), (a + 2, b + 2, b + 3, a + 3), (a, b, b + 2, a + 2), (a + 1, a + 3, b + 3, b + 1)]
    f += [(0, 2, 3, 1), (4 * ns, 4 * ns + 1, 4 * ns + 3, 4 * ns + 2)]
    add_mesh(f"{corner}_fender_flap_nasm", v, f, M.get("flap_grey"), steer, variant="Mission:nasm")


def flag_decal(corner, parent, centre=math.radians(63), height=0.159, width=0.245, inboard=0.025, variant=None):
    """US flag sticker on the outer (forward or aft) face of a fender extension (as17-146-22367).

    Size, white border and inboard offset measured on the Smithsonian close-up NASM2020-00054 (the sticker
    covers ~45% of the extension face along the arc and ~52% across). Canton on the viewer's left: -Y seen
    from the front, +Y seen from behind."""
    sx, sy = P.CORNERS[corner]
    wc = Vector(P.wheel_center(corner))
    R = P.WHEEL_RADIUS + P.FENDER_CLEARANCE + 0.008 + 0.0032
    cf = sy * (P.TRACK / 2 - 0.01)          # fender centre line (crown reference)
    cy = cf - sy * inboard                   # sticker centre, shifted toward the chassis
    da = height / R
    nu, nv = 12, 8
    verts, faces, uvs = [], [], []
    for i in range(nv + 1):
        t = centre + da / 2 - da * i / nv          # i=0 at the bottom (larger angle)
        for j in range(nu + 1):
            u = j / nu
            y = cy + sx * (width * u - width / 2)  # hoist side on the viewer's left
            r = R + fender_crown(y - cf, P.FENDER_WIDTH / 2 + 0.008 * 0.6)
            verts.append(Vector((wc.x + sx * r * math.sin(t), y, wc.z + r * math.cos(t))))
            uvs.append((u, i / nv))
    for i in range(nv):
        for j in range(nu):
            a = i * (nu + 1) + j
            q = (a, a + 1, a + nu + 2, a + nu + 1)
            faces.append(q)
    add_mesh(f"{corner}_flag_decal", verts, faces, M.get("flag_decal"), parent, uvs=uvs, smooth=True, variant=variant)


# ---------------------------------------------------------------- main

def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    out = argv[argv.index("--out") + 1] if "--out" in argv else "build/lrv.blend"
    reset()
    ch = build_chassis()
    sec = lambda c: SECTIONS["fwd" if P.CORNERS[c][0] > 0 else "aft"]  # noqa: E731
    arm_ends = {c: build_suspension(c, sec(c)) for c in P.CORNERS}
    build_steering_linkage(arm_ends)
    build_fenders()
    # component modules: each exposes build(chassis_link); built if present
    import importlib
    for mod in P.COMPONENT_MODULES:
        try:
            m = importlib.import_module(mod)
        except ModuleNotFoundError as err:
            if err.name != mod:
                raise
            continue
        m.build(ch)
        print(f"built component module {mod}")
    assign_to_sections(ch)
    for ob in bpy.data.objects:
        if ob.type == "MESH":
            ensure_uvs(ob)
    print(f"textured {M.upgrade_all()} plain materials")
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    bpy.ops.wm.save_as_mainfile(filepath=os.path.abspath(out))
    print(f"saved {out}: {len(bpy.data.objects)} objects")


if __name__ == "__main__":
    main()
