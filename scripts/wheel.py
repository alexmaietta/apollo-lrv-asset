"""LRV wire-mesh wheel: woven tire, segmented titanium chevrons, outer disc, bump-stop cage.

Built in a wheel-local frame (Y = axle, +Y outboard, Z up at rest) and mapped to world
by `xform`. Right-side wheels use a Y-mirror, so face winding is flipped there.
"""
import bisect
import math

from mathutils import Matrix, Vector

import materials as M
import params as P
from geo import add_mesh, revolve_geo, rivet_geo, tube_geo

TAU = 2 * math.pi


class TireProfile:
    """Superellipse half-section from the inboard rim, over the crown, to the outboard rim."""

    def __init__(self, steps=64):
        r_rim, R, hw, n = P.TIRE_RIM_RADIUS, P.WHEEL_RADIUS, P.WHEEL_WIDTH / 2, P.TIRE_SHAPE_N
        self.pts = []
        for i in range(steps + 1):
            t = -math.pi / 2 + math.pi * i / steps
            s, c = math.sin(t), math.cos(t)
            self.pts.append((r_rim + (R - r_rim) * abs(c) ** (2 / n), hw * math.copysign(abs(s) ** (2 / n), s)))
        self.arc = [0.0]
        for (r0, a0), (r1, a1) in zip(self.pts, self.pts[1:]):
            self.arc.append(self.arc[-1] + math.hypot(r1 - r0, a1 - a0))
        self.length = self.arc[-1]

    def v_at_radius(self, r, side):
        """Arc length where the profile radius falls to r, on the -1 (inboard) or +1 side."""
        idx = range(len(self.pts)) if side < 0 else range(len(self.pts) - 1, -1, -1)
        for i in idx:
            if self.pts[i][0] >= r:
                return self.arc[i]
        return self.length / 2

    def at(self, v):
        """(r, a, outward normal (nr, na)) at arc length v."""
        v = min(max(v, 0.0), self.length)
        i = min(max(bisect.bisect_right(self.arc, v) - 1, 0), len(self.pts) - 2)
        (r0, a0), (r1, a1) = self.pts[i], self.pts[i + 1]
        f = (v - self.arc[i]) / max(self.arc[i + 1] - self.arc[i], 1e-9)
        dr, da = r1 - r0, a1 - a0
        ln = math.hypot(dr, da)
        return r0 + f * dr, a0 + f * da, (da / ln, -dr / ln)


def _surface_point(prof, v, s, lift):
    """Point on the tire at profile arc length v, circumferential distance s (metres at the crown)."""
    r, a, (nr, na) = prof.at(v)
    t = s / P.WHEEL_RADIUS
    rr = r + nr * lift
    return Vector((rr * math.sin(t), a + na * lift, rr * math.cos(t)))


def _chevron(prof, k, xform, flip, verts, faces, rivets):
    """One V chevron: two legs from the tread centre line out toward each rim."""
    L, mid = prof.length, prof.length / 2
    w = P.CHEVRON_STRIP / 2
    tan_t, cos_t = math.tan(P.CHEVRON_ANGLE), math.cos(P.CHEVRON_ANGLE)
    s0 = TAU * P.WHEEL_RADIUS * k / P.CHEVRON_COUNT
    steps = 14
    for side in (-1, 1):
        v_end = prof.v_at_radius(P.CHEVRON_END_R, side)
        base = len(verts)
        for i in range(steps + 1):
            v = mid + (v_end - mid) * i / steps
            d = abs(v - mid)
            sc = s0 + d * tan_t
            for ds in (-w / cos_t, w / cos_t):
                for lift in (0.0006, 0.0022):
                    verts.append(xform @ _surface_point(prof, v, sc + ds, lift))
        for i in range(steps):
            o, p = base + 4 * i, base + 4 * (i + 1)
            quads = [(o + 1, o + 3, p + 3, p + 1), (o + 0, p + 0, p + 2, o + 2),
                     (o + 0, o + 1, p + 1, p + 0), (o + 2, p + 2, p + 3, o + 3)]
            if side < 0:
                quads = [q[::-1] for q in quads]
            faces.extend(q[::-1] if flip else q for q in quads)
        for o, end in ((base, 0), (base + 4 * steps, 1)):
            q = (o, o + 2, o + 3, o + 1)
            if (end == 1) != (side < 0):
                q = q[::-1]
            faces.append(q[::-1] if flip else q)
        # riveted joints between strip segments, apex rivet shared by both legs
        for j in range(0 if side > 0 else 1, P.CHEVRON_RIVETS + 1):
            v = mid + (v_end - mid) * j / P.CHEVRON_RIVETS * 0.92
            sc = s0 + abs(v - mid) * tan_t
            p0 = xform @ _surface_point(prof, v, sc, 0.0020)
            p1 = xform @ _surface_point(prof, v, sc, 0.0040)
            rivet_geo(p0, p1 - p0, 0.0055, 0.0028, *rivets)


def _bar_geo(p0, p1, wdir, w, t, ndir, verts, faces):
    """Flat bar from p0 to p1, width along wdir, thickness along ndir."""
    base = len(verts)
    for p in (p0, p1):
        for sw, sn in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            verts.append(p + wdir * (sw * w / 2) + ndir * (sn * t / 2))
    for i in range(4):
        j = (i + 1) % 4
        faces.append((base + i, base + j, base + 4 + j, base + 4 + i))
    faces.append((base + 3, base + 2, base + 1, base + 0))
    faces.append((base + 4, base + 5, base + 6, base + 7))


def _profile_point(profile, r, t, side):
    """Point at radius r on a revolve profile (r, axial) and its normal toward the `side` (+1/-1) axial face."""
    for (r0, a0), (r1, a1) in zip(profile, profile[1:]):
        if min(r0, r1) <= r <= max(r0, r1) and r0 != r1:
            k = (r - r0) / (r1 - r0)
            a = a0 + (a1 - a0) * k
            nr, na = (a1 - a0), -(r1 - r0)
            if na * side < 0:
                nr, na = -nr, -na
            d = Vector((math.sin(t), 0, math.cos(t)))
            ln = math.hypot(nr, na)
            return d * r + Vector((0, a, 0)), d * (nr / ln) + Vector((0, na / ln, 0))
    raise ValueError(r)


def build_wheel(corner, parent):
    sx, sy = P.CORNERS[corner]
    c = Vector(P.wheel_center(corner))
    spin = Matrix.Rotation(P.WHEEL_REST_SPIN[corner], 4, "Y")   # resting rotation, so the disc bands don't all align
    xform = Matrix.Translation(c) @ Matrix.Diagonal((1, sy, 1, 1)) @ spin
    flip = sy < 0
    hw = P.WHEEL_WIDTH / 2
    prof = TireProfile()

    # woven tire carcass: UV u along the circumference, v along the profile (metres)
    v, f, uv = revolve_geo(prof.pts, 128, xform, flip, u_scale=P.WHEEL_RADIUS)
    add_mesh(f"{corner}_tire_mesh", v, f, M.get("wire_mesh"), parent, uvs=uv, smooth=True)

    v, f = [], []
    rv, rf = [], []
    for k in range(P.CHEVRON_COUNT):
        _chevron(prof, k, xform, flip, v, f, (rv, rf))
    add_mesh(f"{corner}_chevrons", v, f, M.get("titanium"), parent)
    add_mesh(f"{corner}_chevron_rivets", rv, rf, M.get("titanium"), parent, smooth=True)

    # outer disc: satin spun aluminium, riveted flange, recessed slightly inside the rim (7020255.jpg)
    ad = hw - 0.018
    rr = P.TIRE_RIM_RADIUS
    R_d = P.HUB_RADIUS
    disc = [(0.0, ad), (R_d - 0.016, ad), (R_d - 0.012, ad + 0.004), (R_d - 0.004, ad + 0.004),
            (R_d, ad), (R_d + 0.004, ad - 0.004), (rr - 0.004, hw - 0.006), (rr, hw)]
    v, f, uv = revolve_geo(disc, 128, xform, flip)   # UV u runs around: spun-machining rings
    add_mesh(f"{corner}_disc_outer", v, f, M.get("aluminium_disc"), parent, uvs=uv, smooth=True)
    nrm = (xform.to_3x3() @ Vector((0, 1, 0))).normalized()

    # painted index band: white band across the disc, black stripe down its middle
    v, f = [], []
    half = R_d - 0.018
    _bar_geo(xform @ Vector((0, ad + 0.0006, half)), xform @ Vector((0, ad + 0.0006, -half)),
             Vector((1, 0, 0)), P.DISC_BAND_W, 0.0006, nrm, v, f)
    add_mesh(f"{corner}_disc_band", v, f, M.get("cream_paint"), parent)
    v, f = [], []
    for zs in (1, -1):
        p0 = xform @ Vector((0, ad + 0.0012, zs * (P.HUB_DECOUPLE_R + 0.008)))
        p1 = xform @ Vector((0, ad + 0.0012, zs * half))
        _bar_geo(p0, p1, Vector((1, 0, 0)), P.DISC_STRIPE_W, 0.0006, nrm, v, f)
    add_mesh(f"{corner}_disc_stripe", v, f, M.get("black_paint"), parent)

    # rivets: flange ring + inner ring
    v, f = [], []
    for ring_r, n in ((R_d - 0.009, 44), (R_d - 0.032, 24)):
        for i in range(n):
            t = TAU * (i + 0.5 * (n == 24)) / n
            d = Vector((math.sin(t), 0, math.cos(t)))
            h = ad + (0.004 if n == 44 else 0.0)
            rivet_geo(xform @ (d * ring_r + Vector((0, h, 0))), nrm, 0.0028, 0.0016, v, f, sides=8)
    add_mesh(f"{corner}_disc_rivets", v, f, M.get("aluminium"), parent, smooth=True)

    # decoupling mechanism (A17 Systems Handbook Drawing 4.2): scalloped decoupling plate bolted to the
    # hub (8 bolts), two radial pins through clevises at 12 and 6 o'clock, each held by a helical
    # torsion spring; pull the spring out and rotate it about the wheel centre line to decouple.
    plate = [(0.0, ad + 0.004), (P.HUB_DECOUPLE_R - 0.004, ad + 0.004), (P.HUB_DECOUPLE_R, ad), (P.HUB_DECOUPLE_R + 0.002, ad)]
    v, f, _ = revolve_geo(plate, 64, xform, flip)
    for xs_ in (1, -1):
        c0 = Vector((xs_ * (P.HUB_DECOUPLE_R - 0.006), ad + 0.004, 0))
        _bar_geo(xform @ (c0 - Vector((0, 0, 0.012))), xform @ (c0 + Vector((0, 0, 0.012))),
                 Vector((1, 0, 0)), 0.01, 0.014, nrm, v, f)
    for zs in (1, -1):                       # clevis blocks the pins run through
        c0 = Vector((0, ad + 0.009, zs * (P.HUB_DECOUPLE_R - 0.010)))
        _bar_geo(xform @ (c0 - Vector((0.009, 0, 0))), xform @ (c0 + Vector((0.009, 0, 0))),
                 Vector((0, 0, 1)), 0.012, 0.010, nrm, v, f)
    add_mesh(f"{corner}_decoupler", v, f, M.get("drive_housing"), parent, smooth=True)
    v, f = [], []
    for k in range(8):                       # plate bolts, in pairs between the scallops
        t = math.radians((35, 75, 105, 145, 215, 255, 285, 325)[k])
        d = Vector((math.sin(t), 0, math.cos(t)))
        tube_geo(xform @ (d * (P.HUB_DECOUPLE_R - 0.011) + Vector((0, ad + 0.004, 0))),
                 xform @ (d * (P.HUB_DECOUPLE_R - 0.011) + Vector((0, ad + 0.0085, 0))), 0.0042, 6, True, v, f)
    for zs in (1, -1):
        # pin: radial rod through the clevis, ring pull at the outer end
        tube_geo(xform @ Vector((0, ad + 0.009, zs * 0.046)), xform @ Vector((0, ad + 0.009, zs * (P.HUB_DECOUPLE_R + 0.016))),
                 0.0024, 8, True, v, f)
        # helical torsion spring beside the pin, axis tangential
        c = Vector((0.010, ad + 0.012, zs * 0.054))
        coil = []
        for i in range(5 * 12 + 1):
            t = TAU * i / 12
            coil.append(c + Vector((-0.012 + 0.024 * i / 60, 0.005 * math.cos(t), 0.005 * math.sin(t))))
        for p0, p1 in zip(coil, coil[1:]):
            tube_geo(xform @ p0, xform @ p1, 0.0011, 5, False, v, f)
    add_mesh(f"{corner}_decoupler_pins", v, f, M.get("steel"), parent, smooth=True)
    cap = [(0.0, ad + 0.034), (0.03, ad + 0.033), (0.042, ad + 0.026), (P.HUB_CAP_RADIUS, ad + 0.016),
           (P.HUB_CAP_RADIUS + 0.003, ad + 0.004)]
    v, f, uv = revolve_geo(cap, 64, xform, flip)
    add_mesh(f"{corner}_hubcap", v, f, M.get("white_paint"), parent, uvs=uv, smooth=True)
    v, f = [], []
    for i in range(6):
        t = TAU * i / 6
        d = Vector((math.sin(t), 0, math.cos(t)))
        p = d * (P.HUB_CAP_RADIUS - 0.009) + Vector((0, ad + 0.0175, 0))
        rivet_geo(xform @ p, nrm, 0.0035, 0.0018, v, f, sides=8)
    add_mesh(f"{corner}_hubcap_screws", v, f, M.get("steel"), parent, smooth=True)

    # inboard disc, bare aluminium: a riveted conical dish running in to the wheel-hub interface
    # flange at the wheel centre plane, where the traction drive bolts on (Dwg 4.4; 7020254.jpg)
    hub_r = P.DRIVE_HUB_R
    inner = [(rr, -hw), (rr - 0.004, -hw + 0.006), (0.16, -hw + 0.022), (0.112, -0.05), (hub_r + 0.004, -0.016),
             (hub_r + 0.002, -0.006)]
    v, f, uv = revolve_geo(inner, 64, xform, flip)
    add_mesh(f"{corner}_disc_inner", v, f, M.get("aluminium_spun"), parent, uvs=uv, smooth=True)
    v, f = [], []
    for ring_r, n in ((rr - 0.026, 40), (0.135, 24)):
        for i in range(n):
            t = TAU * i / n
            p, nn = _profile_point(inner, ring_r, t, -1)
            rivet_geo(xform @ p, (xform.to_3x3() @ nn).normalized(), 0.0026, 0.0015, v, f, sides=8)
    add_mesh(f"{corner}_disc_inner_rivets", v, f, M.get("aluminium"), parent, smooth=True)
    # rotating hub flange (wheel-hub interface) with its bolt circle, facing inboard
    flange = [(0.052, -0.004), (hub_r + 0.002, -0.004), (hub_r + 0.002, -0.010), (hub_r - 0.004, -0.012), (0.052, -0.012)]
    v, f, _ = revolve_geo(flange, 64, xform, flip)
    for i in range(12):
        t = TAU * (i + 0.5) / 12
        d = Vector((math.sin(t), 0, math.cos(t)))
        tube_geo(xform @ (d * (hub_r - 0.008) + Vector((0, -0.011, 0))),
                 xform @ (d * (hub_r - 0.008) + Vector((0, -0.0145, 0))), 0.0036, 6, True, v, f)
    add_mesh(f"{corner}_hub_flange", v, f, M.get("drive_housing"), parent, smooth=True)

    # titanium inner frame (bump stop): from the side it reads as a slotted annulus —
    # flat hoops at the inner/outer radius joined by wide bars, on both faces, with wide
    # axial bars across the outer hoop (7020255.jpg)
    ri, ro, bw = P.BUMPSTOP_INNER, P.BUMPSTOP_OUTER, P.BUMPSTOP_WIDTH / 2
    v, f = [], []
    for r0, r1 in ((ri - 0.012, ri + 0.006), (ro - 0.006, ro + 0.01)):
        for a in (bw, -bw):
            band = [(r0, a - 0.0015), (r1, a - 0.0015), (r1, a + 0.0015), (r0, a + 0.0015), (r0, a - 0.0015)]
            bv, bf, _ = revolve_geo(band, 128, xform, flip)
            o = len(v)
            v.extend(bv)
            f.extend(tuple(i + o for i in q) for q in bf)
    yax = nrm
    for k in range(P.BUMPSTOP_BARS):
        t = TAU * (k + 0.5) / P.BUMPSTOP_BARS
        d = Vector((math.sin(t), 0, math.cos(t)))
        tang = (xform.to_3x3() @ Vector((math.cos(t), 0, -math.sin(t)))).normalized()
        rad = (xform.to_3x3() @ d).normalized()
        for a in (bw, -bw):
            _bar_geo(xform @ (d * ri + Vector((0, a, 0))), xform @ (d * ro + Vector((0, a, 0))), tang,
                     P.BUMPSTOP_BAR_W, 0.0025, yax, v, f)
        _bar_geo(xform @ (d * (ro + 0.008) + Vector((0, bw, 0))), xform @ (d * (ro + 0.008) + Vector((0, -bw, 0))),
                 tang, P.BUMPSTOP_BAR_W, 0.0025, rad, v, f)
    add_mesh(f"{corner}_bumpstop", v, f, M.get("titanium"), parent)
