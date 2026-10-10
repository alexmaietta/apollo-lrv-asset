"""Electrical harnesses: battery/DCE trunks along the frame and the drive/steering cables to each wheel.

Cables are split per link so they move with the suspension and steering; each segment ends at
a joint. Routing follows refs/images/7020254.jpg (cables down the upper arm to the traction
drive) and the QTU walkaround (bundles along the inboard tops of the side members).
"""
import math

import bpy
from mathutils import Vector

import materials as M
import params as P
from geo import box_geo, add_mesh, pipe


def _link(name):
    return bpy.data.objects[f"link:{name}"]


def _mats():
    return (M.define("harness_jacket", (0.72, 0.71, 0.66), 0.0, 0.6),
            M.define("harness_clamp", (0.03, 0.03, 0.03), 0.2, 0.45))


def _clamps(points, every, parent, name, mat, r):
    """Black band clamps spaced along a polyline."""
    v, f = [], []
    acc = 0.0
    for a, b in zip(points, points[1:]):
        a, b = Vector(a), Vector(b)
        seg = (b - a).length
        while acc < seg:
            p = a.lerp(b, acc / seg)
            box_geo(p, (0.012, r * 2.6, r * 2.6), v, f)
            acc += every
        acc -= seg
    if v:
        add_mesh(name, v, f, mat, parent)


def build(ch):
    jacket, clamp = _mats()
    z_top = P.wl(100.0 + P.FRAME_H_IN) + 0.012
    yo = P.bl(P.CENTER_WIDTH_IN / 2) - P.FRAME_W_IN * P.IN - 0.02
    xf, xa = P.sta(P.STA_FWD_HINGE), P.sta(P.STA_AFT_HINGE)
    fwd, aft = _link("chassis_fwd"), _link("chassis_aft")
    ys = P.bl(P.END_SIDE_BL_IN) - 0.03

    for s in (-1, 1):
        side = "l" if s > 0 else "r"
        # forward electronics bay -> hinge (rides on the forward section)
        run_f = [(P.sta(30.0), s * 0.12, z_top), (P.sta(36.0), s * ys, z_top), (xf + 0.02, s * ys, z_top)]
        pipe(f"harness_fwd_{side}", run_f, 0.011, jacket, fwd)
        # centre section trunk along the inboard top of the side member (hinge to hinge)
        run_c = [(xf - 0.01, s * ys, z_top), (xf - 0.08, s * yo, z_top), (xa + 0.08, s * yo, z_top),
                 (xa + 0.01, s * ys, z_top)]
        pipe(f"harness_ctr_{side}", run_c, 0.011, jacket, ch)
        _clamps(run_c, 0.28, ch, f"harness_ctr_{side}_clamps", clamp, 0.011)
        run_a = [(xa - 0.02, s * ys, z_top), (P.sta(112.0), s * ys, z_top)]
        pipe(f"harness_aft_{side}", run_a, 0.011, jacket, aft)

    # per wheel: section -> upper arm pivot, along the arm's leading tube, kingpin -> traction drive
    for c, (sx, sy) in P.CORNERS.items():
        sec = fwd if sx > 0 else aft
        wc = Vector(P.wheel_center(c))
        up = Vector((wc.x + sx * P.ARM_BASE * 0.42, sy * (P.UPPER_PIVOT_Y + 0.01), wc.z + P.UPPER_PIVOT_DZ + 0.02))
        kp_hi = Vector((wc.x + sx * 0.02, sy * (P.KINGPIN_Y - 0.025), wc.z + P.KNUCKLE_UPPER_DZ + 0.02))
        start = Vector((wc.x + sx * P.ARM_BASE * 0.5, sy * ys, z_top))
        pipe(f"{c}_drive_cable_sec", [start, start + Vector((0, sy * 0.02, -0.01)), up], 0.008, jacket, sec)
        pipe(f"{c}_drive_cable_arm", [up, up.lerp(kp_hi, 0.5) + Vector((0, 0, 0.012)), kp_hi], 0.008, jacket,
             _link(f"{c}_upper_arm"))
        drive = Vector((wc.x, sy * (P.KINGPIN_Y + 0.03), wc.z + 0.07))
        pipe(f"{c}_drive_cable_knuckle", [kp_hi, kp_hi + Vector((0, sy * 0.02, -0.02)), drive], 0.008, jacket,
             _link(f"{c}_steer"))
