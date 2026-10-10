"""Export a URDF (and per-link textured OBJ meshes) from a built .blend.

Run: blender -b build/lrv.blend -P scripts/export_urdf.py -- [--outdir build/urdf]

URDF can't express closed kinematic loops, so each double wishbone is written the way
Isaac Sim's converter writes VIPER's: the lower arm is the sprung joint, and the upper arm,
upright and damper follow it through <mimic> joints. Multipliers come from linearising the
real linkage about the rest pose (exact for small travel; the USD keeps true loop joints).
The steering linkage (sector -> ball-jointed tie rods -> knuckle arms) is closed the same way: the sector is
the actuated joint, each kingpin is a <mimic> of it, and each tie rod becomes a yaw hinge at its sector stud
(URDF has no ball joint) mimicking the sector. Multipliers are the linkage's slope at centre, so the URDF
steers parallel-ish and does not reproduce Ackermann at large angles (inner 50 / outer 22 deg at full lock);
params.steer_linkage gives the exact kingpin angles, and the USD/MJCF keep the true loop.
Pinned configuration joints (stowage hinges, fender extensions, seat folds, dust covers, HGA
fold/telescope) are written fixed in the deployed pose. URDF has no springs, so the torsion-bar
suspension is kinematic here; use the USD for dynamics.
Units m / kg / rad; frame X forward, Y left, Z up.
"""
import json
import math
import os
import sys
import xml.etree.ElementTree as ET

import bpy
from mathutils import Matrix, Vector

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P  # noqa: E402


def export_meshes(outdir, links):
    """One OBJ (+ MTL, copied textures) per link, in link-local coordinates."""
    mesh_dir = os.path.join(outdir, "meshes")
    os.makedirs(mesh_dir, exist_ok=True)
    empties = {o["link"]: o for o in bpy.data.objects if o.type == "EMPTY" and "link" in o}
    for e in empties.values():
        e.parent = None
    for e in empties.values():
        e.location = (0, 0, 0)
    bpy.context.view_layer.update()
    written = {}
    for name, e in empties.items():
        meshes = [c for c in e.children if c.type == "MESH"]
        if not meshes:
            continue
        bpy.ops.object.select_all(action="DESELECT")
        for m in meshes:
            m.select_set(True)
        bpy.context.view_layer.objects.active = meshes[0]
        path = os.path.join(mesh_dir, f"{name}.obj")
        bpy.ops.wm.obj_export(filepath=path, export_selected_objects=True, apply_modifiers=True,
                              export_materials=True, path_mode="COPY", forward_axis="Y", up_axis="Z",
                              export_triangulated_mesh=True, export_normals=True, export_uv=True)
        written[name] = f"meshes/{name}.obj"
    return written


def linkage_mimics(links):
    """Linearised follower ratios for each corner's four-bar and damper, per unit lower-arm angle."""
    out = {}
    l_low = P.KINGPIN_Y - P.LOWER_PIVOT_Y
    l_up = P.KINGPIN_Y - P.UPPER_PIVOT_Y
    k_up = l_low / l_up                                    # equal vertical travel at the upright
    for c, (sx, sy) in P.CORNERS.items():
        up_piv = Vector(links[f"{c}_upper_arm"]["origin"])
        d_chs = Vector(links[f"{c}_damper_cyl"]["origin"])
        rod_o = Vector(links[f"{c}_damper_rod"]["origin"])
        axis = Vector(links[f"{c}_damper_rod"]["joint"]["axis"])
        d_arm = rod_o + axis * 0.0
        # rod clevis on the upper arm: recover it from the loop joint anchor
        for lp in LOOPS:
            if lp["name"] == f"{c}_damper_clevis":
                d_arm = Vector(lp["anchor"])
        eps = 1e-4
        rot = Matrix.Rotation(eps * k_up, 3, "X")
        d_arm2 = up_piv + rot @ (d_arm - up_piv)
        v0, v1 = d_arm - d_chs, d_arm2 - d_chs
        ang0 = math.atan2(v0.z, v0.y)
        ang1 = math.atan2(v1.z, v1.y)
        dcyl = (ang1 - ang0) / eps
        dlen = (v1.length - v0.length) / eps
        out[c] = {"upper_arm": k_up, "upright": -1.0, "damper_cyl": dcyl, "damper_rod": dlen}
    return out


def steer_mimics():
    """Kingpin and tie-rod-yaw followers of each sector, per unit sector angle (central difference at 0)."""
    out, eps = {}, 1e-5
    for c, (sx, _) in P.CORNERS.items():
        lead = "fwd_steer_sector" if sx > 0 else "aft_steer_sector"
        hi, lo = P.steer_linkage(eps, c), P.steer_linkage(-eps, c)
        out[f"{c}_steer"] = (lead, (hi[0] - lo[0]) / (2 * eps))
        out[f"{c}_tie_rod"] = (lead, (hi[1] - lo[1]) / (2 * eps))  # yaw about +Z relative to the sector
    return out


LOOPS = []

# Joints held by pins/latches in normal operation. URDF has no drives, so these would become
# free hinges; they're written as fixed in the deployed pose (the USD keeps them as driven joints).
LOCK_ROLES = {"chassis_fold", "fender_extension", "seat_fold", "dust_cover", "hga_fold", "hga_telescope"}


def build_urdf(links, meshes, mimics, name="lrv"):
    import make_usd as MU                                    # reuse mass resolution
    MU.resolve_masses(links)
    robot = ET.Element("robot", name=name)
    mats = {}
    for lname, L in links.items():
        link = ET.SubElement(robot, "link", name=lname)
        m = MU.mass_of(lname)
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "mass", value=f"{m:.4f}")
        r = 0.15 if lname.startswith("chassis") else 0.05
        i = 0.4 * m * r * r
        ET.SubElement(inertial, "inertia", ixx=f"{i:.6f}", iyy=f"{i:.6f}", izz=f"{i:.6f}", ixy="0", ixz="0", iyz="0")
        if lname in meshes:
            vis = ET.SubElement(link, "visual", name=f"{lname}_visual")
            geo = ET.SubElement(vis, "geometry")
            ET.SubElement(geo, "mesh", filename=meshes[lname])
        col = None
        if lname.endswith("_wheel"):
            col = ET.SubElement(link, "collision", name=f"{lname}_collision")
            ET.SubElement(col, "origin", xyz="0 0 0", rpy=f"{math.pi / 2:.6f} 0 0")
            ET.SubElement(ET.SubElement(col, "geometry"), "cylinder", radius=f"{P.WHEEL_RADIUS}", length=f"{P.WHEEL_WIDTH}")
        elif lname in MU_BOXES:
            centre, size = MU_BOXES[lname]
            o = Vector(L["origin"])
            col = ET.SubElement(link, "collision", name=f"{lname}_collision")
            ET.SubElement(col, "origin", xyz=" ".join(f"{v:.5f}" for v in (Vector(centre) - o)), rpy="0 0 0")
            ET.SubElement(ET.SubElement(col, "geometry"), "box", size=" ".join(f"{v:.5f}" for v in size))

    for lname, L in links.items():
        if L["parent"] is None:
            continue
        spec = L["joint"]
        jt = "fixed" if spec.get("role") in LOCK_ROLES else spec["type"]
        if jt == "spherical":                                # tie-rod ball end -> yaw hinge (see module doc)
            spec = {**spec, "axis": (0.0, 0.0, 1.0), "limit": (-math.pi / 2, math.pi / 2)}
            jt = "revolute"
        utype = {"revolute": "revolute", "prismatic": "prismatic", "fixed": "fixed"}[jt]
        if jt == "revolute" and "limit" not in spec:
            utype = "continuous"
        j = ET.SubElement(robot, "joint", name=lname, type=utype)
        po, co = Vector(links[L["parent"]]["origin"]), Vector(L["origin"])
        ET.SubElement(j, "origin", xyz=" ".join(f"{v:.5f}" for v in (co - po)), rpy="0 0 0")
        ET.SubElement(j, "parent", link=L["parent"])
        ET.SubElement(j, "child", link=lname)
        if jt == "fixed":
            continue
        ET.SubElement(j, "axis", xyz=" ".join(f"{v:.6f}" for v in spec["axis"]))
        d = spec.get("drive") or {}
        if "limit" in spec or utype in ("revolute", "prismatic"):
            lo, hi = spec.get("limit", (-math.pi, math.pi))
            effort = d.get("max_force", 1000.0)
            vel = P.WHEEL_MAX_SPEED * 1.5 if spec.get("role") == "wheel" else 10.0
            ET.SubElement(j, "limit", lower=f"{lo:.6f}", upper=f"{hi:.6f}", effort=f"{effort}", velocity=f"{vel:.3f}")
        elif d.get("max_force"):
            ET.SubElement(j, "limit", effort=f"{d['max_force']}", velocity=f"{P.WHEEL_MAX_SPEED * 1.5:.3f}")
        ET.SubElement(j, "dynamics", damping=f"{d.get('damping', 0.0) if d.get('kind') in ('damper', 'spring') else 0.0}",
                      friction="0")
        # four-bar followers
        corner, _, part = lname.partition("_")
        if lname in STEER_MIMICS:
            lead, k = STEER_MIMICS[lname]
            ET.SubElement(j, "mimic", joint=lead, multiplier=f"{k:.6f}", offset="0")
        elif corner in mimics and part in mimics[corner]:
            ET.SubElement(j, "mimic", joint=f"{corner}_lower_arm", multiplier=f"{mimics[corner][part]:.6f}", offset="0")
    ET.indent(robot)
    return ET.ElementTree(robot)


MU_BOXES = {}
STEER_MIMICS = {}


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    outdir = os.path.abspath(argv[argv.index("--outdir") + 1] if "--outdir" in argv else "build/urdf")
    os.makedirs(outdir, exist_ok=True)
    empties = [o for o in bpy.data.objects if o.type == "EMPTY" and "link" in o]
    links = {e["link"]: {"parent": e.parent["link"] if e.parent else None, "origin": list(e.matrix_world.translation),
                         "joint": json.loads(e["joint"]), "mass": e.get("mass")} for e in empties}
    LOOPS.extend(json.loads(bpy.context.scene.get("loops", "[]")))
    mimics = linkage_mimics(links)
    STEER_MIMICS.update(steer_mimics())
    import make_usd as MU
    from pxr import Gf  # noqa: F401  (make_usd imports pxr; Blender ships it)
    MU_BOXES.update({k: (tuple(c), tuple(s)) for k, (c, s) in MU.chassis_boxes().items()})
    meshes = export_meshes(outdir, links)
    tree = build_urdf(links, meshes, mimics)
    path = os.path.join(outdir, "lrv.urdf")
    tree.write(path, xml_declaration=True, encoding="utf-8")
    print(f"wrote {path}: {len(links)} links, {len(meshes)} meshes, mimic corners {sorted(mimics)}")
    print("mimic ratios:", json.dumps({c: {k: round(v, 4) for k, v in m.items()} for c, m in mimics.items()}))
    print("steering mimics:", json.dumps({k: (lead, round(m, 4)) for k, (lead, m) in STEER_MIMICS.items()}))


if __name__ == "__main__":
    main()
