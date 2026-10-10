"""Compose the simulation-ready LRV USD: visuals + rigid bodies + joints + drives.

Run: .venv/bin/python scripts/make_usd.py [--build build] [--out build/usd/lrv.usda]

Inputs come from export_visuals.py (visuals.usdc, links.json). Units: metres, kg, Z-up.
USD angular quantities are in degrees (limits, targets, and drive gains are per degree).
"""
import argparse
import json
import math
import os
import shutil
import sys

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P  # noqa: E402

ROOT = "/LRV"
DEG = 180.0 / math.pi


MASSES = {}


def resolve_masses(links):
    """Explicit link masses, else params.MASS by suffix; the chassis takes the remainder of MASS_TOTAL."""
    for name, L in links.items():
        if name == "chassis":
            continue
        if L.get("mass") is not None:
            MASSES[name] = L["mass"]
            continue
        for key, m in P.MASS.items():
            if name == key or name.endswith("_" + key):
                MASSES[name] = m
                break
        else:
            raise KeyError(f"no mass for link {name}")
    # payload links (LCRU, TV, antennas, tools...) are on top of the 209 kg vehicle
    vehicle = sum(v for k, v in MASSES.items() if not links[k]["joint"].get("payload"))
    MASSES["chassis"] = P.MASS_TOTAL - vehicle
    assert MASSES["chassis"] > 0, MASSES


def mass_of(link):
    return MASSES[link]


def axis_rotation(axis):
    """Joint frames use X as the joint axis; rotate X onto `axis`."""
    q = Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.Vec3d(*axis)).GetQuat()
    return Gf.Quatf(q)


def retarget(layer, old, new):
    """Rewrite relationship targets and attribute connections that point under `old`."""
    old, new = Sdf.Path(old), Sdf.Path(new)

    def fix(path):
        return path.ReplacePrefix(old, new) if path.HasPrefix(old) else path

    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.RelationshipSpec):
            items = list(spec.targetPathList.explicitItems)
            if items:
                spec.targetPathList.explicitItems = [fix(p) for p in items]
        elif isinstance(spec, Sdf.AttributeSpec):
            items = list(spec.connectionPathList.explicitItems)
            if items:
                spec.connectionPathList.explicitItems = [fix(p) for p in items]

    layer.Traverse(Sdf.Path.absoluteRootPath, visit)


def chassis_boxes():
    """World-space (centre, size) contact boxes for the three chassis sections."""
    z0, z1 = P.wl(100.0), P.wl(100.0 + P.FRAME_H_IN)
    zc, h = (z0 + z1) / 2, z1 - z0
    xf, xa = P.sta(P.STA_FWD_HINGE), P.sta(P.STA_AFT_HINGE)
    out = {"chassis": ((xf + xa) / 2, P.bl(P.CENTER_WIDTH_IN), xf - xa)}
    for name, xh, xe in (("chassis_fwd", xf, P.sta(P.STA_FWD_END)), ("chassis_aft", xa, P.sta(P.STA_AFT_END))):
        out[name] = ((xh + xe) / 2, 2 * P.bl(P.END_SIDE_BL_IN + P.FRAME_W_IN / 2), abs(xe - xh))
    return {k: (Gf.Vec3d(x, 0, zc), Gf.Vec3f(lx, w, h)) for k, (x, w, lx) in out.items()}


def add_collision(stage, link, path, origin):
    """Simple contact geometry: wheels are cylinders, chassis sections boxes. Others don't collide."""
    boxes = chassis_boxes()
    if link.endswith("_wheel"):
        c = UsdGeom.Cylinder.Define(stage, f"{path}/collision")
        c.CreateAxisAttr("Y")
        c.CreateRadiusAttr(P.WHEEL_RADIUS)
        c.CreateHeightAttr(P.WHEEL_WIDTH)
        prim = c.GetPrim()
    elif link in boxes:
        centre, size = boxes[link]
        c = UsdGeom.Cube.Define(stage, f"{path}/collision")
        c.CreateSizeAttr(1.0)
        x = UsdGeom.Xformable(c)
        x.AddTranslateOp().Set(centre - origin)
        x.AddScaleOp().Set(size)
        prim = c.GetPrim()
    else:
        return
    UsdGeom.Imageable(prim).CreatePurposeAttr(UsdGeom.Tokens.guide)
    UsdPhysics.CollisionAPI.Apply(prim)


def set_mass(stage, link, path):
    """Mass from params; CoM and inertia from the visual bounding box as a solid box."""
    prim = stage.GetPrimAtPath(path)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    vis = stage.GetPrimAtPath(f"{path}/visual")
    box = cache.ComputeRelativeBound(vis, prim).ComputeAlignedRange()
    m = mass_of(link)
    api = UsdPhysics.MassAPI.Apply(prim)
    api.CreateMassAttr(m)
    if box.IsEmpty():
        return
    size = box.GetSize()
    com = Gf.Vec3f(box.GetMidpoint())
    api.CreateCenterOfMassAttr(com)
    sx, sy, sz = (max(s, 0.02) for s in size)
    api.CreateDiagonalInertiaAttr(Gf.Vec3f(m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz),
                                           m / 12 * (sx * sx + sy * sy)))
    api.CreatePrincipalAxesAttr(Gf.Quatf(1, 0, 0, 0))


def add_joint(stage, name, spec, parent_path, child_path, local0, local1):
    jt = spec["type"]
    path = f"{ROOT}/joints/{name}"
    if jt == "fixed":
        j = UsdPhysics.FixedJoint.Define(stage, path)
    elif jt == "revolute":
        j = UsdPhysics.RevoluteJoint.Define(stage, path)
    elif jt == "prismatic":
        j = UsdPhysics.PrismaticJoint.Define(stage, path)
    elif jt == "spherical":   # tie-rod ball studs: free, no drive or limits
        j = UsdPhysics.SphericalJoint.Define(stage, path)
    else:
        raise ValueError(jt)
    j.CreateBody0Rel().SetTargets([parent_path])
    j.CreateBody1Rel().SetTargets([child_path])
    j.CreateLocalPos0Attr(Gf.Vec3f(*local0))
    j.CreateLocalPos1Attr(Gf.Vec3f(*local1))
    rot = axis_rotation(spec.get("axis", (1, 0, 0)))
    j.CreateLocalRot0Attr(rot)
    j.CreateLocalRot1Attr(rot)
    if jt in ("fixed", "spherical"):
        return j
    j.CreateAxisAttr("X")
    angular = jt == "revolute"
    scale = DEG if angular else 1.0
    if "limit" in spec:
        lo, hi = spec["limit"]
        j.CreateLowerLimitAttr(lo * scale)
        j.CreateUpperLimitAttr(hi * scale)
    d = spec.get("drive")
    if d:
        drive = UsdPhysics.DriveAPI.Apply(j.GetPrim(), "angular" if angular else "linear")
        drive.CreateTypeAttr("force")
        gain = 1.0 / scale  # USD gains are per degree for angular drives
        drive.CreateStiffnessAttr(d.get("stiffness", 0.0) * gain)
        drive.CreateDampingAttr(d.get("damping", 0.0) * gain)
        if "max_force" in d:
            drive.CreateMaxForceAttr(d["max_force"])
        drive.CreateTargetPositionAttr(d.get("target", 0.0) * scale)
        drive.CreateTargetVelocityAttr(0.0)
    if spec.get("role"):
        j.GetPrim().SetCustomDataByKey("lrv:role", spec["role"])
    return j


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build", default="build")
    ap.add_argument("--out", default="build/usd/lrv.usda")
    a = ap.parse_args()

    graph = json.load(open(os.path.join(a.build, "links.json")))
    links, loops = graph["links"], graph["loops"]
    resolve_masses(links)
    vis_layer = Sdf.Layer.FindOrOpen(os.path.join(a.build, "visuals.usdc"))

    os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
    # textures exported by Blender sit next to visuals.usdc; keep relative paths valid
    tex_src = os.path.join(a.build, "textures")
    if os.path.isdir(tex_src):
        shutil.copytree(tex_src, os.path.join(os.path.dirname(a.out), "textures"), dirs_exist_ok=True)

    stage = Usd.Stage.CreateNew(a.out)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    root = UsdGeom.Xform.Define(stage, ROOT)
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetMetadata("kind", "component")
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())

    scene = UsdPhysics.Scene.Define(stage, "/physicsScene")
    scene.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    scene.CreateGravityMagnitudeAttr(P.G_MOON)

    layer = stage.GetRootLayer()
    stage.DefinePrim(f"{ROOT}/Looks", "Scope")
    for child in vis_layer.GetPrimAtPath("/visuals/_materials").nameChildren:
        Sdf.CopySpec(vis_layer, child.path, layer, f"{ROOT}/Looks/{child.name}")

    UsdGeom.Scope.Define(stage, f"{ROOT}/joints")
    for name, L in links.items():
        path = f"{ROOT}/{name}"
        x = UsdGeom.Xform.Define(stage, path)
        x.AddTranslateOp().Set(Gf.Vec3d(*L["origin"]))
        UsdPhysics.RigidBodyAPI.Apply(x.GetPrim())
        Sdf.CopySpec(vis_layer, f"/visuals/L_{name}", layer, f"{path}/visual")
        add_collision(stage, name, path, Gf.Vec3d(*L["origin"]))

    retarget(layer, "/visuals/_materials", f"{ROOT}/Looks")
    for name in links:
        set_mass(stage, name, f"{ROOT}/{name}")

    for name, L in links.items():
        if L["parent"] is None:
            continue
        po = Gf.Vec3d(*links[L["parent"]]["origin"])
        co = Gf.Vec3d(*L["origin"])
        add_joint(stage, name, L["joint"], f"{ROOT}/{L['parent']}", f"{ROOT}/{name}", co - po, (0, 0, 0))

    for lp in loops:
        po = Gf.Vec3d(*links[lp["parent"]]["origin"])
        co = Gf.Vec3d(*links[lp["child"]]["origin"])
        anchor = Gf.Vec3d(*lp["anchor"])
        j = add_joint(stage, lp["name"], {"type": lp["type"], "axis": lp["axis"]},
                      f"{ROOT}/{lp['parent']}", f"{ROOT}/{lp['child']}", anchor - po, anchor - co)
        j.CreateExcludeFromArticulationAttr(True)

    stage.GetRootLayer().Save()
    n_j = len([p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)])
    print(f"wrote {a.out}: {len(links)} bodies, {n_j} joints ({len(loops)} loop)")


if __name__ == "__main__":
    main()
