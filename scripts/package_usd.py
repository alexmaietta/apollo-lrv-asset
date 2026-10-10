"""Package the LRV as an Isaac Sim / SimReady robot asset (Isaac Sim robot asset layout).

Run: .venv/bin/python scripts/package_usd.py [--full build/usd/full] [--proxy build/usd/proxy]
                                             [--out build/asset/lrv]

Inputs come from export_visuals.py (visuals.usdc per LOD, links.json, textures/).

lrv/
  lrv.usda                    interface: variant sets Physics / Fidelity / Mission, SimReady metadata
  payloads/base.usda          link tree under /lrv/Geometry, instanceable visual + collider roots
  payloads/robot.usda         Isaac robot schema (links, joints, sites)
  payloads/geometries.usd     mesh data (crate)
  payloads/instances.usda     instance prototypes: mesh + bound material (+ collider APIs)
  payloads/materials.usda     UsdPreviewSurface materials, textures in payloads/Textures
  payloads/physics_materials.usda
  payloads/Physics/{physics,physx,mujoco}.usda
  payloads/Fidelity/{full,proxy}.usda
  payloads/Mission/{a15,a17}.usda
  .thumbs/256x256/lrv.usda.png
"""
import argparse
import datetime
import json
import math
import os
import re
import shutil
import sys

from pxr import Gf, Kind, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P  # noqa: E402

DEG = 180.0 / math.pi
ROOT = "/lrv"
CREATOR = "lrv pipeline (scripts/package_usd.py)"
VERSION = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "VERSION")).read().strip()  # release version

# Joint roles that are passive (springs, linkage followers, pinned configuration joints)
PASSIVE_ROLES = {"suspension", "suspension_upper", "passive", "damper"}
LOCK_ROLES = {"chassis_fold", "fender_extension", "seat_fold", "dust_cover", "hga_fold", "hga_telescope"}

# link-name pattern -> semantic class label (SemanticsLabelsAPI:class)
LINK_CLASS = [
    (r"_wheel$", "wheel"), (r"_steer$", "steering_knuckle"), (r"(_steer_sector|_tie_rod)$", "steering_linkage"), (r"_(lower|upper)_arm$", "suspension_arm"),
    (r"_upright$", "suspension_upright"), (r"_damper", "damper"), (r"fender_ext$", "fender"),
    (r"^chassis", "chassis"), (r"^seat_back", "seat"), (r"^hc_", "hand_controller"),
    (r"^hga", "antenna"), (r"^lga", "antenna"), (r"^lcru$", "radio"), (r"^tv_", "camera"), (r"^tcu$", "camera"),
    (r"_cover$", "dust_cover"), (r"^pallet$", "pallet"), (r"^(tongs|rake|hammer|scoop|gnomon|extension_handle)",
                                                        "tool"),
    (r"^sep_", "instrument"), (r"^traverse_gravimeter$", "instrument"),
]

# Interface sites (childless Xforms -> IsaacSiteAPI), mainly for the astronaut rig:
# (parent link, name, world position from handbook stations, forward axis, description)
SITES = [  # measured from the crew-station geometry (Drawing 5.1/5.2), world metres at rest
    ("chassis", "seat_h_point_l", (-0.2413, 0.3493, 0.7773), "X", "Commander hip point (suited, PLSS against seat back)"),
    ("chassis", "seat_h_point_r", (-0.2413, -0.3493, 0.7773), "X", "LMP hip point (suited, PLSS against seat back)"),
    ("chassis", "footrest_l", (0.6422, 0.3505, 0.5164), "X", "Commander footrest, centre of top surface"),
    ("chassis", "footrest_r", (0.6422, -0.3505, 0.5164), "X", "LMP footrest, centre of top surface"),
    ("chassis", "toe_hold_l", (0.5271, 0.6706, 0.4979), "X", "Outboard toe hold loop top, left"),
    ("chassis", "toe_hold_r", (0.5271, -0.6706, 0.4979), "X", "Outboard toe hold loop top, right"),
    ("chassis", "handhold_l", (0.2616, 0.1803, 0.9703), "Z", "Inboard handhold upper grip, left"),
    ("chassis", "handhold_r", (0.2616, -0.1803, 0.9703), "Z", "Inboard handhold upper grip, right"),
    ("chassis", "nav_reference", (0.0, 0.0, P.wl(100.0)), "X", "Vehicle reference: mid-wheelbase on the chassis floor"),
    ("hc_throttle", "t_handle_grip", (0.0508, 0.0, 0.9487), "Z", "T-handle crossbar centre (hand target for the rig)"),
]


def sanitize(name):
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return s if s[0].isalpha() or s[0] == "_" else "_" + s


# ---------------------------------------------------------------- layers

def new_stage(path, default_prim=True, sublayers=()):
    if os.path.exists(path):
        os.remove(path)
    stage = Usd.Stage.CreateNew(path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(stage, 1.0)
    layer = stage.GetRootLayer()
    layer.customLayerData = {"creator": CREATOR}
    for s in sublayers:
        layer.subLayerPaths.append(s)
    if default_prim:
        layer.defaultPrim = "lrv"
    return stage


def apply_schemas(prim, names):
    """Prepend API schema tokens without needing the schema registered (codeless Isaac/Newton/Mjc)."""
    ops = prim.GetMetadata("apiSchemas") or Sdf.TokenListOp()
    items = []
    for lst in (ops.explicitItems, ops.prependedItems, ops.addedItems, ops.appendedItems):
        items += [i for i in lst if i not in items]
    for n in names:
        if n not in items:
            items.append(n)
    ops = Sdf.TokenListOp()
    ops.prependedItems = items
    prim.SetMetadata("apiSchemas", ops)


def set_attr(prim, name, vtype, value, uniform=False, custom=False):
    a = prim.CreateAttribute(name, vtype, custom, Sdf.VariabilityUniform if uniform else Sdf.VariabilityVarying)
    a.Set(value)
    return a


def xform_ops(prim, translate=(0, 0, 0), orient=Gf.Quatd(1, 0, 0, 0), scale=(1, 1, 1), double=True):
    x = UsdGeom.Xformable(prim)
    x.ClearXformOpOrder()
    x.AddTranslateOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*translate))
    if double:
        x.AddOrientOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Quatd(orient))
        x.AddScaleOp(UsdGeom.XformOp.PrecisionDouble).Set(Gf.Vec3d(*scale))
    else:
        x.AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(orient))
        x.AddScaleOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Vec3f(*scale))


def retarget(layer, old, new, root=None):
    """Rewrite targets/connections under `old` to `new`, visiting only specs under `root` (default all)."""
    old, new = Sdf.Path(old), Sdf.Path(new)

    def fix(p):
        return p.ReplacePrefix(old, new) if p.HasPrefix(old) else p

    def visit(path):
        spec = layer.GetObjectAtPath(path)
        if isinstance(spec, Sdf.RelationshipSpec):
            items = list(spec.targetPathList.explicitItems) or list(spec.targetPathList.prependedItems)
            if items:
                spec.targetPathList.ClearEdits()
                spec.targetPathList.explicitItems = [fix(p) for p in items]
        elif isinstance(spec, Sdf.AttributeSpec):
            items = list(spec.connectionPathList.explicitItems) or list(spec.connectionPathList.prependedItems)
            if items:
                spec.connectionPathList.ClearEdits()
                spec.connectionPathList.explicitItems = [fix(p) for p in items]

    layer.Traverse(Sdf.Path(root) if root else Sdf.Path.absoluteRootPath, visit)


# ---------------------------------------------------------------- inputs

class Source:
    """Blender export of one LOD: per-link meshes with their local transforms and materials."""

    def __init__(self, d):
        self.dir = d
        self.layer = Sdf.Layer.FindOrOpen(os.path.join(d, "visuals.usdc"))
        self.stage = Usd.Stage.Open(self.layer)
        self.items = {}  # link -> [(geom_name, mesh_path, local matrix, material name)]
        for link_prim in self.stage.GetPrimAtPath("/visuals").GetChildren():
            if not link_prim.GetName().startswith("L_"):
                continue
            link = link_prim.GetName()[2:]
            out = []
            for obj in link_prim.GetChildren():
                mesh = next((c for c in Usd.PrimRange(obj) if c.IsA(UsdGeom.Mesh)), None)
                if mesh is None:
                    continue
                m = UsdGeom.Xformable(obj).GetLocalTransformation()
                if mesh != obj:
                    m = UsdGeom.Xformable(mesh).GetLocalTransformation() * m
                tgt = UsdShade.MaterialBindingAPI(mesh).GetDirectBindingRel().GetTargets()
                out.append((sanitize(obj.GetName()), mesh.GetPath(), m, tgt[0].name if tgt else None))
            self.items[link] = out


def link_paths(links):
    paths = {}

    def path(name):
        if name not in paths:
            parent = links[name]["parent"]
            paths[name] = f"{ROOT}/Geometry/{name}" if parent is None else f"{path(parent)}/{name}"
        return paths[name]

    for n in links:
        path(n)
    return paths


def dfs(links, root="chassis"):
    kids = {}
    for n, L in links.items():
        if L["parent"]:
            kids.setdefault(L["parent"], []).append(n)
    order = []

    def walk(n):
        order.append(n)
        for c in sorted(kids.get(n, [])):
            walk(c)

    walk(root)
    return order


# ---------------------------------------------------------------- geometry, instances, materials

def write_geometries(src, path, colliders=None):
    """geometries.usd: /Geometries/<g>/<g> meshes (no materials, no APIs), visual and collider."""
    stage = new_stage(path, default_prim=False)
    layer = stage.GetRootLayer()
    UsdGeom.Scope.Define(stage, "/Geometries")
    for items in src.items.values():
        for g, mesh_path, _m, _mat in items:
            UsdGeom.Xform.Define(stage, f"/Geometries/{g}")
            Sdf.CopySpec(src.layer, mesh_path, layer, Sdf.Path(f"/Geometries/{g}/{g}"))
            spec = layer.GetPrimAtPath(f"/Geometries/{g}/{g}")
            for rel in [r for r in spec.relationships if r.name.startswith("material:binding")]:
                spec.RemoveProperty(rel)
            if "apiSchemas" in spec.ListInfoKeys():
                spec.ClearInfo("apiSchemas")
            prim = stage.GetPrimAtPath(f"/Geometries/{g}/{g}")
            mesh = UsdGeom.Mesh(prim)
            mesh.CreateSubdivisionSchemeAttr().Set(UsdGeom.Tokens.none)
            if _mat in DOUBLE_SIDED_MATERIALS or is_open(mesh):
                mesh.CreateDoubleSidedAttr().Set(True)
    for group in (colliders or {}).values():
        for link_cols in group.values():
            for inst, kind, data, _pmat in link_cols:
                write_collider_geometry(stage, inst, kind, data)
    layer.Save()


DOUBLE_SIDED_MATERIALS = {"wire_mesh", "eq_hga_mesh", "flag_decal"}


def is_open(mesh):
    """True if any edge is used by only one face (a thin shell / strip rather than a closed solid)."""
    counts = mesh.GetFaceVertexCountsAttr().Get()
    idx = mesh.GetFaceVertexIndicesAttr().Get()
    if not counts or not idx:
        return False
    edges = {}
    o = 0
    for c in counts:
        f = idx[o:o + c]
        for i in range(c):
            a, b = f[i], f[(i + 1) % c]
            k = (a, b) if a < b else (b, a)
            edges[k] = edges.get(k, 0) + 1
        o += c
    return any(v == 1 for v in edges.values())


def write_materials(src, path, tex_out, mission_sets):
    """materials.usda: copy Blender's UsdPreviewSurface networks, tidy them to spec, add mission copies."""
    stage = new_stage(path, default_prim=False)
    layer = stage.GetRootLayer()
    UsdGeom.Scope.Define(stage, "/Materials")
    src_mats = src.layer.GetPrimAtPath("/visuals/_materials")
    names = [c.name for c in src_mats.nameChildren]
    for n in names:
        Sdf.CopySpec(src.layer, f"/visuals/_materials/{n}", layer, Sdf.Path(f"/Materials/{n}"))
    retarget(layer, "/visuals/_materials", "/Materials")
    # mission-specific fender materials: same network, different texture set
    for mat, (set_name, new_name) in mission_sets.items():
        if mat in names:
            Sdf.CopySpec(layer, f"/Materials/{mat}", layer, Sdf.Path(f"/Materials/{new_name}"))
            retarget(layer, f"/Materials/{mat}", f"/Materials/{new_name}", root=f"/Materials/{new_name}")
            names.append(new_name)
    os.makedirs(tex_out, exist_ok=True)
    for prim in stage.Traverse():
        if not prim.IsA(UsdShade.Shader):
            continue
        sh = UsdShade.Shader(prim)
        sid = sh.GetIdAttr().Get()
        if sid == "UsdPrimvarReader_float2":
            sh.GetInput("varname").Set("st")
        elif sid == "UsdUVTexture":
            f = sh.GetInput("file")
            fname = os.path.basename(f.Get().path)
            mat_name = prim.GetPath().GetParentPath().name
            for mat, (set_name, new_name) in mission_sets.items():
                if mat_name == new_name:
                    base = os.path.basename(fname).rsplit("_", 1)[-1]
                    fname = f"{os.path.basename(set_name)}_{base}"
                    shutil.copy(os.path.join(P_TEX, set_name, fname), os.path.join(tex_out, fname))
            src_file = os.path.join(src.dir, "textures", fname)
            if os.path.exists(src_file):
                shutil.copy(src_file, os.path.join(tex_out, fname))
            f.Set(Sdf.AssetPath(f"./Textures/{fname}"))
            bias = sh.GetInput("bias")
            if bias and bias.Get() is not None and tuple(bias.Get())[3] != 0:
                b = tuple(bias.Get())
                bias.Set(Gf.Vec4f(b[0], b[1], b[2], 0.0))
                sh.GetInput("scale").Set(Gf.Vec4f(2, 2, 2, 1))
            # bypass UsdTransform2d (tiling is baked into the UVs at export)
            st = sh.GetInput("st")
            src_conn = st.GetConnectedSource() if st else None
            if src_conn and UsdShade.Shader(src_conn[0].GetPrim()).GetIdAttr().Get() == "UsdTransform2d":
                t2d = UsdShade.Shader(src_conn[0].GetPrim())
                upstream = t2d.GetInput("in").GetConnectedSource()
                if upstream:
                    st.ConnectToSource(upstream[0], upstream[1])
    # drop now-unused UsdTransform2d nodes
    for prim in list(stage.Traverse()):
        if prim.IsA(UsdShade.Shader) and UsdShade.Shader(prim).GetIdAttr().Get() == "UsdTransform2d":
            stage.RemovePrim(prim.GetPath())
    layer.Save()
    return names


def write_physics_materials(path):
    stage = new_stage(path, default_prim=False)
    UsdGeom.Scope.Define(stage, "/PhysicsMaterials")
    for name, (mu_s, mu_d) in {"wheel_mesh": (0.9, 0.8), "aluminium": (0.5, 0.4)}.items():
        m = UsdShade.Material.Define(stage, f"/PhysicsMaterials/{name}")
        api = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
        api.CreateStaticFrictionAttr(mu_s)
        api.CreateDynamicFrictionAttr(mu_d)
        api.CreateRestitutionAttr(0.0)
        apply_schemas(m.GetPrim(), ["NewtonMaterialAPI"])
        set_attr(m.GetPrim(), "newton:torsionalFriction", Sdf.ValueTypeNames.Float, 0.005)
        set_attr(m.GetPrim(), "newton:rollingFriction", Sdf.ValueTypeNames.Float, 0.0001)
    stage.GetRootLayer().Save()


HULL_MIN_VOLUME = 2e-5   # m^3 bounding volume below which parts get no hull collider
MIN_HULL_THICKNESS = 0.006  # m


def _hull(points):
    """Convex hull (points, triangle indices) of a point cloud, as float lists."""
    import numpy as np
    from scipy.spatial import ConvexHull
    pts = np.asarray(points, dtype=float)
    # flat parts (discs, panels, strips) would give planar hulls that MuJoCo/PhysX reject:
    # give every hull at least MIN_HULL_THICKNESS along its thinnest principal direction
    c = pts.mean(0)
    _, sv, vt = np.linalg.svd(pts - c, full_matrices=False)
    thin = vt[-1]
    extent = np.ptp((pts - c) @ thin)
    if extent < MIN_HULL_THICKNESS:
        d = thin * (MIN_HULL_THICKNESS - extent) / 2
        pts = np.concatenate([pts + d, pts - d])
    h = ConvexHull(pts)
    used = np.unique(h.simplices)
    remap = {int(v): i for i, v in enumerate(used)}
    tris = []
    for simplex, eq in zip(h.simplices, h.equations):
        a, b, c = (int(v) for v in simplex)
        n = np.cross(pts[b] - pts[a], pts[c] - pts[a])
        if np.dot(n, eq[:3]) < 0:          # outward winding
            b, c = c, b
        tris += [remap[a], remap[b], remap[c]]
    return [Gf.Vec3f(*pts[v]) for v in used], tris


WHEEL_HULL_RINGS = ((0.0, 32), (0.52, 10), (1.0, 6))   # (|y| / half-width, vertices per ring): 64 total


def _wheel_hull(points):
    """Tyre collider as a crowned polygonal hull with <= 64 vertices (Newton's and PhysX-GPU's hull cap).

    Letting the engine reduce the dense tyre hull to 64 points leaves an uneven polygon (support radius
    0.369-0.411 m, the rover rode ~1.5 cm low and bumped); here most vertices go on the centre ring.
    """
    import numpy as np
    pts = np.asarray(points, dtype=float)
    r, y = np.hypot(pts[:, 0], pts[:, 2]), pts[:, 1]
    hw = np.abs(y).max()
    ring = []
    for frac, n in WHEEL_HULL_RINGS:
        for sy in ((1,) if frac == 0 else (-1, 1)):
            band = np.abs(y - sy * frac * hw) <= 0.06 * hw + 1e-6
            rr = r[band].max()
            off = 0.5 if (n, sy) != (WHEEL_HULL_RINGS[0][1], 1) else 0.0
            ring += [(rr * np.cos(2 * np.pi * (k + off) / n), sy * frac * hw, rr * np.sin(2 * np.pi * (k + off) / n))
                     for k in range(n)]
    return _hull(ring)


def _link_points(src, item):
    """World-free (link-local) points of one visual item."""
    _g, mesh_path, m, _mat = item
    pts = UsdGeom.Mesh(src.stage.GetPrimAtPath(mesh_path)).GetPointsAttr().Get() or []
    return [m.Transform(Gf.Vec3d(p)) for p in pts]


def collider_specs(src, links):
    """{group: {link: [(instance name, kind, data, physics material)]}} in link-local coordinates.

    groups: wheel_cylinder | wheel_mesh  (WheelCollider variant)
            chassis | hull                (BodyColliders variant: chassis = frame boxes, full = boxes + hulls)
    """
    import make_usd as MU
    groups = {"wheel_cylinder": {}, "wheel_mesh": {}, "chassis": {}, "hull": {}}
    boxes = MU.chassis_boxes()
    for name, L in links.items():
        o = Gf.Vec3d(*L["origin"])
        items = src.items.get(name, [])
        if name.endswith("_wheel"):
            groups["wheel_cylinder"][name] = [(f"{name}_col", "cylinder", None, "wheel_mesh")]
            pts = []
            for it in items:
                if any(k in it[0] for k in ("tire_mesh", "chevrons")):
                    pts += _link_points(src, it)
            groups["wheel_mesh"][name] = [(f"{name}_hull_col", "hull", _wheel_hull(pts), "wheel_mesh")]
            continue
        if name in boxes:
            centre, size = boxes[name]
            c = Gf.Vec3d(centre) - o
            hx, hy, hz = size[0] / 2, size[1] / 2, size[2] / 2
            pts = [Gf.Vec3f(c[0] + sx * hx, c[1] + sy * hy, c[2] + sz * hz)
                   for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]
            groups["chassis"][name] = [(f"{name}_col", "box", pts, "aluminium")]
        hulls = []
        for it in items:
            if it[0].endswith("_nasm"):
                continue  # Smithsonian look-alike duplicates / trim: the base part already has a hull
            if name in boxes and ("frame" in it[0] or "floor" in it[0]):
                continue  # the box already covers the frame and floor
            pts = _link_points(src, it)
            if len(pts) < 4:
                continue
            lo = [min(p[i] for p in pts) for i in range(3)]
            hi = [max(p[i] for p in pts) for i in range(3)]
            if (hi[0] - lo[0]) * (hi[1] - lo[1]) * (hi[2] - lo[2]) < HULL_MIN_VOLUME:
                continue
            try:
                hulls.append((f"{it[0]}_hull_col", "hull", _hull(pts), "aluminium"))
            except Exception:
                continue  # degenerate (flat) parts
        if hulls:
            groups["hull"][name] = hulls
    return groups


def write_instances(src, path, mission_geoms, colliders):
    stage = new_stage(path, default_prim=False)
    UsdGeom.Scope.Define(stage, "/Instances")
    entries = []
    for items in src.items.values():
        for g, _mp, _m, mat in items:
            entries.append((g, g, mat))
            if g in mission_geoms:
                for suffix, new_mat in mission_geoms[g]:
                    entries.append((f"{g}__{suffix}", g, new_mat))
    for inst, g, mat in entries:
        root = stage.DefinePrim(f"/Instances/{inst}", "Xform")
        root.GetReferences().AddReference("./geometries.usd", f"/Geometries/{g}")
        if mat:
            mprim = stage.DefinePrim(f"/Instances/{inst}/VisualMaterials/{mat}", "Material")
            stage.DefinePrim(f"/Instances/{inst}/VisualMaterials", "Scope")
            mprim.GetReferences().AddReference("./materials.usda", f"/Materials/{mat}")
            mprim.SetInstanceable(True)
            mesh = stage.OverridePrim(f"/Instances/{inst}/{g}")
            UsdShade.MaterialBindingAPI.Apply(mesh)
            UsdShade.MaterialBindingAPI(mesh).Bind(UsdShade.Material(mprim))
    # collider prototypes for every collider variant (analytic cylinders, boxes, convex hulls)
    for group in colliders.values():
        for link_cols in group.values():
            for inst, kind, data, pmat in link_cols:
                write_collider(stage, inst, kind, data, pmat)
    stage.GetRootLayer().Save()


def write_collider_geometry(stage, inst, kind, data):
    """Collider shape data in geometries.usd (crate): /Geometries/<inst>/<inst>."""
    UsdGeom.Xform.Define(stage, f"/Geometries/{inst}")
    if kind == "cylinder":
        g = UsdGeom.Cylinder.Define(stage, f"/Geometries/{inst}/{inst}")
        g.CreateAxisAttr("Y")
        g.CreateRadiusAttr(P.WHEEL_RADIUS)
        g.CreateHeightAttr(P.WHEEL_WIDTH)
        g.CreateExtentAttr([Gf.Vec3f(-P.WHEEL_RADIUS, -P.WHEEL_WIDTH / 2, -P.WHEEL_RADIUS),
                            Gf.Vec3f(P.WHEEL_RADIUS, P.WHEEL_WIDTH / 2, P.WHEEL_RADIUS)])
        return
    g = UsdGeom.Mesh.Define(stage, f"/Geometries/{inst}/{inst}")
    if kind == "box":
        pts, idx = data, [0, 2, 3, 1, 4, 5, 7, 6, 0, 1, 5, 4, 2, 6, 7, 3, 0, 4, 6, 2, 1, 3, 7, 5]
        counts = [4] * 6
        face_n = [(0, 0, -1), (0, 0, 1), (0, -1, 0), (0, 1, 0), (-1, 0, 0), (1, 0, 0)]
        normals = [Gf.Vec3f(*n) for n in face_n for _ in range(4)]
    else:
        pts, idx = data
        counts = [3] * (len(idx) // 3)
        normals = []
        for k in range(0, len(idx), 3):
            a, b, c = (Gf.Vec3f(pts[i]) for i in idx[k:k + 3])
            n = Gf.Cross(b - a, c - a)
            n = n.GetNormalized() if n.GetLength() > 0 else Gf.Vec3f(0, 0, 1)
            normals += [n, n, n]
    g.CreatePointsAttr(pts)
    g.CreateFaceVertexCountsAttr(counts)
    g.CreateFaceVertexIndicesAttr(idx)
    g.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    g.CreateExtentAttr(UsdGeom.PointBased.ComputeExtent(pts))
    g.CreateNormalsAttr(normals)
    g.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)


def write_collider(stage, inst, kind, data, pmat):
    """Collider prototype in instances.usda: references the crate geometry, adds collision APIs."""
    root = stage.DefinePrim(f"/Instances/{inst}", "Xform")
    root.GetReferences().AddReference("./geometries.usd", f"/Geometries/{inst}")
    prim = stage.OverridePrim(f"/Instances/{inst}/{inst}")
    if kind == "cylinder":
        apis = ["PhysicsCollisionAPI", "NewtonCollisionAPI", "MaterialBindingAPI"]
    else:
        apis = ["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "NewtonCollisionAPI", "NewtonMeshCollisionAPI",
                "MaterialBindingAPI"]
        prim.CreateAttribute("physics:approximation", Sdf.ValueTypeNames.Token, False,
                             Sdf.VariabilityUniform).Set("convexHull")
    apply_schemas(prim, apis)
    prim.CreateAttribute("purpose", Sdf.ValueTypeNames.Token, False, Sdf.VariabilityUniform).Set(UsdGeom.Tokens.guide)
    pm = stage.DefinePrim(f"/Instances/{inst}/PhysicsMaterials/{pmat}", "Material")
    stage.DefinePrim(f"/Instances/{inst}/PhysicsMaterials", "Scope")
    pm.GetReferences().AddReference("./physics_materials.usda", f"/PhysicsMaterials/{pmat}")
    rel = prim.CreateRelationship("material:binding:physics", False)
    rel.SetTargets([pm.GetPath()])
    rel.SetMetadata("bindMaterialAs", "weakerThanDescendants")


def write_option_layer(path, paths, placements=(), cameras=(), links=None):
    """Variant payload: places collider instance roots (and/or cameras) on links."""
    stage = new_stage(path)
    stage.OverridePrim(ROOT)
    for link, inst in placements:
        p = stage.DefinePrim(f"{paths[link]}/{inst}", "Xform")
        p.GetReferences().AddReference("../instances.usda", f"/Instances/{inst}")
        p.SetInstanceable(True)
        xform_ops(p)
    for cam in cameras:
        write_camera(stage, cam, paths, links)
    stage.GetRootLayer().Save()


def write_collider_variants(pay, paths, groups):
    def place(*names):
        return [(link, c[0]) for n in names for link, cols in groups[n].items() for c in cols]

    write_option_layer(os.path.join(pay, "WheelCollider", "cylinder.usda"), paths, place("wheel_cylinder"))
    write_option_layer(os.path.join(pay, "WheelCollider", "mesh.usda"), paths, place("wheel_mesh"))
    write_option_layer(os.path.join(pay, "BodyColliders", "wheels_only.usda"), paths)
    write_option_layer(os.path.join(pay, "BodyColliders", "chassis.usda"), paths, place("chassis"))
    write_option_layer(os.path.join(pay, "BodyColliders", "full.usda"), paths, place("chassis", "hull"))
    return sum(len(c) for cols in groups["hull"].values() for c in [cols])


# ---------------------------------------------------------------- base / robot

def link_class(name):
    for pat, cls in LINK_CLASS:
        if re.search(pat, name):
            return cls
    return None


def write_base(src, path, links, paths, colliders, sites):
    stage = new_stage(path, sublayers=["./robot.usda"])
    root = UsdGeom.Xform.Define(stage, ROOT).GetPrim()
    Usd.ModelAPI(root).SetKind(Kind.Tokens.component)
    Usd.ModelAPI(root).SetAssetInfo({"name": "lrv", "version": VERSION})
    root.SetMetadata("documentation", (
        "Apollo Lunar Roving Vehicle LRV-3 (Apollo 17), Boeing/GM 1971-72. Four-wheel drive with wire-mesh tires, "
        "double-Ackermann steering (motor-driven sector per axle, ball-jointed tie rods closed by loop joints), double-wishbone torsion-bar suspension (closed four-bar loops), T-handle hand "
        "controller, high-gain antenna, LCRU, GCTA TV camera and aft payload pallet. Geometry and materials from the "
        "A17 Systems Handbook, LRV Operations Handbook and Smithsonian NASM LRV qualification-unit photographs."))
    apply_schemas(root, ["GeomModelAPI", "SemanticsLabelsAPI:class", "SemanticsLabelsAPI:wikidata_qcode"])
    set_attr(root, "semantics:labels:class", Sdf.ValueTypeNames.TokenArray, ["lunar_roving_vehicle", "vehicle"])
    set_attr(root, "semantics:labels:wikidata_qcode", Sdf.ValueTypeNames.TokenArray, ["Q275816"])
    for scope in ("Geometry", "Physics", "Materials", "VisualMaterials"):
        UsdGeom.Scope.Define(stage, f"{ROOT}/{scope}")
    for name in dfs(links):
        L = links[name]
        prim = UsdGeom.Xform.Define(stage, paths[name]).GetPrim()
        po = Gf.Vec3d(*links[L["parent"]]["origin"]) if L["parent"] else Gf.Vec3d(0, 0, 0)
        xform_ops(prim, Gf.Vec3d(*L["origin"]) - po, double=False)
        cls = link_class(name)
        if cls:
            apply_schemas(prim, ["SemanticsLabelsAPI:class"])
            set_attr(prim, "semantics:labels:class", Sdf.ValueTypeNames.TokenArray, [cls])
        for g, _mp, m, _mat in src.items.get(name, []):
            if g in TAGS:
                continue  # placed by its variant payload instead
            inst = stage.DefinePrim(f"{paths[name]}/{g}", "Xform")
            inst.GetReferences().AddReference("./instances.usda", f"/Instances/{g}")
            inst.SetInstanceable(True)
            tf = Gf.Transform(m)
            xform_ops(inst, tf.GetTranslation(), tf.GetRotation().GetQuat(), tf.GetScale())
    for link, sname, pos, _axis, _desc in sites:
        prim = UsdGeom.Xform.Define(stage, f"{paths[link]}/{sname}").GetPrim()
        xform_ops(prim, Gf.Vec3d(*pos) - Gf.Vec3d(*links[link]["origin"]), double=False)
    stage.GetRootLayer().Save()
    return stage


CAMERAS = []
TAGGED_LAYERS = []
TAGS = {}   # geometry name -> "Set:option" (parts that exist only in one variant)


def write_tagged_variants(pay, src, paths):
    """Place tagged parts into payloads/<Set>/<option>.usda (adding to the layer if it exists)."""
    by_layer = {}
    for link, items in src.items.items():
        for g, _mp, m, _mat in items:
            if g in TAGS:
                set_name, opts = TAGS[g].split(":", 1)
                for opt in opts.split(","):   # "Set:a,b" = present in options a and b only
                    by_layer.setdefault((set_name, opt.strip()), []).append((link, g, m))
    for (set_name, opt), parts in by_layer.items():
        os.makedirs(os.path.join(pay, set_name), exist_ok=True)
        path = os.path.join(pay, set_name, f"{opt}.usda")
        stage = Usd.Stage.Open(path) if os.path.exists(path) else new_stage(path)
        stage.OverridePrim(ROOT)
        for link, g, m in parts:
            p = stage.DefinePrim(f"{paths[link]}/{g}", "Xform")
            p.GetReferences().AddReference("../instances.usda", f"/Instances/{g}")
            p.SetInstanceable(True)
            tf = Gf.Transform(m)
            xform_ops(p, tf.GetTranslation(), tf.GetRotation().GetQuat(), tf.GetScale())
        stage.GetRootLayer().Save()
    return sorted(by_layer)


def write_camera(stage, cam, paths, links):
    """UsdGeom.Camera on its link (so it follows the pan/tilt joints). USD lens units are mm by convention."""
    fwd, up = Gf.Vec3d(*cam["forward"]), Gf.Vec3d(*cam["up"])
    right = Gf.Cross(fwd, up).GetNormalized()
    m = Gf.Matrix3d(right[0], right[1], right[2], up[0], up[1], up[2], -fwd[0], -fwd[1], -fwd[2])
    rot = m.ExtractRotation()
    c = UsdGeom.Camera.Define(stage, f"{paths[cam['link']]}/{cam['name']}")
    xform_ops(c.GetPrim(), Gf.Vec3d(*cam["pos"]) - Gf.Vec3d(*links[cam["link"]]["origin"]), rot.GetQuat())
    c.CreateProjectionAttr(UsdGeom.Tokens.perspective)
    # UsdGeomCamera lens units are tenths of a scene unit (decimetres at metersPerUnit = 1), not mm
    lens = lambda mm: mm / 100.0  # noqa: E731
    c.CreateFocalLengthAttr(lens(cam["focal_mm"]))
    c.CreateHorizontalApertureAttr(lens(cam["aperture_mm"][0]))
    c.CreateVerticalApertureAttr(lens(cam["aperture_mm"][1]))
    c.CreateFStopAttr(cam["fstop"])
    c.CreateFocusDistanceAttr(cam["focus_m"])
    c.CreateClippingRangeAttr(Gf.Vec2f(*cam["clip_m"]))
    p = c.GetPrim()
    t = Sdf.ValueTypeNames
    set_attr(p, "lrv:focalLengthRange", t.Float2, Gf.Vec2f(*(lens(v) for v in cam["focal_range_mm"])), custom=True)
    p.GetAttribute("lrv:focalLengthRange").SetDocumentation("Zoom range, same units as focalLength (tenths of a scene unit)")
    set_attr(p, "lrv:fStopRange", t.Float2, Gf.Vec2f(*cam["fstop_range"]), custom=True)
    set_attr(p, "lrv:resolution", t.Int2, Gf.Vec2i(*cam["resolution"]), custom=True)
    set_attr(p, "lrv:frameRate", t.Float, cam["frame_rate"], custom=True)
    set_attr(p, "lrv:panTiltRate", t.Float, cam["pan_tilt_rate_deg_s"], custom=True)
    p.SetMetadata("documentation", f"{cam['sensor']}. Optics: {cam['source']}.")
    apply_schemas(p, ["SemanticsLabelsAPI:class"])
    set_attr(p, "semantics:labels:class", t.TokenArray, ["camera_sensor"])


def write_robot(path, links, paths, sites, joints, tree_joints):
    stage = new_stage(path)
    root = stage.OverridePrim(ROOT)
    apply_schemas(root, ["IsaacRobotAPI"])
    t = Sdf.ValueTypeNames
    set_attr(root, "isaac:robotType", t.Token, "Wheeled")
    set_attr(root, "isaac:description", t.String, "Apollo Lunar Roving Vehicle (LRV-3, Apollo 17)")
    set_attr(root, "isaac:namespace", t.String, "lrv")
    set_attr(root, "isaac:license", t.Token, "Other")
    set_attr(root, "isaac:version", t.String, VERSION)
    set_attr(root, "isaac:source", t.String, "lrv repo (scripted Blender build: scripts/build.py)")
    site_by_link = {}
    for link, sname, *_ in sites:
        site_by_link.setdefault(link, []).append(f"{paths[link]}/{sname}")
    order = []
    for n in dfs(links):
        order.append(paths[n])
        order.extend(site_by_link.get(n, []))
    rl = root.CreateRelationship("isaac:physics:robotLinks", False)
    rl.GetPrim()  # noqa: B018
    rl_spec_targets = [Sdf.Path(p) for p in order]
    rj = root.CreateRelationship("isaac:physics:robotJoints", False)
    for rel, targets in ((rl, rl_spec_targets), (rj, [Sdf.Path(f"{ROOT}/Physics/{j}") for j in tree_joints])):
        for tgt in targets:
            rel.AddTarget(tgt, Usd.ListPositionFrontOfPrependList if False else Usd.ListPositionBackOfPrependList)
    for n in links:
        apply_schemas(stage.OverridePrim(paths[n]), ["IsaacLinkAPI"])
    for link, sname, _pos, axis, desc in sites:
        sp = stage.OverridePrim(f"{paths[link]}/{sname}")
        apply_schemas(sp, ["IsaacSiteAPI"])
        set_attr(sp, "isaac:Description", t.String, desc)
        set_attr(sp, "isaac:forwardAxis", t.Token, axis, uniform=True)
    for j in tree_joints:
        jp = stage.OverridePrim(f"{ROOT}/Physics/{j}")
        apply_schemas(jp, ["IsaacJointAPI"])
        role = joints[j].get("role")
        set_attr(jp, "isaac:actuator", t.BoolArray,
                 [not (role in PASSIVE_ROLES or role in LOCK_ROLES or joints[j]["type"] == "fixed")])
    stage.GetRootLayer().Save()


# ---------------------------------------------------------------- physics layers

def mass_props(src, links):
    """Mass (params / links.json), CoM + inertia as a solid box over each link's visual bounds."""
    import make_usd as MU
    MU.resolve_masses(links)
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    out = {}
    for name in links:
        m = MU.mass_of(name)
        prim = src.stage.GetPrimAtPath(f"/visuals/L_{name}")
        box = cache.ComputeRelativeBound(prim, prim).ComputeAlignedRange() if prim else Gf.Range3d()
        if box.IsEmpty():
            com, size = Gf.Vec3f(0, 0, 0), (0.05, 0.05, 0.05)
        else:
            com, size = Gf.Vec3f(box.GetMidpoint()), [max(s, 0.02) for s in box.GetSize()]
        sx, sy, sz = size
        inertia = Gf.Vec3f(m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy))
        out[name] = (m, com, inertia)
    return out


def write_physics(path, links, paths, loops, masses):
    stage = new_stage(path)
    t = Sdf.ValueTypeNames
    tree_joints = []
    for name in dfs(links):
        prim = stage.OverridePrim(paths[name])
        apis = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
        if name == "chassis":
            apis += ["PhysicsArticulationRootAPI", "NewtonArticulationRootAPI"]
        apply_schemas(prim, apis)
        m, com, inertia = masses[name]
        set_attr(prim, "physics:mass", t.Float, m)
        set_attr(prim, "physics:centerOfMass", t.Point3f, com)
        set_attr(prim, "physics:diagonalInertia", t.Float3, inertia)
        set_attr(prim, "physics:principalAxes", t.Quatf, Gf.Quatf(1, 0, 0, 0))
        if name == "chassis":
            set_attr(prim, "newton:selfCollisionEnabled", t.Bool, False)
    stage.OverridePrim(f"{ROOT}/Physics")

    def joint(name, spec, parent, child, pos0, pos1, loop=False):
        jt = spec["type"]
        kind = {"revolute": "PhysicsRevoluteJoint", "prismatic": "PhysicsPrismaticJoint",
                "spherical": "PhysicsSphericalJoint", "fixed": "PhysicsFixedJoint"}[jt]
        prim = stage.DefinePrim(f"{ROOT}/Physics/{name}", kind)
        prim.CreateRelationship("physics:body0", False).SetTargets([Sdf.Path(paths[parent])])
        prim.CreateRelationship("physics:body1", False).SetTargets([Sdf.Path(paths[child])])
        rot = Gf.Quatf(Gf.Rotation(Gf.Vec3d(1, 0, 0), Gf.Vec3d(*spec.get("axis", (1, 0, 0)))).GetQuat())
        set_attr(prim, "physics:localPos0", t.Point3f, Gf.Vec3f(*pos0))
        set_attr(prim, "physics:localPos1", t.Point3f, Gf.Vec3f(*pos1))
        set_attr(prim, "physics:localRot0", t.Quatf, rot)
        set_attr(prim, "physics:localRot1", t.Quatf, rot)
        if loop:
            set_attr(prim, "physics:excludeFromArticulation", t.Bool, True, uniform=True)
        if jt == "fixed":
            return prim
        if jt == "spherical":
            # tie-rod ball ends: free in all three rotations (no cone limit), no drive. Newton imports a tree
            # PhysicsSphericalJoint as a BALL joint and an excluded one as a single MuJoCo CONNECT.
            return prim
        set_attr(prim, "physics:axis", t.Token, "X", uniform=True)
        angular = jt == "revolute"
        sc = DEG if angular else 1.0
        if "limit" in spec:
            lo, hi = spec["limit"]
            set_attr(prim, "physics:lowerLimit", t.Float, lo * sc)
            set_attr(prim, "physics:upperLimit", t.Float, hi * sc)
        if loop:
            return prim
        kind_tok = "angular" if angular else "linear"
        apply_schemas(prim, [f"PhysicsJointStateAPI:{kind_tok}"])
        set_attr(prim, f"state:{kind_tok}:physics:position", t.Float, 0.0)
        set_attr(prim, f"state:{kind_tok}:physics:velocity", t.Float, 0.0)
        d = spec.get("drive")
        if not d:
            # passive linkage joint: a zero-gain drive makes its (free) actuation explicit (NEWTON.DJ.001)
            d = {"kind": "passive", "stiffness": 0.0, "damping": 0.0, "max_force": 1.0e6, "target": 0.0}
        if d:
            apply_schemas(prim, [f"PhysicsDriveAPI:{kind_tok}"])
            base = f"drive:{kind_tok}:physics:"
            set_attr(prim, base + "type", t.Token, "force", uniform=True)
            set_attr(prim, base + "stiffness", t.Float, d.get("stiffness", 0.0) / sc)
            set_attr(prim, base + "damping", t.Float, d.get("damping", 0.0) / sc)
            set_attr(prim, base + "maxForce", t.Float, d.get("max_force", 1.0e6 if d.get("kind") in ("spring", "damper")
                                                                  else 1000.0))
            set_attr(prim, base + "targetPosition", t.Float, d.get("target", 0.0) * sc)
            set_attr(prim, base + "targetVelocity", t.Float, 0.0)
        return prim

    joints = {}
    for name in dfs(links)[1:]:
        L = links[name]
        po, co = Gf.Vec3d(*links[L["parent"]]["origin"]), Gf.Vec3d(*L["origin"])
        joint(name, L["joint"], L["parent"], name, co - po, (0, 0, 0))
        joints[name] = L["joint"]
        tree_joints.append(name)
    for lp in loops:
        po, co = Gf.Vec3d(*links[lp["parent"]]["origin"]), Gf.Vec3d(*links[lp["child"]]["origin"])
        a = Gf.Vec3d(*lp["anchor"])
        joint(lp["name"], {"type": lp["type"], "axis": lp["axis"]}, lp["parent"], lp["child"], a - po, a - co, loop=True)
    stage.GetRootLayer().Save()
    return tree_joints, joints


def write_physx(path, tree_joints, joints, paths):
    stage = new_stage(path, sublayers=["./physics.usda"])
    t = Sdf.ValueTypeNames
    ch = stage.OverridePrim(paths["chassis"])
    apply_schemas(ch, ["PhysxArticulationAPI"])
    set_attr(ch, "physxArticulation:enabledSelfCollisions", t.Bool, False)
    set_attr(ch, "physxArticulation:solverPositionIterationCount", t.Int, 32)
    set_attr(ch, "physxArticulation:solverVelocityIterationCount", t.Int, 1)
    for j in tree_joints:
        spec = joints[j]
        if spec["type"] == "fixed":
            continue
        jp = stage.OverridePrim(f"{ROOT}/Physics/{j}")
        apply_schemas(jp, ["PhysxJointAPI"])
        vel = P.WHEEL_MAX_SPEED * 1.5 * DEG if spec.get("role") == "wheel" else 360.0
        if spec["type"] == "prismatic":
            vel = 1.0
        set_attr(jp, "physxJoint:maxJointVelocity", t.Float, vel)
    stage.GetRootLayer().Save()


def write_mujoco(path, tree_joints, joints):
    """MuJoCo layer (also what Isaac Sim's Newton backend selects): actuators for every servo."""
    stage = new_stage(path, sublayers=["./physics.usda"])
    t = Sdf.ValueTypeNames
    for j in tree_joints:
        spec = joints[j]
        if spec["type"] == "fixed":
            continue
        jp = stage.OverridePrim(f"{ROOT}/Physics/{j}")
        apply_schemas(jp, ["MjcJointAPI"])
        set_attr(jp, "mjc:armature", t.Double, 0.01, uniform=True)
        d = spec.get("drive")
        role = spec.get("role")
        if not d or role in PASSIVE_ROLES:
            continue  # torsion springs and the damper keep their PhysicsDriveAPI (no Mjc replacement yet)
        kind_tok = "angular" if spec["type"] == "revolute" else "linear"
        jp.SetMetadata("apiSchemas", Sdf.TokenListOp.Create(
            deletedItems=[f"PhysicsDriveAPI:{kind_tok}", f"PhysicsJointStateAPI:{kind_tok}"],
            prependedItems=["MjcJointAPI"]))
        act = stage.DefinePrim(f"{ROOT}/Physics/{j}_actuator", "MjcActuator")
        act.CreateRelationship("mjc:target", False).SetTargets([jp.GetPath()])
        f = d.get("max_force", 1000.0)
        set_attr(act, "mjc:forceRange:min", t.Double, -f, uniform=True)
        set_attr(act, "mjc:forceRange:max", t.Double, f, uniform=True)
        set_attr(act, "mjc:forceLimited", t.Token, "true", uniform=True)
        kp, kd = d.get("stiffness", 0.0), d.get("damping", 0.0)  # MuJoCo units: per radian
        if d.get("kind") == "velocity" or kp == 0.0:
            gain, bias = [kd], [0.0, 0.0, -kd]
        else:
            gain, bias = [kp], [0.0, -kp, -kd]
        set_attr(act, "mjc:gainPrm", t.DoubleArray, gain + [0.0] * (10 - len(gain)), uniform=True)
        set_attr(act, "mjc:biasType", t.Token, "affine", uniform=True)
        set_attr(act, "mjc:biasPrm", t.DoubleArray, bias + [0.0] * (10 - len(bias)), uniform=True)
    stage.GetRootLayer().Save()


# ---------------------------------------------------------------- variants

def write_fidelity(out_dir, src, proxy, paths, payload_rel="../"):
    full = new_stage(os.path.join(out_dir, "full.usda"))
    full.OverridePrim(ROOT)
    full.GetRootLayer().Save()
    st = new_stage(os.path.join(out_dir, "proxy.usda"))
    layer = st.GetRootLayer()
    st.OverridePrim(ROOT)
    for link, items in src.items.items():
        for g, *_ in items:
            st.OverridePrim(f"{paths[link]}/{g}").SetActive(False)
    UsdGeom.Scope.Define(st, f"{ROOT}/ProxyLooks")
    for m in proxy.layer.GetPrimAtPath("/visuals/_materials").nameChildren:
        Sdf.CopySpec(proxy.layer, m.path, layer, Sdf.Path(f"{ROOT}/ProxyLooks/{m.name}"))
    retarget(layer, "/visuals/_materials", f"{ROOT}/ProxyLooks")
    for link, items in proxy.items.items():
        if link not in paths:
            continue
        holder = f"{paths[link]}/proxy_visuals"
        UsdGeom.Xform.Define(st, holder)
        for g, mesh_path, m, _mat in items:
            Sdf.CopySpec(proxy.layer, mesh_path, layer, Sdf.Path(f"{holder}/{g}"))
            UsdGeom.Xformable(st.GetPrimAtPath(f"{holder}/{g}")).ClearXformOpOrder()
            tf = Gf.Transform(m)
            xform_ops(st.GetPrimAtPath(f"{holder}/{g}"), tf.GetTranslation(), tf.GetRotation().GetQuat(), tf.GetScale())
    retarget(layer, "/visuals/_materials", f"{ROOT}/ProxyLooks")
    layer.Save()


def write_missions(out_dir, src, paths, mission_geoms):
    a17 = new_stage(os.path.join(out_dir, "a17.usda"))
    a17.OverridePrim(ROOT)
    a17.GetRootLayer().Save()
    a15 = new_stage(os.path.join(out_dir, "a15.usda"))
    a15.OverridePrim(ROOT)
    for link, items in src.items.items():
        for g, _mp, m, _mat in items:
            if g not in mission_geoms:
                continue
            a15.OverridePrim(f"{paths[link]}/{g}").SetActive(False)
            inst = a15.DefinePrim(f"{paths[link]}/{g}__a15", "Xform")
            inst.GetReferences().AddReference("../instances.usda", f"/Instances/{g}__a15")
            inst.SetInstanceable(True)
            tf = Gf.Transform(m)
            xform_ops(inst, tf.GetTranslation(), tf.GetRotation().GetQuat(), tf.GetScale())
    a15.GetRootLayer().Save()


def write_interface(path, links, masses, extents):
    stage = new_stage(path)
    layer = stage.GetRootLayer()
    root = stage.DefinePrim(ROOT, "Xform")
    root.GetReferences().AddReference("./payloads/base.usda")
    vsets = root.GetVariantSets()
    sets = [
        ("Physics", {"none": None, "physics": "Physics/physics.usda", "physx": "Physics/physx.usda",
                     "mujoco": "Physics/mujoco.usda"}, "physx"),
        ("Fidelity", {"none": None, "full": "Fidelity/full.usda", "proxy": "Fidelity/proxy.usda"}, "full"),
        ("Mission", {"none": None, "a15": "Mission/a15.usda", "a17": "Mission/a17.usda"}, "a17"),
        ("WheelCollider", {"none": None, "cylinder": "WheelCollider/cylinder.usda",
                           "mesh": "WheelCollider/mesh.usda"}, "mesh"),
        ("BodyColliders", {"none": None, "wheels_only": "BodyColliders/wheels_only.usda",
                           "chassis": "BodyColliders/chassis.usda", "full": "BodyColliders/full.usda"}, "full"),
        ("Sensor", {"none": None, "all": "Sensor/all.usda"}, "all"),
    ]
    known = {name: opts for name, opts, _d in sets}
    for set_name, opt in TAGGED_LAYERS:
        if set_name not in known:
            sets.append((set_name, {"none": None}, None))
            known[set_name] = sets[-1][1]
        known[set_name].setdefault(opt, f"{set_name}/{opt}.usda")
    defaults = getattr(P, "VARIANT_DEFAULTS", {})
    for set_name, options, default in sets:
        default = defaults.get(set_name, default) or next(o for o in options if o != "none")
        vs = vsets.AddVariantSet(set_name)
        for opt, payload in options.items():
            vs.AddVariant(opt)
            if payload:
                vs.SetVariantSelection(opt)
                with vs.GetVariantEditContext():
                    root.GetPayloads().AddPayload(f"./payloads/{payload}")
        vs.SetVariantSelection(default)
    today = datetime.date.today().isoformat()
    meta = {
        "author": "Alex Maietta", "asset_name": "lrv", "asset_type": "robot", "asset_license": "CC-BY-4.0",
        "category": "vehicle", "source_file": "build/lrv.blend (scripts/build.py)", "usd_date_generated": today,
        "qcode": "Q275816", "rigid_body_count": len(links), "asset_extents": Gf.Vec3f(*extents),
        "mass": float(sum(m for m, _c, _i in masses.values())),
        "asset_version": VERSION,
    }
    usd_version = ".".join(str(x) for x in Usd.GetVersion())
    layer.customLayerData = {"creator": CREATOR, "asset_name": "lrv", "asset_type": "robot",
                             "asset_version": VERSION, "usd_version": usd_version,
                             "source_file": meta["source_file"], "usd_date_generated": today,
                             "SimReady_Metadata": meta}
    layer.Save()


def finish_extents(asset_dir):
    """Author GeomModelAPI extentsHint on /lrv in base.usda from the composed asset."""
    stage = Usd.Stage.Open(os.path.join(asset_dir, "lrv.usda"))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), UsdGeom.Imageable.GetOrderedPurposeTokens())
    model = UsdGeom.ModelAPI(stage.GetPrimAtPath(ROOT))
    hint = model.ComputeExtentsHint(cache)
    vis = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    rng = vis.ComputeWorldBound(stage.GetPrimAtPath(ROOT)).ComputeAlignedRange()
    base = Usd.Stage.Open(os.path.join(asset_dir, "payloads", "base.usda"))
    UsdGeom.ModelAPI(base.GetPrimAtPath(ROOT)).SetExtentsHint(hint)
    base.GetRootLayer().Save()
    return tuple(rng.GetSize())


# hero surfaces keep their full resolution; small auto-textured parts are capped (asset size budget)
TEX_FULL_RES = ("wire_weave", "fiberglass_fender", "aluminium", "titanium", "white_paint", "floor_panel",
                "drive_housing", "hga_knit", "console_panel", "eq_lcru_panel", "flag_us", "eq_gold_kapton",
                "eq_beta_cloth")
TEX_MAX = 1024
if os.environ.get("LRV_DRAFT"):   # draft packaging: every texture capped at 512
    TEX_FULL_RES, TEX_MAX = (), 512


def shrink_textures(tex_dir):
    from PIL import Image
    saved = 0
    for f in sorted(os.listdir(tex_dir)):
        if not f.endswith(".png") or f.startswith(TEX_FULL_RES):
            continue
        p = os.path.join(tex_dir, f)
        before = os.path.getsize(p)
        with Image.open(p) as im:
            if max(im.size) <= TEX_MAX:
                continue
            im = im.resize((TEX_MAX, TEX_MAX * im.size[1] // im.size[0]), Image.LANCZOS)
            im.save(p, optimize=True, compress_level=9)
        saved += before - os.path.getsize(p)
    return saved


P_TEX = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "textures", "gen")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--full", default="build/usd/full")
    ap.add_argument("--proxy", default="build/usd/proxy")
    ap.add_argument("--out", default="build/asset/lrv")
    ap.add_argument("--thumbnail", default="", help="256x256 PNG to install as the asset thumbnail")
    a = ap.parse_args()

    graph = json.load(open(os.path.join(a.full, "links.json")))
    links, loops = graph["links"], graph["loops"]
    CAMERAS.extend(graph.get("cameras", []))
    TAGS.update({sanitize(k): v for k, v in graph.get("variant_objects", {}).items()})
    paths = link_paths(links)
    src, proxy = Source(a.full), Source(a.proxy)
    out = os.path.abspath(a.out)
    pay = os.path.join(out, "payloads")
    for d in ("Physics", "Fidelity", "Mission", "WheelCollider", "BodyColliders", "Sensor", "Textures"):
        os.makedirs(os.path.join(pay, d), exist_ok=True)

    # Mission variants differ by tagged parts (flags, extensions, flaps, stops, repair), not by material
    # swaps: the flight fenders are all the same fibreglass. The swap mechanism stays for future use.
    mission_sets = {}
    mission_geoms = {}
    for items in src.items.values():
        for g, _mp, _m, mat in items:
            if mat in mission_sets:
                mission_geoms[g] = [("a15", mission_sets[mat][1])]

    sites = []
    for link, name, pos, axis, desc in SITES:
        if pos is None:  # grip: a little above the throttle (palm) pivot
            o = links[link]["origin"]
            pos = (o[0], o[1], o[2] + 0.03)
        sites.append((link, name, pos, axis, desc))

    colliders = collider_specs(src, links)
    write_geometries(src, os.path.join(pay, "geometries.usd"), colliders)
    mats = write_materials(src, os.path.join(pay, "materials.usda"), os.path.join(pay, "Textures"), mission_sets)
    print(f"textures: shrank small-part maps by {shrink_textures(os.path.join(pay, 'Textures')) / 1e6:.0f} MB")
    write_physics_materials(os.path.join(pay, "physics_materials.usda"))
    write_instances(src, os.path.join(pay, "instances.usda"), mission_geoms, colliders)
    write_base(src, os.path.join(pay, "base.usda"), links, paths, colliders, sites)
    n_hulls = write_collider_variants(pay, paths, colliders)
    write_option_layer(os.path.join(pay, "Sensor", "all.usda"), paths, cameras=CAMERAS, links=links)
    masses = mass_props(src, links)
    tree_joints, joints = write_physics(os.path.join(pay, "Physics", "physics.usda"), links, paths, loops, masses)
    write_physx(os.path.join(pay, "Physics", "physx.usda"), tree_joints, joints, paths)
    write_mujoco(os.path.join(pay, "Physics", "mujoco.usda"), tree_joints, joints)
    write_robot(os.path.join(pay, "robot.usda"), links, paths, sites, joints, tree_joints)
    write_fidelity(os.path.join(pay, "Fidelity"), src, proxy, paths)
    write_missions(os.path.join(pay, "Mission"), src, paths, mission_geoms)
    TAGGED_LAYERS.extend(write_tagged_variants(pay, src, paths))
    write_interface(os.path.join(out, "lrv.usda"), links, masses, (3.1, 2.3, 1.2))
    extents = finish_extents(out)
    write_interface(os.path.join(out, "lrv.usda"), links, masses, extents)
    if a.thumbnail and os.path.exists(a.thumbnail):
        os.makedirs(os.path.join(out, ".thumbs", "256x256"), exist_ok=True)
        shutil.copy(a.thumbnail, os.path.join(out, ".thumbs", "256x256", "lrv.usda.png"))
    n_geo = sum(len(v) for v in src.items.values())
    print(f"packaged {out}: {len(links)} links, {len(tree_joints)} tree joints + {len(loops)} loops, "
          f"{n_geo} meshes, {len(mats)} materials, {len(sites)} sites, {n_hulls} hull colliders, "
          f"{len(CAMERAS)} cameras, extents {tuple(round(e, 3) for e in extents)}")


if __name__ == "__main__":
    main()
