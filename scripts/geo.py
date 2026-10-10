"""Mesh-building helpers shared by the build modules (world coords, rest pose)."""
import json
import math

import bpy
from mathutils import Vector

TAU = 2 * math.pi

# ---------------------------------------------------------------- scene helpers

def reset():
    bpy.ops.wm.read_factory_settings(use_empty=True)
    sc = bpy.context.scene
    sc.unit_settings.system = "METRIC"
    return sc


def link(name, loc, parent=None, joint=None, mass=None):
    """Link frame at its joint origin. `joint` describes the joint to `parent`:
    {"type": revolute|prismatic|spherical|fixed, "axis": (x,y,z), "limit": (lo, hi) in rad or m,
     "drive": {...}, "role": str} — stored as JSON for the exporters. Spherical joints (ball joints,
    e.g. tie-rod ends) take no axis, limit or drive."""
    e = bpy.data.objects.new(f"link:{name}", None)
    e["link"] = name
    e["joint"] = json.dumps(joint or {"type": "fixed"})
    if mass is not None:
        e["mass"] = float(mass)  # kg; links without one fall back to params.MASS
    e.empty_display_type = "ARROWS"
    e.empty_display_size = 0.12
    bpy.context.scene.collection.objects.link(e)
    e.location = Vector(loc) - (parent.matrix_world.translation if parent else Vector())
    if parent:
        e.parent = parent
    bpy.context.view_layer.update()
    return e


def loop_joint(name, parent, child, anchor, axis, jtype="revolute", **extra):
    """Loop-closing joint (excluded from the articulation tree); anchor in world coords.
    jtype "revolute" (axis = hinge axis) or "spherical" (a ball joint: point coincidence only, axis unused)."""
    sc = bpy.context.scene
    loops = json.loads(sc.get("loops", "[]"))
    loops.append({"name": name, "type": jtype, "parent": parent, "child": child,
                  "anchor": list(anchor), "axis": list(axis), **extra})
    sc["loops"] = json.dumps(loops)


def add_mesh(name, verts, faces, mat, parent, uvs=None, smooth=False, variant=None):
    """verts in world coordinates (rest pose); stored relative to the parent link.

    variant: optional "Set:option[,option...]" tag (e.g. "Mission:a17_repair", "LCRU:radiator_open",
    "Mission:a15,a17"); the USD packager then places this part only in those variants' payloads
    instead of the base asset (so it is absent from every other option of that set).
    """
    origin = parent.matrix_world.translation
    me = bpy.data.meshes.new(name)
    me.from_pydata([Vector(v) - origin for v in verts], [], faces)
    if uvs is not None:
        uv = me.uv_layers.new(name="UVMap")
        for poly in me.polygons:
            for li in poly.loop_indices:
                uv.data[li].uv = uvs[me.loops[li].vertex_index]
    me.validate()
    if smooth:
        me.shade_smooth()
    if mat:
        me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    ob.parent = parent
    if variant:
        ob["variant"] = variant
    return ob


def tag_variant(ob, variant):
    """Tag an existing object as belonging to a variant ("Set:option"); see add_mesh."""
    ob["variant"] = variant
    return ob


def _frame(d):
    d = d.normalized()
    up = Vector((0, 0, 1)) if abs(d.z) < 0.9 else Vector((1, 0, 0))
    a = d.cross(up).normalized()
    b = d.cross(a).normalized()
    return a, b


def tube_geo(p0, p1, r, sides=12, caps=True, verts=None, faces=None):
    verts = [] if verts is None else verts
    faces = [] if faces is None else faces
    p0, p1 = Vector(p0), Vector(p1)
    a, b = _frame(p1 - p0)
    base = len(verts)
    for p in (p0, p1):
        for i in range(sides):
            t = TAU * i / sides
            verts.append(p + r * (math.cos(t) * a + math.sin(t) * b))
    for i in range(sides):
        j = (i + 1) % sides
        faces.append((base + i, base + j, base + sides + j, base + sides + i))
    if caps:
        faces.append(tuple(base + i for i in reversed(range(sides))))
        faces.append(tuple(base + sides + i for i in range(sides)))
    return verts, faces


def tubes(name, segments, r, mat, parent, sides=12):
    v, f = [], []
    for p0, p1 in segments:
        tube_geo(p0, p1, r, sides, True, v, f)
    return add_mesh(name, v, f, mat, parent, smooth=True)


def box_geo(c, size, verts=None, faces=None):
    verts = [] if verts is None else verts
    faces = [] if faces is None else faces
    c, (sx, sy, sz) = Vector(c), size
    base = len(verts)
    for z in (-1, 1):
        for y in (-1, 1):
            for x in (-1, 1):
                verts.append(c + Vector((x * sx / 2, y * sy / 2, z * sz / 2)))
    for q in ((0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)):
        faces.append(tuple(base + i for i in q))
    return verts, faces


def box(name, c, size, mat, parent):
    v, f = box_geo(c, size)
    return add_mesh(name, v, f, mat, parent)


def revolve_geo(profile, segments, xform, flip=False, u_scale=None):
    """Revolve a (radius, axial) profile about the local Y axis.

    xform maps local wheel coordinates to world. UVs are in metres: u around the
    circumference (at u_scale radius), v along the profile arc length.
    """
    verts, faces, uvs = [], [], []
    n = len(profile)
    arc = [0.0]
    for i in range(1, n):
        (r0, a0), (r1, a1) = profile[i - 1], profile[i]
        arc.append(arc[-1] + math.hypot(r1 - r0, a1 - a0))
    ru = u_scale or max(r for r, _ in profile)
    for s in range(segments + 1):
        t = TAU * s / segments
        for i, (r, a) in enumerate(profile):
            verts.append(xform @ Vector((r * math.sin(t), a, r * math.cos(t))))
            uvs.append((t * ru, arc[i]))
    for s in range(segments):
        for i in range(n - 1):
            q = (s * n + i, s * n + i + 1, (s + 1) * n + i + 1, (s + 1) * n + i)
            faces.append(q[::-1] if flip else q)
    return verts, faces, uvs


def box_beam_geo(p0, p1, w, h, verts=None, faces=None):
    """Rectangular box-section member from p0 to p1: width w horizontal, height h vertical."""
    verts = [] if verts is None else verts
    faces = [] if faces is None else faces
    p0, p1 = Vector(p0), Vector(p1)
    d = (p1 - p0).normalized()
    side = d.cross(Vector((0, 0, 1)))
    side = side.normalized() if side.length > 1e-6 else Vector((0, 1, 0))
    up = side.cross(d).normalized()
    base = len(verts)
    for p in (p0, p1):
        for sw, sh in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            verts.append(p + side * (sw * w / 2) + up * (sh * h / 2))
    for i in range(4):
        j = (i + 1) % 4
        faces.append((base + i, base + j, base + 4 + j, base + 4 + i))
    faces.append((base + 3, base + 2, base + 1, base + 0))
    faces.append((base + 4, base + 5, base + 6, base + 7))
    return verts, faces


def pipe(name, points, r, mat, parent, closed=False, sides=10, smooth=True):
    """Continuous swept tube through world-space points (rolled edges, cables, hoses)."""
    origin = parent.matrix_world.translation
    cu = bpy.data.curves.new(name, "CURVE")
    cu.dimensions = "3D"
    cu.bevel_depth = r
    cu.bevel_resolution = max(1, sides // 4)
    cu.use_fill_caps = not closed
    sp = cu.splines.new("POLY")
    sp.points.add(len(points) - 1)
    for p, q in zip(sp.points, points):
        p.co = (*(Vector(q) - origin), 1.0)
    sp.use_cyclic_u = closed
    tmp = bpy.data.objects.new(name + "_curve", cu)
    bpy.context.scene.collection.objects.link(tmp)
    dg = bpy.context.evaluated_depsgraph_get()
    me = bpy.data.meshes.new_from_object(tmp.evaluated_get(dg))
    bpy.data.objects.remove(tmp)
    bpy.data.curves.remove(cu)
    me.name = name
    if smooth:
        me.shade_smooth()
    if mat:
        me.materials.append(mat)
    ob = bpy.data.objects.new(name, me)
    bpy.context.scene.collection.objects.link(ob)
    ob.parent = parent
    return ob


def ensure_uvs(ob, scale=1.0):
    """Give a mesh without UVs a box projection in object-local metres (seamless enough for tileable sets)."""
    me = ob.data
    if me.uv_layers:
        return
    uv = me.uv_layers.new(name="UVMap")
    co = [v.co for v in me.vertices]
    for poly in me.polygons:
        n = poly.normal
        ax = max(range(3), key=lambda i: abs(n[i]))
        a, b = [(1, 2), (0, 2), (0, 1)][ax]
        flip = -1.0 if n[ax] < 0 else 1.0
        for li in poly.loop_indices:
            p = co[me.loops[li].vertex_index]
            uv.data[li].uv = (p[a] * scale * (flip if ax != 2 else 1.0), p[b] * scale)


def rivet_geo(p, n, r, h=None, verts=None, faces=None, sides=10):
    """Domed (button-head) rivet at world point p, facing direction n."""
    verts = [] if verts is None else verts
    faces = [] if faces is None else faces
    h = h if h is not None else r * 0.55
    n = Vector(n).normalized()
    a, b = _frame(n)
    base = len(verts)
    rings = [(r, 0.0), (r * 0.92, h * 0.45), (r * 0.7, h * 0.8), (r * 0.38, h * 0.97)]
    for rr, z in rings:
        for i in range(sides):
            t = TAU * i / sides
            verts.append(Vector(p) + n * z + rr * (math.cos(t) * a + math.sin(t) * b))
    verts.append(Vector(p) + n * h)
    top = len(verts) - 1
    for k in range(len(rings) - 1):
        for i in range(sides):
            j = (i + 1) % sides
            faces.append((base + k * sides + i, base + k * sides + j, base + (k + 1) * sides + j, base + (k + 1) * sides + i))
    last = base + (len(rings) - 1) * sides
    for i in range(sides):
        faces.append((last + i, last + (i + 1) % sides, top))
    return verts, faces


def sensor_camera(name, parent, pos, forward, up, spec):
    """Optical sensor on a link: a Blender camera (for renders) + a record the USD packager turns into
    a UsdGeom.Camera under the link, so it follows every joint upstream of it.

    spec: {"focal_mm": f, "focal_range_mm": (lo, hi), "aperture_mm": (h, v), "fstop": n,
           "fstop_range": (lo, hi), "focus_m": d, "clip_m": (near, far), "source": str, ...}
    """
    fwd, upv = Vector(forward).normalized(), Vector(up).normalized()
    right = fwd.cross(upv).normalized()
    upv = right.cross(fwd).normalized()
    cam = bpy.data.cameras.new(name)
    cam.lens = spec["focal_mm"]
    cam.sensor_fit = "HORIZONTAL"
    cam.sensor_width, cam.sensor_height = spec["aperture_mm"]
    cam.clip_start, cam.clip_end = spec.get("clip_m", (0.05, 5000.0))
    ob = bpy.data.objects.new(name, cam)
    bpy.context.scene.collection.objects.link(ob)
    # Blender/USD cameras look down -Z with +Y up
    from mathutils import Matrix
    rot = Matrix((right, upv, -fwd)).transposed().to_4x4()
    ob.matrix_world = Matrix.Translation(Vector(pos)) @ rot
    mw = ob.matrix_world.copy()
    ob.parent = parent
    ob.matrix_parent_inverse = parent.matrix_world.inverted()
    ob.matrix_world = mw
    sc = bpy.context.scene
    cams = json.loads(sc.get("cameras", "[]"))
    cams.append({"name": name, "link": parent["link"], "pos": list(pos), "forward": list(fwd), "up": list(upv),
                 **{k: (list(v) if isinstance(v, tuple) else v) for k, v in spec.items()}})
    sc["cameras"] = json.dumps(cams)
    return ob
