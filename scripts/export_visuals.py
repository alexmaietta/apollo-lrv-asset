"""Export per-link visual geometry + the joint graph from a built .blend.

Run: blender -b build/lrv.blend -P scripts/export_visuals.py -- [--outdir build] [--lod full|medium|proxy]

Levels of detail (USD "fidelity" variants):
  full    everything as built
  medium  dense meshes decimated to ~25%, tiny parts (rivets, screws) dropped
  proxy   one box per part in its average colour; no textures

Writes:
  <outdir>/visuals.usdc  one root prim per link (/visuals/L_<link>), meshes in link-local coords
  <outdir>/links.json    link tree: parent, world origin (rest pose), joint spec, loop joints
"""
import json
import os
import sys

import bpy
from mathutils import Vector

TINY = 2e-7      # m^3 bounding volume below which parts are dropped from reduced LODs


def _bbox_volume(ob):
    pts = [Vector(c) for c in ob.bound_box]
    d = Vector((max(p.x for p in pts) - min(p.x for p in pts), max(p.y for p in pts) - min(p.y for p in pts),
                max(p.z for p in pts) - min(p.z for p in pts)))
    return d.x * d.y * d.z, pts


def _avg_colour(mat):
    """Representative linear colour of a material (mean of its base colour image if textured)."""
    if mat is None or not mat.node_tree:
        return (0.6, 0.6, 0.6)
    bsdf = next((n for n in mat.node_tree.nodes if n.type == "BSDF_PRINCIPLED"), None)
    if bsdf is None:
        return tuple(mat.diffuse_color)[:3]
    s = bsdf.inputs["Base Color"]
    if not s.is_linked:
        return tuple(s.default_value)[:3]
    node = s.links[0].from_node
    if node.type == "TEX_IMAGE" and node.image:
        node.image.reload()
        px = node.image.pixels[:]
        n = len(px) // 4
        if n == 0:
            return tuple(mat.diffuse_color)[:3]
        step = max(1, n // 4096)
        r = sum(px[i * 4] for i in range(0, n, step)) / len(range(0, n, step))
        g = sum(px[i * 4 + 1] for i in range(0, n, step)) / len(range(0, n, step))
        b = sum(px[i * 4 + 2] for i in range(0, n, step)) / len(range(0, n, step))
        return (r, g, b)  # image pixels are already linear in Blender
    return tuple(mat.diffuse_color)[:3]


def bake_uv_tiling():
    """Fold each material's Mapping-node scale into the mesh UVs, then reset the mapping.

    Viewers that ignore UsdTransform2d (e.g. Newton's GL viewer) then still get textures at the
    right real-world scale; Isaac Sim / Hydra / Blender see identical results either way.
    """
    scale = {}
    for m in bpy.data.materials:
        if not m.node_tree:
            continue
        mp = next((n for n in m.node_tree.nodes if n.type == "MAPPING"), None)
        if mp is not None:
            s = tuple(mp.inputs["Scale"].default_value)
            if abs(s[0] - 1) > 1e-6 or abs(s[1] - 1) > 1e-6:
                scale[m.name] = (s[0], s[1])
    for ob in bpy.data.objects:
        if ob.type != "MESH" or not ob.data.uv_layers:
            continue
        me = ob.data
        if me.users > 1:
            ob.data = me = me.copy()
        mats = [ms.material for ms in ob.material_slots]
        uv = me.uv_layers.active.data
        for poly in me.polygons:
            m = mats[poly.material_index] if poly.material_index < len(mats) else None
            if m is None or m.name not in scale:
                continue
            sx, sy = scale[m.name]
            for li in poly.loop_indices:
                u, v = uv[li].uv
                uv[li].uv = (u * sx, v * sy)
    for name in scale:
        mp = next(n for n in bpy.data.materials[name].node_tree.nodes if n.type == "MAPPING")
        mp.inputs["Scale"].default_value = (1.0, 1.0, 1.0)
    return len(scale)


def reduce(lod):
    flat = {}
    for ob in list(bpy.data.objects):
        if ob.type != "MESH":
            continue
        vol, pts = _bbox_volume(ob)
        if vol < TINY:
            bpy.data.objects.remove(ob)
            continue
        if lod == "medium":
            if len(ob.data.polygons) > 1500 and "tire_mesh" not in ob.name:
                dm = ob.modifiers.new("lod", "DECIMATE")
                dm.ratio = 0.25
        elif lod == "proxy":
            mat = ob.data.materials[0] if ob.data.materials else None
            col = _avg_colour(mat)
            key = tuple(round(c, 2) for c in col)
            if key not in flat:
                m = bpy.data.materials.new(f"proxy_{len(flat)}")
                m.use_nodes = True
                b = m.node_tree.nodes["Principled BSDF"]
                b.inputs["Base Color"].default_value = (*col, 1.0)
                b.inputs["Roughness"].default_value = 0.6
                flat[key] = m
            lo = Vector((min(p.x for p in pts), min(p.y for p in pts), min(p.z for p in pts)))
            hi = Vector((max(p.x for p in pts), max(p.y for p in pts), max(p.z for p in pts)))
            me = bpy.data.meshes.new(ob.name + "_proxy")
            v = [(x, y, z) for z in (lo.z, hi.z) for y in (lo.y, hi.y) for x in (lo.x, hi.x)]
            me.from_pydata(v, [], [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)])
            me.materials.append(flat[key])
            ob.modifiers.clear()
            ob.data = me


def main():
    argv = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    outdir = os.path.abspath(argv[argv.index("--outdir") + 1] if "--outdir" in argv else "build")
    lod = argv[argv.index("--lod") + 1] if "--lod" in argv else "full"
    os.makedirs(outdir, exist_ok=True)

    links = {}
    empties = [o for o in bpy.data.objects if o.type == "EMPTY" and "link" in o]
    for e in empties:
        links[e["link"]] = {
            "parent": e.parent["link"] if e.parent else None,
            "origin": list(e.matrix_world.translation),
            "joint": json.loads(e["joint"]),
            "mass": e.get("mass"),
        }
    graph = {"links": links, "loops": json.loads(bpy.context.scene.get("loops", "[]")),
             "cameras": json.loads(bpy.context.scene.get("cameras", "[]")),
             "variant_objects": {o.name: o["variant"] for o in bpy.data.objects if "variant" in o}}
    with open(os.path.join(outdir, "links.json"), "w") as f:
        json.dump(graph, f, indent=1)

    # Flatten: every link to the origin so its meshes come out in link-local coordinates.
    for e in empties:
        e.parent = None
    for e in empties:
        e.location = (0, 0, 0)
        e.rotation_euler = (0, 0, 0)
        e.name = f"L_{e['link']}"
    bpy.context.view_layer.update()
    if lod != "full":
        reduce(lod)
    print(f"baked UV tiling for {bake_uv_tiling()} materials")

    bpy.ops.wm.usd_export(
        filepath=os.path.join(outdir, "visuals.usdc"),
        root_prim_path="/visuals",
        export_materials=True,
        generate_preview_surface=True,
        export_textures_mode="NEW",
        overwrite_textures=True,
        relative_paths=True,
        export_lights=False,
        export_cameras=False,
        export_custom_properties=False,
        evaluation_mode="RENDER",
        convert_scene_units="METERS",
    )
    print(f"exported {len(links)} links ({lod}), {len(graph['loops'])} loop joints -> {outdir}")


if __name__ == "__main__":
    main()
