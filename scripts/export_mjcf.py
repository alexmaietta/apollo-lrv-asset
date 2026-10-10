"""Export the LRV as MJCF (MuJoCo XML) with true closed-loop suspension.

Run: .venv/bin/python scripts/export_mjcf.py [--links build/usd/full/links.json]
       [--mesh-src build/urdf/meshes] [--out build/mjcf] [--locks weld|stiff] [--tex-size 512]
       [--validate] [--newton]

Inputs: links.json (export_visuals.py) and the per-link OBJ meshes written by export_urdf.py
(re-exported with Blender if missing or older than links.json).
Outputs in --out: lrv.xml (the vehicle), lrv_scene.xml (floor + light, <include>s lrv.xml),
meshes/<link>__<material>.obj (each link OBJ split by material, link-local coordinates) and
textures/<name>.png (base-colour maps, downscaled).

Modelling (units m / kg / rad, X forward, Y left, Z up; every link frame is unrotated at rest):
- Body tree = links.json tree; the chassis has a freejoint. Joint frames sit at the link origin.
- Pinned configuration joints (LOCK_ROLES) are welded in the deployed pose (--locks weld) or kept
  as stiff hinges/slides holding their target (--locks stiff).
- Double wishbones and dampers are closed with <equality><connect>: each revolute loop joint gets
  two connect points on its hinge axis (anchor +/- LOOP_HALF_SPAN), which together act as a hinge.
- Steering: each sector is a position-servoed hinge; each tie rod hangs off its sector stud on a <joint
  type="ball"> and is closed to the knuckle arm by one connect (a ball joint is exactly a point constraint).
  The kingpins are unactuated hinges that follow the linkage; the tie rods' free spin about their own axis
  is harmless and lightly damped.
- Torsion bars: hinge stiffness + springref (= the drive's pre-wound target) on the lower and
  upper arms. Dampers: slide-joint damping on the rod. Suspension travel limits are joint ranges.
- Actuators: velocity servos on the wheels, position servos (kp/kv from the drive spec) on the
  steering sectors, hand controller, HGA/LGA and TV pan/tilt joints.
- Contact geometry: wheel cylinders and chassis-section boxes only (no vehicle self-collision);
  visual meshes are contype=conaffinity=0, group 1.
- Inertials: params/make_usd mass, CoM + diagonal inertia from the visual bounding box (solid box).
"""
import argparse
import json
import math
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT_DIR = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import params as P  # noqa: E402

LOCK_ROLES = {"chassis_fold", "fender_extension", "seat_fold", "dust_cover", "hga_fold", "hga_telescope"}
POSITION_ROLES = {"steer", "hand_controller_steer", "hand_controller_brake", "hand_controller_throttle",
                  "reverse_inhibit", "hga_azimuth", "hga_elevation", "lga_azimuth", "lga_elevation",
                  "tv_pan", "tv_tilt"}
LOOP_HALF_SPAN = 0.05      # m, connect points either side of a loop hinge's anchor
ARMATURE = {"wheel": 0.05, "steer": 0.02, "suspension": 0.01, "suspension_upper": 0.01, "damper": 0.01,
            "passive": 0.005}
ARMATURE_DEFAULT = 0.002
BALL_DAMPING = 0.01        # N*m*s/rad on tie-rod ball joints (only the unconstrained spin about the rod sees it)
FRICTION = (0.8, 0.02, 0.002)


def fmt(v, nd=6):
    if isinstance(v, (int, float, np.floating)):
        v = [v]
    out = []
    for x in v:
        s = f"{float(x):.{nd}f}".rstrip("0").rstrip(".")
        out.append("0" if s in ("-0", "") else s)
    return " ".join(out)


# ---------------------------------------------------------------- meshes / materials

def ensure_meshes(mesh_src, links_path):
    """Re-export link OBJs with Blender when they're missing or older than links.json."""
    probe = os.path.join(mesh_src, "chassis.obj")
    if os.path.exists(probe) and os.path.getmtime(probe) >= os.path.getmtime(links_path):
        return
    outdir = os.path.dirname(os.path.abspath(mesh_src))
    blender = os.environ.get("BLENDER", "blender")
    print(f"exporting link meshes with Blender -> {outdir}")
    subprocess.run([blender, "-b", os.path.join(ROOT_DIR, "build", "lrv.blend"), "-P",
                    os.path.join(HERE, "export_urdf.py"), "--", "--outdir", outdir], check=True,
                   stdout=subprocess.DEVNULL)


def parse_mtl(path):
    """newmtl -> {Kd, d, map_Kd, map_d, scale}."""
    mats, cur = {}, None
    if not os.path.exists(path):
        return mats
    for line in open(path):
        t = line.split()
        if not t:
            continue
        if t[0] == "newmtl":
            cur = mats.setdefault(t[1], {"scale": 1.0})
        elif cur is None:
            continue
        elif t[0] == "Kd":
            cur["Kd"] = tuple(float(x) for x in t[1:4])
        elif t[0] == "d":
            cur["d"] = float(t[1])
        elif t[0] in ("map_Kd", "map_d"):
            cur[t[0]] = t[-1]
            if "-s" in t and t[0] == "map_Kd":
                cur["scale"] = float(t[t.index("-s") + 1])
    return mats


def split_obj(path):
    """Parse a triangulated OBJ; return (bbox_min, bbox_max, {material: obj text}) with per-material re-indexing."""
    V, T, N = [], [], []
    groups = {}
    mat = "default"
    for line in open(path):
        if line.startswith("v "):
            V.append(line[2:].strip())
        elif line.startswith("vt "):
            T.append(line[3:].strip())
        elif line.startswith("vn "):
            N.append(line[3:].strip())
        elif line.startswith("usemtl "):
            mat = line.split()[1]
        elif line.startswith("f "):
            groups.setdefault(mat, []).append(line.split()[1:])
    if not V:
        return None, None, {}
    pts = np.array([[float(x) for x in v.split()[:3]] for v in V])
    out = {}
    for m, faces in groups.items():
        maps = ({}, {}, {})
        lv, lt, ln, lf = [], [], [], []
        for f in faces:
            corners = []
            for c in f:
                idx = c.split("/")
                ids = []
                for k, (src, dst, m_) in enumerate(((V, lv, maps[0]), (T, lt, maps[1]), (N, ln, maps[2]))):
                    if k < len(idx) and idx[k]:
                        i = int(idx[k])
                        i = i - 1 if i > 0 else len(src) + i
                        if i not in m_:
                            m_[i] = len(dst) + 1
                            dst.append(src[i])
                        ids.append(str(m_[i]))
                    else:
                        ids.append("")
                corners.append("/".join(ids).rstrip("/"))
            for k in range(1, len(corners) - 1):  # fan-triangulate just in case
                lf.append(f"f {corners[0]} {corners[k]} {corners[k + 1]}")
        if len(lf) == 0:
            continue
        txt = ["v " + v for v in lv] + ["vt " + t for t in lt] + ["vn " + n for n in ln] + lf
        out[m] = "\n".join(txt) + "\n"
    return pts.min(0), pts.max(0), out


def convert_texture(src, dst, size, alpha_src=None):
    from PIL import Image
    im = Image.open(src).convert("RGB")
    if alpha_src and os.path.exists(alpha_src):
        a = Image.open(alpha_src).convert("L").resize(im.size)
        im = im.convert("RGBA")
        im.putalpha(a)
    if max(im.size) > size:
        s = size / max(im.size)
        im = im.resize((max(1, round(im.size[0] * s)), max(1, round(im.size[1] * s))), Image.LANCZOS)
    im.save(dst)


def srgb(c):
    """Linear albedo -> display value (MuJoCo rgba is used as-is by the renderer)."""
    return tuple(min(1.0, max(0.0, x)) ** (1 / 2.2) for x in c)


FALLBACK_RGB = {"fender": P.FENDER_COLOR[P.MISSION][0], "fender_ext": P.FENDER_COLOR[P.MISSION][1]}


class Assets:
    def __init__(self, mesh_src, out, tex_size):
        self.src, self.out, self.tex_size = mesh_src, out, tex_size
        self.meshes, self.textures, self.materials = [], {}, {}
        os.makedirs(os.path.join(out, "meshes"), exist_ok=True)
        os.makedirs(os.path.join(out, "textures"), exist_ok=True)

    def material(self, name, spec):
        if name in self.materials:
            return name
        attrs = {"name": name, "specular": "0.3", "shininess": "0.3"}
        tex = spec.get("map_Kd")
        tex_path = os.path.join(self.src, tex) if tex else None
        if tex_path and os.path.exists(tex_path):
            alpha = os.path.join(self.src, spec["map_d"]) if spec.get("map_d") else None
            stem = os.path.splitext(tex)[0] + ("_rgba" if alpha else "")
            if stem not in self.textures:
                convert_texture(tex_path, os.path.join(self.out, "textures", stem + ".png"), self.tex_size, alpha)
                self.textures[stem] = f"textures/{stem}.png"
            attrs["texture"] = stem
            if spec["scale"] != 1.0:
                attrs["texrepeat"] = fmt([spec["scale"], spec["scale"]])
            attrs["rgba"] = "1 1 1 " + ("0.999" if alpha else "1")  # alpha < 1 enables blending of the cut-out
        else:
            kd = spec.get("Kd") or FALLBACK_RGB.get(name) or (0.6, 0.6, 0.6)
            attrs["rgba"] = fmt([*srgb(kd), spec.get("d", 1.0)], 4)
        self.materials[name] = attrs
        return name

    def link(self, link):
        """Split the link OBJ by material; return (bbox, [(mesh name, material)])."""
        path = os.path.join(self.src, f"{link}.obj")
        if not os.path.exists(path):
            return None, []
        lo, hi, parts = split_obj(path)
        mtl = parse_mtl(os.path.join(self.src, f"{link}.mtl"))
        geoms = []
        for i, (m, txt) in enumerate(sorted(parts.items())):
            safe = re.sub(r"[^A-Za-z0-9_]", "_", m)
            mesh = f"{link}__{safe}"
            with open(os.path.join(self.out, "meshes", mesh + ".obj"), "w") as f:
                f.write(txt)
            self.meshes.append(mesh)
            geoms.append((mesh, self.material(safe, mtl.get(m, {"scale": 1.0}))))
        return (lo, hi), geoms

    def write(self, asset):
        for stem, rel in sorted(self.textures.items()):
            ET.SubElement(asset, "texture", name=stem, type="2d", file=rel)
        for name, attrs in sorted(self.materials.items()):
            ET.SubElement(asset, "material", **attrs)
        for m in self.meshes:
            ET.SubElement(asset, "mesh", name=m, file=f"meshes/{m}.obj", inertia="shell")


# ---------------------------------------------------------------- model

def masses(links):
    import make_usd as MU          # resolve_masses / chassis_boxes (imports pxr, from usd-core)
    MU.resolve_masses(links)
    boxes = {k: (tuple(c), tuple(s)) for k, (c, s) in MU.chassis_boxes().items()}
    return dict(MU.MASSES), boxes


def children_of(links):
    kids = {n: [] for n in links}
    for n, L in links.items():
        if L["parent"]:
            kids[L["parent"]].append(n)
    return kids


def build(graph, assets, locks="weld"):
    links, loops, cams = graph["links"], graph["loops"], graph.get("cameras", [])
    mass, boxes = masses(links)
    kids = children_of(links)
    origin = {n: np.array(L["origin"]) for n, L in links.items()}

    root = ET.Element("mujoco", model="lrv")
    ET.SubElement(root, "compiler", angle="radian", meshdir=".", texturedir=".", inertiafromgeom="false",
                  autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", gravity=f"0 0 {-P.G_MOON}", integrator="implicitfast",
                  cone="elliptic", impratio="10", iterations="100", ls_iterations="50")
    ET.SubElement(root, "size", memory="64M")
    default = ET.SubElement(root, "default")
    dc = ET.SubElement(default, "default", {"class": "lrv"})
    ET.SubElement(dc, "joint", armature=fmt(ARMATURE_DEFAULT), damping="0")
    vis = ET.SubElement(dc, "default", {"class": "visual"})
    ET.SubElement(vis, "geom", type="mesh", contype="0", conaffinity="0", group="1", density="0")
    col = ET.SubElement(dc, "default", {"class": "collision"})
    # contype 1 / conaffinity 2: hits the floor and other default-class objects, never the rover itself
    ET.SubElement(col, "geom", contype="1", conaffinity="2", group="3", density="0", condim="3",
                  friction=fmt(FRICTION), rgba="0.9 0.3 0.2 0.4")
    asset = ET.SubElement(root, "asset")
    world = ET.SubElement(root, "worldbody")
    equality = ET.SubElement(root, "equality")
    actuator = ET.SubElement(root, "actuator")
    contact = ET.SubElement(root, "contact")
    info = {"joints": {}, "actuators": [], "loops": []}

    def body(name, parent_el):
        L = links[name]
        rel = origin[name] - (origin[L["parent"]] if L["parent"] else 0)
        attrs = {"name": name, "pos": fmt(rel)}
        if L["parent"] is None:
            attrs["childclass"] = "lrv"
        b = ET.SubElement(parent_el, "body", **attrs)
        bbox, geoms = assets.link(name)
        m = mass[name]
        if bbox is not None:
            lo, hi = bbox
            com, size = (lo + hi) / 2, np.maximum(hi - lo, 0.02)
        else:
            com, size = np.zeros(3), np.full(3, 0.05)
        sx, sy, sz = size
        I = [m / 12 * (sy * sy + sz * sz), m / 12 * (sx * sx + sz * sz), m / 12 * (sx * sx + sy * sy)]
        ET.SubElement(b, "inertial", pos=fmt(com), mass=fmt(m), diaginertia=fmt(I, 8))

        spec = L["joint"]
        role, jt, d = spec.get("role"), spec["type"], spec.get("drive") or {}
        if L["parent"] is None:
            ET.SubElement(b, "freejoint", name="root")
        elif jt == "spherical":  # tie-rod ball end at the sector stud
            ET.SubElement(b, "joint", name=name, type="ball", armature=fmt(ARMATURE.get(role, ARMATURE_DEFAULT)),
                          damping=fmt(BALL_DAMPING))
            info["joints"][name] = role
        elif jt != "fixed" and not (role in LOCK_ROLES and locks == "weld"):
            ja = {"name": name, "type": "hinge" if jt == "revolute" else "slide", "axis": fmt(spec["axis"])}
            if "limit" in spec and not (jt == "revolute" and spec["limit"][1] - spec["limit"][0] >= 2 * math.pi - 1e-6):
                ja["range"] = fmt(spec["limit"])
            ja["armature"] = fmt(ARMATURE.get(role, ARMATURE_DEFAULT))
            if d.get("kind") == "spring" or (role in LOCK_ROLES):
                ja["stiffness"] = fmt(d.get("stiffness", 0.0))
                ja["springref"] = fmt(d.get("target", 0.0))
                if d.get("damping"):
                    ja["damping"] = fmt(d["damping"])
            elif d.get("kind") == "damper":
                ja["damping"] = fmt(d["damping"])
            ET.SubElement(b, "joint", **ja)
            info["joints"][name] = role
            if role == "wheel":
                w = 1.5 * P.WHEEL_MAX_SPEED
                ET.SubElement(actuator, "velocity", name=name, joint=name, kv=fmt(d.get("damping", P.WHEEL_DRIVE_D)),
                              ctrlrange=fmt([-w, w]), forcerange=fmt([-d.get("max_force", P.WHEEL_MAX_TORQUE),
                                                                     d.get("max_force", P.WHEEL_MAX_TORQUE)]))
                info["actuators"].append(name)
            elif role in POSITION_ROLES and d.get("kind") == "position":
                aa = {"name": name, "joint": name, "kp": fmt(d["stiffness"]), "kv": fmt(d.get("damping", 0.0))}
                if "range" in ja:
                    aa["ctrlrange"] = ja["range"]
                else:
                    aa["ctrlrange"] = fmt([-math.pi, math.pi])
                if d.get("max_force"):
                    aa["forcerange"] = fmt([-d["max_force"], d["max_force"]])
                ET.SubElement(actuator, "position", **aa)
                info["actuators"].append(name)

        for mesh, mat in geoms:
            ET.SubElement(b, "geom", {"class": "visual", "name": mesh, "mesh": mesh, "material": mat})
        if name.endswith("_wheel"):
            ET.SubElement(b, "geom", {"class": "collision", "name": f"{name}_collision", "type": "cylinder",
                                      "size": fmt([P.WHEEL_RADIUS, P.WHEEL_WIDTH / 2]), "quat": "0.707107 0.707107 0 0"})
        elif name in boxes:
            c, s = boxes[name]
            ET.SubElement(b, "geom", {"class": "collision", "name": f"{name}_collision", "type": "box",
                                      "pos": fmt(np.array(c) - origin[name]), "size": fmt(np.array(s) / 2)})
        for cam in cams:
            if cam["link"] == name:
                fwd, up = np.array(cam["forward"], float), np.array(cam["up"], float)
                right = np.cross(fwd, up)
                right /= np.linalg.norm(right)
                fovy = math.degrees(2 * math.atan(cam["aperture_mm"][1] / 2 / cam["focal_mm"]))
                ET.SubElement(b, "camera", name=cam["name"], pos=fmt(np.array(cam["pos"]) - origin[name]),
                              xyaxes=fmt([*right, *up]), fovy=fmt(fovy, 3),
                              resolution=fmt(cam.get("resolution", (640, 480))))
                info["camera"] = (cam["name"], fovy)
        for k in kids[name]:
            body(k, b)

    roots = [n for n, L in links.items() if L["parent"] is None]
    assert roots == ["chassis"], roots
    body("chassis", world)

    # true loop closures: two connects on each loop hinge axis (= a revolute joint)
    for lp in loops:
        a, ax = np.array(lp["anchor"]), np.array(lp["axis"], float)
        ax /= np.linalg.norm(ax)
        offsets = (-LOOP_HALF_SPAN, LOOP_HALF_SPAN) if lp["type"] == "revolute" else (0.0,)
        info["connects"] = info.get("connects", 0) + len(offsets)
        for i, off in enumerate(offsets):
            ET.SubElement(equality, "connect", name=f"{lp['name']}_{i}", body1=lp["parent"], body2=lp["child"],
                          anchor=fmt(a + off * ax - origin[lp["parent"]]), solref="0.004 1")
        info["loops"].append(lp["name"])
        # the loop's two bodies overlap at the joint; contacts are already off (visual-only), keep MuJoCo quiet
        ET.SubElement(contact, "exclude", body1=lp["parent"], body2=lp["child"])

    assets.write(asset)
    for el in (equality, actuator, contact):
        if len(el) == 0:
            root.remove(el)
    return root, info


def scene_xml():
    root = ET.Element("mujoco", model="lrv_scene")
    ET.SubElement(root, "include", file="lrv.xml")
    ET.SubElement(root, "statistic", center="0 0 0.6", extent="3")
    v = ET.SubElement(root, "visual")
    ET.SubElement(v, "headlight", diffuse="0.5 0.5 0.5", ambient="0.15 0.15 0.15", specular="0 0 0")
    ET.SubElement(v, "global", azimuth="140", elevation="-20", offwidth="1280", offheight="960")
    ET.SubElement(v, "quality", shadowsize="4096")
    ET.SubElement(v, "rgba", haze="0 0 0 1")  # black lunar sky, no horizon glow
    a = ET.SubElement(root, "asset")
    ET.SubElement(a, "texture", name="regolith", type="2d", builtin="flat", rgb1="0.42 0.40 0.38",
                  rgb2="0.36 0.34 0.32", mark="random", random="0.4", markrgb="0.5 0.48 0.45", width="512", height="512")
    ET.SubElement(a, "material", name="regolith", texture="regolith", texrepeat="20 20", reflectance="0")
    ET.SubElement(a, "texture", name="sky", type="skybox", builtin="flat", rgb1="0 0 0", rgb2="0 0 0",
                  width="64", height="64")
    w = ET.SubElement(root, "worldbody")
    ET.SubElement(w, "light", name="sun", directional="true", pos="0 0 10", dir="0.5 0.3 -0.6",
                  diffuse="1.1 1.1 1.1", castshadow="true")
    ET.SubElement(w, "geom", name="floor", type="plane", size="0 0 0.05", material="regolith",
                  friction=fmt(FRICTION), condim="3")
    return root


def write_xml(root, path):
    ET.indent(root)
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=False)


# ---------------------------------------------------------------- validation

def validate(scene_path, links):
    import mujoco
    from sim_newton import ackermann

    m = mujoco.MjModel.from_xml_path(scene_path)
    d = mujoco.MjData(m)
    print(f"loaded {scene_path}: nbody={m.nbody} njnt={m.njnt} nq={m.nq} nu={m.nu} neq={m.neq} "
          f"nmesh={m.nmesh} ntex={m.ntex} nmat={m.nmat} ncam={m.ncam}  mass={m.body_subtreemass[1]:.1f} kg")
    jid = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, i): i for i in range(m.njnt)}
    aid = {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i for i in range(m.nu)}
    ch = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "chassis")

    def q(n):
        return d.qpos[m.jnt_qposadr[jid[n]]]

    def loop_err():
        err = 0.0
        for e in range(m.neq):
            if m.eq_type[e] != mujoco.mjtEq.mjEQ_CONNECT:
                continue
            b1, b2 = m.eq_obj1id[e], m.eq_obj2id[e]
            p1 = d.xpos[b1] + d.xmat[b1].reshape(3, 3) @ m.eq_data[e, 0:3]
            p2 = d.xpos[b2] + d.xmat[b2].reshape(3, 3) @ m.eq_data[e, 3:6]
            err = max(err, np.linalg.norm(p1 - p2))
        return err

    def state(tag):
        corners = list(P.CORNERS)
        arms = " ".join(f"{math.degrees(q(f'{c}_lower_arm')):6.2f}" for c in corners)
        uppers = " ".join(f"{math.degrees(q(f'{c}_upper_arm')):6.2f}" for c in corners)
        damp = " ".join(f"{1000 * q(f'{c}_damper_rod'):6.1f}" for c in corners)
        steer = " ".join(f"{math.degrees(q(f'{c}_steer')):6.1f}" for c in corners)
        x, y, z = d.xpos[ch]
        qw, qx, qy, qz = d.xquat[ch]
        yaw = math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
        v = np.linalg.norm(d.qvel[:3]) * 3.6
        print(f"{tag:>10} t={d.time:5.2f} chassis=({x:6.2f},{y:6.2f},{z:6.4f}) yaw={yaw:7.1f} v={v:5.2f}km/h "
              f"lower[lf rf lr rr]={arms} upper={uppers} damper_mm={damp} steer={steer} loop_err={1000 * loop_err():.2f}mm")
        return z

    def command(speed_kmh, steer_deg):
        """Wheel speeds from the Ackermann law; steering goes to the two sectors (the linkage does the rest)."""
        w = speed_kmh / 3.6 / P.WHEEL_RADIUS
        for end in ("fwd", "aft"):
            d.ctrl[aid[f"{end}_steer_sector"]] = P.steer_sector_angle(math.radians(steer_deg))
        for c, (_ang, ratio) in ackermann(math.radians(steer_deg)).items():
            d.ctrl[aid[f"{c}_wheel"]] = w * ratio

    mujoco.mj_forward(m, d)
    z0 = d.xpos[ch][2]
    state("rest")
    command(0, 0)
    while d.time < 3.0:
        mujoco.mj_step(m, d)
    z = state("settled")
    lim = {c: m.jnt_range[jid[f"{c}_lower_arm"]] for c in P.CORNERS}
    margins = {c: (math.degrees(q(f"{c}_lower_arm") - lim[c][0]), math.degrees(lim[c][1] - q(f"{c}_lower_arm")))
               for c in P.CORNERS}
    print(f"settle: chassis origin {1000 * (z - z0):+.1f} mm vs rest pose; lower-arm margin to stops "
          + ", ".join(f"{c} {a:.1f}/{b:.1f} deg" for c, (a, b) in margins.items()))

    # drive: 2 s straight at 6 km/h, then an 18 deg virtual-steer double-Ackermann turn
    speed, delta = 6.0, 18.0
    t0 = d.time
    command(speed, 0)
    while d.time < t0 + 3.0:
        mujoco.mj_step(m, d)
    state("straight")
    command(speed, delta)
    xy, yaws = [], []
    t1 = d.time
    while d.time < t1 + 8.0:
        mujoco.mj_step(m, d)
        if d.time > t1 + 2.0:
            xy.append(d.xpos[ch][:2].copy())
        if round(d.time / m.opt.timestep) % 1000 == 0:
            state("turn")
    # kingpins vs the linkage's own kinematics and vs the exact double-Ackermann law
    phi = {e: q(f"{e}_steer_sector") for e in ("fwd", "aft")}
    exact = ackermann(math.radians(delta))
    print("steer @ %.0f deg virtual: sectors fwd %.2f aft %.2f deg (cmd %.2f)" % (
        delta, math.degrees(phi["fwd"]), math.degrees(phi["aft"]), math.degrees(P.steer_sector_angle(math.radians(delta)))))
    for c, (sx, _) in P.CORNERS.items():
        kin = P.steer_linkage(phi["fwd" if sx > 0 else "aft"], c)[0]
        print(f"  {c}: kingpin {math.degrees(q(f'{c}_steer')):7.2f}  linkage {math.degrees(kin):7.2f}  "
              f"Ackermann {math.degrees(exact[c][0]):7.2f} deg")
    xy = np.array(xy)
    A = np.c_[2 * xy, np.ones(len(xy))]
    sol = np.linalg.lstsq(A, (xy ** 2).sum(1), rcond=None)[0]
    R = math.sqrt(sol[2] + sol[0] ** 2 + sol[1] ** 2)
    R_geo = (P.WHEELBASE / 2) / math.tan(math.radians(delta))
    print(f"turn: fitted chassis-origin radius {R:.2f} m (centre {sol[0]:.2f},{sol[1]:.2f}); "
          f"double-Ackermann geometry R = (wheelbase/2)/tan(delta) = {R_geo:.2f} m")
    return {"settle_dz": z - z0, "R": R, "R_geo": R_geo}


def try_newton(path):
    try:
        import newton
        b = newton.ModelBuilder(up_axis=newton.Axis.Z)
        b.add_mjcf(path)
        model = b.finalize()
        print(f"newton add_mjcf OK: bodies={model.body_count} joints={model.joint_count} "
              f"dofs={model.joint_dof_count} shapes={model.shape_count} "
              f"equality={getattr(model, 'equality_constraint_count', '?')}")
        return True
    except Exception as e:  # report, don't fail the export
        import traceback
        traceback.print_exc()
        print(f"newton add_mjcf FAILED: {type(e).__name__}: {e}")
        return False


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--links", default=os.path.join(ROOT_DIR, "build/usd/full/links.json"))
    ap.add_argument("--mesh-src", default=os.path.join(ROOT_DIR, "build/urdf/meshes"))
    ap.add_argument("--out", default=os.path.join(ROOT_DIR, "build/mjcf"))
    ap.add_argument("--locks", choices=("weld", "stiff"), default="weld")
    ap.add_argument("--tex-size", type=int, default=512)
    ap.add_argument("--validate", action="store_true")
    ap.add_argument("--newton", action="store_true")
    a = ap.parse_args()

    graph = json.load(open(a.links))
    ensure_meshes(a.mesh_src, a.links)
    os.makedirs(a.out, exist_ok=True)
    assets = Assets(a.mesh_src, a.out, a.tex_size)
    root, info = build(graph, assets, a.locks)
    lrv = os.path.join(a.out, "lrv.xml")
    write_xml(root, lrv)
    scene = os.path.join(a.out, "lrv_scene.xml")
    write_xml(scene_xml(), scene)
    print(f"wrote {lrv}: {len(graph['links'])} bodies, {len(info['joints'])} joints, {len(info['actuators'])} "
          f"actuators, {len(info['loops'])} loops ({info.get('connects', 0)} connects), {len(assets.meshes)} meshes, "
          f"{len(assets.materials)} materials, {len(assets.textures)} textures; locks={a.locks}")
    if a.validate:
        validate(scene, graph["links"])
    if a.newton:
        try_newton(lrv)


if __name__ == "__main__":
    main()
