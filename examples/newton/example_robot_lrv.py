# SPDX-FileCopyrightText: Copyright (c) 2025 The Newton Developers
# SPDX-License-Identifier: Apache-2.0

###########################################################################
# Example Robot LRV
#
# Apollo 17 Lunar Roving Vehicle (LRV-3) loaded from USD with
# newton.ModelBuilder.add_usd(). Each corner is a real double-wishbone four-bar
# (loop joint) on torsion-bar springs with a sliding damper; the four wheel
# drives are velocity-controlled. Steering is the real double-Ackermann linkage:
# a position-controlled sector per axle drives two ball-jointed tie rods (loop
# joints) to the passive kingpins; front and rear turn opposite, turn centre on
# the lateral axis midway between the axles.
# The T-handle hand controller follows the commands, and the high-gain antenna,
# TV camera and fender extensions can be posed from the GUI.
#
# Keys: i/k throttle, j/l steer, space stop (arrows/WASD are the camera).
#
# Command (from the lrv repo root):
#   python examples/newton/example_robot_lrv.py --speed 6   (uses the committed assets/lrv/lrv.usda)
#
# Stock Newton loads and simulates the asset. For full material display in the
# GL viewer (opacity cut-outs, double-sided meshes, roughness/metallic), apply
# newton_viewer_materials.patch to a Newton checkout.
#
###########################################################################

import math
import os

import numpy as np
import warp as wp

import newton
import newton.examples
from newton.sensors import SensorTiledCamera

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
LRV_USD = os.environ.get("LRV_USD", os.path.join(REPO, "assets", "lrv", "lrv.usda"))

# Vehicle geometry for the steering law (A17 Systems Handbook / Ops Handbook LS006-002-2H)
WHEELBASE = 2.286  # [m] 90 in
KINGPIN_Y = 0.808  # [m] kingpin lateral offset
TRACK = 1.829  # [m] 72 in, wheel centre to wheel centre
WHEEL_RADIUS = 0.409  # [m] 32.2 in tire
STEER_LIMIT = math.radians(50.0)  # inner wheel; outer reaches ~22-23 deg
STEER_SECTOR_RATIO = 1.40  # sector angle per virtual steer angle (lrv scripts/params.py)
MAX_SPEED = 14.0  # [km/h] level-ground design maximum
G_MOON = 1.62
CORNERS = {"lf": (1, 1), "rf": (1, -1), "lr": (-1, 1), "rr": (-1, -1)}


def ackermann(delta):
    """Kingpin angles (exact law) and relative wheel speeds for a virtual (bicycle) steer angle ``delta``.

    The kingpins are steered by the linkage; only the wheel-speed ratios are commanded from this.
    """
    if abs(delta) < 1e-4:
        return {c: (0.0, 1.0) for c in CORNERS}
    radius = (WHEELBASE / 2) / math.tan(delta)  # signed, +left: turn centre at (0, radius)
    out = {}
    for c, (sx, sy) in CORNERS.items():
        x, y = sx * WHEELBASE / 2, sy * KINGPIN_Y
        ang = math.atan2(x, radius - y) if radius > 0 else -math.atan2(x, y - radius)
        ang = max(-STEER_LIMIT, min(STEER_LIMIT, ang))
        out[c] = (ang, math.hypot(x, radius - sy * TRACK / 2) / abs(radius))
    return out


class Example:
    def __init__(self, viewer, args):
        self.fps = 60
        self.frame_dt = 1.0 / self.fps
        self.sim_time = 0.0
        self.sim_substeps = 8
        self.sim_dt = self.frame_dt / self.sim_substeps
        self.world_count = args.world_count
        self.viewer = viewer

        self.speed = float(np.clip(args.speed, -MAX_SPEED, MAX_SPEED))  # [km/h]
        self.steer = float(np.radians(args.steer))  # virtual steer [rad]
        self.poses = {"hga_az": 0.0, "hga_el": 0.0, "tv_pan": 0.0, "tv_camera": 0.0, "fender_ext": 0.0}

        # targets indexed like joint_q (the root is a FREE joint, so coords != dofs)
        newton.use_coord_layout_targets = True
        g = 9.81 if args.earth else G_MOON
        lrv = newton.ModelBuilder(up_axis=newton.Axis.Z, gravity=(0.0, 0.0, -g))
        newton.solvers.SolverMuJoCo.register_custom_attributes(lrv)
        lrv.default_shape_cfg.mu = 0.8  # wire-mesh tire on regolith
        lrv.default_shape_cfg.ke = 2.0e4
        lrv.default_shape_cfg.kd = 5.0e2
        # The asset follows the Isaac Sim layout: "physics" is the neutral UsdPhysics layer; Isaac's
        # "physx"/"mujoco" variants add engine-specific schemas on top of it (MjcActuator etc.).
        from pxr import Usd

        stage = Usd.Stage.Open(args.usd)
        root = stage.GetDefaultPrim()
        for set_name, choice in (("Physics", args.physics), ("Fidelity", args.fidelity), ("Mission", args.mission),
                                 ("WheelCollider", args.wheel_collider), ("BodyColliders", args.body_colliders),
                                 ("Sensor", "none" if args.no_camera else "all")):
            if root.GetVariantSets().HasVariantSet(set_name):
                root.GetVariantSet(set_name).SetVariantSelection(choice)
        lrv.add_usd(
            stage,
            xform=wp.transform(wp.vec3(0.0, 0.0, 0.05), wp.quat_identity()),
            collapse_fixed_joints=False,
            enable_self_collisions=False,
            hide_collision_shapes=True,
        )

        self.cameras = self._find_cameras(stage, lrv)
        self.bodies_per_world = lrv.body_count

        self.labels = [s.rsplit("/", 1)[-1] for s in lrv.joint_label]
        self.wheel_dofs = {c: self._dof(lrv, f"{c}_wheel") for c in CORNERS}
        # both sectors take the same command (+ = left); the rear sector's axis is -Z
        self.sector_coords = [self._coord(lrv, f"{end}_steer_sector") for end in ("fwd", "aft")]
        self.hc = {k: self._coord(lrv, k) for k in ("hc_steer", "hc_throttle") if k in self.labels}
        self.pose_coords = {}
        for key in self.poses:
            names = [n for n in self.labels if n == key or n.endswith(key)]
            self.pose_coords[key] = [self._coord(lrv, n) for n in names]
        self.coords_per_world = lrv.joint_coord_count
        self.dofs_per_world = lrv.joint_dof_count

        builder = newton.ModelBuilder(up_axis=newton.Axis.Z, gravity=(0.0, 0.0, -g))
        builder.replicate(lrv, self.world_count)
        builder.add_ground_plane(color=(0.42, 0.41, 0.39))  # regolith grey for the camera sensors
        self.model = builder.finalize()

        self.solver = newton.solvers.SolverMuJoCo(
            self.model,
            iterations=100,
            ls_iterations=50,
            cone="elliptic",
            impratio=10,
            njmax=400,
            nconmax=64,
        )
        self.state_0 = self.model.state()
        self.state_1 = self.model.state()
        self.control = self.model.control()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.state_0)
        self.collision_pipeline = newton.CollisionPipeline(self.model)
        self.contacts = self.collision_pipeline.contacts()

        self._target_q = self.control.joint_target_q.numpy()
        self._target_qd = self.control.joint_target_qd.numpy()
        self._write_targets()

        self.viewer.set_model(self.model)
        self._setup_camera_sensor(args)
        self.viewer.set_world_offsets((6.0, 6.0, 0.0))
        print(f"lrv: {self.speed:+.1f} km/h, steer {math.degrees(self.steer):+.0f} deg (i/k throttle, j/l steer, space stop)")
        self.capture()

    # -------------------------------------------------------------- joint lookup
    def _joint(self, builder, name):
        return self.labels.index(name)

    def _dof(self, builder, name):
        return builder.joint_qd_start[self._joint(builder, name)]

    def _coord(self, builder, name):
        return builder.joint_q_start[self._joint(builder, name)]

    # -------------------------------------------------------------- camera sensor
    @staticmethod
    def _find_cameras(stage, builder):
        """USD cameras in the asset, each with its owning body and its pose in that body's frame.

        The sensor renders from body_q * offset every frame, so a camera rides whatever joints
        sit between it and the chassis (the GCTA camera follows its pan and tilt drives).
        """
        from pxr import Gf, Usd, UsdGeom

        body_paths = {label: i for i, label in enumerate(builder.body_label)}
        cache = UsdGeom.XformCache(Usd.TimeCode.Default())
        cams = []
        for prim in Usd.PrimRange(stage.GetDefaultPrim()):
            if not prim.IsA(UsdGeom.Camera):
                continue
            body, parent = None, prim.GetParent()
            while parent and parent.GetPath() != parent.GetPath().absoluteRootPath:
                if str(parent.GetPath()) in body_paths:
                    body = parent
                    break
                parent = parent.GetParent()
            if body is None:
                continue
            local = cache.GetLocalToWorldTransform(prim) * cache.GetLocalToWorldTransform(body).GetInverse()
            t = local.ExtractTranslation()
            q = local.RemoveScaleShear().ExtractRotationQuat()
            cams.append({
                "name": prim.GetName(),
                "prim": UsdGeom.Camera(prim),
                "body": body_paths[str(body.GetPath())],
                "offset": wp.transformf(wp.vec3f(*t), wp.quatf(*q.GetImaginary(), q.GetReal())),
                "aperture": (UsdGeom.Camera(prim).GetHorizontalApertureAttr().Get(),
                             UsdGeom.Camera(prim).GetVerticalApertureAttr().Get()),
                "focal": UsdGeom.Camera(prim).GetFocalLengthAttr().Get(),
                # same units as focalLength (USD lens units: tenths of a scene unit); 6:1 zoom fallback
                "focal_range": tuple(prim.GetAttribute("lrv:focalLengthRange").Get()
                                     or (UsdGeom.Camera(prim).GetFocalLengthAttr().Get(),
                                         6.0 * UsdGeom.Camera(prim).GetFocalLengthAttr().Get())),
            })
        return cams

    def _setup_camera_sensor(self, args):
        self.sensor = None
        if not self.cameras or args.no_camera:
            return
        self.cam_w, self.cam_h = args.camera_width, args.camera_height
        self.sensor = SensorTiledCamera(model=self.model)
        self.sensor.default_render_config.enable_shadows = True
        self.sensor.default_render_config.enable_textures = True
        self.sensor.utils.create_default_light(enable_shadows=True)
        n = len(self.cameras)
        self.cam_color = self.sensor.utils.create_color_image_output(self.cam_w, self.cam_h, n)
        self.cam_depth = self.sensor.utils.create_depth_image_output(self.cam_w, self.cam_h, n)
        self.cam_depth_rgba = wp.empty((self.world_count * n, self.cam_h, self.cam_w, 4), dtype=wp.uint8)
        self.cam_rays = self.sensor.utils.compute_camera_rays_usd_pinhole(
            self.cam_w, self.cam_h, [c["prim"] for c in self.cameras]
        )
        self.zoom = [c["focal"] for c in self.cameras]
        self.camera_main_view = False
        self.camera_every = max(1, args.camera_every)
        self._frame = 0

    def _update_zoom(self):
        """Recompute rays for the current focal lengths (the aperture stays the USD film back)."""
        fovs = [2.0 * math.atan(0.5 * c["aperture"][1] / f) for c, f in zip(self.cameras, self.zoom)]
        self.cam_rays = self.sensor.utils.compute_camera_rays_pinhole(
            self.cam_w, self.cam_h, camera_fovs=wp.array(fovs, dtype=wp.float32), out_rays=self.cam_rays
        )

    def _camera_transforms(self):
        body_q = self.state_0.body_q.numpy()
        rows = []
        for c in self.cameras:
            row = []
            for w in range(self.world_count):
                q = body_q[w * self.bodies_per_world + c["body"]]
                x_body = wp.transformf(wp.vec3f(*q[:3]), wp.quatf(*q[3:7]))
                row.append(wp.transform_multiply(x_body, c["offset"]))
            rows.append(row)
        return wp.array(rows, dtype=wp.transformf)

    def render_cameras(self):
        self.model.bvh_refit_shapes(self.state_0)
        self.sensor.update(
            self.state_0,
            self._camera_transforms(),
            self.cam_rays,
            color_image=self.cam_color,
            depth_image=self.cam_depth,
            clear_data=SensorTiledCamera.GRAY_CLEAR_DATA,
        )
        utils = self.sensor.utils
        utils.to_rgba_from_depth(self.cam_depth, depth_range=(0.0, 30.0), out_buffer=self.cam_depth_rgba)
        color = utils.to_rgba_from_color(self.cam_color)
        for i, c in enumerate(self.cameras):
            self.viewer.log_image(c["name"], color[i :: len(self.cameras)])
            self.viewer.log_image(c["name"] + "_depth", self.cam_depth_rgba[i :: len(self.cameras)])
        if self.camera_main_view:
            self.viewer.log_image(self.cameras[0]["name"], color[0:1], fullscreen=True)

    # -------------------------------------------------------------- commands
    def _write_targets(self):
        w = self.speed / 3.6 / WHEEL_RADIUS
        cmd = ackermann(self.steer)
        for world in range(self.world_count):
            qo, do = world * self.coords_per_world, world * self.dofs_per_world
            for q in self.sector_coords:
                self._target_q[qo + q] = STEER_SECTOR_RATIO * self.steer
            for c, (_ang, ratio) in cmd.items():
                self._target_qd[do + self.wheel_dofs[c]] = w * ratio
            # T-handle mirrors the commands: +-14 deg travel at full throttle / full steer
            if "hc_throttle" in self.hc:
                self._target_q[qo + self.hc["hc_throttle"]] = math.radians(14.0) * self.speed / MAX_SPEED
            if "hc_steer" in self.hc:
                self._target_q[qo + self.hc["hc_steer"]] = -math.radians(14.0) * self.steer / math.radians(25.0)
            for key, coords in self.pose_coords.items():
                for q in coords:
                    self._target_q[qo + q] = self.poses[key]
        self.control.joint_target_q.assign(self._target_q)
        self.control.joint_target_qd.assign(self._target_qd)

    def _poll_keyboard(self):
        v = self.viewer
        if v.is_key_down("i"):
            self.speed = min(MAX_SPEED, self.speed + 6.0 * self.frame_dt)
        if v.is_key_down("k"):
            self.speed = max(-MAX_SPEED, self.speed - 6.0 * self.frame_dt)
        if v.is_key_down("space"):
            self.speed = 0.0
        if v.is_key_down("j"):
            self.steer = min(math.radians(25.0), self.steer + 0.8 * self.frame_dt)
        if v.is_key_down("l"):
            self.steer = max(-math.radians(25.0), self.steer - 0.8 * self.frame_dt)

    # -------------------------------------------------------------- sim loop
    def capture(self):
        self.graph = None
        if wp.get_device().is_cuda:
            with wp.ScopedCapture() as capture:
                self.simulate()
            self.graph = capture.graph

    def simulate(self):
        self.collision_pipeline.collide(self.state_0, self.contacts)
        for _ in range(self.sim_substeps):
            self.state_0.clear_forces()
            self.viewer.apply_forces(self.state_0)
            self.solver.step(self.state_0, self.state_1, self.control, self.contacts, self.sim_dt)
            self.state_0, self.state_1 = self.state_1, self.state_0

    def step(self):
        self._poll_keyboard()
        self._write_targets()
        if self.graph:
            wp.capture_launch(self.graph)
        else:
            self.simulate()
        self.sim_time += self.frame_dt

    def render(self):
        if self.sensor is not None:
            self._frame += 1
            if self._frame % self.camera_every == 0:
                self.render_cameras()
        self.viewer.begin_frame(self.sim_time)
        self.viewer.log_state(self.state_0)
        self.viewer.log_contacts(self.contacts, self.state_0)
        self.viewer.end_frame()

    def gui(self, ui):
        if self.sensor is not None:
            for i, c in enumerate(self.cameras):
                lo, hi = c["focal_range"]
                ui.text(f"{c['name']} zoom (mm)")
                mm = 100.0  # USD lens units (tenths of a metre) -> mm
                changed, f = ui.slider_float(f"##zoom_{i}", float(self.zoom[i]) * mm, float(lo) * mm, float(hi) * mm)
                f /= mm
                if changed:
                    self.zoom[i] = f
                    self._update_zoom()
            _, self.camera_main_view = ui.checkbox("TV camera as main view", self.camera_main_view)
        ui.text("Speed (km/h)")
        changed, value = ui.slider_float("##speed", self.speed, -MAX_SPEED, MAX_SPEED)
        if changed:
            self.speed = value
        ui.text("Steer (deg, virtual)")
        changed, deg = ui.slider_float("##steer", float(np.degrees(self.steer)), -25.0, 25.0)
        if changed:
            self.steer = float(np.radians(deg))
        for key, label, lo, hi in (
            ("hga_az", "HGA azimuth", -180.0, 180.0),
            ("hga_el", "HGA elevation", -50.0, 40.0),
            ("tv_pan", "TV pan", -134.0, 214.0),
            ("tv_camera", "TV tilt", -45.0, 85.0),
            ("fender_ext", "Fender extension", -50.0, 50.0),
        ):
            if not self.pose_coords.get(key):
                continue
            ui.text(label + " (deg)")
            changed, deg = ui.slider_float(f"##{key}", float(np.degrees(self.poses[key])), lo, hi)
            if changed:
                self.poses[key] = float(np.radians(deg))

    def test_final(self):
        newton.examples.test_body_state(
            self.model, self.state_0, "all bodies are above the ground", lambda q, qd: q[2] > -0.05
        )
        if self.sensor is not None:
            self.render_cameras()
            img = self.cam_color.numpy().view(np.uint8)
            assert img.max() > img.min(), "TV camera image is flat"
        body_q = self.state_0.body_q.numpy()
        print(f"lrv final chassis position {np.round(body_q[0][:3], 3)}, t = {self.sim_time:.1f} s")

    @staticmethod
    def create_parser():
        parser = newton.examples.create_parser()
        newton.examples.add_world_count_arg(parser)
        parser.add_argument("--usd", default=LRV_USD, help="Path to the LRV USD (env LRV_USD).")
        parser.add_argument("--physics", default="physics", choices=["physics", "physx", "mujoco"], help="USD Physics variant.")
        parser.add_argument("--speed", type=float, default=0.0, help="Initial speed command [km/h].")
        parser.add_argument("--steer", type=float, default=0.0, help="Initial virtual steer angle [deg].")
        parser.add_argument("--earth", action="store_true", help="Earth gravity (the LRV can't carry itself!).")
        parser.add_argument("--no-camera", action="store_true", help="Disable the ray-traced USD camera sensors.")
        parser.add_argument("--camera-width", type=int, default=640, help="Camera sensor width [px] (NTSC 4:3).")
        parser.add_argument("--camera-height", type=int, default=480, help="Camera sensor height [px].")
        parser.add_argument("--camera-every", type=int, default=2, help="Render the camera sensors every N frames.")
        parser.add_argument("--fidelity", default="full", choices=["full", "proxy"], help="Visual detail variant.")
        parser.add_argument("--mission", default="a17", choices=["a15", "a17"], help="Mission configuration variant.")
        parser.add_argument("--wheel-collider", default="mesh", choices=["mesh", "cylinder"],
                            help="Wheel collision shape: convex hull of tire + chevrons, or analytic cylinder.")
        parser.add_argument("--body-colliders", default="full", choices=["full", "chassis", "wheels_only"],
                            help="Non-wheel collision: per-part convex hulls, chassis frame boxes, or none.")
        parser.set_defaults(world_count=1)
        return parser


if __name__ == "__main__":
    parser = Example.create_parser()
    viewer, args = newton.examples.init(parser)
    newton.examples.run(Example(viewer, args), args)
