"""Drive the LRV USD in Newton (SolverMuJoCo, lunar gravity).

Run: .venv/bin/python scripts/sim_newton.py [--usd build/usd/lrv.usda] [--viewer gl|null]
       [--record build/sim/run.npz] [--script settle_drive_turn]

Steering uses the LRV's double-Ackermann law (front and rear sets turn opposite, turn
centre on the lateral axis through the middle of the wheelbase). Wheel speeds follow the
turn geometry so the drive servos don't fight each other.
"""
import argparse
import math
import os
import sys

import numpy as np
import warp as wp

import newton
from newton.solvers import SolverMuJoCo

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import params as P  # noqa: E402


def ackermann(delta):
    """Kingpin angles per corner and relative wheel speeds for a virtual steer angle `delta`."""
    out = {}
    if abs(delta) < 1e-4:
        return {c: (0.0, 1.0) for c in P.CORNERS}
    R = (P.WHEELBASE / 2) / math.tan(delta)  # signed, +left: centre at (0, R)
    for c, (sx, sy) in P.CORNERS.items():
        x, y = sx * P.WHEELBASE / 2, sy * P.KINGPIN_Y
        ang = math.atan2(x, R - y) if R > 0 else -math.atan2(x, y - R)
        ang = max(-P.STEER_LIMIT, min(P.STEER_LIMIT, ang))
        wy = sy * P.TRACK / 2
        out[c] = (ang, math.hypot(x, R - wy) / abs(R))
    return out


SCRIPTS = {
    # (t_start, speed km/h, virtual steer deg)
    "settle_drive_turn": [(0.0, 0, 0), (2.0, 6, 0), (7.0, 6, 18), (13.0, 6, -18), (19.0, 0, 0), (21.0, None, None)],
    "settle": [(0.0, 0, 0), (4.0, None, None)],
}


class Sim:
    def __init__(self, usd, viewer):
        b = newton.ModelBuilder(up_axis=newton.Axis.Z, gravity=(0.0, 0.0, -P.G_MOON))
        SolverMuJoCo.register_custom_attributes(b)
        b.default_shape_cfg.mu = 0.8
        b.default_shape_cfg.ke = 2.0e4
        b.default_shape_cfg.kd = 5.0e2
        from pxr import Usd
        stage = Usd.Stage.Open(usd)
        root = stage.GetDefaultPrim()
        if root.GetVariantSets().HasVariantSet("Physics"):   # packaged asset: neutral UsdPhysics layer
            root.GetVariantSet("Physics").SetVariantSelection("physics")
        b.add_usd(stage, collapse_fixed_joints=False, enable_self_collisions=False, hide_collision_shapes=True)
        b.joint_q[2] = 0.03  # drop in from just above the ground
        b.add_ground_plane()
        self.model = b.finalize()
        self.labels = [s.rsplit("/", 1)[-1] for s in b.joint_label]
        qd_start = self.model.joint_qd_start.numpy()
        q_start = self.model.joint_q_start.numpy()
        self.dof = {n: int(qd_start[i]) for i, n in enumerate(self.labels)}
        self.coord = {n: int(q_start[i]) for i, n in enumerate(self.labels)}

        self.solver = SolverMuJoCo(self.model, iterations=100, ls_iterations=50, cone="elliptic", impratio=10,
                                   nconmax=64, njmax=400)
        self.s0, self.s1 = self.model.state(), self.model.state()
        self.control = self.model.control()
        newton.eval_fk(self.model, self.model.joint_q, self.model.joint_qd, self.s0)
        self.pipe = newton.CollisionPipeline(self.model)
        self.contacts = self.pipe.contacts()
        self.fps, self.sub = 60, 8
        self.dt = 1.0 / self.fps / self.sub
        self.t = 0.0
        self.viewer = viewer
        if viewer:
            viewer.set_model(self.model)
        self.tpos = self.control.joint_target_q.numpy()
        self.tvel = self.control.joint_target_qd.numpy()

    def command(self, speed_kmh, steer_deg):
        w = speed_kmh / 3.6 / P.WHEEL_RADIUS
        # the kingpins are passive: the front/aft sectors drive them through the tie rods
        for sector in ("fwd_steer_sector", "aft_steer_sector"):
            self.tpos[self.coord[sector]] = P.steer_sector_angle(math.radians(steer_deg))
        for c, (_ang, ratio) in ackermann(math.radians(steer_deg)).items():
            self.tvel[self.dof[f"{c}_wheel"]] = w * ratio
        self.control.joint_target_q.assign(self.tpos)
        self.control.joint_target_qd.assign(self.tvel)

    def step_frame(self):
        self.pipe.collide(self.s0, self.contacts)
        for _ in range(self.sub):
            self.s0.clear_forces()
            self.solver.step(self.s0, self.s1, self.control, self.contacts, self.dt)
            self.s0, self.s1 = self.s1, self.s0
        self.t += 1.0 / self.fps
        if self.viewer:
            self.viewer.begin_frame(self.t)
            self.viewer.log_state(self.s0)
            self.viewer.end_frame()

    def report(self):
        body_q = self.s0.body_q.numpy()
        q = self.s0.joint_q.numpy() if hasattr(self.s0, "joint_q") else None
        x, y, z = body_q[0][:3]
        qx, qy, qz, qw = body_q[0][3:7]
        yaw = math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))
        v = np.linalg.norm(self.s0.body_qd.numpy()[0][:3])
        arms = steer = ""
        if q is not None:
            arms = " ".join(f"{math.degrees(q[self.coord[f'{c}_lower_arm']]):6.1f}" for c in P.CORNERS)
            steer = " ".join(f"{math.degrees(q[self.coord[f'{c}_steer']]):6.1f}" for c in P.CORNERS)
        print(f"t={self.t:5.2f} pos=({x:6.2f},{y:6.2f},{z:5.3f}) yaw={yaw:7.1f} v={v * 3.6:5.2f}km/h "
              f"arms[lf rf lr rr]={arms} steer={steer}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", default="build/asset/lrv/lrv.usda")
    ap.add_argument("--viewer", default="null")
    ap.add_argument("--script", default="settle_drive_turn")
    ap.add_argument("--record", default="")
    a = ap.parse_args()
    viewer = newton.viewer.ViewerGL() if a.viewer == "gl" else None
    sim = Sim(a.usd, viewer)
    plan = SCRIPTS[a.script]
    frames = []
    k = 0
    while True:
        while k + 1 < len(plan) and sim.t >= plan[k + 1][0]:
            k += 1
        speed, steer = plan[k][1], plan[k][2]
        if speed is None:
            break
        sim.command(speed, steer)
        sim.step_frame()
        if a.record:
            frames.append(sim.s0.body_q.numpy().copy())
        if round(sim.t * sim.fps) % 30 == 0:
            sim.report()
    if a.record:
        os.makedirs(os.path.dirname(os.path.abspath(a.record)), exist_ok=True)
        np.savez_compressed(a.record, body_q=np.array(frames), fps=sim.fps,
                            bodies=np.array([s.rsplit("/", 1)[-1] for s in sim.model.body_label]))
        print("recorded", len(frames), "frames ->", a.record)


if __name__ == "__main__":
    main()
