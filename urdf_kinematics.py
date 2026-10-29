"""URDF chain kinematics for the GEArm bridge.

Single source of truth for FK/IK on the PC side. The DH-table solver
(kinematics_host.c, built from the firmware's robot.c) is kept only as an
independent cross-check; when the two disagree the caller must stop, not
average them.

Frame convention matches the web viewer: the URDF is Z-up, so the whole chain is
carried under one root rotation S that maps (x, y, z) -> (x, z, -y), i.e. Rx(-90).
That makes every returned number the same coordinate the user sees on screen.
Pure stdlib: no numpy, so it runs anywhere the bridge runs.
"""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

JOINTS = 6

# Root rotation: URDF Z-up -> screen Y-up.
_S = [[1.0, 0.0, 0.0, 0.0],
      [0.0, 0.0, 1.0, 0.0],
      [0.0, -1.0, 0.0, 0.0],
      [0.0, 0.0, 0.0, 1.0]]


def _mat(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def _trans(x, y, z):
    return [[1.0, 0.0, 0.0, x], [0.0, 1.0, 0.0, y], [0.0, 0.0, 1.0, z], [0.0, 0.0, 0.0, 1.0]]


def _rpy(roll, pitch, yaw):
    """URDF rpy is roll-pitch-yaw about fixed axes: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cz, sz = math.cos(yaw), math.sin(yaw)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cx, sx = math.cos(roll), math.sin(roll)
    rz = [[cz, -sz, 0.0, 0.0], [sz, cz, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    ry = [[cy, 0.0, sy, 0.0], [0.0, 1.0, 0.0, 0.0], [-sy, 0.0, cy, 0.0], [0.0, 0.0, 0.0, 1.0]]
    rx = [[1.0, 0.0, 0.0, 0.0], [0.0, cx, -sx, 0.0], [0.0, sx, cx, 0.0], [0.0, 0.0, 0.0, 1.0]]
    return _mat(_mat(rz, ry), rx)


def _rot_axis(axis, angle):
    """Rotation about a unit axis, Rodrigues form (axis is always a unit vector)."""
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y, 0.0],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x, 0.0],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c, 0.0],
            [0.0, 0.0, 0.0, 1.0]]


def _decompose(m):
    pos = (m[0][3], m[1][3], m[2][3])
    # Upper-left 3x3 -> quaternion (Shepperd's method).
    r00, r01, r02 = m[0][0], m[0][1], m[0][2]
    r10, r11, r12 = m[1][0], m[1][1], m[1][2]
    r20, r21, r22 = m[2][0], m[2][1], m[2][2]
    trace = r00 + r11 + r22
    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        q = (0.25 / s, (r21 - r12) * s, (r02 - r20) * s, (r10 - r01) * s)
    elif r00 > r11 and r00 > r22:
        s = 2.0 * math.sqrt(1.0 + r00 - r11 - r22)
        q = ((r21 - r12) / s, 0.25 * s, (r01 + r10) / s, (r02 + r20) / s)
    elif r11 > r22:
        s = 2.0 * math.sqrt(1.0 + r11 - r00 - r22)
        q = ((r02 - r20) / s, (r01 + r10) / s, 0.25 * s, (r12 + r21) / s)
    else:
        s = 2.0 * math.sqrt(1.0 + r22 - r00 - r11)
        q = ((r10 - r01) / s, (r02 + r20) / s, (r12 + r21) / s, 0.25 * s)
    n = math.sqrt(sum(v * v for v in q))
    return pos, tuple(v / n for v in q)


class UrdfKinematics:
    """FK/IK on the URDF joint chain, expressed in the viewer's Y-up frame."""

    def __init__(self, urdf_path):
        root = ET.parse(Path(urdf_path)).getroot()
        self.links = {}
        for link in root.findall("link"):
            name = link.get("name")
            visual = link.find("visual")
            mesh = visual.find("mesh") if visual is not None else None
            color_node = visual.find("color") if visual is not None else None
            rgba = (color_node.get("rgba") if color_node is not None else None) or "0.75 0.75 0.75 1"
            parts = [float(v) for v in rgba.split()[:3]]
            self.links[name] = {
                "mesh": mesh.get("filename").split("/")[-1] if mesh is not None else None,
                "color": tuple(round(c * 255) for c in parts),
            }

        self.chain = []
        for joint in root.findall("joint"):
            if joint.get("type") not in ("revolute", "continuous"):
                continue
            origin = joint.find("origin")
            xyz = [float(v) for v in (origin.get("xyz") if origin is not None else "0 0 0").split()]
            rpy = [float(v) for v in (origin.get("rpy") if origin is not None else "0 0 0").split()]
            axis_attr = joint.find("axis")
            axis = [float(v) for v in (axis_attr.get("xyz") if axis_attr is not None else "0 0 1").split()]
            norm = math.sqrt(sum(v * v for v in axis)) or 1.0
            self.chain.append({
                "name": joint.get("name"),
                "parent": joint.find("parent").get("link"),
                "child": joint.find("child").get("link"),
                "origin": _trans(*xyz),
                "rpy": _rpy(*rpy),
                "axis": tuple(v / norm for v in axis),
                "limit": self._limit(joint),
            })
        if len(self.chain) != JOINTS:
            raise ValueError("expected %d movable joints, found %d" % (JOINTS, len(self.chain)))
        self.names = [j["name"] for j in self.chain]
        self.limits = [j["limit"] for j in self.chain]

    @staticmethod
    def _limit(joint):
        """Joint travel in degrees, or None when the URDF declares no limit.

        Continuous joints legitimately have none; a missing limit means
        "unknown", NOT "unbounded", so callers must treat None as unverified.
        """
        node = joint.find("limit")
        if node is None:
            return None
        lower, upper = node.get("lower"), node.get("upper")
        if lower is None or upper is None:
            return None
        return (math.degrees(float(lower)), math.degrees(float(upper)))

    def clamp(self, joints_deg):
        """Clamp to declared limits. Joints without a limit pass through."""
        out = []
        for value, limit in zip(joints_deg, self.limits):
            if limit is None:
                out.append(value)
            else:
                out.append(min(max(value, limit[0]), limit[1]))
        return out

    def limit_violations(self, joints_deg, tol=1e-6):
        """(index, angle, limit) for every joint outside its declared travel."""
        bad = []
        for i, (value, limit) in enumerate(zip(joints_deg, self.limits)):
            if limit is None:
                continue
            if value < limit[0] - tol or value > limit[1] + tol:
                bad.append((i, value, limit))
        return bad

    def fk(self, joints_deg):
        """joints_deg: 6 angles in degrees. Returns (pos_mm, quat) in the Y-up frame."""
        m = _S
        for angle_deg, joint in zip(joints_deg, self.chain):
            m = _mat(m, joint["origin"])
            m = _mat(m, joint["rpy"])
            m = _mat(m, _rot_axis(joint["axis"], math.radians(angle_deg)))
        pos, quat = _decompose(m)
        return (pos[0] * 1000.0, pos[1] * 1000.0, pos[2] * 1000.0), quat

    def joint_positions(self, joints_deg):
        """World position of every joint frame, in mm, Y-up. Used by the 2D view."""
        points = []
        m = _S
        for angle_deg, joint in zip(joints_deg, self.chain):
            m = _mat(m, joint["origin"])
            m = _mat(m, joint["rpy"])
            m = _mat(m, _rot_axis(joint["axis"], math.radians(angle_deg)))
            points.append([m[0][3] * 1000.0, m[1][3] * 1000.0, m[2][3] * 1000.0])
        return points

    def ik(self, target_mm, seed_deg=None, max_iter=300, tol_mm=1e-3):
        """Damped least squares with a few fixed restarts, mirroring the viewer.

        Returns (joints_deg, residual_mm, iterations, converged). seed_deg is the
        caller's current pose so a small move stays a small move.
        """
        target = (target_mm[0] / 1000.0, target_mm[1] / 1000.0, target_mm[2] / 1000.0)
        seeds = [list(seed_deg) if seed_deg else [0.0] * JOINTS,
                 [0.0] * JOINTS,
                 [17.19, 17.19, 17.19, 0.0, 0.0, 0.0],
                 [-17.19, -17.19, -17.19, 0.0, 0.0, 0.0],
                 [34.38, -17.19, 22.92, 0.0, 0.0, 0.0],
                 [-34.38, 17.19, -22.92, 0.0, 0.0, 0.0]]
        best = None
        for seed in seeds:
            result = self._ik_once(target, seed, max_iter, tol_mm)
            if best is None or result[1] < best[1]:
                best = result
            if best[3]:
                break
        return best

    def _ik_once(self, target, q, max_iter, tol_mm):
        lam = 1e-4
        h = 1e-6
        for iteration in range(max_iter):
            pos, _ = self.fk(q)
            err = [target[i] - pos[i] / 1000.0 for i in range(3)]
            norm = math.sqrt(sum(e * e for e in err))
            if norm * 1000.0 < tol_mm:
                return q[:], norm * 1000.0, iteration, True
            jac = []
            for j in range(JOINTS):
                saved = q[j]
                q[j] = saved + h
                bumped, _ = self.fk(q)
                q[j] = saved
                jac.append([(bumped[i] / 1000.0 - pos[i] / 1000.0) / h for i in range(3)])
            # Solve (JtJ + lam I) dq = Jt e with Gaussian elimination.
            a = [[sum(jac[r][i] * jac[c][i] for i in range(3)) + (lam if r == c else 0.0)
                  for c in range(JOINTS)] for r in range(JOINTS)]
            b = [sum(jac[r][i] * err[i] for i in range(3)) for r in range(JOINTS)]
            for col in range(JOINTS):
                pivot = max(range(col, JOINTS), key=lambda r: abs(a[r][col]))
                if abs(a[pivot][col]) < 1e-12:
                    return q[:], norm * 1000.0, -1, False
                if pivot != col:
                    a[col], a[pivot] = a[pivot], a[col]
                    b[col], b[pivot] = b[pivot], b[col]
                for r in range(col + 1, JOINTS):
                    factor = a[r][col] / a[col][col]
                    if not factor:
                        continue
                    for c in range(col, JOINTS):
                        a[r][c] -= factor * a[col][c]
                    b[r] -= factor * b[col]
            dq = [0.0] * JOINTS
            for r in range(JOINTS - 1, -1, -1):
                acc = b[r] - sum(a[r][c] * dq[c] for c in range(r + 1, JOINTS))
                dq[r] = acc / a[r][r]
            step = math.sqrt(sum(v * v for v in dq))
            scale = 0.15 / step if step > 0.15 else 1.0
            for r in range(JOINTS):
                q[r] += dq[r] * scale
        pos, _ = self.fk(q)
        residual = math.sqrt(sum((target[i] - pos[i] / 1000.0) ** 2 for i in range(3))) * 1000.0
        return q[:], residual, max_iter, residual < tol_mm

    def roundtrip(self, samples=80, seed=1234):
        """FK -> IK -> FK agreement over random poses. Returns (worst_mm, failures)."""
        import random
        rng = random.Random(seed)
        worst = 0.0
        failures = 0
        for _ in range(samples):
            pose = [rng.uniform(-90.0, 90.0) for _ in range(JOINTS)]
            target, _ = self.fk(pose)
            solved, residual, _, converged = self.ik(target, seed_deg=pose)
            if not converged:
                failures += 1
                continue
            back, _ = self.fk(solved)
            delta = math.sqrt(sum((target[i] - back[i]) ** 2 for i in range(3)))
            worst = max(worst, delta)
        return worst, failures
