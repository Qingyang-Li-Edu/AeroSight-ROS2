#!/usr/bin/env python3
"""
Cartesian pose control for G1 in RViz: target left/right hand pose in **pelvis** frame.

Uses analytical FK (PyKDL, URDF-derived chain) + numerical IK (NumPy only).

Usage:
  source /opt/ros/humble/setup.bash
  # Stop joint_state_publisher_gui first — otherwise it keeps overwriting /joint_states and the robot will not stay moved.
  /usr/bin/python3 -m g1_demos.cartesian --left 0.35 0.25 0.05 -90 0 0

Pose is position x y z (m) and orientation roll pitch yaw.
Default angle unit is degrees (--rpy-rad for radians).

Interactive:
  /usr/bin/python3 -m g1_demos.cartesian
  cart> left 0.35 0.22 0.02 -90 10 0
"""

from __future__ import annotations

import sys

import argparse
import contextlib
import math
import threading
import time
from typing import Dict, List, Sequence, Tuple

import numpy as np

from g1_demos.common import JOINT_NAMES  # also performs the Python/ROS environment check
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    import PyKDL as kdl
except ImportError as exc:
    print(
        "错误: 未找到 PyKDL 模块。\n"
        f"当前解释器: {sys.executable}\n"
        "可选用以下一种方式：\n"
        "  1) 使用系统 Python（与 apt 的 python3-pykdl 配套）:\n"
        "     source /opt/ros/humble/setup.bash\n"
        "     /usr/bin/python3 -m g1_demos.cartesian\n"
        "  2) 在 conda 环境中安装: conda install -c conda-forge python-orocos-kdl\n"
        "  3) 在 Ubuntu 上: sudo apt install python3-pykdl\n"
        f"原错误: {exc}"
    )
    raise SystemExit(1) from exc


WAIST_JOINTS = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")

LEFT_ARM_JOINTS = [
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
]

RIGHT_ARM_JOINTS = [
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
]

# URDF: pelvis -> torso via waist (axes Z, X, Y)
_WAIST_SEGMENTS = [
    {"xyz": (0.0, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0), "axis": (0.0, 0.0, 1.0)},  # yaw
    {"xyz": (-0.0039635, 0.0, 0.044), "rpy": (0.0, 0.0, 0.0), "axis": (1.0, 0.0, 0.0)},  # roll
    {"xyz": (0.0, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0), "axis": (0.0, 1.0, 0.0)},  # pitch
]

# torso_link -> left_hand_point_contact / right_hand_point_contact (revolute + fixed tail)
_LEFT_CHAIN = [
    {"xyz": (0.0039563, 0.10022, 0.24778), "rpy": (0.27931, 5.4949e-05, -0.00019159), "axis": (0.0, 1.0, 0.0)},
    {"xyz": (0.0, 0.038, -0.013831), "rpy": (-0.27925, 0.0, 0.0), "axis": (1.0, 0.0, 0.0)},
    {"xyz": (0.0, 0.00624, -0.1032), "rpy": (0.0, 0.0, 0.0), "axis": (0.0, 0.0, 1.0)},
    {"xyz": (0.015783, 0.0, -0.080518), "rpy": (0.0, 0.0, 0.0), "axis": (0.0, 1.0, 0.0)},
    {"xyz": (0.100, 0.00188791, -0.010), "rpy": (0.0, 0.0, 0.0), "axis": (1.0, 0.0, 0.0)},
    {"xyz": (0.038, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0), "axis": (0.0, 1.0, 0.0)},
    {"xyz": (0.046, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0), "axis": (0.0, 0.0, 1.0)},
]
_LEFT_FIXED = [
    {"xyz": (0.0415, 0.003, 0.0), "rpy": (0.0, 0.0, 0.0)},
    {"xyz": (0.05, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0)},
]

_RIGHT_CHAIN = [
    {"xyz": (0.0039563, -0.10021, 0.24778), "rpy": (-0.27931, 5.4949e-05, 0.00019159), "axis": (0.0, 1.0, 0.0)},
    {"xyz": (0.0, -0.038, -0.013831), "rpy": (0.27925, 0.0, 0.0), "axis": (1.0, 0.0, 0.0)},
    {"xyz": (0.0, -0.00624, -0.1032), "rpy": (0.0, 0.0, 0.0), "axis": (0.0, 0.0, 1.0)},
    {"xyz": (0.015783, 0.0, -0.080518), "rpy": (0.0, 0.0, 0.0), "axis": (0.0, 1.0, 0.0)},
    {"xyz": (0.100, -0.00188791, -0.010), "rpy": (0.0, 0.0, 0.0), "axis": (1.0, 0.0, 0.0)},
    {"xyz": (0.038, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0), "axis": (0.0, 1.0, 0.0)},
    {"xyz": (0.046, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0), "axis": (0.0, 0.0, 1.0)},
]
_RIGHT_FIXED = [
    {"xyz": (0.0415, -0.003, 0.0), "rpy": (0.0, 0.0, 0.0)},
    {"xyz": (0.05, 0.0, 0.0), "rpy": (0.0, 0.0, 0.0)},
]

_JOINT_LIMITS_RAD: Dict[str, Tuple[float, float]] = {
    "waist_yaw_joint": (-2.618, 2.618),
    "waist_roll_joint": (-0.52, 0.52),
    "waist_pitch_joint": (-0.52, 0.52),
    "left_shoulder_pitch_joint": (-3.0892, 2.6704),
    "left_shoulder_roll_joint": (-1.5882, 2.2515),
    "left_shoulder_yaw_joint": (-2.618, 2.618),
    "left_elbow_joint": (-1.0472, 2.0944),
    "left_wrist_roll_joint": (-1.972222054, 1.972222054),
    "left_wrist_pitch_joint": (-1.614429558, 1.614429558),
    "left_wrist_yaw_joint": (-1.614429558, 1.614429558),
    "right_shoulder_pitch_joint": (-3.0892, 2.6704),
    "right_shoulder_roll_joint": (-2.2515, 1.5882),
    "right_shoulder_yaw_joint": (-2.618, 2.618),
    "right_elbow_joint": (-1.0472, 2.0944),
    "right_wrist_roll_joint": (-1.972222054, 1.972222054),
    "right_wrist_pitch_joint": (-1.614429558, 1.614429558),
    "right_wrist_yaw_joint": (-1.614429558, 1.614429558),
}


def _frame_from_rpy_xyz(rpy: Sequence[float], xyz: Sequence[float]) -> kdl.Frame:
    return kdl.Frame(
        kdl.Rotation.RPY(float(rpy[0]), float(rpy[1]), float(rpy[2])),
        kdl.Vector(float(xyz[0]), float(xyz[1]), float(xyz[2])),
    )


def _fk_revolute_chain(segments: List[dict], q: Sequence[float], fixed_tail: List[dict]) -> kdl.Frame:
    t = kdl.Frame.Identity()
    for i, seg in enumerate(segments):
        f0 = _frame_from_rpy_xyz(seg["rpy"], seg["xyz"])
        axis = kdl.Vector(float(seg["axis"][0]), float(seg["axis"][1]), float(seg["axis"][2]))
        fq = kdl.Frame(kdl.Rotation.Rot(axis, float(q[i])))
        t = t * f0 * fq
    for seg in fixed_tail:
        t = t * _frame_from_rpy_xyz(seg["rpy"], seg["xyz"])
    return t


def fk_waist(q_waist: Sequence[float]) -> kdl.Frame:
    return _fk_revolute_chain(_WAIST_SEGMENTS, q_waist, [])


def fk_arm_torso(side: str, q_arm: Sequence[float]) -> kdl.Frame:
    if side == "left":
        return _fk_revolute_chain(_LEFT_CHAIN, q_arm, _LEFT_FIXED)
    if side == "right":
        return _fk_revolute_chain(_RIGHT_CHAIN, q_arm, _RIGHT_FIXED)
    raise ValueError("side must be 'left' or 'right'")


def fk_hand_in_pelvis(
    q_waist: Sequence[float], side: str, q_arm: Sequence[float], positions: Dict[str, float]
) -> kdl.Frame:
    """FK using waist + arm; legs ignored (pelvis = root in URDF)."""
    _ = positions  # reserved if later we add floating base
    t_base_torso = fk_waist(q_waist)
    t_torso_ee = fk_arm_torso(side, q_arm)
    return t_base_torso * t_torso_ee


def kdl_frame_to_numpy(f: kdl.Frame) -> np.ndarray:
    m = np.eye(4)
    for i in range(3):
        for j in range(3):
            m[i, j] = f.M[i, j]
    m[0, 3], m[1, 3], m[2, 3] = f.p[0], f.p[1], f.p[2]
    return m


def numpy_to_kdl_frame(m: np.ndarray) -> kdl.Frame:
    r = kdl.Rotation(
        float(m[0, 0]),
        float(m[0, 1]),
        float(m[0, 2]),
        float(m[1, 0]),
        float(m[1, 1]),
        float(m[1, 2]),
        float(m[2, 0]),
        float(m[2, 1]),
        float(m[2, 2]),
    )
    return kdl.Frame(r, kdl.Vector(float(m[0, 3]), float(m[1, 3]), float(m[2, 3])))


def pose_from_xyz_rpy(x: float, y: float, z: float, r: float, p: float, yaw: float) -> np.ndarray:
    """4x4 homogeneous matrix: position + intrinsic roll,pitch,yaw (same as PyKDL RPY applied as Rotation.RPY)."""
    rot = kdl.Rotation.RPY(float(r), float(p), float(yaw))
    f = kdl.Frame(rot, kdl.Vector(float(x), float(y), float(z)))
    return kdl_frame_to_numpy(f)


def _rotmat_to_rotvec(R: np.ndarray) -> np.ndarray:
    """Log-map SO(3) -> R^3 (axis * angle). Stable for small angles."""
    tr = float(np.trace(R))
    c = (tr - 1.0) * 0.5
    c = float(np.clip(c, -1.0, 1.0))
    s = np.sqrt(max(0.0, 1.0 - c * c))
    w = np.array(
        [R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]],
        dtype=np.float64,
    )
    if s < 1e-8:
        return 0.5 * w
    theta = np.arccos(c)
    return (theta / (2.0 * s)) * w


def solve_arm_ik(
    side: str,
    t_torso_ee_target: np.ndarray,
    q_seed: np.ndarray,
    *,
    rot_weight: float = 0.35,
    max_iters: int = 150,
    tol: float = 0.018,
    damp: float = 2e-4,
    fd_eps: float = 1e-6,
) -> Tuple[np.ndarray, bool]:
    joint_names = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
    lower = np.array([_JOINT_LIMITS_RAD[j][0] for j in joint_names], dtype=np.float64)
    upper = np.array([_JOINT_LIMITS_RAD[j][1] for j in joint_names], dtype=np.float64)

    r_tgt = t_torso_ee_target[:3, :3].astype(np.float64)
    p_tgt = t_torso_ee_target[:3, 3].astype(np.float64)

    def residual(q: np.ndarray) -> np.ndarray:
        m = kdl_frame_to_numpy(fk_arm_torso(side, q))
        p_err = m[:3, 3] - p_tgt
        r_cur = m[:3, :3]
        R_rel = r_tgt @ r_cur.T
        r_err = rot_weight * _rotmat_to_rotvec(R_rel)
        return np.concatenate([p_err, r_err])

    q = np.clip(q_seed.astype(np.float64).copy(), lower, upper)
    best_q = q.copy()
    best_norm = float("inf")

    for _ in range(max_iters):
        r = residual(q)
        n = float(np.linalg.norm(r))
        if n < best_norm:
            best_norm = n
            best_q = q.copy()
        if n < tol:
            return q, True

        jac = np.zeros((6, 7), dtype=np.float64)
        for i in range(7):
            dq = np.zeros(7, dtype=np.float64)
            dq[i] = fd_eps
            q_plus = np.clip(q + dq, lower, upper)
            jac[:, i] = (residual(q_plus) - r) / max(fd_eps, 1e-12)

        jtj = jac.T @ jac + damp * np.eye(7)
        rhs = -jac.T @ r
        try:
            step = np.linalg.solve(jtj, rhs)
        except np.linalg.LinAlgError:
            step = np.linalg.lstsq(jtj, rhs, rcond=None)[0]

        alpha = 1.0
        for _ls in range(12):
            q_try = np.clip(q + alpha * step, lower, upper)
            if np.linalg.norm(residual(q_try)) < n:
                q = q_try
                break
            alpha *= 0.5
        else:
            q = np.clip(q + 0.25 * step, lower, upper)

    err = float(np.linalg.norm(residual(best_q)))
    return best_q, err < 0.025


class CartesianController(Node):
    def __init__(self):
        super().__init__("cartesian_controller")
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.positions = {name: 0.0 for name in JOINT_NAMES}
        self.timer = self.create_timer(0.02, self.publish_joint_states)

        self.sub = self.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        self._got_external = False

        self.get_logger().info(
            "Cartesian controller publishing /joint_states at 50 Hz. "
            "If another joint publisher is running, stop it first."
        )

    def _on_joint_state(self, msg: JointState):
        if msg.position is None or len(msg.position) != len(msg.name):
            return
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        updated = False
        for jn in JOINT_NAMES:
            if jn in name_to_idx:
                idx = name_to_idx[jn]
                self.positions[jn] = float(msg.position[idx])
                updated = True
        if updated:
            self._got_external = True

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.positions.keys())
        msg.position = list(self.positions.values())
        self.pub.publish(msg)

    def set_cartesian(
        self,
        side: str,
        x: float,
        y: float,
        z: float,
        roll: float,
        pitch: float,
        yaw: float,
        *,
        angles_are_degrees: bool,
    ) -> bool:
        if angles_are_degrees:
            roll, pitch, yaw = math.radians(roll), math.radians(pitch), math.radians(yaw)

        t_pelvis_ee = pose_from_xyz_rpy(x, y, z, roll, pitch, yaw)

        q_waist = np.array([self.positions[j] for j in WAIST_JOINTS])
        t_waist = fk_waist(q_waist)
        t_waist_np = kdl_frame_to_numpy(t_waist)
        t_torso_ee = np.linalg.inv(t_waist_np) @ t_pelvis_ee

        arm_joints = LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS
        q_seed = np.array([self.positions[j] for j in arm_joints])

        q_sol, ok = solve_arm_ik(side, t_torso_ee, q_seed)
        if not ok:
            self.get_logger().warn(
                f"IK did not fully converge for '{side}' arm (cost may still be usable). "
                "Try another seed pose or target."
            )

        for jn, q in zip(arm_joints, q_sol):
            self.positions[jn] = float(q)

        self.get_logger().info(
            f"{'Left' if side=='left' else 'Right'} hand target (pelvis): "
            f"pos=({x:.3f},{y:.3f},{z:.3f}) m | RPY(deg)=({math.degrees(roll):.1f},{math.degrees(pitch):.1f},{math.degrees(yaw):.1f})"
        )
        return ok


def _run_cli(args: argparse.Namespace) -> None:
    rclpy.init()
    ctrl = CartesianController()
    spin_thread = threading.Thread(target=rclpy.spin, args=(ctrl,))
    spin_thread.start()
    time.sleep(0.6)

    use_deg = not bool(getattr(args, "rpy_rad", False))
    if args.left is not None:
        xs = args.left
        ctrl.set_cartesian("left", xs[0], xs[1], xs[2], xs[3], xs[4], xs[5], angles_are_degrees=use_deg)
    if args.right is not None:
        xs = args.right
        ctrl.set_cartesian("right", xs[0], xs[1], xs[2], xs[3], xs[4], xs[5], angles_are_degrees=use_deg)

    # Keep publishing so RViz / robot_state_publisher receive updates. Exiting immediately often looks
    # like "no motion", especially if joint_state_publisher_gui is still running (it overwrites us).
    hold = float(getattr(args, "hold", 3.0))
    time.sleep(max(0.0, hold))

    ctrl.destroy_node()
    rclpy.shutdown()
    with contextlib.suppress(RuntimeError):
        spin_thread.join(timeout=1.0)


def _interactive() -> None:
    rclpy.init()
    ctrl = CartesianController()
    spin_thread = threading.Thread(target=rclpy.spin, args=(ctrl,))
    spin_thread.start()
    time.sleep(0.6)

    print("\n=== G1 Cartesian control (target frame: pelvis) ===")
    print("Commands:")
    print("  left  x y z  roll pitch yaw   — left hand (default RPY in degrees)")
    print("  right x y z  roll pitch yaw  — right hand")
    print("  fk left|right               — print FK pose of hand in pelvis (numpy)")
    print("  quit")
    print()

    try:
        while True:
            line = input("cart> ").strip()
            if not line:
                continue
            parts = line.lower().split()
            if parts[0] in ("q", "quit", "exit"):
                break
            if parts[0] == "fk" and len(parts) == 2:
                side = parts[1]
                if side not in ("left", "right"):
                    print("  Usage: fk left  OR  fk right")
                    continue
                qw = [ctrl.positions[j] for j in WAIST_JOINTS]
                arm = [ctrl.positions[j] for j in (LEFT_ARM_JOINTS if side == "left" else RIGHT_ARM_JOINTS)]
                t = fk_hand_in_pelvis(qw, side, arm, ctrl.positions)
                m = kdl_frame_to_numpy(t)
                print("  T (pelvis <- ee) translation:", m[0, 3], m[1, 3], m[2, 3])
                continue

            if parts[0] not in ("left", "right") or len(parts) != 7:
                print("  Usage: left x y z r p y   (r p y in degrees by default)")
                continue
            side = parts[0]
            vals = list(map(float, parts[1:7]))
            ctrl.set_cartesian(
                side,
                vals[0],
                vals[1],
                vals[2],
                vals[3],
                vals[4],
                vals[5],
                angles_are_degrees=True,
            )
    except (KeyboardInterrupt, EOFError):
        pass

    ctrl.destroy_node()
    rclpy.shutdown()
    with contextlib.suppress(RuntimeError):
        spin_thread.join(timeout=1.0)


def main() -> None:
    parser = argparse.ArgumentParser(description="Cartesian hand pose -> joint angles for G1 (RViz).")
    parser.add_argument(
        "--left",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "R", "P", "Y"),
        help="Left hand target in pelvis frame: x y z (m) roll pitch yaw",
    )
    parser.add_argument(
        "--right",
        nargs=6,
        type=float,
        metavar=("X", "Y", "Z", "R", "P", "Y"),
        help="Right hand target in pelvis frame",
    )
    parser.add_argument(
        "--rpy-rad",
        action="store_true",
        help="Interpret roll pitch yaw as radians (default: degrees).",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=3.0,
        metavar="SEC",
        help="After applying pose, keep publishing /joint_states for this many seconds (CLI only). "
        "Increase if RViz does not update; use interactive mode to stay running.",
    )
    args = parser.parse_args()

    if args.left is None and args.right is None:
        _interactive()
    else:
        _run_cli(args)


if __name__ == "__main__":
    main()
