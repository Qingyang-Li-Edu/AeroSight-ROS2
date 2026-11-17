"""Shared pieces for all G1 demos: joint list, environment guard and a joint-state publisher node."""

from __future__ import annotations

import contextlib
import math
import sys
import threading
import time
from typing import Callable, Dict

if sys.version_info[:2] != (3, 10):
    raise SystemExit(
        "ROS2 Humble needs Python 3.10, got "
        f"{sys.executable} ({sys.version.split()[0]}).\n"
        "Run inside the container:  scripts/run_demo.sh <demo>\n"
        "or:  source /opt/ros/humble/setup.bash && /usr/bin/python3 -m g1_demos.<demo>"
    )

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
except ModuleNotFoundError as exc:  # pragma: no cover
    raise SystemExit(
        f"ROS2 Python deps missing ({exc}). Did you `source /opt/ros/humble/setup.bash`?"
    ) from exc

JOINT_NAMES = [
    # Left leg (6)
    "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint",
    "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
    # Right leg (6)
    "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint",
    "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint",
    # Waist (3)
    "waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint",
    # Left arm (7)
    "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
    "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint", "left_wrist_yaw_joint",
    # Right arm (7)
    "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
    "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint", "right_wrist_yaw_joint",
]

STEP_DT = 0.02  # 50 Hz


class G1Publisher(Node):
    """Publishes /joint_states at 50 Hz from `self.positions` (radians)."""

    def __init__(self, node_name: str = "g1_controller"):
        if not rclpy.ok():
            rclpy.init()
        super().__init__(node_name)
        self.pub = self.create_publisher(JointState, "/joint_states", 10)
        self.positions: Dict[str, float] = self.stand_pose()
        self.create_timer(STEP_DT, self._publish)

    def _publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self.positions)
        msg.position = list(self.positions.values())
        self.pub.publish(msg)

    # -- pose helpers ---------------------------------------------------
    @staticmethod
    def stand_pose() -> Dict[str, float]:
        return {n: 0.0 for n in JOINT_NAMES}

    def current_pose(self) -> Dict[str, float]:
        return dict(self.positions)

    def set_pose(self, joints: Dict[str, float]) -> None:
        for name, angle in joints.items():
            if name in self.positions:
                self.positions[name] = angle
            else:
                self.get_logger().warn(f"Unknown joint: {name}")

    def go_to_stand(self) -> None:
        self.positions = self.stand_pose()

    # -- motion helpers -------------------------------------------------
    def hold_pose(self, pose: Dict[str, float], duration: float) -> None:
        self.set_pose(pose)
        time.sleep(duration)

    def blend_to_pose(self, target: Dict[str, float], duration: float) -> None:
        """Cosine-eased interpolation from the current pose to `target`."""
        start = self.current_pose()
        steps = max(1, int(duration / STEP_DT))
        for i in range(steps):
            k = 0.5 - 0.5 * math.cos(math.pi * (i + 1) / steps)
            self.set_pose({n: start[n] + (target.get(n, start[n]) - start[n]) * k for n in JOINT_NAMES})
            time.sleep(STEP_DT)

    def play_motion(self, duration: float, motion_fn: Callable[[float], Dict[str, float]]) -> None:
        """Call motion_fn(t) every STEP_DT for `duration` seconds and apply the returned joints."""
        for i in range(max(1, int(duration / STEP_DT))):
            self.set_pose(motion_fn(i * STEP_DT))
            time.sleep(STEP_DT)


def run_node(node: Node, body: Callable[[], None]) -> None:
    """Spin `node` in a background thread, run `body()` in the foreground, then shut down cleanly."""
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    time.sleep(0.5)  # let the publisher come up
    try:
        body()
    except (KeyboardInterrupt, EOFError):
        print("\nInterrupted.")
    finally:
        with contextlib.suppress(Exception):
            node.destroy_node()
        with contextlib.suppress(Exception):
            if rclpy.ok():
                rclpy.shutdown()
        spin.join(timeout=1.0)
