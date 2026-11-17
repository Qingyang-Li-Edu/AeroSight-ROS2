"""Interactive joint-space controller for the G1 in RViz2.

Run:  python3 -m g1_demos.joint_control
Commands: stand | wave | walk | set <joint-part> <deg> | list | quit
"""

import math

from g1_demos.common import JOINT_NAMES, G1Publisher, run_node


class JointController(G1Publisher):
    def wave_right_arm(self) -> None:
        self.get_logger().info("Waving right arm...")
        self.play_motion(2.0, lambda t: {
            "right_shoulder_pitch_joint": -1.5,
            "right_shoulder_roll_joint": -0.3,
            "right_elbow_joint": 0.8 + 0.5 * math.sin(2 * math.pi * t / 1.0),
        })
        self.go_to_stand()

    def walk_pose(self) -> None:
        self.get_logger().info("Walk pose demo...")

        def motion(t: float):
            p = math.sin(2 * math.pi * t / 2.0)
            return {
                "left_hip_pitch_joint": 0.3 * p, "left_knee_joint": -0.6 * abs(p),
                "left_ankle_pitch_joint": 0.3 * p,
                "right_hip_pitch_joint": -0.3 * p, "right_knee_joint": -0.6 * abs(p),
                "right_ankle_pitch_joint": -0.3 * p,
                "left_shoulder_pitch_joint": -0.3 * p, "right_shoulder_pitch_joint": 0.3 * p,
            }

        self.play_motion(4.0, motion)
        self.go_to_stand()

    def set_by_name(self, part: str, deg: float) -> None:
        matches = [j for j in JOINT_NAMES if part in j]
        if len(matches) == 1:
            self.set_pose({matches[0]: math.radians(deg)})
            print(f"  {matches[0]} = {deg} deg")
        elif matches:
            print(f"  Ambiguous, matches: {matches}")
        else:
            print(f"  No joint matching '{part}'")

    def list_joints(self) -> None:
        for i, name in enumerate(JOINT_NAMES):
            print(f"  [{i:2d}] {name:30s} = {math.degrees(self.positions[name]):7.2f} deg")


HELP = "Commands: stand | wave | walk | set <joint-part> <deg> | list | quit"


def repl(node: JointController) -> None:
    print("\n=== G1 joint controller ===\n" + HELP)
    while True:
        cmd = input("g1> ").strip().lower()
        if cmd in ("quit", "q"):
            return
        if cmd == "stand":
            node.go_to_stand()
        elif cmd == "wave":
            node.wave_right_arm()
        elif cmd == "walk":
            node.walk_pose()
        elif cmd == "list":
            node.list_joints()
        elif cmd.startswith("set ") and len(cmd.split()) == 3:
            try:
                node.set_by_name(cmd.split()[1], float(cmd.split()[2]))
            except ValueError:
                print("  Usage: set <joint-part> <degrees>")
        else:
            print("  " + HELP)


def main():
    node = JointController("g1_joint_control")
    run_node(node, lambda: repl(node))


if __name__ == "__main__":
    main()
