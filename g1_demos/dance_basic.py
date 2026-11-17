"""Automatic 4-stage dance demo (sway, arm wave, step-touch, final pose).

Run:  python3 -m g1_demos.dance_basic
"""

import time

import math

from g1_demos.common import G1Publisher, run_node


class BasicDance(G1Publisher):
    def __init__(self):
        super().__init__("g1_dance_basic")

    def dance_intro(self):
        self.get_logger().info("Stage 1: warm up sway")

        def motion(t: float):
            sway = 0.18 * math.sin(2.0 * math.pi * 0.8 * t)
            arm = 0.45 * math.sin(2.0 * math.pi * 1.6 * t)
            return {
                "waist_roll_joint": 0.12 * math.sin(2.0 * math.pi * 0.8 * t),
                "left_hip_roll_joint": sway,
                "right_hip_roll_joint": -sway,
                "left_ankle_roll_joint": -0.6 * sway,
                "right_ankle_roll_joint": 0.6 * sway,
                "left_shoulder_roll_joint": 0.4 + 0.5 * arm,
                "right_shoulder_roll_joint": -0.4 - 0.5 * arm,
                "left_elbow_joint": -0.7,
                "right_elbow_joint": 0.7,
            }

        self.play_motion(4.0, motion)

    def dance_arm_wave(self):
        self.get_logger().info("Stage 2: arm wave")

        def motion(t: float):
            wave = math.sin(2.0 * math.pi * 2.2 * t)
            shoulder_pitch = -1.0 + 0.25 * math.sin(2.0 * math.pi * 1.1 * t)
            return {
                "left_shoulder_pitch_joint": shoulder_pitch,
                "right_shoulder_pitch_joint": shoulder_pitch,
                "left_shoulder_roll_joint": 0.6,
                "right_shoulder_roll_joint": -0.6,
                "left_elbow_joint": -0.9 - 0.45 * wave,
                "right_elbow_joint": 0.9 + 0.45 * wave,
                "left_wrist_yaw_joint": 0.3 * wave,
                "right_wrist_yaw_joint": -0.3 * wave,
                "waist_yaw_joint": 0.18 * math.sin(2.0 * math.pi * 1.1 * t),
            }

        self.play_motion(4.0, motion)

    def dance_step_touch(self):
        self.get_logger().info("Stage 3: step touch")

        def motion(t: float):
            beat = math.sin(2.0 * math.pi * 1.0 * t)
            lift = max(0.0, beat)
            return {
                "left_hip_pitch_joint": 0.22 * beat,
                "right_hip_pitch_joint": -0.22 * beat,
                "left_knee_joint": -0.45 * lift,
                "right_knee_joint": -0.45 * max(0.0, -beat),
                "left_ankle_pitch_joint": -0.12 * beat,
                "right_ankle_pitch_joint": 0.12 * beat,
                "waist_yaw_joint": 0.25 * beat,
                "left_shoulder_pitch_joint": -0.35 * beat,
                "right_shoulder_pitch_joint": 0.35 * beat,
            }

        self.play_motion(4.0, motion)

    def dance_final_pose(self):
        self.get_logger().info("Stage 4: final pose")

        def motion(t: float):
            settle = min(1.0, t / 1.5)
            return {
                "waist_yaw_joint": 0.35 * (1.0 - settle),
                "waist_roll_joint": -0.12 * (1.0 - settle),
                "left_shoulder_pitch_joint": -1.2 + 0.2 * settle,
                "right_shoulder_pitch_joint": -1.2 + 0.2 * settle,
                "left_shoulder_roll_joint": 0.9 - 0.3 * settle,
                "right_shoulder_roll_joint": -0.9 + 0.3 * settle,
                "left_elbow_joint": -1.2 + 0.2 * settle,
                "right_elbow_joint": 1.2 - 0.2 * settle,
            }

        self.play_motion(2.0, motion)
        time.sleep(0.5)
        self.go_to_stand()
        time.sleep(1.0)

    def perform_dance(self):
        self.go_to_stand()
        time.sleep(1.0)
        self.dance_intro()
        self.dance_arm_wave()
        self.dance_step_touch()
        self.dance_final_pose()


def main():
    node = BasicDance()
    print("\n=== G1 dance demo (basic) ===")
    run_node(node, node.perform_dance)


if __name__ == "__main__":
    main()
