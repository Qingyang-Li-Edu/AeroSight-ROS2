"""Beat-driven 5-stage dance demo (124 BPM) with smooth pose blending.

Run:  python3 -m g1_demos.dance_beat
"""

import math
import time

from g1_demos.common import G1Publisher, run_node

BPM = 124.0
BEAT = 60.0 / BPM


class BeatDance(G1Publisher):
    def __init__(self):
        super().__init__("g1_dance_beat")

    def play_beats(self, beats: float, motion_fn):
        self.play_motion(beats * BEAT, motion_fn)

    def beat_phase(self, t: float, multiplier: float = 1.0, phase: float = 0.0):
        return 2.0 * math.pi * multiplier * t / BEAT + phase

    def opening_pose(self):
        self.get_logger().info("Stage 1: opening pose")
        pose = self.stand_pose()
        pose.update(
            {
                "left_shoulder_roll_joint": 0.65,
                "right_shoulder_roll_joint": -0.65,
                "left_elbow_joint": -1.0,
                "right_elbow_joint": 1.0,
                "waist_pitch_joint": 0.08,
                "left_knee_joint": -0.12,
                "right_knee_joint": -0.12,
            }
        )
        self.blend_to_pose(pose, 1.2)
        self.hold_pose(pose, 0.4)

    def groove_sway(self):
        self.get_logger().info("Stage 2: groove sway")
        base = self.current_pose()

        def motion(t: float):
            downbeat = math.sin(self.beat_phase(t, 0.5))
            upbeat = math.sin(self.beat_phase(t, 1.0))
            snap = max(0.0, math.sin(self.beat_phase(t, 2.0)))
            return {
                **base,
                "waist_roll_joint": 0.15 * upbeat,
                "waist_yaw_joint": 0.20 * downbeat,
                "left_hip_roll_joint": 0.16 * upbeat,
                "right_hip_roll_joint": -0.16 * upbeat,
                "left_ankle_roll_joint": -0.08 * upbeat,
                "right_ankle_roll_joint": 0.08 * upbeat,
                "left_shoulder_pitch_joint": -0.25 + 0.08 * snap,
                "right_shoulder_pitch_joint": -0.25 + 0.08 * snap,
                "left_shoulder_roll_joint": 0.65 + 0.10 * downbeat,
                "right_shoulder_roll_joint": -0.65 - 0.10 * downbeat,
                "left_wrist_yaw_joint": 0.22 * snap,
                "right_wrist_yaw_joint": -0.22 * snap,
            }

        self.play_beats(8.0, motion)

    def arm_flourish(self):
        self.get_logger().info("Stage 3: arm flourish")
        base = self.stand_pose()

        def motion(t: float):
            lead = math.sin(self.beat_phase(t, 1.0))
            fill = math.sin(self.beat_phase(t, 2.0, math.pi / 3.0))
            accent = max(0.0, math.sin(self.beat_phase(t, 4.0)))
            return {
                **base,
                "waist_yaw_joint": 0.14 * lead,
                "waist_roll_joint": 0.08 * fill,
                "left_shoulder_pitch_joint": -1.10 + 0.10 * lead,
                "right_shoulder_pitch_joint": -1.10 - 0.10 * lead,
                "left_shoulder_roll_joint": 0.78 + 0.16 * fill,
                "right_shoulder_roll_joint": -0.78 - 0.16 * fill,
                "left_elbow_joint": -1.05 - 0.28 * accent,
                "right_elbow_joint": 1.05 + 0.28 * accent,
                "left_wrist_yaw_joint": 0.32 * fill,
                "right_wrist_yaw_joint": -0.32 * fill,
            }

        self.play_beats(8.0, motion)

    def bounce_step(self):
        self.get_logger().info("Stage 4: bounce step")
        base = self.stand_pose()
        base.update(
            {
                "left_shoulder_roll_joint": 0.35,
                "right_shoulder_roll_joint": -0.35,
                "left_elbow_joint": -0.75,
                "right_elbow_joint": 0.75,
            }
        )

        def motion(t: float):
            side = math.sin(self.beat_phase(t, 1.0))
            bounce = max(0.0, math.sin(self.beat_phase(t, 2.0)))
            push = max(0.0, math.sin(self.beat_phase(t, 1.0, math.pi)))
            knee_bounce = -0.12 - 0.24 * bounce
            return {
                **base,
                "left_hip_pitch_joint": 0.18 * side,
                "right_hip_pitch_joint": -0.18 * side,
                "left_hip_yaw_joint": 0.08 * side,
                "right_hip_yaw_joint": 0.08 * side,
                "left_knee_joint": knee_bounce - 0.10 * side,
                "right_knee_joint": knee_bounce + 0.10 * side,
                "left_ankle_pitch_joint": -0.08 * side,
                "right_ankle_pitch_joint": 0.08 * side,
                "waist_yaw_joint": 0.24 * side,
                "waist_pitch_joint": 0.05 * bounce,
                "left_shoulder_pitch_joint": -0.22 * push,
                "right_shoulder_pitch_joint": 0.22 * push,
            }

        self.play_beats(8.0, motion)

    def spin_finish(self):
        self.get_logger().info("Stage 5: spin finish")
        base = self.stand_pose()

        def motion(t: float):
            turn = math.sin(self.beat_phase(t, 0.5))
            hit = max(0.0, math.sin(self.beat_phase(t, 2.0)))
            sweep = math.sin(self.beat_phase(t, 1.0, math.pi / 2.0))
            return {
                **base,
                "waist_yaw_joint": 0.42 * turn,
                "waist_roll_joint": -0.10 * sweep,
                "left_shoulder_pitch_joint": -1.05 + 0.08 * hit,
                "right_shoulder_pitch_joint": -1.05 + 0.08 * hit,
                "left_shoulder_roll_joint": 0.70 - 0.15 * turn,
                "right_shoulder_roll_joint": -0.70 - 0.15 * turn,
                "left_elbow_joint": -1.25 + 0.18 * hit,
                "right_elbow_joint": 1.25 - 0.18 * hit,
                "left_knee_joint": -0.18 - 0.14 * hit,
                "right_knee_joint": -0.18 - 0.14 * max(0.0, -math.sin(self.beat_phase(t, 2.0))),
            }

        self.play_beats(6.0, motion)

        final_pose = self.stand_pose()
        final_pose.update(
            {
                "waist_yaw_joint": 0.20,
                "waist_roll_joint": -0.08,
                "left_shoulder_pitch_joint": -1.25,
                "right_shoulder_pitch_joint": -1.25,
                "left_shoulder_roll_joint": 0.95,
                "right_shoulder_roll_joint": -0.45,
                "left_elbow_joint": -1.35,
                "right_elbow_joint": 0.85,
                "left_wrist_yaw_joint": 0.35,
                "right_wrist_yaw_joint": -0.15,
            }
        )
        self.blend_to_pose(final_pose, 0.8)
        self.hold_pose(final_pose, 1.2)

    def perform_dance(self):
        self.blend_to_pose(self.stand_pose(), 0.6)
        self.hold_pose(self.stand_pose(), 0.4)
        self.opening_pose()
        self.groove_sway()
        self.arm_flourish()
        self.bounce_step()
        self.spin_finish()
        self.blend_to_pose(self.stand_pose(), 1.0)
        self.hold_pose(self.stand_pose(), 0.8)


def main():
    node = BeatDance()
    print(f"\n=== G1 dance demo (beat, {int(BPM)} BPM) ===")
    run_node(node, node.perform_dance)


if __name__ == "__main__":
    main()
