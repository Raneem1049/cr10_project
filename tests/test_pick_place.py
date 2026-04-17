#!/usr/bin/env python3

from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest


PROJECT_DIR = Path(__file__).resolve().parents[1]
if str(PROJECT_DIR) not in sys.path:
    sys.path.insert(0, str(PROJECT_DIR))

from cr10_project.cr10_kinematics import CR10Kinematics
from cr10_project.cr10_pick_place import (
    CubeParameters,
    build_pick_place_keyframes,
    build_pick_place_trajectory,
    cube_center_robot_frame,
    mirrored_target_pose,
    summarize_plan,
    tool_yaw_from_cube,
)
from cr10_project.trajectory_generators import mirror_yaw


class TestPickPlacePlanning(unittest.TestCase):
    def setUp(self) -> None:
        self.kinematics = CR10Kinematics()

    def test_robot_frame_uses_robot_height_parameter(self) -> None:
        params = CubeParameters(x=0.44, y=0.12, z=0.93, h=0.68)
        self.assertEqual(cube_center_robot_frame(params), (0.44, 0.12, 0.25))

    def test_mirror_target_flips_x_and_orientation(self) -> None:
        params = CubeParameters(alpha=0.40)
        _, release = mirrored_target_pose(params)
        self.assertAlmostEqual(release.x, -params.x)
        self.assertAlmostEqual(release.y, params.y)
        # Effective gripper yaw in world frame matches mirrored cube (mod 180°)
        q1 = math.atan2(release.y, release.x)
        effective_yaw = q1 - release.wrist_yaw
        expected_yaw = tool_yaw_from_cube(mirror_yaw(params.alpha))
        diff = math.atan2(
            math.sin(effective_yaw - expected_yaw),
            math.cos(effective_yaw - expected_yaw),
        )
        self.assertTrue(
            abs(diff) < 1e-6 or abs(abs(diff) - math.pi) < 1e-6,
        )

    def test_top_down_grasp_uses_joint5_instead_of_joint6(self) -> None:
        params = CubeParameters(alpha=math.radians(20.0))
        _, release = mirrored_target_pose(params)

        joints = self.kinematics.inverse_kinematics(
            release.x,
            release.y,
            release.z,
            wrist_yaw=release.wrist_yaw,
        )
        self.assertAlmostEqual(joints[4], release.wrist_yaw, places=6)
        self.assertAlmostEqual(joints[5], 0.0, places=6)

        # Effective gripper yaw matches the mirrored cube orientation (mod 180°)
        effective_yaw = joints[0] - joints[4]
        expected_yaw = tool_yaw_from_cube(mirror_yaw(params.alpha))
        diff = math.atan2(
            math.sin(effective_yaw - expected_yaw),
            math.cos(effective_yaw - expected_yaw),
        )
        self.assertTrue(
            abs(diff) < 1e-5 or abs(abs(diff) - math.pi) < 1e-5,
        )

    def test_default_pick_place_plan_is_reachable(self) -> None:
        params = CubeParameters()
        trajectory = build_pick_place_trajectory(params)

        for sample in trajectory:
            joints = self.kinematics.inverse_kinematics(
                sample.pose.x,
                sample.pose.y,
                sample.pose.z,
                wrist_yaw=sample.pose.wrist_yaw,
            )
            self.assertTrue(
                self.kinematics.validate_joint_solution(
                    joints,
                    sample.pose.point,
                    tolerance=1e-5,
                )
            )

        self.assertEqual(trajectory[0].cube_state, "source")
        self.assertEqual(trajectory[-1].cube_state, "target")

    def test_default_pick_place_sequence_stays_continuous_with_reference_joints(self) -> None:
        params = CubeParameters()
        trajectory = build_pick_place_trajectory(params)

        previous_joints = None
        max_step_change = 0.0
        for sample in trajectory:
            joints = self.kinematics.inverse_kinematics(
                sample.pose.x,
                sample.pose.y,
                sample.pose.z,
                wrist_yaw=sample.pose.wrist_yaw,
                reference_joints=previous_joints,
            )
            if previous_joints is not None:
                max_step_change = max(
                    max_step_change,
                    max(
                        abs(math.degrees(math.atan2(math.sin(c - p), math.cos(c - p))))
                        for p, c in zip(previous_joints, joints)
                    ),
                )
            previous_joints = joints

        self.assertLess(max_step_change, 10.0)

    def test_large_cube_is_rejected_for_the_gripper(self) -> None:
        with self.assertRaises(ValueError):
            build_pick_place_keyframes(CubeParameters(c=0.10))

    def test_summary_contains_key_project_outputs(self) -> None:
        summary = summarize_plan(CubeParameters())
        self.assertIn("cube_center_ground_frame", summary)
        self.assertIn("cube_center_robot_frame", summary)
        self.assertIn("mirror_target_center_robot_frame", summary)
        self.assertIn("keyframes", summary)
        self.assertTrue(summary["gripper_can_grasp"])


if __name__ == "__main__":
    unittest.main()
