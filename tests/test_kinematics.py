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
from cr10_project.trajectory_generators import circle_parallel_to_ground, interpolate_line


class TestCR10Kinematics(unittest.TestCase):
    def setUp(self) -> None:
        self.kinematics = CR10Kinematics()

    def test_forward_inverse_consistency(self) -> None:
        targets = [
            (0.48, 0.10, 0.32),
            (0.40, -0.15, 0.36),
            (0.55, 0.00, 0.30),
            (-0.42, 0.18, 0.34),
        ]

        for target in targets:
            joints = self.kinematics.inverse_kinematics(*target)
            self.assertTrue(
                self.kinematics.validate_joint_solution(joints, target, tolerance=1e-6)
            )
            self.assertAlmostEqual(
                self.kinematics.tool_pitch(joints),
                self.kinematics.geometry.default_tool_pitch,
                places=6,
            )

    def test_demo_trajectories_are_reachable(self) -> None:
        samples = interpolate_line((0.48, 0.10, 0.32), (0.40, -0.15, 0.36), 12)
        samples += circle_parallel_to_ground((0.42, 0.00, 0.34), 0.08, 18)

        max_error = 0.0
        for target in samples:
            joints = self.kinematics.inverse_kinematics(*target)
            max_error = max(max_error, self.kinematics.position_error(joints, target))

        self.assertLess(max_error, 1e-6)

    def test_reference_joints_keep_demo_line_on_one_branch(self) -> None:
        samples = interpolate_line((0.48, 0.10, 0.32), (0.40, -0.15, 0.36), 40)

        sequence = []
        previous_joints = None
        for target in samples:
            joints = self.kinematics.inverse_kinematics(
                *target,
                reference_joints=previous_joints,
            )
            sequence.append(joints)
            previous_joints = joints

        q3_signs = [1 if joints[2] > 0.0 else -1 for joints in sequence]
        self.assertEqual(len(set(q3_signs)), 1)

        max_step_change = 0.0
        for previous, current in zip(sequence, sequence[1:]):
            max_step_change = max(
                max_step_change,
                max(
                    abs(math.degrees(math.atan2(math.sin(c - p), math.cos(c - p))))
                    for p, c in zip(previous, current)
                ),
            )

        self.assertLess(max_step_change, 10.0)

    def test_reference_joints_keep_demo_circle_on_one_branch(self) -> None:
        samples = circle_parallel_to_ground((0.42, 0.00, 0.34), 0.08, 80)

        sequence = []
        previous_joints = None
        for target in samples:
            joints = self.kinematics.inverse_kinematics(
                *target,
                reference_joints=previous_joints,
            )
            sequence.append(joints)
            previous_joints = joints

        q3_signs = [1 if joints[2] > 0.0 else -1 for joints in sequence]
        self.assertEqual(len(set(q3_signs)), 1)

        max_step_change = 0.0
        for previous, current in zip(sequence, sequence[1:]):
            max_step_change = max(
                max_step_change,
                max(
                    abs(math.degrees(math.atan2(math.sin(c - p), math.cos(c - p))))
                    for p, c in zip(previous, current)
                ),
            )

        self.assertLess(max_step_change, 10.0)

    def test_unreachable_target_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            self.kinematics.inverse_kinematics(2.0, 0.0, 2.0)

    def test_forward_transform_matches_urdf_chain_with_joint6_offset(self) -> None:
        joints = (
            math.radians(18.0),
            math.radians(48.0),
            math.radians(-92.0),
            math.radians(-46.0),
            math.radians(12.0),
            math.radians(25.0),
        )

        urdf_transform = _urdf_tcp_transform(joints)
        python_transform = self.kinematics.forward_transform(joints)
        python_position = self.kinematics.forward_kinematics(joints)

        for row in range(4):
            for column in range(4):
                self.assertAlmostEqual(
                    python_transform[row][column],
                    urdf_transform[row][column],
                    places=9,
                )

        self.assertAlmostEqual(python_position[0], urdf_transform[0][3], places=9)
        self.assertAlmostEqual(python_position[1], urdf_transform[1][3], places=9)
        self.assertAlmostEqual(python_position[2], urdf_transform[2][3], places=9)

    def test_fk_zero_config_matches_known_tcp(self) -> None:
        """At all-zero joints the TCP must be at the sum of all X offsets, Z = base height."""
        x, y, z = self.kinematics.forward_kinematics((0, 0, 0, 0, 0, 0))
        self.assertAlmostEqual(x, 0.12 + 0.42 + 0.34 + 0.08 + 0.10, places=9)
        self.assertAlmostEqual(y, 0.0, places=9)
        self.assertAlmostEqual(z, 0.42, places=9)

    def test_positive_q2_rotates_arm_downward(self) -> None:
        """Standard URDF Ry: positive q2 about +Y sends X toward -Z (arm down)."""
        joints_zero = (0, 0, 0, 0, 0, 0)
        joints_q2 = (0, 0.3, 0, 0, 0, 0)
        z_zero = self.kinematics.forward_kinematics(joints_zero)[2]
        z_q2 = self.kinematics.forward_kinematics(joints_q2)[2]
        self.assertLess(z_q2, z_zero, "Positive q2 must move TCP downward with URDF Ry(+Y).")

    def test_q2_half_pi_arm_straight_down(self) -> None:
        """q2 = pi/2, others zero: upper arm points straight down from shoulder."""
        x, y, z = self.kinematics.forward_kinematics((0, math.pi / 2, 0, 0, 0, 0))
        self.assertAlmostEqual(x, 0.12, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        expected_z = 0.42 - 0.42 - 0.34 - 0.08 - 0.10
        self.assertAlmostEqual(z, expected_z, places=6)

    def test_q1_rotates_tcp_in_xy_plane(self) -> None:
        """q1 = pi/4, others zero: TCP rotated 45deg in XY plane."""
        x, y, z = self.kinematics.forward_kinematics((math.pi / 4, 0, 0, 0, 0, 0))
        arm_reach = 0.12 + 0.42 + 0.34 + 0.08 + 0.10
        self.assertAlmostEqual(x, arm_reach * math.cos(math.pi / 4), places=9)
        self.assertAlmostEqual(y, arm_reach * math.sin(math.pi / 4), places=9)
        self.assertAlmostEqual(z, 0.42, places=9)

    def test_tool_pitch_positive_means_pointing_down(self) -> None:
        """With corrected URDF convention, default_tool_pitch = +pi/2 means pointing down."""
        target = (0.48, 0.10, 0.32)
        joints = self.kinematics.inverse_kinematics(*target)
        pitch = self.kinematics.tool_pitch(joints)
        self.assertAlmostEqual(pitch, math.pi / 2.0, places=6)


def _urdf_tcp_transform(joints: tuple[float, float, float, float, float, float]) -> tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]:
    q1, q2, q3, q4, q5, q6 = joints
    transform = _translation(0.0, 0.0, 0.0)
    chain = (
        _rotation_z(q1),
        _translation(0.0, 0.0, 0.42),
        _translation(0.12, 0.0, 0.0),
        _rotation_y_joint(q2),
        _translation(0.42, 0.0, 0.0),
        _rotation_y_joint(q3),
        _translation(0.34, 0.0, 0.0),
        _rotation_y_joint(q4),
        _translation(0.08, 0.0, 0.0),
        _rotation_x(q5),
        _rotation_z(q6),
        _translation(0.10, 0.0, 0.0),
    )
    for step in chain:
        transform = _matmul(transform, step)
    return transform


def _matmul(
    left: tuple[
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ],
    right: tuple[
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
        tuple[float, float, float, float],
    ],
) -> tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]:
    return tuple(
        tuple(sum(left[row][k] * right[k][column] for k in range(4)) for column in range(4))
        for row in range(4)
    )


def _translation(x: float, y: float, z: float) -> tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]:
    return (
        (1.0, 0.0, 0.0, x),
        (0.0, 1.0, 0.0, y),
        (0.0, 0.0, 1.0, z),
        (0.0, 0.0, 0.0, 1.0),
    )


def _rotation_x(angle: float) -> tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        (1.0, 0.0, 0.0, 0.0),
        (0.0, cosine, -sine, 0.0),
        (0.0, sine, cosine, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _rotation_y_joint(angle: float) -> tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        (cosine, 0.0, sine, 0.0),
        (0.0, 1.0, 0.0, 0.0),
        (-sine, 0.0, cosine, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


def _rotation_z(angle: float) -> tuple[
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
    tuple[float, float, float, float],
]:
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return (
        (cosine, -sine, 0.0, 0.0),
        (sine, cosine, 0.0, 0.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.0, 0.0, 0.0, 1.0),
    )


if __name__ == "__main__":
    unittest.main()
