#!/usr/bin/env python3

from __future__ import annotations

from collections import deque
from dataclasses import asdict, dataclass
import math
from typing import Dict, List, Sequence, Tuple

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from .cr10_kinematics import CR10Kinematics
from .trajectory_generators import (
    CartesianPose,
    interpolate_pose,
    interpolate_scalar,
    mirror_point,
    mirror_yaw,
)


Point3 = Tuple[float, float, float]


@dataclass(frozen=True)
class CubeParameters:
    """Assignment parameters expressed in the world frame."""

    c: float = 0.05
    x: float = 0.48
    y: float = 0.18
    z: float = 0.90
    alpha: float = math.radians(20.0)
    h: float = 0.65


@dataclass(frozen=True)
class Robotiq2F85:
    max_opening: float = 0.085
    finger_clearance: float = 0.006
    approach_clearance: float = 0.10
    lift_clearance: float = 0.18

    def required_opening(self, cube_side: float) -> float:
        return cube_side + 2.0 * self.finger_clearance

    def can_grasp(self, cube_side: float) -> bool:
        return self.required_opening(cube_side) <= self.max_opening + 1e-9


@dataclass(frozen=True)
class PickPlaceKeyframe:
    name: str
    pose: CartesianPose
    gripper_opening: float
    cube_state: str


@dataclass(frozen=True)
class PickPlaceSample:
    phase: str
    pose: CartesianPose
    gripper_opening: float
    cube_state: str


def cube_center_ground_frame(params: CubeParameters) -> Point3:
    return (params.x, params.y, params.z)


def cube_center_robot_frame(params: CubeParameters) -> Point3:
    return (params.x, params.y, params.z - params.h)


def tool_yaw_from_cube(alpha: float) -> float:
    """Return an equivalent tool-axis yaw for a symmetric parallel gripper.

    The cube orientation lives in the world frame, but the current URDF can
    only spin the top-down gripper without moving the TCP by using joint5.
    Because the two-finger grasp is 180-degree symmetric, we can choose the
    equivalent angle in [-pi/2, pi/2] and still align the jaws with the cube.
    """

    wrapped = math.atan2(math.sin(alpha), math.cos(alpha))
    if wrapped > math.pi / 2.0:
        wrapped -= math.pi
    elif wrapped < -math.pi / 2.0:
        wrapped += math.pi
    return wrapped


def source_grasp_pose(
    params: CubeParameters,
    *,
    gripper: Robotiq2F85 | None = None,
) -> Tuple[CartesianPose, CartesianPose]:
    tool = gripper or Robotiq2F85()
    center = cube_center_robot_frame(params)
    top_surface_z = center[2] + params.c / 2.0
    yaw = tool_yaw_from_cube(params.alpha)
    approach = CartesianPose(center[0], center[1], top_surface_z + tool.approach_clearance, yaw)
    grasp = CartesianPose(center[0], center[1], top_surface_z, yaw)
    return approach, grasp


def mirrored_target_pose(
    params: CubeParameters,
    *,
    gripper: Robotiq2F85 | None = None,
) -> Tuple[CartesianPose, CartesianPose]:
    tool = gripper or Robotiq2F85()
    mirrored_center = mirror_point(cube_center_robot_frame(params))
    top_surface_z = mirrored_center[2] + params.c / 2.0
    yaw = tool_yaw_from_cube(mirror_yaw(params.alpha))
    approach = CartesianPose(
        mirrored_center[0],
        mirrored_center[1],
        top_surface_z + tool.approach_clearance,
        yaw,
    )
    release = CartesianPose(mirrored_center[0], mirrored_center[1], top_surface_z, yaw)
    return approach, release


def build_pick_place_keyframes(
    params: CubeParameters,
    *,
    gripper: Robotiq2F85 | None = None,
) -> List[PickPlaceKeyframe]:
    tool = gripper or Robotiq2F85()
    if not tool.can_grasp(params.c):
        raise ValueError(
            "The cube is too large for the modeled Robotiq 2F-85 opening."
        )

    source_approach, source_grasp = source_grasp_pose(params, gripper=tool)
    target_approach, target_release = mirrored_target_pose(params, gripper=tool)

    open_width = min(tool.max_opening, tool.required_opening(params.c) + 0.010)
    closed_width = max(params.c - 0.010, 0.0)
    safe_side_y = math.copysign(
        max(abs(params.y), 0.18),
        params.y if abs(params.y) > 1e-6 else 1.0,
    )
    home_side_y = math.copysign(max(abs(params.y), 0.32), safe_side_y)
    carry_z = max(source_approach.z, target_approach.z) + tool.lift_clearance

    # Keep the arm at least 0.42 m from the base Z-axis during transfer
    # to prevent tight elbow-folding that causes self-intersection.
    safe_transfer_y = math.copysign(
        max(abs(params.y), 0.42), safe_side_y,
    )
    transfer_carry_z = carry_z + 0.04

    source_lift = CartesianPose(source_grasp.x, source_grasp.y, carry_z, source_grasp.wrist_yaw)
    transfer_depart = CartesianPose(
        source_grasp.x * 0.35,
        safe_transfer_y,
        transfer_carry_z,
        source_grasp.wrist_yaw,
    )
    transfer_arrive = CartesianPose(
        target_release.x * 0.35,
        safe_transfer_y,
        transfer_carry_z,
        target_release.wrist_yaw,
    )
    target_lift = CartesianPose(target_release.x, target_release.y, carry_z, target_release.wrist_yaw)
    return_transit = CartesianPose(0.0, safe_transfer_y, transfer_carry_z, 0.0)
    home = CartesianPose(0.35, home_side_y, max(carry_z + 0.06, 0.58), 0.0)

    return [
        PickPlaceKeyframe("home", home, open_width, "source"),
        PickPlaceKeyframe("approach_pick", source_approach, open_width, "source"),
        PickPlaceKeyframe("grasp_pick", source_grasp, open_width, "source"),
        PickPlaceKeyframe("close_gripper", source_grasp, closed_width, "attached"),
        PickPlaceKeyframe("lift_pick", source_lift, closed_width, "attached"),
        PickPlaceKeyframe("transfer_depart", transfer_depart, closed_width, "attached"),
        PickPlaceKeyframe("transfer_arrive", transfer_arrive, closed_width, "attached"),
        PickPlaceKeyframe("approach_place", target_lift, closed_width, "attached"),
        PickPlaceKeyframe("release_place", target_release, closed_width, "attached"),
        PickPlaceKeyframe("open_gripper", target_release, open_width, "target"),
        PickPlaceKeyframe("retreat_place", target_approach, open_width, "target"),
        PickPlaceKeyframe("return_transit", return_transit, open_width, "target"),
        PickPlaceKeyframe("return_home", home, open_width, "target"),
    ]


def build_pick_place_trajectory(
    params: CubeParameters,
    *,
    gripper: Robotiq2F85 | None = None,
    steps_per_segment: int = 28,
    hold_steps: int = 12,
) -> List[PickPlaceSample]:
    keyframes = build_pick_place_keyframes(params, gripper=gripper)
    trajectory: List[PickPlaceSample] = []

    for index in range(len(keyframes) - 1):
        start = keyframes[index]
        end = keyframes[index + 1]

        same_pose = start.pose == end.pose
        steps = hold_steps if same_pose else steps_per_segment

        poses = interpolate_pose(start.pose, end.pose, steps)
        openings = interpolate_scalar(start.gripper_opening, end.gripper_opening, steps)

        for sample_index, pose in enumerate(poses):
            if trajectory and sample_index == 0:
                continue

            cube_state = start.cube_state if sample_index < len(poses) - 1 else end.cube_state
            phase = start.name if sample_index < len(poses) - 1 else end.name
            trajectory.append(
                PickPlaceSample(
                    phase=phase,
                    pose=pose,
                    gripper_opening=openings[sample_index],
                    cube_state=cube_state,
                )
            )

    return trajectory


def summarize_plan(params: CubeParameters, *, gripper: Robotiq2F85 | None = None) -> Dict[str, object]:
    tool = gripper or Robotiq2F85()
    source_approach, source_grasp = source_grasp_pose(params, gripper=tool)
    target_approach, target_release = mirrored_target_pose(params, gripper=tool)
    keyframes = build_pick_place_keyframes(params, gripper=tool)

    return {
        "parameters": asdict(params),
        "gripper_required_opening": round(tool.required_opening(params.c), 4),
        "gripper_can_grasp": tool.can_grasp(params.c),
        "cube_center_ground_frame": cube_center_ground_frame(params),
        "cube_center_robot_frame": cube_center_robot_frame(params),
        "mirror_target_center_robot_frame": mirror_point(cube_center_robot_frame(params)),
        "source_cube_yaw_rad": params.alpha,
        "target_cube_yaw_rad": mirror_yaw(params.alpha),
        "source_tool_yaw_rad": tool_yaw_from_cube(params.alpha),
        "target_tool_yaw_rad": tool_yaw_from_cube(mirror_yaw(params.alpha)),
        "approach_pick": source_approach.point,
        "grasp_pick": source_grasp.point,
        "approach_place": target_approach.point,
        "release_place": target_release.point,
        "keyframes": [
            {
                "name": frame.name,
                "pose": frame.pose.point,
                "yaw": frame.pose.wrist_yaw,
                "gripper_opening": frame.gripper_opening,
                "cube_state": frame.cube_state,
            }
            for frame in keyframes
        ],
    }


class CR10PickPlaceNode(Node):
    JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    def __init__(self) -> None:
        super().__init__("cr10_pick_place_node")

        defaults = CubeParameters()
        self.declare_parameter("c", defaults.c)
        self.declare_parameter("x", defaults.x)
        self.declare_parameter("y", defaults.y)
        self.declare_parameter("z", defaults.z)
        self.declare_parameter("alpha", defaults.alpha)
        self.declare_parameter("h", defaults.h)
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("steps_per_segment", 28)
        self.declare_parameter("hold_steps", 12)
        self.declare_parameter("repeat_demo", True)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("trace_max_points", 1000)

        self.params = CubeParameters(
            c=float(self.get_parameter("c").value),
            x=float(self.get_parameter("x").value),
            y=float(self.get_parameter("y").value),
            z=float(self.get_parameter("z").value),
            alpha=float(self.get_parameter("alpha").value),
            h=float(self.get_parameter("h").value),
        )
        self.gripper = Robotiq2F85()
        self.kinematics = CR10Kinematics()
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.repeat_demo = bool(self.get_parameter("repeat_demo").value)
        self.trace_max_points = int(self.get_parameter("trace_max_points").value)
        self.steps_per_segment = int(self.get_parameter("steps_per_segment").value)
        self.hold_steps = int(self.get_parameter("hold_steps").value)

        self.source_center = cube_center_robot_frame(self.params)
        self.target_center = mirror_point(self.source_center)
        self.trajectory = build_pick_place_trajectory(
            self.params,
            gripper=self.gripper,
            steps_per_segment=self.steps_per_segment,
            hold_steps=self.hold_steps,
        )
        self.joint_sequence = self._solve_joint_sequence(self.trajectory)

        scene_qos = QoSProfile(depth=1)
        scene_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.trace_pub = self.create_publisher(Marker, "/ee_trace", 10)
        self.scene_pub = self.create_publisher(MarkerArray, "/project_scene", scene_qos)

        self.trace_points: deque[Point] = deque(maxlen=self.trace_max_points)
        self.step_index = 0
        self.scene_pub.publish(self._build_scene_markers(self.trajectory[0]))
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        plan = summarize_plan(self.params, gripper=self.gripper)
        self.get_logger().info(
            "Pick-and-place plan ready with "
            f"{len(self.trajectory)} samples. "
            f"Source center in robot frame: {plan['cube_center_robot_frame']}, "
            f"target center: {plan['mirror_target_center_robot_frame']}."
        )

    def _on_timer(self) -> None:
        if self.step_index >= len(self.trajectory):
            if not self.repeat_demo:
                self.timer.cancel()
                self.get_logger().info("Pick-and-place sequence complete. Node is holding.")
                return
            self.step_index = 0
            self.trace_points.clear()
            self.scene_pub.publish(self._build_scene_markers(self.trajectory[0]))

        sample = self.trajectory[self.step_index]
        joints = self.joint_sequence[self.step_index]
        self.publish_pose(joints)
        self.scene_pub.publish(self._build_scene_markers(sample))
        self.step_index += 1

    def _solve_joint_sequence(
        self,
        samples: Sequence[PickPlaceSample],
    ) -> List[Tuple[float, ...]]:
        sequence: List[Tuple[float, ...]] = []
        previous_joints: Tuple[float, ...] | None = None
        for sample in samples:
            joints = self.kinematics.inverse_kinematics(
                sample.pose.x,
                sample.pose.y,
                sample.pose.z,
                tool_pitch=self.kinematics.geometry.default_tool_pitch,
                wrist_yaw=sample.pose.wrist_yaw,
                reference_joints=previous_joints,
            )
            sequence.append(joints)
            previous_joints = joints
        return sequence

    def publish_pose(self, joints: Sequence[float]) -> None:
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.JOINT_NAMES
        joint_state.position = list(joints)
        self.joint_pub.publish(joint_state)

        x, y, z = self.kinematics.forward_kinematics(joints)
        self.trace_points.append(Point(x=x, y=y, z=z))
        self.trace_pub.publish(self._make_trace_marker())

    def _make_trace_marker(self) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "pick_place_trace"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.012
        marker.color.r = 0.95
        marker.color.g = 0.45
        marker.color.b = 0.10
        marker.color.a = 1.0
        marker.lifetime = Duration(sec=0, nanosec=0)
        marker.points = list(self.trace_points)
        return marker

    def _build_scene_markers(self, sample: PickPlaceSample) -> MarkerArray:
        markers = MarkerArray()
        top_z = self.source_center[2] - self.params.c / 2.0
        table_thickness = 0.06
        table_size_x = 0.36
        table_size_y = 0.32

        source_table_center = (
            self.source_center[0],
            self.source_center[1],
            top_z - table_thickness / 2.0,
        )
        target_table_center = (
            self.target_center[0],
            self.target_center[1],
            top_z - table_thickness / 2.0,
        )

        markers.markers.append(
            self._make_box_marker(
                marker_id=1,
                namespace="source_table",
                position=source_table_center,
                size=(table_size_x, table_size_y, table_thickness),
                color=(0.55, 0.55, 0.58, 0.95),
            )
        )
        markers.markers.append(
            self._make_box_marker(
                marker_id=2,
                namespace="target_table",
                position=target_table_center,
                size=(table_size_x, table_size_y, table_thickness),
                color=(0.45, 0.52, 0.60, 0.95),
            )
        )
        markers.markers.append(
            self._make_box_marker(
                marker_id=3,
                namespace="target_ghost",
                position=self.target_center,
                size=(self.params.c, self.params.c, self.params.c),
                color=(0.15, 0.50, 0.95, 0.38),
                yaw=mirror_yaw(self.params.alpha),
            )
        )
        markers.markers.append(
            self._make_box_marker(
                marker_id=4,
                namespace="cube",
                position=self._cube_position_from_sample(sample),
                size=(self.params.c, self.params.c, self.params.c),
                color=(0.95, 0.65, 0.12, 0.98),
                yaw=self._cube_yaw_from_sample(sample),
            )
        )
        markers.markers.append(
            self._make_text_marker(
                marker_id=5,
                namespace="status_text",
                position=(0.0, -0.45, 0.55),
                text=(
                    f"phase={sample.phase} | cube={sample.cube_state} | "
                    f"gripper={sample.gripper_opening:.3f} m"
                ),
            )
        )
        return markers

    def _cube_position_from_sample(self, sample: PickPlaceSample) -> Point3:
        if sample.cube_state == "attached":
            return (
                sample.pose.x,
                sample.pose.y,
                sample.pose.z - self.params.c / 2.0,
            )
        if sample.cube_state == "target":
            return self.target_center
        return self.source_center

    def _cube_yaw_from_sample(self, sample: PickPlaceSample) -> float:
        if sample.cube_state == "attached":
            return sample.pose.wrist_yaw
        if sample.cube_state == "target":
            return mirror_yaw(self.params.alpha)
        return self.params.alpha

    def _make_box_marker(
        self,
        *,
        marker_id: int,
        namespace: str,
        position: Point3,
        size: Point3,
        color: Tuple[float, float, float, float],
        yaw: float = 0.0,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x = size[0]
        marker.scale.y = size[1]
        marker.scale.z = size[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = Duration(sec=0, nanosec=0)
        return marker

    def _make_text_marker(
        self,
        *,
        marker_id: int,
        namespace: str,
        position: Point3,
        text: str,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.95
        marker.text = text
        marker.lifetime = Duration(sec=0, nanosec=0)
        return marker


def main() -> None:
    rclpy.init()
    node = CR10PickPlaceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
