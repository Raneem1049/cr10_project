#!/usr/bin/env python3

from __future__ import annotations

from collections import deque
import ast
import math
from typing import Iterable, List, Sequence, Tuple

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from rclpy.exceptions import InvalidParameterTypeException
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile

from .cr10_kinematics import CR10Kinematics
from .trajectory_generators import circle_parallel_to_ground, interpolate_line


Point3 = Tuple[float, float, float]


class CR10IKRVizNode(Node):
    JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]

    def __init__(self) -> None:
        super().__init__("cr10_ik_rviz_node")

        self.kinematics = CR10Kinematics()
        self.base_frame = str(self.declare_parameter("base_frame", "base_link").value)
        self.mode = str(self.declare_parameter("mode", "demo").value).lower()
        self.repeat_demo = bool(self.declare_parameter("repeat_demo", True).value)
        self.publish_rate_hz = float(self.declare_parameter("publish_rate_hz", 30.0).value)
        self.trace_max_points = int(self.declare_parameter("trace_max_points", 800).value)
        self.line_steps = int(self.declare_parameter("line_steps", 90).value)
        self.circle_steps = int(self.declare_parameter("circle_steps", 160).value)
        self.transition_steps = int(self.declare_parameter("transition_steps", 35).value)
        self.hold_steps = int(self.declare_parameter("hold_steps", 15).value)
        self.tool_pitch = math.radians(
            float(self.declare_parameter("tool_pitch_deg", 90.0).value)
        )

        self.line_start = self._read_vector_parameter("line_start", [0.48, 0.10, 0.32])
        self.line_end = self._read_vector_parameter("line_end", [0.40, -0.15, 0.36])
        self.circle_center = self._read_vector_parameter("circle_center", [0.42, 0.00, 0.34])
        self.circle_radius = float(self.declare_parameter("circle_radius", 0.08).value)

        scene_qos = QoSProfile(depth=1)
        scene_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.trace_pub = self.create_publisher(Marker, "/ee_trace", 10)
        self.scene_pub = self.create_publisher(MarkerArray, "/project_scene", scene_qos)

        self.trace_points: deque[Point] = deque(maxlen=self.trace_max_points)
        self.reference_markers = self._build_reference_markers()
        self.cartesian_sequence = self._build_cartesian_sequence()
        self.joint_sequence = self._solve_joint_sequence(self.cartesian_sequence)

        self.step_index = 0
        self.scene_pub.publish(self.reference_markers)
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)

        self.get_logger().info(
            f"CR10 IK RViz node ready in '{self.mode}' mode with "
            f"{len(self.joint_sequence)} prevalidated samples."
        )

    def _read_vector_parameter(self, name: str, default: Sequence[float]) -> Point3:
        try:
            value = self.declare_parameter(name, list(default)).value
        except InvalidParameterTypeException:
            value = self.declare_parameter(name, str(list(default))).value
        if isinstance(value, str):
            value = ast.literal_eval(value)
        vector = tuple(float(component) for component in value)
        if len(vector) != 3:
            raise ValueError(f"Parameter '{name}' must contain exactly three values.")
        return vector  # type: ignore[return-value]

    def _build_cartesian_sequence(self) -> List[Point3]:
        if self.mode == "line":
            return interpolate_line(self.line_start, self.line_end, self.line_steps)

        if self.mode == "circle":
            return circle_parallel_to_ground(
                self.circle_center,
                self.circle_radius,
                self.circle_steps,
            )

        if self.mode != "demo":
            raise ValueError("Mode must be one of: line, circle, demo.")

        line = interpolate_line(self.line_start, self.line_end, self.line_steps)
        circle = circle_parallel_to_ground(
            self.circle_center,
            self.circle_radius,
            self.circle_steps,
        )
        transition = interpolate_line(line[-1], circle[0], self.transition_steps)

        sequence: List[Point3] = []
        sequence.extend(line)
        sequence.extend([line[-1]] * self.hold_steps)
        sequence.extend(transition[1:])
        sequence.extend([circle[0]] * self.hold_steps)
        sequence.extend(circle)
        sequence.extend([circle[-1]] * self.hold_steps)
        return sequence

    def _solve_joint_sequence(self, points: Sequence[Point3]) -> List[Tuple[float, ...]]:
        sequence: List[Tuple[float, ...]] = []
        previous_joints: Tuple[float, ...] | None = None
        for point in points:
            joints = self.kinematics.inverse_kinematics(
                *point,
                tool_pitch=self.tool_pitch,
                reference_joints=previous_joints,
            )
            sequence.append(joints)
            previous_joints = joints
        return sequence

    def _on_timer(self) -> None:
        if self.step_index >= len(self.joint_sequence):
            if not self.repeat_demo:
                self.timer.cancel()
                self.get_logger().info("Trajectory complete. Node is holding the last pose.")
                return
            self.step_index = 0
            self.trace_points.clear()
            self.scene_pub.publish(self.reference_markers)

        joints = self.joint_sequence[self.step_index]
        self.publish_pose(joints)
        self.step_index += 1

    def publish_pose(self, joints: Iterable[float]) -> None:
        position = list(joints)
        joint_state = JointState()
        joint_state.header = Header()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = self.JOINT_NAMES
        joint_state.position = position
        self.joint_pub.publish(joint_state)

        x, y, z = self.kinematics.forward_kinematics(position)
        self.trace_points.append(Point(x=x, y=y, z=z))
        self.publish_trace()

    def publish_trace(self) -> None:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "end_effector_trace"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.012
        marker.color.a = 1.0
        marker.color.r = 0.95
        marker.color.g = 0.25
        marker.color.b = 0.15
        marker.lifetime = Duration(sec=0, nanosec=0)
        marker.points = list(self.trace_points)
        self.trace_pub.publish(marker)

    def _build_reference_markers(self) -> MarkerArray:
        markers = MarkerArray()
        marker_id = 0

        if self.mode in {"line", "demo"}:
            markers.markers.append(
                self._make_sphere_marker(
                    marker_id,
                    "line_start",
                    self.line_start,
                    (0.10, 0.70, 0.10, 0.95),
                    scale=0.04,
                )
            )
            marker_id += 1
            markers.markers.append(
                self._make_sphere_marker(
                    marker_id,
                    "line_end",
                    self.line_end,
                    (0.80, 0.20, 0.20, 0.95),
                    scale=0.04,
                )
            )
            marker_id += 1

        if self.mode in {"circle", "demo"}:
            markers.markers.append(
                self._make_sphere_marker(
                    marker_id,
                    "circle_center",
                    self.circle_center,
                    (0.15, 0.35, 0.90, 0.95),
                    scale=0.03,
                )
            )
            marker_id += 1

            reference_circle = circle_parallel_to_ground(
                self.circle_center,
                self.circle_radius,
                120,
            )
            markers.markers.append(
                self._make_line_strip_marker(
                    marker_id,
                    "circle_reference",
                    reference_circle,
                    (0.15, 0.35, 0.90, 0.45),
                    width=0.008,
                )
            )

        return markers

    def _make_sphere_marker(
        self,
        marker_id: int,
        namespace: str,
        point: Point3,
        color: Tuple[float, float, float, float],
        *,
        scale: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = point[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.lifetime = Duration(sec=0, nanosec=0)
        return marker

    def _make_line_strip_marker(
        self,
        marker_id: int,
        namespace: str,
        points: Sequence[Point3],
        color: Tuple[float, float, float, float],
        *,
        width: float,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.points = [Point(x=p[0], y=p[1], z=p[2]) for p in points]
        marker.lifetime = Duration(sec=0, nanosec=0)
        return marker


def main() -> None:
    rclpy.init()
    node = CR10IKRVizNode()
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
