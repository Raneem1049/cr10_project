#!/usr/bin/env python3

from __future__ import annotations

from pathlib import Path
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node


PROJECT_DIR = Path(__file__).resolve().parent
URDF_PATH = PROJECT_DIR / "urdf" / "cr10_project.urdf"
RVIZ_PATH = PROJECT_DIR / "rviz" / "project_demo.rviz"


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": URDF_PATH.read_text(encoding="utf-8")}],
                output="screen",
            ),
            ExecuteProcess(
                cmd=["rviz2", "-d", str(RVIZ_PATH)],
                output="screen",
            ),
            ExecuteProcess(
                cmd=[sys.executable, "-m", "cr10_project.cr10_ik_rviz_node"],
                output="screen",
            ),
        ]
    )


def main() -> int:
    launch_service = LaunchService()
    launch_service.include_launch_description(generate_launch_description())
    return launch_service.run()


if __name__ == "__main__":
    raise SystemExit(main())
