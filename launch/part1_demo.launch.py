from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = 'cr10_project'
SHARE_DIR = Path(get_package_share_directory(PACKAGE_NAME))
URDF_PATH = SHARE_DIR / 'urdf' / 'cr10_project.urdf'
RVIZ_PATH = SHARE_DIR / 'rviz' / 'project_demo.rviz'


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration('use_rviz')
    mode = LaunchConfiguration('mode')
    repeat_demo = LaunchConfiguration('repeat_demo')
    line_start = LaunchConfiguration('line_start')
    line_end = LaunchConfiguration('line_end')
    circle_center = LaunchConfiguration('circle_center')
    circle_radius = LaunchConfiguration('circle_radius')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('mode', default_value='demo'),
            DeclareLaunchArgument('repeat_demo', default_value='true'),
            DeclareLaunchArgument('line_start', default_value='[0.48, 0.10, 0.32]'),
            DeclareLaunchArgument('line_end', default_value='[0.40, -0.15, 0.36]'),
            DeclareLaunchArgument('circle_center', default_value='[0.42, 0.00, 0.34]'),
            DeclareLaunchArgument('circle_radius', default_value='0.08'),
            SetEnvironmentVariable('RCUTILS_COLORIZED_OUTPUT', '1'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                parameters=[{'robot_description': URDF_PATH.read_text(encoding='utf-8')}],
                output='screen',
            ),
            ExecuteProcess(
                cmd=['rviz2', '-d', str(RVIZ_PATH)],
                output='screen',
                condition=IfCondition(use_rviz),
            ),
            Node(
                package=PACKAGE_NAME,
                executable='cr10_ik_rviz_node',
                output='screen',
                parameters=[
                    {
                        'mode': ParameterValue(mode, value_type=str),
                        'repeat_demo': ParameterValue(repeat_demo, value_type=bool),
                        'line_start': ParameterValue(line_start, value_type=str),
                        'line_end': ParameterValue(line_end, value_type=str),
                        'circle_center': ParameterValue(circle_center, value_type=str),
                        'circle_radius': ParameterValue(circle_radius, value_type=float),
                    }
                ],
            ),
        ]
    )
