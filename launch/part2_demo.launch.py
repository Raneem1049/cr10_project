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
    repeat_demo = LaunchConfiguration('repeat_demo')
    c = LaunchConfiguration('c')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    alpha = LaunchConfiguration('alpha')
    h = LaunchConfiguration('h')

    return LaunchDescription(
        [
            DeclareLaunchArgument('use_rviz', default_value='true'),
            DeclareLaunchArgument('repeat_demo', default_value='true'),
            DeclareLaunchArgument('c', default_value='0.05'),
            DeclareLaunchArgument('x', default_value='0.48'),
            DeclareLaunchArgument('y', default_value='0.18'),
            DeclareLaunchArgument('z', default_value='0.90'),
            DeclareLaunchArgument('alpha', default_value='0.3490658503988659'),
            DeclareLaunchArgument('h', default_value='0.65'),
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
                executable='cr10_pick_place',
                output='screen',
                parameters=[
                    {
                        'repeat_demo': ParameterValue(repeat_demo, value_type=bool),
                        'c': ParameterValue(c, value_type=float),
                        'x': ParameterValue(x, value_type=float),
                        'y': ParameterValue(y, value_type=float),
                        'z': ParameterValue(z, value_type=float),
                        'alpha': ParameterValue(alpha, value_type=float),
                        'h': ParameterValue(h, value_type=float),
                    }
                ],
            ),
        ]
    )
