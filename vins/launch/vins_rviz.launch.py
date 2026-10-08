from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_name = 'vins'
    pkg_share = get_package_share_directory(package_name)

    config_arg = DeclareLaunchArgument(
        'config',
        description='Path to VINS config file (YAML)'
    )

    bag_arg = DeclareLaunchArgument(
        'bag_folder',
        description='Path to rosbag2 folder to play'
    )

    qos_overrides_arg = DeclareLaunchArgument(
        'qos_overrides_path',
        default_value='',
        description='Optional path to a QoS profile overrides YAML file, '
                     'passed to "ros2 bag play --qos-profile-overrides-path". '
                     'Leave empty to disable.'
    )

    config = LaunchConfiguration('config')
    bag_folder = LaunchConfiguration('bag_folder')
    qos_overrides_path = LaunchConfiguration('qos_overrides_path')
    has_qos_overrides = PythonExpression(["'", qos_overrides_path, "' != ''"])

    rviz_config = os.path.join(pkg_share, 'rviz', 'vins_rviz_config_ros2.rviz')

    vins_node = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'vins', 'vins_node',
            config
        ],
        output='screen'
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )

    bag_play = TimerAction(
        period=3.0,
        actions=[
            # Used when qos_overrides_path is set.
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag_folder,
                     '--qos-profile-overrides-path', qos_overrides_path],
                output='screen',
                condition=IfCondition(has_qos_overrides)
            ),
            # Used when qos_overrides_path is left empty (default).
            ExecuteProcess(
                cmd=['ros2', 'bag', 'play', bag_folder],
                output='screen',
                condition=UnlessCondition(has_qos_overrides)
            )
        ]
    )

    return LaunchDescription([
        config_arg,
        bag_arg,
        qos_overrides_arg,
        vins_node,
        rviz2,
        bag_play,
    ])
