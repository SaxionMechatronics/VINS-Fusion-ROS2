from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    vins_pkg_share = get_package_share_directory('vins')

    config = LaunchConfiguration('config')
    bag_folder = LaunchConfiguration('bag_folder')

    config_arg = DeclareLaunchArgument(
        'config',
        description='Path to VINS config file (YAML)'
    )

    bag_arg = DeclareLaunchArgument(
        'bag_folder',
        description='Path to rosbag2 folder to play'
    )

    vins_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                vins_pkg_share,
                'launch',
                'vins_rviz.launch.py'
            ])
        ),
        launch_arguments={
            'config': config,
            'bag_folder': bag_folder,
        }.items()
    )

    loop_closure_proc = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'loop_fusion', 'loop_fusion_node',
            config
        ],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        bag_arg,
        vins_rviz_launch,
        loop_closure_proc,
    ])
