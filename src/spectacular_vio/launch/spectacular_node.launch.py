from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    spectacularai_node = Node(
        package='spectacular_vio',
        executable='spectacular_node',
        parameters=[
            { 'recordingFolder': LaunchConfiguration("recordingFolder") },
        ],
    )

    pkg_spectacular_vio = FindPackageShare("spectacular_vio")
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz").perform(context)),
        package='rviz2', executable='rviz2', output='screen',
        arguments=[
            '-d', PathJoinSubstitution([
                FindPackageShare("spectacular_vio"),
                'spectacular.rviz'
            ])
        ]
    )

    return [
        spectacularai_node,
        rviz_node
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value='True'),
            DeclareLaunchArgument("recordingFolder", default_value='')
        ] + [
            OpaqueFunction(function=launch_setup)
        ]
    )
