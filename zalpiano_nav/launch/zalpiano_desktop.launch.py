from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import launch_ros
import os


def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='zalpiano_nav').find('zalpiano_nav')
    map_file_path = os.path.join(pkg_share, 'map/map.yaml')

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("zalpiano_desc"),
            "rviz",
            "urdf_config.rviz"
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    # Nav2 Map Server Node
    map_server_node = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file_path,
            'use_sim_time': False            
            }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    aruco_detect_node = Node(
        package="zalpiano_detect",
        executable="aruco_detect",
        name="aruco_detect",
    )

    aruco_to_center_node = Node(
        package="zalpiano_detect",
        executable="aruco_to_center",
        name="aruco_to_center",
    )

    return LaunchDescription([
        aruco_detect_node,
        aruco_to_center_node,
        rviz_node,
        map_server_node,
    ])
