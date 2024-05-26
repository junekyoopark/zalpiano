from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_file_path = PathJoinSubstitution(
        [
            FindPackageShare("zalpiano_nav"),
            "map",
            "map.yaml"
        ]
    )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("zalpiano_desc"),
            "rviz",
            "urdf_config.rviz"
        ]
    )

    twist_mux_params = PathJoinSubstitution(
        [
            FindPackageShare("zalpiano_nav"),
            "config",
            "twist_mux.yaml"
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )

    # Nav2 Map Server Node
    map_server_node = Node(
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

    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/zalpiano_base_controller/cmd_vel_unstamped')]
    )

    joy_node = Node(
        package = "joy",
        executable="joy_node",
        name="joy",
        parameters=[{
            'device_name': "DualSense Wireless Controller",
        }],
    )

    joy_teleop_node = Node(
        package="p9n_node",
        executable="teleop_twist_joy_node_exec",
        name="joy_teleop",
    )

    goal_pose_publisher_node = Node(
        package="zalpiano_detect",
        executable="goal_pose_publisher",
        name="goal_pose_publisher",
    )

    joy_e_stop_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_e_stop",
        parameters=[{
            'device_name': "SparkFun Pro Micro",
        }],
        remappings=[('/joy', '/joy_e_stop')]
    )

    e_stop_button_node = Node(
        package="zalpiano_nav",
        executable="e_stop_button",
        name="e_stop_button",
    )

    return LaunchDescription([
        aruco_detect_node,
        aruco_to_center_node,
        rviz_node,
        map_server_node,
        twist_mux_node,
        joy_node,
        joy_teleop_node,
        goal_pose_publisher_node,
        joy_e_stop_node,
        e_stop_button_node,
    ])
