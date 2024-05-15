from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("odrive_demo_description"),
                    "urdf",
                    "odrive_diffbot.urdf.xacro"
                ]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("odrive_demo_bringup"),
            "config",
            "diffbot_controllers.yaml",
        ]
    )

    # rviz_config_file = PathJoinSubstitution(
    #     [
    #         FindPackageShare("diffbot_description"),
    #         "config",
    #         "diffbot.rviz"
    #     ]
    # )

    rviz_config_file = PathJoinSubstitution(
        [
            FindPackageShare("zalpiano_desc"),
            "rviz",
            "urdf_config.rviz"
        ]
    )

    ekf_aruco_config_file = PathJoinSubstitution(
        [
            FindPackageShare("zalpiano_nav"),
            "config",
            "ekf_aruco.yaml"
        ]
    )

    ekf_encoder_config_file = PathJoinSubstitution(
        [
            FindPackageShare("zalpiano_nav"),
            "config",
            "ekf_encoder.yaml"
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["zalpiano_base_controller", "-c", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
    )


    robot_localization_node_aruco = Node(
        package='robot_localization',
        executable='ekf_node',
        name='aruco_ekf_filter_node',
        output='screen',
        parameters=[ekf_aruco_config_file, {'use_sim_time': False}]
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
        # rviz_node,
        robot_localization_node_aruco,
    ])
