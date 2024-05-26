from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    ld = LaunchDescription()

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

    nav2_params_file = PathJoinSubstitution(
        [
            FindPackageShare("zalpiano_nav"),
            "config",
            "nav2_params.yaml"
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

    #for lifecycle configure and activate 
    #see https://answers.ros.org/question/398095/ros2-nav2-map_server-problems-loading-map-with-nav2_map_server/
    lifecycle_nodes = ['map_server']
    autostart = True
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,  # see https://github.com/ros2/launch/issues/188
        parameters=[{'autostart': autostart},
                    {'node_names': lifecycle_nodes}])


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





    # Include external launch files
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'navigation_launch.py'
        ])),
        launch_arguments={'params_file': nav2_params_file}.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('nav2_bringup'), 'launch', 'rviz_launch.py'
        ]))
    )

    ld.add_action(aruco_detect_node)
    ld.add_action(aruco_to_center_node)
    ld.add_action(rviz_node)
    ld.add_action(map_server_node)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(twist_mux_node)
    ld.add_action(joy_node)
    ld.add_action(joy_teleop_node)
    ld.add_action(goal_pose_publisher_node)
    ld.add_action(joy_e_stop_node)
    ld.add_action(e_stop_button_node)

    ld.add_action(navigation_launch)
    ld.add_action(rviz_launch)

    return ld
