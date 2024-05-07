import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os

def generate_launch_description():
    pkg_share = launch_ros.substitutions.FindPackageShare(package='boxing_bot_description').find('boxing_bot_description')
    default_model_path = os.path.join(pkg_share, 'src/description/boxing_bot_description.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')

    # Define the path to the world file
    world_file_path = os.path.expanduser('~/test.world')  # Modify this path as necessary

    map_file_path = LaunchConfiguration('map_file_path', default=os.path.expanduser('~/ros2_ws/src/map/map.yaml'))  # Use LaunchConfiguration for flexibility

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])}]
    )

    # # Updated command to include the world file
    # gazebo_process = launch.actions.ExecuteProcess(
    #     cmd=['gazebo', '--verbose', world_file_path, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'], output='screen',
    # )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        arguments=[default_model_path],
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    # spawn_entity = launch_ros.actions.Node(
    # package='gazebo_ros',
    # executable='spawn_entity.py',
    # arguments=['-entity', 'boxing_bot', '-topic', 'robot_description'],
    # output='screen'
    # )


    robot_localization_node_continuous = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_continuous',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf_continuous.yaml'), {'use_sim_time': False}]
    )


    robot_localization_node_aruco_0 = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_aruco_0',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf_aruco_0.yaml'), {'use_sim_time': False}]
    )

    robot_localization_node_aruco_1 = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_aruco_1',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf_aruco_1.yaml'), {'use_sim_time': False}]
    )

    robot_localization_node_aruco_2 = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_aruco_2',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf_aruco_2.yaml'), {'use_sim_time': False}]
    )

    robot_localization_node_aruco_3 = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_aruco_3',
        output='screen',
        parameters=[os.path.join(pkg_share, 'config/ekf_aruco_3.yaml'), {'use_sim_time': False}]
    )


    # Nav2 Map Server Node
    map_server_node = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file_path,
            'use_sim_time': True            
            }],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]
    )

    nav_gazebo_test_node = launch_ros.actions.Node(
        package='nav_gazebo_test',
        executable='nav_gazebo',
        name='nav_gazebo',
        output='screen',
        parameters=[{'use_sim_time': True}])


    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        launch.actions.DeclareLaunchArgument(name='use_sim_time', default_value='True',
                                            description='Flag to enable use_sim_time'),
        launch.actions.DeclareLaunchArgument(name='dont_use_sim_time', default_value='False',
                                            description='Flag to disable use_sim_time'),
        launch.actions.DeclareLaunchArgument(
            name='map_file_path',
            default_value=os.path.expanduser('~/ros2_ws/src/map/map.yaml'),
            description='Path to map yaml file'
        ),

        joint_state_publisher_node,
        robot_state_publisher_node,
        # gazebo_process,  # Ensure this is added to the launch description
        # spawn_entity,
        map_server_node,
        robot_localization_node_aruco_0,
        # robot_localization_node_aruco_1,
        # robot_localization_node_aruco_2,
        # robot_localization_node_aruco_3,
        robot_localization_node_continuous,
        rviz_node,
        nav_gazebo_test_node
    ])
