import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node

def generate_launch_description():

    robot_name='mrbuggy3'

    # Setup project paths
    pkg_project_bringup = get_package_share_directory('mrbuggy3_gz_bringup')
    pkg_project_description = get_package_share_directory('mrbuggy3_gz_resource')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Load the SDF file from "description" package
    sdf_file  =  os.path.join(pkg_project_description, 'models', 'mrbuggy3', 'model.sdf')
    with open(sdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_description,
            'worlds',
            'default.sdf'
        ])}.items(),
    )

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_project_bringup, 'config', 'ros_gz_bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }],
        output='screen'
    )

    # Include odom_to_tf.launch.py and pass frame parameters
    # odom_to_tf = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_project_bringup, 'launch', 'odom_to_tf.launch.py')
    #     ),
    #     launch_arguments={
    #         'frame_id': LaunchConfiguration('frame_id'),
    #         'child_frame_id': LaunchConfiguration('child_frame_id')
    #     }.items()
    # )

    # Rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='sim_rviz2',
        arguments=['-d' + os.path.join(pkg_project_bringup, 'rviz', 'rviz.rviz')]
    )

    return LaunchDescription([
        gz_sim,
        DeclareLaunchArgument('frame_id', default_value='odom', description='Frame ID of the parent frame'),
        DeclareLaunchArgument('child_frame_id', default_value='base_link', description='Frame ID of the child frame'),
        bridge,
        robot_state_publisher,
        rviz_node,
        # odom_to_tf
    ])
