from os import environ
from ament_index_python.packages import get_package_share_directory
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import ExecuteProcess
from launch.conditions import LaunchConfigurationEquals, IfCondition
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument('x', default_value=['0'],
        description='x position'),
    DeclareLaunchArgument('y', default_value=['-0.3'],
        description='y position'),
    DeclareLaunchArgument('z', default_value=['0'],
        description='z position'),
    DeclareLaunchArgument('yaw', default_value=['0'],
        description='yaw position'),
    # DeclareLaunchArgument('rviz', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Start rviz.'),
    # DeclareLaunchArgument('sync', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run async or sync SLAM'),
    # DeclareLaunchArgument('localization', default_value='slam',
    #                       choices=['off', 'localization', 'slam'],
    #                       description='Whether to run localization or SLAM'),
    # DeclareLaunchArgument('nav2', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run nav2'),
    DeclareLaunchArgument('corti', default_value='true',
                          choices=['true', 'false'],
                          description='Run corti'),
    # DeclareLaunchArgument('cerebri', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run cerebri'),
    DeclareLaunchArgument('bridge', default_value='true',
                          choices=['true', 'false'],
                          description='Run bridges'),
    # DeclareLaunchArgument('synapse_ros', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run synapse_ros'),
    # DeclareLaunchArgument('synapse_gz', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run synapse_gz'),
    # DeclareLaunchArgument('joy', default_value='true',
    #                       choices=['true', 'false'],
    #                       description='Run joy'),
    DeclareLaunchArgument('description', default_value='true',
                          choices=['true', 'false'],
                          description='Run description'),
    DeclareLaunchArgument('world', default_value='default',
                          description='GZ World'),
    # DeclareLaunchArgument(
    #     'map_yaml',
    #     default_value=[LaunchConfiguration('world'), '.yaml'],
    #     description='Map yaml'),
    # DeclareLaunchArgument('cerebri_gdb', default_value='false',
    #                       choices=['true', 'false'],
    #                       description='Run cerebri with gdb debugger.'),
    # DeclareLaunchArgument('uart_shell', default_value='false',
    #                       choices=['true', 'false'],
    #                       description='Run cerebri with UART shell.'),
    DeclareLaunchArgument('spawn_model', default_value='true',
                          choices=['true', 'false'],
                          description='Spawn MR Buggy3 Model'),
    DeclareLaunchArgument('use_sim_time', default_value='true',
                          choices=['true', 'false'],
                          description='Use sim time'),
    # DeclareLaunchArgument('log_level', default_value='error',
    #                       choices=['info', 'warn', 'error'],
    #                       description='log level'),
]


def generate_launch_description():

    pkg_project_description = get_package_share_directory('mrbuggy3_gz_resource')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #         [get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])]),
    #     launch_arguments=[('gz_args', [
    #         LaunchConfiguration('world'), '.sdf', ' -v 0', ' -r'
    #         ])]
    # )
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': PathJoinSubstitution([
            pkg_project_description,
            'worlds',
            'default.sdf'
        ])}.items(),
    )
    # Setup to launch the simulator and Gazebo world
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': PathJoinSubstitution([
    #         pkg_project_description,
    #         'worlds',
    #         'default.sdf' 
    #     ])}.items(),
    # )
        # Robot description
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory('mrbuggy3_description'), 'launch', 'robot_description.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('description')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-world', 'default',
            '-name', 'mrbuggy3',
            '-x', LaunchConfiguration('x'),
            '-y', LaunchConfiguration('y'),
            '-z', LaunchConfiguration('z'),
            '-Y', LaunchConfiguration('yaw'),
            '-file', PathJoinSubstitution([get_package_share_directory(
                'mrbuggy3_gz_resource'),
                'models/mrbuggy3/model.sdf'])
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration("spawn_model")))
    # pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    # pkg_project_description = get_package_share_directory('mrbuggy3_gz_resource')
    # gz_sim = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
    #     launch_arguments={'gz_args': PathJoinSubstitution([
    #         pkg_project_description,
    #         'worlds',
    #         'default.sdf'
    #     ])}.items(),
    # )
    # cerebri = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #         [get_package_share_directory('cerebri_bringup'), 'launch', 'cerebri.launch.py'])]),
    #     condition=IfCondition(LaunchConfiguration('cerebri')),
    #     launch_arguments=[('gdb', LaunchConfiguration('cerebri_gdb')),
    #                       ('vehicle', 'mrbuggy3'),
    #                       ('uart_shell', LaunchConfiguration('uart_shell'))],
    # )

    # joy = Node(
    #     package='joy',
    #     executable='joy_node',
    #     output='screen',
    #     arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    #     condition=IfCondition(LaunchConfiguration('joy')),
    #     parameters=[{
    #         'use_sim_time': LaunchConfiguration('use_sim_time')
    #         }],
    #     remappings=[('/joy', '/cerebri/in/joy')]
    # )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=[
            '/world/default/model/mrbuggy3/link/lidar_link/sensor/lidar/scan' +
             '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        remappings=[
            ('/world/default/model/mrbuggy3/link/lidar_link/sensor/lidar/scan',
             '/scan')
        ])


    cam_rgb_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=[
            '/world/default/model/mrbuggy3/link/camera_link/sensor/camera/image' +
             '@sensor_msgs/msg/Image[gz.msgs.Image'
        ],
        remappings=[
            ('/world/default/model/mrbuggy3/link/camera_link/sensor/camera/image',
             '/rgb_image'
             )
        ])
    cam_info_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        arguments=[
            '/world/default/model/mrbuggy3/link/camera_link/sensor/camera/camera_info' +
             '@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'         
        ],
        remappings=[
            ('/world/default/model/mrbuggy3/link/camera_link/sensor/camera/camera_info',
             '/camera_info'
             )
        ])
    odom_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        condition=IfCondition(LaunchConfiguration('bridge')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
        arguments=[
            '/model/mrbuggy3/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry'
            ],
        remappings=[
            ('/model/mrbuggy3/odometry', '/odom')
            ])

    odom_base_tf_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('bridge')),
        arguments=[
           ['/model/mrbuggy3/pose' +
            '@tf2_msgs/msg/TFMessage' +
            '[gz.msgs.Pose_V']
        ],
        remappings=[
           (['/model/mrbuggy3/pose'], '/tf')
        ])
    cmd_vel_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('bridge')),
        arguments=[
            ['/cmd_vel' +
            '@geometry_msgs/msg/Twist' +
            '[gz.msgs.Twist']
        ],
        remappings=[
            (['/cmd_vel'],
            '/cmd_vel')
        ]
    )
    pose_bridge = Node(
        package='ros_gz_bridge', 
        executable='parameter_bridge',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('bridge')),
        arguments=[
           ['/model/mrbuggy3/pose' +
            '@tf2_msgs/msg/TFMessage' +
            '[gz.msgs.Pose_V']
        ],
        remappings=[
           (['/model/mrbuggy3/pose'],
            '/_internal/sim_ground_truth_pose')
        ])


    # rviz2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #     [get_package_share_directory(
    #     'mrbuggy3_rviz'), 'launch', 'view_robot.launch.py'])]),
    #     condition=IfCondition(LaunchConfiguration('rviz')),
    #     launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    # nav2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #     [get_package_share_directory(
    #     'mrbuggy3_nav2'), 'launch', 'nav2.launch.py'])]),
    #     condition=IfCondition(LaunchConfiguration('nav2')),
    #     launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    corti = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([PathJoinSubstitution(
        [get_package_share_directory('corti'), 'launch', 'corti.launch.py'])]),
        condition=IfCondition(LaunchConfiguration('corti')),
        launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time'))])

    # slam = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #     [get_package_share_directory(
    #     'mrbuggy3_nav2'), 'launch', 'slam.launch.py'])]),
    #     condition=LaunchConfigurationEquals('localization', 'slam'),
    #     launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
    #         ('sync', LaunchConfiguration('sync'))])

    # localization = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([PathJoinSubstitution(
    #     [get_package_share_directory(
    #     'mrbuggy3_nav2'), 'launch', 'localization.launch.py'])]),
    #     condition=LaunchConfigurationEquals('localization', 'localization'),
    #     launch_arguments=[('use_sim_time', LaunchConfiguration('use_sim_time')),
    #         ('map', PathJoinSubstitution([get_package_share_directory(
    #             'mrbuggy3_nav2'), 'maps', LaunchConfiguration('map_yaml')]))])

    tf_to_odom = Node(
        condition=IfCondition(LaunchConfiguration('corti')),
        package='corti',
        executable='tf_to_odom',
        output='screen',
        parameters=[{
            'base_frame': 'map',
            'target_frame': 'base_link',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        remappings=[
            ('/odom', '/cerebri/in/odometry')
            ])

    odom_to_tf = Node(
        condition=IfCondition(LaunchConfiguration('corti')),
        package='corti',
        executable='odom_to_tf',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        remappings=[
            ('/odom', '/cerebri/out/odometry')
            ])

    # Define LaunchDescription variable
    return LaunchDescription(ARGUMENTS + [
        robot_description,
        # synapse_ros,
        # synapse_gz,
        gz_sim,
        # cerebri,
        # joy,
        odom_bridge,
        clock_bridge,
        lidar_bridge,
        odom_base_tf_bridge,
        pose_bridge,
        # rviz2,
        spawn_robot,
        # nav2,
        corti,
        # slam,
        # localization,
        cmd_vel_bridge,
        tf_to_odom,
        odom_to_tf,
        cam_info_bridge,
        cam_rgb_bridge,
    ])
