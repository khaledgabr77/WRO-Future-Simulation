import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable

from launch_ros.actions import Node
from math import radians

def generate_launch_description():


    pkg_robot_system = get_package_share_directory('robot_system')
    
    
    # Path to the robot_system robot description launch file
    robot_description_launch = os.path.join(
        get_package_share_directory('robot_system'),
        'launch',
        'robot_description.launch.py'  
    )
    # Include the robot_system launch file
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(robot_description_launch)
    )

    # Path to the rplidar_ros launch file
    rplidar_ros_launch = os.path.join(
        get_package_share_directory('rplidar_ros'),
        'launch',
        'rplidar_c1_launch.py'  
    )
    # Include the rplidar_ros launch file
    rplidar_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_ros_launch),
        launch_arguments={
            'serial_port': EnvironmentVariable('RPLIDAR_SERIAL_PORT', default_value='/dev/lidar'), 
            'frame_id' : 'lidar_link',
            'inverted': 'false',
            'flip_x_axis' : 'true',
            'scan_mode' : 'Standard'
        }.items()
    )

    # Path to the camera_publisher launch file
    camera_publisher_launch = os.path.join(
        get_package_share_directory('camera_publisher'),
        'launch',
        'cam_publisher_cpp.launch.py'  
    )
    # Include the camera_publisher launch file
    camera_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_publisher_launch)
    )
    
    # Path to the perception launch file
    perception_launch = os.path.join(
        get_package_share_directory('robot_perception'),
        'launch',
        'perception.launch.py'  
    )

    # Include the perception launch file
    perception_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(perception_launch),
        launch_arguments={
            'yolo_model': EnvironmentVariable('YOLO_MODEL_PATH', default_value=''), 
        }.items()
    )

    # Path to the ftg_launch file
    ftg_launch_path = os.path.join(
        get_package_share_directory('robot_controllers'),
        'launch',
        'ftg_python.launch.py'  
    )

    # Include the ftg_launch file
    ftg_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ftg_launch_path)
    )

    # Path to the ftg_launch file
    ftg_cpp_launch_path = os.path.join(
        get_package_share_directory('robot_controllers'),
        'launch',
        'ftg.launch.py'  
    )

    # Include the ftg_launch file
    ftg_cpp_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ftg_cpp_launch_path),
        launch_arguments={
            'wheel_base': EnvironmentVariable('WHEEL_BASE', default_value='0.165'),
            'car_width': EnvironmentVariable('CAR_WIDTH', default_value='0.127'),
            'wheel_radius': EnvironmentVariable('WHEEL_RADIUS', default_value='0.025'),
            'safety_radius': EnvironmentVariable('SAFETY_RADIUS', default_value='0.12'),
            'bubble_safety_radius': EnvironmentVariable('BUBBLE_SAFETY_RADIUS', default_value='0.0335'),
            'speed_safety_radius': EnvironmentVariable('SPEED_SAFETY_RADIUS', default_value='0.10'),
            'max_speed': EnvironmentVariable('MAX_SPEED', default_value='0.3'),
            'min_speed': EnvironmentVariable('MIN_SPEED', default_value='0.15'),
            'max_range': EnvironmentVariable('MAX_RANGE', default_value='3.0'),
            'field_of_view': EnvironmentVariable('FIELD_OF_VIEW', default_value='170.0'),
            'enable_disparity_extender': EnvironmentVariable('ENABLE_DISPARITY_EXTENDER', default_value='true'),
            'enable_corner_case': EnvironmentVariable('ENABLE_CORNER_CASE', default_value='false'),
            'use_labeled_scan': EnvironmentVariable('USE_LABELED_SCAN', default_value='true'),
            'publish_speed': EnvironmentVariable('PUBLISH_SPEED', default_value='false'),
            'discontinuity_threshold': EnvironmentVariable('DISCONTINUITY_THRESHOLD', default_value='0.3'),
            'disparity_width_ratio_from_car_width': EnvironmentVariable('DISPARITY_WIDTH_RATIO_FROM_CAR_WIDTH', default_value='0.6'),
            'safety_angle_degrees': EnvironmentVariable('SAFETY_ANGLE_DEGREES', default_value='15.0'),
            'max_sub_window_size': EnvironmentVariable('MAX_SUB_WINDOW_SIZE', default_value='100'),
            'sub_window_step': EnvironmentVariable('SUB_WINDOW_STEP', default_value='3'),
            'best_point_conv_size': EnvironmentVariable('BEST_POINT_CONV_SIZE', default_value='5'),
            'disparity_threshold': EnvironmentVariable('DISPARITY_THRESHOLD', default_value='0.5'),
            'emergency_stop_distance': EnvironmentVariable('EMERGENCY_STOP_DISTANCE', default_value='0.15'),
            'emergency_stop_fov_ratio': EnvironmentVariable('EMERGENCY_STOP_FOV_RATIO', default_value='0.2'),
            'scan_filter_window_size' : EnvironmentVariable('SCAN_FILTER_WINDOW_SIZE', default_value='5'),
            'scan_topic' : EnvironmentVariable('SCAN_TOPIC', default_value='/scan'),
            'reverse_speed' : EnvironmentVariable('REVERSE_SPEED', default_value='0.1'),
            'reverse_time_period' : EnvironmentVariable('REVERSE_TIME_PERIOD', default_value='0.5'),
            'numer_of_laps_per_mission' : EnvironmentVariable('NUMBER_OF_LAPS_PER_MISSION', default_value='1.0'),
            'zone_entrance_time_period' : EnvironmentVariable('ZONE_ENTRANCE_TIME_PERIOD', default_value='1.0'),
            
        }.items()
    )

    # Path to the car_controller launch file
    car_controller_launch_path = os.path.join(
        get_package_share_directory('wro_firmware'),
        # 'launch',
        'car_controller.launch.py'  
    )

    # Include the car_controller launch file
    car_controller_launch_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(car_controller_launch_path)
    )

    # # Rviz2
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     output='screen',
    #     name='real_rviz2',
    #     arguments=['-d' + os.path.join(pkg_robot_system, 'rviz', 'rviz.rviz')]
    # )
    
    # # Static TF base_link -> mono_camera
    # # .15 0 .25 0 0 1.5707


    # # Static TF base_link -> lidar_link
    # # .15 0 .25 0 0 1.5707
    # lidar_x = 0.0 # 0.05
    # lidar_y = 0.0
    # lidar_z = 0.0 #0.15 
    # lidar_roll = radians(-90.0)
    # lidar_pitch = 0.0
    # lidar_yaw = radians(-90.0)
    # lidar_tf_node = Node(
    #     package='tf2_ros',
    #     name='base2lidar_tf_node',
    #     executable='static_transform_publisher',
    #     # arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), 'base_link', 'camera_link'],
    #     arguments=[str(lidar_x), str(lidar_y), str(lidar_z), str(lidar_yaw), str(lidar_pitch), str(lidar_roll), 'base_link', 'lidar_link'],
        
    # )
    cam_x = 0.0105 # 0.05
    cam_y = 0.0
    cam_z = -0.002 #0.15 
    cam_roll = radians(-90.0)
    cam_pitch = 0.0
    cam_yaw = radians(-90.0)
    cam_tf_node = Node(
        package='tf2_ros',
        name='base2depth_tf_node',
        executable='static_transform_publisher',
        # arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), 'base_link', 'camera_link'],
        arguments=[str(cam_x), str(cam_y), str(cam_z), str(cam_yaw), str(cam_pitch), str(cam_roll), 'camera_link', 'optical_camera_link'],
        
    )

    return LaunchDescription([
        robot_description,
        # rviz_node,
        cam_tf_node,
        # lidar_tf_node,
        camera_publisher,
        rplidar_ros,
        perception_launch_include,
        # ftg_launch_include,
        ftg_cpp_launch_include,
        car_controller_launch_include
    ])

