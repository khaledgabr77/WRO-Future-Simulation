# launch/ftg_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare each parameter as a launch argument
    wheel_base_arg = DeclareLaunchArgument('wheel_base', default_value='0.165', description='wheel_base in meters')
    car_width_arg = DeclareLaunchArgument('car_width', default_value='0.127', description='car_width in meters')
    wheel_radius_arg = DeclareLaunchArgument('wheel_radius', default_value='0.025', description='wheel_radius in meters')
    safety_radius_arg = DeclareLaunchArgument('safety_radius', default_value='0.12', description='safety_radius in meters')
    bubble_safety_radius_arg = DeclareLaunchArgument('bubble_safety_radius', default_value='0.0335', description='bubble_safety_radius in meters')
    speed_safety_radius_arg = DeclareLaunchArgument('speed_safety_radius', default_value='0.10', description='speed_safety_radius in meters')
    max_speed_arg = DeclareLaunchArgument('max_speed', default_value='0.3', description='max_speed in m/s')
    min_speed_arg = DeclareLaunchArgument('min_speed', default_value='0.15', description='min_speed in m/s')
    max_range_arg = DeclareLaunchArgument('max_range', default_value='3.0', description='max_range in meters')
    field_of_view_arg = DeclareLaunchArgument('field_of_view', default_value='170.0', description='Field of view in degrees')
    enable_disparity_extender_arg = DeclareLaunchArgument('enable_disparity_extender', default_value='true', description='Enable disparity extender')
    enable_corner_case_arg = DeclareLaunchArgument('enable_corner_case', default_value='false', description='Enable corner case handling')
    use_labeled_scan_arg = DeclareLaunchArgument('use_labeled_scan', default_value='false', description='Use labeled scan')
    publish_speed_arg = DeclareLaunchArgument('publish_speed', default_value='true', description='Publish speed')
    discontinuity_threshold_arg = DeclareLaunchArgument('discontinuity_threshold', default_value='0.3', description='Discontinuity threshold')
    disparity_width_ratio_from_car_width_arg = DeclareLaunchArgument('disparity_width_ratio_from_car_width', default_value='0.6', description='Disparity width ratio from car width')
    safety_angle_degrees_arg = DeclareLaunchArgument('safety_angle_degrees', default_value='15.0', description='Safety angle in degrees')
    max_sub_window_size_arg = DeclareLaunchArgument('max_sub_window_size', default_value='100', description='Maximum sub-window size')
    sub_window_step_arg = DeclareLaunchArgument('sub_window_step', default_value='3', description='Sub-window step')
    best_point_conv_size_arg = DeclareLaunchArgument('best_point_conv_size', default_value='5', description='Best point convolution size')
    disparity_threshold_arg = DeclareLaunchArgument('disparity_threshold', default_value='0.5', description='Disparity threshold')
    emergency_stop_distance_arg = DeclareLaunchArgument('emergency_stop_distance', default_value='0.15', description='Emergency stop distance')
    emergency_stop_fov_ratio_arg = DeclareLaunchArgument('emergency_stop_fov_ratio', default_value='0.2', description='Emergency stop FOV ratio')
    scan_filter_window_size_arg = DeclareLaunchArgument('scan_filter_window_size', default_value='5', description='Scan filter window size')
    scan_topic_arg = DeclareLaunchArgument('scan_topic', default_value='/scan', description='Scan topic name')
    

    # Fetch the configurations for each parameter
    wheel_base = LaunchConfiguration('wheel_base')
    car_width = LaunchConfiguration('car_width')
    wheel_radius = LaunchConfiguration('wheel_radius')
    safety_radius = LaunchConfiguration('safety_radius')
    bubble_safety_radius = LaunchConfiguration('bubble_safety_radius')
    speed_safety_radius = LaunchConfiguration('speed_safety_radius')
    max_speed = LaunchConfiguration('max_speed')
    min_speed = LaunchConfiguration('min_speed')
    max_range = LaunchConfiguration('max_range')
    field_of_view = LaunchConfiguration('field_of_view')
    enable_disparity_extender = LaunchConfiguration('enable_disparity_extender')
    enable_corner_case = LaunchConfiguration('enable_corner_case')
    use_labeled_scan = LaunchConfiguration('use_labeled_scan')
    publish_speed = LaunchConfiguration('publish_speed')
    discontinuity_threshold = LaunchConfiguration('discontinuity_threshold')
    disparity_width_ratio_from_car_width = LaunchConfiguration('disparity_width_ratio_from_car_width')
    safety_angle_degrees = LaunchConfiguration('safety_angle_degrees')
    max_sub_window_size = LaunchConfiguration('max_sub_window_size')
    sub_window_step = LaunchConfiguration('sub_window_step')
    best_point_conv_size = LaunchConfiguration('best_point_conv_size')
    disparity_threshold = LaunchConfiguration('disparity_threshold')
    emergency_stop_distance = LaunchConfiguration('emergency_stop_distance')
    emergency_stop_fov_ratio = LaunchConfiguration('emergency_stop_fov_ratio')
    scan_filter_window_size = LaunchConfiguration('scan_filter_window_size')
    scan_topic = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        # Add all DeclareLaunchArgument instances
        wheel_base_arg, car_width_arg, wheel_radius_arg, safety_radius_arg, bubble_safety_radius_arg,
        speed_safety_radius_arg, max_speed_arg, min_speed_arg, max_range_arg, field_of_view_arg,
        enable_disparity_extender_arg, enable_corner_case_arg, use_labeled_scan_arg, publish_speed_arg,
        discontinuity_threshold_arg, disparity_width_ratio_from_car_width_arg, safety_angle_degrees_arg,
        max_sub_window_size_arg, sub_window_step_arg, best_point_conv_size_arg, disparity_threshold_arg,
        emergency_stop_distance_arg, emergency_stop_fov_ratio_arg, scan_filter_window_size_arg, scan_topic_arg,

        # Launch the node with parameters
        Node(
            package='robot_controllers',
            executable='ftg_node',
            name='ftg_node',
            output='screen',
            parameters=[
                {'wheel_base': wheel_base},
                {'car_width': car_width},
                {'wheel_radius': wheel_radius},
                {'safety_radius': safety_radius},
                {'bubble_safety_radius': bubble_safety_radius},
                {'speed_safety_radius': speed_safety_radius},
                {'max_speed': max_speed},
                {'min_speed': min_speed},
                {'max_range': max_range},
                {'field_of_view': field_of_view},
                {'enable_disparity_extender': enable_disparity_extender},
                {'enable_corner_case': enable_corner_case},
                {'use_labeled_scan': use_labeled_scan},
                {'publish_speed': publish_speed},
                {'discontinuity_threshold': discontinuity_threshold},
                {'disparity_width_ratio_from_car_width': disparity_width_ratio_from_car_width},
                {'safety_angle_degrees': safety_angle_degrees},
                {'max_sub_window_size': max_sub_window_size},
                {'sub_window_step': sub_window_step},
                {'best_point_conv_size': best_point_conv_size},
                {'disparity_threshold': disparity_threshold},
                {'emergency_stop_distance': emergency_stop_distance},
                {'emergency_stop_fov_ratio': emergency_stop_fov_ratio},
                {'scan_filter_window_size': scan_filter_window_size}
            ],
            remappings=[
               ('/scan', scan_topic),
                # ('/scan', '/scan'),
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
