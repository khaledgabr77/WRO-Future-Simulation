# launch/ftg_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controllers',
            executable='ftg_node',
            name='ftg_node',
            output='screen',
            parameters=[
                {'wheel_base': 0.165},
                {'car_width': 0.127},
                {'wheel_radius': 0.025},
                {'safety_radius': 0.12},
                {'bubble_safety_radius': 0.0335}, # 0.0635 # 0.08 # 0.0335
                {'speed_safety_radius': 0.10},
                {'max_speed': 0.3},
                {'min_speed': 0.15},
                {'max_range': 3.0},
                {'field_of_view': 170.0},  # Set desired FOV in degrees
                {'enable_disparity_extender': True},  # Set to False to disable
                {'enable_corner_case': False},  # Set to False to disable
                {'use_labeled_scan': True},
                {'publish_speed': True},
                {'discontinuity_threshold': 0.3},
                {'disparity_width_ratio_from_car_wdith': 0.6},
                {'safety_angle_degrees': 15.0}, # Not used
                {'max_sub_window_size': 100}, # Not used
                {'sub_window_step': 3}, # Not used
                {'best_point_conv_size': 5},
                {'disparity_threshold': 0.5},
                {'emergency_stop_distance': 0.15},
                {'emergency_stop_fov_ratio': 0.2}
            ],
            remappings=[
            ('/scan', '/labeled_scan'),
            # ('/scan', '/scan'),
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])