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
                {'wheel_base': 0.2255},
                {'car_width': 0.15},
                {'wheel_radius': 0.0365},
                {'safety_radius': 0.03},
                {'max_speed': 0.6},
                {'min_speed': 0.2},
                {'max_range': 3.0},
                {'field_of_view': 180.0},  # Set desired FOV in degrees
                {'enable_disparity_extender': True},  # Set to False to disable
                {'enable_corner_case': False},  # Set to False to disable
                {'use_labeled_scan': True},
                {'publish_speed': True},
                {'discontinuity_threshold': 0.5},
                {'safety_angle_degrees': 15.0},
                {'max_sub_window_size': 20},
                 {'sub_window_step': 1}
            ],
            remappings=[
            ('/scan', '/labeled_scan'),
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
