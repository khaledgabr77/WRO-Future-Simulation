# launch/ftg_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_controllers',
            executable='ftg_node.py',
            name='ftg_node',
            output='screen',
            parameters=[
                {'wheel_base': 0.165},
                {'car_width': 0.127},
                {'wheel_radius': 0.025},
                {'safety_radius': 0.1}, # Not used
                {'gap_edge_safety_dist': 0.3},
                {'bubble_safety_radius': 0.1},
                {'speed_safety_radius': 0.15},
                {'max_speed': 0.6},
                {'min_speed': 0.2},
                {'max_range': 3.0},
                {'field_of_view': 170.0},  # Set desired FOV in degrees
                {'enable_disparity_extender': True},  # Set to False to disable
                {'enable_corner_case': False},  # Set to False to disable
                {'use_labeled_scan': False},
                {'publish_speed': False},
                {'discontinuity_threshold': 0.1},
                {'best_point_conv_size': 2},
                {'disparity_threshold': 1.0},
                {'emergency_stop_distance': 0.1},
                {'sliding_window_size' : 50}
            ],
            remappings=[
            # ('/scan', '/labeled_scan'),
            ('/scan', '/scan'),
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])
