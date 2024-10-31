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
                {'car_width': 0.2},
                {'wheel_radius': 0.0365},
                {'safety_radius': 0.04},
                {'max_speed': 1.0},
                {'min_speed': 0.2},
                {'max_range': 3.0},
                {'field_of_view': 180.0},  # Set desired FOV in degrees
                {'enable_disparity_extender': True},  # Set to False to disable
                {'enable_corner_case': False},  # Set to False to disable
            ],
            arguments=['--ros-args', '--log-level', 'info'],
        ),
    ])