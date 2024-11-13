#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wro_firmware',
            executable='car_controller',
            name='car_controller',
            output='screen',
            parameters=[
                {'serial_port': '/dev/arduino'},
                {'baud_rate': 9600}
            ]
        )
    ])
