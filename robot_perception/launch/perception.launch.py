import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from math import radians

from launch_ros.actions import Node

def generate_launch_description():

    # YOLO
    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value='/home/d2d/shared_volume/ros2_ws/src/WRO-Future-Simulation/robot_perception/yolo_models/wro.with_lines.2.v11.seg.engine',
        description='Path to YOLO model'
    )
    yolo_model = LaunchConfiguration('yolo_model')
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolov8_bringup'),
                'launch/yolov8.launch.py'
            ])
        ]),
        launch_arguments={
            'model': yolo_model,
            'threshold' : '0.7',
            'input_image_topic' : '/bgr_image',
            'device': 'cuda:0',
            'task': 'segment'
        }.items()
    )

    # RGB to BGR Converter Node
    rgb_to_bgr_node = Node(
        package='robot_perception',
        executable='rgb_to_bgr_converter',
        name='rgb_to_bgr_converter',
        remappings=[
            ('/input_image', LaunchConfiguration('input_image', default='/rgb_image')),
            ('/output_image', LaunchConfiguration('output_image', default='/bgr_image')),
        ]
    )

    # lidar image projection
    lidar2image_node = Node(
        package='robot_perception',
        executable='lidar2image_node',
        name='lidar2image_node',
    )
    return LaunchDescription([
        yolo_model_arg,
        rgb_to_bgr_node,
        yolo_launch,
        lidar2image_node,
        
    ])
