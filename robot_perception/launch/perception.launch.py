import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

def generate_launch_description():

    # robot_name='mrbuggy3'

    # YOLO
    yolo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('yolov8_bringup'),
                'launch/yolov11.launch.py'
            ])
        ]),
        launch_arguments={
            'model': '/home/user/shared_volume/ros2_ws/src/WRO-Future-Simulation/robot_perception/yolo_models/wro.sim.v11n.seg.pt',
            'threshold' : '0.5',
            'input_image_topic' : '/rgb_image',
            'device': 'cuda:0',
            # 'image_reliability': '1'
        }.items()
    )


    return LaunchDescription([
        yolo_launch
    ])
