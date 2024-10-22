from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments for frame IDs
        DeclareLaunchArgument('frame_id', default_value='odom', description='Frame ID of the parent frame'),
        DeclareLaunchArgument('child_frame_id', default_value='base_link', description='Frame ID of the child frame'),

        # Node to publish the transform
        Node(
            package='mrbuggy3_gz_bringup',
            executable='odom_to_tf_publisher.py',
            name='odom_to_tf_publisher',
            output='screen',
            parameters=[
                {'frame_id': LaunchConfiguration('frame_id')},
                {'child_frame_id': LaunchConfiguration('child_frame_id')},
                {'use_sim_time': True}
            ]
        )
    ])
