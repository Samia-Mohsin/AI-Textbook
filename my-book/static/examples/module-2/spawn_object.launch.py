from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Spawn an object in Gazebo
    spawn_object = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'my_object',
            '-x', '1.0',
            '-y', '2.0',
            '-z', '0.5',
            '-database', 'coke_can'  # Use a model from Gazebo's database
        ],
        output='screen'
    )

    return LaunchDescription([
        spawn_object,
    ])