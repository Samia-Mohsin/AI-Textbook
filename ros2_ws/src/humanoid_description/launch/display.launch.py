from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': [
                PathJoinSubstitution([
                    FindPackageShare('humanoid_description'),
                    'urdf',
                    'humanoid.urdf.xacro'
                ])
            ]
        }]
    )

    # RViz node
    rviz = Node(
        condition=use_rviz,
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('humanoid_description'),
                'rviz',
                'model.rviz'
            ])
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Whether to launch RViz'
        ),
        robot_state_publisher,
        # rviz  # Commented out since rviz config doesn't exist yet
    ])