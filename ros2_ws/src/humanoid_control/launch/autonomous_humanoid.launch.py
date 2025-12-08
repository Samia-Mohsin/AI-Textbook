from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch configuration
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Main autonomous humanoid node
    autonomous_humanoid_node = Node(
        package='humanoid_control',
        executable='autonomous_humanoid',
        name='autonomous_humanoid',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_model': 'custom_humanoid'},
            {'safety_distance': 0.8}
        ],
        remappings=[
            ('/vla/command', '/humanoid/command'),
            ('/camera/rgb/image_raw', '/camera/image_raw'),
            ('/scan', '/laser_scan'),
        ]
    )

    # Voice interface node (optional)
    voice_interface_node = Node(
        package='humanoid_control',
        executable='voice_interface',
        name='voice_interface',
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # Navigation system (assuming Nav2 is available)
    nav2_bringup_launch_dir = PathJoinSubstitution(
        [FindPackageShare('nav2_bringup'), 'launch']
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([nav2_bringup_launch_dir, 'navigation_launch.py'])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time if true'
        ),
        autonomous_humanoid_node,
        voice_interface_node,
        # navigation_launch,  # Uncomment if Nav2 is available
    ])