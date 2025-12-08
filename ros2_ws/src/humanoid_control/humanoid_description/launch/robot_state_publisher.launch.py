import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Get package share directory
    pkg_share = get_package_share_directory('humanoid_description')
    
    # Path to URDF file
    urdf_path = os.path.join(pkg_share, 'urdf', 'humanoid.urdf.xacro')

    # Robot state publisher node
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(urdf_path).read()
        }]
    )

    # Joint state publisher node (for visualization purposes)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['joint_states']
        }]
    )

    # RViz2 node
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=lambda context: LaunchConfiguration('rviz').perform(context) == 'true'
    )

    # Declare the launch argument for RViz
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Open RViz2 if true'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        rviz_arg,
        robot_state_publisher,
        joint_state_publisher,
        rviz_node
    ])