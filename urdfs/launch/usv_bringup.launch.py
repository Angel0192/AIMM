from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    autonomy_boat_dir = get_package_share_directory('autonomy_boat')

    return LaunchDescription([

        DeclareLaunchArgument('override_location', default_value='true'),
        DeclareLaunchArgument('x', default_value='158'),
        DeclareLaunchArgument('y', default_value='108'),
        DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('world_name', default_value='RAITE'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('world_name2', default_value=os.path.join(autonomy_boat_dir, 'World', 'E.world')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(autonomy_boat_dir, 'launch', LaunchConfiguration('world_name').perform({}) + '.launch.py')
            ),
            launch_arguments={
                'urdf': os.path.join(autonomy_boat_dir, 'config', 'aimm.urdf'),
                'gui': LaunchConfiguration('gui'),
                'verbose': LaunchConfiguration('verbose'),
            }.items()
        ),

        Node(
            package='topic_tools',
            executable='relay',
            name='odom_filtered_relay',
            arguments=['robot_localization/odometry/filtered', 'odom'],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'publish_frequency': 60}]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'publish_frequency': 60}]
        ),

    ])