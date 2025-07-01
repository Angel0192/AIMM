from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    autonomy_boat_dir = get_package_share_directory('autonomy_boat')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('headless', default_value='true'),
        DeclareLaunchArgument('world_name', default_value=os.path.join(autonomy_boat_dir, 'World', 'E.world')),
        DeclareLaunchArgument('front_laser', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(autonomy_boat_dir, 'launch', 'usv_bringup.launch.py')
            ),
            launch_arguments={'gui': 'false'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(autonomy_boat_dir, 'launch', 'include', 'pointcloud2laserscan.launch.py')
            )
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_boat',
            arguments=['-d', os.path.join(autonomy_boat_dir, 'rviz', 'jackal_test.rviz')],
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz_boat2',
            arguments=['-d', os.path.join(autonomy_boat_dir, 'rviz', 'jackal_test.rviz')],
            output='screen'
        )
    ])