from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    rviz_config_file = LaunchConfiguration('rviz_config')

    return LaunchDescription([

        DeclareLaunchArgument(
            'rviz_config',
            default_value=['$(find lpv_gazebo)/config/rviz_example.rviz'],
            description='Path to the RViz config file'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='lpv_visualization',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])