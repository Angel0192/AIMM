from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    
    urdf_file = LaunchConfiguration('urdf')
    rviz_config_file = LaunchConfiguration('rviz_config')

    return LaunchDescription([

        # Declare arguments
        DeclareLaunchArgument(
            'urdf',
            default_value=['$(find aimm_description)/urdf/master.urdf.xacro'],
            description='Path to the URDF xacro file'
        ),

        DeclareLaunchArgument(
            'rviz_config',
            default_value=['$(find aimm_description)/rviz/rviz_aimm.rviz'],
            description='RViz config file'
        ),

        # Robot Description (xacro processing)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file])
            }]
        ),

        # Joint State Publisher (static mode since GUI=false)
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'source_list': [], 'use_gui': False}],
            output='screen'
        ),

        # RViz visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='viz',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])