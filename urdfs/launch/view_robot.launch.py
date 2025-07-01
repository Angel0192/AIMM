from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    autonomy_boat_share = FindPackageShare('autonomy_boat').find('autonomy_boat')

    urdf_file = PathJoinSubstitution([autonomy_boat_share, 'config', 'aimm.urdf'])
    rviz_config_file = PathJoinSubstitution([autonomy_boat_share, 'rviz', 'aimm.rviz'])

    return LaunchDescription([
        
        # Load URDF as robot_description parameter
        ExecuteProcess(
            cmd=[
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                urdf_file
            ],
            output='screen',
            shell=True
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': Command([
                    PathJoinSubstitution([FindExecutable(name='xacro')]),
                    ' ',
                    urdf_file
                ])
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz',
            arguments=['-d', rviz_config_file]
        )
    ])