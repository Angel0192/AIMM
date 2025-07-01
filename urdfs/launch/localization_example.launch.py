from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    return LaunchDescription([

        # Kalman filter fusing IMU and GPS
        Node(
            package='robot_localization',
            executable='ekf_localization_node',
            namespace='lpv/robot_localization',
            name='ekf_localization',
            parameters=[
                {'sensor_timeout': 2.0},
                {'two_d_mode': False},
                {'map_frame': 'map'},
                {'odom_frame': 'lpv/odom'},
                {'base_link_frame': 'lpv/base_link'},
                {'world_frame': 'lpv/odom'},
                {'publish_tf': True},
                {'frequency': 60},
                {'imu0': '/lpv/sensors/imu/imu/data'},
                {'imu0_config': [False, False, False,
                                 True,  True,  True,
                                 False, False, False,
                                 True,  True,  True,
                                 True,  True,  True]},
                {'imu0_differential': False},
                {'imu0_remove_gravitational_acceleration': True},
                {'odom0': '/lpv/robot_localization/odometry/gps'},
                {'odom0_config': [True,  True,  True,
                                  False, False, False,
                                  False, False, False,
                                  False, False, False,
                                  False, False, False]},
                {'odom0_differential': False},
                {'smooth_lagged_data': True},
                {'silent_tf_failure': True}
            ],
            output='screen'
        ),

        # GPS to odometry conversion
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            namespace='lpv/robot_localization',
            name='navsat_transform_node',
            parameters=[
                {'tf_prefix': 'lpv'},
                {'frequency': 60},
                {'magnetic_declination_radians': 0},
                {'broadcast_utm_transform': True},
                {'wait_for_datum': True},
                {'use_odometry_yaw': True},
                {'datum': [-33.73, 150.67]},
                {'yaw_offset': 0},
                {'publish_filtered_gps': True}
            ],
            remappings=[
                ('/lpv/sensors/gps/gps/fix', '/lpv/robot_localization/gps/fix')
            ],
            output='screen',
            respawn=True
        )
    ])