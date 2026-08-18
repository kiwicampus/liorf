import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    share_dir = get_package_share_directory('liorf_mapping')
    parameter_file = LaunchConfiguration('params_file')
    use_wheel_odom = LaunchConfiguration('use_wheel_odom')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'mapping.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'lio_sam_livox.yaml'),
        description='FPath to the ROS2 parameters file to use.')

    use_wheel_odom_declare = DeclareLaunchArgument(
        'use_wheel_odom',
        default_value='false',
        description='Use wheel odometry preintegration instead of IMU-only preintegration.')

    return LaunchDescription([
        params_declare,
        use_wheel_odom_declare,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            condition=UnlessCondition(use_wheel_odom),
            package='liorf_mapping',
            executable='liorf_mapping_imuPreintegration',
            name='liorf_mapping_imuPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            condition=IfCondition(use_wheel_odom),
            package='liorf_mapping',
            executable='liorf_mapping_wheelOdomPreintegration',
            name='liorf_mapping_wheelOdomPreintegration',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='liorf_mapping',
            executable='liorf_mapping_imageProjection',
            name='liorf_mapping_imageProjection',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='liorf_mapping',
            executable='liorf_mapping_mapOptmization',
            name='liorf_mapping_mapOptmization',
            parameters=[parameter_file],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            output='screen'
        )
    ])