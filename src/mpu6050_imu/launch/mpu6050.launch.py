from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='mpu6050_imu', executable='mpu6050_node', name='mpu6050_node',
             output='screen', parameters=[{'frame_id': 'imu_link'}]),
    ])
