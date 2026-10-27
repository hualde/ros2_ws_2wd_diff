from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    wheel_separation_arg = DeclareLaunchArgument(
        'wheel_separation', default_value='0.125',
        description='Distancia entre ruedas en metros')

    max_linear_arg = DeclareLaunchArgument(
        'max_linear', default_value='0.5',
        description='Velocidad lineal máxima en m/s (a throttle=1.0)')

    min_throttle_arg = DeclareLaunchArgument(
        'min_throttle', default_value='0.35',
        description='Throttle mínimo para vencer el deadband de los motores')

    odom_rate_arg = DeclareLaunchArgument(
        'odom_rate_hz', default_value='20.0',
        description='Frecuencia de publicación de odometría en Hz')

    motor_driver_node = Node(
        package='motor_driver',
        executable='motor_driver_node',
        name='motor_driver_node',
        output='screen',
        parameters=[{
            'wheel_separation': LaunchConfiguration('wheel_separation'),
            'max_linear': LaunchConfiguration('max_linear'),
            'min_throttle': LaunchConfiguration('min_throttle'),
            'odom_rate_hz': LaunchConfiguration('odom_rate_hz'),
        }]
    )

    return LaunchDescription([
        wheel_separation_arg,
        max_linear_arg,
        min_throttle_arg,
        odom_rate_arg,
        motor_driver_node,
    ])