from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            'wheel_separation', default_value='0.125',
            description='Distancia entre ruedas en metros'),
        DeclareLaunchArgument(
            'max_linear', default_value='0.5',
            description='Velocidad lineal máxima en m/s (a throttle=1.0)'),
        DeclareLaunchArgument(
            'min_throttle', default_value='0.35',
            description='Throttle mínimo para vencer el deadband de los motores'),
        DeclareLaunchArgument(
            'odom_rate_hz', default_value='20.0',
            description='Frecuencia de publicación de odometría en Hz'),
        DeclareLaunchArgument(
            'left_gain', default_value='1.0',
            description='Ganancia de la rueda izquierda (bajar si se desvía a la derecha)'),
        DeclareLaunchArgument(
            'right_gain', default_value='1.0',
            description='Ganancia de la rueda derecha (bajar si se desvía a la izquierda)'),
    ]

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
            'left_gain': LaunchConfiguration('left_gain'),
            'right_gain': LaunchConfiguration('right_gain'),
        }]
    )

    return LaunchDescription(args + [motor_driver_node])
