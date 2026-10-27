import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_description')
    motor_share = get_package_share_directory('motor_driver')

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    with_motors_arg = DeclareLaunchArgument(
        'with_motors', default_value='true',
        description='Lanza tambien motor_driver_node (odometria y motores). '
                    'Ponlo a false para ver solo el modelo.')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    motor_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(motor_share, 'launch', 'motor_driver.launch.py')),
        condition=IfCondition(LaunchConfiguration('with_motors')),
    )

    return LaunchDescription([
        with_motors_arg,
        robot_state_publisher,
        motor_driver,
    ])
