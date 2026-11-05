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
    camera_share = get_package_share_directory('rpi_camera')
    sllidar_share = get_package_share_directory('sllidar_ros2')

    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.urdf')
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    with_motors_arg = DeclareLaunchArgument(
        'with_motors', default_value='true',
        description='Lanza tambien motor_driver_node (odometria y motores). '
                    'Ponlo a false para ver solo el modelo.')

    with_camera_arg = DeclareLaunchArgument(
        'with_camera', default_value='true',
        description='Lanza tambien rpi_camera_node (imagen en /camera/image_raw).')

    with_lidar_arg = DeclareLaunchArgument(
        'with_lidar', default_value='true',
        description='Lanza tambien el RPLIDAR C1 (/scan, frame laser).')

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

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(camera_share, 'launch', 'rpi_camera.launch.py')),
        condition=IfCondition(LaunchConfiguration('with_camera')),
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_share, 'launch', 'sllidar_c1_launch.py')),
        launch_arguments={
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102N_USB_to_UART_Bridge_Controller_b21cfaaeae31f111a5e4945f30d20014-if00-port0',
            'frame_id': 'laser',
        }.items(),
        condition=IfCondition(LaunchConfiguration('with_lidar')),
    )

    return LaunchDescription([
        with_motors_arg,
        with_camera_arg,
        with_lidar_arg,
        robot_state_publisher,
        motor_driver,
        camera,
        lidar,
    ])
