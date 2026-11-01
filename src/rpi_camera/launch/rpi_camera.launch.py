from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('source', default_value='csi',
                              description='csi (nvarguscamerasrc) o usb (webcam UVC)'),
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        DeclareLaunchArgument('capture_width', default_value='1280'),
        DeclareLaunchArgument('capture_height', default_value='720'),
        DeclareLaunchArgument('fps', default_value='15'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('upside_down', default_value='true'),
        DeclareLaunchArgument('jpeg_quality', default_value='70'),
    ]

    node = Node(
        package='rpi_camera',
        executable='rpi_camera_node',
        name='rpi_camera_node',
        output='screen',
        parameters=[{
            'source': LaunchConfiguration('source'),
            'device': LaunchConfiguration('device'),
            'capture_width': LaunchConfiguration('capture_width'),
            'capture_height': LaunchConfiguration('capture_height'),
            'fps': LaunchConfiguration('fps'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'upside_down': LaunchConfiguration('upside_down'),
            'jpeg_quality': LaunchConfiguration('jpeg_quality'),
        }],
    )
    return LaunchDescription(args + [node])
