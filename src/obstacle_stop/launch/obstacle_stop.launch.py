from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument('stop_distance', default_value='0.40',
                              description='Distancia (m) a la que se bloquea el avance'),
        DeclareLaunchArgument('slow_distance', default_value='0.70',
                              description='Distancia (m) a partir de la que se reduce la velocidad'),
    ]

    # Cámara: rejilla 3x3 de distancias (no se puede lanzar a la vez que oak_camera_node)
    spatial_grid = Node(
        package='oak_camera_driver',
        executable='oak_spatial_grid_node',
        name='oak_spatial_grid_node',
        output='screen',
    )

    obstacle_stop = Node(
        package='obstacle_stop',
        executable='obstacle_stop_node',
        name='obstacle_stop_node',
        output='screen',
        parameters=[{
            'stop_distance': LaunchConfiguration('stop_distance'),
            'slow_distance': LaunchConfiguration('slow_distance'),
        }],
    )

    return LaunchDescription(args + [spatial_grid, obstacle_stop])
