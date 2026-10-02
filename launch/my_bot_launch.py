import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_bot')

    return LaunchDescription([

        Node(
            package='my_bot',
            executable='patrol_node.py',
            name='patrol_node',
            parameters=[os.path.join(pkg, 'config', 'patrol_params.yaml')],
            output='screen',
            on_exit=Shutdown(),
        ),

        Node(
            package='my_bot',
            executable='indicator_node.py',
            name='indicator_node',
            output='screen',
            on_exit=Shutdown(),
        ),

        # environment_speed_node uitgeschakeld: de 0.2 m/s-limiet bij bochten
        # liet de robot kruipen en hielp niet. DWB regelt zelf de bochtsnelheid.

        Node(
            package='my_bot',
            executable='mqtt_hmi_bridge.py',
            name='mqtt_hmi_bridge',
            output='screen',
            on_exit=Shutdown(),
            parameters=[{
                'broker_host': 'localhost',
                'broker_port': 1883,
            }],
        ),

        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='foxglove_bridge',
            output='log',
            on_exit=Shutdown(),
            parameters=[{'port': 8765}],
            ros_arguments=['--log-level', 'FATAL'],
        ),

    ])
