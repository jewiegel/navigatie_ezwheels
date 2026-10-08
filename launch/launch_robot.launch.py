#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_bot')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value='/maps/ziekenhuisV1.yaml',
        description='Volledig pad naar de kaart (.yaml)',
    )
    map_file = LaunchConfiguration('map')

    # ── Robot State Publisher ────────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={'use_sim_time': 'false'}.items(),
    )

    # ── LIDAR: IDEC SE2L via URG/SCIP protocol (Ethernet) ───────────────────
    # Pas het IP-adres aan naar het geconfigureerde adres van de SE2L scanner.
    lidar = Node(
        package='urg_node2',
        executable='urg_node2_node',
        name='lidar',
        parameters=[{
            'ip_address': '192.168.0.10',
            'ip_port': 10940,
            'frame_id': 'laser_frame',
        }],
        output='screen',
    )

    # ── Twist Mux ────────────────────────────────────────────────────────────
    twist_mux_params = os.path.join(pkg, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': False}],
        remappings=[('/cmd_vel_out', '/cmd_vel')],
        output='screen',
    )

    # ── Nav2 Localization (AMCL) ─────────────────────────────────────────────
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': 'false',
            'params_file': os.path.join(pkg, 'config', 'nav2_params_robot.yaml'),
        }.items(),
    )

    # ── Nav2 Navigation ──────────────────────────────────────────────────────
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': os.path.join(pkg, 'config', 'nav2_params_robot.yaml'),
        }.items(),
    )

    # ── Patrol node ──────────────────────────────────────────────────────────
    patrol_node = Node(
        package='my_bot',
        executable='patrol_node.py',
        name='patrol_node',
        parameters=[os.path.join(pkg, 'config', 'patrol_params.yaml')],
        output='screen',
    )

    # ── Indicator node ───────────────────────────────────────────────────────
    indicator_node = Node(
        package='my_bot',
        executable='indicator_node.py',
        name='indicator_node',
        output='screen',
    )

    # ── Environment speed node ───────────────────────────────────────────────
    environment_speed_node = Node(
        package='my_bot',
        executable='environment_speed_node.py',
        name='environment_speed_node',
        output='screen',
    )

    # ── MQTT HMI bridge (FT2J → /buzzer + /indicators) ──────────────────────
    mqtt_bridge = Node(
        package='my_bot',
        executable='mqtt_hmi_bridge.py',
        name='mqtt_hmi_bridge',
        parameters=[{
            'broker_host': '192.168.0.100',
            'broker_port': 1883,
        }],
        output='screen',
    )

    return LaunchDescription([
        map_arg,
        rsp,
        lidar,
        twist_mux,
        localization,
        navigation,
        patrol_node,
        indicator_node,
        environment_speed_node,
        mqtt_bridge,
    ])
