import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_bot')
    slam = LaunchConfiguration('slam')
    sim_time = {'use_sim_time': 'true'}

    def include(name, args=None, **kwargs):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', name)),
            launch_arguments={**sim_time, **(args or {})}.items(),
            **kwargs,
        )

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='/ros2_ws/maps/ziekenhuis.yaml',
                              description='Kaart voor AMCL (genegeerd bij slam:=true)'),
        DeclareLaunchArgument('slam', default_value='false',
                              description='true = SLAM i.p.v. AMCL op een opgeslagen kaart'),

        include('launch_sim.launch.py'),

        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(pkg, 'config', 'setup.rviz')],
             parameters=[{'use_sim_time': True}], output='log'),

        # Wachten tot Gazebo /clock, /laser_1/scan_filtered en /tf publiceert, anders TF-fouten bij opstarten.
        TimerAction(period=8.0, actions=[
            include('localization_launch.py', {'map': LaunchConfiguration('map')},
                    condition=UnlessCondition(slam)),
            include('online_async_launch.py', condition=IfCondition(slam)),
            include('navigation_launch.py'),
        ]),
    ])
