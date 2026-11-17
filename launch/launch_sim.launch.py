import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
import xml.etree.ElementTree as ET

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

# Vrije plek in de gang van het AWS-ziekenhuis, robot kijkt naar -y (zuid).
HOSPITAL_START_POSE = {'x': '-0.01', 'y': '12.47', 'yaw': '-1.55'}

def walking_people(context):
    """Bridge + WalkingPeopleNode voor elke actor in de wereld (geen actors: niets)."""
    world_file = os.path.join(get_package_share_directory('my_bot'), 'worlds',
                              context.launch_configurations['world'])
    names = [a.get('name') for a in ET.parse(world_file).getroot().iter('actor')]
    if not names:
        return []
    topics = [arg for n in names for arg in (
        f'/model/{n}_lichaam/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        f'/model/{n}_lichaam/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist')]
    return [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='people_bridge',
             arguments=topics, output='log'),
        Node(package='my_bot', executable='WalkingPeopleNode.py', name='walking_people',
             output='screen', parameters=[{'world_file': world_file, 'use_sim_time': True}]),
    ]


def generate_launch_description():

    # De pakketnaam is hier aangepast naar 'my_bot'
    package_name='my_bot' 

    # Inclusief de robot_state_publisher (rsp) launch file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Inclusief de Gazebo launch file (Aangepast van gazebo_ros naar ros_gz_sim)
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r ', PathJoinSubstitution([
                    get_package_share_directory('my_bot'), 'worlds', LaunchConfiguration('world')])]
                }.items(),
             )
    
    twist_mux_params = os.path.join(package_name, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')]
    )

    # De spawner node (Aangepast van spawn_entity.py naar create)
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                                   '-Y', LaunchConfiguration('yaw')],
                        output='screen')
    
    joint_state_publisher = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher',
    parameters=[{'use_sim_time': True}],
)
    
    # De Bridge configuratie
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist',
            '/laser_1/scan_filtered@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/odom@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model',
        ],
        output='screen'
    )


    # Alles lanceren
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='ziekenhuis_merged.sdf',
                              description='Wereldbestand in my_bot/worlds'),
        # Startpose per wereld; overschrijfbaar met x:= y:= yaw:=.
        *[DeclareLaunchArgument(arg, default_value=PythonExpression([
            "'", hospital, "' if '", LaunchConfiguration('world'), "'.startswith('hospital_aws') else '0.0'"]))
          for arg, hospital in HOSPITAL_START_POSE.items()],
        # Meshes van de AWS-ziekenhuiswereld (git clone naast dit pakket); de meubels komen van Gazebo Fuel.
        AppendEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH',
                               '/ros2_ws/src/aws-robomaker-hospital-world/models'),
        # Fuel-cache in de gemounte workspace, anders downloadt elke nieuwe container ~56 MB opnieuw.
        SetEnvironmentVariable('IGN_FUEL_CACHE_PATH', '/ros2_ws/fuel_cache'),
        rsp,
        gazebo,
        twist_mux,
        spawn_entity,
        bridge,
        joint_state_publisher,
        OpaqueFunction(function=walking_people),
    ])