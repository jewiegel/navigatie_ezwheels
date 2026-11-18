import importlib.util
import os

import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, SetEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

# Vrije plek in de gang van het AWS-ziekenhuis, robot kijkt naar -y (zuid).
HOSPITAL_START_POSE = {'x': '-0.01', 'y': '12.47', 'yaw': '-1.55'}

def walking_people(context):
    """people:=true: paspoppen uit people_file spawnen, met bridge en WalkingPeopleNode."""
    if context.launch_configurations['people'].lower() != 'true':
        return []
    # Modellen en hun startplek komen uit het node zelf, zodat ze op één plek gedefinieerd zijn.
    spec = importlib.util.spec_from_file_location('walking_people', os.path.join(
        get_package_prefix('my_bot'), 'lib', 'my_bot', 'WalkingPeopleNode.py'))
    walking = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(walking)
    people_file = context.launch_configurations['people_file']
    with open(people_file) as f:
        models = walking.models(yaml.safe_load(f))
    share = get_package_share_directory('my_bot')
    spawn = [Node(package='ros_gz_sim', executable='create', output='log',
                  arguments=['-name', name, '-x', str(x), '-y', str(y),
                             '-file', os.path.join(share, 'models', model_dir, 'model.sdf')])
             for name, (model_dir, x, y) in models.items()]
    topics = [arg for n in models for arg in (
        f'/model/{n}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
        f'/model/{n}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist')]
    return spawn + [
        Node(package='ros_gz_bridge', executable='parameter_bridge', name='people_bridge',
             arguments=topics, output='log'),
        Node(package='my_bot', executable='WalkingPeopleNode.py', name='walking_people',
             output='screen', parameters=[{'people_file': people_file, 'use_sim_time': True}]),
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
        DeclareLaunchArgument('people', default_value='false',
                              description='Lopende paspoppen uit people_file (voor beendetectie)'),
        DeclareLaunchArgument('people_file', default_value=os.path.join(
            get_package_share_directory(package_name), 'config', 'people.yaml')),
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