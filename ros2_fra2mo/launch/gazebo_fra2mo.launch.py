import os
import yaml

from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
    ExecuteProcess,
    TimerAction
)

from launch.event_handlers import OnProcessExit, OnProcessStart

def generate_launch_description():

    # Percorsi ai file
    pkg_fra2mo = get_package_share_directory('ros2_fra2mo')
    pkg_iiwa = get_package_share_directory('iiwa_description')
    xacro_fra2mo = os.path.join(pkg_fra2mo, "urdf", "fra2mo.urdf.xacro")
    xacro_iiwa = os.path.join(pkg_iiwa, "urdf", "iiwa.urdf.xacro")

    models_path = os.path.join(pkg_fra2mo, 'models')
    world_file = os.path.join(pkg_fra2mo, "worlds", "personal_project_world.sdf")

        # --- Funzione per estrarre le posizioni iniziali ---
    def load_yaml_initial_pos(file_name, prefix):
        path = os.path.join(pkg_iiwa, 'config', file_name)
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        pos_dict = data['initial_positions']
        names = [f"{prefix}{k}" for k in pos_dict.keys()]
        values = [str(v) for v in pos_dict.values()]
        return names, values

    iiwa1_names, iiwa1_values = load_yaml_initial_pos(
        'initial_positions.yaml', 'iiwa_'
    )

    # Nodo robot_state_publisher
    rsp_fra2mo = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{"robot_description": ParameterValue(Command(['xacro ', xacro_fra2mo]), value_type=str)},
                    {"use_sim_time": True}
            ]
    )

    rsp_iiwa = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace = 'iiwa',
        parameters=[{"robot_description": ParameterValue(Command(['xacro ', xacro_iiwa]), value_type=str)},
                    {"use_sim_time": True}
            ]
    )

    
    # Gazebo simulation launch description
    gazebo_ignition = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                    'launch',
                                    'gz_sim.launch.py'])]),
            launch_arguments={'gz_args': [world_file, ' -r']}.items())

    position_fra2mo = [-7.9, 3.10, 0.1]

    # Define a Node to spawn the robot in the Gazebo simulation
    spawn_fra2mo = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'fra2mo',
                    "-x", str(position_fra2mo[0]),
                    "-y", str(position_fra2mo[1]),
                    "-z", str(position_fra2mo[2]),]
    )

    position_iiwa = [8.40, -4, 0.1]

    spawn_iiwa = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'iiwa/robot_description',
                   '-name', 'iiwa',
                    "-x", str(position_iiwa[0]),
                    "-y", str(position_iiwa[1]),
                    "-z", str(position_iiwa[2]),
                    '-joint-names'] + iiwa1_names +
                   ['-joint-positions'] + iiwa1_values,
        output='screen'
    )

    jsb_iiwa = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster',
                   '-c', '/iiwa/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen'
    )

    arm_iiwa = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['iiwa_arm_controller',
                   '-c', '/iiwa/controller_manager',
                   '--controller-manager-timeout', '60'],
        output='screen'
    )


    load_iiwa_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_iiwa,
            on_exit=[jsb_iiwa, arm_iiwa],
        )
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/fra2mo/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry',
                   '/model/fra2mo/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
                   '/lidar@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                   '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                   '/iiwa_camera@sensor_msgs/msg/Image[gz.msgs.Image',
                   'camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                   '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/tf_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                   '/iiwa/gripper/attach_package@std_msgs/msg/Empty@ignition.msgs.Empty',
                   '/iiwa/gripper/detach_package@std_msgs/msg/Empty@ignition.msgs.Empty',
                   '/iiwa/gripper/state_package@std_msgs/msg/Bool@ignition.msgs.Boolean',
                   '/iiwa/gripper/attach_package2@std_msgs/msg/Empty@ignition.msgs.Empty',
                   '/iiwa/gripper/detach_package2@std_msgs/msg/Empty@ignition.msgs.Empty',
                   '/iiwa/gripper/state_package2@std_msgs/msg/Bool@ignition.msgs.Boolean'],
                   #'/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'], 
        output='screen'
    )

    detach_package1 = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once',
             '/iiwa/gripper/detach_package', 'std_msgs/msg/Empty', '{}'],
        output='screen'
    )

    detach_handler = RegisterEventHandler(
        OnProcessStart(
            target_action = bridge,
            on_start = [
                TimerAction(period=5.0, actions=[detach_package1])
            ]
        )
    )   

    iiwa_handler = RegisterEventHandler(
        OnProcessStart(
            target_action = bridge,
            on_start = [
                TimerAction(period=1.0,
                             actions=[rsp_iiwa, spawn_iiwa, load_iiwa_controllers])
            ]
        )
    )

    odom_tf = Node(
        package='ros2_fra2mo',
        executable='dynamic_tf_publisher',
        name='odom_tf',
        parameters=[{"use_sim_time": True}]
    )

    ign_clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
        ],
        output="screen",
        namespace="fra2mo"
    )
 
    nodes_to_start = [gazebo_ignition, rsp_fra2mo, iiwa_handler, bridge, spawn_fra2mo, detach_handler, 
                      odom_tf, ign_clock_bridge]

    return LaunchDescription([SetEnvironmentVariable(name="GZ_SIM_RESOURCE_PATH", value = os.path.join(pkg_fra2mo, 'models') + ':' + pkg_iiwa + ':' + os.environ.get('GZ_SIM_RESOURCE_PATH', ''))] + nodes_to_start)