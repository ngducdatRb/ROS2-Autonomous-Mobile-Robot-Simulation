import os
import xacro

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ──────────────────────────────────────────────────────────────────────────────────
    # PACKAGE PATHS
    # ──────────────────────────────────────────────────────────────────────────────────
    pkg_desc       = get_package_share_directory('robot_description')
    pkg_gazebo     = get_package_share_directory('robot_gazebo')
    pkg_bringup    = get_package_share_directory('robot_bringup')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # ──────────────────────────────────────────────────────────────────────────────────
    # FILE PATHS
    # ──────────────────────────────────────────────────────────────────────────────────
    file_xacro      = os.path.join(pkg_desc, 'urdf', 'robot.xacro')
    file_cfg_world  = os.path.join(pkg_gazebo, 'worlds', 'demo.sdf')
    file_cfg_rviz   = os.path.join(pkg_bringup, 'config', 'robot.rviz')
    file_cfg_bridge = os.path.join(pkg_bringup, 'config', 'robot_bridge.yaml')
    file_ekf_config = os.path.join(pkg_bringup, 'config', 'robot_localization.yaml')
    file_gz_sim     = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

    # ──────────────────────────────────────────────────────────────────────────────────
    # CONVERT XACRO TO URDF
    # ──────────────────────────────────────────────────────────────────────────────────
    doc = xacro.process_file(file_xacro)
    file_urdf = doc.toprettyxml(indent=' ')

    # ──────────────────────────────────────────────────────────────────────────────────
    # ROBOT STATE PUBLISHER
    # ──────────────────────────────────────────────────────────────────────────────────
    rsp = Node(
        package    ='robot_state_publisher',
        executable ='robot_state_publisher',
        output     ='screen',
        parameters =[
            {'use_sim_time': True},
            {'robot_description': file_urdf},
        ]
    )

    # ──────────────────────────────────────────────────────────────────────────────────
    # GAZEBO IGNITION
    # ──────────────────────────────────────────────────────────────────────────────────
    ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(file_gz_sim),
        launch_arguments={'gz_args': f'-r -s {file_cfg_world}'}.items()
    )

    # ──────────────────────────────────────────────────────────────────────────────────
    # RVIZ
    # ──────────────────────────────────────────────────────────────────────────────────
    rviz = Node(
        package     ='rviz2',
        executable  ='rviz2',
        output      ='screen',
        arguments   =['-d', file_cfg_rviz]

    )

    # ──────────────────────────────────────────────────────────────────────────────────
    # SPAWN ROBOT
    # ──────────────────────────────────────────────────────────────────────────────────
    robot = Node(
        package     ='ros_gz_sim',
        executable  ='create',
        output      ='screen',
        arguments   =[
            '-name', 'robot',
            '-topic', 'robot_description',
            '-x', '0', '-y', '0', '-z', '0.01'
        ]
    )

    # ──────────────────────────────────────────────────────────────────────────────────
    # ROS GZ BRIDGE
    # ──────────────────────────────────────────────────────────────────────────────────
    bridge = Node(
        package     ='ros_gz_bridge',
        executable  ='parameter_bridge',
        output      ='screen',
        parameters  =[{
            'config_file': file_cfg_bridge,
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
        }]
    )

    # ──────────────────────────────────────────────────────────────────────────────────
    # ROBOT LOCALIZATION
    # ──────────────────────────────────────────────────────────────────────────────────
    ekf = Node(
        package     ='robot_localization',
        executable  ='ekf_node',
        name        ='ekf_filter_node',
        output      ='screen',
        parameters  = [file_ekf_config],
    )


    # ──────────────────────────────────────────────────────────────────────────────────
    # LAUNCH
    # ──────────────────────────────────────────────────────────────────────────────────
    return LaunchDescription([
        ekf,
        ign,
        bridge,
        TimerAction(period=5.0, actions=[ 
            rsp,
            robot,
            rviz,
        ])
    ])
