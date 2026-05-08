import os
import xacro

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory


# ****************************************************************************
# PACKAGE PATHS
# ****************************************************************************
pkg_gazebo      = get_package_share_directory('robot_gazebo')
pkg_bringup     = get_package_share_directory('robot_bringup')
pkg_description = get_package_share_directory('robot_description')
pkg_ros_gz_sim  = get_package_share_directory('ros_gz_sim')

# ****************************************************************************
# FILE PATHS
# ****************************************************************************
file_xacro      = os.path.join(pkg_description, 'urdf', 'robot.xacro')
file_cfg_rviz   = os.path.join(pkg_bringup, 'config', 'robot.rviz')
file_cfg_world  = os.path.join(pkg_gazebo, 'worlds', 'world_slam.sdf')
file_gz_sim     = os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')

# ****************************************************************************
# CFG
# ****************************************************************************
ign_world = 'demo'

ROBOTS = [
    {'ns': 'r1', 'x': '0.0', 'y': '0.0', 'z': '0.01', 'Y': '0.0'},
    {'ns': 'r2', 'x': '2.0', 'y': '0.0', 'z': '0.01', 'Y': '0.7'},
]


# ****************************************************************************
# HELPER FUNCTION
# ****************************************************************************
def bridge(topic: str, ros_type: str, ign_type: str, direction: str) -> str:
    return f'{topic}@{ros_type}{direction}{ign_type}'

def make_robot_nodes(ns: str, x: str, y: str, z: str, Y: str) -> list:
    # ─────────────────────────────────────────────────────────────────────────
    # Topic Name
    # ─────────────────────────────────────────────────────────────────────────
    TOPIC_IMU     = f'/{ns}/imu'
    TOPIC_LIDAR   = f'/{ns}/lidar'
    TOPIC_CMD_VEL = f'/model/{ns}/cmd_vel'
    TOPIC_JOINTS  = f'/world/{ign_world}/model/{ns}/joint_state'
    TOPIC_ODOM_PUBLISHER  = f'/{ns}/odom/publisher'
    TOPIC_ODOM_DIFF_DRIVE = f'/{ns}/odom/diff_drive'
    TOPIC_TF_DIFF_DRIVE     = f'/{ns}/tf/diff_drive'
    TOPIC_TF_ODOM_PUBLISHER = f'/{ns}/tf/odom_publisher'
    # TOPIC_IMAGE_COLOR   = f'/world/{ign_world}/model/{ns}/link/{ns}_base_footprint/sensor/rgbd_camera/image'
    # TOPIC_IMAGE_DEPTH   = f'/world/{ign_world}/model/{ns}/link/{ns}_base_footprint/sensor/rgbd_camera/depth_image'
    # TOPIC_CAMERA_INFO   = f'/world/{ign_world}/model/{ns}/link/{ns}_base_footprint/sensor/rgbd_camera/camera_info'
    # TOPIC_CAMERA_POINTS = f'/world/{ign_world}/model/{ns}/link/{ns}_base_footprint/sensor/rgbd_camera/camera_info' 

    # ─────────────────────────────────────────────────────────────────────────
    # Convert Xacro → Urdf
    # ─────────────────────────────────────────────────────────────────────────
    doc = xacro.process_file(file_xacro, mappings={'ns': ns})
    robot_urdf = doc.toprettyxml(indent=' ')

    # ─────────────────────────────────────────────────────────────────────────
    # Bridge Args
    # ─────────────────────────────────────────────────────────────────────────
    bridge_args = [
        bridge(TOPIC_CMD_VEL,
            'geometry_msgs/msg/Twist',
            'ignition.msgs.Twist',
            ']'),

        bridge(TOPIC_JOINTS,
            'sensor_msgs/msg/JointState',
            'ignition.msgs.Model',
            '['),

        bridge(TOPIC_LIDAR,
            'sensor_msgs/msg/LaserScan',
            'ignition.msgs.LaserScan',
            '['),
        
        bridge(TOPIC_IMU,
            'sensor_msgs/msg/Imu',
            'ignition.msgs.IMU',
            '['),

        bridge(TOPIC_ODOM_DIFF_DRIVE,
            'nav_msgs/msg/Odometry',
            'ignition.msgs.Odometry',
            '['),

        bridge(TOPIC_ODOM_PUBLISHER,
            'nav_msgs/msg/Odometry',
            'ignition.msgs.Odometry',
            '['),

        bridge(TOPIC_TF_DIFF_DRIVE,
            'tf2_msgs/msg/TFMessage',
            'ignition.msgs.Pose_V',
            '['),

        bridge(TOPIC_TF_ODOM_PUBLISHER,
            'tf2_msgs/msg/TFMessage',
            'ignition.msgs.Pose_V',
            '['),
    
        # bridge(TOPIC_IMAGE_COLOR,
        #     'sensor_msgs/msg/Image',
        #     'ignition.msgs.Image',
        #     '['),

        # bridge(TOPIC_IMAGE_DEPTH,
        #     'sensor_msgs/msg/Image',
        #     'ignition.msgs.Image',
        #     '['),

        # bridge(TOPIC_CAMERA_INFO,
        #     'sensor_msgs/msg/CameraInfo',
        #     'ignition.msgs.CameraInfo',
        #     '['),

        # bridge(TOPIC_CAMERA_POINTS,
        #     'sensor_msgs/msg/PointCloud2',
        #     'ignition.msgs.PointCloudPacked',
        #     '['),
    ]

    # ─────────────────────────────────────────────────────────────────────────
    # Ramap Topic Name
    # ─────────────────────────────────────────────────────────────────────────
    remappings=[
        (TOPIC_IMU, f'/{ns}/imu'),
        (TOPIC_LIDAR, f'/{ns}/scan'),
        (TOPIC_CMD_VEL, f'/{ns}/cmd_vel'),
        (TOPIC_JOINTS, f'/{ns}/joint_states'),
        (TOPIC_TF_DIFF_DRIVE, '/tf'),
        (TOPIC_TF_ODOM_PUBLISHER, f'/{ns}/tf/odom_publisher'),
        (TOPIC_ODOM_PUBLISHER, f'/{ns}/odom/odom_publisher'),
        (TOPIC_ODOM_DIFF_DRIVE, f'/{ns}/odom/diff_drive'),
        # (TOPIC_IMAGE_COLOR, f'/{ns}/camera/color/image'),
        # (TOPIC_IMAGE_DEPTH, f'/{ns}/camera/depth/image'),
        # (TOPIC_CAMERA_INFO, f'/{ns}/camera/info'),
        # (TOPIC_CAMERA_POINTS, f'/{ns}/camera/depth/points'),
    ]

    # ─────────────────────────────────────────────────────────────────────────
    # Spawn Robot
    # ─────────────────────────────────────────────────────────────────────────
    run_robot = Node(
        package     = 'ros_gz_sim',
        executable  = 'create',
        name        = 'spawn',
        namespace   = ns,
        output      = 'screen',
        parameters  = [{'use_sim_time': True}],
        arguments   = [
            '-string', robot_urdf,
            '-name', ns,
            '-x', x, '-y', y, '-z', z, '-Y', Y,
        ],
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Run Robot State Publisher
    # ─────────────────────────────────────────────────────────────────────────
    run_rsp = Node(
        package     = 'robot_state_publisher',
        executable  = 'robot_state_publisher',
        name        = 'rsp',
        namespace   = ns,
        output      = 'screen',
        parameters  = [
            {'use_sim_time': True},
            {'robot_description': robot_urdf},
        ]
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Run Bridge
    # ─────────────────────────────────────────────────────────────────────────
    run_bridge = Node(
        package     = 'ros_gz_bridge',
        executable  = 'parameter_bridge',
        name        = 'bridge',
        namespace   = ns,
        output      = 'screen',
        arguments   = bridge_args,
        remappings  = remappings,
        parameters  = [
            {'use_sim_time': True},
            {'qos_overrides./tf_static.publisher.durability': 'transient_local'}
        ],
    )

    return [run_robot, run_rsp, run_bridge]


# ****************************************************************************
# LAUNCH DECRIPTION
# ****************************************************************************
def generate_launch_description():
    # ─────────────────────────────────────────────────────────────────────────
    # Simulate Time
    # ─────────────────────────────────────────────────────────────────────────
    simu_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'True'
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Gazebo Ignition
    # ─────────────────────────────────────────────────────────────────────────
    open_ign = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(file_gz_sim),
        launch_arguments={'gz_args': f' -r {file_cfg_world}'}.items()
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Rviz
    # ─────────────────────────────────────────────────────────────────────────
    open_rviz = Node(
        package     = 'rviz2',
        executable  = 'rviz2',
        name        = 'rviz2',
        output      = 'screen',
        arguments   = ['-d', file_cfg_rviz]
        
    )

    # ─────────────────────────────────────────────────────────────────────────
    # Generate Nodes for Each Robot
    # ─────────────────────────────────────────────────────────────────────────
    robot_nodes = []
    for robot in ROBOTS:
        robot_nodes.extend(
            make_robot_nodes(
                ns = robot['ns'],
                x  = robot['x'],
                y  = robot['y'],
                z  = robot['z'],
                Y  = robot['Y'],
            )
        )


    # ─────────────────────────────────────────────────────────────────────────
    # Launch
    # ─────────────────────────────────────────────────────────────────────────
    return LaunchDescription([
        simu_time,
        open_ign,
        open_rviz,
        *robot_nodes
    ])
