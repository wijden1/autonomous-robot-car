import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('car_description')
    robot_description = xacro.process_file(
        os.path.join(pkg, 'urdf', 'car.urdf.xacro')).toxml()
    world = os.path.join(pkg, 'worlds', 'track.sdf')
    auto = LaunchConfiguration('auto')
    use_can = LaunchConfiguration('use_can')
    firmware_dir = os.path.expanduser('~/autonomous-robot-car/firmware/build')

    auto_arg = DeclareLaunchArgument(
        'auto', default_value='true',
        description='Start lane following (true) or drive manually (false)')
    can_arg = DeclareLaunchArgument(
        'use_can', default_value='false',
        description='Drive the wheels through the FreeRTOS motor ECUs over CAN (vcan0)')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r ' + world}.items())

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'car',
                   '-x', '-1.0', '-y', '-1.0', '-z', '0.05'])

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/front_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ])

    # Without CAN: the safety node drives Gazebo directly (/cmd_vel)
    safety_direct = Node(
        package='car_control',
        executable='safety_node',
        parameters=[{'stop_distance': 0.3}],
        condition=UnlessCondition(use_can))

    # With CAN: the safety node output goes to the CAN bridge instead
    safety_can = Node(
        package='car_control',
        executable='safety_node',
        parameters=[{'stop_distance': 0.3}],
        remappings=[('cmd_vel', 'cmd_vel_safe')],
        condition=IfCondition(use_can))

    can_bridge = Node(
        package='car_control',
        executable='can_bridge',
        parameters=[{'can_interface': 'vcan0', 'status_timeout': 0.3}],
        condition=IfCondition(use_can))

    motor_left = ExecuteProcess(
        cmd=[os.path.join(firmware_dir, 'motor_firmware'), '--can', 'vcan0', '--side', 'left'],
        cwd=firmware_dir, name='motor_left', output='screen',
        condition=IfCondition(use_can))

    motor_right = ExecuteProcess(
        cmd=[os.path.join(firmware_dir, 'motor_firmware'), '--can', 'vcan0', '--side', 'right'],
        cwd=firmware_dir, name='motor_right', output='screen',
        condition=IfCondition(use_can))

    lane_detector = Node(
        package='car_control',
        executable='lane_detector',
        condition=IfCondition(auto))

    lane_controller = Node(
        package='car_control',
        executable='lane_controller',
        parameters=[{'speed': 0.15, 'kp': 1.5, 'kd': 0.2}],
        condition=IfCondition(auto))

    sign_detector = Node(
        package='car_control',
        executable='sign_detector',
        parameters=[{'min_area': 1500}],
        condition=IfCondition(auto))

    return LaunchDescription([
        auto_arg, can_arg, gazebo, robot_state_publisher, spawn, bridge,
        safety_direct, safety_can, can_bridge, motor_left, motor_right,
        lane_detector, lane_controller, sign_detector,
    ])