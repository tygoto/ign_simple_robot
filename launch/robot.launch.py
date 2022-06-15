
import os

from ament_index_python.packages import get_package_share_directory

from os import environ

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg_ign_simple_robot = get_package_share_directory('ign_simple_robot')
    
    robot_sdf_path = os.path.join(pkg_ign_simple_robot, 'models', 'robot.sdf')
    
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': f'-r {robot_sdf_path}'}.items(),
    )
    
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=['/camera@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/camera2@sensor_msgs/msg/Image@ignition.msgs.Image',
                   '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
                   '/model/vehicle/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
                   '/model/vehicle/odometry@nav_msgs/msg/Odometry@ignition.msgs.Odometry'],
        parameters=[{'qos_overrides./model/vehicle.subscriber.reliability' : 'reliable'}],
        output='screen'
    )
    
    return LaunchDescription([
        ign_gazebo,
        bridge,
    ])
