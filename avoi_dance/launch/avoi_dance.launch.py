from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to the launch files
    bringup_dir = os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch')

    # Launch for the first robot in Gazebo
    robot_1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([bringup_dir, '/create3_gazebo_aws_small.launch.py']),
        launch_arguments={'namespace': 'robot_1'}.items(),
    )

    # Launch for the second robot with delay
    robot_2_launch_delayed = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([bringup_dir, '/create3_spawn.launch.py']),
                launch_arguments={'namespace': 'robot_2', 'x': '0.0'}.items(),
            )
        ]
    )

    # Avoidance node for both robots
    avoid_dance_node = Node(
        package='avoi_dance',
        executable='avoi_dance',
        name='avoi_dance',
        namespace='robot_avoidance',
    )

    # Timer action to delay the launch of avoi_dance node
    delay_avoid_dance_node = TimerAction(
        period=20.0,
        actions=[avoid_dance_node],
    )

    return LaunchDescription([
        robot_1_launch,
        robot_2_launch_delayed,
        delay_avoid_dance_node,
    ])
