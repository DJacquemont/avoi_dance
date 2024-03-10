from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir_avoi = os.path.join(get_package_share_directory('avoi_dance'), 'launch')
    bringup_dir = os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch')

    # Launch for the first robot in Gazebo
    robot_1_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(bringup_dir_avoi, 'create3_gazebo_aws_small.launch.py')),
    launch_arguments={'namespace': 'robot_1', 'x': '0.0'}.items(),
    )

    # Launch for the second robot with delay
    robot_2_launch_delayed = TimerAction(
    period=10.0,
    actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'create3_spawn.launch.py')),
            launch_arguments={'namespace': 'robot_2', 'spawn_dock': 'false', 'x': '1.0'}.items(),
            )
        ]
    )

    # Declare launch arguments for nmespace, and odometry topic
    declare_namespace_argument_robot_1 = DeclareLaunchArgument(
        'namespace_1',
        default_value='/robot_1',
        description='Namespace for the robot 1 navigation node.')
    
    declare_odom_topic_argument_robot_1 = DeclareLaunchArgument(
        'odom_topic_1',
        default_value='/robot_1/odom',
        description='Odometry topic for the robot 1.')
        
    declare_namespace_argument_robot_2 = DeclareLaunchArgument(
        'namespace_2',
        default_value='/robot_2',
        description='Namespace for the robot 2 navigation node.')
    
    declare_odom_topic_argument_robot_2 = DeclareLaunchArgument(
        'odom_topic_2',
        default_value='/robot_2/odom',
        description='Odometry topic for the robot 2.')

    # Avoidance node for robot 1
    robot_nav_node_1 = Node(
        package='avoi_dance',
        executable='robot_nav',
        arguments=[
            LaunchConfiguration('namespace_1'),
            LaunchConfiguration('odom_topic_1'),
        ],
        output='screen',
    )

    # Avoidance node for robot 2
    robot_nav_node_2 = Node(
        package='avoi_dance',
        executable='robot_nav',
        arguments=[
            LaunchConfiguration('namespace_2'),
            LaunchConfiguration('odom_topic_2'),
            ],
        output='screen',
    )

    # Timer action to delay the launch of avoi_dance nodes
    delay_robot_nav_node_1 = TimerAction(
        period=30.0,
        actions=[robot_nav_node_1],
    )
    delay_robot_nav_node_2 = TimerAction(
        period=30.0,
        actions=[robot_nav_node_2],
    )

    return LaunchDescription([
        declare_namespace_argument_robot_1,
        declare_namespace_argument_robot_2,
        declare_odom_topic_argument_robot_1,
        declare_odom_topic_argument_robot_2,
        robot_1_launch,
        robot_2_launch_delayed,
        delay_robot_nav_node_1,
        delay_robot_nav_node_2,
    ])
