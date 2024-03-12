"""@package docstring
File: avoi_dance.launch.py
Author: Dimitri JACQUEMONT
Date: 2024-03-12
Description: Launch file for the avoi_dance simulation.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_dir_avoi = os.path.join(get_package_share_directory('avoi_dance'), 'launch')
    bringup_dir = os.path.join(get_package_share_directory('irobot_create_gazebo_bringup'), 'launch')

    # Set the robots position and orientation, and the random seed
    x_robot_1 = '0.0'
    y_robot_1 = '0.0'
    yaw_robot_1 = '0.0'
    seed_1 = '42'

    x_robot_2 = '1.0'
    y_robot_2 = '0.0'
    yaw_robot_2 = '0.0'
    seed_2 = '43'

    return LaunchDescription([
        # Launch for the first robot in Gazebo
        IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(bringup_dir_avoi, 'create3_gazebo_aws_small.launch.py')),
            # launch_arguments={'x': x_robot_1, 'y': y_robot_1, 'yaw': yaw_robot_1}.items(),

            # The algorithm implemented in the avoi_dance node does not properly work in the AWS world because 
            # the robot is not able to sense its environment (issues with topics /hazard_detection and /ir_intensity), 
            # and thus collides with object (appart from the other robot which has a known position). A workaround 
            # is to use an empty world without any obstacles, and only the two robots.
            
            PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'create3_gazebo.launch.py')),
            launch_arguments={'namespace': 'robot_1', 'spawn_dock': 'false', 'x': x_robot_1, 'y': y_robot_1, 'yaw': yaw_robot_1}.items(),
        ),
        
        # Launch for the second robot with delay
        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'create3_spawn.launch.py')),
                    launch_arguments={'namespace': 'robot_2', 'spawn_dock': 'false', 'x': x_robot_2, 'y': y_robot_2, 'yaw': yaw_robot_2}.items(),
                )
            ]
        ),
        
        # Declare launch arguments
        DeclareLaunchArgument('namespace_1', default_value='robot_1', description='Namespace for the robot 1 navigation node.'),
        DeclareLaunchArgument('odom_topic_1', default_value='/robot_1/odom', description='Odometry topic for the robot 1.'),
        DeclareLaunchArgument('seed_1', default_value=seed_1, description='Random seed for the robot 1.'),
        DeclareLaunchArgument('cmd_vel_topic_1', default_value='/robot_1/cmd_vel', description='Command velocity topic for the robot 1.'),
        DeclareLaunchArgument('initial_x_robot_1', default_value=x_robot_1, description='Initial x position of robot_1'),
        DeclareLaunchArgument('initial_y_robot_1', default_value=y_robot_1, description='Initial y position of robot_1'),
        DeclareLaunchArgument('initial_yaw_robot_1', default_value=yaw_robot_1, description='Initial yaw of robot_1'),
        DeclareLaunchArgument('namespace_2', default_value='robot_2', description='Namespace for the robot 2 navigation node.'),
        DeclareLaunchArgument('odom_topic_2', default_value='/robot_2/odom', description='Odometry topic for the robot 2.'),
        DeclareLaunchArgument('seed_2', default_value=seed_2, description='Random seed for the robot 2.'),
        DeclareLaunchArgument('cmd_vel_topic_2', default_value='/robot_2/cmd_vel', description='Command velocity topic for the robot 2.'),
        DeclareLaunchArgument('initial_x_robot_2', default_value=x_robot_2, description='Initial x position of robot_2'),
        DeclareLaunchArgument('initial_y_robot_2', default_value=y_robot_2, description='Initial y position of robot_2'),
        DeclareLaunchArgument('initial_yaw_robot_2', default_value=yaw_robot_2, description='Initial yaw of robot_2'),
        
        # Timer action to delay the launch of avoi_dance nodes
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='avoi_dance',
                    executable='robot_nav',
                    namespace=LaunchConfiguration('namespace_1'),
                    parameters=[{
                        'namespace_': LaunchConfiguration('namespace_1'),
                        'odometry_topic_self': LaunchConfiguration('odom_topic_1'),
                        'odometry_topic_other': LaunchConfiguration('odom_topic_2'),
                        'seed': LaunchConfiguration('seed_1'),
                        'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic_1'),
                        'initial_x_self': LaunchConfiguration('initial_x_robot_1'),
                        'initial_y_self': LaunchConfiguration('initial_y_robot_1'),
                        'initial_yaw_self': LaunchConfiguration('initial_yaw_robot_1'),
                        'initial_x_other': LaunchConfiguration('initial_x_robot_2'),
                        'initial_y_other': LaunchConfiguration('initial_y_robot_2'),
                        'initial_yaw_other': LaunchConfiguration('initial_yaw_robot_2'),
                    }],
                    output='screen',
                )
            ]
        ),
        
        TimerAction(
            period=18.0,
            actions=[
                Node(
                    package='avoi_dance',
                    executable='robot_nav',
                    namespace=LaunchConfiguration('namespace_2'),
                    parameters=[{
                        'namespace_': LaunchConfiguration('namespace_2'),
                        'odometry_topic_self': LaunchConfiguration('odom_topic_2'),
                        'odometry_topic_other': LaunchConfiguration('odom_topic_1'),
                        'seed': LaunchConfiguration('seed_2'),
                        'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic_2'),
                        'initial_x_self': LaunchConfiguration('initial_x_robot_2'),
                        'initial_y_self': LaunchConfiguration('initial_y_robot_2'),
                        'initial_yaw_self': LaunchConfiguration('initial_yaw_robot_2'),
                        'initial_x_other': LaunchConfiguration('initial_x_robot_1'),
                        'initial_y_other': LaunchConfiguration('initial_y_robot_1'),
                        'initial_yaw_other': LaunchConfiguration('initial_yaw_robot_1'),
                    }],
                    output='screen',
                )
            ]
        ),
    ])
