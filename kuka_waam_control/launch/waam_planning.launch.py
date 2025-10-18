import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description_planning():
    """Launch MoveIt planning for WAAM"""
    
    use_sim = LaunchConfiguration('use_sim', default='true')
    robot_model = LaunchConfiguration('robot_model', default='kr6_r900_sixx')
    
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation time and fake hardware'
        ),
        DeclareLaunchArgument(
            'robot_model',
            default_value='kr6_r900_sixx',
            description='Robot model'
        ),
    ]
    
    # Move Group
    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kuka_kr_moveit_config'),
                'launch',
                'moveit_planning_gazebo.launch.py' if use_sim else 'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model,
            'robot_family_support': 'kuka_agilus_support',
        }.items()
    )
    
    return LaunchDescription(
        declared_arguments + [
            move_group_launch,
        ]
    )

