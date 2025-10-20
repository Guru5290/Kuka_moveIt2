from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    robot_model = LaunchConfiguration('robot_model', default='kr6_r900_sixx')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_moveit = LaunchConfiguration('use_moveit', default='true')
    world_file = LaunchConfiguration('world_file', default='waam_cell.sdf')
    
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model',
            default_value='kr6_r900_sixx',
            description='Robot model to use'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RVIZ'
        ),
        DeclareLaunchArgument(
            'use_moveit',
            default_value='true',
            description='Start MoveIt move_group'
        ),
        DeclareLaunchArgument(
            'world_file',
            default_value='waam_cell.sdf',
            description='Gazebo world file'
        ),
    ]
    
    # Launch Gazebo with KUKA robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kuka_gazebo'),
                'launch',
                'gazebo.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model,
            'robot_family_support': 'kuka_agilus_support',
            'gz_world': PathJoinSubstitution([
                FindPackageShare('kuka_waam_gazebo'),
                'worlds',
                world_file
            ]),
            'use_fake_hardware': 'false',
        }.items()
    )
    
    # Launch MoveIt (without base nodes - Gazebo handles them)
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kuka_waam_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': robot_model,
            'use_sim': 'true',
            'load_base_nodes': 'false',  # CRITICAL: Gazebo handles robot_state_publisher, ros2_control, and controllers
        }.items(),
        condition=IfCondition(use_moveit)
    )
    
    # RVIZ
    rviz_config = PathJoinSubstitution([
        FindPackageShare('kuka_waam_moveit_config'),
        'config',
        'moveit.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription(
        declared_arguments + [
            gazebo_launch,
            moveit_launch,
            rviz_node
        ]
    )
