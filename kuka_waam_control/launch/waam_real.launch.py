from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command



def generate_launch_description_real():
    """Launch real robot with WAAM capabilities"""
    
    # Declare arguments
    robot_ip = LaunchConfiguration('robot_ip', default='192.168.1.100')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    enable_welding = LaunchConfiguration('enable_welding', default='false')
    
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.100',
            description='IP address of KUKA robot controller'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RVIZ for monitoring'
        ),
        DeclareLaunchArgument(
            'enable_welding',
            default_value='false',
            description='Enable actual welding (DANGEROUS!)'
        ),
    ]
    
    # KVP Interface
    kvp_interface = Node(
        package='kuka_kvp_command_interface',
        executable='kuka_kvp_interface',
        name='kuka_kvp_interface',
        parameters=[{
            'robot_ip': robot_ip,
            'port': 7000,
        }],
        output='screen'
    )
    
    # Robot State Publisher
    robot_description = PathJoinSubstitution([
        FindPackageShare('kuka_waam_description'),
        'urdf',
        'kr6_r900_waam.urdf.xacro'
    ])
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': Command(['xacro ', robot_description]),
            'use_sim_time': False,
        }],
        output='screen'
    )
    
    # MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('kuka_waam_moveit_config'),
                'launch',
                'move_group.launch.py'
            ])
        ]),
        launch_arguments={
            'robot_model': 'kr6_r900_sixx',
            'use_sim': 'false',
            'robot_ip': robot_ip,
        }.items()
    )
    
    # RVIZ for monitoring
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
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz)
    )
    
    # WAAM State Machine
    waam_state_machine = Node(
        package='kuka_waam_control',
        executable='waam_state_machine.py',
        name='waam_state_machine',
        parameters=[{
            'execution_mode': 'real',
            'enable_welding': enable_welding,
            'robot_ip': robot_ip,
        }],
        output='screen'
    )
    
    # Safety Monitor
    safety_monitor = Node(
        package='kuka_waam_control',
        executable='safety_monitor.py',
        name='safety_monitor',
        parameters=[{
            'max_joint_velocity': 1.0,  # rad/s
            'max_cartesian_velocity': 0.5,  # m/s
            'emergency_stop_enabled': True,
        }],
        output='screen'
    )
    
    return LaunchDescription(
        declared_arguments + [
            kvp_interface,
            robot_state_publisher,
            moveit_launch,
            rviz_node,
            waam_state_machine,
            safety_monitor,
        ]
    )