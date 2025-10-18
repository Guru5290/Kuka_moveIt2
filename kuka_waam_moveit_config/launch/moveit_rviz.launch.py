from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Launch RVIZ with MoveIt motion planning plugin"""
    
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'rviz_config',
            default_value='moveit.rviz',
            description='RVIZ config file'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'monitor_real_robot',
            default_value='false',
            description='Monitor real robot state'
        )
    )
    
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # RVIZ config path
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('kuka_waam_moveit_config'),
        'config',
        rviz_config
    ])
    
    # Robot description for RVIZ
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('kuka_waam_description'),
            'urdf',
            'kr6_r900_waam.urdf.xacro'
        ])
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # RVIZ node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription(declared_arguments + [rviz_node])