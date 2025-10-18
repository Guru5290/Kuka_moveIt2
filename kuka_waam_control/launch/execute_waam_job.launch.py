from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare



def generate_launch_description_execute():
    """Execute a WAAM job from G-code"""
    
    gcode_file = LaunchConfiguration('gcode_file')
    threshold = LaunchConfiguration('threshold', default='3.0')
    preview_only = LaunchConfiguration('preview_only', default='false')
    enable_welding = LaunchConfiguration('enable_welding', default='false')
    
    declared_arguments = [
        DeclareLaunchArgument(
            'gcode_file',
            description='Path to G-code file'
        ),
        DeclareLaunchArgument(
            'threshold',
            default_value='3.0',
            description='Point filtering threshold in mm'
        ),
        DeclareLaunchArgument(
            'preview_only',
            default_value='false',
            description='Preview only without executing'
        ),
        DeclareLaunchArgument(
            'enable_welding',
            default_value='false',
            description='Enable actual welding'
        ),
    ]
    
    # Transpile G-code to KRL
    transpiler = Node(
        package='kuka_waam_control',
        executable='gcode_transpiler.py',
        name='gcode_transpiler',
        arguments=[gcode_file, threshold],
        output='screen'
    )
    
    # Execute KRL program
    krl_executor = Node(
        package='kuka_waam_control',
        executable='krl_executor_node.py',
        name='krl_executor',
        parameters=[{
            'krl_file': [gcode_file, '.src'],  # Will be gcode_file.src
            'preview_only': preview_only,
            'enable_welding': enable_welding,
            'use_moveit': True,
        }],
        output='screen'
    )
    
    return LaunchDescription(
        declared_arguments + [
            transpiler,
            krl_executor,
        ]
    )


if __name__ == '__main__':
    # This file contains multiple launch descriptions
    # Use the appropriate function when including
    pass
