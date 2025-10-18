from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml


def load_yaml(package_name, file_path):
    """Load yaml file"""
    package_path = FindPackageShare(package_name).find(package_name)
    absolute_file_path = PathJoinSubstitution([package_path, file_path])
    
    try:
        with open(absolute_file_path.perform(None), 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        return {}


def launch_setup(context, *args, **kwargs):
    """Setup launch description"""
    
    # Parameters
    robot_model = LaunchConfiguration('robot_model').perform(context)
    use_sim = LaunchConfiguration('use_sim').perform(context)
    
    # Robot description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('kuka_waam_description'),
            'urdf',
            'kr6_r900_waam.urdf.xacro'
        ]),
        ' prefix:= ',
        ' use_fake_hardware:=', use_sim,
        ' mode:=mock' if use_sim == 'true' else ' mode:=hardware'
    ])
    
    robot_description = {'robot_description': robot_description_content.perform(context)}
    
    # SRDF
    robot_description_semantic_content = Command([
        'cat ',
        PathJoinSubstitution([
            FindPackageShare('kuka_waam_moveit_config'),
            'config',
            'kr6_waam.srdf'
        ])
    ])
    
    robot_description_semantic = {
        'robot_description_semantic': robot_description_semantic_content.perform(context)
    }
    
    # Kinematics
    kinematics_yaml = load_yaml(
        'kuka_waam_moveit_config',
        'config/kinematics.yaml'
    )
    robot_description_kinematics = {
        'robot_description_kinematics': kinematics_yaml
    }
    
    # Planning
    ompl_planning_yaml = load_yaml(
        'kuka_waam_moveit_config',
        'config/ompl_planning.yaml'
    )
    
    # Joint limits
    joint_limits_yaml = load_yaml(
        'kuka_waam_moveit_config',
        'config/joint_limits.yaml'
    )
    robot_description_planning = {
        'robot_description_planning': joint_limits_yaml
    }
    
    # Pilz limits
    pilz_cartesian_limits_yaml = load_yaml(
        'kuka_waam_moveit_config',
        'config/pilz_cartesian_limits.yaml'
    )
    
    # Controllers
    moveit_controllers_yaml = load_yaml(
        'kuka_waam_moveit_config',
        'config/moveit_controllers.yaml'
    )
    moveit_controllers = {
        'moveit_simple_controller_manager': moveit_controllers_yaml,
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager'
    }
    
    # Planning pipeline
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning_yaml
    }
    
    # Trajectory execution
    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.5,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01,
    }
    
    # Planning scene monitor
    planning_scene_monitor_parameters = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'planning_scene_monitor_options': {
            'name': 'planning_scene_monitor',
            'robot_description': 'robot_description',
            'joint_state_topic': '/joint_states',
            'attached_collision_object_topic': '/moveit_attached_collision_object',
            'publish_planning_scene_topic': '/moveit_publish_planning_scene',
            'monitored_planning_scene_topic': '/moveit_monitored_planning_scene',
            'wait_for_initial_state_timeout': 10.0,
        }
    }
    
    # Move group node
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipelines,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            pilz_cartesian_limits_yaml,
            {
                'use_sim_time': use_sim == 'true',
                'publish_robot_description_semantic': True,
            }
        ],
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim == 'true'}
        ]
    )
    
    # ROS2 Control Node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            PathJoinSubstitution([
                FindPackageShare('kuka_waam_description'),
                'config',
                'ros2_controllers.yaml'
            ]),
            {'use_sim_time': use_sim == 'true'}
        ],
        output='screen',
        condition=IfCondition(use_sim)
    )
    
    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager',
            '/controller_manager'
        ],
    )
    
    # Joint trajectory controller spawner
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'joint_trajectory_controller',
            '--controller-manager',
            '/controller_manager'
        ],
    )
    
    nodes_to_start = [
        robot_state_publisher,
        move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ]
    
    return nodes_to_start


def generate_launch_description():
    """Generate launch description"""
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'robot_model',
            default_value='kr6_r900_sixx',
            description='Robot model'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description='Use simulation'
        )
    )
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )