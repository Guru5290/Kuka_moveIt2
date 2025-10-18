#!/usr/bin/env python3
"""
KRL Executor Node - Bridges KRL commands to MoveIt2 and ROS2 Control
Executes transpiled KRL programs through ROS2 interfaces
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import RobotTrajectory, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

import re
import numpy as np
from enum import Enum
from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass


class ExecutionMode(Enum):
    """Execution mode for the robot"""
    SIMULATION = "simulation"
    REAL_ROBOT = "real"
    DRY_RUN = "dry_run"


@dataclass
class RobotPose:
    """Robot pose in Cartesian or joint space"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    a: float = 180.0
    b: float = 0.0
    c: float = 0.0
    joints: Optional[List[float]] = None


class KRLExecutorNode(Node):
    """
    ROS2 Node that executes KRL commands through MoveIt2
    """
    
    def __init__(self):
        super().__init__('krl_executor_node')
        
        # Declare parameters
        self.declare_parameter('krl_file', '')
        self.declare_parameter('execution_mode', 'simulation')
        self.declare_parameter('use_moveit', True)
        self.declare_parameter('planning_group', 'manipulator')
        self.declare_parameter('robot_ip', '192.168.1.100')
        self.declare_parameter('enable_welding', True)
        self.declare_parameter('preview_only', False)
        
        # Get parameters
        self.krl_file = self.get_parameter('krl_file').value
        self.execution_mode = ExecutionMode(
            self.get_parameter('execution_mode').value
        )
        self.use_moveit = self.get_parameter('use_moveit').value
        self.planning_group = self.get_parameter('planning_group').value
        self.enable_welding = self.get_parameter('enable_welding').value
        self.preview_only = self.get_parameter('preview_only').value
        
        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # Joint state storage
        self.current_joint_state: Optional[JointState] = None
        self.joint_names = [
            'joint_a1', 'joint_a2', 'joint_a3',
            'joint_a4', 'joint_a5', 'joint_a6'
        ]
        
        # Robot state
        self.origin_offset = [0.0, 0.0, 0.0]  # X, Y, Z offsets
        self.origin_set = False
        self.welding_active = False
        self.current_layer = 0
        
        # Setup publishers
        self.welding_state_pub = self.create_publisher(
            Bool, '/welding_state', 10
        )
        
        # Setup subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Setup action clients
        if self.use_moveit:
            self.move_group_client = ActionClient(
                self,
                MoveGroup,
                '/move_action',
                callback_group=self.callback_group
            )
        
        # Setup service clients
        self.emergency_stop_srv = self.create_client(
            Trigger,
            '/emergency_stop',
            callback_group=self.callback_group
        )
        
        self.get_logger().info("KRL Executor Node initialized")
        self.get_logger().info(f"Execution mode: {self.execution_mode.value}")
        self.get_logger().info(f"Using MoveIt: {self.use_moveit}")
        self.get_logger().info(f"Welding enabled: {self.enable_welding}")
        
        # Wait for services
        if self.use_moveit:
            self.get_logger().info("Waiting for MoveIt action server...")
            self.move_group_client.wait_for_server()
            self.get_logger().info("MoveIt action server ready")
    
    def joint_state_callback(self, msg: JointState):
        """Store current joint state"""
        self.current_joint_state = msg
    
    def parse_e6pos(self, krl_line: str) -> Optional[RobotPose]:
        """
        Parse E6POS from KRL line
        Example: target_pos = {X 100.0, Y 200.0, Z 300.0, A 180.0, B 0.0, C 90.0}
        """
        match = re.search(
            r'X\s*([-+]?[0-9]*\.?[0-9]+),\s*'
            r'Y\s*([-+]?[0-9]*\.?[0-9]+),\s*'
            r'Z\s*([-+]?[0-9]*\.?[0-9]+),\s*'
            r'A\s*([-+]?[0-9]*\.?[0-9]+),\s*'
            r'B\s*([-+]?[0-9]*\.?[0-9]+),\s*'
            r'C\s*([-+]?[0-9]*\.?[0-9]+)',
            krl_line
        )
        
        if match:
            return RobotPose(
                x=float(match.group(1)),
                y=float(match.group(2)),
                z=float(match.group(3)),
                a=float(match.group(4)),
                b=float(match.group(5)),
                c=float(match.group(6))
            )
        return None
    
    def apply_offset(self, pose: RobotPose) -> RobotPose:
        """Apply origin offset to pose"""
        if self.origin_set:
            pose.x += self.origin_offset[0]
            pose.y += self.origin_offset[1]
            pose.z += self.origin_offset[2]
        return pose
    
    def set_welding_output(self, state: bool):
        """Control welding output"""
        if not self.enable_welding or self.preview_only:
            self.get_logger().info(
                f"Welding {'ON' if state else 'OFF'} (simulated)"
            )
            return
        
        self.welding_active = state
        msg = Bool()
        msg.data = state
        self.welding_state_pub.publish(msg)
        
        self.get_logger().info(f"Welding {'ON' if state else 'OFF'}")
    
    async def execute_ptp(self, pose: RobotPose):
        """Execute PTP (Point-to-Point) motion"""
        self.get_logger().info(
            f"PTP to: X={pose.x:.2f}, Y={pose.y:.2f}, Z={pose.z:.2f}"
        )
        
        if self.preview_only:
            self.get_logger().info("Preview mode - motion skipped")
            return True
        
        if self.use_moveit:
            return await self.execute_moveit_goal(pose, "PTP")
        else:
            return await self.execute_direct_motion(pose, "PTP")
    
    async def execute_lin(self, pose: RobotPose):
        """Execute LIN (Linear) motion"""
        self.get_logger().info(
            f"LIN to: X={pose.x:.2f}, Y={pose.y:.2f}, Z={pose.z:.2f}"
        )
        
        if self.preview_only:
            self.get_logger().info("Preview mode - motion skipped")
            return True
        
        if self.use_moveit:
            return await self.execute_moveit_goal(pose, "LIN")
        else:
            return await self.execute_direct_motion(pose, "LIN")
    
    async def execute_moveit_goal(self, pose: RobotPose, 
                                  motion_type: str) -> bool:
        """Execute motion using MoveIt2"""
        # Create MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request.group_name = self.planning_group
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.3
        
        # Set pose target
        from geometry_msgs.msg import PoseStamped
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose.position.x = pose.x / 1000.0  # mm to m
        pose_stamped.pose.position.y = pose.y / 1000.0
        pose_stamped.pose.position.z = pose.z / 1000.0
        
        # Convert A, B, C to quaternion
        from scipy.spatial.transform import Rotation
        r = Rotation.from_euler('zyx', [pose.c, pose.b, pose.a], degrees=True)
        quat = r.as_quat()
        pose_stamped.pose.orientation.x = quat[0]
        pose_stamped.pose.orientation.y = quat[1]
        pose_stamped.pose.orientation.z = quat[2]
        pose_stamped.pose.orientation.w = quat[3]
        
        goal.request.goal_constraints.append(
            self.create_pose_constraint(pose_stamped)
        )
        
        # Send goal and wait
        future = self.move_group_client.send_goal_async(goal)
        
        try:
            goal_handle = await future
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejected by MoveIt")
                return False
            
            result_future = goal_handle.get_result_async()
            result = await result_future
            
            if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
                self.get_logger().info(f"{motion_type} motion successful")
                return True
            else:
                self.get_logger().error(
                    f"{motion_type} motion failed: {result.result.error_code.val}"
                )
                return False
                
        except Exception as e:
            self.get_logger().error(f"Motion execution error: {str(e)}")
            return False
    
    def create_pose_constraint(self, pose_stamped):
        """Create position constraint for MoveIt"""
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive
        
        constraint = Constraints()
        
        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose_stamped.header
        pos_constraint.link_name = "tool0"
        
        # Create bounding box
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [0.001, 0.001, 0.001]
        
        pos_constraint.constraint_region.primitives.append(primitive)
        pos_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        pos_constraint.weight = 1.0
        
        # Orientation constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header = pose_stamped.header
        ori_constraint.link_name = "tool0"
        ori_constraint.orientation = pose_stamped.pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0
        
        constraint.position_constraints.append(pos_constraint)
        constraint.orientation_constraints.append(ori_constraint)
        
        return constraint
    
    async def execute_direct_motion(self, pose: RobotPose, 
                                    motion_type: str) -> bool:
        """Execute motion directly without MoveIt (for simple cases)"""
        # This would interface with KVP or ros2_control directly
        self.get_logger().warn(
            "Direct motion not implemented - use MoveIt mode"
        )
        return False
    
    def parse_krl_file(self, filepath: str) -> List[str]:
        """Parse KRL file and extract commands"""
        commands = []
        
        try:
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith(';'):
                        commands.append(line)
            
            self.get_logger().info(
                f"Loaded {len(commands)} commands from {filepath}"
            )
            return commands
            
        except Exception as e:
            self.get_logger().error(f"Failed to parse KRL file: {str(e)}")
            return []
    
    async def execute_krl_program(self):
        """Main execution loop for KRL program"""
        if not self.krl_file:
            self.get_logger().error("No KRL file specified")
            return
        
        commands = self.parse_krl_file(self.krl_file)
        if not commands:
            self.get_logger().error("No valid commands found")
            return
        
        self.get_logger().info("Starting KRL program execution")
        self.get_logger().info(f"Total commands: {len(commands)}")
        
        target_pose = RobotPose()
        
        for i, cmd in enumerate(commands):
            self.get_logger().info(f"Command {i+1}/{len(commands)}: {cmd[:50]}...")
            
            try:
                # Parse target position
                if 'target_pos = {' in cmd:
                    pose = self.parse_e6pos(cmd)
                    if pose:
                        target_pose = pose
                
                # Apply offsets
                elif '.X = ' in cmd and 'x_offset' in cmd:
                    pass  # Offset handled in apply_offset()
                elif '.Y = ' in cmd and 'y_offset' in cmd:
                    pass
                elif '.Z = ' in cmd and 'z_offset' in cmd:
                    pass
                
                # Execute motion
                elif cmd.startswith('PTP '):
                    await self.execute_ptp(self.apply_offset(target_pose))
                
                elif cmd.startswith('LIN '):
                    await self.execute_lin(self.apply_offset(target_pose))
                
                # Welding control
                elif '$OUT[1] = TRUE' in cmd:
                    self.set_welding_output(True)
                
                elif '$OUT[1] = FALSE' in cmd:
                    self.set_welding_output(False)
                
                # Wait commands
                elif 'WAIT SEC' in cmd:
                    match = re.search(r'WAIT SEC\s+([-+]?[0-9]*\.?[0-9]+)', cmd)
                    if match:
                        wait_time = float(match.group(1))
                        self.get_logger().info(f"Waiting {wait_time}s...")
                        await self.get_clock().sleep_for(
                            rclpy.duration.Duration(seconds=wait_time)
                        )
                
                # Layer transitions
                elif 'LAYER' in cmd and 'COMPLETE' in cmd:
                    self.current_layer += 1
                    self.get_logger().info(
                        f"Layer {self.current_layer} completed"
                    )
                
                # User confirmation
                elif 'WAIT FOR $IN[11]' in cmd:
                    self.get_logger().info(
                        "Waiting for user confirmation (press Enter)..."
                    )
                    input()
                
            except Exception as e:
                self.get_logger().error(
                    f"Error executing command: {str(e)}"
                )
                # Ask user whether to continue
                response = input("Continue execution? (y/n): ")
                if response.lower() != 'y':
                    break
        
        self.get_logger().info("KRL program execution completed")
        self.set_welding_output(False)


async def main_async():
    """Async main function"""
    rclpy.init()
    
    node = KRLExecutorNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Execute KRL program
        await node.execute_krl_program()
        
        # Spin to keep node alive
        executor.spin()
        
    except KeyboardInterrupt:
        node.get_logger().info("Execution interrupted by user")
    finally:
        node.set_welding_output(False)
        node.destroy_node()
        rclpy.shutdown()


def main():
    """Main entry point"""
    import asyncio
    asyncio.run(main_async())


if __name__ == '__main__':
    main()