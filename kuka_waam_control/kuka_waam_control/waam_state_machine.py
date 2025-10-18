from enum import Enum
from kuka_waam_msgs.msg import WAAMStatus, LayerInfo, WeldingState
from kuka_waam_msgs.srv import ExecuteGcode, SetWeldingParams
from std_srvs.srv import Trigger
from std_msgs.msg import Bool
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from safety_monitor import SafetyMonitor


class WAAMState(Enum):
    """WAAM process states"""
    IDLE = "IDLE"
    INITIALIZING = "INITIALIZING"
    MOVING_TO_START = "MOVING_TO_START"
    WELDING = "WELDING"
    LAYER_COMPLETE = "LAYER_COMPLETE"
    COOLING = "COOLING"
    WAITING_USER = "WAITING_USER"
    PAUSED = "PAUSED"
    ERROR = "ERROR"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    COMPLETED = "COMPLETED"


class WAAMStateMachine(Node):
    """
    State machine for WAAM process control
    """
    
    def __init__(self):
        super().__init__('waam_state_machine')
        
        # Parameters
        self.declare_parameter('execution_mode', 'simulation')
        self.declare_parameter('enable_welding', False)
        self.declare_parameter('auto_continue', False)  # Auto-continue after cooling
        
        self.execution_mode = self.get_parameter('execution_mode').value
        self.enable_welding = self.get_parameter('enable_welding').value
        self.auto_continue = self.get_parameter('auto_continue').value
        
        # State
        self.current_state = WAAMState.IDLE
        self.previous_state = WAAMState.IDLE
        self.current_layer = 0
        self.total_layers = 0
        self.layer_start_time = None
        self.cooling_start_time = None
        self.errors = []
        self.warnings = []
        
        # Publishers
        self.status_pub = self.create_publisher(WAAMStatus, '/waam_status', 10)
        self.layer_info_pub = self.create_publisher(LayerInfo, '/layer_info', 10)
        self.welding_state_pub = self.create_publisher(WeldingState, '/welding_state', 10)
        
        # Subscribers
        self.emergency_stop_sub = self.create_subscription(
            Bool, '/emergency_stop_signal', self.emergency_stop_callback, 10
        )
        
        # Services
        self.execute_gcode_srv = self.create_service(
            ExecuteGcode, '/execute_waam_program', self.execute_gcode_callback
        )
        self.set_welding_srv = self.create_service(
            SetWeldingParams, '/set_welding_params', self.set_welding_params_callback
        )
        self.pause_srv = self.create_service(Trigger, '/pause_waam', self.pause_callback)
        self.resume_srv = self.create_service(Trigger, '/resume_waam', self.resume_callback)
        self.abort_srv = self.create_service(Trigger, '/abort_waam', self.abort_callback)
        
        # Timer for status publishing
        self.status_timer = self.create_timer(0.5, self.publish_status)
        
        self.get_logger().info('WAAM State Machine initialized')
        self.get_logger().info(f'Execution mode: {self.execution_mode}')
        self.get_logger().info(f'Welding enabled: {self.enable_welding}')
    
    def transition_to(self, new_state: WAAMState, reason: str = ""):
        """Transition to new state"""
        self.previous_state = self.current_state
        self.current_state = new_state
        
        log_msg = f'State transition: {self.previous_state.value} → {new_state.value}'
        if reason:
            log_msg += f' ({reason})'
        
        self.get_logger().info(log_msg)
        
        # State entry actions
        if new_state == WAAMState.COOLING:
            self.cooling_start_time = self.get_clock().now()
        elif new_state == WAAMState.WELDING:
            if self.layer_start_time is None:
                self.layer_start_time = self.get_clock().now()
        elif new_state == WAAMState.LAYER_COMPLETE:
            self.current_layer += 1
            self.layer_start_time = None
    
    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop signal"""
        if msg.data and self.current_state != WAAMState.EMERGENCY_STOP:
            self.transition_to(WAAMState.EMERGENCY_STOP, "Emergency stop triggered")
            self.errors.append("Emergency stop activated")
    
    def execute_gcode_callback(self, request, response):
        """Execute G-code file"""
        if self.current_state not in [WAAMState.IDLE, WAAMState.COMPLETED]:
            response.success = False
            response.message = f"Cannot execute: current state is {self.current_state.value}"
            return response
        
        self.get_logger().info(f'Executing G-code: {request.gcode_file}')
        
        try:
            # Transition to initializing
            self.transition_to(WAAMState.INITIALIZING)
            
            # Here you would integrate with the KRL executor
            # For now, simulate success
            response.success = True
            response.message = "G-code execution started"
            response.layers_completed = 0
            response.total_time = 0.0
            
        except Exception as e:
            self.transition_to(WAAMState.ERROR, str(e))
            response.success = False
            response.message = f"Execution failed: {str(e)}"
        
        return response
    
    def set_welding_params_callback(self, request, response):
        """Set welding parameters"""
        self.get_logger().info('Updating welding parameters')
        
        # Update parameters
        if request.voltage > 0:
            self.get_logger().info(f'Voltage: {request.voltage}V')
        if request.current > 0:
            self.get_logger().info(f'Current: {request.current}A')
        
        # Create response with current params
        response.success = True
        response.message = "Welding parameters updated"
        
        current_params = WeldingState()
        current_params.voltage = request.voltage if request.voltage > 0 else 24.0
        current_params.current = request.current if request.current > 0 else 180.0
        current_params.wire_feed_rate = request.wire_feed_rate if request.wire_feed_rate > 0 else 8.0
        current_params.travel_speed = request.travel_speed if request.travel_speed > 0 else 8.0
        current_params.weld_mode = request.weld_mode if request.weld_mode else "GMAW"
        
        response.current_params = current_params
        
        return response
    
    def pause_callback(self, request, response):
        """Pause WAAM process"""
        if self.current_state in [WAAMState.WELDING, WAAMState.MOVING_TO_START]:
            self.transition_to(WAAMState.PAUSED)
            response.success = True
            response.message = "Process paused"
        else:
            response.success = False
            response.message = f"Cannot pause from state {self.current_state.value}"
        
        return response
    
    def resume_callback(self, request, response):
        """Resume WAAM process"""
        if self.current_state == WAAMState.PAUSED:
            self.transition_to(self.previous_state, "Resuming")
            response.success = True
            response.message = "Process resumed"
        else:
            response.success = False
            response.message = "Process is not paused"
        
        return response
    
    def abort_callback(self, request, response):
        """Abort WAAM process"""
        if self.current_state != WAAMState.IDLE:
            self.transition_to(WAAMState.IDLE, "Aborted by user")
            self.current_layer = 0
            self.total_layers = 0
            response.success = True
            response.message = "Process aborted"
        else:
            response.success = False
            response.message = "No process to abort"
        
        return response
    
    def publish_status(self):
        """Publish current status"""
        status = WAAMStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.state = self.current_state.value
        status.current_operation = self.get_operation_description()
        status.emergency_stop = (self.current_state == WAAMState.EMERGENCY_STOP)
        status.warnings = self.warnings
        status.errors = self.errors
        
        # Layer info
        layer_info = LayerInfo()
        layer_info.header.stamp = status.header.stamp
        layer_info.layer_number = self.current_layer
        layer_info.total_layers = self.total_layers
        layer_info.completion_pct = (self.current_layer / self.total_layers * 100.0) if self.total_layers > 0 else 0.0
        
        if self.layer_start_time:
            layer_time = (self.get_clock().now() - self.layer_start_time).nanoseconds / 1e9
            layer_info.layer_time = layer_time
        
        if self.cooling_start_time:
            cooling_elapsed = (self.get_clock().now() - self.cooling_start_time).nanoseconds / 1e9
            layer_info.cooling_time = max(0.0, 30.0 - cooling_elapsed)  # 30s cooling
        
        status.layer_info = layer_info
        
        self.status_pub.publish(status)
        self.layer_info_pub.publish(layer_info)
    
    def get_operation_description(self) -> str:
        """Get human-readable operation description"""
        descriptions = {
            WAAMState.IDLE: "System idle",
            WAAMState.INITIALIZING: "Initializing system",
            WAAMState.MOVING_TO_START: f"Moving to start position for layer {self.current_layer + 1}",
            WAAMState.WELDING: f"Depositing layer {self.current_layer + 1} of {self.total_layers}",
            WAAMState.LAYER_COMPLETE: f"Layer {self.current_layer} completed",
            WAAMState.COOLING: f"Cooling layer {self.current_layer}",
            WAAMState.WAITING_USER: "Waiting for user confirmation",
            WAAMState.PAUSED: "Process paused",
            WAAMState.ERROR: "Error state",
            WAAMState.EMERGENCY_STOP: "EMERGENCY STOP ACTIVE",
            WAAMState.COMPLETED: f"Build completed - {self.current_layer} layers"
        }
        return descriptions.get(self.current_state, "Unknown state")


def main_safety_monitor(args=None):
    """Safety monitor main"""
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_state_machine(args=None):
    """State machine main"""
    rclpy.init(args=args)
    node = WAAMStateMachine()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    import sys
    if 'safety' in sys.argv:
        main_safety_monitor()
    else:
        main_state_machine()
