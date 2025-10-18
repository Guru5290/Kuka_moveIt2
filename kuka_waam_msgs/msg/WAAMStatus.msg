std_msgs/Header header

string state             # IDLE, RUNNING, PAUSED, COOLING, ERROR
string current_operation # Description of current operation
LayerInfo layer_info     # Current layer information
WeldingState weld_state  # Current welding state
geometry_msgs/Pose current_pose  # Current tool pose
bool emergency_stop      # Emergency stop active
string[] warnings        # Active warnings
string[] errors 