std_msgs/Header header

uint32 layer_number      # Current layer number
float32 layer_height     # Height of this layer (mm)
float32 total_height     # Total build height so far (mm)
uint32 total_layers      # Total planned layers
float32 completion_pct   # Completion percentage
float32 layer_time       # Time to complete this layer (s)
float32 cooling_time     # Cooling time remaining (s)
bool user_confirmation   # Waiting for user confirmation