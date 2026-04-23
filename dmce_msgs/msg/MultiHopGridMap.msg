# The ID of the robot currently transmitting this message
uint32 senderId

# --- VERSION VECTOR ---
# These two arrays must be the same length. They represent the "latest known data"
# that has been fused into this specific map update. 
# e.g., included_robotIds[0] = 1, latest_timestamps[0] = 10:05:01
uint32[] included_robotIds
time[] latest_timestamps

# --- SPARSE MAP DATA ---
# The actual fused occupancy data being shared
#uint32 length
#float64[] x_positions
#float64[] y_positions
#float32[] values

# --- FULL MAP DATA ---
# The actual fused occupancy grid being shared
grid_map_msgs/GridMap map