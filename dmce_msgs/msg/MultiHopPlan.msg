# The ID of the robot currently transmitting this message bundle
uint32 senderId

# --- VERSION VECTOR & PLANS ---
# Arrays of equal length representing the latest known plan for each robot
uint32[] robotIds
time[] latest_timestamps
nav_msgs/Path[] paths