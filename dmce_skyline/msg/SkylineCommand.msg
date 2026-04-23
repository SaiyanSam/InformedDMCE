# SkylineCommand.msg
std_msgs/Header header

#int32 robot_id

geometry_msgs/Point target_position
bool stop_command

# to help the local planner know if the target is a "Global Rescue" point
bool is_recovery 