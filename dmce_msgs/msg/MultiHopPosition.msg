# The ID of the robot currently transmitting this message
uint32 senderId

# --- KNOWN POSITIONS ---
# Arrays of equal length representing the swarm's state according to the sender
uint32[] robotIds
float64[] x_positions
float64[] y_positions

# The time the specific robot was actually recorded at this position.
# C will use this to determine if B's forwarded position for A is newer 
# than C's current belief of where A is.
time[] timestamps