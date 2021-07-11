# Point to move toward
geometry_msgs/Point destination

# Command to designate the operating mode
bool stop
bool is_relative
bool is_deca

# Parameter that changes operating speed so we don't have to keep remaking goto_node
float32 speed

# Why not add more parameters?
float32 destination_stop_distance
float32 emergency_stop_distance
uint8 emergency_stop_angle