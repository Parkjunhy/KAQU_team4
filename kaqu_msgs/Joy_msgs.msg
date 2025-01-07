# std_msgs/Header header

#states: robot's states
# states[0] start
# states[1] walk
# states[2] rest
bool[3] states       

# pose.position: slant-x, slant_y, height 
# pose.orientation: roll, pitch, yaw
geometry_msgs/Pose pose   