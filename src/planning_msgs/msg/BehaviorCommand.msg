# BehaviorCommand.msg

# Behavior constants
uint8 FOLLOW_LANE            = 0
uint8 FOLLOW_LEADING_VEHICLE = 1
uint8 AVOID_STATIC_OBSTACLE  = 2
uint8 STOP_AT_TRAFFIC_LIGHT  = 3
uint8 WAIT_BEHIND_OBSTACLE   = 4

std_msgs/Header header
uint8 behavior
float32 target_speed
float32 safe_distance
string reason
