std_msgs/Header header
float32 distance
float32 angle
float32 pixel_angle
float32 longitudinal_distance
float32 lateral_distance
float32 u
float32 v
float32 roi_radius
uint8 stability
# 0: unknown/no target
# 1: green light visible and aim data valid
# 2: door open but green light occluded
# 3: door not fully open/blocked
uint8 light_detected
