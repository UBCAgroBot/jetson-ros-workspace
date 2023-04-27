# A 2D bounding box ros message

std_msgs/Header header

# Bounding box coordindates
float64 x_min
float64 y_min
float64 x_max 
float64 y_max

int32 obj_id # unique ID of the bounding box object
