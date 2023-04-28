# A 2D bounding box ros message

std_msgs/Header header

# Bounding box coordindates
float64 center_x
float64 center_y
float64 size_x 
float64 size_y
float64 confidence_val
int32 plant_id

int32 obj_id # unique ID of the bounding box object
