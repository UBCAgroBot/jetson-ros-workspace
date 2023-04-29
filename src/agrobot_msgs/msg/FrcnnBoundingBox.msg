# Frcnn Bounding Box ros message

std_msgs/Header header

# Bounding box coordindates
vision_msgs/BoundingBox2D[] boxes
float64 confidence_val
int32 plant_id

int32 obj_id # unique ID of the bounding box object
