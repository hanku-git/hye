std_msgs/Header header

# Bounding boxes in pixels
sensor_msgs/RegionOfInterest[] boxes

# Integer class IDs for each bounding box
int32[] class_ids

# String class IDs for each bouding box
string[] class_names

# Float probability scores of the class_id
float32[] scores

# Instance masks as Image
sensor_msgs/Image[] masks

# Detected class Count
int32 class_cnt

# 
float32[] nearby_ratio_set
bool[] is_multiple_masks_set

# bolt picking data
geometry_msgs/Pose2D[] picking_poses