# Integer class IDs for each bounding box
int32 class_ids
string class_name
int32 learning_data_idx

# pixel
uint64 img_rows
uint64 img_cols

# Mask index
bool skip_detection_mask
bool is_mask_pixel_fixed
uint64 mask_pixel_x
uint64 mask_pixel_y
uint64 mask_pixel_scale_x
uint64 mask_pixel_scale_y

int32 sampling_num
bool is_base_frame_unknown

# Mask results
Resultsam sam_result
bool do_save_data
bool do_save_mrcnn_learning_data
string save_path
string detection_mode
string time_now
uint64 scan_num

# Mask cloud
sensor_msgs/PointCloud2 mask_cloud
sensor_msgs/PointCloud2 scan_cloud

bool do_single_matching


float64[] robot_dh_parameters
float64[] robot_tcp_default
float64[] robot_tcp
