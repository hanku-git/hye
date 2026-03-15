#undef PTHREAD_STACK_MIN
#define PTHREAD_STACK_MIN 16384

// #include <zivid_camera/Capture.h>
#include <cmath>
#include <memory>
#include "boost/thread.hpp"
#include "hanyang_matching_msgs/srv/capture.hpp"
#include "hanyang_matching_msgs/srv/capture_assistant_suggest_settings.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>

#include <iostream>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/io/pcd_io.h>
// #include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/simple_publisher_plugin.hpp>
#include <cv_bridge/cv_bridge.h>

#include "ArenaApi.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "hanyang_matching_msgs/srv/zivid_do_scan.hpp"
#include "hanyang_matching_msgs/srv/part_scanning.hpp"
#include "hanyang_matching_msgs/msg/resultmrcnn.hpp"
#include "hanyang_matching_msgs/msg/mask_cloud.hpp"

#include "module/KUAISMath.h"

#include <time.h>      // c++ 현재 시간 출력
#include <sys/stat.h>  // folder check
#include <sys/types.h> // folder
#include <errno.h>     // folder

bool do_save_data = false;
bool do_scan_sampling = false;
size_t sampling_num = 1;
struct stat statbuf;
const char *path;
char save_path[255];
char load_path[255];

cv::Mat raw_points;
cv::Mat mm_points;

static int target_id = 35; // bolt bush
size_t obj_num = 41; // 드럼통, 기존:30
static std::string target_name = "none";
bool is_mask_pixel_fixed = false;
static int mask_pixel_x = 0;
static int mask_pixel_y = 0;
static int mask_pixel_scale_x = 0;
static int mask_pixel_scale_y = 0;

static int learning_data_idx = 0;

int cnt = 0;
cv_bridge::CvImagePtr img_ptr;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scan(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);
std_msgs::msg::Header helios_header;
// const rclcpp::Duration default_wait_duration{30};
constexpr auto ca_suggest_settings_service_name = "/zivid_camera/capture_assistant/suggest_settings";
constexpr auto service_name = "/zivid_scanning";
static const std::string OPENCV_WINDOW = "Image window";
std::vector<double> scan_position_q;
KUAISMATH m_math;

size_t scan_num = 0;
const int UR5CB = 7001;
const int UR10e = 7002;

image_transport::Publisher raw_image_publisher_;
image_transport::Publisher mask_image_publisher_;

// ros::Publisher cloud_pub_;
std::shared_ptr<rclcpp::Node> node;
// auto cloud_pub_ = node->create_publisher;
rclcpp::Publisher<hanyang_matching_msgs::msg::MaskCloud>::SharedPtr cloud_pub_ ;

void on_image_color(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    printf("2D color image received\n");
    // RCLCPP_INFO(node->get_logger(), "2D color image received");
    try
    {
        img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        // cv::resize(img_ptr->image, img_ptr->image, cv::Size(640,480));
        // sensor_msgs::msg::Image::SharedPtr msg_raw = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_ptr->image).toImageMsg();
        raw_image_publisher_.publish(img_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception &e)
    {
        // ROS_ERROR("cv_bridge exception: %s", e.what());
        RCLCPP_ERROR_THROTTLE(node->get_logger(), *node->get_clock(), 1000, "cv_bridge exception: %s", e.what());
        return;
    }
    // ros::NodeHandle n3;
    // hanyang_matching_msgs::srv::PartScanning srv3;
    // ros::ServiceClient client3 = n3.serviceClient<hanyang_matching_msgs::PartScanning>("/is_grasping_scanned");
    /*********************************************************************************************
    std::shared_ptr<rclcpp::Node> node3 = rclcpp::Node::make_shared("zivid_scan3");
    rclcpp::Client<hanyang_matching_msgs::srv::PartScanning>::SharedPtr client3 =
    node3->create_client<hanyang_matching_msgs::srv::PartScanning>("/is_grasping_scanned");
    auto request3 = std::make_shared<hanyang_matching_msgs::srv::PartScanning::Request>();
    request3->is_scanned = true;
    printf("on image 22222\n");
    auto result3 = client3->async_send_request(request3);
    // Wait for the result.
    printf("on image 33333\n");
    if (rclcpp::spin_until_future_complete(node3, result3) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        // ROS_LOG_INFO("scanned finished - service call!");
        printf("scanned finished - service call!\n");
        // RCLCPP_INFO(node3->get_logger(),"scanned finished - service call!");
    }
    else
    {
        // ROS_LOG_INFO("scanned finished - service call fail!");
        printf("scanned finished - service call fail!\n");
        // RCLCPP_INFO(node3->get_logger(),"scanned finished - service call fail!");
    }
    ***********************************************************************************************/
}


void on_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{   
    printf("PointCloud received\n");
    // ROS_LOG_INFO("PointCloud received");
    // RCLCPP_INFO(node->get_logger(),"PointCloud received");
    pcl::fromROSMsg(*msg, *cloud_scan);
}

void on_mrcnn_hanyang_matching_result(const hanyang_matching_msgs::msg::Resultmrcnn& msg)
{
    // ROS_LOG_INFO("mrcnn topic - zivid! (bin_picking)");
    printf("mrcnn topic - zivid! (bin_picking)\n");
    // RCLCPP_INFO(node->get_logger(),"mrcnn topic - zivid! (bin_picking)");
    int target_detection_id = 1;
    double score_mrcnn = 0.0;
    bool is_score = false;
    bool is_target = false;
    int stage_idx = 0;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_mask(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_mask_downsample(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointXYZRGBA pt_tmp;

    std::string detection_mode;
    int detected_id = 1;      // 1: cpu, 2: ram, 3: hdmi, 4: usb, 5: power, 6: ssd, 7: cooler
    if (!is_mask_pixel_fixed) //// Mask RCNN
    {
        detection_mode = "mask_rcnn_detection";
        //// Mask RCNN results processing
        if (msg.class_cnt != 0)
        {
            int class_cnt = msg.class_cnt;

            // ROS_LOG_INFO("class index count: %d", class_cnt);
            // RCLCPP_INFO(node->get_logger(),"class index count: %d", class_cnt);
            printf("class index count: %d\n", class_cnt);
            unsigned int idx_target_mask = 0;
            //// mrcnn results processing
            for (int i = 0; i < class_cnt; i++)
            {
                if (msg.class_ids[i] == target_detection_id)
                {
                    //// 중복 검출 방지 - 더 높은 score를 갖는 물체를 선택
                    if (!is_score)
                    {
                        is_score = true;
                        score_mrcnn = msg.scores[i];
                        // ROS_LOG_INFO("%dth detected target: %d (score: %f)", i, msg.class_ids[i], msg.scores[i]); //'bg' + (1)'cpu_m', (2)'ram_m', (3)'hdmi_m', (4)'usb_m', (5)'power_m'
                        // RCLCPP_INFO(node->get_logger(),"%dth detected target: %d (score: %f)", i, msg.class_ids[i], msg.scores[i]);
                        printf("%dth detected target: %d (score: %f)\n", i, msg.class_ids[i], msg.scores[i]);
                    }
                    else
                    {
                        if (score_mrcnn >= msg.scores[i])
                        {
                            continue;
                        }
                        else
                        {
                            score_mrcnn = msg.scores[i];
                            // ROS_LOG_INFO("%dth detected target: %d (score: %f)", i, msg.class_ids[i], msg.scores[i]); //'bg' + (1)'cpu_m', (2)'ram_m', (3)'hdmi_m', (4)'usb_m', (5)'power_m'
                            // RCLCPP_INFO(node->get_logger(),"%dth detected target: %d (score: %f)", i, msg.class_ids[i], msg.scores[i]);
                            printf("%dth detected target: %d (score: %f)\n", i, msg.class_ids[i], msg.scores[i]);
                        }
                    }

                    ////////////////////////////////////////////////////////////////
                    //// TODO: 빈피킹에서 Overlap 되는 경우에 대한 처리 필요
                    //////// Bounding box comparison - Overlap case check!
                    bool is_overlap = false;
                    bool is_same_stage_overlap = false;
                    sensor_msgs::msg::RegionOfInterest box1;
                    box1 = msg.boxes[i];
                    int px_min = box1.x_offset;
                    int px_max = box1.x_offset + box1.width;
                    int py_min = box1.y_offset;
                    int py_max = box1.y_offset + box1.height;
                    for (int j = 0; j < class_cnt; j++)
                    {
                        if (msg.class_ids[j] != target_detection_id)
                        {
                            sensor_msgs::msg::RegionOfInterest box2;
                            box2 = msg.boxes[j];
                            int roi_x = box2.x_offset + box2.width / 2;
                            int roi_y = box2.y_offset + box2.height / 2;
                            if ((roi_x >= px_min && roi_x <= px_max) && (roi_y >= py_min && roi_y <= py_max))
                            {
                                // ROS_LOG_INFO("Detection failure: overlap between object %d (target) and object %d (mismatch)", target_detection_id, msg.class_ids[j]);
                                // RCLCPP_INFO(node->get_logger(),"Detection failure: overlap between object %d (target) and object %d (mismatch)", target_detection_id, msg.class_ids[j]);
                                printf("Detection failure: overlap between object %d (target) and object %d (mismatch)\n", target_detection_id, msg.class_ids[j]);
                                is_overlap = true;
                            }
                        }
                    }
                    ////////////////////////////////////////////////////////////////
                    
                    idx_target_mask = i; // update
                }
            }
            /////////////////////////////////////////////////////////////////
            ///////////////////// Mask cloud generation /////////////////////
            /////////////////////////////////////////////////////////////////
            detected_id = msg.class_ids[idx_target_mask]; // detection results id
            printf("------- Detected object index: object%d-------\n", detected_id);
            sensor_msgs::msg::Image mask_img;
            mask_img = msg.masks[idx_target_mask];
            // mask_img.data = mask_img.data *255;
            std::cout<<"mask_img_size: "<<mask_img.width<<", "<<mask_img.height<<", "<<mask_img.data.size()<<"\n";
            cv_bridge::CvImagePtr mask_ptr;
            cv_bridge::CvImage msg_crop;
            mask_ptr = cv_bridge::toCvCopy(mask_img, sensor_msgs::image_encodings::BGR8);
            std::cout<<"msg_size: "<<mask_ptr->image.rows<<", "<<mask_ptr->image.cols<<"\n";
            cv::Mat img_scan_tmp = img_ptr->image; // raw scan image
            //// from Mask RCNN results
            // sensor_msgs::RegionOfInterest mask_box;
            auto mask_box = msg.boxes[idx_target_mask];
            // 관심영역 자르기 (Crop ROI).
            cv::Rect rect(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);
            std::cout<<"size: "<<mask_box.x_offset<<", "<<mask_box.y_offset<<"\n";
            cv::Mat img_crop = img_scan_tmp(rect);
            //msg_crop = cv_bridge::toCvCopy(img_crop, sensor_msgs::image_encodings::BGR8);
            msg_crop.header = std_msgs::msg::Header();
            msg_crop.encoding = sensor_msgs::image_encodings::BGR8;
            msg_crop.image = img_crop;
            // sensor_msgs::ImagePtr msg_crop = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_crop).toImageMsg();
            mask_image_publisher_.publish(msg_crop.toImageMsg());
            // ROS_LOG_INFO("Mask image published!");
            // RCLCPP_INFO(node->get_logger(),"Mask image published!");
            printf("Mask image published!\n");
            if (0) // image show
            {
                cv::imshow(OPENCV_WINDOW, img_crop);
                cv::waitKey(2000);
                cv::destroyAllWindows(); // close the window
            }
            std::cout << "[Downsampling of the mask cloud] - sampling number: " << sampling_num << std::endl;
            cv::Mat img_tmp = mask_ptr->image; // mask image
            size_t pixel_margin = 15;
            std::cout<<"img_size: "<<img_tmp.size<<"\n"<<"cloud_size: "<<cloud_scan->size()<<"\n";
            m_math.printCloudMinMax(*cloud_scan, "cloud_scan - before");
            // m_math.extractCloudFromMaskImageWithMargin(pixel_margin, *cloud_scan, img_tmp, sampling_num, *cloud_mask_downsample); // w/o downsampling
            m_math.extractCloudFromMaskImage(*cloud_scan, img_tmp, sampling_num, *cloud_mask_downsample); // w/o downsampling
            m_math.printCloudMinMax(*cloud_mask_downsample, "cloud_mask_downsample - after");


            if(1)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZ>);;
                pcl::copyPointCloud(*cloud_mask_downsample, *cloud_plot);
                std::stringstream ss;
                ss << "scan_cloud_mask.ply";
                pcl::io::savePLYFile(ss.str(), *cloud_plot, true);
            }





            // ROS_LOG_INFO("cloud in mask(downsample): %d", cloud_mask_downsample->size());
            // RCLCPP_INFO(node->get_logger(),"cloud in mask(downsample): %ld", cloud_mask_downsample->size());
            printf("cloud in mask(downsample): %ld\n", cloud_mask_downsample->size());
            *cloud_out = *cloud_mask_downsample;
            is_target = true;
        }
        else // no mask results
        {
            // ROS_LOG_INFO("mrcnn - no detected results!!!");
            // RCLCPP_INFO(node->get_logger(),"mrcnn - no detected results!!!");
            printf("mrcnn - no detected results!!!\n");
            // ROS_LOG_INFO("Object detection failure! (Mask RCNN) - No results");
            // RCLCPP_INFO(node->get_logger(),"Object detection failure! (Mask RCNN) - No results");
            printf("Object detection failure! (Mask RCNN) - No results\n");
            is_target = false;
        }
    }

    hanyang_matching_msgs::msg::MaskCloud msg_cloud;
    if (is_target)
    {
        if (target_detection_id == detected_id)
        {
            msg_cloud.class_ids = target_id - obj_num; // target object idx
            msg_cloud.class_name = target_name;        // target name

            msg_cloud.is_mask_pixel_fixed = is_mask_pixel_fixed;
            msg_cloud.mask_pixel_x = mask_pixel_x;
            msg_cloud.mask_pixel_y = mask_pixel_y;
            msg_cloud.mask_pixel_scale_x = mask_pixel_scale_x;
            msg_cloud.mask_pixel_scale_y = mask_pixel_scale_y;

            msg_cloud.sampling_num = sampling_num;

            // mask rcnn results
            msg_cloud.mrcnn_result = msg;
            msg_cloud.do_save_data = do_save_data;
            msg_cloud.save_path = "";
            msg_cloud.detection_mode = detection_mode;
            // msg_cloud.time_now = time_now;
            msg_cloud.scan_num = scan_num;

            msg_cloud.learning_data_idx = learning_data_idx;
            pcl::toROSMsg(*cloud_out, msg_cloud.mask_cloud);
            pcl::toROSMsg(*cloud_scan, msg_cloud.scan_cloud);
            cloud_pub_->publish(msg_cloud);
            // ROS_LOG_INFO("Object detection success! (Mask RCNN)");
            // RCLCPP_INFO(node->get_logger(),"Object detection success! (Mask RCNN)");
            printf("Object detection success! (Mask RCNN)\n");
            // ROS_LOG_INFO("-----------------------------------Node: zivid_scan/zivid_scan2.cpp");
            // RCLCPP_INFO(node->get_logger(),"-----------------------------------Node: zivid_scan/zivid_scan2.cpp");
            printf("-----------------------------------Node: zivid_scan/zivid_scan2.cpp\n");
        }
        else // 다른 파트를 인식한 경우
        {
            msg_cloud.class_ids = -1;      // target idx
            msg_cloud.class_name = "none"; // target name

            msg_cloud.mask_pixel_x = 0;
            msg_cloud.mask_pixel_y = 0;
            msg_cloud.mask_pixel_scale_x = 0;
            msg_cloud.mask_pixel_scale_y = 0;

            msg_cloud.sampling_num = sampling_num;

            // mask rcnn results
            msg_cloud.mrcnn_result = msg;
            msg_cloud.do_save_data = do_save_data;
            msg_cloud.save_path = "";
            msg_cloud.detection_mode = detection_mode;
            // msg_cloud.time_now = time_now;
            msg_cloud.scan_num = scan_num;

            cloud_out->clear();
            cloud_scan->clear();
            pcl::toROSMsg(*cloud_out, msg_cloud.mask_cloud);
            pcl::toROSMsg(*cloud_scan, msg_cloud.scan_cloud);
            cloud_pub_->publish(msg_cloud);
            // ROS_LOG_INFO("Object detection failure! (Mask RCNN for Bin-picking): Different index between target and detection");
            // RCLCPP_INFO(node->get_logger(),"Object detection failure! (Mask RCNN for Bin-picking): Different index between target and detection");
            printf("Object detection failure! (Mask RCNN for Bin-picking): Different index between target and detection\n");
            // ROS_LOG_INFO("-----------------------------------Node: zivid_scan/zivid_scan2.cpp");
            // RCLCPP_INFO(node->get_logger(),"-----------------------------------Node: zivid_scan/zivid_scan2.cpp");
            printf("-----------------------------------Node: zivid_scan/zivid_scan2.cpp\n");
        }
    }
    else
    {
        msg_cloud.class_ids = -1;      // target idx
        msg_cloud.class_name = "none"; // target name

        msg_cloud.mask_pixel_x = 0;
        msg_cloud.mask_pixel_y = 0;
        msg_cloud.mask_pixel_scale_x = 0;
        msg_cloud.mask_pixel_scale_y = 0;

        msg_cloud.sampling_num = sampling_num;

        // mask rcnn results
        msg_cloud.mrcnn_result = msg;
        msg_cloud.do_save_data = do_save_data;
        msg_cloud.save_path = "";
        msg_cloud.detection_mode = detection_mode;
        // msg_cloud.time_now = time_now;
        msg_cloud.scan_num = scan_num;

        cloud_out->clear();
        cloud_scan->clear();
        pcl::toROSMsg(*cloud_out, msg_cloud.mask_cloud);
        pcl::toROSMsg(*cloud_scan, msg_cloud.scan_cloud);
        cloud_pub_->publish(msg_cloud);
        // ROS_LOG_INFO("Object detection failure! (Mask RCNN for Bin-picking): No target!!!");
        // RCLCPP_INFO(node->get_logger(),"Object detection failure! (Mask RCNN for Bin-picking): No target!!!");
        printf("Object detection failure! (Mask RCNN for Bin-picking): No target!!!\n");
        // ROS_LOG_INFO("-----------------------------------Node: zivid_scan/zivid_scan2.cpp");
        // RCLCPP_INFO(node->get_logger(),"-----------------------------------Node: zivid_scan/zivid_scan2.cpp");
        printf("-----------------------------------Node: zivid_scan/zivid_scan2.cpp\n");
    }


    // Visualization
    if (0)
    {
        pcl::visualization::PCLVisualizer viewer("Filtering");
        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        float txt_color = 0.0; // black text

        // CAD cloud is black
        // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_in_color_h(cloud_target, 0, 0, 0);
        // viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v1", v1);
        // viewer.addPointCloud(cloud_target, cloud_in_color_h, "cloud_in_v2", v2);

        // raw cloud is green
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_raw_color_h(cloud_scan, 10, 10, 255);
        viewer.addPointCloud(cloud_scan, cloud_raw_color_h, "cloud_raw_v1", v1);

        // aligned point cloud is red
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_align_color_h(cloud_mask_downsample, 10, 10, 255);
        viewer.addPointCloud(cloud_mask_downsample, cloud_align_color_h, "cloud_aligned", v2);
        // viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud_aligned name");

        // Adding text descriptions in each viewport
        viewer.addText("Black: Original point cloud\nBlue: raw point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_1", v1);
        viewer.addText("Black: Original point cloud\nBlue: Downsampling point cloud", 10, 15, 16, txt_color, txt_color, txt_color, "icp_info_2", v2);

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
            boost::this_thread::sleep(boost::posix_time::microseconds(100000));
        }
        viewer.close();
    }


    cloud_out->clear();
    cloud_scan->clear();
}

bool do_scanning(const std::shared_ptr<hanyang_matching_msgs::srv::ZividDoScan::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::ZividDoScan::Response> response)
{
    // ROS_LOG_INFO("Scanning start!");
    //RCLCPP_INFO(node->get_logger(),"Scanning start!");
    target_id = request->target_id;
    target_name = request->target_name;
    do_save_data = request->do_save_data;
    do_scan_sampling = request->do_scan_sampling;
    sampling_num = request->sampling_num;
    is_mask_pixel_fixed = request->is_mask_pixel_fixed;
    mask_pixel_x = request->mask_pixel_x;
    mask_pixel_y = request->mask_pixel_y;
    mask_pixel_scale_x = request->mask_pixel_scale_x;
    mask_pixel_scale_y = request->mask_pixel_scale_y;
    learning_data_idx = request->learning_data_idx;

    scan_num++;
    obj_num = 41; // 드럼통, 기존:30

    scan_position_q.resize(6);
    scan_position_q = request->scan_position;
    printf("Scanning & mrcnn target object(idx #%d): %s\n", target_id, target_name.c_str());
    // ROS_LOG_INFO("target scanning position : %f, %f, %f, %f, %f, %f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);
    RCLCPP_INFO(node->get_logger(),"target scanning position : %f, %f, %f, %f, %f, %f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);

    // 2) scanning
    // ros::NodeHandle n2;
    auto node2 = rclcpp::Node::make_shared("zivid_scan2");
    // ros::ServiceClient client2 = n2.serviceClient<hanyang_matching_msgs::Capture>("/zivid_camera/capture");
    auto client2 = node2->create_client<hanyang_matching_msgs::srv::Capture>("/zivid/capture");
    // hanyang_matching_msgs::srv::Capture srv;
    auto request2 = std::make_shared<hanyang_matching_msgs::srv::Capture::Request>();
    auto result2 = client2->async_send_request(request2);
    if (rclcpp::spin_until_future_complete(node2, result2) ==
    rclcpp::FutureReturnCode::SUCCESS)
    {
        // ROS_LOG_INFO("Success(scan)");
        RCLCPP_INFO(node2->get_logger(),"Success(scan)");
    }
    else
    {
        // ROS_LOG_INFO("Failure(scan)");
        RCLCPP_INFO(node2->get_logger(),"Failure(scan)");
    }

    response->is_detected = true;
    return true;
    
}
// Get Helios XYZ data bytes and intensity data:
// void GetHeliosImage(Arena::IDevice* pHeliosDevice, cv::Mat& intensity_image, cv::Mat& xyz_mm)
// {
//     // Get Helios XYZ data bytes and intensity data:
//     Arena::SetNodeValue<GenICam::gcstring>(pHeliosDevice->GetNodeMap(), "PixelFormat", "Coord3D_ABCY16");

//     // Read the scale factor and offsets to convert from unsigned 16-bit values 
//     // in the Coord3D_ABCY16 pixel format to coordinates in mm
//     GenApi::INodeMap* node_map = pHeliosDevice->GetNodeMap();
//     double xyz_scale_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateScale");
//     Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateA");
//     double x_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");
//     Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateB");
//     double y_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");
//     Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateC");
//     double z_offset_mm = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");

//     pHeliosDevice->StartStream();
//     Arena::IImage* image = pHeliosDevice->GetImage(2000);


//     size_t height, width;
//     height = image->GetHeight();
//     width = image->GetWidth();
//     cout<<"height: "<<height<<endl;
//     cout<<"width: "<<width<<endl;

//     xyz_mm = cv::Mat((int)height, (int)width, CV_32FC3);
//     intensity_image = cv::Mat((int)height, (int)width, CV_16UC1);

//     const uint16_t* input_data;
//     input_data = (uint16_t*)image->GetData();

//     cloud_scan->clear();
//     cloud_scan->height = height;
//     cloud_scan->width = width;
//     helios_header.stamp.sec =
//         static_cast<uint32_t>(image->GetTimestampNs() / 1000000000);
//     helios_header.stamp.nanosec =
//         static_cast<uint32_t>(image->GetTimestampNs() % 1000000000);
//     // helios_header.frame_id = std::to_string(image->GetFrameId());
//     helios_header.frame_id = "map";

//     for (unsigned int ir = 0; ir < height; ++ir)
//     {
//         for (unsigned int ic = 0; ic < width; ++ic)
//         {
//             // Get unsigned 16 bit values for X,Y,Z coordinates
//             ushort x_u16 = input_data[0];
//             ushort y_u16 = input_data[1];
//             ushort z_u16 = input_data[2];
//             pcl::PointXYZRGBA point;
        
//             // Convert 16-bit X,Y,Z to float values in mm
//             xyz_mm.at<cv::Vec3f>(ir, ic)[0] = (float)(x_u16 * xyz_scale_mm + x_offset_mm);
//             //point.x = (float)(x_u16 * xyz_scale_mm + x_offset_mm);
//             xyz_mm.at<cv::Vec3f>(ir, ic)[1] = (float)(y_u16 * xyz_scale_mm + y_offset_mm);   
//             //point.y = (float)(y_u16 * xyz_scale_mm + y_offset_mm);      
//             xyz_mm.at<cv::Vec3f>(ir, ic)[2] = (float)(z_u16 * xyz_scale_mm + z_offset_mm);      
//             //point.z = (float)(z_u16 * xyz_scale_mm + z_offset_mm);    
//             intensity_image.at<ushort>(ir, ic) = input_data[3]; // // Intensity value
//             point.x = (float)(0.001*(x_u16 * xyz_scale_mm + x_offset_mm));
//             point.y = (float)(0.001*(y_u16 * xyz_scale_mm + y_offset_mm)); 
//             point.z = (float)(0.001*(z_u16 * xyz_scale_mm + z_offset_mm)); 
//             cloud_scan -> points.push_back(point);
//             input_data += 4;
//         }
//     }
//     // pcl::PointXYZRGBA point;
//     // point.x = (float)(x_u16);
//     // point.y = (float)(y_u16);
//     // point.z = (float)(z_u16);
//     // cloud_scan -> points.push_back(point);
//     pHeliosDevice->StopStream();
    
//     // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
//     // viewer.showCloud (cloud_scan);
//     // int cnt = 0;
//     // while (!viewer.wasStopped()) {
//     //     cnt++;
//     // }

//     // Optional: Show the Helios intensity image
//     // cv::imshow("HLS Intensity", intensity_image);
//     // cv::waitKey(0);
// }

// bool isApplicableDeviceHelios2(Arena::DeviceInfo deviceInfo)
// {
// 	return ((deviceInfo.ModelName().find("HLT") != GenICam::gcstring::npos) || (deviceInfo.ModelName().find("HTP") != GenICam::gcstring::npos) \
// 		|| (deviceInfo.ModelName().find("HTW") != GenICam::gcstring::npos));
// }

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("zivid_scan");
    cloud_pub_ = node->create_publisher<hanyang_matching_msgs::msg::MaskCloud>("/cloud_mask_results", 1);
    // auto helios_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>("/helios_cloud_results", 1);

    //// ZIVID Scanning
    // auto zivid_adv = n.advertiseService("/zivid_scanning", do_scanning);
    auto zivid_adv = node->create_service<hanyang_matching_msgs::srv::ZividDoScan>("/zivid_scanning", do_scanning);
    //// ZIVID raw data 
    // auto points_sub = n.subscribe("/zivid_camera/points/xyzrgba", 1, on_points);
    auto points_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/helios_pub/points", 1, on_points);
    // auto image_color_sub = n.subscribe("/zivid_camera/color/image_color", 1, on_image_color);
    auto image_color_sub = node->create_subscription<sensor_msgs::msg::Image>("/helios_pub/resizedimages", 1, on_image_color);
    //// Mask RCNN results
    // auto mrcnn_sub = n.subscribe("/mask_rcnn_zivid/result", 1, on_mrcnn_hanyang_matching_result);
    auto mrcnn_sub = node->create_subscription<hanyang_matching_msgs::msg::Resultmrcnn>("/mask_rcnn_realsense/result", 1, on_mrcnn_hanyang_matching_result);

    // /**************************************************** Helios Camera preparing  ************************************************************/
    // bool exceptionThrown = false;
    
    // Arena::ISystem* pSystem = Arena::OpenSystem();
    // pSystem->UpdateDevices(100);
    // std::vector<Arena::DeviceInfo> deviceInfos = pSystem->GetDevices();
    // if (deviceInfos.size() == 0)
    // {
    //     std::cout << "\nNo camera connected\nPress enter to complete\n";
    //     std::getchar();
    //     return 0;
    // }
    // Arena::IDevice* pDeviceHLT = nullptr;
    // for (auto& deviceInfo : deviceInfos)
	// 	{
    //         if (!pDeviceHLT && isApplicableDeviceHelios2(deviceInfo))
	// 		{
	// 			pDeviceHLT = pSystem->CreateDevice(deviceInfo);

	// 			// enable stream auto negotiate packet size
	// 			Arena::SetNodeValue<bool>(
	// 				pDeviceHLT->GetTLStreamNodeMap(),
	// 				"StreamAutoNegotiatePacketSize",
	// 				true);

	// 			// enable stream packet resend
	// 			Arena::SetNodeValue<bool>(
	// 				pDeviceHLT->GetTLStreamNodeMap(),
	// 				"StreamPacketResendEnable",
	// 				true);
	// 		}
	// 		else if (isApplicableDeviceHelios2(deviceInfo))
	// 		{
	// 			throw std::logic_error("too many Helios 2 devices connected");
	// 		}
	// 	}
    //     if (!pDeviceHLT)
	// 		throw std::logic_error("No applicable Helios 2 devices");

    //     if (pDeviceHLT)
	// 	{
	// 		std::cout << "Commence example\n\n";
	// 		GetHeliosImage(pDeviceHLT, raw_points, mm_points);
	// 		std::cout << "\nExample complete\n";
	// 	}
    // /*********************************************************************************************************************************/
    // sensor_msgs::msg::PointCloud2 cloud_msg;

    // pcl::toROSMsg(*cloud_scan, cloud_msg);
    // cloud_msg.header = helios_header;
    // helios_pub->publish(cloud_msg);

    // cloud_pub_ = n.advertise<hanyang_matching_msgs::MaskCloud>("/cloud_mask_results", 1);    
    //// visualizer
    image_transport::ImageTransport it(node);
    raw_image_publisher_ = it.advertise("/ku_scan/camera/image", 1);
    mask_image_publisher_ = it.advertise("/ku_scan/camera/mask_image", 1);
    rclcpp::spin(node);
    rclcpp::shutdown();

    // if (pDeviceHLT)
    //     pSystem->DestroyDevice(pDeviceHLT);
    // Arena::CloseSystem(pSystem);

    return (0);
}