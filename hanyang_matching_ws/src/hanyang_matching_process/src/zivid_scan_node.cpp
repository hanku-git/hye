#undef PTHREAD_STACK_MIN
#define PTHREAD_STACK_MIN 16384

// #include <zivid_interfaces/srv/capture.hpp>
#include <cmath>
#include <memory>
#include "boost/thread.hpp"
// #include "hanyang_matching_msgs/srv/capture.hpp"
// #include "hanyang_matching_msgs/srv/capture_assistant_suggest_settings.hpp"
#include <std_srvs/srv/empty.hpp>
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
#include <pcl/visualization/pcl_visualizer.h>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/simple_publisher_plugin.hpp>
#include <cv_bridge/cv_bridge.h>

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "hanyang_matching_msgs/srv/zivid_do_scan.hpp"
#include "hanyang_matching_msgs/srv/part_scanning.hpp"
#include "hanyang_matching_msgs/msg/resultsam.hpp"
#include "hanyang_matching_msgs/msg/mask_cloud.hpp"
#include "hanyang_matching_msgs/msg/scan_command_msg.hpp"

#include "module/KUPCLMath.h"
#include "module/image2cloud_processing.h"

#include <time.h>      // c++ 현재 시간 출력
#include <sys/stat.h>  // folder check
#include <sys/types.h> // folder
#include <errno.h>     // folder

#include <chrono>

#include "define.h"

using namespace std::chrono_literals;


// // ROS log macro
// const std::string log_prefix = std::string("SCAN_NODE");
// #define _DEBUG
// #define EMPTY_INFO(loger, ...) loger;
// #ifdef _DEBUG
// #define RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), ...)  RCLCPP_INFO (rclcpp::get_logger(log_prefix), __VA_ARGS__);
// #else
// #define RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), ...)  EMPTY_INFO (rclcpp::get_logger(log_prefix), __VA_ARGS__);
// #endif
// #define ROS_LOG_WARN(...)  RCLCPP_WARN (rclcpp::get_logger(log_prefix), __VA_ARGS__);
// #define ROS_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger(log_prefix), __VA_ARGS__);

// #define SCANNER_TYPE_ZIVID2 false
// #define SCANNER_TYPE_ZIVID1 false
// #define SCANNER_TYPE_RVBUST_I2370 true

// std::vector<double> view_frustum_ = {-1.0, 1.0, -1.0, 1.0, 0.650, 0.860}; // {min_x, max_x, min_y, max_y, min_z, max_z}
// std::vector<double> view_frustum_ = {-1.0, 1.0, -1.0, 1.0, 0.450, 1.000}; // {min_x, max_x, min_y, max_y, min_z, max_z}
// std::vector<double> view_frustum_ = {-1.0, 1.0, -1.0, 1.0, 0.650, 0.900}; // {min_x, max_x, min_y, max_y, min_z, max_z}


#if SCANNER_TYPE_ZIVID2
    std::vector<double> view_frustum_ = {-3.0, 3.0, -3.0, 3.0, -3.0, 3.0}; // {min_x, max_x, min_y, max_y, min_z, max_z}
#endif

#if SCANNER_TYPE_ZIVID1
    std::vector<double> view_frustum_ = {-3.0, 3.0, -3.0, 3.0, -3.0, 3.0}; // {min_x, max_x, min_y, max_y, min_z, max_z}
#endif

#if SCANNER_TYPE_RVBUST_I2370
    // std::vector<double> view_frustum_ = {-1.0, 1.0, -1.0, 1.0, 0.650, 0.860}; // {min_x, max_x, min_y, max_y, min_z, max_z}
    std::vector<double> view_frustum_ = {-1.0, 1.0, -1.0, 1.0, 0.250, 1.0000}; // {min_x, max_x, min_y, max_y, min_z, max_z}
#endif

#if SCANNER_TYPE_RVBUST_P31330
    std::vector<double> view_frustum_ = {-1.5, 1.5, -1.5, 1.5, 0.600, 1.8000}; // {min_x, max_x, min_y, max_y, min_z, max_z}
#endif





bool do_save_data = false;
bool do_image_processing = true;
bool do_save_mrcnn_learning_data = false;
bool do_scan_sampling = false;
bool do_robot_calibration = false;
bool is_ply_saved = false;
int cal_scan_num = 0;
std::string cal_save_folder_name;
size_t sampling_num = 1;
bool is_base_frame_unknown = false;
struct stat statbuf;
const char *path;
char save_path[255];
char load_path[255];

static int target_id = 0;
size_t obj_num = 0;
static std::string target_name = "none";
bool is_mask_pixel_fixed = false;
bool skip_detection_mask = false;
static int mask_pixel_x = 0;
static int mask_pixel_y = 0;
static int mask_pixel_scale_x = 0;
static int mask_pixel_scale_y = 0;

static int learning_data_idx = 0;

bool do_single_matching = true;

int cnt = 0;
cv_bridge::CvImagePtr img_ptr;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scan(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZRGBA>);

// const rclcpp::Duration default_wait_duration{30};
// constexpr auto ca_suggest_settings_service_name = "/zivid_camera/capture_assistant/suggest_settings";
constexpr auto service_name = "/zivid_scanning";
static const std::string OPENCV_WINDOW = "Image window";
KUPCLMATH m_math;
IMAGE2CLOUDPROCESSING m_image2cloud_proc;

bool do_align_S_to_B = false;
size_t scan_num = 0;
size_t robot_cal_scan_num = 0;
const int UR5CB = 7001;
const int UR10e = 7002;

image_transport::Publisher raw_image_publisher_;
image_transport::Publisher mask_image_publisher_;

rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr raw_scanned_points_xyz_publisher_ ;


rclcpp::Publisher<hanyang_matching_msgs::msg::MaskCloud>::SharedPtr cloud_results_publisher_ ;
rclcpp::Publisher<hanyang_matching_msgs::msg::MaskCloud>::SharedPtr cloud_pub_ ;
rclcpp::Publisher<hanyang_matching_msgs::msg::ScanCommandMsg>::SharedPtr scan_command_pub_ ;
std::shared_ptr<rclcpp::Node> node;

size_t m_test_scan_count = 0;

std::vector<double> robot_dh_vec_;
std::vector<double> robot_tcp_default_;
std::vector<double> robot_tcp_;
std::vector<double> scan_position_q;

/** @brief Korean: 입력된 점군 데이터의 plot 함수
* @param[in] cloud_in : 입력 점군 데이터
*/
void plotCloud(pcl::PointCloud<pcl::PointXYZ> &cloud_in, float view_scale, std::string plot_name)
{
	pcl::visualization::PCLVisualizer viewer;
	viewer.setBackgroundColor(1, 1, 1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plot(new pcl::PointCloud<pcl::PointXYZ>);
	*cloud_plot = cloud_in;

    // Cal. centroid
    Eigen::Vector4f centroid_CAD;
    pcl::compute3DCentroid(*cloud_plot, centroid_CAD);
    viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
    std::vector<pcl::visualization::Camera> cams;
    viewer.getCameras(cams);
    for (auto &&camera : cams)
    {
        for (int i = 0; i < 3; i++)
        {
            camera.focal[i] = centroid_CAD[i];
        }
    }
    viewer.setCameraParameters(cams[0]);


	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud_plot, 0, 0, 0);
	viewer.addPointCloud<pcl::PointXYZ>(cloud_plot, single_color, plot_name.c_str());
	float txt_gray_lvl = 0.0; // black text
	std::string name = plot_name;
	viewer.addText(plot_name.c_str(), 10, 15, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, name);

	while (!viewer.wasStopped())
	{
        viewer.spin();
		// viewer.spinOnce(100);
	}
}

void filterCloudUsingViewFrustum(const pcl::PointCloud<pcl::PointXYZRGB> &cloud_in, const std::vector<double> view_frustum, pcl::PointCloud<pcl::PointXYZRGB> &cloud_out)
{
    //// TODO: Base frame이 아닌, Bin의 object frame으로 측정 점군을 변환한 뒤에, bin bounding box를 적용하기
    //// Alignment - measured cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(cloud_in, *cloud_src);
    // m_math.cloudScaling(*cloud_src, 0); // mm to m

    double bin_min_x = view_frustum[0];
    double bin_max_x = view_frustum[1];
    double bin_min_y = view_frustum[2];
    double bin_max_y = view_frustum[3];
    double bin_min_z = view_frustum[4];
    double bin_max_z = view_frustum[5];
    if (1)
    {
        printf("Pre-defined view frustum: (min,max): x: (%f, %f), y: (%f, %f), z: (%f, %f)\n", bin_min_x, bin_max_x, bin_min_y, bin_max_y, bin_min_z, bin_max_z);
        printf("\ncloud_base_aligned after_pass_through - bounding box size\n");

        for (size_t i = 0; i < view_frustum.size(); i++)
        {
            printf("%f ", view_frustum[i]);
        }
        printf("\n");
        // m_math.printCloudMinMax(*cloud_src, "cloud_filtered before pass through removal");
    }
    ///////////////////////////////////////////////////////
    //// plane removal filter - using bin bounding box info.
    pcl::PassThrough<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud_src);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(bin_min_x, bin_max_x);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(bin_min_y, bin_max_y);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(bin_min_z, bin_max_z);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    // m_math.cloudScaling(*cloud_filtered, 1); // m to mm

    // update
    cloud_out = *cloud_filtered;
 
    if (0)
    {
        printf("code here 3!\n");
        double visualize_pt_size_1 = 2.5;
        double visualize_pt_size_2 = 5.0;
        pcl::visualization::PCLVisualizer viewer("Cloud before/after Bin Bounding Box Filtering");
        // m_math.cloudScaling(*cloud_filtered, 0); // mm to m

        ///////////////////////////////////////////
        printf("code here 4!\n");
        float view_scale = 0.8;
        Eigen::Vector4f centroid_CAD;
        pcl::compute3DCentroid(*cloud_filtered, centroid_CAD);
        viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = centroid_CAD[i];
            }
        }
        viewer.setCameraParameters(cams[0]);
        ///////////////////////////////////////////
        printf("code here 5!\n");

        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        printf("code here 6!\n");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_in_color_h(cloud_src, 0, 0, 0);
        viewer.addPointCloud(cloud_src, cloud_in_color_h, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_filtered, cloud_in_color_h, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        printf("code here 7!\n");
        //// Bounding box plot
        Eigen::Vector3f pp1(bin_max_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp2(bin_min_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp3(bin_min_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp4(bin_max_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp5(bin_max_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp6(bin_min_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp7(bin_min_x, bin_min_y, bin_max_z);
        Eigen::Vector3f pp8(bin_max_x, bin_min_y, bin_max_z);
        pcl::PointXYZ ppt1(pp1(0), pp1(1), pp1(2));
        pcl::PointXYZ ppt2(pp2(0), pp2(1), pp2(2));
        pcl::PointXYZ ppt3(pp3(0), pp3(1), pp3(2));
        pcl::PointXYZ ppt4(pp4(0), pp4(1), pp4(2));
        pcl::PointXYZ ppt5(pp5(0), pp5(1), pp5(2));
        pcl::PointXYZ ppt6(pp6(0), pp6(1), pp6(2));
        pcl::PointXYZ ppt7(pp7(0), pp7(1), pp7(2));
        pcl::PointXYZ ppt8(pp8(0), pp8(1), pp8(2));
        viewer.addLine(ppt1, ppt2, 1.0, 0.0, 0.0, "1p edge");
        viewer.addLine(ppt2, ppt3, 1.0, 0.0, 0.0, "2p edge");
        viewer.addLine(ppt3, ppt4, 1.0, 0.0, 0.0, "3p edge");
        viewer.addLine(ppt4, ppt1, 1.0, 0.0, 0.0, "4p edge");
        viewer.addLine(ppt5, ppt6, 1.0, 0.0, 0.0, "5p edge");
        viewer.addLine(ppt6, ppt7, 1.0, 0.0, 0.0, "6p edge");
        viewer.addLine(ppt7, ppt8, 1.0, 0.0, 0.0, "7p edge");
        viewer.addLine(ppt8, ppt5, 1.0, 0.0, 0.0, "8p edge");
        viewer.addLine(ppt1, ppt5, 1.0, 0.0, 0.0, "9p edge");
        viewer.addLine(ppt2, ppt6, 1.0, 0.0, 0.0, "10p edge");
        viewer.addLine(ppt3, ppt7, 1.0, 0.0, 0.0, "11p edge");
        viewer.addLine(ppt4, ppt8, 1.0, 0.0, 0.0, "12p edge");

        printf("code here 8!\n");

        while (!viewer.wasStopped())
        {
            // viewer.spinOnce(100);
            viewer.spin();
            // boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        }
        viewer.close();
        printf("code here 9\n");

    }
}

void filterCloudUsingViewFrustum(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud_in, const std::vector<double> view_frustum, pcl::PointCloud<pcl::PointXYZRGBA> &cloud_out)
{
    //// TODO: Base frame이 아닌, Bin의 object frame으로 측정 점군을 변환한 뒤에, bin bounding box를 적용하기
    //// Alignment - measured cloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::copyPointCloud(cloud_in, *cloud_src);
    // m_math.cloudScaling(*cloud_src, 0); // mm to m

    double bin_min_x = view_frustum[0];
    double bin_max_x = view_frustum[1];
    double bin_min_y = view_frustum[2];
    double bin_max_y = view_frustum[3];
    double bin_min_z = view_frustum[4];
    double bin_max_z = view_frustum[5];
    if (1)
    {
        printf("Pre-defined view frustum: (min,max): x: (%f, %f), y: (%f, %f), z: (%f, %f)\n", bin_min_x, bin_max_x, bin_min_y, bin_max_y, bin_min_z, bin_max_z);
        printf("\ncloud_base_aligned after_pass_through - bounding box size\n");

        for (size_t i = 0; i < view_frustum.size(); i++)
        {
            printf("%f ", view_frustum[i]);
        }
        printf("\n");
        // m_math.printCloudMinMax(*cloud_src, "cloud_filtered before pass through removal");
    }
    ///////////////////////////////////////////////////////
    //// plane removal filter - using bin bounding box info.
    pcl::PassThrough<pcl::PointXYZRGBA> filter;
    filter.setInputCloud(cloud_src);
    filter.setFilterFieldName("x");
    filter.setFilterLimits(bin_min_x, bin_max_x);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("y");
    filter.setFilterLimits(bin_min_y, bin_max_y);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    filter.setInputCloud(cloud_filtered);
    filter.setFilterFieldName("z");
    filter.setFilterLimits(bin_min_z, bin_max_z);
    filter.setNegative(false);
    filter.filter(*cloud_filtered);

    // m_math.cloudScaling(*cloud_filtered, 1); // m to mm

    // update
    cloud_out = *cloud_filtered;
 
    if (0)
    {
        printf("code here 3!\n");
        double visualize_pt_size_1 = 2.5;
        double visualize_pt_size_2 = 5.0;
        pcl::visualization::PCLVisualizer viewer("Cloud before/after Bin Bounding Box Filtering");
        // m_math.cloudScaling(*cloud_filtered, 0); // mm to m

        ///////////////////////////////////////////
        printf("code here 4!\n");
        float view_scale = 0.8;
        Eigen::Vector4f centroid_CAD;
        pcl::compute3DCentroid(*cloud_filtered, centroid_CAD);
        viewer.setCameraPosition(centroid_CAD[0], centroid_CAD[1] + view_scale, centroid_CAD[2] + view_scale, 0, 0, 1);
        std::vector<pcl::visualization::Camera> cams;
        viewer.getCameras(cams);
        for (auto &&camera : cams)
        {
            for (int i = 0; i < 3; i++)
            {
                camera.focal[i] = centroid_CAD[i];
            }
        }
        viewer.setCameraParameters(cams[0]);
        ///////////////////////////////////////////
        printf("code here 5!\n");

        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer.setBackgroundColor(1, 1, 1, v1);
        viewer.setBackgroundColor(1, 1, 1, v2);
        // CAD cloud is black
        printf("code here 6!\n");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBA> cloud_in_color_h(cloud_src, 0, 0, 0);
        viewer.addPointCloud(cloud_src, cloud_in_color_h, "cloud_in_v1", v1);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v1");
        viewer.addPointCloud(cloud_filtered, cloud_in_color_h, "cloud_in_v2", v2);
        viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, visualize_pt_size_1, "cloud_in_v2");

        printf("code here 7!\n");
        //// Bounding box plot
        Eigen::Vector3f pp1(bin_max_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp2(bin_min_x, bin_max_y, bin_min_z);
        Eigen::Vector3f pp3(bin_min_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp4(bin_max_x, bin_min_y, bin_min_z);
        Eigen::Vector3f pp5(bin_max_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp6(bin_min_x, bin_max_y, bin_max_z);
        Eigen::Vector3f pp7(bin_min_x, bin_min_y, bin_max_z);
        Eigen::Vector3f pp8(bin_max_x, bin_min_y, bin_max_z);
        pcl::PointXYZ ppt1(pp1(0), pp1(1), pp1(2));
        pcl::PointXYZ ppt2(pp2(0), pp2(1), pp2(2));
        pcl::PointXYZ ppt3(pp3(0), pp3(1), pp3(2));
        pcl::PointXYZ ppt4(pp4(0), pp4(1), pp4(2));
        pcl::PointXYZ ppt5(pp5(0), pp5(1), pp5(2));
        pcl::PointXYZ ppt6(pp6(0), pp6(1), pp6(2));
        pcl::PointXYZ ppt7(pp7(0), pp7(1), pp7(2));
        pcl::PointXYZ ppt8(pp8(0), pp8(1), pp8(2));
        viewer.addLine(ppt1, ppt2, 1.0, 0.0, 0.0, "1p edge");
        viewer.addLine(ppt2, ppt3, 1.0, 0.0, 0.0, "2p edge");
        viewer.addLine(ppt3, ppt4, 1.0, 0.0, 0.0, "3p edge");
        viewer.addLine(ppt4, ppt1, 1.0, 0.0, 0.0, "4p edge");
        viewer.addLine(ppt5, ppt6, 1.0, 0.0, 0.0, "5p edge");
        viewer.addLine(ppt6, ppt7, 1.0, 0.0, 0.0, "6p edge");
        viewer.addLine(ppt7, ppt8, 1.0, 0.0, 0.0, "7p edge");
        viewer.addLine(ppt8, ppt5, 1.0, 0.0, 0.0, "8p edge");
        viewer.addLine(ppt1, ppt5, 1.0, 0.0, 0.0, "9p edge");
        viewer.addLine(ppt2, ppt6, 1.0, 0.0, 0.0, "10p edge");
        viewer.addLine(ppt3, ppt7, 1.0, 0.0, 0.0, "11p edge");
        viewer.addLine(ppt4, ppt8, 1.0, 0.0, 0.0, "12p edge");

        printf("code here 8!\n");

        while (!viewer.wasStopped())
        {
            // viewer.spinOnce(100);
            viewer.spin();
            // boost::this_thread::sleep(boost::posix_time::microseconds(10000));
        }
        viewer.close();
        printf("code here 9\n");

    }
}


void on_image_color(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
{
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "2D color image received");
    try
    {
        img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        raw_image_publisher_.publish(img_ptr->toImageMsg());
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("scan_node"), *node->get_clock(), 1000, "cv_bridge exception: %s", e.what());
        return;
    }

    // std::shared_ptr<rclcpp::Node> node3 = rclcpp::Node::make_shared("zivid_scan");
    // rclcpp::Client<hanyang_matching_msgs::srv::PartScanning>::SharedPtr client3 =
    // node3->create_client<hanyang_matching_msgs::srv::PartScanning>("/is_grasping_scanned");
    // auto request3 = std::make_shared<hanyang_matching_msgs::srv::PartScanning::Request>();
    // request3->is_scanned = true;

    // while (!client3->wait_for_service(1s)) {
    //     if (!rclcpp::ok()) {
    //         ROS_LOG_ERROR("Interrupted while waiting for the service. Exiting.");
    //         break;
    //     }
    //     ROS_LOG_ERROR("service not available, waiting again...");
    // }

    // auto result3 = client3->async_send_request(request3);
    // // Wait for the result.
    // if (rclcpp::spin_until_future_complete(node3, result3) == rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "scanned finished - service call!");
    // } else {
    //     ROS_LOG_ERROR("scanned finished - service call fail!");
    // }

    //// save results
    if (do_save_data) {

        std::stringstream ss;
        ss << "/home/[zivid_scan_data]/scan_image_" << scan_num << ".jpg";

        cv::imwrite(ss.str().c_str(), img_ptr->image);
        // ////////////////////////////////////////////////////////////////////////////////////
        // ////////////////////////////////////////////////////////////////////////////////////
        // //// File name
        // time_t timer;
        // struct tm *t;
        // timer = time(NULL);    // 1970년 1월 1일 0시 0분 0초부터 시작하여 현재까지의 초
        // t = localtime(&timer); // 포맷팅을 위해 구조체에 넣기
        // std::stringstream ss;
        // char ch[12]; // 1(sign) + 11(character)
        // int ich;
        // ich = snprintf(ch, 12, "%02dh-%02dm-%02ds", t->tm_hour, t->tm_min, t->tm_sec);
        // std::string time_now(ch);
        // ss << save_path << "/scan_image_" << time_now << "_" << scan_num << ".jpg";
        // cv::imwrite(ss.str(), img_ptr->image);
        // ////////////////////////////////////////////////////////////////////////////////////
        // ////////////////////////////////////////////////////////////////////////////////////
    }
}

void on_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    //// NOTICE: Zivid2+ raw metric: [mm], resolution: 1224x1024 (point size: 1253376)
    //// NOTICE: Zivid1 raw metric: [m], resolution: 1920x1200 (point size: 2304000)
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "PointCloud received");
    pcl::fromROSMsg(*msg, *cloud_scan); 

#if SCANNER_TYPE_ZIVID2
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "ZIVID2+ 3D scanner");
    m_math.cloudScaling(*cloud_scan, 0); // mm to m, (22.04, SDK newest version) Zivid2 & Zivid1 raw metric: [mm]
#endif

#if SCANNER_TYPE_ZIVID1
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "ZIVID1 3D scanner");
    m_math.cloudScaling(*cloud_scan, 0); // mm to m, (22.04, SDK newest version) Zivid2 & Zivid1 raw metric: [mm]
#endif

#if SCANNER_TYPE_RVBUST_I2370
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "RVBUST (RVC-I2370) 3D scanner");
#endif

#if SCANNER_TYPE_RVBUST_P31330
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "RVBUST (RVC-P31300) 3D scanner");
#endif

    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "ROSMsg to pcd");

    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud_scan, *cloud_merge_vis);
    m_math.printCloudMinMax(*cloud_scan, "raw_scan_cloud");
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "raw_scan_cloud size: %zu", cloud_scan->size());

#if SCANNER_TYPE_RVBUST_I2370
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "RVBUST 3D scanner (RVC-I2370) remove Nan points");
    // boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud_merge_vis, *indices);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_merge_vis);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud_merge_vis);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merge_vis_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_merge_vis, *cloud_merge_vis_xyz);
    m_math.printCloudMinMax(*cloud_merge_vis_xyz, "raw_scan_cloud - after removing Nan points");
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "raw_scan_cloud size (after removing Nan points): %zu", cloud_merge_vis_xyz->size());

    //// Filtering using view frustum
    filterCloudUsingViewFrustum(*cloud_merge_vis, view_frustum_, *cloud_merge_vis);
    pcl::copyPointCloud(*cloud_merge_vis, *cloud_merge_vis_xyz);
    m_math.printCloudMinMax(*cloud_merge_vis_xyz, "raw_scan_cloud - after filter using view frustum");
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "raw_scan_cloud size (after filter using view frustum): %zu", cloud_merge_vis_xyz->size());

#endif


#if SCANNER_TYPE_RVBUST_P31330
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "RVBUST 3D scanner (RVC-P31300) remove Nan points");
    // boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
    pcl::IndicesPtr indices(new std::vector<int>);
    pcl::removeNaNFromPointCloud(*cloud_merge_vis, *indices);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(cloud_merge_vis);
    extract.setIndices(indices);
    extract.setNegative(false);
    extract.filter(*cloud_merge_vis);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merge_vis_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_merge_vis, *cloud_merge_vis_xyz);
    m_math.printCloudMinMax(*cloud_merge_vis_xyz, "raw_scan_cloud - after removing Nan points");
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "raw_scan_cloud size (after removing Nan points): %zu", cloud_merge_vis_xyz->size());

    //// Filtering using view frustum
    filterCloudUsingViewFrustum(*cloud_merge_vis, view_frustum_, *cloud_merge_vis);
    pcl::copyPointCloud(*cloud_merge_vis, *cloud_merge_vis_xyz);
    m_math.printCloudMinMax(*cloud_merge_vis_xyz, "raw_scan_cloud - after filter using view frustum");
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "raw_scan_cloud size (after filter using view frustum): %zu", cloud_merge_vis_xyz->size());

#endif



    pcl::toROSMsg(*cloud_merge_vis, cloud_msg);
    cloud_msg.header.frame_id = "ku_cloud_frame";
    raw_scanned_points_xyz_publisher_->publish(cloud_msg);

    //// Timer log
    time_t timer;
    struct tm *t;
    timer = time(NULL);    // 1970년 1월 1일 0시 0분 0초부터 시작하여 현재까지의 초
    t = localtime(&timer); // 포맷팅을 위해 구조체에 넣기


    char ch_test[255];
    int ich_test;
    ich_test = snprintf(ch_test, 255, "/home/[zivid_scan_data]/%4d-%02d-%02d-%02dh-%02dm-%02ds", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday, t->tm_hour, t->tm_min, t->tm_sec);
    std::string test_time_now(ch_test);

    m_test_scan_count++;
    ROS_LOG_WARN("[Test scan #%zu]Test Time: %s", m_test_scan_count, test_time_now.c_str());
    ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);

    if(skip_detection_mask) {
        std::string detection_mode;
        detection_mode = "skip_detection_mode";

        ROS_LOG_WARN("ON_POINTS: SKIP DETECTION MASK!!!");
        //// File name
        time_t timer;
        struct tm *t;
        timer = time(NULL);    // 1970년 1월 1일 0시 0분 0초부터 시작하여 현재까지의 초
        t = localtime(&timer); // 포맷팅을 위해 구조체에 넣기
        std::stringstream ss;
        char ch[12]; // 1(sign) + 11(character)
        int ich;
        ich = snprintf(ch, 12, "%02dh-%02dm-%02ds", t->tm_hour, t->tm_min, t->tm_sec);
        std::string time_now(ch);
        ss << save_path << "/scan_cloud_" << detection_mode << "_" << time_now << "_" << scan_num << ".ply";
        std::string save_path_str(save_path);

        hanyang_matching_msgs::msg::MaskCloud msg_cloud;
        //// DH, TCP
        msg_cloud.robot_dh_parameters = robot_dh_vec_;
        msg_cloud.robot_tcp_default = robot_tcp_default_;
        msg_cloud.robot_tcp = robot_tcp_;
        msg_cloud.class_ids = target_id - obj_num; // target object idx
        msg_cloud.class_name = target_name;        // target name

        msg_cloud.skip_detection_mask = skip_detection_mask;
        msg_cloud.is_mask_pixel_fixed = is_mask_pixel_fixed;
        msg_cloud.mask_pixel_x = 0;
        msg_cloud.mask_pixel_y = 0;
        msg_cloud.mask_pixel_scale_x = 0;
        msg_cloud.mask_pixel_scale_y = 0;

        msg_cloud.sampling_num = sampling_num;
        msg_cloud.is_base_frame_unknown = is_base_frame_unknown;

        // mask results
        // msg_cloud.sam_result = msg;
        msg_cloud.do_save_data = do_save_data;
        msg_cloud.do_save_mrcnn_learning_data = do_save_mrcnn_learning_data;
        msg_cloud.save_path = save_path_str;
        msg_cloud.detection_mode = detection_mode;
        msg_cloud.time_now = time_now;
        msg_cloud.scan_num = scan_num;

        msg_cloud.learning_data_idx = learning_data_idx;

        msg_cloud.do_single_matching = do_single_matching;

        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "ONLY SCAN w/o Detection! (skip_detection_mask)");
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "-----------------------------------Node: zivid_scan/zivid_scan_node.cpp");

        //// Publish cloud
        pcl::toROSMsg(*cloud_out, msg_cloud.scan_cloud);
        pcl::toROSMsg(*cloud_scan, msg_cloud.scan_cloud);
        cloud_pub_->publish(msg_cloud);
        cloud_out->clear();
        cloud_scan->clear();

    }


    //// save results
    if (do_save_data)
    {
        ////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////
        //// File name
        char ch[12]; // 1(sign) + 11(character)
        int ich;
        ich = snprintf(ch, 12, "%02dh-%02dm-%02ds", t->tm_hour, t->tm_min, t->tm_sec);
        std::string time_now(ch);
        std::stringstream ss;
        ss << save_path << "/scan_cloud_raw_" << time_now << "_" << scan_num << ".ply";

        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_save(new pcl::PointCloud<pcl::PointXYZRGBA>);
        pcl::copyPointCloud(*cloud_scan, *cloud_save);

        //// Remove Nan points
        pcl::IndicesPtr indices(new std::vector<int>);
        pcl::removeNaNFromPointCloud(*cloud_save, *indices);
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud(cloud_save);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*cloud_save);

        filterCloudUsingViewFrustum(*cloud_save, view_frustum_, *cloud_save);

        pcl::io::savePLYFile(ss.str(), *cloud_save, true); // binary
        ROS_LOG_WARN("PLY CLOUD SAVED! - %s", ss.str().c_str());

        //// ply save
        //// raw data
        // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // pcl::fromROSMsg(*msg, *cloud);

        // //// downsampled data
        // size_t scale_downsample = 3;
        // pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGBA>);
        // pcl::PointXYZRGBA pt_tmp;
        // for (size_t x = 0; x < 1920; x++) // 1920
        // {
        //     for (size_t y = 0; y < 1200; y++) // 1200
        //     {
        //         pt_tmp.x = cloud_scan->points[1920 * y + x].x;
        //         pt_tmp.y = cloud_scan->points[1920 * y + x].y;
        //         pt_tmp.z = cloud_scan->points[1920 * y + x].z;
        //         pt_tmp.r = cloud_scan->points[1920 * y + x].r;
        //         pt_tmp.g = cloud_scan->points[1920 * y + x].g;
        //         pt_tmp.b = cloud_scan->points[1920 * y + x].b;
        //         if (std::isnan(pt_tmp.x) == 0 && std::isnan(pt_tmp.y) == 0 && std::isnan(pt_tmp.z) == 0)
        //         {
        //             // Downsampling in scan cloud - 중간 픽셀만 선택
        //             if (x % scale_downsample == 0 && y % scale_downsample == 0)
        //             {
        //                 cloud_downsampled->points.push_back(pt_tmp);
        //             }
        //         }
        //     }
        // }
        // boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
        // pcl::removeNaNFromPointCloud(*cloud_downsampled, *indices);
        // pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        // extract.setInputCloud(cloud_downsampled);
        // extract.setIndices(indices);
        // extract.setNegative(false);
        // extract.filter(*cloud_downsampled);
        // pcl::io::savePLYFile(ss.str(), *cloud_downsampled, true); // binary


        // //// ZIVID robot calibration data save
        // double sensor_min_x = -0.15;
        // double sensor_max_x = 0.5;
        // double sensor_min_y = -0.5;
        // double sensor_max_y = 0.5;
        // double sensor_min_z = 0.3;
        // double sensor_max_z = 0.8;
        // if (1)
        // {
        //     printf("Pre-defined Sensor(ZIVID) bounding box: (min,max): x: (%f, %f), y: (%f, %f), z: (%f, %f)\n", sensor_min_x, sensor_max_x, sensor_min_y, sensor_max_y, sensor_min_z, sensor_max_z);
        //     m_math.printCloudMinMax(*cloud_downsampled, "cloud_filtered before sensor View Frustum removal");
        // }
        // ///////////////////////////////////////////////////////
        // //// plane removal filter - using bin bounding box info.
        // pcl::PassThrough<pcl::PointXYZRGBA> filter;
        // filter.setInputCloud(cloud_downsampled);
        // filter.setFilterFieldName("x");
        // filter.setFilterLimits(sensor_min_x, sensor_max_x);
        // filter.setNegative(false);
        // filter.filter(*cloud_downsampled);

        // filter.setInputCloud(cloud_downsampled);
        // filter.setFilterFieldName("y");
        // filter.setFilterLimits(sensor_min_y, sensor_max_y);
        // filter.setNegative(false);
        // filter.filter(*cloud_downsampled);

        // filter.setInputCloud(cloud_downsampled);
        // filter.setFilterFieldName("z");
        // filter.setFilterLimits(sensor_min_z, sensor_max_z);
        // filter.setNegative(false);
        // filter.filter(*cloud_downsampled);

        // if (1) m_math.printCloudMinMax(*cloud_downsampled, "cloud_filtered after sensor View Frustum removal");

        // // Final results
        // sensor_msgs::msg::PointCloud2 cloud_msg;
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
        // cloud_vis->clear();
        // pcl::copyPointCloud(*cloud_downsampled, *cloud_vis);
        // pcl::toROSMsg(*cloud_vis, cloud_msg); // [m]
        // cloud_msg.header.frame_id = "zivid_optical_frame";
        // cloud_results_publisher_.publish(cloud_msg);


        // ss.str("");
        // m_math.cloudScaling(*cloud_downsampled, 1); // m to mm
        // if (1) m_math.printCloudMinMax(*cloud_downsampled, "cloud_filtered after cloud scaling to [mm]");
        // ss << save_path << "/robot_cal_scan_" << time_now << "_" << robot_cal_scan_num << ".ply";
        // pcl::io::savePLYFile(ss.str(), *cloud_downsampled, true); // binary
    }
}

void on_sam_hanyang_matching_result(const hanyang_matching_msgs::msg::Resultsam &msg)
{
    if(skip_detection_mask) {
        ROS_LOG_WARN("[%s] SKIP_DETECTION_MASK! - return", __func__);
        return;
    }
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "sam topic - zivid! (bin_picking)");
    int target_detection_id = 1;
    double score_mrcnn = 0.0;
    bool is_score = false;
    bool is_target = false;
    std::string detection_mode;
    int detected_id = 0;

    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_mask(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_mask_downsample(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointXYZRGBA pt_tmp;
    if(do_image_processing) {
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "on_sam_hanyang_matching_result - Do processing!");
    } else {
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "on_sam_hanyang_matching_result - No processing!");
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "-----------------------------------Node: zivid_scan/zivid_scan_node.cpp");
        cloud_out->clear();
        cloud_scan->clear();
        return;
    }

    ////////////////////////////////////////////////////////
    //// Mask selection
    //// SAM results processing
    detection_mode = "mask_sam_detection";
    ROS_LOG_WARN("[on_sam_hanyang_matching_result] msg.class_cnt: %zu", msg.class_cnt);
    ROS_LOG_WARN("[on_sam_hanyang_matching_result] msg.class_cnt: %zu", msg.class_cnt);
    ROS_LOG_WARN("[on_sam_hanyang_matching_result] msg.class_cnt: %zu", msg.class_cnt);
    ROS_LOG_WARN("[on_sam_hanyang_matching_result] msg.class_cnt: %zu", msg.class_cnt);
    ROS_LOG_WARN("[on_sam_hanyang_matching_result] msg.class_cnt: %zu", msg.class_cnt);
    if (msg.class_cnt != 0) {
        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        ///////////////////////// Mask selection ////////////////////////
        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        //// 여러 mask 중에 특정 조건에 맞는 1개의 마스크를 선택 ////
        int class_cnt = msg.class_cnt;

        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "class index count: %d", class_cnt);
        unsigned int idx_target_mask = 0;
        //// SAM results processing
        for (int i = 0; i < class_cnt; i++) {
            // printf("msg.class_ids[i]: %d\n", msg.class_ids[i]);
            // if (msg.class_ids[i] == target_detection_id) { sam_node에서 mask 분류기로 미리 분류됨
                //////// 중복 검출 방지 - 더 높은 score를 갖는 물체를 선택
                // if (!is_score) {
                //     is_score = true;
                //     score_mrcnn = msg.scores[i];
                //     RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "%dth detected target: %d (score: %f)", i, msg.class_ids[i], msg.scores[i]);
                // } else {
                //     if (score_mrcnn >= msg.scores[i]) {
                //         continue;
                //     } else {
                //         score_mrcnn = msg.scores[i];
                //         RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "%dth detected target: %d (score: %f)", i, msg.class_ids[i], msg.scores[i]);
                //     }
                // }
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
                for (int j = 0; j < class_cnt; j++) {
                    if (msg.class_ids[j] != target_detection_id) {
                        sensor_msgs::msg::RegionOfInterest box2;
                        box2 = msg.boxes[j];
                        int roi_x = box2.x_offset + box2.width / 2;
                        int roi_y = box2.y_offset + box2.height / 2;
                        if ((roi_x >= px_min && roi_x <= px_max) && (roi_y >= py_min && roi_y <= py_max)) {
                            RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Detection failure: overlap between object %d (target) and object %d (mismatch)", target_detection_id, msg.class_ids[j]);
                            is_overlap = true;
                        }
                    }
                }
                ////////////////////////////////////////////////////////////////
                idx_target_mask = i; // update
            // }
        }
        /////////////////////////////////////////////////////////////////

        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        ///////////////////// Mask cloud generation /////////////////////
        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        /////////////////////////////////////////////////////////////////
        detected_id = msg.class_ids[idx_target_mask]; // detection results id
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "------- Detected object index: object%d-------", detected_id);
        sensor_msgs::msg::Image mask_img;
        mask_img = msg.masks[idx_target_mask];
        cv_bridge::CvImagePtr mask_ptr;
        mask_ptr = cv_bridge::toCvCopy(mask_img, sensor_msgs::image_encodings::BGR8);
        cout << "debug taek 111"<<endl;
        cout << "idx_target_mask:" << idx_target_mask << endl;
        if(0) {
            if (1) { // crop
                cv::Mat img_scan_tmp = img_ptr->image; // raw scan image
                sensor_msgs::msg::RegionOfInterest mask_box;
                mask_box = msg.boxes[idx_target_mask];
                cout << "debug taek 222"<<endl;
                cv::Rect rect(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);
                cv::Mat img_crop = img_scan_tmp(rect); // 관심영역 자르기 (Crop ROI).
                sensor_msgs::msg::Image::SharedPtr msg_crop = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_crop).toImageMsg();
                cout << "debug taek 333"<<endl;
                mask_image_publisher_.publish(msg_crop);
                RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Mask image published!");
                if (0) {
                    cv::imshow(OPENCV_WINDOW, img_crop);
                    cv::waitKey(2000);
                    cv::destroyAllWindows(); // close the window
                }
            } else {
                cv::Mat img_crop = mask_ptr->image;
                sensor_msgs::msg::Image::SharedPtr msg_crop = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_crop).toImageMsg();
                mask_image_publisher_.publish(msg_crop);
                RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Mask image published!");
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "[Downsampling of the mask cloud] - sampling number: %zu", sampling_num);
        cv::Mat img_tmp = mask_ptr->image; // mask image
        if (0) {
            m_image2cloud_proc.extractCloudFromMaskImage(*cloud_scan, img_tmp, sampling_num, *cloud_mask_downsample); // w/o downsampling
            // RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "input cloud: %zu", cloud_scan->size());
            RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "cloud in mask(downsample): %zu", cloud_mask_downsample->size());
        } else {
            size_t pixel_margin = 1;
            m_image2cloud_proc.extractCloudFromMaskImageWithMargin(pixel_margin, *cloud_scan, img_tmp, sampling_num, *cloud_mask_downsample); // w/o downsampling
            // RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "input cloud: %zu", cloud_scan->size());
            RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "cloud in mask(downsample): %zu", cloud_mask_downsample->size());
        }

#if SCANNER_TYPE_RVBUST_I2370
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "RVBUST 3D scanner (RVC-I2370) remove Nan points");
        // boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
        pcl::IndicesPtr indices(new std::vector<int>);
        pcl::removeNaNFromPointCloud(*cloud_mask_downsample, *indices);
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud(cloud_mask_downsample);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*cloud_mask_downsample);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mask_downsample_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_mask_downsample, *cloud_mask_downsample_xyz);
        m_math.printCloudMinMax(*cloud_mask_downsample_xyz, "mask_cloud - after removing Nan points");
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "mask_cloud size (after removing Nan points): %zu", cloud_mask_downsample_xyz->size());

        //// Filtering using view frustum
        filterCloudUsingViewFrustum(*cloud_mask_downsample, view_frustum_, *cloud_mask_downsample);
        pcl::copyPointCloud(*cloud_mask_downsample, *cloud_mask_downsample_xyz);
        m_math.printCloudMinMax(*cloud_mask_downsample_xyz, "mask_cloud - after filter using view frustum");
        if(0) {
            plotCloud(*cloud_mask_downsample_xyz, 0.8, "cloud_mask_downsample");
        }
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "mask_cloud size (after filter using view frustum): %zu", cloud_mask_downsample_xyz->size());
#endif

#if SCANNER_TYPE_RVBUST_P31330
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "RVBUST 3D scanner (RVC-P31300) remove Nan points");
        // boost::shared_ptr<std::vector<int> > indices(new std::vector<int>);
        pcl::IndicesPtr indices(new std::vector<int>);
        pcl::removeNaNFromPointCloud(*cloud_mask_downsample, *indices);
        pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
        extract.setInputCloud(cloud_mask_downsample);
        extract.setIndices(indices);
        extract.setNegative(false);
        extract.filter(*cloud_mask_downsample);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_mask_downsample_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*cloud_mask_downsample, *cloud_mask_downsample_xyz);
        m_math.printCloudMinMax(*cloud_mask_downsample_xyz, "mask_cloud - after removing Nan points");
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "mask_cloud size (after removing Nan points): %zu", cloud_mask_downsample_xyz->size());

        //// Filtering using view frustum
        filterCloudUsingViewFrustum(*cloud_mask_downsample, view_frustum_, *cloud_mask_downsample);
        pcl::copyPointCloud(*cloud_mask_downsample, *cloud_mask_downsample_xyz);
        m_math.printCloudMinMax(*cloud_mask_downsample_xyz, "mask_cloud - after filter using view frustum");
        if(0) {
            plotCloud(*cloud_mask_downsample_xyz, 0.8, "cloud_mask_downsample");
        }
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "mask_cloud size (after filter using view frustum): %zu", cloud_mask_downsample_xyz->size());
#endif




        //// update
        *cloud_out = *cloud_mask_downsample;
        is_target = true;
    } else { // no mask results
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "SAM - no detected results!!!");
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Object detection failure! (SAM) - No results");
        is_target = false;
    }

    //// File name
    time_t timer;
    struct tm *t;
    timer = time(NULL);    // 1970년 1월 1일 0시 0분 0초부터 시작하여 현재까지의 초
    t = localtime(&timer); // 포맷팅을 위해 구조체에 넣기
    std::stringstream ss;
    char ch[12]; // 1(sign) + 11(character)
    int ich;
    ich = snprintf(ch, 12, "%02dh-%02dm-%02ds", t->tm_hour, t->tm_min, t->tm_sec);
    std::string time_now(ch);
    ss << save_path << "/scan_cloud_" << detection_mode << "_" << time_now << "_" << scan_num << ".ply";
    std::string save_path_str(save_path);

    hanyang_matching_msgs::msg::MaskCloud msg_cloud;
    //// DH, TCP
    msg_cloud.robot_dh_parameters = robot_dh_vec_;
    msg_cloud.robot_tcp_default = robot_tcp_default_;
    msg_cloud.robot_tcp = robot_tcp_;

    if (is_target) {
        if (target_detection_id == detected_id) {
            msg_cloud.class_ids = target_id - obj_num; // target object idx
            msg_cloud.class_name = target_name;        // target name

            msg_cloud.skip_detection_mask = skip_detection_mask;
            msg_cloud.is_mask_pixel_fixed = is_mask_pixel_fixed;
            msg_cloud.mask_pixel_x = mask_pixel_x;
            msg_cloud.mask_pixel_y = mask_pixel_y;
            msg_cloud.mask_pixel_scale_x = mask_pixel_scale_x;
            msg_cloud.mask_pixel_scale_y = mask_pixel_scale_y;

            msg_cloud.sampling_num = sampling_num;
            msg_cloud.is_base_frame_unknown = is_base_frame_unknown;

            // mask results
            msg_cloud.sam_result = msg;
            msg_cloud.do_save_data = do_save_data;
            msg_cloud.do_save_mrcnn_learning_data = do_save_mrcnn_learning_data;
            msg_cloud.save_path = save_path_str;
            msg_cloud.detection_mode = detection_mode;
            msg_cloud.time_now = time_now;
            msg_cloud.scan_num = scan_num;

            msg_cloud.learning_data_idx = learning_data_idx;

            msg_cloud.do_single_matching = do_single_matching;

            RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Object detection success! (SAM)");

            if(target_id == 37) { // Bolt
                RCLCPP_WARN(rclcpp::get_logger("SCAN_NODE"), "************************************");
                RCLCPP_WARN(rclcpp::get_logger("SCAN_NODE"), "[Bolt] picking_poses result");

                std::vector<std::vector<double>> set_picking_poses;
                for (size_t i = 0; i < msg.picking_poses.size(); i++) {
                    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "[Mask idx #%zu] x: %0.3f, y: %0.3f, theta: %0.3f", i + 1 , msg.picking_poses[i].x, msg.picking_poses[i].y, msg.picking_poses[i].theta);
                    std::vector<double> pose_set(3);
                    pose_set[0] = msg.picking_poses[i].x;
                    pose_set[1] = msg.picking_poses[i].y;
                    pose_set[2] = msg.picking_poses[i].theta;
                    set_picking_poses.push_back(pose_set);
                }

                RCLCPP_WARN(rclcpp::get_logger("SCAN_NODE"), "************************************");
                for (size_t i = 0; i < set_picking_poses.size(); i++) {
                    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "[Mask idx #%zu] x: %0.3f, y: %0.3f, theta: %0.3f", i + 1, set_picking_poses[i][0], set_picking_poses[i][1], set_picking_poses[i][2]);
                }
            }



            RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "-----------------------------------Node: zivid_scan/zivid_scan_node.cpp");
        } else { // 다른 파트를 인식한 경우
            msg_cloud.class_ids = target_id - obj_num; // target object idx
            msg_cloud.class_name = "none"; // target name

            msg_cloud.skip_detection_mask = skip_detection_mask;
            msg_cloud.mask_pixel_x = 0;
            msg_cloud.mask_pixel_y = 0;
            msg_cloud.mask_pixel_scale_x = 0;
            msg_cloud.mask_pixel_scale_y = 0;

            msg_cloud.sampling_num = sampling_num;
            msg_cloud.is_base_frame_unknown = is_base_frame_unknown;

            // mask results
            msg_cloud.sam_result = msg;
            msg_cloud.do_save_data = do_save_data;
            msg_cloud.do_save_mrcnn_learning_data = do_save_mrcnn_learning_data;
            msg_cloud.save_path = save_path_str;
            msg_cloud.detection_mode = detection_mode;
            msg_cloud.time_now = time_now;
            msg_cloud.scan_num = scan_num;

            msg_cloud.do_single_matching = do_single_matching;

            cloud_out->clear();
            
            RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Object detection failure! (SAM for Bin-picking): Different index between target and detection");
            RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "-----------------------------------Node: zivid_scan/zivid_scan_node.cpp");
        }
    } else {
        msg_cloud.class_ids = target_id - obj_num; // target object idx
        msg_cloud.class_name = "none"; // target name

        msg_cloud.skip_detection_mask = skip_detection_mask;
        msg_cloud.mask_pixel_x = 0;
        msg_cloud.mask_pixel_y = 0;
        msg_cloud.mask_pixel_scale_x = 0;
        msg_cloud.mask_pixel_scale_y = 0;

        msg_cloud.sampling_num = sampling_num;
        msg_cloud.is_base_frame_unknown = is_base_frame_unknown;

        // mask results
        msg_cloud.sam_result = msg;
        msg_cloud.do_save_data = do_save_data;
        msg_cloud.do_save_mrcnn_learning_data = do_save_mrcnn_learning_data;
        msg_cloud.save_path = save_path_str;
        msg_cloud.detection_mode = detection_mode;
        msg_cloud.time_now = time_now;
        msg_cloud.scan_num = scan_num;

        msg_cloud.do_single_matching = do_single_matching;

        cloud_out->clear();

        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Object detection failure! (SAM for Bin-picking): No target!!!");
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "-----------------------------------Node: zivid_scan/zivid_scan_node.cpp");
    }

    //// Publish cloud
    pcl::toROSMsg(*cloud_out, msg_cloud.mask_cloud);
    pcl::toROSMsg(*cloud_scan, msg_cloud.scan_cloud);
    cloud_pub_->publish(msg_cloud);
    cloud_out->clear();
    cloud_scan->clear();
}

bool do_scanning(const std::shared_ptr<hanyang_matching_msgs::srv::ZividDoScan::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::ZividDoScan::Response> response)
{
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Scanning start!");
    target_id = request->target_id;
    target_name = request->target_name;
    do_save_data = request->do_save_data;
    do_image_processing = request->do_image_processing;

    do_single_matching = request->do_single_matching;

    do_save_mrcnn_learning_data = request->do_save_mrcnn_learning_data;
    do_scan_sampling = request->do_scan_sampling;
    sampling_num = request->sampling_num;
    is_base_frame_unknown = request->is_base_frame_unknown;

    skip_detection_mask = request->skip_detection_mask;
    is_mask_pixel_fixed = request->is_mask_pixel_fixed;
    mask_pixel_x = request->mask_pixel_x;
    mask_pixel_y = request->mask_pixel_y;
    mask_pixel_scale_x = request->mask_pixel_scale_x;
    mask_pixel_scale_y = request->mask_pixel_scale_y;
    scan_num++;
    if(robot_cal_scan_num > 60) robot_cal_scan_num = 0;
    robot_cal_scan_num++;

    learning_data_idx = request->learning_data_idx;

    //// Robot calibration
    do_robot_calibration = request->do_robot_calibration;
    cal_save_folder_name = request->cal_save_folder_name;
    cal_scan_num = request->cal_scan_num;

    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Scanning & mrcnn target object(idx #%d): %s", target_id, target_name.c_str());
    if (target_id == 21) { // Harting
        do_align_S_to_B = true;
    } else if (target_id >= 31) { // bin picking object (31~36)
        do_align_S_to_B = false;
        // obj_num = 41; // 드럼통, 기존:30 // 기존 빈피킹
        obj_num = 41; // 드럼통 과제
    }

    //// 241125 추가
    std::vector<double> robot_dh_vec = request->robot_dh_parameters; // [m], [deg]
    std::vector<double> robot_tcp_default = request->robot_tcp_default; // [m], [deg]
    std::vector<double> robot_tcp = request->robot_tcp; // [m], [deg]
    printf("\n\n\nDH: ");
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 4; j++)
        {
            printf("%0.6f ", robot_dh_vec[i * 4 + j]);
        }
        printf("\n");
    }

    ROS_LOG_INFO("\nRobot TCP default[m, deg]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f", robot_tcp_default[0], robot_tcp_default[1], robot_tcp_default[2], robot_tcp_default[3], robot_tcp_default[4], robot_tcp_default[5]);
    ROS_LOG_INFO("\nRobot TCP[m, deg]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f", robot_tcp[0], robot_tcp[1], robot_tcp[2], robot_tcp[3], robot_tcp[4], robot_tcp[5]);

    /////////////////////////////////////////////
    robot_dh_vec_ = robot_dh_vec;
    robot_tcp_default_ = robot_tcp_default;
    robot_tcp_ = robot_tcp;
    ////////////////////////////////////////////

    scan_position_q.resize(6);
    scan_position_q = request->scan_position;
    printf("Scanning & sam target object(idx #%d): %s\n", target_id, target_name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Target scanning JS position : %f, %f, %f, %f, %f, %f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);

    if (request->do_load_data) {
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Only Loading mode (no scanning) - Uncheck the load data button!");

        // load raw image
        std::stringstream ss;
        ss << load_path << "/test_image.jpg";
        cv::Mat img_tmp = imread(ss.str(), cv::IMREAD_COLOR);
        cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
        cv_ptr->image = img_tmp;
        img_ptr = cv_ptr; // update
        sensor_msgs::msg::Image::SharedPtr msg_raw = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_ptr->image).toImageMsg();
        raw_image_publisher_.publish(msg_raw);

        // load mask image
        cv::Mat img_scan_tmp = img_ptr->image; // raw scan image
        cv::Rect rect(mask_pixel_x, mask_pixel_y, mask_pixel_scale_x, mask_pixel_scale_y); // 관심영역 자르기 (Crop ROI).
        cv::Mat img_crop = img_scan_tmp(rect);
        sensor_msgs::msg::Image::SharedPtr msg_crop = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_crop).toImageMsg();
        mask_image_publisher_.publish(msg_crop);

        // load cloud
        ss.str("");
        ss << load_path << "/test_cloud.ply";

        pcl::io::loadPLYFile(ss.str(), *cloud_out);
        pcl::io::loadPLYFile(ss.str(), *cloud_scan);

        hanyang_matching_msgs::msg::MaskCloud msg_cloud;
        //// DH, TCP
        msg_cloud.robot_dh_parameters = robot_dh_vec_;
        msg_cloud.robot_tcp_default = robot_tcp_default_;
        msg_cloud.robot_tcp = robot_tcp_;
        msg_cloud.class_ids = target_id - obj_num; // target idx
        msg_cloud.class_name = target_name;        // target name

        msg_cloud.skip_detection_mask = skip_detection_mask;
        msg_cloud.is_mask_pixel_fixed = is_mask_pixel_fixed;
        msg_cloud.mask_pixel_x = mask_pixel_x;
        msg_cloud.mask_pixel_y = mask_pixel_y;
        msg_cloud.mask_pixel_scale_x = mask_pixel_scale_x;
        msg_cloud.mask_pixel_scale_y = mask_pixel_scale_y;

        pcl::toROSMsg(*cloud_out, msg_cloud.mask_cloud);
        pcl::toROSMsg(*cloud_scan, msg_cloud.scan_cloud);

        // ros::NodeHandle nh;
        // auto cloud_pub_ = nh.advertise<hanyang_matching_msgs::MaskCloud>("/cloud_mask_results", 1);
        cloud_pub_->publish(msg_cloud);
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Object detection success! (Mask RCNN)");
        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "-----------------------------------Node: zivid_scan/zivid_scan_node.cpp");

        response->is_detected = true;
        return true;
    }

    // 2) scanning command - topic (/zivid/capture2)
    hanyang_matching_msgs::msg::ScanCommandMsg scan_msg;
    scan_msg.target_name = target_name;
    scan_command_pub_->publish(scan_msg);

    // // 2) scanning - service (/zivid/capture)
    // std::shared_ptr<rclcpp::Node> node2 = rclcpp::Node::make_shared("zivid_camera_client");
    // rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client2 = node2->create_client<std_srvs::srv::Empty>("/zivid/capture");
    // auto request2 = std::make_shared<std_srvs::srv::Empty::Request>();
    // auto result2 = client2->async_send_request(request2);

    // if (rclcpp::spin_until_future_complete(node2, result2) == rclcpp::FutureReturnCode::SUCCESS) {
    //     RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Success(scan)");
    // } else {
    //     RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "Failure(scan)");
    // }

    response->is_detected = true;

    return true;
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("zivid_scan");

    //// Directory path
    snprintf(load_path, 255, "/home/[zivid_scan_data]");
    time_t timer;
    struct tm *t;
    timer = time(NULL);    // 1970년 1월 1일 0시 0분 0초부터 시작하여 현재까지의 초
    t = localtime(&timer); // 포맷팅을 위해 구조체에 넣기
    int idir_ch;
    idir_ch = snprintf(save_path, 255, "/home/[zivid_scan_data]/%4d-%02d-%02d", t->tm_year + 1900, t->tm_mon + 1, t->tm_mday);
    path = save_path; // 경로
    if (stat(path, &statbuf) != -1) {
        if (S_ISDIR(statbuf.st_mode)) { // 디렉토리인지 확인
            std::cout << "Path exist!! ==> " << path << std::endl;
        }
    } else {
        std::cout << "No path ==> " << path << std::endl;
        if (mkdir(path, 0776) == -1 && errno != EEXIST) {
            fprintf(stderr, "%s directory create error: %s\n", strerror(errno));
        } else {
            std::cout << "Folder has been created! ==> " << path << std::endl;
        }
    }


    auto zivid_adv = node->create_service<hanyang_matching_msgs::srv::ZividDoScan>("/zivid_scanning", &do_scanning);
    auto points_sub = node->create_subscription<sensor_msgs::msg::PointCloud2>("/zivid/points/xyzrgba", 1, on_points);
    auto image_color_sub = node->create_subscription<sensor_msgs::msg::Image>("/zivid/color/image_color", 1, on_image_color);
    auto sam_sub = node->create_subscription<hanyang_matching_msgs::msg::Resultsam>("/sam_zivid/result", 1, on_sam_hanyang_matching_result);

    cloud_pub_ = node->create_publisher<hanyang_matching_msgs::msg::MaskCloud>("/cloud_mask_results", 1);
    scan_command_pub_ = node->create_publisher<hanyang_matching_msgs::msg::ScanCommandMsg>("/zivid/capture2", 1);

    //// rviz visualizer
    image_transport::ImageTransport it(node);
    raw_image_publisher_ = it.advertise("/ku_scan/camera/image", 1);
    mask_image_publisher_ = it.advertise("/ku_scan/camera/mask_image", 1);
    raw_scanned_points_xyz_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ku_scan/cloud/xyzrgba/raw_scan", 1);

    printf("scan_node ready...\n");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return (0);
}