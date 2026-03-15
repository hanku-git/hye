#include <thread>
#include "module/template_matching.h"
#include "module/KUPCLMath.h"
#include "module/image2cloud_processing.h"
#include "state_estimation/contact_state_estimator.h"
#include "rclcpp/rclcpp.hpp"
// #include <ros/node_handle.h>

#include <hanyang_matching_msgs/srv/capture.hpp>
#include <hanyang_matching_msgs/srv/capture_assistant_suggest_settings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
// #include <octomap_msgs/octomap.h>
// #include <octomap_msgs/conversions.h>
#include "hanyang_matching_msgs/srv/grasping_pose.hpp"
#include "hanyang_matching_msgs/srv/do_template_matching.hpp"
#include "hanyang_matching_msgs/srv/do_pose_estimation.hpp"
#include "hanyang_matching_msgs/msg/detected_pose.hpp"
#include "hanyang_matching_msgs/msg/resultmrcnn.hpp"
#include "hanyang_matching_msgs/msg/mask_cloud.hpp"
#include "hanyang_matching_msgs/msg/robot_state.hpp"
#include "hanyang_matching_msgs/srv/do_pcl_test.hpp"

#include "hanyang_matching_msgs/srv/grp_pose.hpp"

#include "hanyang_matching_msgs/msg/do_template_matching_msg.hpp"
#include "hanyang_matching_msgs/msg/matching_result_msg.hpp"

#include "hanyang_matching_msgs/msg/bin_picking_results.h"

#include <mutex>

std::mutex mtx;

// // ROS log macro
// const std::string log_prefix = std::string("MATCHING_NODE");
// #define _DEBUG
// #define EMPTY_INFO(loger, ...) loger;
// #ifdef _DEBUG
// #define ROS_LOG_INFO(...)  RCLCPP_INFO (rclcpp::get_logger(log_prefix), __VA_ARGS__);
// #else
// #define ROS_LOG_INFO(...)  EMPTY_INFO (rclcpp::get_logger(log_prefix), __VA_ARGS__);
// #endif
// #define ROS_LOG_WARN(...)  RCLCPP_WARN (rclcpp::get_logger(log_prefix), __VA_ARGS__);
// #define ROS_LOG_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger(log_prefix), __VA_ARGS__);



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

bool is_image_ai_detection_ = false; // HD_llm


const rclcpp::Duration default_wait_duration{30,0};
constexpr auto ca_suggest_settings_service_name = "/zivid_camera/capture_assistant/suggest_settings";
static const std::string OPENCV_WINDOW = "Image window";
static int m_target_id = 0;
static std::string target_name = "none";

bool do_save_data = false;
bool do_save_mrcnn_learning_data = false;

cv_bridge::CvImagePtr img_ptr;
cv_bridge::CvImagePtr img_raw_ptr;
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scan(new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZRGBA>);

Eigen::Matrix4f m_T_bin_robot_camera_calibration_nominal;

bool skip_detection_mask = false;
bool is_mask_pixel_fixed = false;
static int mask_pixel_x = 0;
static int mask_pixel_y = 0;
static int mask_pixel_scale_x = 0;
static int mask_pixel_scale_y = 0;

static int learning_data_idx = 0;

size_t m_detected_mask_num = 0;


bool do_matching_process_topic = false;
size_t matching_sampling_num = 1;
bool matching_debug_mode = false;


std::vector<double> scan_position_q_;
std::vector<double> scan_position_x_;
bool is_scan_finished = false;

std::string save_path;

std::vector<double> m_hinge_pose_transformation;
bool is_open3d_processing_finished_ = false;



IMAGE2CLOUDPROCESSING m_image2cloud_proc;
TEMPLATE_MATCHING m_template_matching;

KUPCLMATH m_math;
PCLPROCESSING m_pcl_proc;
CContactStateEstimator m_state_eval;

rclcpp::Publisher<hanyang_matching_msgs::msg::MatchingResultMsg>::SharedPtr scanning_and_detection_results_publisher_;
rclcpp::Publisher<hanyang_matching_msgs::msg::MatchingResultMsg>::SharedPtr matching_pose_results_publisher_ ;

//// Visualizer
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr aligned_points_xyz_publisher_ ;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_matching_results_publisher_ ;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pre_matching_results_publisher_ ;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pre_matching_results_no_cad_publisher_ ;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_segmentation_results_publisher_ ;

// rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_simulation_results_publisher_ ;
// rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr map_simulation_only_bin_publisher_ ;
// rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr map_simulation_object_publisher_ ;
// rclcpp::Publisher<geometry_msgs::PoseStamped>::SharedPtr object_frame_results_publisher_ ;

image_transport::Publisher selected_mask_image_publisher_;
image_transport::Publisher raw_iamge_with_selected_mask_publisher_;

const int UR5CB = 7001;
const int UR10e = 7002;


std::string m_package_path = "";
std::string m_home_folder_path = "";
std::string USER_NAME_ = "";

using json = nlohmann::json;



///////////////////////
//// Thread 전역변수 ////
///////////////////////
bool m_is_pose_assigned = false;
std::shared_ptr<bool> ptr_is_pose_assigned = std::make_shared<bool>(false);
size_t m_trial_count = 0;

std::vector<double> m_grasp_pose;
std::vector<std::vector<double> > m_grasp_sub_pose_set;
std::vector<double> m_zig_pose;
std::vector<double> m_object_bin_to_object_teach_pose_to_be_saved_from_initial_teaching;
std::vector<double> m_object_bin_to_base_pose_to_be_saved_from_initial_teaching;


Eigen::Matrix4f m_T_O2GP_final;


double m_matching_accuracy = 0.0;
bool m_is_feasible_pose = false;

double m_approach_distance = 0.0;
bool m_is_grasping_pose_flipped = false;

uint16_t m_gripper_open_length = 0;
uint16_t m_gripper_close_length = 0;
uint16_t m_gripper_tip_index = 0;

double m_matching_accuracy_limit = 0.0;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud_rviz_pose_results(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud_rviz_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_cloud_rviz_pre_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_scan_base_aligned_with_bin_collision_check(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_CAD_for_quick_align(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr m_cloud_CAD_quick_align_with_est_normal(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

// MASK_DATA m_mask_data;
CTARGET_OBJECT_DATA m_object_data_init;

size_t m_idx_target_mask_in = 0;
size_t m_idx_target_mask_process = 0;
size_t m_final_thread_id = 0;
std::vector<size_t> m_pre_selected_idx_set;
std::vector<std::vector<bool>> m_thread_is_matching_success;
std::vector<std::vector<size_t>> m_thread_pre_selected_mask_idx;
std::vector<std::vector<double>> m_thread_matching_accuracy;



std::vector<double> createDomain(const double start, const double end, const double step){
    std::vector<double> domain;
    for(double i=start; i<end; i += step){
        domain.push_back(i);
    }
    return domain;
}

///////////////////////
//// Thread 전역변수 ////
///////////////////////



void thread_func(size_t thread_id, size_t thread_num) {

    //// A) Index allocation
    mtx.lock();
    std::vector<size_t> mask_confident_idx_set_in;
    for (size_t i = 0; i < m_object_data_init.mask_data.mask_confident_idx_set.size(); i++) {
        if(i % thread_num + 1 != thread_id) { // 스레드 번호로 나눈 나머지에 1을 더한 값이 스레드 번호와 같은 경우만 처리
            continue;
        }
        mask_confident_idx_set_in.push_back(m_object_data_init.mask_data.mask_confident_idx_set[i]);
    }
    mtx.unlock();

    //// B) Cal. grasping pose
    mtx.lock();
    std::vector<size_t> pre_selected_idx_set;
    std::vector<bool> is_each_mask_matching_success;
    std::vector<double> each_mask_matching_accuracy;
    std::vector<CTARGET_OBJECT_DATA> set_object_data_in(mask_confident_idx_set_in.size());
    mtx.unlock();
    for (size_t i = 0; i < mask_confident_idx_set_in.size(); i++) {
        mtx.lock();
        bool l_is_pose_assigned_1 = m_is_pose_assigned;
        *ptr_is_pose_assigned = m_is_pose_assigned;
        mtx.unlock();
        if(l_is_pose_assigned_1) {
            break;
        }

        mtx.lock();
        m_template_matching.initializeTargetObjectData(set_object_data_in[i]);
        set_object_data_in[i] = m_object_data_init;

        std::cout << "&set_object_data_in[" << i  << "]: " << &set_object_data_in[i] << " [Thread #" << thread_id << "]\n";
        std::cout << "&m_object_data_init: " << &m_object_data_init << " [Thread #" << thread_id << "]\n";

        //// 초기 측정 데이터 업로드
        //// NOTICE: 서로 다른 스레드에서 포인트 클라우드의 포인터 자료형을 사용하면 서로 메모리가 공유되므로, 스레드 내부에서 포인트 클라우드를 변경하지 않도록 주의해서 사용
        *set_object_data_in[i].cloud_scan_base_aligned_with_bin_collision_check_process = *m_cloud_scan_base_aligned_with_bin_collision_check;
        set_object_data_in[i].cloud_matching_results->clear();
        set_object_data_in[i].cloud_pre_matching_results->clear();
        // *set_object_data_in[i].cloud_CAD_for_quick_align = *m_cloud_CAD_for_quick_align;
        // *set_object_data_in[i].cloud_CAD_quick_align_with_est_normal = *m_cloud_CAD_quick_align_with_est_normal;
        // set_object_data_in[i].mask_data = m_mask_data;
        //// 초기 측정 데이터 업로드
        mtx.unlock();


        ROS_LOG_WARN("(----- Multi thread id: #%zu *****)", thread_id);
        ROS_LOG_WARN("Trial #%zu (----- Multi thread id: #%zu *****)", i + 1, thread_id);
        ROS_LOG_WARN("(----- Multi thread id: #%zu *****)", thread_id);

        // mtx.lock();
        // object_data->multi_thread_test_num++;
        // mtx.unlock();

        int n_method = 1; // 1: ICP(기존 데모), 2: GICP
        bool b_process = false;
        switch(n_method) {
            case 1: {
                b_process = m_template_matching.icp_matchingSingleProcessVer3(ptr_is_pose_assigned, mask_confident_idx_set_in, pre_selected_idx_set, thread_id, set_object_data_in[i], false);
                break;
            }
            case 2: {
                b_process = m_template_matching.gicp_matchingSingleProcessVer1(ptr_is_pose_assigned, mask_confident_idx_set_in, pre_selected_idx_set, thread_id, set_object_data_in[i], false);
                break;
            }
            default: {
                break;
            }

        }
        

        
        mtx.lock();
        bool l_is_pose_assigned_2 = m_is_pose_assigned;
        *ptr_is_pose_assigned = m_is_pose_assigned;
        mtx.unlock();
        if(l_is_pose_assigned_2) {
            break;
        }


        is_each_mask_matching_success.push_back(set_object_data_in[i].is_matching_success);
        each_mask_matching_accuracy.push_back(set_object_data_in[i].matching_accuracy);

        if(set_object_data_in[i].is_matching_success) {
            ROS_LOG_ERROR("[Process Finished] Mask index #%zu - Matching Success! [Thread: %zu]", pre_selected_idx_set.back()+1, thread_id);
        } else {
            ROS_LOG_ERROR("[Process Finished] Mask index #%zu - Matching Failure, SKIP! [Thread: %zu]", pre_selected_idx_set.back()+1, thread_id);
        }
        
        if(b_process) {
            mtx.lock();
            bool l_is_pose_assigned_3 = m_is_pose_assigned;
            *ptr_is_pose_assigned = m_is_pose_assigned;
            mtx.unlock();
            
            if(!l_is_pose_assigned_3) {
                ROS_LOG_WARN("[Thread %zu] Pose assigned!!!", thread_id);
                ROS_LOG_WARN("[Thread %zu] Pose assigned!!!", thread_id);
                ROS_LOG_WARN("[Thread %zu] Pose assigned!!!", thread_id);
                ////////////////////////////////////////////////
                //// update
                mtx.lock();
                m_grasp_pose = set_object_data_in[i].grasping_pose;
                m_grasp_sub_pose_set.push_back(set_object_data_in[i].grasping_sub_pose);
                m_zig_pose = set_object_data_in[i].zig_pose;
                m_object_bin_to_object_teach_pose_to_be_saved_from_initial_teaching = set_object_data_in[i].object_bin_to_object_teach_pose_to_be_saved_from_initial_teaching;
                m_object_bin_to_base_pose_to_be_saved_from_initial_teaching = set_object_data_in[i].object_bin_to_base_pose_to_be_saved_from_initial_teaching;
                m_T_O2GP_final = set_object_data_in[i].T_O2GP_final;

                m_matching_accuracy = set_object_data_in[i].matching_accuracy;
                ROS_LOG_WARN("[Thread %zu] Pose assigned!!!", thread_id);
                
                pcl::copyPointCloud(*set_object_data_in[i].cloud_final_results_for_collision_check, *m_cloud_rviz_pose_results);

                m_is_pose_assigned = true;
                *ptr_is_pose_assigned = m_is_pose_assigned;
                ROS_LOG_WARN("[Thread %zu] Pose assigned!!!", thread_id);


                //// NOTICE: Thread 함수에서 object_data 포인터를 사용하지 말 것. --> 데이터 꼬임
                //// NOTICE: 웬만하면 전역변수와 mutex로 처리해야 함.
                //// update
                m_is_feasible_pose = set_object_data_in[i].is_feasible_pose;
                ROS_LOG_WARN("[Thread %zu] Pose assigned!!!", thread_id);
                pcl::copyPointCloud(*set_object_data_in[i].cloud_matching_results, *m_cloud_rviz_matching_results);
                pcl::copyPointCloud(*set_object_data_in[i].cloud_pre_matching_results, *m_cloud_rviz_pre_matching_results);
                m_idx_target_mask_in = set_object_data_in[i].mask_data.idx_target_mask_in;
                m_idx_target_mask_process = set_object_data_in[i].mask_data.idx_target_mask_process;
                m_approach_distance = set_object_data_in[i].approach_distance;
                m_is_grasping_pose_flipped = set_object_data_in[i].is_grasping_pose_flipped;
                m_gripper_open_length = set_object_data_in[i].gripper_open_length;
                m_gripper_close_length = set_object_data_in[i].gripper_close_length;
                m_gripper_tip_index = set_object_data_in[i].gripper_tip_index;
                m_matching_accuracy_limit = set_object_data_in[i].matching_accuracy_limit;
                m_final_thread_id = thread_id;

                ROS_LOG_WARN("[m_final_thread_id: %zu] Pose assigned!!!", thread_id);

                mtx.unlock();
                ////////////////////////////////////////////////
            }
            break;
        }
    }      

    for (size_t j = 0; j < pre_selected_idx_set.size(); j++) {
        ROS_LOG_ERROR("[(final)pre_selected_idx_set] #%zu [Thread #%zu]", pre_selected_idx_set[j] + 1,  thread_id);
    }

    for (size_t j = 0; j < is_each_mask_matching_success.size(); j++) {

        if(is_each_mask_matching_success[j]) {
            ROS_LOG_ERROR("Is matching ... [Thread #%zu] - Matching Success", thread_id);
        } else {
            ROS_LOG_ERROR("Is matching ... [Thread #%zu] - Matching Failure", thread_id);
        }
    }

    mtx.lock();
    m_pre_selected_idx_set.insert( m_pre_selected_idx_set.end(), pre_selected_idx_set.begin(), pre_selected_idx_set.end() );
    mtx.unlock();

    mtx.lock();
    m_thread_pre_selected_mask_idx[thread_id - 1] = pre_selected_idx_set;
    mtx.unlock();

    mtx.lock();
    m_thread_is_matching_success[thread_id - 1] = is_each_mask_matching_success;
    mtx.unlock();

    mtx.lock();
    m_thread_matching_accuracy[thread_id - 1] = each_mask_matching_accuracy;
    mtx.unlock();

}


bool do_template_initialize(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    try
    {
        int target_id = request->target_id;
        int obj_num = 0;
        obj_num = 41; // 드럼통, 기존:30

        int mask_id_tmp = target_id-obj_num;
        bool do_overwrite_JSON = request->do_overwrite_json;


        if(!request->do_set_only_parameters) {
            ROS_LOG_WARN("do_template_initialize");
            m_template_matching.setTargetObjectTemplate(mask_id_tmp - 1, do_overwrite_JSON);
        } else {
            ROS_LOG_WARN("set_template_parameters");
            m_template_matching.setTemplateParameters();
        }

        std::vector<double> robot_dh_vec = request->robot_dh_parameters; // [m, deg]
        std::vector<double> robot_tcp_default = request->robot_tcp_default; // [m, deg]
        std::vector<double> robot_tcp = request->robot_tcp; // [m, deg]

        ROS_LOG_WARN("robot_dh_vec.size: %zu", robot_dh_vec.size());
        ROS_LOG_WARN("robot_dh_vec.size: %zu", robot_tcp_default.size());
        ROS_LOG_WARN("robot_dh_vec.size: %zu", robot_tcp.size());
        m_template_matching.setTargetObjectRobotParameters(mask_id_tmp - 1, robot_dh_vec, robot_tcp_default, robot_tcp);
        
        printf("\n\n\nDH: ");
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 4; j++) {
                printf("%0.6f ", robot_dh_vec[i * 4 + j]);
            }
            printf("\n");
        }
        ROS_LOG_INFO("\nRobot TCP default[m, deg]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f", robot_tcp_default[0], robot_tcp_default[1], robot_tcp_default[2], robot_tcp_default[3], robot_tcp_default[4], robot_tcp_default[5]);
        ROS_LOG_INFO("\nRobot TCP[m, deg]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f", robot_tcp[0], robot_tcp[1], robot_tcp[2], robot_tcp[3], robot_tcp[4], robot_tcp[5]);

        return true;
    }
    catch(std::string &e)
    {
        return false;
    }
}

void matching_process_multi_threaded_bin_picking()
{
    pcl::console::TicToc time;
    time.tic();

    hanyang_matching_msgs::msg::MatchingResultMsg matching_result_msg;

    //////////////////////
    //// 전역변수 초기화 ////
    //////////////////////
    m_is_pose_assigned = false;
    *ptr_is_pose_assigned = m_is_pose_assigned;

    m_trial_count = 0;

    m_grasp_pose.clear();
    m_grasp_sub_pose_set.clear();
    m_zig_pose.clear();
    m_T_O2GP_final = Eigen::Matrix4f::Identity();
    m_pre_selected_idx_set.clear();
    m_thread_pre_selected_mask_idx. clear();
    m_thread_is_matching_success.clear();
    m_thread_matching_accuracy.clear();
    m_object_bin_to_object_teach_pose_to_be_saved_from_initial_teaching.clear();
    m_object_bin_to_base_pose_to_be_saved_from_initial_teaching.clear();
    m_matching_accuracy = -1.0;
    m_is_feasible_pose = false;
    m_approach_distance = 0.0;
    m_is_grasping_pose_flipped = false;
    m_gripper_open_length = 0;
    m_gripper_close_length = 0;
    m_gripper_tip_index = 0;
    m_matching_accuracy_limit = 0.0;

    m_cloud_rviz_pose_results->clear();
    m_cloud_rviz_matching_results->clear();
    m_cloud_rviz_pre_matching_results->clear();
    m_cloud_scan_base_aligned_with_bin_collision_check->clear();
    m_cloud_CAD_for_quick_align->clear();
    m_cloud_CAD_quick_align_with_est_normal->clear();
    //////////////////////
    //// 전역변수 초기화 ////
    //////////////////////


    bool do_scan_sampling = false;
    size_t sampling_num = matching_sampling_num;
    std::vector<double> grasp_pose;
    std::vector<std::vector<double>> grasp_sub_pose_set;
    std::vector<double> zig_pose;
    double matching_accuracy = 0.0;
    double approach_distance = 0.0;

    //// 인식된 Mask가 없는 경우
    if(m_detected_mask_num == 0) {
        matching_result_msg.is_pose = false;
        matching_result_msg.matching_accuracy = 0.0;
        matching_result_msg.approach_distance = 0.0;
        matching_result_msg.detected_mask_num = 0;
        // matching_result_msg.gripper_open_length = object_data->gripper_open_length;
        // matching_result_msg.pose = grasp_pose;
        printf("No Mask detected (matching_process_multi_threaded_bin_picking)!!!\n");
        is_scan_finished = false;
        // return false;
    }

    // if (is_scan_finished || matching_debug_mode)
    if (is_scan_finished) {
        if (m_target_id != -1) {
            ROS_LOG_INFO("template matching(bin picking): target object %d(%s)", m_target_id, target_name.c_str());
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_scan_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_mask_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::copyPointCloud(*cloud_scan, *cloud_scan_in);
            pcl::copyPointCloud(*cloud_input, *cloud_mask_in);
            // measured joint angle
            Eigen::VectorXf q_scan(6);
            for (int i = 0; i < 6; i++) {
                q_scan[i] = scan_position_q_[i];
            }

            if(is_image_ai_detection_) {
                if (m_target_id == 7) { // bolt
                    ROS_LOG_INFO(" bolt!!! (matching_process_hanyang_matching_topic)");
                    // matching_result_msg.is_pose = true;
                    // matching_result_msg.pose = grasp_pose; // [m], [deg]
                    // matching_result_msg.sub_pose = grasp_sub_pose_set[0];
                    // matching_result_msg.zig_pose = zig_pose; // [m], [deg]
                    // matching_result_msg.matching_accuracy = matching_accuracy;
                    // matching_result_msg.approach_distance = 0.001 * m_approach_distance; // mm to m
                    // matching_result_msg.is_grasping_pose_flipped = m_is_grasping_pose_flipped;
                    // matching_result_msg.gripper_open_length = m_gripper_open_length;
                    // matching_result_msg.gripper_close_length = m_gripper_close_length;
                    // matching_result_msg.matching_accuracy_limit = m_matching_accuracy_limit;
                    // matching_result_msg.detected_mask_num = m_detected_mask_num;
                    // matching_result_msg.gripper_tip_index = m_gripper_tip_index;
                    // return;
                }
            }


            ////////////////////////////////////////////////
            //// TODO: 멀티 스레드
            ROS_LOG_WARN("Multi-threaded process start!");
            // while(!m_is_pose_assigned) {
            //     m_trial_count++;
            //     if(m_trial_count > 10) {
            //         ROS_LOG_ERROR("Trial count limits! - Process Failed!");
            //         ROS_LOG_ERROR("Trial count limits! - Process Failed!");
            //         ROS_LOG_ERROR("Trial count limits! - Process Failed!");
            //         ROS_LOG_ERROR("Trial count limits! - Process Failed!");
            //         ROS_LOG_ERROR("Trial count limits! - Process Failed!");
            //         break;
            //     }
            //     ROS_LOG_ERROR("Total Process Trial #%zu", m_trial_count);
            //     ROS_LOG_ERROR("Total Process Trial #%zu", m_trial_count);
            //     ROS_LOG_ERROR("Total Process Trial #%zu", m_trial_count);
            //     ROS_LOG_ERROR("Total Process Trial #%zu", m_trial_count);
            //     ROS_LOG_ERROR("Total Process Trial #%zu", m_trial_count);
              
                m_template_matching.icp_matchingInitProcess(true, *cloud_mask_in, *cloud_scan_in, m_target_id, target_name, q_scan, do_scan_sampling, sampling_num, false); // (meas_filtering param,  plot)

                ///////////////////////////           
                //// 초기 측정 데이터 저장 ////
                m_template_matching.initializeTargetObjectData(m_object_data_init);
                m_object_data_init = *m_template_matching.m_ptr_template_list[m_target_id - 1];


                std::cout << "&m_template_matching.m_ptr_template_list[m_target_id - 1]: " << &m_template_matching.m_ptr_template_list[m_target_id - 1] << " [Initial process]\n";
                std::cout << "&m_template_matching.m_ptr_template_list[m_target_id - 1]: " << &m_template_matching.m_ptr_template_list[m_target_id - 1] << " [Initial process]\n";
                std::cout << "&m_template_matching.m_ptr_template_list[m_target_id - 1]: " << &m_template_matching.m_ptr_template_list[m_target_id - 1] << " [Initial process]\n";
                std::cout << "&m_object_data_init: " << &m_object_data_init << " [Initial process]\n";
                std::cout << "&m_object_data_init: " << &m_object_data_init << " [Initial process]\n";
                std::cout << "&m_object_data_init: " << &m_object_data_init << " [Initial process]\n";


                std::cout << "------------------------------------------------------\n";
                std::cout << "&m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_base_aligned_rgbn: " << &m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_base_aligned_rgbn << " [Initial process]\n";
                std::cout << "&m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_quick_align_with_est_normal: " << &m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_quick_align_with_est_normal << " [Initial process]\n";
                std::cout << "------------------------------------------------------\n";
                std::cout << "&m_object_data_init.cloud_base_aligned_rgbn: " << &m_object_data_init.cloud_base_aligned_rgbn << " [Initial process]\n";
                std::cout << "&m_object_data_init.cloud_CAD_quick_align_with_est_normal: " << &m_object_data_init.cloud_CAD_quick_align_with_est_normal << " [Initial process]\n";
                std::cout << "------------------------------------------------------\n";



                *m_cloud_scan_base_aligned_with_bin_collision_check = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_scan_base_aligned_with_bin_collision_check_process;
                // *m_cloud_CAD_for_quick_align = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_for_quick_align;
                // *m_cloud_CAD_quick_align_with_est_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_quick_align_with_est_normal;
                // m_mask_data = m_template_matching.m_ptr_template_list[m_target_id - 1]->mask_data;
                //// 초기 측정 데이터 저장 ////
                ///////////////////////////           

                size_t thread_num = 10;

                m_thread_pre_selected_mask_idx.resize(thread_num);
                m_thread_is_matching_success.resize(thread_num);
                m_thread_matching_accuracy.resize(thread_num);

                std::vector<std::thread> multi_thread;
                for (size_t i = 0; i < thread_num; i++) {
                    multi_thread.push_back(std::thread(thread_func, i + 1, thread_num));
                }
                                
                for (size_t i = 0; i < thread_num; i++) {
                    multi_thread[i].join();
                }
            // }
            ROS_LOG_WARN("Multi-threaded process end!");


            CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[m_target_id - 1];
            ////////////////////////////////////////////////
            //// update
            grasp_pose = m_grasp_pose;
            grasp_sub_pose_set = m_grasp_sub_pose_set;
            zig_pose = m_zig_pose;
            matching_accuracy = m_matching_accuracy;
            object_data->T_O2GP_final = m_T_O2GP_final; // for zig pose initial teaching
            object_data->object_bin_to_object_teach_pose_to_be_saved_from_initial_teaching = m_object_bin_to_object_teach_pose_to_be_saved_from_initial_teaching; // for zig pose initial teaching
            object_data->object_bin_to_base_pose_to_be_saved_from_initial_teaching = m_object_bin_to_base_pose_to_be_saved_from_initial_teaching; // for zig pose initial teaching
            ////////////////////////////////////////////////



            ROS_LOG_WARN("object_data->multi_thread_test_num: %i", object_data->multi_thread_test_num);
            ROS_LOG_WARN("m_pre_selected_idx_set: %i", m_pre_selected_idx_set.size());
            for (int i = 0; i < m_pre_selected_idx_set.size(); i++) {
                ROS_LOG_WARN("(skipped) pre-selected mask idx: %i", m_pre_selected_idx_set[i] + 1);
            }

            ROS_LOG_WARN("***** [m_final_thread_id: %zu] *****", m_final_thread_id);

            for (size_t i = 0; i < thread_num; i++) {
                for (size_t j = 0; j < m_thread_is_matching_success[i].size(); j++) {
                    if(m_thread_is_matching_success[i][j]) {
                        ROS_LOG_ERROR("[Thread #%zu] mask #%zu - Matching Success (accu.: %0.2f)", i + 1, m_thread_pre_selected_mask_idx[i][j] + 1, m_thread_matching_accuracy[i][j]);
                    } else {
                        ROS_LOG_ERROR("[Thread #%zu] mask #%zu - Matching Failure (accu.: %0.2f)", i + 1, m_thread_pre_selected_mask_idx[i][j] + 1, m_thread_matching_accuracy[i][j]);
                    }
                }
            }



            ROS_LOG_INFO("here 4!");
            ROS_LOG_WARN("Final-selected mask idx: %i", m_idx_target_mask_process + 1);
            if(m_is_feasible_pose) {
                ROS_LOG_WARN("[%s] Feasible pose!!!", __func__);
                ROS_LOG_INFO("[%s] Grasping pose (%s): %f %f %f %f %f %f\n", __func__, target_name.c_str(), grasp_pose[0], grasp_pose[1], grasp_pose[2], grasp_pose[3], grasp_pose[4], grasp_pose[5]);


                if(object_data->is_zig_pose_initial_teaching_process) {
                    Eigen::Matrix4f T_O2GP_final = object_data->T_O2GP_final;
                    ROS_LOG_WARN("*************************************************");
                    ROS_LOG_WARN("********** object_data->T_O2GP_final ************");
                    ROS_LOG_WARN("|%0.4f %0.4f %0.4f %0.1f|", T_O2GP_final(0, 0), T_O2GP_final(0, 1), T_O2GP_final(0, 2), T_O2GP_final(0, 3));
                    ROS_LOG_WARN("|%0.4f %0.4f %0.4f %0.1f|", T_O2GP_final(1, 0), T_O2GP_final(1, 1), T_O2GP_final(1, 2), T_O2GP_final(1, 3));
                    ROS_LOG_WARN("|%0.4f %0.4f %0.4f %0.1f|", T_O2GP_final(2, 0), T_O2GP_final(2, 1), T_O2GP_final(2, 2), T_O2GP_final(2, 3));
                    ROS_LOG_WARN("|%0.4f %0.4f %0.4f %0.1f|", T_O2GP_final(3, 0), T_O2GP_final(3, 1), T_O2GP_final(3, 2), T_O2GP_final(3, 3));
                    ROS_LOG_WARN("*************************************************");
        
                    std::vector<double> tmp11 = object_data->object_bin_to_object_teach_pose_to_be_saved_from_initial_teaching;
                    ROS_LOG_WARN("object_bin_to_object_teach_pose_to_be_saved_from_initial_teaching: %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f\n", tmp11[0], tmp11[1], tmp11[2], tmp11[3], tmp11[4], tmp11[5]);
                    ROS_LOG_WARN("*************************************************");

                } else {
                    ROS_LOG_WARN("*****************************************");
                    ROS_LOG_WARN("\n*** [GRASPING POSE]: %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f", grasp_pose[0], grasp_pose[1], grasp_pose[2], grasp_pose[3], grasp_pose[4], grasp_pose[5]);
                    ROS_LOG_WARN("*****************************************");
                    ROS_LOG_WARN("\n*** [ZIG POSE]: %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f", zig_pose[0], zig_pose[1], zig_pose[2], zig_pose[3], zig_pose[4], zig_pose[5]);
                    ROS_LOG_WARN("*****************************************");
                }

            } else {
                ROS_LOG_WARN("[%s] Infeasible pose!!!", __func__);
            }

            /////////////////////////////////////////////////
            //// rviz visualizer
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_pre_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_final_matching_pose_results(new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);

            if (m_is_feasible_pose) {
                ROS_LOG_INFO("[%s] is_feasible_pose (do_matching_bin_picking) ---- true", __func__);
            } else {
                ROS_LOG_INFO("[%s] is_feasible_pose(do_matching_bin_picking) ---- false", __func__);
            }
            pcl::copyPointCloud(*object_data->cloud_CAD, *cloud_tgt_vis);
            // pcl::PointXYZ minPt, maxPt;
            // pcl::getMinMax3D(*object_data->cloud_CAD, minPt, maxPt);
            pcl::PointXYZ minPt = object_data->cad_min_pt;
            pcl::PointXYZ maxPt = object_data->cad_max_pt;
            for (size_t i = 0; i < cloud_tgt_vis->size(); i++) {
                if (cloud_tgt_vis->points[i].z < maxPt.z - 3.0) {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 0;
                    cloud_tgt_vis->points[i].b = 0;
                } else {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 255; // 가장 윗면 녹색 점 표시
                    cloud_tgt_vis->points[i].b = 0;
                }
            }

            //// Matching results update
            pcl::copyPointCloud(*m_cloud_rviz_matching_results, *cloud_rviz_matching_results);
            //// Pre matching results
            pcl::copyPointCloud(*m_cloud_rviz_pre_matching_results, *cloud_rviz_pre_matching_results);
            //// rviz topic results
            // 1-1) Final matching results
            // ROS_LOG_INFO("Publishing cloud/xyzrgba");

            IMAGE2CLOUDPROCESSING m_image2cloud_proc;
            sensor_msgs::msg::PointCloud2 cloud_msg;
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg);
            cloud_msg.header.frame_id = "ku_cloud_frame";
            aligned_points_xyz_publisher_->publish(cloud_msg);

            // 1-2) CAD cloud - black
            for (size_t i = 0; i < cloud_tgt_vis->size(); i++) {
                if (cloud_tgt_vis->points[i].z >= maxPt.z - 3.0) {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 0;
                    cloud_tgt_vis->points[i].b = 0;
                }
            }
            sensor_msgs::msg::PointCloud2 cloud_msg_1;
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
            cloud_msg_1.header.frame_id = "ku_matching_frame";
            cloud_matching_results_publisher_->publish(cloud_msg_1);

            // 2) Pre matching results (w/ CAD)
            sensor_msgs::msg::PointCloud2 cloud_msg_2;
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_pre_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_2);
            cloud_msg_2.header.frame_id = "ku_pre_matching_frame";
            cloud_pre_matching_results_publisher_->publish(cloud_msg_2);
            /////////////////////////////////////////////////

            // 3) Pre matching results (w/o CAD)
            sensor_msgs::msg::PointCloud2 cloud_msg_3;
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_rviz_pre_matching_results;

            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            // m_math.printCloudMinMax(*cloud_merge_vis, "cloud_merge_vis m");
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_3);
            cloud_msg_3.header.frame_id = "ku_pre_matching_frame_no_cad";
            cloud_pre_matching_results_no_cad_publisher_->publish(cloud_msg_3);
            ///////////////////////////////////////////////

            // 4) Final Pose results with collision check
            //// Collision check final results
            pcl::copyPointCloud(*m_cloud_rviz_pose_results, *cloud_rviz_final_matching_pose_results);
            
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_rviz_final_matching_pose_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            sensor_msgs::msg::PointCloud2 cloud_msg_4;
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
            cloud_msg_4.header.frame_id = "ku_segmentation_frame";
            cloud_segmentation_results_publisher_->publish(cloud_msg_4);

            ///////////////////////////////////////////////

            ROS_LOG_INFO("[%s] matching_accuracy (%s): %f \n", __func__, target_name.c_str(), matching_accuracy);

            ////////////////////////////////////////////////////////////////
            ////////////////////// Visualizer //////////////////////////////
            ////////////////////////////////////////////////////////////////
            //// Mask raw index & new index
            //// Mask image publish
            cv::Mat img_scan_tmp = img_ptr->image; // raw scan image
            //// a) from Mask RCNN results (raw index)
            sensor_msgs::msg::RegionOfInterest mask_box;
            mask_box = object_data->mask_data.boxes[m_idx_target_mask_in];
            cv::Rect rect_raw(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);
            mask_box = object_data->mask_data.boxes[m_idx_target_mask_process];
            cv::Rect rect_new(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);

            //// b) Selected mask with color image
            cv::rectangle(img_scan_tmp, rect_raw, cv::Scalar(0, 0, 255), 10, 4, 0);
            cv::rectangle(img_scan_tmp, rect_new, cv::Scalar(0, 255, 0), 10, 4, 0);
            // cv::rectangle(img_scan_tmp, rect, cv::Scalar(0, 255, 0), cv::FILLED, 8, 0);	// 내부 채우기
            sensor_msgs::msg::Image::SharedPtr msg_img_with_selected_mask = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_scan_tmp).toImageMsg();
            raw_iamge_with_selected_mask_publisher_.publish(msg_img_with_selected_mask);
            ROS_LOG_INFO("[%s] Selected mask image published!", __func__);
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////

            if (m_is_feasible_pose) {
                ROS_LOG_INFO("[%s] Feasible pose! (matching_process_hanyang_matching_topic)", __func__);
                matching_result_msg.is_pose = true;
                matching_result_msg.pose = grasp_pose; // [m], [deg]
                matching_result_msg.sub_pose = grasp_sub_pose_set[0];
                matching_result_msg.zig_pose = zig_pose; // [m], [deg]
                matching_result_msg.matching_accuracy = matching_accuracy;
                matching_result_msg.approach_distance = 0.001 * m_approach_distance; // mm to m
                matching_result_msg.is_grasping_pose_flipped = m_is_grasping_pose_flipped;
                matching_result_msg.gripper_open_length = m_gripper_open_length;
                matching_result_msg.gripper_close_length = m_gripper_close_length;
                matching_result_msg.matching_accuracy_limit = m_matching_accuracy_limit;
                matching_result_msg.detected_mask_num = m_detected_mask_num;
                matching_result_msg.gripper_tip_index = m_gripper_tip_index;



                std::cout << "--- Response - Gripper open length: " << m_gripper_open_length << std::endl;
                std::cout << "--- Response - Gripper close length: " << m_gripper_close_length << std::endl;
                std::cout << "--- Response - Gripper tip_index: " << m_gripper_tip_index << std::endl;

            } else { // Pose infeasible --> 충돌 또는 충돌 회피 자세 없는 경우
                ROS_LOG_INFO("[%s] Infeasible pose! (matching_process_hanyang_matching_topic)", __func__);
                matching_result_msg.is_pose = false;
                matching_result_msg.matching_accuracy = 0.0;
                matching_result_msg.approach_distance = 0.0;
                // matching_result_msg.gripper_open_length = m_gripper_open_length;
                matching_result_msg.detected_mask_num = m_detected_mask_num;


            }
            is_scan_finished = false;
        } else {
            matching_result_msg.is_pose = false;
            matching_result_msg.matching_accuracy = 0.0;
            matching_result_msg.approach_distance = 0.0;
            matching_result_msg.detected_mask_num = 0;
            // matching_result_msg.gripper_open_length = m_gripper_open_length;
            // matching_result_msg.pose = grasp_pose;
            printf("Grasping pose pose cannot be estimated: 1!!! (matching_process_hanyang_matching_topic)\n");
            is_scan_finished = false;
        }
    } else {
        matching_result_msg.is_pose = false;
        matching_result_msg.matching_accuracy = 0.0;
        matching_result_msg.approach_distance = 0.0;
        matching_result_msg.detected_mask_num = 0;
        // matching_result_msg.gripper_open_length = m_gripper_open_length;
        // matching_result_msg.pose = grasp_pose;
        printf("Grasping pose cannot be estimated: 2!!! (matching_process_hanyang_matching_topic)\n");
        is_scan_finished = false;
    }
    
    //// Topic publish
    matching_pose_results_publisher_->publish(matching_result_msg);

    ROS_LOG_WARN("matching_process - process time: %0.3f [ms]", time.toc());

}

void matching_process_bin_picking()
{
    hanyang_matching_msgs::msg::MatchingResultMsg matching_result_msg;

    //// TODO: 별도의 downsampling이 필요하면 추후 사용
    // bool do_scan_sampling = msg.do_scan_sampling;
    bool do_scan_sampling = false;
    size_t sampling_num = matching_sampling_num;
    std::vector<double> grasp_pose;
    std::vector<std::vector<double> > grasp_sub_pose_set;
    std::vector<double> zig_pose;
    double matching_accuracy = 0.0;
    double approach_distance = 0.0;

    //// 인식된 Mask가 없는 경우
    if(m_detected_mask_num == 0) {
        matching_result_msg.is_pose = false;
        matching_result_msg.matching_accuracy = 0.0;
        matching_result_msg.approach_distance = 0.0;
        matching_result_msg.detected_mask_num = 0;
        // matching_result_msg.gripper_open_length = object_data->gripper_open_length;
        // matching_result_msg.pose = grasp_pose;
        printf("No Mask detected (do_matching_bin_picking)!!!\n");
        is_scan_finished = false;
        // return false;
    }

    // if (is_scan_finished || matching_debug_mode)
    if (is_scan_finished) {
        if (m_target_id != -1) {
            ROS_LOG_INFO("template matching(bin picking): target object %d(%s)", m_target_id, target_name.c_str());
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_scan_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_mask_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::copyPointCloud(*cloud_scan, *cloud_scan_in);
            pcl::copyPointCloud(*cloud_input, *cloud_mask_in);
            // measured joint angle
            Eigen::VectorXf q_scan(6);
            for (int i = 0; i < 6; i++) {
                q_scan[i] = scan_position_q_[i];
            }

            pcl::console::TicToc time;
            time.tic();
            // printf("scan position(q): %f %f %f %f %f %f\n", q_scan[0], q_scan[1], q_scan[2], q_scan[3], q_scan[4], q_scan[5]);
            m_template_matching.icp_matchingBinPicking(true, *cloud_mask_in, *cloud_scan_in, m_target_id, target_name, q_scan, grasp_pose, grasp_sub_pose_set, zig_pose, matching_accuracy, do_scan_sampling, sampling_num, false); // (meas_filtering param,  plot)
            ROS_LOG_INFO("here 4!");
            ROS_LOG_INFO("Grasping pose (%s): %f %f %f %f %f %f\n", target_name.c_str(), grasp_pose[0], grasp_pose[1], grasp_pose[2], grasp_pose[3], grasp_pose[4], grasp_pose[5]);
            std::cout << " icp_matchingBinPicking process time: " << time.toc() << " ms" << std::endl;

            /////////////////////////////////////////////////
            //// rviz visualizer
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_pre_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_final_matching_pose_results(new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);

            CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[m_target_id - 1];

            if (object_data->is_feasible_pose) {
                ROS_LOG_INFO("is_feasible_pose (do_matching_bin_picking) ---- true");
            } else {
                ROS_LOG_INFO("is_feasible_pose(do_matching_bin_picking) ---- false");
            }
            pcl::copyPointCloud(*object_data->cloud_CAD, *cloud_tgt_vis);
            // pcl::PointXYZ minPt, maxPt;
            // pcl::getMinMax3D(*object_data->cloud_CAD, minPt, maxPt);
            pcl::PointXYZ minPt = object_data->cad_min_pt;
            pcl::PointXYZ maxPt = object_data->cad_max_pt;
            for (size_t i = 0; i < cloud_tgt_vis->size(); i++) {
                if (cloud_tgt_vis->points[i].z < maxPt.z - 3.0) {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 0;
                    cloud_tgt_vis->points[i].b = 0;
                } else {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 255; // 가장 윗면 녹색 점 표시
                    cloud_tgt_vis->points[i].b = 0;
                }
            }
            //// Matching results update
            pcl::copyPointCloud(*object_data->cloud_matching_results, *cloud_rviz_matching_results);
            //// Pre matching results
            pcl::copyPointCloud(*object_data->cloud_pre_matching_results, *cloud_rviz_pre_matching_results);
            //// rviz topic results
            // 1-1) Final matching results
            ROS_LOG_INFO("Publishing cloud/xyzrgba");

            IMAGE2CLOUDPROCESSING m_image2cloud_proc;
            sensor_msgs::msg::PointCloud2 cloud_msg;
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg);
            cloud_msg.header.frame_id = "ku_cloud_frame";
            aligned_points_xyz_publisher_->publish(cloud_msg);

            // 1-2) CAD cloud - black
            for (size_t i = 0; i < cloud_tgt_vis->size(); i++) {
                if (cloud_tgt_vis->points[i].z >= maxPt.z - 3.0) {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 0;
                    cloud_tgt_vis->points[i].b = 0;
                }
            }
            sensor_msgs::msg::PointCloud2 cloud_msg_1;
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
            cloud_msg_1.header.frame_id = "ku_matching_frame";
            cloud_matching_results_publisher_->publish(cloud_msg_1);

            // 2) Pre matching results (w/ CAD)
            sensor_msgs::msg::PointCloud2 cloud_msg_2;
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_pre_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_2);
            cloud_msg_2.header.frame_id = "ku_pre_matching_frame";
            cloud_pre_matching_results_publisher_->publish(cloud_msg_2);
            /////////////////////////////////////////////////

            // 3) Pre matching results (w/o CAD)
            sensor_msgs::msg::PointCloud2 cloud_msg_3;
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_rviz_pre_matching_results;

            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            // m_math.printCloudMinMax(*cloud_merge_vis, "cloud_merge_vis m");
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_3);
            cloud_msg_3.header.frame_id = "ku_pre_matching_frame_no_cad";
            cloud_pre_matching_results_no_cad_publisher_->publish(cloud_msg_3);
            ///////////////////////////////////////////////

            // 4) Final Pose results with collision check
            //// Collision check final results
            pcl::copyPointCloud(*object_data->cloud_final_results_for_collision_check, *cloud_rviz_final_matching_pose_results);
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_rviz_final_matching_pose_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            sensor_msgs::msg::PointCloud2 cloud_msg_4;
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
            cloud_msg_4.header.frame_id = "ku_segmentation_frame";
            cloud_segmentation_results_publisher_->publish(cloud_msg_4);

            ///////////////////////////////////////////////

            ROS_LOG_INFO("matching_accuracy (%s): %f \n", target_name.c_str(), matching_accuracy);

            ////////////////////////////////////////////////////////////////
            ////////////////////// Visualizer //////////////////////////////
            ////////////////////////////////////////////////////////////////
            //// Mask raw index & new index
            //// Mask image publish
            cv::Mat img_scan_tmp = img_ptr->image; // raw scan image
            //// a) from Mask RCNN results (raw index)
            sensor_msgs::msg::RegionOfInterest mask_box;
            mask_box = object_data->mask_data.boxes[object_data->mask_data.idx_target_mask_in];
            cv::Rect rect_raw(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);
            mask_box = object_data->mask_data.boxes[object_data->mask_data.idx_target_mask_process];
            cv::Rect rect_new(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);

            //// b) Selected mask with color image
            cv::rectangle(img_scan_tmp, rect_raw, cv::Scalar(0, 0, 255), 10, 4, 0);
            cv::rectangle(img_scan_tmp, rect_new, cv::Scalar(0, 255, 0), 10, 4, 0);
            // cv::rectangle(img_scan_tmp, rect, cv::Scalar(0, 255, 0), cv::FILLED, 8, 0);	// 내부 채우기
            sensor_msgs::msg::Image::SharedPtr msg_img_with_selected_mask = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_scan_tmp).toImageMsg();
            raw_iamge_with_selected_mask_publisher_.publish(msg_img_with_selected_mask);
            ROS_LOG_INFO("Selected mask image published!");
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////

            if (object_data->is_feasible_pose) {
                ROS_LOG_INFO("Feasible pose! (matching_process_hanyang_matching_topic)");
                matching_result_msg.is_pose = true;
                matching_result_msg.pose = grasp_pose; // [m], [deg]
                matching_result_msg.sub_pose = grasp_sub_pose_set[0];
                matching_result_msg.zig_pose = zig_pose; // [m], [deg]
                matching_result_msg.matching_accuracy = matching_accuracy;
                matching_result_msg.approach_distance = 0.001 * object_data->approach_distance; // mm to m
                matching_result_msg.is_grasping_pose_flipped = object_data->is_grasping_pose_flipped;
                matching_result_msg.gripper_open_length = object_data->gripper_open_length;
                matching_result_msg.gripper_close_length = object_data->gripper_close_length;
                matching_result_msg.matching_accuracy_limit = object_data->matching_accuracy_limit;
                matching_result_msg.detected_mask_num = object_data->detected_mask_num;
                matching_result_msg.gripper_tip_index = object_data->gripper_tip_index;



                std::cout << "--- Response - Gripper open length: " << object_data->gripper_open_length << std::endl;
                std::cout << "--- Response - Gripper close length: " << object_data->gripper_close_length << std::endl;
                std::cout << "--- Response - Gripper tip_index: " << object_data->gripper_tip_index << std::endl;

                //// Topic publish
                //// 성공한 경우에만 publish
                matching_pose_results_publisher_->publish(matching_result_msg);
            } else { // Pose infeasible --> 충돌 또는 충돌 회피 자세 없는 경우
                ROS_LOG_INFO("Infeasible pose! (matching_process_hanyang_matching_topic)");
                matching_result_msg.is_pose = false;
                matching_result_msg.matching_accuracy = 0.0;
                matching_result_msg.approach_distance = 0.0;
                // matching_result_msg.gripper_open_length = object_data->gripper_open_length;
                matching_result_msg.detected_mask_num = object_data->detected_mask_num;
            }
            is_scan_finished = false;
        } else {
            matching_result_msg.is_pose = false;
            matching_result_msg.matching_accuracy = 0.0;
            matching_result_msg.approach_distance = 0.0;
            matching_result_msg.detected_mask_num = 0;
            // matching_result_msg.gripper_open_length = object_data->gripper_open_length;
            // matching_result_msg.pose = grasp_pose;
            printf("Grasping pose pose cannot be estimated: 1!!! (matching_process_hanyang_matching_topic)\n");
            is_scan_finished = false;
        }
    } else {
        matching_result_msg.is_pose = false;
        matching_result_msg.matching_accuracy = 0.0;
        matching_result_msg.approach_distance = 0.0;
        matching_result_msg.detected_mask_num = 0;
        // matching_result_msg.gripper_open_length = object_data->gripper_open_length;
        // matching_result_msg.pose = grasp_pose;
        printf("Grasping pose cannot be estimated: 2!!! (matching_process_hanyang_matching_topic)\n");
        is_scan_finished = false;
    }
    //// Topic publish
    //// 실패한 경우는 publish 하지 않음.
    // matching_pose_results_publisher_->publish(matching_result_msg);
}

void do_receive_cloud(const hanyang_matching_msgs::msg::MaskCloud &msg)
{
    std::string detection_mode = msg.detection_mode;
    ROS_LOG_INFO("*******************************************");
    ROS_LOG_INFO("*******************************************");
    ROS_LOG_INFO("*******************************************");
    ROS_LOG_INFO("do_receive_cloud(mask_detection_mode: %s)!", detection_mode.c_str());
    ROS_LOG_INFO("*******************************************");
    ROS_LOG_INFO("*******************************************");
    ROS_LOG_INFO("*******************************************");
    m_target_id = msg.class_ids;
    target_name = msg.class_name;
    ROS_LOG_WARN("target_id: %zu, target_name: %s", m_target_id, target_name.c_str());
    skip_detection_mask = msg.skip_detection_mask;
    is_mask_pixel_fixed = msg.is_mask_pixel_fixed;
    mask_pixel_x = msg.mask_pixel_x;
    mask_pixel_y = msg.mask_pixel_y;
    mask_pixel_scale_x = msg.mask_pixel_scale_x;
    mask_pixel_scale_y = msg.mask_pixel_scale_y;

    do_save_data = msg.do_save_data;
    do_save_mrcnn_learning_data = msg.do_save_mrcnn_learning_data;

    learning_data_idx = msg.learning_data_idx;

    pcl::fromROSMsg(msg.scan_cloud, *cloud_scan);

    ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////
    //// DH, TCP, JS position 할당
    std::vector<double> robot_dh_vec = msg.robot_dh_parameters; // [m, deg]
    std::vector<double> robot_tcp_default = msg.robot_tcp_default; // [m, deg]
    std::vector<double> robot_tcp = msg.robot_tcp; // [m, deg]

    if(target_name == "eye_in_hand") {
        ROS_LOG_WARN("[%s] EYE-IN-HAND CALIBRATION!", __func__);
        m_target_id = 1;
    }
    
    printf("\n\n\nDH: ");
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 4; j++) {
            printf("%0.6f ", robot_dh_vec[i * 4 + j]);
        }
        printf("\n");
    }
    ROS_LOG_INFO("\nRobot TCP default[m, deg]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f", robot_tcp_default[0], robot_tcp_default[1], robot_tcp_default[2], robot_tcp_default[3], robot_tcp_default[4], robot_tcp_default[5]);
    ROS_LOG_INFO("\nRobot TCP[m, deg]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f", robot_tcp[0], robot_tcp[1], robot_tcp[2], robot_tcp[3], robot_tcp[4], robot_tcp[5]);

    // std::vector<double> scan_position_q = request->r2cam_calibration_js_position; // [m], [deg]
    // ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);
    ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////
        
    if(skip_detection_mask)
    {
        ROS_LOG_INFO("*******************************************");
        ROS_LOG_INFO("ONLY SCAN - Skip Mask Processing!");
        ROS_LOG_INFO("*******************************************");
        ROS_LOG_INFO("cloud_scan size: %zu", cloud_scan->size());
        ROS_LOG_INFO("------------- [do_receive_cloud] -Node: template_matching/template_matching.cpp");

        // if(is_scan_finished)
        if(1)
        {
            ROS_LOG_WARN("SCAN DATA PROCESSING...");
            // measured joint angle
            Eigen::VectorXf q_scan(6);
            Eigen::VectorXf x_scan(6);
            for (int i = 0; i < 6; i++) {
                q_scan[i] = scan_position_q_[i];
                x_scan[i] = scan_position_x_[i];
            }
            ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", q_scan[0], q_scan[1], q_scan[2], q_scan[3], q_scan[4], q_scan[5]);
            ROS_LOG_WARN("Current CS pose [m, deg]: %0.5f, %0.5f, %0.5f, %0.2f, %0.2f, %0.2f", x_scan[0], x_scan[1], x_scan[2], x_scan[3], x_scan[4], x_scan[5]);

            ROS_LOG_WARN("HERE 4 - skip_detection_mask [m_target_id = %zu]", m_target_id);

            m_template_matching.m_ptr_template_list[m_target_id - 1]->robot_scan_JS_position = q_scan;
            m_template_matching.m_ptr_template_list[m_target_id - 1]->robot_scan_CS_pose = x_scan;

            CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[m_target_id - 1];


            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_scan_process_rgba(new pcl::PointCloud<pcl::PointXYZRGBA>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_scan_process_rgbn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::copyPointCloud(*cloud_scan, *cloud_scan_process_rgba);
            m_math.printCloudMinMax(*cloud_scan_process_rgba, "Raw data");

            pcl::IndicesPtr indices(new std::vector<int>);
            pcl::removeNaNFromPointCloud(*cloud_scan_process_rgba, *indices);
            pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
            extract.setInputCloud(cloud_scan_process_rgba);
            extract.setIndices(indices);
            extract.setNegative(false);
            extract.filter(*cloud_scan_process_rgba);
            m_math.removeDuplicatesCloud(*cloud_scan_process_rgba, *cloud_scan_process_rgba);

            m_math.printCloudMinMax(*cloud_scan_process_rgba, "Before view frustum");
            m_pcl_proc.filterCloudUsingViewFrustum(*cloud_scan_process_rgba, view_frustum_, *cloud_scan_process_rgba);
            m_math.printCloudMinMax(*cloud_scan_process_rgba, "After view frustum");

            ////////////////////////////////////////////////
            ////////////////////////////////////////////////
            ////////////////////////////////////////////////
            //// Alignment to base frame
            pcl::copyPointCloud(*cloud_scan_process_rgba, *cloud_scan_process_rgbn); // metric: [m]
            *object_data->cloud_measured_rgbn = *cloud_scan_process_rgbn; // metric: [m]

            if(target_name == "eye_in_hand") {
                ROS_LOG_WARN("[%s] EYE-IN-HAND CALIBRATION! - return!", __func__);
                return;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr process_cloud_base_aligned(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr process_cloud_base_aligned_rgbn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr process_cloud_sensor_frame(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr process_cloud_sensor_frame_rgbn(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            object_data->debug_mode = false;
            m_template_matching.doMeasuredCloudProcessing(object_data);
            *process_cloud_base_aligned = *object_data->cloud_base_aligned; // update
            ROS_LOG_INFO("---------------------------------------------------------");
            ROS_LOG_INFO("cloud_scan size: %zu", cloud_scan->size());
            ROS_LOG_INFO("cloud w.r.t base frame(No AI detection): %d", process_cloud_base_aligned->size());
            ROS_LOG_INFO("------------- [do_receive_cloud] -Node: template_matching/template_matching.cpp");

            // if(do_save_data) {
            if(1) {
                ROS_LOG_WARN("SCAN DATA PROCESSING RESULT SAVE...");
                save_path = msg.save_path;

                size_t scan_num = msg.scan_num;

                //// Timer log
                time_t timer;
                struct tm *t;
                timer = time(NULL);    // 1970년 1월 1일 0시 0분 0초부터 시작하여 현재까지의 초
                t = localtime(&timer); // 포맷팅을 위해 구조체에 넣기

                char ch[12]; // 1(sign) + 11(character)
                int ich;
                ich = snprintf(ch, 12, "%02dh-%02dm-%02ds", t->tm_hour, t->tm_min, t->tm_sec);
                std::string time_now(ch);
                std::stringstream ss;
                ss << save_path << "/scan_cloud_base_aligned_" << time_now << "_" << scan_num << ".ply";




                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_save(new pcl::PointCloud<pcl::PointXYZRGBA>);
                pcl::copyPointCloud(*process_cloud_base_aligned, *cloud_save);

                // //// Remove Nan points
                // pcl::IndicesPtr indices(new std::vector<int>);
                // pcl::removeNaNFromPointCloud(*cloud_save, *indices);
                // pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
                // extract.setInputCloud(cloud_save);
                // extract.setIndices(indices);
                // extract.setNegative(false);
                // extract.filter(*cloud_save);

                // filterCloudUsingViewFrustum(*cloud_save, view_frustum_, *cloud_save);

                //// Base aligned
                pcl::io::savePLYFile(ss.str(), *cloud_save, true); // binary
                ROS_LOG_WARN("[Base aligned] PLY CLOUD SAVED! - %s", ss.str().c_str());



                //// Sensor coordinate
                std::stringstream ss_raw;
                ss_raw << save_path << "/raw_scan_cloud_" << time_now << "_" << scan_num << ".ply";
                m_math.cloudScaling(*cloud_scan_process_rgba, 1); // m to mm
                pcl::io::savePLYFile(ss_raw.str(), *cloud_scan_process_rgba, true); // binary

            }


            ROS_LOG_WARN("SCAN DATA PROCESSING END");
        }



        return;
    }

    // Do Mask RCNN Processing
    if (m_target_id != -1)
    {
        pcl::fromROSMsg(msg.mask_cloud, *cloud_input);
        is_scan_finished = true;
        ROS_LOG_INFO("cloud_mask is received!");
        ROS_LOG_INFO("cloud_scan size: %zu", cloud_scan->size());
        ROS_LOG_INFO("cloud_mask size: %zu", cloud_input->size());
        ROS_LOG_INFO("------------- [do_receive_cloud] -Node: template_matching/template_matching.cpp");
    }
    else
    {
        is_scan_finished = false;
        ROS_LOG_INFO("cloud_mask is not received! (detection failure)");
        ROS_LOG_INFO("------------- [do_receive_cloud] -Node: template_matching/template_matching.cpp");
    }
    //////////////////////////////////////////////////////////////////////////////////////
    //// TODO: 230111 현재, score가 0.98보다 큰 mask 중에서 
    //// 1) base frame 기준의 z가 가장 높은 mask를 선택하고, 
    //// 2) 일정 수준 이내의 z를 갖는 mask 들 중에서는 가장 bin의 중심에 가까운 mask를 선택하도록 처리함.
    //// 추후에 특정 기준을 추가하거나, Mask RCNN을 안쓰는 경우에 대한 처리 추가
    // size_t idx_target_mask = 0;
    size_t sampling_num_quick_matching = 2;

    ////
    MASK_DATA mask_set_in;
    ROS_LOG_WARN("HERE 1");
    if(detection_mode == "mask_rcnn_detection")
    {
        // mask_set_in.target_id = m_target_id;
        // mask_set_in.class_cnt = msg.mrcnn_result.class_cnt;
        // mask_set_in.class_ids = msg.mrcnn_result.class_ids;
        // mask_set_in.scores = msg.mrcnn_result.scores;
        // mask_set_in.class_names = msg.mrcnn_result.class_names;
        // mask_set_in.mask_imgs = msg.mrcnn_result.masks;
        // mask_set_in.boxes = msg.mrcnn_result.boxes;
    }
    else if(detection_mode == "mask_sam_detection")
    {
        mask_set_in.target_id = m_target_id;
        mask_set_in.class_cnt = msg.sam_result.class_cnt;
        mask_set_in.class_ids = msg.sam_result.class_ids;
        mask_set_in.scores = msg.sam_result.scores;
        mask_set_in.class_names = msg.sam_result.class_names;
        mask_set_in.mask_imgs = msg.sam_result.masks;
        mask_set_in.boxes = msg.sam_result.boxes;
        mask_set_in.ext_nearby_ratio = msg.sam_result.nearby_ratio_set;
        ROS_LOG_WARN("HERE 2");

        if(is_image_ai_detection_) {
            if (m_target_id == 7 || m_target_id == 8){ // bolt or hinge
                mask_set_in.picking_poses = msg.sam_result.picking_poses;
            }
        }
    }
    m_detected_mask_num = mask_set_in.class_cnt;

    if(m_detected_mask_num == 0) {
        ROS_LOG_WARN("No Detected Mask!");
        ROS_LOG_INFO("No mask detected! (detection failure)");
        ROS_LOG_INFO("------------- [do_receive_cloud] -Node: template_matching/template_matching.cpp");
        return;
    }

    if(is_scan_finished)
    {
        ROS_LOG_WARN("[%s] HERE 3", __func__);
        // measured joint angle
        Eigen::VectorXf q_scan(6);
        Eigen::VectorXf x_scan(6);
        for (int i = 0; i < 6; i++) {
            q_scan[i] = scan_position_q_[i];
            x_scan[i] = scan_position_x_[i];
        }
        ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", q_scan[0], q_scan[1], q_scan[2], q_scan[3], q_scan[4], q_scan[5]);
        ROS_LOG_WARN("Current CS pose [m, deg]: %0.5f, %0.5f, %0.5f, %0.2f, %0.2f, %0.2f", x_scan[0], x_scan[1], x_scan[2], x_scan[3], x_scan[4], x_scan[5]);

        ROS_LOG_WARN("HERE 4 - is_scan_finished [mask_set_in.target_id = %zu]", mask_set_in.target_id);

        m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->robot_scan_JS_position = q_scan;
        m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->robot_scan_CS_pose = x_scan;
        // initialize
        ROS_LOG_WARN("HERE 4-1");
        m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.detection_mode = detection_mode; // Mask detection mode
        ROS_LOG_WARN("HERE 4-2");
        m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.cloud_mask_rgba_list.clear();
        ROS_LOG_WARN("HERE 4-3");
        m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.mask_cloud_base_aligned_list.clear();
        ROS_LOG_WARN("HERE 4-4");
        m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.mask_cloud_base_aligned_rgbn_list.clear();
        ROS_LOG_WARN("HERE 4-5");
        m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.xyz_centroid_list.clear();
        ROS_LOG_WARN("HERE 5");
        // Parameters setting
        mask_set_in.thre_score = 0.94;
        // mask_set_in.pixel_margin = 15;
        if(mask_set_in.target_id == 2) // hole_surface
        {
            // mask_set_in.pixel_margin = 200;
            mask_set_in.pixel_margin = 1;
            ROS_LOG_WARN("DRUM HOLE SURFACE - pixel margin: %zu", mask_set_in.pixel_margin);

        } else if(mask_set_in.target_id == 3) {
            // mask_set_in.pixel_margin = 200;
            mask_set_in.pixel_margin = 1;
            ROS_LOG_WARN("DRUM HOLE UNSCREWING - pixel margin: %zu", mask_set_in.pixel_margin);
        } else if(mask_set_in.target_id == 4) {
            // mask_set_in.pixel_margin = 200;
            mask_set_in.pixel_margin = 1;
            ROS_LOG_WARN("DRUM HOLE STAR - pixel margin: %zu", mask_set_in.pixel_margin);
        } else if(mask_set_in.target_id == 5) {
            mask_set_in.pixel_margin = 1;
            ROS_LOG_WARN("DRUM HOSE SCANNING - pixel margin: %zu", mask_set_in.pixel_margin);
        } else {
            mask_set_in.pixel_margin = 1;
            ROS_LOG_WARN("Pixel margin: %zu", mask_set_in.pixel_margin);
        }
        cout << "class_ids" << mask_set_in.class_ids[0] << endl;
        mask_set_in.target_detection_id = mask_set_in.class_ids[0];
        mask_set_in.sampling_num = msg.sampling_num;


        hanyang_matching_msgs::msg::MatchingResultMsg scanning_result_msg;
        
        //// Mask results processing
        if (m_detected_mask_num != 0)
        {
            //// View frustum of the 3D scanner 
            m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->view_frustum_3d_scanner = view_frustum_;
            ///////////////////////////////////////////////////////////////////////
            ///////////////////// Initial Mask Processing /////////////////////////
            ///////////////////////////////////////////////////////////////////////
            ROS_LOG_INFO("class index count: %d", mask_set_in.class_cnt);
            if(detection_mode == "mask_rcnn_detection")
            {
                m_template_matching.initialMaskProcessingVer1(mask_set_in, *cloud_scan); // For Mask RCNN
            }
            else if(detection_mode == "mask_sam_detection")
            {
                //// TODO: code here, 230808, SAM 적용
                m_template_matching.initialMaskProcessingVer2(mask_set_in, *cloud_scan); // For SAM
            }
            //// Mask information update
            m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.mask_confident_idx_set = mask_set_in.mask_confident_idx_set; 
            m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.boxes = mask_set_in.boxes;

            if(is_image_ai_detection_) {
                if (m_target_id == 7){ // bolt
                    m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.picking_poses = mask_set_in.picking_poses;
                } else if (m_target_id == 8){ // hinge
                    m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.picking_poses = mask_set_in.picking_poses;
                }
            }
            if(detection_mode == "mask_sam_detection") {
                m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.ext_nearby_ratio = mask_set_in.ext_nearby_ratio;
            }
            m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->is_base_frame_unknown = msg.is_base_frame_unknown;
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////

            ///////////////////////////////////////////////////////////////////////
            ///////////////////// First Mask Selection ////////////////////////////
            ///////////////////////////////////////////////////////////////////////
            m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.pre_selected_idx_set.clear(); // initialize 1
            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_mask_selected(new pcl::PointCloud<pcl::PointXYZRGBA>);
            //// Select initial Mask
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp1(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tmp2(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp3(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_tmp4(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

            if(is_image_ai_detection_) {
                if (m_target_id == 7) { // bolt
                    m_template_matching.selectMaskSAM(m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1], *cloud_tmp1, *cloud_tmp2, *cloud_tmp3, *cloud_tmp4);
                }
                else{
                    m_template_matching.selectMaskSAM(m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1], *cloud_tmp1, *cloud_tmp2, *cloud_tmp3, *cloud_tmp4);
                }
            } else {
                m_template_matching.selectMaskSAM(m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1], *cloud_tmp1, *cloud_tmp2, *cloud_tmp3, *cloud_tmp4);
            }
            // selected initial index
            mask_set_in.idx_target_mask = m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.idx_target_mask_process; // current index

            *cloud_mask_selected = m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.cloud_mask_rgba_list[mask_set_in.idx_target_mask]; // [m]

            // ROS_LOG_INFO("input cloud: %d", cloud_scan->size());
            ROS_LOG_INFO("cloud in mask(selected): %zu", cloud_mask_selected->size());
            *cloud_input = *cloud_mask_selected; // update

            ROS_LOG_INFO("*******************************************");
            ROS_LOG_INFO("Final mask index: #%zu", mask_set_in.idx_target_mask + 1);
            ROS_LOG_INFO("*******************************************");
            ROS_LOG_INFO("cloud_mask selected!");
            ROS_LOG_INFO("cloud_scan size: %zu", cloud_scan->size());
            ROS_LOG_INFO("cloud_mask size: %zu", cloud_input->size());
            ROS_LOG_INFO("------------- [do_receive_cloud] -Node: template_matching/template_matching.cpp");
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////////////////////
            if(is_image_ai_detection_) {
                if(m_target_id == 7 || m_target_id == 8) { // bolt(7), hinge(8)
                    std::vector<std::vector<double>> l_set_image_picking_position_xyq;
                    for (size_t i = 0; i < msg.sam_result.picking_poses.size(); i++) {
                        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "[Mask idx #%zu] x: %0.3f, y: %0.3f, theta: %0.3f, centroid: %0.3f", i , msg.sam_result.picking_poses[i].x, msg.sam_result.picking_poses[i].y, msg.sam_result.picking_poses[i].theta, msg.sam_result.nearby_ratio_set[i]);
                        std::vector<double> pose_set(3);
                        pose_set[0] = msg.sam_result.picking_poses[i].x;
                        pose_set[1] = msg.sam_result.picking_poses[i].y;
                        pose_set[2] = msg.sam_result.picking_poses[i].theta;
                        l_set_image_picking_position_xyq.push_back(pose_set);
                    }

                    RCLCPP_WARN(rclcpp::get_logger("SCAN_NODE"), "************************************");
                    for (size_t i = 0; i < l_set_image_picking_position_xyq.size(); i++) {
                        RCLCPP_INFO(rclcpp::get_logger("SCAN_NODE"), "[Mask idx #%zu] x: %0.3f, y: %0.3f, theta: %0.3f", i , l_set_image_picking_position_xyq[i][0], l_set_image_picking_position_xyq[i][1], l_set_image_picking_position_xyq[i][2]);
                    }

                    //// update
                    m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.set_image_picking_position_xyq = l_set_image_picking_position_xyq;
                    m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.mask_imgs = mask_set_in.mask_imgs;
                            
                    if(is_image_ai_detection_) {
                        if(m_target_id == 8) {
                            // ROS_LOG_WARN("Target custom transformation pose : %0.4f, %0.4f, %0.4f, %0.4f \n
                            //                                                   %0.4f, %0.4f, %0.4f, %0.4f \n, 
                            //                                                   %0.4f, %0.4f, %0.4f, %0.4f \n, 
                            //                                                   %0.4f, %0.4f, %0.4f, %0.4f \n", ... 
                            //     m_hinge_pose_transformation[0], m_hinge_pose_transformation[1], m_hinge_pose_transformation[2], m_hinge_pose_transformation[3], 
                            //     m_hinge_pose_transformation[4], m_hinge_pose_transformation[5], m_hinge_pose_transformation[6], m_hinge_pose_transformation[7],
                            //     m_hinge_pose_transformation[8], m_hinge_pose_transformation[9], m_hinge_pose_transformation[10], m_hinge_pose_transformation[11],
                            //     m_hinge_pose_transformation[12], m_hinge_pose_transformation[13], m_hinge_pose_transformation[14], m_hinge_pose_transformation[15]);

                            ROS_LOG_WARN("hinge_pose_transformation update");
                            ROS_LOG_WARN("hinge_pose_transformation update");
                            ROS_LOG_WARN("hinge_pose_transformation update");

                            // update
                            m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.hinge_pose_transformation = m_hinge_pose_transformation;
                            m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->target_id = m_target_id;
                        }
                    }
                }
            }


            //// Mask index update
            m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.pre_selected_idx_set.clear();  // initialize 2
            m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->mask_data.pre_selected_idx_set.push_back(mask_set_in.idx_target_mask); // selected index 


            ////////////////////////////////////////////////////////////////
            ////////////////////// Visualizer //////////////////////////////
            ////////////////////////////////////////////////////////////////
            //// Mask image publish
            cv::Mat img_scan_tmp = img_ptr->image; // raw scan image
            //// a) from Mask RCNN results
            sensor_msgs::msg::RegionOfInterest mask_box;
            mask_box = mask_set_in.boxes[mask_set_in.idx_target_mask];
            cv::Rect rect(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);
            // 관심영역 자르기 (Crop ROI).
            cv::Mat img_crop = img_scan_tmp(rect);
            // cv::imwrite("/home/irl/[scanDataHanyang]/input/scan_image_fixed_mask.jpg", img_crop);
            sensor_msgs::msg::Image::SharedPtr msg_crop = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_crop).toImageMsg();
            selected_mask_image_publisher_.publish(msg_crop);
            if (0)
            {
                cv::imshow(OPENCV_WINDOW, img_crop);
                cv::waitKey(2000);
                cv::destroyAllWindows(); // close the window
            }

            //// b) Selected mask with color image
            cv::rectangle(img_scan_tmp, rect, cv::Scalar(0, 0, 255), 10, 4, 0);
            // cv::rectangle(img_scan_tmp, rect, cv::Scalar(0, 255, 0), cv::FILLED, 8, 0);	// 내부 채우기
            sensor_msgs::msg::Image::SharedPtr msg_img_with_selected_mask = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_scan_tmp).toImageMsg();
            raw_iamge_with_selected_mask_publisher_.publish(msg_img_with_selected_mask);
            ROS_LOG_INFO("Selected mask image published!");
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////


            // 241017
            scanning_result_msg.is_scanning_and_detection_finished = true;

            if(is_image_ai_detection_) {
                if(m_target_id == 8) { // hinge의 경우만
                    scanning_result_msg.is_open3d_processing_finished = false;
                } else {
                    scanning_result_msg.is_open3d_processing_finished = true;
                }
            } else {
                scanning_result_msg.is_open3d_processing_finished = true;
            }
            scanning_result_msg.is_detection_success = true;

        } else {
            //// No Mask

            scanning_result_msg.is_scanning_and_detection_finished = true;

            if(is_image_ai_detection_) {
                if(m_target_id == 8) { // hinge의 경우만
                    scanning_result_msg.is_open3d_processing_finished = false;
                } else {
                    scanning_result_msg.is_open3d_processing_finished = true;
                }
            } else {
                scanning_result_msg.is_open3d_processing_finished = true;
            }

            // Success skip for key code target when no mask
            {
                std::string tgt_name = m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->target_object_name;
                if(tgt_name == "right_drum_key_code") {
                ROS_LOG_WARN("[matching_node] No mask but keycode target – treat as success skip");
                scanning_result_msg.is_detection_success = true;
                } else {
                scanning_result_msg.is_detection_success = false;
                }
            }
        }

        //// Topic publish
        scanning_and_detection_results_publisher_->publish(scanning_result_msg);

        // For key code target, scanning only: skip matching stage regardless of flags
        {
            std::string tgt_name = m_template_matching.m_ptr_template_list[mask_set_in.target_id - 1]->target_object_name;
            if(tgt_name == "right_drum_key_code") {
            ROS_LOG_WARN("[matching_node] Keycode target – skip matching stage after scan publish");
            return;
            }
        }



        //// Matching topic
        if(do_matching_process_topic && !msg.do_single_matching) 
        {
            ROS_LOG_INFO("--- Matching process (topic) start!");
            // matching_process_bin_picking();
            matching_process_multi_threaded_bin_picking();
            ROS_LOG_INFO("--- Matching process (topic) end!");


            if(do_matching_process_topic) {
                ROS_LOG_WARN("--- do_matching_process_topic - true!");
                
            }
            if(!msg.do_single_matching) {
                ROS_LOG_WARN("--- msg.do_single_matching - false!");
                
            }

        } else {
            ROS_LOG_WARN("--- Matching process (topic) - false!");
            if(!do_matching_process_topic) {
                ROS_LOG_WARN("--- do_matching_process_topic - false!");
                
            }
            if(msg.do_single_matching) {
                ROS_LOG_WARN("--- msg.do_single_matching - true!");
                
            }
        }
        
    }
    ///////////////////////////////////////////////////////////////////////////////

    if (do_save_data)
    {
        ROS_LOG_INFO("Data save // do_receive_cloud - template_matching.cpp");
        save_path = msg.save_path;
        // std::string detection_mode = msg.detection_mode;
        std::string time_now = msg.time_now;
        size_t scan_num = msg.scan_num;
        
        std::stringstream ss;
        ss << save_path << "/scan_cloud_" << detection_mode << "_" << time_now << "_" << scan_num << ".ply";
        pcl::io::savePLYFile(ss.str(), *cloud_input, true);


        if(0) {
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_query(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_process_results(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_process_xyz(new pcl::PointCloud<pcl::PointXYZ>);
            CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[m_target_id - 1];

            pcl::copyPointCloud(*cloud_scan, *cloud_query);

            Eigen::VectorXf q_scan(6);
            // measured joint angle
            q_scan[0] = 48.852;
            q_scan[1] = -73.240;
            q_scan[2] = -99.896;
            q_scan[3] = -86.879;
            q_scan[4] = 78.754;
            q_scan[5] = 47.872;

            m_template_matching.UIFuncDoMeasuredCloudFilteringUsingBinBoundingBox2(object_data, q_scan, *cloud_query, *cloud_process_results);


            ss.str("");
            ss << save_path << "/scan_cloud_bin_filtered_" << detection_mode << "_" << time_now << "_" << scan_num << ".ply";
            pcl::copyPointCloud(*cloud_process_results, *cloud_process_xyz);
            pcl::io::savePLYFile(ss.str(), *cloud_process_xyz, true);

            // Mask image
            sensor_msgs::msg::Image mask_img;
            mask_img = mask_set_in.mask_imgs[mask_set_in.idx_target_mask];
            cv_bridge::CvImagePtr mask_ptr;
            mask_ptr = cv_bridge::toCvCopy(mask_img, sensor_msgs::image_encodings::BGR8);
            ////////////////////////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////////////////////////
            //// File name
            ss.str("");
            ss << save_path << "/mask_image_" << detection_mode << "_" << time_now << "_" << scan_num << ".jpg";
            cv::imwrite(ss.str(), mask_ptr->image);

            if(do_save_mrcnn_learning_data)
            {
                //// Directory path
                struct stat statbuf;
                const char *mrcnn_path;
                const char *mrcnn_img_path;
                const char *mrcnn_mask_path;
                char save_mrcnn_path[255];
                char save_mrcnn_img_path[255];
                char save_mrcnn_mask_path[255];
                

                snprintf(save_mrcnn_path, 255, "/root/home/[mrcnn_learning_data]/%s", target_name.c_str());
                snprintf(save_mrcnn_img_path, 255, "/root/home/[mrcnn_learning_data]/%s/rgbs", target_name.c_str());
                snprintf(save_mrcnn_mask_path, 255, "/root/home/[mrcnn_learning_data]/%s/masks", target_name.c_str());
                mrcnn_path = save_mrcnn_path; // 경로
                mrcnn_img_path = save_mrcnn_img_path;
                mrcnn_mask_path = save_mrcnn_mask_path;
                if (stat(mrcnn_path, &statbuf) != -1)
                {
                    if (S_ISDIR(statbuf.st_mode))
                    { // 디렉토리인지 확인
                        std::cout << "mrcnn learning path exist!! ==> " << mrcnn_path << std::endl;
                    }
                }
                else
                {
                    std::cout << "No path ==> " << mrcnn_path << std::endl;
                    if (mkdir(mrcnn_path, 0776) == -1 && errno != EEXIST)
                    {
                        fprintf(stderr, "%s directory create fail: %s\n", strerror(errno));
                    }
                    else
                    {
                        mkdir(mrcnn_img_path, 0776);
                        mkdir(mrcnn_mask_path, 0776);
                        std::cout << "mrcnn learning folder has been created! ==> " << mrcnn_path << std::endl;
                    }
                }
                ////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////
                //// Learning data
                ss.str("");
                ss << save_mrcnn_img_path << "/" << learning_data_idx << ".jpg";
                cv::imwrite(ss.str(), img_raw_ptr->image); // raw scan image

                ss.str("");
                ss << save_mrcnn_mask_path << "/" << learning_data_idx << ".jpg";
                cv::imwrite(ss.str(), mask_ptr->image); // mask image
                ////////////////////////////////////////////////////////////////////////////////////
                ////////////////////////////////////////////////////////////////////////////////////
            }
        }

    }

}


void robotStateCallback(const hanyang_matching_msgs::msg::RobotState &msg)
{
  if (msg.actualq.size() != 0)
  {
    // ROS_LOG_WARN("robotStateCallback");
    scan_position_q_ = msg.actualq;
    scan_position_x_ = msg.actualx;
    // ROS_LOG_WARN("scan_position_q_: %0.5f, %0.5f, %0.5f, %0.2f, %0.2f, %0.2f", scan_position_q_[0], scan_position_q_[1], scan_position_q_[2], scan_position_q_[3], scan_position_q_[4], scan_position_q_[5]);
    // ROS_LOG_WARN("scan_position_x_: %0.5f, %0.5f, %0.5f, %0.2f, %0.2f, %0.2f", scan_position_x_[0], scan_position_x_[1], scan_position_x_[2], scan_position_x_[3], scan_position_x_[4], scan_position_x_[5]);
  }
  else
  {
    scan_position_q_[0] = 48.850;
    scan_position_q_[1] = -73.240;
    scan_position_q_[2] = -99.900;
    scan_position_q_[3] = -86.880;
    scan_position_q_[4] = 78.750;
    scan_position_q_[5] = 47.870;
    RCLCPP_INFO(rclcpp::get_logger("matching_node"), "Robot scan position is not received!(check connection!)");
  }
}

void hinge_pose_callback(const std_msgs::msg::Float32MultiArray &msg)
{
    std::vector<double> hinge_pose_transformation(msg.data.begin(), msg.data.end());
    printf("hinge pose recieved!!! \n");
    // ROS_LOG_INFO("Target custom transformation pose : %f, %f, %f, %f, %f, %f", 
    //     hinge_pose_transformation[0], hinge_pose_transformation[1], hinge_pose_transformation[2], hinge_pose_transformation[3], 
    //     hinge_pose_transformation[4], hinge_pose_transformation[5], hinge_pose_transformation[6], hinge_pose_transformation[7],
    //     hinge_pose_transformation[8], hinge_pose_transformation[9], hinge_pose_transformation[10], hinge_pose_transformation[11],
    //     hinge_pose_transformation[12], hinge_pose_transformation[13], hinge_pose_transformation[14], hinge_pose_transformation[15]);


    Eigen::Matrix4f tmp = Eigen::Matrix4f::Identity();
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
            tmp(i, j) = hinge_pose_transformation[i*4 + j];
        }
    }
    ROS_LOG_WARN("\n\n");
    std::cout << "HTM_hinge(T_S2O)" << std::endl;
    std::cout << tmp << std::endl;
    ROS_LOG_WARN("\n\n");

    m_hinge_pose_transformation = hinge_pose_transformation;

    //// For hinge
    m_template_matching.m_ptr_template_list[7]->mask_data.hinge_pose_transformation = m_hinge_pose_transformation;

    hanyang_matching_msgs::msg::MatchingResultMsg scanning_result_msg;
    scanning_result_msg.is_scanning_and_detection_finished = false;
    scanning_result_msg.is_open3d_processing_finished = true;
    //// Topic publish
    scanning_and_detection_results_publisher_->publish(scanning_result_msg);


}

void on_raw_image(const sensor_msgs::msg::Image::ConstPtr &msg)
{
    RCLCPP_INFO(rclcpp::get_logger("matching_node"), "Raw image received");
    try
    {
        img_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        img_raw_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
        RCLCPP_INFO(rclcpp::get_logger("matching_node"),"cv_bridge exception: %s", e.what());
        return;
    }
}

bool do_matching_bin_detection(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    int target_id = request->target_id;
    std::string target_name = request->target_name;
    int obj_num = 0;
    obj_num = 41; // 드럼통, 기존:30

    int mask_id_tmp = target_id-obj_num;

    ROS_LOG_INFO("template matching(bin detection): target object %d(%s)", mask_id_tmp, target_name.c_str());

    std::vector<double> custom_transformation_pose = request->custom_transformation_pose;
    ROS_LOG_INFO("Target custom transformation pose : %f, %f, %f, %f, %f, %f", custom_transformation_pose[0], custom_transformation_pose[1], custom_transformation_pose[2], custom_transformation_pose[3], custom_transformation_pose[4], custom_transformation_pose[5]);

    // CTARGET_OBJECT_DATA object_data = m_template_matching.getTemplate(mask_id_tmp - 1); // object #1, 2, ...
    CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[mask_id_tmp - 1];
    
    object_data->custom_transformation_pose = custom_transformation_pose; // [m], [deg]


    // m_math.printCloudMinMax(*object_data->cloud_bin_CAD_uniformly_downsampled_bin_frame_raw, "cloud_bin_CAD_uniformly_downsampled_bin_frame_raw");
    if(0) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_bin_CAD_uniformly_downsampled_bin_frame_raw, 800.0, 75.0, "cloud_bin_CAD_uniformly_downsampled_bin_frame_raw"); }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_scan, *cloud_scan_xyz);
    m_pcl_proc.removeNanPoints(*cloud_scan_xyz, *cloud_scan_xyz);
    m_pcl_proc.filterCloudUsingViewFrustum(*cloud_scan_xyz, view_frustum_, *cloud_scan_xyz);

    m_math.cloudScaling(*cloud_scan_xyz, 1); // m to mm
    *object_data->cloud_bin_measured_sensor_frame = *cloud_scan_xyz;
    if(0) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_bin_measured_sensor_frame, 800.0, 75.0, "cloud_bin_measured_sensor_frame"); }

    m_template_matching.doMeasuredBinCloudProcessing(object_data);
    m_template_matching.doBinMatching(object_data);
    //// NOTICE: base zig detection을 이용하면 아래 함수를 수행해야 함.
    //// NOTICE: base zig detection을 이용하면 아래 함수를 수행해야 함.
    //// NOTICE: base zig detection을 이용하면 아래 함수를 수행해야 함.
    m_template_matching.saveBin2BaseHTM(object_data);
    ROS_LOG_INFO("Bin detection is finished!");
    // update

    ////////////////////////////
    //// rviz viewer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bin_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*object_data->cloud_bin_CAD_uniformly_downsampled_bin_frame_raw, *cloud_bin_CAD_rgb);
    
    // Pre-matching results
    pcl::copyPointCloud(*object_data->cloud_pre_matching_results, *cloud_merge_vis);
    *cloud_merge_vis += *cloud_bin_CAD_rgb;
    m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    sensor_msgs::msg::PointCloud2 cloud_msg_4;
    pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
    cloud_msg_4.header.frame_id = "ku_segmentation_frame";
    cloud_segmentation_results_publisher_->publish(cloud_msg_4);

    // Final results
    sensor_msgs::msg::PointCloud2 cloud_msg_1;
    cloud_merge_vis->clear();
    pcl::copyPointCloud(*object_data->cloud_matching_results, *cloud_merge_vis);
    *cloud_merge_vis += *cloud_bin_CAD_rgb;
    m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
    cloud_msg_1.header.frame_id = "ku_matching_frame";
    cloud_matching_results_publisher_->publish(cloud_msg_1);

    ////////////////////////////

    response->is_pose = true;
    response->matching_accuracy = object_data->matching_accuracy;
    return true;
}

bool do_matching_base_zig_detection(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    ///////////////////////////////////////////////////////
    //// TODO: 아래 함수를 GUI와 연동
    std::vector<double> robot_dh_vec = request->robot_dh_parameters; // [m], [deg]
    std::vector<double> robot_tcp_default = request->robot_tcp_default; // [m], [deg]
    std::vector<double> robot_tcp = request->robot_tcp; // [m], [deg]

    if(m_template_matching.m_ptr_template_tool_list.size() == 0) {
        m_template_matching.initializeGlobalCalibrationToolTemplate(robot_dh_vec, robot_tcp_default, robot_tcp);
    } else {
        ROS_LOG_INFO("Global Calibration Tool is already loaded!");
    }
    
    
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
    ///////////////////////////////////////////////////////

    int target_id = request->target_id;
    std::string target_name = request->target_name;
    int obj_num = 0;
    obj_num = 41; // 드럼통, 기존:30

    int mask_id_tmp = target_id-obj_num;

    ROS_LOG_INFO("template matching(bin detection): target object %d(%s)", mask_id_tmp, target_name.c_str());

    std::vector<double> custom_transformation_pose = request->custom_transformation_pose;
    ROS_LOG_INFO("Target custom transformation pose : %f, %f, %f, %f, %f, %f", custom_transformation_pose[0], custom_transformation_pose[1], custom_transformation_pose[2], custom_transformation_pose[3], custom_transformation_pose[4], custom_transformation_pose[5]);

    CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_tool_list[0]; // global calibration
    object_data->custom_transformation_pose = custom_transformation_pose; // [m], [deg]

    // m_math.printCloudMinMax(*object_data->cloud_bin_CAD_uniformly_downsampled_bin_frame_raw, "cloud_bin_CAD_uniformly_downsampled_bin_frame_raw");
    if(0) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_base_zig_CAD_uniformly_downsampled_object_frame_raw, 800.0, 75.0, "cloud_base_zig_CAD_uniformly_downsampled_object_frame_raw"); }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_scan, *cloud_scan_xyz);
    m_pcl_proc.removeNanPoints(*cloud_scan_xyz, *cloud_scan_xyz);
    m_pcl_proc.filterCloudUsingViewFrustum(*cloud_scan_xyz, view_frustum_, *cloud_scan_xyz);

    m_math.cloudScaling(*cloud_scan_xyz, 1); // m to mm
    *object_data->cloud_base_zig_measured_sensor_frame = *cloud_scan_xyz;
    if(0) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_base_zig_measured_sensor_frame, 800.0, 75.0, "cloud_base_zig_measured_sensor_frame"); }

    m_template_matching.doMeasuredBaseZigProcessing(object_data);
    m_template_matching.doBaseZigMatching(object_data);
    ROS_LOG_INFO("Base zig detection is finished!");
    // update

    ////////////////////////////
    //// rviz viewer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bin_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*object_data->cloud_base_zig_CAD_uniformly_downsampled_object_frame_raw, *cloud_bin_CAD_rgb);
    
    // Pre-matching results
    pcl::copyPointCloud(*object_data->cloud_pre_matching_results, *cloud_merge_vis);
    *cloud_merge_vis += *cloud_bin_CAD_rgb;
    m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    sensor_msgs::msg::PointCloud2 cloud_msg_4;
    pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
    cloud_msg_4.header.frame_id = "ku_segmentation_frame";
    cloud_segmentation_results_publisher_->publish(cloud_msg_4);

    // Final results
    sensor_msgs::msg::PointCloud2 cloud_msg_1;
    cloud_merge_vis->clear();
    pcl::copyPointCloud(*object_data->cloud_matching_results, *cloud_merge_vis);
    *cloud_merge_vis += *cloud_bin_CAD_rgb;
    m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
    cloud_msg_1.header.frame_id = "ku_matching_frame";
    cloud_matching_results_publisher_->publish(cloud_msg_1);

    ////////////////////////////

    response->is_pose = true;
    response->matching_accuracy = object_data->matching_accuracy;
    return true;
}

bool do_matching_eye_in_hand(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    ROS_LOG_WARN("*******************************");
    ROS_LOG_WARN("do_matching_eye_in_hand");
    ROS_LOG_WARN("*******************************");
    ///////////////////////////////////////////////////////
    //// TODO: 아래 함수를 GUI와 연동
    std::vector<double> robot_dh_vec = request->robot_dh_parameters; // [m], [deg]
    std::vector<double> robot_tcp_default = request->robot_tcp_default; // [m], [deg]
    std::vector<double> robot_tcp = request->robot_tcp; // [m], [deg]

    std::vector<double> scan_position_q = request->r2cam_calibration_js_position; // [m], [deg]
    ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);


    ROS_LOG_WARN("");

    if(1) {
        if(m_template_matching.m_ptr_template_tool_list.size() == 0) {
            m_template_matching.initializeGlobalCalibrationToolTemplate(robot_dh_vec, robot_tcp_default, robot_tcp);
        } else {
            ROS_LOG_INFO("Global Calibration Tool is already loaded!");
        }
    }

        
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
    ///////////////////////////////////////////////////////

    int target_id = request->target_id;
    std::string target_name = request->target_name;
    int obj_num = 0;
    obj_num = 41; // 드럼통, 기존:30 표시0107

    int mask_id_tmp = target_id-obj_num;

    ROS_LOG_INFO("template matching(bin detection): target object %d(%s)", mask_id_tmp, target_name.c_str());

    std::vector<double> custom_transformation_pose = request->custom_transformation_pose;
    ROS_LOG_WARN("[Eye-in-hand] Target custom transformation pose : %f, %f, %f, %f, %f, %f", custom_transformation_pose[0], custom_transformation_pose[1], custom_transformation_pose[2], custom_transformation_pose[3], custom_transformation_pose[4], custom_transformation_pose[5]);

    CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_tool_list[0]; // global calibration
    object_data->custom_transformation_pose = custom_transformation_pose; // [m], [deg]


    object_data->pose_B2E_measured = request->pose_b2e_measured; // [m], [deg]
    m_math.printVector(object_data->pose_B2E_measured, "Target B2E(w.r.t. tool master flange) pose");


    // m_math.printCloudMinMax(*object_data->cloud_bin_CAD_uniformly_downsampled_bin_frame_raw, "cloud_bin_CAD_uniformly_downsampled_bin_frame_raw");
    if(0) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_base_zig_CAD_uniformly_downsampled_object_frame_raw, 800.0, 75.0, "cloud_base_zig_CAD_uniformly_downsampled_object_frame_raw"); }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_scan, *cloud_scan_xyz);
    m_pcl_proc.removeNanPoints(*cloud_scan_xyz, *cloud_scan_xyz);
    m_pcl_proc.filterCloudUsingViewFrustum(*cloud_scan_xyz, view_frustum_, *cloud_scan_xyz);

    m_math.cloudScaling(*cloud_scan_xyz, 1); // m to mm
    *object_data->cloud_base_zig_measured_sensor_frame = *cloud_scan_xyz;
    if(0) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_base_zig_measured_sensor_frame, 800.0, 75.0, "cloud_base_zig_measured_sensor_frame"); }

    m_template_matching.doMeasuredEyeInHandZigProcessing(object_data);
    m_template_matching.doEyeInHandZigMatching(object_data);
    ROS_LOG_WARN("[Eye-in-hand] detection is finished!");
    // update

    ////////////////////////////
    //// rviz viewer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bin_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*object_data->cloud_base_zig_CAD_uniformly_downsampled_object_frame_raw, *cloud_bin_CAD_rgb);
    
    // Pre-matching results
    pcl::copyPointCloud(*object_data->cloud_pre_matching_results, *cloud_merge_vis);
    *cloud_merge_vis += *cloud_bin_CAD_rgb;
    m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    sensor_msgs::msg::PointCloud2 cloud_msg_4;
    pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
    cloud_msg_4.header.frame_id = "ku_segmentation_frame";
    cloud_segmentation_results_publisher_->publish(cloud_msg_4);

    // Final results
    sensor_msgs::msg::PointCloud2 cloud_msg_1;
    cloud_merge_vis->clear();
    pcl::copyPointCloud(*object_data->cloud_matching_results, *cloud_merge_vis);
    *cloud_merge_vis += *cloud_bin_CAD_rgb;
    m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
    cloud_msg_1.header.frame_id = "ku_matching_frame";
    cloud_matching_results_publisher_->publish(cloud_msg_1);

    ////////////////////////////

    response->is_pose = true;
    response->matching_accuracy = object_data->matching_accuracy;
    return true;
}
bool do_optimization_hanyang_matching_workspace(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    //// TODO: 별도의 downsampling이 필요하면 추후 사용
    // bool do_scan_sampling = request->do_scan_sampling;
    bool do_scan_sampling = false;
    size_t sampling_num = request->sampling_num;
    std::vector<double> grasp_pose;
    std::vector<std::vector<double> > grasp_sub_pose_set;
    double matching_accuracy = 0.0;
    double approach_distance = 0.0;

    if (is_scan_finished || request->debug_mode)
    {
        if (m_target_id != -1)
        {
            ROS_LOG_INFO("template matching(bin picking): target object %d(%s)", m_target_id, target_name.c_str());
            //   TEMPLATE_MATCHING m_tmp;
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_scan_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_mask_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::copyPointCloud(*cloud_scan, *cloud_scan_in);
            m_pcl_proc.removeNanPoints(*cloud_scan_in, *cloud_scan_in);
            m_pcl_proc.filterCloudUsingViewFrustum(*cloud_scan_in, view_frustum_, *cloud_scan_in);
            pcl::copyPointCloud(*cloud_input, *cloud_mask_in);

            // measured joint angle
            Eigen::VectorXf q_scan(6);
            for (int i = 0; i < 6; i++)
            {
                q_scan[i] = scan_position_q_[i];
            }

            //// Hole pose detection

            //// TODO: 포인터로 변경
            // CTARGET_OBJECT_DATA object_data = m_template_matching.getTemplate(m_target_id - 1); // object #1, 2, ...
            CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[m_target_id - 1];

            printf("scan position(q): %f %f %f %f %f %f\n", q_scan[0], q_scan[1], q_scan[2], q_scan[3], q_scan[4], q_scan[5]);
            m_template_matching.optimizeBinWorkspace(object_data, true, *cloud_mask_in, *cloud_scan_in, m_target_id, target_name, q_scan, grasp_pose, grasp_sub_pose_set, matching_accuracy, do_scan_sampling, sampling_num, false); // (meas_filtering param,  plot)
            ROS_LOG_INFO("Grasping pose (%s): %f %f %f %f %f %f\n", target_name.c_str(), grasp_pose[0], grasp_pose[1], grasp_pose[2], grasp_pose[3], grasp_pose[4], grasp_pose[5]);

            /////////////////////////////////////////////////
            //// rviz visualizer
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_pre_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_final_matching_pose_results(new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);

            if (object_data->is_feasible_pose) {
                ROS_LOG_INFO("is_feasible_pose (do_matching_bin_picking) ---- true");
            } else {
                ROS_LOG_INFO("is_feasible_pose(do_matching_bin_picking) ---- false");
            }
            pcl::copyPointCloud(*object_data->cloud_CAD, *cloud_tgt_vis);
            // pcl::PointXYZ minPt, maxPt;
            // pcl::getMinMax3D(*object_data->cloud_CAD, minPt, maxPt);
            pcl::PointXYZ minPt = object_data->cad_min_pt;
            pcl::PointXYZ maxPt = object_data->cad_max_pt;
            for (size_t i = 0; i < cloud_tgt_vis->size(); i++)
            {
                if (cloud_tgt_vis->points[i].z < maxPt.z - 3.0)
                {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 0;
                    cloud_tgt_vis->points[i].b = 0;
                }
                else
                {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 255; // 가장 윗면 녹색 점 표시
                                                      //   cloud_tgt_vis->points[i].g = 0;
                    cloud_tgt_vis->points[i].b = 0;
                }
            }
            // Matching results update
            pcl::copyPointCloud(*object_data->cloud_matching_results, *cloud_rviz_matching_results);
            //// Pre matching results
            pcl::copyPointCloud(*object_data->cloud_pre_matching_results, *cloud_rviz_pre_matching_results);
            

            //// rviz topic results
            // 1-1) Final matching results
            ROS_LOG_INFO("Publishing cloud/xyzrgba");

            sensor_msgs::msg::PointCloud2 cloud_msg;
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg);
            cloud_msg.header.frame_id = "ku_cloud_frame";
            aligned_points_xyz_publisher_->publish(cloud_msg);

            // 1-2) CAD cloud - black
            for (size_t i = 0; i < cloud_tgt_vis->size(); i++)
            {
                if (cloud_tgt_vis->points[i].z >= maxPt.z - 3.0)
                {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 0;
                    cloud_tgt_vis->points[i].b = 0;
                }
            }
            sensor_msgs::msg::PointCloud2 cloud_msg_1;
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
            cloud_msg_1.header.frame_id = "ku_matching_frame";
            cloud_matching_results_publisher_->publish(cloud_msg_1);

            // 2) Pre matching results (w/ CAD)
            sensor_msgs::msg::PointCloud2 cloud_msg_2;
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_pre_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_2);
            cloud_msg_2.header.frame_id = "ku_pre_matching_frame";
            cloud_pre_matching_results_publisher_->publish(cloud_msg_2);
            /////////////////////////////////////////////////

            // 3) Pre matching results (w/o CAD)
            sensor_msgs::msg::PointCloud2 cloud_msg_3;
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_rviz_pre_matching_results;

            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            // m_math.printCloudMinMax(*cloud_merge_vis, "cloud_merge_vis m");
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_3);
            cloud_msg_3.header.frame_id = "ku_pre_matching_frame_no_cad";
            cloud_pre_matching_results_no_cad_publisher_->publish(cloud_msg_3);
            ///////////////////////////////////////////////

            // 4) Final Pose results with collision check
            //// Collision check final results
            pcl::copyPointCloud(*object_data->cloud_final_results_for_collision_check, *cloud_rviz_final_matching_pose_results);
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_rviz_final_matching_pose_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            sensor_msgs::msg::PointCloud2 cloud_msg_4;
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
            cloud_msg_4.header.frame_id = "ku_segmentation_frame";
            cloud_segmentation_results_publisher_->publish(cloud_msg_4);

            ///////////////////////////////////////////////

            ROS_LOG_INFO("matching_accuracy (%s): %f \n", target_name.c_str(), matching_accuracy);

            ////////////////////////////////////////////////////////////////
            ////////////////////// Visualizer //////////////////////////////
            ////////////////////////////////////////////////////////////////
            //// Mask raw index & new index
            //// Mask image publish
            cv::Mat img_scan_tmp = img_ptr->image; // raw scan image
            //// a) from Mask RCNN results (raw index)
            sensor_msgs::msg::RegionOfInterest mask_box;
            mask_box = object_data->mask_data.boxes[object_data->mask_data.idx_target_mask_in];
            cv::Rect rect_raw(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);
            mask_box = object_data->mask_data.boxes[object_data->mask_data.idx_target_mask_process];
            cv::Rect rect_new(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);

            //// b) Selected mask with color image
            cv::rectangle(img_scan_tmp, rect_raw, cv::Scalar(0, 0, 255), 10, 4, 0);
            cv::rectangle(img_scan_tmp, rect_new, cv::Scalar(0, 255, 0), 10, 4, 0);
            // cv::rectangle(img_scan_tmp, rect, cv::Scalar(0, 255, 0), cv::FILLED, 8, 0);	// 내부 채우기
            sensor_msgs::msg::Image::SharedPtr msg_img_with_selected_mask = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_scan_tmp).toImageMsg();
            raw_iamge_with_selected_mask_publisher_.publish(msg_img_with_selected_mask);
            ROS_LOG_INFO("Selected mask image published!");
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////

            if (object_data->is_feasible_pose)
            {
                ROS_LOG_INFO("Feasible pose!");
                response->is_pose = true;
                response->pose = grasp_pose; // [m], [deg]
                response->sub_pose = grasp_sub_pose_set[0];
                response->matching_accuracy = matching_accuracy;
                response->approach_distance = 0.001 * object_data->approach_distance; // mm to m
                response->is_grasping_pose_flipped = object_data->is_grasping_pose_flipped;
                response->gripper_open_length = object_data->gripper_open_length;
                response->gripper_close_length = object_data->gripper_close_length;
                response->matching_accuracy_limit = object_data->matching_accuracy_limit;
                response->detected_mask_num = object_data->detected_mask_num;

                response->gripper_tip_index = object_data->gripper_tip_index;

                std::cout << "--- Response - Gripper open length: " << object_data->gripper_open_length << std::endl;
                std::cout << "--- Response - Gripper close length: " << object_data->gripper_close_length << std::endl;
                std::cout << "--- Response - Gripper tip_index: " << object_data->gripper_tip_index << std::endl;
            }
            else // Pose infeasible --> 충돌 또는 충돌 회피 자세 없는 경우
            {
                ROS_LOG_INFO("Infeasible pose!");
                response->is_pose = false;
                response->matching_accuracy = 0.0;
                response->approach_distance = 0.0;
                // response->gripper_open_length = object_data->gripper_open_length;
                response->detected_mask_num = object_data->detected_mask_num;
            }

            is_scan_finished = false;
            return true;
        }
        else
        {
            response->is_pose = false;
            response->matching_accuracy = 0.0;
            response->approach_distance = 0.0;
            response->detected_mask_num = 0;
            // response->gripper_open_length = object_data->gripper_open_length;
            // response->pose = grasp_pose;
            printf("Grasping pose pose cannot be estimated: 1!!!\n");
            is_scan_finished = false;
            // return false;
            return true;
        }
    }
    else
    {
        response->is_pose = false;
        response->matching_accuracy = 0.0;
        response->approach_distance = 0.0;
        response->detected_mask_num = 0;
        // response->gripper_open_length = object_data->gripper_open_length;
        // response->pose = grasp_pose;
        printf("Grasping pose cannot be estimated: 2!!!\n");
        is_scan_finished = false;
        // return false;
        return true;
    }
}

bool do_get_robot_camera_calibration_js_position(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    ///////////////////////////////////////////////////////
    //// TODO: 아래 함수를 GUI와 연동
    std::vector<double> robot_dh_vec = request->robot_dh_parameters; // [m], [deg]
    std::vector<double> robot_tcp_default = request->robot_tcp_default; // [m], [deg]
    std::vector<double> robot_tcp = request->robot_tcp; // [m], [deg]
    m_template_matching.initializeGlobalCalibrationToolTemplate(robot_dh_vec, robot_tcp_default, robot_tcp);
    
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
    ///////////////////////////////////////////////////////

    int id_tmp = request->target_id - 30;
    std::string name_tmp = request->target_name;

    CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_tool_list[0];

    object_data->target_id = id_tmp;

    if(!request->is_r2cam_calibration_js_position_set_by_teaching) { // Automatic pose generation
        ROS_LOG_INFO("(Automatic generation) Get robot to camera calibration JS position (robot to camera calibration): target object %d(%s)", id_tmp, name_tmp.c_str());
        m_template_matching.getCalibrationRobotJSPosition(object_data);
    } else { // Teaching
        ROS_LOG_INFO("(By teaching) Set robot to camera calibration JS position (robot to camera calibration): target object %d(%s)", id_tmp, name_tmp.c_str());
        object_data->r2cam_calibration_JS_position = request->r2cam_calibration_js_position; // [deg]
        m_template_matching.setCalibrationRobotJSPosition(object_data);
    }

    m_math.printVector(object_data->r2cam_calibration_JS_position, "object_data->r2cam_calibration_JS_position");
    m_math.printHTM(object_data->T_bin_robot_camera_calibration_nominal, "object_data->T_bin_robot_camera_calibration_nominal");

    m_T_bin_robot_camera_calibration_nominal = object_data->T_bin_robot_camera_calibration_nominal;

    ////////////////////////////
    //// rviz viewer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_calibration_js_position_results(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*object_data->cloud_final_results_for_collision_check, *cloud_rviz_calibration_js_position_results);
    m_math.cloudScaling(*cloud_rviz_calibration_js_position_results, 0); // mm to m
    sensor_msgs::msg::PointCloud2 cloud_msg_4;
    pcl::toROSMsg(*cloud_rviz_calibration_js_position_results, cloud_msg_4); // metric: [m]
    cloud_msg_4.header.frame_id = "ku_segmentation_frame";
    cloud_segmentation_results_publisher_->publish(cloud_msg_4);
    ////////////////////////////

    response->is_pose = true;
    response->robot_camera_calibration_js_position = object_data->r2cam_calibration_JS_position;
    
    return true;
}

bool do_matching_robot_camera_calibration(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    ROS_LOG_INFO("template matching(robot to camera calibration): target object %d(%s)", m_target_id, target_name.c_str());

    std::vector<double> custom_transformation_pose = request->custom_transformation_pose;
    ROS_LOG_INFO("Target custom transformation pose : %f, %f, %f, %f, %f, %f", custom_transformation_pose[0], custom_transformation_pose[1], custom_transformation_pose[2], custom_transformation_pose[3], custom_transformation_pose[4], custom_transformation_pose[5]);

    // CTARGET_OBJECT_DATA object_data = m_template_matching.getTemplate(m_target_id - 1); // object #1, 2, ...
    // CTARGET_OBJECT_DATA object_data = m_template_matching.m_template_tool_list[0];
    CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_tool_list[0];
    
    object_data->custom_transformation_pose = custom_transformation_pose; // [m], [deg]


    if(0) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_calibration_tool_CAD_uniformly_downsampled_object_frame, 800.0, 75.0, "cloud_calibration_tool_CAD_uniformly_downsampled_object_frame"); }


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud_scan, *cloud_scan_xyz);
    ROS_LOG_INFO("raw_scan_cloud size (before removing Nan points): %zu", cloud_scan_xyz->size());
    m_pcl_proc.removeNanPoints(*cloud_scan_xyz, *cloud_scan_xyz);
    m_pcl_proc.filterCloudUsingViewFrustum(*cloud_scan_xyz, view_frustum_, *cloud_scan_xyz);
    m_math.cloudScaling(*cloud_scan_xyz, 1); // m to mm
    *object_data->cloud_calibration_tool_measured_sensor_frame = *cloud_scan_xyz;
    if(0) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_calibration_tool_measured_sensor_frame, 800.0, 75.0, "cloud_calibration_tool_measured_sensor_frame"); }

    m_template_matching.doMeasuredCalibrationToolCloudProcessing(object_data);
    object_data->T_bin_robot_camera_calibration_nominal = m_T_bin_robot_camera_calibration_nominal; 

    object_data->pose_B2E_measured = request->pose_b2e_measured; // [m], [deg]
    m_math.printVector(object_data->pose_B2E_measured, "Target B2E(w.r.t. tool master flange) pose");
    m_template_matching.doRobotCameraCalibrationMatching(object_data);
    ROS_LOG_INFO("Robot to camera calibration is finished!");
    // update

    ////////////////////////////
    //// rviz viewer
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bin_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*object_data->cloud_calibration_tool_CAD_uniformly_downsampled_object_frame, *cloud_bin_CAD_rgb);
    
    // Pre-matching results
    pcl::copyPointCloud(*object_data->cloud_pre_matching_results, *cloud_merge_vis);
    *cloud_merge_vis += *cloud_bin_CAD_rgb;
    m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    sensor_msgs::msg::PointCloud2 cloud_msg_4;
    pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
    cloud_msg_4.header.frame_id = "ku_segmentation_frame";
    cloud_segmentation_results_publisher_->publish(cloud_msg_4);

    // Final results
    sensor_msgs::msg::PointCloud2 cloud_msg_1;
    cloud_merge_vis->clear();
    pcl::copyPointCloud(*object_data->cloud_matching_results, *cloud_merge_vis);
    *cloud_merge_vis += *cloud_bin_CAD_rgb;
    m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
    cloud_msg_1.header.frame_id = "ku_matching_frame";
    cloud_matching_results_publisher_->publish(cloud_msg_1);

    ////////////////////////////

    response->is_pose = true;
    response->matching_accuracy = object_data->matching_accuracy;
    return true;
}

bool do_task_evaluate_grasping_pose(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    // size_t target_id = request->target_id - 30;
    // m_target_id = target_id;
    // ROS_LOG_INFO("do_task_evaluate_grasping_pose(): target object %d(%s)", m_target_id, target_name.c_str());
    // std::vector<double> custom_transformation_pose = request->custom_transformation_pose;
    // ROS_LOG_INFO("Target custom transformation pose : %f, %f, %f, %f, %f, %f", custom_transformation_pose[0], custom_transformation_pose[1], custom_transformation_pose[2], custom_transformation_pose[3], custom_transformation_pose[4], custom_transformation_pose[5]);

    // CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[m_target_id - 1];
    // object_data->custom_transformation_pose = custom_transformation_pose; // [m], [deg]

    // //// TODO: cloud_evaluate_grasping_pose_CAD_base_aligned
    // //// TODO: cloud_evaluate_grasping_pose_CAD_base_aligned
    // //// TODO: 이 부분은 자세 산출 시에 추출된 정보를 바로 이용하는 것으로
    // //// TODO: do_evaluate_grasping_pose과는 다른 부분임
    // if(1) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_evaluate_grasping_pose_CAD_base_aligned, 800.0, 75.0, "cloud_evaluate_grasping_pose_CAD_base_aligned"); }


    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud(*cloud_scan, *cloud_scan_xyz);
    // m_math.cloudScaling(*cloud_scan_xyz, 1); // m to mm
    // *object_data->cloud_evaluate_grasping_pose_measured_sensor_frame = *cloud_scan_xyz;
    // m_math.printCloudMinMax(*cloud_scan_xyz, "cloud_scan_xyz");


    // //// TODO: 지정된 자세에서 고정된 그리퍼 파트만 나오도록 영역 설정하기
    // //// TODO: 지정된 자세에서 고정된 그리퍼 파트만 나오도록 영역 설정하기
    // //// TODO: 지정된 자세에서 고정된 그리퍼 파트만 나오도록 영역 설정하기
    // m_template_matching.doMeasuredCloudProcessingForEvaluatingGraspingPose(object_data);
    // if(1) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_evaluate_grasping_pose_measured_sensor_frame, 800.0, 75.0, "cloud_evaluate_grasping_pose_measured_sensor_frame"); }
    // // object_data->T_bin_robot_camera_calibration_nominal = m_T_bin_robot_camera_calibration_nominal; 

    // object_data->pose_B2E_measured = request->pose_B2E_measured; // [m], [deg]
    // m_math.printVector(object_data->pose_B2E_measured, "Target B2E(w.r.t. tool master flange) pose");


    // //// TODO: 충돌 감지 병합된 Octomap 복사
    // object_data->m_OctoMap_evaluate_grasping_pose_CAD = object_data->m_set_OctoMap_gripper_CAD_tool_frame[object_data->gripper_idx_now];
    // object_data->md_KdTree_octoMap_evaluate_grasping_pose_CAD = object_data->set_md_KdTree_octoMap_gripper_CAD_tool_frame[object_data->gripper_idx_now];
    // m_template_matching.doMatchingForEvaluationGraspingPoseVer2(object_data);
    // ROS_LOG_INFO("Evaluation grasping pose is finished!");
    // // update

    // ////////////////////////////
    // //// rviz viewer
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bin_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::copyPointCloud(*object_data->cloud_evaluate_grasping_pose_CAD_base_aligned, *cloud_bin_CAD_rgb);
    
    // // Pre-matching results
    // pcl::copyPointCloud(*object_data->cloud_pre_matching_results, *cloud_merge_vis);
    // *cloud_merge_vis += *cloud_bin_CAD_rgb;
    // m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    // sensor_msgs::PointCloud2 cloud_msg_4;
    // pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
    // cloud_msg_4.header.frame_id = "ku_segmentation_frame";
    // cloud_segmentation_results_publisher_.publish(cloud_msg_4);

    // // Final results
    // sensor_msgs::PointCloud2 cloud_msg_1;
    // cloud_merge_vis->clear();
    // pcl::copyPointCloud(*object_data->cloud_matching_results, *cloud_merge_vis);
    // *cloud_merge_vis += *cloud_bin_CAD_rgb;
    // m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    // pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
    // cloud_msg_1.header.frame_id = "ku_matching_frame";
    // cloud_matching_results_publisher_.publish(cloud_msg_1);

    // // ////////////////////////////

    // // response->is_pose = true;
    // response->matching_accuracy = object_data->matching_accuracy;
    return true;
}

bool do_evaluate_grasping_pose(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    // size_t target_id = request->target_id - 30;
    // m_target_id = target_id;
    // ROS_LOG_INFO("do_evaluate_grasping_pose(): target object %d(%s)", m_target_id, target_name.c_str());
    // std::vector<double> custom_transformation_pose = request->custom_transformation_pose;
    // ROS_LOG_INFO("Target custom transformation pose : %f, %f, %f, %f, %f, %f", custom_transformation_pose[0], custom_transformation_pose[1], custom_transformation_pose[2], custom_transformation_pose[3], custom_transformation_pose[4], custom_transformation_pose[5]);

    // CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[m_target_id - 1];
    
    // object_data->custom_transformation_pose = custom_transformation_pose; // [m], [deg]

    // //// Gripper CAD idx
    // for (size_t i = 0; i < object_data->set_gripper_CAD_open_length.size(); i++)
    // {
    //     if(object_data->set_gripper_CAD_open_length[i] == object_data->gripper_open_length)
    //     {
    //         object_data->gripper_idx_now = i;
    //         break;
    //     }
    // }
    // ROS_LOG_INFO("Current gripper index: %zu", object_data->gripper_idx_now);
    // pcl::copyPointCloud(object_data->set_cloud_evaluate_grasping_pose_CAD_tool_frame[object_data->gripper_idx_now], *object_data->cloud_evaluate_grasping_pose_CAD_object_frame);
    

    // //// TODO: cloud_evaluate_grasping_pose_CAD_object_frame
    // //// TODO: cloud_evaluate_grasping_pose_CAD_object_frame
    // //// TODO: 이 부분은 저장된 CAD 파일을 이용하는 것으로
    // //// TODO: do_task_evaluate_grasping_pose 다른 부분임
    // if(0) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_evaluate_grasping_pose_CAD_object_frame, 800.0, 75.0, "cloud_calibration_tool_CAD_uniformly_downsampled_object_frame"); }


    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scan_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::copyPointCloud(*cloud_scan, *cloud_scan_xyz);
    // m_math.cloudScaling(*cloud_scan_xyz, 1); // m to mm
    // *object_data->cloud_evaluate_grasping_pose_measured_sensor_frame = *cloud_scan_xyz;
    // m_math.printCloudMinMax(*cloud_scan_xyz, "cloud_scan_xyz");
    // m_template_matching.doMeasuredCloudProcessingForEvaluatingGraspingPose(object_data);
    // if(0) { m_pcl_proc.plotCloudWithFrame(*object_data->cloud_evaluate_grasping_pose_measured_sensor_frame, 800.0, 75.0, "cloud_calibration_tool_measured_sensor_frame"); }
    // object_data->T_bin_robot_camera_calibration_nominal = m_T_bin_robot_camera_calibration_nominal; 

    // object_data->pose_B2E_measured = request->pose_B2E_measured; // [m], [deg]
    // m_math.printVector(object_data->pose_B2E_measured, "Target B2E(w.r.t. tool master flange) pose");

    // object_data->m_OctoMap_evaluate_grasping_pose_CAD = object_data->m_set_OctoMap_gripper_CAD_tool_frame[object_data->gripper_idx_now];
    // object_data->md_KdTree_octoMap_evaluate_grasping_pose_CAD = object_data->set_md_KdTree_octoMap_gripper_CAD_tool_frame[object_data->gripper_idx_now];
    // m_template_matching.doMatchingForEvaluationGraspingPose(object_data);
    // ROS_LOG_INFO("Evaluation grasping pose is finished!");
    // // update

    // ////////////////////////////
    // //// rviz viewer
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_bin_CAD_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pcl::copyPointCloud(*object_data->cloud_evaluate_grasping_pose_CAD_object_frame, *cloud_bin_CAD_rgb);
    
    // // Pre-matching results
    // pcl::copyPointCloud(*object_data->cloud_pre_matching_results, *cloud_merge_vis);
    // *cloud_merge_vis += *cloud_bin_CAD_rgb;
    // m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    // sensor_msgs::PointCloud2 cloud_msg_4;
    // pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
    // cloud_msg_4.header.frame_id = "ku_segmentation_frame";
    // cloud_segmentation_results_publisher_.publish(cloud_msg_4);

    // // Final results
    // sensor_msgs::PointCloud2 cloud_msg_1;
    // cloud_merge_vis->clear();
    // pcl::copyPointCloud(*object_data->cloud_matching_results, *cloud_merge_vis);
    // *cloud_merge_vis += *cloud_bin_CAD_rgb;
    // m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    // pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
    // cloud_msg_1.header.frame_id = "ku_matching_frame";
    // cloud_matching_results_publisher_.publish(cloud_msg_1);

    // // ////////////////////////////

    // response->is_pose = true;
    // response->matching_accuracy = object_data->matching_accuracy;
    return true;
}




bool do_zig_pose_initial_teaching(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    size_t target_id = request->target_id - 30;
    m_target_id = target_id;
    ROS_LOG_INFO("Zig Pose Initial Teaching Process: target object %d(%s)", m_target_id, target_name.c_str());
    CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[m_target_id - 1];
    

    m_template_matching.doZigPoseInitialTeaching(object_data);

    ROS_LOG_INFO("Zig Pose Initial Teaching Process - Finished!");

    // update
    response->is_pose = true;
    return true;
}

bool do_matching_bin_picking(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    //// TODO: 별도의 downsampling이 필요하면 추후 사용
    // bool do_scan_sampling = request->do_scan_sampling;
    bool do_scan_sampling = false;
    size_t sampling_num = request->sampling_num;
    std::vector<double> grasp_pose;
    std::vector<std::vector<double> > grasp_sub_pose_set;
    std::vector<double> zig_pose;
    double matching_accuracy = 0.0;
    double approach_distance = 0.0;

    bool is_symmetric_CAD = request->is_symmetric;
    if(is_symmetric_CAD) {
        for (size_t i = 0; i < 10; i++) ROS_LOG_WARN("is_symmetric_CAD: TRUE");
    } else {
        for (size_t i = 0; i < 10; i++) ROS_LOG_WARN("is_symmetric_CAD: FALSE");
    }

    //// 인식된 Mask가 없는 경우
    if(m_detected_mask_num == 0)
    {
        response->is_pose = false;
        response->matching_accuracy = 0.0;
        response->approach_distance = 0.0;
        response->detected_mask_num = 0;
        // response->gripper_open_length = object_data->gripper_open_length;
        // response->pose = grasp_pose;
        printf("No Mask detected (do_matching_bin_picking)!!!\n");
        is_scan_finished = false;
        // return false;
        return true;
    }

    if (is_scan_finished || request->debug_mode)
    {
        if (m_target_id != -1)
        {
            ROS_LOG_INFO("template matching(bin picking): target object %d(%s)", m_target_id, target_name.c_str());
            //   TEMPLATE_MATCHING m_tmp;
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_scan_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_mask_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
            pcl::copyPointCloud(*cloud_scan, *cloud_scan_in);
            pcl::copyPointCloud(*cloud_input, *cloud_mask_in);

            // measured joint angle
            Eigen::VectorXf q_scan(6);
            for (int i = 0; i < 6; i++)
            {
                q_scan[i] = scan_position_q_[i];
            }

            //// Hole pose detection
            // printf("scan position(q): %f %f %f %f %f %f\n", q_scan[0], q_scan[1], q_scan[2], q_scan[3], q_scan[4], q_scan[5]);
            pcl::console::TicToc time;
            time.tic();

            ///////////////////////////////////////////
            /////////////// bolt picking///////////////
            ///////////////////////////////////////////

            if(is_image_ai_detection_) {
                if (m_target_id == 7) { // bolt
                    //// Image picking point를 직접 사용하는 경우, ex) 볼트
                    ROS_LOG_INFO(" bolt!!! (image-based picking process)");
                    m_template_matching.m_ptr_template_list[m_target_id - 1]->is_image_based_picking_process = true;

                    Eigen::Matrix4f grasp_pose;
                    size_t selected_idx = m_template_matching.m_ptr_template_list[m_target_id - 1]->mask_data.pre_selected_idx_set[0];
                    geometry_msgs::msg::Pose2D picking_pose = m_template_matching.m_ptr_template_list[m_target_id - 1]->mask_data.picking_poses[selected_idx];

                    cout << endl << endl << endl;

                } else if(m_target_id == 8) { // hinge
                    ROS_LOG_WARN("HINGE 1");
                    m_template_matching.m_ptr_template_list[m_target_id - 1]->do_matching_with_image_info = true; // 
                    ROS_LOG_WARN("HINGE 2");
                    Eigen::Matrix4f grasp_pose;
                    ROS_LOG_WARN("HINGE 3");
                    size_t selected_idx = m_template_matching.m_ptr_template_list[m_target_id - 1]->mask_data.pre_selected_idx_set[0];
                    ROS_LOG_WARN("HINGE 4");
                    geometry_msgs::msg::Pose2D picking_pose = m_template_matching.m_ptr_template_list[m_target_id - 1]->mask_data.picking_poses[0];
                    ROS_LOG_WARN("HINGE 5");

                    cout << "x: "  << picking_pose.x << "y: "  << picking_pose.y << "theta: " << picking_pose.theta << endl;
                    ROS_LOG_WARN("HINGE");
                    ROS_LOG_WARN("HINGE");
                    ROS_LOG_WARN("HINGE");
                    cout << endl << endl << endl;

                
                } else {
                    m_template_matching.m_ptr_template_list[m_target_id - 1]->is_image_based_picking_process = false;
                    m_template_matching.m_ptr_template_list[m_target_id - 1]->do_matching_with_image_info = false; // 
                }
            } else {
                m_template_matching.m_ptr_template_list[m_target_id - 1]->is_image_based_picking_process = false;
                m_template_matching.m_ptr_template_list[m_target_id - 1]->do_matching_with_image_info = false; // 
            }

            
            ///////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////
            //// 250324
            m_template_matching.m_ptr_template_list[m_target_id - 1]->is_symmetric_CAD = is_symmetric_CAD;
            // Object CAD - Raw CAD Selection
            if(!is_symmetric_CAD) { // 기존 CAD
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_raw_CAD;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_for_quick_align = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_raw_CAD_for_quick_align;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_quick_align_with_mesh_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_raw_CAD_quick_align_with_mesh_normal;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_quick_align_with_est_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_raw_CAD_quick_align_with_est_normal;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_uniformly_downsampled = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_raw_CAD_uniformly_downsampled;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_uniformly_downsampled_with_mesh_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_raw_CAD_uniformly_downsampled_with_mesh_normal;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_uniformly_downsampled_with_est_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_raw_CAD_uniformly_downsampled_with_est_normal;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_with_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_raw_CAD_with_normal;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_with_normal_downsampled = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_raw_CAD_with_normal_downsampled;
                
                m_template_matching.m_ptr_template_list[m_target_id - 1]->quick_align_cloud_CAD_feature = m_template_matching.m_ptr_template_list[m_target_id - 1]->quick_align_cloud_raw_CAD_feature;
                m_template_matching.m_ptr_template_list[m_target_id - 1]->quick_align_cloud_CAD_feature_with_normal = m_template_matching.m_ptr_template_list[m_target_id - 1]->quick_align_cloud_raw_CAD_feature_with_normal;

                m_template_matching.m_ptr_template_list[m_target_id - 1]->cad_height_z = m_template_matching.m_ptr_template_list[m_target_id - 1]->cad_raw_height_z;
                m_template_matching.m_ptr_template_list[m_target_id - 1]->cad_cross_length = m_template_matching.m_ptr_template_list[m_target_id - 1]->cad_raw_cross_length;

                m_template_matching.m_ptr_template_list[m_target_id - 1]->object_bounding_box_CAD_frame = m_template_matching.m_ptr_template_list[m_target_id - 1]->object_bounding_box_CAD_raw_frame;

                m_template_matching.m_ptr_template_list[m_target_id - 1]->m_OctoMap_CAD = m_template_matching.m_ptr_template_list[m_target_id - 1]->m_OctoMap_raw_CAD;
                m_template_matching.m_ptr_template_list[m_target_id - 1]->md_KdTree_octoMap_CAD = m_template_matching.m_ptr_template_list[m_target_id - 1]->md_KdTree_octoMap_raw_CAD;
            } else { // symmetric CAD
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_symmetric_CAD;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_for_quick_align = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_symmetric_CAD_for_quick_align;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_quick_align_with_mesh_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_symmetric_CAD_quick_align_with_mesh_normal;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_quick_align_with_est_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_symmetric_CAD_quick_align_with_est_normal;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_uniformly_downsampled = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_symmetric_CAD_uniformly_downsampled;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_uniformly_downsampled_with_mesh_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_symmetric_CAD_uniformly_downsampled_with_mesh_normal;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_uniformly_downsampled_with_est_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_symmetric_CAD_uniformly_downsampled_with_est_normal;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_with_normal = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_symmetric_CAD_with_normal;
                *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_CAD_with_normal_downsampled = *m_template_matching.m_ptr_template_list[m_target_id - 1]->cloud_symmetric_CAD_with_normal_downsampled;

                m_template_matching.m_ptr_template_list[m_target_id - 1]->quick_align_cloud_CAD_feature = m_template_matching.m_ptr_template_list[m_target_id - 1]->quick_align_cloud_symmetric_CAD_feature;
                m_template_matching.m_ptr_template_list[m_target_id - 1]->quick_align_cloud_CAD_feature_with_normal = m_template_matching.m_ptr_template_list[m_target_id - 1]->quick_align_cloud_symmetric_CAD_feature_with_normal;

                m_template_matching.m_ptr_template_list[m_target_id - 1]->cad_height_z = m_template_matching.m_ptr_template_list[m_target_id - 1]->cad_symmetric_height_z;
                m_template_matching.m_ptr_template_list[m_target_id - 1]->cad_cross_length = m_template_matching.m_ptr_template_list[m_target_id - 1]->cad_symmetric_cross_length;

                m_template_matching.m_ptr_template_list[m_target_id - 1]->object_bounding_box_CAD_frame = m_template_matching.m_ptr_template_list[m_target_id - 1]->object_bounding_box_CAD_symmetric_frame;

                m_template_matching.m_ptr_template_list[m_target_id - 1]->m_OctoMap_CAD = m_template_matching.m_ptr_template_list[m_target_id - 1]->m_OctoMap_symmetric_CAD;
                m_template_matching.m_ptr_template_list[m_target_id - 1]->md_KdTree_octoMap_CAD = m_template_matching.m_ptr_template_list[m_target_id - 1]->md_KdTree_octoMap_symmetric_CAD;
            }

            ///////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////
            ///////////////////////////////////////////////////////


            m_template_matching.icp_matchingBinPicking(true, *cloud_mask_in, *cloud_scan_in, m_target_id, target_name, q_scan, grasp_pose, grasp_sub_pose_set, zig_pose, matching_accuracy, do_scan_sampling, sampling_num, false); // (meas_filtering param,  plot)

            ROS_LOG_INFO("here 4!");
            std::cout << " icp_matchingBinPicking process time: " << time.toc() << " ms" << std::endl;

            /////////////////////////////////////////////////
            //// rviz visualizer
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_pre_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_final_matching_pose_results(new pcl::PointCloud<pcl::PointXYZRGB>);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);

            CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[m_target_id - 1];
            if (object_data->is_feasible_pose) {
                ROS_LOG_INFO("Grasping pose (%s): %f %f %f %f %f %f\n", target_name.c_str(), grasp_pose[0], grasp_pose[1], grasp_pose[2], grasp_pose[3], grasp_pose[4], grasp_pose[5]);
                ROS_LOG_INFO("is_feasible_pose (do_matching_bin_picking) ---- true");

                if(is_image_ai_detection_) {
                    if (m_target_id == 7) {
                        matching_accuracy = 1.0; 
                    }
                }
            }
            else {
                ROS_LOG_INFO("is_feasible_pose(do_matching_bin_picking) ---- false");
            }
            pcl::copyPointCloud(*object_data->cloud_CAD, *cloud_tgt_vis);
            pcl::PointXYZ minPt = object_data->cad_min_pt;
            pcl::PointXYZ maxPt = object_data->cad_max_pt;
            for (size_t i = 0; i < cloud_tgt_vis->size(); i++)
            {
                if (cloud_tgt_vis->points[i].z < maxPt.z - 3.0)
                {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 0;
                    cloud_tgt_vis->points[i].b = 0;
                }
                else
                {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 255; // 가장 윗면 녹색 점 표시
                                                      //   cloud_tgt_vis->points[i].g = 0;
                    cloud_tgt_vis->points[i].b = 0;
                }
            }
            // Matching results update
            pcl::copyPointCloud(*object_data->cloud_matching_results, *cloud_rviz_matching_results);
            //// Pre matching results
            pcl::copyPointCloud(*object_data->cloud_pre_matching_results, *cloud_rviz_pre_matching_results);
            

            //// rviz topic results
            // 1-1) Final matching results
            ROS_LOG_INFO("Publishing cloud/xyzrgba");

            IMAGE2CLOUDPROCESSING m_image2cloud_proc;
            sensor_msgs::msg::PointCloud2 cloud_msg;
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg);
            cloud_msg.header.frame_id = "ku_cloud_frame";
            aligned_points_xyz_publisher_->publish(cloud_msg);

            // 1-2) CAD cloud - black
            for (size_t i = 0; i < cloud_tgt_vis->size(); i++)
            {
                if (cloud_tgt_vis->points[i].z >= maxPt.z - 3.0)
                {
                    cloud_tgt_vis->points[i].r = 0;
                    cloud_tgt_vis->points[i].g = 0;
                    cloud_tgt_vis->points[i].b = 0;
                }
            }
            sensor_msgs::msg::PointCloud2 cloud_msg_1;
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
            cloud_msg_1.header.frame_id = "ku_matching_frame";
            cloud_matching_results_publisher_->publish(cloud_msg_1);

            // 2) Pre matching results (w/ CAD)
            sensor_msgs::msg::PointCloud2 cloud_msg_2;
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_tgt_vis;
            *cloud_merge_vis += *cloud_rviz_pre_matching_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_2);
            cloud_msg_2.header.frame_id = "ku_pre_matching_frame";
            cloud_pre_matching_results_publisher_->publish(cloud_msg_2);
            /////////////////////////////////////////////////

            // 3) Pre matching results (w/o CAD)
            sensor_msgs::msg::PointCloud2 cloud_msg_3;
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_rviz_pre_matching_results;

            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            // m_math.printCloudMinMax(*cloud_merge_vis, "cloud_merge_vis m");
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_3);
            cloud_msg_3.header.frame_id = "ku_pre_matching_frame_no_cad";
            cloud_pre_matching_results_no_cad_publisher_->publish(cloud_msg_3);
            ///////////////////////////////////////////////

            // 4) Final Pose results with collision check
            //// Collision check final results
            pcl::copyPointCloud(*object_data->cloud_final_results_for_collision_check, *cloud_rviz_final_matching_pose_results);
            cloud_merge_vis->clear();
            *cloud_merge_vis = *cloud_rviz_final_matching_pose_results;
            m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
            sensor_msgs::msg::PointCloud2 cloud_msg_4;
            pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
            cloud_msg_4.header.frame_id = "ku_segmentation_frame";
            cloud_segmentation_results_publisher_->publish(cloud_msg_4);

            ///////////////////////////////////////////////

            ///////////////////////////////////////////////
            // ////////////////////////////////////////////////////////////////////
            // //// Object Frame
            // geometry_msgs::PoseStamped object_frame_tf_msg;

            // Eigen::Matrix4f T_B2O = object_data->T_matching_B2O;
            // Eigen::Quaternionf q_B2O(T_B2O.block<3,3>(0,0));
            // q_B2O.normalize();
            // object_frame_tf_msg.pose.position.x = 0.001*T_B2O(0, 3); // [m]
            // object_frame_tf_msg.pose.position.y = 0.001*T_B2O(1, 3);
            // object_frame_tf_msg.pose.position.z = 0.001*T_B2O(2, 3);
            // // Orientation quaternion
            // object_frame_tf_msg.pose.orientation.x = q_B2O.x();
            // object_frame_tf_msg.pose.orientation.y = q_B2O.y();
            // object_frame_tf_msg.pose.orientation.z = q_B2O.z();
            // object_frame_tf_msg.pose.orientation.w = q_B2O.w();
            // object_frame_tf_msg.header.frame_id = "ku_segmentation_frame";

            // object_frame_wrt_base_results_publisher_.publish(object_frame_tf_msg); // [mm]
            // ////////////////////////////////////////////////////////////////////
            // //// Tool Frame
            // geometry_msgs::PoseStamped tool_frame_tf_msg;

            // Eigen::Matrix4f T_B2T = object_data->T_matching_B2T;
            // Eigen::Quaternionf q_B2T(T_B2T.block<3,3>(0,0));
            // q_B2T.normalize();
            // tool_frame_tf_msg.pose.position.x = 0.001*T_B2T(0, 3); // [m]
            // tool_frame_tf_msg.pose.position.y = 0.001*T_B2T(1, 3);
            // tool_frame_tf_msg.pose.position.z = 0.001*T_B2T(2, 3);
            // // Orientation quaternion
            // tool_frame_tf_msg.pose.orientation.x = q_B2T.x();
            // tool_frame_tf_msg.pose.orientation.y = q_B2T.y();
            // tool_frame_tf_msg.pose.orientation.z = q_B2T.z();
            // tool_frame_tf_msg.pose.orientation.w = q_B2T.w();
            // tool_frame_tf_msg.header.frame_id = "ku_segmentation_frame";

            // tool_frame_wrt_base_results_publisher_.publish(tool_frame_tf_msg); // [mm]
            // ////////////////////////////////////////////////////////////////////


            if(!m_template_matching.m_ptr_template_list[m_target_id - 1]->is_image_based_picking_process) {
                ROS_LOG_WARN("Template matching-based grasping pose generation (%s)", target_name.c_str());
                ROS_LOG_INFO("matching_accuracy (%s): %f", target_name.c_str(), matching_accuracy);
                ROS_LOG_INFO("Target Mask Index #%zu)\n", m_template_matching.m_ptr_template_list[m_target_id - 1]->mask_data.idx_target_mask_process + 1);
            } else {
                ROS_LOG_WARN("Image-based grasping pose generation (%s)", target_name.c_str());
                ROS_LOG_INFO("Target Mask Index #%zu\n", m_template_matching.m_ptr_template_list[m_target_id - 1]->mask_data.idx_target_mask_process + 1);
            }

            ////////////////////////////////////////////////////////////////
            ////////////////////// Visualizer //////////////////////////////
            ////////////////////////////////////////////////////////////////
            //// Mask raw index & new index
            //// Mask image publish
            cv::Mat img_scan_tmp = img_ptr->image; // raw scan image
            //// a) from Mask RCNN results (raw index)
            sensor_msgs::msg::RegionOfInterest mask_box;
            mask_box = object_data->mask_data.boxes[object_data->mask_data.idx_target_mask_in];
            cv::Rect rect_raw(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);
            mask_box = object_data->mask_data.boxes[object_data->mask_data.idx_target_mask_process];
            cv::Rect rect_new(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);

            //// b) Selected mask with color image
            cv::rectangle(img_scan_tmp, rect_raw, cv::Scalar(0, 0, 255), 10, 4, 0);
            cv::rectangle(img_scan_tmp, rect_new, cv::Scalar(0, 255, 0), 10, 4, 0);
            // cv::rectangle(img_scan_tmp, rect, cv::Scalar(0, 255, 0), cv::FILLED, 8, 0);	// 내부 채우기
            sensor_msgs::msg::Image::SharedPtr msg_img_with_selected_mask = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_scan_tmp).toImageMsg();
            raw_iamge_with_selected_mask_publisher_.publish(msg_img_with_selected_mask);
            ROS_LOG_INFO("Selected mask image published!");
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////
            ////////////////////////////////////////////////////////////////

            if (object_data->is_feasible_pose)
            {
                ROS_LOG_INFO("Feasible pose!");
                response->is_pose = true;
                response->pose = grasp_pose; // [m], [deg]
                response->sub_pose = grasp_sub_pose_set[0];
                response->zig_pose = zig_pose; // [m], [deg]
                response->matching_accuracy = matching_accuracy;
                response->approach_distance = 0.001 * object_data->approach_distance; // mm to m
                response->is_grasping_pose_flipped = object_data->is_grasping_pose_flipped;
                response->gripper_open_length = object_data->gripper_open_length;
                response->gripper_close_length = object_data->gripper_close_length;
                response->matching_accuracy_limit = object_data->matching_accuracy_limit;
                response->detected_mask_num = object_data->detected_mask_num;
                response->gripper_tip_index = object_data->gripper_tip_index;

                std::cout << "--- Response - Gripper open length: " << object_data->gripper_open_length << std::endl;
                std::cout << "--- Response - Gripper close length: " << object_data->gripper_close_length << std::endl;
                std::cout << "--- Response - Gripper tip_index: " << object_data->gripper_tip_index << std::endl;

            }
            else // Pose infeasible --> 충돌 또는 충돌 회피 자세 없는 경우
            {
                ROS_LOG_INFO("Infeasible pose!");
                response->is_pose = false;
                response->matching_accuracy = 0.0;
                response->approach_distance = 0.0;
                // response->gripper_open_length = object_data->gripper_open_length;
                response->detected_mask_num = object_data->detected_mask_num;
            }

            is_scan_finished = false;
            return true;
        }
        else
        {
            response->is_pose = false;
            response->matching_accuracy = 0.0;
            response->approach_distance = 0.0;
            response->detected_mask_num = 0;
            // response->gripper_open_length = object_data->gripper_open_length;
            // response->pose = grasp_pose;
            printf("Grasping pose pose cannot be estimated: 1!!!\n");
            is_scan_finished = false;
            // return false;
            return true;
        }
    }
    else
    {
        response->is_pose = false;
        response->matching_accuracy = 0.0;
        response->approach_distance = 0.0;
        response->detected_mask_num = 0;
        // response->gripper_open_length = object_data->gripper_open_length;
        // response->pose = grasp_pose;
        printf("Grasping pose cannot be estimated: 2!!!\n");
        is_scan_finished = false;
        // return false;
        return true;
    }
}

void do_matching_hanyang_matching_topic(const hanyang_matching_msgs::msg::DoTemplateMatchingMsg &msg)
{
    printf("\n\n*********************************************\n");
    printf("*********************************************\n");
    printf("*********************************************\n");
    printf("Do_matching_hanyang_matching_topic subscribe!\n");
    printf("Do_matching_hanyang_matching_topic subscribe!\n");
    printf("Do_matching_hanyang_matching_topic subscribe!\n\n");
    printf("Matching will be started when the mask clouds are received!\n\n");
    printf("Matching will be started when the mask clouds are received!\n\n");
    printf("Matching will be started when the mask clouds are received!\n\n");

    //// For test
    // if(!msg.do_matching)
    // {
    //     printf("msg.do_matching --> false\n");
    //     // matching_result_msg.is_pose = false;
    //     do_matching_process_topic = false;
    //     return;
    // }
    printf("*********************************************\n");
    printf("*********************************************\n");
    printf("*********************************************\n");

    do_matching_process_topic = true;
    matching_sampling_num = msg.sampling_num;
    matching_debug_mode = msg.debug_mode;
}

bool do_multiple_matching_bin_picking(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{

    // //// TODO: 별도의 downsampling이 필요하면 추후 사용
    // // bool do_scan_sampling = request.do_scan_sampling;
    // bool do_scan_sampling = false;
    // size_t sampling_num = request.sampling_num;
    // std::vector<double> grasp_pose;
    // std::vector<std::vector<double> > grasp_sub_pose_set;
    // std::vector<double> zig_pose;
    // double matching_accuracy = 0.0;
    // double approach_distance = 0.0;

    // //// 인식된 Mask가 없는 경우
    // if(m_detected_mask_num == 0)
    // {
    //     response.is_pose = false;
    //     response.matching_accuracy = 0.0;
    //     response.approach_distance = 0.0;
    //     response.detected_mask_num = 0;
    //     // response.gripper_open_length = object_data->gripper_open_length;
    //     // response.pose = grasp_pose;
    //     printf("No Mask detected (do_multiple_matching_bin_picking)!!!\n");
    //     is_scan_finished = false;
    //     // return false;
    //     return true;
    // }

    // if (is_scan_finished || request.debug_mode)
    // {
    //     if (m_target_id != -1)
    //     {

    //         ROS_LOG_INFO("template matching(bin picking): target object %d(%s)", m_target_id, target_name.c_str());
    //         //   TEMPLATE_MATCHING m_tmp;
    //         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_scan_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_mask_in(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    //         pcl::copyPointCloud(*cloud_scan, *cloud_scan_in);
    //         pcl::copyPointCloud(*cloud_input, *cloud_mask_in);

    //         // measured joint angle
    //         Eigen::VectorXf q_scan(6);
    //         for (int i = 0; i < 6; i++)
    //         {
    //             q_scan[i] = scan_position_q_[i];
    //         }

    //         m_template_matching.icp_matchingMultipleBinPicking(true, *cloud_mask_in, *cloud_scan_in, m_target_id, target_name, q_scan, grasp_pose, grasp_sub_pose_set, zig_pose, matching_accuracy, do_scan_sampling, sampling_num, false); // (meas_filtering param,  plot)
    //         ROS_LOG_INFO("here 4!");
    //         ROS_LOG_INFO("Grasping pose (%s): %f %f %f %f %f %f\n", target_name.c_str(), grasp_pose[0], grasp_pose[1], grasp_pose[2], grasp_pose[3], grasp_pose[4], grasp_pose[5]);

    //         /////////////////////////////////////////////////
    //         //// rviz visualizer
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_pre_matching_results(new pcl::PointCloud<pcl::PointXYZRGB>);
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rviz_final_matching_pose_results(new pcl::PointCloud<pcl::PointXYZRGB>);

    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_tgt_vis(new pcl::PointCloud<pcl::PointXYZRGB>);
    //         pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_merge_vis(new pcl::PointCloud<pcl::PointXYZRGB>);

    //         CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[m_target_id - 1];
    //         if (object_data->is_feasible_pose)
    //             ROS_LOG_INFO("is_feasible_pose (do_multiple_matching_bin_picking) ---- true");
    //         else
    //             ROS_LOG_INFO("is_feasible_pose(do_multiple_matching_bin_picking) ---- false");

    //         pcl::copyPointCloud(*object_data->cloud_CAD, *cloud_tgt_vis);
    //         pcl::PointXYZ minPt = object_data->cad_min_pt;
    //         pcl::PointXYZ maxPt = object_data->cad_max_pt;
    //         for (size_t i = 0; i < cloud_tgt_vis->size(); i++)
    //         {
    //             if (cloud_tgt_vis->points[i].z < maxPt.z - 3.0)
    //             {
    //                 cloud_tgt_vis->points[i].r = 0;
    //                 cloud_tgt_vis->points[i].g = 0;
    //                 cloud_tgt_vis->points[i].b = 0;
    //             }
    //             else
    //             {
    //                 cloud_tgt_vis->points[i].r = 0;
    //                 cloud_tgt_vis->points[i].g = 255; // 가장 윗면 녹색 점 표시
    //                                                   //   cloud_tgt_vis->points[i].g = 0;
    //                 cloud_tgt_vis->points[i].b = 0;
    //             }
    //         }
    //         // Matching results update
    //         pcl::copyPointCloud(*object_data->cloud_matching_results, *cloud_rviz_matching_results);
    //         //// Pre matching results
    //         pcl::copyPointCloud(*object_data->cloud_pre_matching_results, *cloud_rviz_pre_matching_results);
            

    //         //// rviz topic results
    //         // 1-1) Final matching results
    //         ROS_DEBUG("Publishing cloud/xyzrgba");

    //         IMAGE2CLOUDPROCESSING m_image2cloud_proc;
    //         sensor_msgs::PointCloud2 cloud_msg;
    //         *cloud_merge_vis = *cloud_tgt_vis;
    //         *cloud_merge_vis += *cloud_rviz_matching_results;
    //         m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    //         pcl::toROSMsg(*cloud_merge_vis, cloud_msg);
    //         cloud_msg.header.frame_id = "ku_cloud_frame";
    //         aligned_points_xyz_publisher_.publish(cloud_msg);

    //         // 1-2) CAD cloud - black
    //         for (size_t i = 0; i < cloud_tgt_vis->size(); i++)
    //         {
    //             if (cloud_tgt_vis->points[i].z >= maxPt.z - 3.0)
    //             {
    //                 cloud_tgt_vis->points[i].r = 0;
    //                 cloud_tgt_vis->points[i].g = 0;
    //                 cloud_tgt_vis->points[i].b = 0;
    //             }
    //         }
    //         sensor_msgs::PointCloud2 cloud_msg_1;
    //         *cloud_merge_vis = *cloud_tgt_vis;
    //         *cloud_merge_vis += *cloud_rviz_matching_results;
    //         m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    //         pcl::toROSMsg(*cloud_merge_vis, cloud_msg_1);
    //         cloud_msg_1.header.frame_id = "ku_matching_frame";
    //         cloud_matching_results_publisher_.publish(cloud_msg_1);

    //         // 2) Pre matching results (w/ CAD)
    //         sensor_msgs::PointCloud2 cloud_msg_2;
    //         cloud_merge_vis->clear();
    //         *cloud_merge_vis = *cloud_tgt_vis;
    //         *cloud_merge_vis += *cloud_rviz_pre_matching_results;
    //         m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    //         pcl::toROSMsg(*cloud_merge_vis, cloud_msg_2);
    //         cloud_msg_2.header.frame_id = "ku_pre_matching_frame";
    //         cloud_pre_matching_results_publisher_.publish(cloud_msg_2);
    //         /////////////////////////////////////////////////

    //         // 3) Pre matching results (w/o CAD)
    //         sensor_msgs::PointCloud2 cloud_msg_3;
    //         cloud_merge_vis->clear();
    //         *cloud_merge_vis = *cloud_rviz_pre_matching_results;

    //         m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    //         // m_math.printCloudMinMax(*cloud_merge_vis, "cloud_merge_vis m");
    //         pcl::toROSMsg(*cloud_merge_vis, cloud_msg_3);
    //         cloud_msg_3.header.frame_id = "ku_pre_matching_frame_no_cad";
    //         cloud_pre_matching_results_no_cad_publisher_.publish(cloud_msg_3);
    //         ///////////////////////////////////////////////

    //         // 4) Final Pose results with collision check
    //         //// Collision check final results
    //         pcl::copyPointCloud(*object_data->cloud_final_multiple_results_for_collision_check, *cloud_rviz_final_matching_pose_results);
    //         cloud_merge_vis->clear();
    //         *cloud_merge_vis = *cloud_rviz_final_matching_pose_results;
    //         m_math.cloudScaling(*cloud_merge_vis, 0); // mm to m
    //         sensor_msgs::PointCloud2 cloud_msg_4;
    //         pcl::toROSMsg(*cloud_merge_vis, cloud_msg_4); // metric: [m]
    //         cloud_msg_4.header.frame_id = "ku_segmentation_frame";
    //         cloud_segmentation_results_publisher_.publish(cloud_msg_4);

    //         ///////////////////////////////////////////////
    //         ////////////////////////////////////////////////////////////////////
    //         //// Object Frame
    //         geometry_msgs::PoseStamped object_frame_tf_msg;

    //         Eigen::Matrix4f T_B2O = object_data->T_matching_B2O;
    //         Eigen::Quaternionf q_B2O(T_B2O.block<3,3>(0,0));
    //         q_B2O.normalize();
    //         object_frame_tf_msg.pose.position.x = 0.001*T_B2O(0, 3); // [m]
    //         object_frame_tf_msg.pose.position.y = 0.001*T_B2O(1, 3);
    //         object_frame_tf_msg.pose.position.z = 0.001*T_B2O(2, 3);
    //         // Orientation quaternion
    //         object_frame_tf_msg.pose.orientation.x = q_B2O.x();
    //         object_frame_tf_msg.pose.orientation.y = q_B2O.y();
    //         object_frame_tf_msg.pose.orientation.z = q_B2O.z();
    //         object_frame_tf_msg.pose.orientation.w = q_B2O.w();
    //         object_frame_tf_msg.header.frame_id = "ku_segmentation_frame";

    //         object_frame_wrt_base_results_publisher_.publish(object_frame_tf_msg); // [mm]
    //         ////////////////////////////////////////////////////////////////////
    //         //// Tool Frame
    //         geometry_msgs::PoseStamped tool_frame_tf_msg;

    //         Eigen::Matrix4f T_B2T = object_data->T_matching_B2T;
    //         Eigen::Quaternionf q_B2T(T_B2T.block<3,3>(0,0));
    //         q_B2T.normalize();
    //         tool_frame_tf_msg.pose.position.x = 0.001*T_B2T(0, 3); // [m]
    //         tool_frame_tf_msg.pose.position.y = 0.001*T_B2T(1, 3);
    //         tool_frame_tf_msg.pose.position.z = 0.001*T_B2T(2, 3);
    //         // Orientation quaternion
    //         tool_frame_tf_msg.pose.orientation.x = q_B2T.x();
    //         tool_frame_tf_msg.pose.orientation.y = q_B2T.y();
    //         tool_frame_tf_msg.pose.orientation.z = q_B2T.z();
    //         tool_frame_tf_msg.pose.orientation.w = q_B2T.w();
    //         tool_frame_tf_msg.header.frame_id = "ku_segmentation_frame";

    //         tool_frame_wrt_base_results_publisher_.publish(tool_frame_tf_msg); // [mm]
    //         ////////////////////////////////////////////////////////////////////






    //         ROS_LOG_INFO("matching_accuracy (%s): %f \n", target_name.c_str(), matching_accuracy);

    //         ////////////////////////////////////////////////////////////////
    //         ////////////////////// Visualizer //////////////////////////////
    //         ////////////////////////////////////////////////////////////////
    //         //// Mask raw index & new index
    //         //// Mask image publish
    //         cv::Mat img_scan_tmp = img_ptr->image; // raw scan image
    //         //// a) from Mask RCNN results (raw index)
    //         sensor_msgs::RegionOfInterest mask_box;
    //         mask_box = object_data->mask_data.boxes[object_data->mask_data.idx_target_mask_in];
    //         cv::Rect rect_raw(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);
    //         mask_box = object_data->mask_data.boxes[object_data->mask_data.idx_target_mask_process];
    //         cv::Rect rect_new(mask_box.x_offset, mask_box.y_offset, mask_box.width, mask_box.height);

    //         //// b) Selected mask with color image
    //         cv::rectangle(img_scan_tmp, rect_raw, cv::Scalar(0, 0, 255), 10, 4, 0);
    //         cv::rectangle(img_scan_tmp, rect_new, cv::Scalar(0, 255, 0), 10, 4, 0);
    //         // cv::rectangle(img_scan_tmp, rect, cv::Scalar(0, 255, 0), cv::FILLED, 8, 0);	// 내부 채우기
    //         sensor_msgs::ImagePtr msg_img_with_selected_mask = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_scan_tmp).toImageMsg();
    //         raw_iamge_with_selected_mask_publisher_.publish(msg_img_with_selected_mask);
    //         ROS_LOG_INFO("Selected mask image published!");
    //         ////////////////////////////////////////////////////////////////
    //         ////////////////////////////////////////////////////////////////
    //         ////////////////////////////////////////////////////////////////

    //         if (object_data->is_feasible_pose)
    //         {
    //             ROS_LOG_INFO("Feasible pose!");
    //             response.is_pose = true;
    //             response.pose = grasp_pose; // [m], [deg]
    //             response.sub_pose = grasp_sub_pose_set[0];
    //             response.zig_pose = zig_pose; // [m], [deg]
    //             response.matching_accuracy = matching_accuracy;
    //             response.approach_distance = 0.001 * object_data->approach_distance; // mm to m
    //             response.is_grasping_pose_flipped = object_data->is_grasping_pose_flipped;
    //             response.gripper_open_length = object_data->gripper_open_length;
    //             response.gripper_close_length = object_data->gripper_close_length;
    //             response.matching_accuracy_limit = object_data->matching_accuracy_limit;
    //             response.detected_mask_num = object_data->detected_mask_num;
    //             response.gripper_tip_index = object_data->gripper_tip_index;

    //             std::cout << "--- Response - Gripper open length: " << object_data->gripper_open_length << std::endl;
    //             std::cout << "--- Response - Gripper close length: " << object_data->gripper_close_length << std::endl;
    //             std::cout << "--- Response - Gripper tip_index: " << object_data->gripper_tip_index << std::endl;

    //             std::cout << "--- Response - bp_result_set_grasping_pose size: " << object_data->bp_result_set_grasping_pose.size() << std::endl;
    //             std::vector<hanyang_matching_msgs::BinPickingResults> bp_result_set_msg;
    //             for (size_t i = 0; i < object_data->bp_result_set_grasping_pose.size(); i++)
    //             {
    //                 hanyang_matching_msgs::BinPickingResults bp_result_tmp;
    //                 bp_result_tmp.pose = object_data->bp_result_set_grasping_pose[i];
    //                 bp_result_tmp.sub_pose = object_data->bp_result_set_sub_grasping_pose_set[i][0];
    //                 bp_result_tmp.zig_pose = object_data->bp_result_set_zig_pose[i];
    //                 bp_result_tmp.matching_accuracy = object_data->bp_result_set_matching_accuracy[i];
    //                 bp_result_tmp.is_pose = object_data->bp_result_set_is_pose_feasible[i];

    //                 bp_result_set_msg.push_back(bp_result_tmp);

    //             }
    //             response.bp_result_set = bp_result_set_msg;

    //         }
    //         else // Pose infeasible --> 충돌 또는 충돌 회피 자세 없는 경우
    //         {
    //             ROS_LOG_INFO("Infeasible pose!");
    //             response.is_pose = false;
    //             response.matching_accuracy = 0.0;
    //             response.approach_distance = 0.0;
    //             // response.gripper_open_length = object_data->gripper_open_length;
    //             response.detected_mask_num = object_data->detected_mask_num;
    //         }

    //         is_scan_finished = false;
    //         return true;
    //     }
    //     else
    //     {
    //         response.is_pose = false;
    //         response.matching_accuracy = 0.0;
    //         response.approach_distance = 0.0;
    //         response.detected_mask_num = 0;
    //         // response.gripper_open_length = object_data->gripper_open_length;
    //         // response.pose = grasp_pose;
    //         printf("Grasping pose pose cannot be estimated: 1!!!\n");
    //         is_scan_finished = false;
    //         // return false;
    //         return true;
    //     }
    // }
    // else
    // {
    //     response.is_pose = false;
    //     response.matching_accuracy = 0.0;
    //     response.approach_distance = 0.0;
    //     response.detected_mask_num = 0;
    //     // response.gripper_open_length = object_data->gripper_open_length;
    //     // response.pose = grasp_pose;
    //     printf("Grasping pose cannot be estimated: 2!!!\n");
    //     is_scan_finished = false;
    //     // return false;
    //     return true;
    // }

    return true;
}

bool do_get_env_parameters(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    try
    {
        ROS_LOG_INFO("do_get_env_parameters!");
        int target_id = request->target_id;
        int obj_num = 0;
        obj_num = 41; // 드럼통, 기존:30

        int mask_id_tmp = target_id-obj_num;
        CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[mask_id_tmp - 1]; // object #1, 2, ...
        std::vector<double> htm_base2sensor_vec = object_data->htm_base2sensor_vec;
        m_math.printVector(htm_base2sensor_vec, "htm_base2sensor_vec");
        response->robot_camera_calibration_htm_b2s_global = htm_base2sensor_vec;
        return true;
    }
    catch(std::string &e)
    {
        return false;
    }
}


bool do_PCL_test(const std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Request> request, std::shared_ptr<hanyang_matching_msgs::srv::DoTemplateMatching::Response> response)
{
    try
    {
        // ROS_LOG_INFO("do_PCL_test! - saveBin2BaseHTM");
        // int target_id = request->target_id;
        // int obj_num = 41; // 드럼통, 기존:30
        // int mask_id_tmp = target_id-obj_num;
        // CTARGET_OBJECT_DATA* object_data = m_template_matching.m_ptr_template_list[mask_id_tmp - 1]; // object #1, 2, ...

        // m_template_matching.saveBin2BaseHTM(object_data);


        return true;
    }
    catch(std::string &e)
    {
        return false;
    }
}


//////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv)
{
    printf("matching_node ready...\n");
    scan_position_q_.resize(6);
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("template_matching_node");

    auto robot_sub_ = node->create_subscription<hanyang_matching_msgs::msg::RobotState>("/bp_robot_state", 20, robotStateCallback);
    auto mask_sub_ = node->create_subscription<hanyang_matching_msgs::msg::MaskCloud>("/cloud_mask_results", 1, do_receive_cloud);
    auto template_input_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_template_initialize", do_template_initialize);

    //// Initial stage
    auto do_matching_bin_detection_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_template_matching_bin_detection", do_matching_bin_detection);
    auto do_matching_base_zig_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_template_matching_base_zig_detection", do_matching_base_zig_detection);
    auto do_matching_eye_in_hand_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_template_matching_eye_in_hand", do_matching_eye_in_hand);

    auto do_optimization_hanyang_matching_workspace_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_optimization_hanyang_matching_workspace", do_optimization_hanyang_matching_workspace);
    auto do_get_robot_camera_calibration_js_position_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_get_robot_camera_calibration_js_position", do_get_robot_camera_calibration_js_position);
    auto do_matching_robot2camera_calibration_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_template_matching_robot_camera_calibration", do_matching_robot_camera_calibration);

    //// Pose estimation
    auto do_matching_hanyang_matching_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_template_matching_bin_picking", do_matching_bin_picking); // bin-picking
    auto do_matching_hanyang_matching_topic_subscriber_ = node->create_subscription<hanyang_matching_msgs::msg::DoTemplateMatchingMsg>("/do_template_matching_hanyang_matching_topic", 1, do_matching_hanyang_matching_topic);

    scanning_and_detection_results_publisher_ = node->create_publisher<hanyang_matching_msgs::msg::MatchingResultMsg>("/scanning_result", 1);
    matching_pose_results_publisher_ = node->create_publisher<hanyang_matching_msgs::msg::MatchingResultMsg>("/cad_matching_result", 1);
    
    // rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
    // // auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    // matching_pose_results_publisher_ = node->create_publisher<hanyang_matching_msgs::msg::MatchingResultMsg>("/cad_matching_result", qos);


    //// EVALUATION GRASPING POSE
    auto do_evaluate_grasping_pose_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_evaluate_grasping_pose", do_evaluate_grasping_pose);
    //// ZIG POSE INITIAL TEACHING
    auto do_zig_pose_initial_teaching_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_zig_pose_initial_teaching", do_zig_pose_initial_teaching);

    //// SIMULATION
    // auto do_simulation_bin_placing_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_simulation_bin_placing", do_simulation_bin_placing);
    //// 
    auto do_get_env_parameters_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_get_env_parameters", do_get_env_parameters);
    //// TEST FUNCTION
    auto do_pcl_test1_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_pcl_test", do_PCL_test);
    // auto do_pcl_test2_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_pcl_feature_extraction", do_PCL_test_featureExtraction); // FPFH, ...
    // auto do_pcl_test3_server_ = node->create_service<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_pcl_segmentation", do_PCL_test_segmentation); // RGS, ...
    
    //// rviz visualizer
    aligned_points_xyz_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ku_scan/cloud/xyzrgba", 1);
    cloud_matching_results_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ku_scan/cloud/xyzrgba/matching", 1);
    cloud_pre_matching_results_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ku_scan/cloud/xyzrgba/pre_matching", 1);
    cloud_pre_matching_results_no_cad_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ku_scan/cloud/xyzrgba/pre_matching_no_cad", 1);
    cloud_segmentation_results_publisher_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("/ku_scan/cloud/xyzrgba/segmentation_results", 1);

    
    //// Hinge resgtrator
    auto hinge_pose_sub = node->create_subscription<std_msgs::msg::Float32MultiArray>("/hinge_pose", 1, hinge_pose_callback);

    //// image view
    auto image_color_sub = node->create_subscription<sensor_msgs::msg::Image>("/zivid/color/image_color", 1, on_raw_image);

    image_transport::ImageTransport it(node);
    selected_mask_image_publisher_ = it.advertise("/ku_scan/camera/selected_mask_image", 1);
    raw_iamge_with_selected_mask_publisher_ = it.advertise("/ku_scan/camera/raw_image_with_selected_mask", 1);
    rclcpp::spin(node);
    rclcpp::shutdown();

    return (0);
}