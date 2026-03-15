/**
 * @file bin_picking_node.cpp
 * @brief ROS Node 관리를 위한 구현파일
 */

// #include <ros/ros.h>
// #include <ros/network.h>
#include <bin_picking/bin_picking_node.hpp>
#include <fstream>
#include <iostream>
#include <random>

using json = nlohmann::json;

// namespace robot_control_ui {


BinPickingNode::BinPickingNode(int argc, char **argv )
    : init_argc(argc), init_argv(argv) {

    detected_pose_.resize(6);
    detected_sub_pose_.resize(6);

    estimated_pose_.resize(6);
    estimated_sub_pose_.resize(6);

    assigned_stacking_pose_.resize(6);

    //// # of gripper driver: 2
    grp_measured_count.resize(2);
    grp_measured_position.resize(2);
    grp_initial_min_position.resize(2);
    grp_initial_max_position.resize(2);
    // is_grp_initialized.resize(2);
    is_grp_set_min_value_finished.resize(2);
    is_grp_set_max_value_finished.resize(2);

    for(int i=0; i<2; i++)
    {
        grp_measured_count[i] = 0;
        grp_measured_position[i] = 0;
        grp_initial_min_position[i] = 0;
        grp_initial_max_position[i] = 1;
        // is_grp_initialized[i] = false;
        is_grp_set_min_value_finished[i] = false;
        is_grp_set_max_value_finished[i] = false;
    }
}

BinPickingNode::~BinPickingNode() {
    rclcpp::shutdown(); // explicitly needed since we use ros::start();
    wait();
}

/** @brief Korean: BinPickingNode 클래스의 초기화를 수행한다.
 * @return true: BinPickingNode 실행 성공, false: BinPickingNode 실행 실패
 */
bool BinPickingNode::init() {
    string prefix = ROBOT_NAME;
    prefix += "_";

    //// NOTICE: terminate called after throwing an instance of 'rclcpp::ContextAlreadyInitialized'
    // rclcpp::init(init_argc, init_argv);
    //// NOTICE: terminate called after throwing an instance of 'rclcpp::ContextAlreadyInitialized'

    node_ = rclcpp::Node::make_shared("bin_picking_node");



    // rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

    //// Topic
    // matchingPoseResultSubscriber_ = node_->create_subscription<hanyang_matching_msgs::msg::MatchingResultMsg>("/cad_matching_result",
    //                             1, bind(&BinPickingNode::templateMatchingCallback, this, placeholders::_1));
    // matchingPoseResultSubscriber_ = node_->create_subscription<hanyang_matching_msgs::msg::MatchingResultMsg>("/cad_matching_result", qos, BinPickingNode::templateMatchingCallback);

    // rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
    // auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
    // auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(1));
    // matchingPoseResultSubscriber_ = node_->create_subscription<hanyang_matching_msgs::msg::MatchingResultMsg>("/cad_matching_result", qos, std::bind(&BinPickingNode::templateMatchingCallback, this, std::placeholders::_1));
    // matchingPoseResultSubscriber_ = node_->create_subscription<hanyang_matching_msgs::msg::MatchingResultMsg>("/cad_matching_result", 1, &BinPickingNode::templateMatchingCallback);
    // matchingPoseResultSubscriber_ = node_->create_subscription<hanyang_matching_msgs::msg::MatchingResultMsg>("/cad_matching_result",
    //                             1, bind(&BinPickingNode::templateMatchingCallback, this, placeholders::_1));

    // doTemplateMatchingBinPickingPublisher_      = node_->create_publisher<hanyang_matching_msgs::msg::DoTemplateMatchingMsg> ("/do_template_matching_bin_picking_topic", qos);
    // uiCommandLearningPublisher_                 = node_->create_publisher<hanyang_matching_msgs::msg::BpUiCommandLearning>     ("/mask_rcnn_zivid/load_weight", qos); // Command to learning client

    doTemplateMatchingBinPickingPublisher_      = node_->create_publisher<hanyang_matching_msgs::msg::DoTemplateMatchingMsg>   ("/do_template_matching_bin_picking_topic", 1);
    uiCommandLearningPublisher_                 = node_->create_publisher<hanyang_matching_msgs::msg::BpUiCommandLearning>     ("/sam_zivid/load_weight", 1); // Command to learning client

    //// Service
    // 3D Scanning
    zividScanningClient_                        = node_->create_client<hanyang_matching_msgs::srv::ZividDoScan>           ("/zivid_scanning");
    doTemplateMatchingClient_                   = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_template_matching");
    doTemplateMatchingHoleClient_               = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_template_matching_hole");
    doTemplateMatchingBinPickingClient_         = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_template_matching_bin_picking");
    doPoseEstimationBinPickingClient_           = node_->create_client<hanyang_matching_msgs::srv::DoPoseEstimation>      ("/do_pose_estimation_bin_picking");

    doTemplateInitializeClient_                 = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_template_initialize");

    robotACalCSPoseWithDefaultTCPClient_        = node_->create_client<hanyang_matching_msgs::srv::SetTcp>                (prefix +"cal_pose_default_tcp_command");
    robotACalJSPositionWithDefaultTCPClient_    = node_->create_client<hanyang_matching_msgs::srv::SetTcp>                (prefix +"cal_js_position_default_tcp_command");

    doBinTemplateMatchingClient_                = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_template_matching_bin_detection");
    doBaseZigTemplateMatchingClient_            = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_template_matching_base_zig_detection");
    doEyeInHandTemplateMatchingClient_            = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>  ("/do_template_matching_eye_in_hand");

    doOptimizationBinPickingWorkspaceClient_    = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_optimization_bin_picking_workspace");

    doGetRobot2CameraCalJSPositionClient_       = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_get_robot_camera_calibration_js_position");
    doRobot2CameraCalTemplateMatchingClient_    = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_template_matching_robot_camera_calibration");

    doEvaluateGraspingPoseClient_               = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_evaluate_grasping_pose");
    doZigPoseInitialTeachingClient_             = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_zig_pose_initial_teaching");


    doSimulationBinPlacingClient_               = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_simulation_bin_placing");
    doGetEnvParametersClient_                   = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_get_env_parameters");

    doPCLTestClient_                            = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_pcl_test");
    doPCLFunctionFeatureExtractionClient_       = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_pcl_feature_extraction");
    doPCLFunctionSegmentationClient_            = node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>    ("/do_pcl_segmentation");

    cnt_scanning_failure = 0; // scanning recovery 수행 횟수

    hinge_pose_sub_ = node_->create_subscription<std_msgs::msg::Float32MultiArray>("/hinge_pose", 1, std::bind(&BinPickingNode::hinge_pose_callback, this, std::placeholders::_1));

    // Tool changing JS position
    setTaskToolChangingJSPosition();
    // Tip changing JS position
    setTaskTipChangingJSPosition();

    //// Robot id
    // m_robot_id = UR5CB; // UR5CB
    m_robot_id = UR10e; // UR10e

    matching_accuracy = 0.0; // template matching accuracy
    matching_accuracy_limit = 1.0;
    approach_distance = 0.0;

    // ROS_LOG_INFO("bin_picking_node - init()");

    start();
    return true;
}

void BinPickingNode::spinNode() {
    rclcpp::spin_some(node_);
}

void BinPickingNode::printVector(const std::vector<double> &vec, std::string str)
{
    printf("%s: ", str.c_str());
    for(int i = 0; i < vec.size(); i++) printf("%0.3f, ", vec[i]);
    printf("\n");
}

void BinPickingNode::hinge_pose_callback(const std_msgs::msg::Float32MultiArray &msg)
{
    is_hinge_pose_recived = true;
}

void BinPickingNode::setTaskToolChangingJSPosition()
{
    //// TODO: 값이 잘못 들어온 경우에 대한 예외 처리 필요
    //// TCP idx
    task_tcp_idx_set_tool_changing_attach_.clear();
    task_tcp_idx_set_tool_changing_attach_.push_back(1); // slave #1 - TCP #1 (cylinder, single gripper)
    task_tcp_idx_set_tool_changing_attach_.push_back(11); // slave #11 - TCP #11 (pnematic, single gripper)
    task_tcp_idx_set_tool_changing_attach_.push_back(2); // slave #2 - TCP #2 (elbow & t joint, single gripper)
    task_tcp_idx_set_tool_changing_attach_.push_back(4); // slave #3 - TCP #4 (bolt bush, single gripper)
    task_tcp_idx_set_tool_changing_attach_.push_back(7); // slave #4 - TCP #7 (Calibration tool, single gripper)

    //// FILE PATH
    std::string file_name("bin_picking_node.cpp");
    std::string config_path_ = __FILE__;
    config_path_.resize(config_path_.length() - file_name.length());
    config_path_ += "../../config/";

    std::stringstream ss;
    std::stringstream ss_json;
    ss_json << config_path_ << "tool_changing_pose/tool_changing_js_position_base_frame.json";
    std::ifstream ifs(ss_json.str().c_str());
    json j_in = json::parse(ifs);
    int cnt_js = j_in["count"];

    task_vec_set_tool_changing_attach_JS_position_.clear();
    task_vec_set_tool_changing_detach_JS_position_.clear();
    task_vec_tool_changing_goal_cs_pose_.clear();
    task_vec_set_tool_changing_attach_JS_position_.resize(cnt_js);
    task_vec_set_tool_changing_detach_JS_position_.resize(cnt_js);
    task_vec_tool_changing_goal_cs_pose_.resize(cnt_js);

    //// json file input - target JS angle
    // printf("*******************************************\n");


    //// Tool bracket left/right side
    ss.str("");
    ss << "is_tool_left_side";
    is_tool_left_side_ = j_in[ss.str()].get<std::vector<bool>>();

    // for(int i = 0; i < is_tool_left_side_.size(); i++) {
    //     if(is_tool_left_side_[i]) {
    //         printf("is_tool_left_side_[%i] = true\n", i);
    //     } else {
    //         printf("is_tool_left_side_[%i] = false\n", i);
    //     }
    // }

    //// Default JS Position
    ss.str("");
    ss << "default_attaching_js_position_robot_left_side";
    tc_robot_left_side_attach_default_js_position_ = j_in[ss.str()].get<std::vector<double>>();
    // printf("tc_robot_left_side_attach_default_js_position_: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", tc_robot_left_side_attach_default_js_position_[0], tc_robot_left_side_attach_default_js_position_[1],tc_robot_left_side_attach_default_js_position_[2],tc_robot_left_side_attach_default_js_position_[3],tc_robot_left_side_attach_default_js_position_[4],tc_robot_left_side_attach_default_js_position_[5]);
    ss.str("");
    ss << "default_detaching_js_position_robot_left_side";
    tc_robot_left_side_detach_default_js_position_ = j_in[ss.str()].get<std::vector<double>>();
    // printf("tc_robot_left_side_detach_default_js_position_: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", tc_robot_left_side_detach_default_js_position_[0], tc_robot_left_side_detach_default_js_position_[1],tc_robot_left_side_detach_default_js_position_[2],tc_robot_left_side_detach_default_js_position_[3],tc_robot_left_side_detach_default_js_position_[4],tc_robot_left_side_detach_default_js_position_[5]);
    ss.str("");
    ss << "default_attaching_js_position_robot_right_side";
    tc_robot_right_side_attach_default_js_position_ = j_in[ss.str()].get<std::vector<double>>();
    // printf("tc_robot_right_side_attach_default_js_position_: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", tc_robot_right_side_attach_default_js_position_[0], tc_robot_right_side_attach_default_js_position_[1],tc_robot_right_side_attach_default_js_position_[2],tc_robot_right_side_attach_default_js_position_[3],tc_robot_right_side_attach_default_js_position_[4],tc_robot_right_side_attach_default_js_position_[5]);
    ss.str("");
    ss << "default_detaching_js_position_robot_right_side";
    tc_robot_right_side_detach_default_js_position_ = j_in[ss.str()].get<std::vector<double>>();
    // printf("tc_robot_right_side_detach_default_js_position_: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", tc_robot_right_side_detach_default_js_position_[0], tc_robot_right_side_detach_default_js_position_[1],tc_robot_right_side_detach_default_js_position_[2],tc_robot_right_side_detach_default_js_position_[3],tc_robot_right_side_detach_default_js_position_[4],tc_robot_right_side_detach_default_js_position_[5]);

    //// Slot JS Position
    for(int i = 0; i < cnt_js; i++)
    {
        ss.str("");
        ss << "attaching_js_position";
        ss << std::setw(3) << std::setfill('0') << i+1;
        std::vector<double> vec_attach = j_in[ss.str()].get<std::vector<double>>();
        task_vec_set_tool_changing_attach_JS_position_[i] = vec_attach;
        // printf("task_vec_set_tool_changing_attach_JS_position_%dth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, task_vec_set_tool_changing_attach_JS_position_[i][0], task_vec_set_tool_changing_attach_JS_position_[i][1],task_vec_set_tool_changing_attach_JS_position_[i][2],task_vec_set_tool_changing_attach_JS_position_[i][3],task_vec_set_tool_changing_attach_JS_position_[i][4],task_vec_set_tool_changing_attach_JS_position_[i][5]);

        ss.str("");
        ss << "detaching_js_position";
        ss << std::setw(3) << std::setfill('0') << i+1;
        std::vector<double> vec_detach = j_in[ss.str()].get<std::vector<double>>();
        task_vec_set_tool_changing_detach_JS_position_[i] = vec_detach;
        // printf("task_vec_set_tool_changing_detach_JS_position_%dth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, task_vec_set_tool_changing_detach_JS_position_[i][0], task_vec_set_tool_changing_detach_JS_position_[i][1],task_vec_set_tool_changing_detach_JS_position_[i][2],task_vec_set_tool_changing_detach_JS_position_[i][3],task_vec_set_tool_changing_detach_JS_position_[i][4],task_vec_set_tool_changing_detach_JS_position_[i][5]);

        ss.str("");
        ss << "tool_goal_cs_pose";
        ss << std::setw(3) << std::setfill('0') << i+1;
        std::vector<double> vec_goal_pose = j_in[ss.str()].get<std::vector<double>>();
        task_vec_tool_changing_goal_cs_pose_[i] = vec_goal_pose;
        // printf("task_vec_tool_changing_goal_cs_pose_%dth: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f\n", i+1, task_vec_tool_changing_goal_cs_pose_[i][0], task_vec_tool_changing_goal_cs_pose_[i][1],task_vec_tool_changing_goal_cs_pose_[i][2],task_vec_tool_changing_goal_cs_pose_[i][3],task_vec_tool_changing_goal_cs_pose_[i][4],task_vec_tool_changing_goal_cs_pose_[i][5]);

    }
    // printf("*******************************************\n");

}

void BinPickingNode::setTaskTipChangingJSPosition()
{
    //// FILE PATH
    std::string file_name("bin_picking_node.cpp");
    std::string config_path_ = __FILE__;
    config_path_.resize(config_path_.length() - file_name.length());
    config_path_ += "../../config/";

    std::stringstream ss;
    std::stringstream ss_json;
    ss_json << config_path_ << "tip_changing_pose/tip_changing_js_position.json";
    std::ifstream ifs(ss_json.str().c_str());
    json j_in = json::parse(ifs);
    int cnt_js = j_in["count"];

    //// TCP idx
    task_tcp_idx_set_tip_changing_attach_.clear();
    task_tcp_idx_set_tip_changing_attach_.resize(cnt_js);

    task_vec_set_tip_changing_attach_JS_position_.clear();
    task_vec_set_tip_changing_detach_JS_position_.clear();
    task_vec_set_tip_changing_attach_JS_position_.resize(cnt_js);
    task_vec_set_tip_changing_detach_JS_position_.resize(cnt_js);

    // printf("*******************************************\n");
    ss.str("");
    ss << "tip_changing_tcp_idx_set";
    task_tcp_idx_set_tip_changing_attach_ = j_in[ss.str()].get<std::vector<unsigned int>>();
    // printf("task_tcp_idx_set_tip_changing_attach_: #%u, #%u\n", task_tcp_idx_set_tip_changing_attach_[0], task_tcp_idx_set_tip_changing_attach_[1]);

    //// json file input - target JS angle
    // printf("*******************************************\n");

    //// Default JS Position
    ss.str("");
    ss << "tip_default_attaching_js_position";
    task_vec_tip_changing_default_attach_JS_position_ = j_in[ss.str()].get<std::vector<double>>();
    // printf("task_vec_tip_changing_default_attach_JS_position: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", task_vec_tip_changing_default_attach_JS_position_[0], task_vec_tip_changing_default_attach_JS_position_[1],task_vec_tip_changing_default_attach_JS_position_[2],task_vec_tip_changing_default_attach_JS_position_[3],task_vec_tip_changing_default_attach_JS_position_[4],task_vec_tip_changing_default_attach_JS_position_[5]);
    ss.str("");
    ss << "tip_default_detaching_js_position";
    task_vec_tip_changing_default_detach_JS_position_ = j_in[ss.str()].get<std::vector<double>>();
    // printf("task_vec_tip_changing_default_detach_JS_position_: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", task_vec_tip_changing_default_detach_JS_position_[0], task_vec_tip_changing_default_detach_JS_position_[1],task_vec_tip_changing_default_detach_JS_position_[2],task_vec_tip_changing_default_detach_JS_position_[3],task_vec_tip_changing_default_detach_JS_position_[4],task_vec_tip_changing_default_detach_JS_position_[5]);

    //// Slot JS Position
    for(int i = 0; i < cnt_js; i++)
    {
        ss.str("");
        ss << "tip_attaching_js_position";
        ss << std::setw(3) << std::setfill('0') << i+1;
        std::vector<double> vec_attach = j_in[ss.str()].get<std::vector<double>>();
        task_vec_set_tip_changing_attach_JS_position_[i] = vec_attach;
        // printf("task_vec_set_tip_changing_attach_JS_position_%dth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, task_vec_set_tip_changing_attach_JS_position_[i][0], task_vec_set_tip_changing_attach_JS_position_[i][1],task_vec_set_tip_changing_attach_JS_position_[i][2],task_vec_set_tip_changing_attach_JS_position_[i][3],task_vec_set_tip_changing_attach_JS_position_[i][4],task_vec_set_tip_changing_attach_JS_position_[i][5]);

        ss.str("");
        ss << "tip_detaching_js_position";
        ss << std::setw(3) << std::setfill('0') << i+1;
        std::vector<double> vec_detach = j_in[ss.str()].get<std::vector<double>>();
        task_vec_set_tip_changing_detach_JS_position_[i] = vec_detach;
        // printf("task_vec_set_tip_changing_detach_JS_position_%dth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, task_vec_set_tip_changing_detach_JS_position_[i][0], task_vec_set_tip_changing_detach_JS_position_[i][1],task_vec_set_tip_changing_detach_JS_position_[i][2],task_vec_set_tip_changing_detach_JS_position_[i][3],task_vec_set_tip_changing_detach_JS_position_[i][4],task_vec_set_tip_changing_detach_JS_position_[i][5]);
    }
    // printf("*******************************************\n");

}

/** @brief Korean: 로봇 캘리브레이션(robot calibration)을 위한 로봇 자세 데이터를 입력한다.
 */
void BinPickingNode::setRobotCalibrationUsingLaserTrackerTarget60JSPosition() {
    // robot_calibration_vec_target_JS_position_.clear();
    // std::stringstream ss;

    // //// json file input - target JS angle
    // std::ifstream ifs(std::string("/home/") + USER_NAME + "/robot_control_ws/src/robot_control_ui/robot_control_ui/config/robot_calibration_using_laser_tracker_JS_position.json");
    // json j_in = json::parse(ifs);
    // int cnt_js = j_in["count"];
    // for(int i=0; i<cnt_js; i++)
    // {
    //     ss.str("");
    //     ss << "position";
    //     ss << std::setw(3) << std::setfill('0') << i+1;
    //     std::vector<double> vec_tmp = j_in[ss.str()].get<std::vector<double>>();
    //     robot_calibration_vec_target_JS_position_.push_back(vec_tmp);
    // }

    // //// JS
    // std::vector<std::vector<double>> target_JS = robot_calibration_vec_target_JS_position_;
    // printf("*******************************************\n");
    // printf("**************** Calibration 60 JS Target******************\n");
    // for(int i=0; i < target_JS.size(); i++)
    // {
    //     printf("JS %dth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, target_JS[i][0], target_JS[i][1],target_JS[i][2],target_JS[i][3],target_JS[i][4],target_JS[i][5]);
    // }
    // printf("*******************************************\n");

}

/** @brief Korean: 로봇 캘리브레이션(robot calibration)을 위한 로봇 자세 데이터를 입력한다.
 */
void BinPickingNode::setRobotCalibrationISO9283Pose() {
    // robot_calibration_vec_target_CS_pose_.clear();
    // std::stringstream ss;

    // //// json file input - target JS angle
    // std::ifstream ifs(std::string("/home/") + USER_NAME + "/robot_control_ws/src/robot_control_ui/robot_control_ui/config/ISO9283_CS_command_pose.json");
    // json j_in = json::parse(ifs);
    // int cnt_js = j_in["count"];
    // for(int i=0; i<cnt_js; i++)
    // {
    //     ss.str("");
    //     ss << "pose";
    //     ss << std::setw(3) << std::setfill('0') << i+1;
    //     std::vector<double> vec_tmp = j_in[ss.str()].get<std::vector<double>>();
    //     robot_calibration_vec_target_CS_pose_.push_back(vec_tmp);
    // }

    // //// CS
    // std::vector<std::vector<double>> target_CS = robot_calibration_vec_target_CS_pose_;
    // printf("*******************************************\n");
    // printf("**************** ISO9283 5 CS Target******************\n");
    // for(int i=0; i < target_CS.size(); i++)
    // {
    //     printf("CS %dth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, target_CS[i][0], target_CS[i][1],target_CS[i][2],target_CS[i][3],target_CS[i][4],target_CS[i][5]);
    // }
    // printf("*******************************************\n");
}


/** @brief Korean: 로봇 캘리브레이션(robot calibration)을 위한 로봇 자세 데이터를 입력한다.
 */
void BinPickingNode::setRobotCalibrationUsing3DScannerTarget60JSPosition() {
    // robot_calibration_vec_target_JS_position_.clear();
    // std::stringstream ss;

    // //// json file input - target JS angle
    // std::ifstream ifs(std::string("/home/") + USER_NAME + "/robot_control_ws/src/robot_control_ui/robot_control_ui/config/robot_calibration_using_3D_scanner_JS_position.json");
    // json j_in = json::parse(ifs);
    // int cnt_js = j_in["count"];
    // for(int i=0; i<cnt_js; i++)
    // {
    //     ss.str("");
    //     ss << "position";
    //     ss << std::setw(3) << std::setfill('0') << i+1;
    //     std::vector<double> vec_tmp = j_in[ss.str()].get<std::vector<double>>();
    //     robot_calibration_vec_target_JS_position_.push_back(vec_tmp);
    // }

    // //// JS
    // std::vector<std::vector<double>> target_JS = robot_calibration_vec_target_JS_position_;
    // printf("*******************************************\n");
    // printf("**************** (Using 3D Scanner)Calibration 60 JS Target******************\n");
    // for(int i=0; i < target_JS.size(); i++)
    // {
    //     printf("JS %dth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, target_JS[i][0], target_JS[i][1],target_JS[i][2],target_JS[i][3],target_JS[i][4],target_JS[i][5]);
    // }
    // printf("*******************************************\n");
}

/** @brief Korean: 로봇 캘리브레이션(robot calibration)을 위한 로봇 자세 데이터를 입력한다.
 */
void BinPickingNode::setZIVIDGlobalCalibrationTargetJSPosition() {
    // robot_calibration_vec_target_JS_position_.clear();
    // std::stringstream ss;

    // //// json file input - target JS angle
    // std::ifstream ifs(std::string("/home/") + USER_NAME + "/robot_control_ws/src/robot_control_ui/robot_control_ui/config/zivid_global_calibration_JS_position.json");
    // json j_in = json::parse(ifs);
    // int cnt_js = j_in["count"];
    // for(int i=0; i<cnt_js; i++)
    // {
    //     ss.str("");
    //     ss << "position";
    //     ss << std::setw(3) << std::setfill('0') << i+1;
    //     std::vector<double> vec_tmp = j_in[ss.str()].get<std::vector<double>>();
    //     robot_calibration_vec_target_JS_position_.push_back(vec_tmp);
    // }

    // //// JS
    // std::vector<std::vector<double>> target_JS = robot_calibration_vec_target_JS_position_;
    // printf("*******************************************\n");
    // printf("**************** Calibration 60 JS Target******************\n");
    // for(int i=0; i < target_JS.size(); i++)
    // {
    //     printf("JS %dth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, target_JS[i][0], target_JS[i][1],target_JS[i][2],target_JS[i][3],target_JS[i][4],target_JS[i][5]);
    // }
    // printf("*******************************************\n");
}

//// NOTICE: Rx, Rz는 fabs() 처리되어있음.
//// NOTICE: Rx, Rz는 fabs() 처리되어있음.
//// NOTICE: Rx, Rz는 fabs() 처리되어있음.
bool BinPickingNode::checkWorkspace(const std::vector<double> pose, const Eigen::MatrixXd &workspace)
{
    return true;

    double x_min = workspace(0, 0); double x_max = workspace(0, 1); // m
    double y_min = workspace(1, 0); double y_max = workspace(1, 1); // m
    double z_min = workspace(2, 0); double z_max = workspace(2, 1);// m
    double rx_min = workspace(3, 0); double rx_max = workspace(3, 1); // deg
    double ry_min = workspace(4, 0); double ry_max = workspace(4, 1); // deg
    double rz_min = workspace(5, 0); double rz_max = workspace(5, 1); // deg

    if ((pose[0] > x_min && pose[0] < x_max) &&
        (pose[1] > y_min && pose[1] < y_max) &&
        (pose[2] > z_min && pose[2] < z_max) &&
        (fabs(pose[3]) > rx_min && fabs(pose[3]) < rx_max) &&
        (pose[4] > ry_min && pose[4] < ry_max) &&
        (fabs(pose[5]) > rz_min && fabs(pose[5]) < rz_max)) {
        ROS_LOG_INFO("Check the pose in the workspace - OK");
        return true;
    } else {
        ROS_LOG_WARN("Check the pose in the workspace - out of range");
        ROS_LOG_WARN("pose: %0.4f, %0.4f, %0.4f, %0.1f, %0.1f, %0.1f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
        return false;
    }
}

// void BinPickingNode::templateMatchingCallback(const hanyang_matching_msgs::msg::MatchingResultMsg &msg) {

//     if(msg.is_pose)
//     {
//         ROS_LOG_INFO("************************************************************");
//         ROS_LOG_INFO("************************************************************");
//         ROS_LOG_INFO("************************************************************");
//         ROS_LOG_INFO("Topic Subscribe!!! - templateMatchingCallback received! (is_pose: true)");
//         ROS_LOG_INFO("pose feasible!");
//         ROS_LOG_INFO("************************************************************");
//         ROS_LOG_INFO("************************************************************");
//         ROS_LOG_INFO("************************************************************");

//         detected_pose_ = msg.pose;
//         detected_sub_pose_ = msg.sub_pose;
//         approach_distance = msg.approach_distance;
//         matching_accuracy = msg.matching_accuracy;
//         matching_accuracy_limit = msg.matching_accuracy_limit;
//         m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_open_length = msg.gripper_open_length;
//         m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_close_length = msg.gripper_close_length;
//         detected_mask_num = msg.detected_mask_num;
//         bool is_grasping_pose_flipped = msg.is_grasping_pose_flipped;
//         is_grasping_pose_cal_finished = true;

//         printf("\n---------------------------------------------------\n");
//         printf("------- *** (Topic subscribe) ***  --------\n");
//         printf("------- *** Template matching results ***  --------\n");
//         printf("---------------------------------------------------\n");
//         ROS_LOG_INFO("Grasping pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", detected_pose_[0], detected_pose_[1], detected_pose_[2], detected_pose_[3], detected_pose_[4], detected_pose_[5]);
//         ROS_LOG_INFO("Grasping sub. pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", detected_sub_pose_[0], detected_sub_pose_[1], detected_sub_pose_[2], detected_sub_pose_[3], detected_sub_pose_[4], detected_sub_pose_[5]);
//         ROS_LOG_INFO("Approach distance: %0.3f", approach_distance);
//         ROS_LOG_INFO("Matching accuracy: %0.3f", matching_accuracy);
//         ROS_LOG_INFO("Matching accuracy limit: %0.3f", matching_accuracy_limit);
//         ROS_LOG_INFO("Gripper_open_length: %u", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_open_length);
//         ROS_LOG_INFO("Gripper_close_length: %u", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_close_length);
//         ROS_LOG_INFO("Detected mask count: %i", detected_mask_num);
//         if(is_grasping_pose_flipped) printf("******** ******** Pose flipped! ******** ********\n");
//         printf("---------------------------------------------------\n");
//         printf("---------------------------------------------------\n");
//         printf("---------------------------------------------------\n\n");
//         ROS_LOG_INFO("Template Matching Success!(Callback11)");
//         is_matching_finished = true;
//     }
//     else
//     {
//         approach_distance = 0.0;
//         matching_accuracy = 0.0;
//         detected_mask_num = msg.detected_mask_num;
//         ROS_LOG_INFO("Detected mask count: %i", detected_mask_num);
//         ROS_LOG_INFO("Template Matching Failure!(Callback): Scanning Recovery task!");

//         ROS_LOG_INFO("************************************************************");
//         ROS_LOG_INFO("************************************************************");
//         ROS_LOG_INFO("************************************************************");
//         ROS_LOG_INFO("Topic Subscribe!!! - templateMatchingCallback received! (is_pose: false)");
//         ROS_LOG_INFO("pose infeasible!");
//         ROS_LOG_INFO("************************************************************");
//         ROS_LOG_INFO("************************************************************");
//         ROS_LOG_INFO("************************************************************");
//         is_grasping_pose_cal_finished = false;
//         is_matching_finished = false;
//     }
// }

//// 3D scanning
bool BinPickingNode::scanZIVID(taskScanningParameter &parameter)
{
    auto req = make_shared<hanyang_matching_msgs::srv::ZividDoScan::Request>();
    req->target_id = parameter.target_id; // 추후 task planner와 통합 필요 // 1: cpu, 2: ram, 3: hdmi, 4: usb, 5: power, 6: ssd, 7: cooler
    req->target_name = parameter.target_name;
    req->is_mask_pixel_fixed = parameter.is_mask_pixel_fixed;
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->sampling_num = parameter.sampling_num;
    req->is_base_frame_unknown = parameter.is_base_frame_unknown;
    req->do_save_data = parameter.do_save_data;
    req->do_save_mrcnn_learning_data = parameter.do_save_mrcnn_learning_data;
    req->do_load_data = parameter.do_not_scan_do_load_data;
    req->skip_detection_mask = parameter.skip_detection_mask;
    req->do_image_processing = parameter.do_image_processing;
    req->do_single_matching = parameter.do_single_matching;

    // Only using fixed mask pixel without using mask rcnn
    req->mask_pixel_x = parameter.mask_pixel_list[0];
    req->mask_pixel_y = parameter.mask_pixel_list[1];
    req->mask_pixel_scale_x = parameter.mask_pixel_list[2];
    req->mask_pixel_scale_y = parameter.mask_pixel_list[3];
    req->learning_data_idx = parameter.learning_data_idx;
    req->scan_position = parameter.scan_position; // scanning joint position
    ROS_LOG_INFO("Do scanning! (grasping parts id: %d)", parameter.target_id);

    //// 241125 추가
    req->robot_dh_parameters = parameter.robot_dh_vec; // [m], [deg]
    req->robot_tcp_default = parameter.robot_tcp_default; // [m], [deg]
    req->robot_tcp = parameter.robot_tcp; // [m], [deg]

    //// 241125 추가
    std::vector<double> robot_dh_vec = parameter.robot_dh_vec; // [m], [deg]
    std::vector<double> robot_tcp_default = parameter.robot_tcp_default; // [m], [deg]
    std::vector<double> robot_tcp = parameter.robot_tcp; // [m], [deg]
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



    while (!zividScanningClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }

    auto result = zividScanningClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        if (recv->is_detected == true) {
            ROS_LOG_INFO("Scanning Success!(Callback)");
            return true;
        } else {
           ROS_LOG_INFO("Scanning Failure!(Callback)");
            return false;
        }
    } else {
        ROS_LOG_INFO("Scanning Failure!(Callback)");
    }
}

bool BinPickingNode::doScanning(taskScanningParameter &parameter)
{
    ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!Scanning Start!!!!!!!!!!!!!!!!!!!!");
    if(scanZIVID(parameter)) {
        return true;
    } else {
        return false;
    }
}

bool BinPickingNode::doScanningAndMatching(taskScanningParameter &scan_param, taskTemplateMatchingParameter &matching_param)
{
    ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! TASK_3D_SCANNING_ZIVID_AND_MATCHING Start!!!!!!!!!!!!!!!!!!!!");
    if(scanZIVID(scan_param)) {
        //// Publish template matching command
        doCADMatchingTopic(matching_param);
        ROS_LOG_INFO("Template matching topic published!!!");
        ROS_LOG_INFO("Template matching topic published!!!");
        ROS_LOG_INFO("Template matching topic published!!!");
        ROS_LOG_INFO("Template matching topic published!!!");
        ROS_LOG_INFO("Template matching topic published!!!");
        ROS_LOG_INFO("Template matching topic published!!!");
        ROS_LOG_INFO("Template matching topic published!!!");
        ROS_LOG_INFO("Template matching topic published!!!");
        ROS_LOG_INFO("Template matching topic published!!!");
        ROS_LOG_INFO("Template matching topic published!!!");
        ROS_LOG_INFO("Template matching topic published!!!");
        return true;
    } else {
        return false;
    }
}

void BinPickingNode::doCADMatchingTopic(taskTemplateMatchingParameter &parameter)
{
    hanyang_matching_msgs::msg::DoTemplateMatchingMsg msg;
    msg.do_scan_sampling = parameter.do_scan_sampling;
    msg.debug_mode = parameter.debug_mode;
    msg.sampling_num = parameter.sampling_num;
    msg.is_base_frame_unknown = parameter.is_base_frame_unknown;
    msg.robot_id = m_robot_id;
    is_matching_finished = false; // edited py taek
    ROS_LOG_INFO("Publish topic - Matching Command (bin picking)!");

    doTemplateMatchingBinPickingPublisher_->publish(msg);
}

bool BinPickingNode::doCADMatchingService(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();

    std::string object_name = parameter.target_name;

    ROS_LOG_WARN("[%s] Target Object Name: %s", __func__, object_name);
    ROS_LOG_WARN("[%s] Target Object Name: %s", __func__, object_name);
    ROS_LOG_WARN("[%s] Target Object Name: %s", __func__, object_name);
    // Explicitly pass target identification to matching node
    req->target_name = object_name;
    req->target_id = parameter.target_id;
    req->do_scan_sampling = parameter.do_scan_sampling;

    req->debug_mode = parameter.debug_mode;
    req->sampling_num = parameter.sampling_num;
    req->is_base_frame_unknown = parameter.is_base_frame_unknown;

    req->is_symmetric = parameter.is_symmetric;

    req->robot_id = m_robot_id;
    is_matching_finished = false;

    ROS_LOG_INFO("[%s] Do template matching (bin picking)!", __func__);
    while (!doTemplateMatchingBinPickingClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        approach_distance = 0.0;
        matching_accuracy = 0.0;
        is_matching_finished = false;
        return false;
    }

    auto result = doTemplateMatchingBinPickingClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        is_matching_finished = true;
        auto recv = result.get();
        if (recv->is_pose == true) {
            detected_pose_ = recv->pose;
            map_object_name_detected_pose_[object_name] = detected_pose_; // 250902
            detected_sub_pose_ = recv->sub_pose;
            map_object_name_detected_sub_pose_[object_name] = detected_sub_pose_; // 250902
            detected_zig_pose_ = recv->zig_pose;
            approach_distance = recv->approach_distance;
            map_object_name_approach_distance_[object_name] = approach_distance; // 250902
            matching_accuracy = recv->matching_accuracy;
            matching_accuracy_limit = recv->matching_accuracy_limit;
            m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_open_length = recv->gripper_open_length;
            m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_close_length = recv->gripper_close_length;
            m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_tip_index = recv->gripper_tip_index;
            detected_mask_num = recv->detected_mask_num;
            bool is_grasping_pose_flipped = recv->is_grasping_pose_flipped;
            is_grasping_pose_cal_finished = true;
            is_grasping_pose_feasible_ = true; // Pose Feasible 여부

            printf("\n---------------------------------------------------\n");
            printf("------- *** Template matching results ***  --------\n");
            printf("---------------------------------------------------\n");
            ROS_LOG_INFO("Grasping pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", detected_pose_[0], detected_pose_[1], detected_pose_[2], detected_pose_[3], detected_pose_[4], detected_pose_[5]);
            ROS_LOG_INFO("Grasping sub. pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", detected_sub_pose_[0], detected_sub_pose_[1], detected_sub_pose_[2], detected_sub_pose_[3], detected_sub_pose_[4], detected_sub_pose_[5]);
            ROS_LOG_INFO("Approach distance: %0.3f", approach_distance);
            ROS_LOG_INFO("Matching accuracy: %0.3f", matching_accuracy);
            ROS_LOG_INFO("Matching accuracy limit: %0.3f", matching_accuracy_limit);
            ROS_LOG_INFO("Gripper_open_length: %u", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_open_length);
            ROS_LOG_INFO("Gripper_close_length: %u", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_close_length);
            ROS_LOG_INFO("gripper_tip_index: #%u", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_tip_index);
            ROS_LOG_INFO("Detected mask count: %i", detected_mask_num);
            if(is_grasping_pose_flipped) printf("******** ******** Pose flipped! ******** ********\n");
            printf("---------------------------------------------------\n");
            ROS_LOG_INFO("Zig pose: %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f", detected_zig_pose_[0], detected_zig_pose_[1], detected_zig_pose_[2], detected_zig_pose_[3], detected_zig_pose_[4], detected_zig_pose_[5]);
            printf("---------------------------------------------------\n");
            printf("---------------------------------------------------\n");
            printf("---------------------------------------------------\n\n");
            ROS_LOG_INFO("Template Matching Success!(Callback11)");
            return true;
        } else {
            is_grasping_pose_cal_finished = true;
            is_grasping_pose_feasible_ = false; // Pose Feasible 여부

            approach_distance = 0.0;
            matching_accuracy = 0.0;
            detected_mask_num = recv->detected_mask_num;
            ROS_LOG_INFO("Detected mask count: %i", detected_mask_num);
            ROS_LOG_INFO("Template Matching Failure!(Callback): Scanning Recovery task!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        is_grasping_pose_cal_finished = true;
        is_grasping_pose_feasible_ = false; // Pose Feasible 여부
        approach_distance = 0.0;
        matching_accuracy = 0.0;
        is_matching_finished = false;
        return false;
    }
}

bool BinPickingNode::doPoseEstimationAI(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoPoseEstimation::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;
    std::string object_name = parameter.object_name;

    ROS_LOG_INFO("Do Pose Estimation (bin picking)!");
    while (!doPoseEstimationBinPickingClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        approach_distance = 0.0;
        matching_accuracy = 0.0;
        return false;
    }

    auto result = doPoseEstimationBinPickingClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        if (recv->is_pose == true) {
            estimated_pose_ = recv->pose;
            map_object_name_estimated_pose_[object_name] = estimated_pose_;
            estimated_sub_pose_ = recv->sub_pose;
            approach_distance = recv->approach_distance;
            grp_width = 1000.0*recv->grp_width;
            double grp_scale = 180.0*(79.5-sqrt(80.0*80.0-((grp_width+21.0)*(grp_width+21.0))/4.0 ));
            if(0)
            {
                m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_open_length = static_cast<uint16_t>(grp_scale);
            }
            else
            {
                m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_open_length = 9500; // open
            }
            is_grasping_pose_cal_finished = true;
            printf("\n---------------------------------------------------\n");
            printf("---------- *** Pose estimation results ***  -------\n");
            printf("---------------------------------------------------\n");
            ROS_LOG_INFO("Grasping pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", estimated_pose_[0], estimated_pose_[1], estimated_pose_[2], estimated_pose_[3], estimated_pose_[4], estimated_pose_[5]);
            ROS_LOG_INFO("Grasping sub. pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", estimated_sub_pose_[0], estimated_sub_pose_[1], estimated_sub_pose_[2], estimated_sub_pose_[3], estimated_sub_pose_[4], estimated_sub_pose_[5]);
            ROS_LOG_INFO("Approach distance: %0.3f", approach_distance);
            ROS_LOG_INFO("Gripper width: %0.3f", grp_width);
            ROS_LOG_INFO("Gripper open_length: %i", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_open_length);
            // ROS_LOG_INFO("Matching accuracy: %0.3f", matching_accuracy);
            printf("---------------------------------------------------\n");
            printf("---------------------------------------------------\n");
            printf("---------------------------------------------------\n\n");
            ROS_LOG_INFO("Pose Estimation Success!(Callback11)");
            return true;
        } else {
            approach_distance = 0.0;
            matching_accuracy = 0.0;
            ROS_LOG_INFO("Pose Estimation Failure!(Callback): Scanning Recovery task!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        approach_distance = 0.0;
        matching_accuracy = 0.0;
        return false;
    }
}

void BinPickingNode::publishUICommandLearningMsg(taskScanningParameter &parameter)
{
    hanyang_matching_msgs::msg::BpUiCommandLearning ui_cmd;
    ui_cmd.target_name = parameter.target_name;
    // ui_cmd.weight_number = parameter.weight_number;
    // ui_cmd.is_sam_mean_size_assigned = parameter.is_sam_mean_size_assigned;
    // ui_cmd.sam_mean_size = parameter.sam_mean_size;
    // ui_cmd.sam_mask_min_area = parameter.sam_mask_min_area;
    // ui_cmd.sam_mask_max_area = parameter.sam_mask_max_area;
    uiCommandLearningPublisher_->publish(ui_cmd);
}


bool BinPickingNode::doTemplateInitialize(taskTemplateMatchingParameter &parameter, bool do_set_only_parameters, bool do_overwrite_JSON) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    req->robot_dh_parameters = parameter.robot_dh_vec;
    req->robot_tcp_default = parameter.robot_tcp_default;
    req->robot_tcp = parameter.robot_tcp;

    req->do_set_only_parameters = do_set_only_parameters;
    req->do_overwrite_json = do_overwrite_JSON;

    if(!do_set_only_parameters) {
        ROS_LOG_INFO("Do Template Initialize! (target_id: #%li)", parameter.target_id);
    } else {
        ROS_LOG_INFO("Set Template Parameters! (target_id: #%li)", parameter.target_id);
    }

    while (!doTemplateInitializeClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }

    auto result = doTemplateInitializeClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        ROS_LOG_INFO("code here4!");
        // is_template_initialized = true;
        ROS_LOG_INFO("Template Initialize Success! (target_id: #%li)", parameter.target_id);
        return true;
    } else {
        ROS_LOG_INFO("code here5!");
        ROS_LOG_INFO("Template Initialize Failure! (target_id: #%li)", parameter.target_id);
        return false;
    }
}

bool BinPickingNode::doBinDetectionTemplateMatching(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->debug_mode = parameter.debug_mode;
    req->custom_transformation_pose = parameter.custom_transformation_pose;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    req->robot_dh_parameters = parameter.robot_dh_vec;
    req->robot_tcp_default = parameter.robot_tcp_default;
    req->robot_tcp = parameter.robot_tcp;

    ROS_LOG_INFO("Do Bin Template matching!");
    is_matching_finished = false;
    while (!doBinTemplateMatchingClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        is_matching_finished = false;
        return false;
    }
    auto result = doBinTemplateMatchingClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        is_matching_finished = true;
        if (recv->is_pose == true) {
            parameter.matching_accuracy = recv->matching_accuracy;
            ROS_LOG_INFO("Bin Template Matching Success!(Callback)");
            ROS_LOG_INFO("Matching accuracy: %0.3f", parameter.matching_accuracy);
            return true;
        } else {
            ROS_LOG_INFO("Bin Template Matching Failure!(Callback)!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        is_matching_finished = false;
        return false;
    }
}

bool BinPickingNode::doBaseZigDetectionTemplateMatching(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->debug_mode = parameter.debug_mode;
    req->custom_transformation_pose = parameter.custom_transformation_pose;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    req->robot_dh_parameters = parameter.robot_dh_vec;
    req->robot_tcp_default = parameter.robot_tcp_default;
    req->robot_tcp = parameter.robot_tcp;

    ROS_LOG_INFO("Do Base Zig Template matching!");
    is_matching_finished = false;
    while (!doBaseZigTemplateMatchingClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        is_matching_finished = false;
        return false;
    }
    auto result = doBaseZigTemplateMatchingClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        is_matching_finished = true;
        if (recv->is_pose == true) {
            parameter.matching_accuracy = recv->matching_accuracy;
            ROS_LOG_INFO("Base Zig Template Matching Success!(Callback)");
            ROS_LOG_INFO("Matching accuracy: %0.3f", parameter.matching_accuracy);
            return true;
        } else {
            ROS_LOG_INFO("Base Zig Template Matching Failure!(Callback)!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        is_matching_finished = false;
        return false;
    }
}

bool BinPickingNode::doEyeInHandDetectionTemplateMatching(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->debug_mode = parameter.debug_mode;
    req->custom_transformation_pose = parameter.custom_transformation_pose;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    req->robot_dh_parameters = parameter.robot_dh_vec;
    req->robot_tcp_default = parameter.robot_tcp_default;
    req->robot_tcp = parameter.robot_tcp;


    req->r2cam_calibration_js_position = parameter.robot_camera_calibration_js_position; // scanning joint position
    std::vector<double> scan_position_q = parameter.robot_camera_calibration_js_position;
    ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);

    //// Current pose (tcp - tool master)
    std::vector<double> pose_b2e_measured = parameter.pose_B2E_measured; // [m], [deg]
    req->pose_b2e_measured = pose_b2e_measured;
    printVector(pose_b2e_measured, "pose_b2e_measured");


    ROS_LOG_INFO("Do Eye-in-hand Detection Template matching!");
    is_matching_finished = false;
    while (!doEyeInHandTemplateMatchingClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        is_matching_finished = false;
        return false;
    }
    auto result = doEyeInHandTemplateMatchingClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        is_matching_finished = true;
        if (recv->is_pose == true) {
            parameter.matching_accuracy = recv->matching_accuracy;
            ROS_LOG_INFO("Eye-in-hand Detection Template Matching Success!(Callback)");
            ROS_LOG_INFO("Matching accuracy: %0.3f", parameter.matching_accuracy);
            return true;
        } else {
            ROS_LOG_INFO("Eye-in-hand Detection Template Matching Failure!(Callback)!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        is_matching_finished = false;
        return false;
    }
}

bool BinPickingNode::doOptimizationBinPickingWorkspace(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->debug_mode = parameter.debug_mode;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    req->robot_dh_parameters = parameter.robot_dh_vec;
    req->robot_tcp_default = parameter.robot_tcp_default;
    req->robot_tcp = parameter.robot_tcp;

    ROS_LOG_INFO("Do Optimization Bin Picking Workspace (bin picking)!");
    is_matching_finished = false;

    while (!doOptimizationBinPickingWorkspaceClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        approach_distance = 0.0;
        matching_accuracy = 0.0;
        is_matching_finished = false;
        return false;
    }
    auto result = doOptimizationBinPickingWorkspaceClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        is_matching_finished = true;
        auto recv = result.get();
        if (recv->is_pose == true) {
            detected_pose_ = recv->pose;
            detected_sub_pose_ = recv->sub_pose;
            approach_distance = recv->approach_distance;
            matching_accuracy = recv->matching_accuracy;
            matching_accuracy_limit = recv->matching_accuracy_limit;
            m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_open_length = recv->gripper_open_length;
            m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_close_length = recv->gripper_close_length;
            detected_mask_num = recv->detected_mask_num;
            bool is_grasping_pose_flipped = recv->is_grasping_pose_flipped;
            is_grasping_pose_cal_finished = true;

            printf("\n---------------------------------------------------\n");
            printf("------- *** Template matching results ***  --------\n");
            printf("---------------------------------------------------\n");
            ROS_LOG_INFO("Grasping pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", detected_pose_[0], detected_pose_[1], detected_pose_[2], detected_pose_[3], detected_pose_[4], detected_pose_[5]);
            ROS_LOG_INFO("Grasping sub. pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", detected_sub_pose_[0], detected_sub_pose_[1], detected_sub_pose_[2], detected_sub_pose_[3], detected_sub_pose_[4], detected_sub_pose_[5]);
            ROS_LOG_INFO("Approach distance: %0.3f", approach_distance);
            ROS_LOG_INFO("Matching accuracy: %0.3f", matching_accuracy);
            ROS_LOG_INFO("Matching accuracy limit: %0.3f", matching_accuracy_limit);
            ROS_LOG_INFO("Gripper_open_length: %u", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_open_length);
            ROS_LOG_INFO("Gripper_close_length: %u", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_close_length);
            ROS_LOG_INFO("Detected mask count: %i", detected_mask_num);
            if(is_grasping_pose_flipped) printf("******** ******** Pose flipped! ******** ********\n");
            printf("---------------------------------------------------\n");
            printf("---------------------------------------------------\n");
            printf("---------------------------------------------------\n\n");
            ROS_LOG_INFO("Bin Optimization Success!(Callback11)");
            return true;
        } else {
            approach_distance = 0.0;
            matching_accuracy = 0.0;
            detected_mask_num = recv->detected_mask_num;
            ROS_LOG_INFO("Detected mask count: %i", detected_mask_num);
            ROS_LOG_INFO("Bin Optimization Failure!(Callback): Scanning Recovery task!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        approach_distance = 0.0;
        matching_accuracy = 0.0;
        is_matching_finished = false;
        return false;
    }
}

bool BinPickingNode::doGetRobot2CameraCalibrationJSPosition(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->debug_mode = parameter.debug_mode;
    // req->custom_transformation_pose = parameter.custom_transformation_pose;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    req->robot_dh_parameters = parameter.robot_dh_vec;
    req->robot_tcp_default = parameter.robot_tcp_default;
    req->robot_tcp = parameter.robot_tcp;

    req->is_r2cam_calibration_js_position_set_by_teaching = parameter.is_r2cam_calibration_js_position_set_by_teaching;

    if(parameter.is_r2cam_calibration_js_position_set_by_teaching) {
        req->r2cam_calibration_js_position = parameter.robot_camera_calibration_js_position;
    }


    printf("\n\n\nDH: ");
    for (int i = 0; i < 6; i++) {
        for(int j = 0; j < 4; j++) {  // [m], [rad] -- a, alpha, d, theta
            printf("%0.6f ", parameter.robot_dh_vec[i * 4 + j]);
        }
        printf("\n");
    }
    printf("\n\n\nRobot TCP default[m, rad]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n\n\n", parameter.robot_tcp_default[0], parameter.robot_tcp_default[1], parameter.robot_tcp_default[2], parameter.robot_tcp_default[3], parameter.robot_tcp_default[4], parameter.robot_tcp_default[5] );
    printf("\n\n\nRobot TCP[m, rad]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n\n\n", parameter.robot_tcp[0], parameter.robot_tcp[1], parameter.robot_tcp[2], parameter.robot_tcp[3], parameter.robot_tcp[4], parameter.robot_tcp[5] );

    ROS_LOG_INFO("Do Get Robot to Calibration JS Position!");
    is_matching_finished = false;

    while (!doGetRobot2CameraCalJSPositionClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = doGetRobot2CameraCalJSPositionClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        is_matching_finished = true;
        auto recv = result.get();
        if (recv->is_pose == true) {
            parameter.robot_camera_calibration_js_position = recv->robot_camera_calibration_js_position;
            ROS_LOG_INFO("Get Robot to Calibration JS Position Success!(Callback)");
            printVector(parameter.robot_camera_calibration_js_position, "robot_camera_calibration_js_position");
            return true;
        } else {
            ROS_LOG_INFO("Get Robot to Calibration JS Position Failure!(Callback)!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

bool BinPickingNode::doRobot2CameraCalibrationTemplateMatching(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->debug_mode = parameter.debug_mode;
    req->custom_transformation_pose = parameter.custom_transformation_pose;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    req->robot_dh_parameters = parameter.robot_dh_vec;
    req->robot_tcp_default = parameter.robot_tcp_default;
    req->robot_tcp = parameter.robot_tcp;

    //// Current pose (tcp - tool master)
    std::vector<double> pose_b2e_measured = parameter.pose_B2E_measured; // [m], [deg]
    req->pose_b2e_measured = pose_b2e_measured;
    printVector(pose_b2e_measured, "pose_b2e_measured");

    ROS_LOG_INFO("Do Robot to Calibration Template matching!");
    is_matching_finished = false;

    while (!doRobot2CameraCalTemplateMatchingClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = doRobot2CameraCalTemplateMatchingClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        is_matching_finished = true;
        auto recv = result.get();
        if (recv->is_pose == true) {
            parameter.matching_accuracy = recv->matching_accuracy;
            ROS_LOG_INFO("Robot to Calibration Template Matching Success!(Callback)");
            ROS_LOG_INFO("Matching accuracy: %0.3f", parameter.matching_accuracy);
            return true;
        } else {
            ROS_LOG_INFO("Robot to Calibration Template Matching Failure!(Callback)!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

bool BinPickingNode::doEvaluateGraspingPose(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->debug_mode = parameter.debug_mode;
    req->custom_transformation_pose = parameter.custom_transformation_pose;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    req->robot_dh_parameters = parameter.robot_dh_vec;
    req->robot_tcp_default = parameter.robot_tcp_default;
    req->robot_tcp = parameter.robot_tcp;

    //// Current pose (tcp - tool master)
    std::vector<double> pose_b2e_measured = parameter.pose_B2E_measured;
    for(int i = 0; i < 3; i++) pose_b2e_measured[i + 3] *= kRad2Deg;
    req->pose_b2e_measured = pose_b2e_measured;
    printVector(pose_b2e_measured, "pose_b2e_measured");

    ROS_LOG_INFO("Evaluate grasping pose!");
    is_matching_finished = false;

    while (!doEvaluateGraspingPoseClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = doEvaluateGraspingPoseClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        is_matching_finished = true;
        auto recv = result.get();
        if (recv->is_pose == true) {
            ROS_LOG_INFO("Evaluate Grasping Pose Success!(Callback)");
            return true;
        } else {
            ROS_LOG_INFO("Evaluate Grasping Pose Failure!(Callback)!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

bool BinPickingNode::doZigPoseInitialTeaching(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->debug_mode = parameter.debug_mode;
    req->custom_transformation_pose = parameter.custom_transformation_pose;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    req->robot_dh_parameters = parameter.robot_dh_vec;
    req->robot_tcp_default = parameter.robot_tcp_default;
    req->robot_tcp = parameter.robot_tcp;

    // //// Current pose (tcp - tool master)
    // std::vector<double> pose_b2e_measured = parameter.pose_B2E_measured;
    // for(int i = 0; i < 3; i++) pose_b2e_measured[i + 3] *= kRad2Deg;
    // req->pose_b2e_measured = pose_b2e_measured;
    // printVector(pose_b2e_measured, "pose_b2e_measured");

    ROS_LOG_INFO("Zig Pose Initial Teaching Process!");
    is_matching_finished = false;

    while (!doZigPoseInitialTeachingClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = doZigPoseInitialTeachingClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        is_matching_finished = true;
        auto recv = result.get();
        if (recv->is_pose == true) {
            ROS_LOG_INFO("Zig Pose Initial Teaching Process Success!(Callback)");
            return true;
        } else {
            ROS_LOG_INFO("Zig Pose Initial Teaching Process Failure!(Callback)!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

bool BinPickingNode::doSimulationBinPlacing(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->debug_mode = parameter.debug_mode;
    req->custom_transformation_pose = parameter.custom_transformation_pose;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    req->robot_dh_parameters = parameter.robot_dh_vec;
    req->robot_tcp_default = parameter.robot_tcp_default;
    req->robot_tcp = parameter.robot_tcp;

    req->do_cloud_clear = parameter.do_cloud_clear;

    ROS_LOG_INFO("Do Simulation Bin Placing!");
    is_matching_finished = false;

    while (!doSimulationBinPlacingClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = doSimulationBinPlacingClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        is_matching_finished = true;
        auto recv = result.get();
        if (recv->is_pose == true) {
            ROS_LOG_INFO("Do Simulation Bin Placing - cloud clear, Success!(Callback)");
            return true;
        } else {
            ROS_LOG_INFO("Do Simulation Bin Placing - cloud clear, Failure!(Callback)!");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

bool BinPickingNode::doGetEnvParameters(taskTemplateMatchingParameter &parameter)
{
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    // Target id
    req->target_id = parameter.target_id;
    ROS_LOG_INFO("Do Get Env. Parameters!");

    while (!doGetEnvParametersClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = doGetEnvParametersClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        std::vector<double> tmp = recv->robot_camera_calibration_htm_b2s_global;
        printVector(tmp, "T_B2S_global");
        parameter.htm_vec_B2S_global = tmp;
        ROS_LOG_INFO("doGetEnvParameters - service call Success!(Callback)");
        return true;
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

bool BinPickingNode::doGrasping(std::vector<double> &output_pose, enum GraspingPoseType pose_type, std::string demo_name, std::string object_name)
{
    try
    {
        /////////////////////////////////////////////////////////////////////////
        bin_scan_fail = false;
        // Fast-path: if we already have a stored pose for this object, allow grasping without flags
        if (object_name != "default" && map_object_name_detected_pose_.count(object_name) > 0) {
            is_grasping_pose_cal_finished = true;
            is_grasping_pose_feasible_ = true;
        }

        if(is_grasping_pose_cal_finished && is_grasping_pose_feasible_) { // Template matching results
            std::vector<double> pose;
            switch(pose_type) {
                case GraspingPoseType::OPTIMAL_POSE: {
                    ROS_LOG_INFO("Do Grasping parts!! (Bin Picking)");
                    if(object_name == "default") {
                        pose = getDetectedPose(); // Grasping pose
                        ROS_LOG_INFO("Detected pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                    } else {
                        pose = map_object_name_detected_pose_[object_name];
                        ROS_LOG_WARN("MapPose[%s] Detected pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", object_name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                    }
                    ROS_LOG_INFO("Matching accuracy: %0.3f", matching_accuracy);
                    break;
                }
                case GraspingPoseType::SUB_OPTIMAL_POSE: {
                    ROS_LOG_INFO("Do Sub Grasping parts!! (Bin Picking)");
                    if(object_name == "default") {
                        pose = getDetectedSubPose(); // Sub-grasping pose
                        ROS_LOG_INFO("Detected sub. pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                    } else {
                        pose = map_object_name_detected_sub_pose_[object_name];
                        ROS_LOG_WARN("MapPose[%s] Detected sub. pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", object_name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                    }
                    ROS_LOG_INFO("Matching accuracy: %0.3f", matching_accuracy);
                    break;
                }
                case GraspingPoseType::AI_ESTIMATED_POSE: {
                    ROS_LOG_INFO("Do Grasping parts!! (Bin Picking - Pose Estimation)");
                    if(object_name == "default") {
                        pose = getEstimatedPose(); // AI estimated pose
                        ROS_LOG_INFO("AI Estimated pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                    } else {
                        pose = map_object_name_estimated_pose_[object_name];
                        ROS_LOG_WARN("MapPose[%s] AI Estimated pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", object_name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                    }
                    break;
                }
                default: {
                    assert(false);
                    break;
                }
            }

            Eigen::MatrixXd workspace(6,2); // pose limit - (min, max)
            if(demo_name != "default") {

                if(demo_name == "drum_task") {
                    ROS_LOG_WARN("[%s] TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING_DRUM_TASK", __func__);
                    ROS_LOG_WARN("[%s] TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING_DRUM_TASK", __func__);
                    ROS_LOG_WARN("[%s] TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING_DRUM_TASK", __func__);

                    workspace(0, 0) = -1.0;
                    workspace(0, 1) = 1.0;
                    workspace(1, 0) = -1.0;
                    workspace(1, 1) = 1.0;
                    workspace(2, 0) = -1.0;
                    workspace(2, 1) = 1.0;

                    workspace(3, 0) = -181.0;
                    workspace(3, 1) = 181.0;
                    workspace(4, 0) = -181.0;
                    workspace(4, 1) = 181.0;
                    workspace(5, 0) = -180.1;
                    workspace(5, 1) = 180.1;

                } else {
                    //// FILE PATH
                    std::string file_name("bin_picking_node.cpp");
                    std::string config_path_ = __FILE__;
                    config_path_.resize(config_path_.length() - file_name.length());
                    config_path_ += "../../config/demo_pose/";
                    std::string tcp_config_path = config_path_ + "demo_blending_cs_waypoint.json";
                    //// JSON INPUT
                    std::stringstream ss;
                    std::ifstream ifs(tcp_config_path.c_str());
                    json j_in = json::parse(ifs);


                    //// 1) Grasping workspace limits
                    //// JSON Load
                    ///////////////////////////////////////////////////////////////////
                    std::vector<double> workspace_in = j_in[demo_name.c_str()]["workspace_grasping"].get<std::vector<double>>();
                    if(workspace_in.size() != 12) {
                        ROS_LOG_ERROR("Check JSON file! - Workspace loading error!");
                        return false;
                    }
                    for(int i=0; i<6; i++) {
                        workspace(i, 0) = workspace_in[2 * i]; // min
                        workspace(i, 1) = workspace_in[2 * i + 1]; // max
                    }
                    printf("demo_name: %s\n", demo_name.c_str());
                    printf("workspace[min]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f\n", workspace_in[0], workspace_in[2], workspace_in[4], workspace_in[6], workspace_in[8], workspace_in[10]);
                    printf("workspace[max]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f\n", workspace_in[1], workspace_in[3], workspace_in[5], workspace_in[7], workspace_in[9], workspace_in[11]);
                    ///////////////////////////////////////////////////////////////////
                    ROS_LOG_WARN("[doGrasping] Demo Mode!");
                }

            } else {
                workspace(0, 0) = -1.0;
                workspace(0, 1) = 1.0;
                workspace(1, 0) = -1.0;
                workspace(1, 1) = 1.0;
                workspace(2, 0) = -1.0;
                workspace(2, 1) = 1.0;

                workspace(3, 0) = 125.0;
                workspace(3, 1) = 181.0;
                workspace(4, 0) = -75.0;
                workspace(4, 1) = 75.0;
                workspace(5, 0) = -180.1;
                workspace(5, 1) = 180.1;

                ROS_LOG_WARN("[doGrasping] Default Mode!");
            }

            // 2) Approach Distance Setting & approach pose generation
            double approach_distance = 0.0; // [m]
            if(object_name == "default") {
                approach_distance = getApproachDistance(); // [m]
                ROS_LOG_INFO("approach_distance: %0.4f", approach_distance);
            } else {
                approach_distance = map_object_name_approach_distance_[object_name];
                ROS_LOG_WARN("[MapApproachDis] approach_distance: %0.4f", approach_distance);
            }

            if (checkWorkspace(pose, workspace) && matching_accuracy > matching_accuracy_limit) { // matching_accuracy 추가
                ROS_LOG_INFO("Grasping pose & matching accuracy OK");
                // tool frame
                std::vector<double> tool_relative_pose(6, 0.0);


                if(demo_name == "drum_task") {
                    tool_relative_pose[0] = -approach_distance; // Tool 기준 -x방향
                } else {
                    tool_relative_pose[2] = -approach_distance; // Tool 기준이므로 부호 반대
                }

                std::vector<double> current_pose = pose;
                std::vector<double> approach_pose;
                // detected_pose로부터 Tool frame 기준의 상대 이동 (approach distance 고려)
                for (std::size_t i = 0; i < 3; i++) { current_pose[i + 3] = current_pose[i + 3] * kDeg2Rad; }
                m_bp_math.transformToolFrame(tool_relative_pose, current_pose, approach_pose); // [m], [rad]
                for (std::size_t i = 0; i < 3; i++) { approach_pose[i + 3] = approach_pose[i + 3] * kRad2Deg; }


                ////

#if DRFL_CONTROL
                //// 두산 로봇의 경우, ZYZ angle
                ROS_LOG_WARN("*******************************************");
                ROS_LOG_WARN("[%s]DRFL_CONTROL MODE! - ZYX to ZYZ euler angle", __func__);
                ROS_LOG_WARN("[%s]DRFL_CONTROL MODE! - ZYX to ZYZ euler angle", __func__);
                ROS_LOG_WARN("[%s]DRFL_CONTROL MODE! - ZYX to ZYZ euler angle", __func__);


                // input: [rad]
                Eigen::Matrix3f Rx = m_bp_math.genRotMf(approach_pose[3] * kDeg2Rad, 0); // 0: rx, 1:ry, 2:rz
                Eigen::Matrix3f Ry = m_bp_math.genRotMf(approach_pose[4] * kDeg2Rad, 1); // 0: rx, 1:ry, 2:rz
                Eigen::Matrix3f Rz = m_bp_math.genRotMf(approach_pose[5] * kDeg2Rad, 2); // 0: rx, 1:ry, 2:rz

                Eigen::Matrix3f R = Rz*Ry*Rx;

                // Convert back to ZYZ Euler angles
                Eigen::Vector3f euler_angles_zyz = m_bp_math.rotationMatrixToZYZEulerAngles(R);
                cout << "Rotation Matrix:\n" << R << endl;
                cout << "ZYZ Euler Angles (degrees):\n" << euler_angles_zyz.transpose() << endl;


                ROS_LOG_WARN("*******************************************");
                // R_ret = Eigen::AngleAxisf(euler_angles_zyz(0)*(M_PI/180.0), Eigen::Vector3f::UnitZ())
                //     * Eigen::AngleAxisf(euler_angles_zyz(1)*(M_PI/180.0), Eigen::Vector3f::UnitY())
                //     * Eigen::AngleAxisf(euler_angles_zyz(2)*(M_PI/180.0), Eigen::Vector3f::UnitZ());
                Eigen::Matrix3f Rz_ret1 = m_bp_math.genRotMf(euler_angles_zyz[0] * kDeg2Rad, 0); // 0: rx, 1:ry, 2:rz
                Eigen::Matrix3f Ry_ret = m_bp_math.genRotMf(euler_angles_zyz[1] * kDeg2Rad, 1); // 0: rx, 1:ry, 2:rz
                Eigen::Matrix3f Rz_ret2 = m_bp_math.genRotMf(euler_angles_zyz[2] * kDeg2Rad, 2); // 0: rx, 1:ry, 2:rz
                Eigen::Matrix3f R_ret = Rz_ret1*Ry_ret*Rz_ret2;
                cout << "[For Checking] Rotation Matrix (from ZYZ):\n" << R_ret << endl;


                ROS_LOG_WARN("*******************************************");


#endif

                ////



                is_grasping_pose_cal_finished = false;
                is_matching_finished = false;
                output_pose = approach_pose;

                is_blending_approach_grasp_pose_assigned = true;
                is_grasping_pose_assigned = true;
                is_zig_pose_assigned = true;

                detected_approach_pose_ = approach_pose; // for blending

                return true;
            } else {
                if(!checkWorkspace(pose, workspace)) ROS_LOG_INFO("Grasping pose is infeasible");
                if(matching_accuracy < matching_accuracy_limit) ROS_LOG_INFO("Matching accuracy limit!");

                bin_scan_fail = true;
                is_grasping_pose_cal_finished = false;
                is_matching_finished = false;

                is_blending_approach_grasp_pose_assigned = false;
                is_grasping_pose_assigned = false;
                is_zig_pose_assigned = false;

                return false;
            }
        } else {
            if(is_grasping_pose_cal_finished) {
                ROS_LOG_WARN("the grasping pose is not yet received (doGrasping)");
            }
            if(is_grasping_pose_feasible_) {
                ROS_LOG_WARN("the grasping pose is infeasible (doGrasping)");
            }
            bin_scan_fail = true;
            is_grasping_pose_cal_finished = false;
            is_matching_finished = false;

            is_blending_approach_grasp_pose_assigned = false;
            is_grasping_pose_assigned = false;
            is_zig_pose_assigned = false;

            return false;
        }
        /////////////////////////////////////////////////////////////////////////
    }
    catch(std::string &e)
    {
        ROS_LOG_ERROR("Check the Demo CS Pose JSON file! - doGrasping");
        bin_scan_fail = true;
        is_grasping_pose_cal_finished = false;
        is_matching_finished = false;

        is_blending_approach_grasp_pose_assigned = false;

        return false;
    }
}

bool BinPickingNode::checkMatchingResult(enum GraspingPoseType pose_type, std::string demo_name)
{
    try
    {
        //// FILE PATH
        std::string file_name("bin_picking_node.cpp");
        std::string config_path_ = __FILE__;
        config_path_.resize(config_path_.length() - file_name.length());
        config_path_ += "../../config/demo_pose/";
        std::string tcp_config_path = config_path_ + "demo_blending_cs_waypoint.json";
        //// JSON INPUT
        std::stringstream ss;
        std::ifstream ifs(tcp_config_path.c_str());
        json j_in = json::parse(ifs);

        /////////////////////////////////////////////////////////////////////////
        if(is_grasping_pose_cal_finished) { // Template matching results
            std::vector<double> pose;
            switch(pose_type) {
                case GraspingPoseType::OPTIMAL_POSE: {
                    ROS_LOG_INFO("Do Grasping parts!! (Bin Picking)");
                    pose = getDetectedPose(); // Grasping pose
                    ROS_LOG_INFO("Detected pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                    ROS_LOG_INFO("Matching accuracy: %0.3f", matching_accuracy);
                    break;
                }
                case GraspingPoseType::SUB_OPTIMAL_POSE: {
                    ROS_LOG_INFO("Do Sub Grasping parts!! (Bin Picking)");
                    pose = getDetectedSubPose(); // Sub-grasping pose
                    ROS_LOG_INFO("Detected sub. pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                    ROS_LOG_INFO("Matching accuracy: %0.3f", matching_accuracy);
                    break;
                }
                case GraspingPoseType::AI_ESTIMATED_POSE: {
                    ROS_LOG_INFO("Do Grasping parts!! (Bin Picking - Pose Estimation)");
                    pose = getEstimatedPose(); // Sub-grasping pose
                    ROS_LOG_INFO("AI Estimated pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                    break;
                }
                default: {
                    assert(false);
                    break;
                }
            }

            //// 1) Grasping workspace limits
            //// JSON Load
            ///////////////////////////////////////////////////////////////////
            Eigen::MatrixXd workspace(6,2); // pose limit - (min, max)
            std::vector<double> workspace_in = j_in[demo_name.c_str()]["workspace_grasping"].get<std::vector<double>>();
            if(workspace_in.size() != 12) {
                ROS_LOG_ERROR("Check JSON file! - Workspace loading error!");
                return false;
            }
            for(int i=0; i<6; i++) {
                workspace(i, 0) = workspace_in[2 * i]; // min
                workspace(i, 1) = workspace_in[2 * i + 1]; // max
            }
            printf("demo_name: %s\n", demo_name.c_str());
            printf("workspace[min]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f\n", workspace_in[0], workspace_in[2], workspace_in[4], workspace_in[6], workspace_in[8], workspace_in[10]);
            printf("workspace[max]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f\n", workspace_in[1], workspace_in[3], workspace_in[5], workspace_in[7], workspace_in[9], workspace_in[11]);
            ///////////////////////////////////////////////////////////////////


            // 2)
            double approach_distance = getApproachDistance(); // [m]
            if (checkWorkspace(pose, workspace) && matching_accuracy > matching_accuracy_limit) { // matching_accuracy 추가
                ROS_LOG_INFO("Grasping pose & matching accuracy OK");

                return true;
            } else {
                if(!checkWorkspace(pose, workspace)) ROS_LOG_INFO("Grasping pose is infeasible");
                if(matching_accuracy < matching_accuracy_limit) ROS_LOG_INFO("Matching accuracy limit!");

                return false;
            }
        } else {
            return false;
        }
        /////////////////////////////////////////////////////////////////////////
    }
    catch(std::string &e)
    {
        ROS_LOG_ERROR("Check the Demo CS Pose JSON file! - checkMatchingResult");
        return false;
    }
}

bool BinPickingNode::generateStackingPose(std::vector<double> &output_pose, size_t stacking_cnt, const std::vector<double> &trans_scale, size_t single_stack_num, size_t line_stack_num, enum StackingType stack_type)
{
    if(!is_detaching_stack_pose_assigned)
    {
        //// NOTICE: Stacking하는 순서는 col 방향부터 한 줄씩
        //// Ex) single_stack_num: 3, line_stack_num: 2 으로 설정하면,
        //// (single_stack_num, line_stack_num) = (0, 0) -> (1, 0) -> (2, 0) -> (0, 1) -> (1, 1) -> (2, 1) 순서대로 수행됨..
        //// TODO: 일단 Base frame 기준으로 생성, 추후 tool 기준 자세를 받아오도록 처리
        std::vector<double> pose = output_pose;
        size_t row_idx = 0;
        size_t col_idx = 0;
        size_t z_idx = stacking_cnt / (single_stack_num * line_stack_num);
        size_t cnt_now = stacking_cnt % (single_stack_num * line_stack_num);
        double row_scale = trans_scale[0];
        double col_scale = trans_scale[1];
        double z_scale = trans_scale[2];
        switch(stack_type) {
            case StackingType::GRID_STACKING: {
                ROS_LOG_INFO("Get Stacking Pose! (GRID_STACKING)");
                row_idx = cnt_now % line_stack_num;
                col_idx = cnt_now / line_stack_num;

                //// 240211, UR10e환경 기준
                pose[0] -= col_scale*static_cast<double>(col_idx); // x, base frame
                // pose[1] -= row_scale*static_cast<double>(row_idx); // y, base frame
                pose[2] += z_scale*static_cast<double>(z_idx); // y, base frame

                ROS_LOG_INFO("Row_idx: %zu, Col_idx: %zu, Z_idx: %zu", row_idx, col_idx, z_idx);
                ROS_LOG_INFO("Row scale: %0.4f, Col scale: %0.4f, Z scale: %0.4f", row_scale*static_cast<double>(row_idx), col_scale*static_cast<double>(col_idx), z_scale*static_cast<double>(z_idx));
                ROS_LOG_INFO("Stacking pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                break;
            }
            case StackingType::CYLINDER_STACKING: {
                size_t stacking_cnt_now = stacking_cnt;

                // 첫 번째 단을 모두 쌓은 경우, 초기화
                if(stacking_cnt_now < m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stack_part_cnt_) {
                    row_idx = 0;
                } else {
                    if(stacking_cnt_now == m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stack_part_cnt_) {
                        m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_z_idx_ = 0;
                    }
                    stacking_cnt_now -= m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stack_part_cnt_;
                    row_idx = 0;
                }

                if(m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_z_idx_ == 0) {
                    cnt_now = stacking_cnt_now;
                    // row_idx = cnt_now % line_stack_num;
                    col_idx = cnt_now / line_stack_num;
                    z_idx = m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_z_idx_;
                } else {

                    for(size_t i = 0; i < m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_z_idx_; i++) {
                        stacking_cnt_now -=  (single_stack_num - i) * line_stack_num;
                    }

                    cnt_now = stacking_cnt_now;
                    // row_idx = cnt_now % line_stack_num;
                    col_idx = cnt_now / line_stack_num;

                    z_idx = m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_z_idx_;
                }
                size_t stacking_cnt_limit = (single_stack_num - m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_z_idx_) * line_stack_num;
                if(stacking_cnt_now == stacking_cnt_limit - 1) {
                    m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_z_idx_++;
                }


                ROS_LOG_INFO("Get Stacking Pose! (CYLINDER_STACKING), current z_idx: %zu, stacking_cnt_now: %zu", z_idx, stacking_cnt_now);

                //// Cylinder는 Tool 기준 변환, 티칭 시에 툴 기준 50mm 투입 거리
                if(0) { // 기존 tool frame 기준
                    std::vector<double> tool_default_translation(6, 0.0);
                    tool_default_translation[2] = 0.050; // tool frame
                    std::vector<double> pose_default = transformRelativePoseToolFrame(pose, tool_default_translation); // [m], [deg]

                    //// a) translation
                    // Row 방향(tool 기준 -x)으로 0.225m, Col 방향으로 0.050
                    std::vector<double> tool_relative_translation(6, 0.0);
                    tool_relative_translation[0] = -row_scale*static_cast<double>(row_idx); // tool frame
                    tool_relative_translation[1] = +col_scale*static_cast<double>(col_idx); // tool frame
                    if(z_idx > 0) {
                        for(size_t i = 0; i < z_idx; i++) {
                            tool_relative_translation[1] += col_scale / 2.0; // tool frame
                        }
                    }
                    tool_relative_translation[2] = -z_scale*static_cast<double>(z_idx); // tool frame
                    std::vector<double> pose_trans = transformRelativePoseToolFrame(pose_default, tool_relative_translation); // [m], [deg]
                    //// b) rotation
                    std::vector<double> tool_relative_rotation(6, 0.0);
                    if(col_idx > 0) {
                        tool_relative_rotation[3] = 45.0; // tool frame
                    }
                    std::vector<double> pose_rot = transformRelativePoseToolFrame(pose_trans, tool_relative_rotation); // [m], [deg]

                    //// c) approach pose
                    std::vector<double> tool_approach_translation(6, 0.0);
                    tool_approach_translation[2] = -0.050; // tool frame
                    std::vector<double> apporach_pose = transformRelativePoseToolFrame(pose_rot, tool_approach_translation); // [m], [deg]

                    //// d) update
                    pose = apporach_pose;
                } else {
                    //// NOTICE: JSON에 저장된 티칭 값은 정답 자세가 아니라, 정답 자세에서 tool 기준으로 5cm위의 자세
                    std::vector<double> tool_default_translation(6, 0.0);
                    tool_default_translation[2] = 0.050; // tool frame
                    std::vector<double> pose_default = transformRelativePoseToolFrame(pose, tool_default_translation); // [m], [deg]

                    //// a) translation
                    // (base 기준 -x)으로 50cm씩 이동하도록 처리
                    pose_default[0] -= col_scale*static_cast<double>(col_idx); // base frame
                    // pose_default[1] -= row_scale*static_cast<double>(row_idx); // base frame
                    if(z_idx > 0) {
                        for(size_t i = 0; i < z_idx; i++) {
                            pose_default[0] -= col_scale / 2.0; // base frame
                        }
                    }
                    pose_default[2] += z_scale*static_cast<double>(z_idx); // base frame


                    //// c) approach pose
                    std::vector<double> tool_approach_translation(6, 0.0);
                    tool_approach_translation[2] = -0.050; // tool frame
                    std::vector<double> apporach_pose = transformRelativePoseToolFrame(pose_default, tool_approach_translation); // [m], [deg]

                    //// d) update
                    pose = apporach_pose;
                }

                ROS_LOG_INFO("Row scale: %0.4f, Col scale: %0.4f, Z scale: %0.4f", row_scale*static_cast<double>(row_idx), col_scale*static_cast<double>(col_idx), z_scale*static_cast<double>(z_idx));
                ROS_LOG_INFO("Row_idx: %zu, Col_idx: %zu, Z_idx: %zu", row_idx, col_idx, z_idx);
                ROS_LOG_INFO("Stacking pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
                break;
            }
            default: {
                assert(false);
                break;
            }
        }
        is_detaching_stack_pose_assigned = true;
        assigned_stacking_pose_ = pose;
        output_pose = pose;
        return true;
    } else {
        return false;
    }
}

std::vector<double> BinPickingNode::transformRelativePoseToolFrame(const std::vector<double> &pose, const std::vector<double> &relative_pose)
{
    std::vector<double> current_pose = pose; // [m], [deg]
    std::vector<double> tool_relative_pose = relative_pose; // [m], [deg]
    std::vector<double> output_pose;
    // Tool frame 기준의 상대 이동
    for (std::size_t i = 0; i < 3; i++) {
        current_pose[i + 3] = current_pose[i + 3] * kDeg2Rad;
        tool_relative_pose[i + 3] = tool_relative_pose[i + 3] * kDeg2Rad;
    }
    m_bp_math.transformToolFrame(tool_relative_pose, current_pose, output_pose); // [m], [rad]
    for (std::size_t i = 0; i < 3; i++) { output_pose[i + 3] = output_pose[i + 3] * kRad2Deg; }

    return output_pose;
}

bool BinPickingNode::getCSPoseWithDefaultTCPCommand(unsigned int tcp_idx, const std::vector<double> &js_position, std::vector<double> &pose) {
    auto req = make_shared<hanyang_matching_msgs::srv::SetTcp::Request>();
	req->tcp_idx   = tcp_idx;
	req->joint_position   = js_position;
    req->operating_mode  = operatingModeTask_;
    req->get_measured_js_value = false;

    while (!robotACalCSPoseWithDefaultTCPClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = robotACalCSPoseWithDefaultTCPClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        pose = recv->pose; // [m], [rad]
        return true;
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

bool BinPickingNode::getCSPoseWithDefaultTCPCommandUsingCurrentJS(unsigned int tcp_idx, std::vector<double> &pose) {
    //// TODO: 22.04
    //// TODO: 22.04
    //// TODO: 22.04
    //// TODO: 22.04
    return false;
    // auto req = make_shared<hanyang_matching_msgs::srv::SetTcp::Request>();
	// req->tcp_idx   = tcp_idx;
    // req->operating_mode  = operatingModeTask_;
    // req->get_measured_js_value = true;

    // while (!robotACalCSPoseWithDefaultTCPClient_->wait_for_service(1s)) {
    //     if (!rclcpp::ok()) {
    //         ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
    //         return false;
    //     }
    //     ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
    //     return false;
    // }
    // auto result = robotACalCSPoseWithDefaultTCPClient_->async_send_request(req);
    // // Wait for the result.
    // if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
    //     auto recv = result.get();
    //     pose = recv->pose; // [m], [rad]
    //     return true;
    // } else {
    //     ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
    //     return false;
    // }
}

bool BinPickingNode::getJSPositionWithDefaultTCPCommand(unsigned int tcp_idx, const std::vector<double> &cs_pose, std::vector<double> &js_position) {
    auto req = make_shared<hanyang_matching_msgs::srv::SetTcp::Request>();
	req->tcp_idx   = tcp_idx;
	req->cs_pose_input   = cs_pose; // [m], [rad]
    req->operating_mode  = operatingModeTask_;

    while (!robotACalJSPositionWithDefaultTCPClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = robotACalJSPositionWithDefaultTCPClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        js_position = recv->js_position_output;
        return true;
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

//// 툴 체인징 시스템의 티칭 후에 저장된 값을 읽어서 센서 좌표계 기준의 자세로 변환하여 JSON 파일로 저장
//// NOTICE: 이 함수는 툴 체인징 시스템의 교시 후에 1회만 수행하면 됨.
//// Input file list
// 1) tool_changing_js_position_base_frame.json
// 2) tool_changing_cs_pose_base_frame.json
//// Output file list
// 1) tool_changing_cs_pose_sensor_frame.json
bool BinPickingNode::transformToolChangingPose2SensorPose(taskTemplateMatchingParameter &parameter)
{
    try
    {
        Eigen::Matrix4f T_B2S_global = Eigen::Matrix4f::Identity();
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                if(j < 3) T_B2S_global(i, j) = parameter.htm_vec_B2S_global[4 * i + j];
                else T_B2S_global(i, j) = 0.001*parameter.htm_vec_B2S_global[4 * i + j]; // [m]
            }
        }
        // std::cout << "T_B2S_global: " << T_B2S_global << std::endl;
        Eigen::Matrix4f T_S2B_global = T_B2S_global.inverse();

        //// json file save
        std::stringstream ss;
        // json j_in;
        // j_in["count"] = task_vec_set_tool_changing_attach_JS_position_.size();

        json j_in2;
        j_in2["count"] = task_vec_set_tool_changing_attach_JS_position_.size();

        json j_out;
        j_out["count"] = task_vec_set_tool_changing_attach_JS_position_.size();



        //// TODO: Base 기준의 JSON에서 저장된 JS position 불러오기
        size_t slot_num = task_vec_set_tool_changing_attach_JS_position_.size();
        for(size_t i = 0; i < slot_num; i++)
        {
            printf("********************* Attaching Slot #%zu *************************\n", i+1);
            // Forward kinematics w.r.t. default TCP (-> tool master flange)
            std::vector<double> vec_cs_pose_attach;
            printVector(task_vec_set_tool_changing_attach_JS_position_[i], "input js position");

            // ss.str("");
            // ss << "attaching_js_position";
            // ss << std::setw(3) << std::setfill('0') << i+1;
            // j_in[ss.str()] = task_vec_set_tool_changing_attach_JS_position_[i];

            if(getCSPoseWithDefaultTCPCommand(7, task_vec_set_tool_changing_attach_JS_position_[i], vec_cs_pose_attach)) // default tcp idx: 7, target JS position
            {
                Eigen::VectorXf cs_pose_attach(6);
                for(int j = 0; j < 6; j++) cs_pose_attach[j] = vec_cs_pose_attach[j]; // [m], [rad]
                Eigen::Matrix4f T_attach_base_frame;
                m_bp_math.pose2HTM(cs_pose_attach, T_attach_base_frame); // [m], [rad]


                for(int j = 0; j < 3; j++) vec_cs_pose_attach[j + 3] *= kRad2Deg;
                printVector(vec_cs_pose_attach, "vec_cs_pose_attach");

                // std::cout << "Slot #" << i+1 << " " << "T_attach_base_frame: " << T_attach_base_frame << std::endl;

                //////////////////////////////////////////
                //// Base frame pose
                ss.str("");
                ss << "attaching_pose";
                ss << std::setw(3) << std::setfill('0') << i+1;
                for(int j = 0; j < 3; j++)
                {
                    vec_cs_pose_attach[j] = round(vec_cs_pose_attach[j]*1e7) / 1e7; // [m]
                    vec_cs_pose_attach[j + 3] = round(vec_cs_pose_attach[j + 3]*1e4) / 1e4; // [deg]
                }
                j_in2[ss.str()] = vec_cs_pose_attach;
                //////////////////////////////////////////



                Eigen::Matrix4f T_attach_sensor_frame = T_S2B_global*T_attach_base_frame;
                std::vector<double> pose_attach_sensor_frame = m_bp_math.htm2pose(T_attach_sensor_frame);
                for(int j = 0; j < 3; j++)
                {
                    pose_attach_sensor_frame[j + 3] *= kRad2Deg;
                    pose_attach_sensor_frame[j] = round(pose_attach_sensor_frame[j]*1e7) / 1e7; // [m]
                    pose_attach_sensor_frame[j + 3] = round(pose_attach_sensor_frame[j + 3]*1e4) / 1e4; // [deg]
                }
                //// Sensor pose
                ss.str("");
                ss << "attaching_pose";
                ss << std::setw(3) << std::setfill('0') << i+1;
                j_out[ss.str()] = pose_attach_sensor_frame;
            }
            else
            {
                printf("Check the robot controller package!\n");
                return false;
            }
            printf("************************************************\n");
        }

        for(size_t i = 0; i < slot_num; i++)
        {
            printf("********************* Detaching Slot #%zu *************************\n", i+1);
            // Forward kinematics w.r.t. default TCP (-> tool master flange)
            std::vector<double> vec_cs_pose_detach;
            printVector(task_vec_set_tool_changing_detach_JS_position_[i], "input js position");

            // ss.str("");
            // ss << "detaching_js_position";
            // ss << std::setw(3) << std::setfill('0') << i+1;
            // j_in[ss.str()] = task_vec_set_tool_changing_detach_JS_position_[i];

            if(getCSPoseWithDefaultTCPCommand(7, task_vec_set_tool_changing_detach_JS_position_[i], vec_cs_pose_detach)) // default tcp idx: 7, target JS position
            {

                Eigen::VectorXf cs_pose_detach(6);
                for(int j = 0; j < 6; j++) cs_pose_detach[j] = vec_cs_pose_detach[j]; // [m], [rad]
                Eigen::Matrix4f T_detach_base_frame;
                m_bp_math.pose2HTM(cs_pose_detach, T_detach_base_frame); // [m], [rad]

                for(int j = 0; j < 3; j++) vec_cs_pose_detach[j + 3] *= kRad2Deg;
                printVector(vec_cs_pose_detach, "vec_cs_pose_detach");
                // std::cout << "Slot #" << i+1 << " " << "T_detach_base_frame: " << T_detach_base_frame << std::endl;

                //////////////////////////////////////////
                //// Base frame pose
                ss.str("");
                ss << "detaching_pose";
                ss << std::setw(3) << std::setfill('0') << i+1;
                for(int j = 0; j < 3; j++)
                {
                    vec_cs_pose_detach[j] = round(vec_cs_pose_detach[j]*1e7) / 1e7; // [m]
                    vec_cs_pose_detach[j + 3] = round(vec_cs_pose_detach[j + 3]*1e4) / 1e4; // [deg]
                }
                j_in2[ss.str()] = vec_cs_pose_detach;
                //////////////////////////////////////////


                Eigen::Matrix4f T_detach_sensor_frame = T_S2B_global*T_detach_base_frame;
                std::vector<double> pose_detach_sensor_frame = m_bp_math.htm2pose(T_detach_sensor_frame);
                for(int j = 0; j < 3; j++)
                {
                    pose_detach_sensor_frame[j + 3] *= kRad2Deg;
                    pose_detach_sensor_frame[j] = round(pose_detach_sensor_frame[j]*1e7) / 1e7; // [m]
                    pose_detach_sensor_frame[j + 3] = round(pose_detach_sensor_frame[j + 3]*1e4) / 1e4; // [deg]
                }
                //// Sensor pose
                ss.str("");
                ss << "detaching_pose";
                ss << std::setw(3) << std::setfill('0') << i+1;
                j_out[ss.str()] = pose_detach_sensor_frame;
            }
            else
            {
                printf("Check the robot controller package!\n");
                return false;
            }
            printf("************************************************\n");
        }



        //// Default JS Position
        std::vector<double> default_vec_cs_pose_attach;
        printVector(tc_robot_left_side_attach_default_js_position_, "tc_robot_left_side_attach_default_js_position_");
        if(getCSPoseWithDefaultTCPCommand(7, tc_robot_left_side_attach_default_js_position_, default_vec_cs_pose_attach)) // default tcp idx: 7, target JS position
        {
            Eigen::VectorXf cs_pose_attach(6);
            for(int j = 0; j < 6; j++) cs_pose_attach[j] = default_vec_cs_pose_attach[j]; // [m], [rad]
            Eigen::Matrix4f T_attach_base_frame;
            m_bp_math.pose2HTM(cs_pose_attach, T_attach_base_frame); // [m], [rad]

            for(int j = 0; j < 3; j++) default_vec_cs_pose_attach[j + 3] *= kRad2Deg;
            printVector(default_vec_cs_pose_attach, "default_vec_cs_pose_attach");

            //////////////////////////////////////////
            //// Base frame pose
            ss.str("");
            ss << "default_attaching_pose";
            for(int j = 0; j < 3; j++)
            {
                default_vec_cs_pose_attach[j] = round(default_vec_cs_pose_attach[j]*1e7) / 1e7; // [m]
                default_vec_cs_pose_attach[j + 3] = round(default_vec_cs_pose_attach[j + 3]*1e4) / 1e4; // [deg]
            }
            j_in2[ss.str()] = default_vec_cs_pose_attach;
            //////////////////////////////////////////

            Eigen::Matrix4f T_attach_sensor_frame = T_S2B_global*T_attach_base_frame;
            std::vector<double> pose_attach_sensor_frame = m_bp_math.htm2pose(T_attach_sensor_frame);
            for(int j = 0; j < 3; j++)
            {
                pose_attach_sensor_frame[j + 3] *= kRad2Deg;
                pose_attach_sensor_frame[j] = round(pose_attach_sensor_frame[j]*1e7) / 1e7; // [m]
                pose_attach_sensor_frame[j + 3] = round(pose_attach_sensor_frame[j + 3]*1e4) / 1e4; // [deg]
            }
            //// Sensor pose
            ss.str("");
            ss << "default_attaching_pose";
            j_out[ss.str()] = pose_attach_sensor_frame;
        }
        else
        {
            printf("Check the robot controller package!\n");
            return false;
        }

        std::vector<double> default_vec_cs_pose_detach;
        printVector(tc_robot_left_side_detach_default_js_position_, "tc_robot_left_side_detach_default_js_position_");
        if(getCSPoseWithDefaultTCPCommand(7, tc_robot_left_side_detach_default_js_position_, default_vec_cs_pose_detach)) // default tcp idx: 7, target JS position
        {
            Eigen::VectorXf cs_pose_detach(6);
            for(int j = 0; j < 6; j++) cs_pose_detach[j] = default_vec_cs_pose_detach[j]; // [m], [rad]
            Eigen::Matrix4f T_detach_base_frame;
            m_bp_math.pose2HTM(cs_pose_detach, T_detach_base_frame); // [m], [rad]

            for(int j = 0; j < 3; j++) default_vec_cs_pose_detach[j + 3] *= kRad2Deg;
            printVector(default_vec_cs_pose_detach, "default_vec_cs_pose_detach");

            //////////////////////////////////////////
            //// Base frame pose
            ss.str("");
            ss << "default_detaching_pose";
            for(int j = 0; j < 3; j++)
            {
                default_vec_cs_pose_detach[j] = round(default_vec_cs_pose_detach[j]*1e7) / 1e7; // [m]
                default_vec_cs_pose_detach[j + 3] = round(default_vec_cs_pose_detach[j + 3]*1e4) / 1e4; // [deg]
            }
            j_in2[ss.str()] = default_vec_cs_pose_detach;
            //////////////////////////////////////////

            Eigen::Matrix4f T_detach_sensor_frame = T_S2B_global*T_detach_base_frame;
            std::vector<double> pose_detach_sensor_frame = m_bp_math.htm2pose(T_detach_sensor_frame);
            for(int j = 0; j < 3; j++)
            {
                pose_detach_sensor_frame[j + 3] *= kRad2Deg;
                pose_detach_sensor_frame[j] = round(pose_detach_sensor_frame[j]*1e7) / 1e7; // [m]
                pose_detach_sensor_frame[j + 3] = round(pose_detach_sensor_frame[j + 3]*1e4) / 1e4; // [deg]
            }
            //// Sensor pose
            ss.str("");
            ss << "default_detaching_pose";
            j_out[ss.str()] = pose_detach_sensor_frame;
        }
        else
        {
            printf("Check the robot controller package!\n");
            return false;
        }

        std::ofstream ofs_in2(std::string("/home/") + USER_NAME + "/robot_control_ws/src/robot_control_ui/robot_control_ui/config/tool_changing_pose/tool_changing_cs_pose_base_frame.json");
        ofs_in2 << j_in2.dump(4) << std::endl;

        std::ofstream ofs(std::string("/home/") + USER_NAME + "/robot_control_ws/src/robot_control_ui/robot_control_ui/config/tool_changing_pose/tool_changing_cs_pose_sensor_frame.json");
        ofs << j_out.dump(4) << std::endl;


        return true;
    }
    catch(std::string &e)
    {
        return false;
    }
}

//// 교시를 통해 미리 저장된 센서 좌표계 기준의 CS 자세를 변경된 로봇 좌표계 기준으로 변환하여 툴 체인징 시스템의 새로운 로봇 좌표계 기준의 자세(CS & JS)를 JSON 파일로 저장
//// NOTICE: 이 함수는 작업대를 이동시켜, T_B2S_global이 변경될 때마다 수행해야 함.
//// Input file list
// 1) tool_changing_cs_pose_sensor_frame.json
//// Output file list
// 1) tool_changing_js_position_base_frame.json
// 2) tool_changing_cs_pose_base_frame.json
bool BinPickingNode::transformToolChangingPose2NewBaseFrame(taskTemplateMatchingParameter &parameter)
{
    try
    {
        //// Load JSON
        std::stringstream ss;
        std::stringstream ss_json;
        ss_json << "/home/" << USER_NAME << "/robot_control_ws/src/robot_control_ui/robot_control_ui/config/tool_changing_pose/tool_changing_cs_pose_sensor_frame.json";
        std::ifstream ifs(ss_json.str().c_str());
        json j_in = json::parse(ifs);
        size_t cnt_js = j_in["count"];

        //// json file input - target JS angle
        printf("*******************************************\n");
        std::vector<std::vector<double>> vec_set_tool_changing_attach_CS_pose_sensor_frame(cnt_js);
        std::vector<std::vector<double>> vec_set_tool_changing_detach_CS_pose_sensor_frame(cnt_js);
        for(size_t i = 0; i < cnt_js; i++)
        {
            ss.str("");
            ss << "attaching_pose";
            ss << std::setw(3) << std::setfill('0') << i+1;
            std::vector<double> vec_attach = j_in[ss.str()].get<std::vector<double>>();
            vec_set_tool_changing_attach_CS_pose_sensor_frame[i] = vec_attach; // [m], [deg]
            printf("vec_set_tool_changing_attach_CS_pose_sensor_frame%zuth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, vec_set_tool_changing_attach_CS_pose_sensor_frame[i][0], vec_set_tool_changing_attach_CS_pose_sensor_frame[i][1],vec_set_tool_changing_attach_CS_pose_sensor_frame[i][2],vec_set_tool_changing_attach_CS_pose_sensor_frame[i][3],vec_set_tool_changing_attach_CS_pose_sensor_frame[i][4],vec_set_tool_changing_attach_CS_pose_sensor_frame[i][5]);

            ss.str("");
            ss << "detaching_pose";
            ss << std::setw(3) << std::setfill('0') << i+1;
            std::vector<double> vec_detach = j_in[ss.str()].get<std::vector<double>>();
            vec_set_tool_changing_detach_CS_pose_sensor_frame[i] = vec_detach; // [m], [deg]
            printf("vec_set_tool_changing_detach_CS_pose_sensor_frame%zuth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, vec_set_tool_changing_detach_CS_pose_sensor_frame[i][0], vec_set_tool_changing_detach_CS_pose_sensor_frame[i][1],vec_set_tool_changing_detach_CS_pose_sensor_frame[i][2],vec_set_tool_changing_detach_CS_pose_sensor_frame[i][3],vec_set_tool_changing_detach_CS_pose_sensor_frame[i][4],vec_set_tool_changing_detach_CS_pose_sensor_frame[i][5]);
        }

        // Default JS Position
        std::vector<double> vec_tool_changing_default_attach_CS_pose_sensor_frame;
        std::vector<double> vec_tool_changing_default_detach_CS_pose_sensor_frame;
        ss.str("");
        ss << "default_attaching_pose";
        vec_tool_changing_default_attach_CS_pose_sensor_frame = j_in[ss.str()].get<std::vector<double>>();
        ss.str("");
        ss << "default_detaching_pose";
        vec_tool_changing_default_detach_CS_pose_sensor_frame = j_in[ss.str()].get<std::vector<double>>();

        printf("*******************************************\n");



        json j_out_1;
        j_out_1["count"] = cnt_js;

        json j_out_2;
        j_out_2["count"] = cnt_js;

        Eigen::Matrix4f T_B2S_global = Eigen::Matrix4f::Identity();
        for(int i = 0; i < 3; i++)
        {
            for(int j = 0; j < 4; j++)
            {
                if(j < 3) T_B2S_global(i, j) = parameter.htm_vec_B2S_global[4 * i + j];
                else T_B2S_global(i, j) = 0.001*parameter.htm_vec_B2S_global[4 * i + j]; // [m]
            }
        }
        std::cout << "T_B2S_global: " << T_B2S_global << std::endl;

        size_t slot_num = cnt_js;
        for(size_t i = 0; i < slot_num; i++)
        {
            printf("********************* Attaching Slot #%zu *************************\n", i+1);
            // Forward kinematics w.r.t. default TCP (-> tool master flange)
            std::vector<double> vec_cs_pose_attach = vec_set_tool_changing_attach_CS_pose_sensor_frame[i]; // [m], [deg]
            printVector(vec_cs_pose_attach, "vec_cs_pose_attach");
            Eigen::VectorXf cs_pose_attach(6);
            for(int j = 0; j < 3; j++)
            {
                cs_pose_attach[j] = vec_cs_pose_attach[j];
                cs_pose_attach[j + 3] = kDeg2Rad*vec_cs_pose_attach[j + 3];
            }
            Eigen::Matrix4f T_attach_sensor_frame;
            m_bp_math.pose2HTM(cs_pose_attach, T_attach_sensor_frame); // [m], [rad]

            // std::cout << "Slot (sensor frame) #" << i+1 << " " << "T_attach_sensor_frame: " << T_attach_sensor_frame << std::endl;

            Eigen::Matrix4f T_attach_base_frame = T_B2S_global*T_attach_sensor_frame;
            std::vector<double> pose_attach_base_frame = m_bp_math.htm2pose(T_attach_base_frame);
            // std::cout << "Slot (base frame) #" << i+1 << " " << "T_attach_base_frame: " << T_attach_base_frame << std::endl;

            //// I.K.
            // pose_attach_base_frame: [m], [rad]
            std::vector<double> vec_js_position_attach_ik_results;
            if(getJSPositionWithDefaultTCPCommand(7, pose_attach_base_frame, vec_js_position_attach_ik_results))
            {
                for(int j = 0; j < 3; j++) pose_attach_base_frame[j + 3] *= kRad2Deg;
                printVector(pose_attach_base_frame, "pose_attach_base_frame");
                printVector(vec_js_position_attach_ik_results, "vec_js_position_attach_ik_results");

                //// Base frame CS pose
                ss.str("");
                ss << "attaching_pose";
                ss << std::setw(3) << std::setfill('0') << i+1;
                for(int j = 0; j < 3; j++)
                {
                    pose_attach_base_frame[j] = round(pose_attach_base_frame[j]*1e7) / 1e7; // [m]
                    pose_attach_base_frame[j + 3] = round(pose_attach_base_frame[j + 3]*1e4) / 1e4; // [deg]
                }
                j_out_1[ss.str()] = pose_attach_base_frame;


                //// Base frame JS position
                ss.str("");
                ss << "attaching_js_position";
                ss << std::setw(3) << std::setfill('0') << i+1;
                for(int j = 0; j < 6; j++) vec_js_position_attach_ik_results[j] = round(vec_js_position_attach_ik_results[j]*1e4) / 1e4; // [deg]
                j_out_2[ss.str()] = vec_js_position_attach_ik_results;
            }
            else
            {
                printf("Check the robot controller package!\n");
                return false;
            }
            printf("************************************************\n");
        }

        for(size_t i = 0; i < slot_num; i++)
        {
            printf("********************* Detaching Slot #%zu *************************\n", i+1);
            // Forward kinematics w.r.t. default TCP (-> tool master flange)
            std::vector<double> vec_cs_pose_detach = vec_set_tool_changing_detach_CS_pose_sensor_frame[i];
            printVector(vec_cs_pose_detach, "vec_cs_pose_detach");
            Eigen::VectorXf cs_pose_detach(6);
            for(int j = 0; j < 3; j++)
            {
                cs_pose_detach[j] = vec_cs_pose_detach[j];
                cs_pose_detach[j + 3] = kDeg2Rad*vec_cs_pose_detach[j + 3];
            }
            Eigen::Matrix4f T_detach_sensor_frame;
            m_bp_math.pose2HTM(cs_pose_detach, T_detach_sensor_frame); // [m], [rad]
            // std::cout << "Slot (sensor frame) #" << i+1 << " " << "T_detach_sensor_frame: " << T_detach_sensor_frame << std::endl;

            Eigen::Matrix4f T_detach_base_frame = T_B2S_global*T_detach_sensor_frame;
            std::vector<double> pose_detach_base_frame = m_bp_math.htm2pose(T_detach_base_frame);
            // std::cout << "Slot (base frame) #" << i+1 << " " << "T_detach_base_frame: " << T_detach_base_frame << std::endl;

            //// I.K.
            // pose_detach_base_frame: [m], [rad]
            std::vector<double> vec_js_position_detach_ik_results;
            if(getJSPositionWithDefaultTCPCommand(7, pose_detach_base_frame, vec_js_position_detach_ik_results))
            {
                for(int j = 0; j < 3; j++) pose_detach_base_frame[j + 3] *= kRad2Deg;
                printVector(pose_detach_base_frame, "pose_detach_base_frame");
                printVector(vec_js_position_detach_ik_results, "vec_js_position_detach_ik_results");

                //// Base frame CS pose
                ss.str("");
                ss << "detaching_pose";
                ss << std::setw(3) << std::setfill('0') << i+1;
                for(int j = 0; j < 3; j++)
                {
                    pose_detach_base_frame[j] = round(pose_detach_base_frame[j]*1e7) / 1e7; // [m]
                    pose_detach_base_frame[j + 3] = round(pose_detach_base_frame[j + 3]*1e4) / 1e4; // [deg]
                }
                j_out_1[ss.str()] = pose_detach_base_frame;


                //// Base frame JS position
                ss.str("");
                ss << "detaching_js_position";
                ss << std::setw(3) << std::setfill('0') << i+1;
                for(int j = 0; j < 6; j++) vec_js_position_detach_ik_results[j] = round(vec_js_position_detach_ik_results[j]*1e4) / 1e4; // [deg]
                j_out_2[ss.str()] = vec_js_position_detach_ik_results;
            }
            else
            {
                printf("Check the robot controller package!\n");
                return false;
            }
        }


        //// Default JS Position - attach
        std::vector<double> default_vec_cs_pose_attach = vec_tool_changing_default_attach_CS_pose_sensor_frame; // [m], [deg]
        printVector(default_vec_cs_pose_attach, "default_vec_cs_pose_attach");
        Eigen::VectorXf cs_pose_attach(6);
        for(int j = 0; j < 3; j++)
        {
            cs_pose_attach[j] = default_vec_cs_pose_attach[j];
            cs_pose_attach[j + 3] = kDeg2Rad*default_vec_cs_pose_attach[j + 3];
        }
        Eigen::Matrix4f T_attach_sensor_frame;
        m_bp_math.pose2HTM(cs_pose_attach, T_attach_sensor_frame); // [m], [rad]

        Eigen::Matrix4f T_attach_base_frame = T_B2S_global*T_attach_sensor_frame;
        std::vector<double> pose_attach_base_frame = m_bp_math.htm2pose(T_attach_base_frame);

        //// I.K.
        // pose_attach_base_frame: [m], [rad]
        std::vector<double> vec_js_position_attach_ik_results;
        if(getJSPositionWithDefaultTCPCommand(7, pose_attach_base_frame, vec_js_position_attach_ik_results))
        {
            for(int j = 0; j < 3; j++) pose_attach_base_frame[j + 3] *= kRad2Deg;
            printVector(pose_attach_base_frame, "pose_attach_base_frame");
            printVector(vec_js_position_attach_ik_results, "vec_js_position_attach_ik_results");

            //// Base frame CS pose
            ss.str("");
            ss << "default_attaching_pose";
            for(int j = 0; j < 3; j++)
            {
                pose_attach_base_frame[j] = round(pose_attach_base_frame[j]*1e7) / 1e7; // [m]
                pose_attach_base_frame[j + 3] = round(pose_attach_base_frame[j + 3]*1e4) / 1e4; // [deg]
            }
            j_out_1[ss.str()] = pose_attach_base_frame;


            //// Base frame JS position
            ss.str("");
            //// TODO: 수정 필요 왼쪽, 오른쪽?
            ss << "default_attaching_js_position_robot_left_side";
            for(int j = 0; j < 6; j++) vec_js_position_attach_ik_results[j] = round(vec_js_position_attach_ik_results[j]*1e4) / 1e4; // [deg]
            j_out_2[ss.str()] = vec_js_position_attach_ik_results;
        }
        else
        {
            printf("Check the robot controller package!\n");
            return false;
        }




        //// Default JS Position - detach
        std::vector<double> default_vec_cs_pose_detach = vec_tool_changing_default_detach_CS_pose_sensor_frame; // [m], [deg]
        printVector(default_vec_cs_pose_detach, "default_vec_cs_pose_detach");
        Eigen::VectorXf cs_pose_detach(6);
        for(int j = 0; j < 3; j++)
        {
            cs_pose_detach[j] = default_vec_cs_pose_detach[j];
            cs_pose_detach[j + 3] = kDeg2Rad*default_vec_cs_pose_detach[j + 3];
        }
        Eigen::Matrix4f T_detach_sensor_frame;
        m_bp_math.pose2HTM(cs_pose_detach, T_detach_sensor_frame); // [m], [rad]

        Eigen::Matrix4f T_detach_base_frame = T_B2S_global*T_detach_sensor_frame;
        std::vector<double> pose_detach_base_frame = m_bp_math.htm2pose(T_detach_base_frame);

        //// I.K.
        // pose_detach_base_frame: [m], [rad]
        std::vector<double> vec_js_position_detach_ik_results;
        if(getJSPositionWithDefaultTCPCommand(7, pose_detach_base_frame, vec_js_position_detach_ik_results))
        {
            for(int j = 0; j < 3; j++) pose_detach_base_frame[j + 3] *= kRad2Deg;
            printVector(pose_detach_base_frame, "pose_detach_base_frame");
            printVector(vec_js_position_detach_ik_results, "vec_js_position_detach_ik_results");

            //// Base frame CS pose
            ss.str("");
            ss << "default_detaching_pose";
            for(int j = 0; j < 3; j++)
            {
                pose_detach_base_frame[j] = round(pose_detach_base_frame[j]*1e7) / 1e7; // [m]
                pose_detach_base_frame[j + 3] = round(pose_detach_base_frame[j + 3]*1e4) / 1e4; // [deg]
            }
            j_out_1[ss.str()] = pose_detach_base_frame;


            //// Base frame JS position
            ss.str("");
            ss << "default_detaching_js_position";
            for(int j = 0; j < 6; j++) vec_js_position_detach_ik_results[j] = round(vec_js_position_detach_ik_results[j]*1e4) / 1e4; // [deg]
            j_out_2[ss.str()] = vec_js_position_detach_ik_results;
        }
        else
        {
            printf("Check the robot controller package!\n");
            return false;
        }


        std::ofstream ofs_out_1(std::string("/home/") + USER_NAME + "/robot_control_ws/src/robot_control_ui/robot_control_ui/config/tool_changing_pose/tool_changing_cs_pose_base_frame.json");
        ofs_out_1 << j_out_1.dump(4) << std::endl;

        std::ofstream ofs_out_2(std::string("/home/") + USER_NAME + "/robot_control_ws/src/robot_control_ui/robot_control_ui/config/tool_changing_pose/tool_changing_js_position_base_frame.json");
        ofs_out_2 << j_out_2.dump(4) << std::endl;


        return true;
    }
    catch(std::string &e)
    {
        return false;
    }
}


//// PCL Test
bool BinPickingNode::doPCLTestFunction(taskTemplateMatchingParameter &parameter, bool b_matching, bool do_scan_sampling, size_t sampling_num, size_t feature_ext_method, double voxel_downsampling_size) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = do_scan_sampling;
    req->sampling_num = sampling_num;
    req->robot_id = m_robot_id;
    req->feature_ext_method = feature_ext_method;
    req->voxel_downsampling_size = voxel_downsampling_size;

    req->do_scan_sampling = parameter.do_scan_sampling;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    // Segmentation method
    req->segmentation_method = parameter.segmentation_method;
	// Region growing segmentation
	req->rgs_normal_k = parameter.rgs_normal_K;
	req->rgs_min_size = parameter.rgs_min_size;
	req->rgs_max_size = parameter.rgs_max_size;
	req->rgs_thre_angle = parameter.rgs_thre_angle;
	req->rgs_thre_curvature = parameter.rgs_thre_curvature;
	req->rgs_neighbor_k = parameter.rgs_neighbor_K;
	// Color-based region growing segmentation
	req->color_rgs_neighbor_r = parameter.color_rgs_neighbor_R;
	req->color_rgs_thre_1st_color = parameter.color_rgs_thre_1st_color;
	req->color_rgs_thre_2nd_color = parameter.color_rgs_thre_2nd_color;
	req->color_rgs_min_cluster_size = parameter.color_rgs_min_cluster_size;
	req->color_rgs_neighbor_r_2 = parameter.color_rgs_neighbor_R_2;
	req->color_rgs_thre_1st_color_2 = parameter.color_rgs_thre_1st_color_2;
	req->color_rgs_thre_2nd_color_2 = parameter.color_rgs_thre_2nd_color_2;
	req->color_rgs_min_cluster_size_2 = parameter.color_rgs_min_cluster_size_2;
	// Euclidean cluster segmentation
	req->euclidean_cluster_tol = parameter.euclidean_cluster_tol;
	req->euclidean_min_cluster_size = parameter.euclidean_min_cluster_size;
	req->euclidean_max_cluster_size = parameter.euclidean_max_cluster_size;

    ROS_LOG_INFO("Do PCL Test Function!");

    while (!doPCLTestClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = doPCLTestClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        if (recv->is_pose == true) {
            ROS_LOG_INFO("Test Function Success!(Callback)");
            return true;
        } else {
            ROS_LOG_INFO("Test Function Failure!(Callback)");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

//// PCL Feature Extraction
bool BinPickingNode::doPCLFunctionFeatureExtraction(taskTemplateMatchingParameter &parameter) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
    req->do_scan_sampling = parameter.do_scan_sampling;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;
    req->voxel_downsampling_size = parameter.voxel_downsampling_size;
    req->feature_ext_method = parameter.feature_ext_method;

    ROS_LOG_INFO("Do PCL Test Function!");

    while (!doPCLFunctionFeatureExtractionClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = doPCLFunctionFeatureExtractionClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        if (recv->is_pose == true) {
            ROS_LOG_INFO("Test Function Success!(Callback)");
            return true;
        } else {
            ROS_LOG_INFO("Test Function Failure!(Callback)");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

//// PCL Segmentation
bool BinPickingNode::doPCLFunctionSegmentation(taskTemplateMatchingParameter &parameter)
{
    auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();

    req->do_scan_sampling = parameter.do_scan_sampling;
    req->sampling_num = parameter.sampling_num;
    req->robot_id = m_robot_id;

    // Target id
    req->target_id = parameter.target_id;
    req->target_name = parameter.target_name;

    // Segmentation method
    req->segmentation_method = parameter.segmentation_method;
	// Region growing segmentation
	req->rgs_normal_k = parameter.rgs_normal_K;
	req->rgs_min_size = parameter.rgs_min_size;
	req->rgs_max_size = parameter.rgs_max_size;
	req->rgs_thre_angle = parameter.rgs_thre_angle;
	req->rgs_thre_curvature = parameter.rgs_thre_curvature;
	req->rgs_neighbor_k = parameter.rgs_neighbor_K;
	// Color-based region growing segmentation
	req->color_rgs_neighbor_r = parameter.color_rgs_neighbor_R;
	req->color_rgs_thre_1st_color = parameter.color_rgs_thre_1st_color;
	req->color_rgs_thre_2nd_color = parameter.color_rgs_thre_2nd_color;
	req->color_rgs_min_cluster_size = parameter.color_rgs_min_cluster_size;
	req->color_rgs_neighbor_r_2 = parameter.color_rgs_neighbor_R_2;
	req->color_rgs_thre_1st_color_2 = parameter.color_rgs_thre_1st_color_2;
	req->color_rgs_thre_2nd_color_2 = parameter.color_rgs_thre_2nd_color_2;
	req->color_rgs_min_cluster_size_2 = parameter.color_rgs_min_cluster_size_2;
	// Euclidean cluster segmentation
	req->euclidean_cluster_tol = parameter.euclidean_cluster_tol;
	req->euclidean_min_cluster_size = parameter.euclidean_min_cluster_size;
	req->euclidean_max_cluster_size = parameter.euclidean_max_cluster_size;

    ROS_LOG_INFO("Do PCL Segmentation Test Function!");

    while (!doPCLFunctionSegmentationClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
    auto result = doPCLFunctionSegmentationClient_->async_send_request(req);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node_, result) == rclcpp::FutureReturnCode::SUCCESS) {
        auto recv = result.get();
        if (recv->is_pose == true) {
            ROS_LOG_INFO("Test Function Success!(Callback)");
            return true;
        } else {
            ROS_LOG_INFO("Test Function Failure!(Callback)");
            return false;
        }
    } else {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}

//// JSON CS Pose
bool BinPickingNode::setDemoJSONCSPose(std::vector<double> &output, std::string demo_name, uint16_t motion_tag)
{
    try
    {
        //// FILE PATH
        std::string file_name("bin_picking_node.cpp");
        std::string config_path_ = __FILE__;
        config_path_.resize(config_path_.length() - file_name.length());
        config_path_ += "../../config/demo_pose/";
        std::string tcp_config_path = config_path_ + "demo_blending_cs_waypoint.json";
        //// JSON INPUT
        std::vector<double> x_home, x_detach_to_home_waypoint_1, x_detach_stacking; // (티칭 필요)로봇 기본 자세
        std::vector<double> x_regrasp_zig_outdoor_waypoint_1, x_regrasp_zig_outdoor_waypoint_2, x_regrasp_zig_outdoor_waypoint_3, x_regrasp_zig_indoor_place; // (티칭 필요)재파지 지그 내/외부 로봇 자세
        std::vector<double> x_cnc_outdoor_waypoint, x_cnc_outdoor, x_cnc_indoor; // (티칭 필요)CNC 내/외부 로봇 자세
        std::vector<double> x_bin_outdoor, x_bin_outdoor_waypoint; // (티칭 필요)상자 위 기본 자세
        std::vector<double> estimated_grasp_approach_pose; // matching node로부터 추정된 파지 직전 자세

        std::stringstream ss;
        std::ifstream ifs(tcp_config_path.c_str());
        json j_in = json::parse(ifs);
        std::vector<double> pose;

        switch(motion_tag) { // JSON input
            case MotionTag::MOTION_TAG_JSON_CSMOVE_REGRASP_PICK: {
                pose = j_in[demo_name.c_str()]["x_regrasp_zig_indoor_pick"].get<std::vector<double>>();
                break;
            }
            case MotionTag::MOTION_TAG_JSON_CSMOVE_REGRASP_PLACE: {
                pose = j_in[demo_name.c_str()]["x_regrasp_zig_indoor_place"].get<std::vector<double>>();
                break;
            }
            case MotionTag::MOTION_TAG_JSON_CSMOVE_STACKING: {
                ROS_LOG_INFO("MotionTag::MOTION_TAG_JSON_CSMOVE_STACKING");
                std::vector<double> x_detach_stacking = j_in[demo_name.c_str()]["x_detach_stacking"].get<std::vector<double>>();
                //// Detaching pose for stacking
                is_detaching_stack_pose_assigned = false; // initialize
                if(is_detaching_stack_pose_assigned) { // 이미 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned detaching stack pose in blending path - OK");
                    x_detach_stacking = getStackingPose();
                } else {
                    ROS_LOG_INFO("Next detaching stack pose in blending path - OK");
                    generateStackingPose(x_detach_stacking, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->detaching_cnt_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_trans_scale_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_single_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_line_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_mode_);
                    pose = x_detach_stacking;
                }
                break;
            }
            default: {
                assert(false);
                break;
            }
        }
        printf("******************************************************\n");
        printf("**************** Demo CS Pose ******************\n");
        printf("Demo CS Pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
        printf("******************************************************\n");


        ///////////////////////////////////////////////////////////////////
        //// 1) Grasping workspace limits
        //// JSON Load
        ///////////////////////////////////////////////////////////////////
        Eigen::MatrixXd workspace(6,2); // pose limit - (min, max)
        std::vector<double> workspace_in = j_in[demo_name.c_str()]["workspace_grasping"].get<std::vector<double>>();
        if(workspace_in.size() != 12) {
            ROS_LOG_ERROR("Check JSON file! - Workspace loading error!");
            return false;
        }
        for(int i=0; i<6; i++) {
            workspace(i, 0) = workspace_in[2 * i]; // min
            workspace(i, 1) = workspace_in[2 * i + 1]; // max
        }
        printf("demo_name: %s\n", demo_name.c_str());
        printf("workspace[min]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f\n", workspace_in[0], workspace_in[2], workspace_in[4], workspace_in[6], workspace_in[8], workspace_in[10]);
        printf("workspace[max]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f\n", workspace_in[1], workspace_in[3], workspace_in[5], workspace_in[7], workspace_in[9], workspace_in[11]);
        ///////////////////////////////////////////////////////////////////

        // 2)
        if (checkWorkspace(pose, workspace)) {
            ROS_LOG_INFO("Demo JSON CS Pose OK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            ROS_LOG_INFO("Demo JSON CS Pose OK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            ROS_LOG_INFO("Demo JSON CS Pose OK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            output = pose;
            return true;
        } else {
            ROS_LOG_INFO("Demo JSON CS Pose Failure !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            ROS_LOG_INFO("Demo JSON CS Pose Failure !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            ROS_LOG_INFO("Demo JSON CS Pose Failure !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            return false; // read error
        }

    }
    catch(std::string &e)
    {
        ROS_LOG_ERROR("Check the Demo CS Pose JSON file!");
        return false;
    }
}

//// Blending
bool BinPickingNode::setBlendingPath(BlendingTraj &blending_traj, std::string demo_name, uint16_t motion_tag)
{
    try
    {
        std::vector<double> x_home, x_detach_stacking; // (티칭 필요)로봇 기본 자세
        std::vector<double> x_regrasp_zig_outdoor_waypoint_1, x_regrasp_zig_outdoor_waypoint_2, x_regrasp_zig_outdoor_waypoint_3, x_regrasp_zig_indoor_place, x_regrasp_zig_indoor_pick, x_regrasp_zig_indoor_waypoint_1; // (티칭 필요)재파지 지그 내/외부 로봇 자세
        std::vector<double> x_cnc_outdoor_waypoint, x_cnc_outdoor, x_cnc_indoor, x_cnc_to_detach_waypoint; // (티칭 필요)CNC 내/외부 로봇 자세
        std::vector<double> x_bin_outdoor, x_bin_outdoor_waypoint; // (티칭 필요)상자 위 기본 자세
        std::vector<double> x_detach_to_home_waypoint_1;
        std::vector<double> estimated_grasp_approach_pose; // 현재 스텝에서의 matching node로부터 추정된 파지 직전 자세
        std::vector<double> before_estimated_grasp_approach_pose; // 이전 스텝에서의 matching node로부터 추정된 파지 직전 자세

        //// FILE PATH
        std::string file_name("bin_picking_node.cpp");
        std::string config_path_ = __FILE__;
        config_path_.resize(config_path_.length() - file_name.length());
        config_path_ += "../../config/demo_pose/";
        std::string tcp_config_path = config_path_ + "demo_blending_cs_waypoint.json";

        std::stringstream ss;
        // std::ifstream ifs("home/bp/robot_control_ws/src/robot_control_ui/robot_control_ui/config/demo_pose/demo_blending_cs_waypoint.json");
        std::ifstream ifs(tcp_config_path.c_str());

        json j_in = json::parse(ifs);
        std::vector<std::vector<double>> set_waypoint;

        //////////////////////////////////////////////////////////
        is_blending_path_in_process_for_scan_and_matching = false;
        //////////////////////////////////////////////////////////

        switch(motion_tag) { // JSON input
            case MotionTag::MOTION_TAG_BLENDING_HOME_TO_CNC_INDOOR: {
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_HOME_TO_CNC_INDOOR");
                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_cnc_outdoor_waypoint = j_in[demo_name.c_str()]["x_cnc_outdoor_waypoint"].get<std::vector<double>>();
                x_cnc_outdoor = j_in[demo_name.c_str()]["x_cnc_outdoor"].get<std::vector<double>>();
                x_cnc_indoor = j_in[demo_name.c_str()]["x_cnc_indoor"].get<std::vector<double>>();
                set_waypoint.push_back(x_home);
                set_waypoint.push_back(x_cnc_outdoor_waypoint);
                set_waypoint.push_back(x_cnc_outdoor);
                set_waypoint.push_back(x_cnc_indoor);
                break;
            }
            case MotionTag::MOTION_TAG_BLENDING_HOME_TO_REGRASP_ZIG: {
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_HOME_TO_REGRASP_ZIG");
                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_regrasp_zig_outdoor_waypoint_1 = j_in[demo_name.c_str()]["x_regrasp_zig_outdoor_waypoint_1"].get<std::vector<double>>();
                x_regrasp_zig_outdoor_waypoint_2 = j_in[demo_name.c_str()]["x_regrasp_zig_outdoor_waypoint_2"].get<std::vector<double>>();
                x_regrasp_zig_outdoor_waypoint_3 = j_in[demo_name.c_str()]["x_regrasp_zig_outdoor_waypoint_3"].get<std::vector<double>>();
                set_waypoint.push_back(x_home);
                set_waypoint.push_back(x_regrasp_zig_outdoor_waypoint_1);
                set_waypoint.push_back(x_regrasp_zig_outdoor_waypoint_2);
                set_waypoint.push_back(x_regrasp_zig_outdoor_waypoint_3);
                break;
            }
            case MotionTag::MOTION_TAG_BLENDING_REGRASP_ZIG_TO_CNC_INDOOR: {
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_REGRASP_ZIG_TO_CNC_INDOOR");
                x_regrasp_zig_indoor_waypoint_1 = j_in[demo_name.c_str()]["x_regrasp_zig_indoor_waypoint_1"].get<std::vector<double>>();
                x_cnc_outdoor_waypoint = j_in[demo_name.c_str()]["x_cnc_outdoor_waypoint"].get<std::vector<double>>();
                x_cnc_outdoor = j_in[demo_name.c_str()]["x_cnc_outdoor"].get<std::vector<double>>();
                x_cnc_indoor = j_in[demo_name.c_str()]["x_cnc_indoor"].get<std::vector<double>>();
                set_waypoint.push_back(x_regrasp_zig_indoor_waypoint_1);
                set_waypoint.push_back(x_cnc_outdoor_waypoint);
                set_waypoint.push_back(x_cnc_outdoor);
                set_waypoint.push_back(x_cnc_indoor);
                break;
            }
            case MotionTag::MOTION_TAG_BLENDING_CNC_INDOOR_TO_DETACH: {
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_CNC_INDOOR_TO_DETACH");
                x_cnc_indoor = j_in[demo_name.c_str()]["x_cnc_indoor"].get<std::vector<double>>();
                x_cnc_outdoor = j_in[demo_name.c_str()]["x_cnc_outdoor"].get<std::vector<double>>();
                x_cnc_to_detach_waypoint = j_in[demo_name.c_str()]["x_cnc_to_detach_waypoint"].get<std::vector<double>>();
                x_detach_stacking = j_in[demo_name.c_str()]["x_detach_stacking"].get<std::vector<double>>();

                //// Detaching pose for stacking
                is_detaching_stack_pose_assigned = false; // initialize

                if(is_detaching_stack_pose_assigned) { // 이미 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned detaching stack pose in blending path - OK");
                    x_detach_stacking = getStackingPose();
                } else {
                    ROS_LOG_INFO("Next detaching stack pose in blending path - OK");
                    generateStackingPose(x_detach_stacking, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->detaching_cnt_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_trans_scale_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_single_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_line_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_mode_);
                }
                std::vector<double> x_detach_stacking_waypoint = x_detach_stacking;
                x_detach_stacking_waypoint[2] = x_cnc_to_detach_waypoint[2]; // 이전 자세의 높이와 동일하도록

                set_waypoint.push_back(x_cnc_indoor);
                set_waypoint.push_back(x_cnc_outdoor);
                set_waypoint.push_back(x_cnc_to_detach_waypoint);
                set_waypoint.push_back(x_detach_stacking_waypoint);
                set_waypoint.push_back(x_detach_stacking);
                break;
            }
            case MotionTag::MOTION_TAG_BLENDING_DETACH_TO_HOME_1: {
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_DETACH_TO_HOME_1");
                x_detach_stacking = j_in[demo_name.c_str()]["x_detach_stacking"].get<std::vector<double>>();
                x_cnc_outdoor = j_in[demo_name.c_str()]["x_cnc_outdoor"].get<std::vector<double>>();
                x_detach_to_home_waypoint_1 = j_in[demo_name.c_str()]["x_detach_to_home_waypoint_1"].get<std::vector<double>>();
                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();

                //// Detaching pose for stacking
                if(is_detaching_stack_pose_assigned) { // 이미 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned detaching stack pose in blending path - OK");
                    x_detach_stacking = getStackingPose();
                } else {
                    ROS_LOG_INFO("Next detaching stack pose in blending path - OK");
                    generateStackingPose(x_detach_stacking, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->detaching_cnt_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_trans_scale_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_single_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_line_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_mode_);
                }

                std::vector<double> x_detach_stacking_waypoint = x_detach_stacking;
                x_detach_stacking_waypoint[2] = x_cnc_outdoor[2]; // 이전 자세의 높이와 동일하도록

                set_waypoint.push_back(x_detach_stacking);
                set_waypoint.push_back(x_detach_stacking_waypoint);
                set_waypoint.push_back(x_cnc_outdoor);
                set_waypoint.push_back(x_detach_to_home_waypoint_1);
                set_waypoint.push_back(x_home);
                break;
            }

            case MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE: { // bin-picking grasping pose
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE");
                bool is_motion_feasible = false;

                x_detach_stacking = j_in[demo_name.c_str()]["x_detach_stacking"].get<std::vector<double>>();
                x_detach_to_home_waypoint_1 = j_in[demo_name.c_str()]["x_detach_to_home_waypoint_1"].get<std::vector<double>>();
                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_bin_outdoor = j_in[demo_name.c_str()]["x_bin_outdoor"].get<std::vector<double>>();

                //// Grasping pose
                if(is_blending_approach_grasp_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned grasping approach pose in blending path - OK");
                    estimated_grasp_approach_pose = getDetectedApproachPose();
                    is_motion_feasible = true;
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                } else { // matching node로부터 수신
                    if(doGrasping(estimated_grasp_approach_pose, GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - OK");
                        is_motion_feasible = true;
                    } else {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - infeasible!");
                    }
                }

                //// Detaching pose for stacking
                if(is_detaching_stack_pose_assigned) { // 이미 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned detaching stack pose in blending path - OK");
                    x_detach_stacking = getStackingPose();
                } else {
                    ROS_LOG_INFO("Next detaching stack pose in blending path - OK");
                    generateStackingPose(x_detach_stacking, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->detaching_cnt_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_trans_scale_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_single_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_line_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_mode_);
                }
                std::vector<double> x_detach_stacking_waypoint = x_detach_stacking;
                x_detach_stacking_waypoint[2] = x_detach_stacking[2] + 0.04; // 이전 자세의 높이와 동일하도록
                x_detach_stacking_waypoint[3] = x_detach_to_home_waypoint_1[3];
                x_detach_stacking_waypoint[4] = x_detach_to_home_waypoint_1[4];
                x_detach_stacking_waypoint[5] = x_detach_to_home_waypoint_1[5];

                if(x_detach_to_home_waypoint_1[2] < x_detach_stacking_waypoint[2]) {
                    x_detach_to_home_waypoint_1[2] = x_detach_stacking_waypoint[2];
                }

                if(is_motion_feasible) {
                    for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경
                        if(i != 2) {
                            x_bin_outdoor[i] = estimated_grasp_approach_pose[i]; // [m]
                        }
                        x_bin_outdoor[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                    }
                    printf("x_bin_outdoor(modified): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                    if(!is_task_in_progress) { // 초기에는 HOME에서 시작
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE - initial progress");
                        set_waypoint.push_back(x_home);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(estimated_grasp_approach_pose);
                        is_task_in_progress = true; // task process start flag
                    } else {
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE - in progress");
                        set_waypoint.push_back(x_detach_stacking_waypoint);
                        set_waypoint.push_back(x_detach_to_home_waypoint_1);
                        set_waypoint.push_back(x_home);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(estimated_grasp_approach_pose);
                    }
                } else { // motion infeasible한 경우, 이 경우, home으로 돌아가도록 처리
                    ROS_LOG_INFO("Estimated grasping approach pose in blending path - doGrasping failure!");
                    if(is_task_in_progress) {
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_TO_HOME_POSE - blending path infeasible");
                        set_waypoint.push_back(x_detach_stacking_waypoint);
                        set_waypoint.push_back(x_detach_to_home_waypoint_1);
                        set_waypoint.push_back(x_home);
                    }
                }
                break;
            }

            case MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_GRASP_APPROACH_POSE: { // bin-picking grasping pose
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_GRASP_APPROACH_POSE");
                bool is_motion_feasible = false;

                x_detach_stacking = j_in[demo_name.c_str()]["x_detach_p2p"].get<std::vector<double>>();
                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_bin_outdoor = j_in[demo_name.c_str()]["x_bin_outdoor"].get<std::vector<double>>();
                x_bin_outdoor_waypoint = j_in[demo_name.c_str()]["x_bin_outdoor_waypoint"].get<std::vector<double>>();
                printf("x_detach_stacking(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_detach_stacking[0], x_detach_stacking[1],x_detach_stacking[2],x_detach_stacking[3],x_detach_stacking[4],x_detach_stacking[5]);
                printf("x_home(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_home[0], x_home[1],x_home[2],x_home[3],x_home[4],x_home[5]);
                printf("x_bin_outdoor(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);

                //// Grasping pose
                if(is_blending_approach_grasp_pose_assigned) { // 이미 수신되어서 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned grasping approach pose in blending path - OK");
                    estimated_grasp_approach_pose = getDetectedApproachPose();
                    is_motion_feasible = true;
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                } else { // matching node로부터 수신
                    if(doGrasping(estimated_grasp_approach_pose, GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - OK");
                        is_motion_feasible = true;
                    } else {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - infeasible!");
                    }
                }

                if(is_motion_feasible) {
                    std::vector<double> l_start_pose = m_measured_pose;
                    for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경
                        // a) x_bin_outdoor modified
                        if(i != 2) {
                            x_bin_outdoor[i] = estimated_grasp_approach_pose[i]; // [m]
                        }
                        x_bin_outdoor[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                        x_bin_outdoor_waypoint[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                    }
                    printf("x_bin_outdoor(modified): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                    if(!is_task_in_progress) { // 초기에는 HOME에서 시작
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_GRASP_APPROACH_POSE - initial progress");
                        set_waypoint.push_back(x_home);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(estimated_grasp_approach_pose);
                        is_task_in_progress = true; // task process start flag
                    } else {
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_GRASP_APPROACH_POSE - in progress");
                        for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경
                            // b) x_home orientation modified
                            x_home[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                        }

                        set_waypoint.push_back(l_start_pose);
                        set_waypoint.push_back(x_bin_outdoor_waypoint);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(estimated_grasp_approach_pose);
                    }
                } else { // motion infeasible한 경우, 이 경우, home으로 돌아가도록 처리
                    ROS_LOG_INFO("Estimated grasping approach pose in blending path - doGrasping failure!");
                    // if(is_task_in_progress) {
                    //     ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_TO_HOME_POSE - blending path infeasible");
                    //     set_waypoint.push_back(x_detach_stacking);
                    //     set_waypoint.push_back(x_detach_to_home_waypoint_1);
                    //     set_waypoint.push_back(x_home);
                    // }
                }
                break;
            }

            case MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_BEFORE_GRASP_APPROACH_POSE: { // bin-picking grasping pose
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_BEFORE_GRASP_APPROACH_POSE");
                bool is_motion_feasible = false;

                x_detach_stacking = j_in[demo_name.c_str()]["x_detach_p2p"].get<std::vector<double>>();
                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_bin_outdoor = j_in[demo_name.c_str()]["x_bin_outdoor"].get<std::vector<double>>();
                x_bin_outdoor_waypoint = j_in[demo_name.c_str()]["x_bin_outdoor_waypoint"].get<std::vector<double>>();
                printf("x_detach_stacking(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_detach_stacking[0], x_detach_stacking[1],x_detach_stacking[2],x_detach_stacking[3],x_detach_stacking[4],x_detach_stacking[5]);
                printf("x_home(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_home[0], x_home[1],x_home[2],x_home[3],x_home[4],x_home[5]);
                printf("x_bin_outdoor(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);

                //// Grasping pose
                if(is_blending_approach_grasp_pose_assigned) { // 이미 수신되어서 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned before grasping approach pose in blending path - OK");
                    before_estimated_grasp_approach_pose = getBeforeDetectedApproachPose();
                    is_motion_feasible = true;
                    printf("Before estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", before_estimated_grasp_approach_pose[0], before_estimated_grasp_approach_pose[1],before_estimated_grasp_approach_pose[2],before_estimated_grasp_approach_pose[3],before_estimated_grasp_approach_pose[4],before_estimated_grasp_approach_pose[5]);
                } else { // matching node로부터 수신
                    if(doGrasping(before_estimated_grasp_approach_pose, GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                        ROS_LOG_INFO("Before estimated grasping approach pose in blending path - OK");
                        is_motion_feasible = true;
                    } else {
                        ROS_LOG_INFO("Before estimated grasping approach pose in blending path - infeasible!");
                    }
                }

                if(is_motion_feasible) {
                    std::vector<double> l_start_pose = m_measured_pose;
                    for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경
                        // a) x_bin_outdoor modified
                        if(i != 2) {
                            x_bin_outdoor[i] = before_estimated_grasp_approach_pose[i]; // [m]
                        }
                        x_bin_outdoor[i + 3] = before_estimated_grasp_approach_pose[i + 3]; // [deg]
                        x_bin_outdoor_waypoint[i + 3] = before_estimated_grasp_approach_pose[i + 3]; // [deg]
                    }
                    printf("x_bin_outdoor(modified): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);
                    printf("before_estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", before_estimated_grasp_approach_pose[0], before_estimated_grasp_approach_pose[1],before_estimated_grasp_approach_pose[2],before_estimated_grasp_approach_pose[3],before_estimated_grasp_approach_pose[4],before_estimated_grasp_approach_pose[5]);
                    if(!is_task_in_progress) { // 초기에는 HOME에서 시작
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_GRASP_APPROACH_POSE - initial progress");
                        set_waypoint.push_back(x_home);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(before_estimated_grasp_approach_pose);
                        is_task_in_progress = true; // task process start flag
                    } else {
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_HOME_TO_GRASP_APPROACH_POSE - in progress");
                        for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경
                            // b) x_home orientation modified
                            x_home[i + 3] = before_estimated_grasp_approach_pose[i + 3]; // [deg]
                        }

                        set_waypoint.push_back(l_start_pose);
                        set_waypoint.push_back(x_bin_outdoor_waypoint);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(before_estimated_grasp_approach_pose);
                    }
                } else { // motion infeasible한 경우, 이 경우, home으로 돌아가도록 처리
                    ROS_LOG_INFO("Estimated grasping approach pose in blending path - doGrasping failure!");
                    // if(is_task_in_progress) {
                    //     ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_TO_HOME_POSE - blending path infeasible");
                    //     set_waypoint.push_back(x_detach_stacking);
                    //     set_waypoint.push_back(x_detach_to_home_waypoint_1);
                    //     set_waypoint.push_back(x_home);
                    // }
                }
                break;
            }

            case MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_P2P_POSE: { // bin-picking grasping pose
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_P2P_POSE");
                bool is_motion_feasible = false;

                x_detach_stacking = j_in[demo_name.c_str()]["x_detach_p2p"].get<std::vector<double>>();
                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_bin_outdoor = j_in[demo_name.c_str()]["x_bin_outdoor"].get<std::vector<double>>();
                x_bin_outdoor_waypoint = j_in[demo_name.c_str()]["x_bin_outdoor_waypoint"].get<std::vector<double>>();
                printf("x_detach_p2p(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_detach_stacking[0], x_detach_stacking[1],x_detach_stacking[2],x_detach_stacking[3],x_detach_stacking[4],x_detach_stacking[5]);
                printf("x_home(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_home[0], x_home[1],x_home[2],x_home[3],x_home[4],x_home[5]);
                printf("x_bin_outdoor(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);

                //// Grasping pose
                if(is_blending_approach_grasp_pose_assigned) { // 이미 수신되어서 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned grasping approach pose in blending path - OK");
                    estimated_grasp_approach_pose = getDetectedApproachPose();
                    is_motion_feasible = true;
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                } else { // matching node로부터 수신
                    if(doGrasping(estimated_grasp_approach_pose, GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - OK");
                        is_motion_feasible = true;
                    } else {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - infeasible!");
                    }
                }

                if(is_motion_feasible) {

                    std::vector<double> l_start_pose = m_measured_pose;
                    for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경

                        // a) x_bin_outdoor modified
                        if(i != 2) {
                            x_bin_outdoor[i] = estimated_grasp_approach_pose[i]; // [m]
                        }
                        x_bin_outdoor[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                        x_bin_outdoor_waypoint[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]

                        // b) x_home orientation modified
                        x_home[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]

                        // c) x_detach_stacking orientation modified
                        x_detach_stacking[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                    }


                    printf("x_bin_outdoor(modified): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);

                    //////////////////////////////////////////////////////////
                    is_blending_path_in_process_for_scan_and_matching = true;
                    //////////////////////////////////////////////////////////

                    if(!is_task_in_progress) { // 초기
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_P2P_POSE - initial progress");
                        set_waypoint.push_back(l_start_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_bin_outdoor_waypoint);
                        set_waypoint.push_back(x_detach_stacking);
                        is_task_in_progress = true; // task process start flag
                    } else {
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_P2P_POSE - in progress");
                        set_waypoint.push_back(l_start_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_bin_outdoor_waypoint);
                        set_waypoint.push_back(x_detach_stacking);
                    }
                } else { // motion infeasible한 경우, 이 경우, home으로 돌아가도록 처리
                    ROS_LOG_INFO("Estimated grasping approach pose in blending path - doGrasping failure!");
                    // if(is_task_in_progress) {
                    //     ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_TO_HOME_POSE - blending path infeasible");
                    //     set_waypoint.push_back(x_detach_stacking);
                    //     set_waypoint.push_back(x_detach_to_home_waypoint_1);
                    //     set_waypoint.push_back(x_home);
                    // }
                }
                break;
            }

            case MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE: { // bin-picking grasping pose
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE");
                bool is_motion_feasible = false;

                x_detach_stacking = j_in[demo_name.c_str()]["x_detach_stacking"].get<std::vector<double>>();
                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_bin_outdoor = j_in[demo_name.c_str()]["x_bin_outdoor"].get<std::vector<double>>();
                x_bin_outdoor_waypoint = j_in[demo_name.c_str()]["x_bin_outdoor_waypoint"].get<std::vector<double>>();
                printf("x_detach_stacking(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_detach_stacking[0], x_detach_stacking[1],x_detach_stacking[2],x_detach_stacking[3],x_detach_stacking[4],x_detach_stacking[5]);
                printf("x_home(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_home[0], x_home[1],x_home[2],x_home[3],x_home[4],x_home[5]);
                printf("x_bin_outdoor(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);

                //// Grasping pose
                if(is_blending_approach_grasp_pose_assigned) { // 이미 수신되어서 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned grasping approach pose in blending path - OK");
                    estimated_grasp_approach_pose = getDetectedApproachPose();
                    is_motion_feasible = true;
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                } else { // matching node로부터 수신
                    if(doGrasping(estimated_grasp_approach_pose, GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - OK");
                        is_motion_feasible = true;
                    } else {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - infeasible!");
                    }
                }

                if(is_motion_feasible) {

                    std::vector<double> l_start_pose = m_measured_pose;
                    for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경

                        // a) x_bin_outdoor modified
                        if(i != 2) {
                            x_bin_outdoor[i] = estimated_grasp_approach_pose[i]; // [m]
                        }
                        x_bin_outdoor[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                        x_bin_outdoor_waypoint[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]

                        // b) x_home orientation modified
                        x_home[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]

                        // c) x_detach_stacking orientation modified
                        // x_detach_stacking[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                    }


                    printf("x_bin_outdoor(modified): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);

                    is_detaching_stack_pose_assigned = false; // initialize
                    if(is_detaching_stack_pose_assigned) { // 이미 자세가 할당된 경우
                        ROS_LOG_INFO("Assigned detaching stack pose in blending path - OK");
                        x_detach_stacking = getStackingPose();
                    } else {
                        ROS_LOG_INFO("Next detaching stack pose in blending path - OK");
                        generateStackingPose(x_detach_stacking, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->detaching_cnt_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_trans_scale_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_single_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_line_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_mode_);
                    }

                    ////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////
                    //// Detach Stacking 자세 입장하기 전 z 높이 추가 (15cm 위에서 내려오도록 처리)
                    std::vector<double> x_detach_stacking_z_add = x_detach_stacking;
                    x_detach_stacking_z_add[2] += 0.15;
                    ////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////


                    //////////////////////////////////////////////////////////
                    is_blending_path_in_process_for_scan_and_matching = true;
                    //////////////////////////////////////////////////////////

                    if(!is_task_in_progress) { // 초기
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE - initial progress");
                        set_waypoint.push_back(l_start_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_bin_outdoor_waypoint);
                        set_waypoint.push_back(x_detach_stacking_z_add);
                        set_waypoint.push_back(x_detach_stacking);
                        is_task_in_progress = true; // task process start flag
                    } else {
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_JSON_POSE - in progress");
                        set_waypoint.push_back(l_start_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_bin_outdoor_waypoint);
                        set_waypoint.push_back(x_detach_stacking_z_add);
                        set_waypoint.push_back(x_detach_stacking);
                    }
                } else { // motion infeasible한 경우, 이 경우, home으로 돌아가도록 처리
                    ROS_LOG_INFO("Estimated grasping approach pose in blending path - doGrasping failure!");
                    // if(is_task_in_progress) {
                    //     ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_TO_HOME_POSE - blending path infeasible");
                    //     set_waypoint.push_back(x_detach_stacking);
                    //     set_waypoint.push_back(x_detach_to_home_waypoint_1);
                    //     set_waypoint.push_back(x_home);
                    // }
                }
                break;
            }

            case MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE: { // bin-picking grasping pose
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE");
                bool is_motion_feasible = false;

                //// NOTICE: detected_zig_pose
                x_detach_stacking = getDetectedZigPose();
                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_bin_outdoor = j_in[demo_name.c_str()]["x_bin_outdoor"].get<std::vector<double>>();
                x_bin_outdoor_waypoint = j_in[demo_name.c_str()]["x_bin_outdoor_waypoint"].get<std::vector<double>>();
                printf("x_detach_stacking(DETECTED_ZIG_POSE): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_detach_stacking[0], x_detach_stacking[1],x_detach_stacking[2],x_detach_stacking[3],x_detach_stacking[4],x_detach_stacking[5]);
                printf("x_home(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_home[0], x_home[1],x_home[2],x_home[3],x_home[4],x_home[5]);
                printf("x_bin_outdoor(JSON): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);

                //// Grasping pose
                if(is_blending_approach_grasp_pose_assigned) { // 이미 수신되어서 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned grasping approach pose in blending path - OK");
                    estimated_grasp_approach_pose = getDetectedApproachPose();
                    is_motion_feasible = true;
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                } else { // matching node로부터 수신
                    if(doGrasping(estimated_grasp_approach_pose, GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - OK");
                        is_motion_feasible = true;
                    } else {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - infeasible!");
                    }
                }

                if(is_motion_feasible) {

                    std::vector<double> l_start_pose = m_measured_pose;
                    for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경

                        // a) x_bin_outdoor modified
                        if(i != 2) {
                            x_bin_outdoor[i] = estimated_grasp_approach_pose[i]; // [m]
                        }
                        x_bin_outdoor[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]

                        // x_bin_outdoor_waypoint[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                        x_bin_outdoor_waypoint[i + 3] = x_detach_stacking[i + 3]; // [deg]

                        // b) x_home orientation modified
                        x_home[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]

                        // c) x_detach_stacking orientation modified
                        // x_detach_stacking[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                    }

                    printf("x_bin_outdoor(modified): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);

                    is_detaching_stack_pose_assigned = false; // initialize
                    if(is_detaching_stack_pose_assigned) { // 이미 자세가 할당된 경우
                        ROS_LOG_INFO("[detected_zig_pose] Assigned detaching stack pose in blending path - OK");
                        //// NOTICE: detected_zig_pose_
                        x_detach_stacking = getStackingPose();
                    } else {
                        //// NOTICE: detected_zig_pose_
                        ROS_LOG_INFO("[detected_zig_pose] Next detaching stack pose in blending path - OK");
                        generateStackingPose(x_detach_stacking, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->detaching_cnt_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_trans_scale_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_single_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_line_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_mode_);
                    }
                    printf("x_detach_stacking: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_detach_stacking[0], x_detach_stacking[1],x_detach_stacking[2],x_detach_stacking[3],x_detach_stacking[4],x_detach_stacking[5]);

                    ////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////
                    //// Detach Stacking 자세 입장하기 전 z 높이 추가 (10cm 위에서 내려오도록 처리)
                    std::vector<double> x_detach_stacking_z_add = x_detach_stacking;
                    x_detach_stacking_z_add[2] += 0.15;
                    ////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////


                    //////////////////////////////////////////////////////////
                    is_blending_path_in_process_for_scan_and_matching = true;
                    //////////////////////////////////////////////////////////

                    if(!is_task_in_progress) { // 초기
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE - initial progress");
                        set_waypoint.push_back(l_start_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_bin_outdoor_waypoint);
                        set_waypoint.push_back(x_detach_stacking_z_add);
                        set_waypoint.push_back(x_detach_stacking);
                        is_task_in_progress = true; // task process start flag
                    } else {
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_PICK_AND_PLACE_GRASP_APPROACH_POSE_TO_STACKING_DETECTED_ZIG_POSE - in progress");
                        set_waypoint.push_back(l_start_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_bin_outdoor_waypoint);
                        set_waypoint.push_back(x_detach_stacking_z_add);
                        set_waypoint.push_back(x_detach_stacking);
                    }
                } else { // motion infeasible한 경우, 이 경우, home으로 돌아가도록 처리
                    ROS_LOG_INFO("Estimated grasping approach pose in blending path - doGrasping failure!");
                    // if(is_task_in_progress) {
                    //     ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_TO_HOME_POSE - blending path infeasible");
                    //     set_waypoint.push_back(x_detach_stacking);
                    //     set_waypoint.push_back(x_detach_to_home_waypoint_1);
                    //     set_waypoint.push_back(x_home);
                    // }
                }
                break;
            }





            case MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_REGRASPING_ZIG: { // 재파지 수행, bin-picking grasping pose
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_REGRASPING_ZIG");
                bool is_motion_feasible = false;

                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_bin_outdoor = j_in[demo_name.c_str()]["x_bin_outdoor"].get<std::vector<double>>();
                x_regrasp_zig_outdoor_waypoint_1 = j_in[demo_name.c_str()]["x_regrasp_zig_outdoor_waypoint_1"].get<std::vector<double>>();
                x_regrasp_zig_outdoor_waypoint_2 = j_in[demo_name.c_str()]["x_regrasp_zig_outdoor_waypoint_2"].get<std::vector<double>>();
                x_regrasp_zig_outdoor_waypoint_3 = j_in[demo_name.c_str()]["x_regrasp_zig_outdoor_waypoint_3"].get<std::vector<double>>();

                if(is_blending_approach_grasp_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned grasping approach pose in blending path - OK");
                    estimated_grasp_approach_pose = getDetectedApproachPose();
                    is_motion_feasible = true;
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                } else { // matching node로부터 수신
                    if(doGrasping(estimated_grasp_approach_pose, GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - OK");
                        is_motion_feasible = true;
                    } else {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - infeasible!");
                    }
                }
                if(is_motion_feasible) {
                    for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경
                        if(i != 2) {
                            x_bin_outdoor[i] = estimated_grasp_approach_pose[i]; // [m]
                        }
                        x_bin_outdoor[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                    }
                    printf("x_bin_outdoor(modified): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                    if(!is_task_in_progress) { // 초기에는 HOME에서 시작
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_REGRASPING_ZIG - initial progress");
                        set_waypoint.push_back(estimated_grasp_approach_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_home);
                        is_task_in_progress = true; // task process start flag
                    } else {
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_REGRASPING_ZIG - in progress");
                        set_waypoint.push_back(estimated_grasp_approach_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_home);
                        set_waypoint.push_back(x_regrasp_zig_outdoor_waypoint_1);
                        set_waypoint.push_back(x_regrasp_zig_outdoor_waypoint_2);
                        set_waypoint.push_back(x_regrasp_zig_outdoor_waypoint_3);
                    }
                } else { // motion infeasible한 경우
                    ROS_LOG_INFO("Estimated grasping approach pose in blending path - doGrasping failure!");
                }
                break;
            }
            case MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_CNC_INNER: { // 재파지 수행, bin-picking grasping pose
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_CNC_INNER");
                bool is_motion_feasible = false;

                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_bin_outdoor = j_in[demo_name.c_str()]["x_bin_outdoor"].get<std::vector<double>>();
                x_cnc_outdoor_waypoint = j_in[demo_name.c_str()]["x_cnc_outdoor_waypoint"].get<std::vector<double>>();
                x_cnc_outdoor = j_in[demo_name.c_str()]["x_cnc_outdoor"].get<std::vector<double>>();
                x_cnc_indoor = j_in[demo_name.c_str()]["x_cnc_indoor"].get<std::vector<double>>();

                if(is_blending_approach_grasp_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned grasping approach pose in blending path - OK");
                    estimated_grasp_approach_pose = getDetectedApproachPose();
                    is_motion_feasible = true;
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                } else { // matching node로부터 수신
                    if(doGrasping(estimated_grasp_approach_pose, GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - OK");
                        is_motion_feasible = true;
                    } else {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - infeasible!");
                    }
                }
                if(is_motion_feasible) {
                    for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경
                        if(i != 2) {
                            x_bin_outdoor[i] = estimated_grasp_approach_pose[i]; // [m]
                        }
                        x_bin_outdoor[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                    }
                    printf("x_bin_outdoor(modified): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                    if(!is_task_in_progress) { // 초기에는 HOME에서 시작
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_CNC_INNER - initial progress");
                        set_waypoint.push_back(estimated_grasp_approach_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_home);
                        is_task_in_progress = true; // task process start flag
                    } else {
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_CNC_INNER - in progress");
                        set_waypoint.push_back(estimated_grasp_approach_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_home);
                        set_waypoint.push_back(x_cnc_outdoor_waypoint);
                        set_waypoint.push_back(x_cnc_outdoor);
                        set_waypoint.push_back(x_cnc_indoor);
                    }
                } else {
                    ROS_LOG_INFO("Estimated grasping approach pose in blending path - doGrasping failure!");
                }
                break;
            }


            case MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_DETACH: { // 재파지 없이 픽앤플레이스의 detach zig로 이동, bin-picking grasping pose
                ROS_LOG_INFO("BLENDING_MOTION_TAG: MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_DETACH");
                bool is_motion_feasible = false;

                x_home = j_in[demo_name.c_str()]["x_home"].get<std::vector<double>>();
                x_bin_outdoor = j_in[demo_name.c_str()]["x_bin_outdoor"].get<std::vector<double>>();
                x_cnc_outdoor_waypoint = j_in[demo_name.c_str()]["x_cnc_outdoor_waypoint"].get<std::vector<double>>();
                x_cnc_outdoor = j_in[demo_name.c_str()]["x_cnc_outdoor"].get<std::vector<double>>();
                x_detach_stacking = j_in[demo_name.c_str()]["x_detach_stacking"].get<std::vector<double>>();

                if(is_blending_approach_grasp_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned grasping approach pose in blending path - OK");
                    estimated_grasp_approach_pose = getDetectedApproachPose();
                    is_motion_feasible = true;
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                } else { // matching node로부터 수신
                    if(doGrasping(estimated_grasp_approach_pose, GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - OK");
                        is_motion_feasible = true;
                    } else {
                        ROS_LOG_INFO("Estimated grasping approach pose in blending path - infeasible!");
                    }
                }

                //// Detaching pose for stacking
                if(is_detaching_stack_pose_assigned) { // 이미 자세가 할당된 경우
                    ROS_LOG_INFO("Assigned detaching stack pose in blending path - OK");
                    x_detach_stacking = getStackingPose();
                } else {
                    ROS_LOG_INFO("Next detaching stack pose in blending path - OK");
                    generateStackingPose(x_detach_stacking, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->detaching_cnt_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_trans_scale_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_single_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_line_stack_num_, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->stacking_mode_);
                }
                std::vector<double> x_detach_stacking_waypoint = x_detach_stacking;
                x_detach_stacking_waypoint[2] = x_cnc_outdoor[2]; // 이전 자세의 높이와 동일하도록

                if(is_motion_feasible) {
                    for(int i = 0; i < 3; i++) { // x_bin_outdoor의 위치(x, y)와 방위를 파지 자세의 방위로 변경
                        if(i != 2) {
                            x_bin_outdoor[i] = estimated_grasp_approach_pose[i]; // [m]
                        }
                        x_bin_outdoor[i + 3] = estimated_grasp_approach_pose[i + 3]; // [deg]
                    }
                    printf("x_bin_outdoor(modified): %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_bin_outdoor[0], x_bin_outdoor[1],x_bin_outdoor[2],x_bin_outdoor[3],x_bin_outdoor[4],x_bin_outdoor[5]);
                    printf("estimated_grasp_approach_pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", estimated_grasp_approach_pose[0], estimated_grasp_approach_pose[1],estimated_grasp_approach_pose[2],estimated_grasp_approach_pose[3],estimated_grasp_approach_pose[4],estimated_grasp_approach_pose[5]);
                    if(!is_task_in_progress) { // 초기에는 HOME에서 시작
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_DETACH - initial progress");
                        set_waypoint.push_back(estimated_grasp_approach_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_home);
                        is_task_in_progress = true; // task process start flag
                    } else {
                        ROS_LOG_INFO("MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_DETACH - in progress");
                        set_waypoint.push_back(estimated_grasp_approach_pose);
                        set_waypoint.push_back(x_bin_outdoor);
                        set_waypoint.push_back(x_home);
                        set_waypoint.push_back(x_cnc_outdoor_waypoint);
                        set_waypoint.push_back(x_cnc_outdoor);
                        set_waypoint.push_back(x_detach_stacking_waypoint);
                        set_waypoint.push_back(x_detach_stacking);
                    }
                } else {
                    ROS_LOG_INFO("Estimated grasping approach pose in blending path - doGrasping failure!");
                }
                break;
            }

            default: {
                ROS_LOG_INFO("BLENDING_MOTION_TAG: NONE");
                assert(false);
                break;
            }
        }
        printf("******************************************************\n");
        printf("*********** Blending Waypoints Loaded! ***************\n");
        printf("******************************************************\n");

        //// Blending motion service call
        if(set_waypoint.size() != 0) {
            //// NOTICE: 주의사항: distance가 특정 방향으로 너무 짧으면 안됨 (현재 파라미터 상 10cm 이하면 동작X) --> 경로에 불연속 발생
            double xd = blending_traj.xd;
            double xdd = blending_traj.xdd;
            double waypoint_xd = blending_traj.waypoint_xd;
            double radius = blending_traj.radius;

            CsDouble x;
            task_planner_.blendingPathInit(blending_traj);


            //// JSON Load
            ///////////////////////////////////////////////////////////////////
            Eigen::MatrixXd workspace(6,2); // pose limit - (min, max)
            std::vector<double> workspace_in = j_in["default"]["workspace_reachable"].get<std::vector<double>>();
            if(workspace_in.size() != 12) {
                ROS_LOG_ERROR("Check JSON file! - Workspace loading error!");
                return false;
            }
            for(int i=0; i<6; i++) {
                workspace(i, 0) = workspace_in[2 * i]; // min
                workspace(i, 1) = workspace_in[2 * i + 1]; // max
            }
            printf("[workspace_reachable] demo_name: %s\n", demo_name.c_str());
            printf("workspace[min]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f\n", workspace_in[0], workspace_in[2], workspace_in[4], workspace_in[6], workspace_in[8], workspace_in[10]);
            printf("workspace[max]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f\n", workspace_in[1], workspace_in[3], workspace_in[5], workspace_in[7], workspace_in[9], workspace_in[11]);
            ///////////////////////////////////////////////////////////////////


            for(size_t i = 0; i < set_waypoint.size(); i++) {
                //// Check Feasible Target Pose (물리적인 값이 아닌 경우)
                //// workspace limits
                if (!checkWorkspace(set_waypoint[i], workspace)) {
                    printf("******************************************************\n");
                    printf("************* Blending path infeasible! **************\n");
                    printf("************ Unreachable waypoint exist! *************\n");
                    printf("******************************************************\n");
                    return false;
                }
                vec2Arr(set_waypoint[i], x);
                task_planner_.blendingPathPushBack(blending_traj, x, xd, xdd, waypoint_xd, radius); // (blending trajectory, target_x, xd, xdd, xd, radius)
            }

            printf("******************************************************\n");
            printf("**************** Blending Waypoints ******************\n");
            for(size_t i = 0; i < set_waypoint.size(); i++) {
                printf("set_waypoint %dth: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", i+1, set_waypoint[i][0], set_waypoint[i][1],set_waypoint[i][2],set_waypoint[i][3],set_waypoint[i][4],set_waypoint[i][5]);
            }
            printf("xd[m/s], xdd[m/s^2], waypoint_xd[m/s], radius[%]: %0.4f, %0.4f, %0.4f, %0.1f\n", xd, xdd, waypoint_xd, radius);
            printf("******************************************************\n");

            return true;
        } else {
            printf("******************************************************\n");
            printf("************* Blending path infeasible! **************\n");
            printf("******************************************************\n");
            return false;
        }

    }
    catch(std::string &e)
    {
        ROS_LOG_ERROR("Check the blending waypoint JSON file!");
        return false;
    }
}

//// JSON CS Pose
bool BinPickingNode::setDemoRecogCSPose(std::vector<double> &output, std::string demo_name, uint16_t motion_tag)
{
    try
    {
        //// Recognized grasping or zig pose
        //// FILE PATH
        std::string file_name("bin_picking_node.cpp");
        std::string config_path_ = __FILE__;
        config_path_.resize(config_path_.length() - file_name.length());
        config_path_ += "../../config/demo_pose/";
        std::string tcp_config_path = config_path_ + "demo_blending_cs_waypoint.json";
        //// JSON INPUT
        std::vector<double> workspace_in; // matching node로부터 추정된 파지 직전 자세

        std::stringstream ss;

        // std::ifstream ifs(std::string("/home/") + USER_NAME + "/robot_control_ws/src/robot_control_ui/robot_control_ui/config/demo_pose/demo_blending_cs_waypoint.json");
        std::ifstream ifs(tcp_config_path.c_str());
        json j_in = json::parse(ifs);

        //// Recognized grasping or zig pose
        bool l_is_pose_assigned = false;
        std::vector<double> pose;
        switch(motion_tag) {
            case MotionTag::MOTION_TAG_PLANAR_VACUUM_Z_FIXED_DETECTED_GRASPING_POSE: { // Detected grasping pose
                ROS_LOG_INFO("TAG: MotionTag::MOTION_TAG_PLANAR_VACUUM_Z_FIXED_DETECTED_GRASPING_POSE");
                if(is_grasping_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    std::vector<double> grasp_z_fixed_pose = getDetectedPose();
                    //// NOTICE: Square peg의 경우, 로봇의 위치정확도 문제로 인해, 과도한 접촉력 발생 --> 임시로 항상 동일한 z값을 할당
                    grasp_z_fixed_pose[2] = j_in[demo_name.c_str()]["fixed_z_position_for_planar_vacuum_gripper"];
                    printf("planar vacuum z fixed position: %0.4f\n", grasp_z_fixed_pose[2]);
                    printf("estimated_grasping_pose (for planar vacuum z fixed pose): %0.4f %0.4f %0.4f %0.1f %0.1f %0.1f\n", grasp_z_fixed_pose[0], grasp_z_fixed_pose[1],grasp_z_fixed_pose[2],grasp_z_fixed_pose[3],grasp_z_fixed_pose[4],grasp_z_fixed_pose[5]);
                    pose = grasp_z_fixed_pose;
                    l_is_pose_assigned = true;
                } else {
                    ROS_LOG_INFO("Check target CS Grasping pose! (setDemoRecogCSPose)");
                }
                break;
            }

            case MotionTag::MOTION_TAG_DETECTED_GRASPING_POSE: { // Detected grasping pose
                ROS_LOG_INFO("TAG: MotionTag::MOTION_TAG_DETECTED_GRASPING_POSE");
                if(is_grasping_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    std::vector<double> grasp_pose = getDetectedPose();
                    printf("estimated_grasping_pose: %0.4f %0.4f %0.4f %0.1f %0.1f %0.1f\n", grasp_pose[0], grasp_pose[1],grasp_pose[2],grasp_pose[3],grasp_pose[4],grasp_pose[5]);
                    pose = grasp_pose;
                    l_is_pose_assigned = true;
                } else {
                    ROS_LOG_INFO("Check target CS Grasping pose! (setDemoRecogCSPose)");
                }
                break;
            }

            case MotionTag::MOTION_TAG_PLANAR_VACUUM_Z_FIXED_BEFORE_DETECTED_GRASPING_POSE: { // Detected grasping pose
                ROS_LOG_INFO("TAG: MotionTag::MOTION_TAG_PLANAR_VACUUM_Z_FIXED_BEFORE_DETECTED_GRASPING_POSE");
                if(is_grasping_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    std::vector<double> grasp_z_fixed_pose = getBeforeDetectedPose();
                    //// NOTICE: Square peg의 경우, 로봇의 위치정확도 문제로 인해, 과도한 접촉력 발생 --> 임시로 항상 동일한 z값을 할당
                    grasp_z_fixed_pose[2] = j_in[demo_name.c_str()]["fixed_z_position_for_planar_vacuum_gripper"];
                    printf("planar vacuum z fixed position: %0.4f\n", grasp_z_fixed_pose[2]);
                    printf("estimated_grasping_pose (for planar vacuum z fixed pose): %0.4f %0.4f %0.4f %0.1f %0.1f %0.1f\n", grasp_z_fixed_pose[0], grasp_z_fixed_pose[1],grasp_z_fixed_pose[2],grasp_z_fixed_pose[3],grasp_z_fixed_pose[4],grasp_z_fixed_pose[5]);
                    pose = grasp_z_fixed_pose;
                    l_is_pose_assigned = true;
                } else {
                    ROS_LOG_INFO("Check target CS Grasping pose! (setDemoRecogCSPose)");
                }
                break;
            }

            case MotionTag::MOTION_TAG_BEFORE_DETECTED_GRASPING_POSE: { // Detected grasping pose
                ROS_LOG_INFO("TAG: MotionTag::MOTION_TAG_BEFORE_DETECTED_GRASPING_POSE");
                if(is_grasping_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    std::vector<double> grasp_pose = getBeforeDetectedPose();
                    printf("estimated_before_grasping_pose: %0.4f %0.4f %0.4f %0.1f %0.1f %0.1f\n", grasp_pose[0], grasp_pose[1],grasp_pose[2],grasp_pose[3],grasp_pose[4],grasp_pose[5]);
                    pose = grasp_pose;
                    l_is_pose_assigned = true;
                } else {
                    ROS_LOG_INFO("Check target CS Grasping pose! (setDemoRecogCSPose)");
                }
                break;
            }

            case MotionTag::MOTION_TAG_DETECTED_ZIG_POSE: { // Detected zig pose
                ROS_LOG_INFO("TAG: MotionTag::MOTION_TAG_DETECTED_ZIG_POSE");
                if(is_zig_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    std::vector<double> zig_pose = getDetectedZigPose();
                    printf("estimated_zig_pose: %0.4f %0.4f %0.4f %0.1f %0.1f %0.1f\n", zig_pose[0], zig_pose[1], zig_pose[2], zig_pose[3], zig_pose[4], zig_pose[5]);
                    pose = zig_pose;
                    l_is_pose_assigned = true;
                } else {
                    ROS_LOG_INFO("Check target CS Zig pose! (setDemoRecogCSPose)");
                }
                break;
            }

            case MotionTag::MOTION_TAG_REGRASPING_POSE: { // Detected zig pose
                ROS_LOG_INFO("TAG: MotionTag::MOTION_TAG_REGRASPING_POSE");
                if(is_zig_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    std::vector<double> regrasping_pose = j_in[demo_name.c_str()]["x_detach_regrasping"].get<std::vector<double>>();
                    printf("MOTION_TAG_REGRASPING_POSE: %0.4f %0.4f %0.4f %0.1f %0.1f %0.1f\n", regrasping_pose[0], regrasping_pose[1], regrasping_pose[2], regrasping_pose[3], regrasping_pose[4], regrasping_pose[5]);
                    pose = regrasping_pose;
                    l_is_pose_assigned = true;
                } else {
                    ROS_LOG_INFO("Check target CS Zig pose! (setDemoRecogCSPose)");
                }
                break;
            }

            case MotionTag::MOTION_TAG_REGRASPING_POSE_ORI_DETECTED: { // Detected zig pose
                ROS_LOG_INFO("TAG: MotionTag::MOTION_TAG_REGRASPING_POSE_ORI_DETECTED");
                if(is_zig_pose_assigned) { // 이미 수신되서 자세가 할당된 경우
                    std::vector<double> zig_pose = getBeforeDetectedZigPose();
                    printf("estimated_zig_pose: %0.4f %0.4f %0.4f %0.1f %0.1f %0.1f\n", zig_pose[0], zig_pose[1], zig_pose[2], zig_pose[3], zig_pose[4], zig_pose[5]);
                    std::vector<double> regrasping_pose = j_in[demo_name.c_str()]["x_detach_regrasping"].get<std::vector<double>>();
                    for(int i=0; i<3; i++) {
                        regrasping_pose[i + 3] = zig_pose[i + 3];
                    }
                    printf("MOTION_TAG_REGRASPING_POSE_ORI_DETECTED: %0.4f %0.4f %0.4f %0.1f %0.1f %0.1f\n", regrasping_pose[0], regrasping_pose[1], regrasping_pose[2], regrasping_pose[3], regrasping_pose[4], regrasping_pose[5]);
                    pose = regrasping_pose;
                    l_is_pose_assigned = true;
                } else {
                    ROS_LOG_INFO("Check target CS Zig pose! (setDemoRecogCSPose)");
                }
                break;
            }

            default: {
                assert(false);
                break;
            }
        }

        printf("******************************************************\n");
        printf("**************** Recognized CS Pose ******************\n");
        printf("Recognized CS Pose: %0.4f, %0.4f, %0.4f, %0.1f, %0.1f, %0.1f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
        printf("******************************************************\n");

        if(l_is_pose_assigned) {
            //// TODO: JSON 파일로 불러오기
            // 1) Grasping workspace limits
            ///////////////////////////////////////////////////////////////////
            Eigen::MatrixXd workspace(6,2); // pose limit - (min, max)
            switch(motion_tag) {
                case MotionTag::MOTION_TAG_PLANAR_VACUUM_Z_FIXED_DETECTED_GRASPING_POSE: { // Detected grasping pose
                    workspace_in = j_in[demo_name.c_str()]["workspace_grasping"].get<std::vector<double>>();
                    break;
                }
                case MotionTag::MOTION_TAG_DETECTED_GRASPING_POSE: { // Detected grasping pose
                    workspace_in = j_in[demo_name.c_str()]["workspace_grasping"].get<std::vector<double>>();
                    break;
                }
                case MotionTag::MOTION_TAG_PLANAR_VACUUM_Z_FIXED_BEFORE_DETECTED_GRASPING_POSE: { // Detected grasping pose
                    workspace_in = j_in[demo_name.c_str()]["workspace_grasping"].get<std::vector<double>>();
                    break;
                }
                case MotionTag::MOTION_TAG_BEFORE_DETECTED_GRASPING_POSE: { // Detected grasping pose
                    workspace_in = j_in[demo_name.c_str()]["workspace_grasping"].get<std::vector<double>>();
                    break;
                }
                case MotionTag::MOTION_TAG_DETECTED_ZIG_POSE: { // Detected zig pose
                    workspace_in = j_in[demo_name.c_str()]["workspace_zig"].get<std::vector<double>>();
                    break;
                }
                case MotionTag::MOTION_TAG_REGRASPING_POSE: { // Detected zig pose
                    workspace_in = j_in[demo_name.c_str()]["workspace_zig"].get<std::vector<double>>();
                    break;
                }
                case MotionTag::MOTION_TAG_REGRASPING_POSE_ORI_DETECTED: { // Detected zig pose
                    workspace_in = j_in[demo_name.c_str()]["workspace_zig"].get<std::vector<double>>();
                    break;
                }
                default: {
                    assert(false);
                    break;
                }
            }
            if(workspace_in.size() != 12) {
                ROS_LOG_ERROR("Check JSON file! - Workspace loading error! (setDemoRecogCSPose)");
                return false;
            }
            for(int i=0; i<6; i++) {
                workspace(i, 0) = workspace_in[2 * i]; // min
                workspace(i, 1) = workspace_in[2 * i + 1]; // max
            }
            printf("demo_name: %s\n", demo_name.c_str());
            printf("workspace[min]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f\n", workspace_in[0], workspace_in[2], workspace_in[4], workspace_in[6], workspace_in[8], workspace_in[10]);
            printf("workspace[max]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f\n", workspace_in[1], workspace_in[3], workspace_in[5], workspace_in[7], workspace_in[9], workspace_in[11]);
            ///////////////////////////////////////////////////////////////////

            // 2)
            if (checkWorkspace(pose, workspace)) {
                ROS_LOG_INFO("Recognized CS Pose OK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("Recognized CS Pose OK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("Recognized CS Pose OK !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
                output = pose;
                return true;
            } else {
                ROS_LOG_INFO("Recognized CS Pose is out of workspace !!!!!!!!!");
                ROS_LOG_INFO("Recognized CS Pose is out of workspace !!!!!!!!!");
                ROS_LOG_INFO("Recognized CS Pose is out of workspace !!!!!!!!!");
                return false; // read error
            }

        } else {
            ROS_LOG_ERROR("Recognized Grasping or Zig CS Pose is not assigned!");
            return false;
        }
    }
    catch(std::string &e)
    {
        ROS_LOG_ERROR("Check JSON file! - Recognized Grasping or Zig CS Pose!");
        return false;
    }
}

std::string BinPickingNode::getDemoName(uint16_t demo_tag)
{
    std::string demo_name;
    switch(demo_tag) { // JSON input
        case BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER: {
            demo_name = "demo_cnc_lathe";
            break;
        }
        case BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG: {
            demo_name = "demo_cnc_milling";
            break;
        }
        case BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH: {
            demo_name = "demo_cnc_bolt_bush";
            break;
        }
        case BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP: {
            demo_name = "demo_pick_and_place_welding_t_joint_2f_gripper";
            break;
        }
        case BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_VACUUM_GRP: {
            demo_name = "demo_pick_and_place_welding_t_joint_vacuum_gripper";
            break;
        }
        case BPDemoType::DEMO_BP_PICK_WELDING_ELBOW_JOINT: {
            demo_name = "demo_pick_and_place_welding_elbow_joint";
            break;
        }
        case BPDemoType::DEMO_BP_PICK_BOLT_BUSH: {
            demo_name = "demo_pick_and_place_bolt_bush";
            break;
        }
        case BPDemoType::DEMO_BP_PICK_CYLINDER: {
            demo_name = "demo_pick_and_place_cylinder";
            break;
        }
        case BPDemoType::DEMO_BP_PICK_SQUARE_PEG: {
            demo_name = "demo_pick_and_place_square_peg";
            break;
        }
        case BPDemoType::DEMO_BP_PICK_GEAR: {
            demo_name = "demo_pick_and_place_gear";
            break;
        }
        default: {
            assert(false);
            break;
        }
    }
    ROS_LOG_INFO("Demo name: %s", demo_name.c_str());
    return demo_name;
}

void BinPickingNode::getTaskInfo(bool is_task_mode) {
    printf("\n************************** Task Info. ******************************\n");
    if(is_task_mode) {
        ROS_LOG_INFO("Task in progress (now task # -)!");
    } else  {
        ROS_LOG_INFO("No task!");
    }
    ROS_LOG_INFO("Current bin-picking task info.: idx #%i, %s, sampling count: %i", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_scan_parameter.target_id, m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_scan_parameter.target_name.c_str(), m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_scan_parameter.sampling_num);

    printf("******************************************************************\n");
    ROS_LOG_INFO("Current gripper info.: gripper open length: %i [count]", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_open_length);
    ROS_LOG_INFO("Current gripper info.: gripper close length: %i [count]", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_close_length);
    // ROS_LOG_INFO("Current gripper info.: gripper current tip index: (-) #%u", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.current_gripper_tip_index);
    ROS_LOG_INFO("Current gripper info.: gripper detected tip index: (+) #%u", m_ptr_bin_picking_node_target_object_list[current_target_object_idx_]->m_grasping_parameter.gripper_tip_index);
    printf("******************************************************************\n");

    ROS_LOG_INFO("Current gripper(#1) info.: gripper measured count: %i [count]", grp_measured_count[0]);
    ROS_LOG_INFO("Current gripper(#1) info.: gripper measured angle: %i [deg]", grp_measured_position[0]);
    ROS_LOG_INFO("Current gripper(#1) info.: gripper initial_min angle: %i [deg]", grp_initial_min_position[0]);
    ROS_LOG_INFO("Current gripper(#1) info.: gripper initial_max angle: %i [deg]", grp_initial_max_position[0]);
    ROS_LOG_INFO("Current gripper(#2) info.: gripper measured count: %i [count]", grp_measured_count[1]);
    ROS_LOG_INFO("Current gripper(#2) info.: gripper measured angle: %i [deg]", grp_measured_position[1]);
    ROS_LOG_INFO("Current gripper(#2) info.: gripper initial_min angle: %i [deg]", grp_initial_min_position[1]);
    ROS_LOG_INFO("Current gripper(#2) info.: gripper initial_max angle: %i [deg]", grp_initial_max_position[1]);
    printf("******************************************************************\n");
    ROS_LOG_INFO("Detected pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", detected_pose_[0], detected_pose_[1], detected_pose_[2], detected_pose_[3], detected_pose_[4], detected_pose_[5]);
    ROS_LOG_INFO("Detected sub. pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", detected_sub_pose_[0], detected_sub_pose_[1], detected_sub_pose_[2], detected_sub_pose_[3], detected_sub_pose_[4], detected_sub_pose_[5]);
    ROS_LOG_INFO("Matching accuracy: %0.3f", matching_accuracy);
    ROS_LOG_INFO("Matching accuracy limit: %0.3f", matching_accuracy_limit);
    ROS_LOG_INFO("Detected mask count: %i", detected_mask_num);
    printf("******************************************************************\n");
    ROS_LOG_INFO("Detected zig pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", detected_zig_pose_[0], detected_zig_pose_[1], detected_zig_pose_[2], detected_zig_pose_[3], detected_zig_pose_[4], detected_zig_pose_[5]);
    printf("******************************************************************\n");
    ROS_LOG_INFO("Estimated pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", estimated_pose_[0], estimated_pose_[1], estimated_pose_[2], estimated_pose_[3], estimated_pose_[4], estimated_pose_[5]);
    ROS_LOG_INFO("Estimated sub. pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f", estimated_sub_pose_[0], estimated_sub_pose_[1], estimated_sub_pose_[2], estimated_sub_pose_[3], estimated_sub_pose_[4], estimated_sub_pose_[5]);
    printf("******************************************************************\n");
}


//////////////////////////////////////////////////////////////////////////////


// }  // namespace assem_control_ui





























