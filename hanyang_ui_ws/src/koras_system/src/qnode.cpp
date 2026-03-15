#include "qnode.hpp"
#include <chrono>
#include <array>
#include <map>
#include <termios.h>
#include <unistd.h>
//Cooking robot
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <QDebug>

// base64
#include <string>
#include <vector>
#include <stdexcept>

#include <bitset>
#include <rcutils/logging_macros.h> 

int task_index_ = 0;
constexpr const char* GREEN  = "\033[32m";
constexpr const char* RESET  = "\033[0m";

// #if DRFL_CONTROL
// Base64 디코딩을 위한 lookup 테이블
static const std::string base64_chars =
             "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
             "abcdefghijklmnopqrstuvwxyz"
             "0123456789+/";

// Base64 체크 함수
inline bool QNode::is_base64(unsigned char c) {
    return (isalnum(c) || (c == '+') || (c == '/'));
}

// Base64 디코딩 함수
std::string QNode::base64_decode(std::string& encoded_string) {
    // 필터링: '+'와 '/'을 문자 0으로, '='을 패딩 문자로 제거
    int in_len = encoded_string.size();
    int i = 0;
    int j = 0;
    int in_ = 0;
    std::vector<unsigned char> char_array_4(4), char_array_3(3);
    std::string ret;

    while (in_len-- && (encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
        char_array_4[i++] = encoded_string[in_]; in_++;
        if (i == 4) {
            for (i = 0; i < 4; i++) {
                char_array_4[i] = base64_chars.find(char_array_4[i]);
            }
            char_array_3[0] = (char_array_4[0] << 2) | (char_array_4[1] >> 4);
            char_array_3[1] = ((char_array_4[1] & 15) << 4) | (char_array_4[2] >> 2);
            char_array_3[2] = ((char_array_4[2] & 3) << 6) | char_array_4[3];
            for (i = 0; (i < 3); i++) {
                ret += char_array_3[i];
            }
            i = 0;
        }
    }

    if (i) {
        for (j = i; j < 4; j++) {
            char_array_4[j] = 0;
        }

        for (j = 0; j < 4; j++) {
            char_array_4[j] = base64_chars.find(char_array_4[j]);
        }
        char_array_3[0] = (char_array_4[0] << 2) | (char_array_4[1] >> 4);
        char_array_3[1] = ((char_array_4[1] & 15) << 4) | (char_array_4[2] >> 2);
        char_array_3[2] = ((char_array_4[2] & 3) << 6) | char_array_4[3];

        for (j = 0; (j < i - 1); j++) {
            ret += char_array_3[j];
        }
    }

    return ret;
}

constexpr uint16_t PLC_WORD_SAFE_STOP  = 0x41003;   // Word 41003
constexpr uint16_t PLC_BIT_SAFE_STOP   = 0;         // Bit 0
constexpr bool     PLC_BIT_ON          = true;
constexpr bool     PLC_BIT_OFF         = false;


string vec2string(vector<uint16_t>& data) {
    std::stringstream hexStream;
    for (uint16_t d : data) {
        hexStream << "\\x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << d;
    }
    string result = hexStream.str();
    return result;
}

vector<uint16_t> dec2hex_vector(uint16_t value, bool is_reversed=false) {	// 10진수를 16진수로 변환
	std::stringstream hexStream;
    vector<uint16_t> result;
    std::string hex;
    uint16_t hex_int;

    if (is_reversed) {
        result.push_back(static_cast<uint16_t>(value & 0xFF));  // 저위 바이트 (0xF4)
        result.push_back(static_cast<uint16_t>(value >> 8));  // 고위 바이트 (0x01)

    } else {
        result.push_back(static_cast<uint16_t>(value >> 8));  // 고위 바이트 (0x01)
        result.push_back(static_cast<uint16_t>(value & 0xFF));  // 저위 바이트 (0xF4)
    }

    return result;
}

std::string dec2hex(uint16_t value, bool is_reversed=false) {	// 10진수를 16진수로 변환
	std::stringstream hexStream;

    if (is_reversed) {
        hexStream << "\\x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (value & 0xFF)  // 하위 바이트
                << "\\x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (value >> 8);  // 상위 바이트
    } else {
    // 각 바이트를 \x로 형식화하여 출력
        hexStream << "\\x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (value >> 8)  // 상위 바이트
                << "\\x" << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << (value & 0xFF);  // 하위 바이트
    }

    // 결과를 hex 변수에 저장
    std::string hex = hexStream.str();
    return hex;
}

uint16_t calculateCRC(const vector<uint16_t>& data) {
    vector<uint16_t> crc_table = {0x0000, 0xC0C1, 0xC181, 0x0140,   0xC301, 0x03C0, 0x0280, 0xC241,
                                    0xC601, 0x06C0, 0x0780, 0xC741,   0x0500, 0xC5C1, 0xC481, 0x0440,
                                    0xCC01, 0x0CC0, 0x0D80, 0xCD41,   0x0F00, 0xCFC1, 0xCE81, 0x0E40,
                                    0x0A00, 0xCAC1, 0xCB81, 0x0B40,   0xC901, 0x09C0, 0x0880, 0xC841,
                                    0xD801, 0x18C0, 0x1980, 0xD941,   0x1B00, 0xDBC1, 0xDA81, 0x1A40,
                                    0x1E00, 0xDEC1, 0xDF81, 0x1F40,   0xDD01, 0x1DC0, 0x1C80, 0xDC41,
                                    0x1400, 0xD4C1, 0xD581, 0x1540,   0xD701, 0x17C0, 0x1680, 0xD641,
                                    0xD201, 0x12C0, 0x1380, 0xD341,   0x1100, 0xD1C1, 0xD081, 0x1040,
                                    0xF001, 0x30C0, 0x3180, 0xF141,   0x3300, 0xF3C1, 0xF281, 0x3240,
                                    0x3600, 0xF6C1, 0xF781, 0x3740,   0xF501, 0x35C0, 0x3480, 0xF441,
                                    0x3C00, 0xFCC1, 0xFD81, 0x3D40,   0xFF01, 0x3FC0, 0x3E80, 0xFE41,
                                    0xFA01, 0x3AC0, 0x3B80, 0xFB41,   0x3900, 0xF9C1, 0xF881, 0x3840,
                                    0x2800, 0xE8C1, 0xE981, 0x2940,   0xEB01, 0x2BC0, 0x2A80, 0xEA41,
                                    0xEE01, 0x2EC0, 0x2F80, 0xEF41,   0x2D00, 0xEDC1, 0xEC81, 0x2C40,
                                    0xE401, 0x24C0, 0x2580, 0xE541,   0x2700, 0xE7C1, 0xE681, 0x2640,
                                    0x2200, 0xE2C1, 0xE381, 0x2340,   0xE101, 0x21C0, 0x2080, 0xE041,
                                    0xA001, 0x60C0, 0x6180, 0xA141,   0x6300, 0xA3C1, 0xA281, 0x6240,
                                    0x6600, 0xA6C1, 0xA781, 0x6740,   0xA501, 0x65C0, 0x6480, 0xA441,
                                    0x6C00, 0xACC1, 0xAD81, 0x6D40,   0xAF01, 0x6FC0, 0x6E80, 0xAE41,
                                    0xAA01, 0x6AC0, 0x6B80, 0xAB41,   0x6900, 0xA9C1, 0xA881, 0x6840,
                                    0x7800, 0xB8C1, 0xB981, 0x7940,   0xBB01, 0x7BC0, 0x7A80, 0xBA41,
                                    0xBE01, 0x7EC0, 0x7F80, 0xBF41,   0x7D00, 0xBDC1, 0xBC81, 0x7C40,
                                    0xB401, 0x74C0, 0x7580, 0xB541,   0x7700, 0xB7C1, 0xB681, 0x7640,
                                    0x7200, 0xB2C1, 0xB381, 0x7340,   0xB101, 0x71C0, 0x7080, 0xB041,
                                    0x5000, 0x90C1, 0x9181, 0x5140,   0x9301, 0x53C0, 0x5280, 0x9241,
                                    0x9601, 0x56C0, 0x5780, 0x9741,   0x5500, 0x95C1, 0x9481, 0x5440,
                                    0x9C01, 0x5CC0, 0x5D80, 0x9D41,   0x5F00, 0x9FC1, 0x9E81, 0x5E40,
                                    0x5A00, 0x9AC1, 0x9B81, 0x5B40,   0x9901, 0x59C0, 0x5880, 0x9841,
                                    0x8801, 0x48C0, 0x4980, 0x8941,   0x4B00, 0x8BC1, 0x8A81, 0x4A40,
                                    0x4E00, 0x8EC1, 0x8F81, 0x4F40,   0x8D01, 0x4DC0, 0x4C80, 0x8C41,
                                    0x4400, 0x84C1, 0x8581, 0x4540,   0x8701, 0x47C0, 0x4680, 0x8641,
                                    0x8201, 0x42C0, 0x4380, 0x8341,   0x4100, 0x81C1, 0x8081, 0x4040};
    uint16_t crc = 0xFFFF;  // 초기 값 설정

    // 각 바이트에 대해 CRC 계산
    for (unsigned char byte : data) {
        crc = (crc >> 8) ^ crc_table[(crc ^ byte) & 0xFF];
    }

    return crc;
}

vector<double> jsToCsPose(const vector<double>& js_pose) {
    vector<double> cs_pose(6); // cs pose [x, y, z, roll, pitch, yaw]

    // // 예시: 간단한 forward kinematics로 위치와 자세 계산
    // // js_pose는 [theta1, theta2, theta3, theta4, theta5, theta6] (각도 단위: 라디안)
    // // cs_pose는 [x, y, z, roll, pitch, yaw] (위치는 mm 또는 cm, 자세는 라디안)

    // // 간단한 예시 변환: 직선 변환 예시 (각각의 관절 길이와 관계를 고려)
    // double L1 = 100.0;  // 링크 길이 1 (예시)
    // double L2 = 100.0;  // 링크 길이 2 (예시)

    // // 위치 계산 (단순 예시: x, y, z)
    // cs_pose[0] = L1 * cos(js_pose[0]) + L2 * cos(js_pose[0] + js_pose[1]); // x
    // cs_pose[1] = L1 * sin(js_pose[0]) + L2 * sin(js_pose[0] + js_pose[1]); // y
    // cs_pose[2] = L2 * sin(js_pose[2]);  // z (단순화된 예시, 실제로는 더 복잡함)

    // // 자세 계산 (회전 행렬을 통해 roll, pitch, yaw를 계산할 수 있음)
    // cs_pose[3] = js_pose[3];  // roll
    // cs_pose[4] = js_pose[4];  // pitch
    // cs_pose[5] = js_pose[5];  // yaw

    return cs_pose;
}

// #endif

struct PlanParam
{
	float time;

	float ps[6];
	float vs[6];
	float as[6];
	float pf[6];
	float vf[6];
	float af[6];

	float A0[6];
	float A1[6];
	float A2[6];
	float A3[6];
	float A4[6];
	float A5[6];
};

struct TraParam
{
	float time;

	float pos[6];
	float vel[6];
	float acc[6];
};

void TrajectoryPlan(PlanParam* plan)
{
    float ps[6],vs[6],as[6];
    float pf[6],vf[6],af[6];
    float tf;

	tf = plan->time;

    for(int i=0; i<6; i++)
    {
        ps[i] = plan->ps[i];
        vs[i] = plan->vs[i];
        as[i] = plan->as[i];
        pf[i] = plan->pf[i];
        vf[i] = plan->vf[i];
        af[i] = plan->af[i];
    }

    for(int i=0; i<6; i++)
    {
        plan->A0[i] = ps[i];
        plan->A1[i] = vs[i];
        plan->A2[i] = as[i]/2;
        plan->A3[i] = (20*pf[i]-20*ps[i]-(8*vf[i]+12*vs[i])*tf-(3*as[i]-af[i])*tf*tf)/(2*tf*tf*tf);
        plan->A4[i] = (30*ps[i]-30*pf[i]+(14*vf[i]+16*vs[i])*tf+(3*as[i]-2*af[i])*tf*tf)/(2*tf*tf*tf*tf);
        plan->A5[i] = (12*pf[i]-12*ps[i]-(6*vf[i]+6*vs[i])*tf-(as[i]-af[i])*tf*tf)/(2*tf*tf*tf*tf*tf);
    }
}

void TrajectoryGenerator(PlanParam *plan, TraParam *tra)
{
    double A0[6],A1[6],A2[6],A3[6],A4[6],A5[6];
	double t = tra->time;

    for(int i=0; i<6; i++)
    {
        A0[i] = plan->A0[i];
        A1[i] = plan->A1[i];
        A2[i] = plan->A2[i];
        A3[i] = plan->A3[i];
        A4[i] = plan->A4[i];
        A5[i] = plan->A5[i];
    }

    for(int i=0; i<6; i++)
    {
        tra->pos[i] = A0[i] + A1[i]*t + A2[i]*t*t + A3[i]*t*t*t + A4[i]*t*t*t*t + A5[i]*t*t*t*t*t;
        tra->vel[i] = A1[i] + 2*A2[i]*t + 3*A3[i]*t*t + 4*A4[i]*t*t*t + 5*A5[i]*t*t*t*t;
        tra->acc[i] = 2*A2[i] + 6*A3[i]*t + 12*A4[i]*t*t + 20*A5[i]*t*t*t;
    }
}

namespace py = pybind11;

#define SEND_INFO_TO_MONITOR(message) \
    SEND_LOG_MESSAGE_TO_MONITOR(LogLevel::INFO, message);
#define SEND_WARN_TO_MONITOR(message) \
    SEND_LOG_MESSAGE_TO_MONITOR(LogLevel::WARN, message);
#define SEND_ERROR_TO_MONITOR(message) \
    SEND_LOG_MESSAGE_TO_MONITOR(LogLevel::ERROR, message);
#define SEND_LOG_MESSAGE_TO_MONITOR(value1, value2) \
    LogInfo logInfo;\
    logInfo._log_level = value1;\
    logInfo._log_message = value2;\
    Q_EMIT sendLogMessage(logInfo);

#if DRFL_CONTROL
// bool QNode::try_connect() {
//     // 5초마다 연결을 시도
//     const std::string ip_address = "192.168.137.100";

//     while (!is_drfl_connected) {

//         if (drfl_.open_connection(ip_address)) {
//             is_drfl_connected = true;
//             SYSTEM_VERSION tSysVerion = {'\0', };
//             drfl_.get_system_version(&tSysVerion);
//             std::cout << "System version: " << tSysVerion._szController << std::endl;

//         } else {
//             std::cout << "Failed to connect. Retrying in 5 seconds..." << std::endl;
//             std::this_thread::sleep_for(std::chrono::seconds(5));  // 5초 대기 후 다시 시도
//         }
//     }

//     return is_drfl_connected;
// }


double computeVectorNorm(const std::vector<double>& v1, const std::vector<double>& v2) {
    if (v1.size() != v2.size()) {
        throw std::invalid_argument("두 벡터의 크기가 동일해야 합니다.");
    }

    double sum = 0.0;
    for (size_t i = 0; i < v1.size(); ++i) {
        double diff = v1[i] - v2[i];
        sum += diff * diff;
    }

    return std::sqrt(sum);
}

array<double, 6> QNode::getRobotStateCallback(int space_type) {
    auto req = make_shared<dsr_msgs2::srv::GetCurrentPose::Request>();
    req->space_type = space_type;   //js: 0, cs: 1

    auto result = client_robot_state_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(10));
    auto recv = result.get();

    return recv->pos;
}

void QNode::drlfPathFinished() {
    if (params_.status.is_path_finished == false
            && (task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE ||
            task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE_500MS_DELAY ||
            task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE_WITH_SCAN_AND_MATCHING ||
            task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE_LLM ||
            task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_APPROACHING ||
            task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_CONVERGING)) {

        params_.status.is_path_finished = true;
        do_check_js_err_ = false;
        is_robot_moving_begin_ = false;
        cnt_for_is_js_err_ok_ = 0;
        cnt_for_is_js_err_exceeds_ = 0;
        std::cout << "[drlfPathFinished] getOperationgStatus" << std::endl;
        std::cout << "[drlfPathFinished] params_.status.is_path_finished" << params_.status.is_path_finished << std::endl;
    }
    // ROS_LOG_INFO("Path finished");
}

void QNode::drflRobotStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    params_.meas.q[0] = msg->position[0] * (180/M_PI);
    params_.meas.q[1] = msg->position[1] * (180/M_PI);
    params_.meas.q[2] = msg->position[4] * (180/M_PI);
    params_.meas.q[3] = msg->position[2] * (180/M_PI);
    params_.meas.q[4] = msg->position[3] * (180/M_PI);
    params_.meas.q[5] = msg->position[5] * (180/M_PI);


    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    //// 두산 로봇 모션 동작 여부 체크, 250223
    //// 기존 getOperatingStatus의 역할
    arr2Vec(params_.meas.q, js_position_now_);
    if(js_position_before_.size() != 0 && do_check_js_err_) {
        try {
            double norm = computeVectorNorm(js_position_now_, js_position_before_);
            // ROS_LOG_WARN("JS position error: %0.9f, [cnt_in: %zu, cnt_exceed: %zu]", norm, cnt_for_is_js_err_ok_, cnt_for_is_js_err_exceeds_);
            if(norm > 1e-3) { // 0.001[deg]
                if(!is_robot_moving_begin_) {
                    is_robot_moving_begin_ = true;
                }
                // ROS_LOG_WARN("[Exceed!]");
                cnt_for_is_js_err_exceeds_++;
            } else {
                cnt_for_is_js_err_ok_++;
                //// 로봇이 이동하다가 멈추는 경우에만
                if(is_robot_moving_begin_) {
                    // if(cnt_for_is_js_err_exceeds_ > 5 || cnt_for_is_js_err_ok_ > 20) {
                        drlfPathFinished();
                    // }
                } else {
                    if(cnt_for_is_js_err_ok_ > 8) { // Target이 가까운 경우
                        drlfPathFinished();
                    }
                }

            }
        } catch (const std::exception& e) {
            std::cerr << "에러 발생: " << e.what() << std::endl;
        }
    }
    arr2Vec(params_.meas.q, js_position_before_);
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////



    // static int count = 0;
    // count++;

    // // 30번째 메시지마다 params_.meas.x 배열 업데이트
    // if (count >= 30) {
    //     for (int i = 0; i < CS_DOF; ++i) {
    //         params_.meas.x[i] = params_.meas.q[0];
    //     }
    //     count = 0; // 카운터 초기화
    // }
}
#endif

void convertAndAssign(std::array<double, JS_DOF>& source, float destination[6]) {
    for (size_t i = 0; i < JS_DOF; ++i) {
        destination[i] = static_cast<float>(source[i]); // double을 float으로 변환
    }
}

QNode* QNode::qnodePtr = nullptr;
std::mutex QNode::mutex_;

QNode::QNode(int argc, char** argv, const std::string& argument)
: init_argc(argc), init_argv(argv) {

    std::cout << "QNode initialized with argument: " << argument << std::endl;
    // TaskPlanner 인스턴스 생성
    if (SW_MODE_HD_LLM) {
        m_bin_picking_node = new BinPickingNode(argc, argv); // scanning and matching
        task_planner_ = std::make_unique<TaskPlannerHyundaiLlm>();
    } else if (SW_MODE_COOKING) {
        // m_py_ui_bridge_node = new PyUiBridgeNode(argc, argv); // COOKING PYTHON UI
        task_planner_ = std::make_unique<TaskPlannerCooking>();
    } else if (SW_MODE_HANYANG_ENG) {
        m_bin_picking_node = new BinPickingNode(argc, argv); // scanning and matching
        task_planner_ = std::make_unique<TaskPlannerHanyangEng>();
    } else {
        m_bin_picking_node = new BinPickingNode(argc, argv); // scanning and matching
        task_planner_ = std::make_unique<TaskPlanner>();  // 기본 TaskPlanner
    }


    robot_dh_vec_.resize(24);
    robot_tcp_default_.resize(6);
    robot_tcp_.resize(6);

    is_grp_initialized.resize(2);
    for(int i=0; i<2; i++) {
        is_grp_initialized[i] = false;
    }

#if CAM_TYPE_REALSENSE_D405
    ROS_LOG_WARN("CAM_TYPE_REALSENSE_D405");
    std::vector<double> pose_ee_camera = {0.01, -0.086011, 0.075173, 45, 0, 0}; // Realsens D405, 최신 브라켓 (deg로 표기)
    // std::vector<double> pose_ee_camera = {0.0088, -0.080427, -0.010657, 45, 0, 0}; // 예전 버전 브라켓 (deg로 표기)
#endif

#if CAM_TYPE_LOGITEC_BRIO
    ROS_LOG_WARN("CAM_TYPE_LOGITEC_BRIO");
    // std::vector<double> pose_ee_camera = {0.0, -0.115356, 0.045728, 45, 0, 0}; // Logitec Brio, 기존 브라켓 (deg로 표기)
    // std::vector<double> pose_ee_camera = {0.0, -0.115356, 0.045728, -45, 0, 180.0}; // Logitec Brio, 180도 뒤집힌 최신 브라켓 (deg로 표기)

    //// 변경 브라켓
    std::vector<double> pose_ee_camera = {0.0, -0.06065, 0.047400, -45, 0, 180.0}; // Logitec Brio, 180도 뒤집힌 최신 브라켓 (deg로 표기)
#endif

    pose_ee_camera_ = ModelMatrix(6, 1, pose_ee_camera);
    tr_ee_camera_ = ModelMatrix::pose2tr(pose_ee_camera_);
    tr_camera_ee_ = ModelMatrix::invTrMat(tr_ee_camera_);

    flag_tag_recog_.store(false);
    tag_info_vec_temp_.clear();
    tag_info_vec_     .clear();


}

QNode::~QNode() {
    rclcpp::shutdown(); // explicitly needed since we use ros::start();
    wait();
}

bool QNode::init() {

    string prefix = ROBOT_NAME;
    prefix += "_";

    rclcpp::init(init_argc, init_argv);
    node_ = rclcpp::Node::make_shared(prefix + "hanyang_eng_koras_system");

    if(SW_MODE_COOKING) {
        ROS_LOG_WARN("POINT 1");
        ROS_LOG_WARN("POINT 1");
        // Cooking robot
        // 퍼블리셔 생성
        realsense_publisher_ = node_->create_publisher<std_msgs::msg::String>("get_realsense_data", 10);

        if (!realsense_publisher_) {
            qDebug() << "Failed to initialize realsense_publisher_!";
        } else {
            qDebug() << "Realsense publisher initialized successfully.";
        }

        // 구독자 생성
        pose_subscription_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            "realsense/tool_pose",
            10,
            std::bind(&QNode::CookGoalPoseCallback, this, std::placeholders::_1));

        image_subscription_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "realsense/detection_image",
            10,
            std::bind(&QNode::CookImageCallback, this, std::placeholders::_1));

        // MetaType 등록
        qRegisterMetaType<std_msgs::msg::Float64MultiArray::SharedPtr>("std_msgs::msg::Float64MultiArray::SharedPtr");
        qRegisterMetaType<sensor_msgs::msg::Image::SharedPtr>("sensor_msgs::msg::Image::SharedPtr");


        // LangSAM 텍스트 프롬프트 발행
        langsam_request_publisher_ = node_->create_publisher<std_msgs::msg::String>(
            "segmentation/request", 10);

        // 중심 좌표 수신 Subscriber
        langsam_coord_subscriber_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            "segmentation/center_coordinates", 10,
            std::bind(&QNode::LangSAMCoordCallback, this, std::placeholders::_1));

        // 분할된 이미지 수신 Subscriber
        langsam_image_subscriber_ = node_->create_subscription<sensor_msgs::msg::Image>(
            "segmentation/segmented_image", 10,
            std::bind(&QNode::LangSAMImageCallback, this, std::placeholders::_1));
        qDebug() << "LangSAM publishers and subscribers initialized.";

        ROS_LOG_WARN("POINT 2");
        ROS_LOG_WARN("POINT 2");
    }

#if DRFL_CONTROL
    string _robot_id    = "dsr01";
    string _srv_name_prefix   = _robot_id + "/"; //ROS2

    // ROS service client
    client_robot_state_       = node_->create_client<dsr_msgs2::srv::GetCurrentPose>(_srv_name_prefix + "system/get_current_pose");
    subscription_robot_state_ = node_->create_subscription<sensor_msgs::msg::JointState>(_srv_name_prefix + "joint_states",
                                100, bind(&QNode::drflRobotStateCallback, this, placeholders::_1));

    client_move_q_            = node_->create_client<dsr_msgs2::srv::MoveJoint>(_srv_name_prefix + "motion/move_joint");
    client_move_x_            = node_->create_client<dsr_msgs2::srv::MoveLine>(_srv_name_prefix + "motion/move_line");
    client_move_spline_q_     = node_->create_client<dsr_msgs2::srv::MoveSplineJoint>(_srv_name_prefix + "motion/move_spline_joint");
    client_move_spline_x_     = node_->create_client<dsr_msgs2::srv::MoveSplineTask>(_srv_name_prefix + "motion/move_spline_task");

    client_jog_               = node_->create_client<dsr_msgs2::srv::Jog>(_srv_name_prefix + "motion/jog");
    client_drl_start_               = node_->create_client<dsr_msgs2::srv::DrlStart>(_srv_name_prefix + "drl/drl_start");

    client_set_robot_mode_    = node_->create_client<dsr_msgs2::srv::SetRobotMode>(_srv_name_prefix + "system/set_robot_mode");
    client_set_tcp_           = node_->create_client<dsr_msgs2::srv::SetCurrentTcp>(_srv_name_prefix + "tcp/set_current_tcp");
    client_create_tcp_        = node_->create_client<dsr_msgs2::srv::ConfigCreateTcp>(_srv_name_prefix + "tcp/config_create_tcp");

    //0213수정
    client_stop_robot        = node_->create_client<dsr_msgs2::srv::MoveStop   >(_srv_name_prefix + "motion/move_stop");
    client_servo_off_         = node_->create_client<dsr_msgs2::srv::ServoOff>(_srv_name_prefix + "system/servo_off");
    client_set_robot_control_ = node_->create_client<dsr_msgs2::srv::SetRobotControl>(_srv_name_prefix + "system/set_robot_control");

    /* 구독자 */
    dsr_state_sub_ =
        node_->create_subscription<dsr_msgs2::msg::RobotState>(
            "/dsr01/robot_state",          // 네임스페이스 확인
            10,
            std::bind(&QNode::dsrRobotStateCb, this, std::placeholders::_1));

    /* 1 초 타이머 */
    dsr_state_timer_ =
        node_->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&QNode::dsrStatePrintTimerCb, this));


    client_get_robot_state_ =
        node_->create_client<dsr_msgs2::srv::GetRobotState>(
            "/dsr01/system/get_robot_state");   // ← 실제 네임스페이스 확인

    /* 5초 주기 타이머 */
    timer_get_robot_state_ =
        node_->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&QNode::requestRobotState, this));

    client_compliance_ctrl = node_->create_client<dsr_msgs2::srv::TaskComplianceCtrl>("task_compliance_ctrl");
    client_release_compliance_ctrl = node_->create_client<dsr_msgs2::srv::ReleaseComplianceCtrl>("/dsr01/force/release_compliance_ctrl");

    client_set_current_tool_ = node_->create_client<dsr_msgs2::srv::SetCurrentTool>(_srv_name_prefix + "tool/set_current_tool");
    client_config_create_tool_ = node_->create_client<dsr_msgs2::srv::ConfigCreateTool>(_srv_name_prefix + "tool/config_create_tool");

    client_change_collision_sensitivity_ = node_->create_client<dsr_msgs2::srv::ChangeCollisionSensitivity>(_srv_name_prefix + "system/change_collision_sensitivity");

    client_get_dsr_torque = node_->create_client<dsr_msgs2::srv::GetJointTorque>(_srv_name_prefix + "aux_control/get_joint_torque");

#else
    // ROS subscription
    subscription_robot_state_ = node_->create_subscription<kcr_control_msg::msg::RobotState>(prefix + "robot_state",
                                100, bind(&QNode::robotStateCallback, this, placeholders::_1));
    subscription_rsc_state_   = node_->create_subscription<kcr_control_msg::msg::RscState>(prefix + "rsc_state",
                                100, bind(&QNode::rscStateCallback, this, placeholders::_1));

    // ROS service client
    client_power_on_off_            = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "power_on_off");
    client_set_enable_              = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "set_enable");

    client_jamming_state_enable_    = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "jamming_state_enable");
    client_clear_error_             = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "clear_error");
    client_set_hand_guide_          = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "set_hand_guide");
    client_set_friction_observer_   = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "set_friction_observer");
    client_set_collision_detection_ = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "set_collision_detection");
    client_set_collision_demo_mode_ = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "set_collision_demo_mode");
    client_set_inv_kine_method_     = node_->create_client<kcr_control_msg::srv::SingleString>(prefix + "set_inv_kine_method");
    client_set_driver_ctrl_mode_    = node_->create_client<kcr_control_msg::srv::SingleString>(prefix + "set_driver_ctrl_mode");

    client_update_param_   = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "update_param");
    client_rollback_param_ = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "rollback_param");

    client_record_start_   = node_->create_client<kcr_control_msg::srv::SingleString>(prefix + "record_start");
    client_record_end_     = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "record_end");

    client_move_q_ = node_->create_client<kcr_control_msg::srv::MoveQ>(prefix + "move_q");
    //client_move_sq_ = node_->create_client<kcr_control_msg::srv::MoveSplineQ>(prefix + "move_sq");
    client_move_x_ = node_->create_client<kcr_control_msg::srv::MoveX>(prefix + "move_x");
    //client_move_sx_ = node_->create_client<kcr_control_msg::srv::MoveSplineX>(prefix + "move_sx");

    client_jog_ = node_->create_client<kcr_control_msg::srv::Jog>(prefix + "jog");

    client_dtc_ = node_->create_client<kcr_control_msg::srv::DirectTorqueControl>(prefix + "dtc");

    client_blending_ = node_->create_client<kcr_control_msg::srv::Blending>(prefix + "blending");

    client_stop_robot_      = node_->create_client<kcr_control_msg::srv::StopRobot   >(prefix + "stop_robot");
    client_set_command_     = node_->create_client<kcr_control_msg::srv::SingleString>(prefix + "set_command");
    client_set_impedance_   = node_->create_client<kcr_control_msg::srv::Impedance   >(prefix + "set_impedance");
    client_set_tcp_payload_ = node_->create_client<kcr_control_msg::srv::TcpPayload  >(prefix + "set_tcp_payload");
    client_save_force_data_ = node_->create_client<kcr_control_msg::srv::SingleString>(prefix + "save_force_data");

    client_led_           = node_->create_client<kcr_control_msg::srv::Command>(prefix + "led");
    client_grp_           = node_->create_client<kcr_control_msg::srv::Command>(prefix + "grp");
    client_grp_ctrl_word_ = node_->create_client<kcr_control_msg::srv::Command>(prefix + "grp_ctrl_word");

    server_operating_status_ = node_->create_service<kcr_control_msg::srv::OperatingStatus>(prefix + "operating_status",
                               bind(&QNode::getOperatingStatus, this, placeholders::_1, placeholders::_2));



    // Joy node related
    subscriber_joy_ = node_->create_subscription<sensor_msgs::msg::Joy>("joy",
                      100, bind(&QNode::getJoyData, this, placeholders::_1));
    client_xpad_jog_ = node_->create_client<kcr_control_msg::srv::XpadJog>(prefix + "xpad_jog");

    joy_button_.resize(11);
    joy_stick_axis_.resize(8);

    rsc_params_.is_joint_connected = false;

    // Apriltag related
    subscriber_tag_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("tf",
                      100, bind(&QNode::tagRecogCallback, this, placeholders::_1));
    publisher_tag_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tag_filtered", 10);

    //// Bin picking
    // client_sensor_command_              = node_->create_client<hanyang_matching_msgs::srv::SetCommand>("/sensor_command");

    //ZIVID_Scanning
    // m_robot_id = UR10e; // UR10e

    // zividScanningClient_= node_->create_client<hanyang_matching_msgs::srv::ZividDoScan>("/zivid_scanning");
    // doTemplateMatchingBinPickingClient_= node_->create_client<hanyang_matching_msgs::srv::DoTemplateMatching>("/do_template_matching_bin_picking");


    // Robot Calibration
    cal_scan_pub = node_->create_publisher<std_msgs::msg::String>("cal_node/scan", 10);
    cal_scan_sub = node_->create_subscription<std_msgs::msg::Bool>("cal_node/save",
                     10, bind(&QNode::CalSaveCallback, this, placeholders::_1));

	cal_check_sub = node_->create_subscription<std_msgs::msg::Float64MultiArray>("cal_node/check",
                    10, bind(&QNode::CalCheckResultCallback, this, placeholders::_1));
#endif


#if NEW_VER_KORAS_GRIPPER_PACKAGE

    //// New Version
    new_client_gripper_cmd_ = node_->create_client<grp_control_msg::srv::SingleInt>("/set_finger_pos");
    new_client_gripper_motor_pos_ctrl_ = node_->create_client<grp_control_msg::srv::PosVelCurCtrl>("/motor_pos_ctrl");
    new_client_gripper_motor_enable_ = node_->create_client<grp_control_msg::srv::Void>("/motor_enable");
    new_client_gripper_motor_reset_pose_ = node_->create_client<grp_control_msg::srv::Void>("/motor_reset_pose");

    new_client_gripper_cmd_initialize_ = node_->create_client<grp_control_msg::srv::Void>("/gripper_initialize");
    new_client_gripper_cmd_open_ = node_->create_client<grp_control_msg::srv::Void>("/grp_open");
    new_client_gripper_cmd_close_ = node_->create_client<grp_control_msg::srv::Void>("/grp_close");
    new_client_gripper_cmd_vac_on_ = node_->create_client<grp_control_msg::srv::Void>("/vacuum_grp_on");
    new_client_gripper_cmd_vac_off_ = node_->create_client<grp_control_msg::srv::Void>("/vacuum_grp_off");

#endif
    // Gripper related
    client_gripper_cmd_ = node_->create_client<grp_control_msg::srv::GripperCommand>("gripper_command_control");
    client_stop_motor_ = node_->create_client<grp_control_msg::srv::StopMotor>("gripper_stop");
    client_grp_enable_ = node_->create_client<grp_control_msg::srv::DriverEnable>("driver_enable");
    subscriber_gripper_state_ = node_->create_subscription<grp_control_msg::msg::GripperMsg>("grp_state",
                    100, bind(&QNode::gripperStateCallback, this, placeholders::_1));

bp_robot_state_pub_ = node_->create_publisher<hanyang_matching_msgs::msg::RobotState>("/bp_robot_state", 20);


isScanningResultSubscriber_ = node_->create_subscription<hanyang_matching_msgs::msg::MatchingResultMsg>("/scanning_result",
							1, bind(&QNode::ScanningResultCallback, this, placeholders::_1));


matchingPoseResultSubscriber_ = node_->create_subscription<hanyang_matching_msgs::msg::MatchingResultMsg>("/cad_matching_result",
							1, bind(&QNode::templateMatchingCallback, this, placeholders::_1));


// Apriltag related
subscriber_tag_ = node_->create_subscription<tf2_msgs::msg::TFMessage>("tf",
				  100, bind(&QNode::tagRecogCallback, this, placeholders::_1));
publisher_tag_ = node_->create_publisher<tf2_msgs::msg::TFMessage>("/tag_filtered", 10);



if(SW_MODE_GRAPHY) {
    client_set_AutomaticMode_       = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "set_Automatic_Mode");
    client_set_ManualMode_          = node_->create_client<kcr_control_msg::srv::SingleBool>(prefix + "set_Manual_Mode");
#if CAM_TYPE_REALSENSE_D405
    imageSubscriber_ = node_->create_subscription<sensor_msgs::msg::Image>("/realsense/d405/color/image_rect_raw",
                                1, bind(&QNode::imageCallback, this, placeholders::_1));
#endif

// sb 25.03.05
#if CAM_TYPE_LOGITEC_BRIO
    // sb 25.03.06
    camera_infoSubscriber_ = node_->create_subscription<sensor_msgs::msg::CameraInfo>("/camera_info",
                                1, bind(&QNode::cameraInfoCallback, this, placeholders::_1));
    camera_infoPublisher_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>("/camera_info_sb", 10);

    imageSubscriber_ = node_->create_subscription<sensor_msgs::msg::Image>("/image_raw",
                                1, bind(&QNode::imageCallback, this, placeholders::_1));
    imagePublisher_ = node_->create_publisher<sensor_msgs::msg::Image>("/image_sb",10);

#endif
        // is_camera_mode_on_ = j_cam_setting_in["Logitec_Brio"]["visualize_camera"];
        is_camera_mode_on_ = false;
        if(is_camera_mode_on_) {
            ROS_LOG_WARN("CAMERA ON");
        } else {
            ROS_LOG_WARN("CAMERA OFF");
        }
} else {
    // imageSubscriber_ = node_->create_subscription<sensor_msgs::msg::Image>("/zivid/color/image_color",
    //                             1, bind(&QNode::imageCallback, this, placeholders::_1));
    imageSubscriber_ = node_->create_subscription<sensor_msgs::msg::Image>("/sam_zivid/visualization",
                                1, bind(&QNode::scannerImageCallback, this, placeholders::_1));

    // Keycode angle subscriber (geometry_msgs/Vector3)
    keycode_angle_subscriber_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/hanyang/coupler/keycode_angle", 10,
        std::bind(&QNode::KeycodeAngleCallback, this, std::placeholders::_1));

    // Keycode holder angle subscriber (geometry_msgs/Vector3)
    keycode_holder_angle_subscriber_ = node_->create_subscription<geometry_msgs::msg::Vector3>(
        "/hanyang/coupler/keycode_holder_angle", 10,
        std::bind(&QNode::KeycodeHolderAngleCallback, this, std::placeholders::_1));



}




    // command_ = COMMAND_DEFAULTS;
    // status_ = STATUS_DEFAULTS;
    // enable_state_ = false;
    // operation_state_ = false;
    // collision_state_ = false;
    // malfunction_state_ = false;

    // control_mode_ = CST_CTRL_MODE;

    test_cnt_ = 777;

#if BIN_PICKING_FLAG
    //// Bin picking node initialize
    m_bin_picking_node->init();
#endif



    //// Graphy PCL
    graphySendUNIZClient_    = node_->create_client<hanyang_matching_msgs::srv::DoPclTest>    ("/graphy_send_uniz_data");

    //// Relay Arduino
    client_relay_command_ = node_->create_client<hanyang_matching_msgs::srv::SendCommand>("/relay_system");



    // ////////////////////////////////////////
    // //// Python UI
    // ROS_LOG_WARN("PYTHON UI INITIALIZE...");
    // m_py_ui_bridge_node->init();
    // ROS_LOG_WARN("PYTHON UI INITIALIZED!");
    // ////////////////////////////////////////







    start(); // qthread의 시작점. start() 함수가 실행되면 자동적으로 run() 함수를 호출한다.
    return true;
}

void QNode::KeycodeAngleCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    if (!msg) return;
    double z_deg = msg->z;
    drum_rotation_angle_ = z_deg;
    keycode_angle_received_ = true;
}

void QNode::KeycodeHolderAngleCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
{
    if (!msg) return;
    double z_deg = msg->z;
    drum_rotation_angle_holder_ = z_deg;
    keycode_holder_angle_received_ = true;
}

void QNode::run() {

    static struct timespec beforeTime;
    static struct timespec currentTime;
    clock_gettime(CLOCK_MONOTONIC, &beforeTime);

    static struct timespec beforeTime_plc_monitoring;
    static struct timespec currentTime_plc_monitoring;
    clock_gettime(CLOCK_MONOTONIC, &beforeTime_plc_monitoring);

    static struct timespec beforeTime_sub1;
    static struct timespec currentTime_sub1;
    clock_gettime(CLOCK_MONOTONIC, &beforeTime_sub1);


    std::thread control_thread([&] () {
        while (rclcpp::ok()) {
            #ifndef TASK_TEST
                // if(!is_task_robot_disable_and_enable) {
                //     if (!params_.status.is_enable) {
                //         is_task_mode_ = false;
                //         current_task_list_.clear();
                //     }
                // }
            #endif


            doControl();

            clock_gettime(CLOCK_MONOTONIC, &currentTime);
            double dt_monitor = (currentTime.tv_sec - beforeTime.tv_sec) + ((double)(currentTime.tv_nsec - beforeTime.tv_nsec) * 0.000000001);
            if(dt_monitor >= 0.01) { // control period: 10ms
                if(m_bin_picking_node->is_task_recognition_on) {
                    m_bin_picking_node->getTaskInfo(is_task_mode_);
                } // bin picking - get task information

                // #if DRFL_CONTROL
                    //// For Matching Node
                    auto bp_robot_state_msg = hanyang_matching_msgs::msg::RobotState();
                    std::vector<double> q_meas_vec;
                    std::vector<double> x_meas_vec;
                    arr2Vec(params_.meas.q, q_meas_vec);
                    arr2Vec(params_.meas.x, x_meas_vec);
                    bp_robot_state_msg.actualq = q_meas_vec;
                    bp_robot_state_msg.actualx = x_meas_vec;
                    bp_robot_state_pub_->publish(bp_robot_state_msg);
                // #endif

                beforeTime = currentTime;
            }



#if IS_PLC_COMMUNICATION

            //// 250804 사용 안함
            // // // clock_gettime(CLOCK_MONOTONIC, &currentTime_plc_monitoring);
            // // // double dt_monitor_plc_monitoring = (currentTime_plc_monitoring.tv_sec - beforeTime_plc_monitoring.tv_sec) + ((double)(currentTime_plc_monitoring.tv_nsec - beforeTime_plc_monitoring.tv_nsec) * 0.000000001);
            // // // if(dt_monitor_plc_monitoring >= 1.0) { // control period: 500ms
            // // //     if(is_monitoring_plc_read_amr_to_robot_status_) {
            // // //     // if(1) {
            // // //         monitorPLCStatus();
            // // //     }
            // // //     beforeTime_plc_monitoring = currentTime_plc_monitoring;
            // // // }
#endif


            ////////////////////////////
            // //// Doosan Get CS Pose, 250221
            // clock_gettime(CLOCK_MONOTONIC, &currentTime_sub1);
            // double dt_monitor_sub1 = (currentTime_sub1.tv_sec - beforeTime_sub1.tv_sec) + ((double)(currentTime_sub1.tv_nsec - beforeTime_sub1.tv_nsec) * 0.000000001);

            //// TODO: ds_controller.cpp에서 service를 참고하여 topic을 만들기
            // if(dt_monitor_sub1 >= 0.08) { // control period: 50ms
            // // if(dt_monitor_sub1 >= 0.05) { // control period: 50ms
            // // if(dt_monitor_sub1 >= 0.05 && !is_task_mode_) { // control period: 50ms
            //     // ROS_LOG_WARN("THIS ...");
            //     #if DRFL_CONTROL
            //         ////////////////////////////////////////
            //         //// Get Measured CS Pose (Srv)
            //         std::array<double, 6> pos = getRobotStateCallback(1);
            //         for (size_t i = 0; i < 3; ++i) {
            //             pos[i] *= 0.001;
            //         }

            //         if(1) {
            //             Eigen::Vector3f euler_angles_zyz_in(pos[3], pos[4], pos[5]); // [deg]
            //             Eigen::Matrix3f R_zyz = m_bp_math.ZYZEulerAnglesToRotationMatrix(euler_angles_zyz_in); // [deg]
            //             Eigen::Vector3f euler_angles_zyx = m_bp_math.rotationMatrixToZYXEulerAngles(R_zyz); // [deg]
            //             for (size_t i = 0; i < 3; ++i) { pos[i + 3] = euler_angles_zyx[i]; } // ZYX Euler angle. [deg]
            //             params_.meas.x = pos;
            //         } else { // test
            //             // double tic_tmp = (double)currentTime_sub1.tv_nsec* 0.000000001;
            //             // ROS_LOG_WARN("*******************************************");
            //             // ROS_LOG_WARN("[%0.6f]%0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f", tic_tmp, pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);
            //             // //// 두산 로봇의 경우, ZYZ angle
            //             // ROS_LOG_WARN("*******************************************");
            //             // ROS_LOG_WARN("[%s]DRFL_CONTROL MODE! - ZYZ to RotM", __func__);
            //             // Eigen::Vector3f euler_angles_zyz_in(pos[3], pos[4], pos[5]); // [deg]
            //             // Eigen::Matrix3f R_zyz = m_bp_math.ZYZEulerAnglesToRotationMatrix(euler_angles_zyz_in); // [deg]
            //             // Eigen::Vector3f euler_angles_zyz = m_bp_math.rotationMatrixToZYZEulerAngles(R_zyz); // [deg]
            //             // cout << "[R_zyz]:\n" << R_zyz << endl;
            //             // cout << "ZYZ Euler Angles (degrees):\n" << euler_angles_zyz.transpose() << endl;
            //             // ROS_LOG_WARN("*******************************************");

            //             // ROS_LOG_WARN("*******************************************");
            //             // ROS_LOG_WARN("[%s]DRFL_CONTROL MODE! - RotM to ZYX", __func__);

            //             // Eigen::Vector3f euler_angles_zyx = m_bp_math.rotationMatrixToZYXEulerAngles(R_zyz); // [deg]
            //             // Eigen::Matrix3f R_zyx = m_bp_math.ZYXEulerAnglesToRotationMatrix(euler_angles_zyx); // [deg]

            //             // cout << "[R_zyx]:\n" << R_zyx << endl;
            //             // cout << "ZYX Euler Angles (degrees):\n" << euler_angles_zyx.transpose() << endl;
            //             // ROS_LOG_WARN("*******************************************");
            //             // ROS_LOG_WARN("*******************************************\n\n");

            //             // // Eigen::Vector3f euler_angles_zyx = m_bp_math.ZYZEulerAnglesToZYXEulerAngles(euler_angles_zyz_in);
            //             // for (size_t i = 0; i < 3; ++i) { pos[i + 3] = euler_angles_zyx[i]; } // ZYX Euler angle. [deg]
            //             // params_.meas.x = pos;
            //         }



            //         ////////////////////////////////////////


            //     #endif

            //     beforeTime_sub1 = currentTime_sub1;
            // }
            ////////////////////////////
            if(SW_MODE_GRAPHY) {
                rclcpp::sleep_for(chrono::milliseconds(8));
            } else {
                rclcpp::sleep_for(chrono::milliseconds(10));
            }
        }
    });

    while (rclcpp::ok()) {
        rclcpp::spin_some(node_);
        rclcpp::sleep_for(chrono::milliseconds(10));
    }

    rclcpp::shutdown();
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QNode::setTaskMode(bool is_task) {
    if (is_task) {
        is_task_mode_ = true;
        is_simul_mode_ = false;
    } else {
        is_task_mode_ = false;
        is_simul_mode_ = true;
    }
}

bool QNode::setPower(bool flag) {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();
    req->flag = flag;

    auto result = client_power_on_off_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setPower");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setPower");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setPower Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setEnable(bool flag) {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();
    req->flag = flag;

    auto result = client_set_enable_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setEnable");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setEnable");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setEnable Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setAutomaticMode(bool flag) {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();
    req->flag = flag;

    auto result = client_set_AutomaticMode_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setEnable");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setEnable");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setEnable Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}
bool QNode::setManualMode(bool flag) {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();
    req->flag = flag;

    auto result = client_set_ManualMode_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setEnable");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setEnable");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setEnable Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::jammingStateEnable() {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();
    auto result = client_jamming_state_enable_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: jammingStateEnable");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: jammingStateEnable");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : jammingStateEnable Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::clearError() {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();
    req->flag = true;

    auto result = client_clear_error_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: clearError");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: clearError");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : clearError Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setHandGuide(bool flag) {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();
    req->flag = flag;

    auto result = client_set_hand_guide_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setHandGuide");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setHandGuide");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setHandGuide Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setFrictionObserver(bool flag) {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();
    req->flag = flag;

    auto result = client_set_friction_observer_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setFrictionObserver");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setFrictionObserver");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setFrictionObserver Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setCollisionDetection(bool flag) {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();
    req->flag = flag;

    auto result = client_set_collision_detection_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setCollisionDetection");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setCollisionDetection");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setCollisionDetection Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setCollisionDemoMode(bool flag) {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();
    req->flag = flag;

    auto result = client_set_collision_demo_mode_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setCollisionDemoMode");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setCollisionDemoMode");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setCollisionDemoMode Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setInvKineMethod(string ik_method) {
    auto req = make_shared<kcr_control_msg::srv::SingleString::Request>();
    req->command = ik_method;

    auto result = client_set_inv_kine_method_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setInvKineMethod");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setInvKineMethod");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setInvKineMethod Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setDriverCtrlMode(string driver_ctrl_mode) {
    auto req =make_shared<kcr_control_msg::srv::SingleString::Request>();
    req->command = driver_ctrl_mode;

    auto result = client_set_driver_ctrl_mode_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setDriverCtrlMode");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setDriverCtrlMode Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::updateParam() {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();

    auto result = client_update_param_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: updateParam");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: updateParam");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : updateParam Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::rollbackParam() {
    auto req = make_shared<kcr_control_msg::srv::SingleBool::Request>();

    auto result = client_rollback_param_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: rollbackParam");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: rollbackParam");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : rollbackParam Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::recordingStart(std::string record_name) {
    auto req = std::make_shared<kcr_control_msg::srv::SingleString::Request>();
    req->command = record_name;

    auto result = client_record_start_->async_send_request(req);
    auto status = result.wait_for(std::chrono::milliseconds(100));

    if (status == std::future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tCommand: %s\n\tFile name: [%s] : %d)", __FUNCTION__, record_name.c_str(), __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: recordingStart");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s\n\tCommand: %s", __FUNCTION__, record_name.c_str());
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: recordingStart");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tCommand: %s\n\tError code: %d", __FUNCTION__, record_name.c_str(), recv->error);
        SEND_ERROR_TO_MONITOR("Function: : recordingStart Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::recordingEnd() {
    auto req = std::make_shared<kcr_control_msg::srv::SingleBool::Request>();

    auto result = client_record_end_->async_send_request(req);
    auto status = result.wait_for(std::chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: recordingEnd");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: recordingEnd");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : recordingEnd Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

MoveErrorType QNode::moveQ(JsDouble q_target, double qd_target, double qdd_target, bool is_relative) {

    ROS_LOG_WARN("****************** [%s] **********************", __func__);
    std::cout << "move q" << std::endl;
        // q_target 출력
        std::cout << "q_target: [";
        for (size_t i = 0; i < q_target.size(); ++i) {
            std::cout << q_target[i];
            if (i < q_target.size() - 1) {
                std::cout << ", ";
            }
        }
        std::cout << "]" << std::endl;

    // qd_target, qdd_target, is_relative 출력
    std::cout << "qd_target: " << qd_target << std::endl;
    std::cout << "qdd_target: " << qdd_target << std::endl;
    std::cout << "is_relative: " << (is_relative ? "true" : "false") << std::endl;

#if DRFL_CONTROL
    auto req = make_shared<dsr_msgs2::srv::MoveJoint::Request>();
    // float[NUM_JOINT] 배열로 변환
    // float targetPos[JS_DOF];
    for (size_t i = 0; i < JS_DOF; ++i) {
        req->pos[i] = static_cast<float>(q_target[i]); // double을 float으로 변환
    }
    req->vel         = static_cast<float>(qd_target);
    req->acc         = static_cast<float>(qdd_target);
    req->mode        = is_relative ? MOVE_MODE_RELATIVE : MOVE_MODE_ABSOLUTE;

    req->sync_type = 1; // 0: sync., 1: async.

    auto result = client_move_q_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));
    auto recv = result.get();

    if (recv->success) {
        if(is_task_mode_) { // Doosan 모션 여부 체크
            do_check_js_err_ = true;
            is_robot_moving_begin_ = false;
            cnt_for_is_js_err_ok_ = 0;
            cnt_for_is_js_err_exceeds_ = 0;
        }
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: moveQ");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : moveQ Error code:" + recv->success);
        is_task_mode_ = false;
        return MoveErrorType::TIME_OUT;
    }

    ROS_LOG_WARN("**********************************************\n");
    return MoveErrorType::NONE;

#else
    auto req = make_shared<kcr_control_msg::srv::MoveQ::Request>();
    arr2Vec(q_target, req->q_target);
    req->qd_target   = qd_target;
    req->qdd_target  = qdd_target;
    req->is_relative = is_relative;

    auto result = client_move_q_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: moveQ");
        return MoveErrorType::TIME_OUT;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: moveQ");
    } else if (recv->error == (uint)MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
        ROS_LOG_WARN("Function: %s\n\tFailed by collision", __FUNCTION__);
        SEND_WARN_TO_MONITOR("Function: moveQ Failed by collision");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : moveQ Error code:" + recv->error);
        is_task_mode_ = false;
    }

    return (MoveErrorType) recv->error;
#endif

}

MoveErrorType QNode::moveX(CsDouble x_target, double xd_target, double xdd_target, bool is_base_frame, bool is_relative, double q_redundant) {
#if DRFL_CONTROL

    ROS_LOG_WARN("****************** [%s] **********************", __func__);
    std::cout << "[ZYX] x_target(mm, deg):            [";
    for (size_t i = 0; i < x_target.size(); ++i) {
        std::cout << (i < 3? x_target[i] * 1000 : x_target[i]);
        if (i < x_target.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
    // ROS_LOG_WARN("%0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f", pos[0], pos[1], pos[2], pos[3], pos[4], pos[5]);


    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    //// Euler ZYX to ZYZ
    // 1) ZYX to Rotation Matrix
    ROS_LOG_WARN("*******************************************");
    ROS_LOG_WARN("[%s] ZYX to Rotation Matrix...", __func__);
    Eigen::Vector3f euler_angles_zyx_in(x_target[3], x_target[4], x_target[5]); // [deg]
    Eigen::Matrix3f R_zyx = m_bp_math.ZYXEulerAnglesToRotationMatrix(euler_angles_zyx_in); // [deg]
    cout << "[R_zyx]:\n" << R_zyx << endl;
    cout << "ZYX Euler Angles (degrees):\n" << euler_angles_zyx_in.transpose() << endl;
    ROS_LOG_WARN("*******************************************");

    // 2) Rotation matrix to ZYZ
    ROS_LOG_WARN("*******************************************");
    Eigen::Vector3f euler_angles_zyz = m_bp_math.rotationMatrixToZYZEulerAngles(R_zyx); // [deg]
    Eigen::Matrix3f R_zyz = m_bp_math.ZYZEulerAnglesToRotationMatrix(euler_angles_zyz); // [deg]
    cout << "[R_zyz]:\n" << R_zyz << endl;
    cout << "ZYZ Euler Angles (degrees):\n" << euler_angles_zyz.transpose() << endl;
    ROS_LOG_WARN("*******************************************");

    // Check ZYX
    ROS_LOG_WARN("*******************************************");
    Eigen::Vector3f euler_angles_zyx = m_bp_math.rotationMatrixToZYXEulerAngles(R_zyz); // [deg]
    Eigen::Matrix3f R_zyx_check = m_bp_math.ZYXEulerAnglesToRotationMatrix(euler_angles_zyx); // [deg]
    cout << "[R_zyx_check]:\n" << R_zyx_check << endl;
    cout << "[Check] ZYX Euler Angles (degrees):\n" << euler_angles_zyx.transpose() << endl;
    ROS_LOG_WARN("*******************************************");

    // 3) Update target orientation
    ROS_LOG_WARN("*******************************************");
    for (size_t i = 0; i < x_target.size(); ++i) { x_target[i + 3] = euler_angles_zyz[i]; }
    std::cout << "[ZYZ]x_target(mm, deg):            [";
    for (size_t i = 0; i < x_target.size(); ++i) {
        std::cout << (i < 3? x_target[i] * 1000 : x_target[i]);
        if (i < x_target.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
    ROS_LOG_WARN("*******************************************");
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////

    // xd_target, xdd_target, is_base_frame, is_relative, q_redundant 출력
    std::cout << "xd_target(deg/s, mm/s):       " << xd_target * 1000 << std::endl;
    std::cout << "xdd_target(deg/s^2, mm/s^2): " << xdd_target * 1000  << std::endl;
    std::cout << "is_base_frame: " << (is_base_frame ? "true" : "false") << std::endl;
    std::cout << "is_relative: " << (is_relative ? "true" : "false") << std::endl;

    auto req = make_shared<dsr_msgs2::srv::MoveLine::Request>();

   // Position 변환: CsDouble에서 float[NUM_TASK]로 변환
    // float targetPos[CS_DOF];
    for (size_t i = 0; i < CS_DOF; ++i) {
        req->pos[i] = i < 3 ? static_cast<float>(x_target[i] * 1000) :
                              static_cast<float>(x_target[i]); // double을 float으로 변환
    }
    for (size_t i = 0; i < 2; ++i) {
        req->vel[i] = static_cast<float> (xd_target * 1000);
    }
    for (size_t i = 0; i < 2; ++i) {
        req->acc[i] = static_cast<float> (xdd_target * 1000);
    }

    // MOVE_MODE 설정 (절대/상대)
    req->mode = is_relative ? MOVE_MODE_RELATIVE : MOVE_MODE_ABSOLUTE;

    // MOVE_REFERENCE 설정 (기준 프레임: base 또는 tool)
    req->ref = is_base_frame ? MOVE_REFERENCE_BASE : MOVE_REFERENCE_TOOL;

    req->sync_type = 1; // 0: sync., 1: async.


    auto result = client_move_x_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(200));
    auto recv = result.get();

    if (recv->success) {
        if(is_task_mode_) { // Doosan 모션 여부 체크
            do_check_js_err_ = true;
            is_robot_moving_begin_ = false;
            cnt_for_is_js_err_ok_ = 0;
            cnt_for_is_js_err_exceeds_ = 0;
        }
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: moveQ");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : moveQ Error code:" + recv->success);
        is_task_mode_ = false;
        return MoveErrorType::TIME_OUT;
    }
    ROS_LOG_WARN("**********************************************\n");

    return MoveErrorType::NONE;

#else

    std::cout << "x_target: [";
    for (size_t i = 0; i < x_target.size(); ++i) {
        std::cout << x_target[i];
        if (i < x_target.size() - 1) {
            std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;

    // xd_target, xdd_target, is_base_frame, is_relative, q_redundant 출력
    std::cout << "xd_target: " << xd_target << std::endl;
    std::cout << "xdd_target: " << xdd_target << std::endl;
    std::cout << "is_base_frame: " << (is_base_frame ? "true" : "false") << std::endl;
    std::cout << "is_relative: " << (is_relative ? "true" : "false") << std::endl;

    auto req = make_shared<kcr_control_msg::srv::MoveX::Request>();
    arr2Vec(x_target, req->x_target);
    req->xd_target     = xd_target;
    req->xdd_target    = xdd_target;
    req->is_base_frame = is_base_frame;
    req->is_relative   = is_relative;
    req->q_redundant   = q_redundant;

    auto result = client_move_x_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: moveX");
        return MoveErrorType::TIME_OUT;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: moveX");
    } else if (recv->error == (uint)MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
        ROS_LOG_WARN("Function: %s\n\tFailed by collision", __FUNCTION__);
        SEND_WARN_TO_MONITOR("Function: moveX Failed by collision");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : moveX Error code:" + recv->error);
        is_task_mode_ = false;
    }

    return (MoveErrorType) recv->error;
#endif
}

MoveErrorType QNode::moveSplineQ(std::vector<JsDouble>& q_targets, double qd_target, double qdd_target, bool is_relative) {
#if DRFL_CONTROL
    std::cout << "move spline q" << std::endl;
    auto req = make_shared<dsr_msgs2::srv::MoveSplineJoint::Request>();

    // q_target 출력
    auto pos_cnt_ = q_targets.size();
    req->pos_cnt = pos_cnt_;
    std::cout << "pos_cnt: " << pos_cnt_ << std::endl;

    std::cout << "q_targets: [";
    for (size_t i = 0; i < pos_cnt_; i++) {
        std::cout << "[";
        std_msgs::msg::Float64MultiArray pos_;
        for (size_t j = 0; j < JS_DOF; j++) {
            pos_.data.push_back(q_targets[i][j]);
            std::cout << q_targets[i][j];
            if (j < JS_DOF - 1) {std::cout << ", ";}
        }
        req->pos.push_back(pos_);
        if (i < pos_cnt_ - 1) {std::cout << "],\n";}
        else {std::cout << "]";}
    }
    std::cout << "]" << std::endl;

    // qd_target, qdd_target, is_relative 출력
    std::cout << "qd_target(deg/s):       " << qd_target << std::endl;
    std::cout << "qdd_target(deg/s^2): " << qdd_target << std::endl;
    std::cout << "is_relative: " << (is_relative ? "true" : "false") << std::endl;

    for (size_t i = 0; i < 6; ++i) {
        req->vel[i] = static_cast<float> (qd_target);
        req->acc[i] = static_cast<float> (qdd_target);
    }
    // req->time = 10;

    // MOVE_MODE 설정 (절대/상대)
    req->mode = is_relative ? MOVE_MODE_RELATIVE : MOVE_MODE_ABSOLUTE;

    auto result = client_move_spline_q_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(200));
    auto recv = result.get();

    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: moveQ");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : moveQ Error code:" + recv->success);
        is_task_mode_ = false;
        return MoveErrorType::TIME_OUT;
    }
#endif
    return MoveErrorType::NONE;
}

MoveErrorType QNode::moveSplineX(std::vector<CsDouble>& x_targets, double xd_target, double xdd_target, bool is_base_frame, bool is_relative, double q_redundant) {
#if DRFL_CONTROL
    std::cout << "move spline x" << std::endl;
    auto req = make_shared<dsr_msgs2::srv::MoveSplineTask::Request>();

    // x_target 출력
    auto pos_cnt_ = x_targets.size();
    req->pos_cnt = pos_cnt_;
    std::cout << "pos_cnt: " << pos_cnt_ << std::endl;

    std::cout << "x_targets: [";
    for (size_t i = 0; i < pos_cnt_; i++) {
        std::cout << "[";
        std_msgs::msg::Float64MultiArray pos_;
        for (size_t j = 0; j < JS_DOF; j++) {
            pos_.data.push_back(x_targets[i][j]);
            std::cout << x_targets[i][j];
            if (j < JS_DOF - 1) {std::cout << ", ";}
        }
        req->pos.push_back(pos_);
        if (i < pos_cnt_ - 1) {std::cout << "],\n";}
        else {std::cout << "]";}
    }
    std::cout << "]" << std::endl;

    // xd_target, xdd_target, is_base_frame, is_relative, q_redundant 출력
    std::cout << "xd_target(deg/s, mm/s):       " << xd_target * 1000 << std::endl;
    std::cout << "xdd_target(deg/s^2, mm/s^2): " << xdd_target * 1000  << std::endl;
    std::cout << "is_base_frame: " << (is_base_frame ? "true" : "false") << std::endl;
    std::cout << "is_relative: " << (is_relative ? "true" : "false") << std::endl;

    for (size_t i = 0; i < 2; ++i) {
        req->vel[i] = static_cast<float> (xd_target * 1000);
        req->acc[i] = static_cast<float> (xdd_target * 1000);
    }
    // req->time = 10;

    // MOVE_MODE 설정 (절대/상대)
    req->mode = is_relative ? MOVE_MODE_RELATIVE : MOVE_MODE_ABSOLUTE;

    // MOVE_REFERENCE 설정 (기준 프레임: base 또는 tool)
    req->ref = is_base_frame ? MOVE_REFERENCE_BASE : MOVE_REFERENCE_TOOL;

    auto result = client_move_spline_x_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(200));
    auto recv = result.get();

    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: moveQ");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : moveX Error code:" + recv->success);
        is_task_mode_ = false;
        return MoveErrorType::TIME_OUT;
    }


#endif

    return MoveErrorType::NONE;
}

bool QNode::drflSetRobotMode(int mode) {
#if DRFL_CONTROL
    // 0 : ROBOT_MODE_MANUAL
    // 1 : ROBOT_MODE_AUTONOMOUS
    // 2 : ROBOT_MODE_MEASURE
    auto req = make_shared<dsr_msgs2::srv::SetRobotMode::Request>();
    req->robot_mode = mode;

    auto result = client_set_robot_mode_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));
    auto recv = result.get();

    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: drflSetRobotMode");
    } else {
        bool result_servo_off = drflServoOff(3);
        bool result_servo_on = drflSetRobotControl(3);
        if (result_servo_off && result_servo_on) {
            result = client_set_robot_mode_->async_send_request(req);
            status = result.wait_for(chrono::milliseconds(200));
            recv = result.get();
            return true;
        } else {
            ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
            SEND_ERROR_TO_MONITOR("Function: : drflSetRobotMode Error code:" + recv->success);
            return false;
        }
    }
    return true;
#else
    return true;
#endif
}

bool QNode::drflServoOff(int type) {
#if DRFL_CONTROL
    auto req = make_shared<dsr_msgs2::srv::ServoOff::Request>();
    req->stop_type = type;

    auto result = client_servo_off_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(200));
    auto recv = result.get();

    if (recv->success) {
    ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
    //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: drflServoOff");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : drflServoOff Error code:" + recv->success);
    }

    return recv->success;
#else
    return true;
#endif
}


bool QNode::drflMoveStop(int stop_mode) {
#if DRFL_CONTROL
    // dsr_msgs2::srv::MoveStop 요청 메시지 객체 생성 및 정지 모드 설정
    auto req = std::make_shared<dsr_msgs2::srv::MoveStop::Request>();
    req->stop_mode = stop_mode;  // 전달된 정지 모드 사용

    // 올바른 클라이언트를 사용하여 MoveStop 서비스 호출
    auto result = client_stop_robot->async_send_request(req);

    // 최대 100ms 동안 응답 대기
    auto status = result.wait_for(std::chrono::milliseconds(100));
    if (status == std::future_status::timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("QNode"), "MoveStop service call timed out.");
        return false;
    }

    // 응답 획득 및 성공 여부 확인
    auto response = result.get();
    if (response->success) {
        RCLCPP_INFO(rclcpp::get_logger("QNode"), "MoveStop service succeeded with stop_mode %d", stop_mode);
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("QNode"), "MoveStop service failed for stop_mode %d", stop_mode);
        return false;
    }
#else
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "DRFL_CONTROL not enabled. MoveStop not executed.");
    return false;
#endif
}

void QNode::drfl_currentpose(const dsr_msgs2::msg::RobotState::SharedPtr msg){
#if DRFL_CONTROL
    std::cout << "Received current_posx: ";
    for (int i = 0; i < 6; ++i) {
        params_.meas.x[i] = msg->current_posx[i];
        std::cout << params_.meas.x[i] << " ";
    }
    std::cout << std::endl;
    RCLCPP_INFO(rclcpp::get_logger("QNode"),
                "Updated current pose: [%f, %f, %f, %f, %f, %f]",
                params_.meas.x[0], params_.meas.x[1], params_.meas.x[2],
                params_.meas.x[3], params_.meas.x[4], params_.meas.x[5]);
#else
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "Current Pose FAILED");
#endif
}

#if DRFL_CONTROL
void QNode::requestRobotState()
{
    if (!client_get_robot_state_->service_is_ready()) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(),
                             *node_->get_clock(), 10000,
                             "GetRobotState service not ready");
        return;
    }

    auto req = std::make_shared<dsr_msgs2::srv::GetRobotState::Request>(); // 빈 요청
    client_get_robot_state_->async_send_request(
        req,
        std::bind(&QNode::robotStateRespCb,
                  this, std::placeholders::_1));
}

/* 서비스 응답 처리 */
void QNode::robotStateRespCb(
    rclcpp::Client<dsr_msgs2::srv::GetRobotState>::SharedFuture future)
{
    auto res = future.get();
    if (!res->success) {
        // RCLCPP_ERROR(node_->get_logger(), "GetRobotState returned success=false");
        return;
    }

    int st = res->robot_state;
    // RCLCPP_INFO(node_->get_logger(), "%s[GetRobotState] robot_state=%d%s", GREEN, st, RESET);


    /* ─── 새 로직: STATE_SAFE_STOP(5) 일 때 PLC 신호 ─── */
    if (st == 5) {      
        is_doosan_robot_collision_ = true;
        // RCLCPP_INFO(node_->get_logger(), "%sis_doosan_robot_collision_ = true;=%d%s", GREEN, st, RESET);

        if (!plc_safe_stop_sent_) {                          // 아직 안 보냈다면

        }
    }
    else {                                                   // 다른 상태
        // RCLCPP_INFO(node_->get_logger(), "%sis_doosan_robot_collision_ = false;%s", GREEN, RESET);
        is_doosan_robot_collision_ = false;

    }
}




/* ── 토픽 수신 콜백 ───────────────────────────── */
void QNode::dsrRobotStateCb(
        const dsr_msgs2::msg::RobotState::SharedPtr msg)
{
    dsr_last_state_  = *msg;      // 전체 복사
    dsr_state_ready_ = true;
}

/* ── 1 초마다 값 출력 ─────────────────────────── */
void QNode::dsrStatePrintTimerCb()
{
    if (!dsr_state_ready_) return;          // 첫 수신 전이면 skip

    const auto& s = dsr_last_state_;

    /* actual_bk[6] 배열을 문자열로 변환 */
    std::ostringstream bk;
    for (int i = 0; i < 6; ++i)
        bk << int(s.actual_bk[i]) << (i < 5 ? ' ' : '\0');

    RCLCPP_INFO(
        node_->get_logger(),
        "[DSR] robot_state: %d (%s) | actual_bk: [%s] | drl_stopped: %s",
        s.robot_state,
        s.robot_state_str.c_str(),
        bk.str().c_str(),
        s.drl_stopped ? "true" : "false");
}
#endif


// 두산 로봇 임피던스 제어 관련
bool QNode::drflTaskComplianceCtrl() {
#if DRFL_CONTROL
    // 버튼 클릭 시 on-demand 방식으로 서비스 클라이언트를 생성
    auto local_client = node_->create_client<dsr_msgs2::srv::TaskComplianceCtrl>("/dsr01/force/task_compliance_ctrl");

    // 서비스 서버가 준비될 때까지 최대 1초 대기
    if (!local_client->wait_for_service(std::chrono::seconds(1))) {
        // RCLCPP_ERROR(this->get_logger(), "TaskComplianceCtrl service not available.");
        return false;
    }

    // 요청 메시지 생성 및 기본 강성값, 기준 좌표계, 전환 시간 설정
    auto req = std::make_shared<dsr_msgs2::srv::TaskComplianceCtrl::Request>();
    req->stx[0] = 2000.0;
    req->stx[1] = 2000.0;
    req->stx[2] = 2000.0;
    req->stx[3] = 100.0;
    req->stx[4] = 100.0;
    req->stx[5] = 100.0;
    req->ref   = 0;     // 기본 기준 좌표계
    req->time  = 0.0;   // 0초: 즉시 적용

    // 서비스 호출
    auto result = local_client->async_send_request(req);
    auto status = result.wait_for(std::chrono::milliseconds(500));
    if (status == std::future_status::timeout) {
        // RCLCPP_ERROR(this->get_logger(), "TaskComplianceCtrl service call timed out.");
        return false;
    }

    auto response = result.get();
    if (response->success) {
        // RCLCPP_INFO(this->get_logger(), "TaskComplianceCtrl service succeeded.");
        return true;
    } else {
        // RCLCPP_ERROR(this->get_logger(), "TaskComplianceCtrl service failed.");
        return false;
    }
#else
    // RCLCPP_WARN(this->get_logger(), "DRFL_CONTROL not enabled. TaskComplianceCtrl not executed.");
    return false;
#endif
}


bool QNode::drflReleaseComplianceCtrl() {
#if DRFL_CONTROL
    // ReleaseComplianceCtrl 서비스 요청 메시지 객체 생성 (요청 필드가 없으므로 빈 객체)
    auto req = std::make_shared<dsr_msgs2::srv::ReleaseComplianceCtrl::Request>();

    auto result = client_release_compliance_ctrl->async_send_request(req);

    // 최대 100ms 동안 응답 대기
    auto status = result.wait_for(std::chrono::milliseconds(100));
    if (status == std::future_status::timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("QNode"), "ReleaseComplianceCtrl service call timed out.");
        return false;
    }

    auto response = result.get();
    if (response->success) {
        RCLCPP_INFO(rclcpp::get_logger("QNode"), "ReleaseComplianceCtrl service succeeded.");
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("QNode"), "ReleaseComplianceCtrl service failed.");
        return false;
    }
#else
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "DRFL_CONTROL not enabled. ReleaseComplianceCtrl not executed.");
    return false;
#endif
}

/////250807 dsr 토크 추가 ing
bool QNode::drflGetJointTorque() {
#if DRFL_CONTROL
    auto req = std::make_shared<dsr_msgs2::srv::GetJointTorque::Request>();

    auto result = client_get_dsr_torque->async_send_request(req);

    // 최대 100ms 동안 응답 대기
    auto status = result.wait_for(std::chrono::milliseconds(100));
    if (status == std::future_status::timeout) {
        // RCLCPP_ERROR(rclcpp::get_logger("QNode"), "GetJointTorque service call timed out.");
        return false;
    }

    auto response = result.get();
    if (response->success) {
        // RCLCPP_INFO(rclcpp::get_logger("QNode"), "GetJointTorque service succeeded.");
        
        // 토크 데이터 출력
        std::string torque_info = "Joint Torque Values: ";
        for (size_t i = 0; i < response->jts.size(); ++i) {
            char buffer[50];
            snprintf(buffer, sizeof(buffer), "J%d: %.3f", (int)(i + 1), response->jts[i]);
            torque_info += buffer;
            if (i < response->jts.size() - 1) {
                torque_info += ", ";
            }
        }
        // RCLCPP_INFO(rclcpp::get_logger("QNode"), "%s", torque_info.c_str());
        // 내부 상태에 저장하여 토크 가드가 즉시 활용할 수 있도록 반영
        size_t n = std::min(response->jts.size(), (size_t)JS_DOF);
        for (size_t i = 0; i < n; ++i) {
            params_.meas.torque_jts[i] = response->jts[i];
        }
        
        return true;
    } else {
        // RCLCPP_ERROR(rclcpp::get_logger("QNode"), "GetJointTorque service failed.");
        return false;
    }
#else
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "DRFL_CONTROL not enabled. GetJointTorque not executed.");
    return false;
#endif
}


bool QNode::drflSetCurrentTool(const std::string &tool_name) {
#if DRFL_CONTROL
    auto req = std::make_shared<dsr_msgs2::srv::SetCurrentTool::Request>();
    req->name = tool_name;

    ROS_LOG_WARN("drflSetCurrentTool: tool_name = %s", tool_name.c_str());

    auto result = client_set_current_tool_->async_send_request(req);
    auto status = result.wait_for(std::chrono::milliseconds(200));
    if (status == std::future_status::timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("QNode"), "SetCurrentTool service call timed out.");
        return false;
    }

    auto recv = result.get();
    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s failed with error code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("drflSetCurrentTool Error code:" + std::to_string(recv->success));
        return false;
    }
#else
    return true;
#endif
}

bool QNode::drflConfigCreateTool(const std::string &tool_name, double weight, const std::array<double, 3> &cog, const std::array<double, 6> &inertia) {
#if DRFL_CONTROL
    auto req = std::make_shared<dsr_msgs2::srv::ConfigCreateTool::Request>();
    req->name   = tool_name;
    req->weight = weight;

    for (size_t i = 0; i < cog.size(); ++i) {
        req->cog[i] = cog[i];
    }
    for (size_t i = 0; i < inertia.size(); ++i) {
        req->inertia[i] = inertia[i];
    }

    ROS_LOG_WARN("drflConfigCreateTool: tool=%s, weight=%.2f", tool_name.c_str(), weight);

    auto result = client_config_create_tool_->async_send_request(req);
    auto status = result.wait_for(std::chrono::milliseconds(200));
    if (status == std::future_status::timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("QNode"), "ConfigCreateTool service call timed out.");
        return false;
    }

    auto recv = result.get();
    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s failed with error code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("drflConfigCreateTool Error code:" + std::to_string(recv->success));
        is_task_mode_ = false;
        return false;
    }
#else
    return true;
#endif
}

bool QNode::drflChangeCollisionSensitivity(int sensitivity) {
#if DRFL_CONTROL
    // ChangeCollisionSensitivity 요청 메시지 객체 생성
    auto req = std::make_shared<dsr_msgs2::srv::ChangeCollisionSensitivity::Request>();
    req->sensitivity = sensitivity;  // 예: 100

    auto result = client_change_collision_sensitivity_->async_send_request(req);
    // 최대 200ms 동안 응답 대기 (필요시 시간 조정 가능)
    auto status = result.wait_for(std::chrono::milliseconds(1000));
    if (status == std::future_status::timeout) {
        RCLCPP_ERROR(rclcpp::get_logger("QNode"), "ChangeCollisionSensitivity service call timed out.");
        return false;
    }

    auto response = result.get();
    if (response->success) {
        RCLCPP_INFO(rclcpp::get_logger("QNode"), "ChangeCollisionSensitivity service succeeded.");
        return true;
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("QNode"), "ChangeCollisionSensitivity service failed.");
        return false;
    }
#else
    RCLCPP_WARN(rclcpp::get_logger("QNode"), "DRFL_CONTROL not enabled. ChangeCollisionSensitivity not executed.");
    return false;
#endif
}


bool QNode::drflSetRobotControl(int type) {
#if DRFL_CONTROL
    auto req = make_shared<dsr_msgs2::srv::SetRobotControl::Request>();
    req->robot_control = type;

    auto result = client_set_robot_control_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(200));
    auto recv = result.get();

    if (recv->success) {
    ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
    //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: drflSetRobotControl");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : drflSetRobotControl Error code:" + recv->success);
    }

    return recv->success;
#else
    return true;
#endif
}

MoveErrorType QNode::jogStart(uint8_t index, double vel, double acc, bool is_joint_space, bool is_base_frame) {
#if DRFL_CONTROL
    auto req = make_shared<dsr_msgs2::srv::Jog::Request>();

    if (is_joint_space) {   // js
        jog_index_ = index;
        req->jog_axis = jog_index_;
        req->move_reference = is_base_frame ? MOVE_REFERENCE_BASE : MOVE_REFERENCE_TOOL;
        req->speed = static_cast<float> (vel);
    } else {                // cs
        jog_index_ = 6 + index;
        req->jog_axis = jog_index_;
        req->move_reference = is_base_frame ? MOVE_REFERENCE_BASE : MOVE_REFERENCE_TOOL;
        req->speed = (index < 3) ? static_cast<float> (vel * 1000) : static_cast<float> (vel);
    }

    auto result = client_jog_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    // if (status == future_status::timeout) {
    //     ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
    //     SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: jogStart");
    //     return MoveErrorType::TIME_OUT;
    // }

    auto recv = result.get();

    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: jogStart");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : jogStart Error code:" + recv->success);
        return MoveErrorType::TIME_OUT;
    }

    return MoveErrorType::NONE;


#else
    if (is_task_mode_) {
        ROS_LOG_WARN("Jog start: End task mode");
        is_task_mode_ = false;
    }

    auto req = make_shared<kcr_control_msg::srv::Jog::Request>();

    req->jog_flag = true;
    req->vel = vel;
    req->acc = acc;
    req->index = index;
    req->is_joint_space = is_joint_space;
    req->is_base_frame  = is_base_frame;

    auto result = client_jog_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: jogStart");
        return MoveErrorType::TIME_OUT;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: jogStart");
    } else if (recv->error == (uint)MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
        ROS_LOG_WARN("Function: %s\n\tFailed by collision", __FUNCTION__);
        SEND_WARN_TO_MONITOR("Function: jogStart Failed by collision");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : jogStart Error code:" + recv->error);
    }

    return (MoveErrorType) recv->error;
#endif
}

MoveErrorType QNode::jogEnd(double stop_time) {
#if DRFL_CONTROL
    auto req = make_shared<dsr_msgs2::srv::Jog::Request>();
    req->jog_axis = jog_index_;
    req->move_reference = jog_frame;
    req->speed = 0;

    jog_index_ = -1;
    jog_frame = -1;

    auto result = client_jog_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: jogStart");
        return MoveErrorType::TIME_OUT;
    }

    auto recv = result.get();

    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: jogStart");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : jogStart Error code:" + recv->success);
        return MoveErrorType::TIME_OUT;
    }

    return MoveErrorType::NONE;

#else
    auto req = make_shared<kcr_control_msg::srv::Jog::Request>();

    req->jog_flag  = false;
    req->stop_time = stop_time;

    auto result = client_jog_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: jogEnd");
        return MoveErrorType::TIME_OUT;
    }
    
    // return (MoveErrorType) recv->error;
    return MoveErrorType::NONE;
#endif
}

MoveErrorType QNode::moveBlending(BlendingTraj traj_blending) {
    auto req = make_shared<kcr_control_msg::srv::Blending::Request>();

    req->xds_target   = traj_blending.xds;
    req->xdds_target  = traj_blending.xdds;
    req->waypoint_xds = traj_blending.waypoint_xds;
    req->radiuses     = traj_blending.radiuses;
    req->qs_redundant = traj_blending.qs_redundant;

    req->waypoint_num = traj_blending.waypoint_num;
    req->is_radius_percent = traj_blending.is_radius_percent;

    req->xs_target.clear();

    for (int i = 0; i < traj_blending.waypoint_num; i++) {
        for (int j = 0; j < CS_DOF; j++) {
            req->xs_target.push_back(traj_blending.xs[i][j]);
        }
    }

    auto result = client_blending_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: moveBlending");
        return MoveErrorType::TIME_OUT;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: moveBlending");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : moveBlending Error code:" + recv->error);
        is_task_mode_ = false;
    }

    return (MoveErrorType) recv->error;
}

bool QNode::setDtcMode(JsBool dtc_mode) {
    auto req = make_shared<kcr_control_msg::srv::DirectTorqueControl::Request>();

    req->setting_mode = true;
    arr2Vec(dtc_mode, req->dtc_mode);

    auto result = client_dtc_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setDtcMode");
        return false;
    }

    return true;
}

bool QNode::setDtcInput(JsDouble dtc_input) {
    auto req = make_shared<kcr_control_msg::srv::DirectTorqueControl::Request>();

    req->setting_mode = false;
    arr2Vec(dtc_input, req->dtc_input);

    auto result = client_dtc_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setDtcInput");
        return false;
    }

    return true;
}

bool QNode::stopRobot(double time_stop) {
#if DRFL_CONTROL
    return true;
#else
    auto req = make_shared<kcr_control_msg::srv::StopRobot::Request>();
    req->time_stop = time_stop;

    auto result = client_stop_robot_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: stopRobot");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: stopRobot");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : stopRobot Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
#endif
}

bool QNode::setCommand(string cmd) {
    auto req = make_shared<kcr_control_msg::srv::SingleString::Request>();
    req->command = cmd;

    auto result = client_set_command_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tCommand: %s\n\tFile name: [%s] : %d)", __FUNCTION__, cmd.c_str(), __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setCommand" + *cmd.c_str());
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s\n\tCommand: %s", __FUNCTION__, cmd.c_str());
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setCommand" + *cmd.c_str());
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tCommand: %s\n\tError code: %d", __FUNCTION__, cmd.c_str(), recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setCommand Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

///////////////////////////////////////////////////////////////////////////
/////////////////////////////      LLM     ////////////////////////////////
///////////////////////////////////////////////////////////////////////////


void QNode::LLMStatusCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg->data == "ready") {
        // GUI에 신호를 보냅니다.
        Q_EMIT LLMNodeReady();
    }
}

void QNode::LLMStartRecording() {
    ROS_LOG_INFO("######################### LLM #########################");
    ROS_LOG_INFO("Start Recording ...");
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_start_record_voice_->async_send_request(req);
}

void QNode::LLMStopRecording() {
    ROS_LOG_INFO("######################### LLM #########################");
    ROS_LOG_INFO("Stop Recording ...");
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    client_stop_record_voice_->async_send_request(req);
}

void QNode::LLMGoalPoseCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    is_box_pose_received_ = true;
    ROS_LOG_INFO("Goal Pose received: Pose2D - X: %f, Y: %f, RZ: %f", msg->x, msg->y, msg->theta);
    emit UpdateGoalPose(*msg);
        // qDebug() << "Received pose for tool:" << toolName
        //          << "x=" << msg->x << ", y=" << msg->y << ", z=" << z
        //          << ", roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw;
    // } else {
    //     qDebug() << "Invalid pose data size: " << msg->data.size();
    // }
}

void QNode::LLMPointCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    cout << "waypoint received" << endl;
    Coordinates coords = {msg->x, msg->y, msg->z};
    pass_point_data_[current_pass_.toStdString()] = coords;

    QString coordinates = QString("X: %1, Y: %2, Z: %3").arg(msg->x).arg(msg->y).arg(msg->z);
    if (params_.llm_param.task == "pass"){
        emit UpdateWaypoints(coordinates, current_pass_); // 정확한 슬롯 이름 사용
    }
    current_coordinates_ = *msg;
}

void QNode::LLMPassImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cout << "pass image recived!" << endl;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        cv::Mat mat = cv_ptr->image;

        QImage image = QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_RGB888);
        if (params_.llm_param.task == "pass"){
            emit UpdateImage(image, current_pass_);
        }
    } catch (cv_bridge::Exception& e) {
        std::cerr << "cv_bridge exception: " << e.what() << std::endl;
    }
}


// void QNode::setLLMText(const hanyang_matching_msgs::msg::LlmCmd::SharedPtr msg) {
//     try {
        // params_.llm_param.task = msg->task;
        // params_.llm_param.target = msg->target;
        // params_.llm_param.goal = msg->goal;
        // params_.llm_param.passes = msg->passes;
        // params_.llm_param.velocities = msg->velocities;
        // std::string input_text = msg->input;
        // std::string output_text = msg->output;
        // // std::string input_log = "input: " + input_text + "\n";
        // // std::string output_log = "output: " + output_text + "\n";
        // std::string llm_log = "input: " + input_text + "\n\n" + "output: " + "\n"+ output_text;
        // std_msgs::msg::String request_msg;
        // sendTaskInfoLog(llm_log);

        // if (params_.llm_param.task == "pass" && !params_.llm_param.passes.empty()) {
        //     for (const auto& pass_point : params_.llm_param.passes) {
        //         current_pass_ = QString::fromStdString(pass_point);
        //         // 데이터 요청 메시지 발행
        //         request_msg.data = current_pass_.toStdString();
        //         realsense_publisher_->publish(request_msg);
        //         ROS_LOG_INFO("Requesting data for pass point: %s", current_pass_.toStdString().c_str());
        //     }
        //     // PASS 작업 후 TARGET, GOAL, PASSES 초기화
        //     params_.llm_param.target = "";
        //     params_.llm_param.goal = "";
        //     params_.llm_param.passes.clear();

        // } else if (params_.llm_param.task == "placement") {
        //     ROS_LOG_INFO("Task is placement. Using previously received data for passes.");
        //     if (!params_.llm_param.goal.empty()) {
        //         std::string current_goal = params_.llm_param.goal;
        //         request_msg.data = params_.llm_param.goal;
        //         realsense_publisher_->publish(request_msg);
        //         ROS_LOG_INFO("Requesting data for goal point: %s", current_goal.c_str());
        //     }
        // } else if (params_.llm_param.task == "velocity") {
        //     // 속도 명령이 들어올 때마다 UI에 반영
        //     std::string velocities_str = "";
        //     for (const auto &velocity : params_.llm_param.velocities) {
        //         velocities_str += std::to_string(velocity) + " ";
        //     }
        //     ROS_LOG_INFO("Velocities updated: %s", velocities_str.c_str());
        //     emit UpdateVelocites(QString::fromStdString(velocities_str));

        //     // VELOCITY 작업 후 TARGET, GOAL, PASSES 초기화
        //     params_.llm_param.target = "";
        //     params_.llm_param.goal = "";
        //     params_.llm_param.passes.clear();

        // } else if (params_.llm_param.task == "move") {
        //     ROS_LOG_INFO("Task is movement.");

        // }
        // else {
        //     // sendTaskInfoLog(params_.llm_param.task);
        //     ROS_LOG_WARN("No pass points available.");
        // }

        // ROS_LOG_INFO("task: %s", params_.llm_param.task.c_str());
        // ROS_LOG_INFO("target: %s", params_.llm_param.target.c_str());
        // ROS_LOG_INFO("goal: %s", params_.llm_param.goal.c_str());

        // std::string passes_str = "";
        // for (const auto &pass : params_.llm_param.passes) {
        //     passes_str += pass + " ";
        // }
        // ROS_LOG_INFO("passes: %s", passes_str.c_str());

        // // std::string velocities_str = "";
        // // for (const auto &velocity : params_.llm_param.velocities) {
        // //     velocities_str += std::to_string(velocity) + " ";
        // // }
        // // ROS_LOG_INFO("velocities: %s", velocities_str.c_str());

        // emit UpdateLLMText(QString::fromStdString(params_.llm_param.target),
        //                     QString::fromStdString(params_.llm_param.goal),
        //                     QString::fromStdString(passes_str));

        // params_.llm_param.llm_text_received = true;

//     } catch (const std::exception& e) {
//         ROS_LOG_ERROR("Exception caught in setLLMText: %s", e.what());
//     }
// }


// Cooking robot
void QNode::sendRealsenseRequest(const QString &request) {
    if (!realsense_publisher_) {
        qDebug() << "Error: realsense_publisher_ is null!";
        return;
    }

    auto msg = std_msgs::msg::String();
    msg.data = request.toStdString();
    realsense_publisher_->publish(msg);

    qDebug() << "Sent realsense request:" << request;

}

void QNode::CookGoalPoseCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (msg->data.size() == 6) {
        double x = msg->data[0];
        double y = msg->data[1];
        double z = msg->data[2];
        double roll = msg->data[3];
        double pitch = msg->data[4];
        double yaw = msg->data[5];

        // current_tool_이 설정되었는지 확인
        if (current_tool_.empty()) {
            qDebug() << "Error: current_tool_ is not set!";
            return;
        }

        QString toolName = QString::fromStdString(current_tool_);
        Q_EMIT newPoseReceived(toolName, x, y, z, roll, pitch, yaw);

        qDebug() << "Received pose for tool:" << toolName
                 << "x=" << x << ", y=" << y << ", z=" << z
                 << ", roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw;
    } else {
        qDebug() << "Invalid pose data size: " << msg->data.size();
    }
}


void QNode::CookImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(msg, "rgb8");
        QImage image(cv_image->image.data, cv_image->image.cols, cv_image->image.rows, QImage::Format_RGB888);

        qDebug() << "Received image with size:" << image.size();
        Q_EMIT newImageReceived(image);
    } catch (cv_bridge::Exception &e) {
        qDebug() << "cv_bridge exception:" << e.what();
    }
}


void QNode::LangSAMCoordCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
    if (is_data_request_active_ && msg->data.size() >= 3) {
        double x = msg->data[0];
        double y = msg->data[1];
        double z = msg->data[2];
        qDebug() << "Received LangSAM Coordinates: x=" << x << ", y=" << y << ", z=" << z;

        ROS_LOG_INFO("Received Coordinates: x=%.3f, y=%.3f, z=%.3f", x, y, z);
    // GUI 업데이트
        // GUI에 좌표값 전송
        Q_EMIT newLangSAMCoordinatesReceived(x, y, z);
        // 좌표 요청 완료: 플래그 초기화
        is_data_request_active_ = false;
        qDebug() << "Received coordinates: X=" << x << ", Y=" << y << ", Z=" << z;
    }
}
void QNode::LangSAMImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if (is_image_request_active_) {
        try {
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
            QImage image(cv_ptr->image.data, cv_ptr->image.cols, cv_ptr->image.rows, QImage::Format_RGB888);
            qDebug() << "Received LangSAM Image";
            Q_EMIT newLangSAMImageReceived(image);
            // 이미지 요청 완료: 플래그 초기화
            is_image_request_active_ = false;
        } catch (cv_bridge::Exception& e) {
            qDebug() << "CV Bridge Error: " << e.what();
        }
    }
}
void QNode::sendLangSAMTextPrompt(const std::string& text_prompt) {
    std_msgs::msg::String msg;
    msg.data = text_prompt;
    langsam_request_publisher_->publish(msg);
    qDebug() << "Sent text prompt to LangSAM: " << QString::fromStdString(text_prompt);
}
// #endif

bool QNode::setImpedance(Impedance imped, bool flag) {
    auto req = make_shared<kcr_control_msg::srv::Impedance::Request>();
    arr2Vec(imped.m, req->m_gain);
    arr2Vec(imped.b, req->b_gain);
    arr2Vec(imped.k, req->k_gain);
    arr2Vec(imped.force_limit, req->force_limit);
    arr2Vec(imped.force_des, req->force_des);
    arr2Vec(imped.force_selection, req->force_selection);
    arr2Vec(imped.imped_selection, req->imped_selection);
    arr2Vec(imped.pos_selection, req->pos_selection);
    req->is_tool_frame = imped.is_tool_frame;
    req->flag_impedance = flag;

    auto result = client_set_impedance_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setImpedance");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setImpedance");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setImpedance Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::drflDrlStart(std::string code) {
#if DRFL_CONTROL
    auto req = make_shared<dsr_msgs2::srv::DrlStart::Request>();
    req->robot_system = 0;
    req->code = code;

    auto result = client_drl_start_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(200));
    auto recv = result.get();

    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: drflDrlStart");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : drflDrlStart Error code:" + recv->success);
        return false;
    }
#else
    return true;
#endif
}




bool QNode::drflSetTcp(std::string tcp_name) {
#if DRFL_CONTROL
    auto req = make_shared<dsr_msgs2::srv::SetCurrentTcp::Request>();
    req->name = tcp_name;

    ROS_LOG_INFO("(drflSetTcp)tcp_name: ");
    std::cout << tcp_name << std::endl;

    auto result = client_set_tcp_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(200));
    auto recv = result.get();

    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setTcp");
        return true;
    } else {
        string code = "set_tcp('" + tcp_name + "')";
        auto result_drl_start = drflDrlStart(code);
        if (result_drl_start) {
            return true;
        } else {
            ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
            SEND_ERROR_TO_MONITOR("Function: : drflSetTcp Error code:" + recv->success);
            return false;
        }
    }
#else
    return true;
#endif
}

bool QNode::drflCreateTcp(std::string tcp_name, CsDouble tcp) {
#if DRFL_CONTROL
    auto req = make_shared<dsr_msgs2::srv::ConfigCreateTcp::Request>();
    ROS_LOG_INFO("(setTcp)tcp(mm, deg) now: %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f", tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5]);
    for (int i=0; i<6; i++) {req->pos[i] = tcp[i];}
    ROS_LOG_INFO("(setTcp)tcp(mm, deg) now: %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f", req->pos[0], req->pos[1], req->pos[2], req->pos[3], req->pos[4], req->pos[5]);
    req->name = tcp_name;

    auto result = client_create_tcp_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(200));
    auto recv = result.get();

    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setTcp");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : drflCreateTcp Error code:" + recv->success);
        is_task_mode_ = false;
        return false;
    }
#else
    return true;
#endif
}

bool QNode::setTcp(CsDouble tcp) {
#if DRFL_CONTROL
    // bool success1_ = drflCreateTcp("curr_tcp", tcp);
    // bool success2_ = drflSetTcp("curr_tcp");
    // return success1_ && success2_;
    return true;

#else
    auto req = make_shared<kcr_control_msg::srv::TcpPayload::Request>();
    ROS_LOG_INFO("(setTcp)tcp now: %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f", tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5]);
    arr2Vec(tcp, req->tcp);
    ROS_LOG_INFO("(setTcp)tcp now: %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f", req->tcp[0], req->tcp[1], req->tcp[2], req->tcp[3], req->tcp[4], req->tcp[5]);
    req->flag_tcp     = true;
    req->flag_payload = false;

    auto result = client_set_tcp_payload_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setTcp");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setTcp");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setTcp Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
#endif
}

bool QNode::setPayload(double payload, std::vector<double> com) {
    auto req = make_shared<kcr_control_msg::srv::TcpPayload::Request>();
    req->payload = payload;
    req->com     = com;
    req->flag_tcp     = false;
    req->flag_payload = true;
    req->flag_com     = true;

    auto result = client_set_tcp_payload_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setPayload");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setPayload");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setPayload Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::saveForceData(string index) {
    auto req = make_shared<kcr_control_msg::srv::SingleString::Request>();
    req->command = index;

    auto result = client_save_force_data_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: saveForceData");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: saveForceData");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : saveForceData Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setCustomCode(string script) {
    // auto req = make_shared<dsr_msgs2::srv::DrlStart::Request>();
    // req->robot_system = 0;
    // req->code = "flange_serial_open(baudrate=115200, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)";

    // auto result = client_drl_start_->async_send_request(req);
    // auto status = result.wait_for(chrono::milliseconds(10000));

    // if (status == future_status::timeout) {
    //     ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
    //     SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpCtrlWord");
    //     return false;
    // }

    // auto recv = result.get();

    // if (recv->success) {
    //     ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
    //     //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpCtrlWord");
    //     return true;
    // } else {
    //     ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
    //     SEND_ERROR_TO_MONITOR("Function: : setGrpCtrlWord Error code:" + recv->success);
    //     is_task_mode_ = false;
    //     return false;
    // }
}

#if DRFL_CONTROL
bool QNode::drflGrpConnect(bool flag) {

    auto req = make_shared<dsr_msgs2::srv::DrlStart::Request>();

    if (flag) {     // connect
        req->robot_system = 0;
        req->code = "flange_serial_open(baudrate=115200, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)";
        std::cout << "flange_serial_open" << std::endl;
    } else {        // disconnect
        req->robot_system = 0;
        req->code = "flange_serial_close()";
        std::cout << "flange_serial_close" << std::endl;
    }

    auto result = client_drl_start_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(10000));

    auto recv = result.get();

    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpCtrlWord");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->success);
        SEND_ERROR_TO_MONITOR("Function: : setGrpCtrlWord Error code:" + recv->success);
        is_task_mode_ = false;
        return false;
    }

}

#endif
bool QNode::setGrpEnable(bool enable) {
    auto req = make_shared<grp_control_msg::srv::DriverEnable::Request>();

    req->enable = enable;

    auto result = client_grp_enable_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpEnable");
        return false;
    }

    auto recv = result.get();

    if (recv->successed) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpEnable");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
        SEND_ERROR_TO_MONITOR("Function: : setGrpEnable Error code:" + recv->successed);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setGrpCmd(uint8_t command, int16_t value) {
    auto req = make_shared<grp_control_msg::srv::GripperCommand::Request>();

    req->command = command;
    req->value   = value;

    auto result = client_gripper_cmd_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpCmd");
        return false;
    }

    auto recv = result.get();

    if (recv->successed) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpCmd");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
        SEND_ERROR_TO_MONITOR("Function: : setGrpCmd Error code:" + recv->successed);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setGrpStop(uint16_t duration) {
    auto req = make_shared<grp_control_msg::srv::StopMotor::Request>();

    req->duration = duration;

    auto result = client_stop_motor_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpStop");
        return false;
    }

    auto recv = result.get();

    if (recv->successed) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpStop");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
        SEND_ERROR_TO_MONITOR("Function: : setGrpStop Error code:" + recv->successed);
        is_task_mode_ = false;
        return false;
    }
}

bool QNode::setLed(uint16_t command, uint16_t color) {
    auto req = make_shared<kcr_control_msg::srv::Command::Request>();

    req->command = command;
    req->value   = color;

    auto result = client_led_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setLed");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        // SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setLed");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setLed Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}


// }

bool QNode::drflSetGrp(string script) {

#if DRFL_CONTROL
    auto req = make_shared<dsr_msgs2::srv::DrlStart::Request>();
    req->robot_system = 0;

    if (script[14] == 'o') drflGrpConnect(true);
    else if (script[14] == 'c') drflGrpConnect(false);
    else {

        std::cout << "script:\n" << script << "\n" << std::endl;

        req->code = script;
        auto result = client_drl_start_->async_send_request(req);
        this_thread::sleep_for(std::chrono::milliseconds(2000));
    }
#else
    return false;
#endif

}

bool QNode::setGrp(uint16_t command, int16_t value, uint16_t address) {
#if DRFL_FLANGE_GRP_CONTROL
    auto req = make_shared<dsr_msgs2::srv::DrlStart::Request>();
    req->robot_system = 0;

    switch (command) {
        case (uint16_t)KR_GRP::INIT: { // init
            req->code = "flange_serial_write(b'\\x01\\x06\\x00\\x00\\x00\\x65\\x49\\xE1')";
            auto result = client_drl_start_->async_send_request(req);
            std::cout << "flange_serial_write(b'\\x01\\x06\\x00\\x00\\x00\\x65\\x49\\xE1')" << std::endl;
            // this_thread::sleep_for(std::chrono::milliseconds(2000));
        } break;
        case (uint16_t)KR_GRP::OPEN: {  // open
            req->code = "flange_serial_write(b'\\x01\\x10\\x00\\x00\\x00\\x02\\x04\\x00\\x68\\x03\\xD4\\x72\\xDC')";  // open 980
            auto result = client_drl_start_->async_send_request(req);

            std::cout << "flange_serial_write(b'\\x01\\x10\\x00\\x00\\x00\\x02\\x04\\x00\\x68\\x03\\xD4\\x72\\xDC')" << std::endl;
        } break;
        case (uint16_t)KR_GRP::CLOSE: {  // close
            req->code = "flange_serial_write(b'\\x01\\x10\\x00\\x00\\x00\\x02\\x04\\x00\\x68\\x00\\x0F\\x32\\x77')";  // open 15
            auto result = client_drl_start_->async_send_request(req);
            std::cout << "flange_serial_write(b'\\x01\\x10\\x00\\x00\\x00\\x02\\x04\\x00\\x68\\x00\\x0F\\x32\\x77')" << std::endl;

        } break;
        case (uint16_t)KR_GRP::POS_CTRL: {
            vector<uint16_t> cmd = {0x01, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x68};
            vector<uint16_t> length = dec2hex_vector(value);
            cmd.insert(cmd.end(), length.begin(), length.end());
            std::string crc = dec2hex(calculateCRC(cmd), true);
            std::string code = vec2string(cmd) + crc;
            grpCmd = code;
            req->code = "flange_serial_write(b'" + code + "')";
            auto result = client_drl_start_->async_send_request(req);
        } break;
        default:
            std::cout << "default" << std::endl;
    }
    // this_thread::sleep_for(std::chrono::milliseconds(2000));

#else

#if NEW_VER_KORAS_GRIPPER_PACKAGE
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    //// New version
    auto req = make_shared<grp_control_msg::srv::SingleInt::Request>();
    req->value   = value;

    auto result = new_client_gripper_cmd_->async_send_request(req);


    auto status = result.wait_for(chrono::milliseconds(200));
    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrp");
        return false;
    }
    auto recv = result.get();
    if (recv->successed) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrp");
        if (params_.status.is_gripper_finished == false && is_task_mode_) {
            params_.status.is_gripper_finished = true;
        }

        if(command == (uint16_t)KR_GRP::INIT) {
            is_grp_initialized[grp_driver_address_-1] = true;
        } else if(command == (uint16_t)KR_GRP::POS_CTRL) {
            if(value > 9000) gripper_state = 1;
            else gripper_state = 0;
            ROS_LOG_WARN("Gripper Open Length: %i", value);
            ROS_LOG_INFO("KORAS gripper Pos. ctrl finished");
            SEND_INFO_TO_MONITOR("KORAS gripper Pos. ctrl finished");
        } else if(command == (uint16_t)KR_GRP::OPEN) {
            gripper_state = 0;
            ROS_LOG_INFO("KORAS gripper OPEN finished");
            SEND_INFO_TO_MONITOR("KORAS gripper OPEN finished");
        } else if(command == (uint16_t)KR_GRP::CLOSE) {
            gripper_state = 1;
            ROS_LOG_INFO("KORAS gripper CLOSE finished");
            SEND_INFO_TO_MONITOR("KORAS gripper CLOSE finished");
        } else if(command == (uint16_t)KR_GRP::VACUUM_ON) {
            gripper_state = 1;
            ROS_LOG_INFO("KORAS gripper Vacuum ON");
            SEND_INFO_TO_MONITOR("KORAS gripper Vacuum ON");
        } else if(command == (uint16_t)KR_GRP::VACUUM_OFF) {
            gripper_state = 0;
            ROS_LOG_INFO("KORAS gripper Vacuum OFF");
            SEND_INFO_TO_MONITOR("KORAS gripper Vacuum OFF");
        } else if(command == (uint16_t)KR_GRP::CHANGE_ADDRESS) {
            gripper_state = 0;
            grp_driver_address_ = address;
            ROS_LOG_INFO("KORAS gripper Slave Change (Slave #%i)", grp_driver_address_);
            SEND_INFO_TO_MONITOR("KORAS gripper Slave Change (Slave #" + grp_driver_address_ + std::string(")"));
        }

        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
        SEND_ERROR_TO_MONITOR("Function: : setGrp Error code:" + recv->successed);
        gripper_state = -1;
        is_task_mode_ = false;
        return false;
    }
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////
#else

    /////////////////////////////////////////////////////////////
    // 아래는 예전 그리퍼 버전
    /////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////

    //// 기존
    #if GRIPPER_COMM_RS485
        ROS_LOG_INFO("GRIPPER_COMM_RS485");
        auto req = make_shared<grp_control_msg::srv::GripperCommand::Request>();
        ROS_LOG_INFO("GRIPPER_COMM_RS48522222222222222");

    #elif
        ROS_LOG_INFO("GRIPPER_ECMASTER");
        auto req = make_shared<kcr_control_msg::srv::Command::Request>();
    #endif

        //// TODO: 기존[0(열림)-10000(닫힘)]에서 변경[0(닫힘)-1000(열림)]되었으므로 나중에 이 부분(req->value   = 10000 - value;)은 삭제하도록 처리
        // uint16_t tmp = static_cast<uint16_t>(1000 - value/10);
        //// TODO: 적색 사용중인 건 닫힘 - 열림이 기존과 같음
        // uint16_t tmp = static_cast<uint16_t>(value/10);

        //// 회전판
        uint16_t tmp = value; // 0(열림)~10000(닫힘)

        req->command = command;
        req->value   = tmp;
        req->slave_num = address;
    #if GRIPPER_COMM_RS485
        auto result = client_gripper_cmd_->async_send_request(req);
    #elif
        auto result = client_grp_->async_send_request(req);
    #endif

        auto status = result.wait_for(chrono::milliseconds(200));
        if (status == future_status::timeout) {
            ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
            SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrp");
            return false;
        }
        auto recv = result.get();

    #if GRIPPER_COMM_RS485
        if (recv->successed) {
            ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
            SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrp");
            if (params_.status.is_gripper_finished == false && is_task_mode_) {
                params_.status.is_gripper_finished = true;
            }

            if(command == (uint16_t)KR_GRP::INIT) {
                is_grp_initialized[grp_driver_address_-1] = true;
            } else if(command == (uint16_t)KR_GRP::POS_CTRL) {
                if(value > 9000) gripper_state = 1;
                else gripper_state = 0;
                ROS_LOG_WARN("Gripper Open Length: %i", tmp);
                ROS_LOG_INFO("KORAS gripper Pos. ctrl finished");
                SEND_INFO_TO_MONITOR("KORAS gripper Pos. ctrl finished");
            } else if(command == (uint16_t)KR_GRP::OPEN) {
                gripper_state = 0;
                ROS_LOG_INFO("KORAS gripper OPEN finished");
                SEND_INFO_TO_MONITOR("KORAS gripper OPEN finished");
            } else if(command == (uint16_t)KR_GRP::CLOSE) {
                gripper_state = 1;
                ROS_LOG_INFO("KORAS gripper CLOSE finished");
                SEND_INFO_TO_MONITOR("KORAS gripper CLOSE finished");
            } else if(command == (uint16_t)KR_GRP::VACUUM_ON) {
                gripper_state = 1;
                ROS_LOG_INFO("KORAS gripper Vacuum ON");
                SEND_INFO_TO_MONITOR("KORAS gripper Vacuum ON");
            } else if(command == (uint16_t)KR_GRP::VACUUM_OFF) {
                gripper_state = 0;
                ROS_LOG_INFO("KORAS gripper Vacuum OFF");
                SEND_INFO_TO_MONITOR("KORAS gripper Vacuum OFF");
            } else if(command == (uint16_t)KR_GRP::CHANGE_ADDRESS) {
                gripper_state = 0;
                grp_driver_address_ = address;
                ROS_LOG_INFO("KORAS gripper Slave Change (Slave #%i)", grp_driver_address_);
                SEND_INFO_TO_MONITOR("KORAS gripper Slave Change (Slave #" + grp_driver_address_ + std::string(")"));
            }

            return true;
        } else {
            ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
            SEND_ERROR_TO_MONITOR("Function: : setGrp Error code:" + recv->successed);
            gripper_state = -1;
            is_task_mode_ = false;
            return false;
        }
    #elif
        if (recv->error == 0) {
            ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
            SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrp");
            if (params_.status.is_gripper_finished == false && is_task_mode_) {
                params_.status.is_gripper_finished = true;
            }
            return true;
        } else {
            ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
            SEND_ERROR_TO_MONITOR("Function: : setGrp Error code:" + recv->error);
            is_task_mode_ = false;
            return false;
        }
    #endif
#endif



#endif

}


bool QNode::setGrpMotorCtrl(uint16_t command, int16_t value, uint16_t address) {
#if NEW_VER_KORAS_GRIPPER_PACKAGE

    //// New version
    auto req = make_shared<grp_control_msg::srv::PosVelCurCtrl::Request>();
    req->position = value;
    auto result = new_client_gripper_motor_pos_ctrl_->async_send_request(req);

    auto status = result.wait_for(chrono::milliseconds(200));
    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpMotorCtrl");
        return false;
    }
    auto recv = result.get();
    if (recv->successed) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpMotorCtrl");
        if (params_.status.is_gripper_finished == false && is_task_mode_) {
            params_.status.is_gripper_finished = true;
        }
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
        SEND_ERROR_TO_MONITOR("Function: : setGrpMotorCtrl Error code:" + recv->successed);
        gripper_state = -1;
        is_task_mode_ = false;
        return false;
    }
#endif
}

bool QNode::setGrpMotorEnable() {
#if NEW_VER_KORAS_GRIPPER_PACKAGE

    //// New version
    auto req = make_shared<grp_control_msg::srv::Void::Request>();
    auto result = new_client_gripper_motor_enable_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(200));
    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpMotorEnable");
        return false;
    }
    auto recv = result.get();
    if (recv->successed) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpMotorEnable");
        if (params_.status.is_gripper_finished == false && is_task_mode_) {
            params_.status.is_gripper_finished = true;
        }
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
        SEND_ERROR_TO_MONITOR("Function: : setGrpMotorEnable Error code:" + recv->successed);
        gripper_state = -1;
        is_task_mode_ = false;
        return false;
    }
#endif
}

bool QNode::setGrpMotorResetPose() {
#if NEW_VER_KORAS_GRIPPER_PACKAGE

    //// New version
    auto req = make_shared<grp_control_msg::srv::Void::Request>();
    auto result = new_client_gripper_motor_reset_pose_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(200));
    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpMotorResetPose");
        return false;
    }
    auto recv = result.get();
    if (recv->successed) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpMotorResetPose");
        if (params_.status.is_gripper_finished == false && is_task_mode_) {
            params_.status.is_gripper_finished = true;
        }
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
        SEND_ERROR_TO_MONITOR("Function: : setGrpMotorResetPose Error code:" + recv->successed);
        gripper_state = -1;
        is_task_mode_ = false;
        return false;
    }
#endif
}

bool QNode::setGrpInitialize() {
    #if NEW_VER_KORAS_GRIPPER_PACKAGE
    
        //// New version
        auto req = make_shared<grp_control_msg::srv::Void::Request>();
        auto result = new_client_gripper_cmd_initialize_->async_send_request(req);
        auto status = result.wait_for(chrono::milliseconds(200));
        if (status == future_status::timeout) {
            ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
            SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpInitialize");
            return false;
        }
        auto recv = result.get();
        if (recv->successed) {
            ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
            SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpInitialize");
            if (params_.status.is_gripper_finished == false && is_task_mode_) {
                params_.status.is_gripper_finished = true;
            }
            return true;
        } else {
            ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
            SEND_ERROR_TO_MONITOR("Function: : setGrpInitialize Error code:" + recv->successed);
            gripper_state = -1;
            is_task_mode_ = false;
            return false;
        }
    #endif
}

bool QNode::setGrpOpen() {
    #if NEW_VER_KORAS_GRIPPER_PACKAGE
    
        //// New version
        auto req = make_shared<grp_control_msg::srv::Void::Request>();
        auto result = new_client_gripper_cmd_open_->async_send_request(req);
        auto status = result.wait_for(chrono::milliseconds(200));
        if (status == future_status::timeout) {
            ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
            SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpOpen");
            return false;
        }
        auto recv = result.get();
        if (recv->successed) {
            ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
            SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpOpen");
            if (params_.status.is_gripper_finished == false && is_task_mode_) {
                params_.status.is_gripper_finished = true;
            }
            return true;
        } else {
            ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
            SEND_ERROR_TO_MONITOR("Function: : setGrpOpen Error code:" + recv->successed);
            gripper_state = -1;
            is_task_mode_ = false;
            return false;
        }
    #endif
}


bool QNode::setGrpClose() {
    #if NEW_VER_KORAS_GRIPPER_PACKAGE
    
        //// New version
        auto req = make_shared<grp_control_msg::srv::Void::Request>();
        auto result = new_client_gripper_cmd_close_->async_send_request(req);
        auto status = result.wait_for(chrono::milliseconds(200));
        if (status == future_status::timeout) {
            ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
            SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpClose");
            return false;
        }
        auto recv = result.get();
        if (recv->successed) {
            ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
            SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpClose");
            if (params_.status.is_gripper_finished == false && is_task_mode_) {
                params_.status.is_gripper_finished = true;
            }
            return true;
        } else {
            ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
            SEND_ERROR_TO_MONITOR("Function: : setGrpClose Error code:" + recv->successed);
            gripper_state = -1;
            is_task_mode_ = false;
            return false;
        }
    #endif
}

bool QNode::setGrpVacOn() {
    #if NEW_VER_KORAS_GRIPPER_PACKAGE
    
        //// New version
        auto req = make_shared<grp_control_msg::srv::Void::Request>();
        auto result = new_client_gripper_cmd_vac_on_->async_send_request(req);
        auto status = result.wait_for(chrono::milliseconds(200));
        if (status == future_status::timeout) {
            ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
            SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpVacOn");
            return false;
        }
        auto recv = result.get();
        if (recv->successed) {
            ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
            SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpVacOn");
            if (params_.status.is_gripper_finished == false && is_task_mode_) {
                params_.status.is_gripper_finished = true;
            }
            return true;
        } else {
            ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
            SEND_ERROR_TO_MONITOR("Function: : setGrpVacOn Error code:" + recv->successed);
            gripper_state = -1;
            is_task_mode_ = false;
            return false;
        }
    #endif
}

bool QNode::setGrpVacOff() {
    #if NEW_VER_KORAS_GRIPPER_PACKAGE
    
        //// New version
        auto req = make_shared<grp_control_msg::srv::Void::Request>();
        auto result = new_client_gripper_cmd_vac_off_->async_send_request(req);
        auto status = result.wait_for(chrono::milliseconds(200));
        if (status == future_status::timeout) {
            ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
            SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpVacOff");
            return false;
        }
        auto recv = result.get();
        if (recv->successed) {
            ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
            SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpVacOff");
            if (params_.status.is_gripper_finished == false && is_task_mode_) {
                params_.status.is_gripper_finished = true;
            }
            return true;
        } else {
            ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
            SEND_ERROR_TO_MONITOR("Function: : setGrpVacOff Error code:" + recv->successed);
            gripper_state = -1;
            is_task_mode_ = false;
            return false;
        }
    #endif
}

//// 그리퍼 모드 선택 가능 버전
bool QNode::setGrpGraphy(uint16_t command, uint16_t value, uint16_t address) {

    return true;

    //// TODO: 그래피의 경우, 통합 시 아래 주석 해제하고 예전 그리퍼 패키지를 쓰거나 최신 그리퍼 프로그램으로 변경
    // if(IS_GRIPPER_COMM_RS485_) { // RS485 통신 버전
    //     ROS_LOG_INFO("[%s] GRIPPER_COMM_RS485", __func__);
    //     auto req = make_shared<grp_control_msg::srv::GripperCommand::Request>();

    //     //// TODO: 기존[0(열림)-10000(닫힘)]에서 변경[0(닫힘)-1000(열림)]되었으므로 나중에 이 부분(req->value   = 10000 - value;)은 삭제하도록 처리
    //     uint16_t tmp = static_cast<uint16_t>(1000 - value/10);
    //     //// TODO: 적색 사용중인 건 닫힘 - 열림이 기존과 같음
    //     // uint16_t tmp = static_cast<uint16_t>(value/10);
    //     req->command = command;
    //     // req->value   = value;
    //     req->value   = tmp;
    //     // req->slave_num = address;

    //     auto result = client_gripper_cmd_->async_send_request(req);
    //     // auto status = result.wait_for(chrono::milliseconds(100));
    //     auto status = result.wait_for(chrono::milliseconds(3000));
    //     if (status == future_status::timeout) {
    //         ROS_LOG_ERROR("[timeout] Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
    //         SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrp");
    //         return false;
    //     }
    //     auto recv = result.get();

    //     if (recv->successed) {
    //         ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
    //         //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrp");
    //         if (params_.status.is_gripper_finished == false && is_task_mode_) {
    //             params_.status.is_gripper_finished = true;
    //         }

    //         if(command == (uint16_t)KR_GRP::INIT) {
    //             is_grp_initialized[grp_driver_address_-1] = true;
    //         } else if(command == (uint16_t)KR_GRP::POS_CTRL) {
    //             if(value > 9000) gripper_state = 1;
    //             else gripper_state = 0;
    //             ROS_LOG_WARN("Gripper Open Length: %i", tmp);
    //             // ROS_LOG_WARN("Gripper Open Length: %i", value);
    //             ROS_LOG_INFO("KORAS gripper Pos. ctrl finished");
    //             //SEND_INFO_TO_MONITOR("KORAS gripper Pos. ctrl finished");
    //         } else if(command == (uint16_t)KR_GRP::OPEN) {
    //             gripper_state = 0;
    //             ROS_LOG_INFO("KORAS gripper OPEN finished");
    //             //SEND_INFO_TO_MONITOR("KORAS gripper OPEN finished");
    //         } else if(command == (uint16_t)KR_GRP::CLOSE) {
    //             gripper_state = 1;
    //             ROS_LOG_INFO("KORAS gripper CLOSE finished");
    //             //SEND_INFO_TO_MONITOR("KORAS gripper CLOSE finished");
    //         } else if(command == (uint16_t)KR_GRP::VACUUM_ON) {
    //             gripper_state = 1;
    //             ROS_LOG_INFO("KORAS gripper Vacuum ON");
    //             //SEND_INFO_TO_MONITOR("KORAS gripper Vacuum ON");
    //         } else if(command == (uint16_t)KR_GRP::VACUUM_OFF) {
    //             gripper_state = 0;
    //             ROS_LOG_INFO("KORAS gripper Vacuum OFF");
    //             //SEND_INFO_TO_MONITOR("KORAS gripper Vacuum OFF");
    //         } else if(command == (uint16_t)KR_GRP::CHANGE_ADDRESS) {
    //             gripper_state = 0;
    //             grp_driver_address_ = address;
    //             ROS_LOG_INFO("KORAS gripper Slave Change (Slave #%i)", grp_driver_address_);
    //             //SEND_INFO_TO_MONITOR("KORAS gripper Slave Change (Slave #" + grp_driver_address_ + std::string(")"));
    //         }

    //         return true;
    //     } else {
    //         ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
    //         SEND_ERROR_TO_MONITOR("Function: : setGrp Error code:" + recv->successed);
    //         gripper_state = -1;
    //         is_task_mode_ = false;
    //         return false;
    //     }

    // } else { // FR 직결 버전
    //     ROS_LOG_INFO("[%s] GRIPPER_COMM_FAIRINO", __func__);
    //     auto req = make_shared<kcr_control_msg::srv::Command::Request>();
    //     //// TODO: 기존[0(열림)-10000(닫힘)]에서 변경[0(닫힘)-1000(열림)]되었으므로 나중에 이 부분(req->value   = 10000 - value;)은 삭제하도록 처리
    //     uint16_t tmp = static_cast<uint16_t>(1000 - value/10);
    //     //// TODO: 적색 사용중인 건 닫힘 - 열림이 기존과 같음
    //     // uint16_t tmp = static_cast<uint16_t>(value/10);
    //     req->command = command;
    //     // req->value   = value;
    //     req->value   = tmp;
    //     // req->slave_num = address;
    //     auto result = client_grp_->async_send_request(req);
    //     // auto status = result.wait_for(chrono::milliseconds(100));
    //     auto status = result.wait_for(chrono::milliseconds(3000));
    //     if (status == future_status::timeout) {
    //         ROS_LOG_ERROR("[timeout] Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
    //         SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrp");
    //         return false;
    //     }
    //     auto recv = result.get();

    //     if (recv->error == 0) {
    //         ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
    //         //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrp");
    //         if (params_.status.is_gripper_finished == false && is_task_mode_) {
    //             params_.status.is_gripper_finished = true;
    //         }
    //         return true;
    //     } else {
    //         ROS_LOG_ERROR("[recv->error != 0]Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
    //         SEND_ERROR_TO_MONITOR("Function: : setGrp Error code:" + recv->error);
    //         is_task_mode_ = false;
    //         return false;
    //     }
    // }
}

bool QNode::setGrpCtrlWord(uint16_t grp_ctrl_word) {
    if(is_simul_mode_)
    {
        ROS_LOG_INFO("[Simulation Mode]Function skiped: %s", __FUNCTION__);
        return true;
    }
    auto req = make_shared<kcr_control_msg::srv::Command::Request>();

    req->command = grp_ctrl_word;

    auto result = client_grp_ctrl_word_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: setGrpCtrlWord");
        return false;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: setGrpCtrlWord");
        return true;
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : setGrpCtrlWord Error code:" + recv->error);
        is_task_mode_ = false;
        return false;
    }
}

void QNode::robotStateCallback(const kcr_control_msg::msg::RobotState::SharedPtr msg) {
    if (msg->q_meas.size() != JS_DOF) {
        ROS_LOG_ERROR("JS_DOF size error %s", __FUNCTION__);
        rclcpp::shutdown();
    }

    vec2Arr(msg->q_meas , params_.meas.q);
    vec2Arr(msg->qd_meas, params_.meas.qd);
    vec2Arr(msg->torque_sensor_value, params_.meas.torque_jts);

    vec2Arr(msg->x_meas , params_.meas.x);
    vec2Arr(msg->xd_meas, params_.meas.xd);
    vec2Arr(msg->force_meas, params_.meas.force);
    vec2Arr(msg->test_var_1, params_.meas.force_sensor);
    vec2Arr(msg->test_var_2, params_.meas.force_tool);
    vec2Arr(msg->test_var_3, params_.meas.force_base);
    

    vec2Arr(msg->flag_collision_axis, params_.col.flag);
    params_.col.check_flag = msg->flag_collision;
    vec2Arr(msg->flag_malfunction_axis, params_.malfunction.flag);
    params_.malfunction.status = msg->flag_malfunction;
    vec2Arr(msg->flag_torque_limit_axis, params_.torque_limit.flag);
    params_.torque_limit.status = msg->flag_torque_limit;

    params_.status.is_enable = msg->is_enable;
    params_.status.is_emergency_stop = msg->is_emergency_stop;
    params_.status.is_path_operating = msg->is_path_operating;

    if (msg->driver_ctrl_mode == "CSP") {
        params_.mode.driver_ctrl_mode = DriverCtrlMode::CSP;
    } else if (msg->driver_ctrl_mode == "CSV") {
        params_.mode.driver_ctrl_mode = DriverCtrlMode::CSV;
    } else if (msg->driver_ctrl_mode == "CST") {
        params_.mode.driver_ctrl_mode = DriverCtrlMode::CST;
    }

    if (msg->ik_method == "jacobian") {
        params_.mode.ik_method = InvKineMethod::JACOBIAN;
    } else if (msg->ik_method == "optimization") {
        params_.mode.ik_method = InvKineMethod::OPTIMIZATION;
    } else if (msg->ik_method == "closed_form") {
        params_.mode.ik_method = InvKineMethod::CLOSED_FORM;
    }

    params_.mode.is_collision_detection_mode = msg->is_collision_detection_mode;
    params_.mode.is_collision_demo_mode      = msg->is_collision_demo_mode;
    params_.mode.is_force_ctrl_mode          = msg->is_force_ctrl_mode;
    params_.mode.is_friction_observer_mode   = msg->is_friction_observer_mode;
    params_.mode.is_impedance_ctrl_mode      = msg->is_impedance_ctrl_mode;
    params_.mode.is_teaching_mode            = msg->is_teaching_mode;

    //// For bp
    auto bp_robot_state_msg = hanyang_matching_msgs::msg::RobotState();
    bp_robot_state_msg.actualq = msg->q_meas;
    bp_robot_state_pub_->publish(bp_robot_state_msg);
}

void QNode::rscStateCallback(const kcr_control_msg::msg::RscState::SharedPtr msg) {
    rsc_params_.rsc_state          = msg->rsc_state;
    rsc_params_.rsc_input_state    = msg->rsc_input_state;
    rsc_params_.is_joint_connected = msg->is_joint_connected;
}

void QNode::gripperStateCallback(const grp_control_msg::msg::GripperMsg::SharedPtr msg) {

#if NEW_VER_KORAS_GRIPPER_PACKAGE

    uint16_t motor_current = msg->motor_current;


    //// NOTICE: 기존[0(열림)-10000(닫힘)]
    //// NOTICE: 변경[1000(열림)-0(닫힘)]
    uint16_t tmp = static_cast<uint16_t>(10000 - motor_current*10);
    motor_current = tmp;
    if(motor_current > 10000) motor_current = 10000;
    else if(motor_current < 0) motor_current = 0;

    grp_meas_pos_ = motor_current;


    uint16_t finger_pos = msg->finger_position;
    grp_meas_pos_ = finger_pos;
    // ROS_LOG_WARN("[New KORAS GRP] gripper_position: %u ", grp_meas_pos_);

#else // 예전 버전
    uint16_t grp_pos = msg->grp_pos;


    //// NOTICE: 기존[0(열림)-10000(닫힘)]
    //// NOTICE: 변경[1000(열림)-0(닫힘)]
    uint16_t tmp = static_cast<uint16_t>(10000 - grp_pos*10);
    grp_pos = tmp;
    if(grp_pos > 10000) grp_pos = 10000;
    else if(grp_pos < 0) grp_pos = 0;

    grp_meas_pos_ = grp_pos;
#endif
    // ROS_LOG_WARN("grp_meas_pos_: %u", grp_meas_pos_);

}


bool QNode::xpadJogControl() {
    auto jog_cmd = make_shared<kcr_control_msg::srv::XpadJog::Request>();

    jog_cmd->xd.resize(6, 0);

    // left stick control
    if (fabs(static_cast<double>(joy_stick_axis_[0])) > 0.2) {
        jog_cmd->xd[1] = jog_xpad_trans_ * static_cast<double>(joy_stick_axis_[0]);
    }

    if (fabs(static_cast<double>(joy_stick_axis_[1])) > 0.2) {
        jog_cmd->xd[0] = jog_xpad_trans_ * static_cast<double>(joy_stick_axis_[1]);
    }

    // Rigth stick control
    // if (fabs(static_cast<double>(joy_stick_axis_[3])) > 0.2) {

    // }

    if (fabs(static_cast<double>(joy_stick_axis_[4])) > 0.2) {
        jog_cmd->xd[2] = jog_xpad_trans_ * static_cast<double>(joy_stick_axis_[4]);
    }




    // // z axis control 1st method : LB RB control
    // if (joy_button_[4] == 1 && joy_button_[5] == 0) {
    //     jog_cmd->xd[2] = -jog_xpad_trans_ * joy_button_[4];
    // } else if (joy_button_[4] == 0 && joy_button_[5] == 1) {
    //     jog_cmd->xd[2] =  jog_xpad_trans_ * joy_button_[5];
    // }

    // // z axis control 2nd method : trigger control
    // if (fabs(static_cast<double>((joy_stick_axis_[2]) - 1) * -0.5) > 0.2) {
    //     jog_cmd->xd[2] = jog_xpad_trans_ * static_cast<double>(joy_stick_axis_[2] - 1) * -0.5 * -1;
    // }

    // if (fabs(static_cast<double>((joy_stick_axis_[5]) - 1) * -0.5) > 0.2) {
    //     jog_cmd->xd[2] = jog_xpad_trans_ * static_cast<double>(joy_stick_axis_[5] - 1) * -0.5;
    // }

    // // z axis control 2nd method : Trigger control (threshold)
    // //  if      (joy_stick_axis_[5] > -0.9 && joy_stick_axis_[5] < 0.9)
    // //    jog_cmd->desXdot[2]  = (joy_stick_axis_[5] - 1) * (-0.5) * jog_xpad_trans_;   // right trigger
    // //  else if (joy_stick_axis_[4] > -0.9 && joy_stick_axis_[4] < 0.9)
    // //    jog_cmd->desXdot[2] += (joy_stick_axis_[4] - 1) *  (0.5) * jog_xpad_trans_;   // left  trigger





    auto result = client_xpad_jog_->async_send_request(jog_cmd);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }

    auto recv = result.get();

    if (!recv->successed) {
        ROS_LOG_ERROR("Function: %s\n", __FUNCTION__);
        is_task_mode_ = false;
        return false;
    }

    return true;
}

void QNode::getJoyData(sensor_msgs::msg::Joy::SharedPtr msg) {
    // A, B, X, Y, LB, RB, double-sqare, 3-lines, home, LS, RS
    joy_button_ = msg->buttons;

    // left stick horizon, vertical, left trigger, right stick horizon, vertical, right trigger
    // Cross Left(+1) / Right(-1), Cross Up(+1) / Down(-1)
    joy_stick_axis_ = msg->axes;

    if (joy_button_[0] == 1 && joy_button_[1] == 0) {
        ROS_LOG_WARN("Xpad: button A -> Task pause");
        pauseTask();

    } else if (joy_button_[1] == 1 && joy_button_[0] == 0) {
        ROS_LOG_INFO("Xpad: button B -> Robot STOP!");
        is_task_mode_ = false;
        stopRobot();
    } else if (joy_button_[2] == 1 && joy_button_[3] == 0) {
        ROS_LOG_WARN("Xpad: button X -> Task resume");
        resumeTask();

    } else if (joy_button_[3] == 1 && joy_button_[2] == 0) {
        ROS_LOG_INFO("Xpad: button Y -> No button");
        // setGrpCmd(MB_GRP_POS_CTRL, 10000);
    }
}

// Apriltag related
void QNode::tagRecogCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    // flag_tag_recog_이 true일때만 해당 기능 동작
    if (!flag_tag_recog_.load()) {
        return;
    }

    // Camera Off이면 return
    if(!is_camera_mode_on_) {
        // ROS_LOG_WARN("---tagRecogCallback - Camera OFF");
        return;
    } else {
        // ROS_LOG_WARN("---tagRecogCallback - Camera ON");
    }


    auto tag_info_prev = tag_info_vec_temp_;
    tag_info_vec_temp_.clear();
    uint tag_num = msg->transforms.size();

    // 인식된 여러 태크들 관리
    for (auto i : msg->transforms) {
        double quaternion[4] = {i.transform.rotation.x, i.transform.rotation.y, i.transform.rotation.z, i.transform.rotation.w};
        double rpy[3];

        ModelMatrix::quaternion2rpy(quaternion, rpy);

        TagInfo current_tag_info;

        current_tag_info.tag_name = i.child_frame_id;
        current_tag_info.pose_camera_tag.element_[0] = i.transform.translation.x;
        current_tag_info.pose_camera_tag.element_[1] = i.transform.translation.y;
        current_tag_info.pose_camera_tag.element_[2] = i.transform.translation.z;
        current_tag_info.pose_camera_tag.element_[3] = rpy[0] * kRad2Deg;
        current_tag_info.pose_camera_tag.element_[4] = rpy[1] * kRad2Deg;
        current_tag_info.pose_camera_tag.element_[5] = rpy[2] * kRad2Deg;

        tag_info_vec_temp_.push_back(current_tag_info);
    }

    // 각 태그별로 LPF 적용 (이때 flag_tag_recog_dl true가 됬을 때의 최초값은 LPF없이 바로 적용, initial value 취급)
    static const double lpf_gain = 0.95;

    auto compensateDegFn = [] (double input) {
        while (input > 180) {
            input -= 360;
        }

        while (input < -180) {
            input += 360;
        }

        return input;
    };

    for (uint i = 0; i < tag_num; i++) {
        for (uint j = 0; j < tag_info_prev.size(); j++) {
            if (tag_info_vec_temp_[i].tag_name == tag_info_prev[j].tag_name) {
                for (int k = 0; k < 3; k++) {
                    double temp = tag_info_vec_temp_[i].pose_camera_tag.element_[k];
                    temp = (1 - lpf_gain) * temp + lpf_gain * tag_info_prev[j].pose_camera_tag.element_[k];
                    tag_info_vec_temp_[i].pose_camera_tag.element_[k] = temp;
                }

                for (int k = 3; k < 6; k++) {
                    double temp = tag_info_vec_temp_[i].pose_camera_tag.element_[k];
                    double prev = tag_info_prev[j].pose_camera_tag.element_[k];
                    double diff = temp - tag_info_prev[j].pose_camera_tag.element_[k];

                    temp = (1 - lpf_gain) * (prev + compensateDegFn(diff)) + lpf_gain * prev;
                    tag_info_vec_temp_[i].pose_camera_tag.element_[k] = compensateDegFn(temp);
                }
            }
        }

        tag_info_vec_temp_[i].tr_camera_tag = ModelMatrix::pose2tr(tag_info_vec_temp_[i].pose_camera_tag);
    }

    if (!tag_info_vec_temp_.empty()) {
        tf2_msgs::msg::TFMessage pub_msg;
        geometry_msgs::msg::TransformStamped current_tag_info;

        for (int i = 0; i < tag_info_vec_temp_.size(); i++) {

            current_tag_info.child_frame_id = tag_info_vec_temp_[0].tag_name;
            current_tag_info.transform.translation.x = tag_info_vec_temp_[0].pose_camera_tag.element_[0];
            current_tag_info.transform.translation.y = tag_info_vec_temp_[0].pose_camera_tag.element_[1];
            current_tag_info.transform.translation.z = tag_info_vec_temp_[0].pose_camera_tag.element_[2];
            current_tag_info.transform.rotation.x = tag_info_vec_temp_[0].pose_camera_tag.element_[3];
            current_tag_info.transform.rotation.y = tag_info_vec_temp_[0].pose_camera_tag.element_[4];
            current_tag_info.transform.rotation.z = tag_info_vec_temp_[0].pose_camera_tag.element_[5];
            current_tag_info.transform.rotation.w = 0;

            pub_msg.transforms.push_back(current_tag_info);
        }

        publisher_tag_->publish(pub_msg);
    }
}

bool QNode::saveTagInfo() {
    if (tag_info_vec_temp_.empty()) {
        // 현재 인식된 태그가 없으면 false 반환
        ROS_LOG_ERROR("\"%s\" function failed, there is no tag info to save", __FUNCTION__);
        return false;
    } else {
        // 현재 위치를 기준으로 base frame to tag의 pose/tr 계산
        for (int i = 0; i < tag_info_vec_temp_.size(); i++) {
            ModelMatrix pose_base_ee(6, 1);

            for (int j = 0; j < 3; j++) {
                pose_base_ee.element_[j]     = params_.meas.x[j];
                pose_base_ee.element_[j + 3] = params_.meas.x[j + 3];
            }

            tag_info_vec_temp_[i].tr_base_tag   = ModelMatrix::pose2tr(pose_base_ee)
                                                  * tr_ee_camera_ * tag_info_vec_temp_[i].tr_camera_tag;
            tag_info_vec_temp_[i].pose_base_tag = ModelMatrix::tr2pose(tag_info_vec_temp_[i].tr_base_tag);
        }

        if (tag_info_vec_.empty()) {
            tag_info_vec_ = tag_info_vec_temp_;
        } else {
            for (auto i : tag_info_vec_temp_) {
                bool flag_exist = false;

                for (int j = 0; j < tag_info_vec_.size(); j++) {
                    if (i.tag_name == tag_info_vec_[j].tag_name) {
                        // 만약 기존에 존재한 이름의 태그가 있을 시 갱신
                        tag_info_vec_[j] = i;
                        flag_exist = true;
                    }
                }

                // 만약 기존에 존재한 이름의 태그가 없을 시 추가
                if (!flag_exist) {
                    tag_info_vec_.push_back(i);
                }
            }
        }

        ROS_LOG_INFO("Tag info saved successfully");
        tag_info_vec_temp_.clear();
        return true;
    }
}

bool QNode::setCurrentTag(std::string tag_name) {
    bool flag_exist = false;

    for (auto i : tag_info_vec_) {
        if (i.tag_name == tag_name) {
            if (flag_exist) ROS_LOG_WARN("There is duplicated tag info named %s", tag_name.c_str());

            tag_info_current_ = i;
            flag_exist = true;
            // break;
        }
    }

    // 입력된 태그 이름에 맞는 정보가 있을 시 true 반환, 없을 시 false 반환 및 tag_info_current_의 tag_name을 null으로 변경
    if (flag_exist) {
        ModelMatrix pose_camera_tag(6, 1);
        pose_camera_tag = tag_info_current_.pose_camera_tag;
        ROS_LOG_WARN("\n[Current Tag: %s]\npose_camera_tag: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f\n\n"
                , tag_name.c_str()
                , pose_camera_tag.element_[0], pose_camera_tag.element_[1], pose_camera_tag.element_[2], pose_camera_tag.element_[3], pose_camera_tag.element_[4], pose_camera_tag.element_[5]);

        return true;
    } else {
        ROS_LOG_ERROR("There's no tag info named %s", tag_name.c_str());
        tag_info_current_ = TagInfo();
        return false;
    }
}

bool QNode::setCurrentTagFromJSON(std::string tag_name) {

    try {
        tag_info_current_ = TagInfo();
        tag_info_current_.tag_name = tag_name;

        ////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////
        //// JSON - 저장된 인식 마커 정보 불러오기
        //// FILE PATH
        std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/marker_pose/graphy_marker_info.json";
        //// JSON INPUT
        std::stringstream ss;
        std::ifstream ifs(pose_config_path.c_str());

        json j_in = json::parse(ifs);
        std::vector<double> l_pose_camera_tag = j_in["[Tag Pose]"][tag_name.c_str()]["marker_detection_result"]["pose_camera_tag"].get<std::vector<double>>();
        std::vector<double> l_pose_base_tag = j_in["[Tag Pose]"][tag_name.c_str()]["marker_detection_result"]["pose_base_tag"].get<std::vector<double>>();

        for(size_t i = 0; i < 6; i++) {
            tag_info_current_.pose_camera_tag.element_[i] = l_pose_camera_tag[i];
            tag_info_current_.pose_base_tag.element_[i] = l_pose_base_tag[i];
        }

        tag_info_current_.tr_camera_tag = ModelMatrix::pose2tr(tag_info_current_.pose_camera_tag);
        tag_info_current_.tr_base_tag = ModelMatrix::pose2tr(tag_info_current_.pose_base_tag);
        ////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////

        // 입력된 태그 이름에 맞는 정보가 있을 시 true 반환, 없을 시 false 반환 및 tag_info_current_의 tag_name을 null으로 변경
        ModelMatrix pose_camera_tag(6, 1);
        pose_camera_tag = tag_info_current_.pose_camera_tag;
        ROS_LOG_WARN("\n[setCurrentTagFromJSON]\n[Current Tag: %s]\npose_camera_tag: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f\n\n"
                , tag_name.c_str()
                , pose_camera_tag.element_[0], pose_camera_tag.element_[1], pose_camera_tag.element_[2], pose_camera_tag.element_[3], pose_camera_tag.element_[4], pose_camera_tag.element_[5]);

        return true;
    } catch (const std::exception & e) {
        ROS_LOG_ERROR("[setCurrentTagFromJSON] There's no tag info named %s in JSON file.", tag_name.c_str());
        tag_info_current_ = TagInfo();
        return false;
    }
}

bool QNode::getTagBasedPose(CsDouble input_pose, CsDouble &output_pose) {
    if (tag_info_current_.tag_name == "null") {
        ROS_LOG_WARN("Set current tag first");
        return false;
    }

    std::vector<double> input_pose_vec;
    arr2Vec(input_pose, input_pose_vec);

    ModelMatrix pose_base_target(6, 1, input_pose_vec), pose_tag_target;

    pose_tag_target = ModelMatrix::tr2pose(ModelMatrix::invTrMat(tag_info_current_.tr_base_tag)
                                           * ModelMatrix::pose2tr(pose_base_target));

    for (int i = 0; i < CS_DOF; i++) {
        output_pose[i] = pose_tag_target.element_[i];
    }

    return true;
}

bool QNode::getTagBasedPose(CsDouble input_pose, CsDouble &output_pose, std::string tag_name) {
    bool flag_exist = false;
    TagInfo tag_temp;

    for (auto i : tag_info_vec_) {
        if (i.tag_name == tag_name) {
            if (flag_exist) {
                ROS_LOG_WARN("There is duplicated tag info named %s", tag_name.c_str());
            }

            tag_temp = i;
            flag_exist = true;
            // break;
        }
    }

    // 입력된 태그 이름에 맞는 정보가 있을 시 true 반환, 없을 시 false 반환 및 tag_info_current_의 tag_name을 null으로 변경
    if (!flag_exist) {
        ROS_LOG_ERROR("There's no tag info named %s", tag_name.c_str());
        return false;
    }

    std::vector<double> input_pose_vec;
    arr2Vec(input_pose, input_pose_vec);

    ModelMatrix pose_base_target(6, 1, input_pose_vec), pose_tag_target;

    pose_tag_target = ModelMatrix::tr2pose(ModelMatrix::invTrMat(tag_temp.tr_base_tag)
                                           * ModelMatrix::pose2tr(pose_base_target));

    for (int i = 0; i < CS_DOF; i++) {
        output_pose[i] = pose_tag_target.element_[i];
    }

    return true;
}

MoveErrorType QNode::moveX_tagBased(CsDouble pose_tag_target, double xd_target, double xdd_target, double q_redundant) {
#if DRFL_CONTROL
#else
    if (tag_info_current_.tag_name == "null") {
        return MoveErrorType::ENABLE_FIRST;
    }

    std::vector<double> pose_target_vec;
    arr2Vec(pose_tag_target, pose_target_vec);

    ModelMatrix pose_tag_target_mat(6, 1, pose_target_vec), pose_base_target;

    pose_base_target = ModelMatrix::tr2pose(tag_info_current_.tr_base_tag * ModelMatrix::pose2tr(pose_tag_target_mat));

    auto req = make_shared<kcr_control_msg::srv::MoveX::Request>();
    req->x_target.resize(CS_DOF);

    for (int i = 0; i < CS_DOF; i++) {
        req->x_target[i] = pose_base_target.element_[i];
    }

    req->xd_target     = xd_target;
    req->xdd_target    = xdd_target;
    req->is_base_frame = true;
    req->is_relative   = false;
    req->q_redundant   = q_redundant;

    auto result = client_move_x_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        SEND_ERROR_TO_MONITOR("Failed to call service\n\tFunction: moveX");
        return MoveErrorType::TIME_OUT;
    }

    auto recv = result.get();

    if (recv->error == 0) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        //SEND_INFO_TO_MONITOR("Function successed\n\tFunction: moveX");
    } else if (recv->error == (uint)MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
        ROS_LOG_WARN("Function: %s\n\tFailed by collision", __FUNCTION__);
        SEND_WARN_TO_MONITOR("Function: moveX Failed by collision");
    } else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->error);
        SEND_ERROR_TO_MONITOR("Function: : moveX Error code:" + recv->error);
        is_task_mode_ = false;
    }

    return (MoveErrorType) recv->error;
#endif
}

bool QNode::sendUNIZData(std::vector<std::vector<std::vector<double>>> &bbox_set) {
    auto req = make_shared<hanyang_matching_msgs::srv::DoPclTest::Request>();



    std::vector<double> vec_bounding_box;
    for(size_t i=0; i<bbox_set.size(); i++) {
        // std::cout << "[sendUNIZData] Aligner #" << i + 1 << " Start!" << std::endl;
        for(size_t j=0; j<bbox_set[i].size(); j++) {
            // std::cout << "P" << j+1 << ": " << bbox_set[i][j][0] << ", " << bbox_set[i][j][1] << ", "  << bbox_set[i][j][2] << std::endl;
            for(size_t k=0; k<3; k++) {
                vec_bounding_box.push_back(bbox_set[i][j][k]);
            }
        }
        // std::cout << "[sendUNIZData] Aligner #" << i + 1 << " End!" << std::endl;
    }


    // for(size_t i=0; i<vec_bounding_box.size(); i++) {
    //     std::cout << "Value " << i + 1 << ": " << vec_bounding_box[i]  << std::endl;
    // }


    req->bounding_box = vec_bounding_box;

    ROS_LOG_INFO("sendUNIZData!");
    while (!graphySendUNIZClient_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }

    auto result = graphySendUNIZClient_->async_send_request(req);


    auto status = result.wait_for(chrono::milliseconds(50000));
    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }

    auto recv = result.get();

    if (recv->is_pose) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);

        std::vector<std::vector<double>> uniz_aligner_pose_set_wrt_top_right_corner;
        std::vector<double> l_uniz_pose_wrt_top_right_corner;
        l_uniz_pose_wrt_top_right_corner = recv->output_pose_set;
        size_t l_aligner_cnt = bbox_set.size();
        for(size_t i = 0; i < l_aligner_cnt; i++) { // aligner 개수
            std::vector<double> l_pose_tmp_recv(6, 0.0);
            for(size_t j = 0; j < 6; j++) {
                l_pose_tmp_recv[j] = l_uniz_pose_wrt_top_right_corner[i * 6 + j];
            }
            uniz_aligner_pose_set_wrt_top_right_corner.push_back(l_pose_tmp_recv);
        }


        ///////////////////////////////////////////
        ///////////////////////////////////////////
        ///////////////////////////////////////////
        //// JSON 저장

        //// JSON Save
        //// FILE PATH
        //// JSON INPUT
        // std::string pose_config_path = "/home/bp/[Pose server]/teaching_pose/graphy_demo_cs_waypoint.json";
        std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/teaching_pose/graphy_demo_cs_waypoint.json";
        std::ifstream ifs(pose_config_path.c_str());
        json j_in = json::parse(ifs);
        json j_out = j_in;
        std::vector<double> pose_update(6, 0.0);
        // std::stringstream ss;
        // ss.str("");

        std::vector<std::vector<double>> uniz_aligner_pose_set_wrt_top_right_corner_sorted(8);

        ////
        ROS_LOG_WARN("l_aligner_cnt: %zu", l_aligner_cnt);
        std::vector<size_t> idx_uniz_aligner(l_aligner_cnt);

        for(size_t i = 0; i < l_aligner_cnt; i++) { // aligner 개수
            std::vector<double> l_pose_tmp = uniz_aligner_pose_set_wrt_top_right_corner[i];
            // ROS_LOG_WARN("UNIZ Aligner pose(w.r.t. top-right corner): %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f", l_pose_tmp[0], l_pose_tmp[1], l_pose_tmp[2], l_pose_tmp[3], l_pose_tmp[4], l_pose_tmp[5]);

            for(int j = 0; j < 3; j++) {
                pose_update[j] = round(0.001*l_pose_tmp[j]*1e7) / 1e7; // [m]
                pose_update[j + 3] = round(l_pose_tmp[j + 3]*1e4) / 1e4; // [deg]
            }

            //// NOTICE: 완전히 인덱스를 6개로 변경 (250319)
            std::string l_aligner_pose_idx;
            switch(i) {
                case 0: {
                    l_aligner_pose_idx = "Pose #1";
                    uniz_aligner_pose_set_wrt_top_right_corner_sorted[0] = l_pose_tmp;
                    idx_uniz_aligner[i] = 0;
                    break;
                }
                case 1: {
                    l_aligner_pose_idx = "Pose #2";
                    uniz_aligner_pose_set_wrt_top_right_corner_sorted[1] = l_pose_tmp;
                    idx_uniz_aligner[i] = 1;
                    break;
                }
                case 2: {
                    // l_aligner_pose_idx = "Pose #4";
                    l_aligner_pose_idx = "Pose #3";
                    uniz_aligner_pose_set_wrt_top_right_corner_sorted[3] = l_pose_tmp;
                    idx_uniz_aligner[i] = 3;
                    break;
                }
                case 3: {
                    // l_aligner_pose_idx = "Pose #5";
                    l_aligner_pose_idx = "Pose #4";
                    uniz_aligner_pose_set_wrt_top_right_corner_sorted[4] = l_pose_tmp;
                    idx_uniz_aligner[i] = 4;
                    break;
                }
                case 4: {
                    // l_aligner_pose_idx = "Pose #6";
                    l_aligner_pose_idx = "Pose #5";
                    uniz_aligner_pose_set_wrt_top_right_corner_sorted[5] = l_pose_tmp;
                    idx_uniz_aligner[i] = 5;
                    break;
                }
                case 5: {
                    // l_aligner_pose_idx = "Pose #7";
                    l_aligner_pose_idx = "Pose #6";
                    uniz_aligner_pose_set_wrt_top_right_corner_sorted[6] = l_pose_tmp;
                    idx_uniz_aligner[i] = 6;
                    break;
                }
                //// NOTICE: 6개 기준 - 3번, 8번 인덱스는 제외
                //// NOTICE: 6개 기준 - 3번, 8번 인덱스는 제외
                //// NOTICE: 6개 기준 - 3번, 8번 인덱스는 제외
                //// NOTICE: 6개 기준 - 3번, 8번 인덱스는 제외
                //// NOTICE: 6개 기준 - 3번, 8번 인덱스는 제외
                case 6: {
                    l_aligner_pose_idx = "Pose #3";
                    uniz_aligner_pose_set_wrt_top_right_corner_sorted[2] = l_pose_tmp;
                    break;
                }
                case 7: {
                    l_aligner_pose_idx = "Pose #8";
                    uniz_aligner_pose_set_wrt_top_right_corner_sorted[7] = l_pose_tmp;
                    break;
                }
            }


            j_out["[Task B-2]_UNIZ"][l_aligner_pose_idx] = pose_update;
        }

        //// NOTICE: 6개 기준 - 3번, 8번 인덱스는 제외
        for(size_t i = 0; i < l_aligner_cnt; i++) { // aligner 개수
            std::vector<double> l_pose_tmp = uniz_aligner_pose_set_wrt_top_right_corner_sorted[idx_uniz_aligner[i]];
            ROS_LOG_WARN("[Sorted]UNIZ Aligner pose(w.r.t. top-right corner): %0.1f, %0.1f, %0.1f, %0.1f, %0.1f, %0.1f", l_pose_tmp[0], l_pose_tmp[1], l_pose_tmp[2], l_pose_tmp[3], l_pose_tmp[4], l_pose_tmp[5]);
        }

        std::ofstream ofs(pose_config_path.c_str());
        ofs << j_out.dump(4) << std::endl;
        // ROS_LOG_WARN("SAVING POSE... [%s - %s]\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());

        ////

        ///////////////////////////////////////////
        ///////////////////////////////////////////
        ///////////////////////////////////////////



        //// UNIZ FLAG
        is_uniz_file_loaded_ = true;
        ROS_LOG_WARN("UNIZ DATA LOADING FINISHED!");
        return true;
    } else {
        ROS_LOG_ERROR("Pose infeasible!!!\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}



bool QNode::sendRelayCommand(const std::string cmd) {
    auto req = make_shared<hanyang_matching_msgs::srv::SendCommand::Request>();

    req->command = cmd;
    ROS_LOG_INFO("sendRelayCommand!");
    while (!client_relay_command_->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
            ROS_LOG_INFO("Interrupted while waiting for the service. Exiting.");
            return false;
        }
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }

    auto result = client_relay_command_->async_send_request(req);

    auto status = result.wait_for(chrono::milliseconds(3000));
    if (status == future_status::timeout) {
        ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }

    auto recv = result.get();

    if (recv->success) {
        ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
        ROS_LOG_WARN("RELAY CTRL FINISHED!");
        return true;
    } else {
        ROS_LOG_ERROR("Relay ctrl failed!\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
        return false;
    }
}


// Task operation related
void QNode::pauseTask() {
    lock_guard<mutex> lg(mutex_);

    auto insertPauseTask = [&] (vector<UnitTask> &current_task_list, TaskParameter &task_param) {
        vector<UnitTask> taskList = current_task_list;

        UnitTask unit_task_pause = task_planner_->ref_unit_task_;

        unit_task_pause.task_mode = TASK_PAUSE;

        if (task_param.task_mode == WAIT_TIL_FIN_ROBOTMOVE || task_param.task_mode == WAIT_TIL_FIN_ROBOTMOVE_500MS_DELAY
                || task_param.task_mode == WAIT_TIL_FIN_ASSEMBLY || task_param.task_mode == WAIT_TIL_FIN_COOPMOVE) {
            // Regenerate tasklist
            current_task_list.clear();
            for (uint i = 0; i <= task_param.task_step; i++) {
                current_task_list.push_back(taskList[i]);
            }

            current_task_list.push_back(unit_task_pause);

            for (uint i = task_param.task_step + 1; i < task_param.task_end_num; i++) {
                current_task_list.push_back(taskList[i]);
            }

            task_param.task_end_num += 1;

            // Skip wait phase
            task_param.is_unit_task_fin = true;
            task_param.time_10ms = 10;
            task_param.task_mode = TASK_DEFAULT;
        } else {
            // regen tasklist
            current_task_list.clear();
            for (uint i = 0; i <= task_param.task_step; i++)
                current_task_list.push_back(taskList[i]);

            current_task_list.push_back(unit_task_pause);

            for (uint i = task_param.task_step + 1; i < task_param.task_end_num; i++)
                current_task_list.push_back(taskList[i]);

            task_param.task_end_num += 1;
        }

        task_param.is_pause_status = true;
        ROS_LOG_WARN("Pause Task !!");
        SEND_WARN_TO_MONITOR("Pause Task !!");
    };

    if (is_task_mode_) {
        insertPauseTask(current_task_list_, task_param_);
    }
}

void QNode::resumeTask() {
    lock_guard<mutex> lg(mutex_);

    task_param_.is_pause_status = false;
    SEND_WARN_TO_MONITOR("Task Resumed");
}

void QNode::beforeTaskStart() {
    // impedance off
    // vector<double> nf          = task_planner_->impedance_default_.nf;
    // vector<double> zeta        = task_planner_->impedance_default_.zeta;
    // vector<double> stiffness   = task_planner_->impedance_default_.stiffness;
    // vector<double> force_limit = task_planner_->impedance_default_.force_limit;

    vector<double> nf          = {5, 5, 5, 5, 5, 5};
    vector<double> zeta        = {10, 10, 10, 10, 10, 10};
    vector<double> stiffness   = {500, 500, 1500, 1, 1, 1};
    vector<double> force_limit = {60, 60, 60, 20, 20, 20};

    // setImpedanceControl(false, stiffness, nf, zeta, force_limit, ROBOT_A);
    // setTCP({0, 0, 0, 0, 0, 0});
    msleep(100);

    // ROS_LOG_WARN("[%s] THIS 1", __func__);

    current_task_list_origin_ = current_task_list_;

    task_param_.task_step = 0;

    task_param_.is_unit_task_fin = false;
    task_param_.is_pause_status  = false;

    task_param_.is_robot_move_fin  = false; // TODO

    task_param_.is_ready_matching = false;


    params_.status.is_path_finished = false;
    params_.status.is_gripper_finished = false;
    params_.status.is_detection_finished = false;

    tip_changing_task_flag_ = false; //// For tip changing
    do_skip_task = false;
    is_teaching_finished_ = false;
    is_task_finished_ = false;
    is_task_robot_disable_and_enable = false;

    //// Marker
    is_tag_detecting_in_progress_ = false;
    is_tag_received_ = false;
    is_marker_detection_converged_ = false;
    is_marker_detection_approached_ = false;
    tag_detection_delay_ = 0;

    // ROS_LOG_WARN("[%s] THIS 2", __func__);


#if BIN_PICKING_FLAG
    m_bin_picking_node->is_grasping_pose_assigned = false;
    m_bin_picking_node->is_zig_pose_assigned = false;

    if(m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.size() != 0) {
        m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.do_tip_changing_for_grasping = false;
    }
#endif
    // ROS_LOG_WARN("[%s] THIS 3", __func__);

    is_parallel_task_mode_ = false;
    task_param_.task_end_num = current_task_list_.size();
    task_param_.task_mode    = current_task_list_.at(0).task_mode;
    is_task_mode_ = true;

    // ROS_LOG_WARN("[%s] THIS 4", __func__);
    if(!is_developer_mode_) {
        Q_EMIT beforeTaskSignal();
    }
    // ROS_LOG_WARN("[%s] THIS 5", __func__);
}

void QNode::loadTaskList() {
    // current_task_list_.clear();
    // current_task_list_ = saved_task_list_;

    // task_param_ = saved_param_;

    // is_coop_move_fin_  = false;

    // is_parallel_task_mode_ = false;

    // is_task_mode_ = true;
}

bool QNode::getOperatingStatus(const shared_ptr<kcr_control_msg::srv::OperatingStatus::Request> req,
                               shared_ptr<kcr_control_msg::srv::OperatingStatus::Response> res) {
    if (req->is_path_finish) {
        if (params_.status.is_path_finished == false
                && (task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE ||
                task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE_500MS_DELAY ||
                task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE_WITH_SCAN_AND_MATCHING ||
				task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE_LLM ||
                task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_APPROACHING ||
                task_param_.task_mode == WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_CONVERGING)) {
            params_.status.is_path_finished = true;

            std::cout << "getOperationgStatus" << std::endl;
            std::cout << "params_.status.is_path_finished" << params_.status.is_path_finished << std::endl;
        }

        res->flag = true;
        ROS_LOG_INFO("Path finished");
    }
    if (req->is_assembly_finish) {
        if (task_param_.task_mode == WAIT_TIL_FIN_ASSEMBLY) {
            // params_.status.is_assembly_finished = true;
        }

        res->flag = true;
        ROS_LOG_INFO("Assembly finished");
    }
    return true;
}

void QNode::doControl() {
    lock_guard<mutex> lg(mutex_);

    // if (!is_task_mode_ && is_jog_mode_) {
    if (is_jog_mode_) {
        if (jog_space_ == XPAD_JOG) {
            xpadJogControl();
        }
    }

    if (!is_task_mode_) {
        return;
    }

    if (is_task_mode_ == true) {

#if BIN_PICKING_FLAG
        if(!SW_MODE_COOKING) {
            //////////////////////////////////////////////////////////////////////////////////////////////////
            //// NOTICE: 작업 도중 팁체인징 수행을 위해 추가한 부분, 특정 조건에서는 작업을 수행하지 않고 그냥 넘어가도록 처리
            //// Flag가 ON이면서, 해당 작업 조건이 false이면 정의된 task skip
            if(tip_changing_task_flag_ && !m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.do_tip_changing_for_grasping) {
                if(current_task_list_.at(task_param_.task_step).task_mode != TASK_SET_TIP_CHANGING_FLAG) {
                    ROS_LOG_WARN("Skip Task!!! - No need to change tip");
                    task_param_.is_unit_task_fin  = true;
                    task_param_.time_10ms = 0;
                    task_param_.task_mode = TASK_DEFAULT;
                    do_skip_task = true;
                } else {
                    do_skip_task = false;
                }
            }
        }
        //////////////////////////////////////////////////////////////////////////////////////////////////
#endif

        ////////////////////////////
        //// Main Task Execution
        if(!do_skip_task) {
            // Torque guard check before executing task step
            if (torque_guard_enabled_) {
                // 토크 데이터는 mainwindow의 timerCallback에서 1초마다 자동 갱신됨
                // (torque_polling_enabled_가 true일 때만)

                // 갱신된 값으로 임계 비교
                bool over_threshold = false;
                double monitored_value = 0.0;
                if (torque_guard_joint_index_ >= 0 && torque_guard_joint_index_ < (int)params_.meas.torque_jts.size()) {
                    monitored_value = params_.meas.torque_jts[torque_guard_joint_index_];
                    over_threshold = std::fabs(monitored_value) >= torque_guard_threshold_;
                } else {
                    // use L-infinity norm (max abs) over all joints by default
                    double max_abs = 0.0;
                    for (double v : params_.meas.torque_jts) max_abs = std::max(max_abs, std::fabs(v));
                    monitored_value = max_abs;
                    over_threshold = max_abs >= torque_guard_threshold_;
                }

                if (over_threshold) {
                    ROS_LOG_WARN("[TORQUE GUARD] Threshold reached. Jump to label if exists. thr=%.3f, val=%.3f, joint=%d",
                                  torque_guard_threshold_, monitored_value, torque_guard_joint_index_);
                    
                    // 임계값 초과 시 토크 폴링 먼저 OFF
                    if (torque_polling_enabled_) {
                        torque_polling_enabled_ = false;
                        ROS_LOG_INFO("[TORQUE POLLING] DISABLED due to threshold exceeded");
                    }
                    
                    // 토크 가드도 OFF (무한 루프 방지)
                    torque_guard_enabled_ = false;
                    ROS_LOG_INFO("[TORQUE GUARD] DISABLED to prevent infinite loop after jump");
                    
#if DRFL_CONTROL
                    drflMoveStop(2);
#else
                    stopRobot();
#endif
                    // 현재 리스트에서 "SCREW_OPEN_STEP" 라벨을 찾아 그 다음 스텝으로 점프
                    ROS_LOG_WARN("SCREW_OPEN_STEP_LOG");

                    const std::string jump_label = "SCREW_OPEN_STEP";
                    bool found = false;
                    for (size_t i = 0; i < current_task_list_.size(); ++i) {
                        if (current_task_list_[i].task_mode == TASK_LABEL && current_task_list_[i].label == jump_label) {
                            task_param_.task_step = static_cast<uint>(i);
                            task_param_.is_unit_task_fin  = true;
                            task_param_.time_10ms = 0;
                            task_param_.task_mode = TASK_DEFAULT;
                            found = true;
                            break;
                        }
                    }
                    if (!found) {
                        // 라벨이 없으면 기존 방식대로 다음 스텝
                        task_param_.is_unit_task_fin  = true;
                        task_param_.time_10ms = 0;
                        task_param_.task_mode = TASK_DEFAULT;
                    }
                    return;
                }
            }
            doTask(current_task_list_, task_param_);
        }
        ////////////////////////////

        // robotA task
        if (task_param_.is_unit_task_fin == true) {
            if (task_param_.time_10ms == 0) {
                task_param_.task_step++;
                task_param_.is_unit_task_fin = false;

                //if(!is_developer_mode_) {
                    Q_EMIT taskFinLogger();
                //}

                if (task_param_.task_step >= task_param_.task_end_num) {
                    task_param_.task_mode = TASK_DEFAULT;
                    task_param_.task_step = 0;
                    task_param_.is_robot_move_fin = true;
                    is_task_mode_ = false;
                    is_task_finished_ = true;
                    printf("************************************************************************ \n");
                    printf("******************--- Current Task is finished!!! ---******************* \n");
                    printf("************************************************************************ \n");
                    //if(!is_developer_mode_) {
                        Q_EMIT taskEndLogger();
                    //}
                }
                else task_param_.task_mode = current_task_list_.at(task_param_.task_step).task_mode;
            }
            else task_param_.time_10ms--;
        }
    } else if (is_simul_mode_) {

    }
}

void QNode::doTask(vector<UnitTask> &current_task_list, TaskParameter &task_param) {
    auto taskFinFunction = [&] (int delayTime) {
        task_param.is_unit_task_fin = true;
        task_param.time_10ms     = delayTime;
        task_param.task_mode      = TASK_DEFAULT;
    };

    switch (task_param.task_mode) {
        case (TASK_DEFAULT): {

        } break;

        case (TASK_TIC): {
            clock_gettime(CLOCK_MONOTONIC, &task_start_time_);
            taskFinFunction(0);
        } break;

        case (TASK_TOC): {
            static struct timespec current_time;

            clock_gettime(CLOCK_MONOTONIC, &current_time);

            double dt = (current_time.tv_sec - task_start_time_.tv_sec) + ((current_time.tv_nsec - task_start_time_.tv_nsec) * 0.000000001);

            cout << "\n======================================================" << endl;
            cout << "Elapsed time during this task : " << dt << "[sec]" << endl;
            cout << "======================================================" << endl;

            taskFinFunction(0);
        } break;

        case (TASK_DELAY): {
            // ROS_LOG_INFO("Time delay : %dms", current_task_list[task_param.task_step].delay_10ms * 10);
            if (current_task_list[task_param.task_step].delay_10ms > 0){
                taskFinFunction(current_task_list[task_param.task_step].delay_10ms);
            } else {
                // is_task_mode_ = false;
                // // ROS_LOG_ERROR("%s Delay time is 0 or negative : %d", prefixString.c_str(), current_task_list[task_param.task_step].delay_10ms);
                // break;
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
        } break;

        case (TASK_DELAY_FOR_TEACHING): {

            if(!is_task_teaching_mode_) { // 티칭 모드인 경우에만 실행됨.
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            if (current_task_list[task_param.task_step].delay_10ms > 0){
                taskFinFunction(current_task_list[task_param.task_step].delay_10ms);
            } else {
                // is_task_mode_ = false;
                // // ROS_LOG_ERROR("%s Delay time is 0 or negative : %d", prefixString.c_str(), current_task_list[task_param.task_step].delay_10ms);
                // break;
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
        } break;

        case (TASK_GRIPPER_CHECK_DELAY_BETWEEN_TASKS): {
            if (current_task_list[task_param.task_step].time_10ms_between_tasks > 0){
                ROS_LOG_WARN("[TASK_GRIPPER_CHECK_DELAY_BETWEEN_TASKS] delay... %u", current_task_list[task_param.task_step].time_10ms_between_tasks);
                current_task_list[task_param.task_step].time_10ms_between_tasks--;
            } else {
                ROS_LOG_WARN("[TASK_GRIPPER_CHECK_DELAY_BETWEEN_TASKS] delay Finished!!! %u", current_task_list[task_param.task_step].time_10ms_between_tasks);
                task_param.task_mode = TASK_RECOG_KORAS_GRIPPER_CHECK_GRASPING; // Delay 후 다시 체크
            }
        } break;

        case (TASK_PAUSE): {
            ROS_LOG_WARN("[TASK] Pause, click the task resume button");
            //Q_EMIT pauseLogger();
            task_param.is_pause_status = true;
            task_param.task_mode = TASK_WAIT_RESUME;
        } break;

        case (TASK_WAIT_RESUME): {
            if (!task_param.is_pause_status) {
                ROS_LOG_WARN("[TASK] Resume taskList");
                taskFinFunction(0);
            }
        } break;

        case (TASK_REWIND): {
            current_task_list_ = current_task_list_origin_;
            beforeTaskStart();
            RCLCPP_WARN(rclcpp::get_logger(log_prefix), "Task cycle: %d end", ++task_cycle_);
        }break;

        case (TASK_CSMOVE_REDUNDANT): {
            CsDouble x_target = current_task_list[task_param.task_step].x_target;

            double vel         = current_task_list[task_param.task_step].vel_cs;
            double acc         = current_task_list[task_param.task_step].acc_cs;
            bool   relative    = current_task_list[task_param.task_step].relative;
            double q_redundant = current_task_list[task_param.task_step].q_redundant;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, true, relative, q_redundant);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                task_param.task_mode = TASK_CSMOVE_REDUNDANT;
                sleep(2);
            }
        } break;

        case (TASK_CSMOVE): {
            CsDouble x_target = current_task_list[task_param.task_step].x_target;

            double vel      = current_task_list[task_param.task_step].vel_cs;
            double acc      = current_task_list[task_param.task_step].acc_cs;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, true, relative);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                task_param.task_mode = TASK_CSMOVE;
                sleep(2);
            }
// #if DRFL_CONTROL
//             taskFinFunction(0);
// #endif
        } break;

        case(TASK_LLM_START): {
            is_llm_task_fin = false;
            std::cout << "TASK_LLM_START" << endl;
            std::cout << "TASK_LLM_START" << endl;
            std::cout << "TASK_LLM_START" << endl;
            taskFinFunction(0);
        }break;

        case(TASK_LLM_FIN): {
            is_llm_task_fin = true;
            taskFinFunction(0);
        }break;

        case (TASK_CSMOVE_LLM): {
            CsDouble x_target = params_.llm_param.x;

            double vel      = 0.1;
            double acc      = 0.2;
            bool   relative = true;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE_LLM;
            MoveErrorType err_type = moveX(x_target, vel, acc, true, relative);

            // if (task_param.is_llm_mode )
        } break;

        case (TASK_JSMOVE): {
            JsDouble q_target = current_task_list[task_param.task_step].q_target;

            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveQ(q_target, vel, acc, relative);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                task_param.task_mode = TASK_JSMOVE;
                sleep(2);
            }
// #if DRFL_CONTROL
//             taskFinFunction(0);
// #endif
        } break;

        case (TASK_JSMOVE_ONLY_J6): {
            JsDouble q_target = params_.meas.q;
            JsDouble target_abs_angle = current_task_list[task_param.task_step].q_target;
            //// Only J6 motion to avoid J6 joint limit
            q_target[5] = target_abs_angle[5];
            //// Only J6 motion
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = false;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveQ(q_target, vel, acc, relative);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                task_param.task_mode = TASK_JSMOVE_ONLY_J6;
                sleep(2);
            }
        } break;

        case (TASK_JSMOVE2): {
            JsDouble q_target = current_task_list[task_param.task_step].q_target;

            double vel      = current_task_list[task_param.task_step].vel_js_custom;
            double acc      = current_task_list[task_param.task_step].acc_js_custom;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveQ(q_target, vel, acc, relative);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                task_param.task_mode = TASK_JSMOVE;
                sleep(2);
            }
// #if DRFL_CONTROL
//             taskFinFunction(0);
// #endif
        } break;

        case (TASK_JSMOVE2_KEYCODE_J6_ONLY): {
            // q_target는 상대 모드 가정, J6만 drum_rotation_angle_ 반영
            JsDouble q_target = {0, 0, 0, 0, 0, 0};
            q_target[5] = drum_rotation_angle_;
            double vel      = current_task_list[task_param.task_step].vel_js_custom;
            double acc      = current_task_list[task_param.task_step].acc_js_custom;
            bool   relative = true;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                task_param.task_mode = TASK_JSMOVE2_KEYCODE_J6_ONLY;
                sleep(2);
            }
        } break;

        case (TASK_J6_ROTATE_BY_KEYCODE_ANGLE): {
            // 상대 회전: 현재 q에서 6축에 keycode z(deg) 만큼 더함
            JsDouble q_now = params_.meas.q;
            JsDouble q_target = q_now;
            double delta_deg = drum_rotation_angle_;
            q_target[5] = q_now[5] + delta_deg;

            double vel = current_task_list[task_param.task_step].vel_js_custom > 0 ?
                         current_task_list[task_param.task_step].vel_js_custom : 30.0;
            double acc = current_task_list[task_param.task_step].acc_js_custom > 0 ?
                         current_task_list[task_param.task_step].acc_js_custom : 30.0;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveQ(q_target, vel, acc, false);
            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                task_param.task_mode = TASK_J6_ROTATE_BY_KEYCODE_ANGLE;
                sleep(2);
            }
        } break;

        case (TASK_RECOG_JSMOVE): {

            JsDouble q_target;
            unsigned int idx = current_task_list[task_param.task_step].bp_param.js_position_idx;
            std::vector<double> jointPosition = task_vec_target_JS_position_[idx-1];
            vec2Arr(jointPosition, q_target);

            double vel      = current_task_list[task_param.task_step].vel_js_custom;
            double acc      = current_task_list[task_param.task_step].acc_js_custom;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveQ(q_target, vel, acc, relative);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                task_param.task_mode = TASK_JSMOVE;
                sleep(2);
            }
        } break;

        case (TASK_TORQUE_GUARD_ON): {
            torque_guard_enabled_ = true;
            torque_guard_joint_index_ = current_task_list[task_param.task_step].torque_guard_joint_index;
            torque_guard_threshold_ = current_task_list[task_param.task_step].torque_guard_threshold;
            ROS_LOG_INFO("[TORQUE GUARD] ENABLED | joint=%d | threshold=%.3f", torque_guard_joint_index_, torque_guard_threshold_);
            taskFinFunction(0);
        } break;

        case (TASK_TORQUE_GUARD_OFF): {
            torque_guard_enabled_ = false;
            ROS_LOG_INFO("[TORQUE GUARD] DISABLED");
            taskFinFunction(0);
        } break;

        case (TASK_TORQUE_POLLING_ON): {
            torque_polling_enabled_ = true;
            ROS_LOG_INFO("[TORQUE POLLING] ENABLED - Starting periodic torque service calls (1Hz)");
            taskFinFunction(0);
        } break;

        case (TASK_TORQUE_POLLING_OFF): {
            torque_polling_enabled_ = false;
            ROS_LOG_INFO("[TORQUE POLLING] DISABLED - Stopped periodic torque service calls");
            taskFinFunction(0);
        } break;

        case (TASK_LABEL): {
            // label은 실행 동작 없음. 다음 태스크로 진행
            taskFinFunction(0);
        } break;

        case (TASK_RECOG_CSMOVE): {
            CsDouble x_target;
            unsigned int idx = current_task_list[task_param.task_step].bp_param.cs_pose_idx;
            std::vector<double> cs_pose = task_vec_target_CS_pose_[idx-1];
            vec2Arr(cs_pose, x_target);
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
        } break;

        case (TASK_TCPMOVE): {
            CsDouble x_target = current_task_list[task_param.task_step].x_target;

            double vel      = current_task_list[task_param.task_step].vel_cs;
            double acc      = current_task_list[task_param.task_step].acc_cs;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, false);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                task_param.task_mode = TASK_CSMOVE;
                sleep(2);
            }
        } break;

        case (TASK_BLENDING): {
            BlendingTraj traj_blending = current_task_list[task_param.task_step].traj_blending;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveBlending(traj_blending);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                task_param.task_mode = TASK_BLENDING;
                sleep(2);
            }
        } break;

        case (TASK_DTC): {
            setDtcMode(current_task_list[task_param.task_step].is_dtc_mode);
            setDtcInput(current_task_list[task_param.task_step].target_torque);
            taskFinFunction(0);
        }

        case (WAIT_TIL_FIN_ROBOTMOVE): {
            if (params_.status.is_path_finished) {
                params_.status.is_path_finished = false;
                taskFinFunction(10);
                std::cout << "type: " << "WAIT_TIL_FIN_ROBOTMOVE if section" << std::endl;
                std::cout << "type is_unit_task_fin: " << task_param_.is_unit_task_fin << std::endl;

            }

        } break;

        case (WAIT_TIL_FIN_ROBOTMOVE_500MS_DELAY): {
            if (params_.status.is_path_finished) {
                params_.status.is_path_finished = false;
                taskFinFunction(50);
            }
        } break;


        case (WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_CONVERGING): {
            if (params_.status.is_path_finished) {
                params_.status.is_path_finished = false;
                ROS_LOG_WARN("PATH FINISHED [For Converging] - WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_CONVERGING");

                if(is_marker_detection_converged_) { // 수렴 조건을 만족하면 다음 작업으로
                    ROS_LOG_WARN("MARKER DETECTION CONVERGED! [For Converging]");

                    ////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////
                    //// JSON - 인식된 마커 정보 저장
                    //// FILE PATH
                    // std::string pose_config_path = "/home/bp/[Pose server]/marker_pose/graphy_demo_marker_cs_waypoint.json";
                    std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/marker_pose/graphy_marker_info.json";

                    //// JSON INPUT
                    std::stringstream ss;
                    std::ifstream ifs(pose_config_path.c_str());

                    try {
                        json j_in;
                        try {
                            j_in = json::parse(ifs);
                        } catch (const std::exception & e) {
                            std::cout << e.what() << std::endl;
                            ROS_LOG_ERROR("Retrying JSON PARSING...");
                            ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                            ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                            ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());

                            ////////////////////////
                            //// retry
                            j_in = json::parse(ifs);
                            ////////////////////////
                            ROS_LOG_ERROR("Retrying JSON - SUCCESS!");
                        }

                        json j_out = j_in;
                        std::vector<double> pose_camera_tag(6, 0.0);
                        std::vector<double> pose_base_tag(6, 0.0);

                        for(size_t i = 0; i < 6; i++) {
                            pose_camera_tag[i] = tag_info_current_.pose_camera_tag.element_[i]; // 현재 인식된 마커 정보
                            pose_base_tag[i] = tag_info_current_.pose_base_tag.element_[i]; // 현재 인식된 마커 정보
                        }

                        // ROS_LOG_WARN("[JSON SAVE] Marker detection results: %0.3f, %0.3f, %0.3f, %0.1f, %0.1f, %0.1f", l_pose_tmp[0], l_pose_tmp[1], l_pose_tmp[2], l_pose_tmp[3], l_pose_tmp[4], l_pose_tmp[5]);

                        for(int j = 0; j < 3; j++) {
                            pose_camera_tag[j] = round(pose_camera_tag[j]*1e6) / 1e6; // [m]
                            pose_camera_tag[j + 3] = round(pose_camera_tag[j + 3]*1e3) / 1e3; // [deg]

                            pose_base_tag[j] = round(pose_base_tag[j]*1e6) / 1e6; // [m]
                            pose_base_tag[j + 3] = round(pose_base_tag[j + 3]*1e3) / 1e3; // [deg]
                        }

                        j_out["[Tag Pose]"][current_task_list[task_param.task_step].tag_name.c_str()]["marker_detection_result"]["pose_camera_tag"] = pose_camera_tag;
                        j_out["[Tag Pose]"][current_task_list[task_param.task_step].tag_name.c_str()]["marker_detection_result"]["pose_base_tag"] = pose_base_tag;

                        std::ofstream ofs(pose_config_path.c_str());
                        ofs << j_out.dump(4) << std::endl;

                    } catch (const std::exception & e) {
                        std::cout << e.what() << std::endl;
                        ROS_LOG_ERROR("[MARKER DETECTION] Check JSON file! - marker result saving error! (Tag pose (Camera to Marker) save failed)");
                        is_task_mode_ = false;
                        stopRobot();
                        task_param.task_mode = TASK_DEFAULT;
                        break;
                    }
                    ////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////
                    ////////////////////////////////////////////////////////////////////////////////////////

                    taskFinFunction(10);

                } else {
                    is_tag_detecting_in_progress_ = false;
                    is_tag_received_ = false;
                    is_marker_detection_converged_ = false;
                    tag_detection_delay_ = 0;
                    ROS_LOG_WARN("WAITING MARKER DETECTION TO BE CONVERGED... [For Converging]");
                    task_param.task_mode = TASK_MARKER_DETECTION_FOR_CONVERGING; // 수렴하지 못한 경우 다시 수행
                }
            }
        } break;

        case (WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_APPROACHING): {
            if (params_.status.is_path_finished) {
                params_.status.is_path_finished = false;
                ROS_LOG_WARN("PATH FINISHED [For Approaching] - WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_APPROACHING");

                if(is_marker_detection_approached_) { // 수렴 조건을 만족하면 다음 작업으로
                    ROS_LOG_WARN("MARKER DETECTION APPROACHED! [For Approaching]");
                    taskFinFunction(10);

                } else {
                    is_tag_detecting_in_progress_ = false;
                    is_tag_received_ = false;
                    is_marker_detection_approached_ = false;
                    tag_detection_delay_ = 0;
                    ROS_LOG_WARN("WAITING MARKER DETECTION TO BE APPROACHED... [For Approaching]");
                    task_param.task_mode = TASK_MARKER_DETECTION_FOR_APPROACHING; // 수렴하지 못한 경우 다시 수행
                }
            }
        } break;

        case (WAIT_TIL_FIN_WITH_500MS_DELAY): {
            taskFinFunction(50);
        } break;


#if BIN_PICKING_FLAG

        case (WAIT_TIL_FIN_ROBOTMOVE_WITH_SCAN_AND_MATCHING): {

            if (m_bin_picking_node->is_blending_path_in_process_for_scan_and_matching) {
                if(!m_bin_picking_node->is_blending_scan_and_matching_conducted) {

                    std::vector<double> measured_x(6);
                    arr2Vec(params_.meas.x , measured_x); // scanning joint position
                    //// TODO: 상자 위치 및 경계 치수 정보와 연동하기.
                    //// TODO: 상자 위치 및 경계 치수 정보와 연동하기.
                    //// TODO: 상자 위치 및 경계 치수 정보와 연동하기.
                    double boundary_point = 0.46500; // [m]
                    if(measured_x[0] > boundary_point) { // x좌표가 0.45보다 크면 실행
                        ROS_LOG_WARN("[obj#%zu]SCAN_AND_MATCHING in Blending Path (current position x %0.4f > %0.4f)", m_bin_picking_node->current_target_object_idx_, measured_x[0], boundary_point);
                        ROS_LOG_WARN("SCAN_AND_MATCHING in Blending Path (current position x %0.4f > %0.4f)", m_bin_picking_node->current_target_object_idx_, measured_x[0], boundary_point);
                        ROS_LOG_WARN("SCAN_AND_MATCHING in Blending Path (current position x %0.4f > %0.4f)", m_bin_picking_node->current_target_object_idx_, measured_x[0], boundary_point);
                        m_bin_picking_node->is_blending_scan_and_matching_conducted = true;

                        ////////////////////////////////////////////////////////
                        ////////////////////// 로봇학회 데모 //////////////////////
                        ////////////////////////////////////////////////////////
                        cnt_selected_object_++; // 1개 파지 후, 곧바로 다음 물체로 변경
                        if(cnt_selected_object_ >= m_bin_picking_node->m_selected_object_index_.size()) {
                            cnt_selected_object_ = 0;
                        }

                        //// object index update
                        size_t l_idx_now = m_bin_picking_node->m_selected_object_index_[cnt_selected_object_];
                        ROS_LOG_WARN("[cnt_selected_object_ #%zu]This 5", cnt_selected_object_);
                        ROS_LOG_WARN("[l_idx_now #%zu]This 5", l_idx_now);
                        if(is_sequential_demo_) {

                            ROS_LOG_WARN("is_sequential_demo_");
                            setCurrentObjectIndex(l_idx_now);
                        }


                        //// Matching information update
                        setSamInfoUpdateCurrentObject(); // cloud clear and parameters update

                        //// Robot TCP update
                        // setRobotInfoUpdateCurrentObject(l_idx_now); // robot tcp update and tool/tip changing idx update

                        //// Current grasping info update
                        m_bin_picking_node->setBeforeDetectedPose(m_bin_picking_node->getDetectedPose());
                        m_bin_picking_node->setBeforeDetectedApproachPose(m_bin_picking_node->getDetectedApproachPose());
                        m_bin_picking_node->setBeforeDetectedZigPose(m_bin_picking_node->getDetectedZigPose());



                        //// Gripper info update
                        //// NOTICE: QNode중간에 다음 물체로 업데이트 되는 경우, 흡착 그리퍼 명령처럼 다음 값이 들어가버리는 경우 발생하니 주의
                        //// NOTICE: QNode중간에 다음 물체로 업데이트 되는 경우, 흡착 그리퍼 명령처럼 다음 값이 들어가버리는 경우 발생하니 주의
                        //// NOTICE: QNode중간에 다음 물체로 업데이트 되는 경우, 흡착 그리퍼 명령처럼 다음 값이 들어가버리는 경우 발생하니 주의
                        grp_is_vacuum_before_ = grp_is_vacuum_;


                        ////////////////////////////////////////////////////////

                        //////////////////////////////////////////////////////////////////////////
                        ////////////////// Scan & Matching in blending path  /////////////////////
                        //////////////////////////////////////////////////////////////////////////
                        taskScanningParameter scan_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter;
                        // scan_parameter.scan_position = getRobotAActualQ(); // scanning joint position
                        arr2Vec(params_.meas.q , scan_parameter.scan_position); // scanning joint position
                        scan_parameter.do_scan_sampling = true;
                        taskTemplateMatchingParameter matching_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_matching_parameter;
                        matching_parameter.do_scan_sampling = 8;
                        if(m_bin_picking_node->doScanningAndMatching(scan_parameter, matching_parameter)) {
                            task_param_.is_ready_matching = true;
                        }
                        //////////////////////////////////////////////////////////////////////////
                        //////////////////////////////////////////////////////////////////////////
                        //////////////////////////////////////////////////////////////////////////
                    }
                }
            }

            if (params_.status.is_path_finished) {
                params_.status.is_path_finished = false;
                taskFinFunction(10);
            }
        } break;

#endif

        ////////////////////////////////////////////////////////////////////////////
        ////////////////////////////// TCP Setting /////////////////////////////////
        case (TASK_SET_DS_TCP): {
            ROS_LOG_INFO("TASK_SET_DS_TCP");
            taskFinFunction(10);
            drflSetTcp(current_task_list[task_param.task_step].demo_name);
        } break;
        case (TASK_SET_ROBOT_MODE_AUTO): {                  
            ROS_LOG_INFO("TASK_SET_ROBOT_MODE_AUTO");
            drflSetRobotMode(1); // 1 = AUTO (속도 제한 없음)
            taskFinFunction(10);
        } break;

        case (TASK_SET_ROBOT_MODE_MANUAL): {               
            ROS_LOG_INFO("TASK_SET_ROBOT_MODE_MANUAL");
            drflSetRobotMode(0); // 0 = MANUAL (속도 제한 있음)
            taskFinFunction(10);
        } break;

        case (TASK_SET_DS_SET_COMPLIANCE_MODE): {
            ROS_LOG_INFO("TASK_SET_DS_SET_COMPLIANCE_MODE");
            bool compliance_mode_on = current_task_list[task_param.task_step].mode_on;
            ////
            drflSetRobotMode(0);
            bool success = false;
            if(compliance_mode_on) {
                ROS_LOG_WARN("[DRFL] Compliance CTRL ON...");
                success = drflTaskComplianceCtrl();
                ROS_LOG_WARN("[DRFL] Compliance CTRL ON!");
            } else {
                ROS_LOG_WARN("[DRFL] Compliance CTRL OFF...");
                success = drflReleaseComplianceCtrl();
                ROS_LOG_WARN("[DRFL] Compliance CTRL OFF!");
            }
            drflSetRobotMode(1);
            if (success) {
                if(compliance_mode_on) {
                    ROS_LOG_INFO("Compliance control enabled.");
                } else {
                    ROS_LOG_INFO("Compliance control disabled.");
                }
                taskFinFunction(10);
            } else {
                if(compliance_mode_on) {
                    ROS_LOG_ERROR("Failed to enable compliance control.");
                } else {
                    ROS_LOG_ERROR("Failed to disable compliance control.");
                }
            }
            ////
        } break;

        case (TASK_SET_TCP): {
            ROS_LOG_INFO("TASK_SET_TCP");
            if(setTcp(current_task_list[task_param.task_step].tcp)) {
                taskFinFunction(10);
            }
        } break;

        case (TASK_RESET_TCP): {
            ROS_LOG_INFO("TASK_RESET_TCP");
            taskFinFunction(10);

            setTcp({0, 0, 0, 0, 0, 0});
        } break;

        ////////////////////////////// TCP Setting /////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////
        /////////////////////////////// IMPEDANCE //////////////////////////////////

        case (TASK_SET_FORCE_BIAS): {
            ROS_LOG_INFO("TASK_SET_FORCE_BIAS");
            taskFinFunction(10);
            setCommand("force_bias");
        } break;

        case (TASK_SAVE_FORCE_DATA): {
            string index = current_task_list[task_param.task_step].load_id.index;
            bool saved = saveForceData(index);

            if (saved) {
                taskFinFunction(0);
            }
        } break;

        case (TASK_LOAD_IDENTIFICATION): {
            string file_name("qnode.cpp");
            string file_path = __FILE__;
            file_path.resize(file_path.length() - file_name.length());
            file_path += "../../robot_control_config/controller_config/";
            file_name = file_path + "load_id_force.json";

            Json::Value data;
            std::ifstream ifs;
            ifs.open(file_name);
            ifs >> data;

            std::vector<ModelMatrix> mat_S_set;
            std::vector<ModelMatrix> mat_f_set;
            int data_num = data.size() / 2;
            for (int i = 1; i <= data_num; i++) {
                string key_f = "force" + std::to_string(i);
                string key_g = "g" + std::to_string(i);
                ModelMatrix S_temp(6, 4);
                S_temp.element_[4*0 + 0] = -data[key_g][0].asDouble();
                S_temp.element_[4*0 + 1] = 0;
                S_temp.element_[4*0 + 2] = 0;
                S_temp.element_[4*0 + 3] = 0;

                S_temp.element_[4*1 + 0] = -data[key_g][1].asDouble();
                S_temp.element_[4*1 + 1] = 0;
                S_temp.element_[4*1 + 2] = 0;
                S_temp.element_[4*1 + 3] = 0;

                S_temp.element_[4*2 + 0] = -data[key_g][2].asDouble();
                S_temp.element_[4*2 + 1] = 0;
                S_temp.element_[4*2 + 2] = 0;
                S_temp.element_[4*2 + 3] = 0;

                S_temp.element_[4*3 + 0] = 0;
                S_temp.element_[4*3 + 1] = 0;
                S_temp.element_[4*3 + 2] = -data[key_g][2].asDouble();
                S_temp.element_[4*3 + 3] = data[key_g][1].asDouble();

                S_temp.element_[4*4 + 0] = 0;
                S_temp.element_[4*4 + 1] = data[key_g][2].asDouble();
                S_temp.element_[4*4 + 2] = 0;
                S_temp.element_[4*4 + 3] = -data[key_g][0].asDouble();

                S_temp.element_[4*5 + 0] = 0;
                S_temp.element_[4*5 + 1] = -data[key_g][1].asDouble();
                S_temp.element_[4*5 + 2] = data[key_g][0].asDouble();
                S_temp.element_[4*5 + 3] = 0;
                mat_S_set.push_back(S_temp);

                ModelMatrix f_temp(6, 1);
                f_temp.element_[0] = data[key_f][0].asDouble();
                f_temp.element_[1] = data[key_f][1].asDouble();
                f_temp.element_[2] = data[key_f][2].asDouble();
                f_temp.element_[3] = data[key_f][3].asDouble();
                f_temp.element_[4] = data[key_f][4].asDouble();
                f_temp.element_[5] = data[key_f][5].asDouble();
                mat_f_set.push_back(f_temp);
            }

            ModelMatrix S_init = mat_S_set[0];
            ModelMatrix f_init = mat_f_set[0];
            ModelMatrix AtA(4, 4);
            ModelMatrix AtW(4, 1);
            for (int i = 0; i < data_num; i++) {
                ModelMatrix mat_S(6, 4);
                ModelMatrix mat_f(6, 1);
                mat_S = mat_S_set[i];
                mat_f = mat_f_set[i];
                for (int j = 0; j < 6; j++) {
                    for (int k = 0; k < 4; k++) {
                        for (int p = 0; p < 4; p++) {
                            AtA.element_[k*4 + p] = AtA.element_[k*4 + p] + (mat_S.element_[j*4 + k] - S_init.element_[j*4 + k])*(mat_S.element_[j*4 + p] - S_init.element_[j*4 + p]);
                        }
                        AtW.element_[k] = AtW.element_[k] + (mat_S.element_[j*4 + k] - S_init.element_[j*4 + k])*(mat_f.element_[j] - f_init.element_[j]);
                    }
                }
            }

            ModelMatrix AtA_inv = AtA.inverse();
            ModelMatrix inertia(4, 1);
            inertia.element_[0] = 0;
            inertia.element_[1] = 0;
            inertia.element_[2] = 0;
            inertia.element_[3] = 0;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    inertia.element_[i] = inertia.element_[i] + AtA_inv.element_[i*4 + j]*AtW.element_[j];
                }
            }

            std::vector<double> output(4);
            output[0] = -inertia.element_[0];
            output[1] = inertia.element_[1] / inertia.element_[0];
            output[2] = inertia.element_[2] / inertia.element_[0];
            output[3] = inertia.element_[3] / inertia.element_[0];
            ROS_LOG_WARN("m(kg): %.3lf, x(m): %.3lf, y(m): %.3lf, z(m): %.3lf",
                output[0], output[1], output[2], output[3]);

            taskFinFunction(0);
        } break;

        // case (TASK_IMPEDANCE_ON): {
        //     ROS_LOG_INFO("%s Impedance mode on", prefixString.c_str());
        //     taskFinFunction(50);

        //     if (operating_mode_ == SIM) break;

        //     vector<double> nf          = task_planner_->impedance_default_.nf;
        //     vector<double> zeta        = task_planner_->impedance_default_.zeta;
        //     vector<double> stiffness   = current_task_list[task_param.task_step].impedance.stiffness;
        //     vector<double> force_limit = task_planner_->impedance_default_.force_limit;

        //     setImpedanceControl(true, stiffness, nf, zeta, force_limit, which_robot);
        // } break;

        // case (TASK_IMPEDANCE_OFF): {
        //     ROS_LOG_INFO("%s Impedance mode off", prefixString.c_str());
        //     taskFinFunction(50);

        //     if (operating_mode_ == SIM) break;

        //     vector<double> nf          = task_planner_->impedance_default_.nf;
        //     vector<double> zeta        = task_planner_->impedance_default_.zeta;
        //     vector<double> stiffness   = task_planner_->impedance_default_.stiffness;
        //     vector<double> force_limit = task_planner_->impedance_default_.force_limit;

        //     setImpedanceControl(false, stiffness, nf, zeta, force_limit, which_robot);
        // } break;

        /////////////////////////////// IMPEDANCE //////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////
        //////////////////////////////  Assembly ///////////////////////////////////

        // case (TASK_ZAXIS_DOWN): {
        //     ROS_LOG_INFO("%s Z axis down until touch task", prefixString.c_str());

        //     task_param.time_10ms = 0;
        //     task_param.task_mode  = WAIT_TIL_FIN_ASSEMBLY;

        //     vector<double> contact_force = current_task_list[task_param.task_step].assembly_param.contact_force;
        //     vector<double> force_limit  = task_planner_->impedance_default_.force_limit;

        //     Imped impedance = task_planner_->impedance_default_;
        //     impedance.stiffness  = current_task_list[task_param.task_step].impedance.stiffness;

        //     zAxisDown(contact_force, force_limit, impedance, which_robot);
        // } break;

        // case (TASK_PEG_IN_HOLE_LG_1): {
        //     ROS_LOG_INFO("%s Peg in hole LG 1 task", prefixString.c_str());

        //     task_param.time_10ms = 0;
        //     task_param.task_mode  = WAIT_TIL_FIN_ASSEMBLY;

        //     vector<double> force_limit  = task_planner_->impedance_default_.force_limit;

        //     Imped impedance = task_planner_->impedance_default_;
        //     impedance.stiffness  = current_task_list[task_param.task_step].impedance.stiffness;

        //     pegInHoleLG_1(force_limit, impedance);
        // } break;

        // case (TASK_PEG_IN_HOLE_LG_2): {
        //     ROS_LOG_INFO("%s Peg in hole LG 2 task", prefixString.c_str());

        //     task_param.time_10ms = 0;
        //     task_param.task_mode  = WAIT_TIL_FIN_ASSEMBLY;

        //     vector<double> force_limit  = task_planner_->impedance_default_.force_limit;

        //     Imped impedance = task_planner_->impedance_default_;
        //     impedance.stiffness  = current_task_list[task_param.task_step].impedance.stiffness;

        //     pegInHoleLG_2(force_limit, impedance);
        // } break;

        // case (WAIT_TIL_FIN_ASSEMBLY): {
        //     if (operating_mode_ == SIM) {
        //         ROS_LOG_ERROR("%s Simulation cannot assemble fastener", prefixString.c_str());
        //         taskFinFunction(10);
        //     }
        //     else {
        //         if (task_param.is_assembly_fin) {
        //             if (task_param.is_assembly_successed) {
        //                 task_param.is_assembly_fin = false;
        //                 taskFinFunction(10);
        //             } else {
        //                 ROS_LOG_ERROR("%s Assembly failed !!", prefixString.c_str());
        //                 task_param.task_step = 0;
        //                 task_param.task_mode = TASK_DEFAULT;
        //                 task_param.is_unit_task_fin = false;
        //                 is_task_mode_ = false;
        //             }
        //         }
        //     }
        // } break;

        //////////////////////////////  Assembly ///////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////
        ///////////////////////////// ROBOTIQ GRP //////////////////////////////////

        // case (TASK_ROBOTIQ_ENABLE): {
        //     taskFinFunction(50);
        //     grpEnable();
        // } break;

        // case (TASK_ROBOTIQ_GRASP): {
        //     ROS_LOG_INFO("%s Robotiq gripper grasp", prefixString.c_str());
        //     taskFinFunction(0);

        //     uint pos   = current_task_list[task_param.task_step].robotiq_param[0];
        //     uint vel   = current_task_list[task_param.task_step].robotiq_param[1];
        //     uint force = current_task_list[task_param.task_step].robotiq_param[2];

        //     grpChangeVelForce(vel, force);
        //     grpMove(0);
        // } break;

        // case (TASK_ROBOTIQ_MOVE): {
        //     ROS_LOG_INFO("%s Robotiq gripper move", prefixString.c_str());
        //     taskFinFunction(0);

        //     uint pos   = current_task_list[task_param.task_step].robotiq_param[0];
        //     uint vel   = current_task_list[task_param.task_step].robotiq_param[1];
        //     uint force = current_task_list[task_param.task_step].robotiq_param[2];

        //     grpChangeVelForce(vel, force);
        //     grpMove(pos);
        // } break;

        // case (TASK_ROBOTIQ_RELEASE): {
        //     ROS_LOG_INFO("%s Robotiq gripper release", prefixString.c_str());
        //     taskFinFunction(0);

        //     uint pos   = current_task_list[task_param.task_step].robotiq_param[0];
        //     uint vel   = current_task_list[task_param.task_step].robotiq_param[1];
        //     uint force = current_task_list[task_param.task_step].robotiq_param[2];

        //     grpChangeVelForce(vel, force);
        //     grpMove(140);
        // } break;

        ///////////////////////////// ROBOTIQ GRP //////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////
        /////////////////////////////// IRL GRP ////////////////////////////////////

        case (TASK_IRLGRP_ENABLE): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_IRLGRP_ENABLE");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            taskFinFunction(10);
            // setGrpEnable(true);
            setGrp(1, 0, 1);
        } break;

        case (TASK_IRLGRP_DISABLE): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_IRLGRP_DISABLE");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            taskFinFunction(10);
            setGrpEnable(false);
        } break;

        case (TASK_IRLGRP_INIT): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_IRLGRP_INIT");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            taskFinFunction(50);
            setGrpCmd(MB_GRP_INIT2, (int16_t) current_task_list[task_param.task_step].grp_pos_percent);
        } break;

        case (TASK_IRLGRP_MOVE): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_IRLGRP_MOVE");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            taskFinFunction(50);
            if ((int16_t) current_task_list[task_param.task_step].grp_pos_percent == 10001) {
                setGrpCmd(MB_VAC_ON, (int16_t) current_task_list[task_param.task_step].grp_pos_percent);
            } else if ((int16_t) current_task_list[task_param.task_step].grp_pos_percent == 10002) {
                setGrpCmd(MB_VAC_OFF, (int16_t) current_task_list[task_param.task_step].grp_pos_percent);
            } else {
                setGrpCmd(MB_GRP_POS_CTRL, (int16_t) current_task_list[task_param.task_step].grp_pos_percent);
            }
        } break;

        case (TASK_IRLGRP_STOP): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_IRLGRP_STOP");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            taskFinFunction(50);
            setGrpStop(50);
        } break;

        case (TASK_IRLGRP): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_IRLGRP");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            taskFinFunction(50);
            setGrp(
                current_task_list[task_param.task_step].grp_command,
                current_task_list[task_param.task_step].grp_value,
                current_task_list[task_param.task_step].grp_address
            );
        } break;

        ////////////////////////////////////////////////////////////////////////////
        ////////////////////////////// Zivid SCAN //////////////////////////////////

        case (TASK_ZIVID_SCAN_FOR_CAL): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_ZIVID_SCAN_FOR_CAL");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            fname = task_planner_->fname_;
            auto cal_msg = std_msgs::msg::String();
            cal_msg.data = fname;
            std::string x_msg = "";
            std::array<double, JS_DOF> x_current = params_.meas.x;
            for (int i = 0; i < JS_DOF; i++) {
                x_msg = x_msg + ", " + std::to_string(x_current[i]);
            }
            if (task_planner_->cal_mode_ == "JS") {
                cal_msg.data = fname + x_msg + "JS";
            }
            else if (task_planner_->cal_mode_ == "CS") {
                cal_msg.data = fname + x_msg + "CS";
            }

            cal_scan_pub->publish(cal_msg);
            task_param.task_mode = TASK_ZIVID_WAIT_SAVE_CAL;
            ROS_LOG_INFO("Cal_node: saving ply ... ");
        } break;

        case (TASK_ZIVID_WAIT_SAVE_CAL): {
            if (is_save_fin_) {
                is_save_fin_ = false;
                taskFinFunction(0);
                ROS_LOG_INFO("Cal_node: Saving Ply Done!! ");
            }
        } break;

        /////////////////////////////// Bin Picking ////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////
        //////////////////////////// Bin Picking related /////////////////////////////////
#if BIN_PICKING_FLAG
        case (TASK_DELAY_SCANNING): {
            if(m_bin_picking_node->getMatchingProcessStatus()) { // Topic으로 수행된 경우 skip
                // task_param.task_mode = GO_TO_NEXT_TASK;
                task_param.task_mode = GO_TO_NEXT_TASK;
            } else {
                if (current_task_list[task_param.task_step].delay_10ms > 0){
                    taskFinFunction(current_task_list[task_param.task_step].delay_10ms);
                }
                else {
                    is_task_mode_ = false;
                    break;
                }
            }
        } break;

#endif
        case (TASK_CSMOVE2): { // vec. & acc. (task-dependent)
            CsDouble x_target = current_task_list[task_param.task_step].x_target;
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;
            ROS_LOG_WARN("[TASK_CSMOVE2] x_target: %0.5f, %0.5f, %0.5f, %0.2f, %0.2f, %0.2f", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
// #if DRFL_CONTROL
//             taskFinFunction(0);
// #endif
        } break;

        case (TASK_CSMOVE2_FOR_TEACHING): { // vec. & acc. (task-dependent)

            if(!is_task_teaching_mode_) { // 티칭 모드인 경우에만 실행됨.
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            CsDouble x_target = current_task_list[task_param.task_step].x_target;
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);

        } break;

        case (TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6): {

            JsDouble q_target = current_task_list[task_param.task_step].q_target;
            double vel      = current_task_list[task_param.task_step].vel_js_custom;
            double acc      = current_task_list[task_param.task_step].acc_js_custom;
            bool   relative = current_task_list[task_param.task_step].relative;


            std::vector<double> pose(6);
            pose = teaching_target_pose_;
            ROS_LOG_WARN("[TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6] Current Target CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);


            JsDouble l_q_now = params_.meas.q;
            CsDouble l_x_now = params_.meas.x;
            ROS_LOG_WARN("[TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6] q_now: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n", l_q_now[0], l_q_now[1], l_q_now[2], l_q_now[3], l_q_now[4], l_q_now[5]);
            ROS_LOG_WARN("[TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6] x_now: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", l_x_now[0], l_x_now[1], l_x_now[2], l_x_now[3], l_x_now[4], l_x_now[5]);
            double q_lb = -172.0;
            double q_ub = 172.0;

            double query_x_displacement = pose[5] - l_x_now[5];
            ROS_LOG_WARN("[TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6] query_x_displacement: %0.3f\n", query_x_displacement);


            double q_dis = 0.0;
            bool do_J6_rotation = false;
            //sbp
            // J6 회전방향과 베이스 기준 Rz회전방향은 서로 반대

            if(query_x_displacement > 0) { // Rz 양의 방향
                // J6의 음의 방향(q_lb까지 얼마나 더 이동 가능한지)으로의 이동 허용 마진 확인
                q_dis = q_lb - l_q_now[5];
                if(fabs(q_dis) < fabs(query_x_displacement)) {
                    do_J6_rotation = true;
                    q_target[5] = fabs(q_target[5]); // 마진을 위해 반대 방향인 양의 방향으로 J6회전

                }
            } else { // Rz 음의 방향
                // J6의 양의 방향(q_ub까지 얼마나 더 이동 가능한지)으로의 이동 허용 마진 확인
                q_dis = q_ub - l_q_now[5];
                if(fabs(q_dis) < fabs(query_x_displacement)) {
                    do_J6_rotation = true;
                    q_target[5] = -fabs(q_target[5]); // 마진을 위해 반대 방향인 음의 방향으로 J6회전

                }
            }


            ROS_LOG_WARN("[TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6] q_dis: %0.3f\n", q_dis);
            ROS_LOG_WARN("[TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6] q_target: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f\n", q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);

            //// 마진 초과하면 반대로 J6 회전, 아니면 스킵
            if(!do_J6_rotation) {


                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            } else {

                task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);

                if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                    task_param.task_mode = TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6;
                    sleep(2);
                }
            }

        } break;


        /// sb 3 24.11.25
        case (TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6_turnMargin): {
            // task planner의 목표 값
            JsDouble q_target = current_task_list[task_param.task_step].q_target;
            double vel      = current_task_list[task_param.task_step].vel_js_custom;
            double acc      = current_task_list[task_param.task_step].acc_js_custom;
            bool   relative = current_task_list[task_param.task_step].relative;

            //현재의 JS 6축의 값
            JsDouble l_q_now = params_.meas.q;

            //현재 각도가 최대 각도 +-173도를 넘으면 스킵
            if(fabs(l_q_now[5]) > 173){
                ROS_LOG_INFO("JS 6 Joint Angle is BIG!");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            //최대 각도와 현재 각도의 거리를 계산
            double P_Margin = 173 - l_q_now[5];
            double M_Margin = -173 - l_q_now[5];

            double Margin_dir = P_Margin - M_Margin;

            //양의 마진과 음의 마진에 따라 동작
            if(Margin_dir > 0){
                q_target[5] = fabs(q_target[5]);
                if (P_Margin < fabs(q_target[5])){
                    ROS_LOG_INFO("J6 is BIG!");
                    task_param.task_mode = GO_TO_NEXT_TASK;
                    break;
                }else {

                    task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
                    MoveErrorType err_type = moveQ(q_target, vel, acc, relative);

                    if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                        task_param.task_mode = TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6_turnMargin;
                        sleep(2);
                    }
                }
            } else {
                q_target[5] = -fabs(q_target[5]);
                if(M_Margin < fabs(q_target[5])){
                    ROS_LOG_INFO("J6 is BIG!");
                    task_param.task_mode = GO_TO_NEXT_TASK;
                    break;
                }else{

                    task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
                    MoveErrorType err_type = moveQ(q_target, vel, acc, relative);

                    if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                        task_param.task_mode = TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6_turnMargin;
                        sleep(2);
                    }
                }
            }


            //reverse 함수의 값을 전달
            q_target_reverse_ = q_target;
            relative_reverse_ = relative;


        } break;


        // sb 3 24.11.25
        case (TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6_reverse): {
            if(relative_reverse_ == true){
                JsDouble q_target = q_target_reverse_;
                double vel      = current_task_list[task_param.task_step].vel_js_custom;
                double acc      = current_task_list[task_param.task_step].acc_js_custom;

                q_target[5] = -(q_target[5]);

                task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
                MoveErrorType err_type = moveQ(q_target, vel, acc, true);

                if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                    ROS_LOG_WARN("33333333");
                    task_param.task_mode = TASK_CHECK_J6_LIMIT_AND_JSMOVE2_ONLY_J6_reverse;
                    sleep(2);
                }
            }
            else {

                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

        } break;

        // sb 25.04.11
        case (TASK_CSMOVE_ROTATE_KNOB_TO_INIT): { // vec. & acc. (task-dependent)
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = false;

            CsDouble x_target;
            std::vector<double> pose(6);
            pose = teaching_target_pose_;
            vec2Arr(pose, x_target);

            teaching_to_init_RZ_ = x_target[5] - 90.0;
            x_target[5] = 90.0;


            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {

                task_param.task_mode = TASK_CSMOVE_ROTATE_KNOB_TO_INIT;
                sleep(2);
            }

        } break;



        /// mw 1 25.01.10
        case (TASK_SET_J6_POSSIBLE_DIRECTION): {
            // task planner의 목표 값
            JsDouble q_target = current_task_list[task_param.task_step].q_target;
            double vel      = current_task_list[task_param.task_step].vel_js_custom;
            double acc      = current_task_list[task_param.task_step].acc_js_custom;
            bool   relative = current_task_list[task_param.task_step].relative;

            //현재의 JS 6축의 값
            JsDouble l_q_now = params_.meas.q;

            //현재 각도가 최대 각도 +-173도를 넘으면 스킵
            if(fabs(l_q_now[5]) > 173){
                ROS_LOG_INFO("[TASK_SET_J6_POSSIBLE_DIRECTION 1] JS 6 Joint Angle is too BIG!");
                // task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            //최대 각도와 현재 각도의 거리를 계산
            double P_Margin = 173 - l_q_now[5];
            double M_Margin = -173 - l_q_now[5];

            double Margin_dir = P_Margin - M_Margin;

            //양의 마진과 음의 마진에 따라 동작
            if(Margin_dir > 0){
                q_target[5] = fabs(q_target[5]);
            } else {
                q_target[5] = -fabs(q_target[5]);
            }

            // set
            q_target_reverse_ = q_target;
            relative_reverse_ = relative;

            task_param.task_mode = GO_TO_NEXT_TASK;

        } break;


        /// mw 1 25.01.10
        case (TASK_JSMOVE_ONLY_J6_ASSIGNED_DIRECTION): {
            // task planner의 목표 값
            JsDouble q_target = current_task_list[task_param.task_step].q_target;
            double vel      = current_task_list[task_param.task_step].vel_js_custom;
            double acc      = current_task_list[task_param.task_step].acc_js_custom;

            for (size_t i = 0; i < 5; i++) {
                q_target[i] = 0.0;
            }
            q_target[5] = fabs(q_target[5]);
            if(q_target_reverse_[5] < 0.0) {
                q_target[5] = -(q_target[5]);
            }

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveQ(q_target, vel, acc, true);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                ROS_LOG_WARN("33333333");
                task_param.task_mode = TASK_JSMOVE_ONLY_J6_ASSIGNED_DIRECTION;
                sleep(2);
            }

        } break;

        /// mw 1 25.01.10
        case (TASK_JSMOVE_ONLY_J6_ASSIGNED_DIRECTION_REVERSE): {
            // task planner의 목표 값
            JsDouble q_target = current_task_list[task_param.task_step].q_target;
            double vel      = current_task_list[task_param.task_step].vel_js_custom;
            double acc      = current_task_list[task_param.task_step].acc_js_custom;

            for (size_t i = 0; i < 5; i++) {
                q_target[i] = 0.0;
            }

            q_target[5] = fabs(q_target[5]);
            if(q_target_reverse_[5] < 0.0) {
                q_target[5] = -(q_target[5]);
            }

            // reverse direction
            q_target[5] = -(q_target[5]);

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveQ(q_target, vel, acc, true);

            if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                ROS_LOG_WARN("33333333");
                task_param.task_mode = TASK_JSMOVE_ONLY_J6_ASSIGNED_DIRECTION_REVERSE;
                sleep(2);
            }

        } break;


        case (TASK_CSMOVE_TOOL_FRAME): { // 상대자세 입력 시, Tool frame 기준의 동작을 생성
            ROS_LOG_INFO("TASK_CSMOVE_TOOL_FRAME!");
            CsDouble x_target = current_task_list[task_param.task_step].x_target;
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = false;
            bool   relative = true;
            ROS_LOG_WARN("[TASK_CSMOVE_TOOL_FRAME] x_target: %0.5f, %0.5f, %0.5f, %0.2f, %0.2f, %0.2f", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);

// #if DRFL_CONTROL
//             taskFinFunction(0);
// #endif

        } break;

        // case (TASK_CSMOVE_TOOL_FRAME_KEYCODE): {
        //     ROS_LOG_INFO("TASK_CSMOVE_TOOL_FRAME_KEYCODE!");
        //     CsDouble x_target = current_task_list[task_param.task_step].x_target;
        //     x_target[5] = drum_rotation_angle_;
        //     double vel      = current_task_list[task_param.task_step].vel_cs_custom;
        //     double acc      = current_task_list[task_param.task_step].acc_cs_custom;
        //     bool   is_base_frame = false;
        //     bool   relative = true;
        //     ROS_LOG_WARN("[TASK_CSMOVE_TOOL_FRAME_KEYCODE] Tool Rz: %.3f deg", x_target[5]);
        //     task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
        //     MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
        // } break;

        case (TASK_CSMOVE_TOOL_FRAME_KEYCODE): {
            ROS_LOG_INFO("TASK_CSMOVE_TOOL_FRAME_KEYCODE!");
            
            CsDouble x_target = current_task_list[task_param.task_step].x_target;
            
            /// 먼저 170도 반시계 회전
            x_target[5] = -170.0;
            
            double vel = current_task_list[task_param.task_step].vel_cs_custom;
            double acc = current_task_list[task_param.task_step].acc_cs_custom;
            bool is_base_frame = false; // tool frame
            bool relative = true;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
            
            if (err_type == MoveErrorType::NONE) {
                ROS_LOG_INFO("Step 1 completed, starting Step 2...");
                x_target[5] = drum_rotation_angle_ - 190.0;
                ROS_LOG_INFO("Step 2: %.3f degrees", x_target[5]);
                err_type = moveX(x_target, vel, acc, is_base_frame, relative);
            }
        } break;

        case (TASK_CSMOVE_TOOL_FRAME_KEYCODE_FOR_HOLDER): {
            ROS_LOG_INFO("TASK_CSMOVE_TOOL_FRAME_KEYCODE_FOR_HOLDER!");
            CsDouble x_target = current_task_list[task_param.task_step].x_target;
            x_target[5] = drum_rotation_angle_holder_;
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = false;
            bool   relative = true;
            ROS_LOG_WARN("[TASK_CSMOVE_TOOL_FRAME_KEYCODE_FOR_HOLDER] Tool Rz: %.3f deg", x_target[5]);
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
        } break;

        case (TASK_CSMOVE_TOOL_FRAME_FOR_TEACHING): { // 상대자세 입력 시, Tool frame 기준의 동작을 생성

            if(!is_task_teaching_mode_) {
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            ROS_LOG_INFO("TASK_CSMOVE_TOOL_FRAME_FOR_TEACHING!");
            CsDouble x_target = current_task_list[task_param.task_step].x_target;
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = false;
            bool   relative = true;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);

        } break;


        case (TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE): { // vec. & acc. (task-dependent)
            CsDouble x_sum = current_task_list[task_param.task_step].x_target;
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;

            // Target teaching pose with x_sum
            CsDouble x_target;
            std::vector<double> pose(6);
            pose = teaching_target_pose_;
            vec2Arr(pose, x_target);
            x_target = task_planner_->arraySumCS(x_target, x_sum);
            ROS_LOG_WARN("[TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE] Target CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);

        } break;


        case (TASK_CSMOVE_TOOL_FRAME_TEACHING_TARGET_POSE): { // vec. & acc. (task-dependent)
            CsDouble x_sum = current_task_list[task_param.task_step].x_target;
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;

            ////////////////////////////////////////////
            // Target teaching pose with tool frame motion
            std::vector<double> current_pose = teaching_target_pose_;
            std::vector<double> tool_relative_pose(6, 0.0); // tool frame
            arr2Vec(x_sum, tool_relative_pose);
            // detected_pose로부터 Tool frame 기준의 상대 이동 (approach distance 고려)
            for (std::size_t i = 0; i < 3; i++) {
                current_pose[i + 3] = current_pose[i + 3] * kDeg2Rad;
                tool_relative_pose[i + 3] = tool_relative_pose[i + 3] * kDeg2Rad;
            }

            std::vector<double> pose(6);
            m_bp_math.transformToolFrame(tool_relative_pose, current_pose, pose); // [m], [rad]
            for (std::size_t i = 0; i < 3; i++) { pose[i + 3] = pose[i + 3] * kRad2Deg; }
            ////////////////////////////////////////////
            CsDouble x_target;
            vec2Arr(pose, x_target);
            ROS_LOG_WARN("[TASK_CSMOVE_TOOL_FRAME_TEACHING_TARGET_POSE] Target CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
        } break;

        case (TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE2): { // vec. & acc. (task-dependent)
            CsDouble x_sum = current_task_list[task_param.task_step].x_target;
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;

            // Target teaching pose with x_sum
            CsDouble x_teaching_target = current_task_list[task_param.task_step].x_teaching_target;
            // std::vector<double> pose(6);
            // pose = teaching_target_pose_;
            // vec2Arr(pose, x_target);
            CsDouble x_target = task_planner_->arraySumCS(x_teaching_target, x_sum);
            ROS_LOG_WARN("[TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE] Target CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);

        } break;


        case (TASK_CSMOVE_TOOL_FRAME_TEACHING_TARGET_POSE2): { // vec. & acc. (task-dependent)
            CsDouble x_sum = current_task_list[task_param.task_step].x_target;
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;

            ////////////////////////////////////////////
            CsDouble x_teaching_target = current_task_list[task_param.task_step].x_teaching_target;
            // Target teaching pose with tool frame motion
            std::vector<double> current_pose(6);
            arr2Vec(x_teaching_target, current_pose);
            std::vector<double> tool_relative_pose(6, 0.0); // tool frame
            arr2Vec(x_sum, tool_relative_pose);
            // detected_pose로부터 Tool frame 기준의 상대 이동 (approach distance 고려)
            for (std::size_t i = 0; i < 3; i++) {
                current_pose[i + 3] = current_pose[i + 3] * kDeg2Rad;
                tool_relative_pose[i + 3] = tool_relative_pose[i + 3] * kDeg2Rad;
            }

            std::vector<double> pose(6);
            m_bp_math.transformToolFrame(tool_relative_pose, current_pose, pose); // [m], [rad]
            for (std::size_t i = 0; i < 3; i++) { pose[i + 3] = pose[i + 3] * kRad2Deg; }
            ////////////////////////////////////////////
            CsDouble x_target;
            vec2Arr(pose, x_target);
            ROS_LOG_WARN("[TASK_CSMOVE_TOOL_FRAME_TEACHING_TARGET_POSE] Target CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
            MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
        } break;


        case (TASK_SET_TEACHING_TARGET_POSE): { // vec. & acc. (task-dependent)

            tag_teaching_pose_ = current_task_list[task_param.task_step].tag_teaching_pose;
            pose_idx_teaching_pose_ = current_task_list[task_param.task_step].pose_idx_teaching_pose;

            //// TODO: is_task_teaching_mode_를 티칭 창에서 모드 선택할 수 있도록 처리
            if(!is_teaching_finished_ && is_task_teaching_mode_) {
                task_param.task_mode = WAIT_TIL_FIN_TEACHING;
                is_in_task_teaching_stage_ = true; // GUI에서 버튼 활성화하기 위한 flag
                break;
            }
            is_in_task_teaching_stage_ = false; // GUI에서 버튼 비활성화하기 위한 flag

            ROS_LOG_WARN("[Teaching Tag]: %s - %s\n", tag_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());


            //// TODO: JSON Pose load
            //// TODO: 아래에서 체크한 뒤에, task planner 내부로 넣기
            //// FILE PATH
            // std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/teaching_pose/graphy_demo_cs_waypoint.json";
            std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/teaching_pose/graphy_demo_cs_waypoint.json";
            //// JSON INPUT
            std::stringstream ss;
            std::ifstream ifs(pose_config_path.c_str());

            json j_in;
            try {
                j_in = json::parse(ifs);
            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("Retrying JSON PARSING...");
                ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());

                ////////////////////////
                //// retry
                j_in = json::parse(ifs);
                ////////////////////////
                ROS_LOG_ERROR("Retrying JSON - SUCCESS!");
            }

            //// 1) JSON Load Robot Workspace Limits
            ///////////////////////////////////////////////////////////////////
            Eigen::MatrixXd workspace(6,2); // pose limit - (min, max)
            std::vector<double> workspace_in;
            try {
                workspace_in = j_in[tag_teaching_pose_.c_str()]["workspace"].get<std::vector<double>>();
            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("Check JSON file! - loading error!");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            if(workspace_in.size() != 12) {
                ROS_LOG_ERROR("Check JSON file! - loading error!");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }
            for(int i=0; i<6; i++) {
                workspace(i, 0) = workspace_in[2 * i]; // min
                workspace(i, 1) = workspace_in[2 * i + 1]; // max
            }

            ///////////////////////////////////////////////////////////////////
            //// 2) JSON Load Teaching Pose
            std::vector<double> teaching_pose_in;
            try {
                teaching_pose_in = j_in[tag_teaching_pose_.c_str()][pose_idx_teaching_pose_.c_str()].get<std::vector<double>>();

                if(tag_teaching_pose_ == "[Task B-2]") { // 기존, 미국 전시회

                    if(pose_idx_teaching_pose_ != "Pose #9" && pose_idx_teaching_pose_ != "Pose #10-1"
                        && pose_idx_teaching_pose_ != "Pose #10-2" && pose_idx_teaching_pose_ != "Pose #10-3"
                        && pose_idx_teaching_pose_ != "Pose #10-4" && pose_idx_teaching_pose_ != "Pose #10-5"
                        && pose_idx_teaching_pose_ != "Pose #10-6" && pose_idx_teaching_pose_ != "Pose #10-7"
                        && pose_idx_teaching_pose_ != "Pose #10-8") {
                        ROS_LOG_WARN("TASK B2 CHECK - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                        ROS_LOG_WARN("TASK B2 CHECK - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                        ROS_LOG_WARN("TASK B2 CHECK - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                        ROS_LOG_WARN("TASK B2 CHECK - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());


                        if(pose_idx_teaching_pose_ == "Pose #4" || pose_idx_teaching_pose_ == "Pose #5") {
                            //// 4, 5번 포즈만 방위를 B-2에 저장된 방위를 사용
                            std::vector<double> tmp_pose = teaching_pose_in;

                            teaching_pose_in = j_in["[Task B-1]"][pose_idx_teaching_pose_.c_str()].get<std::vector<double>>();

                            teaching_pose_in[3] = tmp_pose[3];
                            teaching_pose_in[4] = tmp_pose[4];
                            teaching_pose_in[5] = tmp_pose[5];
                            ROS_LOG_WARN("Task B-1 position used! - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                        } else {
                            //// B-1의 자세를 사용하도록 처리
                            teaching_pose_in = j_in["[Task B-1]"][pose_idx_teaching_pose_.c_str()].get<std::vector<double>>();
                            ROS_LOG_WARN("Task B-1 pose used! - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                        }
                    }
                } else if(tag_teaching_pose_ == "[Task B-2]_UNIZ") { // UNIZ 연동 버전
                    ROS_LOG_WARN("[UNIZ data] - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                    ROS_LOG_WARN("[UNIZ data] - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                    ROS_LOG_WARN("[UNIZ data] - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                    ROS_LOG_WARN("[UNIZ data] - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                    ROS_LOG_WARN("[UNIZ data] - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());

                    if(pose_idx_teaching_pose_ != "Pose #9" && pose_idx_teaching_pose_ != "Pose #10-1"
                        && pose_idx_teaching_pose_ != "Pose #10-2" && pose_idx_teaching_pose_ != "Pose #10-3"
                        && pose_idx_teaching_pose_ != "Pose #10-4" && pose_idx_teaching_pose_ != "Pose #10-5"
                        && pose_idx_teaching_pose_ != "Pose #10-6" && pose_idx_teaching_pose_ != "Pose #10-7"
                        && pose_idx_teaching_pose_ != "Pose #10-8"
                        && pose_idx_teaching_pose_ != "Pose top-right corner") {
                        ROS_LOG_WARN("[Task B-2]_UNIZ CHECK - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                        ROS_LOG_WARN("[Task B-2]_UNIZ CHECK - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                        ROS_LOG_WARN("[Task B-2]_UNIZ CHECK - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());
                        ROS_LOG_WARN("[Task B-2]_UNIZ CHECK - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());

                        if(!is_uniz_file_loaded_) {
                            ROS_LOG_ERROR("UNIZ DATA NOT EXIST!");
                            ROS_LOG_ERROR("UNIZ DATA NOT EXIST!");
                            ROS_LOG_ERROR("UNIZ DATA NOT EXIST!");
                            ROS_LOG_ERROR("UNIZ DATA NOT EXIST!");
                            ROS_LOG_ERROR("UNIZ DATA NOT EXIST!");
                            ROS_LOG_ERROR("UNIZ DATA NOT EXIST!");
                            ROS_LOG_ERROR("UNIZ DATA NOT EXIST!");
                            ROS_LOG_ERROR("UNIZ DATA NOT EXIST!");
                            ROS_LOG_ERROR("UNIZ DATA NOT EXIST!");

                            is_task_mode_ = false;
                            stopRobot();
                            task_param.task_mode = TASK_DEFAULT;
                            break;
                        }

                        ////////////////////////////////////////////////////////////////////////////////////////////
                        // JSON 파일 열기
                        std::string config_path = "/home/irl-sbc250j/Desktop/UNIZ data/uniz_setting/uniz_mode_setting.json";
                        std::ifstream ifs(config_path.c_str());
                        json j_uniz_setting_in;
                        try {
                            j_uniz_setting_in = json::parse(ifs);
                        } catch (const std::exception & e) {
                            std::cout << e.what() << std::endl;
                            ROS_LOG_ERROR("Retrying JSON PARSING...");
                            ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", config_path.c_str());
                            ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", config_path.c_str());
                            ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", config_path.c_str());

                            ////////////////////////
                            //// retry
                            j_uniz_setting_in = json::parse(ifs);
                            ////////////////////////
                            ROS_LOG_ERROR("Retrying JSON - SUCCESS!");
                        }

                        if (!ifs.is_open()) {
                            std::cerr << "uniz_mode_setting.json loading - failed!" << std::endl;
                        }
                        bool is_mode_obb = j_uniz_setting_in["is_mode_obb"];
                        if(!is_mode_obb) { // 기존 AABB
                            ROS_LOG_WARN("UNIZ AABB Mode");
                        //// NOTICE: 현재는 일단 이전 티칭 값 방위를 사용
                        //// TODO: UNIZ 방위 추출하여 사용해야 함.
                        //// TODO: UNIZ 방위 추출하여 사용해야 함.
                        //// TODO: UNIZ 방위 추출하여 사용해야 함.
                        if(pose_idx_teaching_pose_ == "Pose #4" || pose_idx_teaching_pose_ == "Pose #5") {
                            //// 4, 5번 포즈만 방위를 B-2에 저장된 방위를 사용
                            std::vector<double> tmp_pose = j_in["[Task B-2]"][pose_idx_teaching_pose_.c_str()].get<std::vector<double>>();

                            teaching_pose_in = j_in["[Task B-1]"][pose_idx_teaching_pose_.c_str()].get<std::vector<double>>();

                            teaching_pose_in[3] = tmp_pose[3];
                            teaching_pose_in[4] = tmp_pose[4];
                            teaching_pose_in[5] = tmp_pose[5];
                        } else {
                            //// B-1의 자세를 사용하도록 처리
                            teaching_pose_in = j_in["[Task B-1]"][pose_idx_teaching_pose_.c_str()].get<std::vector<double>>();
                            }
                        } else { // OBB, 방위 적용, 240910
                            ROS_LOG_WARN("UNIZ OBB Mode");
                            std::vector<double> uniz_pose = j_in["[Task B-2]_UNIZ"][pose_idx_teaching_pose_.c_str()].get<std::vector<double>>();


                            Eigen::VectorXf uniz_pose_tmp(6);
                            for(int j = 0; j < 3; j++) {
                                uniz_pose_tmp[j] = uniz_pose[j]; // [m]
                                uniz_pose_tmp[j + 3] = kDeg2Rad*uniz_pose[j + 3]; // [rad]
                            }
                            Eigen::Matrix4f T_uniz_pose_tmp;
                            m_bp_math.pose2HTM(uniz_pose_tmp, T_uniz_pose_tmp); // [m], [rad]

                            Eigen::Matrix3f R_uniz_orientation = Eigen::Matrix3f::Identity();
                            R_uniz_orientation = T_uniz_pose_tmp.block<3, 3>(0, 0);


                            //// NOTICE: UNIZ에서 추출된 방위는
                            //// 얼라이너 앞니 꼭대기로부터 얼라이너 중심을 향하는 방향벡터를 +y로 설정하고, UNIZ 좌표계 기준 +z로 설정된 방위임
                            //// 여기서는 y축 방향으로 180도 회전행렬을 곱하면 최종 방위가 산출
                            Eigen::Matrix3f rotM_y180 = Eigen::Matrix3f::Identity();
                    		rotM_y180(0, 0) = cos(kDeg2Rad*180.0);	rotM_y180(0, 2) = sin(kDeg2Rad*180.0);
		                    rotM_y180(2, 0) = -sin(kDeg2Rad*180.0);	rotM_y180(2, 2) = cos(kDeg2Rad*180.0);

                            //// Orientation
                            R_uniz_orientation = R_uniz_orientation*rotM_y180;

                            Eigen::Vector3f RPY = m_bp_math.rotm2ZYX(R_uniz_orientation);
                            teaching_pose_in[3] = RPY[0]*kRad2Deg;
                            teaching_pose_in[4] = RPY[1]*kRad2Deg;
                            teaching_pose_in[5] = RPY[2]*kRad2Deg;
                            ROS_LOG_WARN("[UNIZ data] orientation used! - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());

                        }

                        //// NOTICE: 현재는 로봇의 base frame 과 UNIZ 기본 좌표계가 일치하도록 처리됨.
                        //// TODO: 방위 구현 후에, 좌표변환으로 변경
                        //// TODO: 방위 구현 후에, 좌표변환으로 변경
                        //// TODO: 방위 구현 후에, 좌표변환으로 변경
                        ////////////////////
                        //// UNIZ Data position update
                        std::vector<double> uniz_pose = j_in["[Task B-2]_UNIZ"][pose_idx_teaching_pose_.c_str()].get<std::vector<double>>();
                        teaching_pose_in[0] = uniz_top_right_initial_pose_[0] + uniz_pose[0];
                        teaching_pose_in[1] = uniz_top_right_initial_pose_[1] + uniz_pose[1];
                        teaching_pose_in[2] = uniz_top_right_initial_pose_[2] + uniz_pose[2];
                        ////////////////////
                        ROS_LOG_WARN("[UNIZ data] position used! - %s", pose_idx_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());

                        ROS_LOG_WARN("[UNIZ HTM Result] teaching_pose_in: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_pose_in[0], teaching_pose_in[1], teaching_pose_in[2], teaching_pose_in[3], teaching_pose_in[4], teaching_pose_in[5]);


                        /////////////////////////////////////////////////////////////////////////////
                        //// TODO: Z기준 추가, 바닥을 찍지 않도록
                        // double l_z_limit = 0.015; // 기존
                        double l_z_limit = 0.01; // 변경, 241223
                        // if(teaching_pose_in[2] - uniz_top_right_initial_pose_[2] < 0.015) {
                        if(teaching_pose_in[2] - uniz_top_right_initial_pose_[2] < l_z_limit) {
                            ROS_LOG_ERROR("B-2 Task Error: (build plate) Z limit over! - Stop Robot!");
                            ROS_LOG_ERROR("teaching_pose_in[2] - uniz_top_right_initial_pose_[2]: %0.4f(< %0.3f)", teaching_pose_in[2] - uniz_top_right_initial_pose_[2], l_z_limit);
                            stopRobot();
                            task_param.task_mode = TASK_DEFAULT;
                            break;
                        }
                        /////////////////////////////////////////////////////////////////////////////

                    }

                    if(pose_idx_teaching_pose_ == "Pose top-right corner") {
                        uniz_top_right_initial_pose_ = teaching_pose_in;
                        ROS_LOG_WARN("[JSON LOAD] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                        ROS_LOG_WARN("[JSON LOAD] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                        ROS_LOG_WARN("[JSON LOAD] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                        ROS_LOG_WARN("[JSON LOAD] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                        ROS_LOG_WARN("[JSON LOAD] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                        ROS_LOG_WARN("[JSON LOAD] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                        ROS_LOG_WARN("[JSON LOAD] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                        ROS_LOG_WARN("[JSON LOAD] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                    }

                }

                ROS_LOG_WARN("[JSON LOAD] TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_pose_in[0], teaching_pose_in[1], teaching_pose_in[2], teaching_pose_in[3], teaching_pose_in[4], teaching_pose_in[5]);
            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("Check JSON file! - loading error! (teaching pose loading failed)");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            //// 3) Check Robot Workspace
            if (checkWorkspace(teaching_pose_in, workspace)) {
                teaching_target_pose_ = teaching_pose_in;
                ROS_LOG_WARN("[QNode] Set Current TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_target_pose_[0], teaching_target_pose_[1], teaching_target_pose_[2], teaching_target_pose_[3], teaching_target_pose_[4], teaching_target_pose_[5]);
            } else {
                ROS_LOG_WARN("workspace[min]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f", workspace_in[0], workspace_in[2], workspace_in[4], workspace_in[6], workspace_in[8], workspace_in[10]);
                ROS_LOG_WARN("workspace[max]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f", workspace_in[1], workspace_in[3], workspace_in[5], workspace_in[7], workspace_in[9], workspace_in[11]);
                ROS_LOG_ERROR("Robot Workspace Limit!");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            is_teaching_finished_ = false; // flag init.
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;

        case (TASK_SET_TEACHING_TARGET_POSE_WITH_MARKER_DETECTION): { // vec. & acc. (task-dependent)

            tag_teaching_pose_ = current_task_list[task_param.task_step].tag_teaching_pose;
            pose_idx_teaching_pose_ = current_task_list[task_param.task_step].pose_idx_teaching_pose;

            //// TODO: is_task_teaching_mode_를 티칭 창에서 모드 선택할 수 있도록 처리
            if(!is_teaching_finished_ && is_task_teaching_mode_) {
                task_param.task_mode = WAIT_TIL_FIN_TEACHING_WITH_MARKER_DETECTION; // yh find
                is_in_task_teaching_stage_ = true; // GUI에서 버튼 활성화하기 위한 flag
                break;
            }
            is_in_task_teaching_stage_ = false; // GUI에서 버튼 비활성화하기 위한 flag

            ROS_LOG_WARN("[WITH MARKER] [Teaching Tag]: %s - %s\n", tag_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());

            //// 인식된 Tag 정보 확인
            ROS_LOG_WARN("[MARKER DETECTION] [Tag Name]: %s", tag_info_current_.tag_name.c_str());
            if (tag_info_current_.tag_name == "null") {
                ROS_LOG_WARN("No tag information!");
                ROS_LOG_WARN("No tag information!");
                ROS_LOG_WARN("No tag information!");
                ROS_LOG_WARN("No tag information!");
                ROS_LOG_WARN("No tag information!");

                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }


            //// TODO: JSON Pose load
            //// TODO: 아래에서 체크한 뒤에, task planner 내부로 넣기
            //// FILE PATH
            std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/marker_pose/graphy_demo_marker_cs_waypoint.json";
            //// JSON INPUT
            std::stringstream ss;
            std::ifstream ifs(pose_config_path.c_str());

            json j_in;
            try {
                j_in = json::parse(ifs);
            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("Retrying JSON PARSING...");
                ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());

                ////////////////////////
                //// retry
                j_in = json::parse(ifs);
                ////////////////////////
                ROS_LOG_ERROR("Retrying JSON - SUCCESS!");
            }


            //// M-1) Marker-based pose JSON Load
            std::vector<double> teaching_marker_pose_in;
            try {
                teaching_marker_pose_in = j_in[tag_teaching_pose_.c_str()][pose_idx_teaching_pose_.c_str()].get<std::vector<double>>();
                ROS_LOG_WARN("[MARKER DETECTION] [JSON LOAD] TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_marker_pose_in[0], teaching_marker_pose_in[1], teaching_marker_pose_in[2], teaching_marker_pose_in[3], teaching_marker_pose_in[4], teaching_marker_pose_in[5]);

            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("[MARKER DETECTION] Check JSON file! - loading error! (teaching pose loading failed)");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            //// M-2) Transform the pose w.r.t. base frame
            std::vector<double> input_pose_vec = teaching_marker_pose_in;
            ModelMatrix pose_tag_target(6, 1, input_pose_vec), pose_base_target;
            pose_base_target = ModelMatrix::tr2pose(tag_info_current_.tr_base_tag
                                                * ModelMatrix::pose2tr(pose_tag_target));

            CsDouble output_pose;
            for (int i = 0; i < CS_DOF; i++) {
                output_pose[i] = pose_base_target.element_[i];
            }
            ROS_LOG_WARN("[MARKER DETECTION] Marker-based Output CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", output_pose[0], output_pose[1], output_pose[2], output_pose[3], output_pose[4], output_pose[5]);

            //// 1) JSON Load Robot Workspace Limits
            ///////////////////////////////////////////////////////////////////
            Eigen::MatrixXd workspace(6,2); // pose limit - (min, max)
            std::vector<double> workspace_in;
            try {
                workspace_in = j_in[tag_teaching_pose_.c_str()]["workspace"].get<std::vector<double>>();
            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("Check JSON file! - loading error!");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            if(workspace_in.size() != 12) {
                ROS_LOG_ERROR("Check JSON file! - loading error!");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }
            for(int i=0; i<6; i++) {
                workspace(i, 0) = workspace_in[2 * i]; // min
                workspace(i, 1) = workspace_in[2 * i + 1]; // max
            }

            ///////////////////////////////////////////////////////////////////
            //// 2) JSON Load Teaching Pose
            std::vector<double> teaching_pose_in;
            try {
                arr2Vec(output_pose, teaching_pose_in);

                if(pose_idx_teaching_pose_ == "Pose top-right corner") {
                    uniz_top_right_initial_pose_ = teaching_pose_in;
                    ROS_LOG_WARN("[MARKER DETECTION] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                    ROS_LOG_WARN("[MARKER DETECTION] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                    ROS_LOG_WARN("[MARKER DETECTION] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                    ROS_LOG_WARN("[MARKER DETECTION] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                    ROS_LOG_WARN("[MARKER DETECTION] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                    ROS_LOG_WARN("[MARKER DETECTION] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                    ROS_LOG_WARN("[MARKER DETECTION] uniz_top_right_initial_pose_: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", uniz_top_right_initial_pose_[0], uniz_top_right_initial_pose_[1], uniz_top_right_initial_pose_[2], uniz_top_right_initial_pose_[3], uniz_top_right_initial_pose_[4], uniz_top_right_initial_pose_[5]);
                }

                ROS_LOG_WARN("[MARKER DETECTION RESULTS] TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_pose_in[0], teaching_pose_in[1], teaching_pose_in[2], teaching_pose_in[3], teaching_pose_in[4], teaching_pose_in[5]);
            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("[MARKER DETECTION RESULTS] Check JSON file! - loading error! (teaching pose loading failed)");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            //// 3) Check Robot Workspace
            if (checkWorkspace(teaching_pose_in, workspace)) {
                teaching_target_pose_ = teaching_pose_in;
                ROS_LOG_WARN("[MARKER DETECTION RESULTS] [QNode] Set Current TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_target_pose_[0], teaching_target_pose_[1], teaching_target_pose_[2], teaching_target_pose_[3], teaching_target_pose_[4], teaching_target_pose_[5]);
            } else {
                ROS_LOG_WARN("[MARKER DETECTION RESULTS] workspace[min]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f", workspace_in[0], workspace_in[2], workspace_in[4], workspace_in[6], workspace_in[8], workspace_in[10]);
                ROS_LOG_WARN("[MARKER DETECTION RESULTS] workspace[max]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f", workspace_in[1], workspace_in[3], workspace_in[5], workspace_in[7], workspace_in[9], workspace_in[11]);
                ROS_LOG_ERROR("[MARKER DETECTION RESULTS] Robot Workspace Limit!");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            is_teaching_finished_ = false; // flag init.
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;


        case (TASK_SET_TEACHING_TARGET_POSE_BASE_FRAME_IN_MARKER_TASK): { // vec. & acc. (task-dependent)

            tag_teaching_pose_ = current_task_list[task_param.task_step].tag_teaching_pose;
            pose_idx_teaching_pose_ = current_task_list[task_param.task_step].pose_idx_teaching_pose;

            //// TODO: is_task_teaching_mode_를 티칭 창에서 모드 선택할 수 있도록 처리
            if(!is_teaching_finished_ && is_task_teaching_mode_) {
                task_param.task_mode = WAIT_TIL_FIN_TEACHING_POSE_BASE_FRAME_IN_MARKER_TASK; // yh find
                is_in_task_teaching_stage_ = true; // GUI에서 버튼 활성화하기 위한 flag
                break;
            }
            is_in_task_teaching_stage_ = false; // GUI에서 버튼 비활성화하기 위한 flag

            ROS_LOG_WARN("[BASE FRAME TEACHING IN MARKER TASK] [Teaching Tag]: %s - %s\n", tag_teaching_pose_.c_str(), pose_idx_teaching_pose_.c_str());

            //// FILE PATH
            std::string pose_config_path = "/home/irl-sbc250j/[Pose server]/marker_pose/graphy_marker_initial_pose.json";
            //// JSON INPUT
            std::stringstream ss;
            std::ifstream ifs(pose_config_path.c_str());

            json j_in;
            try {
                j_in = json::parse(ifs);
            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("Retrying JSON PARSING...");
                ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());

                ////////////////////////
                //// retry
                j_in = json::parse(ifs);
                ////////////////////////
                ROS_LOG_ERROR("Retrying JSON - SUCCESS!");
            }


            //// M-1) Marker-based pose JSON Load
            std::vector<double> teaching_marker_pose_in;
            try {
                teaching_marker_pose_in = j_in[tag_teaching_pose_.c_str()][pose_idx_teaching_pose_.c_str()].get<std::vector<double>>();
                ROS_LOG_WARN("[BASE FRAME TEACHING IN MARKER TASK] [JSON LOAD] TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_marker_pose_in[0], teaching_marker_pose_in[1], teaching_marker_pose_in[2], teaching_marker_pose_in[3], teaching_marker_pose_in[4], teaching_marker_pose_in[5]);

            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("[BASE FRAME TEACHING IN MARKER TASK] Check JSON file! - loading error! (teaching pose loading failed)");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            //// M-2) Transform the pose w.r.t. base frame
            std::vector<double> input_pose_vec = teaching_marker_pose_in;
            CsDouble output_pose;
            for (int i = 0; i < CS_DOF; i++) {
                output_pose[i] = input_pose_vec[i];
            }
            ROS_LOG_WARN("[BASE FRAME TEACHING IN MARKER TASK] Marker-based Output CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", output_pose[0], output_pose[1], output_pose[2], output_pose[3], output_pose[4], output_pose[5]);

            //// 1) JSON Load Robot Workspace Limits
            ///////////////////////////////////////////////////////////////////
            Eigen::MatrixXd workspace(6,2); // pose limit - (min, max)
            std::vector<double> workspace_in;
            try {
                workspace_in = j_in[tag_teaching_pose_.c_str()]["workspace"].get<std::vector<double>>();
            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("Check JSON file! - loading error!");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            if(workspace_in.size() != 12) {
                ROS_LOG_ERROR("Check JSON file! - loading error!");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }
            for(int i=0; i<6; i++) {
                workspace(i, 0) = workspace_in[2 * i]; // min
                workspace(i, 1) = workspace_in[2 * i + 1]; // max
            }

            ///////////////////////////////////////////////////////////////////
            //// 2) JSON Load Teaching Pose
            std::vector<double> teaching_pose_in;
            try {
                arr2Vec(output_pose, teaching_pose_in);
                ROS_LOG_WARN("[BASE FRAME TEACHING IN MARKER TASK RESULTS] TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_pose_in[0], teaching_pose_in[1], teaching_pose_in[2], teaching_pose_in[3], teaching_pose_in[4], teaching_pose_in[5]);
            } catch (const std::exception & e) {
                std::cout << e.what() << std::endl;
                ROS_LOG_ERROR("[BASE FRAME TEACHING IN MARKER TASK RESULTS] Check JSON file! - loading error! (teaching pose loading failed)");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            //// 3) Check Robot Workspace
            if (checkWorkspace(teaching_pose_in, workspace)) {
                teaching_target_pose_ = teaching_pose_in;
                ROS_LOG_WARN("[BASE FRAME TEACHING IN MARKER TASK RESULTS] [QNode] Set Current TeachingTarget CS pose: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f\n", teaching_target_pose_[0], teaching_target_pose_[1], teaching_target_pose_[2], teaching_target_pose_[3], teaching_target_pose_[4], teaching_target_pose_[5]);
            } else {
                ROS_LOG_WARN("[BASE FRAME TEACHING IN MARKER TASK RESULTS] workspace[min]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f", workspace_in[0], workspace_in[2], workspace_in[4], workspace_in[6], workspace_in[8], workspace_in[10]);
                ROS_LOG_WARN("[BASE FRAME TEACHING IN MARKER TASK RESULTS] workspace[max]: %0.5f %0.5f %0.5f %0.1f %0.1f %0.1f", workspace_in[1], workspace_in[3], workspace_in[5], workspace_in[7], workspace_in[9], workspace_in[11]);
                ROS_LOG_ERROR("[BASE FRAME TEACHING IN MARKER TASK RESULTS] Robot Workspace Limit!");
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }

            is_teaching_finished_ = false; // flag init.
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;

        case (TASK_SEND_TASK_LOG): { // vec. & acc. (task-dependent)
            std::string task_log = current_task_list[task_param.task_step].task_log;
            sendTaskInfoLog(task_log);
            // setLogMessageStr(task_log);

            ////
            current_task_log_ = task_log;
            ROS_LOG_WARN("[TASK_SEND_TASK_LOG] current_task_log_: %s", current_task_log_.c_str());

            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;


        // sb 25.02.10
        case (TASK_INDEX_COUNT): {
            int index_= current_task_list[task_param.task_step].task_index_n;

            task_index_ = index_;
            // if (index_ == 1){
            //     task_index_ = 1;
            // } else if(index_ == 2){
            //     task_index_ = 2;
            // } else if(index_ == 3){
            //     task_index_ = 3;
            // } else if(index_ == 4){
            //     task_index_ = 4;
            // } else if(index_ == 5){
            //     task_index_ = 5;
            // } else if(index_ == 6){
            //     task_index_ = 6;
            // } else if(index_ == 7){
            //     task_index_ = 7;
            // } else {
            //     task_index_ = 0;
            // }

            // std::string task_log = current_task_list[task_param.task_step].task_log;
            // sendTaskInfoLog(task_log);
            // // setLogMessageStr(task_log);
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;



#if BIN_PICKING_FLAG
        //// NOTICE: [JSON 파일과 연동된 부분] JSON에 저장된 Demo 자세를 불러와서 할당
        case (TASK_JSON_POSE_TAG_CSMOVE): {
            double vel      = current_task_list[task_param.task_step].vel_cs;
            double acc      = current_task_list[task_param.task_step].acc_cs;
            bool   relative = current_task_list[task_param.task_step].relative;
            uint16_t demo_tag = current_task_list[task_param.task_step].bp_param.demo_tag;
            uint16_t motion_tag = current_task_list[task_param.task_step].bp_param.motion_tag;
            std::string demo_name = m_bin_picking_node->getDemoName(demo_tag);

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;

            // Set pose from JSON
            CsDouble x_target;
            std::vector<double> pose(6);
            if(m_bin_picking_node->setDemoJSONCSPose(pose, demo_name, motion_tag)) {
                ROS_LOG_INFO("JSON CS Pose Input Success!");
                CsDouble x_sum = current_task_list[task_param.task_step].x_target;
                vec2Arr(pose, x_target);
                x_target = task_planner_->arraySumCS(x_target, x_sum);
                printf("Target task CS pose: %0.4f %0.4f %0.4f %0.3f %0.3f %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
                MoveErrorType err_type = moveX(x_target, vel, acc, true, relative);
            } else {
                ROS_LOG_INFO("JSON CS Pose Input Failure!");
                ROS_LOG_ERROR("JSON CS Pose loading failure!\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
                task_param.task_mode = TASK_DEFAULT;
            }
        } break;

        case (TASK_RECOG_BLENDING): {
            BlendingTraj traj_blending = current_task_list[task_param.task_step].traj_blending;
            uint16_t demo_tag = current_task_list[task_param.task_step].bp_param.demo_tag;
            uint16_t motion_tag = current_task_list[task_param.task_step].bp_param.motion_tag;
            std::string demo_name = m_bin_picking_node->getDemoName(demo_tag);

            // current pose update
            std::vector<double> measured_x(6);
            arr2Vec(params_.meas.x , measured_x); // scanning joint position
            m_bin_picking_node->m_measured_pose = measured_x;

            // Set pose from JSON
            if(m_bin_picking_node->setBlendingPath(traj_blending, demo_name, motion_tag)) {
                ROS_LOG_INFO("Blending path generation success!");
                task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE_WITH_SCAN_AND_MATCHING;
                MoveErrorType err_type = moveBlending(traj_blending);
            } else {
                is_task_blending_path_infeasible_ = true;
                ROS_LOG_INFO("Blending path generation failure!");
                ROS_LOG_ERROR("Blending path generation failure!\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
                task_param.task_mode = TASK_DEFAULT;
            }

        } break;

        //// NOTICE: [Template matching 결과와 연동된 부분] 추정된 파지자세 또는 지그자세의 approach pose에 변위를 더한 후, CS move
        case (TASK_RECOG_POSE_TAG_CSMOVE): {
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   relative = current_task_list[task_param.task_step].relative;
            uint16_t demo_tag = current_task_list[task_param.task_step].bp_param.demo_tag;
            uint16_t motion_tag = current_task_list[task_param.task_step].bp_param.motion_tag;
            std::string demo_name = m_bin_picking_node->getDemoName(demo_tag);

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;

            // Set pose from JSON
            CsDouble x_target;
            std::vector<double> pose(6);
            if(m_bin_picking_node->setDemoRecogCSPose(pose, demo_name, motion_tag)) {
                ROS_LOG_INFO("Recog. CS Pose Input Success!");
                CsDouble x_sum = current_task_list[task_param.task_step].x_target;
                printf("Approach displacement: %0.4f %0.4f %0.4f %0.3f %0.3f %0.3f\n", x_sum[0], x_sum[1], x_sum[2], x_sum[3], x_sum[4], x_sum[5]);
                vec2Arr(pose, x_target);
                x_target = task_planner_->arraySumCS(x_target, x_sum);
                printf("Target task CS pose: %0.4f %0.4f %0.4f %0.3f %0.3f %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
                MoveErrorType err_type = moveX(x_target, vel, acc, true, relative);
            } else {
                ROS_LOG_INFO("Demo Recog. CS Pose Input Failure!");
                ROS_LOG_ERROR("Demo Recog.CS Pose loading failure!\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
                task_param.task_mode = TASK_DEFAULT;
            }
        } break;





        case (TASK_RECOG_TOOL_CHANGING_ATTACH_JSMOVE): {
            JsDouble q_target;
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> jointPosition = m_bin_picking_node->task_vec_set_tool_changing_attach_JS_position_[tool_changing_attach_id-1];
            if(jointPosition.size()==6 && tool_changing_attach_id > 0 && !is_tool_attached)  {
                setToolChangingCurrentIndex(tool_changing_attach_id); // Set Current Attached Tool Index
                is_tool_attached = true; // update
                vec2Arr(jointPosition, q_target);
                printf("Target task JS position (Tool Attaching %dth slave): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", tool_changing_attach_id, q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            } else {
                if(jointPosition.size() != 6) {
                    ROS_LOG_INFO("Check target JS position (tool attaching JS position)!");
                }
                if(tool_changing_attach_id == 0) {
                    ROS_LOG_INFO("Check Attaching Slave Index!");
                }
                if(is_tool_attached) {
                    ROS_LOG_INFO("Gripper is already attached!");
                }
            }
        } break;

        case (TASK_RECOG_TOOL_CHANGING_DETACH_JSMOVE): {
            JsDouble q_target;
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> jointPosition = m_bin_picking_node->task_vec_set_tool_changing_detach_JS_position_[tool_changing_detach_id-1];
            if(jointPosition.size()==6 && tool_changing_detach_id > 0 && is_tool_attached) {
                is_tool_attached = false; // update
                vec2Arr(jointPosition, q_target);
                printf("Target task JS position (Tool Detaching %dth slave): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", tool_changing_detach_id, q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            } else {
                if(jointPosition.size() != 6) {
                    ROS_LOG_INFO("Check target JS position (tool detaching JS position)!");
                }
                if(tool_changing_detach_id == 0) {
                    ROS_LOG_INFO("Check Detaching Slave Index!");
                }
                if(!is_tool_attached) {
                    ROS_LOG_INFO("Gripper is already detached!");
                }
            }
        } break;

        case (TASK_RECOG_CURRENT_TOOL_DETACHING_JSMOVE): {
            JsDouble q_target;
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> jointPosition = m_bin_picking_node->task_vec_set_tool_changing_detach_JS_position_[tool_changing_current_id-1];
            if(jointPosition.size()==6 && tool_changing_current_id > 0) {
                is_tool_attached = false; // update
                vec2Arr(jointPosition, q_target);
                printf("Target task JS position (Tool Detaching %dth slave): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", tool_changing_current_id, q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            } else {
                if(jointPosition.size() != 6) {
                    ROS_LOG_INFO("Check target JS position (tool detaching JS position)!");
                }
                if(tool_changing_current_id == 0) {
                    ROS_LOG_INFO("Check Current Detaching Slave Index!");
                }
            }
        } break;

        case (TASK_RECOG_TOOL_CHANGING_GOAL_CSMOVE): {
            CsDouble x_sum = current_task_list[task_param.task_step].x_target;
            double vel      = current_task_list[task_param.task_step].vel_cs;
            double acc      = current_task_list[task_param.task_step].acc_cs;
            // double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            // double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;
            bool   is_tool_attaching = current_task_list[task_param.task_step].is_tool_attaching;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> pose;
            if(is_tool_attaching) { // tool attaching
                pose = m_bin_picking_node->task_vec_tool_changing_goal_cs_pose_[tool_changing_attach_id-1];
                if(m_bin_picking_node->is_tool_left_side_[tool_changing_attach_id-1]) {
                    x_sum[0] = -fabs(x_sum[0]);
                } else {
                    x_sum[0] = fabs(x_sum[0]);
                }
            } else { // tool detaching
                pose = m_bin_picking_node->task_vec_tool_changing_goal_cs_pose_[tool_changing_detach_id-1];
                if(m_bin_picking_node->is_tool_left_side_[tool_changing_detach_id-1]) {
                    x_sum[0] = -fabs(x_sum[0]);
                } else {
                    x_sum[0] = fabs(x_sum[0]);
                }
            }
            if(pose.size()==6) {
                CsDouble x_in;
                vec2Arr(pose, x_in);
                CsDouble x_target = task_planner_->arraySumCS(x_in, x_sum);
                task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
                printf("Target task CS pose (Tool Goal - CS): %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
                MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
            } else {
                if(pose.size() != 6) {
                    ROS_LOG_INFO("Check target CS pose (tool goal CS pose)!");
                }
            }
        } break;

        case (TASK_RECOG_SELECT_ROBOT_TCP): {
            unsigned int tcp_idx = m_bin_picking_node->task_tcp_idx_set_tool_changing_attach_[tool_changing_attach_id-1];
            sendChangeRobotTCPCommand(tcp_idx);
            ROS_LOG_INFO("Set TCP Task! - TCP #%i", tcp_idx);
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;

        case (TASK_RECOG_TOOL_CHANGING_DEFAULT_ATTACH_JSMOVE): {
            JsDouble q_target;
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> jointPosition;
            if(m_bin_picking_node->is_tool_left_side_[tool_changing_attach_id-1]) {
                jointPosition = m_bin_picking_node->tc_robot_left_side_attach_default_js_position_;
            } else {
                jointPosition = m_bin_picking_node->tc_robot_right_side_attach_default_js_position_;
            }
            if(jointPosition.size()==6) {
                vec2Arr(jointPosition, q_target);
                printf("Target task JS position (Tool Attaching - Default JS): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            } else {
                if(jointPosition.size() != 6) {
                    ROS_LOG_INFO("Check target JS position (tool attaching Default JS position)!");
                }
            }
        } break;

        case (TASK_RECOG_TOOL_CHANGING_DEFAULT_DETACH_JSMOVE): {
            JsDouble q_target;
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> jointPosition;
            if(m_bin_picking_node->is_tool_left_side_[tool_changing_detach_id-1]) {
                jointPosition = m_bin_picking_node->tc_robot_left_side_detach_default_js_position_;
            } else {
                jointPosition = m_bin_picking_node->tc_robot_right_side_detach_default_js_position_;
            }
            if(jointPosition.size()==6) {
                vec2Arr(jointPosition, q_target);
                printf("Target task JS position (Tool Detaching - Default JS): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            } else {
                if(jointPosition.size() != 6) {
                    ROS_LOG_INFO("Check target JS position (tool detaching Default JS position)!");
                }
            }
        } break;


        //////////////////////// Tip Changing
        case (TASK_RECOG_TIP_CHANGING_ATTACH_JSMOVE): {
            JsDouble q_target;
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> jointPosition = m_bin_picking_node->task_vec_set_tip_changing_attach_JS_position_[tip_changing_attach_id-1];
            if(jointPosition.size()==6 && tip_changing_attach_id > 0 && !is_tip_attached) {
                is_tip_attached = true; // update
                setTipChangingCurrentIndex(tip_changing_attach_id);
                vec2Arr(jointPosition, q_target);
                printf("Target task JS position (Tip Attaching %dth slave): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", tip_changing_attach_id, q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            } else {
                if(jointPosition.size() != 6) {
                    ROS_LOG_INFO("Check target JS position (tip attaching JS position)!");
                }
                if(tip_changing_attach_id == 0) {
                    ROS_LOG_INFO("Check Attaching Tip Index!");
                }
                if(is_tip_attached) {
                    ROS_LOG_INFO("Tip is already attached!");
                }
            }
        } break;

        case (TASK_RECOG_TIP_CHANGING_DETACH_JSMOVE): {
            JsDouble q_target;
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> jointPosition = m_bin_picking_node->task_vec_set_tip_changing_detach_JS_position_[tip_changing_detach_id-1];
            if(jointPosition.size()==6 && tip_changing_detach_id > 0 && is_tip_attached) {
                is_tip_attached = false; // update
                vec2Arr(jointPosition, q_target);
                printf("Target task JS position (Tip Detaching %dth slave): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", tip_changing_detach_id, q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            } else {
                if(jointPosition.size() != 6) {
                    ROS_LOG_INFO("Check target JS position (tip detaching JS position)!");
                }
                if(tip_changing_detach_id == 0) {
                    ROS_LOG_INFO("Check Detaching Tip Index!");
                }
                if(!is_tip_attached) {
                    ROS_LOG_INFO("Tip is already detached!");
                }
            }
        } break;

        case (TASK_RECOG_CURRENT_TIP_CHANGING_DETACH_JSMOVE): {
            JsDouble q_target;
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> jointPosition = m_bin_picking_node->task_vec_set_tip_changing_detach_JS_position_[tip_changing_current_id-1];
            if(jointPosition.size()==6 && tip_changing_current_id > 0 && is_tip_attached) {
                is_tip_attached = false; // update
                vec2Arr(jointPosition, q_target);
                printf("Target task JS position (Current Tip Detaching %dth slave): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", tip_changing_current_id, q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            } else {
                if(jointPosition.size() != 6) {
                    ROS_LOG_INFO("Check target JS position (tip detaching JS position)!");
                }
                if(tip_changing_current_id == 0) {
                    ROS_LOG_INFO("Check Detaching Tip Index!");
                }
                if(!is_tip_attached) {
                    ROS_LOG_INFO("Tip is already detached!");
                }
            }
        } break;

        case (TASK_RECOG_TIP_CHANGING_DEFAULT_ATTACH_JSMOVE): {
            JsDouble q_target;
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> jointPosition = m_bin_picking_node->task_vec_tip_changing_default_attach_JS_position_;
            if(jointPosition.size()==6) {
                vec2Arr(jointPosition, q_target);
                printf("Target task JS position (Tip Attaching - Default JS): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            } else {
                if(jointPosition.size() != 6) {
                    ROS_LOG_INFO("Check target JS position (tip attaching Default JS position)!");
                }
            }
        } break;

        case (TASK_RECOG_TIP_CHANGING_DEFAULT_DETACH_JSMOVE): {
            JsDouble q_target;
            double vel      = current_task_list[task_param.task_step].vel_js;
            double acc      = current_task_list[task_param.task_step].acc_js;
            bool   relative = current_task_list[task_param.task_step].relative;
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> jointPosition = m_bin_picking_node->task_vec_tip_changing_default_detach_JS_position_;
            if(jointPosition.size()==6) {
                vec2Arr(jointPosition, q_target);
                printf("Target task JS position (Tip Detaching - Default JS): %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", q_target[0], q_target[1], q_target[2], q_target[3], q_target[4], q_target[5]);
                MoveErrorType err_type = moveQ(q_target, vel, acc, relative);
            } else {
                if(jointPosition.size() != 6) {
                    ROS_LOG_INFO("Check target JS position (tip detaching Default JS position)!");
                }
            }
        } break;

        case (TASK_RECOG_TIP_CHANGING_SET_ROBOT_TCP): {
            unsigned int tcp_idx = m_bin_picking_node->task_tcp_idx_set_tip_changing_attach_[tip_changing_current_id-1];
            sendChangeRobotTCPCommand(tcp_idx);
            ROS_LOG_INFO("Set TCP Task (Tip Changing Task)! - TCP #%i", tcp_idx);
            task_param.task_mode = GO_TO_NEXT_TASK;

        } break;
#endif

        case (TASK_RECOG_TIP_CHANGING_SET_CURRENT_TIP_INDEX): {
            size_t current_tip_idx = current_task_list[task_param.task_step].bp_param.current_tip_idx;
            ROS_LOG_WARN("TASK_RECOG_TIP_CHANGING_SET_CURRENT_TIP_INDEX (Tip Changing Task) - Tool Tip %zu", current_tip_idx);
            tip_changing_attach_id = current_tip_idx; // tip #1, 2, ...
            tip_changing_detach_id = current_tip_idx; // tip #1, 2, ...
            setTipChangingCurrentIndex(tip_changing_attach_id);
            task_param.task_mode = GO_TO_NEXT_TASK;

        } break;

        case (TASK_SELECT_ROBOT_TCP): {
            unsigned int tcp_idx = current_task_list[task_param.task_step].bp_param.tcp_idx;
            sendChangeRobotTCPCommand(tcp_idx);
            ROS_LOG_INFO("Set TCP Task! - TCP #%i", tcp_idx);
            task_param.task_mode = GO_TO_NEXT_TASK;
            // if(tcp_idx == robot_current_tcp_idx_) {
            //     sendChangeRobotTCPCommand(tcp_idx);
            //     ROS_LOG_INFO("Set TCP Task! - TCP #%i", tcp_idx);
            //     task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            // } else {
            //     ROS_LOG_INFO("TCP is already set! - current TCP #%i", robot_current_tcp_idx_);
            //     task_param.task_mode = GO_TO_NEXT_TASK;
            // }

        } break;

        case (TASK_DS_CUSTOM_CODE): {

            // flange_serial_ 부분 확인해서, 몇 자리까지 가져오고, cmd 전달
            /// cmd 종류에 따라 init, open, close, percent로 실행

            string script = current_task_list[task_param.task_step].task_log;
            drflSetGrp(script);
            taskFinFunction(0);
        } break;

        case (TASK_KORAS_GRIPPER_INITIALIZE): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_KORAS_GRIPPER_INITIALIZE");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            uint16_t position = 0;
            setGrp((uint16_t)KR_GRP::INIT, position, grp_driver_address_);
            ROS_LOG_INFO("Initialize Gripper Task! - Driver #%i", grp_driver_address_);
            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
            // params_.status.is_gripper_finished = true;
        } break;

        case (TASK_KORAS_GRIPPER_INITIALIZE_2): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_KORAS_GRIPPER_INITIALIZE_2");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            uint16_t position = grp_init_2_grp_cmd_;
            if(grp_do_initialize_) {
                setGrp((uint16_t)KR_GRP::INIT_2, position, grp_driver_address_);
                ROS_LOG_WARN("Initialize_2 Gripper Task! [grp_init_2_grp_cmd_: %u] - Driver #%i", grp_init_2_grp_cmd_, grp_driver_address_);
                task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
                // params_.status.is_gripper_finished = true;
            } else {
                task_param.task_mode = GO_TO_NEXT_TASK;
            }
        } break;

        case (TASK_KORAS_GRIPPER_COMMAND): {
#if !IS_SCANNING_AND_GRASPING_MODE
                ROS_LOG_WARN("SKIP - IS_SCANNING_AND_GRASPING_MODE : FALSE");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
#endif
                ROS_LOG_WARN("[NEW VERSION] TASK_KORAS_GRIPPER_COMMAND");
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_KORAS_GRIPPER_COMMAND");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            uint16_t grp_cmd = current_task_list[task_param.task_step].bp_param.gripper_command;
            int16_t position = current_task_list[task_param.task_step].bp_param.gripper_param[0]; // 0~10000
            // uint16_t duration = current_task_list[task_param.task_step].bp_param.gripper_param[1]; // duration;
            // if(position > 10000) {
            //     position = 10000;
            // } else if(position < 0) {
            //     position = 0;
            // }

            if(!grp_is_vacuum_) {


#if NEW_VER_KORAS_GRIPPER_PACKAGE

                // 변경 그리퍼 패키지 ver.2
                //// 변경 (한양이엔지 용)
                if(grp_cmd == 104) {
                    ROS_LOG_WARN("[NEW VERSION] TASK_KORAS_GRIPPER_COMMAND: GRP POS CTRL!");
                    setGrp(grp_cmd, position, grp_driver_address_);
                } else if(grp_cmd == 5) { // MOTOR POS CTRL, duration: 600
                    ROS_LOG_WARN("[NEW VERSION] TASK_KORAS_GRIPPER_COMMAND: MOTOR POSITION CTRL!");
                    setGrpMotorCtrl(grp_cmd, position, grp_driver_address_);
                } else if(grp_cmd == 1) { // MOTOR ENABLE
                    ROS_LOG_WARN("[NEW VERSION] TASK_KORAS_GRIPPER_COMMAND: MOTOR ENABLE!");
                    setGrpMotorEnable();
                } else if(grp_cmd == 8) { // MOTOR RESET POSE
                    ROS_LOG_WARN("[NEW VERSION] TASK_KORAS_GRIPPER_COMMAND: MOTOR RESET POSE!");
                    setGrpMotorResetPose();
                } else if(grp_cmd == (uint16_t)KR_GRP::INIT) { // Initialize
                    ROS_LOG_WARN("[NEW VERSION] TASK_KORAS_GRIPPER_COMMAND: setGrpInitialize!");
                    ROS_LOG_WARN("[NEW VERSION] TASK_KORAS_GRIPPER_COMMAND: setGrpInitialize!");
                    ROS_LOG_WARN("[NEW VERSION] TASK_KORAS_GRIPPER_COMMAND: setGrpInitialize!");
                    setGrpInitialize();
                } else if(grp_cmd == 102) { // Initialize
                    setGrpOpen();
                } else if(grp_cmd == 103) { // Initialize
                    setGrpClose();
                } else if(grp_cmd == 106) { // Initialize
                    setGrpVacOn();
                } else if(grp_cmd == 107) { // Initialize
                    setGrpVacOff();
                } else {
                    setGrp(grp_cmd, position, grp_driver_address_);
                }

#else
                //// 기존 그리퍼 패키지 ver.1
                if(setGrp(grp_cmd, position, grp_driver_address_)) {
                    ROS_LOG_WARN("[Default] Gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    if(IS_GRIPPER_COMM_RS485_) {
                        task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
                    } else { //
                        // task_param.task_mode = WAIT_TIL_FIN_GRIPPER_delay_100; // sb 25.02.03, FR 그리퍼 제어의 경우, 딜레이 추가
                        task_param.task_mode = WAIT_TIL_FIN_GRIPPER; // mw 25.03.17, 딜레이 task_planner에서 직접 넣도록 변경
                    }
                } else {
                    ROS_LOG_WARN("[TASK_KORAS_GRIPPER_COMMAND] re-command (setGrp)");
                    break;
                }

#endif

            ROS_LOG_WARN("[Default] Gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
            } else { // 흡착 파지
                if(grp_cmd == (uint16_t)KR_GRP::INIT) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::INIT_2) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::POS_CTRL) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::OPEN) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::CLOSE) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else {
#if NEW_VER_KORAS_GRIPPER_PACKAGE
                    if(grp_cmd == (uint16_t)KR_GRP::VACUUM_ON) { // Initialize
                        ROS_LOG_WARN("[Vacuum ON]Gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                        setGrpVacOn();
                    } else if(grp_cmd == (uint16_t)KR_GRP::VACUUM_OFF) { // Initialize
                        ROS_LOG_WARN("[Vacuum OFF]Gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                        setGrpVacOff();
                    }
#else
                    setGrp(grp_cmd, position, grp_driver_address_);
#endif

                    ROS_LOG_WARN("[Vacuum]Gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
                }
            }

        } break;

        case (TASK_KORAS_GRIPPER_COMMAND_FOR_TEACHING): {

            if(!is_task_teaching_mode_) {
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            uint16_t grp_cmd = current_task_list[task_param.task_step].bp_param.gripper_command;
            uint16_t position = current_task_list[task_param.task_step].bp_param.gripper_param[0]; // 0~10000
            // uint16_t duration = current_task_list[task_param.task_step].bp_param.gripper_param[1]; // duration;
            if(position > 10000) {
                position = 10000;
            } else if(position < 0) {
                position = 0;
            }

            if(!grp_is_vacuum_) {
            setGrp(grp_cmd, position, grp_driver_address_);
            ROS_LOG_WARN("[Default] Gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
            } else { // 흡착 파지
                if(grp_cmd == (uint16_t)KR_GRP::INIT) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::INIT_2) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::POS_CTRL) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::OPEN) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::CLOSE) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else {
                    setGrp(grp_cmd, position, grp_driver_address_);
                    ROS_LOG_WARN("[Vacuum]Gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
                }
            }

        } break;

        case (TASK_KORAS_GRIPPER_COMMAND_TOOL_DETACHING): {
            uint16_t grp_cmd = current_task_list[task_param.task_step].bp_param.gripper_command;
            uint16_t position = current_task_list[task_param.task_step].bp_param.gripper_param[0]; // 0~10000
            // uint16_t duration = current_task_list[task_param.task_step].bp_param.gripper_param[1]; // duration;
            if(position > 10000) {
                position = 10000;
            } else if(position < 0) {
                position = 0;
            }

            if(!grp_is_vacuum_before_) {
                setGrp(grp_cmd, position, grp_driver_address_);
                ROS_LOG_WARN("[Default] Gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
            } else { // 흡착 파지
                if(grp_cmd == (uint16_t)KR_GRP::INIT) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::INIT_2) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::POS_CTRL) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::OPEN) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else if(grp_cmd == (uint16_t)KR_GRP::CLOSE) {
                    ROS_LOG_WARN("[Vacuum] Skip gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = GO_TO_NEXT_TASK;
                } else {
                    setGrp(grp_cmd, position, grp_driver_address_);
                    ROS_LOG_WARN("[Vacuum]Gripper Command: %u! - Driver #%i", grp_cmd, grp_driver_address_);
                    task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
                }
            }

        } break;

        case (TASK_KORAS_SET_GRIPPER_DRIVER): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_KORAS_SET_GRIPPER_DRIVER");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            if(grp_driver_address_ != current_task_list[task_param.task_step].bp_param.grp_driver_idx) {
                grp_driver_address_ = current_task_list[task_param.task_step].bp_param.grp_driver_idx;
                setGrp((uint16_t)KR_GRP::CHANGE_ADDRESS, 0, grp_driver_address_);
            }
            params_.status.is_gripper_finished = true;

            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
        } break;

        case (TASK_RECOG_KORAS_GRIPPER_CHECK_GRASPING): {
            uint16_t target_position = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_open_length; // 0~10000

            //// Checking value between the measured position and the target position of the gripper
            ROS_LOG_WARN("target_position: %u //// grp_meas_pos_: %u ", target_position, grp_meas_pos_);
            int l_val_check = static_cast<int>(target_position / 10.0) - grp_meas_pos_;
            ROS_LOG_WARN("l_val_check: %i", l_val_check);

            int l_thre_check = 50;
            if(l_val_check > l_thre_check || l_val_check < -l_thre_check) {
                ROS_LOG_WARN("Gripper Target Position Limit Over!");

                //// Stop robot
                is_task_mode_ = false;
                stopRobot();

                std::stringstream ss_log;
                ss_log << "[ERROR - GRIPPER POSITION LIMIT] Check the gripper state!";
                std::string task_log = ss_log.str();
                sendTaskErrorLog(task_log);

                ////////////////////////////////////////////////////////////////////////////
                //// 기존
                // task_param.task_mode = TASK_DEFAULT;
                // break;

                //// 변경: Gripper 명령 계속 보내도록 처리


                current_task_list[task_param.task_step].cnt_grp_pos_cmd++;

                if(current_task_list[task_param.task_step].cnt_grp_pos_cmd < 2) { // 그리퍼 열기 명령 2번 반복
                    current_task_list[task_param.task_step].time_10ms_between_tasks = 100; // 1s
                    ROS_LOG_WARN("***");
                    ROS_LOG_WARN("RE-COMMAND (GRIPPER), target position: %u", target_position);
                    ROS_LOG_WARN("[Grasping mode] TASK_RECOG_KORAS_GRIPPER_COMMAND - gripper_open_length: %i", target_position);
                    ROS_LOG_WARN("***");
                    ROS_LOG_WARN("...");
                    setGrp((uint16_t)KR_GRP::POS_CTRL, target_position, grp_driver_address_);
                } else { // 그리퍼 열기가 먹통이면 이니셜라이즈
                    current_task_list[task_param.task_step].cnt_grp_pos_cmd = 0;
                    current_task_list[task_param.task_step].time_10ms_between_tasks = 500; // 4s
                    setGrpInitialize();
                    ROS_LOG_WARN("\n\n******************************************************");
                    ROS_LOG_WARN("RE-COMMAND (GRIPPER), INITIALIZE!");
                    ROS_LOG_WARN("******************************************************\n\n");
                }
                task_param.task_mode = TASK_GRIPPER_CHECK_DELAY_BETWEEN_TASKS;
                ////////////////////////////////////////////////////////////////////////////
            } else {
                ROS_LOG_WARN("Gripper Target Position Check OK!");
                task_param.task_mode = GO_TO_NEXT_TASK;
            }
        } break;





#if BIN_PICKING_FLAG
        // KORAS Gripper
        case (TASK_RECOG_KORAS_GRIPPER_COMMAND): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_RECOG_KORAS_GRIPPER_COMMAND");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            uint16_t grp_cmd = current_task_list[task_param.task_step].bp_param.gripper_command;
            uint16_t position = 0; // 0~10000
            //// NOTICE: 토픽 결과가 빠르게 들어오면 current_target_object_idx_가 바뀌므로 이전 값을 사용
            if(is_stacking_mode_) {
                ROS_LOG_WARN("[Stacking mode] TASK_RECOG_KORAS_GRIPPER_COMMAND - gripper_open_length: %i", position);
                position = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->before_target_object_idx_]->m_grasping_parameter.gripper_open_length; // 0~10000
            } else {
                ROS_LOG_WARN("[Grasping mode] TASK_RECOG_KORAS_GRIPPER_COMMAND - gripper_open_length: %i", position);
                position = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_open_length; // 0~10000
            }
            if(grp_cmd != 5) {
                // if(position > 10000) position = 10000;
                // else if(position < 0) position = 0;
            }
            setGrp(grp_cmd, position, grp_driver_address_);
            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
            // params_.status.is_gripper_finished = true;
        } break;

        case (TASK_KORAS_SET_GRIPPER_MIN_VALUE): {
            ROS_LOG_INFO("Set min grip length Task! - Driver #%i", grp_driver_address_);
            m_bin_picking_node->grp_initial_min_position[grp_driver_address_-1] = m_bin_picking_node->grp_measured_position[grp_driver_address_-1];
            m_bin_picking_node->is_grp_set_min_value_finished[grp_driver_address_-1] = true;
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;

        case (TASK_KORAS_SET_GRIPPER_MAX_VALUE): {
            ROS_LOG_INFO("Set max grip length Task! - Driver #%i", grp_driver_address_);
            m_bin_picking_node->grp_initial_max_position[grp_driver_address_-1] = m_bin_picking_node->grp_measured_position[grp_driver_address_-1];
            m_bin_picking_node->is_grp_set_max_value_finished[grp_driver_address_-1] = true;
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;


        case (TASK_START_STACKING_MODE): {
            ROS_LOG_INFO("TASK_START_STACKING_MODE");
            is_stacking_mode_ = true;
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;


        case (TASK_STOP_STACKING_MODE): {
            ROS_LOG_INFO("TASK_STOP_STACKING_MODE");
            is_stacking_mode_ = false;
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;


        ///////////////////////////////////////// TASK SCANNING AND Matching
                //// 3D scanning
        case (TASK_3D_SCANNING): {
            if(m_bin_picking_node->getMatchingProcessStatus()) {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Matching (topic) done !!!!!!!!!!!!!!!!!!!!");
                task_param.task_mode = GO_TO_NEXT_TASK;
            } else {
                if(!m_bin_picking_node->is_initial_scan_done) {
                    taskScanningParameter scan_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter;
            // scan_parameter.scan_position = getRobotAActualQ();
            arr2Vec(params_.meas.q , scan_parameter.scan_position); // scanning joint position
            m_bin_picking_node->is_initial_scan_done = true;
            if(m_bin_picking_node->doScanning(scan_parameter)) {
                if (params_.status.is_detection_finished == false && is_task_mode_) {
                    params_.status.is_detection_finished = true;
                    task_param_.is_ready_matching = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRASPING;
           } else {
                    ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Matching in progress... (TASK_3D_SCANNING)!!!!!!!!!!!!!!!!!!!!");
                }
            }
        } break;

        //// Process 3D scanning and matching
        case (TASK_3D_SCANNING_AND_MATCHING): {
            ROS_LOG_WARN("m_bin_picking_node->m_ptr_bin_picking_node_target_object_list: %zu", m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.size());
            ROS_LOG_WARN("m_bin_picking_node->current_target_object_idx_: %zu", m_bin_picking_node->current_target_object_idx_);
            // Resolve target index from object name and validate before dereferencing
            if (m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.empty()) {
                ROS_LOG_ERROR("[TASK_3D_SCANNING_AND_MATCHING] Target object list is empty");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            std::string object_name_local = current_task_list[task_param.task_step].object_name;
            size_t resolved_idx = getTargetObjectIndex(object_name_local);
            if (resolved_idx == std::numeric_limits<size_t>::max() ||
                resolved_idx >= m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.size()) {
                ROS_LOG_ERROR("[TASK_3D_SCANNING_AND_MATCHING] Invalid target index for %s", object_name_local.c_str());
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            m_bin_picking_node->current_target_object_idx_ = resolved_idx;

            taskScanningParameter scan_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[resolved_idx]->m_scan_parameter;
            if (object_name_local == "right_drum_key_code" || object_name_local == "right_drum_holder_with_key_code") {
                scan_parameter.target_name = object_name_local;
                m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[resolved_idx]->m_scan_parameter.target_name = object_name_local;
            }
            // scan_parameter.scan_position = getRobotAActualQ(); // scanning joint position
            arr2Vec(params_.meas.q , scan_parameter.scan_position); // scanning joint position
            scan_parameter.do_scan_sampling = current_task_list[task_param.task_step].bp_param.detection.do_scan_sampling;
            taskTemplateMatchingParameter matching_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_matching_parameter;
            matching_parameter.do_scan_sampling = current_task_list[task_param.task_step].bp_param.detection.do_scan_sampling;
            if(m_bin_picking_node->doScanningAndMatching(scan_parameter, matching_parameter)) {
                if (params_.status.is_detection_finished == false && is_task_mode_) {
                    params_.status.is_detection_finished = true;
                    task_param_.is_ready_matching = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRASPING;
        } break;

        //// 3D scanning
        case (TASK_3D_SCANNING_SHARED_TASK_NO_WAITING): {
            ROS_LOG_WARN("[(No waiting)TASK_3D_SCANNING_SHARED_TASK_NO_WAITING] Target part id: %d", m_bin_picking_node->current_target_object_idx_); // task planner
            // m_bin_picking_node->current_target_object_idx_ = task_target_id;

            // Initialize flag
            is_scanning_and_detection_finished = false;
            is_open3d_processing_finished = false;

            taskScanningParameter scan_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter;

            //// 241125 추가
            scan_parameter.robot_dh_vec = getRobotDHParameters(); // [m], [deg]
            scan_parameter.robot_tcp_default = getRobotDefaultTCP(); // [m], [deg]
            scan_parameter.robot_tcp = getRobotTCP(); // [m], [deg]

            arr2Vec(params_.meas.q , scan_parameter.scan_position); // scanning joint position
            std::vector<double> scan_position_q = scan_parameter.scan_position;
            ROS_LOG_WARN("Current JS position [deg]: %0.2f, %0.2f, %0.2f, %0.2f, %0.2f, %0.2f", scan_position_q[0], scan_position_q[1], scan_position_q[2], scan_position_q[3], scan_position_q[4], scan_position_q[5]);


            taskTemplateMatchingParameter template_parameter;
            template_parameter.target_id = scan_parameter.target_id;
            template_parameter.target_name = scan_parameter.target_name;
            template_parameter.robot_dh_vec = getRobotDHParameters();
            template_parameter.robot_tcp_default = getRobotDefaultTCP();
            template_parameter.robot_tcp = getRobotTCP();

            std::vector<double> robot_dh_vec = template_parameter.robot_dh_vec; // [m], [deg]
            std::vector<double> robot_tcp_default = template_parameter.robot_tcp_default; // [m], [deg]
            std::vector<double> robot_tcp = template_parameter.robot_tcp; // [m], [deg]
            printf("\n\n\nDH: ");
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 4; j++)
                {
                    printf("%0.6f ", robot_dh_vec[i * 4 + j]);
                }
                printf("\n");
            }

            ROS_LOG_WARN("[QNODE] Get DH, TCP");
            ROS_LOG_INFO("\nRobot TCP default[m, deg]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f", robot_tcp_default[0], robot_tcp_default[1], robot_tcp_default[2], robot_tcp_default[3], robot_tcp_default[4], robot_tcp_default[5]);
            ROS_LOG_INFO("\nRobot TCP[m, deg]: %0.6f, %0.6f, %0.6f, %0.3f, %0.3f, %0.3f", robot_tcp[0], robot_tcp[1], robot_tcp[2], robot_tcp[3], robot_tcp[4], robot_tcp[5]);
            ROS_LOG_WARN("[QNODE] Get DH, TCP");


            m_bin_picking_node->doTemplateInitialize(template_parameter, true, true);
            //// Matching information update
            setSamInfoUpdateCurrentObject(); // cloud clear and parameters update
            ROS_LOG_WARN("THIS 2");

            ROS_LOG_WARN("THIS 3");

            if(m_bin_picking_node->doScanning(scan_parameter)) {
                if (params_.status.is_detection_finished == false && is_task_mode_) {
                            params_.status.is_detection_finished = true;
                            task_param_.is_ready_matching = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRASPING;

        } break;


        //// 3D scanning
        case (TASK_3D_SCANNING_TARGET_OBJECT_NO_WAITING): { // object 바로 입력

#if DRFL_CONTROL
            /////////////////////////////////////////////////
            getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
            /////////////////////////////////////////////////
#endif
            ////////////////////////////////////////////////////////////////////////
            std::string object_name = current_task_list[task_param.task_step].object_name;
            ROS_LOG_WARN("[TASK_3D_SCANNING_SHARED_TASK] Target object name: %s", object_name.c_str()); // task planner
            m_bin_picking_node->current_target_object_idx_ = getTargetObjectIndex(object_name);
            if (m_bin_picking_node->current_target_object_idx_ == std::numeric_limits<size_t>::max() ||
                m_bin_picking_node->current_target_object_idx_ >= m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.size()) {
                ROS_LOG_ERROR("[TASK_3D_SCANNING_SHARED_TASK_NO_WAITING] Invalid target index for %s", object_name.c_str());
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            ////////////////////////////////////////////////////////////////////////


            // Initialize flag
            is_scanning_and_detection_finished = false;
            is_open3d_processing_finished = false;

            taskScanningParameter scan_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter;

            //// 241125 추가
            scan_parameter.robot_dh_vec = getRobotDHParameters(); // [m], [deg]
            scan_parameter.robot_tcp_default = getRobotDefaultTCP(); // [m], [deg]
            scan_parameter.robot_tcp = getRobotTCP(); // [m], [deg]

            arr2Vec(params_.meas.q , scan_parameter.scan_position); // scanning joint position

            taskTemplateMatchingParameter template_parameter;
            template_parameter.target_id = scan_parameter.target_id;
            template_parameter.target_name = scan_parameter.target_name;
            template_parameter.robot_dh_vec = getRobotDHParameters();
            template_parameter.robot_tcp_default = getRobotDefaultTCP();
            template_parameter.robot_tcp = getRobotTCP();
            m_bin_picking_node->doTemplateInitialize(template_parameter, true, true);

            //// Matching information update
            setSamInfoUpdateCurrentObject(); // cloud clear and parameters update

            ROS_LOG_WARN("[TASK_3D_SCANNING_SHARED_TASK] THIS 1");

            if(m_bin_picking_node->doScanning(scan_parameter)) {
                if (params_.status.is_detection_finished == false && is_task_mode_) {
                            params_.status.is_detection_finished = true;
                            task_param_.is_ready_matching = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRASPING; // No waiting

        } break;


        //// 3D scanning
        case (TASK_3D_SCANNING_TARGET_OBJECT): { // object 바로 입력

#if !IS_SCANNING_AND_GRASPING_MODE
            ROS_LOG_WARN("SKIP - IS_SCANNING_AND_GRASPING_MODE : FALSE");
            task_param.task_mode = GO_TO_NEXT_TASK;
            break;
#endif

#if DRFL_CONTROL
            /////////////////////////////////////////////////
            getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
            /////////////////////////////////////////////////
#endif
            ////////////////////////////////////////////////////////////////////////
            std::string object_name = current_task_list[task_param.task_step].object_name;
            ROS_LOG_WARN("[TASK_3D_SCANNING_SHARED_TASK] Target object name: %s", object_name.c_str()); // task planner
            m_bin_picking_node->current_target_object_idx_ = getTargetObjectIndex(object_name);
            if (m_bin_picking_node->current_target_object_idx_ == std::numeric_limits<size_t>::max() ||
                m_bin_picking_node->current_target_object_idx_ >= m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.size()) {
                ROS_LOG_ERROR("[TASK_3D_SCANNING_SHARED_TASK] Invalid target index for %s", object_name.c_str());
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            ////////////////////////////////////////////////////////////////////////

            // Initialize flag
            is_scanning_and_detection_finished = false;
            is_open3d_processing_finished = false;

            taskScanningParameter scan_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter;
            if (object_name == "right_drum_key_code") {
                scan_parameter.target_name = object_name;
                m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.target_name = object_name;
            } else if (object_name == "right_drum_holder_with_key_code") {
                scan_parameter.target_name = object_name;
                m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.target_name = object_name;
            } else if (object_name == "right_drum_holder_without_key_code") {
                scan_parameter.target_name = object_name;
                m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.target_name = object_name;
            } else if (object_name == "right_drum_key_code_unscrewing") {
                scan_parameter.target_name = object_name;
                m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.target_name = object_name;
            }

            //// 241125 추가
            scan_parameter.robot_dh_vec = getRobotDHParameters(); // [m], [deg]
            scan_parameter.robot_tcp_default = getRobotDefaultTCP(); // [m], [deg]
            scan_parameter.robot_tcp = getRobotTCP(); // [m], [deg]

            arr2Vec(params_.meas.q , scan_parameter.scan_position); // scanning joint position

            taskTemplateMatchingParameter template_parameter;
            template_parameter.target_id = scan_parameter.target_id;
            template_parameter.target_name = scan_parameter.target_name;
            template_parameter.robot_dh_vec = getRobotDHParameters();
            template_parameter.robot_tcp_default = getRobotDefaultTCP();
            template_parameter.robot_tcp = getRobotTCP();
            m_bin_picking_node->doTemplateInitialize(template_parameter, true, true);

            //// Matching information update
            setSamInfoUpdateCurrentObject(); // cloud clear and parameters update

            ROS_LOG_WARN("[TASK_3D_SCANNING_SHARED_TASK] THIS 1");

            if(m_bin_picking_node->doScanning(scan_parameter)) {
                if (params_.status.is_detection_finished == false && is_task_mode_) {
                            params_.status.is_detection_finished = true;
                            task_param_.is_ready_matching = true;
                }
            }
            // task_param.task_mode = WAIT_TIL_FIN_GRASPING; // no waiting
            task_param.task_mode = WAIT_TIL_FIN_GRASPING_SCANNING_SHARED_TASK; // waiting, 241017, hd_llm

        } break;

        case (TASK_MATCHING_TARGET_OBJECT): { // CAD Template matching
#if !IS_SCANNING_AND_GRASPING_MODE
            ROS_LOG_WARN("SKIP - IS_SCANNING_AND_GRASPING_MODE : FALSE");
            task_param.task_mode = GO_TO_NEXT_TASK;
            break;
#endif
            if (task_param_.is_ready_matching) {

#if DRFL_CONTROL
                /////////////////////////////////////////////////
                getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
                /////////////////////////////////////////////////
#endif
                ////////////////////////////////////////////////////////////////////////
                std::string object_name = current_task_list[task_param.task_step].object_name;
                ROS_LOG_WARN("[TASK_3D_SCANNING_SHARED_TASK] Target object name: %s", object_name.c_str()); // task planner
                m_bin_picking_node->current_target_object_idx_ = getTargetObjectIndex(object_name);
                ////////////////////////////////////////////////////////////////////////

                std::string demo_name = current_task_list[task_param.task_step].demo_name;

                taskTemplateMatchingParameter parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_matching_parameter;
                parameter.debug_mode = true;
                parameter.is_base_frame_unknown = false;
                parameter.do_scan_sampling = current_task_list[task_param.task_step].bp_param.detection.do_scan_sampling;

                parameter.robot_dh_vec = getRobotDHParameters();
                parameter.robot_tcp_default = getRobotDefaultTCP();
                parameter.robot_tcp = getRobotTCP();

                parameter.is_symmetric = current_task_list[task_param.task_step].is_symmetric;

                parameter.target_name = object_name; // 250902

                if(m_bin_picking_node->doCADMatchingService(parameter)) {
                    if (params_.status.is_detection_finished == false && is_task_mode_) {
                        params_.status.is_detection_finished = true;
                    }
                } else {
                    if (params_.status.is_detection_finished == false && is_task_mode_) {
                        params_.status.is_detection_finished = true;
                    }
                }
                ROS_LOG_INFO("Template matching service received!!!");
                task_param_.is_ready_matching = false;

            }
            task_param.task_mode = WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK;
        } break;

        //// 3D scanning
        case (TASK_3D_SCANNING_SHARED_TASK): {
#if DRFL_CONTROL
            /////////////////////////////////////////////////
            getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
            /////////////////////////////////////////////////
#endif
            ROS_LOG_WARN("[TASK_3D_SCANNING_SHARED_TASK] Target part id: %d", m_bin_picking_node->current_target_object_idx_); // task planner
            // m_bin_picking_node->current_target_object_idx_ = task_target_id;

            // Initialize flag
            is_scanning_and_detection_finished = false;
            is_open3d_processing_finished = false;

            taskScanningParameter scan_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter;

            //// 241125 추가
            scan_parameter.robot_dh_vec = getRobotDHParameters(); // [m], [deg]
            scan_parameter.robot_tcp_default = getRobotDefaultTCP(); // [m], [deg]
            scan_parameter.robot_tcp = getRobotTCP(); // [m], [deg]

            arr2Vec(params_.meas.q , scan_parameter.scan_position); // scanning joint position

            taskTemplateMatchingParameter template_parameter;
            template_parameter.target_id = scan_parameter.target_id;
            template_parameter.target_name = scan_parameter.target_name;
            template_parameter.robot_dh_vec = getRobotDHParameters();
            template_parameter.robot_tcp_default = getRobotDefaultTCP();
            template_parameter.robot_tcp = getRobotTCP();
            m_bin_picking_node->doTemplateInitialize(template_parameter, true, true);

            //// Matching information update
            setSamInfoUpdateCurrentObject(); // cloud clear and parameters update

            ROS_LOG_WARN("[TASK_3D_SCANNING_SHARED_TASK] THIS 1");

            if(m_bin_picking_node->doScanning(scan_parameter)) {
                if (params_.status.is_detection_finished == false && is_task_mode_) {
                            params_.status.is_detection_finished = true;
                            task_param_.is_ready_matching = true;
                }
            }
            // task_param.task_mode = WAIT_TIL_FIN_GRASPING;
            task_param.task_mode = WAIT_TIL_FIN_GRASPING_SCANNING_SHARED_TASK; // 241017

        } break;

        case (TASK_MATCHING_SHARED_TASK): { // CAD Template matching
            if (task_param_.is_ready_matching) {
#if DRFL_CONTROL
                /////////////////////////////////////////////////
                getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
                /////////////////////////////////////////////////
#endif

                std::string demo_name = current_task_list[task_param.task_step].demo_name;

                taskTemplateMatchingParameter parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_matching_parameter;
                parameter.debug_mode = true;
                parameter.is_base_frame_unknown = false;
                parameter.do_scan_sampling = current_task_list[task_param.task_step].bp_param.detection.do_scan_sampling;

                parameter.robot_dh_vec = getRobotDHParameters();
                parameter.robot_tcp_default = getRobotDefaultTCP();
                parameter.robot_tcp = getRobotTCP();
                
                parameter.is_symmetric = current_task_list[task_param.task_step].is_symmetric;
                if(m_bin_picking_node->doCADMatchingService(parameter)) {
                    if (params_.status.is_detection_finished == false && is_task_mode_) {
                        params_.status.is_detection_finished = true;
                    }
                } else {
                    if (params_.status.is_detection_finished == false && is_task_mode_) {
                        params_.status.is_detection_finished = true;
                    }
                }
                ROS_LOG_INFO("Template matching service received!!!");
                task_param_.is_ready_matching = false;
            }
            task_param.task_mode = WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK;
        } break;

        case (TASK_3D_SCANNING_AND_MATCHING_SHARED_TASK): { // CAD Template matching

            if (!task_param_.is_ready_matching && m_bin_picking_node->is_grasping_pose_cal_finished) {
                ROS_LOG_WARN("[TASK_3D_SCANNING_AND_MATCHING_SHARED_TASK] Target part id: %d", m_bin_picking_node->current_target_object_idx_); // task planner
                // m_bin_picking_node->current_target_object_idx_ = task_target_id;



                taskScanningParameter scan_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter;

                taskTemplateMatchingParameter template_parameter;
                template_parameter.target_id = scan_parameter.target_id;
                template_parameter.target_name = scan_parameter.target_name;
                template_parameter.robot_dh_vec = getRobotDHParameters();
                template_parameter.robot_tcp_default = getRobotDefaultTCP();
                template_parameter.robot_tcp = getRobotTCP();
                m_bin_picking_node->doTemplateInitialize(template_parameter, true, true);

                //// Matching information update
                setSamInfoUpdateCurrentObject(); // cloud clear and parameters update

                arr2Vec(params_.meas.q , scan_parameter.scan_position); // scanning joint position

                if(m_bin_picking_node->doScanning(scan_parameter)) {
                    if (params_.status.is_detection_finished == false && is_task_mode_) {
                        // params_.status.is_detection_finished = true;
                        task_param_.is_ready_matching = true;
                    }
                }

            }


            if(task_param_.is_ready_matching && !m_bin_picking_node->is_grasping_pose_cal_finished) {

                std::string demo_name = current_task_list[task_param.task_step].demo_name;

                taskTemplateMatchingParameter parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_matching_parameter;
                parameter.debug_mode = true;
                parameter.is_base_frame_unknown = false;
                parameter.do_scan_sampling = current_task_list[task_param.task_step].bp_param.detection.do_scan_sampling;

                parameter.robot_dh_vec = getRobotDHParameters();
                parameter.robot_tcp_default = getRobotDefaultTCP();
                parameter.robot_tcp = getRobotTCP();
                if(m_bin_picking_node->doCADMatchingService(parameter)) {
                    if (params_.status.is_detection_finished == false && is_task_mode_) {
                        params_.status.is_detection_finished = true;
                    }
                }
                ROS_LOG_INFO("Template matching service received!!!");
                task_param_.is_ready_matching = false;
                task_param.task_mode = WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK;
            }

        } break;


        //// Shared Task - Grasping - ZIVID Scanning & Matching with task recognition
        case (TASK_RECOG_ZIVID_SCANNING_AND_MATCHING): {
            ROS_LOG_WARN("[TASK_RECOG_ZIVID_SCANNING_AND_MATCHING] Target part id: %d", m_bin_picking_node->current_target_object_idx_); // task planner
            // m_bin_picking_node->current_target_object_idx_ = task_target_id;

            //// Matching information update
            setSamInfoUpdateCurrentObject(); // cloud clear and parameters update

            ROS_LOG_WARN("Target part id(m_bin_picking_node->current_target_object_idx_): %zu", m_bin_picking_node->current_target_object_idx_); // task planner
            taskScanningParameter scan_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter;
            scan_parameter.scan_position = getRobotAActualQ(); // scanning joint position
            arr2Vec(params_.meas.q , scan_parameter.scan_position); // scanning joint position
            scan_parameter.do_scan_sampling = current_task_list[task_param.task_step].bp_param.detection.do_scan_sampling;
            taskTemplateMatchingParameter matching_parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_matching_parameter;
            matching_parameter.do_scan_sampling = current_task_list[task_param.task_step].bp_param.detection.do_scan_sampling;
            if(m_bin_picking_node->doScanningAndMatching(scan_parameter, matching_parameter)) {
                if (params_.status.is_detection_finished == false && is_task_mode_) {
                    params_.status.is_detection_finished = true;
                    task_param_.is_ready_matching = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRASPING;


        } break;


        //// Initial 3D scanning and matching
        case (TASK_CHECK_MATCHING_FINISHED): {
            if(m_bin_picking_node->getMatchingProcessStatus()) {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Matching (topic) done !!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Matching (topic) done !!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Matching (topic) done !!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Matching (topic) done !!!!!!!!!!!!!!!!!!!!");
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Matching (topic) done !!!!!!!!!!!!!!!!!!!!");
                m_bin_picking_node->ui_flag_is_initial_scan_done = true;
                //// Flag in blending
                m_bin_picking_node->is_blending_scan_and_matching_conducted = false;

                //// 24.02.12 추가
                m_bin_picking_node->is_check_matching_finished = true;

                task_param.task_mode = GO_TO_NEXT_TASK;
            } else {
                // ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Matching in progress... !!!!!!!!!!!!!!!!!!!!");
                //// TODO: 매칭 또는 자세 산출 실패한 경우에 다시 스캔 및 매칭 수행하도록 처리
                //// TODO: 매칭 또는 자세 산출 실패한 경우에 다시 스캔 및 매칭 수행하도록 처리
                //// TODO: 매칭 또는 자세 산출 실패한 경우에 다시 스캔 및 매칭 수행하도록 처리
            }
        } break;
        ///////////////////////////////////////// TASK SCANNING AND Matching

        //// Initial 3D scanning and matching
        case (TASK_CHECK_MATCHING_FINISHED_AND_TIP_CHANGING): {
            if(m_bin_picking_node->getMatchingProcessStatus()) {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Matching (topic) done !!!!!!!!!!!!!!!!!!!!");
                //// Tip은 초기에 장착되어 있는 것으로 가정
                //// Tip은 초기에 장착되어 있는 것으로 가정
                //// Tip은 초기에 장착되어 있는 것으로 가정
                if(m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.is_tip_changing_applied) {
                    if(is_tip_attached) {
                        ROS_LOG_INFO("Current gripper tip index: #%u", tip_changing_current_id);
                        ROS_LOG_INFO("Next gripper tip index: #%u", m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_tip_index);
                        tip_changing_detach_id = tip_changing_current_id; // tip #1, 2, ...
                        tip_changing_attach_id = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_tip_index; // tip #1, 2, ...
                        //// Tip attaching task
                        if(tip_changing_current_id != m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_tip_index) {
                            ROS_LOG_INFO("!!!!!!!!!!!! Tip Changing from tip #%u to tip #%u !!!!!!!!!!!!", tip_changing_detach_id, tip_changing_attach_id);
                            m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.do_tip_changing_for_grasping = true;
                        } else {
                            ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! No tip changing !!!!!!!!!!!!!!!!!!!!");
                            m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.do_tip_changing_for_grasping = false;
                        }
                        task_param.task_mode = GO_TO_NEXT_TASK;
                    } else {
                        ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Tip should be attached first !!!!!!!!!!!!!!!!!!!!");
                    }
                } else {
                    task_param.task_mode = GO_TO_NEXT_TASK;
                }
            } else {
                ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!! Matching in progress... !!!!!!!!!!!!!!!!!!!!");
            }
        } break;

        case (TASK_SET_TIP_CHANGING_FLAG): {
            tip_changing_task_flag_ = current_task_list[task_param.task_step].bp_param.tip_changing_task_flag;
            if(tip_changing_task_flag_) {
                ROS_LOG_INFO("Tip Changing Flag ON!");
            } else {
                ROS_LOG_INFO("Tip Changing Flag OFF!");
            }
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;

        ///////////////////////////////////////// TASK Pose estimation
        case (TASK_POSE_ESTIMATION_CAD_MATCHING): { // CAD Template matching
            // Topic으로 수행된 경우 skip
            if(m_bin_picking_node->getMatchingProcessStatus()) {
                task_param.task_mode = GO_TO_NEXT_TASK;
            } else { // 기존 서비스
                if (task_param_.is_ready_matching) {
                    taskTemplateMatchingParameter parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_matching_parameter;
                    parameter.do_scan_sampling = current_task_list[task_param.task_step].bp_param.detection.do_scan_sampling;
                    if(m_bin_picking_node->doCADMatchingService(parameter)) {
                        if (params_.status.is_detection_finished == false && is_task_mode_) {
                            params_.status.is_detection_finished = true;
                        }
                    }
                    ROS_LOG_INFO("Template matching service received!!!");
                    task_param_.is_ready_matching = false;
                }
                task_param.task_mode = WAIT_TIL_FIN_GRASPING;
            }
        } break;

        case (TASK_POSE_ESTIMATION_AI): {  // AI estimation
            if (task_param_.is_ready_matching) {
                taskTemplateMatchingParameter parameter = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_matching_parameter;
                parameter.do_scan_sampling = current_task_list[task_param.task_step].bp_param.detection.do_scan_sampling;
                parameter.object_name = "default";
                if(m_bin_picking_node->doPoseEstimationAI(parameter)) {
                    if (params_.status.is_detection_finished == false && is_task_mode_) {
                        params_.status.is_detection_finished = true;
                    }
                }
                ROS_LOG_INFO("Pose Estimation srv received!!!");
                task_param_.is_ready_matching = false;
            }
            task_param.task_mode = WAIT_TIL_FIN_GRASPING;
        } break;

        //// Grasping
        case (TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING): {
            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_PLC_MODBUS_COMMAND");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> pose;
            if(m_bin_picking_node->doGrasping(pose, GraspingPoseType::OPTIMAL_POSE)) {
                if (!relative) {
                    CsDouble x_target;
                    vec2Arr(pose, x_target);
                    MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
                    printf("Target Grasping Pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
                }
            }
        } break;

        //// Grasping
        case (TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING_SHARED_TASK): {

#if !IS_SCANNING_AND_GRASPING_MODE
            ROS_LOG_WARN("SKIP - IS_SCANNING_AND_GRASPING_MODE : FALSE");
            task_param.task_mode = GO_TO_NEXT_TASK;
            break;
#endif

            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            std::string demo_name      = current_task_list[task_param.task_step].demo_name;
            std::string object_name      = current_task_list[task_param.task_step].object_name;

            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;

            if(demo_name == "drum_task") {
                ROS_LOG_WARN("TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING_DRUM_TASK");
                ROS_LOG_WARN("TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING_DRUM_TASK");
                ROS_LOG_WARN("TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING_DRUM_TASK");
            }

            std::vector<double> pose;
            if(m_bin_picking_node->doGrasping(pose, GraspingPoseType::OPTIMAL_POSE, demo_name, object_name)) {
                if (!relative) {
                    CsDouble x_target;
                    vec2Arr(pose, x_target);

                    if(demo_name == "pc_part") {
                        ROS_LOG_WARN("demo_name == pc_part // Rx, Ry, Rz, 180.0 0.0 90.0 fixed");
                        ROS_LOG_WARN("demo_name == pc_part // Rx, Ry, Rz, 180.0 0.0 90.0 fixed");
                        ROS_LOG_WARN("demo_name == pc_part // Rx, Ry, Rz, 180.0 0.0 90.0 fixed");
                        ROS_LOG_WARN("demo_name == pc_part // Rx, Ry, Rz, 180.0 0.0 90.0 fixed");
                        ROS_LOG_WARN("demo_name == pc_part // Rx, Ry, Rz, 180.0 0.0 90.0 fixed");
                        ROS_LOG_WARN("demo_name == pc_part // Rx, Ry, Rz, 180.0 0.0 90.0 fixed");
                        x_target[3] = 180.0;
                        x_target[4] = 0.0;
                        x_target[5] = 90.0;
                    }

                    MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
                    printf("Target Grasping Pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
                }
            }
        } break;

        case (TASK_GRASPING_SUB_OPTIMAL_POSE_BIN_PICKING): {
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> pose;
            if(m_bin_picking_node->doGrasping(pose, GraspingPoseType::SUB_OPTIMAL_POSE)) {
                if (!relative) {
                    CsDouble x_target;
                    vec2Arr(pose, x_target);
                    MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
                    printf("Target Grasping Pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
                }
            }
        } break;

        case (TASK_GRASPING_AI_ESTIMATED_POSE_BIN_PICKING): {
            double vel      = current_task_list[task_param.task_step].vel_cs_custom;
            double acc      = current_task_list[task_param.task_step].acc_cs_custom;
            bool   is_base_frame = true;
            bool   relative = current_task_list[task_param.task_step].relative;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            std::vector<double> pose;
            if(m_bin_picking_node->doGrasping(pose, GraspingPoseType::AI_ESTIMATED_POSE)) {
                if (!relative) {
                    CsDouble x_target;
                    vec2Arr(pose, x_target);
                    MoveErrorType err_type = moveX(x_target, vel, acc, is_base_frame, relative);
                    printf("Target Grasping Pose: %0.6f %0.6f %0.6f %0.3f %0.3f %0.3f\n", x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);
                }
            }
        } break;

        case (TASK_DETACH_COUNTING): {
            size_t l_idx_now = m_bin_picking_node->before_target_object_idx_;

            if(m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->detaching_cnt_ < m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->max_cnt_detaching_ - 1) {
                m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->detaching_cnt_++;
            } else {
                m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->detaching_cnt_ = 0;
                m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->stacking_z_idx_ = 0;
            }
            ROS_LOG_WARN("[before_target_object_idx_: #%zu ]Current detaching count(maximum count: %zu): %zu", l_idx_now, m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->max_cnt_detaching_, m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[l_idx_now]->detaching_cnt_);
            taskFinFunction(10);
        } break;
#endif

        //// PLC MODBUS
        case (TASK_PLC_MODBUS_COMMAND): {

#if !IS_PLC_COMMUNICATION
            task_param.task_mode = GO_TO_NEXT_TASK;
            break;
#endif
            if(is_plc_simulation_mode_) {
                ROS_LOG_INFO("[PLC Simulation Mode] skiped task: TASK_PLC_MODBUS_COMMAND");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_PLC_MODBUS_COMMAND");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            int plc_command = current_task_list[task_param.task_step].bp_param.plc_command;
            if(setPLCModbusCommand(plc_command)) {
                if (params_.status.is_gripper_finished == false && is_task_mode_) {
                    params_.status.is_gripper_finished = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
        } break;

        case (TASK_PLC_MODBUS_COMMAND_WRITE_REGISTER): {

#if !IS_PLC_COMMUNICATION
            task_param.task_mode = GO_TO_NEXT_TASK;
            break;
#endif
            if(is_plc_simulation_mode_) {
                ROS_LOG_INFO("[PLC Simulation Mode] skiped task: TASK_PLC_MODBUS_COMMAND_WRITE_REGISTER");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_PLC_MODBUS_COMMAND");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            int plc_command = current_task_list[task_param.task_step].bp_param.plc_command;
            int plc_write_data = current_task_list[task_param.task_step].bp_param.plc_write_data;

            if(setPLCModbusCommandWriteRegister(plc_command, plc_write_data)) {
                if (params_.status.is_gripper_finished == false && is_task_mode_) {
                    params_.status.is_gripper_finished = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
        } break;



        case (TASK_PLC_INITIALIZE_STATUS_FROM_ROBOT_TO_AMR): {
            uint16_t register_address = current_task_list[task_param.task_step].register_address;
            bool is_lsb = current_task_list[task_param.task_step].is_lsb;
            bool status = current_task_list[task_param.task_step].status;

            ROS_LOG_INFO("[%s] TASK_PLC_INITIALIZE_STATUS_FROM_ROBOT_TO_AMR : Word: %u, (%s) -> %s",
                __func__, register_address, is_lsb ? "LSB" : "MSB", status ? "ON" : "OFF");

            if(is_plc_simulation_mode_) {
                ROS_LOG_INFO("[PLC Simulation Mode] skiped task: TASK_PLC_INITIALIZE_STATUS_FROM_ROBOT_TO_AMR");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_PLC_INITIALIZE_STATUS_FROM_ROBOT_TO_AMR");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            if(PLCModbusInitializeStatusFromRobotToAMR(register_address, is_lsb, 0, status)) {
                if (params_.status.is_gripper_finished == false && is_task_mode_) {
                    params_.status.is_gripper_finished = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
        } break;

        case (TASK_PLC_WRITE_STATUS_FROM_ROBOT_TO_AMR): {

#if !IS_PLC_COMMUNICATION
            task_param.task_mode = GO_TO_NEXT_TASK;
            break;
#endif

            // uint16_t register_address = static_cast<uint16_t>(current_task_list[task_param.task_step].register_address);
            uint16_t register_address = current_task_list[task_param.task_step].register_address;
            bool is_lsb = current_task_list[task_param.task_step].is_lsb;
            // uint16_t bit_address = static_cast<uint16_t>(current_task_list[task_param.task_step].bit_address);
            uint16_t bit_address = current_task_list[task_param.task_step].bit_address;
            bool status = current_task_list[task_param.task_step].status;

            ROS_LOG_INFO("[%s] TASK_PLC_WRITE_STATUS_FROM_ROBOT_TO_AMR : Word: %u, Bit %d (%s) -> %s",
                __func__, register_address, bit_address, is_lsb ? "LSB" : "MSB", status ? "ON" : "OFF");


            //// WRITE FLAG UPDATA - QNODE에서 현재 WRITE한 FLAG를 저장
            if(register_address == 1000 || register_address == 0x41000) {
                plc_status_word_41000_[bit_address] = status;
            }
            if(register_address == 1002 || register_address == 0x41002) {
                plc_status_word_41002_[bit_address] = status;
            }

            if(is_plc_simulation_mode_) {
                ROS_LOG_INFO("[PLC Simulation Mode] skiped task: TASK_PLC_WRITE_STATUS_FROM_ROBOT_TO_AMR");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_PLC_WRITE_STATUS_FROM_ROBOT_TO_AMR");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            if(PLCModbusWriteStatusFromRobotToAMR(register_address, is_lsb, bit_address, status)) {
                if (params_.status.is_gripper_finished == false && is_task_mode_) {
                    params_.status.is_gripper_finished = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
        } break;

        case (TASK_PLC_WRITE_ROTATION_ANGLE_FROM_ROBOT_TO_AMR): {
            uint16_t register_address = current_task_list[task_param.task_step].register_address;
            uint16_t bit_address = current_task_list[task_param.task_step].bit_address;
            double angle = current_task_list[task_param.task_step].angle;

            ROS_LOG_INFO("[PLC ANGLE] queued angle = %.3f  |  drum_rotation_angle_ = %.3f",
                angle, drum_rotation_angle_);


                
            if(angle < -1000.0f) {
                ROS_LOG_WARN("DRUM ROTATION ANGLE DETECTION RESULT!");
                angle = drum_rotation_angle_;
            }

            ROS_LOG_INFO("[%s] TASK_PLC_WRITE_ROTATION_ANGLE_FROM_ROBOT_TO_AMR : Word: %u, Bit %d, Angle: %0.1f",
                __func__, register_address, bit_address, angle);

            if(is_plc_simulation_mode_) {
                ROS_LOG_INFO("[PLC Simulation Mode] skiped task: TASK_PLC_WRITE_ROTATION_ANGLE_FROM_ROBOT_TO_AMR");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_PLC_WRITE_ROTATION_ANGLE_FROM_ROBOT_TO_AMR");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            if(setPLCModbusRotationAngle(register_address, bit_address, angle)) {
                if (params_.status.is_gripper_finished == false && is_task_mode_) {
                    params_.status.is_gripper_finished = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
        } break;

        case (TASK_PLC_WRITE_BARCODE_FROM_ROBOT_TO_AMR): {

            if(is_plc_simulation_mode_) {
                ROS_LOG_INFO("[PLC Simulation Mode] skiped task: TASK_PLC_WRITE_BARCODE_FROM_ROBOT_TO_AMR");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }

            if(is_simul_mode_) {
                ROS_LOG_INFO("[Simulation Mode] skiped task: TASK_PLC_WRITE_BARCODE_FROM_ROBOT_TO_AMR");
                task_param.task_mode = GO_TO_NEXT_TASK;
                break;
            }
            uint16_t register_address = current_task_list[task_param.task_step].register_address;
            uint16_t bit_address = current_task_list[task_param.task_step].bit_address;
            std::string barcode = current_task_list[task_param.task_step].barcode;

            if(setPLCModbusBarcodeWrite(register_address, bit_address, barcode)) {
                if (params_.status.is_gripper_finished == false && is_task_mode_) {
                    params_.status.is_gripper_finished = true;
                }
            }
            task_param.task_mode = WAIT_TIL_FIN_GRIPPER;
        } break;


        case (TASK_PLC_DO_MONITORING_FLAG_ON_OFF): {
            // is_plc_task_mode_monitoring_ = current_task_list[task_param.task_step].mode_on;
            // is_monitoring_plc_read_amr_to_robot_status_ = current_task_list[task_param.task_step].mode_on;
            is_plc_task_in_progress_ = current_task_list[task_param.task_step].mode_on;
            if(is_plc_task_in_progress_) {
                ROS_LOG_WARN("is_plc_task_in_progress_ = false");
                ROS_LOG_WARN("is_plc_task_in_progress_ = false");
                ROS_LOG_WARN("is_plc_task_in_progress_ = false");
            } else {
                ROS_LOG_WARN("is_plc_task_in_progress_ = true");
                ROS_LOG_WARN("is_plc_task_in_progress_ = true");
                ROS_LOG_WARN("is_plc_task_in_progress_ = true");
            }
            taskFinFunction(0);
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;

        case(TASK_SET_FLAG_ON_ROBOT_ENABLE_DISABLE_MODE): {
            is_task_robot_disable_and_enable = true;
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;

        case(TASK_SET_FLAG_OFF_ROBOT_ENABLE_DISABLE_MODE): {
            is_task_robot_disable_and_enable = false;
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;

        case(TASK_ROBOT_ENABLE): {
            if(is_task_robot_disable_and_enable) {
                setEnable(true);
            }
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;

        case(TASK_ROBOT_DISABLE): {
            if(is_task_robot_disable_and_enable) {
                setEnable(false);
            }
            task_param.task_mode = GO_TO_NEXT_TASK;
        } break;


        case (GO_TO_NEXT_TASK): {
                taskFinFunction(0);
                ROS_LOG_INFO("GO_TO_NEXT_TASK!");
        } break;

        case (WAIT_TIL_FIN_GRIPPER): {
            if (params_.status.is_gripper_finished) {
                params_.status.is_gripper_finished = false;
                taskFinFunction(10);
            }
        } break;

        case (WAIT_TIL_FIN_GRIPPER_delay_100): {

            if (params_.status.is_gripper_finished) {
                params_.status.is_gripper_finished = false;
                taskFinFunction(100);
            }
        } break;

        case (WAIT_TIL_FIN_GRASPING): {
            if (params_.status.is_detection_finished) {
                params_.status.is_detection_finished = false;
                taskFinFunction(10);
            }
        } break;

        case (WAIT_TIL_FIN_TEACHING): {
            if(is_teaching_finished_) {
                ROS_LOG_WARN("WAIT_TIL_FIN_TEACHING to TASK_SET_TEACHING_TARGET_POSE!!! // Teaching finished!");
                task_param.task_mode = TASK_SET_TEACHING_TARGET_POSE;
            } else {
                // ROS_LOG_WARN("WAIT_TIL_FIN_TEACHING");
            }
        } break;

        case (WAIT_TIL_FIN_TEACHING_WITH_MARKER_DETECTION): {
            if(is_teaching_finished_) {
                ROS_LOG_WARN("WAIT_TIL_FIN_TEACHING_WITH_MARKER_DETECTION to TASK_SET_TEACHING_TARGET_POSE_WITH_MARKER_DETECTION!!! // Teaching finished!");
                task_param.task_mode = TASK_SET_TEACHING_TARGET_POSE_WITH_MARKER_DETECTION;
            } else {
                // ROS_LOG_WARN("WAIT_TIL_FIN_TEACHING_WITH_MARKER_DETECTION");
            }
        } break;

        case (WAIT_TIL_FIN_TEACHING_POSE_BASE_FRAME_IN_MARKER_TASK): {
            if(is_teaching_finished_) {
                ROS_LOG_WARN("WAIT_TIL_FIN_TEACHING_WITH_MARKER_DETECTION to TASK_SET_TEACHING_TARGET_POSE_BASE_FRAME_IN_MARKER_TASK!!! // Teaching finished!");
                task_param.task_mode = TASK_SET_TEACHING_TARGET_POSE_BASE_FRAME_IN_MARKER_TASK;
            } else {
                // ROS_LOG_WARN("WAIT_TIL_FIN_TEACHING_WITH_MARKER_DETECTION");
            }
        } break;
        
        case (WAIT_TIL_FIN_GRASPING_SCANNING_SHARED_TASK): {
            // For key code scanning, proceed when scanning is done without waiting open3d
            std::string object_name = current_task_list[task_param.task_step].object_name;
            if(object_name == "right_drum_key_code") {
                if (is_scanning_and_detection_finished) {
                    params_.status.is_detection_finished = false;
                    is_scanning_and_detection_finished = false;
                    is_open3d_processing_finished = false;
                    taskFinFunction(10);
                }
                break;
            }

            if(is_scanning_and_detection_finished && is_open3d_processing_finished) {
                if (params_.status.is_detection_finished) {
                    params_.status.is_detection_finished = false;
                    is_scanning_and_detection_finished = false;
                    is_open3d_processing_finished = false;
                    taskFinFunction(10);
                }
            }

        } break;

        case (WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK): {
            if (params_.status.is_detection_finished) {
                if(m_bin_picking_node->is_grasping_pose_cal_finished) {
                    std::string demo_name = current_task_list[task_param.task_step].demo_name;
                    if(m_bin_picking_node->checkMatchingResult(GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                        //// 매칭 성공하면 다음 작업으로
                        ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING SHARED_TASK - SUCCESS!!!");
                        ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING SHARED_TASK - SUCCESS!!!");
                        ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING SHARED_TASK - SUCCESS!!!");
                        ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING SHARED_TASK - SUCCESS!!!");
                        ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING SHARED_TASK - SUCCESS!!!");
                        ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING SHARED_TASK - SUCCESS!!!");
                        ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING SHARED_TASK - SUCCESS!!!");
                        ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING SHARED_TASK - SUCCESS!!!");
                        ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING SHARED_TASK - SUCCESS!!!");
                        ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING SHARED_TASK - SUCCESS!!!");
                        params_.status.is_detection_finished = false;

                        ////////////////////////////////////////////////////////////////////////////
                        ////  250507, drum rotation angle detection
                        std::string object_name = current_task_list[task_param.task_step].object_name;

                        if(object_name == "drum_lid_total") {
                            //// Base Angle Rz: +-180deg
                            ROS_LOG_WARN("*************************************************");
                            ROS_LOG_WARN("Rotation Angle Detection Start...");
                            ROS_LOG_WARN("*************************************************");
                            
                            std::vector<double> detected_pose = m_bin_picking_node->getDetectedPose();
                            ROS_LOG_INFO("[Rotation Angle Detection] pose: %0.4f %0.4f %0.4f %0.1f %0.1f %0.1f", detected_pose[0], detected_pose[1], detected_pose[2], detected_pose[3], detected_pose[4], detected_pose[5]);


                            //// TODO: angle difference
                            drum_rotation_angle_ = 180.0 - detected_pose[5];

                            if(drum_rotation_angle_ > 180.0) {
                                drum_rotation_angle_ -= 360.0;
                            }

                            ROS_LOG_WARN("*************************************************");
                            ROS_LOG_WARN("Rotation Angle Detection Finished!");
                            ROS_LOG_WARN("Rotation Angle: %0.1f", drum_rotation_angle_);
                            ROS_LOG_WARN("*************************************************");
                        }



                        
                        ////////////////////////////////////////////////////////////////////////////


                        taskFinFunction(10);

                    } else {
                        ROS_LOG_ERROR("WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK - FAILURE 1!!!");
                        params_.status.is_detection_finished = false;
                        task_param_.is_ready_matching = true;

                        m_bin_picking_node->is_grasping_pose_cal_finished = false; // Topic 종료 여부
                        m_bin_picking_node->is_matching_finished = false; // Topic 종료 여부
                        m_bin_picking_node->is_grasping_pose_feasible_ = false; // doGrasping에 사용

                        // task_param.task_mode = TASK_MATCHING_SHARED_TASK;
                        task_param.task_mode = TASK_MATCHING_TARGET_OBJECT;
                        
                    }

                } else {
                    ROS_LOG_ERROR("WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK - FAILURE 2!!!");
                    params_.status.is_detection_finished = false;
                    task_param_.is_ready_matching = true;

                    m_bin_picking_node->is_grasping_pose_cal_finished = false; // Topic 종료 여부
                    m_bin_picking_node->is_matching_finished = false; // Topic 종료 여부
                    m_bin_picking_node->is_grasping_pose_feasible_ = false; // doGrasping에 사용

                    // task_param.task_mode = TASK_MATCHING_SHARED_TASK;
                    task_param.task_mode = TASK_MATCHING_TARGET_OBJECT;
                }

                // if(m_bin_picking_node->is_grasping_pose_cal_finished && m_bin_picking_node->is_grasping_pose_feasible_) {

                //     std::string demo_name = current_task_list[task_param.task_step].demo_name;
                //     if(m_bin_picking_node->checkMatchingResult(GraspingPoseType::OPTIMAL_POSE, demo_name)) {
                //         //// 매칭 성공하면 다음 작업으로
                //         ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK - SUCCESS!!!");
                //         params_.status.is_detection_finished = false;
                //         taskFinFunction(10);
                //     } else {
                //         ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK - FAILURE!!!");
                //         params_.status.is_detection_finished = false;
                //         task_param_.is_ready_matching = true;


                //         m_bin_picking_node->is_grasping_pose_cal_finished = false; // Topic 종료 여부
                //         m_bin_picking_node->is_matching_finished = false; // Topic 종료 여부
                //         m_bin_picking_node->is_grasping_pose_feasible_ = false; // doGrasping에 사용

                //         task_param.task_mode = TASK_MATCHING_SHARED_TASK;
                //     }

                // } else if(m_bin_picking_node->is_grasping_pose_cal_finished && !m_bin_picking_node->is_grasping_pose_feasible_) {
                //     ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK - FAILURE!!!");
                //     params_.status.is_detection_finished = false;
                //     task_param_.is_ready_matching = true;

                //     m_bin_picking_node->is_grasping_pose_cal_finished = false; // Topic 종료 여부
                //     m_bin_picking_node->is_matching_finished = false; // Topic 종료 여부
                //     m_bin_picking_node->is_grasping_pose_feasible_ = false; // doGrasping에 사용

                //     task_param.task_mode = TASK_MATCHING_SHARED_TASK;
                // }
            } else {
                ROS_LOG_WARN("WAIT_TIL_FIN_GRASPING_MATCHING_SHARED_TASK - is_detection_finished: false!!!");
            }
        } break;
        //////////////////////////// Bin Picking related /////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////// Apriltag related ///////////////////////////////////

        case TASK_START_TAG_RECOG: {
            tag_info_vec_temp_.clear();
            flag_tag_recog_.store(true);
            taskFinFunction(0);
        } break;

        case TASK_END_SAVE_TAG_RECOG: {
            flag_tag_recog_.store(false);
            if (saveTagInfo()) {
                taskFinFunction(0);
            } else {
                // task_param.task_mode = TASK_PAUSE;
                // task_param.task_mode = GO_TO_NEXT_TASK; // 마커 PC

                task_param.task_mode = TASK_START_TAG_RECOG; // 마커 인식에 실패한 경우, 다시 인식 시작하도록
                ROS_LOG_WARN("Detecting marker...");
            }
        } break;

        case TASK_SET_CURRENT_TAG: {
            if (setCurrentTag(current_task_list[task_param.task_step].tag_name)) {
                taskFinFunction(0);
            } else {
                ROS_LOG_ERROR("Check Tag Name! - Unknown Tag [%s]! ", current_task_list[task_param.task_step].tag_name.c_str());
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }
        } break;

        case TASK_SET_CURRENT_TAG_FROM_JSON: {
            if (setCurrentTagFromJSON(current_task_list[task_param.task_step].tag_name)) {
                taskFinFunction(0);
            } else {
                ROS_LOG_ERROR("Check Tag Name! - Unknown Tag [%s]! ", current_task_list[task_param.task_step].tag_name.c_str());
                is_task_mode_ = false;
                stopRobot();
                task_param.task_mode = TASK_DEFAULT;
                break;
            }
        } break;

        case TASK_CSMOVE_TAG_BASED: {
            ModelMatrix pose_tag_target(6, 1), pose_base_target(6, 1);

            for (int i = 0; i < 3; i++) {
                pose_tag_target.element_[i]     = current_task_list[task_param.task_step].x_target_tag_based[i];
                pose_tag_target.element_[i + 3] = current_task_list[task_param.task_step].x_target_tag_based[i + 3];
            }

            pose_base_target = ModelMatrix::tr2pose(tag_info_current_.tr_base_tag * ModelMatrix::pose2tr(pose_tag_target));

            CsDouble x_target;

            for (int i = 0; i < 3; i++) {
                x_target[i]     = pose_base_target.element_[i];
                x_target[i + 3] = pose_base_target.element_[i + 3];
            }

            ROS_LOG_WARN("Target pose: %f     %f     %f     %f     %f     %f"
                , x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);

            double vel = current_task_list[task_param.task_step].vel_cs;
            double acc = current_task_list[task_param.task_step].acc_cs;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, true, false);
        } break;

        case TASK_CSMOVE_TO_TAG: {
            ModelMatrix tr_base_camera(6, 1), pose_camera_tag(6, 1), tr_camera_tag(4, 4), pose_base_target(6, 1);

            tr_base_camera = tag_info_current_.tr_base_tag
                             * ModelMatrix::invTrMat(tag_info_current_.tr_camera_tag);

            pose_camera_tag = tag_info_current_.pose_camera_tag;
            ROS_LOG_WARN("[pose_camera_tag]: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                , pose_camera_tag.element_[0], pose_camera_tag.element_[1], pose_camera_tag.element_[2], pose_camera_tag.element_[3], pose_camera_tag.element_[4], pose_camera_tag.element_[5]);

            uint distance_mm = current_task_list[task_param.task_step].des_tag_distance_mm;

            ModelMatrix pose_tag_180deg;

#if CAM_TYPE_REALSENSE_D405
                if (distance_mm == 0) {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, pose_camera_tag.element_[2], 180, 0, 0});
                } else {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, (double) distance_mm / 1000, 180, 0, 0});
                }

#endif

#if CAM_TYPE_LOGITEC_BRIO
                // if (distance_mm == 0) {
                //     pose_tag_180deg = ModelMatrix(6, 1, {0, 0, pose_camera_tag.element_[2], 180, 0, 180}); // 카메라 브라켓 뒤집힘
                // } else {
                //     pose_tag_180deg = ModelMatrix(6, 1, {0, 0, (double) distance_mm / 1000, 180, 0, 180}); // 카메라 브라켓 뒤집힘
                // }

                //// tag도 같이 뒤집음
                if (distance_mm == 0) {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, pose_camera_tag.element_[2], 180, 0, 0}); // 카메라 브라켓 뒤집힘
                } else {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, (double) distance_mm / 1000, 180, 0, 0}); // 카메라 브라켓 뒤집힘
                }
#endif
            ROS_LOG_WARN("[pose_tag_180deg]: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                , pose_tag_180deg.element_[0], pose_tag_180deg.element_[1], pose_tag_180deg.element_[2], pose_tag_180deg.element_[3], pose_tag_180deg.element_[4], pose_tag_180deg.element_[5]);

            tr_camera_tag = ModelMatrix::pose2tr(pose_camera_tag);

            pose_base_target = ModelMatrix::tr2pose(tr_base_camera * tr_camera_tag
                                                    * ModelMatrix::pose2tr(pose_tag_180deg) * tr_camera_ee_);

            CsDouble x_target;

            for (int i = 0; i < 6; i++) {
                x_target[i] = pose_base_target.element_[i];
            }

            ModelMatrix temp = ModelMatrix::tr2pose(tr_camera_tag * ModelMatrix::pose2tr(pose_tag_180deg));
            ROS_LOG_WARN("pose_camera_tag_180: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                , temp.element_[0], temp.element_[1], temp.element_[2], temp.element_[3], temp.element_[4], temp.element_[5]);

            ROS_LOG_WARN("Target pose: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                , x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);

            double vel = current_task_list[task_param.task_step].vel_cs;
            double acc = current_task_list[task_param.task_step].acc_cs;

            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            MoveErrorType err_type = moveX(x_target, vel, acc, true, false);
        } break;

        case TASK_MARKER_DETECTION_FOR_APPROACHING: {

            ////////////////////////////////////////////////////////////////////////////////////////
            //// 1) TASK_START_TAG_RECOG
            if(!is_tag_received_ && !is_tag_detecting_in_progress_) {
                tag_info_vec_temp_.clear();
                flag_tag_recog_.store(true);
                is_tag_detecting_in_progress_ = true;
                ROS_LOG_WARN("MARKER DETECTION START! [For Approaching]");
                break;
            }
            ////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////////////
            //// 2) Delay to get marker images && 3) TASK_END_SAVE_TAG_RECOG
            if(!is_tag_received_ && is_tag_detecting_in_progress_) {
                if(tag_detection_delay_ > 50) { // 500ms
                    flag_tag_recog_.store(false);
                    if (saveTagInfo()) {
                        is_tag_received_ = true;
                    } else {
                        is_tag_detecting_in_progress_ = false;
                        is_tag_received_ = false;
                        is_marker_detection_approached_ = false;
                        tag_detection_delay_ = 0;
                        ROS_LOG_WARN("Detecting marker... [For Approaching]");
                        break;
                    }
                } else {
                    tag_detection_delay_++;
                    // ROS_LOG_WARN("Delay... 10ms [For Approaching]");
                    break;
                }
            }
            ////////////////////////////////////////////////////////////////////////////////////////

            bool is_second_tag_approach = current_task_list[task_param.task_step].is_second_tag_approach;

            ////////////////////////////////////////////////////////////////////////////////////////
            //// 4) Set Current Tag
            if(is_tag_received_) {
                if (setCurrentTag(current_task_list[task_param.task_step].tag_name)) {
                    ROS_LOG_WARN("Set current tag [%s] [For Approaching]", current_task_list[task_param.task_step].tag_name.c_str());
                } else {
                    ROS_LOG_ERROR("Check Tag Name! - Unknown Tag [%s]! [For Approaching]", current_task_list[task_param.task_step].tag_name.c_str());
                    is_task_mode_ = false;
                    stopRobot();
                    task_param.task_mode = TASK_DEFAULT;
                    break;
                }
                ////////////////////////////////////////////////////////////////////////////////////////


                ////////////////////////////////////////////////////////////////////////////////////////
                //// Check convergence
                //// JSON Tag Pose load
                //// FILE PATH
                std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/marker_pose/graphy_marker_info.json";
                //// JSON INPUT
                std::stringstream ss;
                std::ifstream ifs(pose_config_path.c_str());

                json j_in;
                try {
                    j_in = json::parse(ifs);
                } catch (const std::exception & e) {
                    std::cout << e.what() << std::endl;
                    ROS_LOG_ERROR("Retrying JSON PARSING...");
                    ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                    ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                    ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());

                    ////////////////////////
                    //// retry
                    j_in = json::parse(ifs);
                    ////////////////////////
                    ROS_LOG_ERROR("Retrying JSON - SUCCESS!");
                }

                //// M-1) Tag pose JSON Load
                //// NOTICE: 현재는 일단 티칭에 사용된 Tag 포즈를 불러다가 사용
                //// TODO: 추후, 티칭 시에는 지정된 방위에 따라 특정 오차 이내의 Tag 포즈를 저장하도록 처리하고,
                //// TODO: 작업 중에 인식하는 경우에는 이 저장된 Tag포즈를 불러다가 비교하도록 처리

                std::vector<double> teaching_tage_cam2tag_pose_in;
                try {
                    teaching_tage_cam2tag_pose_in = j_in["[Tag Pose]"][current_task_list[task_param.task_step].tag_name.c_str()]["pose_cam2tag_ideal"].get<std::vector<double>>();
                    ROS_LOG_WARN("[MARKER DETECTION] [JSON LOAD] Tag pose (Camera to Marker): %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f [For Approaching]\n", teaching_tage_cam2tag_pose_in[0], teaching_tage_cam2tag_pose_in[1], teaching_tage_cam2tag_pose_in[2], teaching_tage_cam2tag_pose_in[3], teaching_tage_cam2tag_pose_in[4], teaching_tage_cam2tag_pose_in[5]);
                    ROS_LOG_WARN("[MARKER DETECTION] [   NOW   ] Tag pose (Camera to Marker): %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f [For Approaching]\n", tag_info_current_.pose_camera_tag.element_[0], tag_info_current_.pose_camera_tag.element_[1], tag_info_current_.pose_camera_tag.element_[2], tag_info_current_.pose_camera_tag.element_[3], tag_info_current_.pose_camera_tag.element_[4], tag_info_current_.pose_camera_tag.element_[5]);

                    //// Check Approach Position z
                    Eigen::Vector3f l_position_load(teaching_tage_cam2tag_pose_in[0], teaching_tage_cam2tag_pose_in[1], teaching_tage_cam2tag_pose_in[2]); // [m]
                    Eigen::Vector3f l_position_now(tag_info_current_.pose_camera_tag.element_[0], tag_info_current_.pose_camera_tag.element_[1], tag_info_current_.pose_camera_tag.element_[2]); // [m]
                    Eigen::Vector3f l_position_err = l_position_load - l_position_now;

                    // double approach_z_limit  = j_in["[Tag Pose]"][current_task_list[task_param.task_step].tag_name.c_str()]["approach_z_limit"];
                    double approach_z_limit = 0.0;
                    if(is_second_tag_approach) { // second approach
                        approach_z_limit = j_in["[Tag Pose]"][current_task_list[task_param.task_step].tag_name.c_str()]["approach_second_z_limit"];
                    } else { // first approach from JSON
                        approach_z_limit  = j_in["[Tag Pose]"][current_task_list[task_param.task_step].tag_name.c_str()]["approach_z_limit"];
                    }
                    if(fabs(l_position_err[2]) < approach_z_limit) {
                        is_marker_detection_approached_ = true;
                    } else {
                        is_marker_detection_approached_ = false;
                    }

                    if(fabs(l_position_err[2]) < approach_z_limit) {
                        ROS_LOG_ERROR("[Approached] position_z_err [m]: %0.5f < (%0.5f)", l_position_err[2], approach_z_limit);
                    } else {
                        ROS_LOG_ERROR("[Not Approached] position_z_err [m]: %0.5f < (%0.5f)", l_position_err[2], approach_z_limit);
                    }

                    double approach_stop_z_limit = 0.05;
                    if(fabs(l_position_err[2]) > approach_stop_z_limit) {
                        ROS_LOG_WARN("[Approach][Too Large Value - Re-trying detection...] position_z_err [m]: %0.5f > (approach_stop_z_limit: %0.5f)", l_position_err[2], approach_stop_z_limit);
                        is_tag_received_ = false;
                        is_tag_detecting_in_progress_ = false;
                        break;
                    }

                } catch (const std::exception & e) {
                    std::cout << e.what() << std::endl;
                    ROS_LOG_ERROR("[MARKER DETECTION] Check JSON file! - loading error! (Tag pose (Camera to Marker) loading failed)");
                    is_task_mode_ = false;
                    stopRobot();
                    task_param.task_mode = TASK_DEFAULT;
                    break;
                }
                ////////////////////////////////////////////////////////////////////////////////////////



                ////////////////////////////////////////////////////////////////////////////////////////
                //// 5) TASK_CSMOVE_TO_TAG
                ModelMatrix tr_base_camera(6, 1), pose_camera_tag(6, 1), tr_camera_tag(4, 4), pose_base_target(6, 1);

                tr_base_camera = tag_info_current_.tr_base_tag
                                * ModelMatrix::invTrMat(tag_info_current_.tr_camera_tag);

                pose_camera_tag = tag_info_current_.pose_camera_tag;
                ROS_LOG_WARN("[pose_camera_tag]: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                    , pose_camera_tag.element_[0], pose_camera_tag.element_[1], pose_camera_tag.element_[2], pose_camera_tag.element_[3], pose_camera_tag.element_[4], pose_camera_tag.element_[5]);

                uint distance_mm = current_task_list[task_param.task_step].des_tag_distance_mm;

                ModelMatrix pose_tag_180deg;

#if CAM_TYPE_REALSENSE_D405
                if (distance_mm == 0) {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, pose_camera_tag.element_[2], 180, 0, 0});
                } else {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, (double) distance_mm / 1000, 180, 0, 0});
                }

#endif

#if CAM_TYPE_LOGITEC_BRIO
                // if (distance_mm == 0) {
                //     pose_tag_180deg = ModelMatrix(6, 1, {0, 0, pose_camera_tag.element_[2], 180, 0, 180}); // 카메라 브라켓 뒤집힘
                // } else {
                //     pose_tag_180deg = ModelMatrix(6, 1, {0, 0, (double) distance_mm / 1000, 180, 0, 180}); // 카메라 브라켓 뒤집힘
                // }

                if (distance_mm == 0) {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, pose_camera_tag.element_[2], 180, 0, 0});
                } else {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, (double) distance_mm / 1000, 180, 0, 0});
                }
#endif
                ROS_LOG_WARN("[pose_tag_180deg]: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                    , pose_tag_180deg.element_[0], pose_tag_180deg.element_[1], pose_tag_180deg.element_[2], pose_tag_180deg.element_[3], pose_tag_180deg.element_[4], pose_tag_180deg.element_[5]);

                tr_camera_tag = ModelMatrix::pose2tr(pose_camera_tag);

                pose_base_target = ModelMatrix::tr2pose(tr_base_camera * tr_camera_tag
                                                        * ModelMatrix::pose2tr(pose_tag_180deg) * tr_camera_ee_);

                CsDouble x_target;

                for (int i = 0; i < 6; i++) {
                    x_target[i] = pose_base_target.element_[i];
                }

                ModelMatrix temp = ModelMatrix::tr2pose(tr_camera_tag * ModelMatrix::pose2tr(pose_tag_180deg));
                ROS_LOG_WARN("pose_camera_tag_180: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                    , temp.element_[0], temp.element_[1], temp.element_[2], temp.element_[3], temp.element_[4], temp.element_[5]);

                ROS_LOG_WARN("Target pose: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                    , x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);

                double vel = current_task_list[task_param.task_step].vel_cs;
                double acc = current_task_list[task_param.task_step].acc_cs;

                task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_APPROACHING;
                MoveErrorType err_type = moveX(x_target, vel, acc, true, false);
                ////////////////////////////////////////////////////////////////////////////////////////

            } else {
                ROS_LOG_WARN("ERROR - No detected marker [For Approaching]");

            }
        } break;

        case TASK_MARKER_DETECTION_FOR_CONVERGING: {

            ////////////////////////////////////////////////////////////////////////////////////////
            //// 1) TASK_START_TAG_RECOG
            if(!is_tag_received_ && !is_tag_detecting_in_progress_) {
                tag_info_vec_temp_.clear();
                flag_tag_recog_.store(true);
                is_tag_detecting_in_progress_ = true;
                ROS_LOG_WARN("MARKER DETECTION START! [For Converging]");
                break;
            }
            ////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////////////
            //// 2) Delay to get marker images && 3) TASK_END_SAVE_TAG_RECOG
            if(!is_tag_received_ && is_tag_detecting_in_progress_) {
                // if(tag_detection_delay_ > 50) { // 500ms
                if(tag_detection_delay_ > 120) { // 1200ms (초점 맞도록 충분한 딜레이 1.2초)
                    flag_tag_recog_.store(false);
                    if (saveTagInfo()) {
                        is_tag_received_ = true;
                    } else {
                        is_tag_detecting_in_progress_ = false;
                        is_tag_received_ = false;
                        is_marker_detection_converged_ = false;
                        tag_detection_delay_ = 0;
                        ROS_LOG_WARN("Detecting marker... [For Converging]");
                        break;
                    }
                } else {
                    tag_detection_delay_++;
                    // ROS_LOG_WARN("Delay... 10ms [For Converging]");
                    break;
                }
            }
            ////////////////////////////////////////////////////////////////////////////////////////

            ////////////////////////////////////////////////////////////////////////////////////////
            //// 4) Set Current Tag
            if(is_tag_received_) {
                if (setCurrentTag(current_task_list[task_param.task_step].tag_name)) {
                    ROS_LOG_WARN("Set current tag [%s] [For Converging]", current_task_list[task_param.task_step].tag_name.c_str());
                } else {
                    ROS_LOG_ERROR("Check Tag Name! - Unknown Tag [%s]! [For Converging]", current_task_list[task_param.task_step].tag_name.c_str());
                    is_task_mode_ = false;
                    stopRobot();
                    task_param.task_mode = TASK_DEFAULT;
                    break;
                }
                ////////////////////////////////////////////////////////////////////////////////////////


                ////////////////////////////////////////////////////////////////////////////////////////
                //// Check convergence
                //// JSON Tag Pose load
                //// FILE PATH
                std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/marker_pose/graphy_marker_info.json";
                //// JSON INPUT
                std::stringstream ss;
                std::ifstream ifs(pose_config_path.c_str());
                json j_in;
                try {
                    j_in = json::parse(ifs);
                } catch (const std::exception & e) {
                    std::cout << e.what() << std::endl;
                    ROS_LOG_ERROR("Retrying JSON PARSING...");
                    ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                    ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());
                    ROS_LOG_ERROR("[JSON PARSE ERROR] JSON file: %s", pose_config_path.c_str());

                    ////////////////////////
                    //// retry
                    j_in = json::parse(ifs);
                    ////////////////////////
                    ROS_LOG_ERROR("Retrying JSON - SUCCESS!");
                }




                //// M-1) Tag pose JSON Load
                //// NOTICE: 현재는 일단 티칭에 사용된 Tag 포즈를 불러다가 사용
                //// TODO: 추후, 티칭 시에는 지정된 방위에 따라 특정 오차 이내의 Tag 포즈를 저장하도록 처리하고,
                //// TODO: 작업 중에 인식하는 경우에는 이 저장된 Tag포즈를 불러다가 비교하도록 처리

                std::vector<double> teaching_tage_cam2tag_pose_in;
                try {
                    teaching_tage_cam2tag_pose_in = j_in["[Tag Pose]"][current_task_list[task_param.task_step].tag_name.c_str()]["pose_cam2tag_ideal"].get<std::vector<double>>();
                    ROS_LOG_WARN("[MARKER DETECTION] [JSON LOAD] Tag pose (Camera to Marker): %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f [For Converging]\n", teaching_tage_cam2tag_pose_in[0], teaching_tage_cam2tag_pose_in[1], teaching_tage_cam2tag_pose_in[2], teaching_tage_cam2tag_pose_in[3], teaching_tage_cam2tag_pose_in[4], teaching_tage_cam2tag_pose_in[5]);
                    ROS_LOG_WARN("[MARKER DETECTION] [   NOW   ] Tag pose (Camera to Marker): %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f [For Converging]\n", tag_info_current_.pose_camera_tag.element_[0], tag_info_current_.pose_camera_tag.element_[1], tag_info_current_.pose_camera_tag.element_[2], tag_info_current_.pose_camera_tag.element_[3], tag_info_current_.pose_camera_tag.element_[4], tag_info_current_.pose_camera_tag.element_[5]);


                    //// Check Position & orientation error
                    Eigen::Vector3f l_position_load(teaching_tage_cam2tag_pose_in[0], teaching_tage_cam2tag_pose_in[1], teaching_tage_cam2tag_pose_in[2]); // [m]
                    Eigen::Vector3f l_orientation_load(teaching_tage_cam2tag_pose_in[3], teaching_tage_cam2tag_pose_in[4], teaching_tage_cam2tag_pose_in[5]); // [deg]

                    Eigen::Vector3f l_position_now(tag_info_current_.pose_camera_tag.element_[0], tag_info_current_.pose_camera_tag.element_[1], tag_info_current_.pose_camera_tag.element_[2]); // [m]
                    Eigen::Vector3f l_orientation_now(tag_info_current_.pose_camera_tag.element_[3], tag_info_current_.pose_camera_tag.element_[4], tag_info_current_.pose_camera_tag.element_[5]); // [deg]


                    Eigen::Vector3f l_position_err = l_position_load - l_position_now;
                    Eigen::Vector3f l_orientation_err = l_orientation_load - l_orientation_now;
                    for(int i = 0; i < 3; i++) {
                        if(l_orientation_err[i] <= -180.0) {
                            l_orientation_err[i] += 360.0;
                        }
                        else if(l_orientation_err[i] >= 180.0) {
                            l_orientation_err[i] -= 360.0;
                        }
                    }

                    double l_position_err_norm = l_position_err.norm();
                    double l_orientation_err_norm = l_orientation_err.norm();

                    double conv_position_limit  = j_in["[Tag Pose]"][current_task_list[task_param.task_step].tag_name.c_str()]["position_limit"];
                    double conv_orientation_limit  = j_in["[Tag Pose]"][current_task_list[task_param.task_step].tag_name.c_str()]["orientation_limit"];

                    if(l_position_err_norm < conv_position_limit && l_orientation_err_norm < conv_orientation_limit) {
                        is_marker_detection_converged_ = true;
                    } else {
                        is_marker_detection_converged_ = false;
                    }

                    if(l_position_err_norm < conv_position_limit) {
                        ROS_LOG_ERROR("[Converged] position_err [m]: %0.5f < (%0.5f)", l_position_err_norm, conv_position_limit);
                    } else {
                        ROS_LOG_ERROR("[Not Converged] position_err [m]: %0.5f < (%0.5f)", l_position_err_norm, conv_position_limit);
                    }
                    if(l_orientation_err_norm < conv_orientation_limit) {
                        ROS_LOG_ERROR("[Converged] orientation_err [deg]: %0.3f < (%0.3f)", l_orientation_err_norm, conv_orientation_limit);
                    } else {
                        ROS_LOG_ERROR("[Not Converged] orientation_err [deg]: %0.3f < (%0.3f)", l_orientation_err_norm, conv_orientation_limit);
                    }

                    double converging_stop_z_limit = 0.05;
                    if(fabs(l_position_err[2]) > converging_stop_z_limit) {
                        ROS_LOG_WARN("[Converging][Too Large Value - Re-trying detection...] position_z_err [m]: %0.5f > (converging_stop_z_limit: %0.5f)", l_position_err[2], converging_stop_z_limit);
                        is_tag_received_ = false;
                        is_tag_detecting_in_progress_ = false;
                        break;
                    }

                } catch (const std::exception & e) {
                    std::cout << e.what() << std::endl;
                    ROS_LOG_ERROR("[MARKER DETECTION] Check JSON file! - loading error! (Tag pose (Camera to Marker) loading failed)");
                    is_task_mode_ = false;
                    stopRobot();
                    task_param.task_mode = TASK_DEFAULT;
                    break;
                }
                ////////////////////////////////////////////////////////////////////////////////////////



                ////////////////////////////////////////////////////////////////////////////////////////
                //// 5) TASK_CSMOVE_TO_TAG
                ModelMatrix tr_base_camera(6, 1), pose_camera_tag(6, 1), tr_camera_tag(4, 4), pose_base_target(6, 1);

                tr_base_camera = tag_info_current_.tr_base_tag
                                * ModelMatrix::invTrMat(tag_info_current_.tr_camera_tag);

                pose_camera_tag = tag_info_current_.pose_camera_tag;
                ROS_LOG_WARN("[pose_camera_tag]: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                    , pose_camera_tag.element_[0], pose_camera_tag.element_[1], pose_camera_tag.element_[2], pose_camera_tag.element_[3], pose_camera_tag.element_[4], pose_camera_tag.element_[5]);

                uint distance_mm = current_task_list[task_param.task_step].des_tag_distance_mm;

                ModelMatrix pose_tag_180deg;

#if CAM_TYPE_REALSENSE_D405
                if (distance_mm == 0) {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, pose_camera_tag.element_[2], 180, 0, 0});
                } else {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, (double) distance_mm / 1000, 180, 0, 0});
                }

#endif

#if CAM_TYPE_LOGITEC_BRIO
                // if (distance_mm == 0) {
                //     pose_tag_180deg = ModelMatrix(6, 1, {0, 0, pose_camera_tag.element_[2], 180, 0, 180}); // 카메라 브라켓 뒤집힘
                // } else {
                //     pose_tag_180deg = ModelMatrix(6, 1, {0, 0, (double) distance_mm / 1000, 180, 0, 180}); // 카메라 브라켓 뒤집힘
                // }

                if (distance_mm == 0) {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, pose_camera_tag.element_[2], 180, 0, 0});
                } else {
                    pose_tag_180deg = ModelMatrix(6, 1, {0, 0, (double) distance_mm / 1000, 180, 0, 0});
                }
#endif
                ROS_LOG_WARN("[pose_tag_180deg]: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                    , pose_tag_180deg.element_[0], pose_tag_180deg.element_[1], pose_tag_180deg.element_[2], pose_tag_180deg.element_[3], pose_tag_180deg.element_[4], pose_tag_180deg.element_[5]);

                tr_camera_tag = ModelMatrix::pose2tr(pose_camera_tag);

                pose_base_target = ModelMatrix::tr2pose(tr_base_camera * tr_camera_tag
                                                        * ModelMatrix::pose2tr(pose_tag_180deg) * tr_camera_ee_);

                CsDouble x_target;

                for (int i = 0; i < 6; i++) {
                    x_target[i] = pose_base_target.element_[i];
                }

                ModelMatrix temp = ModelMatrix::tr2pose(tr_camera_tag * ModelMatrix::pose2tr(pose_tag_180deg));
                ROS_LOG_WARN("pose_camera_tag_180: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                    , temp.element_[0], temp.element_[1], temp.element_[2], temp.element_[3], temp.element_[4], temp.element_[5]);

                ROS_LOG_WARN("Target pose: %0.5f     %0.5f     %0.5f     %0.3f     %0.3f     %0.3f"
                    , x_target[0], x_target[1], x_target[2], x_target[3], x_target[4], x_target[5]);

                double vel = current_task_list[task_param.task_step].vel_cs;
                double acc = current_task_list[task_param.task_step].acc_cs;

                task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE_FOR_MARKER_DETECTION_CONVERGING;
                MoveErrorType err_type = moveX(x_target, vel, acc, true, false);
                ////////////////////////////////////////////////////////////////////////////////////////

            } else {
                ROS_LOG_WARN("ERROR - No detected marker [For Converging]");

            }
        } break;

        case TASK_SET_CAM_AUTO_FOCUS: {
            setCameraAutoFocus(current_task_list[task_param.task_step].is_cam_auto_focus_on);
            taskFinFunction(0);
        } break;

/////////////////PYBIND
        case TASK_PYBIND_CSMOVE: {
            std::string func_name = current_task_list[task_param.task_step].pybind_func_name;

            try {
                if (!ros_thread.is_none()) {
                    py::gil_scoped_acquire acquire;  // Python 함수 호출 전 GIL 확보

                    // Python 함수 호출 (좌표, 속도, 가속도, 상대 좌표 여부를 받아옴)
                    py::object result = ros_thread.attr(func_name.c_str())();  // Python 함수 호출

                    // Python에서 반환된 결과를 처리 (tuple 형태로 반환된다고 가정)
                    py::tuple result_tuple = result.cast<py::tuple>();

                    // CsDouble(6개 좌표)로 변환
                    std::vector<double> x_target_vec = result_tuple[0].cast<std::vector<double>>();
                    if (x_target_vec.size() != 6) {
                        throw std::runtime_error("잘못된 좌표 크기입니다.");
                    }
                    CsDouble x_target = { x_target_vec[0], x_target_vec[1], x_target_vec[2],
                                        x_target_vec[3], x_target_vec[4], x_target_vec[5] };

                    // 속도와 가속도, 상대 좌표 여부를 받아옴
                    double vel = result_tuple[1].cast<double>();     // 속도
                    double acc = result_tuple[2].cast<double>();     // 가속도
                    bool relative = result_tuple[3].cast<bool>();    // 상대 좌표 여부

                    // 로봇 이동 명령 호출
                    task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
                    MoveErrorType err_type = moveX(x_target, vel, acc, true, relative);

                    if (err_type == MoveErrorType::COL_DETECTED && params_.mode.is_collision_demo_mode) {
                        task_param.task_mode = TASK_CSMOVE;
                        sleep(2);  // 충돌이 감지된 경우 일정 시간 대기 후 재시도
                    }
                    taskFinFunction(0);  // 작업 완료 처리 함수 호출
                }
            } catch (py::error_already_set &e) {
                std::cerr << "Python Error: " << e.what() << std::endl;
            } catch (const std::exception &e) {
                std::cerr << "C++ Exception: " << e.what() << std::endl;
            } catch (...) {
                std::cerr << "Unknown exception occurred!" << std::endl;
            }

        } break;

        case TASK_DRFL_API_TASK: {
            ROS_LOG_INFO("DRFL function called !! ");
            task_param.task_mode = WAIT_TIL_FIN_ROBOTMOVE;
            current_task_list[task_param.task_step].function_wrapper->execute();
            //taskFinFunction(0);
        } break;



        ///////////////////////////// Apriltag  related //////////////////////////////////
    }
}

/////////////////////////////////////// Bin Picking ///////////////////////////////////
void QNode::beforeBinPickingTaskStart(bool is_initial_process) {
    vector<double> nf          = {5, 5, 5, 5, 5, 5};
    vector<double> zeta        = {10, 10, 10, 10, 10, 10};
    vector<double> stiffness   = {500, 500, 1500, 1, 1, 1};
    vector<double> force_limit = {60, 60, 60, 20, 20, 20};

    ROS_LOG_WARN("[%s] THIS 1", __func__);
    current_task_list_origin_ = current_task_list_;

    task_param_.task_step = 0;

    task_param_.is_unit_task_fin = false;
    task_param_.is_pause_status  = false;

    task_param_.is_robot_move_fin  = false; // TODO
    task_param_.is_ready_matching = false;

    params_.status.is_path_finished = false;
    params_.status.is_gripper_finished = false;
    params_.status.is_detection_finished = false;

    tip_changing_task_flag_ = false; //// For tip changing
    do_skip_task = false;
    is_teaching_finished_ = false;
    is_task_finished_ = false;
    is_task_robot_disable_and_enable = false;

    //// Marker
    is_tag_detecting_in_progress_ = false;
    is_tag_received_ = false;
    is_marker_detection_converged_ = false;
    is_marker_detection_approached_ = false;
    tag_detection_delay_ = 0;

    ROS_LOG_WARN("[%s] THIS 2", __func__);


    ROS_LOG_WARN("[%s] THIS 3", __func__);

    is_parallel_task_mode_ = false;

    task_param_.task_end_num = current_task_list_.size();
    task_param_.task_mode    = current_task_list_.at(0).task_mode;

    is_task_mode_ = true;


    ROS_LOG_WARN("[%s] THIS 4", __func__);
    if(!is_developer_mode_) {
        Q_EMIT beforeTaskSignal();
    }
    ROS_LOG_WARN("[%s] THIS 5", __func__);


// #if BIN_PICKING_FLAG
    //////simulation ros2
    is_simul_mode_ = false;

    ////////////////////////////////////////
    //// Bin picking
    // Task parameters
    task_param_.is_detection_fin = false;
    task_param_.is_rough_align_fin = false;
    task_param_.is_gripper_fin = false;
    task_param_.is_task_seq_fin = false;
    task_param_.is_task_fail = false;
    task_param_.is_ready_matching = false;
    task_param_.is_ready_task_recog = false;
    task_param_.is_recovery_task = false;
    task_param_.is_scanning_recovery_task = false;
    task_param_.is_json_trajectory = false;
    task_param_.is_scanning_failure_flag = false;
    task_param_.is_assembly_failure_flag = false;
    // Process parametrs
    m_bin_picking_node->is_blending_approach_grasp_pose_assigned = false;
    m_bin_picking_node->is_blending_path_in_process_for_scan_and_matching = false;
    m_bin_picking_node->is_blending_scan_and_matching_conducted = false;

    m_bin_picking_node->is_check_matching_finished = false;

    m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_matching_parameter.debug_mode = false;

    //// NOTICE: Initial scan & matching 동작과 연계된 파라미터
    //// NOTICE: Initial scan & matching 동작과 연계된 파라미터
    //// NOTICE: Initial scan & matching 동작과 연계된 파라미터
    if(is_initial_process) { // topic update
        m_bin_picking_node->is_grasping_pose_cal_finished = false; // Topic 종료 여부
        m_bin_picking_node->is_matching_finished = false; // Topic 종료 여부
        //// 24.02.12 추가
        m_bin_picking_node->is_grasping_pose_feasible_ = false; // doGrasping에 사용
    }

    ////

    tip_changing_task_flag_ = false;
    m_bin_picking_node->is_grasping_pose_assigned = false;
    m_bin_picking_node->is_zig_pose_assigned = false;

    if(m_bin_picking_node->m_ptr_bin_picking_node_target_object_list.size() != 0) {
        m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.do_tip_changing_for_grasping = false;
    }
    do_skip_task = false;
    //// Stacking mode flag for gripper open length
    is_stacking_mode_ = false;

    ////////////////////////////////////////
// #endif
}

bool QNode::sendChangeRobotTCPCommand(unsigned int tcp_idx) {

    //// FILE PATH
    std::string file_name("qnode.cpp");
    std::string config_path_ = __FILE__;
    config_path_.resize(config_path_.length() - file_name.length());
    config_path_ += "../config/robot_config/";
    std::string tcp_config_path = config_path_ + "robot_tcp.json";
        //// JSON INPUT
    CsDouble tcp;
    std::vector<std::vector<double>> robot_tcp_set; // 정의된 tcp를 순서대로 저장
    std::ifstream ifs_robot_tcp_config(tcp_config_path.c_str());
    json j_robot_tcp = json::parse(ifs_robot_tcp_config);
    int cnt_tcp = j_robot_tcp["tcp_count"];
    std::stringstream ss;
    for(int i=0; i<cnt_tcp; i++) {
        ss.str("");
        ss << "tcp";
        ss << std::setw(3) << std::setfill('0') << i+1;
        std::vector<double> vec_tcp = j_robot_tcp[ss.str()].get<std::vector<double>>();
        robot_tcp_set.push_back(vec_tcp); // [m], [deg]
        // ROS_LOG_INFO("tcp00%i: %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f", i+1, robot_tcp_set[i][0], robot_tcp_set[i][1], robot_tcp_set[i][2], robot_tcp_set[i][3], robot_tcp_set[i][4], robot_tcp_set[i][5]);
    }

    for (int i = 0; i < CS_DOF; i++) {
        tcp[i] = robot_tcp_set[tcp_idx - 1][i];
    }
    ROS_LOG_INFO("tcp now: %0.4f, %0.4f, %0.4f, %0.4f, %0.4f, %0.4f", tcp[0], tcp[1], tcp[2], tcp[3], tcp[4], tcp[5]);

    // current index update
    robot_current_tcp_idx_ = tcp_idx; // tcp #1, 2, 3, ...


    std::vector<double> vec_tcp(6, 0.0);
    std::vector<double> vec_default_tcp(6, 0.0);
    for (int i = 0; i < CS_DOF; i++) {
        vec_tcp[i] = robot_tcp_set[tcp_idx - 1][i];
        vec_default_tcp[i] = robot_tcp_set[7 - 1][i];
    }

    // update
    setRobotTCP(vec_tcp);
    setRobotDefaultTCP(vec_default_tcp);

    return setTcp(tcp);
}

/** @brief Korean: 부하동정(load identification)을 위한 로봇 자세 데이터를 입력한다.
 */
void QNode::setLoadIDTargetJSPosition() {
    // load_id_vec_target_JS_pose_.clear();
    // std::stringstream ss;

    // //// json file input - target JS angle
    // std::ifstream ifs("../minwoo_ws/src/assem_controller/config/load_id_targetJS.json");
    // json j_in = json::parse(ifs);
    // int cnt_js = j_in["count"];
    // for(int i=0; i<cnt_js; i++)
    // {
    //     ss.str("");
    //     ss << "position";
    //     ss << std::setw(3) << std::setfill('0') << i+1;
    //     std::vector<double> vec_tmp = j_in[ss.str()].get<std::vector<double>>();
    //     load_id_vec_target_JS_pose_.push_back(vec_tmp);
    // }

    // //// JS
    // std::vector<std::vector<double>> target_JS = load_id_vec_target_JS_pose_;
    // printf("*******************************************\n");
    // printf("****************JS Target******************\n");
    // for(int i=0; i < target_JS.size(); i++)
    // {
    //     printf("JS %dth: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", i+1, target_JS[i][0], target_JS[i][1],target_JS[i][2],target_JS[i][3],target_JS[i][4],target_JS[i][5]);
    // }
    // printf("*******************************************\n");

    // //// Task generation
    // taskPlanner_.makeLoadIDTaskList(cnt_js);
}

/** @brief Korean: ROS 서비스 노드(load_identification_command)의 요청함수, UI 상의 버튼을 통한 부하동정(load identification)의 실행을 위한 명령을 assem_controller 패키지로 송신한다.
 * @param[in] b_id : true
 */
void QNode::doLoadIdentification(bool b_id)
{
    // ROS_LOG_INFO("Call load id");
	// assem_control_msgs::DoLoadID service;
    // service.request.do_load_id = b_id;
    // load_id_client_.call(service);
}

/** @brief Korean: ROS 서비스 노드(sensor_command)의 요청함수, UI 상의 버튼을 통한 ATI F/T sensor의 구동을 위한 명령을 assem_controller 패키지로 송신한다.
 * @param[in] isRobotA : true
 * @return true: 서비스 통신 성공, false: 서비스 통신 실패
 */
bool QNode::setInitSensorBias() {
	// assem_control_msgs::SetCommand service;
	// service.request.command = "sensor_init_bias_flag";

	// if (isRobotA)
    //     return robotACommandClient_.call(service);
}

// KORAS gripper service call
bool QNode::checkConnectionKorasGripperDriver()
{
    auto req = make_shared<grp_control_msg::srv::GripperCommand::Request>();

    // req->command = cmd;
    // req->value   = position;
    // req->slave_num = grp_address;

    auto result = client_gripper_cmd_->async_send_request(req);
    auto status = result.wait_for(chrono::milliseconds(100));

    if (status == future_status::timeout) {
    ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
    return false;
    }

    auto recv = result.get();

    if (recv->successed) {
    ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
    ROS_LOG_INFO("KORAS gripper driver connected!");
    gripper_state = 0;
        return true;
} else {
        ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
        ROS_LOG_INFO("KORAS gripper driver is not connected!");
        gripper_state = -1;
        return false;
    }


}

// // KORAS gripper service call
// bool QNode::sendKorasGripperDriverCommand(uint16_t cmd, uint16_t position, uint16_t grp_address)
// {
//     auto req = make_shared<grp_control_msg::srv::GripperCommand::Request>();

//     req->command = cmd;
//     req->value   = position;
//     req->slave_num = grp_address;

//     auto result = client_gripper_cmd_->async_send_request(req);
//     auto status = result.wait_for(chrono::milliseconds(100));

//     if (status == future_status::timeout) {
//         ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
//         return false;
//     }

//     auto recv = result.get();

//     if (recv->successed) {
//         ROS_LOG_INFO("Function successed: %s", __FUNCTION__);
//         if (params_.status.is_gripper_finished == false && is_task_mode_) {
//             params_.status.is_gripper_finished = true;
//         }

//         if(cmd == (uint16_t)KR_GRP::INIT) {
//             is_grp_initialized[grp_driver_address_-1] = true;
//         } else if(cmd == (uint16_t)KR_GRP::POS_CTRL) {
//             if(position > 9000) gripper_state = 1;
//             else gripper_state = 0;
//             ROS_LOG_INFO("KORAS gripper Pos. ctrl finished");
//         } else if(cmd == (uint16_t)KR_GRP::OPEN) {
//             gripper_state = 0;
//             ROS_LOG_INFO("KORAS gripper OPEN finished");
//         } else if(cmd == (uint16_t)KR_GRP::CLOSE) {
//             gripper_state = 1;
//             ROS_LOG_INFO("KORAS gripper CLOSE finished");
//         } else if(cmd == (uint16_t)KR_GRP::VACUUM_ON) {
//             gripper_state = 1;
//             ROS_LOG_INFO("KORAS gripper Vacuum ON");
//         } else if(cmd == (uint16_t)KR_GRP::VACUUM_OFF) {
//             gripper_state = 0;
//             ROS_LOG_INFO("KORAS gripper Vacuum OFF");
//         } else if(cmd == (uint16_t)KR_GRP::CHANGE_ADDRESS) {
//             gripper_state = 0;
//             grp_driver_address_ = grp_address;
//             ROS_LOG_INFO("KORAS gripper Slave Change (Slave #%i)", grp_driver_address_);
//         }

//         return true;
//     } else {
//         ROS_LOG_ERROR("Function: %s\n\tError code: %d", __FUNCTION__, recv->successed);
//         gripper_state = -1;
//         is_task_mode_ = false;
//         return false;
//     }
// }

//// Communication - UDP
bool QNode::setUDPComm(const std::string cmd, const std::string &ip, const int &port)
{
    // auto req = make_shared<hanyang_matching_msgs::srv::SetCommand::Request>();
    // req->command     = cmd;
    // req->ip_udp    = ip;
    // req->port_udp = port;

    // auto result = client_sensor_command_->async_send_request(req);
    // auto status = result.wait_for(chrono::milliseconds(100));
    // if (status == future_status::timeout) {
    //     ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
    //     return false;
    // }
    // auto recv = result.get();
    return true;
}

//// Communication - modbus tcp
bool QNode::setMODBUSComm(bool do_connect)
{
    std::string ip_in = ip_modbus_;
    int16_t port_in = port_modbus_;
    // std::cout << ip_modbus_ << "::" << port_modbus_;

    // Connect
    if (_modbus.use_count() != 1) {
        _modbus = std::make_shared<modbus>(ip_in, port_in);
        // _modbus = std::shared_ptr<modbus>(ip_in, port_in);
    }

    if(do_connect) {
        if(_modbus->modbus_connect()) {
            // ROS_LOG_WARN("MODBUS CONNECT SUCCESS!");
        } else {
            ROS_LOG_INFO("MODBUS CONNECT FAILED!");
            return false;
        }
    } else {
        _modbus->modbus_close();
        // ROS_LOG_INFO("MODBUS CLOSED!");
    }
    return true;
}

bool QNode::sendMessageMODBUS(uint16_t address, uint16_t amount, bool flag_coil)
{
    int modbus_status = _modbus->modbus_write_coils(address, amount, &flag_coil);
    if(modbus_status == 0) {
        ROS_LOG_INFO("SendMessageMODBUS - SUCCESS(address. #%i)", address);
        return true;
    } else {
        ROS_LOG_INFO("SendMessageMODBUS - FAILED(err. #%i)", modbus_status);
        setMODBUSComm(true);
        return false;
    }
}

bool QNode::setPLCModbusCommand(int plc_command)
{
    ROS_LOG_WARN("[%s] !!!", __func__);

    bool is_command_stable = false;
    // setMODBUSComm(true);
    switch(plc_command) {
        case LATHE_CHUCK_CLOSE: {
            is_command_stable = sendMessageMODBUS(0, 1);
            break;
        }
        case LATHE_CHUCK_OPEN: {
            is_command_stable = sendMessageMODBUS(1, 1);
            break;
        }
        case MILLING_CHUCK_CLOSE: {
            is_command_stable = sendMessageMODBUS(2, 1);
            break;
        }
        case MILLING_CHUCK_OPEN: {
            is_command_stable = sendMessageMODBUS(3, 1);
            break;
        }
        case CNC_DOOR_CLOSE: {
            is_command_stable = sendMessageMODBUS(4, 1);
            break;
        }
        case CNC_DOOR_OPEN: {
            is_command_stable = sendMessageMODBUS(5, 1);
            break;
        }
        case CNC_LED_RED_ON: {
            is_command_stable = sendMessageMODBUS(0x22, 1, true);
            break;
        }
        case CNC_LED_RED_OFF: {
            is_command_stable = sendMessageMODBUS(0x22, 1, false);
            break;
        }
        case CNC_LED_YELLOW_ON: {
            is_command_stable = sendMessageMODBUS(0x21, 1, true);
            break;
        }
        case CNC_LED_YELLOW_OFF: {
            is_command_stable = sendMessageMODBUS(0x21, 1, false);
            break;
        }
        case CNC_LED_GREEN_ON: {
            is_command_stable = sendMessageMODBUS(0x20, 1, true);
            break;
        }
        case CNC_LED_GREEN_OFF: {
            is_command_stable = sendMessageMODBUS(0x20, 1, false);
            break;
        }
        default: {
            assert(false);
            break;
        }

    }
    // setMODBUSComm(false);
    return is_command_stable;
}

//음수를 2의 보수로 변환
inline uint16_t convertToModbusRegisterData(int value){
    if(value < 0) {
        return static_cast<uint16_t>(65536 + value);
    }
    return static_cast<uint16_t>(value);
}

bool QNode::setPLCModbusCommandWriteRegister(int plc_command, int32_t write_data)
{
    ROS_LOG_WARN("[%s] !!!", __func__);

    bool is_command_stable = false;
    uint16_t write_value[2];

    // setMODBUSComm(true);
    switch(plc_command) {

        case STEP_MOTOR_POSITION_CONTROL: {
            write_value[0] = static_cast<uint16_t> (write_data & 0xFFFF);
            write_value[1] = static_cast<uint16_t>((write_data >> 16) & 0xFFFF);

            // 0~360.0 degree to pulse  1600 pulse = 360 degree / 4.44 pulse = 1 degree
            // const double plusePerDegree = 4.44;
            // int target_pulse = static_cast<int>(std::round(target_degree * plusePerDegree));

            int modbus_status = _modbus->modbus_write_registers(0x40000, 2 , write_value);
            if(modbus_status == 0) {
                ROS_LOG_INFO("SendMessageMODBUS - SUCCESS(address. #%i)", 40000);
                modbus_status = setPLCModbusCommand(0);
                if(modbus_status == 0) {
                    ROS_LOG_INFO("SendMessageMODBUS - SUCCESS(address. #%i)", 0);
                    is_command_stable = true;
                }
            } else {
                ROS_LOG_INFO("SendMessageMODBUS - FAILED(err. #%i)", modbus_status);
                is_command_stable = false;
                setMODBUSComm(true);

            }
            break;
        }
        case STEP_MOTOR_SET_VELOCITY: {
            write_value[0] = static_cast<uint16_t> (write_data & 0xFFFF);
            write_value[1] = static_cast<uint16_t>((write_data >> 16) & 0xFFFF);
            int modbus_status = _modbus->modbus_write_registers(0x40002, 2, write_value);
            if(modbus_status == 0) {
                ROS_LOG_INFO("SendMessageMODBUS - SUCCESS(address. #%i)", 40002);
                is_command_stable = true;
            } else {
                ROS_LOG_INFO("SendMessageMODBUS - FAILED(err. #%i)", modbus_status);
                is_command_stable = false;
                setMODBUSComm(true);

            }
            break;
        }
        default: {
            assert(false);
            break;
        }

    }
    return is_command_stable;
}

////////////////////////////////////////
//// 한양이엔지 PLC
bool QNode::PLCModbusWriteStatusFromRobotToAMR(uint16_t register_address, bool is_lsb, uint16_t bit_address, bool status)
{
    ROS_LOG_WARN("[%s] !!!", __func__);

    // lock_guard<mutex> lg(mutex_);
    
    uint16_t current_value = 0;
    int modbus_status;

    // 현재 레지스터 값 읽기
    // setMODBUSComm(true);
#if IS_PLC_LS_XG5000
    modbus_status = _modbus->modbus_read_input_registers(register_address, 1, &current_value);
#else
    modbus_status = _modbus->modbus_read_holding_registers(register_address, 1, &current_value);
#endif
    // setMODBUSComm(false);

    if (modbus_status != 0) {
        ROS_LOG_INFO("[%s] Failed to read PLC register, error code: %i", __func__, modbus_status);
        setMODBUSComm(true);
        return false;
    } else {
        if(0) {
            ROS_LOG_INFO("[%s] current_value: %u", __func__, current_value);
            // 이진수 출력
            std::bitset<16> binary_value(current_value);
            std::string bit_string = binary_value.to_string();
            std::string formatted_bit_string =
                bit_string.substr(0,4) + " " +
                bit_string.substr(4,4) + " " +
                bit_string.substr(8,4) + " " +
                bit_string.substr(12,4);

            ROS_LOG_WARN("[%s] Read current_value (binary): %s", __func__, formatted_bit_string.c_str());
        }
    }

    // 비트 위치 계산
    uint16_t actual_bit_pos = is_lsb ? bit_address : 15 - bit_address;

    // 비트 변경: ON이면 1, OFF이면 0으로 설정
    if (status) {
        current_value |= (1 << actual_bit_pos);  // 해당 비트 ON
    } else {
        current_value &= ~(1 << actual_bit_pos); // 해당 비트 OFF
    }

    if(0) {
        ROS_LOG_INFO("[%s] [modbus_write_registers] current_value: %u", __func__, current_value);
    }

    // 수정된 값 다시 쓰기
    // setMODBUSComm(true);
    if(1) {
    modbus_status = _modbus->modbus_write_registers(register_address, 1, &current_value);
    } else {
        modbus_status = _modbus->modbus_write_register(register_address, current_value);
    }
    // setMODBUSComm(false);

    if (modbus_status == 0) {

        if(0) {
        ROS_LOG_INFO("[%s] PLC Task Status Updated: Bit %d (%s) -> %s",
                     __func__, bit_address, is_lsb ? "LSB" : "MSB", status ? "ON" : "OFF");
        }
        //// WRITE FLAG UPDATA - QNODE에서 현재 WRITE한 FLAG를 저장
        if(register_address == 1000 || register_address == 0x41000) {
            plc_status_word_41000_[bit_address] = status;
        }
        if(register_address == 1002 || register_address == 0x41002) {
            plc_status_word_41002_[bit_address] = status;
        }

        if(0) {
            // 이진수 출력
            std::bitset<16> binary_value(current_value);
            std::string bit_string = binary_value.to_string();
            std::string formatted_bit_string =
                bit_string.substr(0,4) + " " +
                bit_string.substr(4,4) + " " +
                bit_string.substr(8,4) + " " +
                bit_string.substr(12,4);

            ROS_LOG_WARN("[%s] Written current_value (binary): %s", __func__, formatted_bit_string.c_str());
        }
        return true;
    } else {
        ROS_LOG_INFO("[%s] Failed to update PLC Task Status, error code: %i", __func__, modbus_status);
        setMODBUSComm(true);
        return false;
    }
}



bool QNode::PLCModbusInitializeStatusFromRobotToAMR(uint16_t register_address, bool is_lsb, uint16_t bit_address, bool status)
{

    ROS_LOG_WARN("[%s] !!!", __func__);

#if !IS_PLC_COMMUNICATION
    return true;
#endif

    uint16_t current_value = 0;
    int modbus_status;

    if(1) {

    // 현재 레지스터 값 읽기
    // setMODBUSComm(true);
#if IS_PLC_LS_XG5000
        modbus_status = _modbus->modbus_read_input_registers(register_address, 1, &current_value);
#else
        modbus_status = _modbus->modbus_read_input_registers(register_address, 1, &current_value);
#endif
        // setMODBUSComm(false);

        if (modbus_status != 0) {
            ROS_LOG_INFO("[%s] Failed to read PLC register, error code: %i", __func__, modbus_status);
            setMODBUSComm(true);
            return false;
        } else {
            if(0) {
                ROS_LOG_INFO("[%s] current_value: %u", __func__, current_value);
                // 이진수 출력
                std::bitset<16> binary_value(current_value);
                std::string bit_string = binary_value.to_string();
                std::string formatted_bit_string =
                    bit_string.substr(0,4) + " " +
                    bit_string.substr(4,4) + " " +
                    bit_string.substr(8,4) + " " +
                    bit_string.substr(12,4);

                ROS_LOG_WARN("[%s] Read current_value (binary): %s", __func__, formatted_bit_string.c_str());
            }
        }

        // 비트 위치 계산
        uint16_t actual_bit_pos = is_lsb ? bit_address : 15 - bit_address;

        // 비트 변경: ON이면 1, OFF이면 0으로 설정
        for(int i = 0; i < 16; ++i) {
            if (0) {
                current_value |= (1 << i);  // 해당 비트 ON
            } else {
                current_value &= ~(1 << i); // 해당 비트 OFF
            }

            //// WRITE FLAG UPDATE - QNODE에서 현재 WRITE한 FLAG를 저장
            if(register_address == 1000 || register_address == 0x41000) {
                plc_status_word_41000_[i] = false;
            }
            if(register_address == 1002 || register_address == 0x41002) {
                plc_status_word_41002_[i] = false;
            }
        }
    }


    // 수정된 값 다시 쓰기
    // setMODBUSComm(true);
    if(1) {
    modbus_status = _modbus->modbus_write_registers(register_address, 1, &current_value);
    } else {
        modbus_status = _modbus->modbus_write_register(register_address, current_value);
    }
    // setMODBUSComm(false);

    if (modbus_status == 0) {
        if(0) {
        ROS_LOG_INFO("[%s] PLC Task Status Updated: Bit %d (%s) -> %s",
                     __func__, bit_address, is_lsb ? "LSB" : "MSB", status ? "ON" : "OFF");
        }

        if(0) {
            // 이진수 출력
            std::bitset<16> binary_value(current_value);
            std::string bit_string = binary_value.to_string();
            std::string formatted_bit_string =
                bit_string.substr(0,4) + " " +
                bit_string.substr(4,4) + " " +
                bit_string.substr(8,4) + " " +
                bit_string.substr(12,4);

            ROS_LOG_WARN("[%s] Written current_value (binary): %s", __func__, formatted_bit_string.c_str());
        }
        return true;
    } else {
        ROS_LOG_INFO("[%s] Failed to update PLC Task Status, error code: %i", __func__, modbus_status);
        setMODBUSComm(true);
        return false;
    }
}







bool QNode::setPLCModbusRotationAngle(uint16_t register_address, uint16_t bit_address, double angle)
{
    ROS_LOG_WARN("[%s] !!!", __func__);

#if IS_PLC_COMMUNICATION
    // 허용 범위 초과 시 오류 처리
    if (angle < -180.0f || angle > 180.0f) {
        ROS_LOG_INFO("Rotation angle out of range (-180.0 to 180.0)");
        return false;
    }

    // 각도를 0.1도 단위로 변환하여 INT16 값으로 저장
    int16_t write_value = static_cast<int16_t>(std::round(angle * 10));
    uint16_t modbus_value;
    if(1) { // uint16_t Modbus 슬레이브가 INT16 지원할 경우 (변환을 명시적으로 수행), uint16_t*로 변환하면 음수 값이 2의 보수 형식으로 해석됨
        modbus_value = static_cast<uint16_t>(write_value);  // 명시적 변환 -32768 ~ 32767
    } else { // Modbus 슬레이브가 부호 없는 값만 지원할 경우 (양수 변환)
        modbus_value = static_cast<uint16_t>(write_value >= 0 ? write_value : (write_value + 65536));  // 음수 처리 0~65535
    }

    // PLC로 전송
    // setMODBUSComm(true); // M00000 트리거
    int modbus_status = _modbus->modbus_write_registers(register_address, 1, &modbus_value);
    // setMODBUSComm(false);

    if (modbus_status == 0) {
        ROS_LOG_INFO("Rotation angle written to PLC successfully: %f degrees", angle);
        return true;
    } else {
        ROS_LOG_INFO("Failed to write rotation angle to PLC, error code: %i", modbus_status);
        setMODBUSComm(true);
        return false;
    }

#else
    if (angle < -180.0f || angle > 180.0f) {
        ROS_LOG_INFO("Rotation angle out of range (-180.0 to 180.0)");
        return false;
    }

    ROS_LOG_INFO("[PLC SIMULATION] Rotation angle written to PLC successfully: %f degrees", angle);
    return true;
#endif

}

bool QNode::setPLCModbusBarcodeWrite(uint16_t register_address, uint16_t bit_address, const std::string& barcode)
{
    ROS_LOG_WARN("[%s] !!!", __func__);

    if (barcode.empty() || barcode.size() > 20) {
        ROS_LOG_INFO("Barcode length invalid (1~20 characters allowed)");
        return false;
    }

    uint16_t write_value[10] = {0}; // 최대 20자 (2바이트씩 저장, 총 10개의 16비트 WORD 필요)
    size_t length = barcode.size();

    // 바코드 데이터를 16비트 단위로 변환 (Big Endian 저장)
    for (size_t i = 0; i < length; i += 2) {
        uint16_t low = static_cast<uint16_t>(barcode[i]);  // 첫 번째 문자 (하위 8비트)
        uint16_t high = (i + 1 < length) ? static_cast<uint16_t>(barcode[i + 1]) : 0x00; // 두 번째 문자 (상위 8비트)
        write_value[i / 2] = (low << 8) | high; // Big Endian 변환 (올바른 순서)
    }

    // 남은 공간을 NULL (0x0000)으로 채움 (잘못된 종료 처리 방지)
    for (size_t j = (length + 1) / 2; j < 10; j++) {
        write_value[j] = 0x0000;
    }

    // PLC로 데이터 전송
    // setMODBUSComm(true); // M00000 트리거
    int modbus_status = _modbus->modbus_write_registers(register_address, 10, write_value);
    // setMODBUSComm(false);

    if (modbus_status == 0) {
        ROS_LOG_INFO("Barcode data written to PLC successfully at 0x%X", register_address);
        return true;
    } else {
        ROS_LOG_INFO("Failed to write barcode to PLC, error code: %i", modbus_status);
        setMODBUSComm(true);
        return false;
    }
}

//// Read
bool QNode::PLCModbusReadStatusFromAMR(uint16_t register_address, uint16_t bit_address, bool &status_read)
{
    ROS_LOG_WARN("[%s] !!!", __func__);

    uint16_t current_value;
    int modbus_status;

    // 현재 레지스터 값 읽기 (비트 값을 확인하기 위해 전체 레지스터를 읽어야 함)
    // setMODBUSComm(true); // M00000 트리거
    modbus_status = _modbus->modbus_read_input_registers(register_address, 1, &current_value);
    // setMODBUSComm(false);

    if (modbus_status != 0) {
        ROS_LOG_INFO("[%s] Failed to read PLC register, error code: %i", __func__, modbus_status);
        setMODBUSComm(true);
        return false;
    }

    // 특정 비트 추출
    bool bit_status = (current_value >> bit_address) & 0x01;
    status_read = bit_status;

    ROS_LOG_INFO("[%s] Register: 0x%X, Bit: %u, Value: %u", __func__, register_address, bit_address, bit_status);

    return true;
}



//// Read only 1 byte
bool QNode::PLCModbusReadStatusFromAMR2(const uint16_t register_address, uint16_t &current_value)
{
    ROS_LOG_WARN("[%s] !!!", __func__);

    int modbus_status;
    // 현재 레지스터 값 읽기 (비트 값을 확인하기 위해 전체 레지스터를 읽어야 함)
    // setMODBUSComm(true); // M00000 트리거

    //// 250804 기존
    // modbus_status = _modbus->modbus_read_input_registers(register_address, 1, &current_value);

    //// 250804 아래로 변경
#if IS_PLC_LS_XG5000
    modbus_status = _modbus->modbus_read_input_registers(register_address, 1, &current_value);
#else
    // modbus_status = _modbus->modbus_read_holding_registers(register_address, 1, &current_value);
    modbus_status = _modbus->modbus_read_input_registers(register_address, 1, &current_value);
#endif

        // setMODBUSComm(false);

        if (modbus_status != 0) {
            ROS_LOG_INFO("[%s] Failed to read PLC register, error code: %i", __func__, modbus_status);
            setMODBUSComm(true);
        return false;
    }

    return true;
        }

// void QNode::monitorPLCStatus(){

//     if(is_plc_simulation_mode_) {
//         ROS_LOG_ERROR("[%s] PLC SIMULATION MODE...", __func__);
//         return; // Simulation
//     }


//     //// LS산전 XG5000의 경우, D레지스터
//     // D 레지스터 번호 = (Modbus 주소 - 0x40000) / 10
//     // 예시) (0x41000 - 0x40000) / 10 = 0x1000 / 10 = 0x100 = 256 → D04096 (4096), 4096 = 256 * 16 → D04096
//     //// 사실상 Modbus 주소를 10으로 나눈 값이 D 레지스터의 16배수로 보이므로, 단순화를 위해 다음 수식을 사용
//     // D 번호 = (Modbus 주소 - 0x40000) / 1
//     // D 레지스터 = D00000 + (Modbus 주소 - 0x40000)
//     // D 레지스터 번호 = (Modbus 주소 - 0x40000) * 1 → D 번호 = base_offset

//     ROS_LOG_WARN("[%s]...", __func__);

//     // 0x40000 → D00000 = 0000
//     // 0x41000 → D04096 = 4096
//     // 0x42000 → D08192 = 8192
//     // setMODBUSComm(true); // M00000 트리거
//     if(1){
//         // printf("*******************************************************************************\n");
//         // printf("\n************************** Monitoring PLC STATUS(2000)******************************\n");
//         // printf("*******************************************************************************\n");

//         uint16_t current_value;
//         int modbus_status;

//         // 현재 레지스터 값 읽기 (비트 값을 확인하기 위해 전체 레지스터를 읽어야 함)
// #if IS_PLC_LS_XG5000
//         uint16_t word_address = 0x42000;
// #else
//         uint16_t word_address = 2000;
// #endif
//         // setMODBUSComm(true); // M00000 트리거
//         modbus_status = _modbus->modbus_read_input_registers(word_address, 1, &current_value);
//         // setMODBUSComm(false);

//         // ROS_LOG_WARN("Modbus IP Address: %s, Port: %u", ip_modbus_.c_str(), port_modbus_);
//         if (modbus_status != 0) {
//             ROS_LOG_INFO("[%s] Failed to read PLC register, error code: %i", __func__, modbus_status);
//             setMODBUSComm(true);
//             return;
//         }

//         ////
//         for(int i = 0; i < 16; ++i) { // 0~13
//             // 특정 비트 추출
//             uint16_t bit_address = static_cast<uint16_t>(i);
//             bool bit_status = (current_value >> bit_address) & 0x01;
//             // D 레지스터 번호 변환

// #if IS_PLC_LS_XG5000
//             uint16_t d_register_number = word_address - 0x40000;
// #else
//             uint16_t d_register_number = word_address;
// #endif

//             // flag update
//             if (plc_status_word_42000_[i] != bit_status) {
//                 plc_status_word_42000_[i] = bit_status;
//                 // ROS_LOG_WARN("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u [Modified]",
//                 //                 __func__, word_address, word_address, d_register_number, bit_address, bit_status);
//             } else {
//                 // ROS_LOG_INFO("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u",
//                 //                 __func__, word_address, word_address, d_register_number, bit_address, bit_status);
//             }
//         }

//     }

//     if(1){
//         // printf("\n************************** Monitoring PLC STATUS(2001)******************************\n");
//         // printf("*******************************************************************************\n");

//         uint16_t current_value;
//         int modbus_status;

//         // 현재 레지스터 값 읽기 (비트 값을 확인하기 위해 전체 레지스터를 읽어야 함)
// #if IS_PLC_LS_XG5000
//         uint16_t word_address = 0x42001;
// #else
//         uint16_t word_address = 2001;
// #endif
//         // setMODBUSComm(true); // M00000 트리거
//         modbus_status = _modbus->modbus_read_input_registers(word_address, 1, &current_value);
//         // setMODBUSComm(false);

//         // ROS_LOG_WARN("Modbus IP Address: %s, Port: %u", ip_modbus_.c_str(), port_modbus_);
//         if (modbus_status != 0) {
//             ROS_LOG_INFO("[%s] Failed to read PLC register, error code: %i", __func__, modbus_status);
//             setMODBUSComm(true);
//             return;
//         }

//         // **int16_t로 변환**
//         int16_t signed_value = static_cast<int16_t>(current_value);

//         // 로그 출력
//         // ROS_LOG_WARN("[%s] Word address: (0x4%X), Signed Value: %d", __func__, word_address, signed_value);

// //         ////
// //         for(int i = 0; i < 1; ++i) {
// //             // 특정 비트 추출
// //             uint16_t bit_address = static_cast<uint16_t>(i);
// //             bool bit_status = (current_value >> bit_address) & 0x01;
// //             // D 레지스터 번호 변환
// // #if IS_PLC_LS_XG5000
// //             uint16_t d_register_number = word_address - 0x40000;
// // #else
// //             uint16_t d_register_number = word_address;
// // #endif
// //             // flag update
// //             if (plc_status_word_42001_[i] != bit_status) {
// //                 plc_status_word_42001_[i] = bit_status;
// //                 ROS_LOG_WARN("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u [Modified]",
// //                                 __func__, word_address, word_address, d_register_number, bit_address, bit_status);
// //             } else {
// //                 ROS_LOG_INFO("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u",
// //                                 __func__, word_address, word_address, d_register_number, bit_address, bit_status);
// //             }
// //         }

//     }

//     if(1){
//         // printf("\n************************** Monitoring PLC STATUS(2002)******************************\n");
//         // printf("*******************************************************************************\n");

//         uint16_t current_value;
//         int modbus_status;

//         // 현재 레지스터 값 읽기 (비트 값을 확인하기 위해 전체 레지스터를 읽어야 함)
// #if IS_PLC_LS_XG5000
//         uint16_t word_address = 0x42002;
// #else
//         uint16_t word_address = 2002;
// #endif
//         // setMODBUSComm(true); // M00000 트리거
//         modbus_status = _modbus->modbus_read_input_registers(word_address, 1, &current_value);
//         // setMODBUSComm(false);

//         // ROS_LOG_WARN("Modbus IP Address: %s, Port: %u", ip_modbus_.c_str(), port_modbus_);
//         if (modbus_status != 0) {
//             ROS_LOG_INFO("[%s] Failed to read PLC register, error code: %i", __func__, modbus_status);
//             setMODBUSComm(true);
//             return;
//         }

//         ////
//         for(int i = 0; i < 7; ++i) {
//             // 특정 비트 추출
//             uint16_t bit_address = static_cast<uint16_t>(i);
//             bool bit_status = (current_value >> bit_address) & 0x01;
//             // D 레지스터 번호 변환
// #if IS_PLC_LS_XG5000
//             uint16_t d_register_number = word_address - 0x40000;
// #else
//             uint16_t d_register_number = word_address;
// #endif
//             // flag update
//             if (plc_status_word_42002_[i] != bit_status) {
//                 plc_status_word_42002_[i] = bit_status;
//                 // ROS_LOG_WARN("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u [Modified]",
//                 //                 __func__, word_address, word_address, d_register_number, bit_address, bit_status);
//             } else {
//                 // ROS_LOG_INFO("[%s] Word address: (0x4%X), Register: 0x%X (D%05u), Bit address: %u, Value: %u",
//                 //                 __func__, word_address, word_address, d_register_number, bit_address, bit_status);
//             }
//         }
//         // printf("******************************************************************\n");
//         // printf("*******************************************************************************\n\n");

//     }
//     // setMODBUSComm(false);


// }




////////////////////////////////////////

void QNode::LLMRecording(){
    ROS_LOG_INFO("######################### LLM #########################");
    ROS_LOG_INFO("Recording ...");

    auto req = make_shared<std_srvs::srv::Trigger::Request>();

    auto result = client_record_voice_->async_send_request(req);
    // auto status = result.wait_for(chrono::milliseconds(100));
}

//////////////////////////////////////////////////////////////////////////////////
//////////////////////////// Robot Calibration /////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
void QNode::CalSaveCallback(std_msgs::msg::Bool is_saved) {
    if (is_saved.data) {
        is_save_fin_ = true;
    }
    else {
        RCLCPP_INFO(rclcpp::get_logger(log_prefix),"saving ply...");
    }
}
void QNode::CalCheckCalibration(std::string folder_name, std::array<double, CS_DOF> x_current) {
    fname = folder_name;
    auto cal_msg = std_msgs::msg::String();
    std::string x = "";

    for (int i = 0; i < CS_DOF; i++) {
        x = x + ", " + std::to_string(x_current[i]);
    }
    cal_msg.data = fname + x + "check";
    cal_scan_pub->publish(cal_msg);
}

void QNode::CalCapBaseCallback(std::string folder_name) {
    fname = folder_name;
    auto cal_msg = std_msgs::msg::String();
    cal_msg.data = fname + "base";
    cal_scan_pub->publish(cal_msg);
}

void QNode::CalCheckResultCallback(std_msgs::msg::Float64MultiArray mesured_err) {
	cal_check_err = mesured_err.data;
    cout<<"Check Result recived!"<<endl<<"Push Refresh button"<<endl;
}
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

#if BIN_PICKING_FLAG

void QNode::ScanningResultCallback(const hanyang_matching_msgs::msg::MatchingResultMsg &msg) {

    if(msg.is_scanning_and_detection_finished)
    {
        ROS_LOG_WARN("*********************************************************");
        ROS_LOG_INFO("ScanningResultCallback! - Scanning and Detection Finished!");
        if(msg.is_detection_success) {
            ROS_LOG_WARN("Detection success!");
        } else {
            ROS_LOG_WARN("Detection failure!");
        }
        ROS_LOG_WARN("*********************************************************");
        is_scanning_and_detection_finished = true;
    }

    if(msg.is_open3d_processing_finished)
    {
        ROS_LOG_WARN("*********************************************************");
        ROS_LOG_INFO("ScanningResultCallback! - Open3D Processing Finished!");
        ROS_LOG_WARN("*********************************************************");
        is_open3d_processing_finished = true;
    }
}

void QNode::templateMatchingCallback(const hanyang_matching_msgs::msg::MatchingResultMsg &msg) {

    if(msg.is_pose)
    {
        ROS_LOG_INFO("*********************************************************");
        ROS_LOG_INFO("*********************************************************");
        ROS_LOG_INFO("*********************************************************");
        ROS_LOG_INFO("Topic Subscribe!!! - templateMatchingCallback received! (is_pose: true)");
        ROS_LOG_INFO("pose feasible!");
        ROS_LOG_INFO("*********************************************************");
        ROS_LOG_INFO("*********************************************************");
        ROS_LOG_INFO("*********************************************************");

        m_bin_picking_node->detected_pose_ = msg.pose;
        m_bin_picking_node->detected_sub_pose_ = msg.sub_pose;
        m_bin_picking_node->detected_zig_pose_ = msg.zig_pose;
        m_bin_picking_node->approach_distance = msg.approach_distance;
        m_bin_picking_node->matching_accuracy = msg.matching_accuracy;
        m_bin_picking_node->matching_accuracy_limit = msg.matching_accuracy_limit;
        m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_open_length = msg.gripper_open_length;
        m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_close_length = msg.gripper_close_length;
        m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_tip_index = msg.gripper_tip_index;
        m_bin_picking_node->detected_mask_num = msg.detected_mask_num;
        bool is_grasping_pose_flipped = msg.is_grasping_pose_flipped;
        m_bin_picking_node->is_matching_finished = true; // 매칭이 끝났는지 여부에 대한 flag
        m_bin_picking_node->is_grasping_pose_cal_finished = true; // Topic cal.이 끝났는지 여부에 대한 flag
        m_bin_picking_node->is_grasping_pose_feasible_ = true; // Pose Feasible 여부

        printf("\n---------------------------------------------------\n");
        printf("------- *** (Topic subscribe) ***  --------\n");
        printf("------- *** Template matching results ***  --------\n");
        printf("---------------------------------------------------\n");
        ROS_LOG_INFO("Grasping pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", m_bin_picking_node->detected_pose_[0], m_bin_picking_node->detected_pose_[1], m_bin_picking_node->detected_pose_[2], m_bin_picking_node->detected_pose_[3], m_bin_picking_node->detected_pose_[4], m_bin_picking_node->detected_pose_[5]);
        ROS_LOG_INFO("Grasping sub. pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", m_bin_picking_node->detected_sub_pose_[0], m_bin_picking_node->detected_sub_pose_[1], m_bin_picking_node->detected_sub_pose_[2], m_bin_picking_node->detected_sub_pose_[3], m_bin_picking_node->detected_sub_pose_[4], m_bin_picking_node->detected_sub_pose_[5]);
        ROS_LOG_INFO("Approach distance: %0.3f", m_bin_picking_node->approach_distance);
        ROS_LOG_INFO("Matching accuracy: %0.3f", m_bin_picking_node->matching_accuracy);
        ROS_LOG_INFO("Matching accuracy limit: %0.3f", m_bin_picking_node->matching_accuracy_limit);
        ROS_LOG_INFO("Gripper_open_length: %u", m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_open_length);
        ROS_LOG_INFO("Gripper_close_length: %u", m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_close_length);
        ROS_LOG_INFO("Gripper_tip_index: #%u", m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_grasping_parameter.gripper_tip_index);
        ROS_LOG_INFO("Detected mask count: %i", m_bin_picking_node->detected_mask_num);
        if(is_grasping_pose_flipped) printf("******** ******** Pose flipped! ******** ********\n");
        printf("---------------------------------------------------\n");
        ROS_LOG_INFO("Zig pose: %0.5f, %0.5f, %0.5f, %0.3f, %0.3f, %0.3f", m_bin_picking_node->detected_zig_pose_[0], m_bin_picking_node->detected_zig_pose_[1], m_bin_picking_node->detected_zig_pose_[2], m_bin_picking_node->detected_zig_pose_[3], m_bin_picking_node->detected_zig_pose_[4], m_bin_picking_node->detected_zig_pose_[5]);
        printf("---------------------------------------------------\n");
        printf("---------------------------------------------------\n");
        printf("---------------------------------------------------\n\n");
        ROS_LOG_INFO("Template Matching Success!(Callback11)");
    }
    else
    {
        m_bin_picking_node->approach_distance = 0.0;
        m_bin_picking_node->matching_accuracy = 0.0;
        m_bin_picking_node->detected_mask_num = msg.detected_mask_num;
        printf("\n---------------------------------------------------\n");
        printf("------- *** (Topic subscribe) ***  --------\n");
        printf("------- *** Template matching results ***  --------\n");
        printf("---------------------------------------------------\n");
        ROS_LOG_INFO("Detected mask count: %i", m_bin_picking_node->detected_mask_num);
        ROS_LOG_INFO("Template Matching Failure!(Callback) - templateMatchingCallback()");

        ROS_LOG_INFO("*********************************************************");
        ROS_LOG_INFO("*********************************************************");
        ROS_LOG_INFO("*********************************************************");
        ROS_LOG_INFO("Topic Subscribe!!! - templateMatchingCallback received! (is_pose: false)");
        ROS_LOG_INFO("pose infeasible!");
        ROS_LOG_INFO("*********************************************************");
        ROS_LOG_INFO("*********************************************************");
        ROS_LOG_INFO("*********************************************************");

        m_bin_picking_node->is_matching_finished = true; // UI에서 버튼 체크 해제, 매칭이 끝났는지 여부에 대한 flag, Dialog subCallback()에서 사용됨
        m_bin_picking_node->is_grasping_pose_cal_finished = true; // Topic cal.이 끝났는지 여부에 대한 flag, Dialog subCallback()에서 사용됨
        m_bin_picking_node->is_grasping_pose_feasible_ = false; // Pose Feasible 여부, doGrasping() 동작 결정하는 flag

    }
}

bool QNode::setCurrentObjectIndex(size_t idx_now) {

    //// update current target object index
    m_bin_picking_node->before_target_object_idx_ = m_bin_picking_node->current_target_object_idx_;
    m_bin_picking_node->current_target_object_idx_ = idx_now;

    if(m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching)
    {
        ROS_LOG_WARN("do_single_matching: TRUE ");
    } else {
        ROS_LOG_WARN("do_single_matching: FALSE ");
    }

    return true;
}

bool QNode::setSamInfoUpdateCurrentObject() {
    // Target object
    m_bin_picking_node->publishUICommandLearningMsg(m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->m_scan_parameter);

    return true;
}

bool QNode::setRobotInfoUpdateCurrentObject() {
    //// update current target object index
    // setCurrentObjectIndex(idx_now);
    //// update data
    // Robot TCP
    size_t tcp_idx = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->tcp_changing_id;
    sendChangeRobotTCPCommand(tcp_idx);
    // Tool id
    tool_changing_attach_id = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->tool_changing_attach_id; // # slave #1, 2, ...
    tool_changing_detach_id = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->tool_changing_detach_id; // # slave #1, 2, ...
    // Tip id
    tip_changing_attach_id = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->tip_changing_attach_id; // # slave #1, 2, ...
    tip_changing_detach_id = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[m_bin_picking_node->current_target_object_idx_]->tip_changing_detach_id; // # slave #1, 2, ...
    return true;
}


// bool QNode::doSingleScanningZIVID(bool b_scan, taskScanningParameter &scan_parameter)
// {
//     auto req = make_shared<hanyang_matching_msgs::srv::ZividDoScan::Request>();
// 	req->scan = b_scan;
//     req->target_id = scan_parameter.target_id; // 추후 task planner와 통합 필요 // 1: cpu, 2: ram, 3: hdmi, 4: usb, 5: power, 6: ssd, 7: cooler
//     req->target_name = scan_parameter.target_name;
//     req->is_mask_pixel_fixed = scan_parameter.is_mask_pixel_fixed;
//     req->do_scan_sampling = scan_parameter.do_scan_sampling;
//     req->sampling_num = scan_parameter.sampling_num;
//     req->is_base_frame_unknown = scan_parameter.is_base_frame_unknown;
//     req->do_save_data = scan_parameter.do_save_data;
//     req->do_save_mrcnn_learning_data = scan_parameter.do_save_mrcnn_learning_data;
//     req->do_load_data = scan_parameter.do_not_scan_do_load_data;
//     req->skip_mask_rcnn_processing = scan_parameter.skip_mask_rcnn_processing;

//     // Only using fixed mask pixel without using mask rcnn
//     req->mask_pixel_x = scan_parameter.mask_pixel_list[0];
//     req->mask_pixel_y = scan_parameter.mask_pixel_list[1];
//     req->mask_pixel_scale_x = scan_parameter.mask_pixel_list[2];
//     req->mask_pixel_scale_y = scan_parameter.mask_pixel_list[3];

//     req->learning_data_idx = scan_parameter.learning_data_idx;

//     req->scan_position = getRobotAActualQ(); // scanning joint position
//     RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Do scanning! (grasping parts id: %d)", scan_parameter.target_id);

//     auto result = zividScanningClient_->async_send_request(req);
//     auto status = result.wait_for(chrono::milliseconds(3000));

//     if (status == future_status::timeout) {
//         ROS_LOG_ERROR("Failed to call service\n\tFunction: %s\n\tFile name: [%s] : %d)", __FUNCTION__, __FILE__, __LINE__);
//         return false;
//     }
//     else{
//         if (result.get()->is_detected == true)
//         {
//             if (taskParam_robotA_.isDetectionFin == false && is_task_mode_)
//             {
//                 taskParam_robotA_.isDetectionFin = true;
//                 task_param_.is_ready_matching = true;
//             }
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Zivid Scanning Success!(Callback)");
//         }
//         else
//         {
//            RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Zivid Scanning Failure!(Callback)");
//         }
//     }

// }

// bool QNode::doTemplateMatchingBinPicking(taskTemplateMatchingParameter &parameter, bool b_matching, bool do_scan_sampling, size_t sampling_num) {
// 	auto req = make_shared<hanyang_matching_msgs::srv::DoTemplateMatching::Request>();
// 	req->do_matching = b_matching;
//     req->do_scan_sampling = do_scan_sampling;
//     req->debug_mode = parameter.debug_mode;
//     req->sampling_num = sampling_num;
//     req->is_base_frame_unknown = parameter.is_base_frame_unknown;

//     req->robot_id = m_robot_id;
//     RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Do template matching (bin picking)!");
//     is_matching_finished = false;

//     auto result = doTemplateMatchingBinPickingClient_->async_send_request(req);
//     auto status = result.wait_for(chrono::milliseconds(20000));

//     if (status == future_status::timeout){
//         approach_distance = 0.0;
//         matching_accuracy = 0.0;
//         is_matching_finished = false;
//         RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Template Matching - service call Failure!");
//     }

//     else{
//         auto recv = result.get();
//         if (recv->is_pose == true)
//         {
//             if (taskParam_robotA_.isDetectionFin == false && is_task_mode_)
//             {
//                 taskParam_robotA_.isDetectionFin = true;
//             }
//             detected_pose_ = recv->pose;
//             detected_sub_pose_ = recv->sub_pose;
//             approach_distance = recv->approach_distance;
//             matching_accuracy = recv->matching_accuracy;
//             matching_accuracy_limit = recv->matching_accuracy_limit;
//             grasping_parameter.gripper_open_length = recv->gripper_open_length;
//             grasping_parameter.gripper_close_length = recv->gripper_close_length;
//             detected_mask_num = recv->detected_mask_num;
//             bool is_grasping_pose_flipped = recv->is_grasping_pose_flipped;
//             isReadyGrasping = true;

//             printf("\n---------------------------------------------------\n");
//             printf("------- *** Template matching results ***  --------\n");
//             printf("---------------------------------------------------\n");
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Grasping pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", detected_pose_[0], detected_pose_[1], detected_pose_[2], detected_pose_[3], detected_pose_[4], detected_pose_[5]);
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Grasping sub. pose: %0.5f %0.5f %0.5f %0.3f %0.3f %0.3f", detected_sub_pose_[0], detected_sub_pose_[1], detected_sub_pose_[2], detected_sub_pose_[3], detected_sub_pose_[4], detected_sub_pose_[5]);
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Approach distance: %0.3f", approach_distance);
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Matching accuracy: %0.3f", matching_accuracy);
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Matching accuracy limit: %0.3f", matching_accuracy_limit);
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Gripper_open_length: %u", grasping_parameter.gripper_open_length);
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Gripper_close_length: %u", grasping_parameter.gripper_close_length);
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Detected mask count: %i", detected_mask_num);
//             if(is_grasping_pose_flipped) printf("******** ******** Pose flipped! ******** ********\n");
//             printf("---------------------------------------------------\n");
//             printf("---------------------------------------------------\n");
//             printf("---------------------------------------------------\n\n");
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Template Matching Success!(Callback11)");
//         }
//         else
//         {
//             approach_distance = 0.0;
//             matching_accuracy = 0.0;
//             detected_mask_num = recv->detected_mask_num;
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Detected mask count: %i", detected_mask_num);
//             RCLCPP_INFO(rclcpp::get_logger(log_prefix),"Template Matching Failure!(Callback): Scanning Recovery task!");
//         }
//         is_matching_finished = true;
//     }

// }
#endif
// sb 25.03.06
void QNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
    // 최신 카메라 정보를 저장 (깊은 복사)
    camera_info_msg_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg);
}

void QNode::imageCallback(const sensor_msgs::msg::Image &msg) {
    if(SW_MODE_GRAPHY) {
        if (is_camera_mode_on_ && !msg.data.empty()) {
            // UI용 QImage 변환
            QImage source_image(msg.data.data(), msg.width, msg.height, QImage::Format_RGB888);
            QImage processed_image = source_image.copy();

            // QImage를 grayscale로 변환
            QImage gray_image = processed_image.convertToFormat(QImage::Format_Grayscale8);


            int black_threshold = 116;  // 검은색 임계값

            // 빠른 픽셀 접근을 위해 scanLine() 사용
            for (int y = 0; y < gray_image.height(); ++y) {
                uchar *line = gray_image.scanLine(y);
                for (int x = 0; x < gray_image.width(); ++x) {
                    int idx = x; // Grayscale이므로 단일 채널 접근
                    uchar gray_value = line[idx];

                    if (gray_value > black_threshold) {
                        line[idx] = 255; // 임계값 초과 → 흰색
                    } else {
                        line[idx] = 0;   // 임계값 이하 → 완전 검은색
                    }
                }
            }


            #if CAM_TYPE_LOGITEC_BRIO
            QImage tmp_image = processed_image.mirrored(true, true);
            q_image_ = tmp_image;
            Q_EMIT newImageAvailable(q_image_);
            #endif
            // ROS 이미지 메시지 생성 및 변환
            sensor_msgs::msg::Image ros_image;
            ros_image.header = msg.header;
            ros_image.height = gray_image.height();
            ros_image.width = gray_image.width();
            ros_image.encoding = "mono8";  // <- mono8로 설정
            ros_image.is_bigendian = false;
            ros_image.step = gray_image.width(); // 단일 채널이므로 width와 동일

            // QImage 데이터를 std::vector<uint8_t>로 변환
            ros_image.data.assign(gray_image.bits(), gray_image.bits() + gray_image.sizeInBytes());

            // 카메라 정보 퍼블리시
            if (camera_info_msg_) {
                camera_info_msg_->header.stamp = ros_image.header.stamp;
                camera_infoPublisher_->publish(*camera_info_msg_);
                imagePublisher_->publish(ros_image);
            } else {
                RCLCPP_WARN(node_->get_logger(), "CameraInfo is not available, skipping publishing.");
            }

        }
    } else {
        QImage source_image(msg.data.data(), msg.width, msg.height, QImage::Format_RGB888);
        q_image_ = source_image;  //initialize image

    }

}


void QNode::scannerImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cout << "zivid image recived!" << endl;
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
        cv::Mat mat = cv_ptr->image;

        // BGR → RGB 변환
        cv::Mat rgbMat;
        cv::cvtColor(mat, rgbMat, cv::COLOR_BGR2RGB);

        // 메모리 연속성 보장 (QImage 변환 시 문제 방지)
        if (!rgbMat.isContinuous()) {
            rgbMat = rgbMat.clone();
        }

        // OpenCV Mat → QImage 변환
        QImage image(rgbMat.data, rgbMat.cols, rgbMat.rows, rgbMat.step, QImage::Format_RGB888);

        emit UpdateImage(image, "zivid");
    } catch (cv_bridge::Exception& e) {
        std::cerr << "[scannerImageCallback] cv_bridge exception: " << e.what() << std::endl;
    }
}



bool QNode::aprilTagDetection(uint tag_num) {
    lock_guard<mutex> lg(mutex_4tag_);
std::cout << " TAG_DETETED: " <<  std::boolalpha << tag_info_temp.is_tag_detected <<std::endl;
    std::cout << " TAG_ID: " <<  stoi(tag_info_temp.id_str.substr(9, 3)) <<std::endl;

    if(tag_info_temp.is_tag_detected) {
        ROS_LOG_WARN("ee to tag: %s, Position: [%f, %f, %f], RPY: [%f, %f, %f]",
                        tag_info_temp.id_str.c_str(),
                        tag_info_temp.pose_ee_tag.element_[0], tag_info_temp.pose_ee_tag.element_[1], tag_info_temp.pose_ee_tag.element_[2],
                        tag_info_temp.pose_ee_tag.element_[3] * kRad2Deg, tag_info_temp.pose_ee_tag.element_[4] * kRad2Deg, tag_info_temp.pose_ee_tag.element_[5] * kRad2Deg);
        ROS_LOG_WARN("base to tag: %s, Position: [%f, %f, %f], RPY: [%f, %f, %f]",
                        tag_info_temp.id_str.c_str(),
                        tag_info_temp.pose_base_tag.element_[0], tag_info_temp.pose_base_tag.element_[1], tag_info_temp.pose_base_tag.element_[2],
                        tag_info_temp.pose_base_tag.element_[3] , tag_info_temp.pose_base_tag.element_[4], tag_info_temp.pose_base_tag.element_[5]);
    }

    // for (const auto& detection : tag_detections) {
    //     if (detection.id == tag_num) {

    //         const auto& pose = detection.pose.pose.pose; // 태그의 Pose 정보

    //         double quaternion[4] = {
    //             pose.orientation.x,
    //             pose.orientation.y,
    //             pose.orientation.z,
    //             pose.orientation.w
    //         };
    //         double rpy[3];

    //         ModelMatrix::quaternion2rpy(quaternion, rpy);

    //         // RCLCPP_INFO로 정보 로깅
    //         RCLCPP_INFO(node_->get_logger(), "Detection ID: %d, Position: [%f, %f, %f]\nOrientation: [%f, %f, %f, %f], RPY: [%f, %f, %f]",
    //                     detection.id, pose.position.x, pose.position.y, pose.position.z,
    //                     pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
    //                     rpy[0]/M_PI*180, rpy[1]/M_PI*180, rpy[2]/M_PI*180);
    //                     // quaternion[0], quaternion[1], quaternion[2]);

    //         return true;
    //     }
    // }
    return false;
}

//// NOTICE: Rx, Rz는 fabs() 처리되어있음.
bool QNode::checkWorkspace(const std::vector<double> pose, const Eigen::MatrixXd &workspace)
{
    double x_min = workspace(0, 0); double x_max = workspace(0, 1); // m
    double y_min = workspace(1, 0); double y_max = workspace(1, 1); // m
    double z_min = workspace(2, 0); double z_max = workspace(2, 1);// m
    double rx_min = workspace(3, 0); double rx_max = workspace(3, 1); // deg
    double ry_min = workspace(4, 0); double ry_max = workspace(4, 1); // deg
    double rz_min = workspace(5, 0); double rz_max = workspace(5, 1); // deg

    if ((pose[0] >= x_min && pose[0] <= x_max) &&
        (pose[1] >= y_min && pose[1] <= y_max) &&
        (pose[2] >= z_min && pose[2] <= z_max) &&
        (fabs(pose[3]) >= rx_min && fabs(pose[3]) <= rx_max) &&
        (pose[4] >= ry_min && pose[4] <= ry_max) &&
        (fabs(pose[5]) >= rz_min && fabs(pose[5]) <= rz_max)) {
        ROS_LOG_INFO("Check the pose in the workspace - OK");
        return true;
    } else {
        ROS_LOG_WARN("Check the pose in the workspace - out of range");
        ROS_LOG_WARN("pose: %0.4f, %0.4f, %0.4f, %0.1f, %0.1f, %0.1f\n", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
        return false;
    }
}

void QNode::setLogMessageStr(std::string &str) {
    task_log_str_ = str;
    ROS_LOG_INFO(task_log_str_.c_str());
}

TaskInfo QNode::getTaskInfo() {
    return taskInfo;
}

void QNode::setTaskInfo(TaskInfo taskInfo) {
    this->taskInfo = taskInfo;
    Q_EMIT TaskInfoSet();
}

void QNode::setTaskInfoForMonitorAtStart(TaskInfo taskInfo) {
    this->taskInfo = taskInfo;
}

void QNode::setTaskInfoForMonitor(TaskInfo taskInfo) {
    this->taskInfo = taskInfo;
    Q_EMIT TaskInfoSet();
}


void QNode::sendTaskInfoLog(std::string &str) {
    ROS_LOG_INFO(str.c_str());
    setLogMessageStr(str);
    // Q_EMIT sendLogMessageStr(str);
    // SEND_WARN_TO_MONITOR(str);
    LogInfo logInfo;
    logInfo._log_level = LogLevel::INFO;
    logInfo._log_message = str;
    Q_EMIT sendLogMessage(logInfo);
}

void QNode::sendTaskErrorLog(std::string &str) {
    ROS_LOG_INFO(str.c_str());
    setLogMessageStr(str);
    LogInfo logInfo;
    logInfo._log_level = LogLevel::ERROR;
    logInfo._log_message = str;
    Q_EMIT sendLogMessage(logInfo);
}

void QNode::setCameraAutoFocus(bool is_auto_focus_on) {
    if(is_auto_focus_on) {
        ROS_LOG_WARN("Auto Focus ON!");
        QProcess::execute("/bin/bash", {"-c", "bash -c \"v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=1\""});
    } else {
        ROS_LOG_WARN("Auto Focus OFF!");
        QProcess::execute("/bin/bash", {"-c", "bash -c \"v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=0\""});
    }
    QProcess::execute("/bin/bash", {"-c", "bash -c \"v4l2-ctl -d /dev/video0 --list-ctrls\""});
}

void QNode::executeHyundaiLlmTask() {
    auto planner = dynamic_cast<TaskPlannerHyundaiLlm*>(task_planner_.get());
    if (planner) {
        planner->HyundaiLlmCustomTask();
    } else {
        std::cerr << "task_planner_ is not an instance of TaskPlannerHyundaiLlm!" << std::endl;
    }
}

void QNode::executeCookingTask() {
    auto planner = dynamic_cast<TaskPlannerCooking*>(task_planner_.get());
    if (planner) {
        planner->CookingCustomTask();
    } else {
        std::cerr << "task_planner_ is not an instance of TaskPlannerCooking!" << std::endl;
    }
}

void QNode::executeHanyangEngTask() {
    auto planner = dynamic_cast<TaskPlannerHanyangEng*>(task_planner_.get());
    if (planner) {
        planner->HanyangEngCustomTask();
    } else {
        std::cerr << "task_planner_ is not an instance of TaskPlannerHanyangEng!" << std::endl;
    }
}

void QNode::getDrflCurrentPose() {
    #if DRFL_CONTROL
        ////////////////////////////////////////
        //// Get Measured CS Pose (Srv)
        std::array<double, 6> pos = getRobotStateCallback(1);
        for (size_t i = 0; i < 3; ++i) {
            pos[i] *= 0.001;
        }

        if(1) {
            Eigen::Vector3f euler_angles_zyz_in(pos[3], pos[4], pos[5]); // [deg]
            Eigen::Matrix3f R_zyz = m_bp_math.ZYZEulerAnglesToRotationMatrix(euler_angles_zyz_in); // [deg]
            Eigen::Vector3f euler_angles_zyx = m_bp_math.rotationMatrixToZYXEulerAngles(R_zyz); // [deg]
            for (size_t i = 0; i < 3; ++i) { pos[i + 3] = euler_angles_zyx[i]; } // ZYX Euler angle. [deg]
            params_.meas.x = pos;
        }

    #endif
}

void QNode::rebuildTargetObjectIndexMap() {
    name_to_index_map_.clear();
    if (m_bin_picking_node == nullptr) return;
    const auto &list = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list;
    for (size_t i = 0; i < list.size(); ++i) {
        if (list[i] == nullptr) continue;
        const std::string &name = list[i]->m_scan_parameter.target_name;
        if (!name.empty()) {
            name_to_index_map_[name] = i;
            ROS_LOG_WARN("[rebuildTargetObjectIndexMap] #%zu -> %s", i, name.c_str());
        }
    }
}

size_t QNode::getTargetObjectIndex(const std::string& object_name) {
    auto it = name_to_index_map_.find(object_name);
    if (it != name_to_index_map_.end()) {
        return it->second;
    }
    // Try rebuilding once in case the map wasn't initialized yet
    rebuildTargetObjectIndexMap();
    it = name_to_index_map_.find(object_name);
    if (it != name_to_index_map_.end()) {
        return it->second;
    }
    // Fallback: linear search by target_name across current object list
    if (m_bin_picking_node) {
        const auto &list = m_bin_picking_node->m_ptr_bin_picking_node_target_object_list;
        ROS_LOG_ERROR("[getTargetObjectIndex] name '%s' not found in map. Linear scan %zu entries:", object_name.c_str(), list.size());
        for (size_t i = 0; i < list.size(); ++i) {
            if (list[i] && list[i]->m_scan_parameter.target_name == object_name) {
                ROS_LOG_WARN("[getTargetObjectIndex] Fallback linear search applied for %s -> %zu", object_name.c_str(), i);
                name_to_index_map_[object_name] = i;
                return i;
            }
            if (list[i]) {
                ROS_LOG_WARN("[getTargetObjectIndex] entry #%zu name: %s", i, list[i]->m_scan_parameter.target_name.c_str());
            }
        }
    }
    return std::numeric_limits<size_t>::max(); // 존재하지 않는 경우
}
