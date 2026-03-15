#include "mainwindow_initial_setup.hpp"
#include "ui_mainwindow_initial_setup.h"
#include <QFileDialog>
#include <QMessageBox>
#include <iostream>
#include <fstream>
#include <stdexcept>
#include <QDebug>
#include <QKeyEvent>
#include <QApplication>
#include <QImage>
#include <QPixmap>

//////////////////////////////////////////// to json /////////////////////////////////////////
nlohmann::json array_to_json(const std::array<double, 6>& vec) {
    nlohmann::json json_array = nlohmann::json::array();
    for (const auto& val : vec) {
        json_array.push_back(val);
    }
    return json_array;
}

nlohmann::json vector_to_json(const std::vector<double>& vec) {
    nlohmann::json json_vec = nlohmann::json::array();
    for (double val : vec) {
        json_vec.push_back(val);
    }
    return json_vec;
}

nlohmann::json cs_double_vector_to_json(const std::vector<CsDouble>& cs_double_vector) {
    nlohmann::json json_array = nlohmann::json::array();  // JSON 배열 생성

    // 각 CsDouble을 순회하면서 JSON 배열로 변환
    for (const auto& cs_double : cs_double_vector) {
        nlohmann::json json_cs_double = nlohmann::json::array();  // CsDouble을 위한 JSON 배열

        // CsDouble의 각 원소를 JSON 배열에 추가
        for (const auto& value : cs_double) {
            json_cs_double.push_back(value);
        }

        // 완성된 JSON 배열을 큰 JSON 배열에 추가
        json_array.push_back(json_cs_double);
    }

    return json_array;
}


nlohmann::json blending_traj_to_json(const BlendingTraj& traj) {
    nlohmann::json json_obj;

    // 기본 필드
    json_obj["waypoint_num"] = traj.waypoint_num;
    json_obj["is_radius_percent"] = traj.is_radius_percent;

    // 수정된 필드
    json_obj["xd"] = traj.xd;
    json_obj["xdd"] = traj.xdd;
    json_obj["waypoint_xd"] = traj.waypoint_xd;
    json_obj["radius"] = traj.radius;

    // 벡터 필드
    json_obj["xs"] = cs_double_vector_to_json(traj.xs);
    json_obj["xds"] = vector_to_json(traj.xds);
    json_obj["xdds"] = vector_to_json(traj.xdds);
    json_obj["radiuses"] = vector_to_json(traj.radiuses);
    json_obj["waypoint_xds"] = vector_to_json(traj.waypoint_xds);
    json_obj["qs_redundant"] = vector_to_json(traj.qs_redundant);

    return json_obj;
}

nlohmann::json params_to_json(const UnitTask& task) {
    nlohmann::json json_params;

    // task_mode에 따라 다른 파라미터를 처리
    switch (task.task_mode) {
        case TASK_CSMOVE:
            json_params["Pose"] = array_to_json(task.x_target);
            json_params["Relative"] = task.relative;
            break;

        case TASK_JSMOVE:
            json_params["Joint Position"] = array_to_json(task.q_target);
            json_params["Relative"] = task.relative;
            break;

        case TASK_CSMOVE_REDUNDANT:
            json_params["Pose"] = array_to_json(task.x_target);
            json_params["q_redundant"] = task.q_redundant;
            json_params["Relative"] = task.relative;
            break;

        case TASK_JSMOVE_ONLY_J6:
            json_params["Joint Position"] = array_to_json(task.q_target);
            json_params["Relative"] = task.relative;
            break;

        case TASK_TCPMOVE:
            json_params["Tcp Pose"] = array_to_json(task.x_target);
            break;

        case TASK_BLENDING:
            json_params["Traj blending"] = blending_traj_to_json(task.traj_blending);
            break;

        case TASK_DTC:
            json_params["Is dtc mode"] = task.is_dtc_mode;
            json_params["Target torque"] = array_to_json(task.q_target);
            break;

        case TASK_SET_TCP:
            json_params["Tcp"] = array_to_json(task.x_target);
            break;

        case TASK_CSMOVE2:
        case TASK_CSMOVE2_FOR_TEACHING:
            json_params["Pose"] = array_to_json(task.x_target);
            json_params["Acc"] = task.vel_cs_custom;
            json_params["Vel"] = task.acc_cs_custom;
            json_params["relative"] = task.relative;
            break;
        case TASK_JSMOVE2:
        case TASK_JSMOVE2_FOR_TEACHING:
            json_params["Joint position"] = array_to_json(task.q_target);
            json_params["Acc"] = task.acc_js_custom;
            json_params["Vel"] = task.vel_js_custom;
            json_params["Relative"] = task.relative;
            break;

        case TASK_JSON_POSE_TAG_CSMOVE:
            json_params["Pose"] = array_to_json(task.x_target);
            json_params["Demo tag"] = task.bp_param.demo_tag;
            json_params["Motion tag"] = task.bp_param.motion_tag;
            break;

        case TASK_RECOG_BLENDING:
            json_params["Traj blending"] = blending_traj_to_json(task.traj_blending);
            json_params["Demo tag"] = task.bp_param.demo_tag;
            json_params["Motion tag"] = task.bp_param.motion_tag;
            break;

        case TASK_SET_TEACHING_TARGET_POSE:
        case TASK_SET_TEACHING_TARGET_POSE_WITH_MARKER_DETECTION:
            json_params["Tag teaching pose"] = task.tag_teaching_pose;
            json_params["Pose idx teaching pose"] = task.pose_idx_teaching_pose;
            break;

        case TASK_SEND_TASK_LOG:
            json_params["Log"] = task.task_log;
            break;

        case TASK_RECOG_POSE_TAG_CSMOVE:
            json_params["Pose"] = array_to_json(task.x_target);
            json_params["Acc"] = task.acc_cs_custom;
            json_params["Vel"] = task.vel_cs_custom;
            json_params["Demo tag"] = task.bp_param.demo_tag;
            json_params["Motion tag"] = task.bp_param.motion_tag;
            break;
        case TASK_POSE_ESTIMATION_CAD_MATCHING:
        case TASK_POSE_ESTIMATION_AI:
            json_params["Do sampling"] = task.bp_param.detection.do_scan_sampling;
            break;

        case TASK_GRASPING_OPTIMAL_POSE_BIN_PICKING:
        case TASK_GRASPING_SUB_OPTIMAL_POSE_BIN_PICKING:
            json_params["Acc"] = task.acc_cs_custom;
            json_params["Vel"] = task.vel_cs_custom;
            break;

        case TASK_SET_TIP_CHANGING_FLAG:
            json_params["Task flag"] = task.bp_param.tip_changing_task_flag;
            break;

        case TASK_KORAS_GRIPPER_COMMAND:
        case TASK_KORAS_GRIPPER_COMMAND_FOR_TEACHING:
        case TASK_KORAS_GRIPPER_COMMAND_TOOL_DETACHING:
            json_params["Grp cmd"] = task.bp_param.gripper_command;
            json_params["Position"] = task.bp_param.gripper_param[0];
            json_params["Speed"] = task.bp_param.gripper_param[1];
            break;

        case TASK_RECOG_KORAS_GRIPPER_CHECK_GRASPING:
            json_params["Target position"] = task.bp_param.gripper_param[0];
            break;

        case TASK_PLC_MODBUS_COMMAND:
            json_params["Plc command"] = task.bp_param.plc_command;
            break;

        case TASK_GRIPPER_RELAY_CONTROL_COMMAND:
            json_params["Relay signal"] = task.bp_param.relay_signal;
            break;

        case TASK_SELECT_ROBOT_TCP:
            json_params["Int"] = task.bp_param.tcp_idx;
            break;

        case TASK_RECOG_TOOL_CHANGING_GOAL_CSMOVE:
            json_params["Is tool attaching"] = task.is_tool_attaching;
            json_params["Pose"] = array_to_json(task.x_target);
            break;

        case TASK_CSMOVE_TO_TAG:
            json_params["tag_name"] = task.tag_name;
            break;

        case TASK_MARKER_DETECTION_FOR_CONVERGING:
        case TASK_MARKER_DETECTION_FOR_APPROACHING:
            json_params["Tag name"] = task.tag_name;
            json_params["Distance mm"] = task.des_tag_distance_mm;
            break;

        case TASK_SET_CAM_AUTO_FOCUS:
            json_params["Is auto focus on"] = task.is_cam_auto_focus_on;
            break;

        default:
            // 기본 처리: 파라미터가 없는 경우나 특별한 처리 필요 없는 경우
            break;
    }

    return json_params;
}

// JSON 파일 저장하기
void MainWindow_initialSetup::on_pushButton_export_json_clicked() {
    QString filename = QFileDialog::getSaveFileName(this, "Save JSON File", "", "JSON Files (*.json)");
    if (!filename.isEmpty()) {
        nlohmann::json json_data;

        for (const auto& task : module_task_json) {
            nlohmann::json task_json;
            task_json["task_type"] = task_to_string(task.task_mode); // task_mode를 string으로 변환하는 함수 필요
            task_json["params"] = params_to_json(task); // 각 task의 파라미터를 JSON으로 변환하는 함수 필요
            json_data["unit_tasks"].push_back(task_json);
        }

        std::ofstream file(filename.toStdString());
        if (file.is_open()) {
            file << json_data.dump(4); // JSON을 파일로 저장
            file.close();
            QMessageBox::information(this, "Export JSON", "Tasks successfully saved.");
        } else {
            QMessageBox::warning(this, "Export JSON", "Failed to save file.");
        }
    }
}

void MainWindow_initialSetup::on_pushButton_import_json_clicked() {
    QString filename = QFileDialog::getOpenFileName(this, "Open JSON File", "", "JSON Files (*.json)");
    if (!filename.isEmpty()) {
        nlohmann::json json_data;
        std::ifstream file(filename.toStdString());
        if (file.is_open()) {
            file >> json_data;
            file.close();

            // Clear previous tasks
            module_tasks_json.clear();

            // Iterate over each task in the "tasks" array
            for (const auto& task_obj : json_data["tasks"]) {
                std::vector<UnitTask> unit_tasks;

                // Iterate over each unit_task in the "unit_tasks" array of the current task
                for (const auto& unit_task : task_obj["unit_tasks"]) {
                    std::string task_type_str = unit_task["task_type"];
                    const auto& params = unit_task["params"];

                    // Find the corresponding function in task_push_back_map
                    auto it = qnode_->task_planner_->task_push_back_map.find(task_type_str);
                    if (it != qnode_->task_planner_->task_push_back_map.end()) {
                        // Call the appropriate TaskPushBack function
                        it->second(unit_tasks, params);
                    } else {
                        std::cerr << "Unknown task_type: " << task_type_str << std::endl;
                    }
                }
                // Add the list of unit tasks for this task to module_tasks_json
                module_tasks_json.push_back(unit_tasks);
            }

            QMessageBox::information(this, "Import JSON", "All tasks successfully imported.");
        } else {
            QMessageBox::warning(this, "Import JSON", "Failed to open file.");
        }
    }
}

void MainWindow_initialSetup::on_pushButton_import_tw_clicked() {
    QString filename = QFileDialog::getOpenFileName(this, "Open TW File", "", "TW Files (*.tw)");

    // 수정, 주석 # 가능한지 확인

    if (!filename.isEmpty()) {
        std::string tw_data;
        std::ifstream file(filename.toStdString());
        if (file.is_open()) {
            file >> tw_data;
            file.close();

            std::string text = qnode_->base64_decode(tw_data);

            // tw_data를 JSON 형식으로 저장하기
            json jsonData;
            jsonData = json::parse(text);  // JSON 객체에 데이터를 추가

            json task1;
            task1["tasks"] = json::array({
                {
                    {"name", jsonData["taskFile"]["name"]},
                    {"unit_tasks", json::array()}
                }
            });

            for (json unit_task : jsonData["taskFile"]["file"]["children"][1]["_pojo"]["children"]) {

                string type_ = unit_task["_type"];
                if (type_ == "MoveJNode") {
                    json pose_ = unit_task["_pojo"]["pose"];
                    bool relative_ = unit_task["_pojo"]["position"] == "ABSOLUTE" ? false : true;
                    task1["tasks"][0]["unit_tasks"].push_back({
                        {"params", json::array({
                            json::array({pose_["pose1"], pose_["pose2"], pose_["pose3"], pose_["pose4"], pose_["pose5"], pose_["pose6"]}),
                            unit_task["_pojo"]["velocity"],
                            unit_task["_pojo"]["acceleration"],
                            relative_
                            })},
                        {"task_type", "TASK_JSMOVE2"}
                    });
                }
                else if (type_ == "MoveLNode") {
                    json pose_ = unit_task["_pojo"]["pose"];
                    bool relative_ = unit_task["_pojo"]["position"] == "ABSOLUTE" ? false : true;
                    task1["tasks"][0]["unit_tasks"].push_back({
                        {"params", json::array({
                            json::array({pose_["pose1"], pose_["pose2"], pose_["pose3"], pose_["pose4"], pose_["pose5"], pose_["pose6"]}),
                            unit_task["_pojo"]["rotationalVelocity"],
                            unit_task["_pojo"]["rotationalAcceleration"],
                            relative_
                            })},
                        {"task_type", "TASK_CSMOVE2"}
                    });
                } else if (type_ == "MoveSJNode") {

                } else if (type_ == "MoveSLNode") {

                } else if (type_ == "CustomCodeNode") {
                    string script;
                    if (unit_task["_pojo"]["type"] == "SINGLE") script = unit_task["_pojo"]["single"];
                    else {
                        script = unit_task["_pojo"]["script"];
                        size_t pos_start = script.find("flange_serial_");
                        if      (script[pos_start + 14] == 'o') script = "flange_serial_open()";
                        else if (script[pos_start + 14] == 'c') script = "flange_serial_close()";
                        else {                                         // flange_serial_write()
                            size_t pos_end = script.find(")");
                            script = script.substr(pos_start, pos_end - pos_start + 1);

                            // 괄호 안 큰따옴표 변경
                            // "flange_serial_write(b"\\x00")" --> "flange_serial_write(b'\\x00')"
                            script.replace(21, 1, "\'");
                            script.replace(pos_end - pos_start - 1, 1, "\'");

                            // 괄호 안 hex 표시 변경
                            // "flange_serial_write(b'\\x00')" --> "flange_serial_write(b'\x00')"
                            size_t pos = 21;
                            while ((pos = script.find("\\\\", pos)) != std::string::npos) {
                                script.replace(pos, 2, "\\");
                                pos += 1;
                            }
                        }
                    }

                    task1["tasks"][0]["unit_tasks"].push_back({
                        {"params", json::array({script})},
                        {"task_type", "TASK_DS_CUSTOM_CODE"}
                    });

                } else if (type_ == "WaitNode") {
                    task1["tasks"][0]["unit_tasks"].push_back({
                        {"params", json::array({
                            unit_task["_pojo"]["sec"].get<int>() * 100
                            })},
                        {"task_type", "TASK_DELAY"}
                    });
                }
            }

            // 파일 저장 대화 상자 표시
            QString saveFilename = QFileDialog::getSaveFileName(this, "Save as JSON", "", "JSON Files (*.json)");
            if (!saveFilename.isEmpty()) {
                std::ofstream outFile(saveFilename.toStdString());
                if (outFile.is_open()) {
                    outFile << task1.dump(4);  // JSON을 포맷팅하여 저장 (4는 들여쓰기 크기)
                    outFile.close();
                } else {
                    std::cerr << "Failed to open file for saving JSON." << std::endl;
                }
            }
        }
    }
}


// void MainWindow_initialSetup::on_pushButton_start_json_clicked() {
//     // Check if there are any tasks left to execute
//     current_task_index = 0;
//     if (current_task_index < module_tasks_json.size()) {
//         // Get the current module_task_json from module_tasks_json
//         module_task_json = module_tasks_json[current_task_index];

//         // Assign the current task list to qnode and execute it
//         qnode_->current_task_list_ = module_task_json;
//         std::cout << "size log:: " << module_task_json.size() << std::endl;

//         // Start the task
//         qnode_->beforeTaskStart();
//         qnode_->task_cycle_ = 0;

//         // Move to the next task
//         current_task_index++;
//     } else {
//         // If all tasks are executed, inform the user
//         QMessageBox::information(this, "Task Execution", "No Tasks Exist");
//     }
// }
// void MainWindow_initialSetup::on_pushButton_start_json_clicked() {
//     // Check if there are any tasks left to execute
//     current_task_index = 0;
//     if (current_task_index < module_tasks_json.size()) {
//         // Get the current module_task_json from module_tasks_json
//         module_task_json = module_tasks_json[current_task_index];

//         // Assign the current task list to qnode and execute it
//         qnode_->current_task_list_ = module_task_json;
//         std::cout << "size log:: " << module_task_json.size() << std::endl;

//         // Start the task
//         qnode_->beforeTaskStart();
//         qnode_->task_cycle_ = 0;

//         // Move to the next task
//         current_task_index++;
//     } else {
//         // If all tasks are executed, inform the user
//         QMessageBox::information(this, "Task Execution", "No Tasks Exist");
//     }
// }

void MainWindow_initialSetup::on_pushButton_doosan_task_start_clicked() {
    if (!cookingPlanner_) {
        setupPlanner(); // 혹시 초기화되지 않았다면 다시 시도
    }

    if (cookingPlanner_->doosan_module_task_.size() > 0) {
        // Get the current module_task_json from module_tasks_json
        qnode_->current_task_list_ = cookingPlanner_->doosan_module_task_;
        qnode_->beforeTaskStart();
        qnode_->task_cycle_ = 0;
    } else {
        // If all tasks are executed, inform the user
        QMessageBox::information(this, "Task Execution", "No Tasks Exist");
    }
}

MainWindow_initialSetup::MainWindow_initialSetup(QNode* qnode, QWidget* parent)
    : qnode_(qnode)
    , QMainWindow(parent)
    , ui(new Ui::MainWindow_initialSetup)
    , current_tool_("Fryer")
    , cookingPlanner_(nullptr)

{
    ROS_LOG_WARN("[%s] [line: %d]", __func__, __LINE__);

    ui->setupUi(this);
    ///////////////////////////////////////////
    setupPlanner(); // TaskPlanner(Cooking) 초기화
    ///////////////////////////////////////////

if(SW_MODE_COOKING) {

    /////////////////////////////////////Cooking robot///////////////////////////////
    connect(ui->pushButton_taskplanner_test, &QPushButton::clicked, this, &MainWindow_initialSetup::onTestButtonClicked);

    connect(qnode_, &QNode::newImageReceived, this, &MainWindow_initialSetup::updateRealsenseImage);
    connect(qnode_, &QNode::newPoseReceived, this, &MainWindow_initialSetup::updateRobotPose);
    connect(qnode_, &QNode::newLangSAMCoordinatesReceived, this, &MainWindow_initialSetup::updateLangSAMCoordinates);
    connect(qnode_, &QNode::newLangSAMImageReceived, this, &MainWindow_initialSetup::updateLangSAMImage);

    qDebug() << "Connected QNode signals to MainWindow slots.";


    connect(ui->pushButton_Fryer, &QPushButton::clicked, this, &MainWindow_initialSetup::onFryerButtonClicked);
    connect(ui->pushButton_Pot, &QPushButton::clicked, this, &MainWindow_initialSetup::onPotButtonClicked);
    connect(ui->pushButton_Bowl, &QPushButton::clicked, this, &MainWindow_initialSetup::onBowlButtonClicked);
    connect(ui->pushButton_MoveToFryer, &QPushButton::clicked, this, &MainWindow_initialSetup::onMoveToFryerClicked);
    connect(ui->pushButton_MoveToPot, &QPushButton::clicked, this, &MainWindow_initialSetup::onMoveToPotClicked);
    connect(ui->pushButton_MoveToBowl, &QPushButton::clicked, this, &MainWindow_initialSetup::onMoveToBowlClicked);
    connect(ui->pushButton_Enter, &QPushButton::clicked, this, &MainWindow_initialSetup::onEnterButtonClicked);
    connect(ui->pushButton_MoveToCenter, &QPushButton::clicked, this, &MainWindow_initialSetup::onMoveToCenterClicked);
    // home_pose_ = {-39.41, 132.64, -96.56, 2.81, -33.83, -143.11};
    // mid_tag_pose_ = {-154.564, 120.774, -116.569, -23.045, -0.511, -70.208};
    // mid_2f_pose_ = {-88.26, 111.15, -106.03, -21.29, -3.11, -71.29};
    // mid_pose_ = {-88.26, 111.15, -106.03, -21.29, -3.11, -71.29};
    connect(ui->pushButton_Home, &QPushButton::clicked, this,
            std::bind(&MainWindow_initialSetup::SetPoseCallback, this, "home"));
    connect(ui->pushButton_Middle, &QPushButton::clicked, this,
            std::bind(&MainWindow_initialSetup::SetPoseCallback, this, "middle"));
    connect(ui->pushButton_Middle_2F, &QPushButton::clicked, this,
            std::bind(&MainWindow_initialSetup::SetPoseCallback, this, "middle2F"));
    connect(ui->pushButton_Middle_Tag, &QPushButton::clicked, this,
            std::bind(&MainWindow_initialSetup::SetPoseCallback, this, "middleTag"));
}


    connect(ui->pushButton_enable_disable  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setEnableBtnCallback);

#if DRFL_CONTROL
    connect(ui->pushButton_set_tcp         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::pushButtonSetTcpClickedCallback);
#else
    connect(ui->pushButton_set_tcp         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setTcpBtnCallback);
#endif

    connect(ui->pushButton_cs_base_step_unit_1         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveTranslationScale_1);
    connect(ui->pushButton_cs_base_step_unit_2         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveTranslationScale_2);
    connect(ui->pushButton_cs_base_step_unit_3         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveTranslationScale_3);
    connect(ui->pushButton_cs_base_step_unit_4         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveTranslationScale_4);
    connect(ui->pushButton_cs_base_step_unit_5         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveTranslationScale_5);
    connect(ui->pushButton_cs_base_step_unit_6         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveTranslationScale_6);
    connect(ui->pushButton_cs_base_step_unit_7         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveTranslationScale_7);
    connect(ui->pushButton_cs_base_step_unit_8         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveTranslationScale_8);

    connect(ui->pushButton_js_base_step_unit_1         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveRotationScale_1);
    connect(ui->pushButton_js_base_step_unit_2         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveRotationScale_2);
    connect(ui->pushButton_js_base_step_unit_3         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveRotationScale_3);
    connect(ui->pushButton_js_base_step_unit_4         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveRotationScale_4);
    connect(ui->pushButton_js_base_step_unit_5         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveRotationScale_5);
    connect(ui->pushButton_js_base_step_unit_6         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveRotationScale_6);
    connect(ui->pushButton_js_base_step_unit_7         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveRotationScale_7);
    connect(ui->pushButton_js_base_step_unit_8         , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setStepMoveRotationScale_8);

    // Start Task for teaching button
    connect(ui->pushButton_teaching_task_start  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::taskStartForTeachingBtnCallback);


    connect(ui->pushButton_task_start  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::taskStartNoTeachingBtnCallback);

    connect(ui->pushButton_teaching_task_start_repeat  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::taskStartRepeatedlyForWithoutTeachingBtnCallback);



    connect(ui->pushButton_total_task_start  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::taskStartTotalTaskBtnCallback);


    // Set teaching pose
    connect(ui->pushButton_set_teaching_pose  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setTeachingPoseBtnCallback);

    //// Marker
    //// Marker
    //// Marker
    // Set teaching pose with marker detection
    connect(ui->pushButton_set_teaching_pose_with_marker  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::setTeachingPoseWithMarkerDetectionBtnCallback);
    connect(ui->pushButton_teaching_task_start_with_marker  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::taskStartForTeachingWithMarkerBtnCallback);
    connect(ui->pushButton_task_start_with_marker  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::taskStartNoTeachingWithMarkerBtnCallback);
    //// Marker
    //// Marker
    //// Marker



    connect(ui->pushButton_generate_tasks  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::genGraphyBtnCallback);


    connect(ui->pushButton_3D_scanning  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::do3DScanningBtnCallback);

    connect(ui->pushButton_KUAISTaskStart  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::doKUAISTaskBtnCallback);




    // Test Function
    connect(ui->pushButton_testFunction  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::graphyTestFunctionBtnCallback);


    // Test Function
    connect(ui->pushButton_robot_power_on_off  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::graphyRobotPowerOnOffBtnCallback);



    QObject::connect(ui->pushButton_MODBUS_WRITE_DATA_BOOL_TYPE, SIGNAL(clicked()), this, SLOT(pushButtonModbusWriteDataBoolType()));
    QObject::connect(ui->pushButton_MODBUS_WRITE_DATA_ROTATION_ANGLE, SIGNAL(clicked()), this, SLOT(pushButtonModbusWriteDataInt16Type()));
    QObject::connect(ui->pushButton_MODBUS_WRITE_DATA_ASCII_BARCODE, SIGNAL(clicked()), this, SLOT(pushButtonModbusWriteDataASCIIType()));


    QObject::connect(ui->pushButton_MODBUS_READ_DATA_BOOL_TYPE, SIGNAL(clicked()), this, SLOT(pushButtonModbusReadDataBoolType()));
    QObject::connect(ui->pushButton_MODBUS_MonitoringPLC, SIGNAL(clicked()), this, SLOT(pushButtonModbusMonitoringPLC()));


    QObject::connect(ui->pushButton_MODBUS_TEST, SIGNAL(clicked()), this, SLOT(pushButtonModbusTest()));


    //plc
    connect(ui->pushButton_set_step_motor_position, &QPushButton::clicked, this,
            &MainWindow_initialSetup::SetStepMotorPosition);
    connect(ui->pushButton_set_step_motor_speed, &QPushButton::clicked, this,
            &MainWindow_initialSetup::SetStepMotorVelocity);




    // Stop button
    connect(ui->pushButton_stop_all  , &QPushButton::clicked, this,
            &MainWindow_initialSetup::stopAllBtnCallback);
    connect(ui->pushButton_task_pause, &QPushButton::clicked, this,
            &MainWindow_initialSetup::pauseTaskBtnCallback);
    connect(ui->pushButton_task_resume, &QPushButton::clicked, this,
            &MainWindow_initialSetup::resumeTaskBtnCallback);


    // Koras Gripper
    connect(ui->pushButton_KorasGripperConnection, &QPushButton::clicked, this, &MainWindow_initialSetup::pushButtonKorasGripperConnectionCallback);
    connect(ui->pushButton_KorasGripperInitialize, &QPushButton::clicked, this, &MainWindow_initialSetup::pushButtonKorasGripperIntializeCallback);
    // connect(ui->pushButton_KorasGripperSlaveChange, &QPushButton::clicked, this, &MainWindow_initialSetup::pushButtonKorasGripperSlaveChangeCallback);
    // connect(ui->pushButton_KorasGripperSetInitMinMaxValue, &QPushButton::clicked, this, &MainWindow_initialSetup::pushButtonKorasGripperSetInitMinMaxValue);
    connect(ui->pushButton_KorasGripperOpen, &QPushButton::clicked, this, &MainWindow_initialSetup::pushButtonKorasGripperOpenCallback);
    connect(ui->pushButton_KorasGripperClose, &QPushButton::clicked, this, &MainWindow_initialSetup::pushButtonKorasGripperCloseCallback);
    connect(ui->pushButton_KorasGripperPosCtrl, &QPushButton::clicked, this, &MainWindow_initialSetup::pushButtonKorasGripperPosCtrlCallback);
    connect(ui->pushButton_KorasGripperPosCtrlOnOff, &QPushButton::clicked, this, &MainWindow_initialSetup::pushButtonKorasGripperPosCtrlOnOffCallback);
    // connect(ui->pushButton_KorasGripperVacuumOnOff, &QPushButton::clicked, this, &MainWindow_initialSetup::pushButtonKorasGripperVacuumOnOffCallback);



    connect(ui->pushButton_exit_developerWindow, &QPushButton::clicked, this, [this] {
        Q_EMIT closeDeveloperWindow();
    });

    connect(ui->pushButton_openTaskGenerator, &QPushButton::clicked, this, [this] {
        Q_EMIT openTaskGeneratorWindow();
    });

    // JS & CS space control
    QObject::connect(ui->pushButton_MoveRobotACS        , SIGNAL(clicked()), this, SLOT(pushButtonCSMoveClickedCallback()));
    QObject::connect(ui->pushButton_jsMove_robotA      , SIGNAL(clicked()), this, SLOT(pushButtonJSMoveClickedCallback()));
    QObject::connect(ui->pushButton_MoveRobotRelativeToolFrame        , SIGNAL(clicked()), this, SLOT(pushButtonCSMoveRelativeToolFrameClickedCallback()));
    QObject::connect(ui->pushButton_setCurrentPoseRobotA, SIGNAL(clicked()), this, SLOT(pushButtonSetCurrentRobotAPoseClickedCallback()));
    QObject::connect(ui->pushButton_setCurQ_robotA     , SIGNAL(clicked()), this, SLOT(pushButtonSetCurQRobotAClickedCallback()));
    QObject::connect(ui->pushButton_getCurQ_robotA     , SIGNAL(clicked()), this, SLOT(pushButtonGetCurQRobotAClickedCallback()));
    QObject::connect(ui->pushButton_getCurX_robotA     , SIGNAL(clicked()), this, SLOT(pushButtonGetCurXRobotAClickedCallback()));
    QObject::connect(ui->checkBox_isrelativeRobotACS     , SIGNAL(clicked()), this, SLOT(pushButtonAbsRelCheckClickedCallback()));

    QObject::connect(ui->pushButton_setUIJSPosition_0     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition0ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_1     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition1ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_2     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition2ClickedCallback()));
    QObject::connect(ui->pushButton_setUIJSPosition_3     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPosition3ClickedCallback()));

    QObject::connect(ui->pushButton_setUIJSPosition_Packing     , SIGNAL(clicked()), this, SLOT(pushButtonSetUIJSPositionPackingClickedCallback()));

    QObject::connect(ui->pushButton_import_json     , SIGNAL(clicked()), this, SLOT(on_pushButton_import_json_clicked()));
    QObject::connect(ui->pushButton_import_tw       , SIGNAL(clicked()), this, SLOT(on_pushButton_import_tw_clicked()));
    // QObject::connect(ui->pushButton_run_json        , SIGNAL(clicked()), this, SLOT(on_pushButton_start_json_clicked()));

    QObject::connect(ui->pushButton_doosan_task_start, SIGNAL(clicked()), this, SLOT(on_pushButton_doosan_task_start_clicked()));

    /////////////////////////////////////// Jog Button ///////////////////////////////////////

    list_jog_q_plus_ << ui->pushButton_jog_j1_plus << ui->pushButton_jog_j2_plus
                     << ui->pushButton_jog_j3_plus << ui->pushButton_jog_j4_plus
                     << ui->pushButton_jog_j5_plus << ui->pushButton_jog_j6_plus;

    list_jog_q_minus_ << ui->pushButton_jog_j1_minus << ui->pushButton_jog_j2_minus
                      << ui->pushButton_jog_j3_minus << ui->pushButton_jog_j4_minus
                      << ui->pushButton_jog_j5_minus << ui->pushButton_jog_j6_minus;

    list_jog_x_plus_ << ui->pushButton_jog_x1_plus << ui->pushButton_jog_x2_plus
                     << ui->pushButton_jog_x3_plus << ui->pushButton_jog_x4_plus
                     << ui->pushButton_jog_x5_plus << ui->pushButton_jog_x6_plus;

    list_jog_x_minus_ << ui->pushButton_jog_x1_minus << ui->pushButton_jog_x2_minus
                      << ui->pushButton_jog_x3_minus << ui->pushButton_jog_x4_minus
                      << ui->pushButton_jog_x5_minus << ui->pushButton_jog_x6_minus;




    list_step_move_q_plus_ << ui->pushButton_jog_j1_plus_step << ui->pushButton_jog_j2_plus_step
                     << ui->pushButton_jog_j3_plus_step << ui->pushButton_jog_j4_plus_step
                     << ui->pushButton_jog_j5_plus_step << ui->pushButton_jog_j6_plus_step;

    list_step_move_q_minus_ << ui->pushButton_jog_j1_minus_step << ui->pushButton_jog_j2_minus_step
                     << ui->pushButton_jog_j3_minus_step << ui->pushButton_jog_j4_minus_step
                     << ui->pushButton_jog_j5_minus_step << ui->pushButton_jog_j6_minus_step;

    list_step_move_x_plus_ << ui->pushButton_jog_x1_plus_step << ui->pushButton_jog_x2_plus_step
                     << ui->pushButton_jog_x3_plus_step << ui->pushButton_jog_x4_plus_step
                     << ui->pushButton_jog_x5_plus_step << ui->pushButton_jog_x6_plus_step;

    list_step_move_x_minus_ << ui->pushButton_jog_x1_minus_step << ui->pushButton_jog_x2_minus_step
                     << ui->pushButton_jog_x3_minus_step << ui->pushButton_jog_x4_minus_step
                     << ui->pushButton_jog_x5_minus_step << ui->pushButton_jog_x6_minus_step;


    list_step_move_x_plus_tool_ << ui->pushButton_jog_x1_plus_step_tool << ui->pushButton_jog_x2_plus_step_tool
                     << ui->pushButton_jog_x3_plus_step_tool << ui->pushButton_jog_x4_plus_step_tool
                     << ui->pushButton_jog_x5_plus_step_tool << ui->pushButton_jog_x6_plus_step_tool;

    list_step_move_x_minus_tool_ << ui->pushButton_jog_x1_minus_step_tool << ui->pushButton_jog_x2_minus_step_tool
                     << ui->pushButton_jog_x3_minus_step_tool << ui->pushButton_jog_x4_minus_step_tool
                     << ui->pushButton_jog_x5_minus_step_tool << ui->pushButton_jog_x6_minus_step_tool;


    QObject::connect(ui->pushButton_robotAJogSelect, SIGNAL(clicked()), this, SLOT(pushButtonJogSelectCallback()));
    QObject::connect(ui->pushButton_jogModeEnd, SIGNAL(clicked()), this, SLOT(pushButtonJogModeEndCallback()));

    for (int i = 0; i < 6; i++) {
        connect(list_jog_q_plus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, true, true);
        });
        connect(list_jog_q_minus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, false, true);
        });
        connect(list_jog_q_plus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });
        connect(list_jog_q_minus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });

    }

    for (int i = 0; i < 6; i++) {
        connect(list_jog_x_plus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, true, false);
        });
        connect(list_jog_x_minus_[i], &QPushButton::pressed, this, [=] () {
            jogBtnPressed(i, false, false);
        });
        connect(list_jog_x_plus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });
        connect(list_jog_x_minus_[i], &QPushButton::released, this, [=] () {
            jogBtnReleased();
        });
    }

    // Step move button
    for (int i = 0; i < 6; i++) {
        connect(list_step_move_q_plus_[i], &QPushButton::clicked, this, [=] () {
            stepMoveQBtnCallback(i, true);
        });
        connect(list_step_move_q_minus_[i], &QPushButton::clicked, this, [=] () {
            stepMoveQBtnCallback(i, false);
        });
        connect(list_step_move_x_plus_[i], &QPushButton::clicked, this, [=] () {
            stepMoveXBtnCallback(i, true);
        });
        connect(list_step_move_x_minus_[i], &QPushButton::clicked, this, [=] () {
            stepMoveXBtnCallback(i, false);
        });
        connect(list_step_move_x_plus_tool_[i], &QPushButton::clicked, this, [=] () {
            stepMoveXToolFrameBtnCallback(i, true);
        });
        connect(list_step_move_x_minus_tool_[i], &QPushButton::clicked, this, [=] () {
            stepMoveXToolFrameBtnCallback(i, false);
        });
    }

    robotAXList_ << ui->lineEdit_robotAActualX_1
                 << ui->lineEdit_robotAActualX_2
                 << ui->lineEdit_robotAActualX_3
                 << ui->lineEdit_robotAActualX_4
                 << ui->lineEdit_robotAActualX_5
                 << ui->lineEdit_robotAActualX_6;

    robotAQList_ << ui->lineEdit_robotAActualQ_1
                 << ui->lineEdit_robotAActualQ_2
                 << ui->lineEdit_robotAActualQ_3
                 << ui->lineEdit_robotAActualQ_4
                 << ui->lineEdit_robotAActualQ_5
                 << ui->lineEdit_robotAActualQ_6;


    robotATargetXList_ << ui->lineEdit_robotATargetX_1
                       << ui->lineEdit_robotATargetX_2
                       << ui->lineEdit_robotATargetX_3
                       << ui->lineEdit_robotATargetX_4
                       << ui->lineEdit_robotATargetX_5
                       << ui->lineEdit_robotATargetX_6;

    robotATargetQList_ << ui->lineEdit_robotATargetQ_1
                       << ui->lineEdit_robotATargetQ_2
                       << ui->lineEdit_robotATargetQ_3
                       << ui->lineEdit_robotATargetQ_4
                       << ui->lineEdit_robotATargetQ_5
                       << ui->lineEdit_robotATargetQ_6;

    list_set_tcp_x_mm_ << ui->lineEdit_set_tcp_x_mm_1
                       << ui->lineEdit_set_tcp_x_mm_2
                       << ui->lineEdit_set_tcp_x_mm_3
                       << ui->lineEdit_set_tcp_x_mm_4
                       << ui->lineEdit_set_tcp_x_mm_5
                       << ui->lineEdit_set_tcp_x_mm_6;

    list_add_tcp_x_mm_ << ui->lineEdit_add_tcp_x_mm_1
                       << ui->lineEdit_add_tcp_x_mm_2
                       << ui->lineEdit_add_tcp_x_mm_3
                       << ui->lineEdit_add_tcp_x_mm_4
                       << ui->lineEdit_add_tcp_x_mm_5
                       << ui->lineEdit_add_tcp_x_mm_6;


    QObject::connect(ui->comboBox_tcp_name, SIGNAL(currentIndexChanged(int)), this, SLOT(comboBoxIndexChangedCallback(int)));
    QObject::connect(ui->pushButton_set_tcp, SIGNAL(clicked()), this, SLOT(pushButtonSetTcpClickedCallback()));
    QObject::connect(ui->pushButton_add_tcp, SIGNAL(clicked()), this, SLOT(pushButtonAddTcpClickedCallback()));


    QObject::connect(ui->radioButton_select_plc_address_write_1     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_2     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_3     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_4     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_5     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_write_6     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcWriteAddressClickedCallback()));

    QObject::connect(ui->radioButton_select_plc_address_read_1     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcReadAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_read_2     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcReadAddressClickedCallback()));
    QObject::connect(ui->radioButton_select_plc_address_read_3     , SIGNAL(clicked()), this, SLOT(pushButtonSelectPlcReadAddressClickedCallback()));


#if IS_PLC_COMMUNICATION
    // initialize Modbus TCP/IP
    pushButtonModbusSetIPAddress();
    qnode_->setMODBUSComm(true); // M00000 트리거
#endif

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &MainWindow_initialSetup::timerCallback);
    timer_->start(100);
}

MainWindow_initialSetup::~MainWindow_initialSetup()
{
    delete ui;
}

//////////////////////////////Cooking robot///////////////////////////////
void MainWindow_initialSetup::comboBoxIndexChangedCallback(int index) {
#if DRFL_CONTROL
    string tcp_name = ui->comboBox_tcp_name->currentText().toStdString();
    if (tcp_name == "default") {
        for (int i=0; i<6; i++) {
            list_set_tcp_x_mm_[i]->setText(QString::number(0, 'f', 1));
        }
    } else {
        for (int i=0; i<6; i++) {
            list_set_tcp_x_mm_[i]->setText(QString::number(qnode_->list_tcp[tcp_name][i], 'f', 1));
        }
    }
#endif
}

void MainWindow_initialSetup::pushButtonSetTcpClickedCallback() {

    qnode_->drflSetRobotMode(0);

    string tcp_name = ui->comboBox_tcp_name->currentText().toStdString();
    if (tcp_name == "default") {tcp_name = "";}
    qnode_->drflSetTcp(tcp_name);

    qnode_->drflSetRobotMode(1);

}

void MainWindow_initialSetup::pushButtonAddTcpClickedCallback() {
#if DRFL_CONTROL
    qnode_->drflSetRobotMode(0);
    QString name_ = ui->lineEdit_tcp_name->text();
    if (ui->comboBox_tcp_name->findText(name_) > -1) {
        QMessageBox::warning(this, "경고", "동일한 이름의 TCP가 있습니다!");
        return;
    }
    std::string tcp_name = name_.toStdString();
    if (tcp_name == "") {
        QMessageBox::warning(this, "경고", "TCP의 이름을 설정해주세요!");
        return;
    }

    CsDouble tcp;
    for (int i = 0; i < CS_DOF; i++) {
        tcp[i] = list_add_tcp_x_mm_[i]->text().toDouble();
    }

    qnode_->drflCreateTcp(tcp_name, tcp);
    qnode_->drflSetRobotMode(1);

    qnode_->list_tcp[tcp_name] = tcp;
    ui->comboBox_tcp_name->addItem(name_);
#endif
}


void MainWindow_initialSetup::SetPoseCallback(const std::string& position) {
    if(SW_MODE_COOKING) {
        double qd = 15;
        double qdd = 30;
        if      (position == "home"){qnode_->moveQ(cookingPlanner_->js_home_pose_, qd, qdd, false);}
        else if (position == "middle") {qnode_->moveQ(cookingPlanner_->js_mid_pose_, qd, qdd, false);}
        else if (position == "middle2F") {qnode_->moveQ(cookingPlanner_->js_mid_2f_pose_, qd, qdd, false);}
        else if (position == "middleTag"){qnode_->moveQ(cookingPlanner_->js_mid_tag_pose_, qd, qdd, false);}
    }
}

void MainWindow_initialSetup::onFryerButtonClicked() {
    if(SW_MODE_COOKING) {
        qDebug() << "Fryer button clicked!";
        current_tool_ = "Fryer";
        qnode_->setCurrentTool("Fryer");
        qnode_->sendRealsenseRequest(current_tool_);
        qDebug() << "Realsense request sent for tool:" << current_tool_;
    }
}

void MainWindow_initialSetup::onPotButtonClicked() {
    if(SW_MODE_COOKING) {
        qDebug() << "Pot button clicked!";
        current_tool_ = "Pot";
        qnode_->setCurrentTool("Pot");
        qnode_->sendRealsenseRequest(current_tool_);
        qDebug() << "Realsense request sent for tool:" << current_tool_;
    }
}

void MainWindow_initialSetup::onBowlButtonClicked() {
    if(SW_MODE_COOKING) {
        qDebug() << "Bowl button clicked!";
        current_tool_ = "Bowl";
        qnode_->setCurrentTool("Bowl");
        qnode_->sendRealsenseRequest(current_tool_);
        qDebug() << "Realsense request sent for tool:" << current_tool_;
    }
}


void MainWindow_initialSetup::updateRealsenseImage(const QImage &image) {
    if(SW_MODE_COOKING) {
        qDebug() << "Updating QLabel with new image for tool:" << current_tool_;
        if (image.isNull()) {
            qDebug() << "Received null image.";
            return;
        }

        if (current_tool_ == "Fryer") {
            ui->label_fryer->setPixmap(QPixmap::fromImage(image).scaled(ui->label_fryer->size(), Qt::KeepAspectRatio));
        } else if (current_tool_ == "Pot") {
            ui->label_pot->setPixmap(QPixmap::fromImage(image).scaled(ui->label_pot->size(), Qt::KeepAspectRatio));
        } else if (current_tool_ == "Bowl") {
            ui->label_else->setPixmap(QPixmap::fromImage(image).scaled(ui->label_else->size(), Qt::KeepAspectRatio));
        } else {
            qDebug() << "Unknown tool. No QLabel updated.";
        }
    }
}

void MainWindow_initialSetup::updateRobotPose(const QString &toolName, double x, double y, double z, double roll, double pitch, double yaw) {
    if(SW_MODE_COOKING) {
        qDebug() << "Updating robot pose for tool:" << toolName;
        qDebug() << "x=" << x << ", y=" << y << ", z=" << z << ", roll=" << roll << ", pitch=" << pitch << ", yaw=" << yaw;

        if (toolName == "Fryer") {
            ui->lineEdit_A_x->setText(QString::number(x));
            ui->lineEdit_A_y->setText(QString::number(y));
            ui->lineEdit_A_z->setText(QString::number(z));
            ui->lineEdit_A_r->setText(QString::number(roll));
            ui->lineEdit_A_p->setText(QString::number(pitch));
            ui->lineEdit_A_yaw->setText(QString::number(yaw));
        } else if (toolName == "Pot") {
            ui->lineEdit_B_x->setText(QString::number(x));
            ui->lineEdit_B_y->setText(QString::number(y));
            ui->lineEdit_B_z->setText(QString::number(z));
            ui->lineEdit_B_r->setText(QString::number(roll));
            ui->lineEdit_B_p->setText(QString::number(pitch));
            ui->lineEdit_B_yaw->setText(QString::number(yaw));
        } else if (toolName == "Bowl") {
            ui->lineEdit_C_x->setText(QString::number(x));
            ui->lineEdit_C_y->setText(QString::number(y));
            ui->lineEdit_C_z->setText(QString::number(z));
            ui->lineEdit_C_r->setText(QString::number(roll));
            ui->lineEdit_C_p->setText(QString::number(pitch));
            ui->lineEdit_C_yaw->setText(QString::number(yaw));
        } else {
            qDebug() << "Unknown tool name:" << toolName;
        }
        // cookingPlanner_->CookToolPointTask(const QString &tool_name, double x, double y, double z, double roll, double pitch, double yaw);
        // qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->task_toolpoints.begin(), qnode_->task_planner_->task_toolpoints.end());

    }
}

void MainWindow_initialSetup::onMoveToFryerClicked() {
    if(SW_MODE_COOKING) {
        qDebug() << "Move to Fryer button clicked!";

        // Fryer 위치 정보 가져오기
        double x = ui->lineEdit_A_x->text().toDouble();
        double y = ui->lineEdit_A_y->text().toDouble();
        double z = ui->lineEdit_A_z->text().toDouble();
        double roll = ui->lineEdit_A_r->text().toDouble();
        double pitch = ui->lineEdit_A_p->text().toDouble();
        double yaw = ui->lineEdit_A_yaw->text().toDouble();
        if (qnode_) {
            // taskplanner_를 통해 작업 생성
            auto task_toolpoints = cookingPlanner_->CookToolPointTask("Fryer", x, y, z, roll, pitch, yaw);
            if (task_toolpoints.empty()) {
                qDebug() << "Error: Task list is empty for Fryer!";
                return;
            }
            // 작업 리스트 설정 및 실행
            qnode_->current_task_list_ = task_toolpoints;
            qnode_->beforeTaskStart();
            qDebug() << "Move to Fryer task initialized and started.";
        } else {
            qDebug() << "Error: QNode is null!";
        }
    }
}

void MainWindow_initialSetup::onMoveToPotClicked() {
    if(SW_MODE_COOKING) {
        qDebug() << "Move to Pot button clicked!";

        // Pot 위치 정보 가져오기
        double x = ui->lineEdit_B_x->text().toDouble();
        double y = ui->lineEdit_B_y->text().toDouble();
        double z = ui->lineEdit_B_z->text().toDouble();
        double roll = ui->lineEdit_B_r->text().toDouble();
        double pitch = ui->lineEdit_B_p->text().toDouble();
        double yaw = ui->lineEdit_B_yaw->text().toDouble();
        if (qnode_) {
            auto task_toolpoints = cookingPlanner_->CookToolPointTask("Pot", x, y, z, roll, pitch, yaw);
            if (task_toolpoints.empty()) {
                qDebug() << "Error: Task list is empty for Pot!";
                return;
            }
            qnode_->current_task_list_ = task_toolpoints;
            qnode_->beforeTaskStart();
            qDebug() << "Move to Pot task initialized and started.";
        } else {
            qDebug() << "Error: QNode is null!";
        }
    }
}

void MainWindow_initialSetup::onMoveToBowlClicked() {
    if(SW_MODE_COOKING) {
        qDebug() << "Move to Bowl button clicked!";

        // Bowl 위치 정보 가져오기
        double x = ui->lineEdit_C_x->text().toDouble();
        double y = ui->lineEdit_C_y->text().toDouble();
        double z = ui->lineEdit_C_z->text().toDouble();
        double roll = ui->lineEdit_C_r->text().toDouble();
        double pitch = ui->lineEdit_C_p->text().toDouble();
        double yaw = ui->lineEdit_C_yaw->text().toDouble();
        if (qnode_) {
            auto task_toolpoints = cookingPlanner_->CookToolPointTask("Bowl", x, y, z, roll, pitch, yaw);
            if (task_toolpoints.empty()) {
                qDebug() << "Error: Task list is empty for Bowl!";
                return;
            }
            qnode_->current_task_list_ = task_toolpoints;
            qnode_->beforeTaskStart();
            qDebug() << "Move to Bowl task initialized and started.";
        } else {
            qDebug() << "Error: QNode is null!";
        }

    }
}

void MainWindow_initialSetup::onEnterButtonClicked() {
    if(SW_MODE_COOKING) {
        current_food_prompt_ = ui->lineEdit_textprompt->text();
        if (current_food_prompt_.isEmpty()) {
            qDebug() << "Error: No food prompt entered!";
            return;
        }
        // 플래그 설정
        qnode_->is_data_request_active_ = true;
        qnode_->is_image_request_active_ = true;

        // QNode를 통해 LangSAM 노드로 프롬프트 전송
        qDebug() << "Sending food prompt to LangSAM: " << current_food_prompt_;
        qnode_->sendLangSAMTextPrompt(current_food_prompt_.toStdString());
    }
}

void MainWindow_initialSetup::updateLangSAMCoordinates(double x, double y, double z) {
    if(SW_MODE_COOKING) {
        ui->lineEdit_X_langsam->setText(QString::number(x));
        ui->lineEdit_Y_langsam->setText(QString::number(y));
        ui->lineEdit_Z_langsam->setText(QString::number(z));
        qDebug() << "Updated LangSAM Coordinates: X=" << x << ", Y=" << y << ", Z=" << z;
    }
}

void MainWindow_initialSetup::updateLangSAMImage(const QImage &image) {
    if(SW_MODE_COOKING) {
        if (!image.isNull()) {
            ui->label_food->setPixmap(QPixmap::fromImage(image).scaled(ui->label_food->size(), Qt::KeepAspectRatio));
            qDebug() << "Updated LangSAM Image.";
        } else {
            qDebug() << "Error: Received null image for LangSAM.";
        }
    }
}

void MainWindow_initialSetup::onMoveToCenterClicked() {
    if(SW_MODE_COOKING) {
        qDebug() << "Move to Center button clicked!";
        // LangSAM 좌표 가져오기
        double x = ui->lineEdit_X_langsam->text().toDouble();
        double y = ui->lineEdit_Y_langsam->text().toDouble();
        double z = ui->lineEdit_Z_langsam->text().toDouble();
        // 기본 Roll, Pitch, Yaw 값 (필요 시 UI에서 입력받아 수정 가능)
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        qDebug() << "LangSAM move coordinates: X=" << x << ", Y=" << y << ", Z=" << z;
    }
}

void MainWindow_initialSetup::closeEvent(QCloseEvent *bar)
{
    //// NOTICE: 다른 widget의 closeEvent와 동시에 수행되는 버그 수정하기
    bar->ignore();
    return;
    // if(qnode_->is_task_button_clicked_) {
    //     bar->ignore();
    //     return;
    // } else {
    //     ROS_LOG_WARN("closeEventInitialSetup");
    //     qnode_->do_open_developer_window_ = false;
    //     bar->accept();
    // }
}

/** @brief Korean: Qt GUI timer callback 함수
 */
void MainWindow_initialSetup::timerCallback() {
    for (std::size_t i = 0; i < 6; i++) {
        robotAXList_[i]->setText(QString::number(qnode_->params_.meas.x[i], 'f', 4));
        robotAQList_[i]->setText(QString::number(qnode_->params_.meas.q[i], 'f', 3));
    }

    if (qnode_->params_.status.is_enable) {
        ui->lineEdit_status_enable->setText("Enabled");
    } else {
        ui->lineEdit_status_enable->setText("Disabled");
    }

    if (qnode_ && qnode_->task_param_.is_robot_move_fin) {
        // qDebug() << "[DEBUG] Robot Move Finished! Updating UI.";
        qnode_->task_param_.is_robot_move_fin = false; // 충돌 방지
    }

    // if(-179.0 < qnode_->params_.meas.q[0] && qnode_->params_.meas.q[0] < 5.0) {
    //     ui->pushButton_setUIJSPosition_1->setEnabled(true);
    // } else {
    //     ui->pushButton_setUIJSPosition_1->setEnabled(false);
    // }
    // if(-105.0 < qnode_->params_.meas.q[0] && qnode_->params_.meas.q[0] < 105.0) {
    //     ui->pushButton_setUIJSPosition_2->setEnabled(true);
    // } else {
    //     ui->pushButton_setUIJSPosition_2->setEnabled(false);
    // }
    // if(-5.0 < qnode_->params_.meas.q[0] && qnode_->params_.meas.q[0] < 179.0) {
    //     ui->pushButton_setUIJSPosition_3->setEnabled(true);
    // } else {
    //     ui->pushButton_setUIJSPosition_3->setEnabled(false);
    // }

    //// Teaching - button enable
    if(ui->pushButton_teaching_task_start->isChecked()) {
        ui->pushButton_teaching_task_start->setText("Task in Progress..");
        if(qnode_->is_in_task_teaching_stage_) {
            ui->pushButton_set_teaching_pose->setEnabled(true);
            if(ui->pushButton_set_teaching_pose->isChecked()) {
                ui->pushButton_set_teaching_pose->setText("Next Task");
            } else {
                ui->pushButton_set_teaching_pose->setText("Set Pose");
            }
        } else {
            ui->pushButton_set_teaching_pose->setEnabled(false);
            ui->pushButton_set_teaching_pose->setText("-");
        }

        if(qnode_->is_task_finished_) {
            ui->pushButton_teaching_task_start->setChecked(false);
        }
    } else {
        ui->pushButton_teaching_task_start->setText("Start Task for Teaching");
        ui->pushButton_enable_disable->setEnabled(true);
    }

    //// A) w/o Teaching - button enable
    if(ui->pushButton_task_start->isChecked()) {
        ui->pushButton_task_start->setText("Task in Progress..");
        if(qnode_->is_task_finished_) {
            ui->pushButton_task_start->setChecked(false);
        }
    } else {
        ui->pushButton_task_start->setText("Start Task");
        ui->pushButton_enable_disable->setEnabled(true);
    }


    //// B) w/o teaching - repeat task
    if(ui->pushButton_teaching_task_start_repeat->isChecked()) {
        ui->pushButton_teaching_task_start_repeat->setText("Task in Progress..");
        if(qnode_->is_task_finished_) {
            ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            ROS_LOG_INFO("Repeat Start Task! (Performed Task Count: %zu)", cnt_repeat_task_);
            ROS_LOG_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            taskStartRepeatedlyForWithoutTeachingBtnCallback();
        }
    } else {
        ui->pushButton_teaching_task_start_repeat->setText("Start Task Repeatedly\n(for Developer)");
        ui->pushButton_enable_disable->setEnabled(true);
    }




    //// Teaching - button enable
    if(ui->pushButton_teaching_task_start_with_marker->isChecked()) {
        ui->pushButton_teaching_task_start_with_marker->setText("Task in Progress..");
        if(qnode_->is_in_task_teaching_stage_) {
            ui->pushButton_set_teaching_pose_with_marker->setEnabled(true);
            if(ui->pushButton_set_teaching_pose_with_marker->isChecked()) {
                ui->pushButton_set_teaching_pose_with_marker->setText("Next Task");
            } else {
                ui->pushButton_set_teaching_pose_with_marker->setText("Set Pose\n(Marker)");
            }
        } else {
            ui->pushButton_set_teaching_pose_with_marker->setEnabled(false);
            ui->pushButton_set_teaching_pose_with_marker->setText("-");
        }

        if(qnode_->is_task_finished_) {
            ui->pushButton_teaching_task_start_with_marker->setChecked(false);
        }
    } else {
        ui->pushButton_teaching_task_start_with_marker->setText("Start Task for Teaching\n(Marker)");
        ui->pushButton_enable_disable->setEnabled(true);
    }

    //// A) w/o Teaching - button enable
    if(ui->pushButton_task_start_with_marker->isChecked()) {
        ui->pushButton_task_start_with_marker->setText("Task in Progress..");
        if(qnode_->is_task_finished_) {
            ui->pushButton_task_start_with_marker->setChecked(false);
        }
    } else {
        ui->pushButton_task_start_with_marker->setText("Start Task\n(Marker)");
        ui->pushButton_enable_disable->setEnabled(true);
    }
}


void MainWindow_initialSetup::setEnableBtnCallback() {
    qnode_->setEnable(!qnode_->params_.status.is_enable);
}

void MainWindow_initialSetup::setTcpBtnCallback() {
    CsDouble tcp;
    for (int i = 0; i < CS_DOF; i++) {
        if(i < 3) {
            tcp[i] = 0.001*list_set_tcp_x_mm_[i]->text().toDouble();
        } else {
            tcp[i] = list_set_tcp_x_mm_[i]->text().toDouble();
        }
    }
    qnode_->setTcp(tcp); // [m], [deg]
}

void MainWindow_initialSetup::setDrflTcpBtnCallback() {

}

void MainWindow_initialSetup::setStepMoveTranslationScale_1() {
    ui->doubleSpinBox_step_move_x_mm->setValue(0.1); // 0.1mm
}

void MainWindow_initialSetup::setStepMoveTranslationScale_2() {
    ui->doubleSpinBox_step_move_x_mm->setValue(0.5); // 0.5mm
}

void MainWindow_initialSetup::setStepMoveTranslationScale_3() {
    ui->doubleSpinBox_step_move_x_mm->setValue(1.0); // 1mm
}

void MainWindow_initialSetup::setStepMoveTranslationScale_4() {
    ui->doubleSpinBox_step_move_x_mm->setValue(5.0); // 5mm
}

void MainWindow_initialSetup::setStepMoveTranslationScale_5() {
    ui->doubleSpinBox_step_move_x_mm->setValue(10.0); // 10mm
}

void MainWindow_initialSetup::setStepMoveTranslationScale_6() {
    ui->doubleSpinBox_step_move_x_mm->setValue(25.0); // 25mm
}

void MainWindow_initialSetup::setStepMoveTranslationScale_7() {
    ui->doubleSpinBox_step_move_x_mm->setValue(50.0); // 50mm
}

void MainWindow_initialSetup::setStepMoveTranslationScale_8() {
    ui->doubleSpinBox_step_move_x_mm->setValue(100.0); // 100mm
}

void MainWindow_initialSetup::setStepMoveRotationScale_1() {
    ui->doubleSpinBox_step_move_q->setValue(0.1); // 0.1deg
}

void MainWindow_initialSetup::setStepMoveRotationScale_2() {
    ui->doubleSpinBox_step_move_q->setValue(0.5); // 0.5deg
}

void MainWindow_initialSetup::setStepMoveRotationScale_3() {
    ui->doubleSpinBox_step_move_q->setValue(1.0); // 1deg
}

void MainWindow_initialSetup::setStepMoveRotationScale_4() {
    ui->doubleSpinBox_step_move_q->setValue(5.0); // 5deg
}

void MainWindow_initialSetup::setStepMoveRotationScale_5() {
    ui->doubleSpinBox_step_move_q->setValue(15.0); // 15deg
}

void MainWindow_initialSetup::setStepMoveRotationScale_6() {
    ui->doubleSpinBox_step_move_q->setValue(30.0); // 30deg
}

void MainWindow_initialSetup::setStepMoveRotationScale_7() {
    ui->doubleSpinBox_step_move_q->setValue(45.0); // 45deg
}

void MainWindow_initialSetup::setStepMoveRotationScale_8() {
    ui->doubleSpinBox_step_move_q->setValue(90.0); // 90deg
}

// Stop button
void MainWindow_initialSetup::stopAllBtnCallback() {
    qnode_->is_task_mode_ = false;
#if DRFL_CONTROL
    qnode_->drflMoveStop(2);
#else
    qnode_->stopRobot();
#endif
    ui->pushButton_enable_disable->setEnabled(true);
}

void MainWindow_initialSetup::pauseTaskBtnCallback() {
    qnode_->pauseTask();
}

void MainWindow_initialSetup::resumeTaskBtnCallback() {
    qnode_->resumeTask();
}

//// KORAS gripper
void MainWindow_initialSetup::pushButtonKorasGripperConnectionCallback()
{
    if (ui->lineEdit_connection->text() == "Off") {
#if DRFL_CONTROL
        qnode_->drflGrpConnect(true);
#endif
        ui->lineEdit_connection->setText("On");
    } else {
#if DRFL_CONTROL
        qnode_->drflGrpConnect(false);
#endif
        ui->lineEdit_connection->setText("Off");
    }
}

void MainWindow_initialSetup::pushButtonKorasGripperIntializeCallback()
{
    uint16_t position = 0;
    ROS_LOG_INFO("Current Driver address #%u", qnode_->grp_driver_address_);
    qnode_->setGrp((uint16_t)KR_GRP::INIT, position, qnode_->grp_driver_address_);
}

// KORAS gripper
void MainWindow_initialSetup::pushButtonKorasDualGripperIntializeCallback()
{
    // uint16_t position = 0;
    // qnode_->setGrp((uint16_t)KR_GRP::INIT, position, 1);
    // qnode_->setGrp((uint16_t)KR_GRP::INIT, position, 2);

    // Dual tool initialize
    // ROS_LOG_INFO("Dual gripper initialize!");
    // qnode_->current_task_list_ = qnode_->task_planner_->module_task_dual_tool_initialize_;
    // qnode_->beforeBinPickingTaskStart(true);
    // qnode_->task_cycle_ = 0;
}

void MainWindow_initialSetup::pushButtonKorasGripperSlaveChangeCallback()
{
    // qnode_->grp_driver_address_ = ui->task_spinBox_gripperSlaveNum->value(); // 1, 2, ...
    // qnode_->setGrp((uint16_t)KR_GRP::CHANGE_ADDRESS, 0, qnode_->grp_driver_address_);
}

void MainWindow_initialSetup::pushButtonKorasGripperSetInitMinMaxValue()
{
    // setKORASGripperInitialValue();
}

void MainWindow_initialSetup::pushButtonKorasGripperOpenCallback()
{
    // uint16_t position = ui->task_spinBox_gripperOpenValue->value(); // (scale: 0(Open)~10000(Close))
    uint16_t position = 0;
    ROS_LOG_INFO("Open!");
    ROS_LOG_INFO("Current Driver address #%u", qnode_->grp_driver_address_);
    qnode_->setGrp((uint16_t)KR_GRP::OPEN, position, qnode_->grp_driver_address_);
}

void MainWindow_initialSetup::pushButtonKorasGripperCloseCallback()
{
    // uint16_t position = ui->task_spinBox_gripperCloseValue->value(); // (scale: 0(Open)~10000(Close))
    uint16_t position = 10000;
    ROS_LOG_INFO("Close!");
    ROS_LOG_INFO("Current Driver address #%u", qnode_->grp_driver_address_);
    qnode_->setGrp((uint16_t)KR_GRP::CLOSE, position, qnode_->grp_driver_address_);
}

void MainWindow_initialSetup::pushButtonKorasGripperPosCtrlCallback() { // (scale: 0(Open)~10000(Close))
    uint16_t position = ui->task_spinBox_gripperPosValue->value(); // (scale: 0~10000)
    std::cout << "Pos.: " << position << std::endl;
    ROS_LOG_INFO("Current Driver address #%u", qnode_->grp_driver_address_);
    qnode_->setGrp((uint16_t)KR_GRP::POS_CTRL, position, qnode_->grp_driver_address_);
    // ui->lineEdit_gripperPosCmd->setText(QString::fromStdString(qnode_->grpCmd));
}

void MainWindow_initialSetup::pushButtonKorasGripperPosCtrlOnOffCallback() { // (scale: 0(Open)~10000(Close))
    uint16_t position = ui->task_spinBox_gripperPosValue->value(); // (scale: 0~10000)
    uint16_t duration = 1000;
    std::cout << "Pos.: " << position << std::endl;
    if(ui->pushButton_KorasGripperPosCtrlOnOff->isChecked()) {
        qnode_->setGrp((uint16_t)KR_GRP::CLOSE, position, qnode_->grp_driver_address_); // Close
    } else {
        qnode_->setGrp((uint16_t)KR_GRP::POS_CTRL, position, qnode_->grp_driver_address_); // Position control
    }
}

void MainWindow_initialSetup::pushButtonKorasGripperVacuumOnOffCallback() {
    // uint16_t position = ui->task_spinBox_gripperPosValue->value();
    // // std::cout << "Pos.: " << position << std::endl;
    // if(ui->pushButton_KorasGripperVacuumOnOff->isChecked()) {
    //     qnode_->setGrp((uint16_t)KR_GRP::VACUUM_ON, position, qnode_->grp_driver_address_); // Vac ON
    // } else {
    //     qnode_->setGrp((uint16_t)KR_GRP::VACUUM_OFF, position, qnode_->grp_driver_address_); // Vac OFF
    // }
}



/** @brief Korean: UI 버튼함수, 직교공간 상의 목표 자세를 현재자세(absolute mode) 또는 0.0(relative mode)으로 초기화하여 UI에 업데이트한다.
 */
void MainWindow_initialSetup::pushButtonAbsRelCheckClickedCallback() {
    std::vector<double> currentRobotAPose(6);
    arr2Vec(qnode_->params_.meas.x, currentRobotAPose);
    for (std::size_t i = 0; i < currentRobotAPose.size(); i++)
    {
        if(ui->checkBox_isrelativeRobotACS->isChecked()) // relative pose
        {
            robotATargetXList_[i]->setText(QString::number(0.0, 'f', 1));
        }
        else // absolute pose
        {
            robotATargetXList_[i]->setText(QString::number(currentRobotAPose[i], 'f', 5));
        }
    }
}


/** @brief Korean: UI 버튼함수, 직교공간 상의 목표 자세로 로봇을 이동한다.
 */
void MainWindow_initialSetup::pushButtonCSMoveClickedCallback() {
    std::vector<double> pose;
    double xd  = ui->task_doubleSpinBox_CSvelRobotA->value();     // [m, deg]
    double xdd = ui->task_doubleSpinBox_CSaccRobotA->value();
    for (std::size_t i = 0; i < robotATargetXList_.size(); i++) {
        pose.push_back(robotATargetXList_[i]->text().toDouble());
    }
    bool is_relative = ui->checkBox_isrelativeRobotACS->isChecked();
    // bool is_base_frame = !ui->checkBox_is_tcp_move_cs->isChecked();
    bool is_base_frame = true;
    if(fabs(pose[0]) > 1.3 || fabs(pose[1]) > 1.3 || fabs(pose[2]) > 1.0) {
        ROS_LOG_INFO("Check position[x, y, z] scale!");
    } else {
        CsDouble x;
        vec2Arr(pose, x);
        if (JS_DOF == 6 || ((JS_DOF == 7) && !(qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION))) {
            qnode_->moveX(x, xd, xdd, is_base_frame, is_relative);
            // qnode_->moveX(x, xd, xdd, is_base_frame, rotation_frame, is_relative);
        } else if ((JS_DOF == 7) && (qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION)) {
            // double q_redundant = list_x_target_[6]->text().toDouble();
            // qnode_->moveX(x, xd, xdd, is_base_frame, is_relative, q_redundant);
        }
    }
}

/** @brief Korean: UI 버튼함수, 관절공간 상의 목표 관절각도로 로봇을 이동한다.
 */
void MainWindow_initialSetup::pushButtonJSMoveClickedCallback() {
    double qd  = ui->task_doubleSpinBox_JSvelRobotA->value();
    double qdd = ui->task_doubleSpinBox_JSaccRobotA->value();
    bool is_relative = false;
    JsDouble q;
    for (std::size_t i = 0; i < robotATargetQList_.size(); i++) {
        q[i] = robotATargetQList_[i]->text().toDouble();
    }
    qnode_->moveQ(q, qd, qdd, is_relative);
    // std::vector<double> jointPosition;
    // double maxVelocity  = ui->task_doubleSpinBox_JSvelRobotA->value();
    // double acceleration = ui->task_doubleSpinBox_JSaccRobotA->value();
    // for (std::size_t i = 0; i < robotATargetQList_.size(); i++) {
    //     jointPosition.push_back(robotATargetQList_[i]->text().toDouble());
    // }
    // bool isrelative = false;

    // qnode_->setTargetJointPosition(jointPosition, maxVelocity, acceleration, isrelative, true);
}

/** @brief Korean: UI 버튼함수, 직교공간 상의 목표 자세로 로봇을 이동한다.
 */
void MainWindow_initialSetup::pushButtonCSMoveRelativeToolFrameClickedCallback() {

    //// Relative인 경우만
    if(ui->checkBox_isrelativeRobotACS->isChecked())
    {
        ROS_LOG_INFO("UI_CSMOVE_RELATIVE_TOOL_FRAME!");

        std::vector<double> pose;
        double xd  = ui->task_doubleSpinBox_CSvelRobotA->value();
        double xdd = ui->task_doubleSpinBox_CSaccRobotA->value();
        for (std::size_t i = 0; i < robotATargetXList_.size(); i++) {
            pose.push_back(robotATargetXList_[i]->text().toDouble());
        }
        bool is_relative = true;
        bool is_base_frame = false;
        std::string rotation_mode = "tool_frame";
        if(fabs(pose[0]) > 1.3 || fabs(pose[1]) > 1.3 || fabs(pose[2]) > 1.0) {
            ROS_LOG_INFO("Check position[x, y, z] scale!");
        } else {
            CsDouble x;
            vec2Arr(pose, x);
            if (JS_DOF == 6 || ((JS_DOF == 7) && !(qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION))) {
                qnode_->moveX(x, xd, xdd, is_base_frame, is_relative);
                // qnode_->moveX(x, xd, xdd, is_base_frame, rotation_frame, is_relative);
            } else if ((JS_DOF == 7) && (qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION)) {
                // double q_redundant = list_x_target_[6]->text().toDouble();
                // qnode_->moveX(x, xd, xdd, is_base_frame, is_relative, q_redundant);
            }
        }
        // qnode_->setTargetRelativePose(rotation_mode, pose, velocity, acceleration, isRelative, true);
    } else {
        ROS_LOG_INFO("CURRENT MODE IS NOT RELATIVE MODE!");
    }
}

/** @brief Korean: UI 버튼함수, 로봇의 Jog 동작을 수행한다.
 */
void MainWindow_initialSetup::pushButtonJogSelectCallback() {
    if (ui->radioButton_JSJog->isChecked()) {
        qnode_->jog_space_ = JS_JOG;
    } else if (ui->radioButton_CSJog->isChecked()) {
        qnode_->jog_space_ = CS_JOG;
    } else if (ui->radioButton_jog_pad->isChecked()) {
        qnode_->jog_space_ = XPAD_JOG;
        qnode_->jog_xpad_trans_ = ui->doubleSpinBox_jog_xpad_xd->value();
        qnode_->jog_xpad_ori_   = ui->doubleSpinBox_jog_xpad_qd->value();
    }

    qnode_->is_jog_mode_ = true;
}

/** @brief Korean: UI 버튼함수, 로봇의 Jog 동작을 종료한다.
 */
void MainWindow_initialSetup::pushButtonJogModeEndCallback() {
    qnode_->is_jog_mode_ = false;
}

void MainWindow_initialSetup::jogBtnPressed(uint index, bool is_plus, bool is_joint_space) {
    if (qnode_->is_jog_mode_) {
        if ((qnode_->jog_space_ == JS_JOG && is_joint_space) || (qnode_->jog_space_ == CS_JOG && !is_joint_space)) {
            double vel, acc;

            if (!is_joint_space && index < 3) {
                vel = ui->doubleSpinBox_JogSpeed_trans->value();   // [m, deg]
                vel *= is_plus ? +1 : -1;
                acc = fabs(vel) * 2.0;
            } else {
                vel = ui->doubleSpinBox_JogSpeed_ori->value();
                vel *= is_plus ? +1 : -1;
                acc = fabs(vel) * 2.0;
            }

            qnode_->jogStart(index, vel, acc, is_joint_space, !ui->radioButton_rotationToolFrame->isChecked());
        }
    }
}

void MainWindow_initialSetup::jogBtnReleased() {
    qnode_->jogEnd(0.5);
}


/** @brief Korean: UI 버튼함수, 현재 직교공간 상의 자세를 받아서 UI에 업데이트한다.
 */
void MainWindow_initialSetup::pushButtonSetCurrentRobotAPoseClickedCallback() {

#if DRFL_CONTROL
    /////////////////////////////////////////////////
    qnode_->getDrflCurrentPose(); //// 두산 로봇 CS pose 할당
    /////////////////////////////////////////////////
#endif
    std::vector<double> currentRobotAPose(6);
    arr2Vec(qnode_->params_.meas.x, currentRobotAPose);
    std::stringstream ss;
    for (std::size_t i = 0; i < currentRobotAPose.size(); i++) {
        robotATargetXList_[i]->setText(QString::number(currentRobotAPose[i], 'f', 5));
        QString pose_str;
        if(i<3) pose_str = QString::number(currentRobotAPose[i], 'f', 5);
        else pose_str = QString::number(currentRobotAPose[i], 'f', 3);
        std::string str = pose_str.toStdString();
        if(i < currentRobotAPose.size()-1) ss << str << ", ";
        else ss << str;
    }
    QString pose_str_out = QString(ss.str().c_str());
    ui->lineEdit_controlView_targetCS->setText(pose_str_out);
}

/** @brief Korean: UI 버튼함수, 현재 관절공간 상의 관절각도를 받아서 UI에 업데이트한다.
 */
void MainWindow_initialSetup::pushButtonSetCurQRobotAClickedCallback() {
    std::vector<double> curQRobotA(6);
    arr2Vec(qnode_->params_.meas.q, curQRobotA);
    std::stringstream ss;
    for (std::size_t i = 0; i < curQRobotA.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(curQRobotA[i], 'f', 3));
        QString q_str = QString::number(curQRobotA[i], 'f', 3);
        std::string str = q_str.toStdString();
        if(i < curQRobotA.size()-1) ss << str << ", ";
        else ss << str;
    }
    QString q_str_out = QString(ss.str().c_str());
    ui->lineEdit_controlView_targetJS->setText(q_str_out);
}

/** @brief Korean: UI 버튼함수, 현재 관절공간 상의 관절각도를 받아서 UI에 업데이트한다.
 */
void MainWindow_initialSetup::pushButtonGetCurQRobotAClickedCallback() {
    QString q_str_in = ui->lineEdit_controlView_targetJS->text();
    QStringList temp_q_str = q_str_in.split(",");
    std::vector<double> curQRobotA(6);
    for (std::size_t i = 0; i < curQRobotA.size(); i++) {
        curQRobotA[i] = QString(temp_q_str[i]).toDouble();
        robotATargetQList_[i]->setText(QString::number(curQRobotA[i], 'f', 3));
    }
}

/** @brief Korean: UI 버튼함수, 현재 직교공간 상의 자세를 받아서 UI에 업데이트한다.
 */
void MainWindow_initialSetup::pushButtonGetCurXRobotAClickedCallback() {
    QString x_str_in = ui->lineEdit_controlView_targetCS->text();
    QStringList temp_x_str = x_str_in.split(",");
    std::vector<double> curXRobotA(6);
    for (std::size_t i = 0; i < curXRobotA.size(); i++) {
        curXRobotA[i] = QString(temp_x_str[i]).toDouble();
        if(i < 3) {
            robotATargetXList_[i]->setText(QString::number(curXRobotA[i], 'f', 5));
        } else {
            robotATargetXList_[i]->setText(QString::number(curXRobotA[i], 'f', 3));
        }
    }
}

/** @brief Korean: UI 버튼함수, Debug, Run mode에 맞는 로봇의 속도 및 가속도를 설정한다.
 */
void MainWindow_initialSetup::pushButtonSetUIRobotAccVel() {
    if(ui->radioButton_setRobotAccVel_DEBUG->isChecked())
    {
        ui->task_doubleSpinBox_CSvelRobotA->setValue(0.05); // [m/s]
        ui->task_doubleSpinBox_CSaccRobotA->setValue(0.1); // [m/s^2]
        ui->task_doubleSpinBox_JSvelRobotA->setValue(15.0); // [deg/s]
        ui->task_doubleSpinBox_JSaccRobotA->setValue(30.0); // [deg/s^2]
    }

    if(ui->radioButton_setRobotAccVel_RUN->isChecked())
    {
        ui->task_doubleSpinBox_CSvelRobotA->setValue(0.3); // [m/s]
        ui->task_doubleSpinBox_CSaccRobotA->setValue(0.6); // [m/s^2]
        ui->task_doubleSpinBox_JSvelRobotA->setValue(45.0); // [deg/s]
        ui->task_doubleSpinBox_JSaccRobotA->setValue(90.0); // [deg/s^2]
    }
}




void MainWindow_initialSetup::pushButtonSetUIJSPosition0ClickedCallback() {
    std::vector<double> position_q;
    if (std::string(ROBOT_NAME) == "DS_M1013") {
        std::vector<double> home_q = {-0.000, -18.713, 105.394, -0.139, 93.361, -90.833};
        position_q = home_q;
    } else if (std::string(ROBOT_NAME) == "UR10e") {
        std::vector<double> home_q = {0.0, -69.226, -88.043, -112.723, 90.500, 0.0};
        position_q = home_q;
    }

    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }

    // Move
    double qd  = ui->task_doubleSpinBox_JSvelRobotA->value();
    double qdd = ui->task_doubleSpinBox_JSaccRobotA->value();
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    qnode_->moveQ(q, qd, qdd, is_relative);


    // //// Gripper
    // uint16_t position = 8500; // (scale: 0~10000)
    // qnode_->setGrp((uint16_t)KR_GRP::POS_CTRL, position, qnode_->grp_driver_address_);

}


/** @brief Korean: UI 버튼함수, 정의된 관절공간 상의 목표 관절각도를 UI에 업데이트한다.
 */
void MainWindow_initialSetup::pushButtonSetUIJSPosition1ClickedCallback() {
    std::vector<double> position_q;
    if (std::string(ROBOT_NAME) == "DS_M1013") {
        std::vector<double> home_q = {-0.000, -18.713, 105.394, -0.139, 93.361, -90.833};
        position_q = home_q;
    } else if (std::string(ROBOT_NAME) == "UR10e") {
        std::vector<double> home_q = {0.0, -69.226, -88.043, -112.723, 90.500, 0.0};
        position_q = home_q;
    }

    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }

    // Move
    double qd  = ui->task_doubleSpinBox_JSvelRobotA->value();
    double qdd = ui->task_doubleSpinBox_JSaccRobotA->value();
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    qnode_->moveQ(q, qd, qdd, is_relative);


    // //// Gripper
    // uint16_t position = 8500; // (scale: 0~10000)
    // qnode_->setGrp((uint16_t)KR_GRP::POS_CTRL, position, qnode_->grp_driver_address_);

}

void MainWindow_initialSetup::pushButtonSetUIJSPosition2ClickedCallback() {
    std::vector<double> position_q;
    if (std::string(ROBOT_NAME) == "DS_M1013") {
        std::vector<double> home_q = {-0.000, -18.713, 105.394, -0.139, 93.361, -90.833};
        position_q = home_q;
    } else if (std::string(ROBOT_NAME) == "UR10e") {
        std::vector<double> home_q = {0.0, -69.226, -88.043, -112.723, 90.500, 0.0};
        position_q = home_q;
    }

    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }

    // Move
    double qd  = ui->task_doubleSpinBox_JSvelRobotA->value();
    double qdd = ui->task_doubleSpinBox_JSaccRobotA->value();
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    qnode_->moveQ(q, qd, qdd, is_relative);

    // //// Gripper
    // uint16_t position = 8500; // (scale: 0~10000)
    // qnode_->setGrp((uint16_t)KR_GRP::POS_CTRL, position, qnode_->grp_driver_address_);
}

void MainWindow_initialSetup::pushButtonSetUIJSPosition3ClickedCallback() {
    std::vector<double> position_q;
    if (std::string(ROBOT_NAME) == "DS_M1013") {
        std::vector<double> home_q = {-0.000, -18.713, 105.394, -0.139, 93.361, -90.833};
        position_q = home_q;
    } else if (std::string(ROBOT_NAME) == "UR10e") {
        std::vector<double> home_q = {0.0, -69.226, -88.043, -112.723, 90.500, 0.0};
        position_q = home_q;
    }
    
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }

    // Move
    double qd  = ui->task_doubleSpinBox_JSvelRobotA->value();
    double qdd = ui->task_doubleSpinBox_JSaccRobotA->value();
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    qnode_->moveQ(q, qd, qdd, is_relative);

    // //// Gripper
    // uint16_t position = 8500; // (scale: 0~10000)
    // qnode_->setGrp((uint16_t)KR_GRP::POS_CTRL, position, qnode_->grp_driver_address_);
}


void MainWindow_initialSetup::pushButtonSetUIJSPositionPackingClickedCallback() {
    std::vector<double> position_q = { 45.000, 0.000, -157.000, -110.000, -170.000, 0.000 };
    for (std::size_t i = 0; i < position_q.size(); i++) {
        robotATargetQList_[i]->setText(QString::number(position_q[i], 'f', 3));
    }

    // Move
    double qd  = 10.0; // 10deg/s
    double qdd = 20.0; // 20deg/s
    bool is_relative = false;
    JsDouble q;
    vec2Arr(position_q, q);
    qnode_->moveQ(q, qd, qdd, is_relative);

    //// Gripper
    uint16_t position = 10000; // (scale: 0~10000)
    qnode_->setGrp((uint16_t)KR_GRP::CLOSE, position, qnode_->grp_driver_address_);
}



void MainWindow_initialSetup::stepMoveQBtnCallback(uint index, bool is_plus) {
    JsDouble q;

    for (int i = 0; i < JS_DOF; i++) {
        q[i] = 0;
    }

    q[index] = is_plus ? ui->doubleSpinBox_step_move_q->value() :
               ui->doubleSpinBox_step_move_q->value() * -1;

    double qd  = ui->doubleSpinBox_step_move_qd ->value();
    double qdd = ui->doubleSpinBox_step_move_qdd->value();

    qnode_->moveQ(q, qd, qdd, true);
}

void MainWindow_initialSetup::stepMoveXBtnCallback(uint index, bool is_plus) {
    CsDouble x;
    double q_redundant;

    for (int i = 0; i < CS_DOF; i++) {
        x[i] = 0;
    }

    if (index == 6) {
        // JS_DOF = 7 case
        q_redundant = is_plus ? ui->doubleSpinBox_step_move_q->value() :
                      ui->doubleSpinBox_step_move_q->value() * -1;
    } else if (index < 3) {
        x[index] = is_plus ? 0.001*ui->doubleSpinBox_step_move_x_mm->value() :
                   0.001*ui->doubleSpinBox_step_move_x_mm->value() * -1;
    } else {
        x[index] = is_plus ? ui->doubleSpinBox_step_move_q->value() :
                   ui->doubleSpinBox_step_move_q->value() * -1;
    }

    double xd  = 0.001*ui->doubleSpinBox_step_move_xd_mm ->value();
    double xdd = 0.001*ui->doubleSpinBox_step_move_xdd_mm->value();

    bool is_base_frame = true; // base frame

    if (JS_DOF == 6) {
        qnode_->moveX(x, xd, xdd, is_base_frame, true);
    } else if ((JS_DOF == 7) && (qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION)) {
        qnode_->moveX(x, xd, xdd, is_base_frame, true, q_redundant);
    }
}

void MainWindow_initialSetup::stepMoveXToolFrameBtnCallback(uint index, bool is_plus) {
    CsDouble x;
    double q_redundant;

    for (int i = 0; i < CS_DOF; i++) {
        x[i] = 0;
    }

    if (index == 6) {
        // JS_DOF = 7 case
        q_redundant = is_plus ? ui->doubleSpinBox_step_move_q->value() :
                      ui->doubleSpinBox_step_move_q->value() * -1;
    } else if (index < 3) {
        x[index] = is_plus ? 0.001*ui->doubleSpinBox_step_move_x_mm->value() :
                   0.001*ui->doubleSpinBox_step_move_x_mm->value() * -1;
    } else {
        x[index] = is_plus ? ui->doubleSpinBox_step_move_q->value() :
                   ui->doubleSpinBox_step_move_q->value() * -1;
    }

    double xd  = 0.001*ui->doubleSpinBox_step_move_xd_mm ->value();
    double xdd = 0.001*ui->doubleSpinBox_step_move_xdd_mm->value();

    bool is_base_frame = false; // tool frame

    if (JS_DOF == 6) {
        qnode_->moveX(x, xd, xdd, is_base_frame, true);
    } else if ((JS_DOF == 7) && (qnode_->params_.mode.ik_method == InvKineMethod::OPTIMIZATION)) {
        qnode_->moveX(x, xd, xdd, is_base_frame, true, q_redundant);
    }
}


// Total Task
void MainWindow_initialSetup::taskStartTotalTaskBtnCallback() {
    // if(ui->pushButton_total_task_start->isChecked()) {
    //     ui->pushButton_enable_disable->setEnabled(false);
    //     //// Teaching Flag OFF
    //     qnode_->is_task_teaching_mode_ = false;
    //     //// Teaching Flag OFF

    //     // size_t l_task_num = ui->treeWidget_graphyTaskList->currentIndex().row();
    //     size_t l_task_num = task_trial_num_;
    //     ROS_LOG_WARN("[Total Task] Task index: %zu", l_task_num);
    //     ROS_LOG_WARN("Current Task count: %zu", qnode_->task_planner_->module_task_graphy_list_[l_task_num].size());

    //     ////////////////////////////////////////////////////////////////////////
    //     //// Do Current Selected Task
    //     // Task planner tab
    //     ROS_LOG_ERROR("here 1");
    //     qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_list_[l_task_num];
    //     ROS_LOG_ERROR("here 2");
    //     qnode_->beforeTaskStart();
    //     qnode_->task_cycle_ = 0;

    //     // Only developer mode
    //     qnode_->is_developer_mode_ = true;
    //     ////////////////////////////////////////////////////////////////////////


    //     task_trial_num_++;


    // } else {
    //     task_trial_num_ = 0;

    //     ui->pushButton_enable_disable->setEnabled(true);
    //     //// Teaching Flag OFF
    //     qnode_->is_teaching_finished_ = false;
    //     qnode_->is_task_teaching_mode_ = false;
    //     qnode_->is_in_task_teaching_stage_ = false;
    //     //// Teaching Flag OFF
    //     stopAllBtnCallback();
    //     ui->pushButton_set_teaching_pose->setEnabled(false);
    // }
}


// w/o Teaching
void MainWindow_initialSetup::taskStartNoTeachingBtnCallback() {
#ifdef GRAPHY_MODE

    if(ui->pushButton_task_start->isChecked()) {

        if(!qnode_->params_.status.is_enable) { // is robot enable
            int ret = QMessageBox::warning(this, tr("Graphy Robot Task Mode"),
                                        tr("Enable the robot first!"));

            ui->pushButton_task_start->setChecked(false);
            return;
        }

        ////////////////////////////
        //// UNIZ DATA LOADING
        if(!qnode_->is_uniz_file_loaded_) {
            ROS_LOG_WARN("UNIZ FILE UPLOAD START!");
            graphyTestFunctionBtnCallback();
        }
        ////////////////////////////



        ui->pushButton_enable_disable->setEnabled(false);
        //// Teaching Flag ON
        qnode_->is_task_teaching_mode_ = false;
        //// Teaching Flag ON
        size_t l_task_num = ui->treeWidget_graphyTaskList->currentIndex().row();


        if(l_task_num >= 0 && l_task_num < 1000) {
            ROS_LOG_WARN("List index: %zu", l_task_num);
            ROS_LOG_WARN("Current Task count: %zu", qnode_->task_planner_->module_task_graphy_list_[l_task_num].size());

            if(qnode_->task_planner_->module_task_graphy_list_[l_task_num].size() == 0) {
                int ret = QMessageBox::warning(this, tr("Graphy Robot Task Mode"),
                                            tr("No task exists! - Check the aligner index of current task"));
                ui->pushButton_task_start->setChecked(false);
                return;
            }

            ////////////////////////////////////////////////////////////////////////
            //// Do Current Selected Task
            // Task planner tab
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_list_[l_task_num];
            qnode_->beforeTaskStart();
            qnode_->task_cycle_ = 0;

            // Only developer mode
            qnode_->is_developer_mode_ = true;
            ////////////////////////////////////////////////////////////////////////

        } else {
            ROS_LOG_ERROR("Select a task in the list!: %zu", l_task_num);
            // QMessageBox::Warning(this, "Warning", "Select a task in the list!");

            // QMessageBox msgBox;
            // msgBox.setText("Select a task in the list!");
            // msgBox.exec();

            int ret = QMessageBox::warning(this, tr("Graphy Robot Task Mode"),
                                        tr("Select a task in the list!"));

            ui->pushButton_task_start->setChecked(false);
            return;

            // int ret = QMessageBox::warning(this, tr("My Application"),
            //                             tr("The document has been modified.\n"
            //                                 "Do you want to save your changes?"),
            //                             QMessageBox::Save | QMessageBox::Discard
            //                             | QMessageBox::Cancel,
            //                             QMessageBox::Save);

            // QMessageBox *msg = new QMessageBox();
            // QMessageBox::StandardButton reply;
            // msg->setDefaultButton(reply);
            // reply = QMessageBox::critical(this, "Select a task in the list!", "Go to next task",
            //                                 QMessageBox::Yes|QMessageBox::No);
            // if (reply == QMessageBox::Yes) {
            //     _taskWindow->getTaskManager()->_qnode->resumeTask();
            // } else {
            // }
        }

    } else {
        ui->pushButton_enable_disable->setEnabled(true);
        //// Teaching Flag OFF
        qnode_->is_teaching_finished_ = false;
        qnode_->is_task_teaching_mode_ = false;
        qnode_->is_in_task_teaching_stage_ = false;
        //// Teaching Flag OFF
        stopAllBtnCallback();
        ui->pushButton_set_teaching_pose->setEnabled(false);
    }


#endif
}



// w/o Teaching
void MainWindow_initialSetup::taskStartRepeatedlyForWithoutTeachingBtnCallback() {
#ifdef GRAPHY_MODE

    if(ui->pushButton_teaching_task_start_repeat->isChecked()) {
        cnt_repeat_task_++;

        if(!qnode_->params_.status.is_enable) { // is robot enable
            int ret = QMessageBox::warning(this, tr("Graphy Robot Task Mode"),
                                        tr("Enable the robot first!"));

            ui->pushButton_teaching_task_start_repeat->setChecked(false);
            return;
        }

        ui->pushButton_enable_disable->setEnabled(false);
        //// Teaching Flag ON
        qnode_->is_task_teaching_mode_ = false;
        //// Teaching Flag ON
        size_t l_task_num = ui->treeWidget_graphyTaskList->currentIndex().row();
        if(l_task_num >= 0 && l_task_num < 1000) {
            ROS_LOG_WARN("List index: %zu", l_task_num);
            ROS_LOG_WARN("Current Task count: %zu", qnode_->task_planner_->module_task_graphy_list_[l_task_num].size());

            if(qnode_->task_planner_->module_task_graphy_list_[l_task_num].size() == 0) {
                int ret = QMessageBox::warning(this, tr("Graphy Robot Task Mode"),
                                            tr("No task exists! - Check the aligner index of current task"));
            ui->pushButton_teaching_task_start_repeat->setChecked(false);
                return;
            }

            ////////////////////////////////////////////////////////////////////////
            //// Do Current Selected Task
            // Task planner tab
            // qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_list_[l_task_num];


            qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_tip_changing_1_to_2;
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_graphy_tip_changing_2_to_1.begin(), qnode_->task_planner_->module_task_graphy_tip_changing_2_to_1.end());

            // qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_tip_changing_1_to_6;
            // qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_graphy_tip_changing_6_to_1.begin(), qnode_->task_planner_->module_task_graphy_tip_changing_6_to_1.end());

            // robot enable/disable
            qnode_->current_task_list_.insert(qnode_->current_task_list_.end(), qnode_->task_planner_->module_task_robot_disable_and_enable.begin(), qnode_->task_planner_->module_task_robot_disable_and_enable.end());


            qnode_->beforeTaskStart();
            qnode_->task_cycle_ = 0;

            // Only developer mode
            qnode_->is_developer_mode_ = true;
            ////////////////////////////////////////////////////////////////////////

        } else {
            ROS_LOG_ERROR("Select a task in the list!: %zu", l_task_num);
            // QMessageBox::Warning(this, "Warning", "Select a task in the list!");

            // QMessageBox msgBox;
            // msgBox.setText("Select a task in the list!");
            // msgBox.exec();

            int ret = QMessageBox::warning(this, tr("Graphy Robot Task Mode"),
                                        tr("Select a task in the list!"));

            ui->pushButton_teaching_task_start_repeat->setChecked(false);
            return;

            // int ret = QMessageBox::warning(this, tr("My Application"),
            //                             tr("The document has been modified.\n"
            //                                 "Do you want to save your changes?"),
            //                             QMessageBox::Save | QMessageBox::Discard
            //                             | QMessageBox::Cancel,
            //                             QMessageBox::Save);

            // QMessageBox *msg = new QMessageBox();
            // QMessageBox::StandardButton reply;
            // msg->setDefaultButton(reply);
            // reply = QMessageBox::critical(this, "Select a task in the list!", "Go to next task",
            //                                 QMessageBox::Yes|QMessageBox::No);
            // if (reply == QMessageBox::Yes) {
            //     _taskWindow->getTaskManager()->_qnode->resumeTask();
            // } else {
            // }
        }

    } else {
        cnt_repeat_task_ = 0;
        ui->pushButton_enable_disable->setEnabled(true);
        //// Teaching Flag OFF
        qnode_->is_teaching_finished_ = false;
        qnode_->is_task_teaching_mode_ = false;
        qnode_->is_in_task_teaching_stage_ = false;
        //// Teaching Flag OFF
        stopAllBtnCallback();
        ui->pushButton_set_teaching_pose->setEnabled(false);
    }
#endif
}

// Teaching
void MainWindow_initialSetup::taskStartForTeachingBtnCallback() {
#ifdef GRAPHY_MODE
    if(ui->pushButton_teaching_task_start->isChecked()) {

        if(!qnode_->params_.status.is_enable) { // is robot enable
            int ret = QMessageBox::warning(this, tr("Graphy Robot Teaching Mode"),
                                        tr("Enable the robot first!"));

            ui->pushButton_teaching_task_start->setChecked(false);
            return;
        }


        ////////////////////////////
        //// UNIZ DATA LOADING
        if(!qnode_->is_uniz_file_loaded_) {
            ROS_LOG_WARN("UNIZ FILE UPLOAD START!");
            graphyTestFunctionBtnCallback();
        }
        ////////////////////////////


        ui->pushButton_enable_disable->setEnabled(false);
        //// Teaching Flag ON
        qnode_->is_task_teaching_mode_ = true;
        //// Teaching Flag ON
        size_t l_task_num = ui->treeWidget_graphyTaskList->currentIndex().row();
        if(l_task_num >= 0 && l_task_num < 1000) {
            ROS_LOG_WARN("List index: %zu", l_task_num);
            ROS_LOG_WARN("Current Task count: %zu", qnode_->task_planner_->module_task_graphy_list_[l_task_num].size());

            if(qnode_->task_planner_->module_task_graphy_list_[l_task_num].size() == 0) {
                int ret = QMessageBox::warning(this, tr("Graphy Robot Teaching Mode"),
                                            tr("No task exists! - Check the aligner index of current task"));
                ui->pushButton_teaching_task_start->setChecked(false);
                return;
            }

            ////////////////////////////////////////////////////////////////////////
            //// Do Current Selected Task
            // Task planner tab
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_list_[l_task_num];
            qnode_->beforeTaskStart();
            qnode_->task_cycle_ = 0;

            // Only developer mode
            qnode_->is_developer_mode_ = true;
            ////////////////////////////////////////////////////////////////////////

        } else {
            ROS_LOG_ERROR("Select a task in the list!: %zu", l_task_num);
            // QMessageBox::Warning(this, "Warning", "Select a task in the list!");

            // QMessageBox msgBox;
            // msgBox.setText("Select a task in the list!");
            // msgBox.exec();

            int ret = QMessageBox::warning(this, tr("Graphy Robot Teaching Mode"),
                                        tr("Select a task in the list!"));


            ui->pushButton_teaching_task_start->setChecked(false);
            return;

            // int ret = QMessageBox::warning(this, tr("My Application"),
            //                             tr("The document has been modified.\n"
            //                                 "Do you want to save your changes?"),
            //                             QMessageBox::Save | QMessageBox::Discard
            //                             | QMessageBox::Cancel,
            //                             QMessageBox::Save);

            // QMessageBox *msg = new QMessageBox();
            // QMessageBox::StandardButton reply;
            // msg->setDefaultButton(reply);
            // reply = QMessageBox::critical(this, "Select a task in the list!", "Go to next task",
            //                                 QMessageBox::Yes|QMessageBox::No);
            // if (reply == QMessageBox::Yes) {
            //     _taskWindow->getTaskManager()->_qnode->resumeTask();
            // } else {
            // }
        }

    } else {
        ui->pushButton_enable_disable->setEnabled(true);
        //// Teaching Flag OFF
        qnode_->is_teaching_finished_ = false;
        qnode_->is_task_teaching_mode_ = false;
        qnode_->is_in_task_teaching_stage_ = false;
        //// Teaching Flag OFF
        stopAllBtnCallback();
        ui->pushButton_set_teaching_pose->setEnabled(false);
    }
#endif
}

// Teaching by human (w.r.t base frame)
void MainWindow_initialSetup::setTeachingPoseBtnCallback() {
#ifdef GRAPHY_MODE
    qnode_->is_task_button_clicked_ = true;

    ROS_LOG_WARN("[UI, Teaching Tag]: %s - %s\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());
    if(ui->pushButton_set_teaching_pose->isChecked()) {


        QMessageBox msgBox;
        size_t cnt = 0;

        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Ok);
        // msgBox.addButton(tr("Save Pose"), QMessageBox::YesRole);
        // msgBox.addButton(tr("Skip"), QMessageBox::NoRole);


        msgBox.setStyleSheet(
            "   QLabel {"
            "font: bold 24px;"
            "}"
            "QPushButton {"
            "font: bold 36px;"
            "}"
        );

        msgBox.setWindowTitle(" ");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("Do you want to save a new pose?");

        std::stringstream ss;
        ss << "[" << qnode_->getTaskTagTeachingPose() << " - " << qnode_->getIdxTeachingPose() << "]";
        msgBox.setInformativeText(ss.str().c_str());

        int ret = msgBox.exec();

        // std::stringstream ss;

        switch(ret)
        {
            case QMessageBox::Ok: {

                //// JSON Save
                //// FILE PATH
                //// JSON INPUT
                // std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/teaching_pose/graphy_demo_cs_waypoint.json";
                std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/teaching_pose/graphy_demo_cs_waypoint.json";
                std::ifstream ifs(pose_config_path.c_str());
                json j_in = json::parse(ifs);
                json j_out = j_in;
                std::vector<double> pose_update(6);
                ss.str("");

                arr2Vec(qnode_->params_.meas.x, pose_update);
                for(int j = 0; j < 3; j++) {
                    pose_update[j] = round(pose_update[j]*1e7) / 1e7; // [m]
                    pose_update[j + 3] = round(pose_update[j + 3]*1e4) / 1e4; // [deg]
                }
                j_out[qnode_->getTaskTagTeachingPose()][qnode_->getIdxTeachingPose()] = pose_update;

                std::ofstream ofs(pose_config_path.c_str());
                ofs << j_out.dump(4) << std::endl;

                for (std::size_t i = 0; i < pose_update.size(); i++) {
                    robotATargetXList_[i]->setText(QString::number(pose_update[i], 'f', 5));
                    QString pose_str;
                    if(i<3) pose_str = QString::number(pose_update[i], 'f', 5);
                    else pose_str = QString::number(pose_update[i], 'f', 3);
                    std::string str = pose_str.toStdString();
                    if(i < pose_update.size()-1) ss << str << ", ";
                    else ss << str;
                }
                QString pose_str_out = QString(ss.str().c_str());
                ui->lineEdit_controlView_targetCS->setText(pose_str_out);
                ROS_LOG_WARN("SAVING POSE... [%s - %s]\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());

                //// Jog 시에 이 플래그가 false가 되므로 다시 켜주기
                qnode_->is_task_mode_ = true;
                ////
                break;
            }
            case QMessageBox::Cancel: {
                ROS_LOG_WARN("SKIP... [%s - %s]\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());
                qnode_->is_task_mode_ = true;
                break;
            }
        }


    } else {

        QMessageBox msgBox;
        size_t cnt = 0;


        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Ok);

        // msgBox.addButton(tr("Finish"), QMessageBox::YesRole);
        // msgBox.addButton(tr("Retry"), QMessageBox::NoRole);


        msgBox.setStyleSheet(
            "   QLabel {"
            "font: bold 24px;"
            "}"
            "QPushButton {"
            "font: bold 36px;"
            "}"
        );

        msgBox.setWindowTitle(" ");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("Click OK when teaching is finished\n(NOTICE: The robot will be moved to the teaching pose.)");

        std::stringstream ss;
        ss << "[" << qnode_->getTaskTagTeachingPose() << " - " << qnode_->getIdxTeachingPose() << "]";
        msgBox.setInformativeText(ss.str().c_str());

        int ret = msgBox.exec();
        switch(ret)
        {
            case QMessageBox::Ok:
                ROS_LOG_WARN("Teaching is successful! [%s - %s]\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());
                qnode_->is_teaching_finished_ = true;
                qnode_->is_task_mode_ = true;
            break;

            case QMessageBox::Cancel:
                ROS_LOG_WARN("Teaching is in progress... [%s - %s]\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());
                qnode_->is_task_mode_ = true;
            break;
        }
    }
#endif
}

// Teaching with Marker Detection (w.r.t marker frame)
void MainWindow_initialSetup::setTeachingPoseWithMarkerDetectionBtnCallback() {
#ifdef GRAPHY_MODE
    qnode_->is_task_button_clicked_ = true;
    ROS_LOG_WARN("[MARKER DETECTION][UI, Teaching Tag]: %s - %s\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());
    if(ui->pushButton_set_teaching_pose_with_marker->isChecked()) {

        QMessageBox msgBox;
        size_t cnt = 0;

        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Ok);
        // msgBox.addButton(tr("Save Pose"), QMessageBox::YesRole);
        // msgBox.addButton(tr("Skip"), QMessageBox::NoRole);


        msgBox.setStyleSheet(
            "   QLabel {"
            "font: bold 24px;"
            "}"
            "QPushButton {"
            "font: bold 36px;"
            "}"
        );

        msgBox.setWindowTitle(" ");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("[MARKER DETECTION] Do you want to save a new pose?");

        std::stringstream ss;
        ss << "[" << qnode_->getTaskTagTeachingPose() << " - " << qnode_->getIdxTeachingPose() << "]";
        msgBox.setInformativeText(ss.str().c_str());

        int ret = msgBox.exec();

        // std::stringstream ss;

        switch(ret)
        {
            case QMessageBox::Ok: {

                //// JSON Save
                //// FILE PATH
                //// JSON INPUT
                std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/marker_pose/graphy_demo_marker_cs_waypoint.json";
                std::ifstream ifs(pose_config_path.c_str());
                json j_in = json::parse(ifs);
                json j_out = j_in;
                std::vector<double> pose_update(6);
                ss.str("");

                //// Pose w.r.t Marker Frame
                CsDouble pose_tag_cur_pose;
                qnode_->getTagBasedPose(qnode_->params_.meas.x, pose_tag_cur_pose);
                arr2Vec(pose_tag_cur_pose, pose_update);

                for(int j = 0; j < 3; j++) {
                    pose_update[j] = round(pose_update[j]*1e7) / 1e7; // [m]
                    pose_update[j + 3] = round(pose_update[j + 3]*1e4) / 1e4; // [deg]
                }
                j_out[qnode_->getTaskTagTeachingPose()][qnode_->getIdxTeachingPose()] = pose_update;

                std::ofstream ofs(pose_config_path.c_str());
                ofs << j_out.dump(4) << std::endl;

                for (std::size_t i = 0; i < pose_update.size(); i++) {
                    robotATargetXList_[i]->setText(QString::number(pose_update[i], 'f', 5));
                    QString pose_str;
                    if(i<3) pose_str = QString::number(pose_update[i], 'f', 5);
                    else pose_str = QString::number(pose_update[i], 'f', 3);
                    std::string str = pose_str.toStdString();
                    if(i < pose_update.size()-1) ss << str << ", ";
                    else ss << str;
                }
                QString pose_str_out = QString(ss.str().c_str());
                ui->lineEdit_controlView_targetCS->setText(pose_str_out);
                ROS_LOG_WARN("[MARKER DETECTION] SAVING POSE... [%s - %s]\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());

                //// Jog 시에 이 플래그가 false가 되므로 다시 켜주기
                qnode_->is_task_mode_ = true;
                ////
                break;
            }
            case QMessageBox::Cancel: {
                ROS_LOG_WARN("[MARKER DETECTION] SKIP... [%s - %s]\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());
                qnode_->is_task_mode_ = true;
                break;
            }
        }


    } else {

        QMessageBox msgBox;
        size_t cnt = 0;


        msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        msgBox.setDefaultButton(QMessageBox::Ok);

        // msgBox.addButton(tr("Finish"), QMessageBox::YesRole);
        // msgBox.addButton(tr("Retry"), QMessageBox::NoRole);


        msgBox.setStyleSheet(
            "   QLabel {"
            "font: bold 24px;"
            "}"
            "QPushButton {"
            "font: bold 36px;"
            "}"
        );

        msgBox.setWindowTitle(" ");
        msgBox.setIcon(QMessageBox::Warning);
        msgBox.setText("[MARKER DETECTION] Click OK when teaching is finished\n(NOTICE: The robot will be moved to the teaching pose.)");

        std::stringstream ss;
        ss << "[" << qnode_->getTaskTagTeachingPose() << " - " << qnode_->getIdxTeachingPose() << "]";
        msgBox.setInformativeText(ss.str().c_str());

        int ret = msgBox.exec();
        switch(ret)
        {
            case QMessageBox::Ok:
                ROS_LOG_WARN("[MARKER DETECTION] Teaching is successful! [%s - %s]\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());
                qnode_->is_teaching_finished_ = true;
                qnode_->is_task_mode_ = true;
            break;

            case QMessageBox::Cancel:
                ROS_LOG_WARN("[MARKER DETECTION] Teaching is in progress... [%s - %s]\n", qnode_->getTaskTagTeachingPose().c_str(), qnode_->getIdxTeachingPose().c_str());
                qnode_->is_task_mode_ = true;
            break;
        }
    }
#endif
}

// Teaching with Marker
void MainWindow_initialSetup::taskStartForTeachingWithMarkerBtnCallback() {
#ifdef GRAPHY_MODE
    if(ui->pushButton_teaching_task_start_with_marker->isChecked()) {

        if(!qnode_->params_.status.is_enable) { // is robot enable
            int ret = QMessageBox::warning(this, tr("Graphy Robot Teaching Mode With Marker"),
                                        tr("Enable the robot first!"));

            ui->pushButton_teaching_task_start_with_marker->setChecked(false);
            return;
        }


        ////////////////////////////
        //// UNIZ DATA LOADING
        if(!qnode_->is_uniz_file_loaded_) {
            ROS_LOG_WARN("UNIZ FILE UPLOAD START!");
            graphyTestFunctionBtnCallback();
        }
        ////////////////////////////


        ui->pushButton_enable_disable->setEnabled(false);
        //// Teaching Flag ON
        qnode_->is_task_teaching_mode_ = true;
        //// Teaching Flag ON
        size_t l_task_num = ui->treeWidget_graphyMarkerTaskList->currentIndex().row();
        if(l_task_num >= 0 && l_task_num < 1000) {
            ROS_LOG_WARN("List index: %zu", l_task_num);
            ROS_LOG_WARN("Current Task count: %zu", qnode_->task_planner_->module_task_graphy_marker_list_[l_task_num].size());

            if(qnode_->task_planner_->module_task_graphy_marker_list_[l_task_num].size() == 0) {
                int ret = QMessageBox::warning(this, tr("Graphy Robot Teaching Mode With Marker"),
                                            tr("No task exists! - Check the aligner index of current task"));
                ui->pushButton_teaching_task_start_with_marker->setChecked(false);
                return;
            }

            ////////////////////////////////////////////////////////////////////////
            //// Do Current Selected Task
            // Task planner tab
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_marker_list_[l_task_num];
            qnode_->beforeTaskStart();
            qnode_->task_cycle_ = 0;

            // Only developer mode
            qnode_->is_developer_mode_ = true;
            ////////////////////////////////////////////////////////////////////////

        } else {
            ROS_LOG_ERROR("Select a task in the list!: %zu", l_task_num);
            // QMessageBox::Warning(this, "Warning", "Select a task in the list!");

            // QMessageBox msgBox;
            // msgBox.setText("Select a task in the list!");
            // msgBox.exec();

            int ret = QMessageBox::warning(this, tr("Graphy Robot Teaching Mode With Marker"),
                                        tr("Select a task in the list!"));


            ui->pushButton_teaching_task_start_with_marker->setChecked(false);
            return;

            // int ret = QMessageBox::warning(this, tr("My Application"),
            //                             tr("The document has been modified.\n"
            //                                 "Do you want to save your changes?"),
            //                             QMessageBox::Save | QMessageBox::Discard
            //                             | QMessageBox::Cancel,
            //                             QMessageBox::Save);

            // QMessageBox *msg = new QMessageBox();
            // QMessageBox::StandardButton reply;
            // msg->setDefaultButton(reply);
            // reply = QMessageBox::critical(this, "Select a task in the list!", "Go to next task",
            //                                 QMessageBox::Yes|QMessageBox::No);
            // if (reply == QMessageBox::Yes) {
            //     _taskWindow->getTaskManager()->_qnode->resumeTask();
            // } else {
            // }
        }

    } else {
        ui->pushButton_enable_disable->setEnabled(true);
        //// Teaching Flag OFF
        qnode_->is_teaching_finished_ = false;
        qnode_->is_task_teaching_mode_ = false;
        qnode_->is_in_task_teaching_stage_ = false;
        //// Teaching Flag OFF
        stopAllBtnCallback();
        ui->pushButton_set_teaching_pose_with_marker->setEnabled(false);
    }
#endif
}

// No Teaching with Marker
void MainWindow_initialSetup::taskStartNoTeachingWithMarkerBtnCallback() {
#ifdef GRAPHY_MODE
    if(ui->pushButton_task_start_with_marker->isChecked()) {

        if(!qnode_->params_.status.is_enable) { // is robot enable
            int ret = QMessageBox::warning(this, tr("Graphy Robot Task Mode With Marker"),
                                        tr("Enable the robot first!"));

            ui->pushButton_task_start_with_marker->setChecked(false);
            return;
        }

        ////////////////////////////
        //// UNIZ DATA LOADING
        if(!qnode_->is_uniz_file_loaded_) {
            ROS_LOG_WARN("UNIZ FILE UPLOAD START!");
            graphyTestFunctionBtnCallback();
        }
        ////////////////////////////



        ui->pushButton_enable_disable->setEnabled(false);
        //// Teaching Flag ON
        qnode_->is_task_teaching_mode_ = false;
        //// Teaching Flag ON
        size_t l_task_num = ui->treeWidget_graphyMarkerTaskList->currentIndex().row();


        if(l_task_num >= 0 && l_task_num < 1000) {
            ROS_LOG_WARN("List index: %zu", l_task_num);
            ROS_LOG_WARN("Current Task count: %zu", qnode_->task_planner_->module_task_graphy_marker_list_[l_task_num].size());

            if(qnode_->task_planner_->module_task_graphy_marker_list_[l_task_num].size() == 0) {
                int ret = QMessageBox::warning(this, tr("Graphy Robot Task Mode With Marker"),
                                            tr("No task exists! - Check the aligner index of current task"));
                ui->pushButton_task_start_with_marker->setChecked(false);
                return;
            }

            ////////////////////////////////////////////////////////////////////////
            //// Do Current Selected Task
            // Task planner tab
            qnode_->current_task_list_ = qnode_->task_planner_->module_task_graphy_marker_list_[l_task_num];
            qnode_->beforeTaskStart();
            qnode_->task_cycle_ = 0;

            // Only developer mode
            qnode_->is_developer_mode_ = true;
            ////////////////////////////////////////////////////////////////////////

        } else {
            ROS_LOG_ERROR("Select a task in the list!: %zu", l_task_num);
            // QMessageBox::Warning(this, "Warning", "Select a task in the list!");

            // QMessageBox msgBox;
            // msgBox.setText("Select a task in the list!");
            // msgBox.exec();

            int ret = QMessageBox::warning(this, tr("Graphy Robot Task Mode With Marker"),
                                        tr("Select a task in the list!"));

            ui->pushButton_task_start_with_marker->setChecked(false);
            return;

            // int ret = QMessageBox::warning(this, tr("My Application"),
            //                             tr("The document has been modified.\n"
            //                                 "Do you want to save your changes?"),
            //                             QMessageBox::Save | QMessageBox::Discard
            //                             | QMessageBox::Cancel,
            //                             QMessageBox::Save);

            // QMessageBox *msg = new QMessageBox();
            // QMessageBox::StandardButton reply;
            // msg->setDefaultButton(reply);
            // reply = QMessageBox::critical(this, "Select a task in the list!", "Go to next task",
            //                                 QMessageBox::Yes|QMessageBox::No);
            // if (reply == QMessageBox::Yes) {
            //     _taskWindow->getTaskManager()->_qnode->resumeTask();
            // } else {
            // }
        }

    } else {
        ui->pushButton_enable_disable->setEnabled(true);
        //// Teaching Flag OFF
        qnode_->is_teaching_finished_ = false;
        qnode_->is_task_teaching_mode_ = false;
        qnode_->is_in_task_teaching_stage_ = false;
        //// Teaching Flag OFF
        stopAllBtnCallback();
        ui->pushButton_set_teaching_pose_with_marker->setEnabled(false);
    }
#endif
}





void MainWindow_initialSetup::genGraphyBtnCallback() {
#ifdef GRAPHY_MODE
    qnode_->task_planner_->ref_unit_task_.vel_js = ui->task_doubleSpinBox_JSvelRobotA->value();
    qnode_->task_planner_->ref_unit_task_.acc_js = ui->task_doubleSpinBox_JSaccRobotA->value();
    qnode_->task_planner_->ref_unit_task_.vel_cs = ui->task_doubleSpinBox_CSvelRobotA->value();
    qnode_->task_planner_->ref_unit_task_.acc_cs = ui->task_doubleSpinBox_CSaccRobotA->value();

    // for (int i = 0; i < CS_DOF; i++) {
    //     qnode_->task_planner_->impedance_default_.m[i] = list_impedance_m_[i]->text().toDouble();
    //     qnode_->task_planner_->impedance_default_.b[i] = list_impedance_b_[i]->text().toDouble();
    //     qnode_->task_planner_->impedance_default_.k[i] = list_impedance_k_[i]->text().toDouble();

    //     qnode_->task_planner_->impedance_default_.force_limit[i]     = list_impedance_force_limit_[i]->text().toDouble();
    //     qnode_->task_planner_->impedance_default_.force_selection[i] = list_impedance_selection_[i]->isChecked();
    // }

    // teeth index
    uint16_t teeth_idx_in = ui->task_spinBox_targetTeethNum->value(); // 1, 2, ..., 8
    qnode_->task_planner_->makeGraphyTaskList(teeth_idx_in - 1);
    qnode_->task_planner_->makeGraphyTaskWithMarkerDetectionList(teeth_idx_in - 1, true); // marker tasks, is_marker_applied: true

#endif
}

void MainWindow_initialSetup::do3DScanningBtnCallback() {
    taskScanningParameter scan_parameter;
    scan_parameter.target_id = 36;
    scan_parameter.target_name = "peg";
    scan_parameter.is_mask_pixel_fixed = false;
    std::vector<int> mask_pixel_list(4,10);
    // mask_pixel_list[0] = fixedMaskPixelList_[0]->text().toInt();
    // mask_pixel_list[1] = fixedMaskPixelList_[1]->text().toInt();
    // mask_pixel_list[2] = fixedMaskPixelList_[2]->text().toInt();
    // mask_pixel_list[3] = fixedMaskPixelList_[3]->text().toInt();
    scan_parameter.mask_pixel_list = mask_pixel_list;
    scan_parameter.sampling_num = ui->lineEdit_scanSamplingNum->text().toInt();
    scan_parameter.is_base_frame_unknown = false;
    scan_parameter.do_scan_sampling = ui->radioButton_assemblyUI_scanSampling->isChecked();
    scan_parameter.do_save_data = ui->radioButton_assemblyUI_scanDataSave->isChecked();
    scan_parameter.do_image_processing = ui->radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    scan_parameter.do_single_matching = true; // matching service
    scan_parameter.do_not_scan_do_load_data = ui->radioButton_assemblyUI_scanDataLoad->isChecked();
    scan_parameter.skip_detection_mask = ui->radioButton_assemblyUI_skipDetectionMask->isChecked();


    //// 241125 추가
    scan_parameter.robot_dh_vec = qnode_->getRobotDHParameters(); // [m], [deg]
    scan_parameter.robot_tcp_default = qnode_->getRobotDefaultTCP(); // [m], [deg]
    scan_parameter.robot_tcp = qnode_->getRobotTCP(); // [m], [deg]


    std::vector<double> robot_dh_vec = scan_parameter.robot_dh_vec;
    std::vector<double> robot_tcp_default = scan_parameter.robot_tcp_default;
    std::vector<double> robot_tcp = scan_parameter.robot_tcp;
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

    arr2Vec(qnode_->params_.meas.q , scan_parameter.scan_position); // scanning joint position


    // // dlgDoScanning(scan_parameter); // signal to BinPickingDialog
    // if(scan_parameter.target_id >= 31 && scan_parameter.target_id < 45) {
    //     qnode_->m_bin_picking_node->scanZIVID(scan_parameter);
    // } else {
    //     ROS_LOG_INFO("Wrong target id!");
    // }
    qnode_->m_bin_picking_node->scanZIVID(scan_parameter);

}


void MainWindow_initialSetup::doKUAISTaskBtnCallback() {
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_save_data = ui->radioButton_assemblyUI_scanDataSave->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_image_processing = ui->radioButton_assemblyUI_scanDoImageProcessing->isChecked();
    qnode_->m_bin_picking_node->m_ptr_bin_picking_node_target_object_list[qnode_->m_bin_picking_node->current_target_object_idx_]->m_scan_parameter.do_single_matching = false; // matching topic
    if(ui->pushButton_KUAISTaskStart->isChecked()) {
        // Flag initialize
        ROS_LOG_WARN("KUAIS Function!");


        qnode_->task_planner_->genKUAISTask();
        qnode_->current_task_list_ = qnode_->task_planner_->kuais_total_task_;

        qnode_->beforeBinPickingTaskStart(true);
        qnode_->task_cycle_ = 0;
    } else {
        stopAllBtnCallback();
    }
}


void MainWindow_initialSetup::graphyTestFunctionBtnCallback() {

#ifdef GRAPHY_MODE
    // ROS_LOG_WARN("Graphy Test Function Start!");
    /////////////////////////////////////////////////////
    ROS_LOG_WARN("UNIZ TXT DATA PROCESSING START!");
    //////// JSON SETTING LOAD
    std::string config_path = "/home/" + USER_NAME + "/Desktop/UNIZ data/uniz_setting/uniz_setting.json";
    std::ifstream ifs(config_path.c_str());
    json j_uniz_setting_in = json::parse(ifs);

    //// File name
    std::string file_name = j_uniz_setting_in["file_name"];

    std::stringstream ss;
    ss << "/home/" << USER_NAME << "/Desktop/UNIZ data/" << file_name << ".txt";
    ROS_LOG_WARN("UNIZ TXT DATA PROCESSING... (file_path: %s)", ss.str().c_str());

    QFile file(ss.str().c_str());
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) return;
    QTextStream in(&file);
    // QRegExp separator("(,|#|\|)");
    QRegExp separator("[(,|#|\|)]");
    /////////////////////////////////////////////////////

    std::vector<std::vector<std::vector<double>>> l_bbox_set;

    size_t l_cnt_aligner = 0;
    while (!in.atEnd()) {
        QString line = in.readLine();
        // std::cout << "New Line Start!" << std::endl;
        QStringList list = line.split(separator);
        std::vector<std::vector<double>> l_bbox_tmp;
        size_t cnt_tmp = 0;
        std::vector<double> vec_tmp(3);
        for(size_t i=1; i<list.size(); i++) {
            cnt_tmp++;
            if(i > 0 && cnt_tmp != 4) {
                // std::cout << "QString List #" << i + 1 << ": " << list[i].toStdString() << std::endl;
                vec_tmp[cnt_tmp - 1] = list[i].toDouble();
            } else {
                l_bbox_tmp.push_back(vec_tmp);
                cnt_tmp = 0;
            }
        }

        l_bbox_set.push_back(l_bbox_tmp);
        // std::cout << "New Line End!" << std::endl;
        l_cnt_aligner++;
    }



    // QString text;
    // text = in.readAll();
    // std::cout << text.toStdString() << std::endl;
    file.close();

    for(size_t i=0; i<l_bbox_set.size(); i++) {
        // std::cout << "Aligner #" << i + 1 << " Start!" << std::endl;
        for(size_t j=0; j<l_bbox_set[i].size(); j++) {
            // std::cout << "P" << j+1 << ": " << l_bbox_set[i][j][0] << ", " << l_bbox_set[i][j][1] << ", "  << l_bbox_set[i][j][2] << std::endl;
        }
        // std::cout << "Aligner #" << i + 1 << " End!" << std::endl;
    }

    qnode_->sendUNIZData(l_bbox_set);

    ROS_LOG_WARN("UNIZ TXT DATA PROCESSING END! (file_path: %s)", ss.str().c_str());




    // QProcess::execute("/bin/bash", {"-c", "bash -c \"v4l2-ctl -d /dev/video0 --list-ctrls\""});
    // QProcess::execute("/bin/bash", {"-c", "bash -c \"v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=0\""});
    // QProcess::execute("/bin/bash", {"-c", "bash -c \"v4l2-ctl -d /dev/video0 --list-ctrls\""});
    // QProcess::execute("/bin/bash", {"-c", "bash -c \"v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=1\""});
    // QProcess::execute("/bin/bash", {"-c", "bash -c \"v4l2-ctl -d /dev/video0 --list-ctrls\""});
    // QProcess::execute("/bin/bash", {"-c", "bash -c \"v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=0\""});
    // QProcess::execute("/bin/bash", {"-c", "bash -c \"v4l2-ctl -d /dev/video0 --list-ctrls\""});


    // ROS_LOG_WARN("Graphy Test Function End!");
#endif

}


void MainWindow_initialSetup::graphyRobotPowerOnOffBtnCallback() {

#ifdef GRAPHY_MODE
    if(ui->pushButton_robot_power_on_off->isChecked()) {
        ROS_LOG_WARN("graphyRobotPowerOnOffBtnCallback! - POWER OFF");
        qnode_->sendRelayCommand("91");
    } else {
        ROS_LOG_WARN("graphyRobotPowerOnOffBtnCallback! - POWER ON");
        qnode_->sendRelayCommand("90");
    }
    #endif
}

// step motor plc
   // STEP_MOTOR_POSITION_CONTROL, 12
   // STEP_MOTOR_SET_VELOCITY 13
void MainWindow_initialSetup::SetStepMotorPosition() {
    double pos_in = ui->lineEdit_pulse_position->text().toDouble();
    pos_in = std::min(pos_in, 360.0);
    pos_in = std::max(pos_in, -360.0);

    //// 0(0deg)~1600(360deg)
    int value = static_cast<int>(round(pos_in*4.444444)); // 1deg = 4.444444 pulse
    ROS_LOG_WARN("Step motor - pos.:%i (%0.2f)", value, pos_in);
    qnode_->setPLCModbusCommandWriteRegister(12, value); // 위치 값 저장
}

void MainWindow_initialSetup::SetStepMotorVelocity() {
    double vel_in = ui->lineEdit_pulse_velocity->text().toDouble();
    vel_in = std::min(vel_in, 120.0);
    vel_in = std::max(vel_in, 0.0);
    int value = static_cast<int>(round(vel_in*4.444444)); // 1deg = 4.444444 pulse
    ROS_LOG_WARN("Step motor - vel.:%i (%0.2f)", value, vel_in);
    qnode_->setPLCModbusCommandWriteRegister(13, value);
}


void MainWindow_initialSetup::pushButtonModbusSetIPAddress() {
#if BIN_PICKING_FLAG
    std::string ip = ui->lineEdit_MODBUS_IP->text().toStdString();
    int portno = static_cast<int>(ui->lineEdit_MODBUS_PORT->text().toDouble());
    qnode_->ip_modbus_ = ip;
    qnode_->port_modbus_ = portno;
#endif
}

void MainWindow_initialSetup::pushButtonModbusConnect() {
#if BIN_PICKING_FLAG
    pushButtonModbusSetIPAddress();
    qnode_->setMODBUSComm(true); // M00000 트리거
#endif
}

void MainWindow_initialSetup::pushButtonModbusClose() {
#if BIN_PICKING_FLAG
    qnode_->setMODBUSComm(false);
#endif
}

void MainWindow_initialSetup::pushButtonSelectPlcWriteAddressClickedCallback() {

#if IS_PLC_LS_XG5000
    //// write
    if(ui->radioButton_select_plc_address_write_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41000");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_2->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41001");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("0.0");
    } else if(ui->radioButton_select_plc_address_write_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41002");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_4->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41003");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_5->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41004");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_6->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x41005");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("0123456789ABCDEFGHIJ");
    }
#else
    //// write
    if(ui->radioButton_select_plc_address_write_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1000");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_2->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1001");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("0.0");
    } else if(ui->radioButton_select_plc_address_write_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1002");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_4->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1003");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_5->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1004");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_write_6->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("1005");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("0123456789ABCDEFGHIJ");
    }
#endif

}

void MainWindow_initialSetup::pushButtonSelectPlcReadAddressClickedCallback() {

#if IS_PLC_LS_XG5000
    //// read
    if(ui->radioButton_select_plc_address_read_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x42000");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_read_2->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x42001");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_read_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("0x42002");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    }
#else
    //// read
    if(ui->radioButton_select_plc_address_read_1->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("2000");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_read_2->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("2001");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    } else if(ui->radioButton_select_plc_address_read_3->isChecked()) {
        ui->lineEdit_MODBUS_WORD_ADDRESS->setText("2002");
        ui->lineEdit_MODBUS_DATA_TO_SEND->setText("-");
    }
#endif
}


/// @brief PLC 관련 함수: Write
void MainWindow_initialSetup::pushButtonModbusWriteDataBoolType() {
    bool ok;
    // 16진수 "0x40005"도 변환 가능하도록 처리
    uint16_t word_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_WORD_ADDRESS->text().toUInt(&ok, 0)); // 0을 전달하면 10진수/16진수 자동 인식
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS WORD ADDRESS! [%s]", __func__);
        return;
    }

    uint16_t bit_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_BIT_ADDRESS->text().toUShort(&ok));
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS BIT ADDRESS! [%s]", __func__);
        return;
    }
    if(ui->pushButton_MODBUS_WRITE_DATA_BOOL_TYPE->isChecked()) {
        qnode_->PLCModbusWriteStatusFromRobotToAMR(word_address, true, bit_address, true);
    } else {
        qnode_->PLCModbusWriteStatusFromRobotToAMR(word_address, true, bit_address, false);
    }
}

void MainWindow_initialSetup::pushButtonModbusWriteDataInt16Type() {

    bool ok;
    uint16_t word_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_WORD_ADDRESS->text().toUInt(&ok, 0)); // 0을 전달하면 10진수/16진수 자동 인식
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS WORD ADDRESS! [%s]", __func__);
        return;
    }

    uint16_t bit_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_BIT_ADDRESS->text().toUShort(&ok));
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS BIT ADDRESS! [%s]", __func__);
        return;
    }

    ////
    double angle = ui->lineEdit_MODBUS_DATA_TO_SEND->text().toDouble(&ok);
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid TARGET ROTATION ANGLE! [%s]", __func__);
        return;
    }

    qnode_->setPLCModbusRotationAngle(word_address, bit_address, angle);
}




void MainWindow_initialSetup::pushButtonModbusWriteDataASCIIType() {

    bool ok;
    uint16_t word_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_WORD_ADDRESS->text().toUInt(&ok, 0)); // 0을 전달하면 10진수/16진수 자동 인식
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS WORD ADDRESS! [%s]", __func__);
        return;
    }

    uint16_t bit_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_BIT_ADDRESS->text().toUShort(&ok));
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS BIT ADDRESS! [%s]", __func__);
        return;
    }

    ////
    std::string str_data = ui->lineEdit_MODBUS_DATA_TO_SEND->text().toStdString();
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid TARGET ROTATION ANGLE! [%s]", __func__);
        return;
    }

    qnode_->setPLCModbusBarcodeWrite(word_address, bit_address, str_data);
}



/// @brief PLC 관련 함수 : Read
/// @brief PLC 관련 함수 : Read
void MainWindow_initialSetup::pushButtonModbusReadDataBoolType() {
    bool ok;
    uint16_t word_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_WORD_ADDRESS->text().toUInt(&ok, 0)); // 0을 전달하면 10진수/16진수 자동 인식
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS WORD ADDRESS! [%s]", __func__);
        return;
    }

    uint16_t bit_address = static_cast<uint16_t>(ui->lineEdit_MODBUS_BIT_ADDRESS->text().toUShort(&ok));
    if (!ok) {
        ROS_LOG_WARN("Error: Invalid MODBUS BIT ADDRESS! [%s]", __func__);
        return;
    }

    bool status_read = false;
    if (qnode_->PLCModbusReadStatusFromAMR(word_address, bit_address, status_read)) {
        if (status_read) {
            ROS_LOG_WARN("[%s] MODBUS Read - Address: 0x%X, Bit: %u → Flag: TRUE", __func__, word_address, bit_address);
        } else {
            ROS_LOG_WARN("[%s] MODBUS Read - Address: 0x%X, Bit: %u → Flag: FALSE", __func__, word_address, bit_address);
        }
    } else {
        ROS_LOG_WARN("[%s] MODBUS Read failed - Address: 0x%X, Bit: %u", __func__, word_address, bit_address);
    }
}

void MainWindow_initialSetup::pushButtonModbusMonitoringPLC() {
    if(ui->pushButton_MODBUS_MonitoringPLC->isChecked()) {
        qnode_->is_plc_task_mode_monitoring_ = true;
    } else {
        qnode_->is_plc_task_mode_monitoring_ = false;
    }
}

void MainWindow_initialSetup::pushButtonModbusTest() {
    // qnode_->PLCModbusWriteStatusFromRobotToAMR(PLCStatusFromRobotToAMR::ROBOT_OPERATION_START, false);
}



void MainWindow_initialSetup::onTestButtonClicked() {
    std::vector<CsDouble> cs_poses = {{0, 100, 0, 0, 0, 0}, {0, -200, 0, 0, 0, 0}};
    qnode_->moveSplineX(cs_poses, 50, 100, true, true, 0);

    std::vector<JsDouble> js_poses = {{0, 20, 0, 0, 0, 0}, {0, 0, 0, 0, 50, 0}};
    qnode_->moveSplineQ(js_poses, 25, 50, true);
}

void MainWindow_initialSetup::setupPlanner() {


#if SW_MODE_COOKING
    cookingPlanner_ = dynamic_cast<TaskPlannerCooking*>(qnode_->task_planner_.get());
    if (!cookingPlanner_) {
        std::cerr << "task_planner_ is not TaskPlannerCooking!" << std::endl;
    }
#endif
}