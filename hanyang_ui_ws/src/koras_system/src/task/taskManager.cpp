#include "taskManager.hpp"


//#include <boost/python.hpp>

using QtNodes::NodeId;
using QtNodes::NodeRole;
using QtNodes::PortType;
using QtNodes::PortIndex;
using QtNodes::ConnectionId;



//TaskManager::TaskManager(DataFlowGraphModel &graphModel)
TaskManager::TaskManager(DataFlowGraphModel &graphModel, QNode* qnode)
    : _graphmodel(graphModel), _qnode(qnode) //QWidget(parent),
{
    task_count = 0;
    tag_counts.resize(5, 0);
    zoom_checkbox = new QCheckBox("Zoom ON/OFF");
    zoom_button = new QPushButton("Zoom Current Task");

    QTreeWidget *moduleTaskTreeView = new QTreeWidget(this);
    loadTaskbutton = new QPushButton("Load module task");
    moduleTaskTreeView->setHeaderLabel("태스크 목록");
    // int moduleCount = 0;

    // teeth_index_in = 0;
    // QLabel* teethLabel = new QLabel("teeth index :");
    // QLineEdit* teethIndexLineEdit = new QLineEdit("0");


    // QObject::connect(teethIndexLineEdit, &QLineEdit::textChanged, this, [this](const QString& text) {
    //     teeth_index_in = text.toInt();
    // });
    teeth_index_in = 0;
    _qnode->task_planner_->ref_unit_task_.vel_js = 30.0;
    _qnode->task_planner_->ref_unit_task_.acc_js = 45.0;
    // _qnode->task_planner_->ref_unit_task_.vel_js = 75.0;
    // _qnode->task_planner_->ref_unit_task_.acc_js = 90.0;
    _qnode->task_planner_->ref_unit_task_.vel_cs = 0.1;
    _qnode->task_planner_->ref_unit_task_.acc_cs = 0.2;

#ifdef GRAPHY_MODE
    _qnode->task_planner_->makeGraphyTaskList(teeth_index_in);
    _qnode->task_planner_->makeGraphyTaskWithMarkerDetectionList(teeth_index_in, true); // marker tasks, is_marker_applied: true
#endif

    auto list = _qnode->task_planner_->module_task_graphy_list_;
    std::vector<QString> name_list = std::vector<QString>
        {
            // tr("[Task 1] X") ,tr("[Task 2] 프린터 열기"),tr("[Task 3] X"),tr("[Task 4] X"),tr("[Task 5] X"),tr("[Task 6] 거치대 놓기"),tr("[Task 7] 거치대 수거"),
            // tr("[Task 8] 경화기 문열기"),tr("[Task 9] 플레이트 집기"),tr("[Task 10] 탈레진기 열기"),tr("[Task 11] 이빨 넣기"),tr("[Task 12] 이빨 뜯기"), tr("[Task 13] 탈레진기 닫기") ,tr("[Task 14] 펜 터치"),tr("[Task 15] X"),
            // tr("[Task 16] 홈 포즈"),tr("[Task 17] 태그 감지"),tr("[Task 18] 탈레진기 버튼 돌리기"),tr("[Task 19] 이빨 넣기"), tr("[Task 20] 경화기 이빨 놓기 x1"), tr("[Task 21] 스크래핑 기계에 프린팅 플레이트 넣기"),
            // tr("[Task 22] 경화기 닫기"), tr("[Task 23] 이빨 뜯기 x8"), tr("[Task 24] 경화기 이빨 놓기 x8"), tr("[Task 25] 스크래핑 장치 열고, 프린팅 플레이트 꺼내서 거치대로 이동"), tr("[Task 26] 볼트 풀고 프린팅 플레이트 놓기"), tr("[Task 27] 프린팅 플레이트 다시 집어넣기 및 고정 핀 원위치 복구"), tr("[Task 28] 프린터 문 닫기"), tr("[Task 29] 전체 작업 수행"),
            // tr("[팁 교환] tip 1 to 2"), tr("[팁 교환] tip 1 to 5"), tr("[팁 교환] tip 1 to 6"),
            // tr("[팁 교환] tip 2 to 1"), tr("[팁 교환] tip 2 to 5"), tr("[팁 교환] tip 2 to 6"),
            // tr("[팁 교환] tip 5 to 1"), tr("[팁 교환] tip 5 to 2"), tr("[팁 교환] tip 5 to 6"),
            // tr("[팁 교환] tip 6 to 1"), tr("[팁 교환] tip 6 to 2"), tr("[팁 교환] tip 6 to 5")

            tr("[Task A-1] 3d_printer_door_open") ,tr("[Task A-2] 3d_printer_plate_pick_and_place"),
            tr("[Task B-1] teeth_tear_x8"),tr("[Tip change] tip_changing_1_to_2"),tr("[Task B-2] pick_teeth_and_place_from_printing_plate_to_legin_plate_x8"),tr("[Tip change] tip_changing_2_to_1"),
            tr("[Task C-1] open_lesin_washing_machine_start"),tr("[Task C-2] pick_plate_and_loading_lesin_washing_machine"),tr("[Task C-3] close_lesin_washing_machine_start"),tr("[Task C-4] pushbutton_and_rotate_lesin_washing_machine"),
            tr("[Tip change] tip_changing_1_to_2"),tr("[Task Z-1] 3d_printer_plate_180_rotation_for_scraping"),tr("[Tip change] tip_changing_2_to_1"),tr("[Task Z-2] 3d_printer_plate_to_scraping_machine"),tr("[Tip change] tip_changing_1_to_2"),
            tr("[Task C-5] open_lesin_washing_machine_end"),tr("[Task C-6] unloading_lesin_washing_machine_and_place_2nd_plate"),tr("[Task C-7] close_lesin_washing_machine_end"),
            tr("[Tip change] tip_changing_2_to_5"),tr("[Task D-1] pick_and_place_to_hardening_plate_x8"),tr("[Tip change] tip_changing_5_to_2"),
            tr("[Task E-1] open_hardening_machine_start"),tr("[Task E-2] pick_plate_and_loading_plate_to_hardening_machine"),tr("[Task E-3] close_hardening_machine"),tr("[Task E-4] touch_monitor_hardening_machine"),
            tr("[Tip change] tip_changing_2_to_1"),tr("[Task Z-7] scraping_machine_door_open"),tr("[Tip change] tip_changing_2_to_1"),tr("[Task Z-3] scraping_machine_to_3d_printer_plate"),tr("[Tip change] tip_changing_1_to_2"),tr("[Task Z-4] 3d_printer_plate_to_3d_printer_home"),tr("[Tip change] tip_changing_2_to_1"),
            tr("[Task E-5] open_hardening_machine_end"),tr("[Task E-6] unloading_plate_from_hardening_machine_and_place_to_home_plate"),
            tr("[Task Z-5] touch_monitor_3d_printer"), tr("[Task Z-6] touch_monitor_hardening_device"),
            tr("[JS] Initial position"), tr("[JS] Stage A-C position"), tr("[JS] Stage D-E position"),
            tr("[팁 교환] tip 1 to 2"), tr("[팁 교환] tip 1 to 5"), tr("[팁 교환] tip 1 to 6"),
            tr("[팁 교환] tip 2 to 1"), tr("[팁 교환] tip 2 to 5"), tr("[팁 교환] tip 2 to 6"),
            tr("[팁 교환] tip 5 to 1"), tr("[팁 교환] tip 5 to 2"), tr("[팁 교환] tip 5 to 6"),
            tr("[팁 교환] tip 6 to 1"), tr("[팁 교환] tip 6 to 2"), tr("[팁 교환] tip 6 to 5")

        };



    auto list_menu_map = new QMap<QString, std::vector<UnitTask>>();

    for (size_t i = 0; i < list.size(); ++i) {
        if (i < name_list.size()) {
            list_menu_map->insert(name_list[i], list[i]);
            auto moduleItem = new QTreeWidgetItem(moduleTaskTreeView);
            moduleItem->setText(0, name_list[i]);
            moduleTaskTreeView->insertTopLevelItem(i,moduleItem);
        }
    }

    QObject::connect(loadTaskbutton, &QPushButton::clicked, this, [this, moduleTaskTreeView, list_menu_map] () {
        if(true) {
            // qDebug() <<  "현재 인덱스 ::"<< (moduleTaskTreeView->currentIndex().row() + 1);

            //선택한 인덱스 리스트 가져오기 map에서.. 혹은... 위치 인덱스로 바로..
            NodeId beforeId = 0;
            NodeId id = 0;

            for(auto unit_task :_qnode->task_planner_->module_task_graphy_list_[moduleTaskTreeView->currentIndex().row()]) {
                // qDebug() << "before & id :: " << beforeId << ", " <<  id << "\n";
                switch (unit_task.task_mode) {

                    case (TASK_DEFAULT): {
                        std::cout << "task default" << std::endl;
                    } break;

                    case (TASK_SET_TCP): {
                        id = _graphmodel.addNode("SET TCP TASK");
                        _graphmodel.setNodeData(id, NodeRole::Position, QPointF(id % 15 * 230, id / 15 * 100));
                        SetTCPModel* model = _graphmodel.delegateModel<SetTCPModel>(id);

                        for (size_t i = 0; i < unit_task.tcp.size(); ++i) {
                            model->lineEdits1[i]->setText(QString::number(unit_task.tcp[i]));
                        }
                        if(beforeId != 0) {
                            _graphmodel.addConnection(ConnectionId{beforeId, 0, id, 0});
                        }
                        beforeId = id;
                    } break;

                    case (TASK_JSMOVE): {
                        id = _graphmodel.addNode("JS TARGET TASK");
                        _graphmodel.setNodeData(id, NodeRole::Position, QPointF(id % 15 * 230, id / 15 * 100));
                        JsTargetModel* model = _graphmodel.delegateModel<JsTargetModel>(id);

                        for (size_t i = 0; i < unit_task.q_target.size(); ++i) {
                            model->lineEdits1[i]->setText(QString::number(unit_task.q_target[i]));
                        }
                        model->is_relative_checkbox->setChecked(unit_task.relative);

                        if(beforeId != 0) {
                            _graphmodel.addConnection(ConnectionId{beforeId, 0, id, 0});
                        }
                        beforeId = id;
                    } break;

                    case (TASK_CSMOVE2): {
                        id = _graphmodel.addNode("CS TARGET TASK");
                        _graphmodel.setNodeData(id, NodeRole::Position, QPointF(id % 15 * 230, id / 15 * 100));
                        CsTargetModel* model = _graphmodel.delegateModel<CsTargetModel>(id);

                        for (size_t i = 0; i < unit_task.x_target.size(); ++i) {
                            model->lineEdits1[i]->setText(QString::number(unit_task.x_target[i]));
                        }
                        model->lineEditsAcc->setText(QString::number(unit_task.acc_cs_custom));
                        model->lineEditsVel->setText(QString::number(unit_task.vel_cs_custom));
                        model->is_relative_checkbox->setChecked(unit_task.relative);
                        if(beforeId != 0) {
                            _graphmodel.addConnection(ConnectionId{beforeId, 0, id, 0});
                        }
                        beforeId = id;
                    } break;

                    case (TASK_CSMOVE_TOOL_FRAME): {
                        id = _graphmodel.addNode("CS TARGET TOOL FRAME TASK");
                        _graphmodel.setNodeData(id, NodeRole::Position, QPointF(id % 15 * 230, id / 15 * 100));
                        CsTargetToolFrameModel* model = _graphmodel.delegateModel<CsTargetToolFrameModel>(id);

                        for (size_t i = 0; i < unit_task.x_target.size(); ++i) {
                            model->lineEdits1[i]->setText(QString::number(unit_task.x_target[i]));
                        }
                        model->lineEditsAcc->setText(QString::number(unit_task.acc_cs_custom));
                        model->lineEditsVel->setText(QString::number(unit_task.vel_cs_custom));
                        if(beforeId != 0) {
                            _graphmodel.addConnection(ConnectionId{beforeId, 0, id, 0});
                        }
                        beforeId = id;
                    } break;

                    case (TASK_DELAY): {
                        id = _graphmodel.addNode("TIME DELAY TASK");
                        _graphmodel.setNodeData(id, NodeRole::Position, QPointF(id % 15 * 230, id / 15 * 100));
                        DelayModel* model = _graphmodel.delegateModel<DelayModel>(id);
                        model->_lineEditDelay->setText(QString::number(unit_task.delay_10ms));

                        if(beforeId != 0) {
                            _graphmodel.addConnection(ConnectionId{beforeId, 0, id, 0});
                        }
                        beforeId = id;
                    } break;

                    case (TASK_KORAS_GRIPPER_COMMAND): {
                        id = _graphmodel.addNode("KORAS GRIPPER TASK");
                        _graphmodel.setNodeData(id, NodeRole::Position, QPointF(id % 15 * 230, id / 15 * 100));
                        BP_KORASGripperModel* model = _graphmodel.delegateModel<BP_KORASGripperModel>(id);
                        model->_comboBoxGrpCmd->setCurrentText(model->grp_cmd_map->key(static_cast<KR_GRP>(unit_task.bp_param.gripper_command)));
                        model->_lineEditPosition->setText(QString::number(unit_task.bp_param.gripper_param[0]));
                        model->_lineEditSpeed->setText(QString::number(unit_task.bp_param.gripper_param[1]));

                        if(beforeId != 0) {
                            _graphmodel.addConnection(ConnectionId{beforeId, 0, id, 0});
                        }
                        beforeId = id;
                    } break;

                    case (TASK_IRLGRP): {
                        id = _graphmodel.addNode("IRL GRIPPER TASK");
                        _graphmodel.setNodeData(id, NodeRole::Position, QPointF(id % 15 * 230, id / 15 * 100));
                        IrlGrpModel* model = _graphmodel.delegateModel<IrlGrpModel>(id);
                        model->_comboBoxGrpCmd->setCurrentText(model->grp_cmd_map->key(static_cast<KR_GRP>((uint16_t)unit_task.grp_command)));
                        model->_lineEditValue->setText(QString::number(unit_task.grp_value));
                        model->_lineEditAddress->setText(QString::number(unit_task.grp_address));

                        if(beforeId != 0) {
                            _graphmodel.addConnection(ConnectionId{beforeId, 0, id, 0});
                        }
                        beforeId = id;
                    } break;

                    case (TASK_KORAS_GRIPPER_INITIALIZE): {
                        id = _graphmodel.addNode("INITIALIZE GRIPPER TASK");
                        _graphmodel.setNodeData(id, NodeRole::Position, QPointF(id % 15 * 230, id / 15 * 100));
                        // InitalizeGripperModel* model = _graphmodel.delegateModel<InitalizeGripperModel>(id);
                        if(beforeId != 0) {
                            _graphmodel.addConnection(ConnectionId{beforeId, 0, id, 0});
                        }
                        beforeId = id;
                    } break;

                    case (TASK_SET_TEACHING_TARGET_POSE): {
                        id = _graphmodel.addNode("SET TEACHING TARGET POSE");
                        _graphmodel.setNodeData(id, NodeRole::Position, QPointF(id % 15 * 230, id / 15 * 100));
                        SetTeachingTargetPoseModel* model = _graphmodel.delegateModel<SetTeachingTargetPoseModel>(id);
                        for (size_t i = 0; i < unit_task.x_target.size(); ++i) {
                            model->lineEdits1[i]->setText(QString::number(unit_task.x_target[i]));
                        }
                        // Teaching pose - Tag
                        model->lineEdits_teaching_pose->setText(QString::fromStdString(unit_task.tag_teaching_pose));
                        model->lineEdits_pose_idx_teaching_pose->setText(QString::fromStdString(unit_task.pose_idx_teaching_pose));

                        if(beforeId != 0) {
                            _graphmodel.addConnection(ConnectionId{beforeId, 0, id, 0});
                        }
                        beforeId = id;
                    } break;

                    case (TASK_CSMOVE_BASE_FRAME_TEACHING_TARGET_POSE): {
                        id = _graphmodel.addNode("CS TARGET TASK BASE FRAME(Teaching Target Pose)");
                        _graphmodel.setNodeData(id, NodeRole::Position, QPointF(id % 15 * 230, id / 15 * 100));
                        CsTargetBaseFrameTeachingTargetPoseModel* model = _graphmodel.delegateModel<CsTargetBaseFrameTeachingTargetPoseModel>(id);
                        for (size_t i = 0; i < unit_task.x_target.size(); ++i) {
                            model->lineEdits1[i]->setText(QString::number(unit_task.x_target[i]));
                        }
                        model->lineEditsAcc->setText(QString::number(unit_task.acc_cs_custom));
                        model->lineEditsVel->setText(QString::number(unit_task.vel_cs_custom));
                        if(beforeId != 0) {
                            _graphmodel.addConnection(ConnectionId{beforeId, 0, id, 0});
                        }
                        beforeId = id;
                    } break;

                    default:
                    std::cout << "TASK IS NOT ADDED: " << unit_task.task_mode << std::endl;
                    break;

                }
            }
        } else {
            QMessageBox::warning(this, "warning", tr("작업을 선택해야합니다."));
        }

    });

    // QObject::connect(_qnode, &QNode::pauseLogger, this, [this]() {
    //     //POP UP MESSAGE
    //     auto nodeId_task = task_nodes[task_count];
    //     if (_graphmodel.nodeData(nodeId_task, NodeRole::Type) == "POPUP MESSAGE") {

    //         QMessageBox *msg = new QMessageBox();
    //         QMessageBox::StandardButton reply;
    //         msg->setDefaultButton(reply);
    //         reply = QMessageBox::critical(this, "Task pause", "Go to next task",
    //                                         QMessageBox::Yes|QMessageBox::No);
    //         if (reply == QMessageBox::Yes) {
    //             _qnode->resumeTask();
    //         } else {
    //         }
    //     }
    // });

    // QObject::connect(_qnode, &QNode::taskFinLogger, this, [this]() {
    //     //UPDATE TASK INFO
    //     TaskInfo taskInfo_ = _qnode->getTaskInfo();
    //     auto completed_nodeId = task_nodes[task_count];
    //     QJsonObject data = _graphmodel.nodeData(completed_nodeId, NodeRole::InternalData).toJsonObject();
    //     int completed_task_index = data["internal-data"].toObject()["task_index"].toInt();

    //     taskInfo_.taskInfo[completed_task_index][tag_counts[completed_task_index]] = true;
    //     taskInfo_.compelete_index = completed_task_index;
    //     taskInfo_.compelete_tag = tag_counts[completed_task_index];

    //     // qDebug() << "인덱스 , 태그 " <<taskInfo_.compelete_index << taskInfo_.compelete_tag << "\n";

    //     tag_counts[completed_task_index] += 1;
    //     //POP UP MESSAGE
    //     if(task_count < task_nodes.size() - 1) {
    //         task_count++;
    //         zoomAction();
    //     }
    //     _qnode->setTaskInfoForMonitor(taskInfo_);
    // });

    QObject::connect(_qnode, &QNode::beforeTaskSignal, this, [this]() {
        setBeforeTaskSignal();
    });

    QObject::connect(zoom_button, &QPushButton::clicked, this, [=] () {
        if(task_nodes.size() > 0) {
            auto nodeId_task = task_nodes[task_count];
            Q_EMIT _graphmodel.zoomInNode(nodeId_task);
        }
    });

    isTaskPaused = false;
    QVBoxLayout *mainLayout = new QVBoxLayout;

    QVBoxLayout *vLayout1_1 = new QVBoxLayout;
    QHBoxLayout *hLayout1 = new QHBoxLayout;
    QVBoxLayout *vLayout1 = new QVBoxLayout;
    QVBoxLayout *vLayout2 = new QVBoxLayout;

    QGridLayout *progressIconLayout = new QGridLayout;
    QHBoxLayout *hLayout2 = new QHBoxLayout;

    editButton = new QPushButton;
    editButton->setIcon(QIcon(":edit_script.png"));
    editButton->setIconSize(QSize(30, 30));
    editButton->setText("edit");
    editButton->setCheckable(true);

    scriptEdit = new QTextEdit;
    scriptEdit->setReadOnly(true);

    taskTreeView = new QTreeWidget(this);
    taskTreeView->header()->close();

    refreshButton = new QPushButton;
    refreshButton->setIcon(QIcon(":refresh_script.png"));
    refreshButton->setText("refresh");
    refreshButton->setIconSize(QSize(30, 30));

    sendEdit = new QTextEdit;
    receiveEdit = new QTextEdit;
    sendEdit->setEnabled(false);
    // receiveEdit->setEnabled(false);

    progressStatusBar = new QProgressBar;
    progressStatusBar->setOrientation(Qt::Horizontal);
    progressStatusBar->setRange(0, 100); // Let's say it goes from 0 to 100
    progressStatusBar->setValue(50); // With a current value of 10

    playButton = new QPushButton;
    pauseButton = new QPushButton;
    resumeButton = new QPushButton;

    stopButton = new QPushButton;

    playButton->setIcon(QIcon(":play.png"));
    pauseButton->setIcon(QIcon(":pause.png"));
    resumeButton->setIcon(QIcon(":resume.png"));
    stopButton->setIcon(QIcon(":stop.svg"));

    playButton->setIconSize(QSize(30, 30));
    pauseButton->setIconSize(QSize(30, 30));
    resumeButton->setIconSize(QSize(30, 30));
    stopButton->setIconSize(QSize(30, 30));

    const QSize BUTTON_SIZE = QSize(200, 200);
    playButton->setMaximumSize(BUTTON_SIZE);
    pauseButton->setMaximumSize(BUTTON_SIZE);
    resumeButton->setMaximumSize(BUTTON_SIZE);
    stopButton->setMaximumSize(BUTTON_SIZE);

    vLayout1_1->addWidget(new QLabel(tr("COMMAND SCRIPT")));
    vLayout1_1->addWidget(editButton);
    //vLayout1_1->addWidget(scriptEdit);
    vLayout1_1->addWidget(taskTreeView);

    vLayout1_1->addWidget(refreshButton, 25);

    vLayout1->addWidget(new QLabel(tr("SEND LOG")));
    vLayout1->addWidget(sendEdit);
    vLayout1->addWidget(new QLabel(tr("RECEIVE LOG")));
    vLayout1->addWidget(receiveEdit);
    hLayout1->addLayout(vLayout1_1);
    hLayout1->addLayout(vLayout1);

    vLayout2->addLayout(progressIconLayout);
    vLayout2->addWidget(progressStatusBar);

    hLayout2->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Ignored));
    hLayout2->addWidget(playButton);
    hLayout2->addWidget(pauseButton);
    hLayout2->addWidget(stopButton);
    hLayout2->addSpacerItem(new QSpacerItem(0, 0, QSizePolicy::Expanding, QSizePolicy::Ignored));
    vLayout2->addLayout(hLayout2);

    mainLayout->addWidget(zoom_checkbox);
    mainLayout->addWidget(zoom_button);
    mainLayout->addWidget(moduleTaskTreeView);
    mainLayout->addWidget(loadTaskbutton);
    mainLayout->addLayout(hLayout1);
    mainLayout->addLayout(vLayout2);
    setLayout(mainLayout);

    connect(playButton, &QPushButton::clicked, this, &TaskManager::OnPlayerButtonClicked);
    connect(pauseButton, &QPushButton::clicked, this, &TaskManager::OnPauseButtonClicked);
    connect(stopButton, &QPushButton::clicked, this, &TaskManager::OnTaskStopButtonClicked);
    connect(editButton, &QPushButton::clicked, this, &TaskManager::OnEditButtonClicked);
    connect(refreshButton, &QPushButton::clicked, this, &TaskManager::OnRefreshButtonClicked);
    setWindowTitle(tr("Task Monitoring"));
}

void TaskManager::GenTaskScriptFromGraph(){
    NodeId startingPoint = QtNodes::InvalidNodeId;
    // find task start model
    for(auto const nodeId : _graphmodel.allNodeIds()) {
        auto jsonObject = _graphmodel.saveNode(nodeId);

        if (_graphmodel.nodeData(nodeId, NodeRole::Type) == "TASK START") {
            startingPoint = nodeId;
        }
    }

    if (startingPoint == QtNodes::InvalidNodeId) {
        QMessageBox::warning(this, "warning", "Task always start from TASK START");
        this->setVisible(false);
        return;
    }
///// 1. sequence (task_nodes) from connection -> 2. according to the sequence (task_nodes) save json object from graph model -> 3. convert to script
// step 1
    scriptEdit->setPlainText("");
    QJsonDocument doc;
    QString strJson;

    NodeId lhs_id = startingPoint;
    NodeId rhs_id = startingPoint;
    task_nodes.clear();

    //initialize taskplanner
    auto conns = _graphmodel.allConnectionIds(startingPoint);
    auto jsonObject = _graphmodel.saveNode(startingPoint);

    QJsonObject interObj = jsonObject["internal-data"].toObject();
    _qnode->task_planner_->ref_unit_task_.vel_cs = interObj["vel_cs"].toDouble();
    _qnode->task_planner_->ref_unit_task_.vel_js = interObj["vel_js"].toDouble();
    _qnode->task_planner_->ref_unit_task_.acc_cs = interObj["acc_cs"].toDouble();
    _qnode->task_planner_->ref_unit_task_.acc_js = interObj["acc_js"].toDouble();

    // bool use_plc = interObj["use_plc"].toBool();

    // auto makeInitializePLCChuck = [&] (std::vector<UnitTask>& targetModuleTask) {
    //     // taskPushBackPLCModbusCmd(targetModuleTask, LATHE_CHUCK_CLOSE);
    //     // taskPushBack_delay_1ms(targetModuleTask, 20);
    //     _qnode->task_planner_->taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_OPEN);
    //     _qnode->task_planner_->taskPushBack_delay_1ms(targetModuleTask, 20);
    //     _qnode->task_planner_->taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
    //     _qnode->task_planner_->taskPushBack_delay_1ms(targetModuleTask, 20);
    //     _qnode->task_planner_->taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
    //     _qnode->task_planner_->taskPushBack_delay_1ms(targetModuleTask, 20);
    //     _qnode->task_planner_->taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
    //     _qnode->task_planner_->taskPushBack_delay_1ms(targetModuleTask, 20);
    //     _qnode->task_planner_->taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
    //     _qnode->task_planner_->taskPushBack_delay_1ms(targetModuleTask, 20);
    // };

    _qnode->task_planner_->bp_gui_node_task_.clear();

    //TODO::enable check for plc
    // if (use_plc) {
    //     makeInitializePLCChuck(_qnode->task_planner_->bp_gui_node_task_);
    // }

    if (conns.size() > 0) {
        while (true) {
            for (auto conn_id : conns) {
                if (conn_id.inNodeId != lhs_id) {
                    rhs_id = conn_id.inNodeId;
                    auto model = _graphmodel.delegateModel<KCRCommandModel>(rhs_id);
                    if(model) {
                        task_nodes.push_back(rhs_id);
                        model->saveGuiTask(*_qnode->task_planner_);
                        std::cout << "inserting tasks in sequence\n";
                    }
                }
            }
            lhs_id = rhs_id;
            conns = _graphmodel.allConnectionIds(lhs_id);
            if (conns.size() == 1) {
                break;
            }
        }
    }

/*  TO DO:
1. category top level ----> Module set 4 types
2. set item with icons (registered model)
*/
    QJsonArray nodesJsonArray;
    int i = 0;
    taskTreeView->clear();
    for (auto const nodeId : task_nodes) {
        auto jobject = _graphmodel.saveNode(nodeId);
        QJsonObject dataObj = jobject["internal-data"].toObject();
        doc.setObject(dataObj);
        strJson = doc.toJson(QJsonDocument::Compact);

        auto item = new QTreeWidgetItem(taskTreeView);
        item->setText(0, strJson);
        taskTreeView->insertTopLevelItem(i,item);

        scriptEdit->insertPlainText(strJson);
        nodesJsonArray.append(jobject);
        i++;
    }
    scriptJson["nodes"] = nodesJsonArray;

// 3
    // for(auto const nodeId : _graphmodel.allNodeIds()) {
    //     auto jsonObject = _graphmodel.saveNode(nodeId);

    //     //if (_graphmodel.nodeData(nodeId, NodeRole::Type) == "TASK START") {
    //        QJsonObject interObj = jsonObject["internal-data"].toObject();
    //        //interObj.remove("model-name");

    //        doc.setObject(interObj);
    //        strJson = doc.toJson(QJsonDocument::Compact);

    //        scriptEdit->insertPlainText(strJson);
    // }

    // after modify script

    // if (_graphmodel.nodeData(startingPoint, NodeRole::Type) == "TASK START") {
    //     auto connection_ids = _graphmodel.allConnectionIds(startingPoint);
    //     if (connection_ids.size() > 0) {
    //         setTaskPlannerList(startingPoint, connection_ids);
    //     }
    //     startFlag_ = true;
    // }

    std::cout << "task list generated counts ::" << _qnode->task_planner_->bp_gui_node_task_.size() << "\n" ;
}

void TaskManager::makeSimulTaskFromScript() {


}
// void TaskManager::InitializeTaskPlanner(NodeId starting_id,
//                                    std::unordered_set<ConnectionId> starting_conn_ids)
// {
//     NodeId lhs_id = starting_id;
//     NodeId rhs_id = starting_id;
//     std::unordered_set<ConnectionId> conns = starting_conn_ids;

//     auto jsonObject = _graphmodel.saveNode(starting_id);

//     QJsonObject interObj = jsonObject["internal-data"].toObject();
//     _qnode->task_planner_->ref_unit_task_.vel_cs = interObj["vel_cs"].toDouble();
//     _qnode->task_planner_->ref_unit_task_.vel_js = interObj["vel_js"].toDouble();
//     _qnode->task_planner_->ref_unit_task_.acc_cs = interObj["acc_cs"].toDouble();
//     _qnode->task_planner_->ref_unit_task_.acc_js = interObj["acc_js"].toDouble();

//     bool use_plc = interObj["use_plc"].toBool();

//     auto makeInitializePLCChuck = [&] (std::vector<UnitTask>& targetModuleTask) {
//         // taskPushBackPLCModbusCmd(targetModuleTask, LATHE_CHUCK_CLOSE);
//         // taskPushBack_delay_1ms(targetModuleTask, 20);
//         _qnode->task_planner_->taskPushBackPLCModbusCmd(targetModuleTask, MILLING_CHUCK_OPEN);
//         _qnode->task_planner_->taskPushBack_delay_1ms(targetModuleTask, 20);
//         _qnode->task_planner_->taskPushBackPLCModbusCmd(targetModuleTask, CNC_DOOR_OPEN);
//         _qnode->task_planner_->taskPushBack_delay_1ms(targetModuleTask, 20);
//         _qnode->task_planner_->taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_RED_OFF);
//         _qnode->task_planner_->taskPushBack_delay_1ms(targetModuleTask, 20);
//         _qnode->task_planner_->taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_YELLOW_ON);
//         _qnode->task_planner_->taskPushBack_delay_1ms(targetModuleTask, 20);
//         _qnode->task_planner_->taskPushBackPLCModbusCmd(targetModuleTask, CNC_LED_GREEN_OFF);
//         _qnode->task_planner_->taskPushBack_delay_1ms(targetModuleTask, 20);
//     };

//     _qnode->task_planner_->bp_gui_node_task_.clear();

//     //TODO::enable check for plc
//     // if (use_plc) {
//     //     makeInitializePLCChuck(_qnode->task_planner_->bp_gui_node_task_);
//     // }

//     // _qnode->init();
//     std::cout << "setTaskPlannerList\n\n";
//     int cnt = 0;
//     while (true) {
//         for (auto conn_id : conns) {
//             if (conn_id.inNodeId != lhs_id) {
//                 rhs_id = conn_id.inNodeId;
//                 auto model = _graphmodel.delegateModel<KCRCommandModel>(rhs_id);
//                 if(model) {
//                     model->saveGuiTask(_qnode->task_planner_);
//                     std::cout << "inserting tasks\n";
//                 }
//                 ++cnt;
//             }
//         }
//         lhs_id = rhs_id;
//         conns = _graphmodel.allConnectionIds(lhs_id);
//         if (conns.size() == 1) {
//             break;
//         }
//     }
//     //// TODO: Task Feasibility
//     //// TODO: Task Simulation and check simul

//     std::cout << "task list generated counts ::" << _qnode->task_planner_->bp_gui_node_task_.size() << "\n" ;
//     std::cout << "task list confirm counts::" << cnt << "\n" ;
// }


void TaskManager::OnProgressCompleted()
{

}

struct ComparePair {
  bool operator()(const std::pair<int, int>& p1, const std::pair<int, int>& p2) const {
    return p1.first == p2.first && p1.second == p2.second;
  }
};

void TaskManager::OnPlayerButtonClicked()
{

    GenTaskScriptFromGraph();

    // color to default
    for(auto nodeId : _graphmodel.allNodeIds()) {
        // qDebug()<< "color to default \n ";
        auto model = _graphmodel.delegateModel<KCRCommandModel>(nodeId);
        if(model) {
            model->setNodeStyle(QtNodes::NodeStyle(R"(
                    {
                        "NodeStyle": {
                        "NormalBoundaryColor": [255, 255, 255],
                        "SelectedBoundaryColor": [255, 165, 0],
                        "GradientColor0": "gray",
                        "GradientColor1": [80, 80, 80],
                        "GradientColor2": [64, 64, 64],
                        "GradientColor3": [58, 58, 58],
                        "ShadowColor": [20, 20, 20],
                        "FontColor" : "white",
                        "FontColorFaded" : "gray",
                        "ConnectionPointColor": [169, 169, 169],
                        "FilledConnectionPointColor": "cyan",
                        "ErrorColor": "red",
                        "WarningColor": [128, 128, 0],
                        "PenWidth": 1.0,
                        "HoveredPenWidth": 1.5,
                        "ConnectionPointDiameter": 8.0,
                        "Opacity": 0.8
                        }
                    }
                )"));
        }
    }

    // chasing tasks..
    #ifndef TASK_TEST
    if(_qnode->params_.status.is_enable) {
    #endif

    setBeforeTaskSignal();
    _qnode->current_task_list_ = _qnode->task_planner_->bp_gui_node_task_;
    _qnode->beforeTaskStart();   //beforeBinPickingTaskStart(true);
    _qnode->task_cycle_ = 0;

    //// GUI Task Mode
    _qnode->is_developer_mode_ = false;

    zoomAction();

    #ifndef TASK_TEST
    } else {
       QMessageBox::warning(this, "warning", "Not finished - ROBOT ENABLE");
    }
    #endif
}

void TaskManager::OnPauseButtonClicked() {
    QIcon icon1(":pause.png");
    QIcon icon2(":resume.png");

    bool enabled = _qnode->params_.status.is_enable;
    if(!isTaskPaused && enabled) {
        _qnode->pauseTask();
        isTaskPaused = true;
        pauseButton->setIcon(icon2);
        //QMessageBox::warning(nullptr, "Warning", "Task paused");

    } else if (isTaskPaused && enabled){
        _qnode->resumeTask();
        isTaskPaused = false;
        pauseButton->setIcon(icon1);
        //QMessageBox::warning(nullptr, "Warning", "Task resumed");
    } else {
        std::cout << "nothing sent\n";
    }
}

void TaskManager::OnTaskStopButtonClicked()
{
    isRobotStop = _qnode->stopRobot();
    _qnode->is_task_mode_ = false;

    if (isRobotStop){
        QMessageBox::warning(nullptr, "Warning", "Robot stopped");
    }

    //if(!isStopped) {
    //    workerThread.terminate();
    //    if(!workerThread.isRunning()) {
    //        playButton->setEnabled(true);
    //        QMessageBox::warning(nullptr, "Warning", "Thread aborted");
    //    }
    //}
}
void TaskManager::OnEditButtonClicked()
{
    if(scriptEdit->isReadOnly()) {
        scriptEdit->setReadOnly(false);
    } else {
        scriptEdit->setReadOnly(true);
    }
    //editButton->setIcon();
}
void TaskManager::OnRefreshButtonClicked()
{
    GenTaskScriptFromGraph();
}


void TaskManager::OnEmergencyState()
{

}

void TaskManager::addIconsFromGraph()
{

    auto list = _graphmodel.dataModelRegistry();

    //qDebug() << list;/*
    //setting graph sequence
    //std::unordered_set<NodeId> a = DataFlowGraphModel::allNodeIds();

    //DataFlowGraphModel::getGraphModel();
}

void TaskManager::zoomAction() {
    auto nodeId_task = task_nodes[task_count];
    auto jobject = _graphmodel.saveNode(nodeId_task);
    QJsonObject dataObj = jobject["internal-data"].toObject();
    QJsonDocument doc;
    doc.setObject(dataObj);
    auto json = doc.toJson(QJsonDocument::Compact);
    receiveEdit->insertPlainText(json);
    if(zoom_checkbox->isChecked()) {
        Q_EMIT _graphmodel.zoomInNode(nodeId_task);
    }
    //to do painter? or brush to RED

        auto model = _graphmodel.delegateModel<KCRCommandModel>(nodeId_task);
        if(model) {
        model->setNodeStyle(QtNodes::NodeStyle(R"(
                {
                    "NodeStyle": {
                    "NormalBoundaryColor": "darkgray",
                    "SelectedBoundaryColor": "deepskyblue",
                    "GradientColor0": "green",
                    "GradientColor1": "green",
                    "GradientColor2": "green",
                    "GradientColor3": "green",
                    "ShadowColor": [10, 10, 10],
                    "FontColor": [10, 10, 10],
                    "FontColorFaded": [10, 10, 10],
                    "ConnectionPointColor": "white",
                    "PenWidth": 2.0,
                    "HoveredPenWidth": 2.5,
                    "ConnectionPointDiameter": 10.0,
                    "Opacity": 0.0
                    }
                }
            )")
        );



        //default node style
        //             "NormalBoundaryColor": [255, 255, 255],
        // "SelectedBoundaryColor": [255, 165, 0],
        // "GradientColor0": "gray",
        // "GradientColor1": [80, 80, 80],
        // "GradientColor2": [64, 64, 64],
        // "GradientColor3": [58, 58, 58],
        // "ShadowColor": [20, 20, 20],
        // "FontColor" : "white",
        // "FontColorFaded" : "gray",
        // "ConnectionPointColor": [169, 169, 169],
        // "FilledConnectionPointColor": "cyan",
        // "ErrorColor": "red",
        // "WarningColor": [128, 128, 0],

        // "PenWidth": 1.0,
        // "HoveredPenWidth": 1.5,

        // "ConnectionPointDiameter": 8.0,

        // "Opacity": 0.8
    }

}

void TaskManager::setBeforeTaskSignal()
{
    std::vector<std::vector<bool>> taskList;
    taskList.resize(5);
    tag_counts.clear();
    tag_counts.resize(5);

    for (auto const nodeId : task_nodes) {
        QJsonObject data = _graphmodel.nodeData(nodeId, NodeRole::InternalData).toJsonObject();
        int task_index = data["internal-data"].toObject()["task_index"].toInt();

        taskList[task_index].push_back(false);
    }

    TaskInfo taskInfo;
    taskInfo.taskInfo = taskList;

    taskInfo.compelete_index = 0;
    taskInfo.compelete_tag = 0;

    _qnode->setTaskInfoForMonitorAtStart(taskInfo);
    task_count = 0;
    Q_EMIT _qnode->taskStartedSignal();
}
// void TaskManager::setTaskMode(bool is_task) {
// }
