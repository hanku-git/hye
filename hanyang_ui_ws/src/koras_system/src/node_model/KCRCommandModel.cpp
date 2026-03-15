#include "KCRCommandModel.hpp"
#include "doubleData.hpp"

    std::list<std::pair<int, QString>> KCRCommandModel::index_color_list =
    {
        {1,
            R"(
                {
                    "NodeStyle": {
                    "NormalBoundaryColor": "darkgray",
                    "SelectedBoundaryColor": "deepskyblue",
                    "GradientColor0": "mintcream",
                    "GradientColor1": "mintcream",
                    "GradientColor2": "mintcream",
                    "GradientColor3": "mintcream",
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
            )"
        },
        {2,             R"(
                {
                    "NodeStyle": {
                    "NormalBoundaryColor": "darkgray",
                    "SelectedBoundaryColor": "deepskyblue",
                    "GradientColor0": "gray",
                    "GradientColor1": "gray",
                    "GradientColor2": "gray",
                    "GradientColor3": "gray",
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
            )"},
        {3,             R"(
                {
                    "NodeStyle": {
                    "NormalBoundaryColor": "darkgray",
                    "SelectedBoundaryColor": "deepskyblue",
                    "GradientColor0": "orange",
                    "GradientColor1": "orange",
                    "GradientColor2": "orange",
                    "GradientColor3": "orange",
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
            )"},
        {4,             R"(
                {
                    "NodeStyle": {
                    "NormalBoundaryColor": "darkgray",
                    "SelectedBoundaryColor": "deepskyblue",
                    "GradientColor0": "hotpink",
                    "GradientColor1": "hotpink",
                    "GradientColor2": "hotpink",
                    "GradientColor3": "hotpink",
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
            )"}//,
        // {5,             R"(
        //         {
        //             "NodeStyle": {
        //             "NormalBoundaryColor": "darkgray",
        //             "SelectedBoundaryColor": "deepskyblue",
        //             "GradientColor0": "seagreen",
        //             "GradientColor1": "seagreen",
        //             "GradientColor2": "seagreen",
        //             "GradientColor3": "seagreen",
        //             "ShadowColor": [10, 10, 10],
        //             "FontColor": [10, 10, 10],
        //             "FontColorFaded": [10, 10, 10],
        //             "ConnectionPointColor": "white",
        //             "PenWidth": 2.0,
        //             "HoveredPenWidth": 2.5,
        //             "ConnectionPointDiameter": 10.0,
        //             "Opacity": 0.0
        //             }
        //         }
        //     )"},
        // {6,             R"(
        //         {
        //             "NodeStyle": {
        //             "NormalBoundaryColor": "darkgray",
        //             "SelectedBoundaryColor": "deepskyblue",
        //             "GradientColor0": "gold",
        //             "GradientColor1": "gold",
        //             "GradientColor2": "gold",
        //             "GradientColor3": "gold",
        //             "ShadowColor": [10, 10, 10],
        //             "FontColor": [10, 10, 10],
        //             "FontColorFaded": [10, 10, 10],
        //             "ConnectionPointColor": "white",
        //             "PenWidth": 2.0,
        //             "HoveredPenWidth": 2.5,
        //             "ConnectionPointDiameter": 10.0,
        //             "Opacity": 0.0
        //             }
        //         }
        //     )"}
    };
KCRCommandModel::KCRCommandModel(): _widget(nullptr) {
    _task_index = 0;
    lineEditTaskIndex = new QLineEdit("0");
    // _task_index = 1;
    // for (const auto& pair : index_color_list) {
    //     // Check if the int value is 1
    //     if (pair.first == _task_index) {
    //         // Output the corresponding QString value
    //         setNodeStyle(NodeStyle(pair.second));
    //         break; // Optional: break out of the loop if you only need the first occurrence
    //     }
    // }
}
unsigned int KCRCommandModel::nPorts(PortType portType) const
{
    unsigned int result = 1;
    switch (portType) {
    case PortType::In:
        result = 1;
        break;

    case PortType::Out:
        result = 1;
    default:
        break;
    }
    return result;
}

//보통은 포트데이터를 위한 타입이지만, 노드 내부 커맨드 타입으로 연결하도록 함
//struct 정의된 것 주입.. 개수 정해서 넣기
//NodeDataType KCRCommandModel::dataType(PortType, PortIndex) const
//{
//    return NodeDataType{"RobotCommand", "CS Move Parameter"};
//}

std::shared_ptr<NodeData> KCRCommandModel::outData(PortIndex)
{
    return std::static_pointer_cast<NodeData>(_result);
}

void KCRCommandModel::setInData(std::shared_ptr<NodeData> data, PortIndex portIndex) {}

QJsonObject KCRCommandModel::save() const {
    QJsonObject modelJson = NodeDelegateModel::save();
    modelJson["task_index"] = _task_index;
    return modelJson;
}

void KCRCommandModel::load(QJsonObject const &p) {
    QJsonValue jindex = p["task_index"].toInt();
    lineEditTaskIndex->setText(QString::number(jindex.toInt()));
    //TO DO ::: get color string from pair list with task_index....
    for (const auto& pair : index_color_list) {
        // Check if the int value is 1
        if (pair.first == jindex.toInt()) {
            // Output the corresponding QString value
            setNodeStyle(NodeStyle(pair.second));
            break; // Optional: break out of the loop if you only need the first occurrence
        }
    }
}

void KCRCommandModel::setWidgetExpand(bool flag) {
    _widget->setVisible(flag);
}

void KCRCommandModel::updateWidgetFromJson(QJsonArray& jarr, std::vector<double>& vec, QVector<QLineEdit*>& lineEdits) {
    if (jarr.size() != vec.size()) {
        // throw std::runtime_error("Size mismatch between QJsonArray and std::vector");
        return;
    } else {
        for (int i = 0; i < jarr.size(); ++i) {
            const QJsonValue& jsonValue = jarr[i];
            if (jsonValue.isDouble()) {
                vec[i] = jsonValue.toDouble();
            }
            else {
                return;
                //throw std::runtime_error("Invalid element type in QJsonArray");
            }
        }
        for (int i = 0; i < vec.size(); ++i) {
            lineEdits[i]->setText(QString::number(vec[i]));
        }
    }
}

void KCRCommandModel::setNodeStyle(NodeStyle const &style) {
    NodeDelegateModel::setNodeStyle(style);
}

void KCRCommandModel::setTaskIndex(const int& task_index) {
    _task_index = task_index;
}

JsTargetModel::JsTargetModel() : KCRCommandModel() {
    target_q.resize(6);
    acc = 0.0;
    vel = 0.0;
    //setNodeStyle(nodeStyle());
}

QString JsTargetModel::caption() const { return QStringLiteral("JS TARGET TASK"); }

QString JsTargetModel::name() const { return QStringLiteral("JS TARGET TASK"); }

NodeDataType JsTargetModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "JS TARGET TASK"};
}

QJsonObject JsTargetModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    printValueType(target_q, modelJson, "target_q");
    modelJson["acc"] = acc;
    modelJson["vel"] = vel;
    modelJson["is_relative"] = is_relative_checkbox->isChecked();
    return modelJson;
}
void JsTargetModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    // QJsonValue jindex = p["task_index"].toInt();
    // lineEditTaskIndex->setText(QString::number(jindex.toInt()));

    QJsonArray jtarget_q = p["target_q"].toArray();
    updateWidgetFromJson(jtarget_q, target_q, lineEdits1);
    QJsonValue jacc = p["acc"].toDouble();
    QJsonValue jvel = p["vel"].toDouble();
    lineEditsAcc->setText(QString::number(jacc.toDouble()));
    lineEditsVel->setText(QString::number(jvel.toDouble()));
    bool flag = p["is_relative"].toBool();
    if(flag) {
        is_relative_checkbox->setCheckState(Qt::CheckState::Checked);
    }
}

QWidget *JsTargetModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        const int numLineEdits = 6;

        for (int i = 0; i < numLineEdits; ++i)
            {
                QLineEdit *lineEdit = new QLineEdit("0.0");
                lineEdits1.append(lineEdit);

                QObject::connect(lineEdit,
                                    &QLineEdit::textChanged,
                                    this,
                                    [i, this](const QString &text) {
                    set_target_q(text.toDouble(), i);
                });
            }
        QGroupBox *groupBox1 = new QGroupBox("target_q");
        QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);

        for (auto edit : lineEdits1) {
            layout1->addWidget(edit);
        }
        lineEditsVel = new QLineEdit("0.0");
        lineEditsAcc = new QLineEdit("0.0");
        QObject::connect(lineEditsVel, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_vel(text.toDouble());
        });
        QObject::connect(lineEditsAcc, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_acc(text.toDouble());
        });
        QLabel *l_vel = new QLabel("&Velocity(degree/s):");
        QLabel *l_acc = new QLabel("&Acceleration(degree/s2):");
        l_vel->setBuddy(lineEditsVel);
        l_acc->setBuddy(lineEditsAcc);

        is_relative_checkbox = new QCheckBox();
        is_relative_checkbox->setText("checked : relative JS");
        layout1->addWidget(is_relative_checkbox);
        layout1->setContentsMargins(0, 0, 0, 0);
        mainLayout->addWidget(groupBox1);
        mainLayout->addWidget(l_vel);
        mainLayout->addWidget(lineEditsVel);
        mainLayout->addWidget(l_acc);
        mainLayout->addWidget(lineEditsAcc);
        QPushButton* updateBtn = new QPushButton("Get current value");

        QObject::connect(updateBtn, &QPushButton::clicked, this, [this]() {
            QNode* ptr = &QNode::getInstance();
            JsDouble arr = ptr->params_.meas.q;
            target_q.resize(6);
            for (size_t i = 0; i < 6; i++)
            {
                lineEdits1[i]->setText(QString::number(arr[i]));
            }
        });
        mainLayout->addWidget(updateBtn);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });


        _widget->setLayout(mainLayout);
    }
    return _widget;
}

void JsTargetModel::saveGuiTask(TaskPlanner& planner) {
    JsDouble jsArr;
    vec2Arr(target_q, jsArr);
    planner.taskPushBack_jsMove(planner.bp_gui_node_task_, jsArr, is_relative_checkbox->isChecked());
}

// void JsTargetModel::setNodeStyle(NodeStyle const &style)
// {
//     KCRCommandModel::setNodeStyle(NodeStyle(R"(
//         {
//             "NodeStyle": {
//             "NormalBoundaryColor": "darkgray",
//             "SelectedBoundaryColor": "deepskyblue",
//             "GradientColor0": "mintcream",
//             "GradientColor1": "mintcream",
//             "GradientColor2": "mintcream",
//             "GradientColor3": "mintcream",
//             "ShadowColor": [10, 10, 10],
//             "FontColor": [10, 10, 10],
//             "FontColorFaded": [10, 10, 10],
//             "ConnectionPointColor": "white",
//             "PenWidth": 2.0,
//             "HoveredPenWidth": 2.5,
//             "ConnectionPointDiameter": 10.0,
//             "Opacity": 1.0
//             }
//         }
//     )"));
// }

CsTargetModel::CsTargetModel() : KCRCommandModel() {
    target_x.resize(6);
    acc = 0.0;
    vel = 0.0;
}

QString CsTargetModel::caption() const { return QStringLiteral("CS TARGET TASK"); }

QString CsTargetModel::name() const { return QStringLiteral("CS TARGET TASK"); }

NodeDataType CsTargetModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "CS TARGET TASK"};
}

QJsonObject CsTargetModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    printValueType(target_x, modelJson, "target_x");
    modelJson["vel"] = vel;
    modelJson["acc"] = acc;
    modelJson["is_relative"] = is_relative_checkbox->isChecked();
    return modelJson;
}
void CsTargetModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonArray jtarget_x = p["target_x"].toArray();
    updateWidgetFromJson(jtarget_x, target_x, lineEdits1);
    QJsonValue jvel = p["vel"].toDouble();
    QJsonValue jacc = p["acc"].toDouble();
    lineEditsVel->setText(QString::number(jvel.toDouble()));
    lineEditsAcc->setText(QString::number(jacc.toDouble()));
    bool flag = p["is_relative"].toBool();
    if(flag) {
        is_relative_checkbox->setCheckState(Qt::CheckState::Checked);
    }
}

QWidget* CsTargetModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        const int numLineEdits = 6;

        for (int i = 0; i < numLineEdits; ++i)
            {
                QLineEdit *lineEdit = new QLineEdit("0.0");
                lineEdits1.append(lineEdit);
                QObject::connect(lineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
                    set_target_x(text.toDouble(), i);
                });
            }
        QGroupBox *groupBox1 = new QGroupBox("target_x");
        QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);
        for (auto edit : lineEdits1) {
            layout1->addWidget(edit);
        }
        lineEditsVel = new QLineEdit("0.0");
        lineEditsAcc = new QLineEdit("0.0");
        is_relative_checkbox = new QCheckBox();
        is_relative_checkbox->setText("checked: relative CS move");
        QObject::connect(lineEditsVel, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_vel(text.toDouble());
        });
        QObject::connect(lineEditsAcc, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_acc(text.toDouble());
        });
        QLabel *l_vel = new QLabel("&Velocity(m/s):");
        QLabel *l_acc = new QLabel("&Acceleration(m/s2):");
        l_acc->setBuddy(lineEditsAcc);
        l_vel->setBuddy(lineEditsVel);

        layout1->addWidget(l_vel);
        layout1->addWidget(lineEditsVel);
        layout1->addWidget(l_acc);
        layout1->addWidget(lineEditsAcc);
        layout1->addWidget(is_relative_checkbox);

        QPushButton* updateBtn = new QPushButton("Get current value");

        QObject::connect(updateBtn, &QPushButton::clicked, this, [this]() {
            QNode* ptr = &QNode::getInstance();
            CsDouble arr = ptr->params_.meas.x;
            target_x.resize(6);
            for (size_t i = 0; i < 6; i++)
            {
                lineEdits1[i]->setText(QString::number(arr[i]));
            }

        });
        layout1->addWidget(updateBtn);

        // save_btn = new QPushButton("Save Position");
        // load_btn = new QPushButton("Load Position");
        // QObject::connect(diag_btn, &QPushButton::clicked, this, [this]() {

        //     QDialog diag = new QDialog();
        //     QVBoxLayout *layout_diag = new QVBoxLayout();
        //     diag->setLayout(layout_diag);
        //     QComboBox _comboBoxDemoType = new QComboBox();
        //     QComboBox _comboBoxMotionTag = new QComboBox();;
        //     layout_diag->addWidget(_comboBoxDemoType);
        //     layout_diag->addWidget(_comboBoxMotionTag);
        //     diag->show();

        // });
        // layout1->addWidget(diag_btn);
        layout1->setContentsMargins(0, 0, 0, 0);

        mainLayout->addWidget(groupBox1);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });

        _widget->setLayout(mainLayout);

    }
    return _widget;
}
void CsTargetModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_csMove2(planner.bp_gui_node_task_, target_x, acc, vel, is_relative_checkbox->isChecked());
}

// void CsTargetModel::setNodeStyle(NodeStyle const &style)
// {
//     KCRCommandModel::setNodeStyle(NodeStyle(R"(
//         {
//             "NodeStyle": {
//             "NormalBoundaryColor": "darkgray",
//             "SelectedBoundaryColor": "deepskyblue",
//             "GradientColor0": "mintcream",
//             "GradientColor1": "mintcream",
//             "GradientColor2": "mintcream",
//             "GradientColor3": "mintcream",
//             "ShadowColor": [10, 10, 10],
//             "FontColor": [10, 10, 10],
//             "FontColorFaded": [10, 10, 10],
//             "ConnectionPointColor": "white",
//             "PenWidth": 2.0,
//             "HoveredPenWidth": 2.5,
//             "ConnectionPointDiameter": 10.0,
//             "Opacity": 1.0
//             }
//         }
//     )"));
// }

CsTargetTaskRecogModel::CsTargetTaskRecogModel() : KCRCommandModel() {
    target_x.resize(6);
    acc = 0.0;
    vel = 0.0;

}

QString CsTargetTaskRecogModel::caption() const { return QStringLiteral("CS TARGET RECOG TASK\n(TYPE , MOTION, RELATIVE)"); }

QString CsTargetTaskRecogModel::name() const { return QStringLiteral("CS TARGET RECOG TASK"); }

NodeDataType CsTargetTaskRecogModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "CS TARGET RECOG TASK"};
}

QJsonObject CsTargetTaskRecogModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    printValueType(target_x, modelJson, "target_x");
    modelJson["type"] = static_cast<int>(demo_type_map->value(_comboBoxDemoType->currentText()));
    modelJson["motion"] = static_cast<int>(motion_tag_map->value(_comboBoxMotionTag->currentText()));
    modelJson["is_relative"] = is_relative_checkbox->isChecked();
    modelJson["vel"] = vel;
    modelJson["acc"] = acc;
    return modelJson;
}
void CsTargetTaskRecogModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonArray jtarget_x = p["target_x"].toArray();
    updateWidgetFromJson(jtarget_x, target_x, lineEdits1);
    _comboBoxDemoType->setCurrentText(demo_type_map->key(static_cast<BPDemoType>(p["type"].toInt())));
    _comboBoxMotionTag->setCurrentText(motion_tag_map->key(static_cast<MotionTag>(p["motion"].toInt())));
    bool flag = p["is_relative"].toBool();
    if(flag) {
        is_relative_checkbox->setCheckState(Qt::CheckState::Checked);
    }
    QJsonValue jvel = p["vel"].toDouble();
    QJsonValue jacc = p["acc"].toDouble();
    lineEditsVel->setText(QString::number(jvel.toDouble()));
    lineEditsAcc->setText(QString::number(jacc.toDouble()));
}

QWidget *CsTargetTaskRecogModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        const int numLineEdits = 6;

        for (int i = 0; i < numLineEdits; ++i)
            {
                QLineEdit *lineEdit = new QLineEdit("0.0");
                lineEdits1.append(lineEdit);
                QObject::connect(lineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
                    set_target_x(text.toDouble(), i);
                });
            }
        QGroupBox *groupBox1 = new QGroupBox("target_x");
        QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);
        for (auto edit : lineEdits1) {
            layout1->addWidget(edit);
        }
        is_relative_checkbox = new QCheckBox();
        is_relative_checkbox->setText("checked : relative CS move");

        layout1->addWidget(is_relative_checkbox);

        // QPushButton* updateBtn = new QPushButton("Get current value");

        // QObject::connect(updateBtn, &QPushButton::clicked, this, [this]() {
        //     QNode* ptr = &QNode::getInstance();
        //     CsDouble arr = ptr->params_.meas.x;
        //     target_x.resize(6);
        //     for (size_t i = 0; i < 6; i++)
        //     {
        //         lineEdits1[i]->setText(QString::number(arr[i]));
        //     }

        // });
        // layout1->addWidget(updateBtn);

        demo_type_map = new QMap<QString, BPDemoType>();
        demo_type_map->insert("CYLINDER", BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER);
        demo_type_map->insert("SQUARE_PEG", BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG);
        demo_type_map->insert("WELDING_T", BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP);
        demo_type_map->insert("WELDING_ELBOW", BPDemoType::DEMO_BP_PICK_WELDING_ELBOW_JOINT);
        demo_type_map->insert("BOLT_BUSH", BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH);
        demo_type_map->insert("GEAR", BPDemoType::DEMO_BP_PICK_GEAR);
        _comboBoxDemoType = new QComboBox();
        _comboBoxDemoType->addItems(QStringList(demo_type_map->keys()));

        motion_tag_map = new QMap<QString, MotionTag>();
        motion_tag_map->insert("HOME_TO_CNC_INDOOR", MotionTag::MOTION_TAG_BLENDING_HOME_TO_CNC_INDOOR);
        motion_tag_map->insert("HOME_TO_REGRASP_ZIG", MotionTag::MOTION_TAG_BLENDING_HOME_TO_REGRASP_ZIG);
        motion_tag_map->insert("REGRASP_ZIG_TO_CNC_INDOOR", MotionTag::MOTION_TAG_BLENDING_REGRASP_ZIG_TO_CNC_INDOOR);
        motion_tag_map->insert("CNC_INDOOR_TO_DETACH", MotionTag::MOTION_TAG_BLENDING_CNC_INDOOR_TO_DETACH);
        motion_tag_map->insert("DETACH_TO_HOME_1", MotionTag::MOTION_TAG_BLENDING_DETACH_TO_HOME_1);
        motion_tag_map->insert("HOME_TO_GRASP_APPROACH_POSE", MotionTag::MOTION_TAG_BLENDING_HOME_TO_GRASP_APPROACH_POSE);
        motion_tag_map->insert("GRASP_APPROACH_POSE_TO_REGRASPING_ZIG", MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_REGRASPING_ZIG);
        motion_tag_map->insert("GRASP_APPROACH_POSE_TO_CNC_INNER", MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_CNC_INNER);
        motion_tag_map->insert("GRASP_APPROACH_POSE_TO_DETACH", MotionTag::MOTION_TAG_BLENDING_GRASP_APPROACH_POSE_TO_DETACH);
        motion_tag_map->insert("CSMOVE_REGRASP_PICK", MotionTag::MOTION_TAG_JSON_CSMOVE_REGRASP_PICK);
        motion_tag_map->insert("CSMOVE_REGRASP_PLACE)", MotionTag::MOTION_TAG_JSON_CSMOVE_REGRASP_PLACE);
        motion_tag_map->insert("TAG_CSMOVE_STACKING", MotionTag::MOTION_TAG_JSON_CSMOVE_STACKING);

        _comboBoxMotionTag = new QComboBox();
        _comboBoxMotionTag->addItems(QStringList(motion_tag_map->keys()));

        layout1->addWidget(_comboBoxDemoType);
        layout1->addWidget(_comboBoxMotionTag);
        layout1->setContentsMargins(0, 0, 0, 0);

        lineEditsVel = new QLineEdit("0.0");
        lineEditsAcc = new QLineEdit("0.0");

        QObject::connect(lineEditsVel, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_vel(text.toDouble());
        });
        QObject::connect(lineEditsAcc, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_acc(text.toDouble());
        });
        QLabel *l_vel = new QLabel("&Velocity(m/s):");
        QLabel *l_acc = new QLabel("&Acceleration(m/s2):");
        l_acc->setBuddy(lineEditsAcc);
        l_vel->setBuddy(lineEditsVel);

        layout1->addWidget(l_acc);
        layout1->addWidget(lineEditsVel);
        layout1->addWidget(l_vel);
        layout1->addWidget(lineEditsAcc);


        mainLayout->addWidget(groupBox1);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });



        _widget->setLayout(mainLayout);

    }
    return _widget;
}
void CsTargetTaskRecogModel::saveGuiTask(TaskPlanner& planner) {
    CsDouble csArr;
    vec2Arr(target_x, csArr);

    planner.taskPushBack_taskRecog_csMoveMotionTag(planner.bp_gui_node_task_, csArr, acc, vel,
    static_cast<int>(demo_type_map->value(_comboBoxDemoType->currentText())),
    static_cast<int>(motion_tag_map->value(_comboBoxMotionTag->currentText())));
    //is_relative_checkbox->isChecked());
}


CsTargetToolFrameModel::CsTargetToolFrameModel() : KCRCommandModel() {
    target_x.resize(6);
    acc = 0.0;
    vel = 0.0;
}

QString CsTargetToolFrameModel::caption() const{ return QStringLiteral("CS TARGET TOOL FRAME TASK"); }

QString CsTargetToolFrameModel::name() const { return QStringLiteral("CS TARGET TOOL FRAME TASK"); }

NodeDataType CsTargetToolFrameModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "CS TARGET TOOL FRAME TASK"};
}

QJsonObject CsTargetToolFrameModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    printValueType(target_x, modelJson, "target_x");
    modelJson["vel"] = vel;
    modelJson["acc"] = acc;
    return modelJson;
}
void CsTargetToolFrameModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonArray jtarget_x = p["target_x"].toArray();
    updateWidgetFromJson(jtarget_x, target_x, lineEdits1);
    QJsonValue jvel = p["vel"].toDouble();
    QJsonValue jacc = p["acc"].toDouble();
    lineEditsVel->setText(QString::number(jvel.toDouble()));
    lineEditsAcc->setText(QString::number(jacc.toDouble()));
}

QWidget *CsTargetToolFrameModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        const int numLineEdits = 6;

        for (int i = 0; i < numLineEdits; ++i)
            {
                QLineEdit *lineEdit = new QLineEdit("0.0");
                lineEdits1.append(lineEdit);
                QObject::connect(lineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
                    set_target_x(text.toDouble(), i);
                });
            }
        QGroupBox *groupBox1 = new QGroupBox("target_x");
        QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);
        for (auto edit : lineEdits1) {
            layout1->addWidget(edit);
        }
        lineEditsVel = new QLineEdit("0.0");
        lineEditsAcc = new QLineEdit("0.0");

        QObject::connect(lineEditsVel, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_vel(text.toDouble());
        });
        QObject::connect(lineEditsAcc, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_acc(text.toDouble());
        });
        QLabel *l_vel = new QLabel("&Velocity(m/s):");
        QLabel *l_acc = new QLabel("&Acceleration(m/s2):");
        l_acc->setBuddy(lineEditsAcc);
        l_vel->setBuddy(lineEditsVel);

        layout1->addWidget(l_acc);
        layout1->addWidget(lineEditsVel);
        layout1->addWidget(l_vel);
        layout1->addWidget(lineEditsAcc);

        QPushButton* updateBtn = new QPushButton("Get current value");

        QObject::connect(updateBtn, &QPushButton::clicked, this, [this]() {
            QNode* ptr = &QNode::getInstance();
            CsDouble arr = ptr->params_.meas.x;
            target_x.resize(6);
            for (size_t i = 0; i < 6; i++)
            {
                lineEdits1[i]->setText(QString::number(arr[i]));
            }

        });
        layout1->addWidget(updateBtn);

        layout1->setContentsMargins(0, 0, 0, 0);
        mainLayout->addWidget(groupBox1);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);

    }
    return _widget;
}
void CsTargetToolFrameModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_csMoveToolFrame(planner.bp_gui_node_task_, target_x, acc, vel, true);
}

CsTargetRedundantModel::CsTargetRedundantModel() : KCRCommandModel() {
    target_x.resize(6);
    acc = 0.0;
    vel = 0.0;
    q_redundant = 0.0;
}

QString CsTargetRedundantModel::caption() const { return QStringLiteral("CS TARGET REDUNDANT TASK"); }

QString CsTargetRedundantModel::name() const { return QStringLiteral("CS TARGET REDUNDANT TASK"); }

NodeDataType CsTargetRedundantModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "CS TARGET REDUNDANT TASK"};
}

QJsonObject CsTargetRedundantModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    printValueType(target_x, modelJson, "target_x");
    modelJson["vel"] = vel;
    modelJson["acc"] = acc;
    modelJson["q_redundant"] = q_redundant;
    return modelJson;
}
void CsTargetRedundantModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonArray jtarget_x = p["target_x"].toArray();
    updateWidgetFromJson(jtarget_x, target_x, lineEdits1);

    QJsonValue jq_redundant = p["q_redundant"].toDouble();
    QJsonValue jvel = p["vel"].toDouble();
    QJsonValue jacc = p["acc"].toDouble();
    lineEditRedundant->setText(QString::number(jq_redundant.toDouble()));
    lineEditsVel->setText(QString::number(jvel.toDouble()));
    lineEditsAcc->setText(QString::number(jacc.toDouble()));
}

QWidget *CsTargetRedundantModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        const int numLineEdits = 6;

        for (int i = 0; i < numLineEdits; ++i)
            {
                QLineEdit *lineEdit = new QLineEdit("0.0");
                lineEdits1.append(lineEdit);
                QObject::connect(lineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
                    set_target_x(text.toDouble(), i);
                });
            }
        QGroupBox *groupBox1 = new QGroupBox("target_x");
        QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);
        for (auto edit : lineEdits1) {
            layout1->addWidget(edit);
        }
        lineEditRedundant = new QLineEdit("0.0");
        lineEditsVel = new QLineEdit("0.0");
        lineEditsAcc = new QLineEdit("0.0");

        QObject::connect(lineEditRedundant, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_q_redundant(text.toDouble());
        });
        QObject::connect(lineEditsVel, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_vel(text.toDouble());
        });
        QObject::connect(lineEditsAcc, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_acc(text.toDouble());
        });
        QLabel *l_red = new QLabel("&redundant q(degree):");
        QLabel *l_vel = new QLabel("&Velocity(mm/s):");
        QLabel *l_acc = new QLabel("&Acceleration(mm/s2):");
        l_red->setBuddy(lineEditRedundant);
        l_acc->setBuddy(lineEditsAcc);
        l_vel->setBuddy(lineEditsVel);

        layout1->addWidget(l_red);
        layout1->addWidget(lineEditRedundant);
        layout1->addWidget(l_acc);
        layout1->addWidget(lineEditsVel);
        layout1->addWidget(l_vel);
        layout1->addWidget(lineEditsAcc);

        QPushButton* updateBtn = new QPushButton("Get current value");

        QObject::connect(updateBtn, &QPushButton::clicked, this, [this]() {
            QNode* ptr = &QNode::getInstance();
            CsDouble arr = ptr->params_.meas.x;
            target_x.resize(6);
            for (size_t i = 0; i < 6; i++)
            {
                lineEdits1[i]->setText(QString::number(arr[i]));
            }

        });
        layout1->addWidget(updateBtn);

        layout1->setContentsMargins(0, 0, 0, 0);
        mainLayout->addWidget(groupBox1);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });

        _widget->setLayout(mainLayout);

    }
    return _widget;
}
void CsTargetRedundantModel::saveGuiTask(TaskPlanner& planner) {
    CsDouble csArr;
    vec2Arr(target_x, csArr);
    planner.taskPushBack_csMove_redundant(planner.bp_gui_node_task_, csArr, q_redundant, false);
}

MoveBlendingModel::MoveBlendingModel() : KCRCommandModel() {}

QString MoveBlendingModel::caption() const{ return QStringLiteral("MOVE BLENDING TASK"); }

QString MoveBlendingModel::name() const { return QStringLiteral("MOVE BLENDING TASK"); }

NodeDataType MoveBlendingModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "MOVE BLENDING TASK"};
}

QJsonObject MoveBlendingModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();

    auto model = _table_widget->model();
    //tablewidget model to json object
    QJsonArray xsJsonArray;
    QJsonArray xdsJsonArray;
    QJsonArray xddsJsonArray;
    QJsonArray radiusesJsonArray;
    QJsonArray wp_xdsJsonArray;
    QJsonArray qs_redundantJsonArray;
    for (int i = 0; i < model->rowCount(); i++) {
        QJsonArray csJsonArray;
        for (int j = 0; j < model->columnCount(); j++) {
            auto value = model->index(i, j).data().toDouble();
            switch (j) {
            case 6:
                xdsJsonArray.append(value);
                break;
            case 7:
                xddsJsonArray.append(value);
                break;
            case 8:
                radiusesJsonArray.append(value);
                break;
            case 9:
                wp_xdsJsonArray.append(value);
                break;
            case 10:
                qs_redundantJsonArray.append(value);
                break;
            default:
                csJsonArray.append(value);
                break;
            }
        }
        xsJsonArray.append(csJsonArray);
    }
    modelJson["xs"] = xsJsonArray;
    modelJson["xds"] = xdsJsonArray;
    modelJson["xdds"] = xddsJsonArray;
    modelJson["radiuses"] = radiusesJsonArray;
    modelJson["waypoint_xds"] = wp_xdsJsonArray;
    modelJson["qs_redundant"] = qs_redundantJsonArray;

    modelJson["waypoint_num"] = _traj.waypoint_num;
    modelJson["is_radius_percent"] = is_radius_percent_checkbox->isChecked();

    return modelJson;
}
void MoveBlendingModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    auto model = _table_widget->model();

    _traj.waypoint_num = p["waypoint_num"].toInt();
    QJsonArray jxs = p["xs"].toArray();
    QJsonArray jxds = p["xds"].toArray();
    QJsonArray jxdds = p["xdds"].toArray();
    QJsonArray jradiuses = p["radiuses"].toArray();
    QJsonArray jwaypoint_xds = p["waypoint_xds"].toArray();
    QJsonArray jqs_redundant = p["qs_redundant"].toArray();

    int rowCount = jxs.size();
    for (int i = 0; i < _traj.waypoint_num; ++i) {
        const QJsonArray& jarr2 = jxs[i].toArray();
        _table_widget->insertRow(_table_widget->rowCount());
        for (int j = 0; j < model->columnCount(); ++j) {
            auto m_index = model->index(i,j);
            switch (j - 6)
            {
            case 0:
                model->setData(m_index, jxds[j].toDouble());
                break;
            case 1:
                model->setData(m_index, jxdds[j].toDouble());
                break;
            case 2:
                model->setData(m_index, jxds[j].toDouble());
                break;
            case 3:
                model->setData(m_index, jradiuses[j].toDouble());
                break;
            case 4:
                model->setData(m_index, jwaypoint_xds[j].toDouble());
                break;
            case 5:
                model->setData(m_index, jqs_redundant[j].toDouble());
                break;
            default:
                model->setData(m_index, jarr2[j].toDouble());
                break;
            }
        }
    }
    bool flag = p["is_radius_percent"].toBool();
    if(flag) {
        is_radius_percent_checkbox->setCheckState(Qt::CheckState::Checked);
    }
}

QWidget *MoveBlendingModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout *layout = new QVBoxLayout(_widget);
        _table_widget = new QTableWidget(0, 11, _widget);

        QLabel *indexLabel = new QLabel("Task index:");
        indexLabel->setBuddy(lineEditTaskIndex);
        layout->addWidget(indexLabel);
        layout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });

        layout->addWidget(_table_widget, 1, Qt::AlignHCenter);

        QStringList header_list;
        header_list << "x" << "y" << "z" << "R" << "P" << "Y" << "vel" << "acc" << "radius" << "way point vel" << "redundant angle velocity";
        _table_widget->setHorizontalHeaderLabels(header_list);

        auto addButton = new QPushButton("add WayPoint");
        auto removeButton = new QPushButton("remove WayPoint");
        is_radius_percent_checkbox = new QCheckBox();
        is_radius_percent_checkbox->setText("Not Checked: mm / Checked: %");

        _table_widget->setMaximumWidth(1000);
        _table_widget->setMinimumWidth(700);
        _table_widget->setMaximumHeight(200);
        _table_widget->setMinimumHeight(100);
        layout->addWidget(addButton);
        layout->addWidget(removeButton);
        layout->addWidget(is_radius_percent_checkbox);

        QObject::connect(addButton, &QPushButton::clicked, this, [&]() {
            _table_widget->insertRow(_table_widget->rowCount());
            _traj.waypoint_num = _table_widget->rowCount();
        });
        QObject::connect(removeButton, &QPushButton::clicked, this, [&]() {
            _table_widget->removeRow(_table_widget->rowCount() - 1);
            _traj.waypoint_num = _table_widget->rowCount();
        });
    }
    return _widget;
}
void MoveBlendingModel::saveGuiTask(TaskPlanner& planner) {
    _traj.xs.clear();
    _traj.xds.clear();
    _traj.xdds.clear();
    _traj.waypoint_xds.clear();
    _traj.radiuses.clear();
    _traj.qs_redundant.clear();
    auto model = _table_widget->model();
    //tablewidget model to Blending trajectory struct
    for (int i = 0; i < model->rowCount(); i++) {
        _traj.xs.push_back(CsDouble());
        for (int j = 0; j < model->columnCount(); j++) {
            auto value = model->index(i, j).data().toDouble();
            if (j < 6) {
                _traj.xs[i][j] = value;
            } else {  // j >= 6
                switch (j - 6) {
                case 0:
                    _traj.xds.push_back(value);
                    break;
                case 1:
                    _traj.xdds.push_back(value);
                    break;
                case 2:
                    _traj.waypoint_xds.push_back(value);
                    break;
                case 3:
                    _traj.radiuses.push_back(value);
                    break;
                case 4:
                    _traj.qs_redundant.push_back(value);
                    break;
                default:
                    break;
                }
            }
        }
    }

    // CsDouble csArr;
    // vec2Arr(target_x, csArr);taskPushBack_moveBlending
    planner.taskPushBack_moveBlending(planner.bp_gui_node_task_, _traj);
}



TCPMoveModel::TCPMoveModel() : KCRCommandModel() {
    target_x.resize(6);
}

QString TCPMoveModel::caption() const { return QStringLiteral("TCP MOVE TASK"); }

QString TCPMoveModel::name() const { return QStringLiteral("TCP MOVE  TASK"); }

NodeDataType TCPMoveModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "TCP MOVE  TASK"};
}

QJsonObject TCPMoveModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    printValueType(target_x, modelJson, "target_x");
    return modelJson;
}
void TCPMoveModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonArray jtarget_x = p["target_x"].toArray();
    updateWidgetFromJson(jtarget_x, target_x, lineEdits1);
}

QWidget *TCPMoveModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        const int numLineEdits = 6;

        for (int i = 0; i < numLineEdits; ++i)
            {
                QLineEdit *lineEdit = new QLineEdit("0.0");
                lineEdits1.append(lineEdit);
                QObject::connect(lineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
                    set_target_x(text.toDouble(), i);
                });
            }
        QGroupBox *groupBox1 = new QGroupBox("target_x");
        QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);
        for (auto edit : lineEdits1) {
            layout1->addWidget(edit);
        }
        layout1->setContentsMargins(0, 0, 0, 0);
        mainLayout->addWidget(groupBox1);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void TCPMoveModel::saveGuiTask(TaskPlanner& planner) {
    CsDouble csArr;
    vec2Arr(target_x, csArr);
    planner.taskPushBack_tcpMove(planner.bp_gui_node_task_, csArr);
}




DelayModel::DelayModel() : KCRCommandModel() {
    _delayTime = 0;
}


QString DelayModel::caption() const{ return QStringLiteral("TIME DELAY TASK (x10ms)"); }

QString DelayModel::name() const { return QStringLiteral("TIME DELAY TASK"); }

NodeDataType DelayModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "TIME DELAY TASK"};
}

QJsonObject DelayModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["delay"] = _delayTime;
    return modelJson;
}

void DelayModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jdelay_time = p["delay"].toInt();
    _lineEditDelay->setText(QString::number(jdelay_time.toInt()));
}

QWidget *DelayModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        _lineEditDelay = new QLineEdit("0");
        QObject::connect(_lineEditDelay, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_time_delay(text.toInt());
        });
        mainLayout->addWidget(_lineEditDelay);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void DelayModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_delay(planner.bp_gui_node_task_, _delayTime);
}



PauseModel::PauseModel() : KCRCommandModel() {}

QString PauseModel::caption() const{ return QStringLiteral("PAUSE TASK"); }

QString PauseModel::name() const { return QStringLiteral("PAUSE TASK"); }


NodeDataType PauseModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "PAUSE TASK"};
}

QJsonObject PauseModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void PauseModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
}

QWidget *PauseModel::embeddedWidget() {
    return nullptr;
}
//bool expandable() const { return false; }

void PauseModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_pause(planner.bp_gui_node_task_);
}

SavePointModel::SavePointModel() : KCRCommandModel() {
}

QString SavePointModel::caption() const{ return QStringLiteral("SAVE POINT TASK"); }

QString SavePointModel::name() const { return QStringLiteral("SAVE POINT TASK"); }


NodeDataType SavePointModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "SAVE POINT TASK"};
}

QJsonObject SavePointModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void SavePointModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
}

QWidget *SavePointModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
//bool SavePointModel::expandable() const { return false; }

void SavePointModel::saveGuiTask(TaskPlanner& planner) {
    planner.savePoint_pushBack(planner.bp_gui_node_task_);
}


RewindModel::RewindModel() : KCRCommandModel() {}

QString RewindModel::caption() const{ return QStringLiteral("REWIND TASK"); }

QString RewindModel::name() const { return QStringLiteral("REWIND TASK"); }


NodeDataType RewindModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "REWIND TASK"};
}

QJsonObject RewindModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void RewindModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
}

QWidget *RewindModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
//bool RewindModel::expandable() const { return false; }

void RewindModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_rewind(planner.bp_gui_node_task_);
}

CSAccVelUpdateModel::CSAccVelUpdateModel() : KCRCommandModel() {
    acc = 0.0;
    vel = 0.0;
}

QString CSAccVelUpdateModel::caption() const{ return QStringLiteral("CS VEL&ACC UPDATE TASK"); }

QString CSAccVelUpdateModel::name() const { return QStringLiteral("CS VEL&ACC UPDATE TASK"); }

NodeDataType CSAccVelUpdateModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "CS VEL&ACC UPDATE TASK"};
}

QJsonObject CSAccVelUpdateModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["vel"] = vel;
    modelJson["acc"] = acc;
    return modelJson;
}
void CSAccVelUpdateModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jvel = p["vel"].toDouble();
    QJsonValue jacc = p["acc"].toDouble();
    lineEditsVel->setText(QString::number(jvel.toDouble()));
    lineEditsAcc->setText(QString::number(jacc.toDouble()));
}

QWidget *CSAccVelUpdateModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();

        lineEditsVel = new QLineEdit("0.0");
        lineEditsAcc = new QLineEdit("0.0");

        QObject::connect(lineEditsVel, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_vel(text.toDouble());
        });
        QObject::connect(lineEditsAcc, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_acc(text.toDouble());
        });
        QLabel *l_vel = new QLabel("&Velocity(mm/s):");
        QLabel *l_acc = new QLabel("&Acceleration(mm/s2):");
        l_acc->setBuddy(lineEditsAcc);
        l_vel->setBuddy(lineEditsVel);

        mainLayout->addWidget(l_vel);
        mainLayout->addWidget(lineEditsVel);
        mainLayout->addWidget(l_acc);
        mainLayout->addWidget(lineEditsAcc);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });

        mainLayout->setContentsMargins(0, 0, 0, 0);

        //mainLayout->addLayout(layout1);

        _widget->setLayout(mainLayout);

    }
    return _widget;
}
void CSAccVelUpdateModel::saveGuiTask(TaskPlanner& planner) {
    planner.ref_unit_task_.vel_cs = vel;
    planner.ref_unit_task_.acc_cs = acc;
    std::cout << "!!!!!!!!!!!!!!!!!!vel, acc updated to " << vel << " "<< acc << "\n";
}

IrlGrpModel::IrlGrpModel() : KCRCommandModel() {
    value_ = 0;
    address_ = 0;
}

QString IrlGrpModel::caption() const{ return QStringLiteral("IRL GRIPPER TASK\n (command, value, address)"); }

QString IrlGrpModel::name() const { return QStringLiteral("IRL GRIPPER TASK"); }

NodeDataType IrlGrpModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "IRL GRIP TASK"};
}

QJsonObject IrlGrpModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["grp_cmd"] = static_cast<int>(grp_cmd_map->value(_comboBoxGrpCmd->currentText()));
    modelJson["value"] = value_;
    modelJson["address"] = address_;
    return modelJson;
}

void IrlGrpModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    _comboBoxGrpCmd->setCurrentText(grp_cmd_map->key(static_cast<KR_GRP>(p["grp_cmd"].toInt())));
    _lineEditValue->setText(QString::number(p["value"].toInt()));
    _lineEditAddress->setText(QString::number(p["address"].toInt()));
}

QWidget *IrlGrpModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        grp_cmd_map = new QMap<QString, KR_GRP>();
        grp_cmd_map->insert("INIT", KR_GRP::INIT);
        grp_cmd_map->insert("OPEN", KR_GRP::OPEN);
        grp_cmd_map->insert("CLOSE", KR_GRP::CLOSE);
        grp_cmd_map->insert("POS_CTRL", KR_GRP::POS_CTRL);
        grp_cmd_map->insert("INIT_2", KR_GRP::INIT_2);
        grp_cmd_map->insert("VACUUM_ON", KR_GRP::VACUUM_ON);
        grp_cmd_map->insert("VACUUM_OFF", KR_GRP::VACUUM_OFF);
        _comboBoxGrpCmd = new QComboBox();
        _comboBoxGrpCmd->addItems(QStringList(grp_cmd_map->keys()));

        _lineEditValue = new QLineEdit("0");
        QObject::connect(_lineEditValue, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_value(text.toInt());
        });
        _lineEditAddress = new QLineEdit("0");
        QObject::connect(_lineEditAddress, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_address(text.toInt());
        });

        QLabel *label1 = new QLabel("&Gripper command: ");
        QLabel *label2 = new QLabel("&value: ");
        QLabel *label3 = new QLabel("&address: ");

        label1->setBuddy(_comboBoxGrpCmd);
        label2->setBuddy(_lineEditValue);
        label3->setBuddy(_lineEditAddress);

        mainLayout->addWidget(label1);
        mainLayout->addWidget(_comboBoxGrpCmd);
        mainLayout->addWidget(label2);
        mainLayout->addWidget(_lineEditValue);
        mainLayout->addWidget(label3);
        mainLayout->addWidget(_lineEditAddress);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

void IrlGrpModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_irl_grp(planner.bp_gui_node_task_,
        grp_cmd_map->value(_comboBoxGrpCmd->currentText()), value_, address_);
}

IrlGrpCmdModel::IrlGrpCmdModel() : KCRCommandModel() {
    _grp_pos_percent = 0;
}

QString IrlGrpCmdModel::caption() const{ return QStringLiteral("IRL GRIPPER CMD TASK (grip range:%)"); }

QString IrlGrpCmdModel::name() const { return QStringLiteral("IRL GRIP CMD TASK"); }

NodeDataType IrlGrpCmdModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "IRL GRIP CMD TASK"};
}

QJsonObject IrlGrpCmdModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["pos_percent"] = _grp_pos_percent;
    return modelJson;
}

void IrlGrpCmdModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jpos_percent = p["pos_percent"].toInt();
    _lineEditGrpPos->setText(QString::number(jpos_percent.toInt()));
}

QWidget *IrlGrpCmdModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        _lineEditGrpPos = new QLineEdit("0");
        QObject::connect(_lineEditGrpPos, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_pos_percent(text.toInt());
        });
        mainLayout->addWidget(_lineEditGrpPos);
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void IrlGrpCmdModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_irl_grp_cmd(planner.bp_gui_node_task_, _grp_pos_percent);
}


IrlGrpInitModel::IrlGrpInitModel() : KCRCommandModel() {
    _grp_pos_percent = 0;
}

QString IrlGrpInitModel::caption() const{ return QStringLiteral("IRL GRIPPER INIT TASK (grip range:%)"); }

QString IrlGrpInitModel::name() const { return QStringLiteral("IRL GRIP INIT TASK"); }

NodeDataType IrlGrpInitModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "IRL GRIP INIT TASK"};
}

QJsonObject IrlGrpInitModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["pos_percent"] = _grp_pos_percent;
    return modelJson;
}

void IrlGrpInitModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jpos_percent = p["pos_percent"].toInt();
    _lineEditGrpPos->setText(QString::number(jpos_percent.toInt()));
}

QWidget *IrlGrpInitModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        _lineEditGrpPos = new QLineEdit("0");
        QObject::connect(_lineEditGrpPos, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_pos_percent(text.toInt());
        });
        mainLayout->addWidget(_lineEditGrpPos);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void IrlGrpInitModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_irl_grp_init(planner.bp_gui_node_task_, _grp_pos_percent);
}

BP_PLCModel::BP_PLCModel() : KCRCommandModel() {
    _command = 0;
}

QString BP_PLCModel::caption() const{ return QStringLiteral("BINPICKING PLC COMMAND"); }

QString BP_PLCModel::name() const { return QStringLiteral("BINPICKING PLC COMMAND"); }

NodeDataType BP_PLCModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "BINPICKING PLC TASK"};
}

QJsonObject BP_PLCModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["plc_command"] = _command;
    return modelJson;
}

void BP_PLCModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jcommand = p["plc_command"].toInt();
    _lineEditPlcCommand->setText(QString::number(jcommand.toInt()));
}

QWidget *BP_PLCModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        _lineEditPlcCommand = new QLineEdit("0");
        QObject::connect(_lineEditPlcCommand, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_command(text.toInt());
        });
        mainLayout->addWidget(_lineEditPlcCommand);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

void BP_PLCModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBackPLCModbusCmd(planner.bp_gui_node_task_, _command);
}

BP_GripperRelayCmdModel::BP_GripperRelayCmdModel() : KCRCommandModel() {}

QString BP_GripperRelayCmdModel::caption() const{ return QStringLiteral("Gripper Relay COMMAND"); }

QString BP_GripperRelayCmdModel::name() const { return QStringLiteral("Gripper Relay COMMAND"); }

NodeDataType BP_GripperRelayCmdModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "GRIPPER RELAY TASK"};
}

QJsonObject BP_GripperRelayCmdModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["relay_command"] = (int)_command;
    return modelJson;
}

void BP_GripperRelayCmdModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    cbox->setCurrentIndex(p["relay_command"].toInt());
}

QWidget *BP_GripperRelayCmdModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        cbox = new QComboBox();
        QStringList list;
        list << "PNE1_OFF" << "PNE1_ON" << "PNE2_OFF" << "PNE2_ON" << "VAC1_OFF" << "VAC1_ON" << "VAC2_OFF"
        << "VAC2_ON" << "ETC1_OFF" << "ETC2_OFF" << "ETC2_ON" ;
        cbox->addItems(list);
        connect(cbox, SIGNAL (activated(int)), this, SLOT(set_relay(int)));
        mainLayout->addWidget(cbox);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void BP_GripperRelayCmdModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBackGripperRelayCmd(planner.bp_gui_node_task_, _command);
}

SetTCPModel::SetTCPModel() : KCRCommandModel() {
    target_x.resize(6);

}

QString SetTCPModel::caption() const{ return QStringLiteral("SET TCP COORDINATE TASK"); }

QString SetTCPModel::name() const { return QStringLiteral("SET TCP TASK"); }

NodeDataType SetTCPModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "SET TCP TASK"};
}

QJsonObject SetTCPModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    printValueType(target_x, modelJson, "target_x");
    return modelJson;
}
void SetTCPModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonArray jtarget_x = p["target_x"].toArray();
    updateWidgetFromJson(jtarget_x, target_x, lineEdits1);
}

QWidget *SetTCPModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        const int numLineEdits = 6;

        for (int i = 0; i < numLineEdits; ++i)
            {
                QLineEdit *lineEdit = new QLineEdit("0.0");
                lineEdits1.append(lineEdit);
                QObject::connect(lineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
                    set_target_x(text.toDouble(), i);
                });
            }
        QGroupBox *groupBox1 = new QGroupBox("Coodinate: xyzRPY");
        QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);
        for (auto edit : lineEdits1) {
            layout1->addWidget(edit);
        }
        layout1->setContentsMargins(0, 0, 0, 0);

        mainLayout->addWidget(groupBox1);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });

        _widget->setLayout(mainLayout);

    }
    return _widget;
}
void SetTCPModel::saveGuiTask(TaskPlanner& planner) {
    CsDouble csArr;
    vec2Arr(target_x, csArr);

    planner.taskPushBack_setTCP(planner.bp_gui_node_task_, csArr);
}


BP_SelectTCPModel::BP_SelectTCPModel() : KCRCommandModel() {
    tcp_idx = 0;
}

QString BP_SelectTCPModel::caption() const{ return QStringLiteral("SELECT TCP INDEX (single - (#1) / dual - left(#5), right(#6))"); }

QString BP_SelectTCPModel::name() const { return QStringLiteral("SELECT TCP TASK"); }

NodeDataType BP_SelectTCPModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "SELECT TCP TASK"};
}

QJsonObject BP_SelectTCPModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["tcp_idx"] = tcp_idx;
    return modelJson;
}

void BP_SelectTCPModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jcommand = p["tcp_idx"].toInt();
    _lineEditTcpIndex->setText(QString::number(jcommand.toInt()));
}

QWidget *BP_SelectTCPModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        _lineEditTcpIndex = new QLineEdit("0");
        QObject::connect(_lineEditTcpIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_tcp_idx(text.toInt());
        });
        mainLayout->addWidget(_lineEditTcpIndex);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void BP_SelectTCPModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_selectTCP(planner.bp_gui_node_task_, tcp_idx);
}

BP_setGripperDriverModel::BP_setGripperDriverModel() : KCRCommandModel() {
    grp_driver_idx = 0;
}

QString BP_setGripperDriverModel::caption() const{ return QStringLiteral("SET GRIPPER DRIVER TASK(Main:2 / Sub:1)"); }

QString BP_setGripperDriverModel::name() const { return QStringLiteral("SET GRIPPER DRIVER TASK"); }

NodeDataType BP_setGripperDriverModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "SET GRIPPER DRIVER TASK"};
}

QJsonObject BP_setGripperDriverModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["grp_driver_idx"] = grp_driver_idx;
    return modelJson;
}

void BP_setGripperDriverModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jcommand = p["grp_driver_idx"].toInt();
    _lineEditDriver->setText(QString::number(jcommand.toInt()));
}

QWidget *BP_setGripperDriverModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        _lineEditDriver = new QLineEdit("0");
        QObject::connect(_lineEditDriver, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_grp_driver_idx(text.toInt());
        });
        mainLayout->addWidget(_lineEditDriver);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void BP_setGripperDriverModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_setGripperDriver(planner.bp_gui_node_task_, grp_driver_idx);
}


BP_Recog_KORASGripperModel::BP_Recog_KORASGripperModel() : KCRCommandModel() {
}

QString BP_Recog_KORASGripperModel::caption() const{ return QStringLiteral("KORAS GRIPPER RECOGNITION TASK"); }

QString BP_Recog_KORASGripperModel::name() const { return QStringLiteral("KORAS GRIPPER RECOGNITION TASK"); }


NodeDataType BP_Recog_KORASGripperModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "KORAS GRIPPER RECOGNITION TASK"};
}

QJsonObject BP_Recog_KORASGripperModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void BP_Recog_KORASGripperModel::load(QJsonObject const &p) {KCRCommandModel::load(p);}

QWidget *BP_Recog_KORASGripperModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
//bool BP_Recog_KORASGripperModel::expandable() const { return false; }

void BP_Recog_KORASGripperModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_taskRecog_KORASGripperCmd(planner.bp_gui_node_task_);
}

BP_KORASGripperModel::BP_KORASGripperModel() : KCRCommandModel() {
}

QString BP_KORASGripperModel::caption() const{ return QStringLiteral("KORAS GRIPPER TASK"); }

QString BP_KORASGripperModel::name() const { return QStringLiteral("KORAS GRIPPER TASK"); }


NodeDataType BP_KORASGripperModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "KORAS GRIPPER TASK"};
}

QJsonObject BP_KORASGripperModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["grp_cmd"] = static_cast<int>(grp_cmd_map->value(_comboBoxGrpCmd->currentText()));
    modelJson["position"] = position;
    modelJson["speed"] = speed;
    return modelJson;
}

void BP_KORASGripperModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    _comboBoxGrpCmd->setCurrentText(grp_cmd_map->key(static_cast<KR_GRP>(p["grp_cmd"].toInt())));
    _lineEditPosition->setText(QString::number(p["position"].toInt()));
    _lineEditSpeed->setText(QString::number(p["speed"].toInt()));
}

QWidget *BP_KORASGripperModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        grp_cmd_map = new QMap<QString, KR_GRP>();
        grp_cmd_map->insert("INIT", KR_GRP::INIT);
        grp_cmd_map->insert("OPEN", KR_GRP::OPEN);
        grp_cmd_map->insert("CLOSE", KR_GRP::CLOSE);
        grp_cmd_map->insert("POS_CTRL", KR_GRP::POS_CTRL);
        grp_cmd_map->insert("INIT_2", KR_GRP::INIT_2);
        grp_cmd_map->insert("VACUUM_ON", KR_GRP::VACUUM_ON);
        grp_cmd_map->insert("VACUUM_OFF", KR_GRP::VACUUM_OFF);
        _comboBoxGrpCmd = new QComboBox();
        _comboBoxGrpCmd->addItems(QStringList(grp_cmd_map->keys()));

        _lineEditPosition = new QLineEdit();
        QObject::connect(_lineEditPosition, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_position(text.toInt());
        });
        _lineEditSpeed = new QLineEdit();
        QObject::connect(_lineEditSpeed, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_speed(text.toInt());
        });

        QLabel *label1 = new QLabel("&Gripper command: ");
        QLabel *label2 = new QLabel("&position: ");
        QLabel *label3 = new QLabel("&speed: ");

        label1->setBuddy(_comboBoxGrpCmd);
        label2->setBuddy(_lineEditPosition);
        label3->setBuddy(_lineEditSpeed);

        mainLayout->addWidget(label1);
        mainLayout->addWidget(_comboBoxGrpCmd);
        mainLayout->addWidget(label2);
        mainLayout->addWidget(_lineEditPosition);
        mainLayout->addWidget(label3);
        mainLayout->addWidget(_lineEditSpeed);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
//bool BP_KORASGripperModel::expandable() const { return false; }

void BP_KORASGripperModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBackKORASGripperCmd(planner.bp_gui_node_task_, static_cast<int>(grp_cmd_map->value(_comboBoxGrpCmd->currentText()))
    , position, speed);
    // taskPushBack_delay_1ms(planner.bp_gui_node_task_, 50);
    // planner.taskPushBack_taskRecog_KORASGripperCheckGrasping(planner.bp_gui_node_task_);
    // taskPushBack_delay_1ms(planner.bp_gui_node_task_, 20);
}

BP_3DScanModel::BP_3DScanModel() : KCRCommandModel() {}

QString BP_3DScanModel::caption() const{ return QStringLiteral("3D SCAN TASK"); }

QString BP_3DScanModel::name() const { return QStringLiteral("3D SCAN TASK"); }

NodeDataType BP_3DScanModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "3D SCAN TASK"};
}

QJsonObject BP_3DScanModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void BP_3DScanModel::load(QJsonObject const &p) {KCRCommandModel::load(p);}

void BP_3DScanModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_taskRecog_3DScanning(planner.bp_gui_node_task_);
}

QWidget* BP_3DScanModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

BP_TemplateMatchingModel::BP_TemplateMatchingModel() : KCRCommandModel() {
    sampling_num = 0;
}

QString BP_TemplateMatchingModel::caption() const{ return QStringLiteral("TEMPLATE MATCHING TASK"); }

QString BP_TemplateMatchingModel::name() const { return QStringLiteral("TEMPLATE MATCHING TASK"); }


NodeDataType BP_TemplateMatchingModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "TEMPLATE MATCHING TASK"};
}

QJsonObject BP_TemplateMatchingModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["do_sampling"] = do_sampling_checkbox->isChecked();
    modelJson["sampling_num"] = (int)sampling_num;
    return modelJson;
}

void BP_TemplateMatchingModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    bool flag = p["do_sampling"].toBool();
    if(flag) {
        do_sampling_checkbox->setCheckState(Qt::CheckState::Checked);
    }
    sampling_num_lineEdit->setText(QString::number(p["sampling_num"].toInt()));
}

QWidget *BP_TemplateMatchingModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        QLabel *label1 = new QLabel("&Do Sampling:");
        QLabel *label2 = new QLabel("&Sampling number:");

        do_sampling_checkbox = new QCheckBox();
        sampling_num_lineEdit = new QLineEdit("0");
        QValidator *validator = new QIntValidator(0, 999);

        sampling_num_lineEdit->setValidator(validator);

        QObject::connect(sampling_num_lineEdit, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_sampling_num(text.toUInt());
        });
        label1->setBuddy(do_sampling_checkbox);
        label2->setBuddy(sampling_num_lineEdit);

        mainLayout->addWidget(label1);
        mainLayout->addWidget(do_sampling_checkbox);
        mainLayout->addWidget(label2);
        mainLayout->addWidget(sampling_num_lineEdit);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });

        _widget->setLayout(mainLayout);
    }
    return _widget;
}
//bool BP_TemplateMatchingModel::expandable() const { return false; }

void BP_TemplateMatchingModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_templateMatching(planner.bp_gui_node_task_, do_sampling_checkbox->isChecked(), sampling_num);
    planner.taskPushBack_delay_1ms(planner.bp_gui_node_task_, 20);
    planner.taskPushBack_taskRecog_checkMatchingFinished(planner.bp_gui_node_task_);
    planner.taskPushBack_delay_1ms(planner.bp_gui_node_task_, 20);
}

BP_ScanTemplateMatchingModel::BP_ScanTemplateMatchingModel() : KCRCommandModel() {
    sampling_num = 0;
}

QString BP_ScanTemplateMatchingModel::caption() const{ return QStringLiteral("TEMPLATE SCAN & MATCHING TASK"); }

QString BP_ScanTemplateMatchingModel::name() const { return QStringLiteral("TEMPLATE SCAN & MATCHING TASK"); }


NodeDataType BP_ScanTemplateMatchingModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "TEMPLATE SCAN & MATCHING TASK"};
}

QJsonObject BP_ScanTemplateMatchingModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["do_sampling"] = do_sampling_checkbox->isChecked();
    modelJson["sampling_num"] = (int)sampling_num;
    return modelJson;
}

void BP_ScanTemplateMatchingModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    bool flag = p["do_sampling"].toBool();
    if(flag) {
        do_sampling_checkbox->setCheckState(Qt::CheckState::Checked);
    }
    sampling_num_lineEdit->setText(QString::number(p["sampling_num"].toInt()));
}

QWidget *BP_ScanTemplateMatchingModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        QLabel *label1 = new QLabel("&Do Sampling:");
        QLabel *label2 = new QLabel("&Sampling number:");

        do_sampling_checkbox = new QCheckBox();
        sampling_num_lineEdit = new QLineEdit("0");
        QValidator *validator = new QIntValidator(0, 999);

        sampling_num_lineEdit->setValidator(validator);

        QObject::connect(sampling_num_lineEdit, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_sampling_num(text.toUInt());
        });
        label1->setBuddy(do_sampling_checkbox);
        label2->setBuddy(sampling_num_lineEdit);

        mainLayout->addWidget(label1);
        mainLayout->addWidget(do_sampling_checkbox);
        mainLayout->addWidget(label2);
        mainLayout->addWidget(sampling_num_lineEdit);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
//bool BP_ScanTemplateMatchingModel::expandable() const { return false; }

void BP_ScanTemplateMatchingModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_taskRecog_3DScanningAndMatching(planner.bp_gui_node_task_, false, 8);
}


BP_ScanningDelayModel::BP_ScanningDelayModel() : KCRCommandModel() {
    _delayTime = 0;
}

QString BP_ScanningDelayModel::caption() const{ return QStringLiteral("SCANNING DELAY TASK (ms)"); }

QString BP_ScanningDelayModel::name() const { return QStringLiteral("SCANNING DELAY TASK"); }

NodeDataType BP_ScanningDelayModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "SCANNING DELAY TASK"};
}

QJsonObject BP_ScanningDelayModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["delay"] = _delayTime;
    return modelJson;
}

void BP_ScanningDelayModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jdelay_time = p["delay"].toInt();
    _lineEditDelay->setText(QString::number(jdelay_time.toInt()));
}

QWidget *BP_ScanningDelayModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        _lineEditDelay = new QLineEdit("0");
        QObject::connect(_lineEditDelay, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_time_delay(text.toInt());
        });
        mainLayout->addWidget(_lineEditDelay);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void BP_ScanningDelayModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_delayScanning(planner.bp_gui_node_task_, _delayTime);
}

BP_DoGraspingModel::BP_DoGraspingModel() : KCRCommandModel() {
    vel = 0;
    acc = 0;
}

QString BP_DoGraspingModel::caption() const{ return QStringLiteral("DO GRASP TASK"); }

QString BP_DoGraspingModel::name() const { return QStringLiteral("DO GRASP TASK"); }

NodeDataType BP_DoGraspingModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "DO GRASP TASK"};
}

QJsonObject BP_DoGraspingModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["vel"] = vel;
    modelJson["acc"] = acc;
    return modelJson;
}

void BP_DoGraspingModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    // QJsonValue jvel = p["vel"].toDouble();
    // QJsonValue jacc = p["acc"].toDouble();
    lineEditsVel->setText(QString::number(p["vel"].toDouble()));
    lineEditsAcc->setText(QString::number(p["acc"].toDouble()));
}

QWidget *BP_DoGraspingModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        lineEditsVel = new QLineEdit("0");
        lineEditsAcc = new QLineEdit("0");
        QObject::connect(lineEditsVel, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_vel(text.toDouble());
        });
        QObject::connect(lineEditsAcc, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_acc(text.toDouble());
        });
        QLabel* label1 = new QLabel("&velocity(m/s):");
        QLabel* label2 = new QLabel("&acceleration(m/s2):");
        label1->setBuddy(lineEditsVel);
        label2->setBuddy(lineEditsAcc);
        mainLayout->addWidget(label1);
        mainLayout->addWidget(lineEditsVel);
        mainLayout->addWidget(label2);
        mainLayout->addWidget(lineEditsAcc);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void BP_DoGraspingModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_doGrasping_binPicking(planner.bp_gui_node_task_, acc, vel);
}

BP_DoSubGraspingModel::BP_DoSubGraspingModel() : KCRCommandModel() {
    vel = 0;
    acc = 0;
}

QString BP_DoSubGraspingModel::caption() const{ return QStringLiteral("DO SUB GRASP TASK (ms)"); }

QString BP_DoSubGraspingModel::name() const { return QStringLiteral("DO SUB GRASP TASK"); }

NodeDataType BP_DoSubGraspingModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "DO SUB GRASP TASK"};
}

QJsonObject BP_DoSubGraspingModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["vel"] = vel;
    modelJson["acc"] = acc;
    return modelJson;
}

void BP_DoSubGraspingModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    lineEditsVel->setText(QString::number(p["vel"].toDouble()));
    lineEditsAcc->setText(QString::number(p["acc"].toDouble()));
}

QWidget *BP_DoSubGraspingModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        lineEditsVel = new QLineEdit("0");
        lineEditsAcc = new QLineEdit("0");
        QObject::connect(lineEditsVel, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_vel(text.toDouble());
        });
        QObject::connect(lineEditsAcc, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_acc(text.toDouble());
        });
        QLabel* label1 = new QLabel("&velocity:");
        QLabel* label2 = new QLabel("&acceleration:");
        label1->setBuddy(lineEditsVel);
        label2->setBuddy(lineEditsAcc);
        mainLayout->addWidget(label1);
        mainLayout->addWidget(lineEditsVel);
        mainLayout->addWidget(label2);
        mainLayout->addWidget(lineEditsAcc);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void BP_DoSubGraspingModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_doSubGrasping_binPicking(planner.bp_gui_node_task_, acc, vel);
}

BP_CountDetachingPose::BP_CountDetachingPose() : KCRCommandModel() {
}

QString BP_CountDetachingPose::caption() const{ return QStringLiteral("COUNT DETACHING POSE"); }

QString BP_CountDetachingPose::name() const { return QStringLiteral("COUNT DETACHING POSE"); }


NodeDataType BP_CountDetachingPose::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "COUNT DETACHING POSE"};
}

QJsonObject BP_CountDetachingPose::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void BP_CountDetachingPose::load(QJsonObject const &p) { KCRCommandModel::load(p);}


//bool BP_CountDetachingPose::expandable() const { return false; }

QWidget* BP_CountDetachingPose::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

void BP_CountDetachingPose::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_countDetachingPose(planner.bp_gui_node_task_);
}

StackingTaskModel::StackingTaskModel() : KCRCommandModel() {

}

QString StackingTaskModel::caption() const{ return QStringLiteral("STACKING TASK(DEMO TYPE)"); }

QString StackingTaskModel::name() const { return QStringLiteral("STACKING TASK"); }

NodeDataType StackingTaskModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "STACKING TASK"};
}

QJsonObject StackingTaskModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["type"] = static_cast<int>(demo_type_map->value(_comboBoxDemoType->currentText()));
    modelJson["is_slow_mode"] = is_slow_mode_checkbox->isChecked();
    return modelJson;
}
void StackingTaskModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    _comboBoxDemoType->setCurrentText(demo_type_map->key(static_cast<BPDemoType>(p["type"].toInt())));
    bool flag = p["is_slow_mode"].toBool();
    if(flag) {
        is_slow_mode_checkbox->setCheckState(Qt::CheckState::Checked);
    }
}

QWidget *StackingTaskModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();

        QGroupBox *groupBox1 = new QGroupBox("Stacking task");
        QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);

        is_slow_mode_checkbox = new QCheckBox();
        is_slow_mode_checkbox->setText("checked: slow_mode");

        layout1->addWidget(is_slow_mode_checkbox);

        // QPushButton* updateBtn = new QPushButton("Get current value");

        // QObject::connect(updateBtn, &QPushButton::clicked, this, [this]() {
        //     QNode* ptr = &QNode::getInstance();
        //     CsDouble arr = ptr->params_.meas.x;
        //     target_x.resize(6);
        //     for (size_t i = 0; i < 6; i++)
        //     {
        //         lineEdits1[i]->setText(QString::number(arr[i]));
        //     }

        // });
        // layout1->addWidget(updateBtn);

        demo_type_map = new QMap<QString, BPDemoType>();
        demo_type_map->insert("CYLINDER", BPDemoType::DEMO_BP_CNC_LATHE_CYLINDER);
        demo_type_map->insert("SQUARE_PEG", BPDemoType::DEMO_BP_CNC_MILLING_SQUARE_PEG);
        demo_type_map->insert("WELDING_T", BPDemoType::DEMO_BP_PICK_WELDING_T_JOINT_2F_GRP);
        demo_type_map->insert("WELDING_ELBOW", BPDemoType::DEMO_BP_PICK_WELDING_ELBOW_JOINT);
        demo_type_map->insert("BOLT_BUSH", BPDemoType::DEMO_BP_CNC_MILLING_BOLT_BUSH);
        demo_type_map->insert("GEAR", BPDemoType::DEMO_BP_PICK_GEAR);
        _comboBoxDemoType = new QComboBox();
        _comboBoxDemoType->addItems(QStringList(demo_type_map->keys()));

        layout1->addWidget(_comboBoxDemoType);
        layout1->setContentsMargins(0, 0, 0, 0);
        mainLayout->addWidget(groupBox1);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });

        _widget->setLayout(mainLayout);

    }
    return _widget;
}
void StackingTaskModel::saveGuiTask(TaskPlanner& planner) {
    bool slow_mode = is_slow_mode_checkbox->isChecked();
    BPDemoType demo_tag = demo_type_map->value(_comboBoxDemoType->currentText());
    //std::vector<UnitTask> targetModuleTask = planner.bp_gui_node_task_;
    int main_grp = 2;
    int sub_grp  = 1;
    double acc_move, vel_move, acc_contact, vel_contact;
    if(slow_mode) {
        acc_move = 0.2; vel_move = 0.1;
        acc_contact = 0.075; vel_contact = 0.03;
    } else {
        acc_move = 0.6; vel_move = 0.3;
        acc_contact = 0.2; vel_contact = 0.1;
    }

    double apporach_distance = 0.0;
    if (demo_tag == DEMO_BP_CNC_MILLING_BOLT_BUSH) {
        apporach_distance = 0.0075;
    } else if (demo_tag == DEMO_BP_CNC_LATHE_CYLINDER) {
        apporach_distance = 0.01;
    } else if (demo_tag == DEMO_BP_CNC_MILLING_SQUARE_PEG) {
        apporach_distance = 0.01;
    } else { // 2-finger grasping - Open
        apporach_distance = 0.01;
    }

    // pose, acc(0.05m/s^2), vel(0.0033m/s)
    //// TODO: JSON파일의 z margin과 연동해야 함.
    //// Tool frame 기준 approach 자세
    planner.taskPushBack_csMoveToolFrame(planner.bp_gui_node_task_, {0, 0, apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
    planner.taskPushBack_delay_1ms(planner.bp_gui_node_task_, 50);

    // Gripper - ungrasp
    if (demo_tag == DEMO_BP_CNC_MILLING_BOLT_BUSH) { // inner grasping
        planner.taskPushBackKORASGripperCmd(planner.bp_gui_node_task_, (uint16_t)KR_GRP::CLOSE);
        planner.taskPushBack_delay_1ms(planner.bp_gui_node_task_, 1000);
    } else if (demo_tag == DEMO_BP_CNC_MILLING_SQUARE_PEG) { // vacuum grasping - OFF
        planner.taskPushBackKORASGripperCmd(planner.bp_gui_node_task_, (uint16_t)KR_GRP::VACUUM_OFF);
        planner.taskPushBack_delay_1ms(planner.bp_gui_node_task_, 1000);
    } else { // 2-finger grasping - Open
        planner.taskPushBack_irl_grp(planner.bp_gui_node_task_, KR_GRP::POS_CTRL, 7250, sub_grp);
        planner.taskPushBack_delay_1ms(planner.bp_gui_node_task_, 1000);
    }
    planner.taskPushBack_countDetachingPose(planner.bp_gui_node_task_);
    planner.taskPushBack_delay_1ms(planner.bp_gui_node_task_, 10);

    planner.taskPushBack_csMoveToolFrame(planner.bp_gui_node_task_, {0, 0, -apporach_distance, 0, 0, 0}, acc_contact, vel_contact, true); // false: absolute, true: relative
    planner.taskPushBack_delay_1ms(planner.bp_gui_node_task_, 50);

}

TaskConfirmModel::TaskConfirmModel() : KCRCommandModel() {}

QString TaskConfirmModel::caption() const{ return QStringLiteral("Popup Message Box"); }

QString TaskConfirmModel::name() const { return QStringLiteral("POPUP MESSAGE"); }

NodeDataType TaskConfirmModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "POPUP MESSAGE"};
}

QJsonObject TaskConfirmModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["message"] = lineEditsMessage->text();
    return modelJson;
}
void TaskConfirmModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QString load_message = p["message"].toString();
    lineEditsMessage->setText(load_message);
}

QWidget *TaskConfirmModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        QLabel* messageLabel = new QLabel("Message: ");
        lineEditsMessage = new QLineEdit("Type your message");
        QObject::connect(lineEditsMessage, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_message(text);
        });
        messageLabel->setBuddy(lineEditsMessage);
        mainLayout->addWidget(messageLabel);
        mainLayout->addWidget(lineEditsMessage);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

void TaskConfirmModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_pause(planner.bp_gui_node_task_);
}

SetTipIndexTaskModel::SetTipIndexTaskModel() : KCRCommandModel() {
    tip_index = 0;
}

QString SetTipIndexTaskModel::caption() const{ return QStringLiteral("SET TIP INDEX"); }

QString SetTipIndexTaskModel::name() const { return QStringLiteral("SET TIP INDEX"); }

NodeDataType SetTipIndexTaskModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "SET TIP INDEX"};
}

QJsonObject SetTipIndexTaskModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["tip_index"] = tip_index;
    return modelJson;
}

void SetTipIndexTaskModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jcommand = p["tip_index"].toInt();
    _lineEditTipIndex->setText(QString::number(jcommand.toInt()));
}

QWidget *SetTipIndexTaskModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        _lineEditTipIndex = new QLineEdit("0");
        QObject::connect(_lineEditTipIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_tip_index(text.toInt());
        });
        mainLayout->addWidget(_lineEditTipIndex);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void SetTipIndexTaskModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_taskRecog_TipChangingSetCurrentTipIndex(planner.bp_gui_node_task_, tip_index);
}


TipChangingAttachModel::TipChangingAttachModel() : KCRCommandModel() {}

QString TipChangingAttachModel::caption() const{ return QStringLiteral("TIP CHANGING ATTACH TASK"); }

QString TipChangingAttachModel::name() const { return QStringLiteral("TIP CHANGING ATTACH TASK"); }

NodeDataType TipChangingAttachModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "TIP CHANGING ATTACH TASK"};
}

QJsonObject TipChangingAttachModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void TipChangingAttachModel::load(QJsonObject const &p) {KCRCommandModel::load(p);}

void TipChangingAttachModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_taskRecog_TipChangingAttach_jsMove(planner.bp_gui_node_task_);
}

QWidget* TipChangingAttachModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

TipChangingDefaultAttachModel::TipChangingDefaultAttachModel() : KCRCommandModel() {}

QString TipChangingDefaultAttachModel::caption() const{ return QStringLiteral("TIP CHANGING DEFAULT ATTACH TASK"); }

QString TipChangingDefaultAttachModel::name() const { return QStringLiteral("TIP CHANGING DEFAULT ATTACH TASK"); }

NodeDataType TipChangingDefaultAttachModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "TIP CHANGING DEFAULT ATTACH TASK"};
}

QJsonObject TipChangingDefaultAttachModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void TipChangingDefaultAttachModel::load(QJsonObject const &p) {KCRCommandModel::load(p);}

void TipChangingDefaultAttachModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_taskRecog_TipChangingDefaultAttach_jsMove(planner.bp_gui_node_task_);
}

QWidget* TipChangingDefaultAttachModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

TipChangingDetachModel::TipChangingDetachModel() : KCRCommandModel() {}

QString TipChangingDetachModel::caption() const{ return QStringLiteral("TIP CHANGING DETACH TASK"); }

QString TipChangingDetachModel::name() const { return QStringLiteral("TIP CHANGING DETACH TASK"); }

NodeDataType TipChangingDetachModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "TIP CHANGING DETACH TASK"};
}

QJsonObject TipChangingDetachModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void TipChangingDetachModel::load(QJsonObject const &p) {KCRCommandModel::load(p);}

void TipChangingDetachModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_taskRecog_TipChangingDetach_jsMove(planner.bp_gui_node_task_);
}

QWidget* TipChangingDetachModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

TipChangingDefaultDetachModel::TipChangingDefaultDetachModel() : KCRCommandModel() {}

QString TipChangingDefaultDetachModel::caption() const{ return QStringLiteral("TIP CHANGING DEFAULT DETACH TASK"); }

QString TipChangingDefaultDetachModel::name() const { return QStringLiteral("TIP CHANGING DEFAULT DETACH TASK"); }

NodeDataType TipChangingDefaultDetachModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "TIP CHANGING DEFAULT DETACH TASK"};
}

QJsonObject TipChangingDefaultDetachModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void TipChangingDefaultDetachModel::load(QJsonObject const &p) {KCRCommandModel::load(p);}

void TipChangingDefaultDetachModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_taskRecog_TipChangingDefaultDetach_jsMove(planner.bp_gui_node_task_);
}

QWidget* TipChangingDefaultDetachModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

TipChangingSetTcpModel::TipChangingSetTcpModel() : KCRCommandModel() {}

QString TipChangingSetTcpModel::caption() const{ return QStringLiteral("TIP CHANGING SET TCP TASK"); }

QString TipChangingSetTcpModel::name() const { return QStringLiteral("TIP CHANGING SET TCP TASK"); }

NodeDataType TipChangingSetTcpModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "TIP CHANGING SET TCP TASK"};
}

QJsonObject TipChangingSetTcpModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void TipChangingSetTcpModel::load(QJsonObject const &p) {KCRCommandModel::load(p);}

void TipChangingSetTcpModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_taskRecog_TipChangingSetTCP(planner.bp_gui_node_task_);
}

QWidget* TipChangingSetTcpModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

InitalizeGripperModel::InitalizeGripperModel() : KCRCommandModel() {}

QString InitalizeGripperModel::caption() const{ return QStringLiteral("INITIALIZE GRIPPER TASK"); }

QString InitalizeGripperModel::name() const { return QStringLiteral("INITIALIZE GRIPPER TASK"); }

NodeDataType InitalizeGripperModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "INITIALIZE GRIPPER TASK"};
}

QJsonObject InitalizeGripperModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void InitalizeGripperModel::load(QJsonObject const &p) {KCRCommandModel::load(p);}

void InitalizeGripperModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_initializeGripper(planner.bp_gui_node_task_);
}

QWidget* InitalizeGripperModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

AprilTagDetectionTaskModel::AprilTagDetectionTaskModel() : KCRCommandModel() {
    tag_number = 0;
}

QString AprilTagDetectionTaskModel::caption() const{ return QStringLiteral("APRIL TAG DETECT"); }

QString AprilTagDetectionTaskModel::name() const { return QStringLiteral("APRIL TAG DETECT"); }

NodeDataType AprilTagDetectionTaskModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "APRIL TAG DETECT"};
}

QJsonObject AprilTagDetectionTaskModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["tag_number"] = tag_number;
    return modelJson;
}

void AprilTagDetectionTaskModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jcommand = p["tag_number"].toInt();
    _lineEditTagNumber->setText(QString::number(jcommand.toInt()));
}

QWidget *AprilTagDetectionTaskModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        _lineEditTagNumber = new QLineEdit("0");
        QObject::connect(_lineEditTagNumber, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_tag_number(text.toInt());
        });
        mainLayout->addWidget(_lineEditTagNumber);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void AprilTagDetectionTaskModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_aprilTagDetection(planner.bp_gui_node_task_, tag_number);
}

MoveToTagTaskModel::MoveToTagTaskModel() : KCRCommandModel() {
    tag_number = 0;
}

QString MoveToTagTaskModel::caption() const{ return QStringLiteral("MOVE TO TAG"); }

QString MoveToTagTaskModel::name() const { return QStringLiteral("MOVE TO TAG"); }

NodeDataType MoveToTagTaskModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "MOVE TO TAG"};
}

QJsonObject MoveToTagTaskModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    modelJson["tag_number"] = tag_number;
    return modelJson;
}

void MoveToTagTaskModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonValue jcommand = p["tag_number"].toInt();
    _lineEditTagNumber->setText(QString::number(jcommand.toInt()));
}

QWidget *MoveToTagTaskModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();
        _lineEditTagNumber = new QLineEdit("0");
        QObject::connect(_lineEditTagNumber, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_tag_number(text.toInt());
        });
        mainLayout->addWidget(_lineEditTagNumber);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}
void MoveToTagTaskModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_moveToTag(planner.bp_gui_node_task_, tag_number);
}

SetTeachingTargetPoseModel::SetTeachingTargetPoseModel() : KCRCommandModel() {
    target_x.resize(6);
    setNodeStyle(nodeStyle());
}

QString SetTeachingTargetPoseModel::caption() const { return QStringLiteral("SET TEACHING TARGET POSE"); }

QString SetTeachingTargetPoseModel::name() const { return QStringLiteral("SET TEACHING TARGET POSE"); }

NodeDataType SetTeachingTargetPoseModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "SET TEACHING TARGET POSE"};
}

QJsonObject SetTeachingTargetPoseModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    printValueType(target_x, modelJson, "target_x");
    modelJson["tag_teaching_pose"] = QString::fromStdString(tag_teaching_pose);
    modelJson["pose_idx_teaching_pose"] = QString::fromStdString(pose_idx_teaching_pose);
    return modelJson;
}
void SetTeachingTargetPoseModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    // QJsonValue jindex = p["task_index"].toInt();
    // lineEditTaskIndex->setText(QString::number(jindex.toInt()));
    QJsonArray jtarget_x = p["target_x"].toArray();
    updateWidgetFromJson(jtarget_x, target_x, lineEdits1);
    QJsonValue jstring_1 = p["tag_teaching_pose"].toString();
    QJsonValue jstring_2 = p["pose_idx_teaching_pose"].toString();

    lineEdits_teaching_pose->setText(jstring_1.toString());
    lineEdits_pose_idx_teaching_pose->setText(jstring_2.toString());
}

QWidget *SetTeachingTargetPoseModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        const int numLineEdits = 6;

        for (int i = 0; i < numLineEdits; ++i)
            {
                QLineEdit *lineEdit = new QLineEdit("0.0");
                lineEdits1.append(lineEdit);

                QObject::connect(lineEdit,
                                    &QLineEdit::textChanged,
                                    this,
                                    [i, this](const QString &text) {
                    set_target_x(text.toDouble(), i);
                });
            }
        QGroupBox *groupBox1 = new QGroupBox("target_x");
        QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);

        for (auto edit : lineEdits1) {
            layout1->addWidget(edit);
        }

        lineEdits_teaching_pose = new QLineEdit("default tag");
        QObject::connect(lineEdits_teaching_pose, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_tag_teaching_pose(text.toStdString());
        });
        layout1->addWidget(lineEdits_teaching_pose);

        lineEdits_pose_idx_teaching_pose = new QLineEdit("default pose idx");
        QObject::connect(lineEdits_pose_idx_teaching_pose, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_pose_idx_teaching_pose(text.toStdString());
        });
        layout1->addWidget(lineEdits_pose_idx_teaching_pose);


        layout1->setContentsMargins(0, 0, 0, 0);
        mainLayout->addWidget(groupBox1);
        QPushButton* updateBtn = new QPushButton("Get current value");

        QObject::connect(updateBtn, &QPushButton::clicked, this, [this]() {
            QNode* ptr = &QNode::getInstance();
            CsDouble arr = ptr->params_.meas.x;
            target_x.resize(6);
            for (size_t i = 0; i < 6; i++)
            {
                lineEdits1[i]->setText(QString::number(arr[i]));
            }

            //// JSON Save
            //// FILE PATH
            // std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/teaching_pose/graphy_demo_cs_waypoint.json";
            std::string pose_config_path = "/home/" + USER_NAME + "/[Pose server]/teaching_pose/graphy_demo_cs_waypoint.json";
            //// JSON INPUT
            std::stringstream ss;
            std::ifstream ifs(pose_config_path.c_str());
            json j_in = json::parse(ifs);
            json j_out = j_in;
            std::vector<double> pose_update(6);
            arr2Vec(arr, pose_update);
            for(int j = 0; j < 3; j++) {
                pose_update[j] = round(pose_update[j]*1e7) / 1e7; // [m]
                pose_update[j + 3] = round(pose_update[j + 3]*1e4) / 1e4; // [deg]
            }
            j_out[get_tag_teaching_pose()][get_pose_idx_teaching_pose()] = pose_update;

            std::ofstream ofs(pose_config_path.c_str());
            ofs << j_out.dump(4) << std::endl;

        });
        mainLayout->addWidget(updateBtn);



        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });

        _widget->setLayout(mainLayout);
    }
    return _widget;
}

void SetTeachingTargetPoseModel::saveGuiTask(TaskPlanner& planner) {
    // CsDouble csArr;
    // vec2Arr(target_x, csArr);
    planner.taskPushBack_setTeachingTargetPose(planner.bp_gui_node_task_, tag_teaching_pose, pose_idx_teaching_pose);
}

void SetTeachingTargetPoseModel::setNodeStyle(NodeStyle const &style)
{
    KCRCommandModel::setNodeStyle(NodeStyle(R"(
        {
            "NodeStyle": {
            "NormalBoundaryColor": "darkgray",
            "SelectedBoundaryColor": "deepskyblue",
            "GradientColor0": "mintcream",
            "GradientColor1": "mintcream",
            "GradientColor2": "mintcream",
            "GradientColor3": "mintcream",
            "ShadowColor": [10, 10, 10],
            "FontColor": [10, 10, 10],
            "FontColorFaded": [10, 10, 10],
            "ConnectionPointColor": "white",
            "PenWidth": 2.0,
            "HoveredPenWidth": 2.5,
            "ConnectionPointDiameter": 10.0,
            "Opacity": 1.0
            }
        }
    )"));
}

CsTargetBaseFrameTeachingTargetPoseModel::CsTargetBaseFrameTeachingTargetPoseModel() : KCRCommandModel() {
    target_x.resize(6);
    acc = 0.0;
    vel = 0.0;
}

QString CsTargetBaseFrameTeachingTargetPoseModel::caption() const { return QStringLiteral("CS TARGET TASK BASE FRAME(Teaching Target Pose)"); }

QString CsTargetBaseFrameTeachingTargetPoseModel::name() const { return QStringLiteral("CS TARGET TASK BASE FRAME(Teaching Target Pose)"); }

NodeDataType CsTargetBaseFrameTeachingTargetPoseModel::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "CS TARGET TASK BASE FRAME(Teaching Target Pose)"};
}

QJsonObject CsTargetBaseFrameTeachingTargetPoseModel::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    printValueType(target_x, modelJson, "target_x");
    modelJson["vel"] = vel;
    modelJson["acc"] = acc;
    return modelJson;
}
void CsTargetBaseFrameTeachingTargetPoseModel::load(QJsonObject const &p) {
    KCRCommandModel::load(p);
    QJsonArray jtarget_x = p["target_x"].toArray();
    updateWidgetFromJson(jtarget_x, target_x, lineEdits1);
    QJsonValue jvel = p["vel"].toDouble();
    QJsonValue jacc = p["acc"].toDouble();
    lineEditsVel->setText(QString::number(jvel.toDouble()));
    lineEditsAcc->setText(QString::number(jacc.toDouble()));
}

QWidget* CsTargetBaseFrameTeachingTargetPoseModel::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QVBoxLayout* mainLayout = new QVBoxLayout();
        const int numLineEdits = 6;

        for (int i = 0; i < numLineEdits; ++i)
            {
                QLineEdit *lineEdit = new QLineEdit("0.0");
                lineEdits1.append(lineEdit);
                QObject::connect(lineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
                    set_target_x(text.toDouble(), i);
                });
            }
        QGroupBox *groupBox1 = new QGroupBox("target_x");
        QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);
        for (auto edit : lineEdits1) {
            layout1->addWidget(edit);
        }
        lineEditsVel = new QLineEdit("0.0");
        lineEditsAcc = new QLineEdit("0.0");
        QObject::connect(lineEditsVel, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_vel(text.toDouble());
        });
        QObject::connect(lineEditsAcc, &QLineEdit::textChanged, this, [this](const QString& text) {
            set_acc(text.toDouble());
        });
        QLabel *l_vel = new QLabel("&Velocity(m/s):");
        QLabel *l_acc = new QLabel("&Acceleration(m/s2):");
        l_acc->setBuddy(lineEditsAcc);
        l_vel->setBuddy(lineEditsVel);
        layout1->addWidget(l_vel);
        layout1->addWidget(lineEditsVel);
        layout1->addWidget(l_acc);
        layout1->addWidget(lineEditsAcc);

        QPushButton* updateBtn = new QPushButton("Get current value");

        QObject::connect(updateBtn, &QPushButton::clicked, this, [this]() {
            QNode* ptr = &QNode::getInstance();
            CsDouble arr = ptr->params_.meas.x;
            target_x.resize(6);
            for (size_t i = 0; i < 6; i++)
            {
                lineEdits1[i]->setText(QString::number(arr[i]));
            }

        });
        layout1->addWidget(updateBtn);
        layout1->setContentsMargins(0, 0, 0, 0);

        mainLayout->addWidget(groupBox1);

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });

        _widget->setLayout(mainLayout);

    }
    return _widget;
}
void CsTargetBaseFrameTeachingTargetPoseModel::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_csMoveBaseFrameTeachingTargetPose(planner.bp_gui_node_task_, target_x, acc, vel);
}

// void CsTargetBaseFrameTeachingTargetPoseModel::setNodeStyle(NodeStyle const &style)
// {
//     KCRCommandModel::setNodeStyle(NodeStyle(R"(
//         {
//             "NodeStyle": {
//             "NormalBoundaryColor": "darkgray",
//             "SelectedBoundaryColor": "deepskyblue",
//             "GradientColor0": "mintcream",
//             "GradientColor1": "mintcream",
//             "GradientColor2": "mintcream",
//             "GradientColor3": "mintcream",
//             "ShadowColor": [10, 10, 10],
//             "FontColor": [10, 10, 10],
//             "FontColorFaded": [10, 10, 10],
//             "ConnectionPointColor": "white",
//             "PenWidth": 2.0,
//             "HoveredPenWidth": 2.5,
//             "ConnectionPointDiameter": 10.0,
//             "Opacity": 1.0
//             }
//         }
//     )"));
// }


RobotEnable::RobotEnable() : KCRCommandModel() {
}

QString RobotEnable::caption() const{ return QStringLiteral("ROBOT ENABLE"); }

QString RobotEnable::name() const { return QStringLiteral("ROBOT ENABLE"); }


NodeDataType RobotEnable::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "ROBOT ENABLE"};
}

QJsonObject RobotEnable::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void RobotEnable::load(QJsonObject const &p) { KCRCommandModel::load(p);}


//bool RobotEnable::expandable() const { return false; }

QWidget* RobotEnable::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

void RobotEnable::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_robotEnable(planner.bp_gui_node_task_);
}


RobotDisable::RobotDisable() : KCRCommandModel() {
}

QString RobotDisable::caption() const{ return QStringLiteral("ROBOT DISABLE"); }

QString RobotDisable::name() const { return QStringLiteral("ROBOT DISABLE"); }


NodeDataType RobotDisable::dataType(PortType portType, PortIndex portIndex) const {
    return NodeDataType{"robotCommand", "ROBOT DISABLE"};
}

QJsonObject RobotDisable::save() const {
    QJsonObject modelJson = KCRCommandModel::save();
    return modelJson;
}

void RobotDisable::load(QJsonObject const &p) { KCRCommandModel::load(p);}


//bool RobotDisable::expandable() const { return false; }

QWidget* RobotDisable::embeddedWidget() {
    if(!_widget) {
        _widget = new QWidget();
        QHBoxLayout* mainLayout = new QHBoxLayout();

        QLabel *indexLabel = new QLabel("Task index:");
        mainLayout->addWidget(indexLabel);
        mainLayout->addWidget(lineEditTaskIndex);
        QObject::connect(lineEditTaskIndex, &QLineEdit::textChanged, this, [this](const QString& text) {
            KCRCommandModel::setTaskIndex(text.toInt());
            for (const auto& pair : index_color_list) {
                // Check if the int value is 1
                if (pair.first == _task_index) {
                    // Output the corresponding QString value
                    setNodeStyle(NodeStyle(pair.second));
                    break; // Optional: break out of the loop if you only need the first occurrence
                }
            }
        });
        _widget->setLayout(mainLayout);
    }
    return _widget;
}

void RobotDisable::saveGuiTask(TaskPlanner& planner) {
    planner.taskPushBack_robotDisable(planner.bp_gui_node_task_);
}
