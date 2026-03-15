#ifndef KCRCOMMANDMODEL_HPP_
#define KCRCOMMANDMODEL_HPP_

#include <QtNodes/Definitions>
#include <QtNodes/NodeDelegateModel>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>
#include <QtCore/QStringList>
#include <QtCore/QMap>

#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QComboBox>

#include <QtCore/QModelIndex>

#include "protocolDefine.hpp"
#include "taskManager.hpp"
#include "task_planner_default.hpp"


using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;
using QtNodes::NodeStyle;

class DoubleData;

template<typename T>
QJsonObject printValueType(T a, QJsonObject &modelJson, QString name)
{
    QJsonArray jsonArray;
    for (const auto &value : a) {
        jsonArray.append(value);
    }
    modelJson[name] = jsonArray;
    return modelJson;
}

//append w.r.t class member data type ex) std::vector<double> target_x and bool, double type
template<typename MemberDataType>
QWidget* appendMemberToWidgets(QWidget* widget, MemberDataType data) {
    QLineEdit* a = new QLineEdit("0.0");
    QGroupBox* groupBox = new QGroupBox("groupBox");

    QVBoxLayout *vbox = new QVBoxLayout;
    vbox->addWidget(a);
    groupBox->setLayout(vbox);

}

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class KCRCommandModel : public NodeDelegateModel
{
    Q_OBJECT
public:

    KCRCommandModel();
    ~KCRCommandModel() = default;
public:
    unsigned int nPorts(PortType portType) const override;

    bool portCaptionVisible(PortType portType, PortIndex portIndex) const override
    {
        Q_UNUSED(portType);
        Q_UNUSED(portIndex);
        return true;
    }
    std::shared_ptr<NodeData> outData(PortIndex port) override;

    void setInData(std::shared_ptr<NodeData> data, PortIndex portIndex) override;

    virtual QWidget *embeddedWidget() = 0; //override { return nullptr; }

    virtual QString caption() const = 0;

    virtual QString name() const = 0;

    virtual NodeDataType dataType(PortType portType, PortIndex portIndex) const = 0; 

//    void setWidgetExpand(bool flag) override {
//        _widget->setVisible(flag);
//    }
    QJsonObject save() const override;

    void load(QJsonObject const &p) override;

    void setWidgetExpand(bool flag) override;

    void updateWidgetFromJson(QJsonArray& jarr, std::vector<double>& vec, QVector<QLineEdit*>& lineEdits);

    virtual void saveGuiTask(TaskPlanner& planner) = 0;

    void setNodeStyle(NodeStyle const &style) override;

    void setTaskIndex(const int& task_index);
            
protected:
    std::weak_ptr<DoubleData> _number1;
    std::weak_ptr<DoubleData> _number2;
    std::shared_ptr<DoubleData> _result;

    int _task_index;
    static std::list<std::pair<int, QString>> index_color_list;
    QLineEdit* lineEditTaskIndex;

    QWidget* _widget;
};

//using vec_d = std::vector<double>;
//#define PROPERTY(type, name) 
//    Q_PROPERTY(type name READ get_##name WRITE set_##name NOTIFY name##_Changed)

class JsTargetModel : public KCRCommandModel {

    Q_OBJECT

public:
    std::vector<double> target_q;
    QVector<QLineEdit*> lineEdits1;

    double acc;
    double vel;
    bool is_relative;
    QLineEdit* lineEditsAcc;
    QLineEdit* lineEditsVel; 
    QCheckBox* is_relative_checkbox;

Q_SIGNALS:
    void target_q_Changed(const double& value, uint i);

public Q_SLOTS:
    void set_target_q(const double& value, uint i) { target_q[i] = value; }//emit target_q_Changed(value, i); }
    void set_acc(const double& value) { acc = value; }
    void set_vel(const double& value) { vel = value; }
    void set_is_relative(const bool value) { is_relative = value; };

public:
    std::vector<double> get_target_q() const { return target_q; }

    JsTargetModel();
    ~JsTargetModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;

    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;

    void saveGuiTask(TaskPlanner& planner) override;

    //void setNodeStyle(NodeStyle const &style) override;
};

class CsTargetModel : public KCRCommandModel {

    Q_OBJECT

public:

    std::vector<double> target_x;
    QVector<QLineEdit*> lineEdits1;

    double acc;
    double vel;
    bool is_relative;
    QLineEdit* lineEditsAcc;
    QLineEdit* lineEditsVel; 
    QCheckBox* is_relative_checkbox;

public Q_SLOTS:
    void set_target_x(const double& value, uint i) { target_x[i] = value; }
    void set_acc(const double& value) { acc = value; }
    void set_vel(const double& value) { vel = value; }
    void set_is_relative(const bool value) { is_relative = value; };
public:

    CsTargetModel();
    ~CsTargetModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;

    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    
    void saveGuiTask(TaskPlanner& planner) override;
};

class CsTargetTaskRecogModel : public KCRCommandModel {

    Q_OBJECT

private:

    std::vector<double> target_x;
    QVector<QLineEdit*> lineEdits1;

    QMap<QString, BPDemoType>* demo_type_map;
    QMap<QString, MotionTag>* motion_tag_map;
    QComboBox * _comboBoxDemoType;
    QComboBox * _comboBoxMotionTag;

    bool is_relative;
    QCheckBox* is_relative_checkbox;

    double acc;
    double vel;
    QLineEdit* lineEditsAcc;
    QLineEdit* lineEditsVel; 


public Q_SLOTS:
    void set_target_x(const double& value, uint i) { target_x[i] = value; }
    void set_is_relative(const bool value) { is_relative = value; };
    void set_acc(const double& value) { acc = value; }
    void set_vel(const double& value) { vel = value; }
public:

    CsTargetTaskRecogModel();
    ~CsTargetTaskRecogModel() {}

    QString caption() const override;
    QString name() const override;
    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class CsTargetToolFrameModel : public KCRCommandModel {

    Q_OBJECT

public:
    std::vector<double> target_x;
    QVector<QLineEdit*> lineEdits1;

    double acc;
    double vel;
    QLineEdit* lineEditsAcc;
    QLineEdit* lineEditsVel; 

public Q_SLOTS:
    void set_target_x(const double& value, uint i) { target_x[i] = value; }
    void set_acc(const double& value) { acc = value; }
    void set_vel(const double& value) { vel = value; }
public:

    CsTargetToolFrameModel();
    ~CsTargetToolFrameModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class CsTargetRedundantModel : public KCRCommandModel {

    Q_OBJECT

private:
    std::vector<double> target_x;
    QVector<QLineEdit*> lineEdits1;

    double q_redundant;
    double vel;
    double acc;
    QLineEdit* lineEditRedundant;
    QLineEdit* lineEditsVel; 
    QLineEdit* lineEditsAcc;

public Q_SLOTS:
    void set_target_x(const double& value, uint i) { target_x[i] = value; }
    void set_acc(const double& value) { acc = value; }
    void set_vel(const double& value) { vel = value; }
    void set_q_redundant(const double& value) { q_redundant = value; }
public:

    CsTargetRedundantModel();
    ~CsTargetRedundantModel() {}

    QString caption() const override;
    QString name() const override;
    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class MoveBlendingModel : public KCRCommandModel {

    Q_OBJECT

private:
    BlendingTraj _traj;

    QVBoxLayout *layout_group_box;
    QVector<QVector<QLineEdit*>> xs_lineEdits;
    QVector<QLineEdit*> xds_lineEdits;
    QVector<QLineEdit*> xdds_lineEdits;
    QVector<QLineEdit*> radiuses_lineEdits;
    QVector<QLineEdit*> waypoint_xds_lineEdits;
    QVector<QLineEdit*> qs_redundant_lineEdits;

    QLineEdit* waypoint_num_lineEdit;
    QCheckBox* is_radius_percent_checkbox;
    
    // QPushButton* addButton;
    // QPushButton* removeButton;

    QTableWidget* _table_widget;
    QModelIndex* _model_index;

Q_SIGNALS:
    void xs_Changed(const double& value, uint i, uint j);
    void xds_Changed(const double& value, uint i);
    void xdds_Changed(const double& value, uint i);
    void radiuses_Changed(const double& value, uint i);
    void waypoint_xds_Changed(const double& value, uint i);
    void qs_redundant_Changed(const double& value, uint i);
    void waypoint_num_Changed(const int value);
    void is_radius_percent_Changed(const bool value);

public Q_SLOTS:
    void set_xs(const double& value, uint i, uint j) { _traj.xs[i][j] = value; };

    void set_xds(const double& value, uint i) { _traj.xds[i] = value; };
    void set_xdds(const double& value, uint i) { _traj.xdds[i] = value; };
    void set_radiuses(const double& value, uint i) { _traj.radiuses[i] = value; };
    void set_waypoint_xds(const double& value, uint i) { _traj.waypoint_xds[i] = value; };
    void set_qs_redundant(const double& value, uint i) { _traj.qs_redundant[i] = value; };

    void set_waypoint_num(const int value) { _traj.waypoint_num = value; };
    void set_is_radius_percent(const bool value) { _traj.is_radius_percent = value; };
    
public:
    MoveBlendingModel();
    ~MoveBlendingModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class TCPMoveModel : public KCRCommandModel {

    Q_OBJECT

private:
    std::vector<double> target_x;
    QVector<QLineEdit*> lineEdits1;

public Q_SLOTS:
    void set_target_x(const double& value, uint i) { target_x[i] = value; }

public:

    TCPMoveModel();
    ~TCPMoveModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class DelayModel : public KCRCommandModel {

    Q_OBJECT

public:
    int _delayTime;
    QLineEdit * _lineEditDelay;

public Q_SLOTS:

public:

    DelayModel();
    ~DelayModel() {}

Q_SIGNALS:
    void timeDelay_Changed(const int& value);

public Q_SLOTS:
    void set_time_delay(const int& value) { _delayTime = value; }

public:
    int get_time_delay() const { return _delayTime; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class PauseModel : public KCRCommandModel {

    Q_OBJECT

Q_SIGNALS:

public:

    PauseModel();
    ~PauseModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class SavePointModel : public KCRCommandModel {

    Q_OBJECT

public:

    SavePointModel();
    ~SavePointModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class RewindModel : public KCRCommandModel {

    Q_OBJECT

public:

    RewindModel();
    ~RewindModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};
class CSAccVelUpdateModel : public KCRCommandModel {

    Q_OBJECT

private:

    double acc;
    double vel;

    QLineEdit* lineEditsAcc;
    QLineEdit* lineEditsVel; 


public Q_SLOTS:
    void set_acc(const double& value) { acc = value; }
    void set_vel(const double& value) { vel = value; }
public:

    CSAccVelUpdateModel();
    ~CSAccVelUpdateModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

// class ImpedanceOnModel : public KCRCommandModel {

//     Q_OBJECT

// private:
//     std::vector<double> _imp_stiffness;
//     QVector<QLineEdit*> lineEdits;   

// Q_SIGNALS:
//     void imp_stiffness_Changed(const double& value, uint i);

// public Q_SLOTS:
//     void set_imp_stiffness(const double& value, uint i) { _imp_stiffness[i] = value; }

// public:

//     ImpedanceOnModel() : KCRCommandModel() {}
//     ~ImpedanceOnModel() {}

//     QString caption() const override{ return QStringLiteral("IMPEDANCE ON TASK"); }

//     QString name() const override { return QStringLiteral("IMPEDANCE ON TASK"); }

//     NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
//         return NodeDataType{"robotCommand", "IMPEDANCE ON TASK"};
//     }

//     QJsonObject save() const override {
//         QJsonObject modelJson = KCRCommandModel::save();        
//         printValueType(_imp_stiffness, modelJson, "imp_stiffness");
//         return modelJson;
//     }

//     void load(QJsonObject const &p) override {
//         QJsonArray jimp_stiffness = p["imp_stiffness"].toArray();
//         updateWidgetFromJson(jimp_stiffness, _imp_stiffness, lineEdits);
//     }

//     QWidget *embeddedWidget() override {
//         if(!_widget) {
//             _widget = new QWidget();
//             QHBoxLayout* mainLayout = new QHBoxLayout();
//             const int numLineEdits = 6;

//             for (int i = 0; i < numLineEdits; ++i)
//                 {
//                     QLineEdit *lineEdit = new QLineEdit("0.0");

//                     lineEdits.append(lineEdit);
//                     QObject::connect(lineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
//                         set_imp_stiffness(text.toDouble(), i);
//                     });
//                 }
//             QGroupBox *groupBox1 = new QGroupBox("grp_imp_stiffness");
//             QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);
//             for (auto edit : lineEdits) {
//                 layout1->addWidget(edit);
//             }

//             layout1->setContentsMargins(0, 0, 0, 0);
//             mainLayout->addWidget(groupBox1);
//             _widget->setLayout(mainLayout);
//         }
//         return _widget;
//     }

//     void saveGuiTask(TaskPlanner& planner) override {
//         CsDouble impedanceStiffness;
//         vec2Arr(_imp_stiffness, impedanceStiffness);
//         planner.taskPushBack_impedance_on(planner.bp_gui_node_task_, impedanceStiffness);  
//     }
// };

// class ImpedanceOffModel : public KCRCommandModel {

//     Q_OBJECT

// public:

//     ImpedanceOffModel() : KCRCommandModel() {
//     }
//     ~ImpedanceOffModel() {}

//     QString caption() const override{ return QStringLiteral("IMPEDANCE OFF TASK"); }

//     QString name() const override { return QStringLiteral("IMPEDANCE OFF TASK"); }

 
//     NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
//         return NodeDataType{"robotCommand", "IMPEDANCE OFF TASK"};
//     }

//     QJsonObject save() const override {
//         QJsonObject modelJson = KCRCommandModel::save();        
//         return modelJson;
//     }

//     void load(QJsonObject const &p) override {}

//     QWidget *embeddedWidget() override {
//         return nullptr;
//     }
//     //bool expandable() const override { return false; } 

//     void saveGuiTask(TaskPlanner& planner) override {
//         planner.taskPushBack_impedance_off(planner.bp_gui_node_task_);  
//     }
// };
class IrlGrpModel : public KCRCommandModel {
    
    Q_OBJECT

public:
// enum class KR_GRP {
//     INIT       = 101,
//     OPEN       = 102,
//     CLOSE      = 103,
//     POS_CTRL   = 104,
//     INIT_2     = 105,
//     VACUUM_ON  = 106,
//     VACUUM_OFF = 107,
// };
    QMap<QString, KR_GRP>* grp_cmd_map;
    int value_; 
    int address_;
    QComboBox * _comboBoxGrpCmd;
    QLineEdit * _lineEditValue;
    QLineEdit * _lineEditAddress;

public Q_SLOTS:
    // void set_grp_cmd(const uint16_t& value) { grp_cmd = static_cast<KR_GRP>(value); }
    // void set_grp_cmd(const KR_GRP& value) { grp_cmd = value; }
    void set_value(const int& value) { value_ = value; }
    void set_address(const int& value) { address_ = value; }

public:

    IrlGrpModel();
    ~IrlGrpModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class IrlGrpCmdModel : public KCRCommandModel {

    Q_OBJECT

private:
    int _grp_pos_percent;
    QLineEdit * _lineEditGrpPos;

public Q_SLOTS:

public:

    IrlGrpCmdModel();
    ~IrlGrpCmdModel() {}

Q_SIGNALS:
    void posPercent_Changed(const int& value);

public Q_SLOTS:
    void set_pos_percent(const int& value) { _grp_pos_percent = value; }

public:
    int get_pos_percent() const { return _grp_pos_percent; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class IrlGrpInitModel : public KCRCommandModel {

    Q_OBJECT

private:
    int _grp_pos_percent;
    QLineEdit * _lineEditGrpPos;

public Q_SLOTS:

public:

    IrlGrpInitModel();
    ~IrlGrpInitModel() {}

Q_SIGNALS:
    void posPercent_Changed(const int& value);

public Q_SLOTS:
    void set_pos_percent(const int& value) { _grp_pos_percent = value; }//emit target_q_Changed(value, i); }

public:
    int get_pos_percent() const { return _grp_pos_percent; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class BP_PLCModel : public KCRCommandModel {

    Q_OBJECT

private:
    int _command;
    QLineEdit * _lineEditPlcCommand;

public Q_SLOTS:

public:

    BP_PLCModel();
    ~BP_PLCModel() {}

Q_SIGNALS:
    void timeDelay_Changed(const int& value);

public Q_SLOTS:
    void set_command(const int& value) { _command = value; }

public:
    int get_command() const { return _command; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class BP_GripperRelayCmdModel : public KCRCommandModel {

    Q_OBJECT

private:
    RelayCommand _command;
    QComboBox* cbox;

public Q_SLOTS:

public:

    BP_GripperRelayCmdModel();
    ~BP_GripperRelayCmdModel() {}

Q_SIGNALS:
    void relay_Changed(const int& value);

public Q_SLOTS:
    void set_relay(const int& value) { _command = static_cast<RelayCommand>(value); }

public:
    //int get_time_delay() const { return _command; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class SetTCPModel : public KCRCommandModel {

    Q_OBJECT

public:
    std::vector<double> target_x;
    QVector<QLineEdit*> lineEdits1;

public Q_SLOTS:
    void set_target_x(const double& value, uint i) { target_x[i] = value; }
public:

    SetTCPModel();
    ~SetTCPModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};


class BP_SelectTCPModel : public KCRCommandModel {

    Q_OBJECT

private:
    int tcp_idx;
    QLineEdit * _lineEditTcpIndex;
// Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6)
public Q_SLOTS:

public:

    BP_SelectTCPModel();
    ~BP_SelectTCPModel() {}

public Q_SLOTS:
    void set_tcp_idx(const int& value) { tcp_idx = value; }

public:
    int get_tcp_idx() const { return tcp_idx; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class BP_setGripperDriverModel : public KCRCommandModel {

    Q_OBJECT

private:
    int grp_driver_idx;
    QLineEdit * _lineEditDriver;
// Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6)
public Q_SLOTS:

public:

    BP_setGripperDriverModel();
    ~BP_setGripperDriverModel() {}

public Q_SLOTS:
    void set_grp_driver_idx(const int& value) { grp_driver_idx = value; }

public:
    int get_grp_driver_idx() const { return grp_driver_idx; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class BP_Recog_KORASGripperModel : public KCRCommandModel {

    Q_OBJECT

public:

    BP_Recog_KORASGripperModel();
    ~BP_Recog_KORASGripperModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class BP_KORASGripperModel : public KCRCommandModel {

    Q_OBJECT

public:
// enum class KR_GRP {
//     INIT       = 101,
//     OPEN       = 102,
//     CLOSE      = 103,
//     POS_CTRL   = 104,
//     INIT_2     = 105,
//     VACUUM_ON  = 106,
//     VACUUM_OFF = 107,
// };
    QMap<QString, KR_GRP>* grp_cmd_map;
    int position; 
    int speed;
    QComboBox * _comboBoxGrpCmd;
    QLineEdit * _lineEditPosition;
    QLineEdit * _lineEditSpeed;

public Q_SLOTS:
    // void set_grp_cmd(const uint16_t& value) { grp_cmd = static_cast<KR_GRP>(value); }
    // void set_grp_cmd(const KR_GRP& value) { grp_cmd = value; }
    void set_position(const int& value) { position = value; }
    void set_speed(const int& value) { speed = value; }

public:

    BP_KORASGripperModel();
    ~BP_KORASGripperModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class BP_3DScanModel : public KCRCommandModel {

    Q_OBJECT

public:
    BP_3DScanModel();
    ~BP_3DScanModel() {}
public:

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
    
};    

//include checking matched task
class BP_TemplateMatchingModel : public KCRCommandModel {

    Q_OBJECT

private: 

    bool do_sampling; 
    unsigned int sampling_num;

    QCheckBox* do_sampling_checkbox;
    QLineEdit* sampling_num_lineEdit;

public:

    void set_do_sampling(const bool value) { do_sampling = value; };
    void set_sampling_num(const unsigned int value) { sampling_num = value; };
    
    BP_TemplateMatchingModel();
    ~BP_TemplateMatchingModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

/// @brief on task planner used(for ROS2 topic) instead of scan model and match model (for service)
class BP_ScanTemplateMatchingModel : public KCRCommandModel {

    Q_OBJECT

private: 

    bool do_sampling; 
    unsigned int sampling_num;

    QCheckBox* do_sampling_checkbox;
    QLineEdit* sampling_num_lineEdit;

public:

    void set_do_sampling(const bool value) { do_sampling = value; };
    void set_sampling_num(const unsigned int value) { sampling_num = value; };
    
    BP_ScanTemplateMatchingModel();
    ~BP_ScanTemplateMatchingModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class BP_ScanningDelayModel : public KCRCommandModel {

    Q_OBJECT

private:
    int _delayTime;
    QLineEdit * _lineEditDelay;

public Q_SLOTS:

public:

    BP_ScanningDelayModel();
    ~BP_ScanningDelayModel() {}

Q_SIGNALS:
    void timeDelay_Changed(const int& value);

public Q_SLOTS:
    void set_time_delay(const int& value) { _delayTime = value; }

public:
    int get_time_delay() const { return _delayTime; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};
class BP_DoGraspingModel : public KCRCommandModel {

    Q_OBJECT

private:
    double vel;
    double acc;
    QLineEdit* lineEditsVel; 
    QLineEdit* lineEditsAcc;

public Q_SLOTS:

public:

    BP_DoGraspingModel();
    ~BP_DoGraspingModel() {}

public Q_SLOTS:
    void set_acc(const double& value) { acc = value; }
    void set_vel(const double& value) { vel = value; }

public:

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class BP_DoSubGraspingModel : public KCRCommandModel {

    Q_OBJECT

private:
    double vel;
    double acc;
    QLineEdit* lineEditsVel; 
    QLineEdit* lineEditsAcc;

public Q_SLOTS:

public:

    BP_DoSubGraspingModel();
    ~BP_DoSubGraspingModel() {}

public Q_SLOTS:
    void set_acc(const double& value) { acc = value; }
    void set_vel(const double& value) { vel = value; }

public:

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class BP_CountDetachingPose : public KCRCommandModel {

    Q_OBJECT

public:

    BP_CountDetachingPose();
    ~BP_CountDetachingPose() {}

    QString caption() const override;
    QString name() const override;
 
    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;
    QJsonObject save() const override;

    void load(QJsonObject const &p) override;

    //bool expandable() const override { return false; } 
    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class StackingTaskModel : public KCRCommandModel {

    Q_OBJECT

private:

    QMap<QString, BPDemoType>* demo_type_map;
    QComboBox * _comboBoxDemoType;

    bool is_slow_mode;
    QCheckBox* is_slow_mode_checkbox;

public Q_SLOTS:
    void set_is_slow_mode(const bool value) { is_slow_mode = value; };

public:

    StackingTaskModel();
    ~StackingTaskModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class TaskConfirmModel : public KCRCommandModel {

    Q_OBJECT

private:
    QString message;
    QLineEdit* lineEditsMessage;
    QMessageBox box;

public Q_SLOTS:
    void set_message(const QString value) { message = value; };
    QString get_message() {return message; }
    //void set_is_slow_mode(const bool value) { is_slow_mode = value; };

public:

    TaskConfirmModel();
    ~TaskConfirmModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class SetTipIndexTaskModel : public KCRCommandModel {

    Q_OBJECT

private:
    int tip_index;
    QLineEdit * _lineEditTipIndex;
// Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6)
public Q_SLOTS:

public:

    SetTipIndexTaskModel();
    ~SetTipIndexTaskModel() {}

public Q_SLOTS:
    void set_tip_index(const int& value) { tip_index = value; }

public:
    int get_tip_index() const { return tip_index; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class TipChangingAttachModel : public KCRCommandModel {

    Q_OBJECT

public:
    TipChangingAttachModel();
    ~TipChangingAttachModel() {}
public:

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};    

class TipChangingDefaultAttachModel : public KCRCommandModel {

    Q_OBJECT

public:
    TipChangingDefaultAttachModel();
    ~TipChangingDefaultAttachModel() {}
public:

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};    

class TipChangingDetachModel : public KCRCommandModel {

    Q_OBJECT

public:
    TipChangingDetachModel();
    ~TipChangingDetachModel() {}
public:

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
}; 

class TipChangingDefaultDetachModel : public KCRCommandModel {
    Q_OBJECT
public:
    TipChangingDefaultDetachModel();
    ~TipChangingDefaultDetachModel() {}
public:

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
}; 

class TipChangingSetTcpModel : public KCRCommandModel {
    Q_OBJECT
public:
    TipChangingSetTcpModel();
    ~TipChangingSetTcpModel() {}
public:

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
}; 

class InitalizeGripperModel : public KCRCommandModel {
    Q_OBJECT
public:
    InitalizeGripperModel();
    ~InitalizeGripperModel() {}
public:

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
}; 

class AprilTagDetectionTaskModel : public KCRCommandModel {

    Q_OBJECT

private:
    int tag_number;
    QLineEdit * _lineEditTagNumber;
// Set TCP (#1~6), single - (#1) // // dual - left(#5), right(#6)
public Q_SLOTS:

public:

    AprilTagDetectionTaskModel();
    ~AprilTagDetectionTaskModel() {}

public Q_SLOTS:
    void set_tag_number(const int& value) { tag_number = value; }

public:
    int get_tag_number() const { return tag_number; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class MoveToTagTaskModel : public KCRCommandModel {

    Q_OBJECT

private:
    int tag_number;
    QLineEdit * _lineEditTagNumber;
public Q_SLOTS:

public:

    MoveToTagTaskModel();
    ~MoveToTagTaskModel() {}

public Q_SLOTS:
    void set_tag_number(const int& value) { tag_number = value; }

public:
    int get_tag_number() const { return tag_number; }

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;
    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};
class SetTeachingTargetPoseModel : public KCRCommandModel {

    Q_OBJECT

public:
    std::vector<double> target_x;
    QVector<QLineEdit*> lineEdits1;
    std::string tag_teaching_pose;
    std::string pose_idx_teaching_pose;
    QLineEdit* lineEdits_teaching_pose;
    QLineEdit* lineEdits_pose_idx_teaching_pose;

Q_SIGNALS:
    void target_x_Changed(const double& value, uint i);

public Q_SLOTS:
    void set_tag_teaching_pose(std::string value) { tag_teaching_pose = value; }
    void set_pose_idx_teaching_pose(std::string value) { pose_idx_teaching_pose = value; }
    void set_target_x(const double& value, uint i) { target_x[i] = value; }//emit target_q_Changed(value, i); }

public:
    std::vector<double> get_target_x() const { return target_x; }
    std::string get_tag_teaching_pose() const { return tag_teaching_pose; }
    std::string get_pose_idx_teaching_pose() const { return pose_idx_teaching_pose; }

    SetTeachingTargetPoseModel();
    ~SetTeachingTargetPoseModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;

    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;

    void saveGuiTask(TaskPlanner& planner) override;

    void setNodeStyle(NodeStyle const &style) override;
};

class CsTargetBaseFrameTeachingTargetPoseModel : public KCRCommandModel {

    Q_OBJECT

public:

    std::vector<double> target_x;
    QVector<QLineEdit*> lineEdits1;

    double acc;
    double vel;
    QLineEdit* lineEditsAcc;
    QLineEdit* lineEditsVel;

public Q_SLOTS:
    void set_target_x(const double& value, uint i) { target_x[i] = value; }
    void set_acc(const double& value) { acc = value; }
    void set_vel(const double& value) { vel = value; }
public:

    CsTargetBaseFrameTeachingTargetPoseModel();
    ~CsTargetBaseFrameTeachingTargetPoseModel() {}

    QString caption() const override;

    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    QJsonObject save() const override;

    void load(QJsonObject const &p) override;

    QWidget *embeddedWidget() override;

    void saveGuiTask(TaskPlanner& planner) override;
};


class RobotEnable : public KCRCommandModel {

    Q_OBJECT

public:

    RobotEnable();
    ~RobotEnable() {}

    QString caption() const override;
    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;
    QJsonObject save() const override;

    void load(QJsonObject const &p) override;

    //bool expandable() const override { return false; }
    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};

class RobotDisable : public KCRCommandModel {

    Q_OBJECT

public:

    RobotDisable();
    ~RobotDisable() {}

    QString caption() const override;
    QString name() const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;
    QJsonObject save() const override;

    void load(QJsonObject const &p) override;

    //bool expandable() const override { return false; }
    QWidget *embeddedWidget() override;
    void saveGuiTask(TaskPlanner& planner) override;
};


#endif