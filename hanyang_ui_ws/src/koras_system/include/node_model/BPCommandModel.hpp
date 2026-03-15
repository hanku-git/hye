#ifndef BPCOMMANDMODEL_HPP_
#define BPCOMMANDMODEL_HPP_

#include <QtNodes/Definitions>
#include <QtNodes/NodeDelegateModel>
#include <QtCore/QJsonObject>
#include <QtCore/QJsonArray>

#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QRadioButton>
#include <QtCore/QModelIndex>

#include "protocolDefine.hpp"
#include "taskManager.hpp"

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

class DoubleData;

// template<typename T>
// QJsonObject printValueType(T a, QJsonObject &modelJson, QString name)
// {
//     QJsonArray jsonArray;
//     for (const auto &value : a) {
//         jsonArray.append(value);
//     }
//     modelJson[name] = jsonArray;
//     return modelJson;
// }

// //append w.r.t class member data type ex) std::vector<double> target_x and bool, double type
// template<typename MemberDataType>
// QWidget* appendMemberToWidgets(QWidget* widget, MemberDataType data) {
//     QLineEdit* a = new QLineEdit("0.0");
//     QGroupBox* groupBox = new QGroupBox("groupBox");

//     QVBoxLayout *vbox = new QVBoxLayout;
//     vbox->addWidget(a);
//     groupBox->setLayout(vbox);

// }

class BPCommandModel : public NodeDelegateModel
{
    Q_OBJECT
public:

    BPCommandModel() : _widget(nullptr) {
    }
    ~BPCommandModel() = default;
public:
    unsigned int nPorts(PortType portType) const override;

    virtual bool portCaptionVisible(PortType portType, PortIndex portIndex) const override
    {
        Q_UNUSED(portType);
        Q_UNUSED(portIndex);
        return true;
    }
    std::shared_ptr<NodeData> outData(PortIndex port) override;

    void setInData(std::shared_ptr<NodeData> data, PortIndex portIndex) override;


    QJsonObject save() const override;

    void setWidgetExpand(bool flag) override;

    void updateWidgetFromJson(QJsonArray& jarr, std::vector<double>& vec, QVector<QLineEdit*>& lineEdits);
   
    virtual QWidget *embeddedWidget() = 0; //override { return nullptr; }

    virtual QString caption() const = 0;

    virtual QString name() const = 0;

    virtual NodeDataType dataType(PortType portType, PortIndex portIndex) const = 0; 

    virtual void saveGuiTask(TaskPlanner& planner) = 0;
            
protected:
    std::weak_ptr<DoubleData> _number1;
    std::weak_ptr<DoubleData> _number2;
    std::shared_ptr<DoubleData> _result;

    QWidget* _widget;
};

class ScanModel : public BPCommandModel {

    Q_OBJECT

private:
    std::vector<bool> target_;
    QVector<QRadioButton *> rd_buttons;
    taskScanningParameter scan_parameter;
public:
    ScanModel() : BPCommandModel() {

    }
    ~ScanModel() {}

    QString caption() const override { return QStringLiteral("SCANNING"); }

    QString name() const override { return QStringLiteral("SCANNING"); }

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
        return NodeDataType{"robotCommand", "SCANNING TASK"};
    }

    QJsonObject save() const override {
        QJsonObject modelJson = BPCommandModel::save();
        // printValueType(target_q, modelJson, "target_q");

        // modelJson["target_qd"] = target_qd[0];
        // modelJson["target_qdd"] = target_qdd[0];
        return modelJson;
    }
    void load(QJsonObject const &p) override {
        // QJsonArray jtarget_q = p["target_q"].toArray();
        // QJsonValue jtarget_qd1 = p["target_qd"].toDouble();
        // QJsonValue jtarget_qdd1 = p["target_qdd"].toDouble();
        // lineEdits2[0]->setText(QString::number(jtarget_qd1.toDouble()));
        // lineEdits3[0]->setText(QString::number(jtarget_qdd1.toDouble()));
    }

    QWidget *embeddedWidget() override {
        if(!_widget) {
            _widget = new QWidget();
            QHBoxLayout* mainLayout = new QHBoxLayout();
           
            rd_buttons.push_back(new QRadioButton("Calibration tool"));
            rd_buttons.push_back(new QRadioButton("Gear tool"));
            rd_buttons.push_back(new QRadioButton("Square peg"));
            rd_buttons.push_back(new QRadioButton("Welding elbow joint"));
            rd_buttons.push_back(new QRadioButton("Welding T joint"));
            rd_buttons.push_back(new QRadioButton("Bolt bush"));
            rd_buttons.push_back(new QRadioButton("Cylinder"));

            QGroupBox *groupBox1 = new QGroupBox("select grasp target");
            QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);

            for(int i = 0; i < rd_buttons.size(); i++) {
                //QObject::connect(rd_buttons[i], SIGNAL(clicked()), this, SLOT(pushButtonSelectTargetObjectClickedCallback()));
                layout1->addWidget(rd_buttons[i]);
            }
            layout1->setContentsMargins(0, 0, 0, 0);
            mainLayout->addWidget(groupBox1);
            _widget->setLayout(mainLayout);            
        }
        return _widget;
    }

    void saveGuiTask(TaskPlanner& planner) override {

    }
};

// class TemplateMatchingModel : public BPCommandModel {

//     Q_OBJECT

// private:
//     std::vector<bool> target_;
//     QVector<QRadioButton *> rd_buttons;
// public:
//     TemplateMatchingModel() : BPCommandModel() {

//     }
//     ~TemplateMatchingModel() {}

//     QString caption() const override { return QStringLiteral("TEMPLATE MATCHING"); }

//     QString name() const override { return QStringLiteral("TEMPLATE MATCHING"); }

//     NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
//         return NodeDataType{"robotCommand", "TEMPLATE MATCHING TASK"};
//     }

//     QJsonObject save() const override {
//         QJsonObject modelJson = BPCommandModel::save();
//         // printValueType(target_q, modelJson, "target_q");

//         // modelJson["target_qd"] = target_qd[0];
//         // modelJson["target_qdd"] = target_qdd[0];
//         return modelJson;
//     }
//     void load(QJsonObject const &p) override {
//         // QJsonArray jtarget_q = p["target_q"].toArray();
//         // QJsonValue jtarget_qd1 = p["target_qd"].toDouble();
//         // QJsonValue jtarget_qdd1 = p["target_qdd"].toDouble();
//         // lineEdits2[0]->setText(QString::number(jtarget_qd1.toDouble()));
//         // lineEdits3[0]->setText(QString::number(jtarget_qdd1.toDouble()));
//     }

//     QWidget *embeddedWidget() override {
//         return nullptr;
//     }

//     void saveGuiTask(TaskPlanner& planner) override {

//         // std::cout << "JS TASK to bp_gui_node_task_";
//         // JsDouble jsArr;
//         // vec2Arr(target_q, jsArr);
//         // planner.taskPushBack_jsMove(planner.bp_gui_node_task_, jsArr, false);
//         // std::cout << "size of task due to js task::" << planner.bp_gui_node_task_.size();
//     }
// };
// class CsTargetModel : public BPCommandModel {

//     Q_OBJECT

// private:
//     std::vector<double> target_x;
//     std::vector<double> target_xd;
//     std::vector<double> target_xdd;
//     QVector<QLineEdit*> lineEdits1;
//     QVector<QLineEdit*> lineEdits2;
//     QVector<QLineEdit*> lineEdits3;

// Q_SIGNALS:
//     void target_x_Changed(const double& value, uint i);
//     void target_xd_Changed(const double& value, uint i);
//     void target_xdd_Changed(const double& value, uint i);
// public Q_SLOTS:
//     void set_target_x(const double& value, uint i) { target_x[i] = value; }//emit target_q_Changed(value, i); }
//     void set_target_xd(const double& value, uint i) { target_xd[i] = value; } //emit target_qd_Changed(value, i); }
//     void set_target_xdd(const double& value, uint i) { target_xdd[i] = value; } //emit target_qdd_Changed(value, i); }
// public:
//     std::vector<double> get_target_x() const { return target_x; }
//     std::vector<double> get_target_xd() const { return target_xd; }
//     std::vector<double> get_target_xdd() const { return target_xdd; }

//     CsTargetModel() : BPCommandModel() {
//         target_x.resize(6);
//         target_xd.resize(6);
//         target_xdd.resize(6);
//     }
//     ~CsTargetModel() {}

//     QString caption() const override{ return QStringLiteral("CS TARGET TASK"); }

//     QString name() const override { return QStringLiteral("CS TARGET TASK"); }

//     NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
//         return NodeDataType{"robotCommand", "CS TARGET TASK"};
//     }

//     QJsonObject save() const override {
//         QJsonObject modelJson = BPCommandModel::save();
//         printValueType(target_x, modelJson, "target_x");
//         modelJson["target_xd"] = target_xd[0];
//         modelJson["target_xdd"] = target_xdd[0];
//         return modelJson;
//     }
//     void load(QJsonObject const &p) override {
//         QJsonArray jtarget_x = p["target_x"].toArray();
//         updateWidgetFromJson(jtarget_x, target_x, lineEdits1);
//         QJsonValue jtarget_xd1 = p["target_xd"].toDouble();
//         QJsonValue jtarget_xdd1 = p["target_xdd"].toDouble();
//         lineEdits2[0]->setText(QString::number(jtarget_xd1.toDouble()));
//         lineEdits3[0]->setText(QString::number(jtarget_xdd1.toDouble()));
//     }

//     QWidget *embeddedWidget() override {
//         if(!_widget) {
//             _widget = new QWidget();
//             QHBoxLayout* mainLayout = new QHBoxLayout();
//             const int numLineEdits = 6;

//             for (int i = 0; i < numLineEdits; ++i)
//                 {
//                     QLineEdit *lineEdit = new QLineEdit("0.0");
//                     QLineEdit *lineEdit2 = new QLineEdit("0.0");
//                     QLineEdit *lineEdit3 = new QLineEdit("0.0");

//                     lineEdits1.append(lineEdit);
//                     lineEdits2.append(lineEdit2);
//                     lineEdits3.append(lineEdit3);
//                     QObject::connect(lineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
//                         set_target_x(text.toDouble(), i);
//                     });
//                     QObject::connect(lineEdit2, &QLineEdit::textChanged, this, [i, this](const QString& text) {
//                         set_target_xd(text.toDouble(), i);
//                     });
//                     QObject::connect(lineEdit3, &QLineEdit::textChanged, this, [i, this](const QString& text) {
//                         set_target_xdd(text.toDouble(), i);
//                     });
//                 }
//             QGroupBox *groupBox1 = new QGroupBox("target_x");
//             QGroupBox *groupBox2 = new QGroupBox("target_xd");
//             QGroupBox *groupBox3 = new QGroupBox("target_xdd");

//             QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);
//             QVBoxLayout *layout2 = new QVBoxLayout(groupBox2);
//             QVBoxLayout *layout3 = new QVBoxLayout(groupBox3);

//             for (auto edit : lineEdits1) {
//                 layout1->addWidget(edit);
//             }
//             for (auto edit : lineEdits2) {
//                 layout2->addWidget(edit);
//             }
//             for (auto edit : lineEdits3) {
//                 layout3->addWidget(edit);
//             }
//             layout1->setContentsMargins(0, 0, 0, 0);
//             //layout1->setSpacing(0);
//             layout2->setContentsMargins(0, 0, 0, 0);
//             //layout2->setSpacing(0);
//             layout3->setContentsMargins(0, 0, 0, 0);
//             //layout3->setSpacing(0);
//             mainLayout->addWidget(groupBox1);
//             mainLayout->addWidget(groupBox2);
//             mainLayout->addWidget(groupBox3);

//             _widget->setLayout(mainLayout);

//         }
//         return _widget;
//     }
//     void saveGuiTask(TaskPlanner& planner) override {
//         CsDouble csArr;
//         vec2Arr(target_x, csArr);
//         planner.taskPushBack_csMove(planner.bp_gui_node_task_, csArr, false);
//     }
// };

// class CsTargetRedundantModel : public BPCommandModel {

//     Q_OBJECT

// private:
//     std::vector<double> target_x;
//     std::vector<double> target_xd;
//     std::vector<double> target_xdd;
//     double q_redundant;
//     QVector<QLineEdit*> lineEdits1;
//     QVector<QLineEdit*> lineEdits2;
//     QVector<QLineEdit*> lineEdits3;
//     QLineEdit* redundant_lineEdit;

// Q_SIGNALS:
//     void target_x_Changed(const double& value, uint i);
//     void target_xd_Changed(const double& value, uint i);
//     void target_xdd_Changed(const double& value, uint i);
//     void q_redundant_Changed(const double& value);
// public Q_SLOTS:
//     void set_target_x(const double& value, uint i) { target_x[i] = value; }//emit target_q_Changed(value, i); }
//     void set_target_xd(const double& value, uint i) { target_xd[i] = value; } //emit target_qd_Changed(value, i); }
//     void set_target_xdd(const double& value, uint i) { target_xdd[i] = value; } //emit target_qdd_Changed(value, i); }
//     void set_q_redundant(const double& value) { q_redundant = value; }
// public:
//     std::vector<double> get_target_x() const { return target_x; }
//     std::vector<double> get_target_xd() const { return target_xd; }
//     std::vector<double> get_target_xdd() const { return target_xdd; }

//     CsTargetRedundantModel() : BPCommandModel() {
//         target_x.resize(6);
//         target_xd.resize(6);
//         target_xdd.resize(6);
//         q_redundant = 0.0;
//     }
//     ~CsTargetRedundantModel() {}

//     QString caption() const override{ return QStringLiteral("CS TARGET REDUNDANT TASK"); }

//     QString name() const override { return QStringLiteral("CS TARGET REDUNDANT TASK"); }

//     NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
//         return NodeDataType{"robotCommand", "CS TARGET REDUNDANT TASK"};
//     }

//     QJsonObject save() const override {
//         QJsonObject modelJson = BPCommandModel::save();
//         printValueType(target_x, modelJson, "target_x");
//         modelJson["target_xd"] = target_xd[0];
//         modelJson["target_xdd"] = target_xdd[0];
//         modelJson["q_redundant"] = q_redundant;
//         return modelJson;
//     }
//     void load(QJsonObject const &p) override {
//         QJsonArray jtarget_x = p["target_x"].toArray();
//         updateWidgetFromJson(jtarget_x, target_x, lineEdits1);
//         QJsonValue jtarget_xd1 = p["target_xd"].toDouble();
//         QJsonValue jtarget_xdd1 = p["target_xdd"].toDouble();
//         QJsonValue jq_redundant = p["q_redundant"].toDouble();
//         lineEdits2[0]->setText(QString::number(jtarget_xd1.toDouble()));
//         lineEdits3[0]->setText(QString::number(jtarget_xdd1.toDouble()));
//         redundant_lineEdit->setText(QString::number(jq_redundant.toDouble()));
//     }

//     QWidget *embeddedWidget() override {
//         if(!_widget) {
//             _widget = new QWidget();
//             QHBoxLayout* mainLayout = new QHBoxLayout();
//             const int numLineEdits = 6;

//             for (int i = 0; i < numLineEdits; ++i)
//                 {
//                     QLineEdit *lineEdit = new QLineEdit("0.0");
//                     QLineEdit *lineEdit2 = new QLineEdit("0.0");
//                     QLineEdit *lineEdit3 = new QLineEdit("0.0");

//                     lineEdits1.append(lineEdit);
//                     lineEdits2.append(lineEdit2);
//                     lineEdits3.append(lineEdit3);
//                     QObject::connect(lineEdit, &QLineEdit::textChanged, this, [i, this](const QString& text) {
//                         set_target_x(text.toDouble(), i);
//                     });
//                     QObject::connect(lineEdit2, &QLineEdit::textChanged, this, [i, this](const QString& text) {
//                         set_target_xd(text.toDouble(), i);
//                     });
//                     QObject::connect(lineEdit3, &QLineEdit::textChanged, this, [i, this](const QString& text) {
//                         set_target_xdd(text.toDouble(), i);
//                     });                    
//                 }
//             redundant_lineEdit = new QLineEdit("0.0");
//             QObject::connect(redundant_lineEdit, &QLineEdit::textChanged, this, [this](const QString& text) {
//                 set_q_redundant(text.toDouble());
//             });
//             QGroupBox *groupBox1 = new QGroupBox("target_x");
//             QGroupBox *groupBox2 = new QGroupBox("target_xd");
//             QGroupBox *groupBox3 = new QGroupBox("target_xdd");

//             QVBoxLayout *layout1 = new QVBoxLayout(groupBox1);
//             QVBoxLayout *layout2 = new QVBoxLayout(groupBox2);
//             QVBoxLayout *layout3 = new QVBoxLayout(groupBox3);

//             for (auto edit : lineEdits1) {
//                 layout1->addWidget(edit);
//             }
//             for (auto edit : lineEdits2) {
//                 layout2->addWidget(edit);
//             }
//             for (auto edit : lineEdits3) {
//                 layout3->addWidget(edit);
//             }
//             layout1->setContentsMargins(0, 0, 0, 0);
//             //layout1->setSpacing(0);
//             layout2->setContentsMargins(0, 0, 0, 0);
//             //layout2->setSpacing(0);
//             layout3->setContentsMargins(0, 0, 0, 0);
//             //layout3->setSpacing(0);
//             mainLayout->addWidget(groupBox1);
//             mainLayout->addWidget(groupBox2);
//             mainLayout->addWidget(groupBox3);
//             mainLayout->addWidget(redundant_lineEdit);
            
//             _widget->setLayout(mainLayout);

//         }
//         return _widget;
//     }
//     void saveGuiTask(TaskPlanner& planner) override {
//         CsDouble csArr;
//         vec2Arr(target_x, csArr);
//         planner.taskPushBack_csMove_redundant(planner.bp_gui_node_task_, csArr, q_redundant, false);
//     }
// };

// class MoveBlendingModel : public BPCommandModel {

//     Q_OBJECT

// private:
//     BlendingTraj _traj;

//     QVBoxLayout *layout_group_box;
//     QVector<QVector<QLineEdit*>> xs_lineEdits;
//     QVector<QLineEdit*> xds_lineEdits;
//     QVector<QLineEdit*> xdds_lineEdits;
//     QVector<QLineEdit*> radiuses_lineEdits;
//     QVector<QLineEdit*> waypoint_xds_lineEdits;
//     QVector<QLineEdit*> qs_redundant_lineEdits;

//     QLineEdit* waypoint_num_lineEdit;
//     QCheckBox* is_radius_percent_checkbox;
    
//     // QPushButton* addButton;
//     // QPushButton* removeButton;

//     QTableWidget* _table_widget;
//     QModelIndex* _model_index;

// Q_SIGNALS:
//     void xs_Changed(const double& value, uint i, uint j);
//     void xds_Changed(const double& value, uint i);
//     void xdds_Changed(const double& value, uint i);
//     void radiuses_Changed(const double& value, uint i);
//     void waypoint_xds_Changed(const double& value, uint i);
//     void qs_redundant_Changed(const double& value, uint i);
//     void waypoint_num_Changed(const int value);
//     void is_radius_percent_Changed(const bool value);

// public Q_SLOTS:
//     void set_xs(const double& value, uint i, uint j) { _traj.xs[i][j] = value; };

//     void set_xds(const double& value, uint i) { _traj.xds[i] = value; };
//     void set_xdds(const double& value, uint i) { _traj.xdds[i] = value; };
//     void set_radiuses(const double& value, uint i) { _traj.radiuses[i] = value; };
//     void set_waypoint_xds(const double& value, uint i) { _traj.waypoint_xds[i] = value; };
//     void set_qs_redundant(const double& value, uint i) { _traj.qs_redundant[i] = value; };

//     void set_waypoint_num(const int value) { _traj.waypoint_num = value; };
//     void set_is_radius_percent(const bool value) { _traj.is_radius_percent = value; };
    
// public:
//     MoveBlendingModel() : BPCommandModel() {
//         _traj = _BlendingTraj();
//         // _traj.waypoint_num = 0;
//         // _traj.is_radius_percent = true;

//         // _traj.xs.clear();
//         // _traj.xds.clear();
//         // _traj.xdds.clear();
//         // _traj.waypoint_xds.clear();
//         // _traj.radiuses.clear();
//         // _traj.qs_redundant.clear();
//     }
//     ~MoveBlendingModel() {}

//     QString caption() const override{ return QStringLiteral("MOVE BLENDING TASK"); }

//     QString name() const override { return QStringLiteral("MOVE BLENDING TASK"); }

//     NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
//         return NodeDataType{"robotCommand", "MOVE BLENDING TASK"};
//     }

//     QJsonObject save() const override {
//         QJsonObject modelJson = BPCommandModel::save();

//         // // _traj.xs.clear();
//         // // _traj.xds.clear();
//         // // _traj.xdds.clear();
//         // // _traj.waypoint_xds.clear();
//         // // _traj.radiuses.clear();
//         // // _traj.qs_redundant.clear();

//         // auto model = _table_widget->model();
//         // //tablewidget model to Blending trajectory struct
//         // for (int i = 0; i < model->rowCount(); i++) {
//         //     _traj.xs.push_back({CsDouble{0.0,}});
//         //     for (int j = 0; j < model->columnCount(); j++) {
//         //         auto value = model->index(i, j).data().toDouble();
//         //         if (j < 6) {
//         //             _traj.xs[i][j] = value;                    
//         //         } else {  // j >= 6 
//         //             switch (j - 5) {
//         //             case 1:
//         //                 _traj.xds.push_back(value);
//         //                 break;
//         //             case 2:
//         //                 _traj.xdds.push_back(value);
//         //                 break;
//         //             case 3:
//         //                 _traj.waypoint_xds.push_back(value);
//         //                 break;
//         //             case 4:
//         //                 _traj.radiuses.push_back(value);
//         //                 break;
//         //             case 5:
//         //                 _traj.qs_redundant.push_back(value);
//         //                 break;
//         //             default:
//         //                 break;
//         //             }                    
//         //         }
//         //     }
//         // }         

//         // //vector<CSDouble>
//         // QJsonArray xsJsonArray;        
//         // for (const auto &arr : _traj.xs) {
//         //     QJsonArray csJsonArray;
//         //     for (const auto &data : arr) {
//         //         csJsonArray.append(data);
//         //     }
//         //     xsJsonArray.append(csJsonArray);
//         // }
//         // modelJson["xs"] = xsJsonArray;

//         // printValueType(_traj.xds, modelJson, "xds");
//         // printValueType(_traj.xdds, modelJson, "xdds");
//         // printValueType(_traj.radiuses, modelJson, "radiuses");
//         // printValueType(_traj.waypoint_xds, modelJson, "waypoint_xds");
//         // printValueType(_traj.qs_redundant, modelJson, "qs_redundant");
//         // modelJson["waypoint_num"] = _traj.waypoint_num;
//         // modelJson["is_radius_percent"] = _traj.is_radius_percent;

//         return modelJson;
//     }
//     void load(QJsonObject const &p) override {

//         // QJsonArray jarr = p["xs"].toArray();
//         // for (int i = 0; i < jarr.size(); ++i) {
//         //     const auto& jarr2 = jarr[i].toArray();
//         //     for (int j = 0; j < jarr2.size(); ++j) {
//         //         const QJsonValue& jsonValue = jarr2[j];
//         //         if (jsonValue.isDouble()) {
//         //             _traj.xs[i][j] = jsonValue.toDouble();
//         //         }
//         //         else {
//         //             return;
//         //             //throw std::runtime_error("Invalid element type in QJsonArray");
//         //         }
//         //     }
//         // }
//         // for (int i = 0; i < xs_lineEdits.size(); ++i) {
//         //     auto vec = xs_lineEdits[i];
//         //     for (int j = 0; j < vec.size(); ++j) {
//         //         vec[j]->setText(QString::number(_traj.xs[i][j]));
//         //     }
//         // }  

//         // QJsonArray jxds = p["xds"].toArray();
//         // QJsonArray jxdds = p["xdds"].toArray();
//         // QJsonArray jradiuses = p["radiuses"].toArray();
//         // QJsonArray jwaypoint_xds = p["waypoint_xds"].toArray();
//         // QJsonArray jqs_redundant = p["qs_redundant"].toArray();
//         // updateWidgetFromJson(jxds, _traj.xds , xds_lineEdits);
//         // updateWidgetFromJson(jxdds, _traj.xdds , xdds_lineEdits);
//         // updateWidgetFromJson(jradiuses, _traj.radiuses , radiuses_lineEdits);
//         // updateWidgetFromJson(jwaypoint_xds, _traj.waypoint_xds , waypoint_xds_lineEdits);   
//         // updateWidgetFromJson(jqs_redundant, _traj.qs_redundant , qs_redundant_lineEdits);  
//         // //load시 동적으로 위젯을 추가
//         // QJsonValue jwaypoint_num = p["target_xdd"].toInt();
//         // QJsonValue jis_radius_percent = p["is_radius_percent"].toBool();
//         // waypoint_num_lineEdit->setText(QString::number(jwaypoint_num.toInt()));
//         // is_radius_percent_checkbox->setChecked(jis_radius_percent.toBool());
//     }
// //처음 불러올때 데이터가있다면..load시에 활용하면됨
//     QWidget *embeddedWidget() override {
//         if(!_widget) {
//             _widget = new QWidget();
//             QVBoxLayout *layout = new QVBoxLayout(_widget);
//             _table_widget = new QTableWidget(1, 11, _widget);
//             _table_widget->setMinimumSize(35,10);
//             layout->addWidget(_table_widget, 1, Qt::AlignHCenter);
         
//             QStringList header_list; 
//             header_list << "x" << "y" << "z" << "R" << "P" << "Y" << "vel" << "acc" << "radius" << "way point vel" << "redundant angle velocity";
//             _table_widget->setHorizontalHeaderLabels(header_list);
            
            
//             auto addButton = new QPushButton("add WayPoint");
//             auto removeButton = new QPushButton("remove WayPoint"); 
//             layout->addWidget(addButton);
//             layout->addWidget(removeButton);

//             QObject::connect(addButton, &QPushButton::clicked, this, [&]() { 
//                 _traj.waypoint_num++;
//                 _table_widget->insertRow(1);
//             });
//             QObject::connect(removeButton, &QPushButton::clicked, this, [&]() { 
//                 if(_table_widget->rowCount() > 0) {
//                     _traj.waypoint_num--;
//                     _table_widget->removeRow(1);

//                 }    
//             });


                    

//         }
//         return _widget;        
//     }
//     void saveGuiTask(TaskPlanner& planner) override {
//         // CsDouble csArr;
//         // vec2Arr(target_x, csArr);
//         // planner.taskPushBack_csMove_redundant(planner.bp_gui_node_task_, csArr, q_redundant, false);
//     }
// };



// class BP_PLCModel : public BPCommandModel {

//     Q_OBJECT

// private:
//     int _command;
//     QLineEdit * _lineEditPlcCommand;

// public Q_SLOTS:

// public:

//     BP_PLCModel() : BPCommandModel() {
//         _command = 0;
//     }
//     ~BP_PLCModel() {}

// Q_SIGNALS:
//     void timeDelay_Changed(const int& value);

// public Q_SLOTS:
//     void set_time_delay(const int& value) { _command = value; }

// public:
//     int get_time_delay() const { return _command; }

//     QString caption() const override{ return QStringLiteral("BINPICKING PLC COMMAND"); }

//     QString name() const override { return QStringLiteral("BINPICKING PLC COMMAND"); }

//     NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
//         return NodeDataType{"robotCommand", "BINPICKING PLC TASK"};
//     }

//     QJsonObject save() const override {
//         QJsonObject modelJson = BPCommandModel::save();
//         modelJson["plc_command"] = _command;
//         return modelJson;
//     }

//     void load(QJsonObject const &p) override {
//         QJsonValue jcommand = p["plc_command"].toInt();
//         _lineEditPlcCommand->setText(QString::number(jcommand.toInt()));
//     }

//     QWidget *embeddedWidget() override {
//         if(!_widget) {
//             _widget = new QWidget();
//             QHBoxLayout* mainLayout = new QHBoxLayout();
//             _lineEditPlcCommand = new QLineEdit("0");
//             QObject::connect(_lineEditPlcCommand, &QLineEdit::textChanged, this, [this](const QString& text) {
//                 set_time_delay(text.toInt());
//             });
//             mainLayout->addWidget(_lineEditPlcCommand);
//             _widget->setLayout(mainLayout);
//         }
//         return _widget;
//     }
//     void saveGuiTask(TaskPlanner& planner) override {
//         planner.taskPushBackPLCModbusCmd(planner.bp_gui_node_task_, _command);        
//     }
// };

// class PauseModel : public BPCommandModel {

//     Q_OBJECT

// Q_SIGNALS:

// public:

//     PauseModel() : BPCommandModel() {}
//     ~PauseModel() {}

//     QString caption() const override{ return QStringLiteral("PAUSE TASK"); }

//     QString name() const override { return QStringLiteral("PAUSE TASK"); }

 
//     NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
//         return NodeDataType{"robotCommand", "PAUSE TASK"};
//     }

//     QJsonObject save() const override {
//         QJsonObject modelJson = BPCommandModel::save();        
//         return modelJson;
//     }

//     void load(QJsonObject const &p) override {}

//     QWidget *embeddedWidget() override {
//         return nullptr;
//     }
//     //bool expandable() const override { return false; } 

//     void saveGuiTask(TaskPlanner& planner) override {
//         planner.taskPushBack_pause(planner.bp_gui_node_task_);  
//     }
// };

// class SavePointModel : public BPCommandModel {

//     Q_OBJECT

// public:

//     SavePointModel() : BPCommandModel() {
//     }
//     ~SavePointModel() {}

//     QString caption() const override{ return QStringLiteral("SAVE POINT TASK"); }

//     QString name() const override { return QStringLiteral("SAVE POINT TASK"); }

 
//     NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
//         return NodeDataType{"robotCommand", "SAVE POINT TASK"};
//     }

//     QJsonObject save() const override {
//         QJsonObject modelJson = BPCommandModel::save();        
//         return modelJson;
//     }

//     void load(QJsonObject const &p) override {}

//     QWidget *embeddedWidget() override {
//         return nullptr;
//     }
//     //bool expandable() const override { return false; } 

//     void saveGuiTask(TaskPlanner& planner) override {
//         planner.savePoint_pushBack(planner.bp_gui_node_task_);  
//     }
// };

// class RewindModel : public BPCommandModel {

//     Q_OBJECT

// public:

//     RewindModel() : BPCommandModel() {
//     }
//     ~RewindModel() {}

//     QString caption() const override{ return QStringLiteral("REWIND TASK"); }

//     QString name() const override { return QStringLiteral("REWIND TASK"); }

 
//     NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
//         return NodeDataType{"robotCommand", "REWIND TASK"};
//     }

//     QJsonObject save() const override {
//         QJsonObject modelJson = BPCommandModel::save();        
//         return modelJson;
//     }

//     void load(QJsonObject const &p) override {}

//     QWidget *embeddedWidget() override {
//         return nullptr;
//     }
//     //bool expandable() const override { return false; } 

//     void saveGuiTask(TaskPlanner& planner) override {
//         planner.taskPushBack_rewind(planner.bp_gui_node_task_);  
//     }
// };





#endif

