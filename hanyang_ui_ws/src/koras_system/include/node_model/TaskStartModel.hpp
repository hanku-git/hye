#ifndef TASK_START_MODEL_HPP
#define TASK_START_MODEL_HPP

#include <iostream>
#include <vector>
#include <QtNodes/NodeDelegateModel>
#include <QtNodes/Definitions>

#include <QtCore/QObject>
#include <QtCore/QJsonValue>
#include <QtWidgets/QLabel>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QCheckBox>

#include <QtGui/QDoubleValidator>


using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;
using QtNodes::ConnectionPolicy;
class QLineEdit;

/// The model dictates the number of inputs and outputs for the Node.
/// In this example it has no logic.
class TaskStartModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    TaskStartModel();

    virtual ~TaskStartModel() {}

public:
    QString caption() const override { return QStringLiteral("TASK START"); }

    bool captionVisible() const override { return true; }

    bool setWidgetVisible(bool flag) {
        if(embeddedWidget() != nullptr) {
            embeddedWidget()->setVisible(flag);
        }
    }

    QString name() const override { return QStringLiteral("TASK START"); }

public:
    QJsonObject save() const override;

    void load(QJsonObject const &p) override;

public:
    unsigned int nPorts(PortType portType) const override;

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override;

    ConnectionPolicy portConnectionPolicy(PortType, PortIndex) const override;

    std::shared_ptr<NodeData> outData(PortIndex port) override;

    void setInData(std::shared_ptr<NodeData>, PortIndex) override {}

    QWidget *embeddedWidget() override; 

public:
    void setNumber(double number);
    //void getImage(QString image) {return _modelImage;}

private Q_SLOTS:
    //void onTextEdited(QString const &string);

private:
    QWidget* _widget;
    std::vector<QLineEdit*> _lineEdits;
    double vel_cs;
    double vel_js;
    double acc_cs;
    double acc_js;
    bool use_plc;
    QCheckBox* use_plc_checkbox;
};
#endif