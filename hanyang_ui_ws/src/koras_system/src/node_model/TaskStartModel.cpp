#include "TaskStartModel.hpp"

#include <QtCore/QJsonValue>
#include <QtGui/QDoubleValidator>
#include <QtWidgets/QLineEdit>
#include <QHBoxLayout>

TaskStartModel::TaskStartModel()
    : _widget(nullptr)
{ //vel_js = 15.0; acc_js = 30.0;
}

QJsonObject TaskStartModel::save() const
{
    QJsonObject modelJson = NodeDelegateModel::save();
    //modelJson["model-name"] = "Task Starting Point";
    modelJson["vel_cs"] = vel_cs;
    modelJson["vel_js"] = vel_js;
    modelJson["acc_cs"] = acc_cs;
    modelJson["acc_js"] = acc_js;
    modelJson["use_plc"] = use_plc_checkbox->isChecked();;
    
    return modelJson;
}

void TaskStartModel::load(QJsonObject const &p)
{    
    _lineEdits[0]->setText(QString::number(p["vel_cs"].toDouble()));
    _lineEdits[1]->setText(QString::number(p["vel_js"].toDouble()));
    _lineEdits[2]->setText(QString::number(p["acc_cs"].toDouble()));
    _lineEdits[3]->setText(QString::number(p["acc_js"].toDouble()));
    _lineEdits[3]->setText(QString::number(p["acc_js"].toDouble()));
    use_plc = p["use_plc"].toBool(); 
    if(use_plc) {
        use_plc_checkbox->setCheckState(Qt::CheckState::Checked);
    } 
}

unsigned int TaskStartModel::nPorts(PortType portType) const
{
    unsigned int result = 1;

    switch (portType) {
    case PortType::In:
        result = 0;
        break;

    case PortType::Out:
        result = 1;
    default:
        break;
    }

    return result;
}

NodeDataType TaskStartModel::dataType(PortType, PortIndex) const
{
    NodeDataType startNodeType;
    startNodeType.id = "TaskParameter";
    startNodeType.name = QStringLiteral("TASK START");
    return startNodeType;
}

ConnectionPolicy TaskStartModel::portConnectionPolicy(PortType portType, PortIndex) const
{
    auto result = ConnectionPolicy::One;
    switch (portType) {
    case PortType::In:
        result = ConnectionPolicy::One;
        break;
    case PortType::Out:
        result = ConnectionPolicy::One;
        break;
    case PortType::None:
        break;
    }
    return result;
}

std::shared_ptr<NodeData> TaskStartModel::outData(PortIndex index)
{
    std::shared_ptr<NodeData> ptr;
    return ptr;
}


void TaskStartModel::setNumber(double n) //set default value from outerspace
{
//    _number = std::make_shared<CSMoveNodeData>(n);

//    Q_EMIT dataUpdated(0);

//    if (_lineEdit)
//        _lineEdit->setText(QString::number(_number->number()));
}

QWidget* TaskStartModel::embeddedWidget() 
{ 
    if(!_widget) {
        _widget = new QWidget();            

        QHBoxLayout* mainLayout = new QHBoxLayout();
        QGroupBox *groupBox1 = new QGroupBox("TASK SETTING");
        QVBoxLayout* layout1 = new QVBoxLayout(groupBox1);

        QLineEdit *lineEdit = new QLineEdit("0.0");
        QLineEdit *lineEdit2 = new QLineEdit("0.0");
        QLineEdit *lineEdit3 = new QLineEdit("0.0");
        QLineEdit *lineEdit4 = new QLineEdit("0.0");

        QLabel *label1 = new QLabel("Velocity CS");
        QLabel *label2 = new QLabel("Velocity JS");
        QLabel *label3 = new QLabel("Acceleration CS");
        QLabel *label4 = new QLabel("Acceleration JS");

        _lineEdits.push_back(lineEdit);
        _lineEdits.push_back(lineEdit2);
        _lineEdits.push_back(lineEdit3);
        _lineEdits.push_back(lineEdit4);

        layout1->addWidget(label1);
        layout1->addWidget(lineEdit);
        layout1->addWidget(label2);
        layout1->addWidget(lineEdit2);
        layout1->addWidget(label3);
        layout1->addWidget(lineEdit3);
        layout1->addWidget(label4);
        layout1->addWidget(lineEdit4);

        use_plc_checkbox = new QCheckBox();
        use_plc_checkbox->setText("use plc task");
        layout1->addWidget(use_plc_checkbox);

        layout1->setContentsMargins(0, 0, 0, 0);

        QObject::connect(lineEdit, &QLineEdit::textChanged, this, [this](const QString& text) {
            vel_cs = text.toDouble();
        });
        QObject::connect(lineEdit2, &QLineEdit::textChanged, this, [this](const QString& text) {
            vel_js = text.toDouble();
        });
        QObject::connect(lineEdit3, &QLineEdit::textChanged, this, [this](const QString& text) {
            acc_cs = text.toDouble();
        });
        QObject::connect(lineEdit4, &QLineEdit::textChanged, this, [this](const QString& text) {
            acc_js = text.toDouble();
        });

        lineEdit->setText("0.15");
        lineEdit2->setText("15.0");
        lineEdit3->setText("0.3");
        lineEdit4->setText("30.0");
        mainLayout->addWidget(groupBox1);
        _widget->setLayout(mainLayout);
    }
    return _widget;   
}