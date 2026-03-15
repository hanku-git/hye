#ifndef PLC_COMMAND_MODEL_HPP_
#define PLC_COMMAND_MODEL_HPP_

#include <QtNodes/NodeDelegateModel>

#include <QtCore/QJsonObject>
#include <QtCore/QObject>
#include <QtCore/QJsonArray>

//#include <QtWidgets>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QPlainTextEdit>

#include <QtNodes/Definitions>

#include <iostream>
#include <memory>
#include "modbusTcpDefine.hpp"
#include "protocolDefine.hpp"

class DoubleData;
using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;

//template <typename T>
//concept ValidContainer2 = std::same_as<T, std::vector<typename T::value_type>> ||
//                          std::same_as<T, std::vector<std::vector<double>>>;


//template<typename T>
//concept ValidContainer =
//    std::same_as<T, std::vector<typename T::value_type>>;
//template<typename T>
//concept Sortable =
//    ValidContainer<T> && std::same_as<typename T::value_type, double>;

//template<typename T>
//concept ValidContainer =
//    std::same_as<T, std::vector<typename T::value_type>>;
//template<typename T>
//concept Sortable =
//    ValidContainer<T> && std::same_as<typename T::value_type, double>;
//QJsonObject printValueType(Sortable auto a, QJsonObject& modelJson, QString name) {
////    QString index;
////    qDebug() << a.size();
////    QJsonArray array;
////    for(int i = 0; i < a.size(); i++) {
////        //index = name + QString::number(i + 1);
////        //qDebug() << index;
////        array.append(a[i]);
////        modelJson[name] = a[i];
////    }
//    QJsonArray jsonArray;
//    for (const double value : a) {
//        jsonArray.append(value);
//    }
//    modelJson[name] = jsonArray;
//    return modelJson;
//}
//template <typename T>
//concept IsDouble = std::same_as<typename T::value_type, double>;
//QJsonObject printValueType(IsDouble auto a, QJsonObject& modelJson, QString name) {
//    modelJson[name] = static_cast<double>(a);
//    return modelJson;
//}


////append w.r.t class member data type ex) std::vector<double> target_x and bool, double type
//template<typename MemberDataType>
//QWidget* appendMemberToWidgets(QWidget* widget, MemberDataType data) {
//    QLineEdit* a = new QLineEdit("0.0");
//    QGroupBox* groupBox = new QGroupBox("groupBox");

//    QVBoxLayout *vbox = new QVBoxLayout;
//    vbox->addWidget(a);
//    //vbox->addStretch(1);
//    groupBox->setLayout(vbox);
//    //widget->
//}

//address : 0~65535 : unsigned short
//num of values  : 0~125 : unsigned char
//coil값은 bool값 0 or 1
//Holding Resister의 값은 16비트 정수로 표현되며, 따라서 0~ 32767 사이의 값을 저장
class PLC_MODBUS_TCP_ReadModel : public NodeDelegateModel
{
    Q_OBJECT
    //Q_DECLARE_METATYPE(KCRCommand)
public:
    enum PLC_MODBUS_TCP_READ {
        ReadCoils = 1,
        ReadDiscreteInputs,
        ReadHoldingResisters,
        ReadInputResisters,
        Read_WriteMultipleResisters = 23
    };
    Q_ENUM(PLC_MODBUS_TCP_READ)

    PLC_MODBUS_TCP_ReadModel() : _widget(nullptr) {
        _name = "MODBUS TCP READ";
        _read_coils.resize(0);
        _read_registers.resize(0);
    }
    ~PLC_MODBUS_TCP_ReadModel() = default;
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

    QString caption() const override { return _name; }
    QString name() const override { return _name; }

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
        return NodeDataType{"robotCommand", tr("%1").arg(_name)};
    }

    void setWidgetExpand(bool flag) override {
        _widget->setVisible(flag);
    }

    void updateWidgetFromJson(QJsonArray& jarr, std::vector<double>& vec, QVector<QLineEdit*>& lineEdits) {
        if (jarr.size() != vec.size()) {
           // throw std::runtime_error("Size mismatch between QJsonArray and std::vector");
           return;
        } else {
            for (size_t i = 0; i < jarr.size(); ++i) {
                const QJsonValue& jsonValue = jarr[i];
                if (jsonValue.isDouble()) {
                    vec[i] = jsonValue.toDouble();
                }
                else {
                    return;
                    //throw std::runtime_error("Invalid element type in QJsonArray");
                }
            }
            for (size_t i = 0; i < vec.size(); ++i) {
                lineEdits[i]->setText(QString::number(vec[i]));
            }
        }
    }

    QJsonObject save() const override {
        QJsonObject modelJson = NodeDelegateModel::save();
//        printValueType(target_q, modelJson, "target_q");
//        printValueType(target_qd, modelJson, "target_qd");
//        printValueType(target_qdd, modelJson, "target_qdd");
        modelJson["protocol"] = Protocol::MODBUS_TCP;
//        qDebug() << modelJson;
        return modelJson;
    }
    void load(QJsonObject const &p) override {
        //if (p.contains("target_q") && p["target_q"].isArray()) {
//        QJsonArray jtarget_q = p["target_q"].toArray();
//        QJsonArray jtarget_qd = p["target_qd"].toArray();
//        QJsonArray jtarget_qdd = p["target_qdd"].toArray();
//        updateWidgetFromJson(jtarget_q, target_q, lineEdits1);
//        updateWidgetFromJson(jtarget_qd, target_qd, lineEdits2);
//        updateWidgetFromJson(jtarget_qdd, target_qdd, lineEdits3);
    }

    QWidget *embeddedWidget() override {
        //연결된 plc station 확인하는 기능 ip port에 해당.
        //각 MODBUS_TCP 8종의 요청에 맞는 폼을 작성.
        if(!_widget) {
            _widget = new QWidget();

            QVBoxLayout* mainLayout = new QVBoxLayout();

            QGroupBox *groupBox1 = new QGroupBox("IP/PORT");
            QHBoxLayout *layout1 = new QHBoxLayout(groupBox1);

            lineEdits_ip = new QLineEdit("127.0.0.1");
            lineEdits_port = new QLineEdit("502");

            layout1->addWidget(lineEdits_ip);
            layout1->addWidget(lineEdits_port);
            layout1->setContentsMargins(0, 0, 0, 0);
            layout1->setSpacing(0);
            QGroupBox *groupBox2 = new QGroupBox("READ VALUES FROM PLC");
            QVBoxLayout *layout2 = new QVBoxLayout(groupBox2);
            QLabel* label_adrs = new QLabel("Starting address");
            lineEdits_address = new QLineEdit("0");
            QLabel* label_numval = new QLabel("Number of values");
            lineEdits_numval = new QLineEdit("1");

            QPushButton* readCoils = new QPushButton("Read coils");
            QPushButton* readDiscreteInputs = new QPushButton("Read discrete inputs");
            QPushButton* readHoldingResisters = new QPushButton("Read holding resisters");
            QPushButton* readInputResisters = new QPushButton("Read input resisters");

            layout2->addWidget(label_adrs);
            layout2->addWidget(lineEdits_address);
            layout2->addWidget(label_numval);
            layout2->addWidget(lineEdits_numval);
            layout2->addWidget(readCoils);
            layout2->addWidget(readDiscreteInputs);
            layout2->addWidget(readHoldingResisters);
            layout2->addWidget(readInputResisters);
            layout2->setContentsMargins(0, 0, 0, 0);
            layout2->setSpacing(0);
            QButtonGroup *buttonGroup = new QButtonGroup();

            buttonGroup->addButton(readCoils);
            buttonGroup->addButton(readDiscreteInputs);
            buttonGroup->addButton(readHoldingResisters);
            buttonGroup->addButton(readInputResisters);
            for (QAbstractButton *button : buttonGroup->buttons()) {
                button->setCheckable(true);
            }
            buttonGroup->setExclusive(true);
//TO DO : connect
            //lineEdits_address : 0~65535
            //lineEdits_numval  : 0~125
            connect(lineEdits_numval, &QLineEdit::textChanged, this, [this](const QString& text) {
                _portNumber = lineEdits_numval->text().toInt();
            });

            connect(buttonGroup, QOverload<QAbstractButton *, bool>::of(&QButtonGroup::buttonToggled),
            [ = ](QAbstractButton * button, bool checked) {
                if (button == readCoils || button == readDiscreteInputs) {
                    _read_coils.resize(_portNumber);
                } else {
                    _read_registers.resize(_portNumber);
                }
            });

            mainLayout->addWidget(groupBox1);
            mainLayout->addWidget(groupBox2);

            _widget->setLayout(mainLayout);

        }
        return _widget;
    }
//taskmanager의 실행중 통신으로 부터 전달받음~port값변경
public Q_SLOTS:
    void readDataFromMODBUS() {
    }
protected:
    QString _name;
    std::weak_ptr<DoubleData> _number1;
    std::weak_ptr<DoubleData> _number2;
    //tcp read data to result
    std::shared_ptr<DoubleData> _result;

    std::string _ip;
    uint16_t _port;
    int _portNumber;

    std::vector<bool> _read_coils;
    std::vector<uint16_t> _read_registers;

    PLC_MODBUS_TCP_READ _fuction;
    uint16_t _readValue;

    QWidget* _widget;
    //ptr for loading
    QButtonGroup* buttonGroup;
    QLineEdit* lineEdits_ip;
    QLineEdit* lineEdits_port;
    QLineEdit *lineEdits_address;
    QLineEdit *lineEdits_numval;

};

class PLC_MODBUS_TCP_WriteModel : public NodeDelegateModel
{
    Q_OBJECT
    //Q_DECLARE_METATYPE(KCRCommand)
public:
    enum PLC_MODBUS_TCP_WRITE {
        WriteSingleCoil = 5,
        WriteSingleResister,
        WriteMultipleCoils = 15,
        WriteMultipleResisters = 16,
        //Read_WriteMultipleResisters = 23
    };
    Q_ENUM(PLC_MODBUS_TCP_WRITE)

    PLC_MODBUS_TCP_WriteModel() : _widget(nullptr) {
        _name = "MODBUS TCP WRITE";
    }
    ~PLC_MODBUS_TCP_WriteModel() = default;
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

    QString caption() const override { return _name; }
    QString name() const override { return _name; }

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
        return NodeDataType{"robotCommand", tr("%1").arg(_name)};
    }

    void setWidgetExpand(bool flag) override {
        _widget->setVisible(flag);
    }

    void updateWidgetFromJson(QJsonArray& jarr, std::vector<double>& vec, QVector<QLineEdit*>& lineEdits) {
        if (jarr.size() != vec.size()) {
           // throw std::runtime_error("Size mismatch between QJsonArray and std::vector");
           return;
        } else {
            for (size_t i = 0; i < jarr.size(); ++i) {
                const QJsonValue& jsonValue = jarr[i];
                if (jsonValue.isDouble()) {
                    vec[i] = jsonValue.toDouble();
                }
                else {
                    return;
                    //throw std::runtime_error("Invalid element type in QJsonArray");
                }
            }
            for (size_t i = 0; i < vec.size(); ++i) {
                lineEdits[i]->setText(QString::number(vec[i]));
            }
        }
    }

    QJsonObject save() const override {
        QJsonObject modelJson = NodeDelegateModel::save();
        modelJson["protocol"] = Protocol::MODBUS_TCP;
        modelJson["ip"] = lineEdits_ip->text();
        modelJson["port"] = lineEdits_port->text().toInt();
        modelJson["address"] = lineEdits_address->text().toInt();
        modelJson["number"] = lineEdits_number->text().toInt();
        // int size = lineEdits_number->text().toInt();
//        if(_fuction == WriteSingleCoil || _fuction == WriteMultipleCoils) {
//            modelJson["data"] = toJsonArray(_write_coils);
//            modelJson["function"] = _fuction;
//        } else if (_fuction == WriteSingleResister || _fuction == WriteMultipleResisters) {
//            modelJson["data"] = toJsonArray(_write_registers);
//            modelJson["function"] = _fuction;
//        }


//        printValueType(target_x, modelJson, "target_x");
//        modelJson["target_xd"] = target_xd[0];
//        modelJson["target_xdd"] = target_xdd[0];
//        modelJson["command"] = KCRCommand::CS_TARGET_PATH;
//        return modelJson;
//    }
//    void load(QJsonObject const &p) override {
//        //if (p.contains("target_q") && p["target_q"].isArray()) {
//        QJsonArray jtarget_x = p["target_x"].toArray();
////        QJsonArray jtarget_xd = p["target_xd"].toArray();
////        QJsonArray jtarget_xdd = p["target_xdd"].toArray();
//        updateWidgetFromJson(jtarget_x, target_x, lineEdits1);
////        updateWidgetFromJson(jtarget_xd, target_xd, lineEdits2);
////        updateWidgetFromJson(jtarget_xdd, target_xdd, lineEdits3);
//        QJsonValue jtarget_xd1 = p["target_xd"].toDouble();
//        QJsonValue jtarget_xdd1 = p["target_xdd"].toDouble();
//        lineEdits2[0]->setText(QString::number(jtarget_xd1.toDouble()));
//        lineEdits3[0]->setText(QString::number(jtarget_xdd1.toDouble()));



        auto str = plainText->toPlainText();
        QStringList pieces = str.split( "," );
//        std::vector<bool> _write_to_coils;

//        bool boolarr[size];
//        if(pieces.size() == size) {
//            for(int i = 0; i < size ; i++) {
//                bool bvalue = (pieces[i] == "1" ? true : false);
//                boolarr[i] = bvalue;
//                jsonArray
//            }
//        } else {
//            qDebug() << "size not equal";
//        }

        QJsonArray jsonArray;
        for (QString piece : pieces) {
            bool bvalue = (piece == "1" ? true : false);
            jsonArray.append(QJsonValue(bvalue));
        }
        modelJson["data"] = jsonArray;
        qDebug() << modelJson["data"];

        return modelJson;
    }
    void load(QJsonObject const &p) override {
        //if (p.contains("target_q") && p["target_q"].isArray()) {
        QJsonValue jip = p["ip"].toString();
        QJsonValue jport = p["port"].toInt();
        QJsonValue jaddress = p["address"].toInt();
        //auto jfunction = p["function"].toInt();
        QJsonValue jnumber = p["numbers"].toInt();
        QJsonValue jvalue = p["data"].toString();

        lineEdits_ip->setText(jip.toString());
        lineEdits_port->setText(QString::number(jport.toInt()));
        lineEdits_address->setText(QString::number(jaddress.toInt()));
        lineEdits_number->setText(QString::number(jnumber.toInt()));
        plainText->setPlainText(jvalue.toString());
    }
    QJsonObject toJsonObject(std::vector<bool> data) {
      QJsonObject jsonObject;
      for (size_t i = 0; i < data.size(); ++i) {
        jsonObject.insert(QString::number(i), QJsonValue(data[i]));
      }
      return jsonObject;
    }
    QJsonObject toJsonObject(std::vector<uint16_t> data) {
      QJsonObject jsonObject;
      for (size_t i = 0; i < data.size(); ++i) {
        jsonObject.insert(QString::number(i), QJsonValue(data[i]));
      }
      return jsonObject;
    }

//    QJsonArray toJsonArray(bool* data) const {
//        QJsonArray jsonArray;
//        for (int data->itr = 0; i < ; ++i) {
//            jsonArray.append(QJsonValue(data[i]));
//        }
//        return jsonArray;
//    }
//    QJsonArray toJsonArray(std::vector<uint16_t> data) const {
//        QJsonArray jsonArray;
//        for (size_t i = 0; i < _write_registers.size(); ++i) {
//            jsonArray.append(QJsonValue(_write_registers[i]));
//        }
//        return jsonArray;
//    }

    std::vector<bool> fromJsonObjectBool(QJsonObject jsonObject) {
      std::vector<bool> data(jsonObject.size());
      for (auto it = jsonObject.begin(); it != jsonObject.end(); ++it) {
        data[it.key().toInt()] = it.value().toBool();
      }
      return data;
    }
    std::vector<uint16_t> fromJsonObjectInt(QJsonObject jsonObject) {
      std::vector<uint16_t> data(jsonObject.size());
      for (auto it = jsonObject.begin(); it != jsonObject.end(); ++it) {
        data[it.key().toInt()] = it.value().toInt();
      }
      return data;
    }
    QWidget *embeddedWidget() override {
        //연결된 plc station 확인하는 기능 ip port에 해당.
        //각 MODBUS_TCP 8종의 요청에 맞는 폼을 작성.
        if(!_widget) {
            _widget = new QWidget();

            QVBoxLayout* mainLayout = new QVBoxLayout();

            QGroupBox *groupBox1 = new QGroupBox("IP/PORT");
            QHBoxLayout *layout1 = new QHBoxLayout(groupBox1);
            lineEdits_ip = new QLineEdit("192.168.0.32");
            lineEdits_port = new QLineEdit("502");
            layout1->addWidget(lineEdits_ip);
            layout1->addWidget(lineEdits_port);
            layout1->setContentsMargins(0, 0, 0, 0);
            layout1->setSpacing(0);
            QGroupBox *groupBox2 = new QGroupBox("WRITE VALUES FROM PLC");
            QVBoxLayout *layout2 = new QVBoxLayout(groupBox2);
            QLabel* label_adrs = new QLabel("Starting address");
            lineEdits_address = new QLineEdit("0");

//            writeSingleCoil = new QPushButton("Write single coil");
//            writeSingleResister = new QPushButton("Write single resister");
            writeMultipleCoils = new QPushButton("Write multiple coils");
//            writeMultipleResisters = new QPushButton("Write multiple resisters");
            //read_WriteMultipleResisters = new QPushButton("Write multiple resisters");
//            Read_WriteMultipleResisters = 23
//            readCoils->setCheckable(true);
//            readDiscreteInputs->setCheckable(true);
//            readHoldingResisters->setCheckable(true);
//            readInputResisters->setCheckable(true);
//            buttonGroup = new QButtonGroup();
//            buttonGroup->addButton(writeSingleCoil, 0);
//            buttonGroup->addButton(writeSingleResister, 1);
//            buttonGroup->addButton(writeMultipleCoils, 15);
//            buttonGroup->addButton(writeMultipleResisters, 16);
//            for (QAbstractButton *button : buttonGroup->buttons()) {
//                button->setCheckable(true);
//            }
//            buttonGroup->setExclusive(true);

            lineEdits_number = new QLineEdit("1");
            QLabel* label_number = new QLabel("number of data");
            // QLabel* label_data = new QLabel("Data to Write");
            plainText = new QPlainTextEdit();

            add_button = new QPushButton("add");
            combo_coil = new QComboBox();

            // QPushButton* remove_button = new QPushButton("delete");

            //data_edit.push_back();


            layout2->addWidget(label_adrs);
            layout2->addWidget(lineEdits_address);
            layout2->addWidget(label_number);
            layout2->addWidget(lineEdits_number);
            layout2->addWidget(writeMultipleCoils);
//            layout2->addWidget(writeMultipleResisters);

//            connect(buttonGroup, QOverload<QAbstractButton *, bool>::of(&QButtonGroup::buttonToggled),
//            [ = ](QAbstractButton * button, bool checked) {
//                _write_number = number_edit->text().toInt();

//                layout2->addWidget(label_data);
//                if (button == writeSingleCoil || button == writeMultipleCoils) {
//                    _write_coils.resize(_write_number);
//                    if(layout2->findChild(edit_register->objectName())) {
//                        layout2->removeWidget(edit_register);
//                    }
//                    combo_coil->addItem("True");
//                    combo_coil->addItem("False");
//                    layout2->addWidget(combo_coil);

//                } else {
//                    _write_registers.resize(_write_number);
//                    if(combo_coil) {
//                        delete combo_coil;
//                    }
//                    if(layout2->findChild(combo_coil->objectName())) {
//                        layout2->removeWidget(combo_coil);
//                    }
//                    layout2->addWidget(edit_register);
//                }
//                layout2->addWidget(add_button);
//                layout2->addWidget(remove_button);
//                _fuction == buttonGroup->id(button);

//                function에 따라 _write_coils or registers 중 1개가 plainText를 통해 사용자가 배열을 입력
//                plainText->insertPlainText();
//                _write_number 로 => plaintext에 추가
//            });

            //layout2->addWidget(label_data);

//            connect(combo_coil, QOverload<int>::of(&QComboBox::currentIndexChanged),
//                    []() { qDebug() << combo_coil->currentText(); });
            connect(combo_coil, QOverload<int>::of(&QComboBox::currentIndexChanged),
                    [this]() { combo_coil->currentText(); });
            layout2->addWidget(new QLabel("Value to write"));
            layout2->addWidget(plainText);
            plainText->setEnabled(true);
            layout2->setContentsMargins(0, 0, 0, 0);
            layout2->setSpacing(0);
            mainLayout->addWidget(groupBox1);
            mainLayout->addWidget(groupBox2);
            _widget->setLayout(mainLayout);

        }
        return _widget;
    }
    // void saveGuiTask(TaskPlanner& planner) override {
    //     planner.taskPushBackPLCModbusCmd(planner.bp_gui_node_task_, CNC_LED_RED_ON);
    // }
//taskmanager의 실행중 통신으로 부터 전달받음~port값변경
public Q_SLOTS:
    void readDataFromMODBUS() {
    }
protected:
    QString _name;
    std::weak_ptr<DoubleData> _number1;
    std::weak_ptr<DoubleData> _number2;
    //tcp read data to result
    std::shared_ptr<DoubleData> _result;


    std::string _ip;
    uint16_t _port;
    uint16_t _address;
    PLC_MODBUS_TCP_WRITE _fuction;
    int _write_number;
    std::vector<bool> _write_coils;
    std::vector<uint16_t> _write_registers;

    QWidget* _widget;
    QLineEdit* lineEdits_ip;
    QLineEdit* lineEdits_port;
    QLineEdit* lineEdits_address;
    QButtonGroup* buttonGroup;
    QPushButton* writeSingleCoil;
    QPushButton* writeSingleResister;
    QPushButton* writeMultipleCoils;
    QPushButton* writeMultipleResisters;
    //QPushButton* read_WriteMultipleResisters;
    QLineEdit* lineEdits_number;

    QVector<QLineEdit*> data_edit;
    QPushButton* add_button;
    QPushButton* remove_button;

    QComboBox* combo_coil;
    QLineEdit* edit_register;
    QPlainTextEdit* plainText;  // text -> write to vectors send to manager
};


class PLC_MODBUS_TCP_READ_COIL : public NodeDelegateModel
{
    Q_OBJECT
    //Q_DECLARE_METATYPE(KCRCommand)
public:
    enum PLC_MODBUS_TCP_READ {
        ReadCoils = 1,
        ReadDiscreteInputs,
        ReadHoldingResisters,
        ReadInputResisters,
        Read_WriteMultipleResisters = 23
    };
    Q_ENUM(PLC_MODBUS_TCP_READ)

    PLC_MODBUS_TCP_READ_COIL() : _widget(nullptr) {
        _name = "MODBUS TCP READ COIL";
        _read_coils.resize(0);
        _read_registers.resize(0);
    }
    ~PLC_MODBUS_TCP_READ_COIL() = default;
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

    QString caption() const override { return _name; }
    QString name() const override { return _name; }

    NodeDataType dataType(PortType portType, PortIndex portIndex) const override {
        return NodeDataType{"robotCommand", tr("%1").arg(_name)};
    }

    void setWidgetExpand(bool flag) override {
        _widget->setVisible(flag);
    }

    void updateWidgetFromJson(QJsonArray& jarr, std::vector<double>& vec, QVector<QLineEdit*>& lineEdits) {
        if (jarr.size() != vec.size()) {
           // throw std::runtime_error("Size mismatch between QJsonArray and std::vector");
           return;
        } else {
            for (size_t i = 0; i < jarr.size(); ++i) {
                const QJsonValue& jsonValue = jarr[i];
                if (jsonValue.isDouble()) {
                    vec[i] = jsonValue.toDouble();
                }
                else {
                    return;
                    //throw std::runtime_error("Invalid element type in QJsonArray");
                }
            }
            for (size_t i = 0; i < vec.size(); ++i) {
                lineEdits[i]->setText(QString::number(vec[i]));
            }
        }
    }

    QJsonObject save() const override {
        QJsonObject modelJson = NodeDelegateModel::save();
        modelJson["ip"] = lineEdits_ip->text();
        modelJson["port"] = lineEdits_port->text().toInt();
        modelJson["protocol"] = Protocol::MODBUS_TCP;
        modelJson["function"] = PLC_MODBUS_TCP_READ::ReadCoils;
        modelJson["address"] = lineEdits_address->text().toInt();
        modelJson["numbers"] = lineEdits_numval->text().toInt();
        return modelJson;
    }
    void load(QJsonObject const &p) override {
        QJsonValue jip = p["ip"].toString();
        QJsonValue jport = p["port"].toInt();
        QJsonValue jaddress = p["address"].toInt();
        QJsonValue jnumber = p["numbers"].toInt();
        lineEdits_ip->setText(jip.toString());
        lineEdits_port->setText(QString::number(jport.toInt()));
        _fuction = PLC_MODBUS_TCP_READ::ReadCoils;
        lineEdits_address->setText(QString::number(jaddress.toInt()));
        lineEdits_numval->setText(QString::number(jnumber.toInt()));
    }

    QWidget *embeddedWidget() override {
        //연결된 plc station 확인하는 기능 ip port에 해당.
        //각 MODBUS_TCP 8종의 요청에 맞는 폼을 작성.
        if(!_widget) {
            _widget = new QWidget();

            QVBoxLayout* mainLayout = new QVBoxLayout();

            QGroupBox *groupBox1 = new QGroupBox("IP/PORT");
            QHBoxLayout *layout1 = new QHBoxLayout(groupBox1);

            lineEdits_ip = new QLineEdit("192.168.0.20");
            lineEdits_port = new QLineEdit("502");

            layout1->addWidget(lineEdits_ip);
            layout1->addWidget(lineEdits_port);
            layout1->setContentsMargins(0, 0, 0, 0);
            layout1->setSpacing(0);
            QGroupBox *groupBox2 = new QGroupBox("READ COILS FROM PLC");
            QVBoxLayout *layout2 = new QVBoxLayout(groupBox2);
            QLabel* label_adrs = new QLabel("Starting address");
            lineEdits_address = new QLineEdit("0");
            QLabel* label_numval = new QLabel("Number of values");
            lineEdits_numval = new QLineEdit("1");

            layout2->addWidget(label_adrs);
            layout2->addWidget(lineEdits_address);
            layout2->addWidget(label_numval);
            layout2->addWidget(lineEdits_numval);

            layout2->setContentsMargins(0, 0, 0, 0);
            layout2->setSpacing(0);

//TO DO : connect
            //lineEdits_address : 0~65535
            //lineEdits_numval  : 0~125
            connect(lineEdits_address, &QLineEdit::textChanged, this, [this](const QString& text) {
                _address = lineEdits_address->text().toInt();
            });

            connect(lineEdits_numval, &QLineEdit::textChanged, this, [this](const QString& text) {
                _portNumber = lineEdits_numval->text().toInt();
            });

            mainLayout->addWidget(groupBox1);
            mainLayout->addWidget(groupBox2);

            _widget->setLayout(mainLayout);

        }
        return _widget;
    }
//taskmanager의 실행중 통신으로 부터 전달받음~port값변경
public Q_SLOTS:
    void readDataFromMODBUS() {
    }
protected:
    QString _name;
    std::weak_ptr<DoubleData> _number1;
    std::weak_ptr<DoubleData> _number2;
    //tcp read data to result
    std::shared_ptr<DoubleData> _result;

    std::string _ip;
    uint16_t _port;
    uint16_t _address;
    int _portNumber;

    std::vector<bool> _read_coils;
    std::vector<uint16_t> _read_registers;

    PLC_MODBUS_TCP_READ _fuction;
    uint16_t _readValue;

    QWidget* _widget;
    //ptr for loading
    QButtonGroup* buttonGroup;
    QLineEdit* lineEdits_ip;
    QLineEdit* lineEdits_port;
    QLineEdit *lineEdits_address;
    QLineEdit *lineEdits_numval;
};
#endif