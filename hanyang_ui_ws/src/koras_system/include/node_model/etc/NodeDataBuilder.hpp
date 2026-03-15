#include <QtNodes/NodeData>
#include <iostream>
#include <concepts>
#include <vector>
#include <string>
#include "KCRNodeDataDefine.hpp"

#include <QtCore/QJsonValue>
#include <QtGui/QDoubleValidator>
#include <QtWidgets/QLineEdit>
#include <QHBoxLayout>
#include <QJsonObject>

using QtNodes::NodeData;
using QtNodes::NodeDataType;
using QtNodes::NodeDelegateModel;
using QtNodes::PortIndex;
using QtNodes::PortType;
using QtNodes::ConnectionPolicy;

template <typename T>
concept ValidContainer2 = std::same_as<T, std::vector<typename T::value_type>> ||
                          std::same_as<T, std::vector<std::vector<double>>>;

// Base case: class with no members
template <typename... Args>
class NodeDataBase {};

// Recursive case: class with n members
template <typename T, typename... Args>
class NodeDataBase<T, Args...> : public NodeDataBase<Args...> {
private:
    T member;

public:
    NodeDataBase(const T& val, const Args&... args) : member(val), NodeDataBase<Args...>(args...) {}

    T getMember() const {
        return member;
    }
};

// Specialization for vector types and nested vector types
template <ValidContainer2... Args>
class NodeDataBase<std::vector<Args...>> : public NodeDataBase<Args...> {
private:
    std::vector<Args...> member;

public:
    NodeDataBase(const std::vector<Args...>& val, const Args&... args) : member(val), NodeDataBase<Args...>(args...) {}

    std::vector<Args...> getMember() const {
        return member;
    }
};

//struct NODE_JS_TARGET_PATH : NodeData {
//    std::vector<double> target_q;
//    std::vector<double> target_qd;
//    std::vector<double> target_qdd;
//    KCRCommand command = JS_TARGET_PATH;
//    NodeDataType type() const override { return NodeDataType{"robotCommand", "JS_TARGET_PATH"}; }
//};

//struct NODE_CS_TARGET_PATH : NodeData {
//    std::vector<double> target_x;
//    std::vector<double> target_xd;
//    std::vector<double> target_xdd;
//    KCRCommand command = CS_TARGET_PATH;
//    NodeDataType type() const override { return NodeDataType{"robotCommand", "CS_TARGET_PATH"}; }
//};


class NodeModelBuilder {

public: NodeModelBuilder() {}

    ~NodeModelBuilder() = default;

    // 1. data struct가 들어감..
    // 2. struct의 시리얼라이즈 함수로 멤버를 인식?
    // 3. 동시에 사용되는 변수는 통일해서 공유해서 써도될까? 공유포인터 사용
//    template <typename T>
//    AbstractModel* createModel()
//    {

//        return new AbstractModel();
//    }
};

//QJsonObject printValueType(std::floating_point auto a, QJsonObject& modelJson, QString name) {
//    modelJson[name] = a;
//    return modelJson;
//}

template<typename T>
concept ValidContainer =
    std::same_as<T, std::vector<typename T::value_type>>;
template<typename T>
concept Sortable =
    ValidContainer<T> && std::same_as<typename T::value_type, double>;

QJsonObject printValueType(Sortable auto& a, QJsonObject& modelJson, QString name) {
    QString index;
    qDebug() << a.size();
    for(int i = 0; i < a.size(); i++) {
        index = name + QString::number(i + 1);
        qDebug() << index;
        modelJson[index] = a[i];
    }
    return modelJson;
}

//// 추상 클래스를 상속하는 클래스
class AbstractModel : public NodeDelegateModel
{
    Q_OBJECT

public:
    QString caption() const override { return QStringLiteral("CS Move"); }

    bool captionVisible() const override { return false; }

    bool setWidgetVisible(bool flag) {
        if(embeddedWidget() != nullptr) {
            embeddedWidget()->setVisible(flag);
        }
    }

    QString name() const override { return QStringLiteral("CS Motion"); }

public:
    void setNumber(double number);
    //void getImage(QString image) {return _modelImage;}

private Q_SLOTS:

private:
    //std::shared_ptr<CSMoveNodeData> _number;
    std::vector<QLineEdit*> _lineEditsCS;
    QWidget* _widget;
    QVBoxLayout *vl;
    //connect to task items
    //const QString _modelImage = ":items/move.png";

    AbstractModel()
        : //_number(std::make_shared<CSMoveNodeData>()),
          _lineEditsCS(),
          _widget(nullptr)
    {
        NodeDataBase base;
    }

    QJsonObject save() const override
    {
        QJsonObject modelJson = NodeDelegateModel::save();
//        std::vector<QString> text_vector = _number->paramsAsText();
//        modelJson["x"] = text_vector[0];
//        modelJson["y"] = text_vector[1];
//        modelJson["z"] = text_vector[2];
//        modelJson["R"] = text_vector[3];
//        modelJson["P"] = text_vector[4];
//        modelJson["Y"] = text_vector[5];
//        modelJson["command"] = kcrRobotCommand::CS_TARGET_PATH;

        std::vector<double> s;
        s.push_back(0.0);
        s.push_back(0.0);
        s.push_back(0.0);
        s.push_back(0.0);

        printValueType(s, modelJson, "plus1");
        //printValueType(2.15151, modelJson, "plus2");

        qDebug() << modelJson;
        return modelJson;
    }

    void load(QJsonObject const &p) override
    {
        QJsonValue v1 = p["x"];
        QJsonValue v2 = p["y"];
        QJsonValue v3 = p["z"];
        QJsonValue v4 = p["R"];
        QJsonValue v5 = p["P"];
        QJsonValue v6 = p["Y"];

        double d1 = 0.0;
        double d2 = 0.0;
        double d3 = 0.0;
        double d4 = 0.0;
        double d5 = 0.0;
        double d6 = 0.0;

        auto check = [&](QJsonValue& v, double& d) mutable {
            QString strNum = v.toString();
            if (!v.isUndefined()) {
                bool ok;
                d = strNum.toDouble(&ok);
            }
            return strNum;
        };

//        param_CS param;
//        param.x = d1;
//        param.y = d2;
//        param.z = d3;
//        param.R = d4;
//        param.P = d5;
//        param.Y = d6;
//        _number = std::make_shared<CSMoveNodeData>(param);

        if (_lineEditsCS.size() > 0 ) {
            _lineEditsCS[0]->setText(check(v1, d1));
            _lineEditsCS[1]->setText(check(v2, d2));
            _lineEditsCS[2]->setText(check(v3, d3));
            _lineEditsCS[3]->setText(check(v4, d4));
            _lineEditsCS[4]->setText(check(v5, d5));
            _lineEditsCS[5]->setText(check(v6, d6));
        }
    }

    unsigned int nPorts(PortType portType) const
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

    template<typename T>
    NodeDataType dataType(PortType, PortIndex) const
    {
        //return CSMoveNodeData().type();
    }

    std::shared_ptr<NodeData> outData(PortIndex)
    {
        //return std::make_shared<CSMoveNodeData>();
    }

    QWidget *embeddedWidget()
    {
        if(!_widget) {
            _widget = new QWidget;
            vl = new QVBoxLayout(_widget);
            _widget->setSizePolicy(QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Minimum);

            vl->setContentsMargins(0, 0, 0, 0);
            vl->setSpacing(0);

    //        auto left = new QVBoxLayout();
    //        left->setSpacing(0);
    //        left->setContentsMargins(0, 0, 0, 0);
    //        //left->addStretch();

            auto main_label = new QLabel("CS move");
            auto lineEdit1 = new QLineEdit("0.0");
            auto lineEdit2 = new QLineEdit("0.0");
            auto lineEdit3 = new QLineEdit("0.0");
            auto lineEdit4 = new QLineEdit("0.0");
            auto lineEdit5 = new QLineEdit("0.0");
            auto lineEdit6 = new QLineEdit("0.0");

            _lineEditsCS.push_back(lineEdit1);
            _lineEditsCS.push_back(lineEdit2);
            _lineEditsCS.push_back(lineEdit3);
            _lineEditsCS.push_back(lineEdit4);
            _lineEditsCS.push_back(lineEdit5);
            _lineEditsCS.push_back(lineEdit6);

            vl->addWidget(main_label);

            for(int i = 0; i < 6 ; i++) {
                connect(_lineEditsCS[i], &QLineEdit::textChanged, [this, i](const QString &str) {
                    bool ok = false;

                    double number = str.toDouble(&ok);
                    if (ok) {
                        switch (i) {
//                        case 0:
//                            _number->setParameter_x(number);
//                            break;
//                        case 1:
//                            _number->setParameter_y(number);
//                            break;
//                        case 2:
//                            _number->setParameter_z(number);
//                            break;
//                        case 3:
//                            _number->setParameter_R(number);
//                            break;
//                        case 4:
//                            _number->setParameter_P(number);
//                            break;
//                        case 5:
//                            _number->setParameter_Y(number);
//                            break;
                        default:
                            break;
                        }
                        Q_EMIT dataUpdated(0);

                    } else {
                        Q_EMIT dataInvalidated(0);
                    }

                });
                vl->addWidget(_lineEditsCS[i]);
            }

    //        hl->addLayout(left);
    //        hl->addSpacing(0);
            //hl->addStretch();
            //hl->setAlignment(_widget, Qt::AlignCenter);

        }
        return _widget;
    }

    void setWidgetExpand(bool flag)
    {
        _widget->setVisible(flag);
    }


};
