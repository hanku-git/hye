#include "BPCommandModel.hpp"
#include "doubleData.hpp"

//BPCommandModel::BPCommandModel()
//    : //_number(std::make_shared<CSMoveNodeData>()),
//      //_lineEditsCS(),
//      _widget(nullptr)
//{}
    QJsonObject BPCommandModel::save() const {
        QJsonObject modelJson = NodeDelegateModel::save();
        modelJson["protocol"] = Protocol::ROS2;
        return modelJson;
    }

    void BPCommandModel::setWidgetExpand(bool flag) {
        _widget->setVisible(flag);
    }

    void BPCommandModel::updateWidgetFromJson(QJsonArray& jarr, std::vector<double>& vec, QVector<QLineEdit*>& lineEdits) {
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
unsigned int BPCommandModel::nPorts(PortType portType) const
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

std::shared_ptr<NodeData> BPCommandModel::outData(PortIndex)
{
    return std::static_pointer_cast<NodeData>(_result);
}

void BPCommandModel::setInData(std::shared_ptr<NodeData> data, PortIndex portIndex)
{

}
