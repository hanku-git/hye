#include "PLCCommandModel.hpp"
#include "doubleData.hpp"

unsigned int PLC_MODBUS_TCP_ReadModel::nPorts(PortType portType) const
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

std::shared_ptr<NodeData> PLC_MODBUS_TCP_ReadModel::outData(PortIndex)
{
    return std::static_pointer_cast<NodeData>(_result);
}

void PLC_MODBUS_TCP_ReadModel::setInData(std::shared_ptr<NodeData> data, PortIndex portIndex)
{
//    auto numberData = std::dynamic_pointer_cast<DoubleData>(data);

//    if (!data) {
//        Q_EMIT dataInvalidated(0);
//    }

//    if (portIndex == 0) {
//        _number1 = numberData;
//    } else {
//        _number2 = numberData;
//    }

    //compute();
}

unsigned int PLC_MODBUS_TCP_READ_COIL::nPorts(PortType portType) const
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

std::shared_ptr<NodeData> PLC_MODBUS_TCP_READ_COIL::outData(PortIndex)
{
    return std::static_pointer_cast<NodeData>(_result);
}

void PLC_MODBUS_TCP_READ_COIL::setInData(std::shared_ptr<NodeData> data, PortIndex portIndex)
{
//    auto numberData = std::dynamic_pointer_cast<DoubleData>(data);

//    if (!data) {
//        Q_EMIT dataInvalidated(0);
//    }

//    if (portIndex == 0) {
//        _number1 = numberData;
//    } else {
//        _number2 = numberData;
//    }

    //compute();
}

unsigned int PLC_MODBUS_TCP_WriteModel::nPorts(PortType portType) const
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
//NodeDataType PLC_CommandModel::dataType(PortType, PortIndex) const
//{
//    return NodeDataType{"PLC_Command", "CS Move Parameter"};
//}

std::shared_ptr<NodeData> PLC_MODBUS_TCP_WriteModel::outData(PortIndex)
{
    return std::static_pointer_cast<NodeData>(_result);
}

void PLC_MODBUS_TCP_WriteModel::setInData(std::shared_ptr<NodeData> data, PortIndex portIndex)
{
//    auto numberData = std::dynamic_pointer_cast<DoubleData>(data);

//    if (!data) {
//        Q_EMIT dataInvalidated(0);
//    }

//    if (portIndex == 0) {
//        _number1 = numberData;
//    } else {
//        _number2 = numberData;
//    }

    //compute();
}
