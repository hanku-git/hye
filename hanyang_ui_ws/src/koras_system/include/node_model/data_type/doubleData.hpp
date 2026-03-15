#ifndef DOUBLEDATA_HPP_
#define DOUBLEDATA_HPP_

#include <QtNodes/NodeData>

using QtNodes::NodeData;
using QtNodes::NodeDataType;

/// The class can potentially incapsulate any user data which
/// need to be transferred within the Node Editor graph
class DoubleData : public NodeData
{
public:
    DoubleData()
        : _number(0.0)
    {}

    DoubleData(double const number)
        : _number(number)
    {}

    NodeDataType type() const override { return NodeDataType{"double", "double"}; }

    double number() const { return _number; }

    QString numberAsText() const { return QString::number(_number, 'd'); }

private:
    double _number;
};
#endif