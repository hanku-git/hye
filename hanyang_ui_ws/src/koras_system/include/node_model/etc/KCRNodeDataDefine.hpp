#include <QtNodes/NodeData>
using QtNodes::NodeData;
using QtNodes::NodeDataType;

//1:1 Match for command with some Model type based on KCRCommandModel

//struct NODE_JS_TARGET_PATH : NodeData {
//    std::vector<double> target_q;
//    std::vector<double> target_qd;
//    std::vector<double> target_qdd;
//    KCRCommand command = KCRCommand::JS_TARGET_PATH;
//    NodeDataType type() const override { return NodeDataType{"robotCommand", "JS_TARGET_PATH"}; }
//};
//struct NODE_CS_TARGET_PATH : NodeData {
//    std::vector<double> target_x;
//    std::vector<double> target_xd;
//    std::vector<double> target_xdd;
//    KCRCommand command = KCRCommand::CS_TARGET_PATH;
//    NodeDataType type() const override { return NodeDataType{"robotCommand", "CS_TARGET_PATH"}; }
//};
