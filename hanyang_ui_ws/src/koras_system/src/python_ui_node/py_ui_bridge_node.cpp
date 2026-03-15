
// #include <ros/ros.h>
// #include <ros/network.h>
#include <python_ui_node/py_ui_bridge_node.hpp>
#include <fstream>
#include <iostream>
#include <random>


PyUiBridgeNode::PyUiBridgeNode(int argc, char **argv )
    : init_argc(argc), init_argv(argv) {

}

PyUiBridgeNode::~PyUiBridgeNode() {
    rclcpp::shutdown(); // explicitly needed since we use ros::start();
    wait();
}

/** @brief Korean: PyUiBridgeNode 클래스의 초기화를 수행한다.
 * @return true: PyUiBridgeNode 실행 성공, false: PyUiBridgeNode 실행 실패
 */
bool PyUiBridgeNode::init() {
    // string prefix = ROBOT_NAME;
    // prefix += "_";

    node_ = rclcpp::Node::make_shared("py_ui_bridge_node");

    // Python UI - callbackFunction
    py_ui_subscription_ = node_->create_subscription<std_msgs::msg::String>(
        "python_to_cpp", 10, [this](const std_msgs::msg::String::SharedPtr msg) {

        //// COPY THIS CODE
        ROS_LOG_WARN("Received from Python: '%s'", msg->data.c_str());
        if (msg->data == "py_msg_start_recording") {
            ROS_LOG_WARN("***** START RECORDING! *****");
            // TODO:CODE HERE


        } else if(msg->data == "py_msg_start_progress") {
            ROS_LOG_WARN("***** START PROGRESS! *****");
            // TODO:CODE HERE


        } else if(msg->data == "py_msg_stop_and_reset") {
            ROS_LOG_WARN("***** STOP AND RESET! *****");
            // TODO:CODE HERE


        }

    });

    // py_ui_subscription_ = node_->create_subscription<std_msgs::msg::String>("python_to_cpp", 1, std::bind(&PyUiBridgeNode::python_to_cpp_callback, this, std::placeholders::_1));

    // ✅ ROS 2 스핀을 별도 스레드에서 실행 (메시지를 지속적으로 수신)
    spin_thread_ = std::thread([this]() {
        rclcpp::spin(node_);
    });

    printf("py_ui_bridge_node initialized and listening to python_to_cpp.\n");

    return true;
}

void PyUiBridgeNode::spinNode() {
    rclcpp::spin_some(node_);
}

void PyUiBridgeNode::python_to_cpp_callback(const std_msgs::msg::String &msg)
{
    ROS_LOG_WARN("Received from Python: '%s'", msg.data.c_str());
    if (msg.data == "button_pressed") {
        ROS_LOG_WARN("Python 버튼 이벤트 감지!");
    }
}





























