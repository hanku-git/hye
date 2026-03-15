
#ifndef PY_UI_BRIDGE_NODE_HPP_
#define PY_UI_BRIDGE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <QThread>

#include <thread>

#include "ui_define.hpp"

#include <bin_picking/nlohmann/json.hpp>
#include <time.h>

#include <std_msgs/msg/string.hpp>


/**
 * @class PyUiBridgeNode
 * @brief 패키지 간 통신을 위한 ROS Node 정의 클래스
 * @ingroup USER_INTERFACE
 */
class PyUiBridgeNode : public QThread {
	Q_OBJECT
public:
	PyUiBridgeNode(int argc, char **argv );
	virtual ~PyUiBridgeNode();

Q_SIGNALS:
	void rosShutdown();

public:
    shared_ptr<rclcpp::Node> node_;
	bool init();
	void spinNode();
	void python_to_cpp_callback(const std_msgs::msg::String &msg);


private:
	int init_argc;
	char** init_argv;

    std::thread spin_thread_;

    //// Python UI
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr py_ui_subscription_;

};


#endif
