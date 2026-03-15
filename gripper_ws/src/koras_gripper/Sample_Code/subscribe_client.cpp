#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "grp_control_msg/msg/gripper_msg.hpp"
#include "grp_control_msg/srv/void.hpp"

rclcpp::Subscription<grp_control_msg::msg::GripperMsg>::SharedPtr grp_subscriber_;
rclcpp::Client<grp_control_msg::srv::Void>::SharedPtr grp_client_1_;
rclcpp::Client<grp_control_msg::srv::Void>::SharedPtr grp_client_2_;
rclcpp::Client<grp_control_msg::srv::Void>::SharedPtr grp_client_3_;

void topicCallback(const grp_control_msg::msg::GripperMsg::SharedPtr msg) {
    std::cout << "finger position: " << msg->finger_position <<
                 "\tmotor current: " << msg->motor_current << std::endl;
}

bool grpInit() {
    auto req = std::make_shared<grp_control_msg::srv::Void::Request>();

    auto result = grp_client_1_->async_send_request(req);
    auto status = result.wait_for(std::chrono::milliseconds(100));

    if (status == std::future_status::timeout) {
        printf("Failed to call service\n");
        printf("Function: %s\n", __FUNCTION__);
        return false;
    }

    auto recv = result.get();

    if (recv->successed) {
        printf("Function successed: %s\n", __FUNCTION__);
        return true;
    } else {
        printf("Function failed\n");
        return false;
    }
}

bool grpOpen() {
    auto req = std::make_shared<grp_control_msg::srv::Void::Request>();

    auto result = grp_client_2_->async_send_request(req);
    auto status = result.wait_for(std::chrono::milliseconds(100));

    if (status == std::future_status::timeout) {
        printf("Failed to call service\n");
        printf("Function: %s\n", __FUNCTION__);
        return false;
    }

    auto recv = result.get();

    if (recv->successed) {
        printf("Function successed: %s\n", __FUNCTION__);
        return true;
    } else {
        printf("Function failed\n");
        return false;
    }
}

bool grpClose() {
    auto req = std::make_shared<grp_control_msg::srv::Void::Request>();

    auto result = grp_client_3_->async_send_request(req);
    auto status = result.wait_for(std::chrono::milliseconds(100));

    if (status == std::future_status::timeout) {
        printf("Failed to call service\n");
        printf("Function: %s\n", __FUNCTION__);
        return false;
    }

    auto recv = result.get();

    if (recv->successed) {
        printf("Function successed: %s\n", __FUNCTION__);
        return true;
    } else {
        printf("Function failed\n");
        return false;
    }
}

int main(int argc, char * argv[]) {
    printf("hi\n");

    rclcpp::init(argc, argv);

    // node setting
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("sub_client_test");

    // subscriber setting
    grp_subscriber_ = node->create_subscription<grp_control_msg::msg::GripperMsg>("grp_state", 10, &topicCallback);

    // client setting
    grp_client_1_ = node->create_client<grp_control_msg::srv::Void>("gripper_initialize");
    grp_client_2_ = node->create_client<grp_control_msg::srv::Void>("grp_open");
    grp_client_3_ = node->create_client<grp_control_msg::srv::Void>("grp_close");

    // gripper init
    grpInit();
    sleep(5);

    int cnt = 0;

    while(rclcpp::ok()) {
        usleep(10000);
        rclcpp::spin_some(node);

        // 약 2초마다 그리퍼 열고 닫고 반복
        if (cnt++ == 200) {
            grpOpen();
        } else if (cnt >= 400) {
            grpClose();
            cnt = 0;
        }
    }

    rclcpp::shutdown();
    return 0;
}
