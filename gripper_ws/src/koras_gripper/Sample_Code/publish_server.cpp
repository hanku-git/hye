#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "msg_srv_sample/msg/msg_sample.hpp"
#include "msg_srv_sample/srv/srv_sample.hpp"

rclcpp::Publisher<msg_srv_sample::msg::MsgSample>::SharedPtr publisher_1_;
rclcpp::Service<msg_srv_sample::srv::SrvSample>::SharedPtr server_1_;

void publishData(int input) {
    msg_srv_sample::msg::MsgSample data;

    int interval = 180 / 10;

    data.num  = input;
    data.data_1  = sin(input * M_PI / 180);
    data.data_2  = sin((input + interval) * M_PI / 180) * 0.95;
    data.data_3  = sin((input + interval * 2) * M_PI / 180) * 0.90;
    data.data_4  = sin((input + interval * 3) * M_PI / 180) * 0.85;
    data.data_5  = sin((input + interval * 4) * M_PI / 180) * 0.80;
    data.data_6  = sin((input + interval * 5) * M_PI / 180) * 0.75;
    data.data_7  = sin((input + interval * 6) * M_PI / 180) * 0.70;
    data.data_8  = sin((input + interval * 7) * M_PI / 180) * 0.65;
    data.data_9  = sin((input + interval * 8) * M_PI / 180) * 0.60;
    data.data_10 = sin((input + interval * 9) * M_PI / 180) * 0.55;

    data.data_noise = data.data_1 + sin((double) rand() / RAND_MAX * M_PI - M_PI / 2) 
                                    * 0.1 * ((double) rand() / RAND_MAX);

    publisher_1_->publish(data);
}

void serverCallback(const std::shared_ptr<msg_srv_sample::srv::SrvSample::Request> request,
                          std::shared_ptr<msg_srv_sample::srv::SrvSample::Response> response) {
    std::cout << "Test server called." << std::endl;
    std::cout << "Requested data: " << request->data << std::endl << std::endl;

    response->success = true;
}

int main(int argc, char * argv[]) {
    printf("hi\n");

    rclcpp::init(argc, argv);

    // node setting
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("pub_server_test");

    // publisher
    publisher_1_ = node->create_publisher<msg_srv_sample::msg::MsgSample>("test1", 10);

    // server
    server_1_ = node->create_service<msg_srv_sample::srv::SrvSample>("test_srv", &serverCallback);

    int cnt = 0;

    while(rclcpp::ok()) {
        usleep(1000);
        rclcpp::spin_some(node);
        publishData(cnt++);
    }

    rclcpp::shutdown();
    return 0;
}
