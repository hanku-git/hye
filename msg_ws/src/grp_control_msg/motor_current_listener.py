#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from grp_control_msg.msg import GripperMsg  # 메시지 타입 import

class MotorCurrentListener(Node):
    def __init__(self):
        super().__init__('motor_current_listener')
        self.subscription = self.create_subscription(
            GripperMsg,
            'gripper_status',  # 메시지가 게시되는 토픽 이름 (올바르게 설정해야 함)
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'Motor Current: {msg.motor_current}')

def main(args=None):
    rclpy.init(args=args)
    node = MotorCurrentListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
