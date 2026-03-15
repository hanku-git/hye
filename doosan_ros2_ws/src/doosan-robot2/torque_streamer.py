import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from dsr_msgs2.srv import GetJointTorque

class TorqueStreamer(Node):
    def __init__(self):
        super().__init__('torque_streamer')
        self.cli = self.create_client(GetJointTorque, '/dsr01/aux_control/get_joint_torque')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('서비스 대기 중...')

        self.get_logger().info('서비스 연결됨. 토크 데이터 요청 시작!')
        self.publisher = self.create_publisher(Float64MultiArray, '/joint_torque_stream', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # 2Hz

    def timer_callback(self):
        req = GetJointTorque.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result():
            msg = Float64MultiArray()
            msg.data = future.result().torque
            self.publisher.publish(msg)
            self.get_logger().info(f'Published torque: {msg.data}')
        else:
            self.get_logger().warn('서비스 호출 실패')

def main(args=None):
    rclpy.init(args=args)
    node = TorqueStreamer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
