import os
import rclpy
from std_msgs.msg import String
from llm_node.ros_setup import LLMNode
from llm_node.model_kiwi import STTModel, KIWIModel

BASE_DIR = os.environ.get("ROBOT_BASE_DIR", os.path.expanduser("~"))
filepath = BASE_DIR + "/llm_ws/src/record/"
rec_file = os.path.join(filepath, "recording.wav")

def main(args=None):
    """ROS 2 LLM Node 실행"""
    rclpy.init(args=args)
    rosnode = LLMNode(rec_file)  # ROS 2 노드 생성
    stt = STTModel(rec_file)
    llm = KIWIModel()

    if os.path.isfile(rec_file):
        os.remove(rec_file)

    print("✅ LLM Node Ready")

    # ★ 여기에 "LLM Node Ready"를 퍼블리시
    ready_msg = String()
    ready_msg.data = "LLM Node Ready"
    rosnode.pub_status.publish(ready_msg)  # ← 추가

    # ROS 2 이벤트 루프 실행
    rclpy.spin(rosnode)
    rclpy.shutdown()

if __name__ == "__main__":
    main()