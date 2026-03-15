import os
import time
import rclpy

from llm_node.ros_setup import LLMNode
# from llm_node.model_message_en import STTModel, LLMModel
# from llm_node.model_message_kr import STTModel, LLMModel
# from llm_node.model_structured import STTModel, LLMModel
from llm_node.model_kiwi import STTModel, KIWIModel
# from llm_node.model_test import STTModel, LLMModel


filepath = "/home/bp/llm_ws/src/record/"
rec_file = filepath + 'recording.wav'

def main (args=None):
    #ROSInit
    rclpy.init(args=args)
    rosnode = LLMNode(rec_file)
    stt = STTModel(rec_file)
    llm = KIWIModel()
    #if os.path.isfile(rec_file):
        #os.remove(rec_file)

    print("LLM Node ready")

#     # 이미지로부터 텍스트를 수동으로 입력
#     image_text = """
# 실린더 2개를 회색 상자에 넣어
# 사각형 부품 2개를 바란색에 넣어
# 가장 많은 부품을 B하고 노란상자로
# 스퀘어 팩 두개를 A작업 하고 파란상자에 담아.
# 제일 수가 적은걸 노란색으로
# B를 한 볼트우시 2개를 회색 상자로
# 원기둥 부품 파란색
# 3개 있는 부품을 B작업 해서 회색상자에 넣어
# 지금 2개만 있는거를 파란상자로 옮겨
# 4개 있는 부품 2개를 A작업 하고 노란상자로 옮겨
# 볼투브 씨를 A하고 노란색에
# 실린더를 전부 B를 하고 파란색에 넣어
# 사각펙을 전부 B를 하고 파란색에 넣어
# 볼트브시를 전부 B를 하고 파란색에 넣어
# 가장 적은거를 전부 A 거쳐서 회색으로 옮겨
# 가장 조금 있는 부품을 노란상자에 담아
# 가장 많은 부품을 모두 다 파란상자에 넣어
# """

#     image_text = """
# 실린더를 회색 상자에 넣어 경유지는 A B를 지나가
# 사각형 부품을 B지점 지나서 바란색에 넣어
# 가장 많은 부품을 B하고 노란상자로
# 스퀘어 팩 두개를 A작업 하고 파란상자에 담아.
# 제일 수가 적은걸 노란색으로
# B를 한 볼트우시 2개를 회색 상자로
# 원기둥 부품 파란색
# 3개 있는 부품을 B작업 해서 회색상자에 넣어
# 지금 2개만 있는거를 파란상자로 옮겨
# 4개 있는 부품 2개를 A작업 하고 노란상자로 옮겨
# 볼투브 씨를 A하고 노란색에
# 실린더를 전부 B를 하고 파란색에 넣어
# 사각펙을 전부 B를 하고 파란색에 넣어
# 볼트브시를 전부 B를 하고 파란색에 넣어
# 가장 적은거를 전부 A 거쳐서 회색으로 옮겨
# 가장 조금 있는 부품을 노란상자에 담아
# 가장 많은 부품을 모두 다 파란상자에 넣어
# """

#     # 개별 명령어로 분리
#     commands = image_text.strip().split("\n")

#     obj_num = list(rosnode.obj_dic.values())
#     for input_text in commands:
#         input_text = input_text.strip()
#         if not input_text:
#             continue
#         start = time.time()
#         print("input text: \n",input_text, str(obj_num))
#         output = llm.get_result(input_text, obj_num)
#         # output = llm.get_result(input_text)
#         end = time.time()
#         print("output text: \n", output)
#         # print("#"*25)
#         # result = eval(output)
#         # print("output list: \n", result)
#         print(f"runtime: {end - start:.5f}sec")
#         print("\n" + "-"*50 + "\n")
    
#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()



#     while rclpy.ok():

#         if os.path.isfile(rec_file):
#             input_text = stt.get_text()     
#             obj_num = list(rosnode.obj_dic.values())

#             start = time.time()
#             print("input text: \n",input_text, str(obj_num))
#             result = llm.get_result(input_text, obj_num)
#             end = time.time()
#             # print("output text: \n", output)
#             print("output list: \n", result)
#             print(f"runtime: {end - start:.5f}sec")
#             print("\n" + "-"*50 + "\n")
        
#             # rosnode.send_request(result)
#             os.rename(rec_file, filepath + 'last_recording.wav')  
#         else:
#             pass
#         rclpy.spin_once(rosnode)

#     rclpy.shutdown()

# if __name__ == "__main__":
#     main()


    while rclpy.ok():

        #if os.path.isfile(rec_file):
            #input_text = stt.get_text()     
                 
            input_text = input("\ninput: ")
            start = time.time()
            print("input text: \n", input_text)
            output = llm.get_result(input_text, debug=True)
            end = time.time()
            print("output text: \n", output)
            print(f"runtime: {end - start:.5f}sec")
        
            #rosnode.send_request(output)
            #os.rename(rec_file, filepath + 'last_recording.wav')  
        #else:
            #pass
        #rclpy.spin_once(rosnode)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
