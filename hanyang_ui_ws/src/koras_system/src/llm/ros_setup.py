from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
import sounddevice as sd
from scipy.io.wavfile import write
import numpy as np
import os
from llm_node.model_kiwi import STTModel, KIWIModel

class LLMNode(Node):
    def __init__(self, filepath):
        """ROS 2 노드 초기화"""
        super().__init__('LLM_node')
        self.filepath = os.path.join(os.path.expanduser("~"), "Desktop/aw_cooking/recordings/voice_input.wav")

        self.is_recording = False
        self.recording_data = []
        self.stream = None

        self.stt_model = STTModel(self.filepath)
        self.kiwi_model = KIWIModel()

        # ROS 서비스 및 퍼블리셔 설정
        self.pub_status = self.create_publisher(String, 'llm_node/status', 10)
        self.srv_start_rec = self.create_service(Trigger, 'llm_node/start_rec', self.start_rec_callback)
        self.srv_stop_rec = self.create_service(Trigger, 'llm_node/stop_rec', self.stop_rec_callback)

    def audio_callback(self, indata, frames, time, status):
        """음성 데이터를 저장하는 콜백 함수"""
        if status:
            self.get_logger().error(f"음성 입력 오류: {status}")
        self.recording_data.append(indata.copy())

    def start_recording(self):
        """🔴 UI 버튼을 눌렀을 때만 녹음 시작"""
        if self.is_recording:
            self.get_logger().warning("⚠️ 이미 녹음 중입니다.")
            return

        self.is_recording = True
        self.recording_data = []

        try:
            self.stream = sd.InputStream(samplerate=44100, channels=2, callback=self.audio_callback)
            self.stream.start()
            self.get_logger().info("🔴 녹음 시작")
        except Exception as e:
            self.get_logger().error(f"🎙️ 녹음 시작 실패: {str(e)}")
            self.is_recording = False

    def stop_recording(self):
        """🛑 녹음 종료 및 STT 변환"""
        if not self.is_recording:
            self.get_logger().warning("⚠️ 녹음이 시작되지 않았습니다.")
            return

        try:
            self.stream.stop()
            self.stream.close()
            self.is_recording = False
            self.save_recording()
            self.get_logger().info("🛑 녹음 종료")

            # 🎙️ STT 변환 실행
            self.process_stt()

        except Exception as e:
            self.get_logger().error(f"🎙️ 녹음 종료 실패: {str(e)}")
            self.is_recording = False  # 예외 발생 시 녹음 상태 초기화

    def save_recording(self):
        """💾 녹음된 데이터를 파일로 저장"""
        if not self.recording_data:
            self.get_logger().warning("⚠️ 녹음 데이터가 없습니다.")
            return

        try:
            data = np.concatenate(self.recording_data, axis=0)

            # ✅ 저장할 폴더 확인 및 생성
            save_dir = os.path.dirname(self.filepath)
            if save_dir and not os.path.exists(save_dir):
                os.makedirs(save_dir, exist_ok=True)

            # 기존 파일 삭제
            if os.path.exists(self.filepath):
                os.remove(self.filepath)

            # ✅ 파일 저장
            write(self.filepath, 44100, data)
            self.get_logger().info(f"🎤 녹음 저장 완료: {self.filepath}")

        except Exception as e:
            self.get_logger().error(f"💾 저장 실패: {str(e)}")


    def start_rec_callback(self, req, res):
        """📢 ROS 서비스 요청을 통한 녹음 시작"""
        if self.is_recording:
            self.get_logger().warning("⚠️ 녹음이 이미 진행 중입니다.")
            res.success = False
            return res

        self.get_logger().info("📢 ROS 서비스 요청: 녹음 시작")
        self.start_recording()
        res.success = True
        return res

    def stop_rec_callback(self, req, res):
        """📢 ROS 서비스 요청을 통한 녹음 종료"""
        self.get_logger().info("📢 ROS 서비스 요청: 녹음 종료")
        self.stop_recording()
        res.success = True
        return res
    
    def process_stt(self):
        """🎙️ STT 변환 후 UI에 전송"""
        try:
            if not os.path.exists(self.filepath):
                self.get_logger().error(f"❌ STT 변환 실패: 파일이 존재하지 않음: {self.filepath}")
                self.publish_status("Recognition Failed. Please Speak Again.")
                return

            text = self.stt_model.get_text().strip()
            if not text:
                text = "Please Try Recording Again."  # 완전히 공백인 경우

            self.get_logger().info(f"🎧 STT 변환 결과(원본): {text}")

            # KIWI 분석 실행
            result = self.kiwi_model.get_result(text)
            self.get_logger().info(f"🛠️ KIWI 분석 결과: {result}")

            if "error" in result:
                # 인식 실패 시
                self.publish_status("Recognition Failed. Please Speak Again.")
            else:
                # 인식 성공, 소스명 추출
                source_name = result["source"]
                # 사용자에게는 "<소스명> 소스가 선택되었습니다." 라고 안내
                self.publish_status(f"{source_name} Sauce Selected.")
                # 그리고 라디오 버튼 자동 선택을 위해 dispense_source 실행
                self.dispense_source(result)

        except FileNotFoundError as e:
            self.get_logger().error(f"❌ STT 변환 실패: {e}")
            self.publish_status("Recognition Failed. Please Speak Again.")

        except Exception as e:
            self.get_logger().error(f"❌ STT 변환 중 예외 발생: {e}")
            self.publish_status("Recognition Failed. Please Speak Again.")


    def dispense_source(self, output):
        """소스 선택 후 UI 반영"""
        try:
            source_name = output.get('source', None)
            if not source_name:
                # 소스 정보가 없는 경우는 이미 error 처리됨
                self.get_logger().warning("No Sauce Selected.")
                return

            self.get_logger().info(f"✅ 소스 선택: {source_name}")
            # 라디오 버튼 체크를 위해 "raw 소스명"을 퍼블리시
            msg = String()
            msg.data = source_name  # 예: "케첩", "갈릭치즈" 등
            self.pub_status.publish(msg)

        except Exception as e:
            self.get_logger().error(f"❌ 소스 선택 중 오류 발생: {str(e)}")

    def publish_status(self, text):
        """STT 결과를 퍼블리시하여 UI에 전달"""
        msg = String()
        msg.data = text
        self.pub_status.publish(msg)
