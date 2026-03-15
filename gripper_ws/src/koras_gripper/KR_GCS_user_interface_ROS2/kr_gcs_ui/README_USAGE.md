# 그리퍼 제어 사용법

이 패키지는 UI 모드와 Headless 모드 두 가지 방식으로 그리퍼를 제어할 수 있습니다.

## 1. UI 모드 (기본)

그래픽 사용자 인터페이스를 통해 그리퍼를 제어합니다.

### 실행 방법

```bash
# 방법 1: 직접 실행
sudo chmod a+rw /dev/ttyUSB0 && ros2 run kr_gcs_ui kr_gcs_ui

# 방법 2: launch 파일 사용
ros2 launch kr_gcs_ui gripper_ui.launch

# 방법 3: --no-ui 옵션으로 UI 없이 실행 (기존 방식과 동일)
sudo chmod a+rw /dev/ttyUSB0 && ros2 run kr_gcs_ui kr_gcs_ui --no-ui
```

## 2. Headless 모드 (새로 추가)

UI 없이 그리퍼 제어만 수행합니다. 서버 환경이나 자동화된 시스템에서 사용하기 적합합니다.

### 실행 방법

```bash
# 방법 1: 기본 설정으로 실행
sudo chmod a+rw /dev/ttyUSB0 && ros2 run kr_gcs_ui kr_gcs_ui_headless

# 방법 2: launch 파일 사용
ros2 launch kr_gcs_ui gripper_headless.launch

# 방법 3: 커스텀 설정으로 실행
sudo chmod a+rw /dev/ttyUSB0 && ros2 run kr_gcs_ui kr_gcs_ui_headless --port /dev/ttyUSB1 --slave 2 --baud 115200
```

### Headless 모드 옵션

- `--port <포트>`: 시리얼 포트 지정 (기본값: /dev/ttyUSB0)
- `--slave <주소>`: 슬레이브 주소 지정 (기본값: 1)
- `--baud <속도>`: 보드레이트 지정 (기본값: 115200)
- `--help, -h`: 도움말 표시

### 예시

```bash
# 다른 포트와 슬레이브 주소로 실행
ros2 run kr_gcs_ui kr_gcs_ui_headless --port /dev/ttyUSB1 --slave 2

# 도움말 보기
ros2 run kr_gcs_ui kr_gcs_ui_headless --help
```

## 3. 권한 설정

시리얼 포트에 접근하기 위해 권한을 설정해야 합니다:

```bash
# USB 포트 권한 설정
sudo chmod a+rw /dev/ttyUSB0

# 또는 udev 규칙을 추가하여 영구적으로 설정
echo 'SUBSYSTEM=="tty", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE="0666"' | sudo tee /etc/udev/rules.d/99-gripper.rules
sudo udevadm control --reload-rules
```

## 4. 종료 방법

- **UI 모드**: 창을 닫거나 Ctrl+C
- **Headless 모드**: Ctrl+C

## 5. 빌드 방법

```bash
# 워크스페이스에서 빌드
cd /home/bp/gripper_ws
colcon build --packages-select kr_gcs_ui

# 소스 설정
source install/setup.bash
```

## 6. 문제 해결

### 권한 오류
```bash
sudo chmod a+rw /dev/ttyUSB0
```

### 포트를 찾을 수 없는 경우
```bash
# 사용 가능한 USB 포트 확인
ls /dev/ttyUSB*

# 또는
dmesg | grep ttyUSB
```

### 그리퍼가 응답하지 않는 경우
- 포트 번호 확인
- 슬레이브 주소 확인
- 보드레이트 확인
- 케이블 연결 확인
