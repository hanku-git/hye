# Installation Guide

## 시스템 요구사항

- **OS**: Ubuntu 20.04/22.04 (ROS2 Foxy/Humble)
- **ROS2**: Foxy or Humble
- **Python**: 3.8+
- **RAM**: 최소 8GB (권장 16GB)
- **Storage**: 최소 2GB 여유 공간

## 단계별 설치

### 1. 의존성 패키지 설치

#### Python 패키지

```bash
# Open3D (핵심)
pip install open3d

# 기타 필수 패키지
pip install numpy scipy opencv-python

# 또는 requirements.txt 사용
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d
pip install -r requirements.txt
```

#### ROS2 패키지

```bash
# 기존 hanyang_matching_msgs 패키지 필요
sudo apt update
sudo apt install ros-${ROS_DISTRO}-cv-bridge
sudo apt install ros-${ROS_DISTRO}-sensor-msgs-py
```

### 2. 패키지 빌드

```bash
cd ~/hanyang_matching_ws

# 이 패키지만 빌드
colcon build --packages-select hanyang_matching_open3d

# 또는 전체 워크스페이스 빌드
colcon build

# 환경 설정
source install/setup.bash
```

### 3. 설치 확인

```bash
# 패키지 확인
ros2 pkg list | grep hanyang_matching_open3d

# 노드 확인
ros2 pkg executables hanyang_matching_open3d

# 서비스 확인 (노드 실행 후)
ros2 service list | grep matching
```

예상 출력:
```
hanyang_matching_open3d
matching_node
/do_template_matching_bin_picking
```

## 설치 검증

### 1. 기본 테스트

```bash
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d
python3 test/test_basic_functionality.py
```

성공 시:
```
......
----------------------------------------------------------------------
Ran 6 tests in 0.XXXs

OK
```

### 2. 노드 실행 테스트

```bash
# 터미널 1: 노드 실행
ros2 run hanyang_matching_open3d matching_node

# 터미널 2: 서비스 확인
ros2 service list | grep matching
ros2 service type /do_template_matching_bin_picking
```

### 3. 파라미터 확인

```bash
ros2 param list
ros2 param get /hanyang_matching_open3d_node matching_accuracy_limit
```

## 문제 해결

### 문제 1: "ModuleNotFoundError: No module named 'open3d'"

**해결**:
```bash
pip install --upgrade open3d
# 또는 특정 버전
pip install open3d==0.17.0
```

### 문제 2: "Package 'hanyang_matching_msgs' not found"

**해결**:
```bash
# hanyang_matching_msgs 패키지 먼저 빌드
cd ~/hanyang_matching_ws
colcon build --packages-select hanyang_matching_msgs
source install/setup.bash

# 그 후 이 패키지 빌드
colcon build --packages-select hanyang_matching_open3d
```

### 문제 3: 빌드 에러 - "setup.py not found"

**해결**:
```bash
# 파일 구조 확인
ls ~/hanyang_matching_ws/src/hanyang_matching_open3d/setup.py

# setup.py가 있다면 권한 확인
chmod +x ~/hanyang_matching_ws/src/hanyang_matching_open3d/setup.py

# 빌드 캐시 정리 후 재빌드
cd ~/hanyang_matching_ws
rm -rf build/ install/ log/
colcon build --packages-select hanyang_matching_open3d
```

### 문제 4: Import 에러

**해결**:
```bash
# Python 경로 확인
python3 -c "import sys; print('\n'.join(sys.path))"

# ROS2 환경 재로드
cd ~/hanyang_matching_ws
source install/setup.bash

# 패키지 경로 확인
ros2 pkg prefix hanyang_matching_open3d
```

### 문제 5: "cv_bridge" 관련 에러

**해결**:
```bash
# cv_bridge 설치
sudo apt install ros-${ROS_DISTRO}-cv-bridge

# Python cv_bridge 확인
python3 -c "from cv_bridge import CvBridge; print('OK')"
```

## 개발 환경 설정

### VS Code 설정

`.vscode/settings.json`:
```json
{
    "python.autoComplete.extraPaths": [
        "/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages",
        "${workspaceFolder}/install/hanyang_matching_open3d/lib/python3.8/site-packages"
    ],
    "python.analysis.extraPaths": [
        "/opt/ros/${ROS_DISTRO}/lib/python3/dist-packages"
    ]
}
```

### 환경변수 설정

`~/.bashrc`에 추가:
```bash
# ROS2 workspace
source ~/hanyang_matching_ws/install/setup.bash

# Python path (선택사항)
export PYTHONPATH=$PYTHONPATH:~/hanyang_matching_ws/install/hanyang_matching_open3d/lib/python3.8/site-packages
```

## 업데이트

### 코드 업데이트

```bash
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d
git pull  # (Git 사용 시)

# 재빌드
cd ~/hanyang_matching_ws
colcon build --packages-select hanyang_matching_open3d
source install/setup.bash
```

### 의존성 업데이트

```bash
# Python 패키지 업데이트
pip install --upgrade -r requirements.txt

# 또는 개별 업데이트
pip install --upgrade open3d numpy scipy opencv-python
```

## 제거

### 패키지 제거

```bash
cd ~/hanyang_matching_ws

# 빌드 파일 제거
rm -rf build/hanyang_matching_open3d
rm -rf install/hanyang_matching_open3d
rm -rf log/hanyang_matching_open3d

# 소스 제거 (선택사항)
rm -rf src/hanyang_matching_open3d
```

### Python 패키지 제거

```bash
pip uninstall open3d numpy scipy opencv-python
```

## 다음 단계

설치가 완료되었다면:

1. [QUICKSTART.md](QUICKSTART.md) - 빠른 시작 가이드
2. [README.md](README.md) - 전체 매뉴얼
3. `config/matching_params.yaml` - 파라미터 조정

## 지원

- 이슈 트래커: 버그 리포트 및 기능 요청
- 문서: README.md, QUICKSTART.md 참조

