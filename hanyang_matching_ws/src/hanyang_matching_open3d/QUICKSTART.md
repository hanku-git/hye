# Quick Start Guide

Open3D 기반 매칭 노드 빠른 시작 가이드

## 1. 설치

### Python 패키지 설치

```bash
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d
pip install -r requirements.txt
```

### ROS2 패키지 빌드

```bash
cd ~/hanyang_matching_ws
colcon build --packages-select hanyang_matching_open3d
source install/setup.bash
```

## 2. 기본 실행

### 노드 실행

```bash
# 터미널 1: 노드 실행
ros2 launch hanyang_matching_open3d matching_node.launch.py

# 터미널 2: 서비스 호출 (예제)
ros2 service call /do_template_matching_bin_picking hanyang_matching_msgs/srv/DoTemplateMatching "{
  target_id: 1,
  target_name: 'bolt',
  debug_mode: false
}"
```

## 3. 기존 PCL 노드와 비교

### PCL 노드 실행 (기존)
```bash
ros2 launch hanyang_matching_process comm2robot_node.xml
```

### Open3D 노드 실행 (새로운)
```bash
ros2 launch hanyang_matching_open3d matching_node.launch.py
```

### 서비스 인터페이스는 동일
- 동일한 서비스 이름: `/do_template_matching_bin_picking`
- 동일한 메시지 타입: `hanyang_matching_msgs/srv/DoTemplateMatching`
- 입출력 필드 호환

## 4. 파라미터 조정

### 파라미터 파일 수정

`config/matching_params.yaml` 파일 편집:

```yaml
matching_node:
  ros__parameters:
    # Quick align 사용 여부
    do_quick_align: true
    
    # ICP 반복 횟수
    icp_max_iterations: 50
    
    # 매칭 정확도 임계값 (%)
    matching_accuracy_limit: 50.0
```

### 런타임 파라미터 변경

```bash
ros2 param set /hanyang_matching_open3d_node matching_accuracy_limit 60.0
```

## 5. 시각화

### RViz로 결과 확인

```bash
# 터미널 1: 노드 실행
ros2 launch hanyang_matching_open3d matching_node.launch.py

# 터미널 2: RViz 실행
rviz2
```

RViz에서 다음 토픽 추가:
- `/cad_matching_result_cloud` - 매칭 결과 (녹색)
- `/pre_matching_result_cloud` - 사전 매칭 결과 (빨간색)
- `/segmentation_result_cloud` - 분할 결과

## 6. 테스트

### 단위 테스트 실행

```bash
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d
python3 test/test_basic_functionality.py
```

### 기대 결과
```
......
----------------------------------------------------------------------
Ran 6 tests in 0.XXXs

OK
```

## 7. CAD 모델 준비

### CAD 파일 위치

기본 검색 경로:
1. `<package_share>/cad_models/<target_name>.ply`
2. `~/cad_models/<target_name>.ply`

### CAD 파일 변환

PCL의 PCD/PLY 파일을 그대로 사용 가능:

```bash
# PLY 파일이 이미 있는 경우
cp /path/to/existing/model.ply ~/cad_models/bolt.ply

# PCD → PLY 변환 (필요시)
pcl_converter input.pcd output.ply -format 1
```

## 8. 디버깅

### 디버그 모드 활성화

```bash
ros2 launch hanyang_matching_open3d matching_node.launch.py debug_mode:=true
```

### 로그 레벨 설정

```bash
ros2 run hanyang_matching_open3d matching_node --ros-args --log-level debug
```

### 점군 저장

매칭 결과를 파일로 저장:

```python
# Python 코드에서
import open3d as o3d
pcd = target_data.cloud_matching_results
o3d.io.write_point_cloud("/tmp/matching_result.pcd", pcd)
```

## 9. 성능 최적화

### 다운샘플링 조정

파라미터 파일에서:
```yaml
voxel_size_downsample: 0.005  # 5mm (기본값)
# 더 빠르게: 0.01 (10mm)
# 더 정밀하게: 0.002 (2mm)
```

### ICP 반복 횟수 조정

```yaml
icp_max_iterations: 50  # 기본값
# 더 빠르게: 30
# 더 정밀하게: 100
```

### Quick Align 비활성화 (빠른 실행)

```yaml
do_quick_align: false  # Quick align 건너뛰기
```

## 10. 문제 해결

### 문제: "No scan cloud available"

**원인**: 점군 토픽을 받지 못함

**해결**:
```bash
# 점군 토픽 확인
ros2 topic list | grep point

# 점군 데이터 확인
ros2 topic echo /raw_scanned_points --no-arr
```

### 문제: "Matching failed" (정확도 낮음)

**원인**: 다운샘플링이 너무 많거나 파라미터 부적합

**해결**:
1. 다운샘플링 크기 줄이기: `voxel_size_downsample: 0.002`
2. ICP 반복 횟수 늘리기: `icp_max_iterations: 100`
3. 매칭 임계값 낮추기: `matching_accuracy_limit: 40.0`

### 문제: 빌드 에러

**원인**: 의존성 패키지 누락

**해결**:
```bash
# Python 패키지 재설치
pip install -r requirements.txt

# ROS2 패키지 확인
ros2 pkg list | grep hanyang_matching_msgs

# 빌드 캐시 정리
cd ~/hanyang_matching_ws
rm -rf build/ install/ log/
colcon build --packages-select hanyang_matching_open3d
```

## 11. 기존 시스템과 통합

### PCL 노드 교체

기존 런치 파일에서:

```xml
<!-- 기존 PCL 노드 -->
<node pkg="hanyang_matching_process" exec="matching_node" name="matching_node"/>

<!-- Open3D 노드로 교체 -->
<node pkg="hanyang_matching_open3d" exec="matching_node" name="hanyang_matching_open3d_node"/>
```

### 혼용 (병렬 실행)

두 노드를 동시에 실행하여 결과 비교:

```bash
# 터미널 1: PCL 노드
ros2 run hanyang_matching_process matching_node

# 터미널 2: Open3D 노드
ros2 run hanyang_matching_open3d matching_node

# 각각 다른 서비스 이름 사용:
# - /do_template_matching_bin_picking (PCL)
# - /do_template_matching_bin_picking_open3d (Open3D)
```

## 12. 추가 리소스

- 전체 매뉴얼: [README.md](README.md)
- API 문서: Python docstrings 참조
- 예제 설정: `config/example_target.json`
- 테스트 코드: `test/test_basic_functionality.py`

## 지원

문제가 발생하면:
1. README.md의 "제한사항" 섹션 확인
2. 로그 메시지 확인 (`--log-level debug`)
3. 이슈 트래커에 버그 리포트 등록

