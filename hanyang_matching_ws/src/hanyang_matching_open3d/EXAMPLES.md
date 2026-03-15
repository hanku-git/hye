# 예제 및 테스트 가이드

이 문서는 Open3D 매칭 노드의 다양한 사용 예제와 테스트 방법을 설명합니다.

---

## 📁 예제 파일 목록

### 테스트 스크립트

| 파일 | 설명 | 사용 목적 |
|-----|------|---------|
| `test/test_basic_functionality.py` | 기본 기능 단위 테스트 | 개발 검증 |
| `test/test_full_pipeline.py` | 전체 파이프라인 통합 테스트 | 시스템 검증 |

### 예제 스크립트

| 파일 | 설명 | 사용 목적 |
|-----|------|---------|
| `examples/example_standalone_matching.py` | ROS 없이 단독 실행 | 알고리즘 개발 |
| `examples/example_ros2_client.py` | ROS2 서비스 클라이언트 | 로봇 통합 |
| `examples/compare_pcl_vs_open3d.py` | PCL vs Open3D 비교 | 성능 검증 |
| `examples/record_test_data.py` | 테스트 데이터 기록 | 데이터 수집 |
| `examples/replay_test_data.py` | 기록 데이터 재생 | 오프라인 테스트 |

---

## 🧪 테스트 실행

### 1. 기본 기능 테스트

```bash
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d

# 모든 테스트 실행
python3 test/test_basic_functionality.py

# 특정 테스트만 실행
python3 test/test_basic_functionality.py TestUtils.test_pose_conversion
```

**예상 출력**:
```
test_compute_centroid (__main__.TestUtils) ... ok
test_pose_conversion (__main__.TestUtils) ... ok
test_remove_nan_points (__main__.TestUtils) ... ok
test_euclidean_clustering (__main__.TestPCLProcessing) ... ok
test_fpfh_feature (__main__.TestPCLProcessing) ... ok
test_normal_estimation (__main__.TestPCLProcessing) ... ok
test_voxel_downsample (__main__.TestPCLProcessing) ... ok

----------------------------------------------------------------------
Ran 7 tests in 2.345s

OK
```

### 2. 전체 파이프라인 테스트

```bash
# 전체 파이프라인 테스트
python3 test/test_full_pipeline.py

# 성능 벤치마크 포함
python3 test/test_full_pipeline.py --benchmark
```

**예상 출력**:
```
============================================================
Testing complete pipeline with synthetic data
============================================================

1. Creating test data...
2. Initializing template...
   CAD loaded: 5000 points
3. Creating measured scan...
   Measured cloud: 10000 points
...
8. Template matching...
   Matching accuracy: 78.45%
   Matching success: True

============================================================
Complete pipeline test PASSED
============================================================
```

---

## 📝 예제 실행

### 예제 1: 단독 실행 (ROS 없이)

알고리즘만 테스트하고 싶을 때 사용.

```bash
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d

# 기본 실행
python3 examples/example_standalone_matching.py
```

**동작**:
1. CAD 모델 로드 (또는 생성)
2. 측정 데이터 로드 (또는 생성)
3. 템플릿 매칭 실행
4. 결과 시각화

**출력 예시**:
```
============================================================
STANDALONE MATCHING EXAMPLE
============================================================

1. Initializing template matching...
2. Initializing template for target object...
3. Loading CAD model...
   CAD loaded: 10000 points
...
8. Running matching pipeline...
   Matching SUCCESS
   Accuracy: 85.34%

9. Matching results:
============================================================
✅ MATCHING SUCCESS
   Accuracy: 85.34%
   Grasping pose: [0.3245, 0.2123, 0.1534, 12.45, 5.67, 89.12]
   Gripper open: 45 mm
   Feasible: True
============================================================
```

### 예제 2: ROS2 서비스 클라이언트

실제 시스템과 통신하는 방법.

```bash
# 터미널 1: 노드 실행
ros2 launch hanyang_matching_open3d full_pipeline.launch.py

# 터미널 2: 클라이언트 실행
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d
python3 examples/example_ros2_client.py
```

**출력 예시**:
```
============================================================
EXAMPLE 1: Basic Matching Request
============================================================
Calling matching service for bolt (ID: 1)...

============================================================
MATCHING SERVICE RESPONSE
============================================================
✅ POSE FOUND

Grasping Pose [m, deg]:
  Position: [0.3245, 0.2123, 0.1534]
  Orientation: [12.45, 5.67, 89.12]

Matching Quality:
  Accuracy: 85.34%
  Approach distance: 50.0 mm
  Flipped: False

Gripper Parameters:
  Open length: 45 mm
  Close length: 35 mm
  Tip index: 1

Detection Info:
  Detected masks: 3
============================================================
```

### 예제 3: PCL vs Open3D 성능 비교

두 노드를 동시 실행하여 결과 비교.

```bash
# 터미널 1: PCL 노드
ros2 run hanyang_matching_process matching_node \
    --ros-args -r /cad_matching_result:=/cad_matching_result_pcl

# 터미널 2: Open3D 노드
ros2 launch hanyang_matching_open3d full_pipeline.launch.py

# 터미널 3: 비교 스크립트
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d
python3 examples/compare_pcl_vs_open3d.py
```

**실행 후 Ctrl+C로 종료하면 최종 통계 출력**:
```
============================================================
PERFORMANCE COMPARISON STATISTICS
============================================================

PCL (C++):
  Total runs: 10
  Successful: 9 (90.0%)
  Avg accuracy: 82.34%

Open3D (Python):
  Total runs: 10
  Successful: 9 (90.0%)
  Avg accuracy: 80.12%

Comparison:
  Accuracy difference: -2.22% (PCL better)
  Success rate difference: 0.0% (Same)

============================================================
POSE COMPARISON
============================================================
Found 9 matching pairs

Pair 1:
  Position error: 3.45 mm
  Orientation error: 1.23 deg
  PCL accuracy: 85.23%
  Open3D accuracy: 83.12%

...

Summary:
  Avg position error: 4.12 mm
  Max position error: 8.34 mm
  Avg orientation error: 1.67 deg
  Max orientation error: 3.45 deg
============================================================
```

### 예제 4: 데이터 기록

실제 실행 데이터를 저장하여 나중에 재생.

```bash
# 터미널 1: 전체 시스템 실행
ros2 launch hanyang_matching_open3d full_pipeline.launch.py

# 터미널 2: 데이터 기록 시작
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d
python3 examples/record_test_data.py
```

**동작**:
- `/cloud_mask_results` 토픽을 듣고 자동 저장
- 각 스캔마다 디렉토리 생성
- 점군, 메타데이터, 결과를 파일로 저장

**저장 구조**:
```
/tmp/matching_test_data_20251022_143000/
├── scan_0001/
│   ├── scan_cloud.pcd
│   ├── mask_cloud.pcd
│   ├── metadata.json
│   └── matching_result.json
├── scan_0002/
│   └── ...
└── scan_0003/
    └── ...
```

### 예제 5: 데이터 재생

저장된 데이터로 오프라인 테스트.

```bash
cd ~/hanyang_matching_ws/src/hanyang_matching_open3d

# 시각화 포함
python3 examples/replay_test_data.py /tmp/matching_test_data_20251022_143000

# 시각화 없이 (빠른 테스트)
python3 examples/replay_test_data.py /tmp/matching_test_data_20251022_143000 --no-vis
```

**출력 예시**:
```
Found 3 scans

============================================================
SCAN 1/3
============================================================

Loading scan from: /tmp/matching_test_data_20251022_143000/scan_0001
  Metadata loaded
  Scan cloud: 12453 points
  Mask cloud: 3421 points
  Matching result loaded

Target: bolt (ID: 1)
CAD model loaded from: ~/cad_models/bolt.ply

Running matching process...
----------------------------------------
Matching SUCCESS
Accuracy: 84.23%
----------------------------------------

Comparison with recorded result:
  Recorded accuracy: 85.12%
  Replayed accuracy: 84.23%
  Position error: 2.34 mm
  Orientation error: 0.89 deg

============================================================
REPLAY SUMMARY
============================================================
Total scans: 3
Successful: 3 (100.0%)
Average accuracy: 83.45%
Average processing time: 1234.5 ms
============================================================
```

---

## 🎯 사용 시나리오별 가이드

### 시나리오 1: 알고리즘 개발

**목표**: ICP 파라미터 튜닝

```bash
# 1. 테스트 데이터 기록
python3 examples/record_test_data.py
# (실제 시스템 실행 후 몇 개 스캔)

# 2. 오프라인 재생으로 파라미터 테스트
# config/matching_params.yaml 수정 후
python3 examples/replay_test_data.py /tmp/matching_test_data_*

# 3. 결과 비교
# 여러 파라미터 조합 시도
```

### 시나리오 2: 시스템 통합 테스트

**목표**: PCL 시스템과 100% 호환 확인

```bash
# 1. 두 노드 동시 실행
# 터미널 1
ros2 run hanyang_matching_process matching_node \
    --ros-args -r /cad_matching_result:=/cad_matching_result_pcl

# 터미널 2
ros2 launch hanyang_matching_open3d full_pipeline.launch.py

# 2. 비교 모니터 실행
# 터미널 3
python3 examples/compare_pcl_vs_open3d.py

# 3. 스캔 트리거 (로봇 제어 또는 수동)
ros2 service call /zivid_scanning ...

# 4. Ctrl+C로 중단 후 통계 확인
```

### 시나리오 3: 성능 벤치마크

**목표**: 처리 속도 측정

```bash
# 성능 벤치마크 실행
python3 test/test_full_pipeline.py --benchmark
```

**출력**:
```
PERFORMANCE BENCHMARK
============================================================

Point cloud size: 1000
  Normal estimation: 12.3 ms
  Voxel downsampling: 2.1 ms (342 points)
  Clustering: 45.6 ms (3 clusters)

Point cloud size: 10000
  Normal estimation: 89.4 ms
  Voxel downsampling: 8.7 ms (3421 points)
  Clustering: 234.5 ms (5 clusters)

Point cloud size: 100000
  Normal estimation: 1234.6 ms
  Voxel downsampling: 67.8 ms (34210 points)
```

### 시나리오 4: 디버깅

**목표**: 매칭 실패 원인 분석

```bash
# 1. 데이터 기록 (실패 케이스)
python3 examples/record_test_data.py

# 2. 재생하며 디버깅
python3 examples/replay_test_data.py /tmp/matching_test_data_* --no-vis

# 3. 파라미터 조정
vi config/matching_params.yaml
# matching_accuracy_limit: 50.0 → 30.0

# 4. 재시도
python3 examples/replay_test_data.py /tmp/matching_test_data_*
```

---

## 🔧 테스트 케이스별 가이드

### 테스트 케이스 1: 기본 기능 검증

```bash
# 단위 테스트
python3 test/test_basic_functionality.py TestUtils
python3 test/test_basic_functionality.py TestPCLProcessing
python3 test/test_basic_functionality.py TestTemplateMatching
```

### 테스트 케이스 2: 합성 데이터 매칭

```bash
# 합성 데이터로 전체 파이프라인
python3 test/test_full_pipeline.py TestFullPipeline.test_complete_pipeline_synthetic_data
```

### 테스트 케이스 3: 실제 데이터 매칭

```bash
# 실제 CAD와 스캔 데이터 준비
mkdir -p ~/cad_models
cp /path/to/bolt.ply ~/cad_models/

# 단독 실행 테스트
python3 examples/example_standalone_matching.py

# ROS2 통합 테스트
ros2 launch hanyang_matching_open3d full_pipeline.launch.py
python3 examples/example_ros2_client.py
```

### 테스트 케이스 4: 멀티스레드 성능

```bash
# 스레드 수 변경하며 테스트
ros2 launch hanyang_matching_open3d full_pipeline.launch.py num_threads:=2
ros2 launch hanyang_matching_open3d full_pipeline.launch.py num_threads:=4
ros2 launch hanyang_matching_open3d full_pipeline.launch.py num_threads:=8

# 처리 시간 비교
ros2 topic hz /cad_matching_result
```

---

## 📊 성능 측정 방법

### 방법 1: ROS2 토픽 모니터링

```bash
# 토픽 주파수 (처리 속도)
ros2 topic hz /cad_matching_result

# 토픽 대역폭 (데이터 크기)
ros2 topic bw /cloud_mask_results

# 토픽 지연 (응답 시간)
ros2 topic delay /cad_matching_result
```

### 방법 2: Python 프로파일링

```bash
# CPU 프로파일링
python -m cProfile -o profile.stats \
    examples/example_standalone_matching.py

# 결과 분석
python -m pstats profile.stats
> sort time
> stats 20  # Top 20 functions
```

### 방법 3: 메모리 프로파일링

```bash
# 메모리 사용량 측정
pip install memory_profiler

# 프로파일링 실행
python -m memory_profiler examples/example_standalone_matching.py
```

---

## 🎨 시각화 예제

### Open3D 직접 시각화

```python
#!/usr/bin/env python3
import open3d as o3d
import numpy as np

# 점군 로드
pcd = o3d.io.read_point_cloud("test_scan.pcd")

# 노멀 추정
pcd.estimate_normals()

# 시각화
o3d.visualization.draw_geometries(
    [pcd],
    window_name="Test Scan",
    width=1280,
    height=720,
    point_show_normal=True
)
```

### RViz로 결과 확인

```bash
# 터미널 1: 노드 실행
ros2 launch hanyang_matching_open3d full_pipeline.launch.py

# 터미널 2: RViz 실행
rviz2

# RViz 설정:
# 1. Fixed Frame: "base_link"
# 2. Add → PointCloud2 → Topic: /cloud_matching_results
# 3. Add → PointCloud2 → Topic: /cloud_pre_matching_results
# 4. Add → PointCloud2 → Topic: /cloud_segmentation_results
```

---

## 🔍 디버깅 예제

### 디버그 모드 실행

```bash
# 상세 로그 출력
ros2 launch hanyang_matching_open3d full_pipeline.launch.py \
    debug_mode:=true \
    --ros-args --log-level debug
```

### 중간 결과 저장

```python
#!/usr/bin/env python3
"""중간 결과 저장 예제"""
import open3d as o3d
from hanyang_matching_open3d.modules.template_matching import TemplateMatching

tm = TemplateMatching()
# ... (매칭 수행)

# 중간 결과 저장
if target_data.cloud_pre_matching_results:
    o3d.io.write_point_cloud(
        "/tmp/pre_matching.pcd",
        target_data.cloud_pre_matching_results
    )

if target_data.cloud_matching_results:
    o3d.io.write_point_cloud(
        "/tmp/matching_results.pcd",
        target_data.cloud_matching_results
    )

# 변환 행렬 저장
import json
with open("/tmp/transformation.json", 'w') as f:
    json.dump({
        'T_pre_matching': target_data.T_pre_matching.tolist(),
        'T_process_matching': target_data.T_process_matching.tolist(),
        'matching_accuracy': target_data.matching_accuracy
    }, f, indent=2)
```

### 단계별 디버깅

```python
#!/usr/bin/env python3
"""단계별 실행 및 시각화"""
import open3d as o3d
from hanyang_matching_open3d.modules import *

# 1. 데이터 로드
pcd = o3d.io.read_point_cloud("scan.pcd")
o3d.visualization.draw_geometries([pcd], window_name="1. Raw scan")

# 2. 필터링
pcl_proc = PCLProcessing()
pcd_filtered = pcl_proc.filter_cloud_using_view_frustum(pcd, view_frustum)
o3d.visualization.draw_geometries([pcd_filtered], window_name="2. Filtered")

# 3. 다운샘플링
pcd_down = pcl_proc.voxel_down_sample(pcd_filtered, 0.005)
o3d.visualization.draw_geometries([pcd_down], window_name="3. Downsampled")

# 4. 노멀 추정
pcd_normal = pcl_proc.estimate_normals(pcd_down)
o3d.visualization.draw_geometries([pcd_normal], window_name="4. With normals",
                                  point_show_normal=True)

# 5. 클러스터링
clusters, largest = pcl_proc.euclidean_cluster_extraction(pcd_down)
o3d.visualization.draw_geometries([largest], window_name="5. Largest cluster")

# ... 계속
```

---

## 📈 결과 분석 예제

### JSON 결과 분석

```python
#!/usr/bin/env python3
"""결과 JSON 분석"""
import json
import numpy as np

# 저장된 결과 로드
with open("/tmp/comparison_results.json", 'r') as f:
    data = json.load(f)

# 통계 계산
pcl_accuracies = [r['matching_accuracy'] 
                  for r in data['pcl_results'] 
                  if r['is_pose']]

open3d_accuracies = [r['matching_accuracy'] 
                     for r in data['open3d_results'] 
                     if r['is_pose']]

print(f"PCL: mean={np.mean(pcl_accuracies):.2f}%, "
      f"std={np.std(pcl_accuracies):.2f}%")
print(f"Open3D: mean={np.mean(open3d_accuracies):.2f}%, "
      f"std={np.std(open3d_accuracies):.2f}%")
```

### 시각화 비교

```python
#!/usr/bin/env python3
"""결과 시각화 비교"""
import matplotlib.pyplot as plt
import json

with open("/tmp/comparison_results.json", 'r') as f:
    data = json.load(f)

# 정확도 비교 그래프
pcl_acc = [r['matching_accuracy'] for r in data['pcl_results']]
open3d_acc = [r['matching_accuracy'] for r in data['open3d_results']]

plt.figure(figsize=(12, 6))

plt.subplot(1, 2, 1)
plt.plot(pcl_acc, 'b-', label='PCL')
plt.plot(open3d_acc, 'r-', label='Open3D')
plt.xlabel('Trial')
plt.ylabel('Matching Accuracy (%)')
plt.legend()
plt.title('Matching Accuracy Comparison')
plt.grid(True)

plt.subplot(1, 2, 2)
plt.hist([pcl_acc, open3d_acc], label=['PCL', 'Open3D'], bins=20)
plt.xlabel('Accuracy (%)')
plt.ylabel('Frequency')
plt.legend()
plt.title('Accuracy Distribution')

plt.tight_layout()
plt.savefig('/tmp/comparison_plot.png')
print("Plot saved to /tmp/comparison_plot.png")
plt.show()
```

---

## 🚨 일반적인 문제 해결

### 문제: 테스트가 실패함

```bash
# 의존성 확인
pip list | grep open3d
pip list | grep numpy

# 재설치
pip install --upgrade -r requirements.txt

# 패키지 재빌드
cd ~/hanyang_matching_ws
colcon build --packages-select hanyang_matching_open3d --cmake-clean-cache
```

### 문제: "No module named 'hanyang_matching_open3d'"

```bash
# 환경 재로드
cd ~/hanyang_matching_ws
source install/setup.bash

# Python 경로 확인
python3 -c "import sys; print('\n'.join(sys.path))"

# PYTHONPATH 설정
export PYTHONPATH=$PYTHONPATH:~/hanyang_matching_ws/install/hanyang_matching_open3d/lib/python3.10/site-packages
```

### 문제: 시각화 창이 안 열림

```bash
# X11 포워딩 확인 (SSH 사용 시)
echo $DISPLAY

# X11 설정
export DISPLAY=:0

# 또는 시각화 없이 실행
python3 examples/replay_test_data.py /tmp/data --no-vis
```

---

## 📚 추가 리소스

### 문서
- `README.md` - 전체 매뉴얼
- `PIPELINE.md` - 파이프라인 상세 설명
- `INTEGRATION_GUIDE.md` - 시스템 통합 가이드
- `QUICKSTART.md` - 빠른 시작

### 코드 예제
- `examples/` - 실행 가능한 예제들
- `test/` - 단위 테스트

### 설정 파일
- `config/matching_params.yaml` - 파라미터 설정
- `config/example_target.json` - 타겟 객체 설정

---

## 💡 팁

1. **빠른 반복 개발**:
   ```bash
   # 데이터 한 번 기록
   python3 examples/record_test_data.py
   
   # 코드 수정 후 즉시 재생 테스트 (빌드 불필요)
   python3 examples/replay_test_data.py /tmp/data
   ```

2. **파라미터 최적화**:
   ```bash
   # 여러 설정 시도
   for limit in 30 40 50 60 70; do
       ros2 param set /node matching_accuracy_limit $limit
       # 테스트 실행
   done
   ```

3. **결과 시각화**:
   - Open3D 창: 마우스로 회전/확대
   - RViz: TF, 마커 등 추가 정보

---

**작성일**: 2025-10-22  
**버전**: 1.0.0

