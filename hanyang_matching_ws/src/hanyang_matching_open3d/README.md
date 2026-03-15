# Hanyang Matching Open3D

Open3D 기반 템플릿 매칭 노드 for 빈피킹 시스템

## 개요

이 패키지는 기존 PCL 기반 `hanyang_matching_process`의 기능을 Open3D Python 라이브러리로 재구현한 것입니다.

## 주요 기능

### 1. 전처리 (Preprocessing)
- **PassThrough 필터링**: 뷰 프러스텀(AABB) 기반 점군 크롭
- **다운샘플링**: Voxel Grid 다운샘플링
- **아웃라이어 제거**: Statistical Outlier Removal
- **NaN 점 제거**

### 2. 분할 (Segmentation)
- **Euclidean Clustering**: DBSCAN 기반 군집화
- **Major Segment 추출**: 가장 큰 클러스터 추출

### 3. 특징 추출 (Feature Extraction)
- **노멀 추정**: KDTree 기반 법선 벡터 계산
- **FPFH 특징**: Fast Point Feature Histograms

### 4. 정합 (Registration)
- **Quick Align**: RANSAC 기반 특징 매칭으로 초기 정렬
- **ICP**: Point-to-Plane ICP 정밀 정합
- **Multi-scale ICP**: GICP 근사 (다중 해상도 ICP)

### 5. 충돌 검사
- **Voxel Grid**: OctoMap 대체용 복셀 그리드 기반 충돌 검사

## PCL vs Open3D 매핑

| PCL 기능 | Open3D 대체 |
|---------|-------------|
| PassThrough | `AxisAlignedBoundingBox.crop()` |
| VoxelGrid | `voxel_down_sample()` |
| StatisticalOutlierRemoval | `remove_statistical_outlier()` |
| NormalEstimation | `estimate_normals()` |
| FPFHEstimation | `compute_fpfh_feature()` |
| SAC-IA (Quick Align) | `registration_ransac_based_on_feature_matching()` |
| ICP with Normals | `registration_icp()` + `TransformationEstimationPointToPlane()` |
| GICP | Multi-scale ICP (근사) |
| EuclideanClusterExtraction | `cluster_dbscan()` |
| OctoMap | `VoxelGrid` (근사) |

## 설치

### 1. Python 패키지 설치

```bash
pip install -r requirements.txt
```

또는:

```bash
pip install open3d numpy scipy opencv-python
```

### 2. ROS2 패키지 빌드

```bash
cd ~/hanyang_matching_ws
colcon build --packages-select hanyang_matching_open3d
source install/setup.bash
```

## 사용법

### 1. 노드 실행

```bash
ros2 launch hanyang_matching_open3d matching_node.launch.py
```

디버그 모드:

```bash
ros2 launch hanyang_matching_open3d matching_node.launch.py debug_mode:=true
```

### 2. 서비스 호출

```bash
ros2 service call /do_template_matching_bin_picking hanyang_matching_msgs/srv/DoTemplateMatching "{
  target_id: 1,
  target_name: 'bolt',
  robot_dh_parameters: [...],
  robot_tcp_default: [0, 0, 0, 0, 0, 0],
  robot_tcp: [0, 0, 0, 0, 0, 0],
  scan_position_q: [0, 0, 0, 0, 0, 0],
  do_scan_sampling: false,
  sampling_num: 1,
  debug_mode: false
}"
```

### 3. 토픽 구독/발행

#### 구독 (Subscriptions)
- `/raw_scanned_points` (sensor_msgs/PointCloud2): 스캔 점군
- `/mask_cloud` (hanyang_matching_msgs/MaskCloud): 마스크 점군 (SAM/Mask-RCNN)

#### 발행 (Publications)
- `/open3d/cad_matching_result_cloud` (sensor_msgs/PointCloud2): 매칭 결과 점군
- `/open3d/pre_matching_result_cloud` (sensor_msgs/PointCloud2): 사전 매칭 결과
- `/open3d/segmentation_result_cloud` (sensor_msgs/PointCloud2): 분할 결과
- `/open3d/cad_matching_result` (hanyang_matching_msgs/MatchingResultMsg): 매칭 결과 메시지

## 파라미터 설정

`config/matching_params.yaml` 파일에서 파라미터를 조정할 수 있습니다:

```yaml
matching_node:
  ros__parameters:
    # Quick align
    do_quick_align: true
    quick_align_max_correspondence_distance: 0.05  # 50mm
    
    # ICP
    icp_max_iterations: 50
    icp_correspondence_distance: 0.05  # 50mm
    
    # Filtering
    do_euclidean_filtering: true
    euclidean_cluster_tolerance: 0.01  # 10mm
    
    # Matching accuracy threshold
    matching_accuracy_limit: 50.0  # %
```

## 디렉토리 구조

```
hanyang_matching_open3d/
├── hanyang_matching_open3d/
│   ├── __init__.py
│   ├── modules/
│   │   ├── __init__.py
│   │   ├── data_classes.py       # 데이터 클래스
│   │   ├── utils.py               # 유틸리티 함수
│   │   ├── pcl_processing.py     # 점군 처리
│   │   └── template_matching.py  # 템플릿 매칭
│   └── nodes/
│       ├── __init__.py
│       └── matching_node.py      # ROS2 노드
├── launch/
│   └── matching_node.launch.py   # 런치 파일
├── config/
│   └── matching_params.yaml      # 파라미터 파일
├── CMakeLists.txt
├── package.xml
├── setup.py
├── setup.cfg
├── requirements.txt
└── README.md
```

## 성능 비교

| 항목 | PCL (C++) | Open3D (Python) |
|-----|-----------|-----------------|
| 컴파일 속도 | 느림 | 빠름 (인터프리터) |
| 실행 속도 | 빠름 | 약간 느림 |
| 개발 속도 | 느림 | 빠름 |
| 메모리 사용 | 적음 | 많음 |
| 유지보수성 | 보통 | 좋음 |

## 제한사항

1. **GICP**: Open3D에 순정 GICP가 없어 Multi-scale ICP로 근사
2. **Region Growing**: 직접 구현 필요 (현재 DBSCAN으로 대체)
3. **OctoMap**: VoxelGrid로 근사 (완전 대체 아님)
4. **성능**: Python GIL로 인해 순수 멀티스레딩은 제한적 (multiprocessing 필요)

## 향후 개선 사항

- [ ] Multi-processing 기반 병렬 마스크 처리
- [ ] OctoMap Python 바인딩 또는 C++ 브릿지
- [ ] Region Growing 정밀 구현
- [ ] Cython/Numba를 통한 성능 최적화
- [ ] 그리퍼 충돌 검사 고도화

## 참고

- Open3D 공식 문서: http://www.open3d.org/docs/
- PCL 공식 문서: https://pointclouds.org/documentation/

## 라이센스

MIT License

## 문의

버그 리포트 및 기능 요청은 이슈 트래커에 등록해주세요.

