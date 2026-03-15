# Open3D Matching Pipeline - 전체 동작 흐름

## 전체 아키텍처

```
┌─────────────────┐
│  로봇 제어 시스템  │
└────────┬────────┘
         │ (1) 스캔 요청
         ↓
┌─────────────────┐
│ Zivid Scan Node │ ← /zivid/points/xyzrgba (Zivid 카메라)
│                 │ ← /zivid/color/image_color
└────────┬────────┘
         │ (2) SAM 요청
         ↓
┌─────────────────┐
│    SAM Node     │
│  (마스크 검출)   │
└────────┬────────┘
         │ (3) 통합 데이터 발행
         ↓ /cloud_mask_results (MaskCloud)
┌─────────────────┐
│  Matching Node  │
│   (Open3D)      │
└────────┬────────┘
         │ (4) 포즈 결과 발행
         ↓ /cad_matching_result (MatchingResultMsg)
┌─────────────────┐
│  로봇 제어 시스템  │
│  (파지 동작)     │
└─────────────────┘
```

---

## 단계별 상세 흐름

### 📡 단계 1: 3D 스캔 데이터 획득

#### 1-1. 스캔 트리거
**로봇 제어 시스템** → 서비스 호출 → **Zivid Scan Node**

서비스:
```
/zivid_scanning (hanyang_matching_msgs/srv/ZividDoScan)
```

요청 파라미터:
- `target_id`: 타겟 객체 ID
- `target_name`: 타겟 객체 이름
- `robot_dh_parameters`: DH 파라미터 [24개]
- `robot_tcp_default`: 기본 TCP [6개]
- `robot_tcp`: 현재 TCP [6개]
- `scan_position_q`: 스캔 시 조인트 각도 [6개]
- `scan_position_x`: 스캔 시 카테시안 포즈 [6개]

#### 1-2. Zivid 카메라 데이터 수신
**Zivid 카메라** → 토픽 발행 → **Scan Node**

토픽:
- `/zivid/points/xyzrgba` (sensor_msgs/PointCloud2): 3D 점군
- `/zivid/color/image_color` (sensor_msgs/Image): RGB 이미지

콜백 처리:
```cpp
void on_points(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg)
{
    pcl::fromROSMsg(*msg, *cloud_scan);
    
    // Zivid2+: mm → m 변환
    m_math.cloudScaling(*cloud_scan, 0);
}
```

---

### 🎭 단계 2: 마스크 검출 (SAM)

#### 2-1. SAM 노드 처리
**Scan Node** → RGB 이미지 전송 → **SAM Node**

SAM 처리:
1. 이미지 분석
2. 객체 분할 (Segment Anything Model)
3. 각 마스크에 대해:
   - 바운딩 박스 계산
   - 신뢰도 점수 산출
   - 주변 객체 밀집도(nearby_ratio) 계산

#### 2-2. SAM 결과 수신
**SAM Node** → 토픽 발행 → **Scan Node**

토픽:
```
/sam_zivid/result (hanyang_matching_msgs/msg/Resultsam)
```

SAM 결과 구조:
```python
class Resultsam:
    class_cnt: int                    # 검출된 객체 개수
    class_ids: List[int]              # 클래스 ID
    class_names: List[str]            # 클래스 이름
    scores: List[float]               # 신뢰도 점수
    masks: List[Image]                # 마스크 이미지
    boxes: List[RegionOfInterest]     # 바운딩 박스
    nearby_ratio_set: List[float]     # 주변 밀집도
```

#### 2-3. 마스크-점군 통합
**Scan Node**에서 처리:

1. 각 마스크 영역에서 점군 추출
2. 신뢰도 필터링 (threshold: 0.94)
3. 주변 밀집도 필터링
4. 중심점 계산
5. 통합 메시지 생성

---

### 📦 단계 3: 통합 데이터 전송

**Scan Node** → 토픽 발행 → **Matching Node**

토픽:
```
/cloud_mask_results (hanyang_matching_msgs/msg/MaskCloud)
```

MaskCloud 메시지 구조:
```python
class MaskCloud:
    # 기본 정보
    class_ids: int                      # 타겟 ID
    class_name: str                     # 타겟 이름
    detection_mode: str                 # "mask_sam_detection"
    
    # 점군 데이터
    scan_cloud: PointCloud2             # 전체 스캔 점군
    mask_cloud: PointCloud2             # 마스크 점군 (선택된 것)
    
    # SAM 결과
    sam_result: Resultsam               # 전체 SAM 결과
    
    # 로봇 파라미터
    robot_dh_parameters: List[float]    # DH [24개]
    robot_tcp_default: List[float]      # TCP 기본 [6개]
    robot_tcp: List[float]              # TCP 현재 [6개]
    
    # 로봇 상태
    scan_position_q: List[float]        # 조인트 각도 [6개]
    scan_position_x: List[float]        # 카테시안 포즈 [6개]
    
    # 제어 플래그
    do_matching_process_topic: bool     # 토픽 모드 매칭 여부
    do_single_matching: bool            # 단일 마스크 모드
```

---

### 🎯 단계 4: 템플릿 매칭 처리 (Open3D)

#### 4-1. 데이터 수신 및 초기화

```python
def on_receive_cloud(self, msg: MaskCloud):
    # 1. 기본 정보 파싱
    self.target_id = msg.class_ids
    self.target_name = msg.class_name
    
    # 2. 점군 변환 (ROS → Open3D)
    self.cloud_scan = ros_to_open3d(msg.scan_cloud)
    self.cloud_input = ros_to_open3d(msg.mask_cloud)
    
    # 3. 로봇 파라미터 저장
    self.robot_dh_vec = msg.robot_dh_parameters
    self.robot_tcp = msg.robot_tcp
    self.scan_position_q = msg.scan_position_q
    
    # 4. 마스크 데이터 파싱
    mask_data = parse_sam_results(msg.sam_result)
    
    # 5. 템플릿 초기화
    if target not initialized:
        initialize_template(target_id, dh, tcp)
        load_cad_model(target_name)
```

#### 4-2. 마스크 처리

```python
def initial_mask_processing_ver2(mask_data, cloud_scan):
    for each mask in mask_data:
        if score < threshold:
            continue
        
        # 마스크 영역에서 점군 추출
        masked_cloud = extract_cloud_from_mask(mask_image)
        
        # 중심점 계산
        centroid = compute_centroid(masked_cloud)
        
        # 저장
        mask_data.cloud_mask_list.append(masked_cloud)
        mask_data.xyz_centroid_list.append(centroid)
        mask_data.mask_confident_idx_set.append(mask_idx)
```

#### 4-3. 멀티스레드 매칭

```python
def matching_process_multi_threaded(target_data, num_threads=4, method=1):
    # 각 마스크 후보에 대해 병렬 처리
    
    with ThreadPoolExecutor(max_workers=num_threads):
        for each confident mask:
            thread_data = copy_target_data()
            thread_data.cloud_measured = mask_cloud[mask_idx]
            
            if method == 1:
                # ICP 매칭
                success = icp_matching_single_process_ver3(
                    is_pose_assigned_event,
                    thread_data
                )
            else:
                # GICP 매칭
                success = gicp_matching_single_process_ver1(
                    is_pose_assigned_event,
                    thread_data
                )
            
            if success and thread_data.is_feasible_pose:
                # 첫 번째 성공한 결과 사용
                is_pose_assigned_event.set()
                copy_results_to_main(thread_data)
                break
```

#### 4-4. 단일 마스크 매칭 프로세스

```python
def icp_matching_single_process_ver3(thread_data):
    # 1. 전처리
    do_measured_cloud_processing(thread_data)
    # - NaN 제거
    # - View frustum 필터링
    # - Statistical outlier removal
    # - Voxel downsampling
    
    # 2. 좌표 변환
    do_alignment_scan2robot_base_frame(thread_data)
    # - Forward kinematics (DH + JS → T_B2E)
    # - Sensor calibration (T_E2S)
    # - 변환: T_B2S = T_B2E @ T_E2S
    
    # 3. 분할 (Pre-matching)
    do_pre_matching_process(thread_data)
    # - Euclidean clustering (DBSCAN)
    # - Major segment 추출
    
    # 4. 특징 추출
    cal_measured_cloud_feature_for_quick_align(thread_data)
    # - Downsample
    # - Normal estimation
    # - FPFH features
    
    # 5. 템플릿 매칭
    do_template_matching_process(thread_data)
    # - Quick align (RANSAC + features)
    # - ICP/Multi-scale ICP
    # - 정확도 평가
    
    # 6. 파지 자세 계산
    cal_grasping_pose_multi_threaded(thread_data)
    # - T_B2GP = T_B2O @ T_O2GP
    # - 접근 거리 적용
    # - 그리퍼 너비 계산
    
    # 7. 실행 가능성 검사
    check_feasibility_multi_threaded(thread_data)
    # - Workspace 범위 확인
    # - 충돌 검사 (voxel grid)
    # - 접근 경로 충돌 검사
    
    return thread_data.is_feasible_pose
```

---

### 📤 단계 5: 결과 전송

#### 5-1. 토픽 발행 (비동기)

```python
def publish_matching_result(success):
    msg = MatchingResultMsg()
    
    if success:
        msg.is_pose = True
        msg.pose = grasp_pose                    # [m, deg]
        msg.sub_pose = grasp_sub_pose_set[0]
        msg.zig_pose = zig_pose
        msg.matching_accuracy = matching_accuracy
        msg.approach_distance = approach_distance
        msg.is_grasping_pose_flipped = is_flipped
        msg.gripper_open_length = gripper_open_length
        msg.gripper_close_length = gripper_close_length
        msg.gripper_tip_index = gripper_tip_index
        msg.detected_mask_num = detected_mask_num
    else:
        msg.is_pose = False
        msg.matching_accuracy = 0.0
    
    # 발행
    matching_pose_results_pub.publish(msg)
```

발행 토픽:
- `/cad_matching_result` (MatchingResultMsg): **메인 출력** - 로봇 제어용
- `/scanning_result` (MatchingResultMsg): 스캔 완료 알림
- `/cloud_matching_results` (PointCloud2): 시각화용
- `/cloud_pre_matching_results` (PointCloud2): 시각화용
- `/cloud_segmentation_results` (PointCloud2): 시각화용

#### 5-2. 서비스 응답 (동기)

```python
def do_matching_bin_picking_callback(request, response):
    # 기존 결과 사용 (이미 토픽에서 처리됨)
    if self.is_pose_assigned:
        response.is_pose = True
        response.pose = self.grasp_pose
        # ... (나머지 필드)
    else:
        response.is_pose = False
    
    return response
```

---

## 데이터 흐름 상세

### 입력 → 처리 → 출력

```
입력 데이터:
├── Zivid 점군 (XYZRGBA)        → cloud_scan
├── RGB 이미지                  → img_raw
├── SAM 마스크 (N개)            → mask_data
├── 로봇 DH 파라미터            → DH_parameters
├── 로봇 TCP                    → robot_tcp
└── 스캔 시 조인트 각도         → robot_scan_JS_position

                ↓ 처리 파이프라인

중간 데이터:
├── 마스크별 점군 추출          → cloud_mask_list[N]
├── 베이스 프레임 정렬          → cloud_base_aligned
├── 분할된 주요 세그먼트        → cloud_pre_matching_results
├── FPFH 특징                   → cloud_measured_fpfh
├── Quick align 변환            → T_pre_matching
├── ICP 정밀 변환               → T_process_matching
└── 매칭 정확도                 → matching_accuracy

                ↓ 후처리

출력 데이터:
├── 파지 포즈 (베이스 프레임)   → grasping_pose [x,y,z,rx,ry,rz]
├── 서브 포즈 세트              → grasp_sub_pose_set
├── 지그 포즈                   → zig_pose
├── 그리퍼 파라미터             → gripper_open/close_length
├── 매칭 정확도                 → matching_accuracy (%)
├── 실행 가능성                 → is_feasible_pose
└── 충돌 정보                   → is_collision
```

---

## 좌표계 변환 체인

```
[Sensor Frame]
       ↓ T_E2S (센서 캘리브레이션)
[End-Effector Frame]
       ↓ T_B2E (순기구학: DH + JS)
[Robot Base Frame]
       ↓ T_B2O (템플릿 매칭 결과)
[Object Frame]
       ↓ T_O2GP (CAD 정의)
[Grasp Point Frame]
       ↓ T_approach (접근 거리)
[Final Grasp Frame] → 로봇으로 전송
```

수식:
```
T_B2GP_final = T_B2E @ T_E2S @ T_S2O @ T_O2GP @ T_approach
```

여기서:
- `T_B2E`: Forward kinematics로 계산
- `T_E2S`: 센서 캘리브레이션 (JSON에서 로드)
- `T_S2O`: 템플릿 매칭 결과
- `T_O2GP`: CAD 모델 정의
- `T_approach`: 접근 거리 오프셋

---

## 멀티스레드 처리 전략

### 병렬 처리 구조

```python
# N개의 마스크 후보
mask_candidates = [mask_0, mask_1, ..., mask_N-1]

# 스레드 풀 생성
with ThreadPoolExecutor(max_workers=4):
    
    # 각 마스크를 병렬로 처리
    for i, mask in enumerate(mask_candidates):
        thread_data[i] = copy(target_data)
        thread_data[i].cloud_measured = mask
        
        # 스레드 시작
        future[i] = submit(
            matching_process,
            thread_data[i]
        )
    
    # 결과 대기 (먼저 성공하면 종료)
    for future in futures:
        if future.result() == SUCCESS:
            # 조기 종료
            is_pose_assigned.set()
            break
```

### 조기 종료 메커니즘

```python
# 공유 이벤트
is_pose_assigned = threading.Event()

# 각 스레드에서
def thread_process():
    # 매칭 시도
    if matching_success and feasible_pose:
        # 이벤트 설정 → 다른 스레드 중단
        is_pose_assigned.set()
        return True
    
    # 다른 스레드가 성공했는지 확인
    if is_pose_assigned.is_set():
        # 조기 종료
        return False
```

---

## 파라미터 설정

### config/matching_params.yaml

```yaml
matching_node:
  ros__parameters:
    # 멀티스레드
    num_threads: 4
    matching_method: 1  # 1=ICP, 2=GICP
    
    # 매칭 임계값
    matching_accuracy_limit: 50.0
    
    # Quick align
    do_quick_align: true
    quick_align_max_correspondence_distance: 0.05
    
    # ICP
    icp_max_iterations: 50
    icp_correspondence_distance: 0.05
    
    # 필터링
    do_euclidean_filtering: true
    euclidean_cluster_tolerance: 0.01
    
    # 충돌 검사
    do_collision_check: true
    collision_pt_threshold: 100
    
    # View frustum
    view_frustum: [-3.0, 3.0, -3.0, 3.0, -3.0, 3.0]
```

---

## 실행 방법

### 전체 시스템 실행

```bash
# 터미널 1: Zivid 카메라 노드
ros2 launch zivid_ros2_driver zivid_camera.launch.py

# 터미널 2: SAM 노드
ros2 run hanyang_sam_node sam_node.py

# 터미널 3: Zivid Scan 노드 (기존 C++)
ros2 run hanyang_matching_process zivid_scan_node

# 터미널 4: Open3D Matching 노드 (새로운)
ros2 launch hanyang_matching_open3d full_pipeline.launch.py
```

### 파라미터 조정 실행

```bash
ros2 launch hanyang_matching_open3d full_pipeline.launch.py \
    num_threads:=8 \
    matching_method:=2 \
    matching_accuracy_limit:=60.0 \
    debug_mode:=true
```

---

## 토픽/서비스 맵

### 입력 (Subscribe)

| 토픽 | 타입 | 출처 | 용도 |
|-----|------|-----|------|
| `/cloud_mask_results` | MaskCloud | Scan Node | **메인 입력** |
| `/zivid/color/image_color` | Image | Zivid | 디버깅용 |

### 출력 (Publish)

| 토픽 | 타입 | 수신자 | 용도 |
|-----|------|-------|------|
| `/cad_matching_result` | MatchingResultMsg | **로봇 제어** | **메인 출력** |
| `/scanning_result` | MatchingResultMsg | UI/모니터 | 스캔 완료 알림 |
| `/cloud_matching_results` | PointCloud2 | RViz | 시각화 |
| `/cloud_pre_matching_results` | PointCloud2 | RViz | 시각화 |
| `/cloud_segmentation_results` | PointCloud2 | RViz | 시각화 |

### 서비스 (Service)

| 서비스 | 타입 | 호출자 | 용도 |
|-------|------|-------|------|
| `/do_template_matching_bin_picking` | DoTemplateMatching | 로봇 제어 | 동기식 매칭 요청 |

---

## 성능 특성

### 처리 시간 (예상)

| 단계 | PCL | Open3D | 비고 |
|-----|-----|--------|------|
| 마스크 처리 | 100ms | 120ms | +20% |
| 좌표 변환 | 10ms | 15ms | +50% |
| 분할 | 200ms | 250ms | +25% |
| Quick align | 500ms | 600ms | +20% |
| ICP | 300ms | 400ms | +33% |
| GICP/Multi-scale | 800ms | 1000ms | +25% |
| 충돌 검사 | 100ms | 150ms | +50% |
| **총합 (싱글)** | **~2s** | **~2.5s** | **+25%** |
| **총합 (멀티×4)** | **~0.8s** | **~1.0s** | **+25%** |

### 메모리 사용 (예상)

| 항목 | PCL | Open3D |
|-----|-----|--------|
| 점군 (1M points) | 48MB | 60MB |
| 특징 데이터 | 10MB | 15MB |
| CAD 모델 | 20MB | 25MB |
| **총합** | **~100MB** | **~130MB** |

---

## 디버깅 및 모니터링

### 로그 출력

```bash
# 상세 로그 활성화
ros2 launch hanyang_matching_open3d full_pipeline.launch.py debug_mode:=true

# 또는
ros2 run hanyang_matching_open3d matching_node_full --ros-args --log-level debug
```

### RViz 시각화

RViz에서 추가할 토픽:
1. `/cloud_matching_results` - 매칭 결과 (녹색 + 파란색 CAD)
2. `/cloud_pre_matching_results` - 사전 매칭 (빨간색)
3. `/cloud_segmentation_results` - 분할 결과
4. `/aligned_points_xyz` - 베이스 정렬 점군

### 실시간 모니터링

```bash
# 매칭 결과 모니터
ros2 topic echo /cad_matching_result

# 성능 모니터
ros2 topic hz /cad_matching_result
ros2 topic bw /cloud_mask_results
```

---

## 트러블슈팅

### 문제: 포즈가 계산되지 않음

**체크리스트**:
1. `/cloud_mask_results` 토픽 수신 확인
2. `detected_mask_num > 0` 확인
3. `matching_accuracy >= matching_accuracy_limit` 확인
4. `is_feasible_pose == True` 확인 (충돌 없음)

**해결**:
```bash
# 임계값 낮추기
ros2 param set /hanyang_matching_open3d_node matching_accuracy_limit 40.0

# 충돌 검사 비활성화 (테스트용)
# config/matching_params.yaml에서 do_collision_check: false
```

### 문제: 매칭 시간이 너무 오래 걸림

**해결**:
```bash
# 스레드 수 증가
ros2 launch hanyang_matching_open3d full_pipeline.launch.py num_threads:=8

# Quick align 비활성화
# config/matching_params.yaml에서 do_quick_align: false

# 다운샘플링 증가
# voxel_size_downsample: 0.01 (10mm)
```

### 문제: 정확도가 낮음

**해결**:
```bash
# ICP 반복 증가
ros2 param set /hanyang_matching_open3d_node icp_max_iterations 100

# Quick align 활성화
# do_quick_align: true

# 다운샘플링 감소 (더 정밀)
# voxel_size_downsample: 0.002 (2mm)
```

---

## PCL vs Open3D 비교

### 완전 구현된 기능 ✅

- [x] Zivid 점군 수신
- [x] SAM 마스크 통합
- [x] 마스크별 점군 추출
- [x] 좌표계 변환 (센서→베이스)
- [x] 전처리 (필터링, 다운샘플링)
- [x] 분할 (Euclidean clustering)
- [x] 특징 추출 (FPFH)
- [x] Quick align (RANSAC)
- [x] ICP 정합
- [x] Multi-scale ICP (GICP 근사)
- [x] 파지 자세 계산
- [x] 그리퍼 너비 계산
- [x] 충돌 검사 (voxel grid)
- [x] 멀티스레드 병렬 처리
- [x] 결과 발행 (토픽/서비스)
- [x] RViz 시각화

### 근사 구현 ⚠️

- [~] GICP → Multi-scale ICP
- [~] OctoMap → VoxelGrid
- [~] Region Growing → DBSCAN

### 미구현 (옵션) ❌

- [ ] 시뮬레이션 기능
- [ ] 학습 데이터 수집
- [ ] 경로 계획 검증

---

## 다음 단계

1. **빌드 및 테스트**
   ```bash
   cd ~/hanyang_matching_ws
   colcon build --packages-select hanyang_matching_open3d
   source install/setup.bash
   ```

2. **단독 테스트**
   ```bash
   ros2 launch hanyang_matching_open3d full_pipeline.launch.py
   ```

3. **전체 시스템 통합**
   - Zivid + SAM + Scan Node + Matching Node 동시 실행
   - 로봇 제어 시스템과 연동

4. **성능 비교**
   - PCL vs Open3D 실행 시간 측정
   - 매칭 정확도 비교
   - 메모리 사용량 비교

---

**작성일**: 2025-10-22  
**버전**: 1.0.0  
**상태**: ✅ Full Pipeline Implemented


