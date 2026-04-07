# 드럼 체킹 로봇 사용 메시지 정의

> 드럼 체킹 워크플로우에서 실제 사용되는 커스텀 메시지/서비스만 정리.  
> 관련 노드: `detect_red_dot_node_for_drum`, `bin_picking_node`, `matching_node`, `qnode`

---

## 워크플로우 흐름

```
[Zivid 카메라]
     │ /zivid/color/image_color (sensor_msgs/Image)
     ▼
[detect_red_dot_node_for_drum]  ← keycode_ws
     │ /hanyang/coupler/keycode_angle (geometry_msgs/Vector3)
     ▼
[QNode / BinPickingNode]  ← hanyang_ui_ws
     │ ZividDoScan.srv  →  [Zivid Scanner Node]
     │ DoTemplateMatching.srv  →  [Matching Node]
     │ GripperCommand.srv  →  [그리퍼]
     │ MoveQ / MoveX.srv  →  [Robot Controller]
     ▼
[MatchingResultMsg] 결과 수신
```

---

## 1. 인식 / 스캐닝 단계

### hanyang_matching_msgs / bin_picking_msgs

| 이름 | 종류 | 주요 필드 | 설명 |
|---|---|---|---|
| `ZividDoScan` | srv | `scan`, `do_image_processing`, `do_single_matching`, `target_id`, `target_name`, `scan_position[]`, `robot_tcp[]`, `skip_detection_mask`, `mask_pixel_x/y` → `is_detected`, `is_saved` | **드럼 타겟 스캔 트리거.** `right_drum_key_code` 타겟명으로 호출 |
| `MaskCloud` | msg | `sam_result (Resultsam)`, `mask_cloud (PointCloud2)`, `scan_cloud (PointCloud2)`, `sampling_num`, `skip_detection_mask`, `mask_pixel_x/y`, `robot_tcp[]` | SAM 마스크 + 포인트클라우드 묶음. 매칭 노드로 전달 |
| `Resultsam` | msg | `boxes[]`, `class_ids[]`, `scores[]`, `masks[]`, `class_cnt`, `picking_poses[]` | SAM 세그멘테이션 결과. 드럼 키코드는 마스크 없어도 성공 처리 |
| `Capture` | srv | (없음) → (없음) | Zivid 캡처 단순 트리거 |
| `CaptureAssistantSuggestSettings` | srv | `max_capture_time`, `ambient_light_frequency` → (없음) | 드럼별 조명 환경에 맞는 캡처 설정 추천 |

---

## 2. 매칭 / 포즈 추정 단계

| 이름 | 종류 | 주요 필드 | 설명 |
|---|---|---|---|
| `DoTemplateMatching` | srv | `target_id`, `target_name`, `voxel_downsampling_size`, `segmentation_method`, `euclidean_cluster_tol`, `robot_dh_parameters[]`, `robot_tcp[]`, `is_symmetric` → `pose[]`, `sub_pose[]`, `is_pose`, `matching_accuracy`, `gripper_open/close_length`, `bp_result_set[]` | **메인 매칭 서비스.** 드럼 키코드 타겟은 매칭 스킵, 스캔만 수행 |
| `DoPoseEstimation` | srv | `target_id`, `target_name`, `feature_ext_method`, `voxel_downsampling_size` → `pose[]`, `sub_pose[]`, `is_pose`, `matching_accuracy`, `grp_width`, `approach_distance` | 포즈 추정 서비스 |
| `DoTaskRecognition` | srv | `do_task_recog`, `target_id` → `recog_stage_id`, `recog_part_id`, `is_recog` | 드럼 태스크 단계 인식 |
| `MatchingResultMsg` | msg | `pose[]`, `sub_pose[]`, `zig_pose[]`, `is_pose`, `matching_accuracy`, `matching_accuracy_limit`, `is_grasping_pose_flipped`, `gripper_open_length`, `gripper_close_length`, `detected_mask_num`, `gripper_tip_index`, `is_scanning_and_detection_finished`, `is_detection_success` | 매칭 최종 결과. UI에서 수신 후 파지 동작 결정 |
| `BinPickingResults` | msg | `pose[]`, `sub_pose[]`, `zig_pose[]`, `is_pose`, `matching_accuracy` | 복수 후보 결과 배열 (bp_result_set) |
| `DetectedPose` | msg | `pose[]`, `is_pose` | 단순 감지 포즈 |
| `DoTemplateMatchingMsg` | msg | `robot_id`, `target_id`, `target_name`, `voxel_downsampling_size`, `euclidean_cluster_tol`, `segmentation_method`, `rgs_*`, `color_rgs_*` | 매칭 파라미터 묶음 |
| `GraspingPose` | srv | `pose[]`, `is_pose` → (없음) | 파지 포즈 설정 |
| `GrpPose` | srv | `rgb_req (Image)`, `cloudreq (PointCloud2)`, `wholecloudreq (PointCloud2)` → `n_grpposes_res`, `grppos_res[]`, `grprot_res[]` | RGB + 포인트클라우드 → 그리퍼 파지 포즈 후보 |

---

## 3. 로봇 제어 단계

### kcr_control_msg (컨트롤러 제어)

| 이름 | 종류 | 주요 필드 | 설명 |
|---|---|---|---|
| `RobotState` | msg | `q_meas[]`, `qd_meas[]`, `x_meas[]`, `torque_meas[]`, `force_meas[]`, `is_enable`, `is_path_operating`, `is_collision_detection_mode`, `is_teaching_mode`, `flag_collision`, `flag_torque_limit` | 컨트롤러 전체 상태. 드럼 회전 각도(J6) 적용 시 참조 |
| `RscState` | msg | `rsc_state`, `rsc_input_state`, `is_joint_connected` | 안전 컨트롤러 상태 |
| `MoveQ` | srv | `q_target[]`, `qd_target`, `qdd_target`, `is_relative` → `error` | 조인트 공간 이동. 드럼 회전 각도 보정 시 J6 단독 사용 |
| `MoveX` | srv | `x_target[]`, `xd_target`, `xdd_target`, `is_relative`, `is_base_frame` → `error` | 태스크 공간 이동. 드럼 어프로치 시 tool frame 기준 X축 방향 이동 |
| `Blending` | srv | `xs_target[]`, `xds_target[]`, `radiuses[]`, `waypoint_num` → `error` | 블렌딩 경로 이동 |
| `Jog` | srv | `index`, `vel`, `acc`, `is_joint_space`, `jog_flag` → `error` | 단일 축 조그 |
| `XpadJog` | srv | `xd[]`, `is_jog_btn_pressed` → `successed` | 조이스틱 조그 |
| `Impedance` | srv | `flag_impedance`, `imped_selection[]`, `m_gain[]`, `b_gain[]`, `k_gain[]`, `force_limit[]`, `force_des[]` → `error` | 임피던스 제어. 드럼 커버 조립 시 컴플라이언스 제어 |
| `DirectTorqueControl` | srv | `setting_mode`, `dtc_mode[]`, `dtc_input[]` → `flag` | 직접 토크 제어 |
| `TcpPayload` | srv | `tcp[]`, `payload`, `flag_tcp`, `flag_payload` → `error` | TCP/페이로드 설정 |
| `StopRobot` | srv | `time_stop` → `error` | 로봇 정지 |
| `SingleBool` | srv | `flag` → `error` | 단순 bool 명령 (서보 on/off 등) |
| `SingleString` | srv | `command` → `error` | 문자열 명령 |
| `Command` | srv | `command`, `value`, `address` → `error` | 저수준 레지스터 명령 |
| `OperatingStatus` | srv | `is_path_finish`, `is_assembly_finish` → `flag` | 동작 완료 상태 확인 |

### grp_control_msg (그리퍼 제어)

| 이름 | 종류 | 주요 필드 | 설명 |
|---|---|---|---|
| `GripperMsg` | msg | `motor_position`, `motor_current`, `finger_position`, `grp_opened`, `grp_closed`, `motor_fault` | 그리퍼 현재 상태 |
| `GripperCommand` | srv | `command`, `value_1`, `value_2`, `slave_num` → `successed` | 그리퍼 명령 |
| `PosVelCurCtrl` | srv | `position`, `velocity`, `current`, `duration` → `successed` | 그리퍼 위치/속도/전류 복합 제어 |
| `SingleInt` | srv | `value (int16)` → `successed` | 단일 값 명령 |
| `StopMotor` | srv | `duration` → `successed` | 그리퍼 모터 정지 |
| `DriverEnable` | srv | `enable` → `successed` | 그리퍼 드라이버 활성화 |
| `Void` | srv | (없음) → `successed` | 빈 요청 (초기화 등) |

---

## 4. UI 상태 교환

### hanyang_matching_msgs / bin_picking_msgs

| 이름 | 종류 | 주요 필드 | 설명 |
|---|---|---|---|
| `RobotState` | msg | `actualq[]`, `actualx[]`, `actualforce[]`, `is_moving`, `connect_state` | UI에서 로봇 현재 상태 표시용 |
| `BpUiCommand` | msg | `meas_js_angle[]`, `meas_cs_pose[]`, `meas_force_sensor[]`, `mode`, `gripper_state` | UI ↔ 컨트롤러 상태 교환 |
| `BpUiCommandLearning` / `ScanCommandMsg` | msg | `target_name`, `sam_mean_size`, `sam_mask_min/max_area` | UI에서 스캔 명령 전달 |
| `GripperCmd` | msg | `position`, `speed`, `force`, `stop`, `emergency_release` | UI → 그리퍼 제어 명령 |
| `SetTcp` | srv | `joint_position[]`, `tcp_idx`, `get_measured_js_value` → `pose[]`, `js_position_output[]` | TCP 설정 및 현재 포즈 조회 |
| `PartScanning` | srv | `is_scanned` → (없음) | 부품 스캔 완료 상태 |
| `PartDetection` | srv | `is_detected` → (없음) | 부품 감지 상태 |

---

## 드럼 체킹 전용 특이사항

1. **`right_drum_key_code` 타겟** — `DoTemplateMatching` 호출 시 매칭 단계를 건너뛰고 스캔만 수행. `detected_mask_num=0` 이어도 성공으로 처리.
2. **키코드 각도 보정** — `/hanyang/coupler/keycode_angle` (Vector3)로 수신한 드럼 회전 각도를 J6 조인트에 오프셋으로 적용 (`is_relative=true`, J6만 사용).
3. **어프로치 방향** — `drum_task` 워크스페이스는 tool frame 기준 **-X 방향**으로 어프로치 거리 적용.
