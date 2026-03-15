# main01 코드 분석 리포트

작성일: 2026-03-08
분석 대상: /media/hanku/1bbc3839-6da8-4713-a264-4b48c4bd85f6/hye_ws/robot_a/main01

---

## 0. 코드 분석 진입 순서 (시니어 개발자용)

> 동영상으로 동작을 이미 봤고, ROS 경험이 있는 상태 기준.
> 목표: "어떤 코드가 어떤 동작을 만드는가"를 가장 빠르게 연결하는 순서.

---

### STEP 1 — 시스템 경계와 하드웨어 연결부터 확인 (30분)

코드를 읽기 전에 "무엇이 무엇과 연결되어 있는가"를 먼저 파악한다.

**읽을 파일:**
```
other01/bashrc
```

확인할 것:
- 어떤 워크스페이스가 몇 개인가 (source 순서 = 의존성 순서)
- 실행 alias에서 IP/포트 확인 → `192.168.100.108:12345` (두산 로봇 컨트롤러)
- `/dev/ttyUSB0` → 그리퍼 (Modbus RTU RS485)
- 어떤 alias가 무엇을 실행하는지 한눈에 파악

**파악 목표:**
```
ui(Qt앱) ↔ dscontroller(두산 드라이버) ↔ 로봇 하드웨어 (192.168.100.108)
         ↔ grip(그리퍼 드라이버)        ↔ /dev/ttyUSB0
         ↔ zvd(Zivid 스캐너)
         ↔ sam, mc(매칭 노드)
         ↔ keycode1, keycode2
```

---

### STEP 2 — 메시지/서비스 인터페이스 파악 (30분)

각 노드 사이에 어떤 데이터가 오가는지 파악. 코드 읽기 전에 인터페이스를 알아야 구현을 이해할 수 있다.

**읽을 파일 (4개):**
```
msg_ws/src/grp_control_msg/srv/PosVelCurCtrl.srv       # 그리퍼 위치 제어
msg_ws/src/grp_control_msg/msg/GripperMsg.msg           # 그리퍼 상태 피드백
msg_ws/src/hanyang_matching_msgs/srv/DoTemplateMatching.srv  # 물체 인식 요청/응답
msg_ws/src/hanyang_matching_msgs/srv/ZividDoScan.srv    # 스캔 트리거
```

**파악 목표:**
- 그리퍼: position(int16), velocity, current, duration → bool successed
- 매칭: object_name 요청 → pose[6], accuracy, gripper_open/close_length 응답
- 이것만 알면 UI↔드라이버 인터페이스 완성

---

### STEP 3 — Task 정의 파일로 "동작 시나리오" 읽기 (1~2시간)

**가장 중요한 파일. 전체 로봇 동작이 여기 정의되어 있다.**

```
hanyang_ui_ws/src/robot_control_config/ui_config/task_planner_hanyang_eng.cpp
```

**읽는 전략 (전체를 다 읽지 말 것 — 6722줄):**

1. 파일 상단에서 함수 목록 파악 (genXxxTask 패턴)
2. `genScanTask()` 읽기 → 스캔 포즈 이동 + 스캔 실행 시퀀스 이해
3. `rightGenSubTask1Task()` 읽기 → 별형 뚜껑 나사 풀기 6사이클 루프 (핵심 동작)
4. `genRightDrumPLCCommTask()` 읽기 → PLC 시퀀스와 홈 복귀

**파악 목표:**
- `vector<UnitTask>`로 작업 시퀀스를 미리 만들어 놓는다는 패턴 이해
- `taskPushBack_jsMove2()`, `taskPushBack_csMoveToolFrame()` 같은 헬퍼 함수들이 UnitTask를 push하는 것임을 확인
- 뚜껑 열기 = J6축 -195도 × 6사이클임을 코드에서 확인

---

### STEP 4 — Task 데이터 구조 파악 (30분)

STEP 3에서 읽다 보면 UnitTask 구조체가 뭔지 궁금해진다. 이때 읽는다.

**읽을 파일 (2개):**
```
hanyang_ui_ws/src/koras_system/include/task_planner_default.hpp  # UnitTask 구조체, KR_GRP enum
hanyang_ui_ws/src/koras_system/include/task_parameter.hpp        # Task enum (X-Macro 패턴)
```

**파악 목표:**
- `UnitTask` 안에 어떤 필드가 있는가 (q_target, x_target, grp_command, object_name 등)
- `KR_GRP` enum: MOTOR_ENABLE=1, MOTOR_POS_CTRL=5, POS_RESET=8, OPEN=102, CLOSE=103
- Task enum이 X-Macro로 관리된다는 패턴 인식

---

### STEP 5 — Task 실행 엔진 (qnode.cpp 핵심부만) (1~2시간)

**읽을 파일:**
```
hanyang_ui_ws/src/koras_system/src/qnode.cpp  (~11000줄 — 절대 전체 읽지 말 것)
```

**읽는 전략:**
1. `doControl()` 함수 찾기 → 10ms 타이머 루프임을 확인
2. `doTask()` 함수 찾기 → switch-case 구조 파악
3. `TASK_JSMOVE2` case → `move_joint` 서비스 호출 코드 확인
4. `TASK_KORAS_GRIPPER_COMMAND` case → `setGrpMotorCtrl()` 호출 흐름 확인
5. `setGrpMotorCtrl()` 함수 → `/motor_pos_ctrl` 서비스 클라이언트 (200ms 타임아웃)
6. 토크 가드 코드 → 토크 임계 초과 시 MoveStop + 레이블 점프

**파악 목표:**
- 큐에 쌓인 UnitTask를 10ms마다 하나씩 꺼내 실행하는 구조
- 동기(SYNC) 서비스 호출 → 완료 대기 후 다음 Task로 진행
- 비정상 상황(토크 초과) 처리 로직

---

### STEP 6 — 그리퍼 드라이버 내부 (1시간)

STEP 5에서 `/motor_pos_ctrl` 서비스가 어디서 처리되는지 궁금해지면 읽는다.

**읽을 파일 순서:**
```
gripper_ws/src/koras_gripper/.../include/modbus_comm.hpp        # Modbus RTU 통신 구현
gripper_ws/src/koras_gripper/.../include/datc_ctrl.hpp          # DATC 명령 enum + 함수 선언
gripper_ws/src/koras_gripper/.../src/datc_ctrl.cpp              # 실제 명령 전송 구현
gripper_ws/src/koras_gripper/.../src/datc_comm_interface.cpp    # ROS2 서비스 서버 (버그 위치!)
```

**주목할 포인트:**
- `SEND_CMD_VECTOR({5, pos, duration})` → Modbus 레지스터 0에 3개 값 write
- 상태 읽기: 레지스터 10~17번에서 motor_pos, motor_cur, motor_vel 등 읽음 (100Hz)
- **버그**: `datc_comm_interface.cpp` 102번째 줄 → `req->duration` 무시, 항상 600ms 고정

**파악 목표:**
- RS485 → Modbus RTU → `/dev/ttyUSB0` → DATC 하드웨어 통신 구조
- POS_RESET(8) → 딜레이 → MOTOR_POS_CTRL(5) 패턴이 왜 필요한지 (상대 위치 제어이기 때문)

---

### STEP 7 — 매칭 파이프라인 (필요할 때) (1~2시간)

비전 인식 이슈(Task 1 실패, 가장 최근 미해결)를 디버깅하거나 매칭 파라미터를 튜닝할 때 읽는다.

**읽을 파일 순서:**
```
hanyang_matching_ws/src/hanyang_zivid_scanner_node/src/zivid_node.py        # Zivid 스캔 노드
hanyang_matching_ws/src/hanyang_sam_node/nodes/sam_node.py                   # SAM 세그멘테이션
hanyang_matching_ws/src/hanyang_matching_open3d/config/matching_params.yaml  # 파라미터 (튜닝 진입점)
hanyang_matching_ws/src/hanyang_matching_open3d/nodes/matching_node_full.py  # Open3D 매칭 구현
```

**파악 목표:**
- Zivid → SAM → Open3D RANSAC + ICP 흐름
- `matching_params.yaml`에서 `matching_accuracy_limit`, `voxel_size_downsample` 등 튜닝 포인트
- Task 1 비전 실패 → 정확도 임계값 or 스캔 품질 문제일 가능성

---

### STEP 8 — 키코드 검출 (필요할 때) (30분)

커플러 구멍 정렬이 안 맞거나 J6 각도 관련 이슈가 생길 때 읽는다.

**읽을 파일:**
```
keycode_ws/src/keyring_detection/keyring_detection/detect_red_dot_node_for_drum.py
```

**파악 목표:**
- 흰색 링 검출 → LSQ 원 피팅 → 빨간 점 각도 → `/hanyang/coupler/keycode_angle` 퍼블리시
- 각도 계산: `atan2(vx, -vy)` (Y축 기준 시계방향)

---

### 분석 우선순위 요약

```
필수 (반드시 읽기):
  STEP 1  other01/bashrc                          (30분)
  STEP 3  task_planner_hanyang_eng.cpp 일부        (1~2시간)
  STEP 4  task_planner_default.hpp, task_parameter.hpp  (30분)
  STEP 5  qnode.cpp 핵심 함수만                   (1~2시간)

필요시 읽기:
  STEP 2  msg_ws 서비스 정의                       (30분)
  STEP 6  gripper_ws 드라이버                      (1시간)
  STEP 7  hanyang_matching_ws                      (1~2시간)
  STEP 8  keycode_ws                               (30분)
```

**총 필수 분석 시간 목표: 3~5시간**

---

### 디버깅 진입점 빠른 참조

| 증상 | 먼저 볼 곳 |
|------|-----------|
| 로봇이 목표 위치로 안 감 | `task_planner_hanyang_eng.cpp` — 해당 SubTask 좌표값 |
| 그리퍼가 안 움직임 | `datc_comm_interface.cpp` + `/dev/ttyUSB0` 권한 확인 (`sudo chmod a+rw /dev/ttyUSB0`) |
| 비전 인식 실패 (Task 1) | `matching_params.yaml` accuracy_limit + `zivid_node.py` 스캔 품질 |
| J6 각도 정렬 안 됨 | `detect_red_dot_node_for_drum.py` + `/hanyang/coupler/keycode_angle` 토픽 확인 |
| 토크 에러로 로봇 정지 | `qnode.cpp` 토크 가드 임계값 + 뚜껑 나사 상태 확인 |
| PLC 신호 안 옴 | `qnode.cpp` TASK_PLC_WRITE_* case + Modbus TCP 연결 확인 |

---

---

## 1. 시스템 개요

두산 M1013 6축 로봇 암으로 드럼통 뚜껑을 열고 닫는 자동화 시스템.

- **로봇**: Doosan M1013 (6축)
- **3D 카메라**: Zivid 2+ MR60 Rev A0
- **바코드 리더**: Keyence 이더넷 바코드 리더
- **그리퍼**: DATC 모터 그리퍼 (Modbus RTU RS485, /dev/ttyUSB0)
- **OS**: Ubuntu 22.04 + ROS2 Humble

---

## 2. 워크스페이스 구조

```
main01/
├── hanyang_ui_ws/          # Qt5 C++ UI + 로봇 제어 (핵심)
├── hanyang_matching_ws/    # 3D 인식 파이프라인
├── keycode_ws/             # 빨간 점 각도 검출
├── msg_ws/                 # 커스텀 ROS2 메시지 정의
├── doosan_ros2_ws/         # 두산 로봇 ROS2 드라이버
├── gripper_ws/             # 그리퍼 드라이버
└── other01/                # 메모, bashrc, 이슈 기록
```

bashrc 소스 순서 (의존성 순서):
```
/opt/ros/humble → ros2_numpy_ws → keycode_ws → pcl_ros_ws → msg_ws
→ doosan_ros2_ws → gripper_ws → hanyang_ui_ws(hanyang_matching_ws)
```

---

## 3. 핵심 서브시스템 1: Task 실행 엔진

### 3-1. 전체 흐름

```
[Qt UI 버튼 클릭]
       ↓
task_planner_hanyang_eng.cpp  (6722줄 — 가장 중요한 파일)
  → vector<UnitTask> 생성 (작업 시퀀스 정의)
       ↓
QNode::doControl() [10ms 타이머 루프]
  → doTask() → switch-case (UnitTask.task_mode 기준)
       ├─ TASK_JSMOVE2              → dsr01/motion/move_joint 서비스
       ├─ TASK_CSMOVE2             → dsr01/motion/move_line 서비스
       ├─ TASK_CSMOVE_TOOL_FRAME   → dsr01/motion/move_line (tool frame 기준)
       ├─ TASK_KORAS_GRIPPER_COMMAND → /motor_reset_pose, /motor_pos_ctrl 서비스
       ├─ TASK_3D_SCANNING_TARGET_OBJECT → /zivid_scanning 서비스
       ├─ TASK_MATCHING_TARGET_OBJECT   → /do_template_matching_bin_picking 서비스
       ├─ TASK_PLC_WRITE_*         → Modbus TCP (PLC/AMR 통신)
       └─ TASK_JSMOVE2_KEYCODE_J6_ONLY → 키코드 각도로 J6만 이동
```

### 3-2. UnitTask 구조체 (`task_planner_default.hpp`)

```cpp
struct UnitTask {
    Task task_mode;           // TASK_ENUM_LIST의 X-Macro로 정의된 enum
    JsDouble q_target;        // 관절 목표각 [6]
    CsDouble x_target;        // 직교좌표 목표 [6]
    double vel_js, acc_js;    // 관절 속도/가속도 (기본: 30.0 / 45.0)
    double vel_cs, acc_cs;    // 직교 속도/가속도 (기본: 0.1m/s / 0.2m/s²)
    bool relative;            // 상대/절대 이동
    uint16_t grp_command;     // KR_GRP enum 값
    uint16_t grp_value;       // 그리퍼 목표 위치값
    uint16_t grp_address;
    uint16_t register_address, bit_address;
    bool status;
    double angle;
    std::string barcode;
    std::string object_name;
    bool is_symmetric;
    BinPickingParam bp_param;
};
```

### 3-3. KR_GRP enum (그리퍼 명령)

```cpp
enum KR_GRP {
    MOTOR_ENABLE   = 1,   // 모터 활성화
    MOTOR_POS_CTRL = 5,   // 위치 제어
    POS_RESET      = 8,   // 엔코더 0으로 리셋
    INIT           = 101, // 그리퍼 초기화
    OPEN           = 102, // 그리퍼 열기
    CLOSE          = 103, // 그리퍼 닫기
    POS_CTRL       = 104, // 핑거 위치 제어
};
```

### 3-4. X-Macro 패턴 (`task_parameter.hpp`)

```cpp
#define TASK_ENUM_LIST \
    X(TASK_JSMOVE2) \
    X(TASK_CSMOVE2) \
    X(TASK_CSMOVE_TOOL_FRAME) \
    X(TASK_KORAS_GRIPPER_COMMAND) \
    X(TASK_3D_SCANNING_TARGET_OBJECT) \
    X(TASK_MATCHING_TARGET_OBJECT) \
    X(TASK_PLC_WRITE_STATUS_FROM_ROBOT_TO_AMR) \
    X(TASK_PLC_WRITE_ROTATION_ANGLE_FROM_ROBOT_TO_AMR) \
    X(TASK_PLC_WRITE_BARCODE_FROM_ROBOT_TO_AMR) \
    X(TASK_JSMOVE2_KEYCODE_J6_ONLY) \
    X(TASK_CSMOVE_TOOL_FRAME_KEYCODE) \
    // ... (총 754줄)
```

### 3-5. 토크 가드 (torque guard)

`qnode.cpp`에서 10ms마다 관절 토크 감시:
- 토크 임계값 초과 시 → `drflMoveStop()` 호출
- 레이블 "SCREW_OPEN_STEP"으로 점프 (뚜껑 열기 자동 중단 후 복구)
- 이슈 메모: 토크 한계 1.0으로 낮춤 (로봇 셧다운 방지)

### 3-6. TCP (Tool Center Point) 전환

| TCP | 용도 |
|-----|------|
| tcp00 | 스캔 포즈 |
| tcp03 | 드럼 구멍 |
| tcp05 | 별형 뚜껑 (star lid) |
| tcp06 | 커플러 |

---

## 4. 핵심 서브시스템 2: 뚜껑 열기 시퀀스

### 4-1. SubTask 목록 (`task_planner_hanyang_eng.cpp`)

| SubTask | 내용 |
|---------|------|
| genScanTask() | Zivid 스캔 포즈 이동 + 스캔 실행 |
| rightGenSubTask1Task() | 별형 뚜껑(star lid) 제거 — 6사이클 회전 |
| rightGenSubTask2Task() | 커플러 처리 |
| rightGenSubTask3~24() | 이후 세부 작업들 |
| genRightDrumPLCCommTask() | PLC 시퀀스 종료 후 홈 포즈 복귀 |

### 4-2. 뚜껑 나사 풀기 6사이클 루프 (SubTask1)

```cpp
for (int cycle = 0; cycle < 6; cycle++) {
    // 1. 그리퍼로 뚜껑 잡기
    right_task_sub_1.insert(...drum_grp_lid_cap_close_...);
    taskPushBack_delay_1ms(right_task_sub_1, 3000);

    // 2. J6축 -195도 회전 (나사 풀기)
    taskPushBack_jsMove2(right_task_sub_1, {0,0,0,0,0,-195}, 70.0, 70.0, true);

    // 3. 그리퍼 열기 (뚜껑 놓기)
    right_task_sub_1.insert(...drum_grp_lid_cap_open_...);
    taskPushBack_jsMove2(right_task_sub_1, {0,0,0,0,0,15}, 40.0, 40.0, true);

    // 4. 위로 올리기
    taskPushBack_csMoveToolFrame(...{0,0,-0.050,0,0,0}...);

    // 5. J6 재위치 (180도 회전)
    taskPushBack_jsMove2(right_task_sub_1, {0,0,0,0,0,180}, 70.0, 70.0, true);

    // 6. 다시 내려가기
    taskPushBack_csMoveToolFrame(...{0,0,0.050,0,0,0}...);
}
// 총 6 × (-195 + 15 + 180) = 0도 (원위치)
// 실제 나사 풀림: 6 × 195 ≈ 3.25바퀴
```

이슈 메모: 사이클 수가 4에서 6으로 증가됨 (더 많은 회전 필요)

---

## 5. 핵심 서브시스템 3: 매칭 파이프라인

### 5-1. 전체 인식 파이프라인

```
Zivid 카메라
    ↓ /zivid/points/xyzrgba (PointCloud2)
    ↓ /zivid/color/image_color (RGB 이미지)
SAM Node (Segment Anything Model)
    ↓ /cloud_mask_results (MaskCloud msg)
Matching Node (PCL C++ 또는 Open3D Python)
    ↓ /cad_matching_result (MatchingResultMsg)
    ↓ /do_template_matching_bin_picking 서비스 응답
QNode (BinPickingNode)
    → detected_pose_[6], matching_accuracy, gripper_open/close_length, approach_distance
```

### 5-2. 두 가지 매칭 구현

| 구현 | 파일 | 언어 | 상태 |
|------|------|------|------|
| PCL C++ | `hanyang_matching_process/src/matching_node.cpp` | C++ | 레거시 |
| Open3D Python | `hanyang_matching_open3d/nodes/matching_node_full.py` | Python | 권장 |

둘 다 동일 서비스명 `/do_template_matching_bin_picking` 사용

### 5-3. DoTemplateMatching.srv

```
# Request
string target_name         # 매칭할 객체 이름
int32  target_id
float64[] robot_dh_parameters
float64[] robot_tcp
float64 matching_accuracy_limit
# ... 매칭 파라미터들

---
# Response
float64[6] pose            # 감지된 포즈 [X,Y,Z,Rx,Ry,Rz]
float64 matching_accuracy
float64 gripper_open_length
float64 gripper_close_length
float64 approach_distance
```

### 5-4. ZividDoScan.srv

```
# Request
bool scan
string target_name
float64[] robot_dh_parameters
float64[] robot_tcp
float64[] scan_position
---
# Response
bool is_detected
bool is_saved
```

### 5-5. 키코드 검출 (`keycode_ws`)

```python
# detect_red_dot_node_for_drum.py
구독: /zivid/color/image_color
처리:
  1. HSV 필터로 흰색 링 검출
  2. 최소제곱법(LSQ)으로 원 피팅
  3. 원 내부에서 빨간 점 검출
  4. 각도 계산: atan2(vx, -vy) (Y축 기준 시계방향)
퍼블리시: /hanyang/coupler/keycode_angle (Vector3: x=vx, y=vy, z=angle°)
```

→ `drum_rotation_angle_`로 저장 → J6축 목표각으로 사용 (커플러 구멍 정렬)

---

## 6. 핵심 서브시스템 4: 그리퍼 드라이버

### 6-1. 통신 스택 전체

```
QNode (hanyang_ui_ws)
    setGrpMotorCtrl() — 200ms 타임아웃
       ↓ /motor_pos_ctrl 서비스 (PosVelCurCtrl.srv)
DatcCommInterface (gripper_ws) — ROS2 서비스 서버
    motorPosCtrl(position, 600)  ← duration 항상 600ms 고정!
       ↓
DatcCtrl::command(MOTOR_POSITION_CONTROL, pos, duration)
    SEND_CMD_VECTOR({5, pos, 600})
       ↓
ModbusComm::sendData(reg_addr=0, data)
    modbus_write_registers(mb_, 0, 3, data)
       ↓ Modbus RTU RS485
/dev/ttyUSB0 → DATC 그리퍼 모터 하드웨어
```

### 6-2. Modbus 프로토콜 상세 (`modbus_comm.hpp`)

```cpp
// 초기화
modbus_new_rtu(port_name, baudrate, 'N', 8, 1)  // 8N1
modbus_rtu_set_serial_mode(mb_, MODBUS_RTU_RS485)
modbus_rtu_set_rts_delay(mb_, 300)

// 명령 전송 (reg 0에 쓰기)
modbus_write_register(mb_, 0, cmd)          // 단일 명령
modbus_write_registers(mb_, 0, n, data)    // 다중 (cmd+params)

// 상태 읽기 (reg 10~17, 100Hz)
modbus_read_registers(mb_, 10, 8, data)
// reg10: 상태 비트, reg11: motor_pos(int16), reg12: motor_cur(int16)
// reg13: motor_vel(int16), reg14: finger_pos, reg17: voltage
```

### 6-3. DATC 명령 코드 (`datc_ctrl.hpp`)

```
DATC_COMMAND enum:
MOTOR_ENABLE=1           → sendData(0, [1])
MOTOR_STOP=2             → sendData(0, [2])
MOTOR_DISABLE=4          → sendData(0, [4])
MOTOR_POSITION_CONTROL=5 → sendData(0, [5, pos, duration])
MOTOR_VELOCITY_CONTROL=6 → sendData(0, [6, vel, 500])
MOTOR_CURRENT_CONTROL=7  → sendData(0, [7, cur, 500])
MOTOR_RESET_POSE=8       → sendData(0, [8])
GRIPPER_INITIALIZE=101   → sendData(0, [101])
GRIPPER_OPEN=102         → sendData(0, [102])
GRIPPER_CLOSE=103        → sendData(0, [103])
SET_FINGER_POSITION=104  → sendData(0, [104, finger_pos])
SET_MOTOR_TORQUE=212     → sendData(0, [212, torque_ratio])   // 50~100%
SET_MOTOR_SPEED=213      → sendData(0, [213, speed_ratio])    // 0~100%
```

### 6-4. 상태 비트 맵

```
bit0: Motor Enable       bit1: Gripper Initialize
bit2: Motor Pos Ctrl     bit3: Motor Vel Ctrl
bit4: Motor Cur Ctrl     bit5: Gripper Open
bit6: Gripper Close      bit9: Motor Fault
```

### 6-5. POS_RESET 패턴

```cpp
// qnode.cpp의 그리퍼 제어 패턴
// 항상: 리셋(8) → 위치제어(5) 순서
taskPushBack_grpCommand(tasks, POS_RESET, 0);        // 엔코더 0 초기화
taskPushBack_delay_1ms(tasks, 200);
taskPushBack_grpCommand(tasks, MOTOR_POS_CTRL, pos); // 상대 위치로 이동
```

위치값 범위: -9500 ~ +4000 (int16)

### 6-6. 발견된 버그

`datc_comm_interface.cpp` 102번째 줄:
```cpp
// req->duration 필드가 서비스 요청에서 전달되어도 무시됨
// res->successed = motorPosCtrl(req->position, req->duration);  // 원래 의도
res->successed = motorPosCtrl(req->position, 600);               // 실제 코드
```
→ PosVelCurCtrl.srv의 `duration` 필드는 실제로 동작하지 않음 (항상 600ms)

---

## 7. 두산 로봇 드라이버 (`doosan_ros2_ws`)

### 7-1. 주요 서비스 정의

**MoveJoint.srv** (`dsr_msgs2/srv/motion/`)
```
float64[6] pos         # 목표 관절각 [도]
float64    vel         # 속도 [deg/sec]
float64    acc         # 가속도 [deg/sec²]
float64    time        # 시간 [sec] (0이면 vel/acc로 계산)
float64    radius      # 블렌딩 반경 [mm]
int8       mode        # 0=절대, 1=상대
int8       blend_type  # 0=DUPLICATE, 1=OVERRIDE
int8       sync_type   # 0=SYNC(완료 대기), 1=ASYNC
---
bool success
```

**MoveLine.srv**
```
float64[6] pos         # 목표 위치 [X,Y,Z,Rx,Ry,Rz]
float64[2] vel         # [mm/sec, deg/sec]
float64[2] acc         # [mm/sec², deg/sec²]
float64    time
float64    radius
int8       ref         # 0=BASE, 1=TOOL, 2=WORLD
int8       mode        # 0=절대, 1=상대
int8       blend_type
int8       sync_type   # 0=SYNC, 1=ASYNC
---
bool success
```

### 7-2. 워크스페이스 패키지 구성

```
doosan_ros2_ws/src/doosan-robot2/
├── dsr_msgs2/        # 서비스/메시지 정의 (motion, tcp, tool, io, modbus 등)
├── dsr_hardware2/    # 실제 EtherCAT 통신 구현 (dsr_hw_interface2.cpp)
├── dsr_bringup2/     # 런치 파일 (real/virtual 모드)
├── dsr_controller2/  # ros2_control 컨트롤러
├── dsr_description2/ # URDF 모델
└── dsr_example2/     # 예제 코드
```

연결 정보 (bashrc alias):
```bash
alias dscontroller='ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=real host:=192.168.100.108 port:=12345 model:=m1013 gui:=false'
alias vcontroller='ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
    mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013'
```

---

## 8. 커스텀 메시지 패키지 (`msg_ws`)

### 8-1. 패키지 목록

| 패키지 | 용도 |
|--------|------|
| hanyang_matching_msgs | 매칭/스캔 서비스 정의 |
| grp_control_msg | 그리퍼 제어 서비스/메시지 |
| kcr_control_msg | 로봇 상태 메시지 (토크 포함) |
| bin_picking_msgs | 빈피킹 관련 |
| llm_msgs | LLM 제어 관련 |

### 8-2. PosVelCurCtrl.srv (`grp_control_msg`)

```
int16   position    # 목표 모터 위치
int16   velocity    # 목표 속도
int16   current     # 목표 전류
uint16  duration    # 지속시간 [ms] (실제론 무시됨 — 버그)
---
bool successed
```

### 8-3. GripperMsg.msg (100Hz 퍼블리시)

```
int16   motor_position       # 현재 모터 위치
int16   motor_current        # 현재 전류
int16   motor_velocity       # 현재 속도
uint16  finger_position      # 핑거 위치
bool    motor_enabled
bool    gripper_initialized
bool    position_ctrl_mode
bool    velocity_ctrl_mode
bool    current_ctrl_mode
bool    grp_opened
bool    grp_closed
bool    motor_fault
```

퍼블리시 토픽: `/grp_state` → QNode가 구독

---

## 9. ROS2 토픽/서비스 전체 맵

### 토픽

| 토픽 | 메시지 타입 | 방향 | 용도 |
|------|------------|------|------|
| `/zivid/points/xyzrgba` | PointCloud2 | Zivid → SAM | 포인트 클라우드 |
| `/zivid/color/image_color` | Image | Zivid → SAM, keycode | RGB 이미지 |
| `/cloud_mask_results` | MaskCloud | SAM → Matching | 마스크+클라우드 |
| `/cad_matching_result` | MatchingResultMsg | Matching → QNode | 포즈 결과 |
| `/hanyang/coupler/keycode_angle` | Vector3 | keycode → QNode | 드럼 각도 |
| `/hanyang/coupler/keycode_holder_angle` | Vector3 | keycode → QNode | 홀더 각도 |
| `dsr01/joint_states` | JointState | DSR → QNode | 로봇 관절 상태 |
| `/grp_state` | GripperMsg | Gripper → QNode | 그리퍼 상태 |

### 서비스

| 서비스 | 타입 | 서버 | 용도 |
|--------|------|------|------|
| `dsr01/motion/move_joint` | MoveJoint.srv | dsr_hardware2 | 관절 이동 |
| `dsr01/motion/move_line` | MoveLine.srv | dsr_hardware2 | 직선 이동 |
| `dsr01/tcp/set_current_tcp` | SetCurrentTcp.srv | dsr_hardware2 | TCP 변경 |
| `/zivid_scanning` | ZividDoScan.srv | zivid_node.py | 스캔 실행 |
| `/do_template_matching_bin_picking` | DoTemplateMatching.srv | matching_node | 물체 인식 |
| `/motor_enable` | Void.srv | DatcCommInterface | 모터 활성화 |
| `/motor_reset_pose` | Void.srv | DatcCommInterface | 엔코더 리셋 |
| `/motor_pos_ctrl` | PosVelCurCtrl.srv | DatcCommInterface | 위치 제어 |

---

## 10. 운용 절차 (`other01/사용법`)

1. `ui` 실행 (hanyang_eng_koras_system)
2. Configuration 탭에서 keycode 버튼 2개 OFF
3. 터미널 2개 추가로 열어서:
   - `keycode1` → `detect_red_dot_node_for_drum.py`
   - `keycode2` → `detect_red_dot_node_for_holder.py`

---

## 11. 알려진 이슈 (`other01/이슈 상황 및 사진/`)

| 날짜 | 이슈 |
|------|------|
| 0421 | Task1 놓을 때 불안정 / 그리퍼 팁 수정 필요 |
| 0923 | Task3 엘보 피봇 정렬 불량 의심 |
| 0930 | Task4 충돌 → mainwindow_widgets_hanyang_eng.cpp 3440번째줄 3.920→2.520으로 수정 |
| 1021 | Task1 비전 인식 실패 (가장 최근, 미해결) |

---

## 12. 중요 파일 경로 빠른 참조

```
# 핵심 로직 파일
hanyang_ui_ws/src/robot_control_config/ui_config/task_planner_hanyang_eng.cpp  (6722줄)
hanyang_ui_ws/src/koras_system/src/qnode.cpp                                    (~11000줄)
hanyang_ui_ws/src/koras_system/include/task_planner_default.hpp                 (UnitTask 구조체)
hanyang_ui_ws/src/koras_system/include/task_parameter.hpp                       (Task enum, 754줄)
hanyang_ui_ws/src/koras_system/include/task/taskManager.hpp                     (Task 실행 UI)
hanyang_ui_ws/src/koras_system/src/bin_picking/bin_picking_node.cpp             (스캔/매칭 브릿지)

# 그리퍼 드라이버
gripper_ws/src/koras_gripper/KR_GCS_user_interface_ROS2/kr_gcs_ui/
    include/datc_ctrl.hpp           (DATC 명령 enum, 제어 함수)
    include/modbus_comm.hpp         (Modbus RTU 통신)
    include/datc_comm_interface.hpp (ROS2 서비스 서버)
    src/datc_ctrl.cpp               (실제 제어 구현)
    src/datc_comm_interface.cpp     (ROS2 인터페이스, duration 버그 위치)

# 매칭
hanyang_matching_ws/src/hanyang_matching_open3d/nodes/matching_node_full.py
hanyang_matching_ws/src/hanyang_matching_open3d/config/matching_params.yaml
hanyang_matching_ws/src/hanyang_zivid_scanner_node/src/zivid_node.py
hanyang_matching_ws/src/hanyang_sam_node/nodes/sam_node.py

# 키코드
keycode_ws/src/keyring_detection/keyring_detection/detect_red_dot_node_for_drum.py
keycode_ws/src/keyring_detection/keyring_detection/detect_red_dot_node_for_holder.py

# 두산 드라이버
doosan_ros2_ws/src/doosan-robot2/dsr_msgs2/srv/motion/MoveJoint.srv
doosan_ros2_ws/src/doosan-robot2/dsr_msgs2/srv/motion/MoveLine.srv
doosan_ros2_ws/src/doosan-robot2/dsr_hardware2/src/dsr_hw_interface2.cpp

# 메시지 정의
msg_ws/src/grp_control_msg/srv/PosVelCurCtrl.srv
msg_ws/src/grp_control_msg/msg/GripperMsg.msg
msg_ws/src/hanyang_matching_msgs/srv/DoTemplateMatching.srv
msg_ws/src/hanyang_matching_msgs/srv/ZividDoScan.srv
msg_ws/src/kcr_control_msg/msg/RobotState.msg
```

---

## 13. 개발 참고

### 빌드 명령 (bashrc alias)
```bash
alias cb='colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release'
alias cb_clean='sudo rm -r install log build'
```

### 실행 alias
```bash
alias ui='ros2 run hanyang_eng_koras_system kr_sys HANYANG_LAUNCH'
alias zvd='ros2 run hanyang_zivid_scanner_node zivid_topic_node.py'
alias sc='ros2 run hanyang_matching_process zivid_scan_node'
alias mc='ros2 run hanyang_matching_process matching_node'
alias sam='ros2 run hanyang_sam_node sam_node.py'
alias vis='ros2 launch hanyang_matching_visualizer visualize_zvd_cad_grasp.xml'
alias grip='sudo chmod a+rw /dev/ttyUSB0 && ros2 launch grp_control slave1.xml'
alias dscontroller='ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=192.168.100.108 port:=12345 model:=m1013 gui:=false'
alias vcontroller='ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=virtual host:=127.0.0.1 port:=12345 model:=m1013'
alias plot='ros2 run plotjuggler plotjuggler'
```

### CUDA (GPU 매칭용)
```
CUDA 12.6: /usr/local/cuda-12.6
```
