#!/bin/bash
# ============================================================
# HYE Bin-Picking System - Field Startup Script
# Desktop: i7-13700 / RTX 4060 / Ubuntu 22.04 / ROS2 Humble
# ============================================================
#
# ┌──────────────────────────────────────────────────────────────────┐
# │                  설치 환경에 따라 수정하는 변수                    │
# ├─────────────────────┬────────────────────────────────────────────┤
# │ 항목                │ 노트북              │ 데스크탑 (필드)        │
# ├─────────────────────┼────────────────────┼────────────────────────┤
# │ WS_ROOT             │ $HOME/ws/hye_ws/.. │ $HOME                  │
# │ ENABLE_DRCF         │ false              │ true                   │
# │ ENABLE_VCONTROLLER  │ false              │ true                   │
# │ VCONTROLLER_MODE    │ virtual            │ real                   │
# │ NVIDIA PRIME        │ 활성화             │ 주석처리               │
# └─────────────────────┴────────────────────┴────────────────────────┘

# ── [1] 환경 정의 (여기만 수정) ──────────────────────────────

WS_ROOT="$HOME/ws/hye_ws/main01"   # 데스크탑: $HOME / 노트북: $HOME/ws/hye_ws/main01

ENABLE_DRCF=false          # DRCF Docker 실행 여부     (노트북: false / 데스크탑: true)
ENABLE_VCONTROLLER=false   # vcontroller 실행 여부     (노트북: false / 데스크탑: true)

ROS_DISTRO="humble"        # ROS2 배포판
ROBOT_MODEL="m1013"        # Doosan 로봇 모델
DRCF_PORT=12345            # DRCF 포트
DRCF_READY_TIMEOUT=30      # DRCF 포트 대기 최대 시간 (초)
VCONTROLLER_WAIT=8         # vcontroller 노드 안정화 대기 (초)
VCONTROLLER_MODE="virtual" # real(필드) / virtual(노트북)

# NVIDIA PRIME (노트북 AMD+NVIDIA 구조일 때만 활성화)
# 데스크탑 전용 GPU이면 아래 두 줄 주석처리
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia

# ── [2] 경로 자동 생성 (수정 불필요) ────────────────────────

DOOSAN_WS="$WS_ROOT/doosan_ros2_ws"
UI_WS="$WS_ROOT/hanyang_ui_ws"
MATCHING_WS="$WS_ROOT/hanyang_matching_ws"
DRCF_SCRIPT="$DOOSAN_WS/install/common2/share/common2/bin/run_drcf.sh"
LOGFILE="/tmp/hye_startup.log"

# ── [3] 공통 설정 ────────────────────────────────────────────

# DISPLAY 자동 감지 (세션 상속값 우선, 없으면 :0 fallback)
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
export ROS_DOMAIN_ID=0

log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $1" | tee -a "$LOGFILE"
}

# GNOME Shell D-Bus 등록 대기 (자동로그인 시 세션 초기화 완료 감지)
log "Waiting for GNOME Shell to be ready..."
until gdbus call --session \
        --dest org.gnome.Shell \
        --object-path /org/gnome/Shell \
        --method org.freedesktop.DBus.Peer.Ping \
        >/dev/null 2>&1; do
    sleep 1
done
log "GNOME Shell ready."

# ── [4] ROS2 환경 로드 ───────────────────────────────────────

source /opt/ros/$ROS_DISTRO/setup.bash
if $ENABLE_VCONTROLLER; then
    source "$DOOSAN_WS/install/setup.bash"
fi
source "$UI_WS/install/setup.bash"
source "$MATCHING_WS/install/setup.bash"

# ── [5] 실행 ─────────────────────────────────────────────────

log "=========================================="
log " HYE System Starting"
log " WS_ROOT          : $WS_ROOT"
log " USER / HOME      : $USER / $HOME"
log " ENABLE_DRCF      : $ENABLE_DRCF"
log " ENABLE_VCONTROLLER: $ENABLE_VCONTROLLER"
log "=========================================="

# STEP 1: DRCF Docker
if $ENABLE_DRCF; then
    log "[1/3] Starting DRCF Docker..."
    if docker ps | grep -q "drcf"; then
        log "  DRCF already running, skipping."
    else
        bash "$DRCF_SCRIPT" $DRCF_PORT $ROBOT_MODEL
        log "  DRCF Docker started."
    fi

    log "  Waiting for DRCF port $DRCF_PORT..."
    for i in $(seq 1 $DRCF_READY_TIMEOUT); do
        if nc -z 127.0.0.1 $DRCF_PORT 2>/dev/null; then
            log "  DRCF ready! (${i}s)"
            break
        fi
        if [ "$i" -eq "$DRCF_READY_TIMEOUT" ]; then
            log "  WARNING: DRCF not ready after ${DRCF_READY_TIMEOUT}s, continuing..."
        fi
        sleep 1
    done
else
    log "[1/3] DRCF Docker SKIPPED (ENABLE_DRCF=false)"
fi

# STEP 2: vcontroller
VCONTROLLER_PID=""
if $ENABLE_VCONTROLLER; then
    log "[2/3] Starting vcontroller (mode=$VCONTROLLER_MODE)..."
    ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
        mode:=$VCONTROLLER_MODE \
        host:=127.0.0.1 \
        port:=$DRCF_PORT \
        model:=$ROBOT_MODEL &
    VCONTROLLER_PID=$!
    log "  vcontroller PID: $VCONTROLLER_PID"
    log "  Waiting ${VCONTROLLER_WAIT}s for nodes to stabilize..."
    sleep $VCONTROLLER_WAIT

    if ros2 service list 2>/dev/null | grep -q "get_current_pose"; then
        log "  Robot controller ready!"
    else
        log "  WARNING: Robot controller service not found, continuing..."
    fi
else
    log "[2/3] vcontroller SKIPPED (ENABLE_VCONTROLLER=false)"
fi

# STEP 3: UI
log "[3/3] Starting HYE UI..."
ros2 run hanyang_eng_koras_system kr_sys HANYANG_LAUNCH &
UI_PID=$!
log "  UI PID: $UI_PID"

log "=========================================="
log " HYE System Started"
log " DRCF port      : $DRCF_PORT"
log " vcontroller    : ${VCONTROLLER_PID:-skipped}"
log " UI             : PID $UI_PID"
log "=========================================="

# UI 종료 시 전체 정리
wait $UI_PID
log "UI exited. Shutting down..."
if [ -n "$VCONTROLLER_PID" ]; then
    kill $VCONTROLLER_PID 2>/dev/null || true
fi
log "Shutdown complete."
