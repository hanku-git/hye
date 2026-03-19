#!/bin/bash
# ============================================================
# HYE Bin-Picking System - Autostart Install Script (XDG)
# 대상: 데스크탑 필드 배포 / Ubuntu 22.04 / GDM3 자동로그인
# ============================================================
#
# ┌─────────────────────────────────────────────────────────┐
# │               설치 환경에 따라 수정하는 변수              │
# ├──────────────────┬──────────────────────────────────────┤
# │ 노트북           │ WS_ROOT="$HOME/ws/hye_ws/main01"     │
# │ 데스크탑 (필드)  │ WS_ROOT="$HOME"                      │
# └──────────────────┴──────────────────────────────────────┘

# ── [1] 환경 정의 (여기만 수정) ──────────────────────────────

WS_ROOT="$HOME/ws/hye_ws/main01"                  # 데스크탑: $HOME / 노트북: $HOME/ws/hye_ws/main01

# ── [2] 경로 자동 생성 (수정 불필요) ────────────────────────

DEPLOY_DIR="$WS_ROOT/deploy"
AUTOSTART_DIR="$HOME/.config/autostart"
STARTUP_SCRIPT="$DEPLOY_DIR/hye_startup.sh"

set -e

echo "=== HYE Autostart Install (XDG) ==="
echo "  WS_ROOT : $WS_ROOT"
echo "  SCRIPT  : $STARTUP_SCRIPT"

# 실행 권한
chmod +x "$STARTUP_SCRIPT"

# autostart 디렉토리 생성
mkdir -p "$AUTOSTART_DIR"

# .desktop 파일 생성 (경로는 변수로 삽입)
# GNOME Shell D-Bus 준비 감지는 hye_startup.sh 내부에서 처리
cat > "$AUTOSTART_DIR/hye.desktop" << EOF
[Desktop Entry]
Type=Application
Name=HYE Bin-Picking System
Comment=HYE Field Auto-Start
Exec=$STARTUP_SCRIPT
Terminal=false
X-GNOME-Autostart-enabled=true
EOF

echo ""
echo "✓ 설치 완료: $AUTOSTART_DIR/hye.desktop"
echo ""
echo "명령어:"
echo "  비활성화: rm $AUTOSTART_DIR/hye.desktop"
echo "  로그:     tail -f /tmp/hye_startup.log"
echo "  수동실행: $STARTUP_SCRIPT"
