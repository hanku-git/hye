"""
드럼통 CAD 포인트클라우드 로드 스크립트 (Isaac Sim Script Editor)

실제 Zivid 스캔으로 만든 PLY CAD 파일을 Isaac Sim 씬에 배치.
비전 알고리즘(keyring 검출, 매칭 노드) 테스트용 가상 씬 구성.

실행 순서:
  1. Isaac Sim Play(▶)
  2. setup_workspace.py 실행 (드럼통 몸체)
  3. 이 스크립트 실행 (드럼 뚜껑 CAD 추가)
  4. File → Save

카메라 렌더링:
  - setup_camera.py 실행 후 카메라를 드럼 위로 이동
  - ros2 topic echo /zivid/color/image_color 로 확인
"""

import struct
import numpy as np
import omni.usd
from pxr import UsdGeom, UsdShade, Vt, Gf, Sdf

stage = omni.usd.get_context().get_stage()

# ── 설정 ────────────────────────────────────────────────────────
CAD_BASE = "/home/hanku/[scanDataHanyang]/CAD/[bin picking]/target_object"

# 드럼통 위치 (setup_workspace.py DRUM_X, DRUM_Y, DRUM_HEIGHT 와 동일)
DRUM_X      = 0.40    # m
DRUM_Y      = 0.30    # m
DRUM_TOP_Z  = 0.440   # m  (드럼통 상면)

# PLY z_min (mm→m): PLY 바닥면이 드럼통 상면에 오도록 Z 오프셋 계산
# drum_lid_total z_min = -26.5mm = -0.0265m
PLY_Z_MIN_M = -0.0265
WORLD_Z     = DRUM_TOP_Z - PLY_Z_MIN_M   # 0.4665 m

# 포인트 크기 (렌더링용)
POINT_SIZE  = 0.0025   # m (2.5mm)

# 다운샘플 스트라이드: 포인트 수 줄여 Isaac Sim 성능 확보
# 1 = 전체 (느림), 4 = 1/4 (권장), 8 = 1/8 (빠름)
DOWNSAMPLE_STRIDE = 4


# ── PLY 읽기 ────────────────────────────────────────────────────
def read_ply(path, stride=1):
    """
    PLY 바이너리 읽기
    포맷: float x,y,z, float nx,ny,nz, uchar r,g,b,a  (28 bytes/vertex)
    반환: xyz_m (N,3) float32, rgb_01 (N,3) float32
    """
    with open(path, 'rb') as f:
        n_verts = 0
        while True:
            line = f.readline().decode('utf-8', errors='ignore').strip()
            if line.startswith('element vertex'):
                n_verts = int(line.split()[-1])
            if line == 'end_header':
                break

        stride_bytes = 28
        raw = np.frombuffer(
            f.read(n_verts * stride_bytes), dtype=np.uint8
        ).reshape(n_verts, stride_bytes)

    # 다운샘플
    raw = raw[::stride]

    # XYZ (mm → m)
    xyz_mm = raw[:, :12].view(np.float32).reshape(-1, 3)[:, :3]
    xyz_m  = xyz_mm * 0.001

    # RGB (0~255 → 0.0~1.0)
    rgb = raw[:, 24:27].astype(np.float32) / 255.0

    return xyz_m, rgb


# ── Isaac Sim 포인트클라우드 Prim 생성 ───────────────────────────
def add_pointcloud(prim_path, xyz_m, rgb):
    """
    UsdGeom.Points로 포인트클라우드 생성
    xyz_m: PLY 로컬 좌표 (m), 월드 오프셋은 여기서 적용
    """
    if stage.GetPrimAtPath(prim_path):
        stage.RemovePrim(prim_path)

    pts_prim = UsdGeom.Points.Define(stage, prim_path)

    # 월드 좌표 = PLY 로컬(m) + 드럼 위치 오프셋
    ox, oy, oz = DRUM_X, DRUM_Y, WORLD_Z
    world_pts = Vt.Vec3fArray([
        Gf.Vec3f(float(p[0] + ox), float(p[1] + oy), float(p[2] + oz))
        for p in xyz_m
    ])
    pts_prim.GetPointsAttr().Set(world_pts)
    pts_prim.GetWidthsAttr().Set(Vt.FloatArray([POINT_SIZE] * len(world_pts)))

    # 꼭짓점 색상
    colors = Vt.Vec3fArray([
        Gf.Vec3f(float(c[0]), float(c[1]), float(c[2]))
        for c in rgb
    ])
    pts_prim.GetDisplayColorAttr().Set(colors)

    # displayOpacity 인터폴레이션 설정
    pts_prim.GetPrim().CreateAttribute(
        "primvars:displayColor:interpolation",
        Sdf.ValueTypeNames.Token
    ).Set("vertex")

    n = len(world_pts)
    print(f"[OK] {prim_path}: {n:,} points  "
          f"center=({ox*1000:.0f},{oy*1000:.0f},{oz*1000:.0f})mm")
    return pts_prim


# ── 기존 DrumModel 제거 ──────────────────────────────────────────
if stage.GetPrimAtPath("/World/DrumModel"):
    stage.RemovePrim("/World/DrumModel")
    print("[CLEAN] /World/DrumModel 제거")

UsdGeom.Xform.Define(stage, "/World/DrumModel")


# ── 드럼 뚜껑 전체 (메인) ───────────────────────────────────────
print(f"\n[1/3] drum_lid_total 로딩 (stride={DOWNSAMPLE_STRIDE})...")
xyz, rgb = read_ply(
    f"{CAD_BASE}/drum_lid_total_CAD.ply",
    stride=DOWNSAMPLE_STRIDE
)
add_pointcloud("/World/DrumModel/LidTotal", xyz, rgb)


# ── 키코드 캡 (작은 흰 캡) ──────────────────────────────────────
print(f"\n[2/3] key_code 로딩...")
xyz, rgb = read_ply(
    f"{CAD_BASE}/right_drum_key_code_CAD.ply",
    stride=DOWNSAMPLE_STRIDE
)
add_pointcloud("/World/DrumModel/KeyCode", xyz, rgb)


# ── 큰 뚜껑 캡 (keyring 있는 흰 캡) ────────────────────────────
print(f"\n[3/3] lid_cap_screwing 로딩...")
xyz, rgb = read_ply(
    f"{CAD_BASE}/right_drum_lid_cap_screwing_CAD.ply",
    stride=DOWNSAMPLE_STRIDE
)
add_pointcloud("/World/DrumModel/LidCap", xyz, rgb)


# ── 완료 ────────────────────────────────────────────────────────
print("\n=== 드럼 CAD 로드 완료 ===")
print(f"드럼 위치  : X={DRUM_X*1000:.0f}mm  Y={DRUM_Y*1000:.0f}mm  상면 Z={DRUM_TOP_Z*1000:.0f}mm")
print(f"모델 Z오프셋: {WORLD_Z*1000:.1f}mm  (PLY z_min={PLY_Z_MIN_M*1000:.1f}mm 보정)")
print(f"다운샘플   : 1/{DOWNSAMPLE_STRIDE}")
print("")
print("다음 단계:")
print("  1. 뷰포트에서 드럼 뚜껑 확인 (/World/DrumModel)")
print("  2. 위치가 맞지 않으면 DRUM_X/DRUM_Y/PLY_Z_MIN_M 수정 후 재실행")
print("  3. File → Save")
print("  4. setup_camera.py 실행 → 카메라를 드럼 위로 이동")
