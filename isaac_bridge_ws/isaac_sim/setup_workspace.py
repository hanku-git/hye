"""
Isaac Sim 물리 작업공간 설정 스크립트 (1회 실행 후 USD Save)

실제 작업공간 구조:
  - 바닥    : Z=0,    물리 충돌체 포함
  - 드럼통  : X=700mm, 지름=585mm, 높이=440mm
  - 로봇 베이스 : 원점 (0, 0, 0)

실행: Isaac Sim Script Editor → Run
저장: File → Save (이후 세션부터 자동 로드)
"""

import omni.usd
from pxr import (
    UsdGeom, UsdShade, UsdPhysics, PhysxSchema,
    Gf, Sdf
)

stage = omni.usd.get_context().get_stage()


# ──────────────────────────────────────────────────────────────
# 유틸
# ──────────────────────────────────────────────────────────────

def make_material(path, color_rgb, roughness=0.6):
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
    mat = UsdShade.Material.Define(stage, path)
    sh  = UsdShade.Shader.Define(stage, path + "/Sh")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color_rgb))
    sh.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(roughness)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def make_box_mesh(prim_path, cx, cy, cz, sx, sy, sz):
    """중심 (cx,cy,cz), 크기 (sx,sy,sz) 의 박스 Mesh 생성 (단위: m)"""
    if stage.GetPrimAtPath(prim_path):
        stage.RemovePrim(prim_path)
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    pts = [
        Gf.Vec3f(cx-hx, cy-hy, cz-hz), Gf.Vec3f(cx+hx, cy-hy, cz-hz),
        Gf.Vec3f(cx+hx, cy+hy, cz-hz), Gf.Vec3f(cx-hx, cy+hy, cz-hz),
        Gf.Vec3f(cx-hx, cy-hy, cz+hz), Gf.Vec3f(cx+hx, cy-hy, cz+hz),
        Gf.Vec3f(cx+hx, cy+hy, cz+hz), Gf.Vec3f(cx-hx, cy+hy, cz+hz),
    ]
    faces = [4] * 6
    idxs  = [
        0,1,2,3,
        4,5,6,7,
        0,1,5,4,
        2,3,7,6,
        1,2,6,5,
        0,3,7,4,
    ]
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    mesh.GetPointsAttr().Set(pts)
    mesh.GetFaceVertexCountsAttr().Set(faces)
    mesh.GetFaceVertexIndicesAttr().Set(idxs)
    return mesh


def add_collider(prim_path):
    """UsdPhysics Collision API 적용 (정적 충돌체)"""
    prim = stage.GetPrimAtPath(prim_path)
    if not prim:
        return
    UsdPhysics.CollisionAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.RigidBodyAPI(prim).CreateKinematicEnabledAttr(True)
    PhysxSchema.PhysxCollisionAPI.Apply(prim)


# ──────────────────────────────────────────────────────────────
# 0. 이전 오브젝트 제거 (USD에 남아있는 경우)
# ──────────────────────────────────────────────────────────────
for path in ["/World/Table", "/World/Bin", "/World/Floor",
             "/World/FloorMats", "/World/Tiles", "/World/Drum",
             "/World/Ceiling", "/World/Mats"]:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
        print(f"[CLEAN] {path} 제거")

# ──────────────────────────────────────────────────────────────
# 1. Physics Scene
# ──────────────────────────────────────────────────────────────
PHYSICS_PATH = "/World/PhysicsScene"
if not stage.GetPrimAtPath(PHYSICS_PATH):
    physics_scene = UsdPhysics.Scene.Define(stage, PHYSICS_PATH)
    physics_scene.CreateGravityDirectionAttr(Gf.Vec3f(0.0, 0.0, -1.0))
    physics_scene.CreateGravityMagnitudeAttr(9.81)
    print("[OK] PhysicsScene 생성")
else:
    print("[SKIP] PhysicsScene 이미 존재")


# ──────────────────────────────────────────────────────────────
# 2. 바닥
# ──────────────────────────────────────────────────────────────
for path in ["/World/Floor", "/World/FloorMats"]:
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)

mat_floor = make_material("/World/Mats/Floor", (0.55, 0.55, 0.55), roughness=0.9)
floor_mesh = make_box_mesh("/World/Floor", 0.0, 0.0, -0.010, 5.0, 5.0, 0.020)
UsdShade.MaterialBindingAPI(floor_mesh).Bind(mat_floor)
add_collider("/World/Floor")
print("[OK] 바닥 생성 + 충돌체 (Z=0)")

# 체커보드 타일 (시각용)
TILE_N, TILE_SIZE = 10, 0.5
mat_w = make_material("/World/Mats/TileW", (0.90, 0.90, 0.90), roughness=0.9)
mat_g = make_material("/World/Mats/TileG", (0.35, 0.35, 0.35), roughness=0.9)
origin = -(TILE_N * TILE_SIZE) / 2 + TILE_SIZE / 2
for row in range(TILE_N):
    for col in range(TILE_N):
        x = origin + col * TILE_SIZE
        y = origin + row * TILE_SIZE
        mat = mat_w if (row + col) % 2 == 0 else mat_g
        tile = make_box_mesh(
            f"/World/Tiles/t_{row}_{col}",
            x, y, 0.001, TILE_SIZE, TILE_SIZE, 0.002
        )
        UsdShade.MaterialBindingAPI(tile).Bind(mat)
print(f"[OK] 체커보드 타일 {TILE_N}×{TILE_N} (시각용)")


# ──────────────────────────────────────────────────────────────
# 3. 드럼통 (200L 표준)
#    지름: 585mm (반지름 292.5mm)
#    높이: 880mm
#    위치: 로봇 앞 X=+700mm
# ──────────────────────────────────────────────────────────────
DRUM_X      = 0.40       # 로봇에서 700mm 앞
DRUM_Y      = 0.30
DRUM_RADIUS = 0.2925     # 585mm / 2
DRUM_HEIGHT = 0.440      # 440mm (880mm 반)
DRUM_TOP_Z  = DRUM_HEIGHT  # 드럼통 상면 Z

if stage.GetPrimAtPath("/World/Drum"):
    stage.RemovePrim("/World/Drum")

drum = UsdGeom.Cylinder.Define(stage, "/World/Drum")
drum.GetRadiusAttr().Set(DRUM_RADIUS)
drum.GetHeightAttr().Set(DRUM_HEIGHT)
# Isaac Sim Cylinder 기본 축: Y축. Z축으로 세우기 위해 axis 설정
drum.GetAxisAttr().Set("Z")

# 위치: 중심이 드럼통 높이 절반
xf = UsdGeom.Xformable(drum)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(DRUM_X, DRUM_Y, DRUM_HEIGHT / 2))

mat_drum = make_material("/World/Mats/Drum", (0.15, 0.25, 0.50), roughness=0.3)
UsdShade.MaterialBindingAPI(drum).Bind(mat_drum)
add_collider("/World/Drum")

print(f"[OK] 드럼통 생성 (지름={DRUM_RADIUS*2*1000:.0f}mm, 높이={DRUM_HEIGHT*1000:.0f}mm, 상면 Z={DRUM_TOP_Z*1000:.0f}mm)")


# ──────────────────────────────────────────────────────────────
# 4. 천장 (시각용)
# ──────────────────────────────────────────────────────────────
if stage.GetPrimAtPath("/World/Ceiling"):
    stage.RemovePrim("/World/Ceiling")
ceiling = make_box_mesh("/World/Ceiling", 0.0, 0.0, 2.5, 5.0, 5.0, 0.02)
mat_ceil = make_material("/World/Mats/Ceiling", (0.45, 0.65, 0.90), roughness=1.0)
UsdShade.MaterialBindingAPI(ceiling).Bind(mat_ceil)
print("[OK] 천장 생성 (Z=2.5m)")


# ──────────────────────────────────────────────────────────────
print("")
print("=== 물리 작업공간 설정 완료 ===")
print(f"바닥   : Z=0m  (충돌체)")
print(f"드럼통 : X={DRUM_X*1000:.0f}mm  Ø{DRUM_RADIUS*2*1000:.0f}mm  H={DRUM_HEIGHT*1000:.0f}mm  상면 Z={DRUM_TOP_Z*1000:.0f}mm  (충돌체)")
print(f"")
print(f"bridge_node.py WORKSPACE_Z_MIN 권장값: {(DRUM_TOP_Z+0.05)*1000:.0f}mm ({DRUM_TOP_Z+0.05:.3f}m)")
print(f"저장: File → Save")
