"""
Isaac Sim 환경 설정: 바닥 체커보드 + 천장 색상 추가
- 바닥: 흰/회색 체커보드 (10x10 타일)
- 천장: 하늘색 단색
"""

import omni.usd
from pxr import UsdGeom, UsdShade, Gf, Sdf

stage = omni.usd.get_context().get_stage()

def make_material(prim_path, color_rgb):
    mat = UsdShade.Material.Define(stage, prim_path)
    sh  = UsdShade.Shader.Define(stage, prim_path + "/Sh")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color_rgb))
    sh.CreateInput("roughness",    Sdf.ValueTypeNames.Float).Set(0.9)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat

def make_tile(prim_path, x, y, size, z, mat):
    """단일 타일 생성"""
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    h = size / 2
    mesh.GetPointsAttr().Set([
        Gf.Vec3f(x-h, y-h, z),
        Gf.Vec3f(x+h, y-h, z),
        Gf.Vec3f(x+h, y+h, z),
        Gf.Vec3f(x-h, y+h, z),
    ])
    mesh.GetFaceVertexCountsAttr().Set([4])
    mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)

def make_quad(prim_path, size, z, color_rgb):
    """단색 사각 평면 생성"""
    mesh = UsdGeom.Mesh.Define(stage, prim_path)
    h = size / 2
    mesh.GetPointsAttr().Set([
        Gf.Vec3f(-h, -h, z),
        Gf.Vec3f( h, -h, z),
        Gf.Vec3f( h,  h, z),
        Gf.Vec3f(-h,  h, z),
    ])
    mesh.GetFaceVertexCountsAttr().Set([4])
    mesh.GetFaceVertexIndicesAttr().Set([0, 1, 2, 3])
    mat = make_material(prim_path + "/Mat", color_rgb)
    UsdShade.MaterialBindingAPI(mesh).Bind(mat)

# ── 바닥 체커보드 ─────────────────────────────────────────
TILE_N    = 10       # 한 방향 타일 수
TILE_SIZE = 0.5      # 타일 크기 (m)
FLOOR_Z   = -0.005   # 바닥 Z 위치

mat_white = make_material("/World/FloorMats/White", (0.90, 0.90, 0.90))
mat_gray  = make_material("/World/FloorMats/Gray",  (0.40, 0.40, 0.40))

origin = -(TILE_N * TILE_SIZE) / 2 + TILE_SIZE / 2

for row in range(TILE_N):
    for col in range(TILE_N):
        x = origin + col * TILE_SIZE
        y = origin + row * TILE_SIZE
        mat = mat_white if (row + col) % 2 == 0 else mat_gray
        path = f"/World/Floor/tile_{row}_{col}"
        make_tile(path, x, y, TILE_SIZE, FLOOR_Z, mat)

print("[OK] 바닥 체커보드 생성 완료")

# ── 천장 (하늘색) ──────────────────────────────────────────
CEILING_Z = 2.5

make_quad("/World/Ceiling", TILE_N * TILE_SIZE, CEILING_Z, (0.45, 0.65, 0.90))
print("[OK] 천장 생성 완료")

print("")
print("=== 환경 설정 완료 ===")
print(f"바닥: 체커보드 {TILE_N}x{TILE_N} 타일 (Z={FLOOR_Z}m)")
print(f"천장: 하늘색   (Z={CEILING_Z}m)")
print("")
print("저장: File → Save")
