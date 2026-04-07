"""
드럼통 OBJ 메쉬 로드 스크립트 (Isaac Sim Script Editor)

convert_ply_to_mesh.py 로 생성된 OBJ 파일을 Isaac Sim에 메쉬로 로드.
메쉬 표면이 있어서 depth 카메라(/zivid/points/xyzrgba) 동작.

실행 순서:
  1. setup_workspace.py 실행 (드럼 몸체)
  2. 이 스크립트 실행 (드럼 뚜껑 메쉬)
  3. setup_camera.py 실행
"""

import asyncio
import omni.usd
import omni.kit.asset_converter as converter
from pxr import UsdGeom, UsdShade, UsdPhysics, Gf, Sdf

stage = omni.usd.get_context().get_stage()

# ── 설정 ─────────────────────────────────────────────────────────
OBJ_DIR     = "/home/hanku/hye/isaac_bridge_ws/isaac_sim/drum_cad"
USD_DIR     = "/home/hanku/hye/isaac_bridge_ws/isaac_sim/drum_cad"

# 드럼 위치 (setup_workspace.py 와 동일)
DRUM_X     = 0.40    # m
DRUM_Y     = 0.30    # m
DRUM_TOP_Z = 0.440   # m

# PLY z_min 오프셋 (mm→m, drum_lid_total z_min=-26.5mm)
PLY_Z_MIN_M = -0.0265
WORLD_Z     = DRUM_TOP_Z - PLY_Z_MIN_M   # 0.4665 m

# OBJ 파일 목록: (obj파일, usd파일, 색상RGB, prim이름)
# right_drum_key_code 와 right_drum_lid_cap_screwing 은 동일한 PLY → drum_lid_total 에 이미 포함
MESHES = [
    ("drum_lid_total.obj", "drum_lid_total.usd",
     (0.08, 0.35, 0.78), "LidTotal"),   # 뚜껑 전체 (캡 포함)
]


def make_material(mat_path, color_rgb):
    if stage.GetPrimAtPath(mat_path):
        stage.RemovePrim(mat_path)
    mat = UsdShade.Material.Define(stage, mat_path)
    sh  = UsdShade.Shader.Define(stage, mat_path + "/Shader")
    sh.CreateIdAttr("UsdPreviewSurface")
    r, g, b = color_rgb
    sh.CreateInput("diffuseColor",  Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(r, g, b))
    sh.CreateInput("roughness",     Sdf.ValueTypeNames.Float).Set(0.4)
    sh.CreateInput("metallic",      Sdf.ValueTypeNames.Float).Set(0.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


async def convert_and_load(obj_path, usd_path, color_rgb, prim_name):
    """OBJ → USD 변환 후 스테이지에 배치"""
    print(f"\n[{prim_name}] OBJ → USD 변환 중...")

    # OBJ → USD 변환
    ctx = converter.AssetConverterContext()
    ctx.ignore_materials = True       # OBJ 재질 무시 (USD에서 재정의)
    ctx.ignore_camera    = True
    ctx.ignore_light     = True

    task = converter.get_instance().create_converter_task(
        obj_path, usd_path, None, ctx
    )
    success = await task.wait_until_finished()
    if not success:
        print(f"  [ERROR] 변환 실패: {task.get_error_message()}")
        return

    print(f"  변환 완료 → {usd_path}")

    # 스테이지에 Reference로 추가
    prim_path = f"/World/DrumMesh/{prim_name}"
    if stage.GetPrimAtPath(prim_path):
        stage.RemovePrim(prim_path)

    xform = UsdGeom.Xform.Define(stage, prim_path)
    xform.GetPrim().GetReferences().AddReference(usd_path)

    # 위치 및 스케일 설정
    # OBJ는 이미 m 단위 (convert_ply_to_mesh.py에서 *0.001 적용)
    # XY: PLY 로컬 좌표에 드럼 위치 더함
    xformable = UsdGeom.Xformable(xform)
    xformable.ClearXformOpOrder()
    xformable.AddTranslateOp().Set(Gf.Vec3d(DRUM_X, DRUM_Y, WORLD_Z))

    # 재질 적용
    mat = make_material(f"/World/DrumMeshMats/{prim_name}Mat", color_rgb)
    UsdShade.MaterialBindingAPI(xform).Bind(mat)

    print(f"  배치: {prim_path}  pos=({DRUM_X},{DRUM_Y},{WORLD_Z:.3f})m")


async def main():
    # 기존 DrumModel(포인트클라우드) 제거, DrumMesh 준비
    for old in ["/World/DrumModel", "/World/DrumMesh", "/World/DrumMeshMats"]:
        if stage.GetPrimAtPath(old):
            stage.RemovePrim(old)
            print(f"[CLEAN] {old} 제거")

    UsdGeom.Xform.Define(stage, "/World/DrumMesh")
    UsdGeom.Xform.Define(stage, "/World/DrumMeshMats")

    for obj_name, usd_name, color, prim_name in MESHES:
        obj_path = f"{OBJ_DIR}/{obj_name}"
        usd_path = f"{USD_DIR}/{usd_name}"
        await convert_and_load(obj_path, usd_path, color, prim_name)

    print("\n=== 드럼 메쉬 로드 완료 ===")
    print("depth 카메라 확인: ros2 topic echo /zivid/points/xyzrgba --once")
    print("저장: File → Save")


asyncio.ensure_future(main())
