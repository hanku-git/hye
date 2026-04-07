"""
Isaac Sim Eye-in-Hand 카메라 설정 스크립트
- /World/m1013/link_6 에 Zivid 카메라 추가
- OmniGraph에 RGB + PointCloud ROS2 퍼블리셔 추가
- 토픽: /zivid/color/image_color, /zivid/points/xyzrgba
"""

import omni.usd
import omni.graph.core as og
from pxr import UsdGeom, Gf

stage = omni.usd.get_context().get_stage()

# ── 1. 카메라 Prim 생성 (link_6 자식) ──────────────────────────────
CAMERA_PATH = "/World/m1013/link_6/zivid_camera"

if stage.GetPrimAtPath(CAMERA_PATH):
    stage.RemovePrim(CAMERA_PATH)
    print(f"[INFO] 기존 카메라 삭제: {CAMERA_PATH}")

camera = UsdGeom.Camera.Define(stage, CAMERA_PATH)
xf = UsdGeom.Xformable(camera)
xf.ClearXformOpOrder()
xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.1))
xf.AddRotateXYZOp().Set(Gf.Vec3f(180.0, 0.0, 0.0))

camera.GetFocalLengthAttr().Set(17.0)
camera.GetHorizontalApertureAttr().Set(20.955)
camera.GetVerticalApertureAttr().Set(15.2908)
camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 5.0))
print(f"[OK] Camera created: {CAMERA_PATH}")

# ── 2. 기존 카메라 퍼블리셔 노드 제거 ─────────────────────────────
GRAPH_PATH = "/World/ROS2Graph"
for node_name in ["RGBPublisher", "DepthPublisher"]:
    node_path = f"{GRAPH_PATH}/{node_name}"
    if stage.GetPrimAtPath(node_path):
        stage.RemovePrim(node_path)
        print(f"[INFO] 기존 노드 삭제: {node_path}")

# ── 3. 카메라 전용 RenderProduct 생성 ─────────────────────────────
import omni.replicator.core as rep

RP_PATH = "/World/RenderProducts/zivid_rp"
if stage.GetPrimAtPath(RP_PATH):
    stage.RemovePrim(RP_PATH)

# 카메라 경로를 명시해서 해당 카메라 전용 렌더 프로덕트 생성
rp = rep.create.render_product(CAMERA_PATH, (1280, 960), name="zivid_rp")
rp_path = rp.path
print(f"[OK] RenderProduct: {rp_path}")

# ── 4. OmniGraph에 ROS2 카메라 퍼블리셔 추가 ──────────────────────
try:
    keys = og.Controller.Keys
    og.Controller.edit(
        GRAPH_PATH,
        {
            keys.CREATE_NODES: [
                ("RGBPublisher",   "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("DepthPublisher", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            keys.SET_VALUES: [
                ("RGBPublisher.inputs:topicName",           "/zivid/color/image_color"),
                ("RGBPublisher.inputs:type",                "rgb"),
                ("RGBPublisher.inputs:frameId",             "zivid_camera"),
                ("RGBPublisher.inputs:renderProductPath",   rp_path),
                ("DepthPublisher.inputs:topicName",         "/zivid/points/xyzrgba"),
                ("DepthPublisher.inputs:type",              "depth_pcl"),
                ("DepthPublisher.inputs:frameId",           "zivid_camera"),
                ("DepthPublisher.inputs:renderProductPath", rp_path),
            ],
        }
    )

    # 기존 노드 연결 시 전체 경로 필요 (Isaac Sim 5.0)
    og.Controller.connect(
        f"{GRAPH_PATH}/OnPlaybackTick.outputs:tick",
        f"{GRAPH_PATH}/RGBPublisher.inputs:execIn"
    )
    og.Controller.connect(
        f"{GRAPH_PATH}/OnPlaybackTick.outputs:tick",
        f"{GRAPH_PATH}/DepthPublisher.inputs:execIn"
    )
    print("[OK] ROS2 camera publishers added")

except Exception as e:
    print(f"[ERROR] OmniGraph edit failed: {e}")

print("")
print("=== 설정 완료 ===")
print(f"Camera        : {CAMERA_PATH}")
print(f"RenderProduct : {rp_path}")
print("Topics        : /zivid/color/image_color")
print("               /zivid/points/xyzrgba")
print("")
print("확인: ros2 topic list | grep zivid")
print("저장: File → Save")

