"""
PLY 포인트클라우드 → OBJ 메쉬 변환 (Isaac Sim 외부에서 실행)

사용법:
    python3 convert_ply_to_mesh.py

출력:
    isaac_bridge_ws/isaac_sim/drum_cad/drum_lid_total.obj
    isaac_bridge_ws/isaac_sim/drum_cad/right_drum_key_code.obj
    isaac_bridge_ws/isaac_sim/drum_cad/right_drum_lid_cap_screwing.obj

변환 방법: Poisson Surface Reconstruction (법선벡터 포함 PLY 활용)
"""

import open3d as o3d
import numpy as np
import os

CAD_BASE = "/home/hanku/[scanDataHanyang]/CAD/[bin picking]/target_object"
OUT_DIR  = os.path.dirname(os.path.abspath(__file__)) + "/drum_cad"
os.makedirs(OUT_DIR, exist_ok=True)

FILES = [
    ("drum_lid_total_CAD.ply",              "drum_lid_total.obj"),
    ("right_drum_key_code_CAD.ply",         "right_drum_key_code.obj"),
    ("right_drum_lid_cap_screwing_CAD.ply", "right_drum_lid_cap_screwing.obj"),
]


def ply_to_mesh(src_ply, dst_obj, voxel_size=0.005):
    """
    PLY 포인트클라우드 → OBJ 메쉬
    voxel_size: 다운샘플 복셀 크기 (m), 클수록 빠름/거침
    """
    print(f"\n[변환] {os.path.basename(src_ply)}")

    # 1. 로드
    pcd = o3d.io.read_point_cloud(src_ply)
    print(f"  원본 포인트: {len(pcd.points):,}")

    # 2. mm → m 스케일
    pts = np.asarray(pcd.points) * 0.001
    pcd.points = o3d.utility.Vector3dVector(pts)

    # 3. 다운샘플 (속도/품질 조절)
    pcd = pcd.voxel_down_sample(voxel_size)
    print(f"  다운샘플 후: {len(pcd.points):,} (voxel={voxel_size*1000:.0f}mm)")

    # 4. 법선벡터 추정 (PLY에 이미 있지만 스케일 변환 후 재계산)
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 3, max_nn=30
        )
    )
    pcd.orient_normals_consistent_tangent_plane(30)

    # 5. Poisson Surface Reconstruction
    print("  Poisson 메쉬 재구성 중...")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=9, width=0, scale=1.1, linear_fit=False
    )

    # 6. 저밀도 삼각형 제거 (경계 노이즈 정리)
    densities = np.asarray(densities)
    threshold = np.percentile(densities, 5)
    verts_to_remove = densities < threshold
    mesh.remove_vertices_by_mask(verts_to_remove)

    mesh.compute_vertex_normals()
    print(f"  메쉬: {len(mesh.vertices):,} verts, {len(mesh.triangles):,} tris")

    # 7. 저장
    o3d.io.write_triangle_mesh(dst_obj, mesh, write_ascii=True)
    print(f"  저장: {dst_obj}")
    return True


for src_name, dst_name in FILES:
    src = f"{CAD_BASE}/{src_name}"
    dst = f"{OUT_DIR}/{dst_name}"
    try:
        ply_to_mesh(src, dst, voxel_size=0.005)
    except Exception as e:
        print(f"  [ERROR] {e}")

print("\n=== 변환 완료 ===")
print(f"출력 폴더: {OUT_DIR}")
print("다음: Isaac Sim setup_drum_mesh.py 실행")
