#!/usr/bin/env python3
"""
ROS2 node to detect a circular ring and associated red marking in streamed
images and publish the resulting direction vector.

This node subscribes to a sensor_msgs/Image topic containing colour
frames (RGBA, RGB or BGR) from a Zivid camera (or any compatible
source). For each incoming frame it performs an adaptive detection
pipeline to locate a small red dot and the surrounding circular ring.
The detected vector from the ring centre to the red mark is then
published on a geometry_msgs/Vector3 topic.  The x and y components
contain the pixel differences (dx, dy) and the z component contains
the angle in degrees.  Angles are measured relative to the 9 o'clock
direction (pointing left), with clockwise rotations reported as
positive values up to +180° and counter‑clockwise rotations reported
as negative values down to −180°.  To reduce jitter the angle is
quantised to 45° increments whenever it lies within ±10° of a
multiple of 45°.

The detection logic is adapted from an earlier standalone script and
augmented to handle challenging cases where the red dot may be absent
or faint.  In such cases the algorithm falls back to selecting the
most plausible ring candidate and choosing the brightest red pixel
within the annulus as the red mark.

Usage:

    ros2 run <package_name> red_dot_detector_node.py

Parameters:
  image_topic   (string): Name of the Image topic to subscribe to.
  output_topic  (string): Name of the Vector3 topic to publish the vector.
"""

import glob
import math
import os
import sys
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image


def new_red_mask_multispace(bgr: np.ndarray, s_min: int = 40, a_thr: int = 150,
                            rg_thr: float = 1.2) -> np.ndarray:
    """Return a conservative mask of red pixels using HSV, LAB and R/G ratio tests."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    a = lab[:, :, 1]
    m1 = cv2.inRange(hsv, (0, s_min, 40), (15, 255, 255))
    m2 = cv2.inRange(hsv, (165, s_min, 40), (180, 255, 255))
    mh = cv2.bitwise_or(m1, m2)
    ma = cv2.inRange(a, a_thr, 255)
    B, G, R = cv2.split(bgr)
    Gsafe = np.maximum(G.astype(np.float32), 1.0)
    ratio = (R.astype(np.float32) + 1.0) / Gsafe
    mr = (ratio >= rg_thr).astype(np.uint8) * 255
    m = cv2.bitwise_and(mh, ma)
    m = cv2.bitwise_and(m, mr)
    m = cv2.medianBlur(m, 5)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), 1)
    return m


def get_red_hint(bgr: np.ndarray) -> Optional[Tuple[int, int]]:
    """Detect the largest compact red cluster and return its centroid."""
    mask = new_red_mask_multispace(bgr)
    num, labels, stats, cents = cv2.connectedComponentsWithStats(mask)
    candidates: List[Tuple[float, float, int]] = []
    for i in range(1, num):
        area = stats[i, cv2.CC_STAT_AREA]
        w = stats[i, cv2.CC_STAT_WIDTH]
        h = stats[i, cv2.CC_STAT_HEIGHT]
        cx, cy = cents[i]
        if area < 10 or area > 2000:
            continue
        aspect = min(w, h) / max(w, h) if max(w, h) > 0 else 0.0
        if aspect < 0.4:
            continue
        candidates.append((cx, cy, area))
    if not candidates:
        return None
    # 면적이 가장 큰 것을 선택 (가장 확실한 빨간점)
    candidates.sort(key=lambda t: -t[2])  # 면적 내림차순 정렬
    cx, cy, area = candidates[0]
    return int(round(cx)), int(round(cy))


def adaptive_whiteness_mask(bgr: np.ndarray) -> np.ndarray:
    """Compute a mask of bright, low‑saturation pixels with fallback for bright scenes."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    S, V = hsv[:, :, 1], hsv[:, :, 2]
    s_p = np.percentile(S, 70)
    v_p = np.percentile(V, 30)
    s_thr = int(np.clip(s_p, 60, 120))
    v_thr = int(np.clip(v_p, 120, 255))
    white = cv2.inRange(hsv, (0, 0, v_thr), (179, s_thr, 255))
    mask = cv2.medianBlur(white, 5)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), 1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), 1)
    if np.count_nonzero(mask) < 500:
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        bright_thresh = np.percentile(enhanced, 80)
        bright_mask = cv2.threshold(enhanced, bright_thresh, 255, cv2.THRESH_BINARY)[1]
        bright_mask = cv2.medianBlur(bright_mask, 7)
        bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), 2)
        bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), 2)
        if np.count_nonzero(bright_mask) > np.count_nonzero(mask):
            mask = bright_mask
    return mask


def adaptive_background_mask(bgr: np.ndarray) -> np.ndarray:
    """Determine a mask for the background region based on the presence of a blue drum."""
    H, W = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    blue = cv2.inRange(hsv, (85, 30, 20), (145, 255, 255))
    blue = cv2.medianBlur(blue, 5)
    blue = cv2.morphologyEx(blue, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), 1)
    blue_ratio = cv2.countNonZero(blue) / (H * W)
    if blue_ratio > 0.15:
        return blue
    else:
        return np.ones((H, W), np.uint8) * 255


def sample_coverage(mask: np.ndarray, cx: int, cy: int, r: float,
                    n: int = 180, tol: int = 1) -> float:
    """Sample how much of a circular path is covered by a binary mask."""
    H, W = mask.shape[:2]
    hit = 0
    tot = 0
    for k in range(n):
        th = 2 * math.pi * k / n
        x = int(round(cx + r * math.cos(th)))
        y = int(round(cy + r * math.sin(th)))
        if 0 <= x < W and 0 <= y < H:
            x0, x1 = max(0, x - tol), min(W - 1, x + tol)
            y0, y1 = max(0, y - tol), min(H - 1, y + tol)
            tot += 1
            if cv2.countNonZero(mask[y0:y1 + 1, x0:x1 + 1]) > 0:
                hit += 1
    return hit / max(1, tot)


def refine_circle_by_edges(edges: np.ndarray, cx: int, cy: int, ro: float,
                           ri_ratio: float = 0.45) -> Tuple[int, int, float]:
    """Refine a circle using least‑squares fitting on edge points within an annulus."""
    H, W = edges.shape[:2]
    ri = int(ro * ri_ratio)
    ann = np.zeros((H, W), np.uint8)
    cv2.circle(ann, (cx, cy), int(ro * 0.98), 255, -1)
    cv2.circle(ann, (cx, cy), int(ri * 1.02), 0, -1)
    pts = np.column_stack(np.where(cv2.bitwise_and(edges, ann) > 0))
    if len(pts) < 8:
        return cx, cy, ro
    x = pts[:, 1].astype(np.float64)
    y = pts[:, 0].astype(np.float64)
    A = np.stack([2 * x, 2 * y, np.ones_like(x)], axis=1)
    b = (x * x + y * y)
    sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    cx_ref, cy_ref, c = sol
    r_ref = math.sqrt(max(1e-6, cx_ref * cx_ref + cy_ref * cy_ref + c))
    return int(round(cx_ref)), int(round(cy_ref)), float(r_ref)


def process_image_from_file(image_path: str, output_dir: str = None) -> Optional[dict]:
    """
    이미지 파일에서 키링 홀더를 감지하고 결과를 시각화하여 저장합니다.
    
    Args:
        image_path: 입력 이미지 파일 경로
        output_dir: 결과 이미지 저장 폴더 (기본값: 시간 폴더)
    
    Returns:
        감지 결과 딕셔너리 또는 None (감지 실패시)
    """
    
    # 결과 폴더 생성 (시간戳 사용)
    if output_dir is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"result_{timestamp}"
    
    Path(output_dir).mkdir(exist_ok=True)
    
    # 이미지 읽기
    bgr = cv2.imread(image_path)
    if bgr is None:
        print(f"이미지를 읽을 수 없습니다: {image_path}")
        return None
    
    # 감지 수행
    result = detect_vector_from_bgr(bgr)
    if result is None:
        print(f"키링 홀더를 감지할 수 없습니다: {image_path}")
        return None
    
    vx, vy, angle_deg, cx, cy, dx, dy = result
    
    # 결과 이미지 생성 (원본 복사)
    result_img = bgr.copy()
    
    # 원형 링 그리기 (초록색) - 대략적인 반지름 계산
    ro = math.sqrt(vx*vx + vy*vy) * 1.5  # 대략적인 외부 반지름
    cv2.circle(result_img, (cx, cy), int(ro), (0, 255, 0), 3)
    cv2.circle(result_img, (cx, cy), int(ro * 0.45), (0, 255, 0), 2)
    
    # 중심점 표시 (파란색)
    cv2.circle(result_img, (cx, cy), 5, (255, 0, 0), -1)
    
    # 빨간 점 표시 (빨간색)
    cv2.circle(result_img, (dx, dy), 8, (0, 0, 255), -1)
    
    # 방향 벡터 화살표 그리기 (노란색)
    cv2.arrowedLine(result_img, (cx, cy), (dx, dy), (0, 255, 255), 3, tipLength=0.3)
    
    # 각도 정보 텍스트 추가
    text = f"Angle: {angle_deg:.1f}°"
    cv2.putText(result_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    cv2.putText(result_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1)
    
    # 벡터 정보 텍스트
    vec_text = f"Vector: ({vx}, {vy})"
    cv2.putText(result_img, vec_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    cv2.putText(result_img, vec_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)
    
    # 결과 저장
    input_filename = Path(image_path).stem
    folder_name = Path(image_path).parent.name  # 폴더 이름 가져오기
    output_path = os.path.join(output_dir, f"{folder_name}_{input_filename}_result.png")
    cv2.imwrite(output_path, result_img)
    
    print(f"결과가 저장되었습니다: {output_path}")
    print(f"감지된 각도: {angle_deg:.2f}도")
    print(f"벡터: ({vx}, {vy})")
    
    return {
        'vector': (vx, vy),
        'angle': angle_deg,
        'center': (cx, cy),
        'red_dot': (dx, dy),
        'output_path': output_path
    }


def process_multiple_images(image_dir: str, output_dir: str = None) -> list:
    """
    폴더 내의 모든 이미지를 처리합니다.
    
    Args:
        image_dir: 이미지 폴더 경로
        output_dir: 결과 저장 폴더 (기본값: 시간 폴더)
    
    Returns:
        처리된 결과 리스트
    """
    
    # 결과 폴더 생성 (시간 사용)
    if output_dir is None:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_dir = f"result_{timestamp}"
    
    results = []
    image_extensions = ['*.png', '*.jpg', '*.jpeg', '*.bmp']
    
    for ext in image_extensions:
        pattern = os.path.join(image_dir, ext)
        image_files = glob.glob(pattern)
        
        for image_path in image_files:
            print(f"\n처리 중: {image_path}")
            result = process_image_from_file(image_path, output_dir)
            if result:
                results.append(result)
    
    print(f"\n총 {len(results)}개 이미지 처리 완료")
    return results


def test_with_keycode_images():
    """1001 키코드 사진 폴더의 이미지들을 테스트합니다."""

    # 두 경로 설정
    image_dirs = ["1001 키코드 사진/위치1", "1001 키코드 사진/위치2","1001 키코드 사진/위치3","1001 키코드 사진/위치4"]
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = f"keycode_test_{timestamp}"

    results = []
    for image_dir in image_dirs:
        results.extend(process_multiple_images(image_dir, output_dir))

    # 결과 요약 출력
    if results:
        print(f"\n=== 처리 결과 요약 (저장 폴더: {output_dir}) ===")
        for i, result in enumerate(results, 1):
            print(f"{i}. 각도: {result['angle']:.2f}도, 벡터: {result['vector']}")




def detect_vector_from_bgr(bgr: np.ndarray) -> Optional[Tuple[int, int, float, int, int, int, int]]:
    """Detect ring and red mark in a BGR image and compute the vector and angle."""
    H, W = bgr.shape[:2]
    min_dim = min(H, W)
    
    
    # 1단계: 빨간점 감지
    hint = get_red_hint(bgr)
    use_hint = hint is not None
    rx, ry = hint if use_hint else (-1, -1)
    
    white = adaptive_whiteness_mask(bgr)
    background = adaptive_background_mask(bgr)
    edges = cv2.Canny(white, 50, 150)
    max_radius_ratio = 0.13 if np.count_nonzero(background) / (H * W) > 0.8 else 0.40
    minR = int(0.05 * min_dim)
    maxR = int(max_radius_ratio * min_dim)
    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1.2,
                               minDist=max(20, int(min_dim / 2.5)),
                               param1=100, param2=30,
                               minRadius=minR, maxRadius=maxR)
    
    if circles is None or len(circles[0]) == 0:
        return None
    best: Optional[Tuple[int, int, float]] = None
    best_score = -1e9
    fallback_candidates: List[Tuple[int, int, float, float]] = []
    ri_ratio = 0.45
    
    
    # 모든 원 후보들을 순서대로 검사하여 적합한 원 찾기
    best = None
    annulus_inner_ratio = 0.3  # 널널한 범위
    annulus_outer_ratio = 1.2  # 널널한 범위
    min_abs_distance = 50     # 최소 절대 거리
    max_abs_distance = 350    # 최대 절대 거리
    
    for i, c in enumerate(circles[0]):
        cx, cy, ro = int(round(c[0])), int(round(c[1])), float(c[2])
        if not (0 <= cx < W and 0 <= cy < H):
            continue
        if ro < minR or ro > maxR:
            continue
        
        # 빨간점으로 검증 (annulus 비율 + 절대 거리 모두 고려)
        if use_hint:
            ri = ro * annulus_inner_ratio
            ro_outer = ro * annulus_outer_ratio
            d_cent = math.hypot(rx - cx, ry - cy)
            
            # annulus 비율 기준
            annulus_ok = (ri <= d_cent <= ro_outer)
            # 절대 거리 기준
            abs_distance_ok = (min_abs_distance <= d_cent <= max_abs_distance)
            
            # 둘 중 하나라도 통과하면 OK
            if annulus_ok or abs_distance_ok:
                best = (cx, cy, ro)
                break  # 적합한 원을 찾았으므로 종료
            else:
                continue
        else:
            # 빨간점이 없으면 첫 번째 유효한 원 선택
            best = (cx, cy, ro)
            break
    
    if best is None:
        return None
    cx, cy, ro = best
    
    
    # 정밀 조정을 조건부로 적용 (엣지 품질이 좋을 때만)
    original_cx, original_cy, original_ro = cx, cy, ro
    
    # 엣지 품질 확인
    ri = int(ro * ri_ratio)
    ann = np.zeros((H, W), np.uint8)
    cv2.circle(ann, (cx, cy), int(ro * 0.98), 255, -1)
    cv2.circle(ann, (cx, cy), int(ri * 1.02), 0, -1)
    edge_points = np.column_stack(np.where(cv2.bitwise_and(edges, ann) > 0))
    
    
    # 충분한 엣지 점이 있고 품질이 좋을 때만 정밀 조정
    if len(edge_points) >= 20:  # 충분한 점이 있을 때만
        refined_cx, refined_cy, refined_ro = refine_circle_by_edges(edges, cx, cy, ro, ri_ratio=ri_ratio)
        
        # 정밀 조정 결과가 너무 크게 변하지 않았을 때만 적용
        center_change = math.hypot(refined_cx - cx, refined_cy - cy)
        radius_change = abs(refined_ro - ro) / ro
        
        if center_change < ro * 0.2 and radius_change < 0.5:  # 20% 이내 변화만 허용
            cx, cy, ro = refined_cx, refined_cy, refined_ro
    
    if use_hint:
        dx, dy = rx, ry
    else:
        ann = np.zeros((H, W), np.uint8)
        cv2.circle(ann, (cx, cy), int(ro * 0.96), 255, -1)
        cv2.circle(ann, (cx, cy), int((ro * ri_ratio) * 1.05), 0, -1)
        B, G, R = cv2.split(bgr)
        ys, xs = np.where(ann > 0)
        if len(xs) == 0:
            return None
        idx = np.argmax(R[ys, xs])
        dx, dy = int(xs[idx]), int(ys[idx])
    vx = dx - cx
    vy = dy - cy
    mx = float(vx)
    my = float(-vy)
    theta = math.atan2(my, mx)
    ref_angle = math.pi
    angle_deg = math.degrees(ref_angle - theta)
    if angle_deg > 180.0:
        angle_deg -= 360.0
    elif angle_deg <= -180.0:
        angle_deg += 360.0
    def quantise(a: float) -> float:
        if -10.0 <= a <= 10.0:
            return 0.0
        if 35.0 <= a <= 55.0:
            return 45.0
        if 80.0 <= a <= 100.0:
            return 90.0
        if 125.0 <= a <= 145.0:
            return 135.0
        if a >= 170.0 or a <= -170.0:
            return 180.0
        if -55.0 <= a <= -35.0:
            return -45.0
        if -100.0 <= a <= -80.0:
            return -90.0
        if -145.0 <= a <= -125.0:
            return -135.0
        return a
    angle_deg = quantise(angle_deg)
    return int(vx), int(vy), float(angle_deg), int(cx), int(cy), int(dx), int(dy)


class RedDotDetectorNode(Node):
    """ROS2 node that subscribes to images, detects ring and red mark, and publishes a vector."""
    def __init__(self) -> None:
        super().__init__('red_dot_detector')
        self.declare_parameter('image_topic', '/zivid/color/image_color')
        self.declare_parameter('output_topic', '/hanyang/coupler/keycode_holder_angle')
        self.declare_parameter('save_results', True)
        self.declare_parameter('result_dir', os.path.join(os.path.expanduser('~'), 'keycode_ws', 'result_holder'))
        
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.vector_pub = self.create_publisher(Vector3, output_topic, 10)
        self.image_sub = self.create_subscription(Image, image_topic, self.image_callback, 10)
        self.bridge = CvBridge()
        
        # 결과 저장을 위한 폴더 설정
        self.result_dir = Path(self.get_parameter('result_dir').get_parameter_value().string_value)
        self.result_dir.mkdir(parents=True, exist_ok=True)
        self.save_seq = 0
        
        self.get_logger().info(f'RedDotDetectorNode subscribed to {image_topic} and publishing on {output_topic}')

    def image_callback(self, msg: Image) -> None:
        try:
            encoding = msg.encoding.lower() if hasattr(msg, 'encoding') else 'rgba8'
            if encoding == 'rgba8':
                img_rgba = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgba8')
                bgr = cv2.cvtColor(img_rgba, cv2.COLOR_RGBA2BGR)
            elif encoding == 'rgb8':
                img_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            elif encoding == 'bgr8':
                bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_img = self.bridge.imgmsg_to_cv2(msg)
                if cv_img.ndim == 3 and cv_img.shape[2] == 4:
                    bgr = cv2.cvtColor(cv_img, cv2.COLOR_RGBA2BGR)
                elif cv_img.ndim == 3 and cv_img.shape[2] == 3:
                    bgr = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
                else:
                    self.get_logger().warning('Unsupported image format; skipping frame')
                    return
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return
        
        res = detect_vector_from_bgr(bgr)
        
        # 이미지 저장
        try:
            save_results = self.get_parameter('save_results').get_parameter_value().bool_value
            if save_results:
                self.save_seq += 1
                stamp = msg.header.stamp if hasattr(msg, 'header') else None
                t_ms = int(stamp.sec * 1000 + stamp.nanosec / 1e6) if stamp else 0
                base = f'{t_ms:013d}_{self.save_seq:06d}'
                
                # 원본 이미지 저장
                raw_path = self.result_dir / f'{base}_raw.png'
                cv2.imwrite(str(raw_path), bgr)
                
                # 결과 이미지 생성 및 저장
                if res is not None:
                    vx, vy, angle_deg, cx, cy, dx, dy = res
                    result_img = bgr.copy()
                    
                    # 원형 링 그리기 (초록색)
                    ro = math.sqrt(vx*vx + vy*vy) * 1.5
                    cv2.circle(result_img, (cx, cy), int(ro), (0, 255, 0), 3)
                    cv2.circle(result_img, (cx, cy), int(ro * 0.45), (0, 255, 0), 2)
                    
                    # 중심점 표시 (파란색)
                    cv2.circle(result_img, (cx, cy), 5, (255, 0, 0), -1)
                    
                    # 빨간 점 표시 (빨간색)
                    cv2.circle(result_img, (dx, dy), 8, (0, 0, 255), -1)
                    
                    # 방향 벡터 화살표 그리기 (노란색)
                    cv2.arrowedLine(result_img, (cx, cy), (dx, dy), (0, 255, 255), 3, tipLength=0.3)
                    
                    # 각도 정보 텍스트
                    text = f"Angle: {angle_deg:.1f}°"
                    cv2.putText(result_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                    cv2.putText(result_img, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 1)
                    
                    # 벡터 정보 텍스트
                    vec_text = f"Vector: ({vx}, {vy})"
                    cv2.putText(result_img, vec_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
                    cv2.putText(result_img, vec_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 0), 1)
                else:
                    result_img = bgr.copy()
                    cv2.putText(result_img, 'no detection', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                
                ann_path = self.result_dir / f'{base}_annotated.png'
                cv2.imwrite(str(ann_path), result_img)
                
        except Exception as e:
            self.get_logger().warn(f'Image save failed: {e}')
        
        if res is None:
            return
        vx, vy, angle_deg, cx, cy, dx, dy = res
        vec_msg = Vector3()
        vec_msg.x = float(vx)
        vec_msg.y = float(vy)
        vec_msg.z = float(angle_deg)
        self.vector_pub.publish(vec_msg)
        self.get_logger().info(f'Detected vector: ({vx}, {vy}), angle={angle_deg:.2f}°')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RedDotDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # 명령행 인수가 있으면 이미지 파일 처리 모드로 실행
    if len(sys.argv) > 1:
        if sys.argv[1] == "test_all":
            # 모든 키코드 이미지 테스트
            test_with_keycode_images()
        elif sys.argv[1] == "test_folder" and len(sys.argv) > 2:
            # 특정 폴더의 모든 이미지 처리
            folder_path = sys.argv[2]
            process_multiple_images(folder_path)
        else:
            # 단일 이미지 처리
            image_path = sys.argv[1]
            process_image_from_file(image_path)
    else:
        # 기본 ROS2 노드 실행
        main()