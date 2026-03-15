#!/usr/bin/env python3
# coding: utf-8
"""
ROS2 node to detect a white ring and red dot in images coming from a Zivid camera.

- Subscribes: /zivid/color/image_color (sensor_msgs/Image, RGBA/RGB/BGR)
- Publishes:  /hanyang/coupler/keycode_angle (geometry_msgs/Vector3)
    * x, y = vector components (pixels), z = angle in degrees
    * angle = +y axis 기준, 시계방향으로 증가 (atan2(vx, -vy))

- Saves images to: result/<timestamp>_raw.png and _annotated.png
"""

import math
import os
from datetime import datetime
from pathlib import Path
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
from rclpy.node import Node
from sensor_msgs.msg import Image

# 전역 설정
FORCE_ROBUST = True       # robust_find_ring만 사용
DISABLE_FALLBACK = True   # 폴백 비활성화
DBG_LOG = True            # 디버그 로그 활성화


# ---------------------- Mask & Utility Functions ---------------------- #
def _whiteness_ratio_in_annulus(bgr: np.ndarray, cx: int, cy: int, ro: int, ri_ratio: float = 0.45) -> float:
    """링 안에서 흰색이 얼마나 많은지 계산"""
    H, W = bgr.shape[:2]
    ri = int(ro * ri_ratio)
    ann = np.zeros((H, W), np.uint8)
    cv2.circle(ann, (cx, cy), int(ro * 0.96), 255, -1)  # 바깥 원
    cv2.circle(ann, (cx, cy), int(ri * 1.05), 0, -1)    # 안쪽 원

    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    S, V = hsv[:, :, 1], hsv[:, :, 2]
    s_thr = int(np.clip(np.percentile(S, 70), 60, 120))  # 색깔 진하기 기준
    v_thr = int(np.clip(np.percentile(V, 30), 120, 255)) # 밝기 기준
    white = cv2.inRange(hsv, (0, 0, 0), (179, s_thr, 255))  # 흰색 찾기
    bright = cv2.inRange(hsv, (0, 0, v_thr), (179, 255, 255))  # 밝은 부분 찾기
    wmask = cv2.bitwise_and(white, bright)  # 흰색이면서 밝은 부분

    return cv2.countNonZero(cv2.bitwise_and(wmask, ann)) / max(1, cv2.countNonZero(ann))


def _circle_fit_lsq(points: np.ndarray) -> Optional[Tuple[float, float, float]]:
    """
    Simple least-squares circle fit (Kåsa method).
    points: (N, 2) array of (x, y)
    return: (cx, cy, r) or None
    """
    if points is None or len(points) < 6:
        return None
    x = points[:, 0].astype(np.float64)
    y = points[:, 1].astype(np.float64)
    A = np.stack([2 * x, 2 * y, np.ones_like(x)], axis=1)
    b = (x * x + y * y)
    try:
        sol, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    except Exception:
        return None
    cx, cy, c = sol[0], sol[1], sol[2]
    r = np.sqrt(max(1e-6, cx * cx + cy * cy + c))
    return float(cx), float(cy), float(r)


def _ring_candidate_score(bgr: np.ndarray, cx: int, cy: int, ro: int, ri_ratio: float,
                          red_hint: Optional[Tuple[int, int]],
                          white_mask: np.ndarray,
                          blue_mask: np.ndarray) -> float:
    """
    후보 원 점수:
      - 흰색 도넛(내/외곽) 원주 커버리지 + 도넛 내부 흰색 비율
      - 파란 드럼 외곽 원주 커버리지
      - (있다면) 빨간 힌트가 ri~ro 대역에 있는지
    """
    H, W = bgr.shape[:2]
    ri = int(ro * ri_ratio)

    # 내/외곽 원주 커버리지
    cov_inner = sample_coverage(white_mask, cx, cy, ri, n=360, tol=1)
    cov_outer = sample_coverage(white_mask, cx, cy, ro, n=360, tol=1)

    # 도넛 흰색 비율
    ann = np.zeros((H, W), np.uint8)
    cv2.circle(ann, (cx, cy), int(ro * 0.96), 255, -1)
    cv2.circle(ann, (cx, cy), int(ri * 1.05), 0, -1)
    white_ratio = cv2.countNonZero(cv2.bitwise_and(white_mask, ann)) / max(1, cv2.countNonZero(ann))

    # 파란 드럼 외곽 원주 커버리지
    cov_blue = sample_coverage(blue_mask, cx, cy, ro, n=180, tol=1)

    # 기본 점수
    score = 150.0 * cov_inner + 150.0 * cov_outer + 120.0 * white_ratio + 50.0 * cov_blue

    # 빨간 힌트 일치성(있을 때 가점/불일치 감점)
    if red_hint is not None:
        rx, ry = red_hint
        d = math.hypot(rx - cx, ry - cy)
        if ri * 0.75 < d < ro * 1.20:
            score += 30.0
        else:
            score -= 60.0
    else:
        score -= 10.0
    return float(score)


def robust_find_ring(bgr: np.ndarray):
    """
    개선된 링 찾기:
      1) 회색 엣지 대신 '흰색 마스크'에서 엣지 추출(배경 억제)
      2) 흰색/파란 드럼 커버리지 + 도넛 흰색 비율 + (있을 때) 빨간 힌트 일치성으로 점수화
      3) 선택 후 외곽 엣지 포인트로 원 리파인(LSQ 피팅) → 중심 튐 억제
    return: ((cx, cy), ro, ri) or None
    """
    if DBG_LOG: print("[robust] start")
    H, W = bgr.shape[:2]

    # 1) 마스크 준비 (기존 함수 재사용)
    white = whiteness_mask(bgr)
    blue = drum_blue_mask(bgr, hmin=90, hmax=140, smin=40, vmin=30, dilate=3)

    # 적색 힌트(있으면 후보 선별에 반영)
    try:
        redmask = preprocess_red_detection(bgr, s_min=50, a_thr=140, rg_thr=1.25)
        ys, xs = np.where(redmask > 0)
        red_hint = (int(xs.mean()), int(ys.mean())) if len(xs) > 0 else None
    except Exception:
        red_hint = None

    # 2) 흰색 기반 엣지 → Hough
    edges = cv2.Canny(white, 50, 150)
    circles = cv2.HoughCircles(
        edges, cv2.HOUGH_GRADIENT, dp=1.2, minDist=max(20, int(min(H, W) // 2.5)),
        param1=120, param2=34,
        minRadius=int(0.05 * min(H, W)),
        maxRadius=int(0.40 * min(H, W))
    )
    if circles is None or len(circles[0]) == 0:
        return None
    if circles is None:
        if DBG_LOG: print("[robust] circles=None")
    else:
        if DBG_LOG: print(f"[robust] circles={len(circles[0])}")

    # 3) 후보 점수화
    RI_RATIO = 0.45
    best = None
    best_score = -1e9
    best_ro = None
    for c in circles[0]:
        cx, cy, ro = int(round(c[0])), int(round(c[1])), int(round(c[2]))
        # 화면 밖/너무 작은 원 제외
        if not (0 <= cx < W and 0 <= cy < H) or ro < 6:
            continue
        sc = _ring_candidate_score(bgr, cx, cy, ro, RI_RATIO, red_hint, white, blue)
        if sc > best_score:
            best_score = sc
            best = (cx, cy)
            best_ro = ro
    if DBG_LOG: print(f"[robust] best_score={best_score:.1f}, param2=34, cut=85")


    # 점수 하한(환경 따라 100~140 사이 조정 가능)
    if best is None or best_score < 85.0:
        return None

    cx, cy = best
    ro = int(best_ro)
    ri = int(ro * RI_RATIO)

    # 4) 중심 리파인: 도넛 영역의 흰색 엣지 포인트로 원 피팅
    ann = np.zeros((H, W), np.uint8)
    cv2.circle(ann, (cx, cy), int(ro * 0.98), 255, -1)
    cv2.circle(ann, (cx, cy), int(ri * 1.02), 0, -1)
    edge_in_ann = cv2.bitwise_and(edges, ann)
    pts = np.column_stack(np.where(edge_in_ann > 0))  # (y, x)

    if len(pts) >= 12:
        pts_xy = pts[:, ::-1].astype(np.float32)  # (x, y)
        fit = _circle_fit_lsq(pts_xy)
        if fit is not None:
            cx_ref, cy_ref, r_ref = fit
            # 반지름 튀는 값 방지: Hough 값과 25% 이상 차이나면 유지
            if abs(r_ref - ro) < 0.25 * ro:
                cx, cy = int(round(cx_ref)), int(round(cy_ref))
                ro = int(round(r_ref))
                ri = int(ro * RI_RATIO)
    if DBG_LOG: print(f"[robust] OK: cx,cy,ro={cx},{cy},{ro}")

    return ((int(cx), int(cy)), float(ro), float(ri))


def detect_red_in_annulus(
    bgr: np.ndarray, cx: int, cy: int, ri: float, ro: float,
    # 기본(엄격) 파라미터 — 현재 코드의 fail 로그 기준으로 유지
    min_area: int = 4,
    max_area: int = 140,
    min_excess: float = 14.0,    # R - max(G,B)
    min_sat: int = 80,
    glare_sat: int = 65, glare_v: int = 230,
    red_h_lo1: int = 0, red_h_hi1: int = 15,     # HSV 빨강 대역1
    red_h_lo2: int = 165, red_h_hi2: int = 180,  # HSV 빨강 대역2
    min_circularity: float = 0.60,
    min_local_contrast: float = 12.0
) -> Optional[Tuple[int, int]]:
    """
    링 도넛(ri~ro) 내부에서 빨간 점을 찾는다.
    1) 엄격 조건으로 후보를 만들고,
    2) (필요 시) 완화 조건으로 재시도한다.
    반환: (dx, dy) 또는 None
    """
    H, W = bgr.shape[:2]

    # ── 도넛 마스크(여유 확대) ────────────────────────────────────────────────
    ann = np.zeros((H, W), np.uint8)
    cv2.circle(ann, (int(cx), int(cy)), int(ro * 0.97), 255, -1)  # 바깥 0.95 → 0.97
    cv2.circle(ann, (int(cx), int(cy)), int(ri * 1.08), 0, -1)    # 안쪽 1.05 → 1.08

    # 색공간/채널
    B, G, R = cv2.split(bgr)
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    Hh, Ss, Vv = cv2.split(hsv)

    hue_red = ((Hh >= red_h_lo1) & (Hh <= red_h_hi1)) | ((Hh >= red_h_lo2) & (Hh <= red_h_hi2))
    glare   = (Ss < glare_sat) & (Vv > glare_v)
    excess_rb = (R.astype(np.float32) - np.maximum(G, B).astype(np.float32))

    # ── 1단계: 엄격 후보 ──────────────────────────────────────────────────────
    base_strict = (ann > 0) & hue_red & (Ss >= min_sat) & (~glare) & (excess_rb > min_excess)
    cand = (base_strict.astype(np.uint8)) * 255
    cand = cv2.morphologyEx(cand, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), 1)
    cand = cv2.morphologyEx(cand, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), 1)

    num1, lab1, stats1, cents1 = cv2.connectedComponentsWithStats(cand, connectivity=8)
    print(f"[red] cand1={num1-1} strict (area>={min_area}, excess>={min_excess}, sat>={min_sat}, contrast>={min_local_contrast})")

    def _pick_center_by_score(comp_lab, comp_stats, comp_cents) -> Optional[Tuple[int, int]]:
        """면적/원형도/국소 대비/빨강 초과로 점수화해 가장 좋은 컴포넌트 중심을 고른다."""
        if comp_stats is None or len(comp_stats) <= 1:
            return None

        # 도넛 배경의 R 채널 기준값(국소 대비 계산용)
        dil = cv2.dilate((comp_lab > 0).astype(np.uint8) * 255, np.ones((7, 7), np.uint8), 1)
        ring_bg = ((ann > 0) & (dil == 0))
        if np.count_nonzero(ring_bg) >= 20:
            bg_med = float(np.median(R[ring_bg]))
        else:
            bg_med = float(np.median(R[ann > 0]))

        best_score, best_xy = -1.0, None
        for k in range(1, len(comp_stats)):
            x, y, w, h, area = comp_stats[k]
            if area < min_area or area > max_area:
                continue
            comp = (lab1 == k).astype(np.uint8) * 255  # lab1 사용(동일 크기)
            cnts, _ = cv2.findContours(comp, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                continue
            per = cv2.arcLength(cnts[0], True)
            if per <= 0:
                continue
            circ = 4.0 * np.pi * (area / (per * per))  # 원형도
            if circ < min_circularity:
                continue

            r_mean = float(np.mean(R[lab1 == k]))
            contrast = r_mean - bg_med
            if contrast < min_local_contrast:
                continue

            exc_mean = float(np.mean(excess_rb[lab1 == k]))
            sat_mean = float(np.mean(Ss[lab1 == k]))
            score = (0.60 * exc_mean) + (0.20 * contrast) + (0.20 * circ * 100.0) + (0.05 * sat_mean)

            if score > best_score:
                best_score = score
                cx_, cy_ = comp_cents[k]
                best_xy = (int(round(cx_)), int(round(cy_)))
        return best_xy

    pick = None
    if num1 > 1:
        pick = _pick_center_by_score(lab1, stats1, cents1)
    if pick is not None:
        return pick

    # ── 2단계: 완화 후보(weak) ───────────────────────────────────────────────
    base_weak = (ann > 0) & hue_red \
                & (Ss >= max(50, min_sat - 20)) \
                & (~glare) \
                & (excess_rb > max(10.0, min_excess - 4.0))
    cand2 = (base_weak.astype(np.uint8)) * 255
    cand2 = cv2.medianBlur(cand2, 3)

    num2, lab2, stats2, cents2 = cv2.connectedComponentsWithStats(cand2, connectivity=8)
    print(f"[red] cand2={num2-1} weak")

    if num2 <= 1:
        return None

    # 완화 단계에서는 가장 큰 컴포넌트를 채택(원형도 검사 생략)
    areas = stats2[1:, cv2.CC_STAT_AREA]
    k = 1 + int(np.argmax(areas))
    area_k = areas[k - 1]
    if area_k < max(3, min_area - 1) or area_k > int(max_area * 1.5):
        return None

    dx, dy = cents2[k]
    return int(round(dx)), int(round(dy))


def drum_blue_mask(bgr: np.ndarray, hmin: int = 90, hmax: int = 140,
                   smin: int = 40, vmin: int = 30, dilate: int = 0) -> np.ndarray:
    """파란색 드럼 부분 찾기"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    base = cv2.inRange(hsv, (int(hmin), int(smin), int(vmin)), (int(hmax), 255, 255))  # 파란색 범위
    m = cv2.medianBlur(base, 5)  # 노이즈 제거
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), 2)  # 구멍 메우기
    if dilate and int(dilate) > 0:
        k = int(dilate)
        ker = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * k + 1, 2 * k + 1))
        m = cv2.dilate(m, ker, iterations=1)  # 크기 늘리기
    return m


def whiteness_mask(bgr: np.ndarray) -> np.ndarray:
    """흰색 영역 마스크 생성"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    S, V = hsv[:, :, 1], hsv[:, :, 2]
    s_thr = int(np.clip(np.percentile(S, 70), 80, 100))
    v_thr = int(np.clip(np.percentile(V, 30), 150, 255))
    white = cv2.inRange(hsv, (0, 0, 0), (179, s_thr, 255))
    bright = cv2.inRange(hsv, (0, 0, v_thr), (179, 255, 255))
    m = cv2.bitwise_and(white, bright)

    m = cv2.medianBlur(m, 7)
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), 2)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), 2)

    if np.sum(m) < 1000:
        m_bright = brightness_based_white_detection(bgr)
        if np.sum(m_bright) > np.sum(m):
            m = m_bright

    return m


def red_mask_multispace(bgr: np.ndarray, s_min: int = 40, a_thr: int = 145,
                        rg_thr: float = 1.3) -> np.ndarray:
    """여러 방법으로 빨간색 찾기"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    a = lab[:, :, 1]
    
    # 방법 1: HSV 색상으로 빨간색 찾기
    m1 = cv2.inRange(hsv, (0, s_min, 40), (12, 255, 255))  # 빨간색 범위 1
    m2 = cv2.inRange(hsv, (170, s_min, 40), (180, 255, 255))  # 빨간색 범위 2
    mh = cv2.bitwise_or(m1, m2)
    
    # 방법 2: LAB 색상으로 빨간색 찾기
    ma = cv2.inRange(a, a_thr, 255)
    
    # 방법 3: 빨간색이 녹색보다 강한지 확인
    B, G, R = cv2.split(bgr)
    Gsafe = np.maximum(G.astype(np.float32), 1.0)
    ratio = (R.astype(np.float32) + 1.0) / Gsafe
    mr = (ratio >= rg_thr).astype(np.uint8) * 255
    
    # 모든 방법 합치기
    m = cv2.bitwise_and(mh, ma)
    m = cv2.bitwise_and(m, mr)
    
    # 노이즈 제거
    m = cv2.medianBlur(m, 5)  # 작은 점 제거
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), 1)   # 구멍 메우기
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8), 1)  # 작은 구멍 메우기
    
    return m


def filter_red_points_by_distance(redmask: np.ndarray, cx: int, cy: int, 
                                 max_vx: int = 150, max_vy: int = 200) -> np.ndarray:
    """중심점에서 너무 먼 빨간 점들 제거"""
    H, W = redmask.shape[:2]
    
    # 가까운 빨간 점들만 선택
    valid_red_points = []
    ys, xs = np.where(redmask > 0)
    
    for i in range(len(xs)):
        x, y = xs[i], ys[i]
        vx = x - int(cx)
        vy = y - int(cy)
        
        # 가까운 점들만 선택
        if abs(vx) <= max_vx and abs(vy) <= max_vy:
            valid_red_points.append((x, y))
    
    if len(valid_red_points) == 0:
        return np.zeros_like(redmask)
    
    # 가까운 점들로 마스크 다시 만들기
    filtered_mask = np.zeros_like(redmask)
    for x, y in valid_red_points:
        filtered_mask[y, x] = 255
    
    return filtered_mask


def sample_coverage(mask: np.ndarray, cx: int, cy: int, r: float,
                    n: int = 360, tol: int = 1) -> float:
    H, W = mask.shape[:2]
    hit = 0
    tot = 0
    for k in range(n):
        th = 2 * np.pi * k / n
        x = int(round(cx + r * np.cos(th)))
        y = int(round(cy + r * np.sin(th)))
        if 0 <= x < W and 0 <= y < H:
            x0, x1 = max(0, x - tol), min(W - 1, x + tol)
            y0, y1 = max(0, y - tol), min(H - 1, y + tol)
            tot += 1
            if cv2.countNonZero(mask[y0:y1 + 1, x0:x1 + 1]) > 0:
                hit += 1
    return hit / max(1, tot)


def find_ring_for_redmask_by_contour(
    bgr: np.ndarray,
    redmask: np.ndarray,
    blue_hmin: int = 70,
    blue_hmax: int = 140,
    blue_smin: int = 10,
    blue_vmin: int = 30,
    blue_dilate: int = 4,
    white_cont_each: float = 0.75,
    blue_cont: float = 0.8,
    ri_ro_min: float = 0.18,
    ri_ro_max: float = 0.70,
    min_white_cover: float = 0.35,
):
    H, W = bgr.shape[:2]
    blue = drum_blue_mask(bgr, blue_hmin, blue_hmax, blue_smin, blue_vmin, dilate=blue_dilate)
    white = whiteness_mask(bgr)
    white_used = cv2.bitwise_and(white, blue)
    ys, xs = np.where(redmask > 0)
    if len(xs) == 0:
        return None, white_used, None, None
    rx, ry = int(xs.mean()), int(ys.mean())
    # 컨투어 근사화와 노이즈 제거 강화
    cnts, hier = cv2.findContours(white_used, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    
    # 작은 컨투어 제거 (노이즈 제거)
    min_area = (H * W) * 0.0005  # 최소 면적 설정
    cnts = [c for c in cnts if cv2.contourArea(c) > min_area]
    if hier is None or len(cnts) == 0 or len(hier) == 0:
        return None, white_used, None, None
    hier = hier[0]
    min_ro = 0.05 * min(H, W)
    max_ro = 0.30 * min(H, W)
    best = None
    best_score = -1e9
    best_ringmask = None
    best_outerline = None
    for i, c in enumerate(cnts):
        child = hier[i][2]
        if child < 0:
            continue
        area = cv2.contourArea(c)
        if area < (H * W) * 0.001 or area > (H * W) * 0.5:
            continue
        inner = None
        j = child
        while j != -1 and j < len(hier) and j < len(cnts):
            cc = cnts[j]
            if cv2.contourArea(cc) > 30:
                (xi, yi), ri = cv2.minEnclosingCircle(cc)
                ri = float(ri)
                inner = (xi, yi, ri)
                break
            if j < len(hier) and len(hier[j]) > 0:
                j = hier[j][0]
            else:
                break
        if inner is None:
            continue
        xi, yi, ri = inner
        cx, cy = int(round(xi)), int(round(yi))
        pts = c.reshape(-1, 2).astype(np.float32)
        dists = np.hypot(pts[:, 0] - xi, pts[:, 1] - yi)
        if len(dists) == 0:
            continue
        # 더 안정적인 외부 반지름 계산 (평균 사용)
        ro = float(np.mean(dists))
        if not (min_ro <= ro <= max_ro):
            continue
        ratio = ri / ro
        if ratio < ri_ro_min or ratio > ri_ro_max:
            continue
        d_cent = math.hypot(rx - cx, ry - cy)
        if d_cent < ri * 0.95 or d_cent > ro * 1.05:
            continue
        cov_inner = sample_coverage(white_used, cx, cy, ri, n=360, tol=1)
        cov_outer = sample_coverage(white_used, cx, cy, ro, n=360, tol=1)
        if cov_inner < white_cont_each or cov_outer < white_cont_each:
            continue
        cov_blue = sample_coverage(blue, cx, cy, ro, n=180, tol=1)
        if cov_blue < blue_cont:
            continue
        ringmask = np.zeros((H, W), np.uint8)
        cv2.circle(ringmask, (cx, cy), int(ro * 0.96), 255, -1)
        cv2.circle(ringmask, (cx, cy), int(ri * 1.05), 0, -1)
        white_ratio = cv2.countNonZero(cv2.bitwise_and(white_used, ringmask)) / max(1, cv2.countNonZero(ringmask))
        if white_ratio < min_white_cover:
            continue
        score = 150 * cov_inner + 150 * cov_outer + 100 * white_ratio - 0.0005 * (ro ** 2)
        if score > best_score:
            best_score = score
            best = ((cx, cy), ro, ri)
            best_ringmask = ringmask
            outerline = np.zeros((H, W), np.uint8)
            cv2.circle(outerline, (cx, cy), int(ro * 0.98), 255, 2)
            best_outerline = outerline
    return best, white_used, best_ringmask, best_outerline


def hough_detect_ring(bgr: np.ndarray, redmask: np.ndarray,
                      min_radius_ratio: float = 0.05,
                      max_radius_ratio: float = 0.4,
                      annulus_inner_ratio: float = 0.3,
                      whiteness_thresh: float = 0.5,
                      param2: int = 30):
    H, W = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    S, V = hsv[:, :, 1], hsv[:, :, 2]
    s_thr = int(np.clip(np.percentile(S, 70), 80, 100))
    v_thr = int(np.clip(np.percentile(V, 30), 150, 255))
    white = cv2.inRange(hsv, (0, 0, 0), (179, s_thr, 255))
    bright = cv2.inRange(hsv, (0, 0, v_thr), (179, 255, 255))
    wmask = cv2.bitwise_and(white, bright)
    wmask = cv2.medianBlur(wmask, 5)
    wmask = cv2.morphologyEx(wmask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), 1)
    wmask = cv2.morphologyEx(wmask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), 1)
    edges = cv2.Canny(wmask, 50, 150)
    minR = int(min_radius_ratio * min(H, W))
    maxR = int(max_radius_ratio * min(H, W))
    circles = cv2.HoughCircles(edges, cv2.HOUGH_GRADIENT, dp=1.2,
                               minDist=H // 3, param1=100, param2=param2,
                               minRadius=minR, maxRadius=maxR)
    ys, xs = np.where(redmask > 0)
    rcx, rcy = (int(xs.mean()), int(ys.mean())) if len(xs) > 0 else (None, None)
    if circles is None or len(circles) == 0 or len(circles[0]) == 0:
        return None
    best = None
    best_score = -1.0
    for c in circles[0]:
        cx, cy, ro = int(round(c[0])), int(round(c[1])), int(round(c[2]))
        ri = int(ro * annulus_inner_ratio)
        ann = np.zeros((H, W), np.uint8)
        cv2.circle(ann, (cx, cy), int(ro * 0.96), 255, -1)
        cv2.circle(ann, (cx, cy), int(ri * 1.05), 0, -1)
        total = cv2.countNonZero(ann)
        if total == 0:
            continue
        white_count = cv2.countNonZero(cv2.bitwise_and(whiteness_mask(bgr), ann))
        ratio = white_count / total
        if ratio < whiteness_thresh:
            continue
        red_ok = 0
        if rcx is not None:
            d = math.hypot(rcx - cx, rcy - cy)
            if ri * 0.9 < d < ro * 1.05:
                red_ok = 1
        score = ratio + 0.2 * red_ok
        if score > best_score:
            best_score = score
            best = ((cx, cy), float(ro), float(ri))
    return best


def preprocess_red_detection(bgr: np.ndarray, s_min: int = 50, a_thr: int = 140, rg_thr: float = 1.25) -> np.ndarray:
    """HSV+LAB+R/G비율로 빨간 점 감지 및 노이즈 제거"""
    return red_mask_multispace(bgr, s_min, a_thr, rg_thr)


def brightness_based_red_detection(bgr: np.ndarray) -> np.ndarray:
    """밝기 기반 빨간 점 감지 (특수 조명 환경용)"""
    # 그레이스케일 변환
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    
    # 히스토그램 균등화로 대비 향상
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
    enhanced = clahe.apply(gray)
    
    # 밝기 임계값 설정 (상위 5% 픽셀)
    bright_thresh = np.percentile(enhanced, 95)
    
    # 밝은 영역 마스크
    bright_mask = cv2.threshold(enhanced, bright_thresh, 255, cv2.THRESH_BINARY)[1]
    
    # 노이즈 제거
    bright_mask = cv2.medianBlur(bright_mask, 5)
    bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), 1)
    
    return bright_mask


def brightness_based_white_detection(bgr: np.ndarray) -> np.ndarray:
    """밝기 기반 흰색 링 감지 (특수 조명 환경용)"""
    # 그레이스케일 변환
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    
    # 히스토그램 균등화로 대비 향상
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
    enhanced = clahe.apply(gray)
    
    # 밝기 임계값 설정 (상위 20% 픽셀)
    bright_thresh = np.percentile(enhanced, 80)
    
    # 밝은 영역 마스크
    bright_mask = cv2.threshold(enhanced, bright_thresh, 255, cv2.THRESH_BINARY)[1]
    
    # 모폴로지 연산으로 링 모양 강화
    bright_mask = cv2.medianBlur(bright_mask, 7)
    bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8), 2)
    bright_mask = cv2.morphologyEx(bright_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), 2)
    
    return bright_mask




def preprocess_white_ring_detection(bgr: np.ndarray, redmask: np.ndarray, 
                                  blue_hmin: int = 70, blue_hmax: int = 140,
                                  blue_smin: int = 10, blue_vmin: int = 30,
                                  blue_dilate: int = 4, white_cont_each: float = 0.85,
                                  blue_cont: float = 0.9, ri_ro_min: float = 0.18,
                                  ri_ro_max: float = 0.70, min_white_cover: float = 0.5):
    """컨투어+Hough로 흰색 링 감지 및 안정성 강화"""
    white_cont_list = [white_cont_each, 0.65, 0.55]
    blue_cont_list = [blue_cont, 0.7, 0.6, 0.5]
    blue_dilate_list = [blue_dilate, 6, 8]
    donut = None
    
    for wc in white_cont_list:
        for bc in blue_cont_list:
            for bd in blue_dilate_list:
                donut, _, _, _ = find_ring_for_redmask_by_contour(
                    bgr, redmask,
                    blue_hmin=blue_hmin, blue_hmax=blue_hmax,
                    blue_smin=blue_smin, blue_vmin=blue_vmin,
                    blue_dilate=bd, white_cont_each=wc,
                    blue_cont=bc, ri_ro_min=ri_ro_min,
                    ri_ro_max=ri_ro_max, min_white_cover=min_white_cover,
                )
                if donut:
                    break
            if donut:
                break
        if donut:
            break
    
    if donut is None:
        donut = hough_detect_ring(
            bgr, redmask,
            min_radius_ratio=0.03, max_radius_ratio=0.45,  # 더 넓은 범위
            annulus_inner_ratio=0.25, whiteness_thresh=0.3, param2=15,  # 더 관대한 임계값
        )
    
    return donut


def preprocess_distance_filtering(redmask: np.ndarray, cx: int, cy: int, 
                                max_vx: int = 150, max_vy: int = 200) -> np.ndarray:
    """중심점 기준 거리 제한으로 잘못된 빨간 점 제거"""
    return filter_red_points_by_distance(redmask, cx, cy, max_vx, max_vy)


# ==================== 간단한 감지 함수들 ====================

def find_white_ring_new(bgr: np.ndarray):
    """새로운 방법으로 흰색 링 찾기"""
    ring = robust_find_ring(bgr) if FORCE_ROBUST else None
    if ring is None and DISABLE_FALLBACK:
        if DBG_LOG: print("[switch] robust failed and fallback disabled")
        return None
    return robust_find_ring(bgr)


def find_white_ring_old(bgr: np.ndarray, redmask_hint: np.ndarray, 
                       blue_hmin: int, blue_hmax: int, blue_smin: int, blue_vmin: int,
                       blue_dilate: int, white_cont_each: float, blue_cont: float, 
                       ri_ro_min: float, ri_ro_max: float, min_white_cover: float):
    """기존 방법으로 흰색 링 찾기"""
    return preprocess_white_ring_detection(
        bgr, redmask_hint, blue_hmin, blue_hmax, blue_smin, blue_vmin,
        blue_dilate, white_cont_each, blue_cont, ri_ro_min, ri_ro_max, min_white_cover
    )


def find_red_dot_new(bgr: np.ndarray, cx: int, cy: int, ri: float, ro: float):
    """새로운 방법으로 빨간 점 찾기 (링 안에서만)"""
    return detect_red_in_annulus(bgr, cx, cy, ri, ro)


def find_red_dot_old(bgr: np.ndarray, cx: int, cy: int, s_min: int, a_thr: int, rg_thr: float):
    """기존 방법으로 빨간 점 찾기"""
    redmask = preprocess_red_detection(bgr, s_min, a_thr, rg_thr)
    filtered_redmask = preprocess_distance_filtering(redmask, cx, cy)
    ys, xs = np.where(filtered_redmask > 0)
    if len(xs) == 0:
        return None
    return int(round(xs.mean())), int(round(ys.mean()))


def make_red_hint(bgr: np.ndarray, s_min: int, a_thr: int, rg_thr: float):
    """빨간 점 힌트 만들기 (링 찾을 때 사용)"""
    return preprocess_red_detection(bgr, s_min, a_thr, rg_thr)

# ==================== 메인 감지 파이프라인 ====================

def detect_vector_from_bgr(
    bgr: np.ndarray,
    *,
    s_min: int = 40,
    a_thr: int = 145,
    rg_thr: float = 1.3,
    blue_hmin: int = 70,
    blue_hmax: int = 140,
    blue_smin: int = 10,
    blue_vmin: int = 30,
    blue_dilate: int = 4,
    white_cont_each: float = 0.85,
    blue_cont: float = 0.9,
    ri_ro_min: float = 0.18,
    ri_ro_max: float = 0.70,
    min_white_cover: float = 0.5,
) -> Optional[Tuple[int, int, float, int, int, int, int]]:
    """
    빨간 점 찾기 (간단 버전)
    
    사용법: 주석 처리해서 원하는 방법만 사용하기
    """
    
    # 1단계: 흰색 링 찾기
    ring = None
    
    # 새로운 방법으로 링 찾기
    ring = find_white_ring_new(bgr)
    
    # 실패하면 기존 방법 사용 (이 줄 주석 처리하면 기존 방법 안 씀)
    if ring is None:
        red_hint = make_red_hint(bgr, s_min, a_thr, rg_thr)
        ring = find_white_ring_old(bgr, red_hint, blue_hmin, blue_hmax, blue_smin, blue_vmin,
                                  blue_dilate, white_cont_each, blue_cont, ri_ro_min, ri_ro_max, min_white_cover)
    
    if ring is None:
        return None
    
    (cx, cy), ro, ri = ring
    
    # 2단계: 빨간 점 찾기
    red_dot = None

    # 1) 새 방법(도넛 안) 시도
    red_dot = find_red_dot_new(bgr, int(cx), int(cy), float(ri), float(ro))

    # 2) 실패 → 기존 방법 시도
    if red_dot is None:
        red_dot = find_red_dot_old(bgr, int(cx), int(cy), s_min, a_thr, rg_thr)

    # 3) 둘 다 실패 → 약화(weak) 재시도: detect_red_in_annulus 완화 파라미터로 직접 호출
    if red_dot is None:
        print("[red] weak-pass retry")
        red_dot = detect_red_in_annulus(
            bgr, int(cx), int(cy), float(ri), float(ro),
            min_area=2, max_area=180,
            min_excess=10.0, min_sat=60,
            min_circularity=0.55, min_local_contrast=8.0,
            glare_sat=70, glare_v=235
        )

    if red_dot is None:
        return None

    
    dx, dy = red_dot
    
    # 3단계: 결과 계산
    vx = dx - int(cx)
    vy = dy - int(cy)
    angle = float((math.degrees(math.atan2(vx, -vy)) + 360.0) % 360.0)
    
    return vx, vy, angle, int(cx), int(cy), int(dx), int(dy)


def draw_vector_overlay(bgr, cx, cy, dx, dy, angle_deg):
    """이미지에 감지 결과를 시각화"""
    vis = bgr.copy()
    cv2.circle(vis, (int(cx), int(cy)), 6, (0, 255, 0), 2)
    cv2.circle(vis, (int(dx), int(dy)), 9, (0, 0, 255), 2)
    cv2.arrowedLine(vis, (int(cx), int(cy)), (int(dx), int(dy)),
                    (0, 0, 255), 2, tipLength=0.15)
    lines = [
        f"Center: ({int(cx)}, {int(cy)})",
        f"RedDot: ({int(dx)}, {int(dy)})",
        f"Angle:  {angle_deg:.2f} deg",
    ]
    box_x, box_y = 10, vis.shape[0] - (len(lines) * 26 + 18)
    (tw, _), _ = cv2.getTextSize(lines[0], cv2.FONT_HERSHEY_SIMPLEX, 0.9, 2)
    cv2.rectangle(vis, (box_x - 8, box_y - 12),
                  (box_x + tw + 300, box_y + len(lines) * 26 + 12),
                  (255, 255, 255), -1)
    for i, t in enumerate(lines):
        yy = box_y + i * 26
        cv2.putText(vis, t, (box_x, yy), cv2.FONT_HERSHEY_SIMPLEX,
                    0.9, (0, 0, 0), 2, cv2.LINE_AA)
    return vis


# ---------------------- ROS2 Node ---------------------- #

class RedDotDetectorNode(Node):
    def __init__(self):
        super().__init__('red_dot_detector')

        # tunables
        self.declare_parameter('s_min', 50)
        self.declare_parameter('a_thr', 140)
        self.declare_parameter('rg_thr', 1.25)
        self.declare_parameter('blue_hmin', 70)
        self.declare_parameter('blue_hmax', 140)
        self.declare_parameter('blue_smin', 10)
        self.declare_parameter('blue_vmin', 30)
        self.declare_parameter('blue_dilate', 4)
        self.declare_parameter('white_cont_each', 0.75)
        self.declare_parameter('blue_cont', 0.8)
        self.declare_parameter('ri_ro_min', 0.18)
        self.declare_parameter('ri_ro_max', 0.70)
        self.declare_parameter('min_white_cover', 0.35)

        # saving
        self.declare_parameter('save_results', True)
        self.declare_parameter('result_dir', os.path.join(os.path.expanduser('~'), 'keycode_ws', 'result_drum'))
        self.cv_bridge = CvBridge()

        # pubs/subs
        self.vector_pub = self.create_publisher(Vector3, '/hanyang/coupler/keycode_angle', 10)
        self.image_sub = self.create_subscription(Image, '/zivid/color/image_color',
                                                  self.image_callback, 10)

        # prepare result dir
        self.result_dir = Path(self.get_parameter('result_dir').get_parameter_value().string_value)
        self.result_dir.mkdir(parents=True, exist_ok=True)
        self.save_seq = 0

        self.get_logger().info('RedDotDetectorNode initialised and waiting for images.')

    def _get_params(self):
        """ROS2 파라미터를 딕셔너리로 변환"""
        ps = {p.name: p.value for p in self.get_parameters([
            's_min', 'a_thr', 'rg_thr', 'blue_hmin', 'blue_hmax',
            'blue_smin', 'blue_vmin', 'blue_dilate',
            'white_cont_each', 'blue_cont', 'ri_ro_min', 'ri_ro_max',
            'min_white_cover', 'save_results'
        ])}
        
        def val(k, cast=float):
            """파라미터 값 추출 및 타입 변환"""
            v = ps[k]
            for attr in ('double_value', 'integer_value', 'string_value', 'bool_value'):
                if hasattr(v, attr):
                    return cast(getattr(v, attr))
            return cast(v)
        
        return {
            's_min': int(val('s_min')),
            'a_thr': int(val('a_thr')),
            'rg_thr': float(val('rg_thr')),
            'blue_hmin': int(val('blue_hmin')),
            'blue_hmax': int(val('blue_hmax')),
            'blue_smin': int(val('blue_smin')),
            'blue_vmin': int(val('blue_vmin')),
            'blue_dilate': int(val('blue_dilate')),
            'white_cont_each': float(val('white_cont_each')),
            'blue_cont': float(val('blue_cont')),
            'ri_ro_min': float(val('ri_ro_min')),
            'ri_ro_max': float(val('ri_ro_max')),
            'min_white_cover': float(val('min_white_cover')),
            'save_results': bool(val('save_results', bool))
        }

    def image_callback(self, msg: Image) -> None:
        """이미지 메시지를 받아서 처리"""
        # BGR 이미지로 변환
        try:
            encoding = msg.encoding.lower() if hasattr(msg, 'encoding') else 'rgba8'
            if encoding == 'rgba8':
                img_rgba = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgba8')
                bgr = cv2.cvtColor(img_rgba, cv2.COLOR_RGBA2BGR)
            elif encoding == 'rgb8':
                img_rgb = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                bgr = cv2.cvtColor(img_rgb, cv2.COLOR_RGB2BGR)
            elif encoding == 'bgr8':
                bgr = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            else:
                cv_img = self.cv_bridge.imgmsg_to_cv2(msg)
                if cv_img.ndim == 3 and cv_img.shape[2] == 4:
                    bgr = cv2.cvtColor(cv_img, cv2.COLOR_RGBA2BGR)
                elif cv_img.ndim == 3 and cv_img.shape[2] == 3:
                    bgr = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
                else:
                    self.get_logger().warning('Unsupported image format; skip.')
                    return
        except Exception as e:
            self.get_logger().error(f'Image conversion failed: {e}')
            return

        p = self._get_params()
        res = detect_vector_from_bgr(
            bgr,
            s_min=p['s_min'], a_thr=p['a_thr'], rg_thr=p['rg_thr'],
            blue_hmin=p['blue_hmin'], blue_hmax=p['blue_hmax'],
            blue_smin=p['blue_smin'], blue_vmin=p['blue_vmin'],
            blue_dilate=p['blue_dilate'], white_cont_each=p['white_cont_each'],
            blue_cont=p['blue_cont'], ri_ro_min=p['ri_ro_min'],
            ri_ro_max=p['ri_ro_max'], min_white_cover=p['min_white_cover']
        )
        # save and/or publish
        stamp = msg.header.stamp if hasattr(msg, 'header') else None
        t_ms = int(stamp.sec * 1000 + stamp.nanosec / 1e6) if stamp else 0
        if p['save_results']:
            self.save_seq += 1
            base = f'{t_ms:013d}_{self.save_seq:06d}'
            raw_path = self.result_dir / f'{base}_raw.png'
            ann_path = self.result_dir / f'{base}_annotated.png'
            try:
                cv2.imwrite(str(raw_path), bgr)
            except Exception as e:
                self.get_logger().warn(f'Raw save failed: {e}')
            try:
                if res is not None:
                    vx, vy, angle, cx, cy, dx, dy = res
                    vis = draw_vector_overlay(bgr, cx, cy, dx, dy, angle)
                else:
                    vis = bgr.copy()
                    cv2.putText(vis, 'no detection', (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,0,255), 2, cv2.LINE_AA)
                cv2.imwrite(str(ann_path), vis)
            except Exception as e:
                self.get_logger().warn(f'Annotated save failed: {e}')

        if res is None:
            return

        #  7개 값 언패킹
        vx, vy, angle, cx, cy, dx, dy = res

        # publish vector
        vec = Vector3()
        vec.x = float(vx)
        vec.y = float(vy)
        vec.z = float(angle)
        self.vector_pub.publish(vec)
        self.get_logger().info(f'pub /hanyang/coupler/keycode_angle: vx={vx}, vy={vy}, angle={angle:.2f}°')


def main(args=None):
    rclpy.init(args=args)
    node = RedDotDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def save_detection_results():
    """이미지들의 감지 결과를 현재 시간 폴더에 저장"""
    # 현재 시간으로 폴더 생성
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    result_dir = Path(f'detection_results_{timestamp}')
    result_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"결과 저장 폴더: {result_dir}")
    
    # 1번부터 10번까지 모든 이미지 처리
    all_numbers = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10,11,12,13,14,15,16,17,18,19,20]
    success_count = 0
    
    # 결과 요약 파일 생성
    summary_file = result_dir / 'detection_summary.txt'
    
    with open(summary_file, 'w', encoding='utf-8') as f:
        f.write(f"감지 결과 요약 - {timestamp}\n")
        f.write("=" * 50 + "\n\n")
        
        for num in all_numbers:
            filename = f'0919_{num}.png'
            try:
                img = cv2.imread(filename)
                if img is not None:
                    result = detect_vector_from_bgr(img)
                    
                    if result is not None:
                        vx, vy, angle, cx, cy, dx, dy = result
                        # 감지 결과가 있는 이미지 생성
                        vis = draw_vector_overlay(img, cx, cy, dx, dy, angle)
                        success_count += 1
                        
                        # 요약 파일에 기록
                        f.write(f"{num:2d}번: SUCCESS - 각도 {angle:.2f}°, 중심({cx},{cy}), 빨간점({dx},{dy})\n")
                        print(f'{num:2d}번: SUCCESS - 각도 {angle:.2f}° → {result_dir}/{num}_detected.png')
                    else:
                        # 감지 실패한 경우
                        vis = img.copy()
                        cv2.putText(vis, 'NO DETECTION', (10, 40), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
                        f.write(f"{num:2d}번: FAILED - 감지 실패\n")
                        print(f'{num:2d}번: FAILED → {result_dir}/{num}_detected.png')
                    
                    # 결과 이미지 저장
                    output_path = result_dir / f'{num:02d}_detected.png'
                    cv2.imwrite(str(output_path), vis)
                    
                else:
                    f.write(f"{num:2d}번: ERROR - 이미지 로드 실패\n")
                    print(f'{num:2d}번: 이미지 로드 실패')
            except IndexError as e:
                f.write(f"{num:2d}번: ERROR - IndexError: {str(e)}\n")
                print(f'{num:2d}번: IndexError - {str(e)}')
            except Exception as e:
                f.write(f"{num:2d}번: ERROR - {type(e).__name__}: {str(e)}\n")
                print(f'{num:2d}번: 오류 - {type(e).__name__}: {str(e)}')
        
        f.write(f"\n총 {success_count}/{len(all_numbers)}개 이미지 감지 성공 ({success_count/len(all_numbers)*100:.1f}%)\n")
    
    print(f'\n총 {success_count}/{len(all_numbers)}개 이미지 감지 성공 ({success_count/len(all_numbers)*100:.1f}%)')
    print(f'결과가 {result_dir} 폴더에 저장되었습니다.')
    print(f'요약 파일: {summary_file}')
    return result_dir


if __name__ == '__main__':
    # 시간 폴더에 감지 결과 저장
    # save_detection_results()
    
    # 기존 ROS2 노드 실행
    main()