#!/usr/bin/env python3
"""
===============================================================================
SNYPTR-RAIL: HIGH-SPEED CAMERA LIVE FEED & TARGET EVENT MONITOR
===============================================================================
1. Streams real-time 800x800 camera feed directly from ESP32-P4 over USB (COM17 @ 3Mbaud).
2. Overlays the calibrated Optimal Target Zone (ROI Ellipse: Center 400,240, Rx=140, Ry=130).
3. Connects to Pop-Up Target ESP32 over Wi-Fi (http://192.168.4.1) for telemetry & commands.
4. Interactive Controls:
     [u] or [0]          : Pop Up & Arm Target (UP,1)
     [d]                 : Lower Target (DOWN,1)
     [`] (Backtick)      : Manual Hit Override (random sound announcement)
     [q] or [Esc]        : Exit Monitor
===============================================================================
"""

import sys
import time
import json
import random
import struct
import zlib
import urllib.request
import subprocess
import threading
import numpy as np

try:
    import cv2
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False

try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    import msvcrt
    HAS_MSVCRT = True
except ImportError:
    HAS_MSVCRT = False

DEFAULT_COM_PORT = "COM17"
DEFAULT_BAUD = 3000000
ESP32_WIFI_URL = "http://192.168.4.1"

# Target Optimal ROI Specifications (calibrated on upright black target)
ROI_CX = 400
ROI_CY = 240
ROI_RX = 140
ROI_RY = 130

HIT_PHRASES = [
    "Target hit! Bullseye!",
    "Confirmed hit on Target 1! Target down!",
    "Direct hit! Excellent shot!",
    "Target neutralized! Clean impact!",
    "Hit confirmed! 10X bullseye!",
    "Target down! Direct hit!",
    "Target 1 hit! Dropping target!"
]

def speak_phrase_async(text):
    """Speaks out loud using Windows SpeechSynthesizer without blocking."""
    try:
        clean_text = text.replace("'", "")
        cmd = f"Add-Type -AssemblyName System.Speech; (New-Object System.Speech.Synthesis.SpeechSynthesizer).Speak('{clean_text}')"
        subprocess.Popen(
            ["powershell", "-NoProfile", "-NonInteractive", "-Command", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            creationflags=0x08000000 # CREATE_NO_WINDOW
        )
    except Exception:
        pass

# =============================================================================
# LALITH689/CAMERA:ORANGE-TIP-TRACKING COMPUTER VISION & HIT DETECTION ENGINE
# Ported directly from Lalith689/camera branch: orange-tip-tracking (commit 24356c4)
# =============================================================================
OPTIMAL_CIRCLE_RADIUS_MM = 34.0  # 34.0 mm radius (6.8 cm diameter circle)
OPTIMAL_CIRCLE_RADIUS_PX = int(OPTIMAL_CIRCLE_RADIUS_MM * 10.0)  # 340 px radius

STATE_IDLE = 0
STATE_IMPACT = 1
STATE_SETTLING = 2

g_target_active = False
g_target_up_time = None
g_bg_warped = None
g_bg_warped_bgr = None
g_prev_warped = None
g_detector_fsm = STATE_IDLE
g_impact_start_time = None
g_impact_frames = []
g_settle_start_time = None
g_last_settle_check = None
g_shot_info = None
g_laser_was_present = False

def detect_orange_tip_hsv(warped_img):
    """
    Detects the bright Orange Rubber Tip of a Nerf bullet in HSV color space.
    Returns (wpx, wpy) centroid of the orange tip in image space, or None if not present.
    """
    if warped_img is None:
        return None
        
    hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
    h_img, w_img = warped_img.shape[:2]
    
    # Bright Vibrant Orange Tip HSV Range (Hue 4 to 22 / 165 to 180, Sat >= 110, Val >= 110)
    lower_orange1 = np.array([4, 110, 110])
    upper_orange1 = np.array([22, 255, 255])
    lower_orange2 = np.array([165, 110, 110])
    upper_orange2 = np.array([180, 255, 255])
    
    m1 = cv2.inRange(hsv, lower_orange1, upper_orange1)
    m2 = cv2.inRange(hsv, lower_orange2, upper_orange2)
    orange_mask = cv2.bitwise_or(m1, m2)
    
    # Restrict search area to inner target card area (40, 40) to (w_img - 40, h_img - 70)
    target_mask = np.zeros((h_img, w_img), dtype=np.uint8)
    cv2.rectangle(target_mask, (40, 40), (w_img - 40, h_img - 70), 255, -1)
    orange_mask = cv2.bitwise_and(orange_mask, target_mask)
    
    kernel3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    orange_mask = cv2.morphologyEx(orange_mask, cv2.MORPH_OPEN, kernel3)
    
    contours, _ = cv2.findContours(orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_cnt = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 80 <= area <= 25000:
            if area > best_area:
                best_area = area
                best_cnt = cnt
                
    if best_cnt is not None:
        M = cv2.moments(best_cnt)
        if M["m00"] > 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            return (cx, cy)
            
    return None

def detect_laser_hsv(warped_img):
    """
    Robustly detects an intensely bright red laser dot in HSV color space.
    Returns (wpx, wpy) in image space, or None if not found.
    """
    if warped_img is None:
        return None
        
    hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
    
    # Red laser hue ranges - require ultra high saturation (S>=180) and brightness (V>=220) to prevent table reflections
    lower_red1 = np.array([0, 180, 220])
    upper_red1 = np.array([12, 255, 255])
    lower_red2 = np.array([168, 180, 220])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    
    h_img, w_img = warped_img.shape[:2]
    target_mask = np.zeros((h_img, w_img), dtype=np.uint8)
    cv2.rectangle(target_mask, (40, 40), (w_img - 40, h_img - 70), 255, -1)
    red_mask = cv2.bitwise_and(red_mask, target_mask)
    
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_pt = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 1.0 <= area <= 300:
            if area > best_area:
                best_area = area
                M = cv2.moments(cnt)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    best_pt = (cx, cy)
                    
    return best_pt

def detect_nerf_dart_hsv(warped_img):
    """
    Unified Nerf Dart & Orange Tip Detector.
    Detects yellow/orange Nerf dart on the target card and pinpoints the Orange Rubber Tip.
    Returns (wpx, wpy) centroid of the Orange Rubber Tip (pin location) in image space.
    """
    if warped_img is None:
        return None
        
    hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
    h_img, w_img = warped_img.shape[:2]
    
    # 1. Unified Yellow/Gold/Orange Nerf Dart Range (Hue 3 to 42, Sat >= 35, Val >= 35)
    lower_nerf1 = np.array([3, 35, 35])
    upper_nerf1 = np.array([42, 255, 255])
    lower_nerf2 = np.array([160, 35, 35])
    upper_nerf2 = np.array([180, 255, 255])
    
    m1 = cv2.inRange(hsv, lower_nerf1, upper_nerf1)
    m2 = cv2.inRange(hsv, lower_nerf2, upper_nerf2)
    dart_mask = cv2.bitwise_or(m1, m2)
    
    target_mask = np.zeros((h_img, w_img), dtype=np.uint8)
    cv2.rectangle(target_mask, (15, 15), (w_img - 15, h_img - 15), 255, -1)
    dart_mask = cv2.bitwise_and(dart_mask, target_mask)
    
    contours, _ = cv2.findContours(dart_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_cnt = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 80 <= area <= 60000:
            if area > best_area:
                best_area = area
                best_cnt = cnt
                
    if best_cnt is None:
        return None
        
    dart_roi_mask = np.zeros((h_img, w_img), dtype=np.uint8)
    cv2.drawContours(dart_roi_mask, [best_cnt], -1, 255, -1)
    
    # 2. Pinpoint Orange Rubber Tip inside the dart (Hue 0..18 or 165..180)
    orange_only_mask = cv2.bitwise_or(
        cv2.inRange(hsv, np.array([0, 45, 45]), np.array([18, 255, 255])),
        cv2.inRange(hsv, np.array([165, 45, 45]), np.array([180, 255, 255]))
    )
    orange_in_dart = cv2.bitwise_and(orange_only_mask, dart_roi_mask)
    
    contours_o, _ = cv2.findContours(orange_in_dart, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_o_area = 0
    best_o_pt = None
    for cnt_o in contours_o:
        area_o = cv2.contourArea(cnt_o)
        if area_o > best_o_area:
            best_o_area = area_o
            M = cv2.moments(cnt_o)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                best_o_pt = (cx, cy)
                
    if best_o_pt is not None:
        return best_o_pt
        
    M = cv2.moments(best_cnt)
    if M["m00"] > 0:
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return (cx, cy)
        
    return None

def detect_universal_foreign_object(warped_bgr, warped_gray, bg_warped_bgr=None, bg_warped_gray=None):
    """
    Universal Foreign Object & Impact Detector (Difference-Gated).
    Uses frame difference relative to baseline background (bg_warped_gray) to detect
    ANY foreign object (Nerf bullet, pin, sticker, hole) appearing on the target.
    """
    global g_bg_warped, g_bg_warped_bgr
    if warped_bgr is None or warped_gray is None or bg_warped_gray is None:
        return None
        
    h_img, w_img = warped_gray.shape[:2]
    
    valid_mask = np.zeros((h_img, w_img), dtype=np.uint8)
    cv2.rectangle(valid_mask, (25, 25), (w_img - 25, h_img - 25), 255, -1)
    
    # 1. Global Illumination Normalization to cancel camera auto-exposure changes
    mean_bg = np.mean(bg_warped_gray)
    mean_curr = np.mean(warped_gray)
    if mean_curr > 0 and abs(mean_curr - mean_bg) > 1.0:
        scale = mean_bg / mean_curr
        norm_curr = np.clip(warped_gray * scale, 0, 255).astype(np.uint8)
    else:
        norm_curr = warped_gray
        
    # 2. Compute Difference relative to pristine background frame
    diff = cv2.absdiff(norm_curr, bg_warped_gray)
    _, thresh = cv2.threshold(diff, 28, 255, cv2.THRESH_BINARY)
    thresh = cv2.bitwise_and(thresh, valid_mask)
    
    # Clean noise with 5x5 Morphological Opening
    kernel5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel5)
    
    contours_d, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    valid_cnts = [cnt for cnt in contours_d if cv2.contourArea(cnt) >= 100]
    if len(valid_cnts) >= 3:
        g_bg_warped = warped_gray.copy()
        g_bg_warped_bgr = warped_bgr.copy()
        print("[SYSTEM] Camera exposure / lighting shift detected. Auto-adapted background frame.", flush=True)
        return None

    total_changed = np.sum(thresh > 0)
    if total_changed < 250:
        return None
        
    # 3. Check for High-Vibrancy Orange Rubber Tip INSIDE the changed region (Sat >= 75, Val >= 80)
    hsv = cv2.cvtColor(warped_bgr, cv2.COLOR_BGR2HSV)
    m_o1 = cv2.inRange(hsv, np.array([0, 75, 80]), np.array([18, 255, 255]))
    m_o2 = cv2.inRange(hsv, np.array([155, 75, 80]), np.array([180, 255, 255]))
    orange_mask = cv2.bitwise_or(m_o1, m_o2)
    orange_in_diff = cv2.bitwise_and(orange_mask, thresh)
    
    contours_o, _ = cv2.findContours(orange_in_diff, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_o_pt = None
    best_o_area = 0
    for cnt_o in contours_o:
        area_o = cv2.contourArea(cnt_o)
        if 25 <= area_o <= 20000:
            if area_o > best_o_area:
                best_o_area = area_o
                M = cv2.moments(cnt_o)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    best_o_pt = (cx, cy)
                    
    if best_o_pt is not None:
        return best_o_pt
        
    # 4. Extract centroid of single localized difference blob (Nerf bullet / foreign impact)
    best_d_pt = None
    best_d_score = 0
    for cnt_d in valid_cnts:
        area_d = cv2.contourArea(cnt_d)
        if area_d >= 250:
            perimeter = cv2.arcLength(cnt_d, True)
            if perimeter > 0:
                compactness = (4 * np.pi * area_d) / (perimeter ** 2)
                if compactness > 0.30:
                    score = area_d * compactness
                    if score > best_d_score:
                        best_d_score = score
                        M = cv2.moments(cnt_d)
                        if M["m00"] > 0:
                            cx = int(M["m10"] / M["m00"])
                            cy = int(M["m01"] / M["m00"])
                            best_d_pt = (cx, cy)
                            
    if best_d_pt is not None:
        return best_d_pt
        
    return None

def trigger_target_down_on_hit():
    """Drops the active physical target via servo controller if hit in the Green Zone."""
    with g_state.lock:
        perm = g_state.permanent_up
    if not perm:
        print(f"[SYSTEM] Green Zone hit detected. Dropping target!", flush=True)
        send_cmd("DOWN,1")
    else:
        print(f"[SYSTEM] Green Zone hit detected. Permanent UP mode active -> Target stays upright!", flush=True)

def process_shot_event(settled_gray, mask, settled_bgr=None):
    """Processes the impact history, calculates the coordinate geometrically, and triggers hit action."""
    global g_bg_warped, g_impact_frames, g_shot_info
    
    if not g_impact_frames:
        return
        
    peak_frame = max(g_impact_frames, key=lambda x: x[0])
    peak_count = peak_frame[0]
    peak_gray = peak_frame[1]
    peak_diff = peak_frame[2]
    peak_bgr = peak_frame[3] if len(peak_frame) > 3 else settled_bgr
    
    cx, cy = None, None
    h_img, w_img = peak_diff.shape[:2]
    inner_mask = np.zeros((h_img, w_img), dtype=np.uint8)
    cv2.rectangle(inner_mask, (40, 40), (w_img - 40, h_img - 70), 255, -1)
    diff_masked = cv2.bitwise_and(peak_diff, inner_mask)
    
    contours, _ = cv2.findContours(diff_masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid_cnts = [cnt for cnt in contours if cv2.contourArea(cnt) >= 150]
    if valid_cnts:
        best_cnt = max(valid_cnts, key=cv2.contourArea)
        if peak_bgr is not None:
            roi_mask = np.zeros(diff_masked.shape[:2], dtype=np.uint8)
            cv2.drawContours(roi_mask, [best_cnt], -1, 255, -1)
            
            hsv = cv2.cvtColor(peak_bgr, cv2.COLOR_BGR2HSV)
            m_o = cv2.bitwise_or(
                cv2.inRange(hsv, np.array([0, 50, 50]), np.array([20, 255, 255])),
                cv2.inRange(hsv, np.array([160, 50, 50]), np.array([180, 255, 255]))
            )
            orange_in_roi = cv2.bitwise_and(m_o, roi_mask)
            contours_o, _ = cv2.findContours(orange_in_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours_o:
                best_o = max(contours_o, key=cv2.contourArea)
                if cv2.contourArea(best_o) >= 15:
                    M_o = cv2.moments(best_o)
                    if M_o["m00"] > 0:
                        cx = int(M_o["m10"] / M_o["m00"])
                        cy = int(M_o["m01"] / M_o["m00"])
                        print(f"[TRACKER] Orange tip pinpointed inside impact blob at ({cx}, {cy})", flush=True)
                        
        if cx is None or cy is None:
            M = cv2.moments(best_cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print(f"[TRACKER] Largest impact contour pinpointed at ({cx}, {cy}) (Area: {cv2.contourArea(best_cnt):.0f} px)", flush=True)
                
    if cx is None or cy is None:
        cx, cy = w_img // 2, h_img // 2
        
    cx = max(40, min(w_img - 40, cx))
    cy = max(40, min(h_img - 70, cy))
        
    # Scale: 10 px/mm relative to target center
    lx_rel_mm = (cx - (w_img / 2.0)) / 10.0
    ly_rel_mm = -(cy - (h_img / 2.0)) / 10.0
    
    distance = np.sqrt(lx_rel_mm**2 + ly_rel_mm**2)
    scoring_radius = max(0.0, distance - 2.25)
    
    score_ring = 0
    is_x = False
    if scoring_radius <= 0.25:
        score_ring = 10; is_x = True
    elif scoring_radius <= 1.5:
        score_ring = 10
    elif scoring_radius <= 4.0:
        score_ring = 9
    elif scoring_radius <= 7.0:
        score_ring = 8
    elif scoring_radius <= 10.0:
        score_ring = 7
    elif scoring_radius <= 13.0:
        score_ring = 6
    elif scoring_radius <= 16.0:
        score_ring = 5
    elif scoring_radius <= 19.0:
        score_ring = 4
    elif scoring_radius <= 22.0:
        score_ring = 3
    elif scoring_radius <= 25.0:
        score_ring = 2
    elif scoring_radius <= 28.0:
        score_ring = 1
        
    is_hit = distance <= OPTIMAL_CIRCLE_RADIUS_MM
    zone = 1 if is_hit else 2
    
    # Check if dart stays stuck
    final_diff = cv2.absdiff(settled_gray, g_bg_warped)
    _, final_diff_thresh = cv2.threshold(final_diff, 20, 255, cv2.THRESH_BINARY)
    final_diff_thresh = cv2.bitwise_and(final_diff_thresh, mask)
    final_change = np.sum(final_diff_thresh > 0)
    is_stuck = final_change > 800
    
    g_shot_info = {
        "x_mm": lx_rel_mm,
        "y_mm": ly_rel_mm,
        "dist": distance,
        "score_ring": score_ring,
        "is_x": is_x,
        "zone": zone,
        "is_stuck": is_stuck,
        "pixel_coords": (cx, cy),
        "timestamp": time.time()
    }
    
    zone_name = "GREEN ZONE - HIT" if zone == 1 else "RED ZONE - MISS"
    ring_str = "10X" if is_x else f"{score_ring}" if score_ring > 0 else "Miss"
    stuck_str = "Stuck" if is_stuck else "Bounced Off"
    print(f"\n >>> [SHOT DETECTED] X: {lx_rel_mm:+.2f} mm | Y: {ly_rel_mm:+.2f} mm | Score: {ring_str} | Zone: {zone_name} ({stuck_str})", flush=True)
    
    if zone == 1:
        with g_state.lock:
            g_state.hit = True
            g_state.hit_x = cx
            g_state.hit_y = cy
            g_state.hit_pixels = peak_count
            g_state.confidence = 100
            g_state.last_hit_timestamp = time.time()

        trigger_target_down_on_hit()
        phrase = random.choice(HIT_PHRASES)
        speak_phrase_async(phrase)
    else:
        print(f"[SYSTEM] Red Zone miss ({distance:.1f} mm from center). Target stays UP.", flush=True)
        
    if is_stuck:
        g_bg_warped = settled_gray.copy()
        print("[HOUSEKEEPING] Dart stays stuck. Merged into target background frame.", flush=True)
        
    g_impact_frames = []

def process_incoming_frame_cv(frame):
    """
    Real-Time Nerf Orange Tip, Laser, and Difference Tracking Engine
    From Lalith689/camera:orange-tip-tracking.
    """
    global g_bg_warped, g_bg_warped_bgr, g_prev_warped, g_detector_fsm, g_impact_start_time
    global g_impact_frames, g_settle_start_time, g_last_settle_check
    global g_laser_was_present, g_target_up_time, g_target_active, g_shot_info

    if frame is None or not HAS_OPENCV:
        return

    h_img, w_img = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # 1. Real-Time Nerf Orange Tip & Laser Detection
    laser_pt = None
    if g_bg_warped is not None and (g_target_up_time is None or (time.time() - g_target_up_time > 1.2)):
        laser_pt = detect_orange_tip_hsv(frame)
        if laser_pt is None:
            laser_pt = detect_laser_hsv(frame)

    laser_hit_triggered = False
    if laser_pt is not None:
        wpx, wpy = laser_pt
        lx_rel_mm = (wpx - (w_img / 2.0)) / 10.0
        ly_rel_mm = -(wpy - (h_img / 2.0)) / 10.0

        dist = np.sqrt(lx_rel_mm**2 + ly_rel_mm**2)
        is_hit = dist <= OPTIMAL_CIRCLE_RADIUS_MM
        zone = 1 if is_hit else 2

        if not g_laser_was_present:
            g_shot_info = {
                "x_mm": lx_rel_mm,
                "y_mm": ly_rel_mm,
                "dist": dist,
                "score_ring": 10 if dist <= 10.0 else 8 if dist <= 20.0 else 5 if dist <= 30.0 else 1,
                "is_x": (dist <= 2.0),
                "zone": zone,
                "is_stuck": False,
                "pixel_coords": (wpx, wpy),
                "timestamp": time.time()
            }

            zone_name = "GREEN ZONE - HIT" if zone == 1 else "RED ZONE - MISS"
            print(f"\n >>> [NERF ORANGE TIP DETECTED] X: {lx_rel_mm:+.2f} mm | Y: {ly_rel_mm:+.2f} mm | Zone: {zone_name}", flush=True)

            if zone == 1:
                with g_state.lock:
                    g_state.hit = True
                    g_state.hit_x = wpx
                    g_state.hit_y = wpy
                    g_state.confidence = 100
                    g_state.last_hit_timestamp = time.time()

                trigger_target_down_on_hit()
                phrase = random.choice(HIT_PHRASES)
                speak_phrase_async(phrase)

            g_laser_was_present = True
        laser_hit_triggered = True
    else:
        g_laser_was_present = False

    # 2. Baseline Background Initialization (Target stationary for 1.2s)
    if g_bg_warped is None:
        if g_target_up_time is None or (time.time() - g_target_up_time > 1.2):
            g_bg_warped = gray.copy()
            g_bg_warped_bgr = frame.copy()
            g_prev_warped = gray.copy()
            with g_state.lock:
                g_state.detector_state = "ARMED"
            print("[SYSTEM] Baseline Background initialized (Target stationary).", flush=True)
    elif g_target_active and not laser_hit_triggered:
        # Global illumination normalization to cancel camera auto-exposure fluctuations
        mean_bg = np.mean(g_bg_warped)
        mean_curr = np.mean(gray)
        if mean_curr > 0 and abs(mean_curr - mean_bg) > 1.0:
            scale = mean_bg / mean_curr
            norm_gray = np.clip(gray * scale, 0, 255).astype(np.uint8)
        else:
            norm_gray = gray

        diff = cv2.absdiff(norm_gray, g_bg_warped)
        _, diff_thresh = cv2.threshold(diff, 18, 255, cv2.THRESH_BINARY)

        mask = np.zeros((h_img, w_img), dtype=np.uint8)
        cv2.rectangle(mask, (40, 40), (w_img - 40, h_img - 70), 255, -1)
        diff_thresh = cv2.bitwise_and(diff_thresh, mask)

        change_pixel_count = np.sum(diff_thresh > 0)
        with g_state.lock:
            g_state.delta_px = int(change_pixel_count)

        # Impact State Machine (threshold: 350 pixels to ignore edge/lighting noise)
        if g_detector_fsm == STATE_IDLE:
            if change_pixel_count > 350:
                g_detector_fsm = STATE_IMPACT
                g_impact_start_time = time.time()
                g_impact_frames = []
                print(f"[EVENT] Impact detected ({change_pixel_count} px change)! Tracking target...", flush=True)

        elif g_detector_fsm == STATE_IMPACT:
            g_impact_frames.append((change_pixel_count, gray.copy(), diff_thresh.copy(), frame.copy()))
            if time.time() - g_impact_start_time > 0.4:
                g_detector_fsm = STATE_SETTLING
                g_settle_start_time = time.time()
                g_last_settle_check = time.time()

        elif g_detector_fsm == STATE_SETTLING:
            if g_prev_warped is not None:
                frame_diff = cv2.absdiff(gray, g_prev_warped)
                _, frame_diff_thresh = cv2.threshold(frame_diff, 15, 255, cv2.THRESH_BINARY)
                frame_diff_thresh = cv2.bitwise_and(frame_diff_thresh, mask)
                frame_diff_count = np.sum(frame_diff_thresh > 0)

                now = time.time()
                if frame_diff_count < 300:
                    if now - g_last_settle_check > 0.2:
                        process_shot_event(gray, mask, frame)
                        g_detector_fsm = STATE_IDLE
                else:
                    g_last_settle_check = now

                if now - g_settle_start_time > 1.5:
                    print("[WARNING] Settle timeout, forcing classification...", flush=True)
                    process_shot_event(gray, mask, frame)
                    g_detector_fsm = STATE_IDLE

        g_prev_warped = gray.copy()

def send_cmd(action):
    global g_target_active, g_target_up_time, g_bg_warped, g_detector_fsm, g_impact_frames, g_laser_was_present

    if action.startswith("UP"):
        g_target_active = True
        g_target_up_time = time.time()
        g_bg_warped = None
        g_detector_fsm = STATE_IDLE
        g_impact_frames = []
        g_laser_was_present = False
        with g_state.lock:
            g_state.hit = False
            g_state.target_state = "UP"
            g_state.detector_state = "CALIBRATING"
    elif action.startswith("DOWN"):
        g_target_active = False
        g_bg_warped = None
        g_detector_fsm = STATE_IDLE
        g_impact_frames = []
        with g_state.lock:
            g_state.hit = False
            g_state.target_state = "DOWN"
            g_state.detector_state = "DOWN"

    # 1. Send immediately over wired USB Serial (Direct wire to P4 -> bridged to Pop ESP32)
    with g_state.lock:
        ser = g_state.serial_handle
    if ser and ser.is_open:
        try:
            ser.write(f"CMD:{action}\n".encode('ascii'))
            ser.flush()
        except Exception:
            pass

    # 2. Also dispatch over Wi-Fi in background thread so UI never blocks
    def _wifi_send():
        try:
            url = f"{ESP32_WIFI_URL}/cmd?action={action}"
            req = urllib.request.Request(url, headers={"User-Agent": "SnyptrLive/2.0"})
            with urllib.request.urlopen(req, timeout=1.0) as resp:
                pass
        except Exception:
            pass
    threading.Thread(target=_wifi_send, daemon=True).start()
    return "SENT"

class SharedMonitorState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_frame = None
        self.frame_id = 0
        self.fps = 0.0
        self.last_frame_time = 0.0
        self.serial_connected = False
        self.serial_port = DEFAULT_COM_PORT
        self.serial_handle = None

        # Telemetry & Hit State
        self.wifi_connected = False
        self.target_state = "DOWN"
        self.detector_state = "DOWN"
        self.hit = False
        self.hit_x = 400
        self.hit_y = 400
        self.hit_pixels = 0
        self.delta_px = 0
        self.color_px = 0
        self.span_x = 0
        self.span_y = 0
        self.confidence = 0
        self.latency_ms = 0.0
        self.last_hit_timestamp = 0.0
        self.permanent_up = False

        # Interactive Digital Zoom & Pan (1.0x to 4.0x)
        self.zoom = 1.0           # Current zoom factor
        self.zoom_cx = 400.0      # Pan center X (0 to 800)
        self.zoom_cy = 400.0      # Pan center Y (0 to 800)
        self.is_dragging = False
        self.drag_start_x = 0
        self.drag_start_y = 0
        self.drag_orig_cx = 400.0
        self.drag_orig_cy = 400.0

g_state = SharedMonitorState()

def adjust_zoom(delta, mouse_x=None, mouse_y=None):
    with g_state.lock:
        old_zoom = g_state.zoom
        new_zoom = round(max(1.0, min(4.0, old_zoom + delta)), 2)
        if mouse_x is not None and mouse_y is not None and new_zoom > 1.0:
            # Shift center towards mouse cursor in frame coordinates
            crop_w = 800.0 / old_zoom
            crop_h = 800.0 / old_zoom
            x1 = max(0.0, min(800.0 - crop_w, g_state.zoom_cx - crop_w / 2.0))
            y1 = max(0.0, min(800.0 - crop_h, g_state.zoom_cy - crop_h / 2.0))
            cursor_frame_x = x1 + (float(mouse_x) / 800.0) * crop_w
            cursor_frame_y = y1 + (float(mouse_y) / 800.0) * crop_h
            # Blend 40% towards cursor position
            g_state.zoom_cx = max(0.0, min(800.0, g_state.zoom_cx * 0.6 + cursor_frame_x * 0.4))
            g_state.zoom_cy = max(0.0, min(800.0, g_state.zoom_cy * 0.6 + cursor_frame_y * 0.4))
        g_state.zoom = new_zoom
        if new_zoom == 1.0:
            g_state.zoom_cx = 400.0
            g_state.zoom_cy = 400.0
    print(f"\n[🔍 ZOOM] Level: {new_zoom:.2f}x | Center: ({g_state.zoom_cx:.0f}, {g_state.zoom_cy:.0f})")

def reset_zoom():
    with g_state.lock:
        g_state.zoom = 1.0
        g_state.zoom_cx = 400.0
        g_state.zoom_cy = 400.0
    print("\n[🔍 ZOOM] Reset to 1.0x (Fit Full Frame)")

def pan_zoom(dx, dy):
    with g_state.lock:
        if g_state.zoom <= 1.0:
            return
        step = 45.0 / g_state.zoom
        g_state.zoom_cx = max(0.0, min(800.0, g_state.zoom_cx + dx * step))
        g_state.zoom_cy = max(0.0, min(800.0, g_state.zoom_cy + dy * step))

def on_mouse_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        with g_state.lock:
            g_state.is_dragging = True
            g_state.drag_start_x = x
            g_state.drag_start_y = y
            g_state.drag_orig_cx = g_state.zoom_cx
            g_state.drag_orig_cy = g_state.zoom_cy
    elif event == cv2.EVENT_MOUSEMOVE and (flags & cv2.EVENT_FLAG_LBUTTON):
        with g_state.lock:
            if g_state.is_dragging and g_state.zoom > 1.0:
                dx = x - g_state.drag_start_x
                dy = y - g_state.drag_start_y
                scale = 1.0 / g_state.zoom
                g_state.zoom_cx = max(0.0, min(800.0, g_state.drag_orig_cx - dx * scale))
                g_state.zoom_cy = max(0.0, min(800.0, g_state.drag_orig_cy - dy * scale))
    elif event == cv2.EVENT_LBUTTONUP:
        with g_state.lock:
            g_state.is_dragging = False
    elif event == cv2.EVENT_LBUTTONDBLCLK:
        reset_zoom()
    elif event == cv2.EVENT_MOUSEWHEEL:
        if flags > 0:
            adjust_zoom(0.25, x, y)
        else:
            adjust_zoom(-0.25, x, y)

def wifi_telemetry_thread_func():
    """Polls http://192.168.4.1/telemetry in background thread."""
    last_hit_reported = False
    while True:
        try:
            req = urllib.request.Request(f"{ESP32_WIFI_URL}/telemetry", headers={"User-Agent": "SnyptrLive/2.0"})
            with urllib.request.urlopen(req, timeout=0.8) as resp:
                data = json.loads(resp.read().decode('utf-8', errors='ignore'))
            
            with g_state.lock:
                g_state.wifi_connected = True
                g_state.hit = data.get("hit", False)
                g_state.target_state = data.get("targetState", "UNKNOWN")
                g_state.detector_state = data.get("detectorState", "DOWN")
                g_state.delta_px = data.get("deltaPx", 0)
                g_state.color_px = data.get("colorPx", 0)
                g_state.span_x = data.get("spanX", 0)
                g_state.span_y = data.get("spanY", 0)
                g_state.confidence = data.get("confidence", 0)
                g_state.latency_ms = data.get("latencyMs", 0.0)
                if "permanentUp" in data or "permUp" in data:
                    g_state.permanent_up = data.get("permanentUp", data.get("permUp", False))

                # Scale coordinates if hit reported
                if g_state.hit:
                    # x and y from telemetry are scaled to 160x160; scale up to 800x800
                    g_state.hit_x = int(data.get("x", 80) * (800 / 160))
                    g_state.hit_y = int(data.get("y", 48) * (800 / 160))
                    g_state.hit_pixels = data.get("dartPixels", 0)
                    if not last_hit_reported:
                        g_state.last_hit_timestamp = time.time()

            if g_state.hit and not last_hit_reported:
                phrase = random.choice(HIT_PHRASES)
                speak_phrase_async(phrase)
                last_hit_reported = True
            elif not g_state.hit:
                last_hit_reported = False

            time.sleep(0.08)
        except Exception:
            with g_state.lock:
                g_state.wifi_connected = False
            time.sleep(0.5)

def find_active_p4_port():
    """Detects available COM ports to find ESP32-P4."""
    if not HAS_SERIAL:
        return DEFAULT_COM_PORT
    ports = [p.device for p in serial.tools.list_ports.comports()]
    if DEFAULT_COM_PORT in ports:
        return DEFAULT_COM_PORT
    if ports:
        return ports[0]
    return DEFAULT_COM_PORT

def p4_serial_reader_thread_func():
    """Reads binary packet stream from ESP32-P4 UART0 over COM port."""
    if not HAS_SERIAL:
        return

    while True:
        port_to_try = find_active_p4_port()
        ser = None
        try:
            ser = serial.Serial(port_to_try, DEFAULT_BAUD, timeout=0.2)
            with g_state.lock:
                g_state.serial_connected = True
                g_state.serial_port = port_to_try
                g_state.serial_handle = ser
            print(f"[P4 COM] >>> Connected to ESP32-P4 on {port_to_try} @ {DEFAULT_BAUD} baud! Streaming camera feed... <<<")

            buffer = bytearray()
            last_fps_calc = time.time()
            fps_frame_counter = 0

            while ser.is_open:
                in_waiting = ser.in_waiting
                if in_waiting > 131072:
                    ser.reset_input_buffer()
                    buffer.clear()
                    continue

                chunk = ser.read(in_waiting if in_waiting > 0 else 1)
                if chunk:
                    buffer.extend(chunk)

                # Search for packet header: 0xAA 0x55 0x01
                while len(buffer) >= 15:
                    if buffer[0] == 0xAA and buffer[1] == 0x55 and buffer[2] == 0x01:
                        frame_id = struct.unpack('<I', buffer[3:7])[0]
                        payload_len = struct.unpack('<I', buffer[7:11])[0]
                        received_crc = struct.unpack('<I', buffer[11:15])[0]

                        if payload_len > 400000 or payload_len == 0:
                            buffer = buffer[1:]
                            continue

                        total_len = 15 + payload_len
                        if len(buffer) < total_len:
                            break # Wait for full packet

                        payload = bytes(buffer[15:total_len])
                        buffer = buffer[total_len:]

                        # Verify CRC32
                        calc_crc = zlib.crc32(payload) & 0xFFFFFFFF
                        if calc_crc == received_crc:
                            # Strip optional 32-byte metadata if appended at end
                            img_bytes = payload
                            hit_marker = False
                            if len(payload) >= 32 and payload[-32:-28] == b'\xDE\xAD\xBE\xEF':
                                img_bytes = payload[:-32]
                                hit_marker = (payload[-31] == 1)

                            if HAS_OPENCV:
                                np_arr = np.frombuffer(img_bytes, dtype=np.uint8)
                                decoded = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                                if decoded is not None:
                                    now = time.time()
                                    fps_frame_counter += 1
                                    if now - last_fps_calc >= 1.0:
                                        with g_state.lock:
                                            g_state.fps = round(fps_frame_counter / (now - last_fps_calc), 1)
                                        fps_frame_counter = 0
                                        last_fps_calc = now

                                    with g_state.lock:
                                        g_state.latest_frame = decoded
                                        g_state.frame_id = frame_id
                                        g_state.last_frame_time = now
                                        if hit_marker:
                                            g_state.hit = True

                                    # Run Real-Time Nerf Orange Tip & Laser Tracking Engine (Lalith689/camera)
                                    process_incoming_frame_cv(decoded)
                        continue
                    else:
                        buffer = buffer[1:]
        except Exception:
            with g_state.lock:
                g_state.serial_connected = False
            time.sleep(1.0)
        finally:
            if ser and ser.is_open:
                try:
                    ser.close()
                except Exception:
                    pass

def trigger_manual_hit_override():
    res = send_cmd("DOWN,1")
    phrase = random.choice(HIT_PHRASES)
    print(f"\n\033[92m[🎯 MANUAL OVERRIDE (`)] {phrase} | Target LOWERED (DOWN,1) -> {res}\033[0m")
    speak_phrase_async(phrase)

def toggle_permanent_up():
    with g_state.lock:
        new_state = not g_state.permanent_up
        g_state.permanent_up = new_state
    
    if new_state:
        send_cmd("PERM_UP,1")
        send_cmd("UP,1") # Raise target immediately
        status_str = "ENABLED (Target raised & stays up continuously)"
    else:
        send_cmd("PERM_UP,0")
        send_cmd("DOWN,1") # Lower target
        status_str = "DISABLED (Target lowered)"
        
    print(f"\n\033[93m[🎯 PERMANENT UP] Mode set to {status_str}\033[0m")

def render_tactical_canvas():
    """Draws a high-tech tactical HUD overlay on the camera frame or synthetic canvas."""
    with g_state.lock:
        frame = g_state.latest_frame.copy() if g_state.latest_frame is not None else None
        target_state = g_state.target_state
        detector_state = g_state.detector_state
        hit = g_state.hit
        hit_x = g_state.hit_x
        hit_y = g_state.hit_y
        hit_pixels = g_state.hit_pixels
        delta_px = g_state.delta_px
        color_px = g_state.color_px
        span_x = g_state.span_x
        span_y = g_state.span_y
        confidence = g_state.confidence
        latency_ms = g_state.latency_ms
        fps = g_state.fps
        serial_conn = g_state.serial_connected
        port = g_state.serial_port
        wifi_conn = g_state.wifi_connected
        perm_up = g_state.permanent_up
        zoom = g_state.zoom
        zoom_cx = g_state.zoom_cx
        zoom_cy = g_state.zoom_cy

    # If no physical camera frame received yet, create an 800x800 dark canvas
    if frame is None:
        frame = np.zeros((800, 800, 3), dtype=np.uint8)
        frame[:] = (18, 22, 18) # Dark tactical background
        
        # Grid lines
        for y in range(100, 800, 100):
            cv2.line(frame, (0, y), (800, y), (28, 36, 28), 1)
        for x in range(100, 800, 100):
            cv2.line(frame, (x, 0), (x, 800), (28, 36, 28), 1)

        cv2.putText(frame, "WAITING FOR CAMERA VIDEO STREAM (COM17 @ 3MBAUD)...",
                    (90, 380), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, "Connect ESP32-P4 USB cable & flash standalone firmware.",
                    (110, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (160, 180, 160), 1, cv2.LINE_AA)

    raw_frame_full = frame.copy()

    # Apply Interactive Digital Zoom (Crop & Scale)
    crop_w = int(800.0 / zoom)
    crop_h = int(800.0 / zoom)
    x1 = max(0, min(800 - crop_w, int(zoom_cx - crop_w / 2.0)))
    y1 = max(0, min(800 - crop_h, int(zoom_cy - crop_h / 2.0)))
    x2 = x1 + crop_w
    y2 = y1 + crop_h

    if zoom > 1.0:
        cropped = frame[y1:y2, x1:x2]
        frame = cv2.resize(cropped, (800, 800), interpolation=cv2.INTER_LINEAR)

    # 1. Full-Frame Active Target Region Corners (Corner Brackets)
    if hit:
        border_color = (0, 0, 255) # Bright Red on HIT
        status_label = "HIT CONFIRMED!"
        status_color = (0, 0, 255)
    elif target_state == "UP" and detector_state == "ARMED":
        border_color = (0, 255, 60) # High-visibility Green when ARMED
        status_label = "DETECTION ARMED (FULL FRAME)"
        status_color = (0, 255, 60)
    elif target_state == "UP":
        border_color = (0, 220, 255) # Yellow when CALIBRATING / RISING
        status_label = "CALIBRATING FULL FRAME"
        status_color = (0, 220, 255)
    else:
        border_color = (80, 80, 80) # Gray when DOWN
        status_label = "TARGET DOWN (IDLE)"
        status_color = (160, 160, 160)

    # Tactical corner brackets around active frame
    b_len = 40
    # Top-Left
    cv2.line(frame, (30, 80), (30 + b_len, 80), border_color, 2, cv2.LINE_AA)
    cv2.line(frame, (30, 80), (30, 80 + b_len), border_color, 2, cv2.LINE_AA)
    # Top-Right
    cv2.line(frame, (770, 80), (770 - b_len, 80), border_color, 2, cv2.LINE_AA)
    cv2.line(frame, (770, 80), (770, 80 + b_len), border_color, 2, cv2.LINE_AA)
    # Bottom-Left
    cv2.line(frame, (30, 720), (30 + b_len, 720), border_color, 2, cv2.LINE_AA)
    cv2.line(frame, (30, 720), (30, 720 - b_len), border_color, 2, cv2.LINE_AA)
    # Bottom-Right
    cv2.line(frame, (770, 720), (770 - b_len, 720), border_color, 2, cv2.LINE_AA)
    cv2.line(frame, (770, 720), (770, 720 - b_len), border_color, 2, cv2.LINE_AA)
    # Center reticle mark
    cv2.drawMarker(frame, (400, 400), (35, 50, 35), cv2.MARKER_CROSS, 24, 1)

    # Optimal Green Zone circle (from Lalith689/camera: radius 34.0mm = 340px)
    opt_center_x = int((400.0 - x1) * 800.0 / crop_w)
    opt_center_y = int((400.0 - y1) * 800.0 / crop_h)
    opt_radius = int(OPTIMAL_CIRCLE_RADIUS_PX * (800.0 / crop_w))
    cv2.circle(frame, (opt_center_x, opt_center_y), opt_radius, (0, 255, 60), 2, cv2.LINE_AA)

    # 2. Draw Hit / Detected Projectile Indicator
    if hit:
        screen_hx = int((hit_x - x1) * (800.0 / crop_w))
        screen_hy = int((hit_y - y1) * (800.0 / crop_h))
        if 0 <= screen_hx <= 800 and 0 <= screen_hy <= 800:
            circle_r = max(18, min(80, int(30 * zoom)))
            cv2.circle(frame, (screen_hx, screen_hy), circle_r, (0, 0, 255), 3, cv2.LINE_AA)
            cv2.circle(frame, (screen_hx, screen_hy), 6, (0, 200, 255), -1, cv2.LINE_AA)
            hit_label = f"IMPACT ({hit_x},{hit_y})"
            if g_shot_info is not None:
                ring_label = "10X" if g_shot_info.get("is_x") else f"Ring {g_shot_info.get('score_ring')}"
                hit_label += f" | {ring_label}"
            cv2.putText(frame, hit_label, (screen_hx + 35, screen_hy + 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2, cv2.LINE_AA)

    # 2b. Mini-Map Radar Thumbnail (Visible when Zoomed In)
    if zoom > 1.0:
        mm_size = 90
        thumb = cv2.resize(raw_frame_full, (mm_size, mm_size))
        rx1 = int(x1 * mm_size / 800.0)
        ry1 = int(y1 * mm_size / 800.0)
        rx2 = int(x2 * mm_size / 800.0)
        ry2 = int(y2 * mm_size / 800.0)
        cv2.rectangle(thumb, (rx1, ry1), (rx2, ry2), (0, 255, 60), 2)
        frame[630:720, 695:785] = thumb
        cv2.rectangle(frame, (695, 630), (785, 720), (0, 220, 255), 1)
        cv2.putText(frame, "RADAR", (698, 624), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 220, 255), 1, cv2.LINE_AA)

    # 3. Tactical Header Banner (Top HUD)
    cv2.rectangle(frame, (0, 0), (800, 64), (16, 20, 16), -1)
    cv2.line(frame, (0, 64), (800, 64), (45, 60, 45), 2)
    
    cv2.putText(frame, "SNYPTR-RAIL // REAL-TIME TARGET CV FEED",
                (18, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.70, (255, 255, 255), 2, cv2.LINE_AA)
    
    # State Pill
    cv2.rectangle(frame, (18, 36), (220, 56), (30, 36, 30), -1)
    cv2.rectangle(frame, (18, 36), (220, 56), status_color, 1)
    cv2.putText(frame, status_label, (26, 51),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, status_color, 1, cv2.LINE_AA)

    # Latency & FPS Badge
    perf_text = f"FPS: {fps:.1f} | Latency: {latency_ms:.1f}ms | UART: {port if serial_conn else 'WAITING'}"
    cv2.putText(frame, perf_text, (230, 51),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 255, 200) if serial_conn else (150, 150, 150), 1, cv2.LINE_AA)

    # Zoom Level Pill (Top HUD)
    zoom_color = (0, 255, 255) if zoom > 1.0 else (120, 150, 120)
    zoom_label = f"ZOOM: {zoom:.2f}x"
    cv2.rectangle(frame, (445, 36), (540, 56), (28, 36, 28), -1)
    cv2.rectangle(frame, (445, 36), (540, 56), zoom_color, 1)
    cv2.putText(frame, zoom_label, (451, 51),
                cv2.FONT_HERSHEY_SIMPLEX, 0.41, zoom_color, 1, cv2.LINE_AA)

    # Permanent Up Pill (Top Right)
    perm_color = (0, 255, 255) if perm_up else (80, 100, 80)
    perm_label = "PERM UP: [ON]" if perm_up else "PERM UP: [OFF]"
    cv2.rectangle(frame, (550, 36), (690, 56), (28, 36, 28), -1)
    cv2.rectangle(frame, (550, 36), (690, 56), perm_color, 1)
    cv2.putText(frame, perm_label, (558, 51),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, perm_color, 1, cv2.LINE_AA)

    # Wi-Fi badge
    wifi_color = (0, 255, 60) if wifi_conn else (0, 0, 255)
    cv2.circle(frame, (765, 30), 6, wifi_color, -1, cv2.LINE_AA)
    cv2.putText(frame, "WIFI" if wifi_conn else "NO-WIFI", (705, 34),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, wifi_color, 1, cv2.LINE_AA)

    # 4. Diagnostics Telemetry Bar (Bottom HUD)
    cv2.rectangle(frame, (0, 735), (800, 800), (16, 20, 16), -1)
    cv2.line(frame, (0, 735), (800, 735), (45, 60, 45), 2)

    diag_text1 = f"DELTA: {delta_px:>3d} px  |  COLOR: {color_px:>3d} px  |  SPAN: {span_x}x{span_y} px  |  CONF: {confidence}%"
    cv2.putText(frame, diag_text1, (18, 760),
                cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 240, 220), 1, cv2.LINE_AA)

    controls_text = "CONTROLS: [u] Up  |  [d] Down  |  [p] Perm Up  |  [+/-/Scroll] Zoom  |  [Drag] Pan  |  [0/r] Reset Zoom  |  [q] Quit"
    cv2.putText(frame, controls_text, (18, 785),
                cv2.FONT_HERSHEY_SIMPLEX, 0.40, (140, 180, 140), 1, cv2.LINE_AA)

    return frame

def main():
    print("=" * 78)
    print("      SNYPTR-RAIL: HIGH-SPEED CAMERA LIVE FEED & TARGET MONITOR")
    print("=" * 78)
    print(f"Connecting to ESP32-P4 Serial Stream on {DEFAULT_COM_PORT} @ {DEFAULT_BAUD} baud...")
    print(f"Connecting to Pop-Up ESP32 Wi-Fi Hotspot at {ESP32_WIFI_URL}...")
    print("Controls:")
    print("  [u]                 : Pop Up & Arm Target (UP,1)")
    print("  [d]                 : Lower Target (DOWN,1)")
    print("  [p]                 : Toggle Permanent Up Mode (Target stays upright on hit)")
    print("  [+] / [-] or [z/x]  : Zoom Camera IN / OUT (1.0x to 4.0x)")
    print("  [Mouse Scroll]      : Smooth Zoom In / Out at cursor position")
    print("  [Mouse Drag / WASD] : Pan camera view when zoomed in")
    print("  [0] or [r] / DblClk : Reset Zoom to 1.0x (Full Frame)")
    print("  [`] (Backtick)      : Manual Hit Override (random voice announcement)")
    print("  [q] or [Esc]        : Exit Monitor")
    print("=" * 78, flush=True)

    # Start Background Serial Video Reader
    threading.Thread(target=p4_serial_reader_thread_func, daemon=True).start()

    # Start Background Wi-Fi Telemetry Reader
    threading.Thread(target=wifi_telemetry_thread_func, daemon=True).start()

    if HAS_OPENCV:
        window_name = "SNYPTR-RAIL // Live Camera Feed"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 800, 800)
        cv2.setMouseCallback(window_name, on_mouse_event)

    try:
        while True:
            # Handle Windows Console Keypresses (non-blocking)
            if HAS_MSVCRT and msvcrt.kbhit():
                ch = msvcrt.getch()
                try:
                    ch_str = ch.decode('utf-8', errors='ignore').lower()
                except Exception:
                    ch_str = ""

                if ch_str in ['`', '~']:
                    trigger_manual_hit_override()
                elif ch_str in ['u']:
                    print("\n[CMD] Raising & Arming Target 1 (UP,1)...")
                    send_cmd("UP,1")
                elif ch_str == 'd':
                    print("\n[CMD] Lowering Target 1 (DOWN,1)...")
                    send_cmd("DOWN,1")
                elif ch_str == 'p':
                    toggle_permanent_up()
                elif ch_str in ['+', '=', 'z']:
                    adjust_zoom(0.25)
                elif ch_str in ['-', '_', 'x']:
                    adjust_zoom(-0.25)
                elif ch_str in ['0', 'r']:
                    reset_zoom()
                elif ch_str == 'q':
                    print("\nQuit requested.")
                    break

            # Render GUI Window
            if HAS_OPENCV:
                canvas = render_tactical_canvas()
                cv2.imshow(window_name, canvas)
                key = cv2.waitKey(20) & 0xFF
                if key == ord('q') or key == 27: # 'q' or ESC
                    print("\nQuit key pressed.")
                    break
                elif key in [ord('u'), ord('U')]:
                    print("\n[CMD] Raising & Arming Target 1 (UP,1)...")
                    send_cmd("UP,1")
                elif key in [ord('d'), ord('D')]:
                    print("\n[CMD] Lowering Target 1 (DOWN,1)...")
                    send_cmd("DOWN,1")
                elif key in [ord('p'), ord('P')]:
                    toggle_permanent_up()
                elif key in [ord('+'), ord('='), ord('z'), ord('Z')]:
                    adjust_zoom(0.25)
                elif key in [ord('-'), ord('_'), ord('x'), ord('X')]:
                    adjust_zoom(-0.25)
                elif key in [ord('0'), ord('r'), ord('R')]:
                    reset_zoom()
                elif key in [ord('w'), ord('W')]:
                    pan_zoom(0, -1)
                elif key in [ord('s'), ord('S')]:
                    pan_zoom(0, 1)
                elif key in [ord('a'), ord('A')]:
                    pan_zoom(-1, 0)
                elif key in [ord('e'), ord('E')]:
                    pan_zoom(1, 0)
                elif key in [ord('`'), ord('~'), ord(' ')]:
                    trigger_manual_hit_override()
            else:
                time.sleep(0.05)

    except KeyboardInterrupt:
        print("\nMonitor stopped.")
    finally:
        if HAS_OPENCV:
            cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
