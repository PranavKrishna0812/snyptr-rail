import serial
import struct
import zlib
import sys
import time
import cv2
import numpy as np
import os
import asyncio
import websockets
import json
import threading

import argparse

g_ws_clients = set()
g_loop = None
g_active_target_id = None
# --- MULTI-TARGET PRESET CALIBRATION ENGINE (TARGETS 1 - 7) ---
CALIBRATION_FILE = "target_calibrations.json"
g_target_calibrations = {i: None for i in range(1, 8)}

def parse_target_index(target_id):
    """Parses target index integer (1 to 7) from string or int."""
    if isinstance(target_id, int):
        return target_id
    elif isinstance(target_id, str):
        if target_id.startswith("T-"):
            try:
                return int(target_id.split("-")[1])
            except:
                pass
        else:
            try:
                return int(target_id)
            except:
                pass
    return None

def send_calibration(corners, target_id=None):
    """Computes homography and transmits the 9 floats to the ESP32-P4."""
    global ser, g_H, g_H_inv, g_bg_warped, g_active_target_id
    
    t_id = target_id if target_id is not None else g_active_target_id
    t_idx = parse_target_index(t_id) if t_id else 1
    if not t_idx:
        t_idx = 1
    
    # Target physical coordinate space mapping (85mm x 78mm) for ESP32-P4
    dst_pts_mm = np.array([
        [0.0, 0.0],
        [85.0, 0.0],
        [85.0, 78.0],
        [0.0, 78.0]
    ], dtype="float32")
    
    # Target pixel coordinate space mapping (850px x 780px) for Python CV warping
    dst_pts_px = np.array([
        [0.0, 0.0],
        [850.0, 0.0],
        [850.0, 780.0],
        [0.0, 780.0]
    ], dtype="float32")
    
    # Compute 3x3 homography matrix for Python warping
    g_H, _ = cv2.findHomography(corners, dst_pts_px)
    g_H_inv = np.linalg.inv(g_H)
    
    # Save homography and corners as Master Calibration for all targets (1 to 7)
    for i in range(1, 8):
        g_target_calibrations[i] = {
            "corners": corners.copy(),
            "H": g_H.copy(),
            "H_inv": g_H_inv.copy()
        }
    save_target_calibrations_to_json()
    
    # Compute 3x3 homography matrix for ESP32-P4 (physical mm)
    H_mm, _ = cv2.findHomography(corners, dst_pts_mm)
    
    # Reset background subtraction since homography changed
    g_bg_warped = None
    g_bg_warped_bgr = None
    
    h_floats = H_mm.flatten()
    
    # Pack homography command: [0xCC, 0x01, 9 floats, XOR Checksum]
    payload = struct.pack('<9f', *h_floats)
    checksum = 0x01
    for byte in payload:
        checksum ^= byte
        
    cal_packet = struct.pack('<BB', 0xCC, 0x01) + payload + struct.pack('<B', checksum)
    if 'ser' in globals() and ser is not None:
        try:
            ser.write(cal_packet)
            ser.flush()
            print(f"[CALIBRATION] Homography coefficients for Target {t_idx} successfully sent to ESP32-P4 board!", flush=True)
        except Exception as ex:
            print(f"[CALIBRATION] Error sending homography packet: {ex}", flush=True)

def save_target_calibrations_to_json():
    """Persists all target calibration presets (1 to 7) to target_calibrations.json."""
    data = {}
    for t_idx, cal in g_target_calibrations.items():
        if cal is not None and "corners" in cal:
            data[str(t_idx)] = cal["corners"].tolist()
    try:
        with open(CALIBRATION_FILE, "w") as f:
            json.dump(data, f, indent=2)
        print(f"[CALIBRATION] Successfully saved target presets to {CALIBRATION_FILE}", flush=True)
    except Exception as e:
        print(f"[CALIBRATION] Failed to save {CALIBRATION_FILE}: {e}", flush=True)

def load_target_calibrations_from_json():
    """Loads pre-saved target calibration preset (master Target 4) and applies to all 7 targets on startup."""
    global g_calibrated_corners, g_H, g_H_inv
    if not os.path.exists(CALIBRATION_FILE):
        return
    try:
        with open(CALIBRATION_FILE, "r") as f:
            data = json.load(f)
        dst_pts_px = np.array([
            [0.0, 0.0],
            [850.0, 0.0],
            [850.0, 780.0],
            [0.0, 780.0]
        ], dtype="float32")
        
        # Use Target 4 (center) master preset if available, else first available target preset
        master_key = "4" if "4" in data else (list(data.keys())[0] if data else None)
        if master_key and master_key in data:
            corners = np.array(data[master_key], dtype="float32")
            H, _ = cv2.findHomography(corners, dst_pts_px)
            H_inv = np.linalg.inv(H)
            for i in range(1, 8):
                g_target_calibrations[i] = {
                    "corners": corners.copy(),
                    "H": H.copy(),
                    "H_inv": H_inv.copy()
                }
            g_calibrated_corners = corners.copy()
            g_H = H.copy()
            g_H_inv = H_inv.copy()
            print(f"[CALIBRATION] Loaded Master Calibration (Target {master_key}) and applied universally to ALL 7 targets!", flush=True)
    except Exception as e:
        print(f"[CALIBRATION] Error reading {CALIBRATION_FILE}: {e}", flush=True)

# Load existing presets on script start
load_target_calibrations_from_json()

async def ws_handler(websocket, path=None):
    global g_target_active, g_calibrated_corners, g_H, g_H_inv, g_bg_warped, g_state, g_impact_frames, g_active_target_id, servo_ser
    g_ws_clients.add(websocket)
    try:
        async for message in websocket:
            try:
                payload = json.loads(message)
                mtype = payload.get("type")
                if mtype == "TARGET_UP":
                    target_id = payload.get("targetId")
                    print(f"[WEBSOCKET] Target {target_id} popped up! Enabling detection and resetting background...", flush=True)
                    g_active_target_id = target_id
                    g_bg_warped = None
                    g_bg_warped_bgr = None
                    g_target_active = True
                    g_state = STATE_IDLE
                    g_impact_frames = []
                    g_target_up_time = time.time()
                    
                    t_idx = parse_target_index(target_id)
                    
                    # Auto-switch calibration matrix to this target's pre-saved preset (if available)
                    if t_idx and t_idx in g_target_calibrations and g_target_calibrations[t_idx] is not None:
                        preset = g_target_calibrations[t_idx]
                        g_calibrated_corners = preset["corners"]
                        g_H = preset["H"]
                        g_H_inv = preset["H_inv"]
                        send_calibration(g_calibrated_corners)
                        print(f"[CALIBRATION] Switched to pre-saved calibration preset for Target {t_idx}!", flush=True)
                    else:
                        print(f"[CALIBRATION] Target {t_idx} has no saved preset yet. Waiting for calibration...", flush=True)

                    # Physically raise the target (channels 0-6 correspond to Target 1 to 7)
                    if servo_ser and t_idx is not None and 1 <= t_idx <= 7:
                        try:
                            cmd_str = f"UP,{t_idx}\n"
                            servo_ser.write(cmd_str.encode())
                            servo_ser.flush()
                            print(f"[SERVO] Sent command '{cmd_str.strip()}' to servo controller", flush=True)
                        except Exception as ex:
                            print(f"[SERVO] Error writing raise command: {ex}", flush=True)
                            
                elif mtype == "TARGET_DOWN":
                    target_id = payload.get("targetId")
                    print(f"[WEBSOCKET] Target {target_id} lowered. Disabling hit detection.", flush=True)
                    g_target_active = False
                    g_state = STATE_IDLE
                    g_impact_frames = []
                    if g_active_target_id == target_id:
                        g_active_target_id = None
                        
                    # Robust target index parsing
                    t_idx = None
                    if isinstance(target_id, int):
                        t_idx = target_id
                    elif isinstance(target_id, str):
                        if target_id.startswith("T-"):
                            try:
                                t_idx = int(target_id.split("-")[1])
                            except:
                                pass
                        else:
                            try:
                                t_idx = int(target_id)
                            except:
                                pass

                    # Physically lower the target
                    if servo_ser and t_idx is not None and 1 <= t_idx <= 7:
                        try:
                            cmd_str = f"DOWN,{t_idx}\n"
                            servo_ser.write(cmd_str.encode())
                            servo_ser.flush()
                            print(f"[SERVO] Sent command '{cmd_str.strip()}' to servo controller", flush=True)
                        except Exception as ex:
                            print(f"[SERVO] Error writing lower command: {ex}", flush=True)
                            
                elif mtype == "TARGET_DOWN_ALL":
                    print("[WEBSOCKET] Lowering all targets.", flush=True)
                    g_target_active = False
                    g_state = STATE_IDLE
                    g_impact_frames = []
                    g_active_target_id = None
                    
                    if servo_ser:
                        try:
                            cmd_str = "DOWN,ALL\n"
                            servo_ser.write(cmd_str.encode())
                            servo_ser.flush()
                            print("[SERVO] Sent command 'DOWN,ALL' to servo controller", flush=True)
                        except Exception as ex:
                            print(f"[SERVO] Error writing lower all command: {ex}", flush=True)
            except Exception as e:
                print(f"[WEBSOCKET] Error parsing message: {e}", flush=True)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        g_ws_clients.discard(websocket)

async def main_ws():
    async with websockets.serve(ws_handler, "localhost", 8765):
        print("[WEBSOCKET] Server running on ws://localhost:8765", flush=True)
        await asyncio.Future()  # Keep server alive indefinitely

def start_ws_server():
    global g_loop
    g_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(g_loop)
    g_loop.run_until_complete(main_ws())

def broadcast_shot(is_hit, x_ratio, y_ratio):
    if g_loop is None:
        return
    payload = {
        "type": "SHOT",
        "isHit": bool(is_hit),
        "xRatio": float(x_ratio),
        "yRatio": float(y_ratio),
        "targetId": g_active_target_id
    }
    msg = json.dumps(payload)
    
    async def send_to_all():
        if g_ws_clients:
            await asyncio.gather(*[client.send(msg) for client in g_ws_clients], return_exceptions=True)
            
    asyncio.run_coroutine_threadsafe(send_to_all(), g_loop)

# Start WebSocket Server in daemon thread
ws_thread = threading.Thread(target=start_ws_server, daemon=True)
ws_thread.start()

# Reconfigure stdout to use UTF-8 to prevent cp1252 mapping crashes on Windows
sys.stdout.reconfigure(encoding='utf-8')

parser = argparse.ArgumentParser(description="ESP32-P4 & Servo Target Controller Bridge")
parser.add_argument("--port", type=str, default="COM15", help="Main COM port of ESP32-P4 camera board")
parser.add_argument("--baud", type=int, default=3000000, help="Baud rate for ESP32-P4 camera board connection")
parser.add_argument("--servo-port", type=str, default="COM4", help="COM port of target servo board")
parser.add_argument("--servo-baud", type=int, default=115200, help="Baud rate for target servo board")
args = parser.parse_args()

port = args.port
baud = args.baud
servo_port = args.servo_port
servo_baud = args.servo_baud

print(f"Opening port {port} at {baud} baud (default DTR/RTS)...", flush=True)
try:
    ser = serial.Serial(port, baud, timeout=0.001)
except Exception as e:
    print(f"Error: Could not open serial port {port}. Details: {e}", flush=True)
    sys.exit(1)

# Open servo serial port
print(f"Opening servo controller port {servo_port} at {servo_baud} baud...", flush=True)
try:
    servo_ser = serial.Serial(servo_port, servo_baud, timeout=0.1)
except Exception as e:
    print(f"Warning: Could not open servo serial port {servo_port}. Physical servo target popping will be disabled. Details: {e}", flush=True)
    servo_ser = None

print("Waiting for ESP32-P4 to boot...", flush=True)
time.sleep(2.0)

# Auto-transmit pre-saved calibration to ESP32-P4 board on startup
if g_calibrated_corners is not None:
    send_calibration(g_calibrated_corners)
    print("[CALIBRATION] Auto-transmitted saved calibration homography to ESP32-P4 on startup!", flush=True)

print("\n--- STANDALONE NERF TARGET MONITOR ACTIVE ---")
print("Controls:")
print("  Press 'c' - Force Manual Calibration (Click 4 corners: TL, TR, BR, BL)")
print("  Press 'a' - Reset and enable Auto-Calibration mode")
print("  Press 'r' - Reset target background frame reference manually")
print("  Press 'q' - Exit the application\n", flush=True)

buffer = bytearray()
fps_start_time = time.time()
fps_counter = 0
fps_display = 0.0

# Calibration state variables
g_calibrated_corners = None
g_H = None
g_H_inv = None
manual_mode = False
clicked_pts = []

# --- NERF TRACKING STATE MACHINE CONFIGURATION ---
STATE_IDLE = 0
STATE_IMPACT = 1
STATE_SETTLING = 2

g_state = STATE_IDLE
g_bg_warped = None
g_bg_warped_bgr = None
g_prev_warped = None
g_impact_frames = []
g_impact_start_time = 0
g_settle_start_time = 0
g_last_settle_check = 0

# Shot telemetry state
g_shot_info = None

# Target active tracking control (synchronized with Electron app popping states)
g_target_active = True
g_lost_card_counter = 0
card_check_counter = 0
g_laser_was_present = False
g_target_up_time = 0.0

def sort_corners(pts):
    """Sorts 4 vertices clockwise starting from Top-Left (TL, TR, BR, BL)."""
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]     # Top-Left
    rect[2] = pts[np.argmax(s)]     # Bottom-Right
    
    diff = np.diff(pts, axis=1).flatten()
    rect[1] = pts[np.argmin(diff)]  # Top-Right
    rect[3] = pts[np.argmax(diff)]  # Bottom-Left
    return rect

def detect_laser_hsv(warped_img):
    """
    Robustly detects an emissive red laser dot in HSV color space.
    Returns (wpx, wpy) in 850x780 warped space, or None if not found.
    """
    if warped_img is None:
        return None
        
    hsv = cv2.cvtColor(warped_img, cv2.COLOR_BGR2HSV)
    
    # Red laser hue ranges - require high saturation (S>=120) and brightness (V>=200)
    lower_red1 = np.array([0, 120, 200])
    upper_red1 = np.array([15, 255, 255])
    lower_red2 = np.array([160, 120, 200])
    upper_red2 = np.array([180, 255, 255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    red_mask = cv2.bitwise_or(mask1, mask2)
    
    # Restrict detection to inner target card area (40, 40) to (810, 710)
    h_img, w_img = warped_img.shape[:2]
    target_mask = np.zeros((h_img, w_img), dtype=np.uint8)
    cv2.rectangle(target_mask, (40, 40), (w_img - 40, h_img - 70), 255, -1)
    red_mask = cv2.bitwise_and(red_mask, target_mask)
    
    contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_pt = None
    best_area = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if 1.0 <= area <= 400:
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
    Returns (wpx, wpy) centroid of the Orange Rubber Tip (pin location) in warped space.
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
        
    # Create mask of the detected Nerf dart
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
        
    # If orange tip is not separately distinct, return centroid of the Nerf dart contour
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
    Restricts search area to (25, 60) .. (405, 910) to eliminate edge wall & bottom table shadows.
    Returns (wpx, wpy) centroid in 430x985 warped pixel space, or None if target is clean.
    """
    if warped_bgr is None or warped_gray is None or bg_warped_gray is None:
        return None
        
    h_img, w_img = warped_gray.shape[:2]
    
    # Restrict search area to inner paper target (25, 25) to (825, 755) out of 850x780
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
    
    # If camera exposure / hand shadow causes scattered contours across the card (>= 3 blobs),
    # auto-adapt background frame to current light level!
    valid_cnts = [cnt for cnt in contours_d if cv2.contourArea(cnt) >= 100]
    if len(valid_cnts) >= 3:
        g_bg_warped = warped_gray.copy()
        g_bg_warped_bgr = warped_bgr.copy()
        print("[SYSTEM] Camera exposure / lighting shift detected. Auto-adapted background frame.", flush=True)
        return None

    # Check total changed pixels. If < 250 pixels changed, target is 100% CLEAN!
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
        if area_d >= 250: # Solid minimum area for real physical object
            perimeter = cv2.arcLength(cnt_d, True)
            if perimeter > 0:
                compactness = (4 * np.pi * area_d) / (perimeter ** 2)
                # Require round/compact shape (compactness > 0.30) to reject edge lines
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



def auto_detect_target(img):
    """Detects the white center circle directly and calculates the 4 virtual corners of the square card."""
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Lowered threshold to 170 to reliably detect mobile screens and targets in normal room light
    _, thresh = cv2.threshold(blurred, 170, 255, cv2.THRESH_BINARY)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    best_circle = None
    min_dist_to_center = float('inf')
    
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # Expected area range for the white center circle in frame
        if 1000 < area < 150000:
            perimeter = cv2.arcLength(cnt, True)
            if perimeter > 0:
                circularity = 4 * np.pi * area / (perimeter ** 2)
                # Filter for circular shapes (circularity > 0.65)
                if circularity > 0.65:
                    (cx, cy), r = cv2.minEnclosingCircle(cnt)
                    if r > 15:
                        # Select the circle candidate closest to the center of the frame (400, 400)
                        dist_to_center = np.sqrt((cx - 400)**2 + (cy - 400)**2)
                        if dist_to_center < min_dist_to_center:
                            min_dist_to_center = dist_to_center
                            best_circle = (cx, cy, r)
                            
    if best_circle is not None:
        cx, cy, r = best_circle
        # Calculate virtual corners of the 50x50mm target card (2.5x larger than the 10mm radius circle)
        corners = np.array([
            [cx - 2.5 * r, cy - 2.5 * r],  # Top-Left
            [cx + 2.5 * r, cy - 2.5 * r],  # Top-Right
            [cx + 2.5 * r, cy + 2.5 * r],  # Bottom-Right
            [cx - 2.5 * r, cy + 2.5 * r]   # Bottom-Left
        ], dtype="float32")
        return corners
    return None

def write_to_sius_log(x_mm, y_mm):
    """Appends physical coordinates relative to center (0,0) to C:\\SIUS\\commserver.log."""
    log_dir = "C:\\SIUS"
    log_file = os.path.join(log_dir, "commserver.log")
    try:
        os.makedirs(log_dir, exist_ok=True)
        with open(log_file, "a", encoding="utf-8") as f:
            f.write(f"X: {x_mm:.2f} Y: {y_mm:.2f}\n")
            f.flush()
        print(f"[INTEGRATION] Sent to Electron dashboard: X={x_mm:.2f} mm, Y={y_mm:.2f} mm", flush=True)
    except Exception as e:
        print(f"[ERROR] Failed to write to C:\\SIUS\\commserver.log: {e}", flush=True)

def broadcast_target_state(target_id, is_active):
    global g_loop, g_ws_clients
    if g_loop is None:
        return
    payload = {
        "type": "TARGET_DOWN" if not is_active else "TARGET_UP",
        "targetId": target_id
    }
    msg = json.dumps(payload)
    
    async def send_to_all():
        if g_ws_clients:
            await asyncio.gather(*[client.send(msg) for client in g_ws_clients], return_exceptions=True)
            
    asyncio.run_coroutine_threadsafe(send_to_all(), g_loop)

def drop_active_target_physically():
    global g_target_active, g_active_target_id, servo_ser
    if not g_target_active:
        return
        
    t_id = g_active_target_id
    t_idx = None
    if isinstance(t_id, int):
        t_idx = t_id
    elif isinstance(t_id, str):
        if t_id.startswith("T-"):
            try:
                t_idx = int(t_id.split("-")[1])
            except:
                pass
        else:
            try:
                t_idx = int(t_id)
            except:
                pass
                
    g_target_active = False
    
    # Broadcast target state update to React frontend
    if t_id is not None:
        broadcast_target_state(t_id, False)
        
    # Send physical DOWN command to Arduino servo board
    if servo_ser and t_idx is not None and 1 <= t_idx <= 7:
        try:
            cmd_str = f"DOWN,{t_idx}\n"
            servo_ser.write(cmd_str.encode())
            servo_ser.flush()
            print(f"[SERVO] Auto-sent lower command '{cmd_str.strip()}' to drop target", flush=True)
        except Exception as ex:
            print(f"[SERVO] Error writing drop command: {ex}", flush=True)

def trigger_target_down_on_hit():
    """Drops the active physical target via servo controller if hit in the Green Zone."""
    print(f"[SYSTEM] Green Zone hit detected. Dropping target!", flush=True)
    drop_active_target_physically()

def process_shot_event(settled_gray, mask, settled_bgr=None):
    """Processes the impact history, calculates the coordinate geometrically, and updates the state."""
    global g_bg_warped, g_impact_frames, g_shot_info
    
    if not g_impact_frames:
        return
        
    # 1. Find the frame with maximum change (peak impact)
    peak_frame = max(g_impact_frames, key=lambda x: x[0])
    peak_count = peak_frame[0]
    peak_gray = peak_frame[1]
    peak_diff = peak_frame[2]
    peak_bgr = peak_frame[3] if len(peak_frame) > 3 else settled_bgr
    
    cx, cy = None, None
    
    # 2. Restrict peak_diff to inner card surface (40, 40) to (810, 710)
    inner_mask = np.zeros(peak_diff.shape[:2], dtype=np.uint8)
    cv2.rectangle(inner_mask, (40, 40), (810, 710), 255, -1)
    diff_masked = cv2.bitwise_and(peak_diff, inner_mask)
    
    # 3. Extract contours from difference mask (cancels out printed card graphics)
    contours, _ = cv2.findContours(diff_masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    # Select the LARGEST difference contour (the physical Nerf bullet)
    valid_cnts = [cnt for cnt in contours if cv2.contourArea(cnt) >= 150]
    if valid_cnts:
        best_cnt = max(valid_cnts, key=cv2.contourArea)
        
        # Check if Orange Tip exists inside this largest diff contour
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
            # Use centroid of the largest impact contour
            M = cv2.moments(best_cnt)
            if M["m00"] > 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                print(f"[TRACKER] Largest impact contour pinpointed at ({cx}, {cy}) (Area: {cv2.contourArea(best_cnt):.0f} px)", flush=True)
                
    if cx is None or cy is None:
        cx, cy = 425, 390
        
    # Clamp (cx, cy) to inner target card space (40, 40) to (810, 710)
    cx = max(40, min(810, cx))
    cy = max(40, min(710, cy))
        
    # 3. Project coordinates to target millimeters (10 px/mm scale)
    x_mm = cx / 10.0
    y_mm = cy / 10.0
    
    lx_rel_mm = x_mm - 42.5
    ly_rel_mm = -(y_mm - 39.0)
    
    distance = np.sqrt(lx_rel_mm**2 + ly_rel_mm**2)
    scoring_radius = max(0.0, distance - 2.25)
    
    score_ring = 0
    is_x = False
    if scoring_radius <= 0.25:
        score_ring = 10
        is_x = True
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
        
    # 4. Classify Zone based on physical distance (inside optimal green circle radius 38.0mm = 7.6cm diameter)
    is_hit = distance <= 38.0
        
    zone = 1 if is_hit else 2 # 1 = Green (HIT/OPTIMAL), 2 = Red (MISS/NON-OPTIMAL)
    
    # Calculate screen ratio coordinates for Web Dashboard (0.0 to 1.0)
    x_ratio = cx / 850.0
    y_ratio = cy / 780.0
    x_ratio = max(0.0, min(1.0, float(x_ratio)))
    y_ratio = max(0.0, min(1.0, float(y_ratio)))
    
    broadcast_shot(zone == 1, x_ratio, y_ratio)
    if zone == 1:
        trigger_target_down_on_hit()
    else:
        print(f"[SYSTEM] Red Zone miss ({distance:.1f} mm from center). Target stays UP.", flush=True)
    
    # 5. Check if the dart is permanently stuck (housekeeping)
    # Compare settled gray frame against original background
    final_diff = cv2.absdiff(settled_gray, g_bg_warped)
    _, final_diff_thresh = cv2.threshold(final_diff, 20, 255, cv2.THRESH_BINARY)
    final_diff_thresh = cv2.bitwise_and(final_diff_thresh, mask)
    final_change = np.sum(final_diff_thresh > 0)
    
    is_stuck = final_change > 800
    
    # Update global shot state
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
    
    # Console log
    zone_name = "GREEN ZONE - HIT" if zone == 1 else "RED ZONE - MISS"
    ring_str = "10X" if is_x else f"{score_ring}" if score_ring > 0 else "Miss"
    stuck_str = "Stuck" if is_stuck else "Bounced Off"
    print(f"\n >>> [SHOT DETECTED] X: {lx_rel_mm:+.2f} mm | Y: {ly_rel_mm:+.2f} mm | Score: {ring_str} | Zone: {zone_name} ({stuck_str})", flush=True)
    
    # Write to SIUS log file for Electron app integration
    write_to_sius_log(x_mm, y_mm)
    
    # 6. Housekeeping: Update background frame if stuck
    if is_stuck:
        g_bg_warped = settled_gray.copy()
        print("[HOUSEKEEPING] Dart stays stuck. Merged into target background frame.", flush=True)
        
    g_impact_frames = []

def mouse_callback(event, x, y, flags, param):
    """Manual calibration click listener."""
    global clicked_pts, g_calibrated_corners, manual_mode
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(clicked_pts) < 4:
            clicked_pts.append((x, y))
            t_idx = parse_target_index(g_active_target_id) if g_active_target_id else 1
            print(f"[CALIB] Target T-0{t_idx} Corner {len(clicked_pts)}/4 clicked at ({x}, {y})", flush=True)
            if len(clicked_pts) == 4:
                print(f"[CALIB] 4 corners clicked for Target T-0{t_idx}! Press 'f' to LOCK & SAVE, or 'r' to RESET/REDO.", flush=True)
                sorted_pts = sort_corners(np.array(clicked_pts, dtype="float32"))
                send_calibration(sorted_pts)
                g_calibrated_corners = sorted_pts
                manual_mode = False

# Pre-calculate Universal Gamma Correction table (gamma=1.2) to optimize frame loop CPU usage
gamma = 1.2
invGamma = 1.0 / gamma
gamma_table = np.array([((i / 255.0) ** invGamma) * 255 for i in np.arange(0, 256)]).astype("uint8")

try:
    cv2.namedWindow("ESP32-P4 Live Stream (Press 'q' to exit)", cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback("ESP32-P4 Live Stream (Press 'q' to exit)", mouse_callback)
    
    while True:
        # Automatic 5.0s timeout disabled - target stays up indefinitely until physical bullet impact
        # if g_target_active and g_target_up_time is not None:
        #     if time.time() - g_target_up_time > 5.0:
        #         print(f"[SYSTEM] 5-second timeout expired. Dropping target!", flush=True)
        #         drop_active_target_physically()
                
        try:
            in_waiting = ser.in_waiting
            if in_waiting > 40000:
                ser.reset_input_buffer()
                buffer.clear()
                print("[SYSTEM] High serial latency detected. Flushing input buffer to restore real-time stream.", flush=True)
                continue
                
            read_len = in_waiting if in_waiting > 0 else 1
            data = ser.read(read_len)
            if data:
                buffer.extend(data)
        except (serial.SerialException, OSError, Exception) as e:
            print(f"[SYSTEM_WARNING] Serial stream read error ({e}). Attempting auto-reconnect...", flush=True)
            time.sleep(0.5)
            try:
                ser.close()
                ser.open()
            except Exception:
                pass
            continue
            
        while len(buffer) >= 3:
            if buffer[0] == 0xAA and buffer[1] == 0x55 and buffer[2] == 0x01:
                if len(buffer) < 15:
                    break
                    
                header = buffer[:15]
                frame_id = struct.unpack('<I', header[3:7])[0]
                payload_len = struct.unpack('<I', header[7:11])[0]
                received_crc = struct.unpack('<I', header[11:15])[0]
                
                # Sanity check: JPEG payload from 800x800 camera should never exceed 250KB.
                # If length is impossible, discard the first sync byte and search for next frame header.
                if payload_len > 250000 or payload_len == 0:
                    print(f"[PARSER_WARNING] Invalid payload length detected: {payload_len}. Dropping sync header to resynchronize.", flush=True)
                    buffer = buffer[1:]
                    continue

                total_packet_len = 15 + payload_len
                if len(buffer) < total_packet_len:
                    break
                    
                payload = bytes(buffer[15:total_packet_len])
                calc_crc = zlib.crc32(payload) & 0xFFFFFFFF
                crc_status = "OK" if calc_crc == received_crc else f"ERROR"
                
                if calc_crc != received_crc:
                    print(f"[PARSER_WARNING] CRC check failed for frame {frame_id} (calculated: {calc_crc:08X}, received: {received_crc:08X}). Discarding corrupt packet.", flush=True)
                    buffer = buffer[total_packet_len:]
                    continue

                if calc_crc == received_crc:
                    try:
                        # Extract Steganographic target metadata from trailing 32 bytes
                        meta_extracted = False
                        
                        img_payload = payload
                        if len(payload) >= 32:
                            meta_bytes = payload[-32:]
                            magic = meta_bytes[:4]
                            if magic == b'\xDE\xAD\xBE\xEF':
                                meta_extracted = True
                                img_payload = payload[:-32] # Strip metadata to get clean JPEG
                                
                        img_np = np.frombuffer(img_payload, dtype=np.uint8)
                        img = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
                        
                        laser_hit_triggered = False
                        if meta_extracted:
                            try:
                                magic_sig, laser_found, zone, score_ring, is_calibrated_flag, lx_px, ly_px, lx_mm, ly_mm, l_dist, _ = struct.unpack(
                                    '<4sBBBBHHfff8s', meta_bytes
                                )
                                # Hardware laser detection fallback (if Python HSV tracker fails, use firmware detection)
                                if laser_found == 1 and not laser_hit_triggered:
                                    if not g_laser_was_present:
                                        # Convert absolute [0.0, 50.0] mm to relative [-25.0, 25.0] mm
                                        lx_rel_mm = lx_mm - 25.0
                                        ly_rel_mm = ly_mm - 25.0
                                        
                                        wpx = int(lx_mm * 800.0 / 50.0)
                                        wpy = int((50.0 - ly_mm) * 800.0 / 50.0)
                                        
                                        # Classify zone based on physical distance (inside green circle = hit)
                                        dist = np.sqrt(lx_rel_mm**2 + ly_rel_mm**2)
                                        is_hit = dist <= 12.5
                                        zone = 1 if is_hit else 2
                                        
                                        g_shot_info = {
                                            "x_mm": lx_rel_mm,
                                            "y_mm": ly_rel_mm,
                                            "dist": dist,
                                            "score_ring": score_ring,
                                            "is_x": (score_ring == 11),
                                            "zone": zone,
                                            "is_stuck": False,
                                            "pixel_coords": (wpx, wpy),
                                            "timestamp": time.time()
                                        }
                                        
                                        x_ratio = lx_mm / 50.0
                                        y_ratio = 1.0 - (ly_mm / 50.0)
                                        x_ratio = max(0.0, min(1.0, float(x_ratio)))
                                        y_ratio = max(0.0, min(1.0, float(y_ratio)))
                                        
                                        broadcast_shot(zone == 1, x_ratio, y_ratio)
                                        if zone == 1:
                                            trigger_target_down_on_hit()
                                            
                                        zone_name = "GREEN ZONE - HIT" if zone == 1 else "RED ZONE - HIT"
                                        print(f"\n >>> [HARDWARE LASER SHOT] X: {lx_rel_mm:+.2f} mm | Y: {ly_rel_mm:+.2f} mm | Score: {score_ring} | Zone: {zone_name}", flush=True)
                                        g_laser_was_present = True
                                    
                                    laser_hit_triggered = True
                                else:
                                    g_laser_was_present = False
                            except Exception as e:
                                print(f"[ERROR] Failed to unpack stego metadata: {e}", flush=True)
                        
                        if img is not None:
                            # 1. Dynamic Clamped Auto White Balance
                            b, g, r = cv2.split(img)
                            b_mean = np.mean(b)
                            g_mean = np.mean(g)
                            r_mean = np.mean(r)
                            if b_mean > 0 and g_mean > 0 and r_mean > 0:
                                k_b = np.clip(0.85 * (g_mean / b_mean), 0.90, 1.30)
                                k_r = np.clip(1.10 * (g_mean / r_mean), 1.00, 1.45)
                                b = cv2.convertScaleAbs(b, alpha=k_b)
                                r = cv2.convertScaleAbs(r, alpha=k_r)
                            img_balanced = cv2.merge([b, g, r])
                            
                            # Auto-calibration loop
                            if g_calibrated_corners is None and not manual_mode:
                                auto_corners = auto_detect_target(img_balanced)
                                if auto_corners is not None:
                                    g_calibrated_corners = auto_corners
                                    send_calibration(auto_corners)
                                    
                            # 2. Universal Gamma Correction (gamma=1.2) using pre-calculated table
                            img_gamma = cv2.LUT(img_balanced, gamma_table)
                            
                            # 3. Gentle Gaussian Denoising (3x3)
                            img_denoised = cv2.GaussianBlur(img_gamma, (3, 3), 0.5)
                            
                            # 4. Unsharp Masking
                            img_sharp = cv2.addWeighted(img_gamma, 1.25, img_denoised, -0.25, 0)
                            
                            # Periodically verify the target card is still visible in the camera view
                            # Commented out to prevent resetting the target lock when fingers, hands, or obstacles obstruct the camera view.
                            # if g_calibrated_corners is not None and not manual_mode:
                            #     card_check_counter += 1
                            #     if card_check_counter % 5 == 0:
                            #         current_corners = auto_detect_target(img_balanced)
                            #         if current_corners is None:
                            #             g_lost_card_counter += 1
                            #             if g_lost_card_counter >= 3:
                            #                 g_calibrated_corners = None
                            #                 g_bg_warped = None
                            #                 g_prev_warped = None
                            #                 g_state = STATE_IDLE
                            #                 g_impact_frames = []
                            #                 g_lost_card_counter = 0
                            #                 print("[SYSTEM] Target card removed from view. Resetting lock.", flush=True)
                            #         else:
                            #             g_lost_card_counter = 0
                            
                            # --- NERF TRACKING CV ENGINE ---
                            if g_calibrated_corners is not None and g_H is not None:
                                # Warp Target to 850x780 flat front view (10 px/mm)
                                warped = cv2.warpPerspective(img_balanced, g_H, (850, 780))
                                warped_gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)
                                warped_gray = cv2.GaussianBlur(warped_gray, (5, 5), 0)
                                
                                # Laser Pointer Real-Time Detection
                                laser_pt = None
                                if g_bg_warped is not None and (g_target_up_time is None or (time.time() - g_target_up_time > 1.2)):
                                    laser_pt = detect_laser_hsv(warped)
                                    
                                laser_hit_triggered = False
                                if laser_pt is not None:
                                    wpx, wpy = laser_pt
                                    x_mm = wpx / 10.0
                                    y_mm = wpy / 10.0
                                    lx_rel_mm = x_mm - 42.5
                                    ly_rel_mm = -(y_mm - 39.0)
                                    
                                    dist = np.sqrt(lx_rel_mm**2 + ly_rel_mm**2)
                                    is_hit = dist <= 38.0
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
                                        
                                        x_ratio = wpx / 850.0
                                        y_ratio = wpy / 780.0
                                        broadcast_shot(zone == 1, x_ratio, y_ratio)
                                        if zone == 1:
                                            trigger_target_down_on_hit()
                                        
                                        zone_name = "GREEN ZONE - HIT" if zone == 1 else "RED ZONE - MISS"
                                        print(f"\n >>> [LASER SHOT] X: {lx_rel_mm:+.2f} mm | Y: {ly_rel_mm:+.2f} mm | Zone: {zone_name}", flush=True)
                                        g_laser_was_present = True
                                    
                                    laser_hit_triggered = True
                                else:
                                    g_laser_was_present = False
                                
                                if g_bg_warped is None:
                                    # Wait 1.2 seconds for the physical target servo to raise the card fully and stop swinging
                                    if g_target_up_time is None or (time.time() - g_target_up_time > 1.2):
                                        g_bg_warped = warped_gray.copy()
                                        g_bg_warped_bgr = warped.copy()
                                        g_prev_warped = warped_gray.copy()
                                        print("[SYSTEM] Baseline Background initialized (Target stationary).", flush=True)
                                elif g_target_active and not laser_hit_triggered:
                                    # Global illumination normalization to cancel camera auto-exposure fluctuations
                                    mean_bg = np.mean(g_bg_warped)
                                    mean_curr = np.mean(warped_gray)
                                    if mean_curr > 0 and abs(mean_curr - mean_bg) > 1.0:
                                        scale = mean_bg / mean_curr
                                        warped_gray = np.clip(warped_gray * scale, 0, 255).astype(np.uint8)

                                    diff = cv2.absdiff(warped_gray, g_bg_warped)
                                    _, diff_thresh = cv2.threshold(diff, 18, 255, cv2.THRESH_BINARY)
                                    
                                    # Mask to isolate inner target card region (40, 40) to (810, 710)
                                    mask = np.zeros((780, 850), dtype=np.uint8)
                                    cv2.rectangle(mask, (40, 40), (810, 710), 255, -1)
                                    diff_thresh = cv2.bitwise_and(diff_thresh, mask)
                                    
                                    change_pixel_count = np.sum(diff_thresh > 0)
                                    
                                    # Impact State Machine (threshold: 350 pixels to ignore edge/lighting noise)
                                    if g_state == STATE_IDLE:
                                        if change_pixel_count > 350:
                                            g_state = STATE_IMPACT
                                            g_impact_start_time = time.time()
                                            g_impact_frames = []
                                            print(f"[EVENT] Impact detected ({change_pixel_count} px change)! Tracking target...", flush=True)
                                            
                                    elif g_state == STATE_IMPACT:
                                        g_impact_frames.append((change_pixel_count, warped_gray.copy(), diff_thresh.copy(), warped.copy()))
                                        if time.time() - g_impact_start_time > 0.4:
                                            g_state = STATE_SETTLING
                                            g_settle_start_time = time.time()
                                            g_last_settle_check = time.time()
                                            
                                    elif g_state == STATE_SETTLING:
                                        if g_prev_warped is not None:
                                            frame_diff = cv2.absdiff(warped_gray, g_prev_warped)
                                            _, frame_diff_thresh = cv2.threshold(frame_diff, 15, 255, cv2.THRESH_BINARY)
                                            frame_diff_thresh = cv2.bitwise_and(frame_diff_thresh, mask)
                                            frame_diff_count = np.sum(frame_diff_thresh > 0)
                                            
                                            now = time.time()
                                            if frame_diff_count < 300:
                                                if now - g_last_settle_check > 0.2:
                                                    process_shot_event(warped_gray, mask, warped)
                                                    g_state = STATE_IDLE
                                            else:
                                                g_last_settle_check = now
                                                
                                            if now - g_settle_start_time > 1.5:
                                                print("[WARNING] Settle timeout, forcing classification...", flush=True)
                                                process_shot_event(warped_gray, mask, warped)
                                                g_state = STATE_IDLE
                                                
                                    g_prev_warped = warped_gray.copy()
                                    
                                # Draw optimal Green Zone circle on warped view (Center: 425, 390 | Radius: 380px = 38mm)
                                cv2.circle(warped, (425, 390), 380, (0, 255, 0), 2)
                                
                                # Draw hit crosshair on warped monitor
                                if g_shot_info is not None and time.time() - g_shot_info["timestamp"] < 5.0:
                                    wpx, wpy = g_shot_info["pixel_coords"]
                                    color = (0, 255, 0) if g_shot_info["zone"] == 1 else (0, 0, 255)
                                    cv2.drawMarker(warped, (wpx, wpy), color, cv2.MARKER_CROSS, 16, 2)
                                    
                                cv2.imshow("Warped Target View (Flat Monitor)", warped)
                            
                            # Calculate FPS
                            fps_counter += 1
                            fps_now = time.time()
                            if fps_now - fps_start_time >= 1.0:
                                fps_display = fps_counter / (fps_now - fps_start_time)
                                fps_counter = 0
                                fps_start_time = fps_now
                                
                            # --- DRAW INTERACTIVE OVERLAYS ON MAIN CAMERA STREAM ---
                            if g_calibrated_corners is not None:
                                # Draw calibrated outline (Green)
                                border_pts = g_calibrated_corners.astype(np.int32).reshape((-1, 1, 2))
                                cv2.polylines(img_sharp, [border_pts], True, (0, 255, 0), 2)
                                
                                # Draw circle center marker (425, 390) projected on camera stream
                                if g_H_inv is not None:
                                    center_pixel = cv2.perspectiveTransform(np.array([[[425.0, 390.0]]], dtype="float32"), g_H_inv)[0][0]
                                    cx, cy = int(center_pixel[0]), int(center_pixel[1])
                                    if 0 <= cx < 800 and 0 <= cy < 800:
                                        cv2.drawMarker(img_sharp, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
                                        
                                # Draw last shot marker projected back to camera space
                                if g_shot_info is not None and time.time() - g_shot_info["timestamp"] < 5.0:
                                    wpx, wpy = g_shot_info["pixel_coords"]
                                    cam_pixel = cv2.perspectiveTransform(np.array([[[float(wpx), float(wpy)]]], dtype="float32"), g_H_inv)[0][0]
                                    hcx, hcy = int(cam_pixel[0]), int(cam_pixel[1])
                                    
                                    color = (0, 255, 0) if g_shot_info["zone"] == 1 else (0, 0, 255)
                                    cv2.circle(img_sharp, (hcx, hcy), 8, color, -1)
                                    cv2.circle(img_sharp, (hcx, hcy), 16, color, 2)
                                    
                                    # Draw telemetry HUD box on upper right
                                    cv2.rectangle(img_sharp, (510, 15), (785, 195), (0, 0, 0), -1)
                                    cv2.rectangle(img_sharp, (510, 15), (785, 195), (255, 255, 255), 2)
                                    
                                    ring_str = "10X" if g_shot_info["is_x"] else f"{g_shot_info['score_ring']}" if g_shot_info['score_ring'] > 0 else "Miss"
                                    zone_str = "GREEN ZONE" if g_shot_info["zone"] == 1 else "RED ZONE"
                                    stuck_str = "STUCK" if g_shot_info["is_stuck"] else "BOUNCED"
                                    
                                    cv2.putText(img_sharp, f"RING: {ring_str}", (525, 55),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 3)
                                    cv2.putText(img_sharp, f"ZONE: {zone_str}", (525, 95),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                                    cv2.putText(img_sharp, f"STATE: {stuck_str}", (525, 130),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                                    cv2.putText(img_sharp, f"X: {g_shot_info['x_mm']:+.1f} | Y: {g_shot_info['y_mm']:+.1f}", (525, 165),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 2)
                            else:
                                if manual_mode:
                                    cv2.putText(img_sharp, "Manual Calibration: Click TL, TR, BR, BL", (15, 780),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                                    for idx, pt in enumerate(clicked_pts):
                                        cv2.circle(img_sharp, pt, 5, (0, 0, 255), -1)
                                        cv2.putText(img_sharp, str(idx+1), (pt[0]+8, pt[1]-8),
                                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                                else:
                                    cv2.putText(img_sharp, "AUTO-SCANNING TARGET CARD...", (15, 780),
                                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                                    
                                # Draw Target Active status overlay
                                status_str = "ACTIVE (LOCKED)" if g_target_active else "INACTIVE (DOWN)"
                                status_color = (0, 255, 0) if g_target_active else (0, 0, 255)
                                cv2.putText(img_sharp, f"TARGET: {status_str}", (15, 90),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, status_color, 2)
                                    
                            # Draw FPS
                            cv2.putText(img_sharp, f"FPS: {fps_display:.1f}", (15, 45), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 255, 255), 3)
                                        
                            cv2.imshow("ESP32-P4 Live Stream (Press 'q' to exit)", img_sharp)
                            
                            key = cv2.waitKey(1) & 0xFF
                            if key == ord('q'):
                                raise KeyboardInterrupt
                            elif ord('1') <= key <= ord('7'):
                                t_idx = key - ord('0')
                                g_active_target_id = f"T-0{t_idx}"
                                g_target_active = True
                                g_target_up_time = None  # Keep target UP during calibration setup
                                clicked_pts = []
                                manual_mode = True
                                
                                if t_idx in g_target_calibrations and g_target_calibrations[t_idx] is not None:
                                    g_calibrated_corners = g_target_calibrations[t_idx]["corners"].copy()
                                    g_H = g_target_calibrations[t_idx]["H"].copy()
                                    g_H_inv = g_target_calibrations[t_idx]["H_inv"].copy()
                                    send_calibration(g_calibrated_corners)
                                    print(f"[CALIB] Selected Target T-0{t_idx} (Existing calibration loaded). Click 4 corners to re-calibrate.", flush=True)
                                else:
                                    g_calibrated_corners = None
                                    g_H = None
                                    g_H_inv = None
                                    print(f"[CALIB] Selected Target T-0{t_idx} for Calibration. Click 4 corners (TL, TR, BR, BL) in stream.", flush=True)
                                
                                if servo_ser:
                                    try:
                                        cmd_str = f"UP,{t_idx}\n"
                                        servo_ser.write(cmd_str.encode())
                                        servo_ser.flush()
                                        print(f"[SERVO] Sent UP command for Target T-0{t_idx}", flush=True)
                                    except Exception as ex:
                                        print(f"[SERVO] Error writing raise command: {ex}", flush=True)
                            elif key == ord('f'):
                                t_idx = parse_target_index(g_active_target_id) if g_active_target_id else 1
                                if len(clicked_pts) == 4:
                                    sorted_pts = sort_corners(np.array(clicked_pts, dtype="float32"))
                                    send_calibration(sorted_pts)
                                    g_calibrated_corners = sorted_pts
                                    clicked_pts = []
                                    print(f"[CALIB] Locked & saved Target T-0{t_idx} calibration into target_calibrations.json!", flush=True)
                                elif g_calibrated_corners is not None:
                                    send_calibration(g_calibrated_corners)
                                    print(f"[CALIB] Re-locked Target T-0{t_idx} calibration!", flush=True)
                                else:
                                    print(f"[CALIB] Please click 4 corners first before pressing 'f' to lock!", flush=True)
                            elif key == ord('s'):
                                g_target_active = not g_target_active
                                g_calibrated_corners = None
                                g_bg_warped = None
                                g_state = STATE_IDLE
                                g_impact_frames = []
                                print(f"[SYSTEM] Manually toggled Target Active = {g_target_active}", flush=True)
                            elif key == ord('c'):
                                g_calibrated_corners = None
                                g_H = None
                                g_H_inv = None
                                clicked_pts = []
                                manual_mode = True
                                print("[SYSTEM] Entered Manual Calibration mode. Please click 4 corners.", flush=True)
                            elif key == ord('a'):
                                g_calibrated_corners = None
                                g_H = None
                                g_H_inv = None
                                clicked_pts = []
                                manual_mode = False
                                print("[SYSTEM] Auto-calibration enabled.", flush=True)
                            elif key == ord('r'):
                                if len(clicked_pts) > 0:
                                    clicked_pts = []
                                    print(f"[CALIB] Reset corner selections for active target. Click 4 corners again.", flush=True)
                                else:
                                    g_bg_warped = None
                                    g_bg_warped_bgr = None
                                    g_prev_warped = None
                                    g_state = STATE_IDLE
                                    g_impact_frames = []
                                    g_shot_info = None
                                    print("[SYSTEM] Target reference background frame reset.", flush=True)
                                
                    except Exception as e:
                        print(f"Frame processing error: {e}", flush=True)
                        
                buffer = buffer[total_packet_len:]
            else:
                buffer = buffer[1:]
                
except KeyboardInterrupt:
    print("\nStopping receiver script...", flush=True)
finally:
    ser.close()
    print("Port closed successfully.", flush=True)
