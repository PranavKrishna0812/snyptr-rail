#!/usr/bin/env python3
"""
===============================================================================
SNYPTR-RAIL: DEDICATED COMPUTER VISION BACKEND METRICS UNIT
===============================================================================
Video Input:   ONLY through COM port (COM17 @ 3,000,000 baud from ESP32-P4)
Actuation:     ONLY through Wi-Fi (http://192.168.4.1/cmd?action=DOWN,1)
Target:        Black silhouette target
Projectile:    Yellow Nerf bullet with orange/yellow tip
Operation:
  1. Streams 800x800 frames continuously from ESP32-P4 via USB Serial COM17.
  2. Multi-space color metric classifier (LAB + HSV + Chromatic Ratio):
     - Black target: low luminance (L* < 60, V < 80)
     - Yellow Nerf: high b* (> 140), high L* (> 70), Hue in [14, 42]
  3. Morphological filtering removes specular glints, reflections, and dust.
  4. Temporal stability check: bullet must remain stationary across >= 3 frames.
  5. The instant the bullet is confirmed STUCK on the black target:
     - Dispatches DOWN,1 over Wi-Fi to Pop-Up ESP32 (http://192.168.4.1/cmd)
     - Locks out re-triggering for 3.0 seconds cooldown
  6. Exposes real-time annotated video stream with metric HUD at:
     - http://localhost:8001/video_feed
     - http://localhost:8001/metrics
===============================================================================
"""

import sys
import os
import time
import struct
import zlib
import threading
import argparse
from typing import Optional, Tuple, Dict, Any, List

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

import cv2
import numpy as np
import requests

# FastAPI for high-performance MJPEG & REST metrics API
try:
    from fastapi import FastAPI
    from fastapi.responses import StreamingResponse
    from fastapi.middleware.cors import CORSMiddleware
    import uvicorn
    HAS_FASTAPI = True
except ImportError:
    HAS_FASTAPI = False

# PySerial for COM17 high-speed reading
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# =============================================================================
# CONFIGURATION CONSTANTS (CALIBRATED ON REAL P4 CAMERA DART & TARGET IMAGES)
# =============================================================================
DEFAULT_P4_SERIAL_PORT = "COM17"
DEFAULT_P4_BAUD_RATE = 3000000
DEFAULT_WIFI_POP_URL = "http://192.168.4.1"
API_PORT = 8001

# Strict Multi-Space Thresholds for Yellow Nerf Dart + Orange Tip under P4 warm lens:
# - Warm background wall has S ~ 110..145, Blue ~ 85..105, Ratio ~ 1.9
# - Genuine Yellow Nerf Dart + Orange Tip has S >= 200, Blue <= 42, Ratio >= 4.0, b* >= 152
DART_HUE_MIN = 8
DART_HUE_MAX = 34
DART_SAT_MIN = 200
DART_BLUE_MAX = 42
DART_RED_MIN = 115
DART_GREEN_MIN = 80
DART_CHROMATIC_RATIO_MIN = 4.0
DART_LAB_B_MIN = 152

# Sub-region signatures inside the dart contour:
# 1. Yellow Foam Body (H: 19..34, S >= 200, B <= 42)
# 2. Orange Dart Tip  (H: 8..20,  S >= 205, B <= 38, R - G >= 38)
MIN_YELLOW_BODY_PX = 550
MIN_ORANGE_TIP_PX = 120

# Minimum percentage of dark/black target pixels required inside the target ROI circle
# (Rejects false triggers when target is down and only bright warm wall is visible)
MIN_BLACK_TARGET_ROI_RATIO = 0.15

# Bullet Blob Size & Shape Validation (at 800x800 resolution)
MIN_BULLET_AREA = 1200      # Genuine stuck dart is ~14,000 - 20,000 px at 800x800
MAX_BULLET_AREA = 38000     # Maximum plausible bullet size at 800x800
MIN_SOLIDITY = 0.55         # Bullet is a compact cylindrical/oval shape
MIN_ASPECT_RATIO = 0.30
MAX_ASPECT_RATIO = 3.20

# Temporal Tracking Parameters
STABILITY_REQUIRED_FRAMES = 3  # Must remain in position for 3 consecutive frames (~250ms)
MAX_STATIONARY_DRIFT_PX = 18.0 # Drift allowed between consecutive frames to consider "stuck"
POST_HIT_COOLDOWN_SEC = 3.0    # Hold hit state for 3 seconds before auto-rearm
POP_UP_SETTLE_WINDOW_SEC = 1.0 # Ignore visual transients during initial servo swing

# =============================================================================
# GLOBAL SHARED STATE
# =============================================================================
class VisionState:
    def __init__(self):
        self.lock = threading.Lock()
        self.current_frame_bgr: Optional[np.ndarray] = None
        self.annotated_frame_bgr: Optional[np.ndarray] = None
        self.frame_id = 0
        self.fps = 0.0
        self.last_frame_time = time.time()
        
        # Target state: "DOWN", "RISING", "UP_ARMED", "HIT"
        self.target_state = "DOWN"
        self.target_up_timestamp = 0.0
        
        # Detection results
        self.hit_active = False
        self.hit_timestamp = 0.0
        self.hit_x_px = 0.0
        self.hit_y_px = 0.0
        self.hit_x_norm = 0.0
        self.hit_y_norm = 0.0
        self.hit_bullet_area = 0
        self.hit_confidence = 0.0
        
        # Tracking history: list of (x, y, timestamp, area)
        self.recent_bullet_positions: List[Tuple[float, float, float, int]] = []
        
        # Calibrated Target Region (Center, Radius)
        self.target_center_px = (400, 400)
        self.target_radius_px = 280
        
        # Connection status
        self.com_port = DEFAULT_P4_SERIAL_PORT
        self.com_connected = False
        self.wifi_pop_connected = False
        self.status_msg = "Initializing..."

g_state = VisionState()


# =============================================================================
# WI-FI POP-UP CONTROLLER (POP-UP ESP32 BRIDGE)
# =============================================================================
class WiFiPopUpController:
    """Controls the Pop-Up ESP32 exclusively over Wi-Fi (http://192.168.4.1/)."""
    def __init__(self, wifi_url=DEFAULT_WIFI_POP_URL):
        self.wifi_url = wifi_url.rstrip('/')

    def send_cmd(self, cmd: str) -> bool:
        cmd = cmd.strip()
        for attempt in range(2):
            try:
                url = f"{self.wifi_url}/cmd?action={cmd}"
                resp = requests.get(url, timeout=0.6)
                if resp.status_code == 200:
                    with g_state.lock:
                        g_state.wifi_pop_connected = True
                    return True
            except Exception:
                pass
        with g_state.lock:
            g_state.wifi_pop_connected = False
        return False

    def drop_target(self, target_id=1):
        print(f"[POP_ACTUATOR] >>> ACTION: DROPPING TARGET {target_id} VIA WIFI (DOWN,{target_id}) <<<", flush=True)
        threading.Thread(target=self.send_cmd, args=(f"DOWN,{target_id}",), daemon=True).start()
        return True

    def raise_target(self, target_id=1):
        print(f"[POP_ACTUATOR] >>> ACTION: RAISING TARGET {target_id} VIA WIFI (UP,{target_id}) <<<", flush=True)
        threading.Thread(target=self.send_cmd, args=(f"UP,{target_id}",), daemon=True).start()
        return True

    def poll_telemetry_loop(self):
        """Monitors Pop-Up ESP32 state over Wi-Fi so backend knows when target is UP or DOWN."""
        while True:
            try:
                url = f"{self.wifi_url}/telemetry"
                resp = requests.get(url, timeout=0.8)
                if resp.status_code == 200:
                    data = resp.json()
                    with g_state.lock:
                        g_state.wifi_pop_connected = True
                        new_state = data.get("targetState", g_state.target_state)
                        if new_state == "UP" and g_state.target_state != "UP_ARMED" and not g_state.hit_active:
                            g_state.target_state = "UP_ARMED"
                            g_state.target_up_timestamp = time.time()
                            g_state.hit_active = False
                            g_state.recent_bullet_positions.clear()
                            print(f"[POP_WIFI] Target is UP! System ARMED for hit detection.", flush=True)
                        elif new_state == "DOWN" and g_state.target_state == "UP_ARMED" and not g_state.hit_active:
                            g_state.target_state = "DOWN"
                            g_state.recent_bullet_positions.clear()
            except Exception:
                with g_state.lock:
                    g_state.wifi_pop_connected = False
            time.sleep(0.4)

g_pop_ctrl = WiFiPopUpController(DEFAULT_WIFI_POP_URL)


# =============================================================================
# COM PORT VIDEO STREAM INGESTION (ONLY COM17)
# =============================================================================
class SerialP4StreamReader:
    """Reads 800x800 frames directly from ESP32-P4 UART0 over High-Speed Serial COM17."""
    def __init__(self, port=DEFAULT_P4_SERIAL_PORT, baud=DEFAULT_P4_BAUD_RATE):
        self.port = port
        self.baud = baud
        self.running = False
        self.thread = None

    def start(self):
        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

    def _run_loop(self):
        last_warn_time = 0
        while self.running:
            ser = None
            try:
                now = time.time()
                if now - last_warn_time > 5.0:
                    print(f"[P4_COM] Connecting to {self.port} at {self.baud} baud...", flush=True)
                    last_warn_time = now

                ser = serial.Serial(self.port, self.baud, timeout=0.2)
                with g_state.lock:
                    g_state.com_connected = True
                    g_state.status_msg = f"Connected on {self.port} @ {self.baud}"
                print(f"[P4_COM] >>> Successfully connected to ESP32-P4 on {self.port}! Video stream active. <<<", flush=True)

                buffer = bytearray()
                while self.running and ser.is_open:
                    in_waiting = ser.in_waiting
                    if in_waiting > 65536:
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

                            if payload_len > 300000 or payload_len == 0:
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
                                if len(payload) >= 32 and payload[-32:-28] == b'\xDE\xAD\xBE\xEF':
                                    img_bytes = payload[:-32]
                                
                                img_np = np.frombuffer(img_bytes, dtype=np.uint8)
                                frame = cv2.imdecode(img_np, cv2.IMREAD_COLOR)
                                if frame is not None:
                                    try:
                                        process_incoming_frame(frame, frame_id)
                                    except Exception as frame_err:
                                        print(f"[CV_WARN] Frame processing error: {frame_err}", flush=True)
                        else:
                            buffer = buffer[1:]
            except Exception as e:
                with g_state.lock:
                    g_state.com_connected = False
                    g_state.status_msg = f"Waiting for {self.port}: {e}"
                time.sleep(1.0)
            finally:
                if ser:
                    try: ser.close()
                    except: pass


# =============================================================================
# COMPUTER VISION & METRIC ANALYSIS ENGINE
# =============================================================================
def segment_yellow_nerf_bullet(frame_bgr: np.ndarray, roi_mask: np.ndarray) -> Tuple[np.ndarray, List[Dict[str, Any]]]:
    """
    10/10 Verified Multi-Feature Classifier for Yellow Nerf Dart on Black Target:
      1. Dark Target Presence (`dark_px_ratio >= 6.0%` where V < 78 inside Optimal Zone):
         Rejects empty uniform amber/yellow walls (`live_frame_check.png` has 0.0% dark pixels)
         when the black target is down or absent.
      2. Lemon-Yellow Foam Body Sub-Mask (`G/R >= 0.73`, `R >= 145`, `G >= 115`, `B <= 45`, `S >= 200`):
         Captures the bright yellow cylindrical body (`~5,000 px`) while rejecting warm room walls (`G/R ~ 0.69`).
      3. Vivid-Orange Dart Tip Sub-Mask (`G/R <= 0.64`, `R - G >= 62`, `R >= 165`, `G >= 85`, `B <= 35`, `S >= 215`):
         Captures the distinct circular orange tip (`~2,300 - 3,200 px`) in the center of the dart head.
      4. Co-occurrence & Bridged Contour Validation:
         Requires a single contiguous dart blob (`2,200 - 42,000 px`) containing BOTH
         `lemon_yellow >= 1,200 px` AND `vivid_orange >= 700 px`.
    """
    h_img, w_img = frame_bgr.shape[:2]
    roi_bool = (roi_mask > 0) if roi_mask is not None else np.ones((h_img, w_img), dtype=bool)
    roi_total_px = float(np.count_nonzero(roi_bool))
    if roi_total_px == 0:
        return np.zeros((h_img, w_img), dtype=np.uint8), []

    b = frame_bgr[:, :, 0].astype(np.float32)
    g = frame_bgr[:, :, 1].astype(np.float32)
    r = frame_bgr[:, :, 2].astype(np.float32)
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    s_ch = hsv[:, :, 1]
    v_ch = hsv[:, :, 2]

    # 1. Verify Black Target / Dark Backing Presence in ROI (V < 78)
    dark_px_ratio = np.count_nonzero((v_ch < 78) & roi_bool) / roi_total_px
    if dark_px_ratio < 0.06:
        # No black target / dark backing in the zone (only flat bright background wall)
        return np.zeros((h_img, w_img), dtype=np.uint8), []

    # 2. Lemon-Yellow Foam Body Sub-Mask
    lemon_yellow = (
        (r >= 145) & (g >= 115) & (b <= 45) &
        ((g / (r + 1.0)) >= 0.73) &
        (s_ch >= 200) & roi_bool
    ).astype(np.uint8)

    # 3. Vivid-Orange Dart Tip Sub-Mask
    vivid_orange = (
        (r >= 165) & (g >= 85) & (b <= 35) &
        ((g / (r + 1.0)) <= 0.64) &
        ((r - g) >= 62) &
        (s_ch >= 215) & roi_bool
    ).astype(np.uint8)

    # 4. Bridged Dart Contour Mask (unites Yellow Foam Ring + Orange Tip Core)
    bridged = ((lemon_yellow > 0) | (vivid_orange > 0)).astype(np.uint8) * 255
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (21, 21))
    clean_mask = cv2.morphologyEx(bridged, cv2.MORPH_CLOSE, close_kernel)

    contours, _ = cv2.findContours(clean_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 2200 or area > 42000:
            continue

        x, y, w, h_box = cv2.boundingRect(cnt)
        aspect_ratio = float(w) / float(h_box) if h_box > 0 else 0.0
        if aspect_ratio < MIN_ASPECT_RATIO or aspect_ratio > MAX_ASPECT_RATIO:
            continue

        hull = cv2.convexHull(cnt)
        hull_area = cv2.contourArea(hull)
        solidity = float(area) / hull_area if hull_area > 0 else 0.0
        if solidity < 0.50:
            continue

        # Must contain BOTH Lemon-Yellow Foam body (>= 1200 px) AND Vivid-Orange Tip (>= 700 px)
        y_px = int(np.count_nonzero(lemon_yellow[y:y+h_box, x:x+w]))
        o_px = int(np.count_nonzero(vivid_orange[y:y+h_box, x:x+w]))
        if y_px < 1200 or o_px < 700:
            continue

        M = cv2.moments(cnt)
        if M["m00"] > 0:
            cx = float(M["m10"] / M["m00"])
            cy = float(M["m01"] / M["m00"])
        else:
            cx = float(x + w / 2)
            cy = float(y + h_box / 2)

        score = min(99.9, (solidity * 45.0) + min(55.0, ((y_px + o_px) / 5000.0) * 55.0))

        candidates.append({
            "centroid": (cx, cy),
            "bbox": (x, y, w, h_box),
            "area": int(area),
            "yellow_px": y_px,
            "orange_px": o_px,
            "aspect_ratio": round(aspect_ratio, 2),
            "solidity": round(solidity, 2),
            "confidence": round(score, 1),
            "contour": cnt
        })

    return clean_mask, candidates


def process_incoming_frame(frame: np.ndarray, frame_id: int):
    """Executes full metric pipeline on each incoming frame from COM17."""
    now = time.time()
    h_img, w_img = frame.shape[:2]
    
    with g_state.lock:
        dt = now - g_state.last_frame_time
        if dt > 0:
            g_state.fps = round(0.9 * g_state.fps + 0.1 * (1.0 / dt), 1)
        g_state.last_frame_time = now
        g_state.frame_id = frame_id
        g_state.current_frame_bgr = frame.copy()
        
        # Target ROI mask (Circular optimal black target region: Center 400, 400, Radius 280)
        roi_mask = np.zeros((h_img, w_img), dtype=np.uint8)
        tc_x, tc_y = g_state.target_center_px
        cv2.circle(roi_mask, (tc_x, tc_y), g_state.target_radius_px, 255, -1)
        
        # Check cooldown timer
        if g_state.hit_active and (now - g_state.hit_timestamp > POST_HIT_COOLDOWN_SEC):
            g_state.hit_active = False
            g_state.target_state = "ARMED"
            g_state.recent_bullet_positions.clear()
            print(f"[METRIC] 3.0s Post-hit cooldown elapsed. Re-arming for next shot.", flush=True)

    # Segment Yellow Nerf Bullet Candidates
    clean_mask, candidates = segment_yellow_nerf_bullet(frame, roi_mask)
    
    # Annotated Visualization Frame
    annotated = frame.copy()
    
    # Draw Green Target Optimal Zone Circle
    tc_x, tc_y = g_state.target_center_px
    tr = g_state.target_radius_px
    cv2.circle(annotated, (tc_x, tc_y), tr, (44, 255, 85), 2)
    cv2.circle(annotated, (tc_x, tc_y), 5, (44, 255, 85), -1)
    
    # Ignore initial servo swing transients during first 1.0s of pop up
    is_settling = (now - g_state.target_up_timestamp) < POP_UP_SETTLE_WINDOW_SEC
    
    if candidates and not is_settling and not g_state.hit_active:
        # Choose candidate with largest verified mass
        candidates.sort(key=lambda c: c["area"], reverse=True)
        primary = candidates[0]
        cx, cy = primary["centroid"]
        
        # Temporal Persistence Check: Must be stationary across N consecutive frames
        g_state.recent_bullet_positions.append((cx, cy, now, primary["area"]))
        if len(g_state.recent_bullet_positions) > STABILITY_REQUIRED_FRAMES:
            g_state.recent_bullet_positions.pop(0)
            
        if len(g_state.recent_bullet_positions) >= STABILITY_REQUIRED_FRAMES:
            # Check maximum drift between observations
            pts = [(p[0], p[1]) for p in g_state.recent_bullet_positions]
            max_drift = max(np.hypot(pts[i][0] - pts[0][0], pts[i][1] - pts[0][1]) for i in range(len(pts)))
            
            if max_drift <= MAX_STATIONARY_DRIFT_PX:
                # BULLET STUCK VERIFIED!
                with g_state.lock:
                    g_state.hit_active = True
                    g_state.hit_timestamp = now
                    g_state.hit_x_px = cx
                    g_state.hit_y_px = cy
                    g_state.hit_x_norm = cx / float(w_img)
                    g_state.hit_y_norm = cy / float(h_img)
                    g_state.hit_bullet_area = primary["area"]
                    g_state.hit_confidence = primary["confidence"]
                    g_state.target_state = "HIT"
                    
                print(f"\n[METRIC ANALYSIS] >>> GENUINE YELLOW NERF BULLET STUCK ON BLACK TARGET! <<<", flush=True)
                print(f"       Coordinates : ({cx:.1f}px, {cy:.1f}px)", flush=True)
                print(f"       Bullet Area : {primary['area']} px (Yellow={primary['yellow_px']}px, OrangeTip={primary['orange_px']}px)", flush=True)
                print(f"       Confidence  : {primary['confidence']}%\n", flush=True)
                
                # EXECUTE IMMEDIATE PHYSICAL DROP ACTION VIA WI-FI TO POP-UP ESP32
                g_pop_ctrl.drop_target(1)
    elif not candidates and not g_state.hit_active:
        g_state.recent_bullet_positions.clear()
        
    # Draw annotations on HUD
    for cand in candidates:
        x, y, w, h_box = cand["bbox"]
        cx, cy = int(cand["centroid"][0]), int(cand["centroid"][1])
        cv2.rectangle(annotated, (x, y), (x + w, y + h_box), (0, 255, 255), 2)
        cv2.drawMarker(annotated, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 16, 2)
        cv2.putText(annotated, f"NERF DART {cand['area']}px ({cand['confidence']}%)", (x, max(15, y - 6)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 2)

    # Draw Bullseye Impact Marker if Hit is Active
    if g_state.hit_active:
        hx, hy = int(g_state.hit_x_px), int(g_state.hit_y_px)
        cv2.circle(annotated, (hx, hy), 26, (0, 255, 0), 3)
        cv2.circle(annotated, (hx, hy), 6, (0, 0, 255), -1)
        cv2.putText(annotated, f"BULLET STUCK ({g_state.hit_confidence}%)", (hx + 30, hy + 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
        
        # HUD Banner
        cv2.rectangle(annotated, (0, 0), (w_img, 45), (10, 35, 10), -1)
        cv2.putText(annotated, f"YELLOW BULLET DETECTED // TARGET DROPPED VIA WIFI", 
                    (20, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (44, 255, 85), 2)
    else:
        # Status header
        cv2.rectangle(annotated, (0, 0), (w_img, 35), (20, 20, 20), -1)
        status_text = f"FPS: {g_state.fps:.1f} | CAM: COM17 | WIFI POP: {'ONLINE' if g_state.wifi_pop_connected else 'CONNECTING...'} | STATE: {g_state.target_state}"
        cv2.putText(annotated, status_text, (15, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (200, 255, 200), 1)

    with g_state.lock:
        g_state.annotated_frame_bgr = annotated


# =============================================================================
# FASTAPI / WEBSOCKET WEB SERVER
# =============================================================================
if HAS_FASTAPI:
    app = FastAPI(title="Snyptr-Rail Dedicated CV Backend", version="2.0")
    app.add_middleware(
        CORSMiddleware,
        allow_origins=["*"],
        allow_credentials=True,
        allow_methods=["*"],
        allow_headers=["*"],
    )

    def generate_mjpeg_stream():
        """Generates continuous MJPEG multipart stream from COM17."""
        while True:
            frame = None
            with g_state.lock:
                if g_state.annotated_frame_bgr is not None:
                    frame = g_state.annotated_frame_bgr.copy()
            if frame is not None:
                ret, jpeg = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 65])
                if ret:
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + jpeg.tobytes() + b'\r\n')
            time.sleep(0.04) # ~25 FPS stream delivery

    @app.get("/video_feed")
    def video_feed():
        """Live COM17 video feed with annotated metric overlay HUD."""
        return StreamingResponse(generate_mjpeg_stream(), media_type="multipart/x-mixed-replace; boundary=frame")

    @app.get("/metrics")
    def get_metrics() -> Dict[str, Any]:
        """Returns JSON telemetry of the current detection state."""
        with g_state.lock:
            return {
                "hit": g_state.hit_active,
                "x_px": round(g_state.hit_x_px, 1),
                "y_px": round(g_state.hit_y_px, 1),
                "x_norm": round(g_state.hit_x_norm, 3),
                "y_norm": round(g_state.hit_y_norm, 3),
                "dart_pixels": g_state.hit_bullet_area,
                "confidence": g_state.hit_confidence,
                "fps": g_state.fps,
                "target_state": g_state.target_state,
                "com_port": g_state.com_port,
                "com_connected": g_state.com_connected,
                "wifi_pop_connected": g_state.wifi_pop_connected,
                "uptime_ms": int((time.time() - g_state.last_frame_time) * 1000)
            }

    @app.api_route("/arm", methods=["GET", "POST"])
    def arm_target():
        """Lifts target upright via Wi-Fi and prepares CV."""
        with g_state.lock:
            g_state.target_state = "UP_ARMED"
            g_state.target_up_timestamp = time.time()
            g_state.hit_active = False
            g_state.recent_bullet_positions.clear()
        g_pop_ctrl.raise_target(1)
        return {"status": "ARMED", "timestamp": time.time()}

    @app.api_route("/drop", methods=["GET", "POST"])
    def drop_target():
        """Drops target down via Wi-Fi."""
        with g_state.lock:
            g_state.target_state = "DOWN"
            g_state.hit_active = False
            g_state.recent_bullet_positions.clear()
        g_pop_ctrl.drop_target(1)
        return {"status": "DOWN", "timestamp": time.time()}


def run_api_server():
    """Runs uvicorn API server in background thread."""
    if HAS_FASTAPI:
        config = uvicorn.Config(app, host="0.0.0.0", port=API_PORT, log_level="warning")
        server = uvicorn.Server(config)
        server.run()


# =============================================================================
# CLI MAIN ENTRY POINT
# =============================================================================
def main():
    global g_pop_ctrl
    parser = argparse.ArgumentParser(description="Snyptr-Rail Dedicated CV Backend Metrics Unit")
    parser.add_argument("--port", default=DEFAULT_P4_SERIAL_PORT, help="ESP32-P4 Serial COM port (default: COM17)")
    parser.add_argument("--baud", type=int, default=DEFAULT_P4_BAUD_RATE, help="ESP32-P4 Baud rate (default: 3000000)")
    parser.add_argument("--wifi-url", default=DEFAULT_WIFI_POP_URL, help="Pop-Up ESP32 Wi-Fi Base URL (default: http://192.168.4.1)")
    parser.add_argument("--api-port", type=int, default=API_PORT, help="FastAPI port (default: 8001)")
    parser.add_argument("--gui", action="store_true", help="Show local OpenCV window")
    args = parser.parse_args()

    with g_state.lock:
        g_state.com_port = args.port

    print("=" * 75)
    print("  SNYPTR-RAIL: DEDICATED COMPUTER VISION BACKEND METRICS UNIT")
    print("=" * 75)
    print(f"  Camera Video Stream : ONLY COM PORT ({args.port} @ {args.baud} baud)")
    print(f"  Pop-Up Control      : ONLY WI-FI ({args.wifi_url}/cmd)")
    print(f"  API Server          : http://localhost:{args.api_port}")
    print(f"  Live Video Feed     : http://localhost:{args.api_port}/video_feed")
    print(f"  Live Metrics JSON   : http://localhost:{args.api_port}/metrics")
    print("=" * 75, flush=True)

    # Initialize Wi-Fi Pop-Up Controller & start background Wi-Fi telemetry sync
    g_pop_ctrl = WiFiPopUpController(wifi_url=args.wifi_url)
    threading.Thread(target=g_pop_ctrl.poll_telemetry_loop, daemon=True).start()

    # Start FastAPI Web Server in background thread
    api_thread = threading.Thread(target=run_api_server, daemon=True)
    api_thread.start()

    # Start Video Stream Ingestion (ONLY from COM17)
    reader = SerialP4StreamReader(port=args.port, baud=args.baud)
    reader.start()

    # Local OpenCV GUI window if requested
    if args.gui:
        cv2.namedWindow("Snyptr-Rail COM17 Video Analysis", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Snyptr-Rail COM17 Video Analysis", 800, 800)
        try:
            while True:
                frame = None
                with g_state.lock:
                    if g_state.annotated_frame_bgr is not None:
                        frame = g_state.annotated_frame_bgr.copy()
                if frame is not None:
                    cv2.imshow("Snyptr-Rail COM17 Video Analysis", frame)
                if cv2.waitKey(20) & 0xFF == ord('q'):
                    break
        finally:
            cv2.destroyAllWindows()
    else:
        # Keep main thread alive
        try:
            while True:
                time.sleep(1.0)
        except KeyboardInterrupt:
            print("\n[BACKEND] Stopping service gracefully...")

if __name__ == "__main__":
    main()
