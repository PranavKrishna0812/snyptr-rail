#!/usr/bin/env python3
"""
===============================================================================
SNYPTR-RAIL: HIGH-SPEED DETERMINISTIC EVENT DETECTION MONITOR
===============================================================================
Polls live target telemetry from the Pop-Up ESP32 over Wi-Fi (http://192.168.4.1).
Exposes real-time multi-signal detector debug metrics:
  - Current System State (DOWN, RISING, ARMED, HIT_LOCKED)
  - Optimal ROI & Reference ROI status
  - Signal 1: Reference Pixel Delta count vs threshold
  - Signal 2: Orange / Yellow Color Mass vs threshold
  - Signal 3: Temporal Motion Delta (frame-to-frame change)
  - Signal 4: Spatial Concentration (Bounding Box Span X x Y)
  - Verdict, Confidence Score & Exact Detection Latency (ms)

Controls:
  [u] or [0]          : Raise & Arm Target (UP,1)
  [d]                 : Lower & Disarm Target (DOWN,1)
  [`] (Backtick key)  : Manual Hit Override
  [q] or [Ctrl+C]     : Exit Monitor
===============================================================================
"""

import sys
import time
import json
import random
import urllib.request
import subprocess
import os

try:
    import msvcrt
    HAS_MSVCRT = True
except ImportError:
    HAS_MSVCRT = False

ESP32_HOST = "http://192.168.4.1"

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

def send_cmd(action):
    try:
        url = f"{ESP32_HOST}/cmd?action={action}"
        req = urllib.request.Request(url, headers={"User-Agent": "SnyptrLive/2.0"})
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            return resp.read().decode('utf-8', errors='ignore')
    except Exception as e:
        return f"ERR: {e}"

def poll_telemetry():
    try:
        url = f"{ESP32_HOST}/telemetry"
        req = urllib.request.Request(url, headers={"User-Agent": "SnyptrLive/2.0"})
        with urllib.request.urlopen(req, timeout=0.8) as resp:
            return json.loads(resp.read().decode('utf-8', errors='ignore'))
    except Exception:
        return None

def trigger_manual_hit_override():
    res = send_cmd("DOWN,1")
    phrase = random.choice(HIT_PHRASES)
    print(f"\n\033[92m[🎯 MANUAL OVERRIDE (`)] {phrase} | Target LOWERED (DOWN,1) -> {res}\033[0m")
    speak_phrase_async(phrase)

def render_gauge(value, max_val=60, width=16, pass_thresh=18):
    clamped = min(value, max_val)
    filled = int((clamped / max_val) * width) if max_val > 0 else 0
    empty = width - filled
    bar = "█" * filled + "░" * empty
    color = "\033[92m" if value >= pass_thresh else "\033[90m"
    return f"{color}[{bar}]\033[0m {value:>3d}"

def clear_screen():
    # ANSI clear screen or fallback
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()

def main():
    print("Connecting to ESP32 Hotspot at http://192.168.4.1...")
    last_hit = False
    last_draw_time = 0
    consecutive_errs = 0

    clear_screen()

    while True:
        try:
            # Handle interactive keypresses on Windows console
            if HAS_MSVCRT and msvcrt.kbhit():
                ch = msvcrt.getch()
                try:
                    ch_str = ch.decode('utf-8', errors='ignore')
                except Exception:
                    ch_str = ""

                if ch_str in ['`', '~']:
                    trigger_manual_hit_override()
                elif ch_str.lower() in ['u', '0']:
                    print("\n[CMD] Raising Target 1 (UP,1)...")
                    send_cmd("UP,1")
                elif ch_str.lower() == 'd':
                    print("\n[CMD] Lowering Target 1 (DOWN,1)...")
                    send_cmd("DOWN,1")
                elif ch_str.lower() == 'q':
                    print("\nExiting monitor.")
                    break

            data = poll_telemetry()
            now = time.time()

            if data is not None:
                consecutive_errs = 0
                hit = data.get("hit", False)
                state = data.get("targetState", "DOWN")
                det_state = data.get("detectorState", state)
                x = data.get("x", 80)
                y = data.get("y", 56)
                dart_px = data.get("dartPixels", 0)
                
                # Diagnostic metrics from P4
                delta_px = data.get("deltaPx", 0)
                color_px = data.get("colorPx", 0)
                motion_px = data.get("motionPx", 0)
                span_x = data.get("spanX", 0)
                span_y = data.get("spanY", 0)
                confidence = data.get("confidence", 0)
                latency_ms = data.get("latencyMs", 0.0)
                p4_link = data.get("p4Link", False)
                roi = data.get("roi", {"cx": 400, "cy": 280, "rx": 160, "ry": 150})

                # Refresh display every ~100ms
                if now - last_draw_time >= 0.10:
                    last_draw_time = now

                    # Format state color
                    if hit:
                        state_str = "\033[92;1m>>> 🎯 TARGET HIT CONFIRMED! <<<\033[0m"
                    elif state == "UP":
                        state_str = "\033[96;1mTARGET UP [ARMED & WATCHING ROI]\033[0m"
                    else:
                        state_str = "\033[90mTARGET DOWN [DETECTION LOCKED]\033[0m"

                    p4_badge = "\033[92m● ONLINE (UART 115200)\033[0m" if p4_link else "\033[91m○ WAITING LINK\033[0m"
                    
                    # Signal status tags
                    s1_tag = "\033[92mPASS\033[0m" if delta_px >= 18 else "\033[90mIDLE\033[0m"
                    s2_tag = "\033[92mPASS\033[0m" if color_px >= 12 else "\033[90mIDLE\033[0m"
                    s3_tag = "\033[92mPASS\033[0m" if motion_px >= 14 else "\033[90mIDLE\033[0m"
                    
                    is_compact = (span_x <= 90 and span_y <= 90 and span_x >= 4 and span_y >= 4)
                    s4_tag = "\033[92mCOMPACT\033[0m" if is_compact else ("\033[93mSCATTERED\033[0m" if (span_x > 90 or span_y > 90) else "\033[90mNONE\033[0m")

                    sys.stdout.write("\033[H") # Move cursor to top-left
                    output = [
                        "=" * 78,
                        "   🎯 SNYPTR-RAIL: HIGH-SPEED DETERMINISTIC EVENT DETECTION MONITOR",
                        "=" * 78,
                        f" Target Servo State : {state_str:<40}",
                        f" P4 Camera Link     : {p4_badge:<35} | Latency: \033[93m{latency_ms:.1f} ms\033[0m",
                        f" Fixed Camera ROI   : Center ({roi.get('cx',400)}, {roi.get('cy',280)}) | Rx={roi.get('rx',160)}, Ry={roi.get('ry',150)} px",
                        "-" * 78,
                        " LIVE DETECTOR SIGNALS (WATCHING OPTIMAL BLACK TARGET ZONE):",
                        f"  [1] Ref Pixel Delta : {render_gauge(delta_px, 60, 16, 18)} px (Thresh: 18)   [{s1_tag}]",
                        f"  [2] Orange/Yellow   : {render_gauge(color_px, 60, 16, 12)} px (Thresh: 12)   [{s2_tag}]",
                        f"  [3] Temporal Motion : {render_gauge(motion_px, 60, 16, 14)} px (Thresh: 14)   [{s3_tag}]",
                        f"  [4] Spatial Cluster : Span: {span_x:>2d} x {span_y:>2d} px (Max: 90x90 px)   [{s4_tag}]",
                        "-" * 78,
                        f" Impact Position    : ({x:3d}, {y:3d}) | Cluster Mass: {dart_px} px | Conf: {confidence}%",
                    ]

                    if hit:
                        output.append(f" Verdict            : \033[92;1m🎯 HIT CONFIRMED! Servo reversed to DOWN (Latency: {latency_ms:.1f}ms)\033[0m")
                    elif state == "UP":
                        output.append(f" Verdict            : \033[96mARMED — Watching optimal zone for Nerf projectile entrance...\033[0m")
                    else:
                        output.append(f" Verdict            : \033[90mSTANDBY — Target concealed. Ready for [u] or [0] to pop up.\033[0m")

                    output.append("-" * 78)
                    output.append(" Controls: [0/u] Pop Up & Arm  |  [d] Lower Target  |  [`] Manual Hit  |  [q] Quit")
                    output.append("=" * 78)
                    
                    sys.stdout.write("\n".join(output) + "\n")
                    sys.stdout.flush()

                if hit and not last_hit:
                    phrase = random.choice(HIT_PHRASES)
                    speak_phrase_async(phrase)
                last_hit = hit
            else:
                consecutive_errs += 1
                if consecutive_errs % 15 == 0:
                    sys.stdout.write(f"\rWaiting for Wi-Fi connection to ESP32_Camera at {ESP32_HOST}...\033[K")
                    sys.stdout.flush()

            time.sleep(0.04)

        except KeyboardInterrupt:
            print("\nMonitor stopped.")
            break
        except Exception as e:
            time.sleep(0.3)

if __name__ == "__main__":
    main()
