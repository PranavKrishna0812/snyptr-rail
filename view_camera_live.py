"""
SNYPTR-RAIL // Live Camera & Telemetry Monitor
Polls live target telemetry from the Pop-Up ESP32 over Wi-Fi (http://192.168.4.1).

Controls:
  [`] (Backtick key) : Manually lower target & randomly announce that target is hit!
  [u]                : Raise Target 1 (UP,1)
  [d]                : Lower Target 1 (DOWN,1)
  [Ctrl+C]           : Exit monitor
"""

import sys
import time
import json
import random
import urllib.request
import subprocess

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
        req = urllib.request.Request(url, headers={"User-Agent": "SnyptrLive/1.0"})
        with urllib.request.urlopen(req, timeout=1.5) as resp:
            return resp.read().decode('utf-8', errors='ignore')
    except Exception as e:
        return f"ERR: {e}"

def poll_telemetry():
    try:
        url = f"{ESP32_HOST}/telemetry"
        req = urllib.request.Request(url, headers={"User-Agent": "SnyptrLive/1.0"})
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            return json.loads(resp.read().decode('utf-8', errors='ignore'))
    except Exception:
        return None

def trigger_manual_hit_override():
    res = send_cmd("DOWN,1")
    phrase = random.choice(HIT_PHRASES)
    print(f"\n\033[92m[🎯 MANUAL OVERRIDE (`)] {phrase} | Target LOWERED (DOWN,1) -> {res}\033[0m")
    speak_phrase_async(phrase)

def main():
    print("=" * 72)
    print("   SNYPTR-RAIL LIVE TELEMETRY & TARGET MONITOR")
    print("=" * 72)
    print(f"Connecting to ESP32 Hotspot at {ESP32_HOST}...")
    print("Controls: Press [`] to lower target & announce random HIT!")
    print("          Press [u] to POP UP, [d] to DROP, [Ctrl+C] to exit")
    print("-" * 72)

    last_state = ""
    last_hit = False

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
                elif ch_str.lower() == 'u':
                    print("\n[CMD] Raising Target 1 (UP,1)...")
                    send_cmd("UP,1")
                elif ch_str.lower() == 'd':
                    print("\n[CMD] Lowering Target 1 (DOWN,1)...")
                    send_cmd("DOWN,1")

            data = poll_telemetry()
            if data is not None:
                hit = data.get("hit", False)
                state = data.get("targetState", "UNKNOWN")
                x = data.get("x", 0)
                y = data.get("y", 0)
                dart_px = data.get("dartPixels", 0)
                yellow_px = data.get("yellowPx", 0)
                orange_px = data.get("orangePx", 0)
                dark_pct = data.get("darkPct", 0.0)
                p4_link = data.get("p4Link", False)

                link_badge = "[P4 LINK: OK]" if p4_link else "[P4 LINK: WAITING]"
                hit_badge = ">>> HIT DETECTED! <<<" if hit else "TARGET IDLE"

                status_line = (
                    f"\rState: {state:<5} | {link_badge:<18} | {hit_badge:<22} | "
                    f"Pos: ({x:3d}, {y:3d}) | Yellow: {yellow_px:4d} px | Orange: {orange_px:3d} px | Dark: {dark_pct:4.1f}%"
                )
                sys.stdout.write(status_line)
                sys.stdout.flush()

                if hit and not last_hit:
                    print(f"\n\033[92m[EVENT] Target 1 HIT! Position: ({x}, {y}) | Mass: {dart_px} px\033[0m")

                last_hit = hit
                last_state = state
            else:
                sys.stdout.write(f"\rWaiting for Wi-Fi connection to ESP32_Camera (192.168.4.1)...")
                sys.stdout.flush()

            time.sleep(0.08)

        except KeyboardInterrupt:
            print("\nMonitor stopped.")
            break
        except Exception as e:
            time.sleep(0.5)

if __name__ == "__main__":
    main()
