import serial
import struct
import zlib
import sys
import time
import numpy as np
import cv2

# Reconfigure stdout to use UTF-8 to prevent cp1252 mapping crashes on Windows
sys.stdout.reconfigure(encoding='utf-8')

port = 'COM15'
baud = 2000000

# Global format variables, populated dynamically from the device's RAW DIAGNOSTICS output line
width = 800
height = 800
bytesperline = None

def parse_diagnostics(line):
    global width, height, bytesperline
    try:
        # Expected format: "RAW DIAGNOSTICS: width=800, height=800, bytesperline=800, ..."
        parts = line.split("RAW DIAGNOSTICS:")[1].split(",")
        for part in parts:
            part = part.strip()
            if part.startswith("width="):
                width = int(part.split("=")[1])
            elif part.startswith("height="):
                height = int(part.split("=")[1])
            elif part.startswith("bytesperline="):
                bytesperline = int(part.split("=")[1])
        print(f"[Parser] Parsed parameters: width={width}, height={height}, bytesperline={bytesperline}", flush=True)
    except Exception as e:
        print(f"[Parser] Error parsing diagnostics line: {e}", flush=True)

def process_raw_frame(payload, raw_filename, received_crc):
    global width, height, bytesperline
    
    payload_len = len(payload)
    
    # 1. Verify payload length is exactly 640000
    if payload_len != 640000:
        print(f"Warning: Payload length {payload_len} is not exactly 640000 (expected 800*800).", flush=True)
        # Verify CRC and print diagnostic results
        calc_crc = (zlib.crc32(payload, 0xFFFFFFFF) ^ 0xFFFFFFFF) & 0xFFFFFFFF
        crc_ok = (calc_crc == received_crc)
        print("="*40, flush=True)
        print("DIAGNOSTIC TRANSPORT PACKET RECEIVED", flush=True)
        print(f"Payload bytes: {payload_len}", flush=True)
        print(f"CRC: {'OK' if crc_ok else 'ERROR'}", flush=True)
        print(f"  Header Extracted CRC:   {received_crc:08X}", flush=True)
        print(f"  Locally Calculated CRC: {calc_crc:08X}", flush=True)
        print("="*40 + "\n", flush=True)
        return
        
    # 2. Convert payload to a NumPy uint8 array
    arr = np.frombuffer(payload, dtype=np.uint8)
    
    # 3. Reshape directly to (800, 800) if bytesperline is 0 or not set, or payload length is exactly width*height
    if bytesperline == 0 or bytesperline is None or payload_len == width * height:
        active_pixels = arr[:width * height].reshape((height, width))
    else:
        # If bytesperline is non-zero and set
        stride = bytesperline
        if len(arr) >= stride * height:
            reshaped = arr[:stride * height].reshape((height, stride))
            active_pixels = reshaped[:, :width]
        else:
            print(f"Warning: Received buffer size ({len(arr)}) is less than stride*height ({stride * height}). Falling back to raw reshape.", flush=True)
            active_pixels = arr[:width * height].reshape((height, width))

    # Calculate basic statistics on the active pixels
    val_min = np.min(active_pixels)
    val_max = np.max(active_pixels)
    val_mean = np.mean(active_pixels)
    val_std = np.std(active_pixels)

    # 4. Confirm the resulting array is non-empty before cv2.cvtColor()
    if active_pixels.size == 0 or active_pixels.shape[0] == 0 or active_pixels.shape[1] == 0:
        print("Error: Active pixels array is empty! Cannot perform demosaicing.", flush=True)
        return

    # 5. Demosaic using cv2.COLOR_BayerBG2BGR (BGGR pattern)
    demosaic_bggr = cv2.cvtColor(active_pixels, cv2.COLOR_BayerBG2BGR)
    cv2.imwrite('raw_frame_1_demosaic.png', demosaic_bggr)
    
    # 6. Generate the RGGB comparison
    demosaic_rggb = cv2.cvtColor(active_pixels, cv2.COLOR_BayerRG2BGR)
    cv2.imwrite('raw_frame_1_demosaic_rggb.png', demosaic_rggb)
    
    calc_crc = (zlib.crc32(payload, 0xFFFFFFFF) ^ 0xFFFFFFFF) & 0xFFFFFFFF
    crc_ok = (calc_crc == received_crc)

    # Print exactly the format required by the user
    print("\n" + "="*40, flush=True)
    print("RAW FRAME RECEIVED", flush=True)
    print(f"Width: {width}", flush=True)
    print(f"Height: {height}", flush=True)
    print(f"Payload bytes: {payload_len}", flush=True)
    print(f"CRC: {'OK' if crc_ok else 'ERROR'}", flush=True)
    print(f"  Header Extracted CRC:   {received_crc:08X}", flush=True)
    print(f"  Locally Calculated CRC: {calc_crc:08X}", flush=True)
    print(f"RAW FILE: {raw_filename}", flush=True)
    print("DEMOSAIC: raw_frame_1_demosaic.png", flush=True)
    print("DEMOSAIC_RGGGB: raw_frame_1_demosaic_rggb.png", flush=True)
    print("="*40 + "\n", flush=True)
    
    print(f"Payload length: {payload_len}", flush=True)
    print(f"min: {val_min}", flush=True)
    print(f"max: {val_max}", flush=True)
    print(f"mean: {val_mean:.2f}", flush=True)
    print(f"std: {val_std:.2f}", flush=True)
    print(f"array shape: {active_pixels.shape}", flush=True)
    print(f"array size: {active_pixels.size}", flush=True)

print(f"Opening port {port} at {baud} baud (default DTR/RTS)...", flush=True)
try:
    ser = serial.Serial(port, baud, timeout=0.1)
    # Perform standard ESP32 hardware reset sequence to boot normally from flash:
    # 1. Force Reset (EN pin pulled low, GPIO0 high)
    ser.dtr = False
    ser.rts = True
    time.sleep(0.1)
    # 2. Release Reset (EN pin pulled high, GPIO0 high) and keep JTAG DTR active:
    ser.dtr = True
    ser.rts = True
    time.sleep(0.2)
except Exception as e:
    print(f"Error: Could not open serial port {port}. Details: {e}", flush=True)
    sys.exit(1)

print("HOST: opened COM15", flush=True)
print("HOST: waiting for MAGIC", flush=True)

buffer = bytearray()
text_line = bytearray()
state = 0  # 0: SEARCHING_MAGIC, 1: WAITING_HEADER, 2: STREAMING_PAYLOAD

frame_id = 0
payload_len = 0
received_crc = 0
bytes_received = 0
calc_crc = 0
out_file = None
raw_filename = 'raw_frame_1_sbggr8.raw'

try:
    while True:
        # If we are streaming payload (state 2), write chunks directly to file
        if state == 2:
            remaining = payload_len - bytes_received
            if remaining > 0:
                if len(buffer) > 0:
                    data = bytes(buffer)
                    buffer = bytearray()
                else:
                    data = ser.read(min(remaining, 65536))
                
                if data:
                    chunk = data[:remaining]
                    out_file.write(chunk)
                    
                    if bytes_received == 0:
                        calc_crc = zlib.crc32(chunk)
                    else:
                        calc_crc = zlib.crc32(chunk, calc_crc)
                        
                    bytes_received += len(chunk)
                    
                    # Print progress when we cross 64KB boundaries or when completed
                    prev_blocks = (bytes_received - len(chunk)) // 65536
                    curr_blocks = bytes_received // 65536
                    if curr_blocks > prev_blocks or bytes_received == payload_len:
                        print(f"HOST: received {bytes_received} / {payload_len}", flush=True)
                    
                    if len(data) > remaining:
                        buffer.extend(data[remaining:])
            
            if bytes_received >= payload_len:
                out_file.close()
                calc_crc = calc_crc & 0xFFFFFFFF
                
                print("HOST: CRC check...", flush=True)
                
                # Load the saved raw file back to process and demosaic
                with open(raw_filename, 'rb') as rf:
                    saved_payload = rf.read()
                
                process_raw_frame(saved_payload, raw_filename, received_crc)
                
                ser.close()
                sys.exit(0)
        else:
            # State 0 or 1: Read as many bytes as are currently available in the buffer, or wait for at least 1 byte
            in_waiting = ser.in_waiting
            read_len = in_waiting if in_waiting > 0 else 1
            data = ser.read(read_len)
            if data:
                buffer.extend(data)
            
            if state == 0:
                magic_idx = buffer.find(b'\xAA\x55\x01')
                if magic_idx != -1:
                    # Process any bytes before the magic as ASCII logs
                    pre_magic = buffer[:magic_idx]
                    for byte in pre_magic:
                        if byte == 10:  # LF
                            if text_line:
                                line_str = text_line.decode('utf-8', errors='replace').strip()
                                if line_str:
                                    print(line_str, flush=True)
                                    if "RAW DIAGNOSTICS:" in line_str:
                                        parse_diagnostics(line_str)
                                text_line = bytearray()
                        elif byte != 13:  # skip CR
                            text_line.append(byte)
                    
                    # Keep only from the magic sequence onwards
                    buffer = buffer[magic_idx:]
                    state = 1
                else:
                    # No magic found yet. Keep only the last 2 bytes to catch split magic
                    if len(buffer) > 2:
                        pre_magic = buffer[:-2]
                        buffer = buffer[-2:]
                        for byte in pre_magic:
                            if byte == 10:  # LF
                                if text_line:
                                    line_str = text_line.decode('utf-8', errors='replace').strip()
                                    if line_str:
                                        print(line_str, flush=True)
                                        if "RAW DIAGNOSTICS:" in line_str:
                                            parse_diagnostics(line_str)
                                    text_line = bytearray()
                            elif byte != 13:  # skip CR
                                text_line.append(byte)
            
            if state == 1:
                if len(buffer) >= 15:
                    # Parse header
                    frame_id = struct.unpack('<I', buffer[3:7])[0]
                    payload_len = struct.unpack('<I', buffer[7:11])[0]
                    received_crc = struct.unpack('<I', buffer[11:15])[0]
                    
                    print("HOST: MAGIC FOUND", flush=True)
                    print("HOST: HEADER RECEIVED", flush=True)
                    print(f"HOST: expecting {payload_len} bytes", flush=True)
                    
                    out_file = open(raw_filename, 'wb')
                    bytes_received = 0
                    calc_crc = 0
                    
                    # Pass any leftover payload bytes (after header) to state 2
                    payload_start = buffer[15:]
                    buffer = bytearray()
                    if payload_start:
                        buffer.extend(payload_start)
                    
                    state = 2
                    
except KeyboardInterrupt:
    print("\nStopping...", flush=True)
finally:
    if ser.is_open:
        ser.close()
    print("Port closed.", flush=True)
