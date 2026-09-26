import urllib.request
import time
import numpy as np
import traceback

url = "http://192.168.4.1/"
print(f"Connecting to {url}...")

# Check for OpenCV
try:
    import cv2
    HAS_OPENCV = True
except ImportError:
    HAS_OPENCV = False
    print("OpenCV not found. Running in console-only diagnostic mode.")
    print("To view the stream visually, install opencv: pip install opencv-python\n")

try:
    # Open connection with 5 second timeout
    stream = urllib.request.urlopen(url, timeout=5)
    print("Connected! Reading MJPEG stream...")
    
    buffer = b""
    frame_count = 0
    last_report_time = time.time()
    bytes_received = 0
    total_frames = 0
    
    while True:
        chunk = stream.read(4096)
        if not chunk:
            print("Stream closed by server.")
            break
            
        buffer += chunk
        bytes_received += len(chunk)
        
        while True:
            # Find JPEG Start of Image (SOI)
            start = buffer.find(b'\xff\xd8')
            if start == -1:
                # If no start found, keep only the last byte in case it's part of a start marker
                if len(buffer) > 0:
                    buffer = buffer[-1:]
                break
                
            # Find JPEG End of Image (EOI)
            end = buffer.find(b'\xff\xd9', start)
            if end == -1:
                # Need more data for this frame
                break
                
            # Extract complete JPEG frame
            jpeg_data = buffer[start:end+2]
            # Remove processed data from buffer
            buffer = buffer[end+2:]
            
            frame_count += 1
            total_frames += 1
            
            # Verify JPEG integrity
            has_ffd8 = jpeg_data.startswith(b'\xff\xd8')
            has_ffd9 = jpeg_data.endswith(b'\xff\xd9')
            
            status = "OK" if (has_ffd8 and has_ffd9) else "INVALID MARKERS"
            
            # Print frame details
            print(f"Frame #{total_frames} | Size: {len(jpeg_data)} bytes | Status: {status}")
            
            if has_ffd8 and has_ffd9:
                # Save the latest valid frame locally
                with open("latest_stream_frame.jpg", "wb") as f:
                    f.write(jpeg_data)
                
                # Show using OpenCV window if available
                if HAS_OPENCV:
                    np_arr = np.frombuffer(jpeg_data, dtype=np.uint8)
                    img = cv2.imdecode(np_arr, cv2.IMREAD_UNCHANGED)
                    if img is not None:
                        cv2.imshow("ESP32-P4 Stream", img)
                        # Exit if 'q' is pressed
                        if cv2.waitKey(1) & 0xFF == ord('q'):
                            print("Quit key pressed.")
                            break
                    else:
                        print("Failed to decode JPEG frame with OpenCV.")
            
            # Calculate and report performance metrics every 2 seconds
            now = time.time()
            if now - last_report_time >= 2.0:
                fps = frame_count / (now - last_report_time)
                kb_sec = (bytes_received / 1024.0) / (now - last_report_time)
                print(f"\n--- STREAM PERFORMANCE METRICS ---")
                print(f"Measured FPS: {fps:.2f}")
                print(f"Throughput: {kb_sec:.1f} KB/s ({kb_sec*8/1024:.2f} Mbps)")
                print(f"Total frames received: {total_frames}")
                print(f"----------------------------------\n")
                
                # Reset interval counters
                frame_count = 0
                bytes_received = 0
                last_report_time = now

except Exception as e:
    print("Error reading stream:")
    traceback.print_exc()
finally:
    if HAS_OPENCV:
        cv2.destroyAllWindows()

