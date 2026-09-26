import serial
import time
import os

def test_write_speed(port, baud, size):
    try:
        s = serial.Serial(port, baud, timeout=1)
        data = os.urandom(size)
        t0 = time.time()
        s.write(data)
        s.flush()
        t1 = time.time()
        print(f"Write {size} bytes at {baud} baud took: {(t1-t0)*1000:.2f} ms")
        s.close()
    except Exception as e:
        print(f"Error on {size} bytes: {e}")

print("Testing at 3000000 baud...")
test_write_speed("COM15", 3000000, 1024)
test_write_speed("COM15", 3000000, 15360)
