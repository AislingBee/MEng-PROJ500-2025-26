"""Quick test: send motor enable frame through Nucleo CAN bridge."""
import serial, struct, time

s = serial.Serial('COM8', 921600, timeout=0.1)
time.sleep(0.5)
s.read_all()

# Init handshake
s.write(bytes.fromhex("41542b41540d0a"))
s.flush()
time.sleep(0.3)
print("Init:", repr(s.read_all()))

# Enable motor 127 (same frame as app.py)
motor_id = 127
ext_id = (0x12 << 24) | (0xFD << 8) | motor_id
reg32 = (ext_id << 3) | 0x04
id_bytes = struct.pack(">I", reg32)
param_bytes = struct.pack("<H", 0x7005)
data = param_bytes + b'\x00\x00' + b'\x00\x00\x00\x00'
frame = b'\x41\x54' + id_bytes + bytes([len(data)]) + data + b'\x0D\x0A'
print(f"Sending enable frame: {frame.hex()}")
s.write(frame)
s.flush()
time.sleep(0.5)

# Read motor response
resp = s.read_all()
if resp:
    print(f"Motor response ({len(resp)} bytes): {resp.hex()}")
else:
    print("No response from motor (CAN bus may not be connected or motor off)")

s.close()
print("Test complete.")
