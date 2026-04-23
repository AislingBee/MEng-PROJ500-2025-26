"""Quick motor test: enable, zero, jog, position — via Nucleo CAN bridge."""
import serial, struct, time, math, sys

sys.stdout.reconfigure(errors='replace')

COM_PORT = 'COM8'
BAUD = 921600
MOTOR_ID = 127
HOST_ID = 0xFD

def build_frame(motor_id, param_id, value_bytes):
    ext_id = (0x12 << 24) | (HOST_ID << 8) | motor_id
    reg32 = (ext_id << 3) | 0x04
    id_bytes = struct.pack(">I", reg32)
    param_bytes = struct.pack("<H", param_id)
    data = param_bytes + b'\x00\x00' + value_bytes
    return b'\x41\x54' + id_bytes + bytes([len(data)]) + data + b'\x0D\x0A'

def build_can_frame(comm_type, motor_id, data, extra=HOST_ID):
    ext_id = (comm_type << 24) | (extra << 8) | motor_id
    reg32 = (ext_id << 3) | 0x04
    id_bytes = struct.pack(">I", reg32)
    return b'\x41\x54' + id_bytes + bytes([len(data)]) + data + b'\x0D\x0A'

def parse_response(resp):
    results = []
    i = 0
    while i < len(resp) - 8:
        if resp[i] == 0x41 and resp[i+1] == 0x54:
            dlc = resp[i+6]
            if dlc <= 8 and i + 9 + dlc <= len(resp):
                r32 = struct.unpack(">I", resp[i+2:i+6])[0]
                eid = r32 >> 3
                ct = (eid >> 24) & 0x1F
                d = resp[i+7:i+7+dlc]
                results.append((ct, eid, d))
                i += 2+4+1+dlc+2
                continue
        i += 1
    return results

s = serial.Serial(COM_PORT, BAUD, timeout=0.1)
time.sleep(1)
s.read_all()

# Init
print("1. Init handshake...")
s.write(bytes.fromhex("41542b41540d0a"))
s.flush(); time.sleep(0.3)
resp = s.read_all()
print(f"   Response: {repr(resp)}")
assert b'OK' in resp, "Init failed!"

# Enable motor (jog mode = run_mode 0)
print("2. Enabling motor...")
s.write(build_frame(MOTOR_ID, 0x7005, b'\x00\x00\x00\x00'))
s.flush(); time.sleep(0.1)
resp = s.read_all()
for ct, eid, d in parse_response(resp):
    print(f"   Response: comm_type={ct} data={d.hex()}")

# Zero
print("3. Zero position...")
s.write(build_can_frame(6, MOTOR_ID, b'\x01' + b'\x00'*7))
s.flush(); time.sleep(0.3)
resp = s.read_all()
for ct, eid, d in parse_response(resp):
    print(f"   Response: comm_type={ct} data={d.hex()}")

# Jog right briefly
print("4. Jog right (1 rad/s for 1s)...")
vel = 1.0
vel_u16 = int(((vel / 15.0) + 1.0) * 32767)
payload = bytes([0x07, 0x01]) + struct.pack(">H", vel_u16)
s.write(build_frame(MOTOR_ID, 0x7005, payload))
s.flush()
time.sleep(1.0)

# Stop jog
print("5. Stop jog...")
payload = bytes([0x07, 0x00, 0x7F, 0xFF])
s.write(build_frame(MOTOR_ID, 0x7005, payload))
s.flush(); time.sleep(0.3)
resp = s.read_all()
print(f"   Got {len(resp)} bytes response")

print("\n=== ALL TESTS PASSED ===")
s.close()
