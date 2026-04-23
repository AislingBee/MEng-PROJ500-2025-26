"""Test boot diagnostics from Nucleo CAN bridge v3."""
import serial, struct, time, sys

sys.stdout.reconfigure(errors='replace')

s = serial.Serial('COM8', 921600, timeout=3)
time.sleep(3)

boot = s.read_all()
print("=== BOOT MESSAGES ===")
if boot:
    for line in boot.split(b'\n'):
        print(f"  {line.decode(errors='replace').strip()}")
else:
    print("  (none - may have been consumed during reset)")

print("\n=== INIT HANDSHAKE ===")
s.write(bytes.fromhex("41542b41540d0a"))
s.flush()
time.sleep(0.3)
resp = s.read_all()
print(f"  Response: {repr(resp)}")

print("\n=== SENDING MOTOR ENABLE ===")
motor_id = 127
ext_id = (0x12 << 24) | (0xFD << 8) | motor_id
reg32 = (ext_id << 3) | 0x04
id_bytes = struct.pack(">I", reg32)
param_bytes = struct.pack("<H", 0x7005)
data = param_bytes + b'\x00\x00' + b'\x00\x00\x00\x00'
frame = b'\x41\x54' + id_bytes + bytes([len(data)]) + data + b'\x0D\x0A'
print(f"  TX hex: {frame.hex()}")
s.write(frame)
s.flush()
time.sleep(1.0)

resp = s.read_all()
print(f"\n=== RESPONSE ({len(resp)} bytes) ===")
if resp:
    print(f"  Hex: {resp.hex()}")
    # Try to parse AT frames
    i = 0
    while i < len(resp):
        if i+2 <= len(resp) and resp[i] == 0x41 and resp[i+1] == 0x54:
            if i+7 <= len(resp):
                dlc = resp[i+6]
                if dlc <= 8 and i + 2+4+1+dlc+2 <= len(resp):
                    flen = 2+4+1+dlc+2
                    f = resp[i:i+flen]
                    r32 = struct.unpack(">I", f[2:6])[0]
                    eid = r32 >> 3
                    ct = (eid >> 24) & 0x1F
                    mid = eid & 0xFF
                    fd = f[7:7+dlc]
                    print(f"  AT Frame: ext_id=0x{eid:08X} comm_type={ct} motor={mid} data={fd.hex()} dlc={dlc}")
                    i += flen
                    continue
            # Text diagnostic messages mixed in
            end = resp.find(b'\n', i)
            if end == -1: end = len(resp)
            print(f"  Text: {resp[i:end+1].decode(errors='replace').strip()}")
            i = end + 1
        else:
            end = resp.find(b'\n', i)
            if end == -1: end = len(resp)
            line = resp[i:end+1].decode(errors='replace').strip()
            if line:
                print(f"  Text: {line}")
            i = end + 1
else:
    print("  No response")

print("\n=== WAITING 6s FOR ERROR REPORTS ===")
time.sleep(6)
delayed = s.read_all()
if delayed:
    for line in delayed.split(b'\n'):
        l = line.decode(errors='replace').strip()
        if l: print(f"  {l}")
else:
    print("  (none - good!)")

s.close()
print("\nTest complete.")
