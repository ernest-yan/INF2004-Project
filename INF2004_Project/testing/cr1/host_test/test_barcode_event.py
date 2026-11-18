"""CR1 host test: BARCODE events from the C codec decode correctly in the
real dashboard.py (letter, action, "seen, not decoded", old 5-byte frames).
Usage: python3 test_barcode_event.py <path to dashboard.py> < frames.txt"""
import importlib.util
import sys
import types

sys.modules["paho"] = types.ModuleType("paho")          # not needed to decode
sys.modules["paho.mqtt"] = types.ModuleType("paho.mqtt")
sys.modules["paho.mqtt.client"] = types.ModuleType("paho.mqtt.client")
spec = importlib.util.spec_from_file_location("dashboard", sys.argv[1])
dash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dash)

fails = checks = 0
def check(ok, what):
    global fails, checks
    checks += 1
    if not ok:
        fails += 1
        print("FAIL:", what)

def expected_letter(code):
    return chr(64 + code) if code <= 26 else chr(48 + code - 27)

sizes = set()
for line in sys.stdin:
    action, code, hexs = line.split()
    action, code, frame = int(action), int(code), bytes.fromhex(hexs)
    sizes.add(len(frame))
    d = dash.decode_event(frame)
    check(d["type"] == "BARCODE" and d["value"] == action, f"action {line}")
    if code:
        check(d.get("letter") == expected_letter(code), f"letter {line} -> {d}")
    else:
        check(d.get("seen_not_decoded") is True and "letter" not in d, f"seen {line} -> {d}")

# An old (pre-letter) 5-byte BARCODE event must not invent a letter.
# Built the way the old firmware did: 28-bit header + 4-bit type + 3-bit
# action = 35 bits, zero-padded to 5 bytes.
for action in range(6):
    bits = (1 << 31) | (4242 << 15) | (9 << 7) | (2 << 3) | action   # 35 bits
    old = (bits << 5).to_bytes(5, "big")
    d = dash.decode_event(old)
    check(d["type"] == "BARCODE" and d["value"] == action and "letter" not in d
          and "seen_not_decoded" not in d, f"old frame action {action} -> {d}")
print(f"BARCODE event codec -> dashboard: {checks} checks, {fails} failed "
      f"(frame size {sorted(sizes)} B, was 5 B)")
sys.exit(1 if fails else 0)
