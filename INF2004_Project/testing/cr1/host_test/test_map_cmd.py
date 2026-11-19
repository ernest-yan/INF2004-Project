"""CR1 host test: the dashboard's MAP command (6-bit letters, 8 bytes) is
byte-identical to the C codec's, and the car's validator accepts it.
Usage: python3 test_map_cmd.py <dashboard.py> <map_cmd_enc binary>"""
import importlib.util
import subprocess
import sys
import types

for m in ("paho", "paho.mqtt", "paho.mqtt.client"):
    sys.modules[m] = types.ModuleType(m)
spec = importlib.util.spec_from_file_location("dashboard", sys.argv[1])
dash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(dash)
enc = sys.argv[2]

fails = checks = 0
def check(ok, what):
    global fails, checks
    checks += 1
    if not ok:
        fails += 1
        print("FAIL:", what)

frames = []
for line in subprocess.run([enc], capture_output=True, text=True, check=True).stdout.split("\n"):
    if not line:
        continue
    letters, seq, hexs = line.split()
    py = dash.encode_cmd(dash.CMD["map"], 0x4A3B, int(seq), letters)
    check(py.hex() == hexs, f"map {letters}: dashboard {py.hex()} != C {hexs}")
    check(len(py) == 8, f"map {letters}: {len(py)} bytes, expected 8")
    frames.append((letters, py.hex()))

res = subprocess.run([enc, "validate"], input="\n".join(h for _, h in frames),
                     capture_output=True, text=True, check=True).stdout.split("\n")
for (letters, _), line in zip(frames, res):
    check(line == f"0 {letters}", f"car validator on dashboard map {letters}: {line}")
check(len(dash.encode_cmd(dash.CMD["start"], 0x4A3B, 1)) == 5, "START still 5 bytes")
print(f"MAP command dashboard <-> C codec: {checks} checks, {fails} failed (8 B, was 9 B)")
sys.exit(1 if fails else 0)
