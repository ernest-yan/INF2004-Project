#!/usr/bin/env python3
r"""
IS33 car dashboard (laptop side) -- Buddy 1 / subsystem 1.

  py -m pip install paho-mqtt                       (Windows)
  py testing\subsystem1\tools\dashboard.py --broker 127.0.0.1

Shows decoded telemetry, sends START / STOP / E-STOP, and logs everything
(with laptop timestamps) to a CSV file you can use as test evidence.

By default a live status panel is pinned to the top of the window: vehicle
state, faults by name, wheel speeds and encoder rates, line sensors, tilt,
obstacle, link health, the last ACK and event, and plain-language warnings
(dead encoder, E-STOP latched, ultrasonic fault, stale link...). Commands
are typed at the bottom as before. --no-panel gives the old scrolling output.
Needs a window at least 21 rows tall; otherwise it falls back automatically.

Barcodes: every scan is shown on its own "Scans" rows: the letter and the
action the map gives it, or for a scan with no letter, how many bars+spaces
were seen and why it failed (see 'help'). The map is re-sent after a car
reboot (--map FULR sets it from the start).

Keep the field tables below in sync with app_program/subsystem1/tlm_codec.c
and app_program/subsystem1/MQTT_PROTOCOL.md.
"""
import argparse
import csv
import json
import os
import shutil
import sys
import threading
import time
from collections import deque

import paho.mqtt.client as mqtt

VERSION = 1
T_ONLINE, T_STATUS, T_HEARTBEAT = "is33/car/online", "is33/car/status", "is33/car/heartbeat"
T_EVENT, T_ACK, T_CMD = "is33/car/event", "is33/car/ack", "is33/car/cmd"

CMD = {"start": 1, "stop": 2, "estop": 3, "reset": 4, "map": 5}
MAP_CHARS = set("ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789")
CMD_NAME = {v: k.upper() for k, v in CMD.items()}
RESULT = ["ACCEPTED", "BAD_LEN", "BAD_VERSION", "BAD_SESSION", "BAD_CMD",
          "STALE_SEQ", "QUEUE_FULL"]
VEH_STATE = ["INIT", "IDLE", "LINE_FOLLOW", "LINE_SEARCH", "BARCODE_NAV",
             "OBSTACLE_AVOID", "SAFE_STOP"]
LINK_STATE = ["INIT", "WIFI_CONNECTING", "MQTT_CONNECTING", "ONLINE",
              "BACKOFF", "SAFE_LOCAL"]
EVENT = {1: "STATE_CHANGE", 2: "BARCODE", 3: "HUMP", 4: "OBSTACLE",
         5: "LIFTED", 6: "ESTOP"}
# Names for the numbers in the status packet, used by the live panel.
# Keep in sync with team_interfaces.h (TLM_FAULT_Sn, barcode_cmd_t,
# event_source_t) and subsystem5/ultrasonic.h (subsystem5_state_t).
FAULT_BITS = {2: "S2 motors/encoders", 3: "S3 line/barcode", 4: "S4 IMU/tilt",
              5: "S5 ultrasonic"}
OBS_STATE = ["MONITOR", "WAIT_STOP", "COARSE", "FINE", "PROFILE_READY", "BYPASS",
             "FAULT", "REVERSE"]
BARCODE = ["none", "LEFT", "RIGHT", "STRAIGHT", "UTURN", "INVALID"]
BC_INVALID = BARCODE.index("INVALID")
# barcode_cmd value -> position in the "map FULR" letters (forward, U-turn,
# left, right).  The car only reports the action, so the letter is looked up
# in the map the car accepted.
MAP_POS = {3: 0, 4: 1, 1: 2, 2: 3}
MAP_ROLE = ["forward", "U-turn", "left", "right"]
# BARCODE event scan diagnosis (S3_SCAN_* in line_barcode.h): why a scan
# did or did not give a letter, and how many bars + spaces it counted.
FULL_BARCODE = 29                 # '*' gap letter gap '*' = 9+1+9+1+9
SCAN_RESULT = ["decoded", "too short", "no * frame", "bad letter"]
SCAN_WHY = {
    "too short": "too short: fewer than {full} bars+spaces (part of the barcode "
                 "missed: sensor off the barcode, or it was crossed at an angle)",
    "no * frame": "no '*' start/stop found (bar widths unclear: speed change, "
                  "sensor height, merged/split bars)",
    "bad letter": "'*' found but the letter bars are unreadable (bar widths off)",
}
EVT_SRC = ["button", "laptop", "IMU lift/overturn", "motion", "ultrasonic", "line"]
# subsystem4/motion_class.h (b4_motion_t) and posture.h (b4_lift_cause_t)
MOTION = ["CALIBRATING", "STATIONARY", "MOVING", "TURNING", "ON_HUMP", "LIFTED",
          "OVERTURNED"]
LIFT_CAUSE = ["-", "LIFTED", "OVERTURNED"]
DRIVING = {"LINE_FOLLOW", "BARCODE_NAV", "OBSTACLE_AVOID"}   # wheels should turn
SPEED_MAX, OBS_CM_MAX = (1 << 10) - 1, (1 << 9) - 1           # field saturation

# Printed at start-up (short form) and by the 'help' command (full form).
BARCODE_HELP = """\
Barcode scans (the "Scans" rows keep the last 3; every one is also printed
as a [BARCODE] line and logged in the CSV with a "summary")
  'A' -> LEFT  (30/29 bars+spaces)   decoded, and what the map makes of it
  'X' NOT in the map                 decoded, but the map has no 'X': the car
                                     goes straight on
  SEEN, no letter (14/29 ...): why   the sensor saw bars but no letter came out:
     too short   under 29 bars+spaces: part of the barcode was missed
     no * frame  enough bars, but no '*' start/stop: bar widths unclear
                 (speed changed mid-scan, sensor height, merged/split bars)
     bad letter  '*' found at both ends, but the letter bars are unreadable
  Firmware before 19 Nov 2026 sends no count or reason (and older still, no
  letter: then the letter is looked up in the map the car ACCEPTED).
The map
  'map FULR' = F forward, U U-turn, L left, R right. The car forgets it on
  every reboot; this dashboard re-sends the last accepted map (or --map) when
  the car comes back with a new session. --no-auto-map turns that off.
  Only maps whose ACCEPTED reply this dashboard saw are known.
  Changes to dashboard.py only apply to a newly started dashboard window."""

# (field name, bits) in wire order, after the 28-bit header.
STATUS_FIELDS = [("vehicle_state", 3), ("estop", 1), ("line", 2), ("barcode", 3),
                 ("speed_l", 10), ("speed_r", 10), ("enc_l", 16), ("enc_r", 16),
                 ("dist_cm", 12), ("pitch", 7), ("hump_max_mm", 8),
                 ("obs_state", 3), ("obs_cm", 9), ("faults", 8)]
# Appended later (19-byte frame). Skipped when an older 17-byte frame arrives.
STATUS_FIELDS_2 = [("yaw_dps", 9), ("motion", 3)]
# Lift-detection signals (21-byte frame): upward speed / 4 mm/s, total tilt.
STATUS_FIELDS_3 = [("up_speed", 8), ("tilt", 7), ("s5_fault", 3)]
# S5 bypass step (22-byte frame, 20 Nov): 0 none, 1 out, 2 pass, 3 return,
# 4 align, 5 back up. Kept after a bypass fault, so the log shows where.
STATUS_FIELDS_4 = [("s5_phase", 3)]
S5_PHASE = ["", "out", "pass", "return", "align", "turn", "look"]
# subsystem5/ultrasonic.h (subsystem5_fault_t)
S5_FAULT = ["", "sensor not answering", "scan invalid", "car did not stop",
            "event lost", "no safe side to pass", "bypass blocked/failed"]
HEARTBEAT_FIELDS = [("uptime_s", 16), ("rssi_neg", 7), ("link", 3), ("cmds_ok", 8),
                    ("cmds_rej", 8), ("queue_full", 8), ("reconnects", 8)]
ACK_FIELDS = [("cmd_seq", 16), ("cmd", 4), ("result", 4)]
OBSTACLE_FIELDS = [("nearest_cm", 9), ("centre_deg", 8), ("width_cm", 8),
                   ("clear_l_cm", 9), ("clear_r_cm", 9), ("decision", 3)]
# 20 Nov: raw coarse scan, cm at 30/60/90/120/150 deg (254 open, 255 no echo).
OBSTACLE_FIELDS_2 = [("raw30", 8), ("raw60", 8), ("raw90", 8), ("raw120", 8), ("raw150", 8)]


def letter_from_code(code: int):
    """6-bit letter code (S3_CODE in line_barcode.h) -> 'A'..'Z' / '0'..'9'."""
    if 1 <= code <= 26:
        return chr(ord("A") + code - 1)
    if 27 <= code <= 36:
        return chr(ord("0") + code - 27)
    return None


def describe_map(letters: str) -> str:
    """'ABCD' -> 'A = forward, B = U-turn, C = left, D = right'."""
    return ", ".join(f"{c} = {role}" for c, role in zip(letters, MAP_ROLE))


def map_table(letters: str) -> str:
    """'ABCD' -> one 'letter = action' line each, for the scrolling output."""
    rows = [f"           {c} = {role}" for c, role in zip(letters, MAP_ROLE)]
    return "\n".join(rows + ["           any other letter = go straight on"])


def describe_barcode(d: dict, map_letters=None):
    """One BARCODE event -> (text, colour): the letter, what the map makes of
    it, and for a scan that gave no letter, how much was seen and why."""
    v = d.get("value")
    action = BARCODE[v] if v is not None and v < len(BARCODE) else f"?{v}"
    n = d.get("elements")
    seen = "" if n is None else f"{n}/{FULL_BARCODE} bars+spaces"
    if d.get("seen_not_decoded"):
        why = SCAN_WHY.get(d.get("scan_result"), "").format(full=FULL_BARCODE)
        text = "SEEN, no letter"
        if seen:
            text += f" ({seen})"
        if why:
            text += f": {why}"
        elif n is None:
            text += " (older firmware: no reason sent)"
        return text, RED
    letter = d.get("letter")
    if letter is None and map_letters and v in MAP_POS:
        letter = map_letters[MAP_POS[v]]
    tail = "" if n is None else f"  ({n} bars+spaces)"
    if v == BC_INVALID:
        name = f"'{letter}'" if letter else "a letter"
        mapped = f"map {map_letters}" if map_letters else "no map sent"
        return f"{name} NOT in the map ({mapped}): car goes straight on{tail}", YELLOW
    if letter is None:
        return f"{action} (letter unknown: older firmware, no map known){tail}", YELLOW
    return f"'{letter}' -> {action}{tail}", GREEN


class BitReader:
    """Reads MSB-first bit fields, mirroring bw_put() in tlm_codec.c."""

    def __init__(self, data: bytes):
        self.value = int.from_bytes(data, "big")
        self.nbits = len(data) * 8
        self.pos = 0

    def get(self, bits: int) -> int:
        if self.pos + bits > self.nbits:
            raise ValueError("frame too short")
        shift = self.nbits - self.pos - bits
        self.pos += bits
        return (self.value >> shift) & ((1 << bits) - 1)

    def fields(self, table):
        return {name: self.get(bits) for name, bits in table}

    def remaining(self) -> int:
        return self.nbits - self.pos


def decode_header(r: BitReader) -> dict:
    ver = r.get(4)
    if ver != VERSION:
        raise ValueError(f"protocol version {ver}, expected {VERSION}")
    return {"session": r.get(16), "seq": r.get(8)}


def decode_status(data: bytes) -> dict:
    r = BitReader(data)
    d = decode_header(r)
    d.update(r.fields(STATUS_FIELDS))
    d["pitch"] -= 64                      # sent as pitch + 64
    if r.nbits - r.pos >= sum(bits for _, bits in STATUS_FIELDS_2):
        d.update(r.fields(STATUS_FIELDS_2))
        d["yaw_dps"] -= 256               # sent as yaw + 256
    if r.nbits - r.pos >= sum(bits for _, bits in STATUS_FIELDS_3):
        d.update(r.fields(STATUS_FIELDS_3))
        d["up_speed"] *= 4                # sent as mm/s / 4
    if r.nbits - r.pos >= sum(bits for _, bits in STATUS_FIELDS_4):
        d.update(r.fields(STATUS_FIELDS_4))
    return d


def decode_heartbeat(data: bytes) -> dict:
    r = BitReader(data)
    d = decode_header(r)
    d.update(r.fields(HEARTBEAT_FIELDS))
    d["rssi_dbm"] = -d.pop("rssi_neg")
    return d


def decode_ack(data: bytes) -> dict:
    r = BitReader(data)
    d = decode_header(r)
    d.update(r.fields(ACK_FIELDS))
    return d


def decode_event(data: bytes) -> dict:
    r = BitReader(data)
    d = decode_header(r)
    etype = r.get(4)
    d["type"] = EVENT.get(etype, f"?{etype}")
    if etype == 1:
        d["from"], d["to"] = VEH_STATE[r.get(3)], VEH_STATE[r.get(3)]
    elif etype in (2, 6):
        d["value"] = r.get(3)
        # BARCODE: CR1 firmware adds the decoded letter as a 6-bit code
        # (0 none, 1-26 A-Z, 27-36 0-9). Older 5-byte events have only 5
        # padding bits left, so they never look like a letter.
        if etype == 2 and r.remaining() >= 6:
            letter = letter_from_code(r.get(6))
            if letter:
                d["letter"] = letter
            elif d["value"] == 0:
                d["seen_not_decoded"] = True    # scan seen, no letter
            # 19 Nov firmware: scan result (2) + bars/spaces counted (6).
            # A 6-byte frame has only 7 padding bits left, so never matches.
            if r.remaining() >= 8:
                d["scan_result"] = SCAN_RESULT[r.get(2)]
                d["elements"] = r.get(6)
    elif etype == 3:
        d["height_mm"] = r.get(8)
    elif etype == 4:
        d.update(r.fields(OBSTACLE_FIELDS))
        if r.nbits - r.pos >= sum(bits for _, bits in OBSTACLE_FIELDS_2):
            d.update(r.fields(OBSTACLE_FIELDS_2))
    elif etype == 5:
        c = r.get(2)
        d["cause"] = LIFT_CAUSE[c] if c < len(LIFT_CAUSE) else f"?{c}"
    return d


def decode_online(data: bytes) -> dict:
    r = BitReader(data)
    ver = r.get(4)
    if ver != VERSION:
        raise ValueError(f"protocol version {ver}, expected {VERSION}")
    online = r.get(1)
    r.get(3)
    return {"version": ver, "online": bool(online), "session": r.get(16)}


def code_from_letter(c: str) -> int:
    """'A'..'Z' / '0'..'9' -> 6-bit letter code (inverse of letter_from_code)."""
    if "A" <= c <= "Z":
        return ord(c) - ord("A") + 1
    if "0" <= c <= "9":
        return ord(c) - ord("0") + 27
    return 0


def encode_cmd(code: int, session: int, seq: int, letters: str = "") -> bytes:
    """5 bytes: version(4) cmd(4) session(16) seq(16).
    MAP adds 4 letters (forward, U-turn, left, right) as 6-bit codes
    (CR1 minimum-bit): 24 more bits, 8 bytes in total."""
    head = (bytes([(VERSION << 4) | code]) + session.to_bytes(2, "big")
            + seq.to_bytes(2, "big"))
    if not letters:
        return head
    packed = 0
    for c in letters:
        packed = (packed << 6) | code_from_letter(c)
    return head + packed.to_bytes(3, "big")


def enc_delta(new: int, old: int) -> int:
    """Difference of two 16-bit encoder counts, allowing for wrap-around."""
    return ((new - old + 0x8000) & 0xFFFF) - 0x8000


def fit(segments, cols: int) -> str:
    """Join (text, ansi_colour) segments, cut to `cols` visible characters."""
    out, left = [], cols
    for text, colour in segments:
        if left <= 0:
            break
        text = text[:left]
        left -= len(text)
        out.append(f"\x1b[{colour}m{text}\x1b[0m" if colour else text)
    return "".join(out)


GREEN, YELLOW, RED, DIM, BOLD, TITLE = "32", "33", "1;31", "2", "1", "7"


class Panel:
    """Live status block pinned to the top of the terminal.

    Log lines (ACKs, events, heartbeats) scroll in the region below it, and
    commands are typed at the bottom as before. Uses plain ANSI escapes, so
    it needs no extra packages (Windows Terminal, PowerShell, Linux, macOS).
    """
    HEIGHT = 19                       # rows the panel occupies (4 for warnings)
    SCAN_ROWS = 3                     # newest barcode scans kept on screen
    STALE_S = 1.5                     # status older than this = link problem
    STILL_S = 2.0                     # encoder unchanged this long = not turning

    def __init__(self, lock, out=sys.stdout):
        self.lock, self.out = lock, out       # lock: shared with all printing
        self.data_lock = threading.Lock()     # guards the fields below
        self.size = None
        self.status, self.t_status = None, None
        self.status_times = deque(maxlen=64)
        self.enc = deque()            # (time, total_l, total_r), last few seconds
        self.enc_total = [0, 0]
        self.prev_line, self.line_changes = None, [0, 0]
        self.hb, self.reconnects0 = None, None
        self.online, self.session = None, 0
        self.last_ack, self.estop_src = None, None
        self.last_event = None
        self.last_letter = None     # newest decoded barcode letter
        self.map_letters = None     # map the car accepted, e.g. "BCFR"
        self.lost_map = None        # map the car had before it rebooted
        self.unnamed_barcode = False  # a barcode was read with no map known
        self.map_sent = None          # map typed, car's ACK not here yet
        # (time, text, colour) per BARCODE event, newest last. Kept on its own
        # rows: the Event row is overwritten by the next STATE_CHANGE within
        # a second while line following, so a barcode there is easy to miss.
        self.scans = deque(maxlen=self.SCAN_ROWS)
        self.scan_total = 0

    # ---- data in (called from the MQTT thread) ----
    def on_status(self, d, now):
        prev = self.status
        self.status, self.t_status = d, now
        self.status_times.append(now)
        if prev is not None and "enc_l" in d and "enc_l" in prev:
            self.enc_total[0] += max(enc_delta(d["enc_l"], prev["enc_l"]), 0)
            self.enc_total[1] += max(enc_delta(d["enc_r"], prev["enc_r"]), 0)
        self.enc.append((now, *self.enc_total))
        while self.enc and now - self.enc[0][0] > self.STILL_S + 1.0:
            self.enc.popleft()
        line = d.get("line")
        if line is not None:
            if self.prev_line is not None:
                changed = line ^ self.prev_line
                self.line_changes[0] += (changed >> 1) & 1
                self.line_changes[1] += changed & 1
            self.prev_line = line
        if not d.get("estop"):
            self.estop_src = None

    def on_heartbeat(self, d):
        self.hb = d
        if self.reconnects0 is None:
            self.reconnects0 = d["reconnects"]

    def on_online(self, d):
        if d["online"] and d["session"] != self.session:
            if self.map_letters:
                self.lost_map = self.map_letters   # rebooted car: its map is gone too
            self.map_letters = None
            self.last_letter = None
        self.online, self.session = d["online"], d["session"]

    def on_map(self, letters):
        self.map_letters = letters
        self.map_sent = None
        self.lost_map = None
        self.unnamed_barcode = False

    def on_map_sent(self, letters):
        self.map_sent = letters       # shown until the car's ACK arrives

    def on_map_refused(self):
        self.map_sent = None

    def letter_for(self, bc):
        """Letter for a barcode action, from the accepted map (None if unknown)."""
        if self.map_letters and bc in MAP_POS:
            return self.map_letters[MAP_POS[bc]]
        return None

    def on_ack(self, now, name, result, rtt):
        self.last_ack = (now, name, result, rtt)

    def on_event(self, d, now):
        self.last_event = (now, d)
        if d["type"] == "BARCODE":
            if "letter" in d:
                self.last_letter = d["letter"]
            elif d.get("value") in MAP_POS:
                self.unnamed_barcode = True
            text, colour = describe_barcode(d, self.map_letters)
            self.scans.append((now, text, colour))
            self.scan_total += 1
        if d["type"] == "ESTOP":
            v = d.get("value", 0)
            self.estop_src = EVT_SRC[v] if v < len(EVT_SRC) else f"source {v}"

    # ---- derived numbers ----
    def enc_rate(self, side, now, window=1.0):
        """Encoder edges per second over the last `window` seconds."""
        recent = [e for e in self.enc if now - e[0] <= window]
        if len(recent) < 2 or recent[-1][0] - recent[0][0] < 0.3:
            return None
        return (recent[-1][1 + side] - recent[0][1 + side]) / (recent[-1][0] - recent[0][0])

    def enc_still(self, side, now):
        """True if this encoder has not moved for STILL_S seconds."""
        if not self.enc or now - self.enc[0][0] < self.STILL_S:
            return False                        # not enough history yet
        old = [e for e in self.enc if now - e[0] >= self.STILL_S]
        return bool(old) and self.enc[-1][1 + side] == old[-1][1 + side]

    def status_hz(self, now):
        t = [x for x in self.status_times if now - x <= 5.0]
        return (len(t) - 1) / (t[-1] - t[0]) if len(t) >= 3 and t[-1] > t[0] else None

    def checks(self, now):
        """Plain-language warnings, most serious first: (text, colour)."""
        w = []
        s = self.status
        if self.online is False:
            w.append(("car is OFFLINE (MQTT last will received)", RED))
        if s is None:
            w.append(("waiting for the first status packet from the car", YELLOW))
            return w
        age = now - self.t_status
        if age > self.STALE_S:
            w.append((f"no status for {age:.0f} s: car off, or Wi-Fi/MQTT dropped", RED))
        state = VEH_STATE[s["vehicle_state"]] if s["vehicle_state"] < len(VEH_STATE) else "?"
        faults = s.get("faults", 0) or 0
        for bit, name in FAULT_BITS.items():
            if faults & (1 << bit):
                w.append((f"{name} reports a fault", RED))
        if faults & ~sum(1 << b for b in FAULT_BITS):
            w.append((f"unknown fault bits (faults={faults})", RED))
        if s.get("estop"):
            src = f" by {self.estop_src}" if self.estop_src else ""
            w.append((f"E-STOP latched{src}: send 'reset', then 'start'", RED))
        if s.get("obs_state") == OBS_STATE.index("FAULT"):
            f5 = s.get("s5_fault")
            why = S5_FAULT[f5] if f5 is not None and 0 < f5 < len(S5_FAULT) else None
            ph = s.get("s5_phase")
            if why and ph and 0 < ph < len(S5_PHASE):
                why += f" during '{S5_PHASE[ph]}'"
            w.append((f"ultrasonic FAULT: {why}; send 'reset'" if why else
                      "ultrasonic in FAULT: check TRIG/ECHO wires and ECHO divider", RED))
        elif s.get("obs_cm") == OBS_CM_MAX:
            w.append(("ultrasonic at max range (511 cm): no echo?", YELLOW))
        if state in DRIVING:
            still_l, still_r = self.enc_still(0, now), self.enc_still(1, now)
            if still_l and still_r:
                w.append((f"in {state} but neither encoder is counting", RED))
            elif still_l or still_r:
                side, other = ("left", "right") if still_l else ("right", "left")
                w.append((f"{side} encoder not counting while {other} one is "
                          f"(wheel stuck or encoder wire loose?)", RED))
        for side in ("l", "r"):
            if s.get(f"speed_{side}") == SPEED_MAX:
                name = "left" if side == "l" else "right"
                w.append((f"{name} speed saturated at {SPEED_MAX} mm/s: "
                          f"check ticks per wheel turn", YELLOW))
        if not self.map_letters:
            if self.lost_map:
                w.append((f"car rebooted, so map {self.lost_map} is forgotten: "
                          f"send 'map {self.lost_map}' again for barcode letters", YELLOW))
            elif self.unnamed_barcode:
                w.append(("barcode read but no accepted map seen since the dashboard "
                          "started: send 'map FULR' to show letters", YELLOW))
        if self.hb:
            link = self.hb["link"]
            if link < len(LINK_STATE) and LINK_STATE[link] != "ONLINE":
                w.append((f"link state {LINK_STATE[link]}", YELLOW))
            extra = self.hb["reconnects"] - (self.reconnects0 or 0)
            if extra > 0:
                w.append((f"{extra} reconnect(s) since the dashboard started", YELLOW))
            if self.hb["rssi_dbm"] and self.hb["rssi_dbm"] < -75:
                w.append((f"weak Wi-Fi ({self.hb['rssi_dbm']} dBm)", YELLOW))
        return w

    # ---- drawing ----
    def render(self, now, cols):
        """The panel as HEIGHT strings, each at most `cols` visible chars."""
        s, hb = self.status, self.hb
        g = (lambda k, dflt="-": dflt if s is None or s.get(k) is None else s[k])

        if self.online is None:
            online = ("waiting for car", YELLOW)
        else:
            online = (("ONLINE" if self.online else "OFFLINE")
                      + f" 0x{self.session:04X}", GREEN if self.online else RED)
        title = [(" IS33 car  ", TITLE), (f" {online[0]} ", online[1])]
        if hb:
            link = LINK_STATE[hb["link"]] if hb["link"] < len(LINK_STATE) else "?"
            rssi = "rssi n/a" if hb["rssi_dbm"] == 0 else f"{hb['rssi_dbm']} dBm"
            title.append((f" link {link} | {rssi} | up {hb['uptime_s']} s", ""))
        if self.t_status is not None:
            hz = self.status_hz(now)
            age = now - self.t_status
            fresh = hz and age <= self.STALE_S
            title.append((f" | status {hz:.1f} Hz" if fresh else " | status", ""))
            title.append((f", {age:.1f} s ago", RED if age > self.STALE_S else ""))

        if s is None:
            state_seg = [("-", DIM)]
        else:
            vs = s["vehicle_state"]
            name = VEH_STATE[vs] if vs < len(VEH_STATE) else f"?{vs}"
            colour = RED if name == "SAFE_STOP" else DIM if name in ("INIT", "IDLE") else GREEN
            state_seg = [(f"{name:<15}", colour + ";1")]
        if s is not None and s.get("estop"):
            src = f" ({self.estop_src})" if self.estop_src else ""
            estop = (f"LATCHED{src}", RED)
        else:
            estop = ("off" if s else "-", GREEN if s else DIM)

        faults = g("faults", None)
        if faults is None:
            fault_seg = [("-", DIM)]
        elif faults == 0:
            fault_seg = [("none", GREEN)]
        else:
            names = [n for b, n in FAULT_BITS.items() if faults & (1 << b)]
            fault_seg = [(", ".join(names) or "unknown", RED), (f"  (faults={faults})", DIM)]

        def wheel(side, label):
            rate = self.enc_rate(0 if side == "l" else 1, now)
            rate_txt = "" if rate is None else f" ({rate:+.0f}/s)"
            spd = g(f"speed_{side}")
            return [(f"{label} ", BOLD), (f"{spd:>4} mm/s", YELLOW if spd == SPEED_MAX else ""),
                    (f"  enc {g('enc_' + side):>5}{rate_txt:<10}", "")]

        line = g("line", None)
        if line is None:
            line_seg = [("-", DIM)]
        else:
            def sensor(name, on, n):
                return [(f"{name} ", ""), ("[ON ]" if on else "[off]", GREEN if on else DIM),
                        (f" {n} changes   ", DIM)]
            line_seg = (sensor("left", line & 2, self.line_changes[0])
                        + sensor("centre", line & 1, self.line_changes[1]))

        bc = g("barcode", None)
        bc_txt = "-" if bc is None else BARCODE[bc] if bc < len(BARCODE) else f"?{bc}"
        letter = self.last_letter or self.letter_for(bc)
        if letter and bc and bc != BC_INVALID:
            bc_txt = f"'{letter}' {bc_txt}"
        elif bc == BC_INVALID:
            bc_txt = (f"'{self.last_letter}' INVALID (not in map)" if self.last_letter
                      else "INVALID (not in map)")
        if self.map_sent and self.map_sent != self.map_letters:
            map_seg = [("map ", DIM), (describe_map(self.map_sent), YELLOW),
                       ("  (sent, waiting for the car)", DIM)]
        elif self.map_letters:
            map_seg = [("map ", DIM), (describe_map(self.map_letters), GREEN)]
        elif self.lost_map:
            map_seg = [("map ", DIM), (f"LOST on reboot (was {self.lost_map})", YELLOW)]
        else:
            map_seg = [("map ", DIM), ("none: every barcode = go straight", YELLOW)]

        scan_rows = []
        for i in range(self.SCAN_ROWS):
            label = "Scans     " if i == 0 else "          "
            k = len(self.scans) - 1 - i             # newest first
            if k < 0:
                text = (f"none yet ({self.scan_total} total)" if i == 0 else "")
                scan_rows.append([(label, DIM), (text, DIM)])
                continue
            t, text, colour = self.scans[k]
            stamp = time.strftime("%H:%M:%S", time.localtime(t))
            scan_rows.append([(label, DIM), (f"{stamp} ", DIM), (text, colour)]
                             + ([(f"  [{self.scan_total} total]", DIM)] if i == 0 else []))
        pitch = g("pitch", None)
        pitch_txt = "-" if pitch is None else f"{pitch:+d} deg"
        yaw = g("yaw_dps", None)
        yaw_txt = "-" if yaw is None else f"{yaw:+d} deg/s"
        mo = g("motion", None)
        motion_name = "-" if mo is None else MOTION[mo] if mo < len(MOTION) else f"?{mo}"
        os_ = g("obs_state", None)
        obs_name = "-" if os_ is None else OBS_STATE[os_] if os_ < len(OBS_STATE) else f"?{os_}"
        obs_cm = g("obs_cm")

        if hb:
            cmds = (f"ok {hb['cmds_ok']}  rejected {hb['cmds_rej']}  queue full "
                    f"{hb['queue_full']}  reconnects {hb['reconnects']}")
        else:
            cmds = "-"
        ack = [("", "")]
        if self.last_ack:
            t, name, result, rtt = self.last_ack
            ack = [("   last ACK ", DIM), (f"{name} {result}",
                    GREEN if result == "ACCEPTED" else RED),
                   (f" ({rtt}, {now - t:.0f} s ago)", DIM)]
        ev = [("-", DIM)]
        if self.last_event:
            t, d = self.last_event
            info = {k: v for k, v in d.items() if k not in ("session", "seq", "type")}
            v = info.get("value")
            if d["type"] == "ESTOP" and v is not None:
                info = f" by {EVT_SRC[v] if v < len(EVT_SRC) else v}"
            elif d["type"] == "BARCODE":
                info = " " + describe_barcode(d, self.map_letters)[0]
            elif d["type"] == "STATE_CHANGE":
                info = f" {info['from']} -> {info['to']}"
            else:
                info = " " + " ".join(f"{k}={x}" for k, x in info.items()) if info else ""
            ev = [(d["type"], BOLD), (info, ""),
                  (f"  ({now - t:.0f} s ago)", DIM)]

        rows = [
            title,
            [("State     ", DIM)] + state_seg + [("  E-STOP ", DIM), estop],
            [("Faults    ", DIM)] + fault_seg,
            [("Wheels    ", DIM)] + wheel("l", "L") + [("|  ", DIM)] + wheel("r", "R"),
            [("Line      ", DIM)] + line_seg,
            [("Barcode   ", DIM), (f"{bc_txt:<22}", "")] + map_seg,
        ] + scan_rows + [
            [("Tilt      ", DIM), (f"pitch {pitch_txt:<9}", ""), ("hump max ", DIM),
             (f"{g('hump_max_mm')} mm   ", ""), ("travel ", DIM),
             (f"{g('dist_cm')} cm", "")],
            [("Motion    ", DIM), (f"{motion_name:<14}",
                                    RED if motion_name in ("LIFTED", "OVERTURNED") else ""),
             ("turn ", DIM), (f"{yaw_txt:<12}", ""),
             ("lift signal ", DIM),
             (f"up {g('up_speed')} mm/s  tilt {g('tilt')} deg", "")],
            [("Obstacle  ", DIM), (f"{obs_name:<14}", RED if obs_name == "FAULT" else ""),
             ("nearest ", DIM), (f"{obs_cm} cm", YELLOW if obs_cm == OBS_CM_MAX else "")],
            [("Commands  ", DIM), (cmds, "")] + ack,
            [("Event     ", DIM)] + ev,
        ]
        warnings = self.checks(now)
        n_warn = self.HEIGHT - len(rows) - 1
        if not warnings:
            rows.append([("Checks    ", DIM), ("all OK", GREEN)])
            rows += [[]] * (n_warn - 1)
        else:
            shown = warnings[:n_warn]
            if len(warnings) > n_warn:          # squeeze the rest onto the last row
                rest = warnings[n_warn - 1:]
                shown = warnings[:n_warn - 1] + [(" | ".join(t for t, _ in rest),
                                                  rest[0][1])]
            for i, (text, colour) in enumerate(shown):
                rows.append([("Checks    " if i == 0 else "          ", DIM),
                             ("! " + text, colour)])
            rows += [[]] * (n_warn - len(shown))
        rows.append([("-" * cols, DIM)])
        return [fit(r, cols) for r in rows]

    # ---- terminal handling ----
    @staticmethod
    def supported():
        if not sys.stdout.isatty():
            return False
        if os.name == "nt":                    # turn on ANSI escapes in the console
            try:
                import ctypes
                k = ctypes.windll.kernel32
                h = k.GetStdHandle(-11)
                mode = ctypes.c_uint32()
                if not k.GetConsoleMode(h, ctypes.byref(mode)):
                    return False
                k.SetConsoleMode(h, mode.value | 0x0004)   # VIRTUAL_TERMINAL_PROCESSING
            except (OSError, AttributeError):
                return False
        rows = shutil.get_terminal_size().lines
        return rows >= Panel.HEIGHT + 6

    def _set_region(self, size):
        """Scroll only the rows below the panel; park the cursor at the bottom."""
        self.size = size
        self.out.write(f"\x1b[{self.HEIGHT + 1};{size.lines}r\x1b[{size.lines};1H")

    def start(self):
        with self.lock:
            self.out.write("\x1b[2J")
            self._set_region(shutil.get_terminal_size())
            self.out.flush()

    def draw(self):
        size = shutil.get_terminal_size()
        if size.lines < self.HEIGHT + 3:
            return                              # window too small to draw
        with self.data_lock:
            lines = self.render(time.time(), size.columns)
        with self.lock:
            if size != self.size:
                self._set_region(size)
            buf = ["\x1b7"]                     # save cursor (where you type)
            for i, text in enumerate(lines, start=1):
                buf.append(f"\x1b[{i};1H\x1b[2K{text}")
            buf.append("\x1b8")                 # back to where you were typing
            self.out.write("".join(buf))
            self.out.flush()

    def stop(self):
        with self.lock:
            self.out.write(f"\x1b[r\x1b[{shutil.get_terminal_size().lines};1H\n")
            self.out.flush()


class Dashboard:
    def __init__(self, args):
        self.args = args
        self.session = 0
        self.next_seq = self.first_seq()
        self.sent_at = {}                 # seq -> send time, for round-trip time
        self.map_pending = {}             # seq -> letters of a MAP not yet ACKed
        self.map_letters = None           # map the car accepted (letter lookup)
        # Map to keep on the car: --map, then the last one it accepted. The
        # car forgets its map on every reboot, so it is re-sent each time a
        # new session comes online (unless --no-auto-map).
        self.want_map = args.map
        self.lock = threading.Lock()
        self.out_lock = threading.Lock()  # one writer at a time on the terminal
        self.panel = Panel(self.out_lock) if args.panel else None
        self.running = True
        self.log_file = open(args.log, "a", newline="")
        self.log = csv.writer(self.log_file)
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id=f"is33-dashboard-{int(time.time())}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    @staticmethod
    def first_seq() -> int:
        """First command seq for a session. The car rejects any seq not above
        the last one it accepted (STALE_SEQ), and it may already have had
        commands from a dashboard window opened earlier. Seeding from the
        clock (seconds) puts a newly opened window above an older one, as
        long as the old one sent fewer commands than seconds have passed."""
        return 1 + int(time.time()) % 60000

    def say(self, text: str):
        with self.out_lock:
            print(text, flush=True)

    def to_panel(self, method: str, *args):
        """Feed the live panel. A panel bug must never stop the dashboard."""
        if self.panel is None:
            return
        try:
            with self.panel.data_lock:
                getattr(self.panel, method)(*args)
        except Exception as exc:                        # noqa: BLE001
            self.say(f"[dash] panel error ({method}): {exc!r}")

    def draw_loop(self):
        while self.running:
            try:
                self.panel.draw()
            except Exception as exc:                    # noqa: BLE001
                self.say(f"[dash] panel draw error: {exc!r}")
            time.sleep(0.25)

    def write_log(self, kind, payload: bytes, decoded):
        self.log.writerow([f"{time.time():.3f}", kind, payload.hex(),
                           json.dumps(decoded)])
        self.log_file.flush()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        self.say(f"[dash] connected to broker ({reason_code})")
        client.subscribe("is33/car/#", qos=1)

    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == T_ONLINE:
                d = decode_online(msg.payload)
                lost, new_session = None, False
                with self.lock:
                    if d["online"] and d["session"] != self.session:
                        self.session, self.next_seq = d["session"], self.first_seq()
                        lost, self.map_letters = self.map_letters, None  # rebooted car forgot the map
                        new_session = True
                self.to_panel("on_online", d)
                self.say(f"[online] {'ONLINE' if d['online'] else 'OFFLINE'} "
                      f"session=0x{d['session']:04X}")
                resend = self.want_map if self.args.auto_map else None
                if new_session and resend:
                    self.write_log(msg.topic, msg.payload, d)
                    self.say(f"[dash] new car session: sending map {resend} "
                             f"({describe_map(resend)}) automatically")
                    self.send("map", letters=resend)
                    return
                if lost:
                    self.say(f"[dash] car rebooted: map {lost} forgotten, barcode "
                             f"letters unknown until you send 'map {lost}' again")
            elif msg.topic == T_STATUS:
                if msg.payload[:1] == b"{":
                    d = json.loads(msg.payload)       # JSON baseline mode
                    d["vehicle_state"] = d.pop("state", 0)
                else:
                    d = decode_status(msg.payload)
                self.to_panel("on_status", d, time.time())
                if not self.args.quiet and self.panel is None:
                    self.say(f"[status] {VEH_STATE[d['vehicle_state']]}"
                          f" estop={d.get('estop')} spd={d.get('speed_l')}/{d.get('speed_r')}"
                          f" dist={d.get('dist_cm')}cm pitch={d.get('pitch')}"
                          f" hump={d.get('hump_max_mm')}mm obs={d.get('obs_cm')}cm"
                          f" yaw={d.get('yaw_dps')}dps motion={d.get('motion')}"
                          f" ({len(msg.payload)} B)")
            elif msg.topic == T_HEARTBEAT:
                d = decode_heartbeat(msg.payload)
                self.to_panel("on_heartbeat", d)
                rssi = "n/a" if d["rssi_dbm"] == 0 else f"{d['rssi_dbm']}dBm"
                self.say(f"[heartbeat] up={d['uptime_s']}s rssi={rssi} "
                      f"link={LINK_STATE[d['link']]} ok={d['cmds_ok']} "
                      f"rej={d['cmds_rej']} qfull={d['queue_full']} "
                      f"reconn={d['reconnects']}")
            elif msg.topic == T_EVENT:
                d = decode_event(msg.payload)
                v = d.get("value")
                if d["type"] == "BARCODE":
                    if ("letter" not in d and not d.get("seen_not_decoded")
                            and self.map_letters and v in MAP_POS):
                        d["letter"] = self.map_letters[MAP_POS[v]]   # older firmware
                        d["letter_from"] = "map"
                    text, _ = describe_barcode(d, self.map_letters)
                    d["summary"] = text                 # also in the CSV log
                    self.to_panel("on_event", d, time.time())
                    self.say(f"[BARCODE] {text}")
                else:
                    self.to_panel("on_event", d, time.time())
                    self.say(f"[EVENT] {d}")
            elif msg.topic == T_ACK:
                d = decode_ack(msg.payload)
                letters = self.map_pending.pop(d["cmd_seq"], None)
                accepted = (letters and d["cmd"] == CMD["map"]
                            and RESULT[d["result"]] == "ACCEPTED")
                if accepted:
                    self.map_letters = letters
                    self.want_map = letters       # re-sent after a reboot
                    self.to_panel("on_map", letters)
                elif letters and d["cmd"] == CMD["map"]:
                    self.to_panel("on_map_refused")
                sent = self.sent_at.pop(d["cmd_seq"], None)
                rtt = f"{(time.time() - sent) * 1000:.0f} ms" if sent else "n/a"
                d["rtt_ms"] = None if sent is None else round((time.time() - sent) * 1000, 1)
                self.to_panel("on_ack", time.time(), CMD_NAME.get(d["cmd"], d["cmd"]),
                              RESULT[d["result"]], rtt)
                self.say(f"[ACK] {CMD_NAME.get(d['cmd'], d['cmd'])} seq={d['cmd_seq']} "
                      f"-> {RESULT[d['result']]} (round trip {rtt})")
                if accepted:
                    self.say(f"[dash] car ACCEPTED map {letters}, now in use:\n"
                             f"{map_table(letters)}")
                elif letters and d["cmd"] == CMD["map"]:
                    kept = (f"the car keeps map {self.map_letters} "
                            f"({describe_map(self.map_letters)})" if self.map_letters
                            else "the car has no map: every barcode = go straight on")
                    self.say(f"[dash] map {letters} NOT accepted ({RESULT[d['result']]}): "
                             f"{kept}. Send 'map {letters}' again.")
            elif msg.topic == T_CMD:
                # Commands from ANY dashboard (ours echoed back too): note a
                # MAP so its letters are known when the car ACKs it.
                p = msg.payload
                if len(p) == 9 and (p[0] & 0x0F) == CMD["map"]:
                    letters = p[5:9].decode("ascii", "replace")
                    self.map_pending[int.from_bytes(p[3:5], "big")] = letters
                return
            else:
                d = {}
            self.write_log(msg.topic, msg.payload, d)
        except (ValueError, IndexError, KeyError) as exc:
            text = msg.payload.decode("utf-8", "replace")
            if msg.payload and text.isprintable():
                self.say(f"[{msg.topic}] (text) {text}")          # early stages send text
                self.write_log(msg.topic, msg.payload, {"text": text})
            else:
                self.say(f"[dash] could not decode {msg.topic}: {exc} ({msg.payload.hex()})")

    def send(self, name: str, session=None, seq=None, letters: str = ""):
        with self.lock:
            session = self.session if session is None else session
            if seq is None:
                seq = self.next_seq
                self.next_seq += 1
        payload = encode_cmd(CMD[name], session, seq, letters)
        self.sent_at[seq] = time.time()
        if name == "map":
            self.map_pending[seq] = letters     # applied when the car ACKs it
            self.to_panel("on_map_sent", letters)
        # Commands are NEVER retained, so the broker never replays them.
        self.client.publish(T_CMD, payload, qos=1, retain=False)
        self.write_log("sent_cmd", payload, {"cmd": name, "session": session, "seq": seq,
                                             "letters": letters})
        self.say(f"[dash] sent {name.upper()} {letters} session=0x{session:04X} seq={seq}")

    def show_help(self):
        cur = (f"current map: {describe_map(self.map_letters)}" if self.map_letters
               else "current map: none known (send 'map FULR')")
        self.say(BARCODE_HELP + "\n" + cur)

    def run(self):
        if self.panel is not None:
            self.panel.start()
            threading.Thread(target=self.draw_loop, daemon=True).start()
        self.client.connect(self.args.broker, self.args.port, keepalive=30)
        self.client.loop_start()
        if self.args.clear_retained:
            self.client.publish(T_CMD, b"", qos=1, retain=True)  # wipe any retained cmd
        self.say("Commands: start | stop | e (E-STOP) | reset | map FULR (forward, U-turn, "
              "left, right) | stale (old-session START) | replay (repeat last seq) | "
              "burst N (N STARTs fast) | help (barcode letters + limits) | q")
        if self.want_map and self.args.auto_map:
            self.say(f"Map {self.want_map} ({describe_map(self.want_map)}) is sent when the "
                     f"car is online and again after every reboot. Type 'help' for barcodes.")
        else:
            self.say("Send 'map FULR' once: it is re-sent automatically after a car "
                     "reboot. Type 'help' for barcodes.")
        try:
            for line in sys.stdin:
                parts = line.strip().lower().split()
                if not parts:
                    continue
                cmd = parts[0]
                if cmd == "q":
                    break
                elif cmd in ("help", "h", "?"):
                    self.show_help()
                elif cmd in ("start", "stop", "reset"):
                    self.send(cmd)
                elif cmd == "map":
                    letters = parts[1].upper() if len(parts) > 1 else ""
                    if (len(letters) != 4 or not set(letters) <= MAP_CHARS
                            or len(set(letters)) != 4):
                        self.say("map needs 4 different letters/digits, e.g. map FULR")
                    else:
                        self.say(f"[dash] map {letters} -> sending to the car:\n"
                                 f"{map_table(letters)}")
                        self.send("map", letters=letters)
                elif cmd in ("e", "estop"):
                    self.send("estop")
                elif cmd == "stale":
                    self.send("start", session=(self.session ^ 0x5A5A) or 1)
                elif cmd == "replay":
                    self.send("start", seq=max(self.next_seq - 1, 1))
                elif cmd == "burst":
                    for _ in range(int(parts[1]) if len(parts) > 1 else 8):
                        self.send("start")
                else:
                    self.say("unknown command (type 'help')")
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if self.panel is not None:
                self.panel.stop()
            self.client.loop_stop()
            self.client.disconnect()
            self.log_file.close()


def main():
    p = argparse.ArgumentParser(description="IS33 car MQTT dashboard",
                                epilog=BARCODE_HELP,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--broker", default="127.0.0.1")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--log", default="tlm_log.csv")
    p.add_argument("--quiet", action="store_true",
                   help="hide 5 Hz status lines (only without the live panel)")
    p.add_argument("--no-panel", dest="panel", action="store_false",
                   help="plain scrolling output instead of the live status panel")
    p.add_argument("--clear-retained", action="store_true",
                   help="erase any retained message on the command topic")
    p.add_argument("--map", type=str.upper, metavar="FULR",
                   help="barcode map (forward, U-turn, left, right) to send as soon "
                        "as the car is online and again after every car reboot")
    p.add_argument("--no-auto-map", dest="auto_map", action="store_false",
                   help="do not re-send the map automatically after a car reboot")
    args = p.parse_args()
    if args.map and (len(args.map) != 4 or not set(args.map) <= MAP_CHARS
                     or len(set(args.map)) != 4):
        p.error("--map needs 4 different letters/digits, e.g. --map FULR")
    if args.panel and not Panel.supported():
        print("[dash] terminal can't show the live panel (not a console, or under "
              f"{Panel.HEIGHT + 6} rows); using plain output")
        args.panel = False
    Dashboard(args).run()


if __name__ == "__main__":
    main()
