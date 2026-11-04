#!/usr/bin/env python3
r"""
IS33 car dashboard (laptop side) -- Buddy 1 / subsystem 1.

  py -m pip install paho-mqtt                       (Windows)
  py testing\subsystem1\tools\dashboard.py --broker 127.0.0.1

Shows decoded telemetry, sends START / STOP / E-STOP, and logs everything
(with laptop timestamps) to a CSV file you can use as test evidence.

Keep the field tables below in sync with app_program/subsystem1/tlm_codec.c
and app_program/subsystem1/MQTT_PROTOCOL.md.
"""
import argparse
import csv
import json
import sys
import threading
import time

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
EVENT = {1: "STATE_CHANGE", 2: "BARCODE", 3: "HUMP_PEAK", 4: "OBSTACLE",
         5: "IMPACT", 6: "ESTOP"}

# (field name, bits) in wire order, after the 28-bit header.
STATUS_FIELDS = [("vehicle_state", 3), ("estop", 1), ("line", 2), ("barcode", 3),
                 ("speed_l", 10), ("speed_r", 10), ("enc_l", 16), ("enc_r", 16),
                 ("dist_cm", 12), ("pitch", 7), ("hump_max_mm", 8),
                 ("obs_state", 3), ("obs_cm", 9), ("faults", 8)]
HEARTBEAT_FIELDS = [("uptime_s", 16), ("rssi_neg", 7), ("link", 3), ("cmds_ok", 8),
                    ("cmds_rej", 8), ("queue_full", 8), ("reconnects", 8)]
ACK_FIELDS = [("cmd_seq", 16), ("cmd", 4), ("result", 4)]
OBSTACLE_FIELDS = [("nearest_cm", 9), ("centre_deg", 8), ("width_cm", 8),
                   ("clear_l_cm", 9), ("clear_r_cm", 9), ("decision", 3)]


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
    elif etype == 3:
        d["height_mm"] = r.get(8)
    elif etype == 4:
        d.update(r.fields(OBSTACLE_FIELDS))
    return d


def decode_online(data: bytes) -> dict:
    r = BitReader(data)
    ver = r.get(4)
    if ver != VERSION:
        raise ValueError(f"protocol version {ver}, expected {VERSION}")
    online = r.get(1)
    r.get(3)
    return {"version": ver, "online": bool(online), "session": r.get(16)}


def encode_cmd(code: int, session: int, seq: int, letters: str = "") -> bytes:
    """5 bytes: version(4) cmd(4) session(16) seq(16).
    MAP adds 4 ASCII letters (forward, U-turn, left, right): 9 bytes."""
    return (bytes([(VERSION << 4) | code]) + session.to_bytes(2, "big")
            + seq.to_bytes(2, "big") + letters.encode("ascii"))


class Dashboard:
    def __init__(self, args):
        self.args = args
        self.session = 0
        self.next_seq = 1
        self.sent_at = {}                 # seq -> send time, for round-trip time
        self.lock = threading.Lock()
        self.log_file = open(args.log, "a", newline="")
        self.log = csv.writer(self.log_file)
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2,
                                  client_id=f"is33-dashboard-{int(time.time())}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def write_log(self, kind, payload: bytes, decoded):
        self.log.writerow([f"{time.time():.3f}", kind, payload.hex(),
                           json.dumps(decoded)])
        self.log_file.flush()

    def on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"[dash] connected to broker ({reason_code})")
        client.subscribe("is33/car/#", qos=1)

    def on_message(self, client, userdata, msg):
        try:
            if msg.topic == T_ONLINE:
                d = decode_online(msg.payload)
                with self.lock:
                    if d["online"] and d["session"] != self.session:
                        self.session, self.next_seq = d["session"], 1
                print(f"[online] {'ONLINE' if d['online'] else 'OFFLINE'} "
                      f"session=0x{d['session']:04X}")
            elif msg.topic == T_STATUS:
                if msg.payload[:1] == b"{":
                    d = json.loads(msg.payload)       # JSON baseline mode
                    d["vehicle_state"] = d.pop("state", 0)
                else:
                    d = decode_status(msg.payload)
                if not self.args.quiet:
                    print(f"[status] {VEH_STATE[d['vehicle_state']]}"
                          f" estop={d.get('estop')} spd={d.get('speed_l')}/{d.get('speed_r')}"
                          f" dist={d.get('dist_cm')}cm pitch={d.get('pitch')}"
                          f" hump={d.get('hump_max_mm')}mm obs={d.get('obs_cm')}cm"
                          f" ({len(msg.payload)} B)")
            elif msg.topic == T_HEARTBEAT:
                d = decode_heartbeat(msg.payload)
                rssi = "n/a" if d["rssi_dbm"] == 0 else f"{d['rssi_dbm']}dBm"
                print(f"[heartbeat] up={d['uptime_s']}s rssi={rssi} "
                      f"link={LINK_STATE[d['link']]} ok={d['cmds_ok']} "
                      f"rej={d['cmds_rej']} qfull={d['queue_full']} "
                      f"reconn={d['reconnects']}")
            elif msg.topic == T_EVENT:
                d = decode_event(msg.payload)
                print(f"[EVENT] {d}")
            elif msg.topic == T_ACK:
                d = decode_ack(msg.payload)
                sent = self.sent_at.pop(d["cmd_seq"], None)
                rtt = f"{(time.time() - sent) * 1000:.0f} ms" if sent else "n/a"
                d["rtt_ms"] = None if sent is None else round((time.time() - sent) * 1000, 1)
                print(f"[ACK] {CMD_NAME.get(d['cmd'], d['cmd'])} seq={d['cmd_seq']} "
                      f"-> {RESULT[d['result']]} (round trip {rtt})")
            elif msg.topic == T_CMD:
                return                                  # our own commands echoed back
            else:
                d = {}
            self.write_log(msg.topic, msg.payload, d)
        except (ValueError, IndexError, KeyError) as exc:
            text = msg.payload.decode("utf-8", "replace")
            if msg.payload and text.isprintable():
                print(f"[{msg.topic}] (text) {text}")          # early stages send text
                self.write_log(msg.topic, msg.payload, {"text": text})
            else:
                print(f"[dash] could not decode {msg.topic}: {exc} ({msg.payload.hex()})")

    def send(self, name: str, session=None, seq=None, letters: str = ""):
        with self.lock:
            session = self.session if session is None else session
            if seq is None:
                seq = self.next_seq
                self.next_seq += 1
        payload = encode_cmd(CMD[name], session, seq, letters)
        self.sent_at[seq] = time.time()
        # Commands are NEVER retained, so the broker never replays them.
        self.client.publish(T_CMD, payload, qos=1, retain=False)
        self.write_log("sent_cmd", payload, {"cmd": name, "session": session, "seq": seq,
                                             "letters": letters})
        print(f"[dash] sent {name.upper()} {letters} session=0x{session:04X} seq={seq}")

    def run(self):
        self.client.connect(self.args.broker, self.args.port, keepalive=30)
        self.client.loop_start()
        if self.args.clear_retained:
            self.client.publish(T_CMD, b"", qos=1, retain=True)  # wipe any retained cmd
        print("Commands: start | stop | e (E-STOP) | reset | map FULR (forward, U-turn, "
              "left, right) | stale (old-session START) | replay (repeat last seq) | "
              "burst N (N STARTs fast) | q")
        try:
            for line in sys.stdin:
                parts = line.strip().lower().split()
                if not parts:
                    continue
                cmd = parts[0]
                if cmd == "q":
                    break
                elif cmd in ("start", "stop", "reset"):
                    self.send(cmd)
                elif cmd == "map":
                    letters = parts[1].upper() if len(parts) > 1 else ""
                    if (len(letters) != 4 or not set(letters) <= MAP_CHARS
                            or len(set(letters)) != 4):
                        print("map needs 4 different letters/digits, e.g. map FULR")
                    else:
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
                    print("unknown command")
        finally:
            self.client.loop_stop()
            self.client.disconnect()
            self.log_file.close()


def main():
    p = argparse.ArgumentParser(description="IS33 car MQTT dashboard")
    p.add_argument("--broker", default="127.0.0.1")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--log", default="tlm_log.csv")
    p.add_argument("--quiet", action="store_true", help="hide 5 Hz status lines")
    p.add_argument("--clear-retained", action="store_true",
                   help="erase any retained message on the command topic")
    Dashboard(p.parse_args()).run()


if __name__ == "__main__":
    main()
