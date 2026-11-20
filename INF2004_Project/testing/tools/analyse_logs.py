"""Mine the dashboard CSV logs for evidence and data-driven tuning values.

Usage (from the repo root):
    py testing/tools/analyse_logs.py testing/subsystem1/logs
    py testing/tools/analyse_logs.py testing/subsystem1/logs/2026-11-19_dashboard.csv

Reads the CSV written by testing/subsystem1/tools/dashboard.py (one row per
MQTT message: time, topic, raw hex, decoded JSON) and prints, per log:

  S1  status frames lost (sequence gaps), status / heartbeat intervals,
      command ACK round trip, reconnects
  S2  wheel speeds on straight sections, left/right encoder balance,
      START -> 90 % of cruising speed (step response), brake events
  S4  tilt / pitch at rest and while driving, HUMP and LIFTED events
  S5  obstacle profiles (and scans where every ray read the same: servo not
      sweeping), trigger -> stop distance, no-echo rate, S5 faults

Notes:
  * The 1 Nov log is from the stub build (synthetic data): it is flagged.
  * The log has no requested wheel speed, so speed tracking is only judged
    where the target is known (S3 line following asks for 70 mm/s).
  * up_speed in STATUS is S4's peak, not a live value: not used for limits.
"""
import collections
import csv
import datetime
import glob
import json
import os
import statistics as st
import sys

S5_STATES = ["MONITOR", "WAIT_STOP", "COARSE", "FINE", "PROFILE_READY", "BYPASS",
             "FAULT", "REVERSE"]
S5_FAULTS = {1: "MONITOR_SENSOR", 2: "SCAN_INVALID", 3: "STOP_TIMEOUT",
             4: "EVENT_DELIVERY", 5: "NO_SAFE_SIDE", 6: "BYPASS"}
ESTOP_SRC = {0: "button", 1: "remote", 2: "IMU (lift)", 3: "motion (S2)",
             4: "ultrasonic", 5: "line"}


def load(path):
    rows = []
    with open(path, encoding="utf-8", errors="replace", newline="") as f:
        for r in csv.reader(f):
            if len(r) < 4:
                continue
            try:
                rows.append((float(r[0]), r[1], json.loads(r[3])))
            except (ValueError, json.JSONDecodeError):
                continue
    return rows


def pct(xs, p):
    xs = sorted(xs)
    return xs[min(len(xs) - 1, int(p / 100.0 * len(xs)))] if xs else float("nan")


def summary(xs):
    if not xs:
        return "n=0"
    return f"n={len(xs)} median={st.median(xs):.0f} p95={pct(xs, 95):.0f} max={max(xs):.0f}"


def hhmmss(t):
    return datetime.datetime.fromtimestamp(t).strftime("%H:%M:%S")


def status(rows):
    return [(t, d) for t, tp, d in rows if tp.endswith("/status")]


def events(rows, kind=None):
    return [(t, d) for t, tp, d in rows
            if tp.endswith("/event") and (kind is None or d.get("type") == kind)]


def s1_link(rows):
    s = status(rows)
    hb = [(t, d) for t, tp, d in rows if tp.endswith("/heartbeat")]
    acks = [d for _, tp, d in rows if tp.endswith("/ack")]
    gaps, lost, got, last = [], 0, 0, {}
    for t, d in s:
        sess, seq = d.get("session"), d.get("seq")
        if sess in last and seq is not None:
            pt, pseq = last[sess]
            step = (seq - pseq) % 256
            if 0 < step < 50:
                lost += step - 1
                got += 1
                if step == 1:
                    gaps.append((t - pt) * 1000.0)
        last[sess] = (t, seq)
    hb_gaps = [(hb[i][0] - hb[i - 1][0]) * 1000.0 for i in range(1, len(hb))
               if hb[i][1].get("session") == hb[i - 1][1].get("session")]
    rtt = [a["rtt_ms"] for a in acks if a.get("rtt_ms") is not None]
    slow = [x for x in rtt if x > 5000]
    print(f"  S1 status frames {len(s)}, lost {lost}/{lost + got} "
          f"({100.0 * lost / max(1, lost + got):.1f} %)")
    print(f"     status interval ms   {summary(gaps)}")
    print(f"     heartbeat interval ms {summary([g for g in hb_gaps if g < 10000])}")
    print(f"     ACK round trip ms    {summary([x for x in rtt if x <= 5000])}"
          + (f"  (+{len(slow)} over 5 s: replies after a reconnect?)" if slow else ""))
    print(f"     ACK results {dict(collections.Counter(a.get('result') for a in acks))}, "
          f"max reconnects {max((d.get('reconnects', 0) for _, d in hb), default=0)}")


def s2_motion(rows):
    s = status(rows)
    straight = [d for _, d in s if d.get("speed_l", 0) > 20 and d.get("speed_r", 0) > 20
                and abs(d["speed_l"] - d["speed_r"]) < 10]
    if len(straight) >= 20:
        print(f"  S2 straight-section speed L {summary([d['speed_l'] for d in straight])}")
        print(f"                            R {summary([d['speed_r'] for d in straight])}")
    el = er = 0
    for (_, a), (_, b) in zip(s, s[1:]):
        if (a.get("session") == b.get("session") and a.get("speed_l", 0) > 20
                and abs(a.get("speed_l", 0) - a.get("speed_r", 0)) < 10):
            dl, dr = b["enc_l"] - a["enc_l"], b["enc_r"] - a["enc_r"]
            if 0 <= dl < 400 and 0 <= dr < 400:
                el += dl
                er += dr
    if el > 1000:
        print(f"     encoder edges on straights L {el} R {er}: R/L = {er / el:.3f} "
              "(check per-wheel ticks/rev with S2-C01)")
    rises = []
    for t0 in [t for t, tp, d in rows if tp == "sent_cmd" and d.get("cmd") == "start"]:
        seg = [(t - t0, d) for t, d in s if 0.0 <= t - t0 <= 4.0]
        cruise = [0.5 * (d["speed_l"] + d["speed_r"]) for dt, d in seg if dt > 2.0]
        if not cruise or st.median(cruise) < 30:
            continue
        target = 0.9 * st.median(cruise)
        hit = next((dt for dt, d in seg if 0.5 * (d["speed_l"] + d["speed_r"]) >= target), None)
        if hit is not None:
            rises.append(hit * 1000.0)
    if rises:
        print(f"     START -> 90 % of cruising speed, ms {summary(rises)}")
    src = collections.Counter(d.get("value") for _, d in events(rows, "ESTOP"))
    if src:
        print("     brake events: " + ", ".join(f"{ESTOP_SRC.get(k, k)} {v}" for k, v in src.items()))


def s4_imu(rows):
    s = [d for _, d in status(rows)]
    drive = [d for d in s if d.get("speed_l", 0) > 30 and d.get("speed_r", 0) > 30]
    if drive:
        print(f"  S4 driving |pitch| deg {summary([abs(d.get('pitch', 0)) for d in drive])}")
        if any("tilt" in d for d in drive):
            print(f"     driving tilt deg     {summary([d.get('tilt', 0) for d in drive])}")
    for t, d in events(rows):
        if d.get("type") in ("HUMP", "HUMP_PEAK", "LIFTED", "IMPACT"):
            before = [x for tt, x in status(rows) if 0 <= t - tt <= 3.0]
            spd = max((max(x.get("speed_l", 0), x.get("speed_r", 0)) for x in before), default=0)
            info = d.get("height_mm", d.get("cause", ""))
            print(f"     {hhmmss(t)} {d['type']:8s} {info!s:8s} max wheel speed in the 3 s before: {spd} mm/s")


def s5_ultrasonic(rows):
    s = status(rows)
    for t, d in events(rows, "OBSTACLE"):
        flat = (d.get("clear_l_cm") == d.get("clear_r_cm")
                and abs(d.get("clear_l_cm", 0) - d.get("nearest_cm", 0)) <= 4)
        print(f"  S5 {hhmmss(t)} profile: nearest {d.get('nearest_cm')} cm @ {d.get('centre_deg')} deg, "
              f"width {d.get('width_cm')} cm, clear L {d.get('clear_l_cm')} R {d.get('clear_r_cm')}, "
              f"side {d.get('decision')}" + ("   <- every ray the same: servo not sweeping?" if flat else ""))
    obs = [d["obs_cm"] for _, d in s if "obs_cm" in d]
    if obs:
        print(f"     no echo in {100.0 * sum(1 for x in obs if x >= 511) / len(obs):.0f} % of readings")
    for i in range(1, len(s)):
        (_, a), (t1, b) = s[i - 1], s[i]
        if a.get("obs_state", 0) == 0 and b.get("obs_state", 0) == 1:
            stop = next(((t, d) for t, d in s[i:i + 25]
                         if d.get("speed_l") == 0 and d.get("speed_r") == 0), None)
            if stop and a.get("obs_cm", 511) < 511:
                v = 0.5 * (a.get("speed_l", 0) + a.get("speed_r", 0))
                print(f"     stop: triggered at {a['obs_cm']} cm ({v:.0f} mm/s), at rest at "
                      f"{stop[1].get('obs_cm')} cm, {1000.0 * (stop[0] - t1):.0f} ms later")
    faults = collections.Counter(d["s5_fault"] for _, d in s if d.get("s5_fault"))
    if faults:
        print("     S5 fault (status frames): "
              + ", ".join(f"{S5_FAULTS.get(k, k)} {v}" for k, v in sorted(faults.items())))


def main(argv):
    if len(argv) != 2:
        print(__doc__)
        return 2
    target = argv[1]
    files = sorted(glob.glob(os.path.join(target, "*.csv"))) if os.path.isdir(target) else [target]
    for path in files:
        rows = load(path)
        if len(rows) < 100:
            continue
        s = [d for _, d in status(rows)]
        print(f"\n===== {os.path.basename(path)}: {len(rows)} messages")
        still = [(a, b) for a, b in zip(s, s[1:])
                 if b.get("speed_l") == 0 and b.get("speed_r") == 0]
        creeping = sum(1 for a, b in still if b.get("enc_l", 0) > a.get("enc_l", 0))
        if still and creeping > len(still) // 2:
            print("  (encoders count up while both speeds read 0: stub/synthetic data, "
                  "treat with care)")
        s1_link(rows)
        s2_motion(rows)
        s4_imu(rows)
        s5_ultrasonic(rows)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
