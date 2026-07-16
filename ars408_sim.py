#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ars408_sim — ARS408 CAN frame simulator for vcan0
=================================================
Generates realistic Continental ARS408 traffic on a virtual CAN bus so
ARS_MiviaCar / MIRA can be tested without the physical radar.

Simulates for each radar (front SensorId 0, rear SensorId 1):
  * RadarState (0x201 + offset) at 1 Hz
  * Object_0_Status / Object_1_General / Object_2_Quality /
    Object_3_Extended at ~14 Hz (72 ms cycle, like the real sensor)
  * A small moving scene: cars, a truck, a pedestrian, a bicycle

Setup (inside the container, as root):
    ip link add dev vcan0 type vcan 2>/dev/null || true
    ip link set up vcan0

Usage:
    python3 ars408_sim.py                 # front + rear on vcan0
    python3 ars408_sim.py --single 0      # one radar only
    python3 ars408_sim.py --can vcan1

Author: Abdelmoutalib Douadi — MIVIA Lab, UNISA (2026)
License: MIT
"""

import argparse
import math
import socket
import struct
import time

CAN_FMT = "<IB3x8s"


def send(sock, can_id: int, data: bytes):
    sock.send(struct.pack(CAN_FMT, can_id, 8, data.ljust(8, b"\x00")))


# --------------------------------------------------------------------------- #
#  Encoders (inverse of the ARS408 Motorola decoding)
# --------------------------------------------------------------------------- #
def enc_radar_state(sensor_id: int) -> bytes:
    d = bytearray(8)
    d[0] = (1 << 6) | (1 << 7)                  # NVM read/write ok
    max_dist_raw = 500 // 2                     # 250 raw → decodes to 500 m
    d[1] = (max_dist_raw >> 2) & 0xFF
    d[2] = (max_dist_raw & 0x03) << 6           # no error flags
    power = 0                                   # standard Tx power
    d[3] = (power >> 1) & 0x03
    d[4] = ((power & 1) << 7) | (sensor_id & 0x07) | (0 << 4)  # sort idx 0
    output_type = 1                             # objects
    d[5] = (output_type << 2) | (1 << 4) | (1 << 5) | (0 << 6)
    d[7] = (0 << 2)                             # RCS threshold standard
    return bytes(d)


def enc_obj_status(n: int) -> bytes:
    return bytes([n & 0xFF]) + b"\x00" * 7


def enc_obj_general(oid, dist_long, dist_lat, vrel_long, vrel_lat,
                    dyn_prop, rcs) -> bytes:
    dl = max(0, min(0x1FFF, int(round((dist_long + 500.0) / 0.2))))
    la = max(0, min(0x7FF, int(round((dist_lat + 204.6) / 0.2))))
    vl = max(0, min(0x3FF, int(round((vrel_long + 128.0) / 0.25))))
    va = max(0, min(0x1FF, int(round((vrel_lat + 64.0) / 0.25))))
    rc = max(0, min(0xFF, int(round((rcs + 64.0) / 0.5))))
    d = bytearray(8)
    d[0] = oid & 0xFF
    d[1] = (dl >> 5) & 0xFF
    d[2] = ((dl & 0x1F) << 3) | ((la >> 8) & 0x07)
    d[3] = la & 0xFF
    d[4] = (vl >> 2) & 0xFF
    d[5] = ((vl & 0x03) << 6) | ((va >> 3) & 0x3F)
    d[6] = ((va & 0x07) << 5) | (dyn_prop & 0x07)
    d[7] = rc
    return bytes(d)


def enc_obj_quality(oid, prob_exist=6, meas_state=2) -> bytes:
    d = bytearray(8)
    d[0] = oid & 0xFF
    d[6] = ((prob_exist & 0x07) << 5) | ((meas_state & 0x07) << 2)
    return bytes(d)


def enc_obj_extended(oid, obj_class) -> bytes:
    d = bytearray(8)
    d[0] = oid & 0xFF
    d[3] = obj_class & 0x07
    return bytes(d)


# --------------------------------------------------------------------------- #
#  Scene: a few moving targets per radar
# --------------------------------------------------------------------------- #
# (class, base_dist_long, dist_lat, vrel_long, rcs, dyn_prop)
SCENE = [
    (1, 25.0,  -1.8,  3.0,  12.0, 0),   # car overtaking left
    (1, 60.0,   1.8, -2.0,  10.0, 0),   # car ahead braking
    (2, 90.0,   0.0, -0.5,  30.0, 0),   # truck far ahead
    (3, 12.0,   4.0,  0.0,  -5.0, 1),   # pedestrian on the right
    (5, 18.0,  -5.5,  1.0,  -2.0, 0),   # bicycle left
    (0, 40.0,   7.0,  0.0,   5.0, 1),   # static point (pole)
]


def main():
    ap = argparse.ArgumentParser(description="ARS408 vcan simulator")
    ap.add_argument("--can", default="vcan0")
    ap.add_argument("--front", type=int, default=0)
    ap.add_argument("--rear", type=int, default=1)
    ap.add_argument("--single", type=int, default=None)
    args = ap.parse_args()

    sensors = [args.single] if args.single is not None \
        else [args.front, args.rear]

    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((args.can,))
    except OSError as e:
        raise SystemExit(
            f"Cannot open {args.can}: {e}\n"
            "Create it first:  ip link add dev vcan0 type vcan && "
            "ip link set up vcan0")

    print(f"[ars408_sim] sending on {args.can} — SensorIds {sensors} "
          "(Ctrl+C to stop)")
    t0 = time.monotonic()
    last_state = 0.0
    cycle = 0.072  # 72 ms, ~13.9 Hz like the real ARS408
    while True:
        t = time.monotonic() - t0
        do_state = (time.monotonic() - last_state) >= 1.0
        for sid in sensors:
            off = sid * 0x10
            if do_state:
                send(sock, 0x201 + off, enc_radar_state(sid))
            phase = 0.0 if sid == sensors[0] else 1.7
            send(sock, 0x60A + off, enc_obj_status(len(SCENE)))
            for oid, (cls, d0, lat, v, rcs, dyn) in enumerate(SCENE):
                dist = d0 + v * (t % 20.0) + 2.0 * math.sin(t / 3.0 + phase)
                dist = max(1.0, min(240.0, dist))
                send(sock, 0x60B + off,
                     enc_obj_general(oid, dist, lat, v,
                                     0.3 * math.sin(t + oid), dyn, rcs))
                send(sock, 0x60C + off, enc_obj_quality(oid))
                send(sock, 0x60D + off, enc_obj_extended(oid, cls))
        if do_state:
            last_state = time.monotonic()
        time.sleep(cycle)


if __name__ == "__main__":
    main()
