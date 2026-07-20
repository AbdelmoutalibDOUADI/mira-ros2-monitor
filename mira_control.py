#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MIRA Control Center - Unified web monitor for the MiviaCar radar stack
======================================================================
Single-file browser dashboard (no X11 needed for monitoring) showing
EVERYTHING in real time:

  * Radar State  - full ARS408-21 RadarState (0x201) decoding per sensor:
                   NVM status, max distance, sensor id, sort index, Tx power,
                   output type, quality/extended-info flags, motion input,
                   RCS threshold, ERROR FLAGS (voltage, temperature,
                   temporary, persistent, interference), measured cycle
                   rate vs the expected ~13.9 Hz, ONLINE/OFFLINE status
  * Detections   - live bird's-eye plot (front forward / rear backward) +
                   full object table: class, distance, relative velocity,
                   RCS, probability of existence, measurement state, motion
  * CAN Frames   - every frame on the bus: ID, decoded name, rate, DLC,
                   data hex, count + estimated bus load
  * Topics       - all ROS2 topics: rate (Hz), bandwidth, count, type
  * Nodes        - every ROS2 node with its publishers / subscribers /
                   services (expandable tree)
  * TF Tree      - live frame tree from /tf + /tf_static with translations
  * RViz2 / rqt_graph launch buttons (requires a display on the machine)

Usage
-----
    source /opt/ros/humble/setup.bash
    python3 mira_control.py                      # auto-detects can1/can0/vcan0
    python3 mira_control.py --can can1           # MiviaCar: radar bus = can1
    python3 mira_control.py --can vcan0 --single 0
    # then open http://localhost:8090
    # over SSH:  ssh -L 8090:localhost:8090 miviaware@172.16.174.56

Requires: ROS2 Humble sourced (rclpy), Linux SocketCAN. Pure stdlib
otherwise - no external Python dependency.

Author: Abdelmoutalib Douadi - MIVIA Lab, UNISA (2026)
License: MIT
"""

import argparse
import json
import os
import socket
import struct
import subprocess
import threading
import time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSProfile, ReliabilityPolicy, HistoryPolicy,
                       DurabilityPolicy)

CAN_FRAME_FMT = "<IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
STALE_S = 1.0
EXPECTED_CYCLE_HZ = 13.9        # ARS408-21: ~72 ms cycle

BASE_NAMES = {
    0x200: "RadarCfg", 0x201: "RadarState", 0x202: "FilterCfg",
    0x203: "FilterState_Header", 0x204: "FilterState_Cfg",
    0x300: "SpeedInformation", 0x301: "YawRateInformation",
    0x600: "Cluster_0_Status", 0x701: "Cluster_1_General",
    0x702: "Cluster_2_Quality", 0x60A: "Object_0_Status",
    0x60B: "Object_1_General", 0x60C: "Object_2_Quality",
    0x60D: "Object_3_Extended", 0x60E: "Object_4_Warning",
    0x408: "CollDetState",
}
CLASS_NAMES = {0: "point", 1: "car", 2: "truck", 3: "pedestrian",
               4: "motorcycle", 5: "bicycle", 6: "wide", 7: "reserved"}
DYNPROP = {0: "moving", 1: "stationary", 2: "oncoming", 3: "stat.cand.",
           4: "unknown", 5: "cross stat.", 6: "cross mov.", 7: "stopped"}
OUTPUT_TYPE = {0: "none", 1: "objects", 2: "clusters", 3: "reserved"}
MOTION_RX = {0: "input ok", 1: "speed missing", 2: "yaw missing",
             3: "speed+yaw missing"}
POWER_CFG = {0: "standard", 1: "-3 dB Tx gain", 2: "-6 dB Tx gain",
             3: "-9 dB Tx gain"}
RCS_THRESH = {0: "standard", 1: "high sensitivity"}
PROB_EXIST = {0: "invalid", 1: "<25%", 2: "<50%", 3: "<75%",
              4: "<90%", 5: "<99%", 6: "<99.9%", 7: "100%"}
MEAS_STATE = {0: "deleted", 1: "new", 2: "measured", 3: "predicted",
              4: "deleted for merge", 5: "new from merge"}


# =========================================================================== #
#  ARS408 decoders (validated against datasheet + real MiviaCar candump)
# =========================================================================== #
class RadarState:
    __slots__ = ("nvm_read", "nvm_write", "max_distance", "persistent_err",
                 "interference", "temperature_err", "temporary_err",
                 "voltage_err", "sensor_id", "sort_index", "power_cfg",
                 "output_type", "send_quality", "send_ext_info",
                 "motion_rx", "rcs_threshold", "stamp")

    def __init__(self):
        for s in self.__slots__:
            setattr(self, s, 0)
        self.stamp = 0.0

    def decode(self, d: bytes):
        self.nvm_read = (d[0] >> 6) & 1
        self.nvm_write = (d[0] >> 7) & 1
        self.max_distance = ((d[1] << 2) | (d[2] >> 6)) * 2
        self.persistent_err = (d[2] >> 5) & 1
        self.interference = (d[2] >> 4) & 1
        self.temperature_err = (d[2] >> 3) & 1
        self.temporary_err = (d[2] >> 2) & 1
        self.voltage_err = (d[2] >> 1) & 1
        self.power_cfg = ((d[3] & 0x03) << 1) | (d[4] >> 7)
        self.sensor_id = d[4] & 0x07
        self.sort_index = (d[4] >> 4) & 0x07
        self.output_type = (d[5] >> 2) & 0x03
        self.send_quality = (d[5] >> 4) & 1
        self.send_ext_info = (d[5] >> 5) & 1
        self.motion_rx = (d[5] >> 6) & 0x03
        self.rcs_threshold = (d[7] >> 2) & 0x07
        self.stamp = time.monotonic()

    def errors(self):
        errs = []
        if self.persistent_err:
            errs.append("PERSISTENT")
        if self.interference:
            errs.append("INTERFERENCE")
        if self.temperature_err:
            errs.append("TEMPERATURE")
        if self.temporary_err:
            errs.append("TEMPORARY")
        if self.voltage_err:
            errs.append("VOLTAGE")
        return errs


class RadarObject:
    __slots__ = ("obj_id", "dist_long", "dist_lat", "vrel_long", "vrel_lat",
                 "rcs", "dyn_prop", "obj_class", "prob_exist", "meas_state",
                 "stamp")

    def __init__(self, obj_id):
        self.obj_id = obj_id
        self.dist_long = self.dist_lat = 0.0
        self.vrel_long = self.vrel_lat = 0.0
        self.rcs = 0.0
        self.dyn_prop = 4
        self.obj_class = None
        self.prob_exist = None
        self.meas_state = None
        self.stamp = time.monotonic()


class Ars408Decoder:
    """Full decoder for one ARS408 sensor (one SensorId)."""

    def __init__(self, sensor_id: int, label: str):
        self.sensor_id = sensor_id
        self.label = label
        self.offset = sensor_id * 0x10
        self.state = RadarState()
        self.objects = {}
        self.n_objects_reported = 0
        self.cycle_times = deque(maxlen=50)     # Object_0_Status arrivals
        self.lock = threading.Lock()

    def cycle_hz(self) -> float:
        if len(self.cycle_times) < 2:
            return 0.0
        span = self.cycle_times[-1] - self.cycle_times[0]
        return (len(self.cycle_times) - 1) / span if span > 0 else 0.0

    def alive(self) -> bool:
        now = time.monotonic()
        if (now - self.state.stamp) < 2.0:
            return True
        return bool(self.cycle_times) and (now - self.cycle_times[-1]) < 2.0

    def handle(self, base: int, d: bytes):
        now = time.monotonic()
        if base == 0x201:
            self.state.decode(d)
        elif base == 0x60A:
            self.n_objects_reported = d[0]
            self.cycle_times.append(now)
            with self.lock:
                stale = [k for k, o in self.objects.items()
                         if now - o.stamp > STALE_S]
                for k in stale:
                    del self.objects[k]
        elif base == 0x60B:
            oid = d[0]
            with self.lock:
                o = self.objects.setdefault(oid, RadarObject(oid))
                o.dist_long = (((d[1] << 5) | (d[2] >> 3)) * 0.2) - 500.0
                o.dist_lat = ((((d[2] & 0x07) << 8) | d[3]) * 0.2) - 204.6
                o.vrel_long = (((d[4] << 2) | (d[5] >> 6)) * 0.25) - 128.0
                o.vrel_lat = ((((d[5] & 0x3F) << 3) | (d[6] >> 5)) * 0.25) - 64.0
                o.dyn_prop = d[6] & 0x07
                o.rcs = d[7] * 0.5 - 64.0
                o.stamp = now
        elif base == 0x60C:
            oid = d[0]
            with self.lock:
                o = self.objects.setdefault(oid, RadarObject(oid))
                o.meas_state = (d[6] >> 2) & 0x07
                o.prob_exist = (d[6] >> 5) & 0x07
                o.stamp = now
        elif base == 0x60D:
            oid = d[0]
            with self.lock:
                o = self.objects.setdefault(oid, RadarObject(oid))
                o.obj_class = d[3] & 0x07
                o.stamp = now


# =========================================================================== #
#  CAN bus reader (Linux SocketCAN raw - no python-can)
# =========================================================================== #
class FrameStats:
    WINDOW = 5.0

    def __init__(self, can_id):
        self.can_id = can_id
        self.count = 0
        self.dlc = 0
        self.data = b""
        self.times = deque()

    def hit(self, dlc, data):
        now = time.monotonic()
        self.count += 1
        self.dlc = dlc
        self.data = data[:dlc]
        self.times.append(now)
        cutoff = now - self.WINDOW
        while self.times and self.times[0] < cutoff:
            self.times.popleft()

    def hz(self):
        if len(self.times) < 2:
            return 0.0
        span = self.times[-1] - self.times[0]
        return (len(self.times) - 1) / span if span > 0 else 0.0


class CanReader(threading.Thread):
    def __init__(self, interface, decoders, bitrate=500000):
        super().__init__(daemon=True)
        self.interface = interface
        self.decoders = decoders            # {sensor_id: Ars408Decoder}
        self.bitrate = bitrate
        self.frames = {}
        self.error = ""
        self.total_frames = 0

    def frame_name(self, can_id):
        for dec in self.decoders.values():
            base = can_id - dec.offset
            if base in BASE_NAMES:
                suffix = f" [{dec.label}]" if len(self.decoders) > 1 else ""
                return BASE_NAMES[base] + suffix
        return BASE_NAMES.get(can_id, "-")

    def bus_load_pct(self):
        """Approximate bus load: (47 + 8*DLC) bits per standard frame."""
        bits_per_s = 0.0
        for st in self.frames.values():
            bits_per_s += st.hz() * (47 + 8 * st.dlc)
        return 100.0 * bits_per_s / self.bitrate if self.bitrate else 0.0

    def run(self):
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW,
                                 socket.CAN_RAW)
            sock.bind((self.interface,))
        except OSError as exc:
            self.error = f"cannot open {self.interface}: {exc}"
            return
        while True:
            try:
                frame = sock.recv(CAN_FRAME_SIZE)
            except OSError as exc:
                self.error = f"read error on {self.interface}: {exc}"
                return
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
            can_id &= socket.CAN_EFF_MASK
            self.total_frames += 1
            st = self.frames.setdefault(can_id, FrameStats(can_id))
            st.hit(dlc, data)
            for dec in self.decoders.values():
                base = can_id - dec.offset
                if base in BASE_NAMES:
                    try:
                        dec.handle(base, data)
                    except Exception:
                        pass
                    break


def autodetect_can():
    """MiviaCar convention: radar on can1 (can0 = Autoware/PIX Hooke)."""
    for iface in ("can1", "can0", "vcan0"):
        if os.path.isdir(f"/sys/class/net/{iface}"):
            return iface
    return ""


# =========================================================================== #
#  ROS2 monitor node: topic stats + node graph + TF tree
# =========================================================================== #
class TopicStats:
    WINDOW = 5.0

    def __init__(self, name, type_name):
        self.name = name
        self.type_name = type_name
        self.count = 0
        self.samples = deque()
        self.lock = threading.Lock()

    def on_message(self, raw: bytes):
        now = time.monotonic()
        with self.lock:
            self.count += 1
            self.samples.append((now, len(raw)))
            cutoff = now - self.WINDOW
            while self.samples and self.samples[0][0] < cutoff:
                self.samples.popleft()

    def rate_hz(self):
        with self.lock:
            n = len(self.samples)
            if n < 2:
                return 0.0
            span = self.samples[-1][0] - self.samples[0][0]
            return (n - 1) / span if span > 0 else 0.0

    def bandwidth_bps(self):
        with self.lock:
            if not self.samples:
                return 0.0
            span = self.samples[-1][0] - self.samples[0][0]
            total = sum(b for _, b in self.samples)
            return total / span if span > 0.1 else float(total)


def human_bytes(bps):
    for unit in ("B/s", "KB/s", "MB/s", "GB/s"):
        if bps < 1024.0:
            return f"{bps:.1f} {unit}"
        bps /= 1024.0
    return f"{bps:.1f} TB/s"


class ControlNode(Node):
    """Topic stats (raw subs) + node graph poll + TF tree subscription."""

    def __init__(self, topic_filter=""):
        super().__init__("mira_control")
        self.topic_filter = topic_filter
        self.stats = {}
        self._subs = {}
        self.graph = {}                 # full_name -> {pubs, subs, srvs}
        self.graph_lock = threading.Lock()
        self.tf_edges = {}              # child -> edge dict
        self.tf_lock = threading.Lock()
        self.create_timer(2.0, self.discover_topics)
        self.create_timer(3.0, self.poll_graph)
        self._setup_tf()

    # ---- topic statistics -------------------------------------------------
    def discover_topics(self):
        for name, types in self.get_topic_names_and_types():
            if name in self._subs or not types:
                continue
            if self.topic_filter and self.topic_filter not in name:
                continue
            if name.startswith(("/parameter_events", "/rosout")):
                continue
            try:
                from rosidl_runtime_py.utilities import get_message
                msg_cls = get_message(types[0])
            except Exception:
                continue
            st = TopicStats(name, types[0])
            self.stats[name] = st
            qos = QoSProfile(depth=5,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST)
            try:
                self._subs[name] = self.create_subscription(
                    msg_cls, name,
                    lambda raw, s=st: s.on_message(raw),
                    qos, raw=True)
            except Exception:
                self.stats.pop(name, None)

    # ---- node graph -------------------------------------------------------
    def poll_graph(self):
        graph = {}
        try:
            names = self.get_node_names_and_namespaces()
        except Exception:
            return
        for name, ns in names:
            full = (ns.rstrip("/") + "/" + name) if ns != "/" else "/" + name
            entry = {"pubs": [], "subs": [], "srvs": []}
            try:
                entry["pubs"] = [
                    {"topic": t, "type": ty[0].split("/")[-1] if ty else "?"}
                    for t, ty in
                    self.get_publisher_names_and_types_by_node(name, ns)]
                entry["subs"] = [
                    {"topic": t, "type": ty[0].split("/")[-1] if ty else "?"}
                    for t, ty in
                    self.get_subscriber_names_and_types_by_node(name, ns)]
                entry["srvs"] = [
                    t for t, _ in
                    self.get_service_names_and_types_by_node(name, ns)
                    if not t.endswith(("/describe_parameters",
                                       "/get_parameter_types",
                                       "/get_parameters", "/list_parameters",
                                       "/set_parameters",
                                       "/set_parameters_atomically",
                                       "/get_type_description"))]
            except Exception:
                pass
            graph[full] = entry
        with self.graph_lock:
            self.graph = graph

    # ---- TF tree ----------------------------------------------------------
    def _setup_tf(self):
        try:
            from tf2_msgs.msg import TFMessage
        except Exception:
            return
        self.create_subscription(
            TFMessage, "/tf",
            lambda m: self._on_tf(m, False),
            QoSProfile(depth=100,
                       reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST))
        self.create_subscription(
            TFMessage, "/tf_static",
            lambda m: self._on_tf(m, True),
            QoSProfile(depth=100,
                       reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       history=HistoryPolicy.KEEP_LAST))

    def _on_tf(self, msg, is_static):
        now = time.monotonic()
        with self.tf_lock:
            for t in msg.transforms:
                tr = t.transform.translation
                self.tf_edges[t.child_frame_id] = {
                    "parent": t.header.frame_id,
                    "x": round(tr.x, 3), "y": round(tr.y, 3),
                    "z": round(tr.z, 3),
                    "static": is_static, "stamp": now,
                }

    def tf_tree(self):
        """Build nested tree {frame, x, y, z, static, fresh, children[]}."""
        with self.tf_lock:
            edges = dict(self.tf_edges)
        children_of = {}
        for child, e in edges.items():
            children_of.setdefault(e["parent"], []).append(child)
        parents = set(e["parent"] for e in edges.values())
        childs = set(edges.keys())
        roots = sorted(parents - childs)
        now = time.monotonic()

        def build(frame):
            node = {"frame": frame, "children": []}
            e = edges.get(frame)
            if e:
                node.update({"x": e["x"], "y": e["y"], "z": e["z"],
                             "static": e["static"],
                             "fresh": e["static"] or (now - e["stamp"] < 3.0)})
            for c in sorted(children_of.get(frame, [])):
                node["children"].append(build(c))
            return node

        return [build(r) for r in roots]


# =========================================================================== #
#  JSON snapshots served to the browser
# =========================================================================== #
STATE = {"node": None, "can": None, "start": time.time(), "iface": ""}


def snapshot_radar():
    can = STATE["can"]
    if can is None:
        return {"error": "CAN disabled", "sensors": []}
    if can.error:
        return {"error": can.error, "sensors": []}
    sensors = []
    for dec in can.decoders.values():
        s = dec.state
        cyc = dec.cycle_hz()
        sensors.append({
            "label": dec.label, "sensor_id_cfg": dec.sensor_id,
            "alive": dec.alive(),
            "cycle_hz": round(cyc, 2),
            "cycle_ok": abs(cyc - EXPECTED_CYCLE_HZ) < 2.5 if cyc else False,
            "n_objects": dec.n_objects_reported,
            "state_seen": s.stamp > 0,
            "sensor_id": s.sensor_id, "sort_index": s.sort_index,
            "max_distance": s.max_distance,
            "output_type": OUTPUT_TYPE.get(s.output_type, "?"),
            "power_cfg": POWER_CFG.get(s.power_cfg, "?"),
            "send_quality": bool(s.send_quality),
            "send_ext_info": bool(s.send_ext_info),
            "motion_rx": MOTION_RX.get(s.motion_rx, "?"),
            "rcs_threshold": RCS_THRESH.get(s.rcs_threshold, "?"),
            "nvm_read": bool(s.nvm_read), "nvm_write": bool(s.nvm_write),
            "errors": s.errors(),
        })
    return {"error": "", "expected_hz": EXPECTED_CYCLE_HZ, "sensors": sensors}


def snapshot_objects():
    can = STATE["can"]
    if can is None:
        return {"error": "CAN disabled", "rows": []}
    rows = []
    for dec in can.decoders.values():
        with dec.lock:
            objs = list(dec.objects.values())
        sign = -1.0 if "rear" in dec.label.lower() else 1.0
        for o in sorted(objs, key=lambda o: abs(o.dist_long)):
            rows.append({
                "sensor": dec.label, "id": o.obj_id,
                "class": CLASS_NAMES.get(o.obj_class, "?")
                if o.obj_class is not None else "?",
                "x": round(sign * o.dist_long, 1),
                "y": round(o.dist_lat, 1),
                "vx": round(o.vrel_long, 2), "vy": round(o.vrel_lat, 2),
                "rcs": round(o.rcs, 1),
                "prob": PROB_EXIST.get(o.prob_exist, "-")
                if o.prob_exist is not None else "-",
                "meas": MEAS_STATE.get(o.meas_state, "-")
                if o.meas_state is not None else "-",
                "motion": DYNPROP.get(o.dyn_prop, "?"),
            })
    return {"error": "", "rows": rows}


def snapshot_can():
    can = STATE["can"]
    if can is None:
        return {"error": "CAN disabled - restart with --can can1", "rows": []}
    if can.error:
        return {"error": can.error, "rows": []}
    rows = []
    for can_id in sorted(can.frames):
        st = can.frames[can_id]
        rows.append({
            "id": f"0x{can_id:03X}",
            "name": can.frame_name(can_id),
            "hz": round(st.hz(), 1), "dlc": st.dlc,
            "data": " ".join(f"{b:02X}" for b in st.data),
            "count": st.count,
        })
    return {"error": "", "rows": rows,
            "iface": can.interface,
            "load": round(can.bus_load_pct(), 1),
            "total": can.total_frames}


def snapshot_topics():
    node = STATE["node"]
    rows = []
    for name in sorted(node.stats):
        st = node.stats[name]
        rows.append({
            "topic": name,
            "type": st.type_name.split("/")[-1],
            "full_type": st.type_name,
            "hz": round(st.rate_hz(), 1),
            "bw": human_bytes(st.bandwidth_bps()),
            "count": st.count,
        })
    return {"rows": rows}


def snapshot_nodes():
    node = STATE["node"]
    with node.graph_lock:
        graph = dict(node.graph)
    rows = []
    for full in sorted(graph):
        e = graph[full]
        rows.append({"node": full, "pubs": e["pubs"], "subs": e["subs"],
                     "srvs": sorted(e["srvs"])})
    return {"rows": rows}


def snapshot_tf():
    return {"roots": STATE["node"].tf_tree()}


def snapshot_status():
    node = STATE["node"]
    can = STATE["can"]
    radars = []
    if can:
        for dec in can.decoders.values():
            radars.append({"label": dec.label, "alive": dec.alive()})
    with node.graph_lock:
        n_nodes = len(node.graph)
    return {
        "uptime": int(time.time() - STATE["start"]),
        "n_topics": len(node.stats),
        "n_nodes": n_nodes,
        "can_iface": STATE["iface"] or "-",
        "can_error": can.error if can else "no CAN",
        "bus_load": round(can.bus_load_pct(), 1) if can else 0.0,
        "display": bool(os.environ.get("DISPLAY")),
        "radars": radars,
    }


# =========================================================================== #
#  Single-page web UI (pure HTML/CSS/JS, no CDN)
# =========================================================================== #
PAGE = r"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MIRA Control Center</title>
<style>
  :root { --bg:#0d1117; --panel:#161b22; --border:#30363d; --fg:#e6edf3;
          --dim:#8b949e; --blue:#58a6ff; --green:#3fb950; --red:#f85149;
          --yellow:#d29922; --purple:#bc8cff; --cyan:#39c5cf; }
  * { box-sizing:border-box; }
  body { margin:0; background:var(--bg); color:var(--fg);
         font:14px/1.45 -apple-system,"Segoe UI",Roboto,Helvetica,Arial,sans-serif; }
  header { display:flex; align-items:center; gap:14px; padding:10px 18px;
           background:var(--panel); border-bottom:1px solid var(--border);
           flex-wrap:wrap; position:sticky; top:0; z-index:5; }
  h1 { font-size:17px; margin:0; color:var(--blue); }
  .sub { color:var(--dim); font-size:12px; }
  nav { display:flex; gap:6px; flex-wrap:wrap; margin-left:auto; }
  nav button { background:transparent; border:1px solid var(--border);
    color:var(--fg); padding:6px 12px; border-radius:6px; cursor:pointer;
    font-size:13px; }
  nav button.active { background:#1f6feb33; border-color:var(--blue);
    color:var(--blue); font-weight:600; }
  nav button.launch { border-color:var(--green); color:var(--green); }
  #statusbar { display:flex; gap:16px; padding:6px 18px; font-size:12px;
    color:var(--dim); background:#010409; border-bottom:1px solid var(--border);
    flex-wrap:wrap; }
  .led { display:inline-block; width:9px; height:9px; border-radius:50%;
    margin-right:5px; background:var(--dim); vertical-align:baseline; }
  .led.on { background:var(--green); box-shadow:0 0 5px var(--green); }
  .led.off { background:var(--red); }
  main { padding:16px 18px; }
  .panel { background:var(--panel); border:1px solid var(--border);
    border-radius:8px; padding:12px; margin-bottom:14px; overflow:auto; }
  table { border-collapse:collapse; width:100%; font-size:13px; }
  th { text-align:left; color:var(--dim); font-weight:600; padding:5px 10px;
    border-bottom:1px solid var(--border); white-space:nowrap; }
  td { padding:4px 10px; border-bottom:1px solid #21262d; white-space:nowrap; }
  .num { text-align:right; font-variant-numeric:tabular-nums; }
  .mono { font-family:ui-monospace,SFMono-Regular,Menlo,monospace; font-size:12px; }
  .dim { color:var(--dim); } .ok { color:var(--green); }
  .fail { color:var(--red); font-weight:600; } .warn { color:var(--yellow); }
  .err { color:var(--red); padding:8px; }
  .grid2 { display:grid; grid-template-columns:1fr 1fr; gap:14px; }
  @media(max-width:1000px){ .grid2 { grid-template-columns:1fr; } }
  .card h2 { margin:0 0 8px; font-size:15px; }
  .kv { display:grid; grid-template-columns:auto 1fr; gap:2px 14px;
    font-size:13px; }
  .kv .k { color:var(--dim); } .kv .v { font-weight:500; }
  .badge { display:inline-block; padding:1px 8px; border-radius:10px;
    font-size:11px; font-weight:600; margin:1px 3px 1px 0; }
  .b-err { background:#f8514922; color:var(--red); border:1px solid var(--red); }
  .b-ok  { background:#3fb95022; color:var(--green); border:1px solid var(--green); }
  .b-static { background:#58a6ff22; color:var(--blue); }
  .b-dyn { background:#d2992222; color:var(--yellow); }
  canvas { background:#010409; border:1px solid var(--border);
    border-radius:8px; width:100%; }
  details { margin:4px 0; }
  summary { cursor:pointer; font-weight:600; }
  details .inner { padding:4px 0 4px 20px; font-size:13px; }
  .tf-node { margin-left:22px; border-left:1px solid var(--border);
    padding:2px 0 2px 12px; }
  .tf-root { margin-left:0; border-left:none; padding-left:0; }
  .tf-frame { font-family:ui-monospace,Menlo,monospace; font-weight:600;
    color:var(--cyan); }
  .legend { display:flex; gap:14px; flex-wrap:wrap; margin-top:8px;
    font-size:12px; color:var(--dim); }
  .swatch { display:inline-block; width:10px; height:10px; border-radius:50%;
    margin-right:5px; }
  input[type=text] { background:#010409; border:1px solid var(--border);
    color:var(--fg); border-radius:6px; padding:5px 10px; width:260px; }
  .toast { position:fixed; bottom:18px; right:18px; background:#1f6feb;
    color:#fff; padding:9px 15px; border-radius:8px; display:none; z-index:10; }
</style>
</head>
<body>
<header>
  <h1>MIRA Control Center</h1>
  <span class="sub">MiviaCar - ARS408 radar / ROS2 / CAN unified monitor</span>
  <nav>
    <button id="tab-radar" class="active" onclick="setView('radar')">Radar State</button>
    <button id="tab-obj" onclick="setView('obj')">Detections</button>
    <button id="tab-can" onclick="setView('can')">CAN</button>
    <button id="tab-topics" onclick="setView('topics')">Topics</button>
    <button id="tab-nodes" onclick="setView('nodes')">Nodes</button>
    <button id="tab-tf" onclick="setView('tf')">TF Tree</button>
    <button class="launch" onclick="launchTool('rviz')">RViz2</button>
    <button class="launch" onclick="launchTool('rqt')">rqt_graph</button>
  </nav>
</header>
<div id="statusbar">loading...</div>
<main>

<div id="view-radar">
  <div id="radar-err" class="err" style="display:none"></div>
  <div class="grid2" id="radar-cards"></div>
</div>

<div id="view-obj" style="display:none">
  <div class="grid2">
    <div class="panel">
      <div id="obj-err" class="err" style="display:none"></div>
      <table><thead><tr>
        <th>Sensor</th><th>ID</th><th>Class</th>
        <th class="num">X (m)</th><th class="num">Y (m)</th>
        <th class="num">Vx (m/s)</th><th class="num">RCS</th>
        <th>Prob</th><th>Meas</th><th>Motion</th>
      </tr></thead><tbody id="obj-body"></tbody></table>
    </div>
    <div class="panel">
      <canvas id="bev" width="640" height="640"></canvas>
      <div class="legend" id="legend"></div>
    </div>
  </div>
</div>

<div id="view-can" style="display:none" class="panel">
  <div id="can-err" class="err" style="display:none"></div>
  <div class="dim" id="can-meta" style="margin-bottom:6px"></div>
  <table><thead><tr>
    <th>CAN ID</th><th>Name</th><th class="num">Hz</th>
    <th class="num">DLC</th><th>Data (hex)</th><th class="num">Count</th>
  </tr></thead><tbody id="can-body"></tbody></table>
</div>

<div id="view-topics" style="display:none" class="panel">
  <input type="text" id="topic-filter" placeholder="filter topics..."
         oninput="renderTopics()">
  <table style="margin-top:8px"><thead><tr>
    <th>Topic</th><th>Type</th><th class="num">Hz</th>
    <th class="num">Bandwidth</th><th class="num">Msgs</th>
  </tr></thead><tbody id="topics-body"></tbody></table>
</div>

<div id="view-nodes" style="display:none" class="panel">
  <input type="text" id="node-filter" placeholder="filter nodes..."
         oninput="renderNodes()">
  <div id="nodes-body" style="margin-top:8px"></div>
</div>

<div id="view-tf" style="display:none" class="panel">
  <div class="dim" style="margin-bottom:8px">
    Live TF tree (from /tf + /tf_static). Translation in metres,
    child relative to parent.</div>
  <div id="tf-body"></div>
</div>

</main>
<div class="toast" id="toast"></div>

<script>
const CLASS_COLORS = { car:'#3fb950', truck:'#d29922', pedestrian:'#f85149',
  motorcycle:'#bc8cff', bicycle:'#39c5cf', wide:'#58a6ff',
  point:'#e6edf3', reserved:'#8b949e', '?':'#8b949e' };
let view = 'radar';
let topicsData = [], nodesData = [];
const esc = s => String(s).replace(/[&<>"]/g,
  c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]));

function setView(v){
  view = v;
  for (const name of ['radar','obj','can','topics','nodes','tf']){
    document.getElementById('view-'+name).style.display =
      name===v ? '' : 'none';
    document.getElementById('tab-'+name)
      .classList.toggle('active', name===v);
  }
  tick();
}
function toast(msg){
  const t = document.getElementById('toast');
  t.textContent = msg; t.style.display='block';
  setTimeout(()=>t.style.display='none', 2500);
}
async function launchTool(which){
  const r = await fetch('/api/'+which, {method:'POST'});
  const j = await r.json(); toast(j.message);
}
async function getJSON(u){ const r = await fetch(u); return r.json(); }

/* ---------------- status bar ---------------- */
async function refreshStatus(){
  const s = await getJSON('/api/status');
  const radars = s.radars.map(r =>
    `<span><span class="led ${r.alive?'on':'off'}"></span>${esc(r.label)}
     ${r.alive?'ONLINE':'OFFLINE'}</span>`).join('');
  document.getElementById('statusbar').innerHTML =
    `<span><span class="led on"></span>ROS2: ${s.n_nodes} nodes /
       ${s.n_topics} topics</span>
     <span><span class="led ${s.can_error?'off':'on'}"></span>CAN
       ${esc(s.can_iface)} - load ${s.bus_load}%</span>
     ${radars}
     <span><span class="led ${s.display?'on':'off'}"></span>DISPLAY
       ${s.display?'available':'absent (RViz2 unavailable)'}</span>
     <span>uptime ${Math.floor(s.uptime/60)}m${s.uptime%60}s</span>`;
}

/* ---------------- radar state ---------------- */
async function refreshRadar(){
  const d = await getJSON('/api/radar');
  const err = document.getElementById('radar-err');
  err.style.display = d.error ? '' : 'none';
  err.textContent = d.error;
  document.getElementById('radar-cards').innerHTML = d.sensors.map(s => {
    const errBadges = s.errors.length
      ? s.errors.map(e=>`<span class="badge b-err">${e}</span>`).join('')
      : '<span class="badge b-ok">NO ERROR</span>';
    const cyc = s.cycle_hz
      ? `<span class="${s.cycle_ok?'ok':'warn'}">${s.cycle_hz} Hz</span>
         <span class="dim">(expected ~${d.expected_hz})</span>`
      : '<span class="dim">-</span>';
    return `<div class="panel card">
      <h2><span class="led ${s.alive?'on':'off'}"></span>
        ${esc(s.label)} radar - ${s.alive?'ONLINE':'OFFLINE'}</h2>
      <div class="kv">
        <span class="k">Cycle rate</span><span class="v">${cyc}</span>
        <span class="k">Objects reported</span><span class="v">${s.n_objects}</span>
        <span class="k">SensorID (state)</span><span class="v">${s.state_seen?s.sensor_id:'-'}</span>
        <span class="k">Max distance cfg</span><span class="v">${s.state_seen?s.max_distance+' m':'-'}</span>
        <span class="k">Output type</span><span class="v">${s.state_seen?esc(s.output_type):'-'}</span>
        <span class="k">Tx power</span><span class="v">${s.state_seen?esc(s.power_cfg):'-'}</span>
        <span class="k">Send quality (0x60C)</span><span class="v">${s.state_seen?(s.send_quality?'yes':'no'):'-'}</span>
        <span class="k">Send ext info (0x60D)</span><span class="v">${s.state_seen?(s.send_ext_info?'yes':'no'):'-'}</span>
        <span class="k">Motion input</span><span class="v">${s.state_seen?esc(s.motion_rx):'-'}</span>
        <span class="k">RCS threshold</span><span class="v">${s.state_seen?esc(s.rcs_threshold):'-'}</span>
        <span class="k">Sort index</span><span class="v">${s.state_seen?s.sort_index:'-'}</span>
        <span class="k">NVM read / write</span>
        <span class="v">${s.state_seen?((s.nvm_read?'ok':'fail')+' / '+(s.nvm_write?'ok':'fail')):'-'}</span>
        <span class="k">Error flags</span><span class="v">${s.state_seen?errBadges:'-'}</span>
      </div></div>`;
  }).join('');
}

/* ---------------- detections ---------------- */
function drawBEV(rows){
  const c = document.getElementById('bev'), ctx = c.getContext('2d');
  const W=c.width, H=c.height, cx=W/2, cy=H/2, RANGE=60, sc=(H/2-24)/RANGE;
  ctx.clearRect(0,0,W,H);
  ctx.strokeStyle='#21262d'; ctx.fillStyle='#8b949e';
  ctx.font='10px monospace'; ctx.textAlign='left';
  for (const r of [10,20,30,40,50,60]){
    ctx.beginPath(); ctx.arc(cx,cy,r*sc,0,2*Math.PI); ctx.stroke();
    ctx.fillText(r+'m', cx+4, cy-r*sc+11);
  }
  ctx.beginPath(); ctx.moveTo(cx,0); ctx.lineTo(cx,H);
  ctx.moveTo(0,cy); ctx.lineTo(W,cy); ctx.stroke();
  ctx.fillStyle='#58a6ff';
  ctx.fillRect(cx-6, cy-11, 12, 22);           // ego vehicle
  ctx.fillStyle='#8b949e';
  ctx.fillText('FRONT', cx+8, 14); ctx.fillText('REAR', cx+8, H-6);
  for (const o of rows){
    const px = cx + o.y*sc, py = cy - o.x*sc;   // x fwd -> up, y left -> right
    if (px<0||px>W||py<0||py>H) continue;
    const col = CLASS_COLORS[o.class] || '#8b949e';
    ctx.beginPath(); ctx.arc(px,py,5,0,2*Math.PI);
    ctx.fillStyle=col; ctx.fill();
    ctx.strokeStyle=col; ctx.beginPath(); ctx.moveTo(px,py);
    ctx.lineTo(px + o.vy*sc*0.8, py - o.vx*sc*0.8); ctx.stroke();
    ctx.fillStyle='#e6edf3';
    ctx.fillText(o.id, px+7, py+3);
  }
}
async function refreshObjects(){
  const d = await getJSON('/api/objects');
  const err = document.getElementById('obj-err');
  err.style.display = d.error ? '' : 'none'; err.textContent = d.error;
  document.getElementById('obj-body').innerHTML = d.rows.map(r =>
    `<tr><td class="dim">${esc(r.sensor)}</td><td>${r.id}</td>
     <td style="color:${CLASS_COLORS[r.class]||'#8b949e'};font-weight:600">
       ${esc(r.class)}</td>
     <td class="num">${r.x.toFixed(1)}</td><td class="num">${r.y.toFixed(1)}</td>
     <td class="num">${r.vx.toFixed(2)}</td><td class="num">${r.rcs.toFixed(1)}</td>
     <td class="dim">${esc(r.prob)}</td><td class="dim">${esc(r.meas)}</td>
     <td class="dim">${esc(r.motion)}</td></tr>`).join('');
  drawBEV(d.rows);
  document.getElementById('legend').innerHTML =
    Object.entries(CLASS_COLORS).filter(([k])=>k!=='?'&&k!=='reserved')
    .map(([k,v])=>`<span><span class="swatch"
      style="background:${v}"></span>${k}</span>`).join('');
}

/* ---------------- CAN ---------------- */
async function refreshCan(){
  const d = await getJSON('/api/can');
  const err = document.getElementById('can-err');
  err.style.display = d.error ? '' : 'none'; err.textContent = d.error;
  document.getElementById('can-meta').textContent = d.error ? '' :
    `interface ${d.iface} - bus load ~${d.load}% - ${d.total} frames total`;
  document.getElementById('can-body').innerHTML = (d.rows||[]).map(r =>
    `<tr><td class="mono">${r.id}</td><td>${esc(r.name)}</td>
     <td class="num ${r.hz>0?'ok':'dim'}">${r.hz.toFixed(1)}</td>
     <td class="num">${r.dlc}</td><td class="mono dim">${r.data}</td>
     <td class="num">${r.count}</td></tr>`).join('');
}

/* ---------------- topics ---------------- */
function renderTopics(){
  const f = document.getElementById('topic-filter').value.toLowerCase();
  document.getElementById('topics-body').innerHTML = topicsData
    .filter(r => !f || r.topic.toLowerCase().includes(f))
    .map(r => `<tr><td class="mono">${esc(r.topic)}</td>
      <td class="dim" title="${esc(r.full_type)}">${esc(r.type)}</td>
      <td class="num ${r.hz>0?'ok':'dim'}">${r.hz.toFixed(1)}</td>
      <td class="num">${esc(r.bw)}</td><td class="num">${r.count}</td></tr>`)
    .join('');
}
async function refreshTopics(){
  topicsData = (await getJSON('/api/topics')).rows; renderTopics();
}

/* ---------------- nodes ---------------- */
function renderNodes(){
  const f = document.getElementById('node-filter').value.toLowerCase();
  document.getElementById('nodes-body').innerHTML = nodesData
    .filter(r => !f || r.node.toLowerCase().includes(f))
    .map(r => {
      const list = (arr, cls) => arr.map(p =>
        `<div><span class="mono" style="color:${cls}">${esc(p.topic)}</span>
         <span class="dim">(${esc(p.type)})</span></div>`).join('')
        || '<div class="dim">none</div>';
      const srvs = r.srvs.map(s =>
        `<div class="mono dim">${esc(s)}</div>`).join('')
        || '<div class="dim">none</div>';
      return `<details><summary><span class="mono"
          style="color:var(--cyan)">${esc(r.node)}</span>
          <span class="dim"> - ${r.pubs.length} pub /
          ${r.subs.length} sub / ${r.srvs.length} srv</span></summary>
        <div class="inner">
          <div class="dim" style="font-weight:600">publishes -></div>
          ${list(r.pubs, 'var(--green)')}
          <div class="dim" style="font-weight:600;margin-top:5px">
            subscribes &lt;-</div>
          ${list(r.subs, 'var(--yellow)')}
          <div class="dim" style="font-weight:600;margin-top:5px">services</div>
          ${srvs}
        </div></details>`;
    }).join('');
}
async function refreshNodes(){
  nodesData = (await getJSON('/api/nodes')).rows; renderNodes();
}

/* ---------------- TF tree ---------------- */
function tfHtml(n, root){
  const info = n.x!==undefined
    ? `<span class="dim"> xyz=(${n.x}, ${n.y}, ${n.z})</span>
       <span class="badge ${n.static?'b-static':'b-dyn'}">
         ${n.static?'static':'dynamic'}</span>
       ${n.fresh?'':'<span class="badge b-err">STALE</span>'}`
    : '<span class="badge b-static">root</span>';
  return `<div class="tf-node ${root?'tf-root':''}">
    <span class="tf-frame">${esc(n.frame)}</span>${info}
    ${n.children.map(c=>tfHtml(c,false)).join('')}</div>`;
}
async function refreshTf(){
  const d = await getJSON('/api/tf');
  document.getElementById('tf-body').innerHTML = d.roots.length
    ? d.roots.map(r=>tfHtml(r,true)).join('')
    : '<div class="dim">No TF frames received yet (is a robot_state_publisher'
      + ' or static_transform_publisher running?)</div>';
}

/* ---------------- main loop ---------------- */
async function tick(){
  try {
    await refreshStatus();
    if (view==='radar') await refreshRadar();
    else if (view==='obj') await refreshObjects();
    else if (view==='can') await refreshCan();
    else if (view==='topics') await refreshTopics();
    else if (view==='nodes') await refreshNodes();
    else if (view==='tf') await refreshTf();
  } catch(e) { /* server restarting */ }
}
tick(); setInterval(tick, 500);
</script>
</body>
</html>
"""


# =========================================================================== #
#  HTTP server
# =========================================================================== #
class Handler(BaseHTTPRequestHandler):
    def _send(self, code, body, ctype):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj):
        self._send(200, json.dumps(obj).encode(), "application/json")

    def do_GET(self):
        routes = {
            "/api/radar": snapshot_radar,
            "/api/objects": snapshot_objects,
            "/api/can": snapshot_can,
            "/api/topics": snapshot_topics,
            "/api/nodes": snapshot_nodes,
            "/api/tf": snapshot_tf,
            "/api/status": snapshot_status,
        }
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, PAGE.encode(), "text/html; charset=utf-8")
        elif self.path in routes:
            self._json(routes[self.path]())
        else:
            self._send(404, b"not found", "text/plain")

    def _launch(self, cmd, label):
        if not os.environ.get("DISPLAY"):
            self._json({"message": f"{label}: no DISPLAY on this machine "
                                   "(use ssh -X or run locally)"})
            return
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL,
                             start_new_session=True)
            self._json({"message": f"{label} launched"})
        except FileNotFoundError:
            self._json({"message": f"{label} not found on this machine"})

    def do_POST(self):
        if self.path == "/api/rviz":
            self._launch(["rviz2"], "RViz2")
        elif self.path == "/api/rqt":
            self._launch(["rqt_graph"], "rqt_graph")
        else:
            self._send(404, b"not found", "text/plain")

    def log_message(self, *args):
        pass


# =========================================================================== #
#  Main
# =========================================================================== #
def main():
    parser = argparse.ArgumentParser(
        description="MIRA Control Center - unified web monitor "
                    "(radar state / detections / CAN / topics / nodes / TF)")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--can", default="", metavar="IFACE",
                        help="CAN interface (default: auto-detect "
                             "can1 > can0 > vcan0)")
    parser.add_argument("--bitrate", type=int, default=500000,
                        help="CAN bitrate for bus-load estimate")
    parser.add_argument("--front", type=int, default=0,
                        help="SensorId of the front radar")
    parser.add_argument("--rear", type=int, default=1,
                        help="SensorId of the rear radar")
    parser.add_argument("--single", type=int, default=None, metavar="ID",
                        help="monitor only one radar with this SensorId")
    parser.add_argument("--filter", default="",
                        help="only ROS topics containing this string")
    args = parser.parse_args()

    iface = args.can or autodetect_can()
    STATE["iface"] = iface
    if iface:
        if args.single is not None:
            decoders = {args.single: Ars408Decoder(args.single, "radar")}
        else:
            decoders = {
                args.front: Ars408Decoder(args.front, "front"),
                args.rear: Ars408Decoder(args.rear, "rear"),
            }
        STATE["can"] = CanReader(iface, decoders, bitrate=args.bitrate)
        STATE["can"].start()
        print(f"[MIRA] CAN monitoring on {iface}")
    else:
        print("[MIRA] no CAN interface found - CAN views disabled")

    rclpy.init()
    STATE["node"] = ControlNode(topic_filter=args.filter)
    threading.Thread(target=rclpy.spin, args=(STATE["node"],),
                     daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"[MIRA] Control Center ->  http://localhost:{args.port}")
    print(f"[MIRA] over SSH:  ssh -L {args.port}:localhost:{args.port} "
          "miviaware@172.16.174.56")
    print("[MIRA] Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
