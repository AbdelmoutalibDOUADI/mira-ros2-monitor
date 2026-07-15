#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MIRA — Monitoring Interface for ROS2 Applications (v2)
======================================================
A lightweight, single-file terminal dashboard for ROS2 + CAN bus.
No GUI, no X11 needed for monitoring — runs in any terminal, even over SSH.

Views (keyboard navigation)
---------------------------
  [1] ROS2 Topics   — rate (Hz), bandwidth, message count, type, health
  [2] CAN Frames    — live frames on can0/vcan0: ID, rate, DLC, data, name
  [3] Radar Objects — ARS408 detected objects: class, distance, speed, RCS
  [r] Launch RViz2  — spawns rviz2 (requires a display / X11)
  [q] Quit

Usage
-----
    source /opt/ros/humble/setup.bash
    python3 mira.py                              # topics view
    python3 mira.py --can can0                   # enable CAN + objects views
    python3 mira.py --can vcan0 --sensor-id 1    # dual-radar setups
    python3 mira.py --dbc ARS408.dbc             # decode CAN with your DBC
    python3 mira.py --filter radar --rules mira_rules.yaml

ARS408 notes
------------
Object CAN IDs follow:  MsgId = BASE + SensorId * 0x10
    Object_0_Status   0x60A   (object count)
    Object_1_General  0x60B   (distance, velocity, RCS)
    Object_3_Extended 0x60D   (object class)
Built-in bit-level decoding is provided; pass --dbc for full DBC decoding
(requires `pip install cantools`).

Author: Abdelmoutalib Douadi — MIVIA Lab, UNISA (2026)
License: MIT
"""

import argparse
import os
import select
import socket
import struct
import subprocess
import sys
import termios
import threading
import time
import tty
from collections import deque

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except ImportError:
    sys.exit("[MIRA] rclpy not found. Did you source /opt/ros/humble/setup.bash ?")

try:
    from rich.console import Console
    from rich.table import Table
    from rich.live import Live
    from rich.panel import Panel
    from rich.layout import Layout
    from rich.text import Text
except ImportError:
    sys.exit("[MIRA] rich not found. Install it with: pip install rich")

try:
    import yaml
except ImportError:
    yaml = None


# =========================================================================== #
#  ROS2 topic statistics (view 1)
# =========================================================================== #
class TopicStats:
    """Rolling statistics for one ROS2 topic (rate + bandwidth)."""

    WINDOW = 5.0

    def __init__(self, name: str, type_name: str):
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

    def rate_hz(self) -> float:
        with self.lock:
            n = len(self.samples)
            if n < 2:
                return 0.0
            span = self.samples[-1][0] - self.samples[0][0]
            return (n - 1) / span if span > 0 else 0.0

    def bandwidth_bps(self) -> float:
        with self.lock:
            if not self.samples:
                return 0.0
            span = self.samples[-1][0] - self.samples[0][0]
            total = sum(b for _, b in self.samples)
            return total / span if span > 0.1 else float(total)


def human_bytes(bps: float) -> str:
    for unit in ("B/s", "KB/s", "MB/s", "GB/s"):
        if bps < 1024.0:
            return f"{bps:.1f} {unit}"
        bps /= 1024.0
    return f"{bps:.1f} TB/s"


class MiraNode(Node):
    """Discovers ROS2 topics dynamically and subscribes with raw=True."""

    def __init__(self, topic_filter: str = ""):
        super().__init__("mira_monitor")
        self.topic_filter = topic_filter
        self.stats: dict = {}
        self._subs = {}
        self.create_timer(2.0, self.discover)

    def discover(self):
        for name, types in self.get_topic_names_and_types():
            if name in self._subs or not types:
                continue
            if self.topic_filter and self.topic_filter not in name:
                continue
            if name.startswith("/parameter_events") or name.startswith("/rosout"):
                continue
            type_name = types[0]
            try:
                from rosidl_runtime_py.utilities import get_message
                msg_cls = get_message(type_name)
            except Exception:
                continue
            st = TopicStats(name, type_name)
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


# =========================================================================== #
#  CAN bus monitoring (view 2) — pure Linux SocketCAN, no python-can needed
# =========================================================================== #
CAN_FRAME_FMT = "<IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

# Known ARS408 message names (base IDs, sensor 0). Offset = sensor_id * 0x10.
ARS408_BASE_NAMES = {
    0x200: "RadarCfg",
    0x201: "RadarState",
    0x600: "Cluster_0_Status",
    0x701: "Cluster_1_General",
    0x60A: "Object_0_Status",
    0x60B: "Object_1_General",
    0x60C: "Object_2_Quality",
    0x60D: "Object_3_Extended",
    0x60E: "Object_4_Warning",
}

OBJ_CLASS_NAMES = {
    0: ("point", "white"),
    1: ("car", "green"),
    2: ("truck", "yellow"),
    3: ("pedestrian", "red"),
    4: ("motorcycle", "magenta"),
    5: ("bicycle", "cyan"),
    6: ("wide", "blue"),
    7: ("reserved", "dim"),
}

DYNPROP_NAMES = {
    0: "moving", 1: "stationary", 2: "oncoming", 3: "stat. cand.",
    4: "unknown", 5: "cross stat.", 6: "cross mov.", 7: "stopped",
}


class CanFrameStats:
    WINDOW = 5.0

    def __init__(self, can_id: int):
        self.can_id = can_id
        self.count = 0
        self.dlc = 0
        self.data = b""
        self.times = deque()
        self.lock = threading.Lock()

    def on_frame(self, dlc: int, data: bytes):
        now = time.monotonic()
        with self.lock:
            self.count += 1
            self.dlc = dlc
            self.data = data[:dlc]
            self.times.append(now)
            cutoff = now - self.WINDOW
            while self.times and self.times[0] < cutoff:
                self.times.popleft()

    def rate_hz(self) -> float:
        with self.lock:
            n = len(self.times)
            if n < 2:
                return 0.0
            span = self.times[-1] - self.times[0]
            return (n - 1) / span if span > 0 else 0.0


class RadarObject:
    """One ARS408 object, merged from Object_1_General + Object_3_Extended."""

    __slots__ = ("obj_id", "dist_long", "dist_lat", "vrel_long", "vrel_lat",
                 "rcs", "dyn_prop", "obj_class", "stamp")

    def __init__(self, obj_id: int):
        self.obj_id = obj_id
        self.dist_long = 0.0
        self.dist_lat = 0.0
        self.vrel_long = 0.0
        self.vrel_lat = 0.0
        self.rcs = 0.0
        self.dyn_prop = 4
        self.obj_class = None
        self.stamp = time.monotonic()


class CanMonitor(threading.Thread):
    """Reads raw SocketCAN frames, keeps per-ID stats and decodes ARS408 objects."""

    def __init__(self, interface: str, sensor_id: int = 0, dbc_path: str = ""):
        super().__init__(daemon=True)
        self.interface = interface
        self.offset = sensor_id * 0x10
        self.frames: dict = {}
        self.objects: dict = {}
        self.n_objects_reported = 0
        self.error = ""
        self.lock = threading.Lock()
        self.db = None
        if dbc_path:
            try:
                import cantools
                self.db = cantools.database.load_file(dbc_path)
            except Exception as e:
                self.error = f"DBC load failed: {e}"

    # ---- ARS408 built-in decoding (Motorola byte order) ------------------- #
    def _decode_object_general(self, d: bytes):
        obj_id = d[0]
        dist_long = (((d[1] << 5) | (d[2] >> 3)) * 0.2) - 500.0
        dist_lat = ((((d[2] & 0x07) << 8) | d[3]) * 0.2) - 204.6
        vrel_long = (((d[4] << 2) | (d[5] >> 6)) * 0.25) - 128.0
        vrel_lat = ((((d[5] & 0x3F) << 3) | (d[6] >> 5)) * 0.25) - 64.0
        dyn_prop = d[6] & 0x07
        rcs = d[7] * 0.5 - 64.0
        with self.lock:
            obj = self.objects.setdefault(obj_id, RadarObject(obj_id))
            obj.dist_long, obj.dist_lat = dist_long, dist_lat
            obj.vrel_long, obj.vrel_lat = vrel_long, vrel_lat
            obj.dyn_prop, obj.rcs = dyn_prop, rcs
            obj.stamp = time.monotonic()

    def _decode_object_extended(self, d: bytes):
        obj_id = d[0]
        obj_class = d[3] & 0x07
        with self.lock:
            obj = self.objects.setdefault(obj_id, RadarObject(obj_id))
            obj.obj_class = obj_class
            obj.stamp = time.monotonic()

    def _decode_object_status(self, d: bytes):
        self.n_objects_reported = d[0]
        # purge stale objects (not refreshed in the last second)
        now = time.monotonic()
        with self.lock:
            stale = [k for k, o in self.objects.items() if now - o.stamp > 1.0]
            for k in stale:
                del self.objects[k]

    def frame_name(self, can_id: int) -> str:
        if self.db is not None:
            try:
                return self.db.get_message_by_frame_id(can_id).name
            except Exception:
                pass
        base = can_id - self.offset
        return ARS408_BASE_NAMES.get(base, "")

    def run(self):
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((self.interface,))
            sock.settimeout(1.0)
        except OSError as e:
            self.error = f"CAN open failed on '{self.interface}': {e}"
            return
        while True:
            try:
                frame = sock.recv(CAN_FRAME_SIZE)
            except socket.timeout:
                continue
            except OSError as e:
                self.error = f"CAN read error: {e}"
                return
            can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, frame)
            can_id &= 0x1FFFFFFF
            st = self.frames.setdefault(can_id, CanFrameStats(can_id))
            st.on_frame(dlc, data)
            base = can_id - self.offset
            if dlc == 8:
                if base == 0x60B:
                    self._decode_object_general(data)
                elif base == 0x60D:
                    self._decode_object_extended(data)
                elif base == 0x60A:
                    self._decode_object_status(data)


# =========================================================================== #
#  Health rules (view 1)
# =========================================================================== #
def load_rules(path: str) -> dict:
    if not path:
        return {}
    if yaml is None:
        return {}
    with open(path) as f:
        data = yaml.safe_load(f) or {}
    return data.get("topics", {})


def health_status(name: str, hz: float, rules: dict) -> Text:
    rule = rules.get(name)
    if not rule:
        return Text("—", style="dim")
    lo, hi = rule.get("min_hz", 0), rule.get("max_hz", float("inf"))
    if lo <= hz <= hi:
        return Text("OK", style="bold green")
    return Text("FAIL", style="bold red")


# =========================================================================== #
#  Views
# =========================================================================== #
def view_topics(node: MiraNode, rules: dict) -> Table:
    table = Table(expand=True, header_style="bold cyan", border_style="dim")
    table.add_column("Topic", overflow="fold", ratio=3)
    table.add_column("Type", overflow="fold", ratio=2, style="magenta")
    table.add_column("Hz", justify="right", ratio=1)
    table.add_column("Bandwidth", justify="right", ratio=1)
    table.add_column("Msgs", justify="right", ratio=1)
    table.add_column("Health", justify="center", ratio=1)
    for name in sorted(node.stats):
        st = node.stats[name]
        hz = st.rate_hz()
        table.add_row(
            name,
            st.type_name.split("/")[-1],
            Text(f"{hz:.1f}", style="green" if hz > 0 else "dim"),
            human_bytes(st.bandwidth_bps()),
            str(st.count),
            health_status(name, hz, rules),
        )
    return table


def view_can(can: CanMonitor) -> Table:
    table = Table(expand=True, header_style="bold cyan", border_style="dim")
    table.add_column("CAN ID", justify="right", ratio=1)
    table.add_column("Name", ratio=2, style="magenta")
    table.add_column("Hz", justify="right", ratio=1)
    table.add_column("DLC", justify="right", ratio=1)
    table.add_column("Data (hex)", ratio=3, style="white")
    table.add_column("Count", justify="right", ratio=1)
    if can.error:
        table.add_row(Text(can.error, style="bold red"), "", "", "", "", "")
        return table
    for can_id in sorted(can.frames):
        st = can.frames[can_id]
        hz = st.rate_hz()
        table.add_row(
            f"0x{can_id:03X}",
            can.frame_name(can_id),
            Text(f"{hz:.1f}", style="green" if hz > 0 else "dim"),
            str(st.dlc),
            " ".join(f"{b:02X}" for b in st.data),
            str(st.count),
        )
    return table


def view_objects(can: CanMonitor) -> Table:
    table = Table(expand=True, header_style="bold cyan", border_style="dim")
    table.add_column("ID", justify="right", ratio=1)
    table.add_column("Class", ratio=2)
    table.add_column("DistX (m)", justify="right", ratio=1)
    table.add_column("DistY (m)", justify="right", ratio=1)
    table.add_column("Vx (m/s)", justify="right", ratio=1)
    table.add_column("Vy (m/s)", justify="right", ratio=1)
    table.add_column("RCS (dBm²)", justify="right", ratio=1)
    table.add_column("Motion", ratio=1, style="cyan")
    with can.lock:
        objs = sorted(can.objects.values(), key=lambda o: o.dist_long)
    for o in objs:
        if o.obj_class is not None:
            cls_name, cls_color = OBJ_CLASS_NAMES.get(o.obj_class, ("?", "dim"))
            cls = Text(cls_name, style=f"bold {cls_color}")
        else:
            cls = Text("…", style="dim")
        table.add_row(
            str(o.obj_id), cls,
            f"{o.dist_long:.1f}", f"{o.dist_lat:.1f}",
            f"{o.vrel_long:.2f}", f"{o.vrel_lat:.2f}",
            f"{o.rcs:.1f}",
            DYNPROP_NAMES.get(o.dyn_prop, "?"),
        )
    return table


VIEW_TITLES = {
    "1": "ROS2 Topics",
    "2": "CAN Frames",
    "3": "Radar Objects (ARS408)",
}


def build_layout(view: str, node: MiraNode, can, rules: dict,
                 start: float, rviz_msg: str) -> Layout:
    if view == "2" and can:
        table = view_can(can)
    elif view == "3" and can:
        table = view_objects(can)
    else:
        view = "1"
        table = view_topics(node, rules)

    uptime = int(time.monotonic() - start)
    n_obj = len(can.objects) if can else 0
    parts = [
        ("  MIRA v2 ", "bold white on blue"),
        (f"  [{view}] {VIEW_TITLES[view]}  ", "bold yellow"),
        ("keys: ", "dim"), ("1", "bold"), (" topics  ", "dim"),
        ("2", "bold"), (" CAN  ", "dim"),
        ("3", "bold"), (" objects  ", "dim"),
        ("r", "bold"), (" RViz2  ", "dim"),
        ("q", "bold"), (" quit  ", "dim"),
        (f"| up {uptime // 60:02d}:{uptime % 60:02d} ", "dim"),
    ]
    if can:
        parts.append((f"| objects: {n_obj} ", "cyan"))
    if rviz_msg:
        parts.append((f"| {rviz_msg} ", "bold green"))
    footer = Text.assemble(*parts)

    layout = Layout()
    layout.split_column(
        Layout(Panel(table, title=f"[bold]MIRA — {VIEW_TITLES[view]}[/bold]",
                     border_style="blue"), name="main"),
        Layout(footer, size=1, name="footer"),
    )
    return layout


# =========================================================================== #
#  Keyboard (raw, non-blocking)
# =========================================================================== #
class Keyboard:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old)

    def get_key(self) -> str:
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return ""


def launch_rviz() -> str:
    try:
        subprocess.Popen(["rviz2"],
                         stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL,
                         start_new_session=True)
        return "RViz2 launched"
    except FileNotFoundError:
        return "rviz2 not found!"


# =========================================================================== #
#  Main
# =========================================================================== #
def main():
    parser = argparse.ArgumentParser(description="MIRA v2 — ROS2 + CAN terminal monitor")
    parser.add_argument("--filter", default="", help="only ROS topics containing this string")
    parser.add_argument("--rules", default="", help="YAML file with topic health rules")
    parser.add_argument("--refresh", type=float, default=0.5, help="UI refresh period (s)")
    parser.add_argument("--can", default="", metavar="IFACE",
                        help="CAN interface to monitor (e.g. can0, vcan0)")
    parser.add_argument("--sensor-id", type=int, default=0,
                        help="ARS408 SensorId (CAN ID offset = id*0x10)")
    parser.add_argument("--dbc", default="", help="optional DBC file for CAN decoding")
    args = parser.parse_args()

    rules = load_rules(args.rules)

    can = None
    if args.can:
        can = CanMonitor(args.can, sensor_id=args.sensor_id, dbc_path=args.dbc)
        can.start()

    rclpy.init()
    node = MiraNode(topic_filter=args.filter)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    console = Console()
    start = time.monotonic()
    view = "1"
    rviz_msg = ""
    rviz_msg_until = 0.0

    try:
        with Keyboard() as kb, \
             Live(build_layout(view, node, can, rules, start, ""),
                  console=console, refresh_per_second=8, screen=True) as live:
            while True:
                deadline = time.monotonic() + args.refresh
                while time.monotonic() < deadline:
                    key = kb.get_key()
                    if key in ("1", "2", "3"):
                        if key in ("2", "3") and not can:
                            rviz_msg = "start with --can can0 to enable this view"
                            rviz_msg_until = time.monotonic() + 3
                        else:
                            view = key
                    elif key == "r":
                        rviz_msg = launch_rviz()
                        rviz_msg_until = time.monotonic() + 3
                    elif key == "q":
                        raise KeyboardInterrupt
                    time.sleep(0.03)
                if time.monotonic() > rviz_msg_until:
                    rviz_msg = ""
                live.update(build_layout(view, node, can, rules, start, rviz_msg))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        console.print("[bold blue]MIRA[/bold blue] stopped. Bye!")


if __name__ == "__main__":
    main()
