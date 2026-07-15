#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MIRA — Monitoring Interface for ROS2 Applications
=================================================
A lightweight, single-file terminal dashboard for ROS2 (Humble/Jazzy).
No GUI, no X11, no web server — runs in any terminal, even over SSH.

Features
--------
* Live topic table: rate (Hz), bandwidth (B/s), message count, type
* Node list with publisher/subscriber counts
* Optional health rules (min/max Hz per topic) via YAML
* Topic filtering with --filter
* Clean color-coded interface (rich)

Usage
-----
    source /opt/ros/humble/setup.bash
    python3 mira.py                          # monitor everything
    python3 mira.py --filter radar           # only topics containing "radar"
    python3 mira.py --rules mira_rules.yaml  # enable health checks
    python3 mira.py --refresh 0.5            # faster UI refresh

Author: Abdelmoutalib Douadi — MIVIA Lab, UNISA (2026)
License: MIT
"""

import argparse
import sys
import threading
import time
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


# --------------------------------------------------------------------------- #
#  Per-topic statistics
# --------------------------------------------------------------------------- #
class TopicStats:
    """Rolling statistics for one topic (rate + bandwidth)."""

    WINDOW = 5.0  # seconds of history used to compute Hz / B/s

    def __init__(self, name: str, type_name: str):
        self.name = name
        self.type_name = type_name
        self.count = 0
        self.samples = deque()          # (timestamp, nbytes)
        self.lock = threading.Lock()

    def on_message(self, raw: bytes):
        now = time.monotonic()
        with self.lock:
            self.count += 1
            self.samples.append((now, len(raw)))
            # drop samples older than the window
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
            total = sum(nbytes for _, nbytes in self.samples)
            return total / span if span > 0.1 else float(total)


def human_bytes(bps: float) -> str:
    """1234567 -> '1.2 MB/s'."""
    for unit in ("B/s", "KB/s", "MB/s", "GB/s"):
        if bps < 1024.0:
            return f"{bps:.1f} {unit}"
        bps /= 1024.0
    return f"{bps:.1f} TB/s"


# --------------------------------------------------------------------------- #
#  The monitoring node
# --------------------------------------------------------------------------- #
class MiraNode(Node):
    """Discovers topics dynamically and subscribes with raw=True (no type import needed
    for size measurement, but the message class is still required by rclpy)."""

    def __init__(self, topic_filter: str = ""):
        super().__init__("mira_monitor")
        self.topic_filter = topic_filter
        self.stats: dict[str, TopicStats] = {}
        self._subs = {}
        # rediscover topics every 2 s
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
                continue  # type not available in this workspace

            st = TopicStats(name, type_name)
            self.stats[name] = st

            qos = QoSProfile(
                depth=5,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
            )
            try:
                self._subs[name] = self.create_subscription(
                    msg_cls, name,
                    lambda raw, s=st: s.on_message(raw),
                    qos, raw=True,
                )
            except Exception:
                self.stats.pop(name, None)


# --------------------------------------------------------------------------- #
#  Health rules (optional YAML)
# --------------------------------------------------------------------------- #
def load_rules(path: str) -> dict:
    """YAML format:
    topics:
      "/radar/pointcloud": {min_hz: 10, max_hz: 20}
      "/lidar/points":     {min_hz: 8}
    """
    if not path:
        return {}
    if yaml is None:
        print("[MIRA] pyyaml missing — rules ignored (pip install pyyaml)")
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


# --------------------------------------------------------------------------- #
#  UI rendering
# --------------------------------------------------------------------------- #
def build_layout(node: MiraNode, rules: dict, start: float) -> Layout:
    table = Table(expand=True, header_style="bold cyan", border_style="dim")
    table.add_column("Topic", overflow="fold", ratio=3)
    table.add_column("Type", overflow="fold", ratio=2, style="magenta")
    table.add_column("Hz", justify="right", ratio=1)
    table.add_column("Bandwidth", justify="right", ratio=1)
    table.add_column("Msgs", justify="right", ratio=1)
    table.add_column("Health", justify="center", ratio=1)

    fails = 0
    for name in sorted(node.stats):
        st = node.stats[name]
        hz = st.rate_hz()
        status = health_status(name, hz, rules)
        if status.plain == "FAIL":
            fails += 1
        hz_style = "green" if hz > 0 else "dim"
        table.add_row(
            name,
            st.type_name.split("/")[-1],
            Text(f"{hz:.1f}", style=hz_style),
            human_bytes(st.bandwidth_bps()),
            str(st.count),
            status,
        )

    uptime = int(time.monotonic() - start)
    footer = Text.assemble(
        ("  MIRA ", "bold white on blue"),
        (f"  topics: {len(node.stats)}  ", "cyan"),
        (f"uptime: {uptime // 60:02d}:{uptime % 60:02d}  ", "dim"),
        (f"health fails: {fails}  ", "bold red" if fails else "green"),
        ("Ctrl+C to quit", "dim"),
    )

    layout = Layout()
    layout.split_column(
        Layout(Panel(table, title="[bold]MIRA — ROS2 Live Monitor[/bold]",
                     border_style="blue"), name="main"),
        Layout(footer, size=1, name="footer"),
    )
    return layout


# --------------------------------------------------------------------------- #
#  Main
# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description="MIRA — ROS2 terminal monitor")
    parser.add_argument("--filter", default="", help="only topics containing this string")
    parser.add_argument("--rules", default="", help="YAML file with health rules")
    parser.add_argument("--refresh", type=float, default=1.0, help="UI refresh period (s)")
    args = parser.parse_args()

    rules = load_rules(args.rules)

    rclpy.init()
    node = MiraNode(topic_filter=args.filter)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    console = Console()
    start = time.monotonic()
    try:
        with Live(build_layout(node, rules, start), console=console,
                  refresh_per_second=4, screen=True) as live:
            while True:
                time.sleep(args.refresh)
                live.update(build_layout(node, rules, start))
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        console.print("[bold blue]MIRA[/bold blue] stopped. Bye!")


if __name__ == "__main__":
    main()
