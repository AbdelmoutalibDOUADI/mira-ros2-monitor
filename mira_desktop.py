#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MIRA Desktop - Live diagnostic interface for ROS2 + CAN (DearPyGui)
===================================================================
A single-window desktop application to diagnose an entire robot live:
lidar, radar, any ROS2 topic, and the raw CAN bus - in one place.

Tabs
----
  * Topics      - live table (Hz, bandwidth, count) + Hz history plot
  * Inspector   - view the latest message of ANY topic as YAML, live
  * PointCloud  - top-down viewer for ANY PointCloud2 topic
                  (lidar or radar), points color-coded by height
  * CAN         - live frame table, bus load, per-ID rate history plot
  * Radar       - ARS408 objects: 2D bird's-eye plot, class filters, CSV
  * Tools       - RViz2 launcher, rosbag record / play, CSV exports

Usage
-----
    source /opt/ros/humble/setup.bash
    pip install dearpygui
    python3 mira_desktop.py --can can0

Requires a display (X11) - same setup as RViz2 in Docker.

Author: Abdelmoutalib Douadi - MIVIA Lab, UNISA (2026)
License: MIT
"""

import argparse
import csv
import os
import signal
import subprocess
import threading
import time
from collections import deque
from datetime import datetime

import dearpygui.dearpygui as dpg
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mira import (MiraNode, CanMonitor, OBJ_CLASS_NAMES, DYNPROP_NAMES,
                  human_bytes, load_rules)

REFRESH_S = 0.5
HISTORY_S = 60.0
MAX_CLOUD_POINTS = 4000
MAX_YAML_CHARS = 12000

CLASS_RGBA = {
    0: (230, 237, 243, 255),   # point
    1: (63, 185, 80, 255),     # car
    2: (210, 153, 34, 255),    # truck
    3: (248, 81, 73, 255),     # pedestrian
    4: (188, 140, 255, 255),   # motorcycle
    5: (57, 197, 207, 255),    # bicycle
    6: (88, 166, 255, 255),    # wide
    7: (139, 148, 158, 255),   # reserved
}

# height buckets for pointcloud coloring (z in meters)
Z_BUCKETS = [
    ("z < 0 m",      lambda z: z < 0.0,          (88, 166, 255, 255)),
    ("0 - 0.5 m",    lambda z: 0.0 <= z < 0.5,   (63, 185, 80, 255)),
    ("0.5 - 1.5 m",  lambda z: 0.5 <= z < 1.5,   (210, 153, 34, 255)),
    ("1.5 - 3 m",    lambda z: 1.5 <= z < 3.0,   (248, 81, 73, 255)),
    ("z >= 3 m",      lambda z: z >= 3.0,         (188, 140, 255, 255)),
]


# =========================================================================== #
#  Diagnostic node - extends MiraNode with inspector + pointcloud viewer
# =========================================================================== #
class DiagnosticNode(MiraNode):
    def __init__(self, topic_filter: str = ""):
        super().__init__(topic_filter)
        self.inspect_topic = ""
        self._inspect_sub = None
        self._inspect_msg = None
        self.inspect_lock = threading.Lock()

        self.cloud_topic = ""
        self._cloud_sub = None
        self._cloud_msg = None
        self.cloud_count = 0
        self.cloud_stamp = 0.0
        self.cloud_lock = threading.Lock()

    def _qos(self):
        return QoSProfile(depth=1,
                          reliability=ReliabilityPolicy.BEST_EFFORT,
                          history=HistoryPolicy.KEEP_LAST)

    def _msg_class(self, topic: str):
        for name, types in self.get_topic_names_and_types():
            if name == topic and types:
                from rosidl_runtime_py.utilities import get_message
                return get_message(types[0])
        return None

    # ---- inspector: latest message of any topic ------------------------- #
    def set_inspect_topic(self, topic: str):
        if self._inspect_sub is not None:
            self.destroy_subscription(self._inspect_sub)
            self._inspect_sub = None
        with self.inspect_lock:
            self._inspect_msg = None
        self.inspect_topic = topic
        if not topic:
            return
        cls = self._msg_class(topic)
        if cls is None:
            return
        self._inspect_sub = self.create_subscription(
            cls, topic, self._on_inspect, self._qos())

    def _on_inspect(self, msg):
        with self.inspect_lock:
            self._inspect_msg = msg

    def inspect_yaml(self) -> str:
        with self.inspect_lock:
            msg = self._inspect_msg
        if msg is None:
            return "(waiting for a message...)"
        try:
            from rosidl_runtime_py import message_to_yaml
            txt = message_to_yaml(msg)
        except Exception as e:
            return f"(cannot render: {e})"
        if len(txt) > MAX_YAML_CHARS:
            txt = txt[:MAX_YAML_CHARS] + "\n... (truncated)"
        return txt

    # ---- pointcloud viewer: any PointCloud2 topic ----------------------- #
    def set_cloud_topic(self, topic: str):
        if self._cloud_sub is not None:
            self.destroy_subscription(self._cloud_sub)
            self._cloud_sub = None
        with self.cloud_lock:
            self._cloud_msg = None
            self.cloud_count = 0
        self.cloud_topic = topic
        if not topic:
            return
        cls = self._msg_class(topic)
        if cls is None:
            return
        self._cloud_sub = self.create_subscription(
            cls, topic, self._on_cloud, self._qos())

    def _on_cloud(self, msg):
        with self.cloud_lock:
            self._cloud_msg = msg
            self.cloud_stamp = time.monotonic()

    def cloud_points(self):
        """Decode latest PointCloud2 -> (xs_by_bucket, ys_by_bucket, n_total)."""
        with self.cloud_lock:
            msg = self._cloud_msg
        empty = [[[], []] for _ in Z_BUCKETS]
        if msg is None:
            return empty, 0
        try:
            from sensor_msgs_py import point_cloud2
            pts = point_cloud2.read_points(
                msg, field_names=("x", "y", "z"), skip_nans=True)
            n = len(pts)
            step = max(1, n // MAX_CLOUD_POINTS)
            buckets = [[[], []] for _ in Z_BUCKETS]
            for i in range(0, n, step):
                p = pts[i]
                x, y, z = float(p[0]), float(p[1]), float(p[2])
                for bi, (_, test, _) in enumerate(Z_BUCKETS):
                    if test(z):
                        # ROS: x forward, y left -> plot: X = -y, Y = x
                        buckets[bi][0].append(-y)
                        buckets[bi][1].append(x)
                        break
            self.cloud_count = n
            return buckets, n
        except Exception:
            return empty, 0

    def pointcloud_topics(self):
        return sorted(name for name, st in self.stats.items()
                      if st.type_name.endswith("PointCloud2"))


# =========================================================================== #
#  Application
# =========================================================================== #
class App:
    def __init__(self, args):
        self.args = args
        self.rules = load_rules(args.rules)

        self.can = None
        if args.can:
            self.can = CanMonitor(args.can, sensor_id=args.sensor_id,
                                  dbc_path=args.dbc)
            self.can.start()

        rclpy.init()
        self.node = DiagnosticNode(topic_filter=args.filter)
        threading.Thread(target=rclpy.spin, args=(self.node,),
                         daemon=True).start()

        self.start = time.monotonic()
        self.can_paused = False
        self.hz_history = {}
        self.can_history = {}
        self.selected_topic = ""
        self.selected_can_id = ""
        self.class_enabled = {i: True for i in CLASS_RGBA}
        self.record_proc = None
        self.play_proc = None
        self.class_series = {}
        self.cloud_series = []
        os.makedirs("exports", exist_ok=True)

    # ------------------------------------------------------------------ #
    #  Actions
    # ------------------------------------------------------------------ #
    def launch_rviz(self):
        try:
            subprocess.Popen(["rviz2"], stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL, start_new_session=True)
            self.status("RViz2 launched")
        except FileNotFoundError:
            self.status("rviz2 not found!", error=True)

    def toggle_record(self):
        if self.record_proc is None:
            topics = dpg.get_value("rec_topics").strip()
            cmd = ["ros2", "bag", "record"]
            cmd += ["-a"] if not topics else topics.split()
            os.makedirs("Bag", exist_ok=True)
            self.record_proc = subprocess.Popen(
                cmd, cwd="Bag", stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL, start_new_session=True)
            dpg.set_item_label("btn_record", "STOP recording")
            self.status("Recording started"
                        f" ({'all topics' if not topics else topics})")
        else:
            os.killpg(os.getpgid(self.record_proc.pid), signal.SIGINT)
            self.record_proc = None
            dpg.set_item_label("btn_record", "START recording")
            self.status("Recording stopped - bag saved under Bag/")

    def toggle_play(self):
        if self.play_proc is None:
            path = dpg.get_value("play_path").strip()
            if not path:
                self.status("Enter a bag path first", error=True)
                return
            cmd = ["ros2", "bag", "play", path,
                   "--rate", str(dpg.get_value("play_rate"))]
            if dpg.get_value("play_loop"):
                cmd.append("--loop")
            self.play_proc = subprocess.Popen(
                cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True)
            dpg.set_item_label("btn_play", "STOP playback")
            self.status(f"Playing {path}")
        else:
            os.killpg(os.getpgid(self.play_proc.pid), signal.SIGINT)
            self.play_proc = None
            dpg.set_item_label("btn_play", "PLAY bag")
            self.status("Playback stopped")

    def export_objects_csv(self):
        if not self.can:
            self.status("CAN disabled", error=True)
            return
        fname = f"exports/objects_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with self.can.lock:
            objs = list(self.can.objects.values())
        with open(fname, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["id", "class", "dist_long_m", "dist_lat_m",
                        "vrel_long_ms", "vrel_lat_ms", "rcs_dbm2", "motion"])
            for o in objs:
                cls = OBJ_CLASS_NAMES.get(o.obj_class, ("?",))[0] \
                    if o.obj_class is not None else "?"
                w.writerow([o.obj_id, cls, f"{o.dist_long:.2f}",
                            f"{o.dist_lat:.2f}", f"{o.vrel_long:.2f}",
                            f"{o.vrel_lat:.2f}", f"{o.rcs:.1f}",
                            DYNPROP_NAMES.get(o.dyn_prop, "?")])
        self.status(f"Exported {len(objs)} objects -> {fname}")

    def export_topics_csv(self):
        fname = f"exports/topics_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with open(fname, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["topic", "type", "hz", "bandwidth_bps", "count"])
            for name in sorted(self.node.stats):
                st = self.node.stats[name]
                w.writerow([name, st.type_name, f"{st.rate_hz():.2f}",
                            f"{st.bandwidth_bps():.0f}", st.count])
        self.status(f"Exported topics -> {fname}")

    def status(self, msg: str, error: bool = False):
        dpg.set_value("status_text", msg)
        dpg.configure_item("status_text",
                           color=(248, 81, 73) if error else (63, 185, 80))

    # ------------------------------------------------------------------ #
    #  UI construction
    # ------------------------------------------------------------------ #
    def build(self):
        dpg.create_context()
        with dpg.theme() as global_theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (13, 17, 23))
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (22, 27, 34))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (33, 38, 45))
                dpg.add_theme_color(dpg.mvThemeCol_Header, (33, 38, 45))
                dpg.add_theme_color(dpg.mvThemeCol_Button, (35, 134, 54))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (46, 160, 67))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
        dpg.bind_theme(global_theme)

        with dpg.window(tag="main"):
            with dpg.group(horizontal=True):
                dpg.add_text("MIRA", color=(88, 166, 255))
                dpg.add_text("Live Diagnostic Interface - ROS2 + CAN",
                             color=(139, 148, 158))
                dpg.add_spacer(width=30)
                dpg.add_text("", tag="status_text", color=(63, 185, 80))
            dpg.add_separator()

            with dpg.tab_bar():
                self._tab_topics()
                self._tab_inspector()
                self._tab_pointcloud()
                self._tab_can()
                self._tab_radar()
                self._tab_tools()

            dpg.add_separator()
            dpg.add_text("", tag="footer", color=(139, 148, 158))

        dpg.create_viewport(title="MIRA - Live Diagnostic Interface",
                            width=1280, height=800)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main", True)

    def _tab_topics(self):
        with dpg.tab(label="  Topics  "):
            with dpg.group(horizontal=True):
                dpg.add_combo([], tag="topic_combo", width=420,
                              label="plot Hz history",
                              callback=lambda s, v: setattr(
                                  self, "selected_topic", v))
                dpg.add_button(label="Export CSV",
                               callback=lambda: self.export_topics_csv())
            with dpg.plot(height=170, width=-1):
                dpg.add_plot_axis(dpg.mvXAxis, label="t (s)", tag="hz_x")
                with dpg.plot_axis(dpg.mvYAxis, label="Hz", tag="hz_y"):
                    dpg.add_line_series([], [], tag="hz_series", label="rate")
            with dpg.table(tag="topics_table", header_row=True,
                           resizable=True, scrollY=True, height=-1,
                           borders_innerH=True, borders_outerH=True):
                for col in ("Topic", "Type", "Hz", "Bandwidth",
                            "Msgs", "Health"):
                    dpg.add_table_column(label=col)

    def _tab_inspector(self):
        with dpg.tab(label="  Inspector  "):
            with dpg.group(horizontal=True):
                dpg.add_combo([], tag="inspect_combo", width=480,
                              label="topic",
                              callback=lambda s, v:
                                  self.node.set_inspect_topic(v))
                dpg.add_checkbox(label="live", tag="inspect_live",
                                 default_value=True)
                dpg.add_text("", tag="inspect_info",
                             color=(139, 148, 158))
            dpg.add_input_text(tag="inspect_text", multiline=True,
                               readonly=True, width=-1, height=-1,
                               default_value="(select a topic above)")

    def _tab_pointcloud(self):
        with dpg.tab(label="  PointCloud  "):
            with dpg.group(horizontal=True):
                dpg.add_combo([], tag="pc_combo", width=480,
                              label="PointCloud2 topic (lidar / radar)",
                              callback=lambda s, v:
                                  self.node.set_cloud_topic(v))
                dpg.add_text("", tag="pc_info", color=(139, 148, 158))
            with dpg.plot(height=-1, width=-1, tag="pc_plot"):
                dpg.add_plot_legend()
                dpg.add_plot_axis(dpg.mvXAxis,
                                  label="lateral (m)  <- left", tag="pc_x")
                with dpg.plot_axis(dpg.mvYAxis,
                                   label="forward (m)", tag="pc_y"):
                    for label, _, rgba in Z_BUCKETS:
                        s = dpg.add_scatter_series([], [], label=label)
                        self.cloud_series.append(s)
                        with dpg.theme() as th:
                            with dpg.theme_component(dpg.mvScatterSeries):
                                dpg.add_theme_color(
                                    dpg.mvPlotCol_MarkerFill, rgba,
                                    category=dpg.mvThemeCat_Plots)
                                dpg.add_theme_color(
                                    dpg.mvPlotCol_MarkerOutline, rgba,
                                    category=dpg.mvThemeCat_Plots)
                                dpg.add_theme_style(
                                    dpg.mvPlotStyleVar_MarkerSize, 2,
                                    category=dpg.mvThemeCat_Plots)
                        dpg.bind_item_theme(s, th)
                dpg.set_axis_limits("pc_x", -50, 50)
                dpg.set_axis_limits("pc_y", -20, 120)
            # free axes after initial framing so the user can zoom/pan
            dpg.set_frame_callback(30, lambda: (
                dpg.set_axis_limits_auto("pc_x"),
                dpg.set_axis_limits_auto("pc_y")))

    def _tab_can(self):
        with dpg.tab(label="  CAN  "):
            with dpg.group(horizontal=True):
                dpg.add_button(label="Pause", tag="btn_can_pause",
                               callback=self._toggle_can_pause)
                dpg.add_combo([], tag="can_combo", width=200,
                              label="plot ID rate",
                              callback=lambda s, v: setattr(
                                  self, "selected_can_id", v))
                dpg.add_text("", tag="can_load", color=(139, 148, 158))
            with dpg.plot(height=150, width=-1):
                dpg.add_plot_axis(dpg.mvXAxis, label="t (s)", tag="can_x")
                with dpg.plot_axis(dpg.mvYAxis, label="frames/s",
                                   tag="can_y"):
                    dpg.add_line_series([], [], tag="can_series",
                                        label="rate")
            with dpg.table(tag="can_table", header_row=True,
                           resizable=True, scrollY=True, height=-1,
                           borders_innerH=True, borders_outerH=True):
                for col in ("CAN ID", "Name", "Hz", "DLC",
                            "Data (hex)", "Count"):
                    dpg.add_table_column(label=col)

    def _tab_radar(self):
        with dpg.tab(label="  Radar Objects  "):
            with dpg.group(horizontal=True):
                for cid, (name, _) in OBJ_CLASS_NAMES.items():
                    if cid == 7:
                        continue
                    dpg.add_checkbox(
                        label=name, default_value=True,
                        callback=lambda s, v, c=cid:
                            self.class_enabled.__setitem__(c, v))
                dpg.add_button(label="Export objects CSV",
                               callback=lambda: self.export_objects_csv())
            with dpg.group(horizontal=True):
                with dpg.plot(height=470, width=540, tag="radar_plot"):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis,
                                      label="lateral (m)  <- left",
                                      tag="radar_x")
                    with dpg.plot_axis(dpg.mvYAxis,
                                       label="longitudinal (m)",
                                       tag="radar_y"):
                        for cid, (name, _) in OBJ_CLASS_NAMES.items():
                            s = dpg.add_scatter_series([], [], label=name)
                            self.class_series[cid] = s
                            with dpg.theme() as th:
                                with dpg.theme_component(
                                        dpg.mvScatterSeries):
                                    dpg.add_theme_color(
                                        dpg.mvPlotCol_MarkerFill,
                                        CLASS_RGBA[cid],
                                        category=dpg.mvThemeCat_Plots)
                                    dpg.add_theme_color(
                                        dpg.mvPlotCol_MarkerOutline,
                                        CLASS_RGBA[cid],
                                        category=dpg.mvThemeCat_Plots)
                                    dpg.add_theme_style(
                                        dpg.mvPlotStyleVar_MarkerSize, 6,
                                        category=dpg.mvThemeCat_Plots)
                            dpg.bind_item_theme(s, th)
                    dpg.set_axis_limits("radar_x", -40, 40)
                    dpg.set_axis_limits("radar_y", 0, 150)
                dpg.set_frame_callback(31, lambda: (
                    dpg.set_axis_limits_auto("radar_x"),
                    dpg.set_axis_limits_auto("radar_y")))
                with dpg.table(tag="obj_table", header_row=True,
                               resizable=True, scrollY=True, height=470,
                               borders_innerH=True, borders_outerH=True):
                    for col in ("ID", "Class", "DistX", "DistY",
                                "Vx", "RCS", "Motion"):
                        dpg.add_table_column(label=col)

    def _tab_tools(self):
        with dpg.tab(label="  Tools  "):
            dpg.add_text("Visualization", color=(88, 166, 255))
            dpg.add_button(label="Launch RViz2", width=200,
                           callback=lambda: self.launch_rviz())
            dpg.add_separator()
            dpg.add_text("Rosbag record", color=(88, 166, 255))
            dpg.add_input_text(tag="rec_topics", width=520,
                               hint="topics separated by spaces (empty = all)")
            dpg.add_button(label="START recording", tag="btn_record",
                           width=200, callback=lambda: self.toggle_record())
            dpg.add_separator()
            dpg.add_text("Rosbag play", color=(88, 166, 255))
            dpg.add_input_text(tag="play_path", width=520,
                               hint="/path/to/bag_folder")
            with dpg.group(horizontal=True):
                dpg.add_checkbox(label="loop", tag="play_loop",
                                 default_value=True)
                dpg.add_input_float(tag="play_rate", label="rate",
                                    default_value=1.0, width=120,
                                    min_value=0.1, max_value=10.0)
            dpg.add_button(label="PLAY bag", tag="btn_play", width=200,
                           callback=lambda: self.toggle_play())

    def _toggle_can_pause(self):
        self.can_paused = not self.can_paused
        dpg.set_item_label("btn_can_pause",
                           "Resume" if self.can_paused else "Pause")

    # ------------------------------------------------------------------ #
    #  Periodic refresh
    # ------------------------------------------------------------------ #
    def _rebuild_table(self, tag: str, rows):
        dpg.delete_item(tag, children_only=True, slot=1)
        for row in rows:
            with dpg.table_row(parent=tag):
                for cell, color in row:
                    dpg.add_text(cell, color=color)

    def refresh(self):
        now = time.monotonic()
        dim = (139, 148, 158)
        green = (63, 185, 80)
        red = (248, 81, 73)
        t_rel = now - self.start

        # ---- topics ---- #
        rows = []
        names = sorted(self.node.stats)
        for name in names:
            st = self.node.stats[name]
            hz = st.rate_hz()
            hist = self.hz_history.setdefault(name, deque())
            hist.append((t_rel, hz))
            while hist and hist[0][0] < t_rel - HISTORY_S:
                hist.popleft()
            rule = self.rules.get(name)
            if rule:
                ok = rule.get("min_hz", 0) <= hz <= rule.get(
                    "max_hz", float("inf"))
                health = ("OK", green) if ok else ("FAIL", red)
            else:
                health = ("-", dim)
            rows.append([
                (name, None), (st.type_name.split("/")[-1], dim),
                (f"{hz:.1f}", green if hz > 0 else dim),
                (human_bytes(st.bandwidth_bps()), None),
                (str(st.count), None), health,
            ])
        self._rebuild_table("topics_table", rows)
        dpg.configure_item("topic_combo", items=names)
        dpg.configure_item("inspect_combo", items=names)
        dpg.configure_item("pc_combo", items=self.node.pointcloud_topics())

        if self.selected_topic in self.hz_history:
            hist = self.hz_history[self.selected_topic]
            xs = [t for t, _ in hist]
            ys = [h for _, h in hist]
            dpg.set_value("hz_series", [xs, ys])
            dpg.configure_item("hz_series", label=self.selected_topic)
            if xs:
                dpg.set_axis_limits("hz_x", xs[0], max(xs[-1], xs[0] + 1))
                dpg.fit_axis_data("hz_y")

        # ---- inspector ---- #
        if self.node.inspect_topic and dpg.get_value("inspect_live"):
            dpg.set_value("inspect_text", self.node.inspect_yaml())
            st = self.node.stats.get(self.node.inspect_topic)
            if st:
                dpg.set_value("inspect_info",
                              f"{st.type_name} - {st.rate_hz():.1f} Hz")

        # ---- pointcloud ---- #
        if self.node.cloud_topic:
            buckets, n = self.node.cloud_points()
            for series, (xs, ys) in zip(self.cloud_series, buckets):
                dpg.set_value(series, [xs, ys])
            age = now - self.node.cloud_stamp if self.node.cloud_stamp else 0
            dpg.set_value("pc_info",
                          f"{n} points"
                          f" (showing ≤{MAX_CLOUD_POINTS})"
                          f" - last msg {age:.1f}s ago")

        # ---- CAN ---- #
        if self.can and not self.can_paused:
            total_hz = 0.0
            rows = []
            ids = []
            for can_id in sorted(self.can.frames):
                st = self.can.frames[can_id]
                hz = st.rate_hz()
                total_hz += hz
                id_str = f"0x{can_id:03X}"
                ids.append(id_str)
                hist = self.can_history.setdefault(id_str, deque())
                hist.append((t_rel, hz))
                while hist and hist[0][0] < t_rel - HISTORY_S:
                    hist.popleft()
                rows.append([
                    (id_str, None),
                    (self.can.frame_name(can_id), (188, 140, 255)),
                    (f"{hz:.1f}", green if hz > 0 else dim),
                    (str(st.dlc), None),
                    (" ".join(f"{b:02X}" for b in st.data), dim),
                    (str(st.count), None),
                ])
            self._rebuild_table("can_table", rows)
            dpg.configure_item("can_combo", items=ids)
            dpg.set_value("can_load",
                          f"bus: {len(self.can.frames)} IDs, "
                          f"{total_hz:.0f} frames/s"
                          + (f"  |  {self.can.error}"
                             if self.can.error else ""))
            if self.selected_can_id in self.can_history:
                hist = self.can_history[self.selected_can_id]
                xs = [t for t, _ in hist]
                ys = [h for _, h in hist]
                dpg.set_value("can_series", [xs, ys])
                dpg.configure_item("can_series",
                                   label=self.selected_can_id)
                if xs:
                    dpg.set_axis_limits("can_x", xs[0],
                                        max(xs[-1], xs[0] + 1))
                    dpg.fit_axis_data("can_y")

        # ---- radar objects ---- #
        if self.can:
            with self.can.lock:
                objs = sorted(self.can.objects.values(),
                              key=lambda o: o.dist_long)
            per_class = {cid: ([], []) for cid in CLASS_RGBA}
            rows = []
            for o in objs:
                cid = o.obj_class if o.obj_class is not None else 7
                if not self.class_enabled.get(cid, True):
                    continue
                xs, ys = per_class[cid]
                xs.append(-o.dist_lat)
                ys.append(o.dist_long)
                cls_name, _ = OBJ_CLASS_NAMES.get(cid, ("?", ""))
                rgba = CLASS_RGBA.get(cid, (139, 148, 158, 255))
                rows.append([
                    (str(o.obj_id), None),
                    (cls_name, rgba[:3]),
                    (f"{o.dist_long:.1f}", None),
                    (f"{o.dist_lat:.1f}", None),
                    (f"{o.vrel_long:.2f}", None),
                    (f"{o.rcs:.1f}", None),
                    (DYNPROP_NAMES.get(o.dyn_prop, "?"), dim),
                ])
            for cid, (xs, ys) in per_class.items():
                dpg.set_value(self.class_series[cid], [xs, ys])
            self._rebuild_table("obj_table", rows)

        # ---- footer ---- #
        up = int(t_rel)
        n_obj = len(self.can.objects) if self.can else 0
        rec = "REC ●" if self.record_proc else ""
        dpg.set_value("footer",
                      f"uptime {up // 60:02d}:{up % 60:02d}   |   "
                      f"topics: {len(self.node.stats)}   |   "
                      f"objects: {n_obj}   {rec}")

    # ------------------------------------------------------------------ #
    def run(self):
        self.build()
        last = 0.0
        while dpg.is_dearpygui_running():
            if time.monotonic() - last >= REFRESH_S:
                try:
                    self.refresh()
                except Exception as e:
                    dpg.set_value("status_text", f"UI error: {e}")
                last = time.monotonic()
            dpg.render_dearpygui_frame()
        for proc in (self.record_proc, self.play_proc):
            if proc:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                except Exception:
                    pass
        dpg.destroy_context()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(
        description="MIRA Desktop - live diagnostic interface for ROS2 + CAN")
    parser.add_argument("--filter", default="",
                        help="only ROS topics containing this string")
    parser.add_argument("--rules", default="",
                        help="YAML file with topic health rules")
    parser.add_argument("--can", default="", metavar="IFACE",
                        help="CAN interface (can0, vcan0)")
    parser.add_argument("--sensor-id", type=int, default=0,
                        help="ARS408 SensorId offset")
    parser.add_argument("--dbc", default="",
                        help="optional DBC file for CAN decoding")
    args = parser.parse_args()
    App(args).run()


if __name__ == "__main__":
    main()
