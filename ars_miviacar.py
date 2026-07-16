#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ARS_MiviaCar — Dedicated dual ARS408-21 radar analysis software
===============================================================
Full CAN-level analysis of the Continental ARS408-21 77 GHz radars
mounted on the MiviaCar research vehicle (front + rear).

Tabs
----
  * Overview     — per-radar RadarState: sensor ID, max distance, output
                   type, power cfg, quality/ext-info flags, motion RX,
                   RCS threshold, ERROR FLAGS, measured cycle rate
  * Objects      — vehicle-centric bird's-eye plot (front + rear merged),
                   full object table: class, distance, velocity, RCS,
                   probability of existence, measurement state, motion
  * Clusters     — raw cluster echoes (0x701) table + scatter plot
  * RCS Analysis — RCS histogram + RCS vs distance scatter, statistics
  * CAN Raw      — every radar CAN frame live: ID, name, rate, data hex

Radar physics reference (ARS408-21):
  * operating band 76..77 GHz, cycle time ~72 ms (≈ 13.9 Hz)
  * far field: 0.25..250 m, near field: 0.25..70..100 m
  * object classes: point, car, truck, pedestrian, motorcycle,
    bicycle, wide

Usage
-----
    source /opt/ros/humble/setup.bash
    python3 ars_miviacar.py --can can0                # front (id 0) + rear (id 1)
    python3 ars_miviacar.py --can vcan0 --front 0 --rear 1
    python3 ars_miviacar.py --can can0 --single 0     # only one radar

Requires a display (X11) — same setup as RViz2 in Docker.

Author: Abdelmoutalib Douadi — MIVIA Lab, UNISA (2026)
License: MIT
"""

import argparse
import csv
import os
import socket
import struct
import threading
import time
from collections import deque
from datetime import datetime

import dearpygui.dearpygui as dpg

CAN_FRAME_FMT = "<IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
REFRESH_S = 0.5
STALE_S = 1.0

BASE_NAMES = {
    0x200: "RadarCfg",
    0x201: "RadarState",
    0x202: "FilterCfg",
    0x203: "FilterState_Header",
    0x204: "FilterState_Cfg",
    0x300: "SpeedInformation",
    0x301: "YawRateInformation",
    0x600: "Cluster_0_Status",
    0x701: "Cluster_1_General",
    0x702: "Cluster_2_Quality",
    0x60A: "Object_0_Status",
    0x60B: "Object_1_General",
    0x60C: "Object_2_Quality",
    0x60D: "Object_3_Extended",
    0x60E: "Object_4_Warning",
    0x408: "CollDetState",
}

CLASS_NAMES = {
    0: "point", 1: "car", 2: "truck", 3: "pedestrian",
    4: "motorcycle", 5: "bicycle", 6: "wide", 7: "reserved",
}
CLASS_RGBA = {
    0: (230, 237, 243, 255), 1: (63, 185, 80, 255),
    2: (210, 153, 34, 255), 3: (248, 81, 73, 255),
    4: (188, 140, 255, 255), 5: (57, 197, 207, 255),
    6: (88, 166, 255, 255), 7: (139, 148, 158, 255),
}
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
#  Decoders
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


class Cluster:
    __slots__ = ("cid", "dist_long", "dist_lat", "vrel_long", "rcs",
                 "dyn_prop", "stamp")

    def __init__(self, cid):
        self.cid = cid
        self.dist_long = self.dist_lat = self.vrel_long = self.rcs = 0.0
        self.dyn_prop = 4
        self.stamp = time.monotonic()


class Ars408Decoder:
    """Full decoder for one ARS408 sensor (one SensorId)."""

    def __init__(self, sensor_id: int, label: str):
        self.sensor_id = sensor_id
        self.label = label
        self.offset = sensor_id * 0x10
        self.state = RadarState()
        self.objects = {}
        self.clusters = {}
        self.n_objects_reported = 0
        self.n_clusters_reported = 0
        self.cycle_times = deque(maxlen=50)   # Object_0_Status arrivals
        self.lock = threading.Lock()

    def cycle_hz(self) -> float:
        if len(self.cycle_times) < 2:
            return 0.0
        span = self.cycle_times[-1] - self.cycle_times[0]
        return (len(self.cycle_times) - 1) / span if span > 0 else 0.0

    def alive(self) -> bool:
        return (time.monotonic() - self.state.stamp) < 2.0 \
            or bool(self.cycle_times) and \
            (time.monotonic() - self.cycle_times[-1]) < 2.0

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
        elif base == 0x600:
            self.n_clusters_reported = d[0] + d[1]
            with self.lock:
                stale = [k for k, c in self.clusters.items()
                         if now - c.stamp > STALE_S]
                for k in stale:
                    del self.clusters[k]
        elif base == 0x701:
            cid = d[0]
            with self.lock:
                c = self.clusters.setdefault(cid, Cluster(cid))
                c.dist_long = (((d[1] << 5) | (d[2] >> 3)) * 0.2) - 500.0
                c.dist_lat = ((((d[2] & 0x03) << 8) | d[3]) * 0.2) - 102.3
                c.vrel_long = (((d[4] << 2) | (d[5] >> 6)) * 0.25) - 128.0
                c.dyn_prop = d[6] & 0x07
                c.rcs = d[7] * 0.5 - 64.0
                c.stamp = now


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
    """One reader for the whole bus, dispatching to per-sensor decoders."""

    def __init__(self, interface: str, decoders):
        super().__init__(daemon=True)
        self.interface = interface
        self.decoders = decoders
        self.frames = {}
        self.error = ""

    def run(self):
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW,
                                 socket.CAN_RAW)
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
            st = self.frames.setdefault(can_id, FrameStats(can_id))
            st.hit(dlc, data)
            if dlc < 8:
                continue
            for dec in self.decoders:
                base = can_id - dec.offset
                if base in (0x201, 0x600, 0x701, 0x60A, 0x60B, 0x60C, 0x60D):
                    dec.handle(base, data)
                    break

    def frame_name(self, can_id):
        for dec in self.decoders:
            base = can_id - dec.offset
            if base in BASE_NAMES:
                tag = f" [{dec.label}]" if len(self.decoders) > 1 else ""
                return BASE_NAMES[base] + tag
        return BASE_NAMES.get(can_id, "")


# =========================================================================== #
#  Application
# =========================================================================== #
class App:
    def __init__(self, args):
        self.args = args
        if args.single is not None:
            self.decoders = [Ars408Decoder(args.single, "RADAR")]
        else:
            self.decoders = [Ars408Decoder(args.front, "FRONT"),
                             Ars408Decoder(args.rear, "REAR")]
        self.reader = CanReader(args.can, self.decoders)
        self.reader.start()
        self.start = time.monotonic()
        self.class_enabled = {i: True for i in CLASS_NAMES}
        self.obj_series = {}     # (decoder_idx, class) -> series
        self.cluster_series = []
        self.rcs_bar = None
        self.rcs_scatter = {}
        os.makedirs("exports", exist_ok=True)

    # ------------------------------------------------------------------ #
    def status(self, msg, error=False):
        dpg.set_value("status_text", msg)
        dpg.configure_item(
            "status_text",
            color=(248, 81, 73) if error else (63, 185, 80))

    def export_csv(self):
        fname = f"exports/ars_objects_{datetime.now():%Y%m%d_%H%M%S}.csv"
        with open(fname, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["radar", "id", "class", "dist_long_m", "dist_lat_m",
                        "vrel_long_ms", "vrel_lat_ms", "rcs_dbm2",
                        "prob_exist", "meas_state", "motion"])
            for dec in self.decoders:
                with dec.lock:
                    for o in dec.objects.values():
                        w.writerow([
                            dec.label, o.obj_id,
                            CLASS_NAMES.get(o.obj_class, "?"),
                            f"{o.dist_long:.2f}", f"{o.dist_lat:.2f}",
                            f"{o.vrel_long:.2f}", f"{o.vrel_lat:.2f}",
                            f"{o.rcs:.1f}",
                            PROB_EXIST.get(o.prob_exist, "?"),
                            MEAS_STATE.get(o.meas_state, "?"),
                            DYNPROP.get(o.dyn_prop, "?")])
        self.status(f"Exported → {fname}")

    # ------------------------------------------------------------------ #
    #  UI
    # ------------------------------------------------------------------ #
    def build(self):
        dpg.create_context()
        with dpg.theme() as theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (13, 17, 23))
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (22, 27, 34))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (33, 38, 45))
                dpg.add_theme_color(dpg.mvThemeCol_Header, (33, 38, 45))
                dpg.add_theme_color(dpg.mvThemeCol_Button, (31, 111, 235))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,
                                    (56, 139, 253))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 4)
        dpg.bind_theme(theme)

        with dpg.window(tag="main"):
            with dpg.group(horizontal=True):
                dpg.add_text("ARS_MiviaCar", color=(88, 166, 255))
                dpg.add_text("Continental ARS408-21 dual-radar analyzer"
                             " — 77 GHz", color=(139, 148, 158))
                dpg.add_spacer(width=30)
                dpg.add_text("", tag="status_text", color=(63, 185, 80))
            dpg.add_separator()
            with dpg.tab_bar():
                self._tab_overview()
                self._tab_objects()
                self._tab_clusters()
                self._tab_rcs()
                self._tab_can()
            dpg.add_separator()
            dpg.add_text("", tag="footer", color=(139, 148, 158))

        dpg.create_viewport(title="ARS_MiviaCar — ARS408 Radar Analyzer",
                            width=1320, height=820)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main", True)

    def _tab_overview(self):
        with dpg.tab(label="  Overview  "):
            dpg.add_text("The ARS408-21 operates in the 76–77 GHz band, "
                         "cycle time ≈ 72 ms (expected ≈ 13.9 Hz).",
                         color=(139, 148, 158))
            with dpg.group(horizontal=True):
                for i, dec in enumerate(self.decoders):
                    with dpg.child_window(width=630, height=-1):
                        dpg.add_text(f"● {dec.label} radar "
                                     f"(SensorId {dec.sensor_id})",
                                     tag=f"ov_title_{i}",
                                     color=(88, 166, 255))
                        dpg.add_separator()
                        for field in ("alive", "cycle", "maxdist", "output",
                                      "power", "quality", "extinfo",
                                      "motionrx", "rcsth", "sort", "nvm",
                                      "errors", "counts"):
                            dpg.add_text("", tag=f"ov_{field}_{i}")

    def _tab_objects(self):
        with dpg.tab(label="  Objects  "):
            with dpg.group(horizontal=True):
                for cid, name in CLASS_NAMES.items():
                    if cid == 7:
                        continue
                    dpg.add_checkbox(
                        label=name, default_value=True,
                        callback=lambda s, v, c=cid:
                            self.class_enabled.__setitem__(c, v))
                dpg.add_button(label="Export CSV",
                               callback=lambda: self.export_csv())
            with dpg.group(horizontal=True):
                with dpg.plot(height=500, width=560, tag="obj_plot"):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis,
                                      label="lateral (m)  ← left",
                                      tag="obj_x")
                    with dpg.plot_axis(
                            dpg.mvYAxis,
                            label="rear ←  vehicle  → front (m)",
                            tag="obj_y"):
                        for di, dec in enumerate(self.decoders):
                            for cid, name in CLASS_NAMES.items():
                                lbl = name if di == 0 else f"{name} (R)"
                                s = dpg.add_scatter_series([], [], label=lbl)
                                self.obj_series[(di, cid)] = s
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
                                            dpg.mvPlotStyleVar_MarkerSize,
                                            6, category=dpg.mvThemeCat_Plots)
                                dpg.bind_item_theme(s, th)
                    dpg.set_axis_limits("obj_x", -40, 40)
                    dpg.set_axis_limits("obj_y", -100, 150)
                dpg.set_frame_callback(30, lambda: (
                    dpg.set_axis_limits_auto("obj_x"),
                    dpg.set_axis_limits_auto("obj_y")))
                with dpg.table(tag="obj_table", header_row=True,
                               resizable=True, scrollY=True, height=500,
                               borders_innerH=True, borders_outerH=True):
                    for col in ("Radar", "ID", "Class", "DistX", "DistY",
                                "Vx", "RCS", "ProbExist", "MeasState",
                                "Motion"):
                        dpg.add_table_column(label=col)

    def _tab_clusters(self):
        with dpg.tab(label="  Clusters  "):
            dpg.add_text("Raw cluster echoes (Cluster_1_General 0x701)"
                         " — switch the radar OutputTypeCfg to 'clusters'"
                         " to receive them.", color=(139, 148, 158))
            with dpg.group(horizontal=True):
                with dpg.plot(height=470, width=560):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="lateral (m)",
                                      tag="cl_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="forward (m)",
                                       tag="cl_y"):
                        for dec in self.decoders:
                            s = dpg.add_scatter_series([], [],
                                                       label=dec.label)
                            self.cluster_series.append(s)
                    dpg.set_axis_limits("cl_x", -50, 50)
                    dpg.set_axis_limits("cl_y", -100, 150)
                dpg.set_frame_callback(31, lambda: (
                    dpg.set_axis_limits_auto("cl_x"),
                    dpg.set_axis_limits_auto("cl_y")))
                with dpg.table(tag="cl_table", header_row=True,
                               resizable=True, scrollY=True, height=470,
                               borders_innerH=True, borders_outerH=True):
                    for col in ("Radar", "ID", "DistX", "DistY", "Vx",
                                "RCS", "Motion"):
                        dpg.add_table_column(label=col)

    def _tab_rcs(self):
        with dpg.tab(label="  RCS Analysis  "):
            dpg.add_text("", tag="rcs_stats", color=(139, 148, 158))
            with dpg.group(horizontal=True):
                with dpg.plot(height=480, width=620, tag="rcs_hist_plot"):
                    dpg.add_plot_axis(dpg.mvXAxis, label="RCS (dBm²)",
                                      tag="rcsh_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="objects",
                                       tag="rcsh_y"):
                        self.rcs_bar = dpg.add_bar_series(
                            [], [], label="RCS distribution", weight=2.0)
                    dpg.set_axis_limits("rcsh_x", -40, 40)
                with dpg.plot(height=480, width=620):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="distance (m)",
                                      tag="rcss_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="RCS (dBm²)",
                                       tag="rcss_y"):
                        for di, dec in enumerate(self.decoders):
                            s = dpg.add_scatter_series([], [],
                                                       label=dec.label)
                            self.rcs_scatter[di] = s
                    dpg.set_axis_limits("rcss_x", 0, 200)
                    dpg.set_axis_limits("rcss_y", -40, 40)
                dpg.set_frame_callback(32, lambda: (
                    dpg.set_axis_limits_auto("rcsh_y"),))

    def _tab_can(self):
        with dpg.tab(label="  CAN Raw  "):
            dpg.add_text("", tag="can_load", color=(139, 148, 158))
            with dpg.table(tag="can_table", header_row=True,
                           resizable=True, scrollY=True, height=-1,
                           borders_innerH=True, borders_outerH=True):
                for col in ("CAN ID", "Name", "Hz", "DLC",
                            "Data (hex)", "Count"):
                    dpg.add_table_column(label=col)

    # ------------------------------------------------------------------ #
    def _rebuild_table(self, tag, rows):
        dpg.delete_item(tag, children_only=True, slot=1)
        for row in rows:
            with dpg.table_row(parent=tag):
                for cell, color in row:
                    dpg.add_text(cell, color=color)

    def refresh(self):
        dim = (139, 148, 158)
        green = (63, 185, 80)
        red = (248, 81, 73)
        yellow = (210, 153, 34)

        # ---- overview ---- #
        for i, dec in enumerate(self.decoders):
            st = dec.state
            alive = dec.alive()
            dpg.set_value(f"ov_alive_{i}",
                          "status: ONLINE" if alive else "status: NO DATA")
            dpg.configure_item(f"ov_alive_{i}",
                               color=green if alive else red)
            hz = dec.cycle_hz()
            dpg.set_value(f"ov_cycle_{i}",
                          f"measured cycle: {hz:.1f} Hz"
                          f"  (expected ≈ 13.9 Hz / 72 ms)")
            dpg.configure_item(f"ov_cycle_{i}",
                               color=green if 10 <= hz <= 18 else yellow)
            if st.stamp:
                dpg.set_value(f"ov_maxdist_{i}",
                              f"max distance cfg: {st.max_distance} m")
                dpg.set_value(f"ov_output_{i}",
                              f"output type: "
                              f"{OUTPUT_TYPE.get(st.output_type, '?')}")
                dpg.set_value(f"ov_power_{i}",
                              f"Tx power: "
                              f"{POWER_CFG.get(st.power_cfg, '?')}")
                dpg.set_value(f"ov_quality_{i}",
                              f"send quality info: "
                              f"{'yes' if st.send_quality else 'no'}")
                dpg.set_value(f"ov_extinfo_{i}",
                              f"send extended info: "
                              f"{'yes' if st.send_ext_info else 'no'}")
                dpg.set_value(f"ov_motionrx_{i}",
                              f"motion input: "
                              f"{MOTION_RX.get(st.motion_rx, '?')}")
                dpg.set_value(f"ov_rcsth_{i}",
                              f"RCS threshold: "
                              f"{RCS_THRESH.get(st.rcs_threshold, '?')}")
                dpg.set_value(f"ov_sort_{i}",
                              f"sort index: {st.sort_index}"
                              f"   |   reported SensorId: {st.sensor_id}")
                dpg.set_value(f"ov_nvm_{i}",
                              f"NVM read/write ok: "
                              f"{st.nvm_read}/{st.nvm_write}")
                errs = st.errors()
                dpg.set_value(f"ov_errors_{i}",
                              "errors: " + (", ".join(errs) if errs
                                            else "none"))
                dpg.configure_item(f"ov_errors_{i}",
                                   color=red if errs else green)
            else:
                dpg.set_value(f"ov_maxdist_{i}",
                              "(waiting for RadarState 0x"
                              f"{0x201 + dec.offset:03X}...)")
            with dec.lock:
                n_obj = len(dec.objects)
                n_cl = len(dec.clusters)
            dpg.set_value(f"ov_counts_{i}",
                          f"objects: {n_obj} (radar reports "
                          f"{dec.n_objects_reported})   |   "
                          f"clusters: {n_cl}")

        # ---- objects (vehicle-centric merged view) ---- #
        rows = []
        all_rcs = []
        per_series = {k: ([], []) for k in self.obj_series}
        for di, dec in enumerate(self.decoders):
            rear = (dec.label == "REAR")
            with dec.lock:
                objs = sorted(dec.objects.values(),
                              key=lambda o: o.dist_long)
            for o in objs:
                cid = o.obj_class if o.obj_class is not None else 7
                if not self.class_enabled.get(cid, True):
                    continue
                xs, ys = per_series[(di, cid)]
                if rear:
                    xs.append(o.dist_lat)
                    ys.append(-o.dist_long)
                else:
                    xs.append(-o.dist_lat)
                    ys.append(o.dist_long)
                all_rcs.append((di, o.dist_long, o.rcs))
                rgba = CLASS_RGBA.get(cid, (139, 148, 158, 255))
                rows.append([
                    (dec.label, (88, 166, 255)),
                    (str(o.obj_id), None),
                    (CLASS_NAMES.get(cid, "?"), rgba[:3]),
                    (f"{o.dist_long:.1f}", None),
                    (f"{o.dist_lat:.1f}", None),
                    (f"{o.vrel_long:.2f}", None),
                    (f"{o.rcs:.1f}", None),
                    (PROB_EXIST.get(o.prob_exist, "—"), dim),
                    (MEAS_STATE.get(o.meas_state, "—"), dim),
                    (DYNPROP.get(o.dyn_prop, "?"), dim),
                ])
        for k, (xs, ys) in per_series.items():
            dpg.set_value(self.obj_series[k], [xs, ys])
        self._rebuild_table("obj_table", rows)

        # ---- clusters ---- #
        rows = []
        for di, dec in enumerate(self.decoders):
            rear = (dec.label == "REAR")
            with dec.lock:
                cls = sorted(dec.clusters.values(),
                             key=lambda c: c.dist_long)
            xs, ys = [], []
            for c in cls:
                if rear:
                    xs.append(c.dist_lat)
                    ys.append(-c.dist_long)
                else:
                    xs.append(-c.dist_lat)
                    ys.append(c.dist_long)
                rows.append([
                    (dec.label, (88, 166, 255)),
                    (str(c.cid), None),
                    (f"{c.dist_long:.1f}", None),
                    (f"{c.dist_lat:.1f}", None),
                    (f"{c.vrel_long:.2f}", None),
                    (f"{c.rcs:.1f}", None),
                    (DYNPROP.get(c.dyn_prop, "?"), dim),
                ])
            dpg.set_value(self.cluster_series[di], [xs, ys])
        self._rebuild_table("cl_table", rows)

        # ---- RCS analysis ---- #
        if all_rcs:
            vals = [r for _, _, r in all_rcs]
            lo, hi = min(vals), max(vals)
            mean = sum(vals) / len(vals)
            dpg.set_value("rcs_stats",
                          f"{len(vals)} objects — RCS min {lo:.1f} / "
                          f"mean {mean:.1f} / max {hi:.1f} dBm²   "
                          "(pedestrian ≈ -10..0, car ≈ 0..20, "
                          "truck ≈ 20..40)")
            # histogram, 2 dBm² bins from -40 to 40
            bins = {}
            for v in vals:
                b = max(-40, min(40, int(v // 2) * 2))
                bins[b] = bins.get(b, 0) + 1
            xs = sorted(bins)
            dpg.set_value(self.rcs_bar,
                          [[x + 1 for x in xs], [bins[x] for x in xs]])
            for di in self.rcs_scatter:
                pts = [(d, r) for dd, d, r in all_rcs if dd == di]
                dpg.set_value(self.rcs_scatter[di],
                              [[p[0] for p in pts], [p[1] for p in pts]])

        # ---- CAN raw ---- #
        total = 0.0
        rows = []
        for can_id in sorted(self.reader.frames):
            st = self.reader.frames[can_id]
            hz = st.hz()
            total += hz
            rows.append([
                (f"0x{can_id:03X}", None),
                (self.reader.frame_name(can_id), (188, 140, 255)),
                (f"{hz:.1f}", green if hz > 0 else dim),
                (str(st.dlc), None),
                (" ".join(f"{b:02X}" for b in st.data), dim),
                (str(st.count), None),
            ])
        self._rebuild_table("can_table", rows)
        dpg.set_value("can_load",
                      f"interface {self.args.can} — "
                      f"{len(self.reader.frames)} IDs, "
                      f"{total:.0f} frames/s"
                      + (f"   |   {self.reader.error}"
                         if self.reader.error else ""))

        # ---- footer ---- #
        up = int(time.monotonic() - self.start)
        dpg.set_value("footer",
                      f"uptime {up // 60:02d}:{up % 60:02d}   |   "
                      + "   |   ".join(
                          f"{d.label}: "
                          f"{'ONLINE' if d.alive() else 'no data'}"
                          for d in self.decoders))

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
        dpg.destroy_context()


def main():
    parser = argparse.ArgumentParser(
        description="ARS_MiviaCar — dual ARS408 radar analyzer")
    parser.add_argument("--can", default="can0", metavar="IFACE",
                        help="CAN interface (default can0)")
    parser.add_argument("--front", type=int, default=0,
                        help="SensorId of the front radar (default 0)")
    parser.add_argument("--rear", type=int, default=1,
                        help="SensorId of the rear radar (default 1)")
    parser.add_argument("--single", type=int, default=None, metavar="ID",
                        help="analyze a single radar with this SensorId")
    args = parser.parse_args()
    App(args).run()


if __name__ == "__main__":
    main()
