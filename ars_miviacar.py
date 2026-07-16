#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ARS_MiviaCar - Dedicated dual ARS408-21 radar analysis software
===============================================================
Full CAN-level analysis of the Continental ARS408-21 77 GHz radars
mounted on the MiviaCar research vehicle (front + rear).

Tabs
----
  * Overview     - per-radar RadarState: sensor ID, max distance, output
                   type, power cfg, quality/ext-info flags, motion RX,
                   RCS threshold, ERROR FLAGS, measured cycle rate
  * Objects      - vehicle-centric bird's-eye plot (front + rear merged),
                   full object table: class, distance, velocity, RCS,
                   probability of existence, measurement state, motion
  * Clusters     - raw cluster echoes (0x701) table + scatter plot
  * RCS Analysis - RCS histogram + RCS vs distance scatter, statistics
  * CAN Raw      - every radar CAN frame live: ID, name, rate, data hex

Radar physics reference (ARS408-21):
  * operating band 76..77 GHz, cycle time ~72 ms (~ 13.9 Hz)
  * far field: 0.25..250 m, near field: 0.25..70..100 m
  * object classes: point, car, truck, pedestrian, motorcycle,
    bicycle, wide

Usage
-----
    source /opt/ros/humble/setup.bash
    python3 ars_miviacar.py --can can0                # front (id 0) + rear (id 1)
    python3 ars_miviacar.py --can vcan0 --front 0 --rear 1
    python3 ars_miviacar.py --can can0 --single 0     # only one radar

Requires a display (X11) - same setup as RViz2 in Docker.

Author: Abdelmoutalib Douadi - MIVIA Lab, UNISA (2026)
License: MIT
"""

import argparse
import math
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
# =========================================================================== #
#  Color science - spectrum (Jet) mapping for RCS
# =========================================================================== #
def jet(v: float):
    """Map v in [0,1] to the classic Jet spectrum colormap (RGBA)."""
    v = max(0.0, min(1.0, v))
    r = max(0.0, min(1.0, 1.5 - abs(4.0 * v - 3.0)))
    g = max(0.0, min(1.0, 1.5 - abs(4.0 * v - 2.0)))
    b = max(0.0, min(1.0, 1.5 - abs(4.0 * v - 1.0)))
    return (int(r * 255), int(g * 255), int(b * 255), 255)


def rcs_color(rcs: float):
    """RCS -30..+40 dBm2 -> spectrum color."""
    return jet((rcs + 30.0) / 70.0)


# =========================================================================== #
#  Application - professional radar console
# =========================================================================== #
PPI_SIZE = 640
RD_ROWS, RD_COLS = 36, 48          # doppler bins x range bins
RD_VMAX, RD_RMAX = 22.0, 200.0     # m/s, m

BG = (6, 9, 15)
GRID = (26, 38, 52)
GRID_TXT = (86, 108, 132)
ACCENT = (0, 229, 255)
FRONT_FOV = (0, 229, 255)
REAR_FOV = (255, 158, 44)


class App:
    def __init__(self, args):
        self.args = args
        if args.single is not None:
            self.decoders = [Ars408Decoder(args.single, "FRONT")]
        else:
            self.decoders = [Ars408Decoder(args.front, "FRONT"),
                             Ars408Decoder(args.rear, "REAR")]
        self.reader = CanReader(args.can, self.decoders)
        self.reader.start()
        self.start = time.monotonic()
        self.class_enabled = {i: True for i in CLASS_NAMES}
        self.range_m = 100.0
        self.color_mode = "class"          # or "rcs"
        self.show_clusters = False
        self.trails = {}                   # (label, oid) -> deque[(x,y)]
        self.rd_grid = [0.0] * (RD_ROWS * RD_COLS)
        self.rcs_bar = None
        self.rcs_scatter = {}
        self.last_table = 0.0
        os.makedirs("exports", exist_ok=True)

    # ------------------------------------------------------------------ #
    def status(self, msg, error=False):
        dpg.set_value("status_text", msg)
        dpg.configure_item("status_text",
                           color=(248, 81, 73) if error else (0, 229, 255))

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
        self.status(f"Exported -> {fname}")

    # ------------------------------------------------------------------ #
    #  UI
    # ------------------------------------------------------------------ #
    def build(self):
        dpg.create_context()
        with dpg.theme() as theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (8, 12, 18))
                dpg.add_theme_color(dpg.mvThemeCol_ChildBg, (12, 17, 25))
                dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (20, 28, 40))
                dpg.add_theme_color(dpg.mvThemeCol_Header, (20, 28, 40))
                dpg.add_theme_color(dpg.mvThemeCol_Tab, (14, 20, 30))
                dpg.add_theme_color(dpg.mvThemeCol_TabActive, (0, 92, 110))
                dpg.add_theme_color(dpg.mvThemeCol_TabHovered, (0, 120, 140))
                dpg.add_theme_color(dpg.mvThemeCol_Button, (0, 92, 110))
                dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered,
                                    (0, 140, 165))
                dpg.add_theme_color(dpg.mvThemeCol_CheckMark, (0, 229, 255))
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 5)
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 10, 8)
        dpg.bind_theme(theme)

        with dpg.window(tag="main"):
            with dpg.group(horizontal=True):
                dpg.add_text("ARS_MiviaCar", color=ACCENT)
                dpg.add_text("|  Continental ARS408-21  |  76-77 GHz "
                             "-  dual-radar analysis console",
                             color=(120, 140, 160))
                dpg.add_spacer(width=30)
                dpg.add_text("", tag="status_text", color=ACCENT)
            dpg.add_separator()
            with dpg.tab_bar():
                self._tab_scope()
                self._tab_range_doppler()
                self._tab_overview()
                self._tab_rcs()
                self._tab_can()
            dpg.add_separator()
            dpg.add_text("", tag="footer", color=(120, 140, 160))

        dpg.create_viewport(title="ARS_MiviaCar - Radar Analysis Console",
                            width=1360, height=860)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("main", True)

    # ---------------------- PPI radar scope ---------------------------- #
    def _tab_scope(self):
        with dpg.tab(label="  Radar Scope  "):
            with dpg.group(horizontal=True):
                dpg.add_combo(("50", "100", "150", "250"),
                              default_value="100", width=90,
                              label="range (m)",
                              callback=lambda s, v:
                                  setattr(self, "range_m", float(v)))
                dpg.add_combo(("class", "RCS spectrum"),
                              default_value="class", width=140,
                              label="blip color",
                              callback=lambda s, v: setattr(
                                  self, "color_mode",
                                  "rcs" if "RCS" in v else "class"))
                dpg.add_checkbox(label="clusters", default_value=False,
                                 callback=lambda s, v:
                                     setattr(self, "show_clusters", v))
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
                dpg.add_drawlist(width=PPI_SIZE, height=PPI_SIZE, tag="ppi")
                with dpg.table(tag="obj_table", header_row=True,
                               resizable=True, scrollY=True,
                               height=PPI_SIZE,
                               borders_innerH=True, borders_outerH=True):
                    for col in ("Radar", "ID", "Class", "DistX", "DistY",
                                "Vx", "RCS", "ProbExist", "State",
                                "Motion"):
                        dpg.add_table_column(label=col)

    def _draw_ppi(self):
        dpg.delete_item("ppi", children_only=True)
        S = PPI_SIZE
        cx = cy = S / 2
        scale = (S / 2 - 24) / self.range_m
        t = time.monotonic() - self.start

        dpg.draw_rectangle((0, 0), (S, S), fill=BG, color=BG, parent="ppi")

        # range rings + labels
        step = self.range_m / 4
        for k in range(1, 5):
            r = k * step * scale
            dpg.draw_circle((cx, cy), r, color=GRID, thickness=1,
                            parent="ppi")
            dpg.draw_text((cx + 4, cy - r + 2), f"{k * step:.0f} m",
                          color=GRID_TXT, size=12, parent="ppi")
        # radial lines every 30 deg
        for deg in range(0, 360, 30):
            a = math.radians(deg)
            r = 4 * step * scale
            dpg.draw_line((cx, cy),
                          (cx + r * math.sin(a), cy - r * math.cos(a)),
                          color=GRID, thickness=1, parent="ppi")

        # FOV wedges (ARS408: far +/-9 deg to 250 m, near +/-45 deg to 70 m)
        def wedge(sign, half_deg, rng, col, alpha):
            rr = min(rng, self.range_m) * scale
            a = math.radians(half_deg)
            pts = [(cx, cy)]
            for i in range(-10, 11):
                aa = a * i / 10
                pts.append((cx + rr * math.sin(aa),
                            cy - sign * rr * math.cos(aa)))
            dpg.draw_polygon(pts, color=(0, 0, 0, 0),
                             fill=(*col, alpha), parent="ppi")
        for dec in self.decoders:
            sign = 1 if dec.label == "FRONT" else -1
            col = FRONT_FOV if dec.label == "FRONT" else REAR_FOV
            wedge(sign, 45, 70, col, 14)
            wedge(sign, 9, 250, col, 22)

        # rotating sweep with fading trail (one per radar)
        for dec in self.decoders:
            sign = 1 if dec.label == "FRONT" else -1
            col = FRONT_FOV if dec.label == "FRONT" else REAR_FOV
            base = (t * 90.0) % 180.0 - 90.0     # -90..+90 deg
            r = 4 * step * scale
            for i in range(10):
                a = math.radians(base - i * 2.5)
                if a < math.radians(-90) or a > math.radians(90):
                    continue
                alpha = int(120 * (1 - i / 10))
                dpg.draw_line(
                    (cx, cy),
                    (cx + r * math.sin(a), cy - sign * r * math.cos(a)),
                    color=(*col, alpha), thickness=2 if i == 0 else 1,
                    parent="ppi")

        # ego vehicle
        w, h = 13, 26
        dpg.draw_rectangle((cx - w, cy - h), (cx + w, cy + h),
                           color=ACCENT, fill=(13, 22, 32), rounding=6,
                           thickness=2, parent="ppi")
        dpg.draw_line((cx - w + 3, cy - h + 8), (cx + w - 3, cy - h + 8),
                      color=ACCENT, thickness=1, parent="ppi")
        dpg.draw_text((cx - 24, cy + h + 4), "MiviaCar",
                      color=(140, 165, 190), size=13, parent="ppi")
        dpg.draw_text((cx - 14, 6), "FRONT", color=(*FRONT_FOV, 160),
                      size=12, parent="ppi")
        if len(self.decoders) > 1:
            dpg.draw_text((cx - 12, S - 18), "REAR", color=(*REAR_FOV, 160),
                          size=12, parent="ppi")

        # object blips + trails + velocity vectors
        for dec in self.decoders:
            rear = (dec.label == "REAR")
            with dec.lock:
                objs = list(dec.objects.values())
                clusters = list(dec.clusters.values()) \
                    if self.show_clusters else []
            for o in objs:
                cid = o.obj_class if o.obj_class is not None else 7
                if not self.class_enabled.get(cid, True):
                    continue
                if rear:
                    px = cx + o.dist_lat * scale
                    py = cy + o.dist_long * scale
                    vy = o.vrel_long * scale * 1.2
                    vx = o.vrel_lat * scale * 1.2
                else:
                    px = cx - o.dist_lat * scale
                    py = cy - o.dist_long * scale
                    vy = -o.vrel_long * scale * 1.2
                    vx = -o.vrel_lat * scale * 1.2
                if not (0 <= px <= S and 0 <= py <= S):
                    continue
                col = rcs_color(o.rcs) if self.color_mode == "rcs" \
                    else CLASS_RGBA.get(cid, (140, 150, 160, 255))
                # trail
                key = (dec.label, o.obj_id)
                tr = self.trails.setdefault(key, deque(maxlen=14))
                tr.append((px, py))
                for i in range(1, len(tr)):
                    a = int(70 * i / len(tr))
                    dpg.draw_line(tr[i - 1], tr[i],
                                  color=(*col[:3], a), thickness=2,
                                  parent="ppi")
                # glow blip
                dpg.draw_circle((px, py), 9, fill=(*col[:3], 36),
                                color=(0, 0, 0, 0), parent="ppi")
                dpg.draw_circle((px, py), 6, fill=(*col[:3], 110),
                                color=(0, 0, 0, 0), parent="ppi")
                dpg.draw_circle((px, py), 3.2, fill=col,
                                color=col, parent="ppi")
                # velocity vector
                if abs(vx) + abs(vy) > 2:
                    dpg.draw_arrow((px + vx, py + vy), (px, py),
                                   color=(*col[:3], 200), thickness=1,
                                   size=6, parent="ppi")
                dpg.draw_text((px + 8, py - 6), f"{o.obj_id}",
                              color=(150, 170, 190), size=11, parent="ppi")
            for c in clusters:
                if rear:
                    px = cx + c.dist_lat * scale
                    py = cy + c.dist_long * scale
                else:
                    px = cx - c.dist_lat * scale
                    py = cy - c.dist_long * scale
                if 0 <= px <= S and 0 <= py <= S:
                    col = rcs_color(c.rcs)
                    dpg.draw_circle((px, py), 1.6, fill=(*col[:3], 170),
                                    color=(0, 0, 0, 0), parent="ppi")

    # ---------------------- Range-Doppler map -------------------------- #
    def _tab_range_doppler(self):
        with dpg.tab(label="  Range-Doppler  "):
            dpg.add_text("Range-Doppler map - every detection paints the "
                         "cell (range, relative velocity) with its RCS; "
                         "the trace fades like a phosphor display.",
                         color=(120, 140, 160))
            with dpg.group(horizontal=True):
                with dpg.plot(height=560, width=980, tag="rd_plot"):
                    dpg.add_plot_axis(dpg.mvXAxis, label="range (m)",
                                      tag="rd_x")
                    with dpg.plot_axis(dpg.mvYAxis,
                                       label="relative velocity (m/s)",
                                       tag="rd_y"):
                        dpg.add_heat_series(
                            self.rd_grid, RD_ROWS, RD_COLS,
                            scale_min=0.0, scale_max=1.0,
                            bounds_min=(0, -RD_VMAX),
                            bounds_max=(RD_RMAX, RD_VMAX),
                            format="", tag="rd_heat")
                dpg.bind_colormap("rd_plot", dpg.mvPlotColormap_Jet)
                dpg.add_colormap_scale(min_scale=-30, max_scale=40,
                                       height=560, width=90,
                                       label="RCS (dBm2)",
                                       colormap=dpg.mvPlotColormap_Jet)

    def _update_range_doppler(self):
        # phosphor decay
        self.rd_grid = [v * 0.90 for v in self.rd_grid]
        for dec in self.decoders:
            with dec.lock:
                objs = list(dec.objects.values())
            for o in objs:
                r = o.dist_long
                v = o.vrel_long
                if not (0 <= r < RD_RMAX and -RD_VMAX <= v < RD_VMAX):
                    continue
                col = int(r / RD_RMAX * RD_COLS)
                row = int((v + RD_VMAX) / (2 * RD_VMAX) * RD_ROWS)
                row = RD_ROWS - 1 - row      # heat series rows top-down
                idx = row * RD_COLS + col
                val = (o.rcs + 30.0) / 70.0
                self.rd_grid[idx] = max(self.rd_grid[idx],
                                        max(0.05, min(1.0, val)))
        dpg.set_value("rd_heat", [self.rd_grid])

    # ---------------------- Overview / RCS / CAN ----------------------- #
    def _tab_overview(self):
        with dpg.tab(label="  Sensor State  "):
            dpg.add_text("ARS408-21 - 76-77 GHz band  |  cycle ~ 72 ms "
                         "(~ 13.9 Hz)  |  far field 0.25-250 m (+/-9 deg)  |  "
                         "near field to 70 m (+/-45 deg)",
                         color=(120, 140, 160))
            with dpg.group(horizontal=True):
                for i, dec in enumerate(self.decoders):
                    with dpg.child_window(width=650, height=-1):
                        dpg.add_text(f"> {dec.label} radar "
                                     f"(SensorId {dec.sensor_id})",
                                     color=ACCENT)
                        dpg.add_separator()
                        for field in ("alive", "cycle", "maxdist", "output",
                                      "power", "quality", "extinfo",
                                      "motionrx", "rcsth", "sort", "nvm",
                                      "errors", "counts"):
                            dpg.add_text("", tag=f"ov_{field}_{i}")

    def _tab_rcs(self):
        with dpg.tab(label="  RCS Analysis  "):
            dpg.add_text("", tag="rcs_stats", color=(120, 140, 160))
            with dpg.group(horizontal=True):
                with dpg.plot(height=520, width=620, tag="rcs_hist_plot"):
                    dpg.add_plot_axis(dpg.mvXAxis, label="RCS (dBm2)",
                                      tag="rcsh_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="objects",
                                       tag="rcsh_y"):
                        self.rcs_bar = dpg.add_bar_series(
                            [], [], label="RCS distribution", weight=2.0)
                    dpg.set_axis_limits("rcsh_x", -40, 40)
                with dpg.plot(height=520, width=620, tag="rcs_sc_plot"):
                    dpg.add_plot_legend()
                    dpg.add_plot_axis(dpg.mvXAxis, label="distance (m)",
                                      tag="rcss_x")
                    with dpg.plot_axis(dpg.mvYAxis, label="RCS (dBm2)",
                                       tag="rcss_y"):
                        for di, dec in enumerate(self.decoders):
                            s = dpg.add_scatter_series([], [],
                                                       label=dec.label)
                            self.rcs_scatter[di] = s
                    dpg.set_axis_limits("rcss_x", 0, 200)
                    dpg.set_axis_limits("rcss_y", -40, 40)
            dpg.set_frame_callback(32, lambda: (
                dpg.set_axis_limits_auto("rcsh_y"),))
            dpg.add_text("Reference: pedestrian ~ -10..0 dBm2  |  "
                         "bicycle ~ -5..5  |  car ~ 0..20  |  truck ~ 20..40",
                         color=(120, 140, 160))

    def _tab_can(self):
        with dpg.tab(label="  CAN Raw  "):
            dpg.add_text("", tag="can_load", color=(120, 140, 160))
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

    def refresh_slow(self):
        """Tables, overview, RCS, range-doppler - every REFRESH_S."""
        dim = (120, 140, 160)
        green = (63, 220, 130)
        red = (248, 81, 73)
        yellow = (255, 196, 60)

        # overview
        for i, dec in enumerate(self.decoders):
            st = dec.state
            alive = dec.alive()
            dpg.set_value(f"ov_alive_{i}",
                          "status: ONLINE" if alive else "status: NO DATA")
            dpg.configure_item(f"ov_alive_{i}",
                               color=green if alive else red)
            hz = dec.cycle_hz()
            dpg.set_value(f"ov_cycle_{i}",
                          f"measured cycle: {hz:.1f} Hz "
                          f"(expected ~ 13.9 Hz / 72 ms)")
            dpg.configure_item(f"ov_cycle_{i}",
                               color=green if 10 <= hz <= 18 else yellow)
            if st.stamp:
                dpg.set_value(f"ov_maxdist_{i}",
                              f"max distance cfg: {st.max_distance} m")
                dpg.set_value(f"ov_output_{i}", "output type: "
                              + OUTPUT_TYPE.get(st.output_type, "?"))
                dpg.set_value(f"ov_power_{i}", "Tx power: "
                              + POWER_CFG.get(st.power_cfg, "?"))
                dpg.set_value(f"ov_quality_{i}", "send quality info: "
                              + ("yes" if st.send_quality else "no"))
                dpg.set_value(f"ov_extinfo_{i}", "send extended info: "
                              + ("yes" if st.send_ext_info else "no"))
                dpg.set_value(f"ov_motionrx_{i}", "motion input: "
                              + MOTION_RX.get(st.motion_rx, "?"))
                dpg.set_value(f"ov_rcsth_{i}", "RCS threshold: "
                              + RCS_THRESH.get(st.rcs_threshold, "?"))
                dpg.set_value(f"ov_sort_{i}",
                              f"sort index: {st.sort_index}   |   "
                              f"reported SensorId: {st.sensor_id}")
                dpg.set_value(f"ov_nvm_{i}",
                              f"NVM read/write ok: "
                              f"{st.nvm_read}/{st.nvm_write}")
                errs = st.errors()
                dpg.set_value(f"ov_errors_{i}", "errors: "
                              + (", ".join(errs) if errs else "none"))
                dpg.configure_item(f"ov_errors_{i}",
                                   color=red if errs else green)
            else:
                dpg.set_value(
                    f"ov_maxdist_{i}",
                    f"(waiting for RadarState 0x{0x201 + dec.offset:03X}...)")
            with dec.lock:
                n_obj = len(dec.objects)
                n_cl = len(dec.clusters)
            dpg.set_value(f"ov_counts_{i}",
                          f"objects: {n_obj} (radar reports "
                          f"{dec.n_objects_reported})   |   "
                          f"clusters: {n_cl}")

        # object table + RCS data
        rows = []
        all_rcs = []
        for dec in self.decoders:
            with dec.lock:
                objs = sorted(dec.objects.values(),
                              key=lambda o: o.dist_long)
            for o in objs:
                cid = o.obj_class if o.obj_class is not None else 7
                if not self.class_enabled.get(cid, True):
                    continue
                all_rcs.append((0 if dec.label == "FRONT" else 1,
                                o.dist_long, o.rcs))
                rgba = CLASS_RGBA.get(cid, (140, 150, 160, 255))
                rows.append([
                    (dec.label,
                     FRONT_FOV if dec.label == "FRONT" else REAR_FOV),
                    (str(o.obj_id), None),
                    (CLASS_NAMES.get(cid, "?"), rgba[:3]),
                    (f"{o.dist_long:.1f}", None),
                    (f"{o.dist_lat:.1f}", None),
                    (f"{o.vrel_long:.2f}", None),
                    (f"{o.rcs:.1f}", rcs_color(o.rcs)[:3]),
                    (PROB_EXIST.get(o.prob_exist, "-"), dim),
                    (MEAS_STATE.get(o.meas_state, "-"), dim),
                    (DYNPROP.get(o.dyn_prop, "?"), dim),
                ])
        self._rebuild_table("obj_table", rows)

        if all_rcs:
            vals = [r for _, _, r in all_rcs]
            dpg.set_value("rcs_stats",
                          f"{len(vals)} objects - RCS min {min(vals):.1f} "
                          f"/ mean {sum(vals) / len(vals):.1f} "
                          f"/ max {max(vals):.1f} dBm2")
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

        self._update_range_doppler()

        # CAN raw
        total = 0.0
        rows = []
        for can_id in sorted(self.reader.frames):
            st = self.reader.frames[can_id]
            hz = st.hz()
            total += hz
            rows.append([
                (f"0x{can_id:03X}", None),
                (self.reader.frame_name(can_id), (140, 190, 255)),
                (f"{hz:.1f}", green if hz > 0 else dim),
                (str(st.dlc), None),
                (" ".join(f"{b:02X}" for b in st.data), dim),
                (str(st.count), None),
            ])
        self._rebuild_table("can_table", rows)
        dpg.set_value("can_load",
                      f"interface {self.args.can} - "
                      f"{len(self.reader.frames)} IDs, "
                      f"{total:.0f} frames/s"
                      + (f"   |   {self.reader.error}"
                         if self.reader.error else ""))

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
        last_slow = 0.0
        last_ppi = 0.0
        while dpg.is_dearpygui_running():
            now = time.monotonic()
            if now - last_ppi >= 0.05:          # PPI at ~20 fps
                try:
                    self._draw_ppi()
                except Exception as e:
                    dpg.set_value("status_text", f"PPI error: {e}")
                last_ppi = now
            if now - last_slow >= REFRESH_S:    # tables at 2 Hz
                try:
                    self.refresh_slow()
                except Exception as e:
                    dpg.set_value("status_text", f"UI error: {e}")
                last_slow = now
            dpg.render_dearpygui_frame()
        dpg.destroy_context()


def main():
    parser = argparse.ArgumentParser(
        description="ARS_MiviaCar - dual ARS408 radar analysis console")
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
