#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MIRA Web — Graphical interface for MIRA (browser-based)
=======================================================
Serves a live dashboard at http://localhost:8080 — no X11 required.

Views:
  * ROS2 Topics  — rate, bandwidth, count, type
  * CAN Frames   — live frames with ARS408 message names
  * Radar Objects — table + real-time 2D bird's-eye radar plot,
                    objects color-coded by class (car, truck, pedestrian...)
  * RViz2 button — launches rviz2 on the machine running MIRA

Usage:
    source /opt/ros/humble/setup.bash
    python3 mira_web.py --can can0
    # then open http://localhost:8080 in your browser
    # (with `docker run --net host`, the host browser reaches it directly)

Author: Abdelmoutalib Douadi — MIVIA Lab, UNISA (2026)
License: MIT
"""

import argparse
import json
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from mira import (MiraNode, CanMonitor, OBJ_CLASS_NAMES, DYNPROP_NAMES,
                  human_bytes, load_rules)

import rclpy

# --------------------------------------------------------------------------- #
#  Shared state
# --------------------------------------------------------------------------- #
STATE = {"node": None, "can": None, "rules": {}, "start": time.time()}


def snapshot_topics():
    node = STATE["node"]
    rules = STATE["rules"]
    rows = []
    for name in sorted(node.stats):
        st = node.stats[name]
        hz = st.rate_hz()
        rule = rules.get(name)
        health = "-"
        if rule:
            lo, hi = rule.get("min_hz", 0), rule.get("max_hz", float("inf"))
            health = "OK" if lo <= hz <= hi else "FAIL"
        rows.append({
            "topic": name,
            "type": st.type_name.split("/")[-1],
            "hz": round(hz, 1),
            "bw": human_bytes(st.bandwidth_bps()),
            "count": st.count,
            "health": health,
        })
    return rows


def snapshot_can():
    can = STATE["can"]
    if can is None:
        return {"error": "CAN disabled — restart with --can can0", "rows": []}
    if can.error:
        return {"error": can.error, "rows": []}
    rows = []
    for can_id in sorted(can.frames):
        st = can.frames[can_id]
        rows.append({
            "id": f"0x{can_id:03X}",
            "name": can.frame_name(can_id),
            "hz": round(st.rate_hz(), 1),
            "dlc": st.dlc,
            "data": " ".join(f"{b:02X}" for b in st.data),
            "count": st.count,
        })
    return {"error": "", "rows": rows}


def snapshot_objects():
    can = STATE["can"]
    if can is None:
        return {"error": "CAN disabled — restart with --can can0", "rows": []}
    rows = []
    with can.lock:
        objs = sorted(can.objects.values(), key=lambda o: o.dist_long)
        for o in objs:
            cls_name = "?"
            if o.obj_class is not None:
                cls_name = OBJ_CLASS_NAMES.get(o.obj_class, ("?",))[0]
            rows.append({
                "id": o.obj_id,
                "class": cls_name,
                "dist_long": round(o.dist_long, 1),
                "dist_lat": round(o.dist_lat, 1),
                "vrel_long": round(o.vrel_long, 2),
                "vrel_lat": round(o.vrel_lat, 2),
                "rcs": round(o.rcs, 1),
                "motion": DYNPROP_NAMES.get(o.dyn_prop, "?"),
            })
    return {"error": "", "rows": rows}


# --------------------------------------------------------------------------- #
#  HTML page (single-page app, no external CDN needed)
# --------------------------------------------------------------------------- #
PAGE = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<title>MIRA — ROS2 Monitor</title>
<style>
  :root {
    --bg: #0d1117; --panel: #161b22; --border: #30363d;
    --text: #e6edf3; --dim: #8b949e; --accent: #58a6ff;
    --green: #3fb950; --red: #f85149; --yellow: #d29922;
  }
  * { box-sizing: border-box; margin: 0; padding: 0; }
  body { background: var(--bg); color: var(--text);
         font-family: 'Segoe UI', system-ui, sans-serif; }
  header { display: flex; align-items: center; gap: 16px;
           padding: 12px 20px; background: var(--panel);
           border-bottom: 1px solid var(--border); }
  header h1 { font-size: 18px; color: var(--accent); }
  header .sub { color: var(--dim); font-size: 13px; }
  nav { display: flex; gap: 4px; margin-left: auto; }
  nav button { background: transparent; color: var(--dim); border: 1px solid transparent;
               padding: 7px 16px; border-radius: 6px; cursor: pointer; font-size: 14px; }
  nav button.active { background: var(--bg); color: var(--text);
                      border-color: var(--border); }
  nav button.rviz { color: var(--green); border-color: var(--green); }
  nav button:hover { color: var(--text); }
  main { padding: 20px; }
  .panel { background: var(--panel); border: 1px solid var(--border);
           border-radius: 8px; overflow: hidden; }
  table { width: 100%; border-collapse: collapse; font-size: 13px; }
  th { text-align: left; padding: 10px 14px; color: var(--accent);
       border-bottom: 1px solid var(--border); font-weight: 600;
       position: sticky; top: 0; background: var(--panel); }
  td { padding: 8px 14px; border-bottom: 1px solid #21262d; }
  tr:hover td { background: #1c2128; }
  .num { text-align: right; font-variant-numeric: tabular-nums; }
  .ok { color: var(--green); font-weight: 600; }
  .fail { color: var(--red); font-weight: 600; }
  .dim { color: var(--dim); }
  .mono { font-family: 'Cascadia Code', 'Fira Code', monospace; }
  .err { padding: 16px; color: var(--red); }
  .toast { position: fixed; bottom: 20px; right: 20px; background: var(--panel);
           border: 1px solid var(--green); color: var(--green); padding: 10px 18px;
           border-radius: 8px; display: none; }
  #objects-layout { display: grid; grid-template-columns: 1fr 420px; gap: 16px; }
  #radar-wrap { background: var(--panel); border: 1px solid var(--border);
                border-radius: 8px; padding: 12px; }
  #radar-wrap h3 { font-size: 13px; color: var(--dim); margin-bottom: 8px; }
  canvas { width: 100%; background: #090c10; border-radius: 6px; }
  .legend { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 8px; font-size: 12px; }
  .legend span::before { content: '●'; margin-right: 4px; }
  @media (max-width: 1000px) { #objects-layout { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<header>
  <h1>MIRA</h1>
  <span class="sub">Monitoring Interface for ROS2 Applications</span>
  <nav>
    <button id="tab-topics" class="active" onclick="setView('topics')">Topics</button>
    <button id="tab-can" onclick="setView('can')">CAN</button>
    <button id="tab-objects" onclick="setView('objects')">Radar Objects</button>
    <button class="rviz" onclick="launchRviz()">▶ RViz2</button>
  </nav>
</header>
<main>
  <div id="view-topics" class="panel">
    <table><thead><tr>
      <th>Topic</th><th>Type</th><th class="num">Hz</th>
      <th class="num">Bandwidth</th><th class="num">Msgs</th><th>Health</th>
    </tr></thead><tbody id="tb-topics"></tbody></table>
  </div>

  <div id="view-can" class="panel" style="display:none">
    <div id="can-err" class="err" style="display:none"></div>
    <table><thead><tr>
      <th>CAN ID</th><th>Name</th><th class="num">Hz</th>
      <th class="num">DLC</th><th>Data (hex)</th><th class="num">Count</th>
    </tr></thead><tbody id="tb-can"></tbody></table>
  </div>

  <div id="view-objects" style="display:none">
    <div id="objects-layout">
      <div class="panel">
        <div id="obj-err" class="err" style="display:none"></div>
        <table><thead><tr>
          <th>ID</th><th>Class</th><th class="num">DistX (m)</th><th class="num">DistY (m)</th>
          <th class="num">Vx (m/s)</th><th class="num">RCS</th><th>Motion</th>
        </tr></thead><tbody id="tb-objects"></tbody></table>
      </div>
      <div id="radar-wrap">
        <h3>Bird's-eye view — radar at bottom center</h3>
        <canvas id="radar" width="400" height="500"></canvas>
        <div class="legend" id="legend"></div>
      </div>
    </div>
  </div>
</main>
<div class="toast" id="toast"></div>

<script>
const CLASS_COLORS = {
  point: '#e6edf3', car: '#3fb950', truck: '#d29922', pedestrian: '#f85149',
  motorcycle: '#bc8cff', bicycle: '#39c5cf', wide: '#58a6ff',
  reserved: '#8b949e', '?': '#8b949e'
};
document.getElementById('legend').innerHTML = Object.entries(CLASS_COLORS)
  .filter(([k]) => !['reserved','?'].includes(k))
  .map(([k, c]) => `<span style="color:${c}">${k}</span>`).join('');

let view = 'topics';
function setView(v) {
  view = v;
  for (const name of ['topics', 'can', 'objects']) {
    document.getElementById('view-' + name).style.display = name === v ? '' : 'none';
    document.getElementById('tab-' + name).classList.toggle('active', name === v);
  }
}

function toast(msg) {
  const t = document.getElementById('toast');
  t.textContent = msg; t.style.display = 'block';
  setTimeout(() => t.style.display = 'none', 2500);
}

async function launchRviz() {
  const r = await fetch('/api/rviz', {method: 'POST'});
  toast((await r.json()).message);
}

function esc(s) { return String(s).replace(/[&<>]/g,
  c => ({'&':'&amp;','<':'&lt;','>':'&gt;'}[c])); }

async function refresh() {
  try {
    if (view === 'topics') {
      const rows = await (await fetch('/api/topics')).json();
      document.getElementById('tb-topics').innerHTML = rows.map(r => `
        <tr><td class="mono">${esc(r.topic)}</td><td class="dim">${esc(r.type)}</td>
        <td class="num ${r.hz > 0 ? 'ok' : 'dim'}">${r.hz.toFixed(1)}</td>
        <td class="num">${esc(r.bw)}</td><td class="num">${r.count}</td>
        <td class="${r.health === 'OK' ? 'ok' : r.health === 'FAIL' ? 'fail' : 'dim'}">${r.health}</td></tr>`).join('');
    } else if (view === 'can') {
      const d = await (await fetch('/api/can')).json();
      document.getElementById('can-err').style.display = d.error ? '' : 'none';
      document.getElementById('can-err').textContent = d.error;
      document.getElementById('tb-can').innerHTML = d.rows.map(r => `
        <tr><td class="mono">${r.id}</td><td>${esc(r.name)}</td>
        <td class="num ${r.hz > 0 ? 'ok' : 'dim'}">${r.hz.toFixed(1)}</td>
        <td class="num">${r.dlc}</td><td class="mono dim">${r.data}</td>
        <td class="num">${r.count}</td></tr>`).join('');
    } else {
      const d = await (await fetch('/api/objects')).json();
      document.getElementById('obj-err').style.display = d.error ? '' : 'none';
      document.getElementById('obj-err').textContent = d.error;
      document.getElementById('tb-objects').innerHTML = d.rows.map(r => `
        <tr><td>${r.id}</td>
        <td style="color:${CLASS_COLORS[r.class] || '#8b949e'};font-weight:600">${esc(r.class)}</td>
        <td class="num">${r.dist_long.toFixed(1)}</td><td class="num">${r.dist_lat.toFixed(1)}</td>
        <td class="num">${r.vrel_long.toFixed(2)}</td><td class="num">${r.rcs.toFixed(1)}</td>
        <td class="dim">${esc(r.motion)}</td></tr>`).join('');
      drawRadar(d.rows);
    }
  } catch (e) { /* server restarting */ }
}

function drawRadar(objs) {
  const cv = document.getElementById('radar'), ctx = cv.getContext('2d');
  const W = cv.width, H = cv.height;
  const MAX_RANGE = 120;   // meters forward
  const MAX_LAT = 40;      // meters left/right
  ctx.clearRect(0, 0, W, H);
  const ox = W / 2, oy = H - 20;
  const sy = (H - 40) / MAX_RANGE, sx = (W - 20) / (2 * MAX_LAT);

  // range rings + grid
  ctx.strokeStyle = '#21262d'; ctx.fillStyle = '#484f58';
  ctx.font = '10px monospace'; ctx.lineWidth = 1;
  for (let r = 20; r <= MAX_RANGE; r += 20) {
    ctx.beginPath();
    ctx.arc(ox, oy, r * sy, Math.PI, 2 * Math.PI);
    ctx.stroke();
    ctx.fillText(r + 'm', ox + 4, oy - r * sy - 3);
  }
  // center line
  ctx.beginPath(); ctx.moveTo(ox, oy); ctx.lineTo(ox, 20); ctx.stroke();

  // radar position
  ctx.fillStyle = '#58a6ff';
  ctx.beginPath(); ctx.arc(ox, oy, 5, 0, 2 * Math.PI); ctx.fill();
  ctx.fillText('ARS408', ox + 8, oy);

  // objects: x = lateral (left+ → left on screen), y = longitudinal
  for (const o of objs) {
    const px = ox - o.dist_lat * sx;
    const py = oy - o.dist_long * sy;
    if (py < 10 || px < 0 || px > W) continue;
    const color = CLASS_COLORS[o.class] || '#8b949e';
    ctx.fillStyle = color;
    ctx.beginPath(); ctx.arc(px, py, 5, 0, 2 * Math.PI); ctx.fill();
    // velocity vector
    ctx.strokeStyle = color; ctx.lineWidth = 1.5;
    ctx.beginPath(); ctx.moveTo(px, py);
    ctx.lineTo(px - o.vrel_lat * sx * 1.5, py - o.vrel_long * sy * 1.5);
    ctx.stroke();
    ctx.fillStyle = '#8b949e';
    ctx.fillText('#' + o.id, px + 7, py + 3);
  }
}

setInterval(refresh, 500);
refresh();
</script>
</body>
</html>
"""


# --------------------------------------------------------------------------- #
#  HTTP server
# --------------------------------------------------------------------------- #
class Handler(BaseHTTPRequestHandler):
    def _send(self, code: int, body: bytes, ctype: str):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _json(self, obj):
        self._send(200, json.dumps(obj).encode(), "application/json")

    def do_GET(self):
        if self.path == "/" or self.path.startswith("/index"):
            self._send(200, PAGE.encode(), "text/html; charset=utf-8")
        elif self.path == "/api/topics":
            self._json(snapshot_topics())
        elif self.path == "/api/can":
            self._json(snapshot_can())
        elif self.path == "/api/objects":
            self._json(snapshot_objects())
        else:
            self._send(404, b"not found", "text/plain")

    def do_POST(self):
        if self.path == "/api/rviz":
            try:
                subprocess.Popen(["rviz2"], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL, start_new_session=True)
                self._json({"message": "RViz2 launched"})
            except FileNotFoundError:
                self._json({"message": "rviz2 not found on this machine"})
        else:
            self._send(404, b"not found", "text/plain")

    def log_message(self, *args):
        pass  # silence request logging


# --------------------------------------------------------------------------- #
#  Main
# --------------------------------------------------------------------------- #
def main():
    parser = argparse.ArgumentParser(description="MIRA Web — browser GUI for ROS2 + CAN")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--filter", default="", help="only ROS topics containing this string")
    parser.add_argument("--rules", default="", help="YAML file with topic health rules")
    parser.add_argument("--can", default="", metavar="IFACE", help="CAN interface (can0, vcan0)")
    parser.add_argument("--sensor-id", type=int, default=0, help="ARS408 SensorId offset")
    parser.add_argument("--dbc", default="", help="optional DBC file for CAN decoding")
    args = parser.parse_args()

    STATE["rules"] = load_rules(args.rules)
    if args.can:
        STATE["can"] = CanMonitor(args.can, sensor_id=args.sensor_id, dbc_path=args.dbc)
        STATE["can"].start()

    rclpy.init()
    STATE["node"] = MiraNode(topic_filter=args.filter)
    threading.Thread(target=rclpy.spin, args=(STATE["node"],), daemon=True).start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), Handler)
    print(f"[MIRA] Web interface running →  http://localhost:{args.port}")
    print("[MIRA] Ctrl+C to stop")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
