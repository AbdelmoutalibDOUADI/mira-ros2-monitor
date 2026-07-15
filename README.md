# MIRA — Monitoring Interface for ROS2 Applications

A lightweight, **single-file terminal dashboard** for ROS2 (Humble / Jazzy).
No GUI, no X11, no web server — runs in any terminal, in Docker, or over SSH.

![status](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue)
![python](https://img.shields.io/badge/python-3.8%2B-green)
![license](https://img.shields.io/badge/license-MIT-lightgrey)

## Features

- **Live topic table** — rate (Hz), bandwidth (B/s), message count, message type
- **Automatic discovery** — new topics appear as soon as they are published
- **Health watchdog** — optional YAML rules (`min_hz` / `max_hz` per topic) with OK/FAIL status
- **Filtering** — monitor only the topics you care about (`--filter radar`)
- **Zero display dependency** — pure terminal UI (works where RViz2 cannot)

## Requirements

- ROS 2 Humble or Jazzy, **sourced** (`rclpy` comes from ROS, not pip)
- Python packages: `rich` (and `pyyaml` if you use health rules)

```bash
source /opt/ros/humble/setup.bash
pip install rich pyyaml
```

## Usage

```bash
python3 mira.py                          # monitor everything
python3 mira.py --filter radar           # only topics containing "radar"
python3 mira.py --rules mira_rules.yaml  # enable health checks
python3 mira.py --refresh 0.5            # faster UI refresh
```

Quit with `Ctrl+C`.

## Health rules

Create a `mira_rules.yaml`:

```yaml
topics:
  "/radar/pointcloud": {min_hz: 10, max_hz: 25}
  "/lidar/points":     {min_hz: 8}
```

Topics outside their bounds show **FAIL** in red; the footer keeps a running count.

## How it works

MIRA subscribes to every discovered topic with `raw=True`, so it receives the
**serialized bytes** of each message. This gives exact bandwidth measurement and
avoids deserialization overhead — the tool stays lightweight even with heavy
`PointCloud2` streams (radar/lidar). Rates are computed over a 5-second rolling
window.

## Author

Abdelmoutalib Douadi — Erasmus+ research intern, MIVIA Lab, UNISA (2026)

## License

MIT
