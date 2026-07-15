# MIRA — Monitoring Interface for ROS2 Applications

A lightweight, **single-file terminal dashboard** for ROS2 + CAN bus.
No GUI, no X11 needed for monitoring — runs in any terminal, in Docker, or over SSH.

![status](https://img.shields.io/badge/ROS2-Humble%20%7C%20Jazzy-blue)
![python](https://img.shields.io/badge/python-3.8%2B-green)
![license](https://img.shields.io/badge/license-MIT-lightgrey)

## Views (keyboard navigation)

| Key | View | Content |
|-----|------|---------|
| `1` | **ROS2 Topics** | rate (Hz), bandwidth, message count, type, health OK/FAIL |
| `2` | **CAN Frames** | live frames on `can0`/`vcan0`: ID, rate, DLC, data hex, message name |
| `3` | **Radar Objects** | ARS408 detected objects: **class** (car, truck, pedestrian, bicycle…), distance, relative speed, RCS, motion state |
| `r` | **Launch RViz2** | spawns `rviz2` for 3D visualization (requires display/X11) |
| `q` | Quit | |

## Requirements

- ROS 2 Humble or Jazzy, **sourced** (`rclpy` comes from ROS, not pip)
- Python packages: `rich` (plus `pyyaml` for health rules, `cantools` for DBC decoding)
- Linux SocketCAN for the CAN views (no `python-can` needed — raw sockets)

```bash
source /opt/ros/humble/setup.bash
pip install rich pyyaml
```

## Usage

```bash
python3 mira.py                              # ROS topics view only
python3 mira.py --can can0                   # + CAN frames & radar objects views
python3 mira.py --can vcan0 --sensor-id 1    # dual-radar setups (CAN ID offset)
python3 mira.py --dbc ARS408.dbc             # decode CAN names with your DBC
python3 mira.py --filter radar --rules mira_rules.yaml
```

## ARS408 radar support

Object CAN IDs follow the Continental convention:

```
MsgId = BASE + SensorId × 0x10
```

| Base ID | Message | Used for |
|---------|---------|----------|
| `0x60A` | Object_0_Status | object count + stale-object purge |
| `0x60B` | Object_1_General | distance (long/lat), relative velocity, RCS, dynamic property |
| `0x60D` | Object_3_Extended | **object class** |

Built-in Motorola bit-level decoding is included (no DBC required). Object
classes: point, car, truck, pedestrian, motorcycle, bicycle, wide.
Pass `--dbc your_file.dbc` (with `pip install cantools`) for full DBC-based
message naming.

## Health rules

Create a `mira_rules.yaml`:

```yaml
topics:
  "/radar/pointcloud": {min_hz: 10, max_hz: 25}
  "/lidar/points":     {min_hz: 8}
```

Topics outside their bounds show **FAIL** in red.

## How it works

- ROS topics are subscribed with `raw=True`: MIRA receives the serialized bytes
  of each message, giving exact bandwidth measurement with no deserialization
  overhead — lightweight even with heavy `PointCloud2` streams.
- CAN frames are read directly from **Linux SocketCAN raw sockets**
  (`AF_CAN`/`CAN_RAW`), so no extra CAN library is required.
- Rates are computed over a 5-second rolling window.

## Author

Abdelmoutalib Douadi — Erasmus+ research intern, MIVIA Lab, UNISA (2026)

## License

MIT
