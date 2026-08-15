# XGC2 VRPN Relay

One reusable ROS1 product for every robot that talks to a Motive / VRPN
server. Robots do not copy this logic. They only assemble:

- which rigid body to subscribe (`tracker`)
- whether PX4 `vision_pose` is on (`vision_out`)

```text
xgc2-vrpn-router          protocol VRPN (ROS-independent)
        ↓
vrpn_client_ros           official client, one tracker
        ↓
xgc2_vrpn_relay           quality + local pose/twist/accel
                          + optional /mavros/vision_pose/pose
        ↓
robot mocap launch        tracker name, vision_out on/off
```

`xgc2-vrpn-router` is the protocol router. This repository is the ROS
adapter, the same split as `xgc2-camera-core` / `xgc2-camera-driver`.

## What it does

`vrpn.launch` starts official `vrpn_client_ros` against **one** tracker.
`refresh_tracker_frequency` is 0, so the client does not ingest every
rigid body on the server.

`mocap.launch` starts that client plus `vrpn_relay`.

Every robot gets pose / twist / accel one-for-one after a quality gate.

PX4 only: also publish `/mavros/vision_pose/pose`.

Vision rate (typical target 30 Hz):

- incoming faster → decide per frame, publish immediately or drop
- incoming slower → publish every accepted frame; no zero-order hold

Quality rejects non-finite values, a degenerate quaternion, `(0,0)`
origin, `|xy| > 900`, a jump larger than 2 m, and a frozen tracker.

## Install

```bash
sudo apt update
sudo apt install xgc2-vrpn-relay
```

The Debian package is `Architecture: all`. It drops the ROS package
into `/opt/ros/melodic` and `/opt/ros/noetic`. Runtime still needs
`ros-${distro}-vrpn-client-ros` on the vehicle.

## Run

UGV / Scout (no vision_pose):

```bash
roslaunch xgc2_vrpn_relay mocap.launch tracker:=Scout1 vision_out:=
```

PX4 UAV:

```bash
roslaunch xgc2_vrpn_relay mocap.launch \
  tracker:=FS150_01 \
  vision_out:=/mavros/vision_pose/pose \
  vision_target_hz:=30.0
```

A robot repository only wraps those two arguments. It does not ship
another copy of `vrpn_relay`.

Tracker names follow `^[A-Za-z][A-Za-z0-9_]*$`. Use `FS150_01` /
`Scout1`, not a generic `uav1` next to a Scout.

## CI

Product CI runs inside `ghcr.io/xgc-team/xgc2-images/xgc2-build-*`.
It does not `apt-get` or `pip install`. Extra packages belong in
`xgc2-images` first.
