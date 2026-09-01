# XGC2 VRPN Relay

One reusable ROS1 product for every robot that talks to a Motive / VRPN
server. Robots do not copy this logic. They only assemble:

- which rigid body to subscribe (`tracker` / `mocap_rigid_body`)
- whether PX4 `vision_pose` is on (`publish_vision`)

```text
xgc2-vrpn-router          protocol VRPN (ROS-independent)
        ↓
vrpn_client_ros           official client, one tracker
        ↓
xgc2_vrpn_relay           explicit onboard quality + pose/twist/accel relay
        ↓
robot onboard assembly    tracker and optional vision output
```

`xgc2-vrpn-router` is the protocol router. This repository is the ROS
adapter, the same split as `xgc2-camera-core` / `xgc2-camera-driver`.

## What it does

`vrpn.launch` starts official `vrpn_client_ros` against **one** tracker.
`refresh_tracker_frequency` is 0, so the client does not ingest every
rigid body on the server.

`mocap.launch` starts that client plus `vrpn_relay`.

Ground-station Experiment localization does not use this package. FS150,
Scout, and Mecanum Robot Adapters own raw VRPN selection, the Experiment XYZ
offset, canonical slot topics, and (for FS150) the 30 Hz vision output. There
is no standalone `experiment_projection` compatibility executable.

Every robot gets pose / twist / accel one-for-one after a quality gate.

PX4 only: also publish `/mavros/vision_pose/pose` under the slot
namespace.

Vision rate (target **30 Hz**, not 50): XGC1 PhyMocap callback drop.

- incoming faster → keep the last five publish times; emit now if the min
  interval elapsed **or** the recent window rate is still below 30 Hz
- incoming slower → publish every accepted frame; no zero-order hold
- never cache + timer: a held last pose keeps feeding PX4 after the tracker
  dies and adds timer latency

Quality rejects non-finite values, a degenerate quaternion, `(0,0)`
origin, `|xy| > 900`, a jump larger than 2 m, and a frozen tracker.

After the gate, `vrpn_relay` applies a world-frame translation to pose
(and optional `vision_out`). Twist and accel are not shifted. Use fixed
`offset_x` / `offset_y` / `offset_z`, or `align_xy:=true` so the first
accepted pose becomes `(align_to_x, align_to_y)` (default `0, 0`);
`z` stays as Motive reports it unless `offset_z` is set.

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

A robot repository only wraps those arguments. It does not ship
another copy of `vrpn_relay`.

Tracker names follow `^[A-Za-z][A-Za-z0-9_]*$`. Use `FS150_01` /
`Scout1`, not a generic `uav1` next to a Scout.

## CI

Product CI runs inside `ghcr.io/xgc-team/xgc2-images/xgc2-build-*`.
It does not `apt-get` or `pip install`. Extra packages belong in
`xgc2-images` first.
