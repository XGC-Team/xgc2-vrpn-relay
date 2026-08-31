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
xgc2_vrpn_relay           robot-local quality + pose/twist/accel relay
experiment_projection     four domain parameters → canonical topics
                          + optional bounded vision_pose output
        ↓
robot / Experiment        source_root, mocap_rigid_body,
                          robot_namespace, publish_vision
```

`xgc2-vrpn-router` is the protocol router. This repository is the ROS
adapter, the same split as `xgc2-camera-core` / `xgc2-camera-driver`.

## What it does

`vrpn.launch` starts official `vrpn_client_ros` against **one** tracker.
`refresh_tracker_frequency` is 0, so the client does not ingest every
rigid body on the server.

`mocap.launch` starts that client plus `vrpn_relay`.

`experiment_projection.launch` consumes one already-selected rigid body
for one Experiment slot. Callers pass four domain parameters only:

- `source_root` — absolute ROS client root (`/vrpn_client_node` or
  `/vrpn_client_node_simulation`)
- `mocap_rigid_body` — one legal tracker (`^[A-Za-z][A-Za-z0-9_]*$`)
- `robot_namespace` — absolute slot namespace (`/uav1`, `/ugv1`)
- `publish_vision` — bool; PX4 only

The node uniquely derives `source_root/body/{pose,twist,accel}` inputs
and `namespace/{pose,twist,accel}` outputs. When `publish_vision` is true
it also publishes `namespace/mavros/vision_pose/pose`. It does not accept
`pose_in` / `pose_out` (or the other four explicit topics) and launch has
no aliases for those names.

It adds the Experiment's explicit XYZ translation to pose, passes twist
and accel unchanged, preserves the source header and timestamp, and bounds
vision output to at most 30 Hz. It does not choose a source, align an
origin, interpolate, repeat samples, or fit clocks.

Every robot gets pose / twist / accel one-for-one after a quality gate.

PX4 only: also publish `/mavros/vision_pose/pose` under the slot
namespace.

Vision rate (typical target 30 Hz):

- incoming faster → decide per frame, publish immediately or drop
- incoming slower → publish every accepted frame; no zero-order hold

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

Experiment projection, physical client, PX4 slot:

```bash
roslaunch xgc2_vrpn_relay experiment_projection.launch \
  source_root:=/vrpn_client_node \
  mocap_rigid_body:=FS150_01 \
  robot_namespace:=/uav1 \
  publish_vision:=true \
  offset_x:=0.0 offset_y:=0.0 offset_z:=0.0
```

Experiment projection, simulated client, UGV slot:

```bash
roslaunch xgc2_vrpn_relay experiment_projection.launch \
  source_root:=/vrpn_client_node_simulation \
  mocap_rigid_body:=Scout1 \
  robot_namespace:=/ugv1 \
  publish_vision:=false
```

A robot repository only wraps those arguments. It does not ship
another copy of `vrpn_relay`.

Tracker names follow `^[A-Za-z][A-Za-z0-9_]*$`. Use `FS150_01` /
`Scout1`, not a generic `uav1` next to a Scout.

## CI

Product CI runs inside `ghcr.io/xgc-team/xgc2-images/xgc2-build-*`.
It does not `apt-get` or `pip install`. Extra packages belong in
`xgc2-images` first.
