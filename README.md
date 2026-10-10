# XGC2 VRPN Relay

One reusable ROS1 product for every robot that talks to a Motive / VRPN
server. Robots do not copy this logic. They only assemble:

- which rigid body to subscribe (`tracker` / `mocap_rigid_body`)
- whether PX4 `vision_pose` is on (`vision_out` set or empty)

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

The ROS package `xgc2_vrpn_relay` has two parts:

- `vrpn_relay`, a roscpp node (this is what the launch files start), and
- `xgc2_vrpn_relay_core`, a ROS-free C++17 library with the logic of that
  node, installed with a CMake config so other products can link it
  instead of writing the same checks again.

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

PX4 only: also publish the vision pose to `vision_out`
(`/mavros/vision_pose/pose` on FS150).

Vision rate (target **30 Hz**, not 50): XGC1 PhyMocap callback drop.

- incoming faster → keep the last five publish times; emit now if the min
  interval elapsed **or** the recent window rate is still below 30 Hz
- incoming slower → publish every accepted frame; no zero-order hold
- never cache + timer: a held last pose keeps feeding PX4 after the tracker
  dies and adds timer latency

### Quality gate

The gate looks at the pose only and always at the raw tracker coordinates,
before any offset. A pose is dropped (and logged, at most every 2 s) when it
has

1. a non-finite value (`nonfinite`),
2. a quaternion with norm below 1e-6 (`bad_quat`; any other quaternion is
   normalized),
3. `(x, y) == (0, 0)` while `reject_origin` is set (`origin`),
4. `|x|` or `|y|` above `max_abs_xy` (`abs_xy`),
5. a 3D distance above `max_jump_m` from the last accepted pose (`jump`),
6. not moved by 1e-4 m since it last moved, for `frozen_s` seconds or more
   (`frozen`).

A value of 0 or less for `max_abs_xy`, `max_jump_m` or `frozen_s` switches
that check off. Twist and accel get only a finite check.

Two properties worth knowing in the field:

- A rejected pose never becomes the new reference. If the tracker
  re-acquires the body more than `max_jump_m` away from the last accepted
  pose, every frame stays rejected as `jump` until the body is back within
  that distance, or the node is restarted.
- "Not moving" is judged from the position where the pose last moved, not
  from the previous frame, so a slow drift is not frozen.

### Offset and alignment

After the gate, `vrpn_relay` applies a world-frame translation to pose
(and optional `vision_out`). Twist and accel are not shifted. Use fixed
`offset_x` / `offset_y` / `offset_z`, or `align_xy:=true` so the first
accepted pose becomes `(align_to_x, align_to_y)` (default `0, 0`);
`z` stays as Motive reports it unless `offset_z` is set. With `align_xy`,
`offset_x` and `offset_y` are not used. There is no yaw alignment. The
alignment is held by the running node: a restart aligns to the new first
frame.

### Time

The header stamp of the incoming pose is passed through; a zero stamp is
replaced by the time the message was received. Frozen detection and the
vision rate use the receipt time of each message, not the time the callback
ran, so a busy node is not mistaken for a frozen tracker. `vrpn.launch` sets
`use_server_time:=false`, so the client stamps messages with the ROS time it
received them. Twist and accel are forwarded unchanged.

### Parameters

All private (`~`) parameters of the node; the launch files pass the ones
marked with a star.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `tracker` ★ | `tracker` | rigid body name, used to build the input topics |
| `vrpn_prefix` | `/vrpn_client_node` | client namespace |
| `pose_in` `twist_in` `accel_in` | `<vrpn_prefix>/<tracker>/pose` … | input topics (`PoseStamped`, `TwistStamped`, `AccelStamped`) |
| `pose_out` `twist_out` `accel_out` ★ | `/pose` `/twist` `/accel` | output topics |
| `vision_out` ★ | empty (off) | PX4 vision pose topic, e.g. `/mavros/vision_pose/pose` |
| `vision_target_hz` ★ | `30.0` | vision pose rate; 0 or less sends every accepted frame |
| `reject_origin` | `true` | quality gate, see above |
| `max_abs_xy` | `900.0` | quality gate [m] |
| `max_jump_m` | `2.0` | quality gate [m] |
| `frozen_s` | `0.5` | quality gate [s] |
| `offset_x` `offset_y` `offset_z` ★ | `0.0` | fixed translation [m] |
| `align_xy` ★ | `false` | first-frame XY alignment |
| `align_to_x` `align_to_y` ★ | `0.0` | alignment target [m] |

A parameter of the wrong type, or a number that is not finite, stops the
node at start with an error.

Known limit: the accel input is `geometry_msgs/AccelStamped`, but the Noetic
`vrpn_client_ros` 0.2.2 advertises its `accel` topic as
`geometry_msgs/TwistStamped` (`rostopic type` shows it). The client then
refuses the relay's accel subscription as a type mismatch, and `/accel`
stays silent. Pose and twist are not affected.

### Lifecycle

- **Start.** The node reads its parameters once, advertises its outputs, then
  subscribes to the inputs. A bad parameter is a fatal error: the node logs it
  and exits with status 1, so a service that restarts it on failure repeats
  until the configuration is fixed.
- **Source lost.** With no input there is no output. Nothing is held or
  repeated and the node has no timers; the outputs resume with the first
  accepted frame.
- **Stop.** SIGINT (what roslaunch sends on shutdown) ends the spin and
  releases the publishers and subscriptions. The node holds no device and no
  file.
- **Restart.** All state lives in the process. After a restart the quality
  history is empty, `jump` is measured from the new first accepted pose, and
  `align_xy` aligns to the new first frame.
- **Ownership.** The robot assembly (the launch file around `mocap.launch`)
  picks the tracker, the offset and the vision output, and supervises the
  process; on FS150 that is `xgc2-fs150-mocap.service`. One relay owns an
  output topic: two relays on `/mavros/vision_pose/pose` would interleave.

## Install

```bash
sudo apt update
sudo apt install ros-noetic-xgc2-vrpn-relay    # focal
sudo apt install ros-melodic-xgc2-vrpn-relay   # bionic
```

The Debian packages are compiled per ROS distro and replace the former
`Architecture: all` Python package `xgc2-vrpn-relay`. Runtime still needs
`ros-${distro}-vrpn-client-ros` on the vehicle (a recommended package).

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

## Core library

`xgc2_vrpn_relay_core` is a static library without any ROS dependency. It
builds with g++ 7.5 (Ubuntu 18.04) and newer. Link it from another product:

```cmake
find_package(xgc2_vrpn_relay_core CONFIG REQUIRED)
target_link_libraries(my_target PRIVATE xgc2_vrpn_relay_core::core)
```

```cpp
#include <xgc2_vrpn_relay/pose_relay.hpp>
#include <xgc2_vrpn_relay/vision_cadence.hpp>

namespace relay = xgc2_vrpn_relay;

relay::PoseRelayConfig config;           // defaults: the node's defaults
config.offset.offset = {1.0, -2.0, 0.0}; // or config.offset.align_xy = true
relay::PoseRelay pose_relay(config);
relay::VisionPublishCadence cadence(30.0);

// per pose, with its receipt time in seconds:
const relay::PoseRelayResult result = pose_relay.process(raw_pose, receipt_s);
if (result.accepted()) {
    publish(result.pose);                 // unit quaternion, offset applied
    if (cadence.take(receipt_s)) publish_vision(result.pose);
} else {
    log(relay::toString(result.rejection));
}
// twists and accelerations: relay::isFinite(motion)
```

| Header | Contents |
| --- | --- |
| `types.hpp` | `Vec3`, `Quaternion`, `Pose`, `Motion`; `isFinite`, `distance`, `norm`, `normalized` |
| `quality.hpp` | `PoseQualityConfig`, `PoseRejection`, `evaluatePose` (pure) and `PoseQualityGate` (keeps the history) |
| `offset.hpp` | `OffsetConfig`, `translated`, `alignmentOffset`, `WorldOffset` |
| `pose_relay.hpp` | `PoseRelay`: gate then offset, the pose path of the node |
| `vision_cadence.hpp` | `VisionPublishCadence` |

## Build and test

With ROS installed:

```bash
.xgc2/scripts/run_source_tests.sh --ros-distro noetic   # core alone, gtest, rostest
.xgc2/scripts/check_cpp_quality.sh                      # clang-format, -Werror, clang-tidy
.xgc2/scripts/build_debs.sh --ros-distro noetic         # tests, install, deb, deb check
```

The core also builds and tests without ROS:

```bash
cmake -S xgc2_vrpn_relay/core -B build/core -DXGC2_VRPN_RELAY_CORE_BUILD_TESTING=ON
cmake --build build/core && build/core/xgc2_vrpn_relay_core_test
```

The rostest starts its own roscore on a private port; it never uses 11311.

## CI

Product CI runs inside `ghcr.io/xgc-team/xgc2-images/xgc2-build-*`.
It does not `apt-get` or `pip install`. Extra packages belong in
`xgc2-images` first. `build_debs_in_docker.sh` builds, tests and checks
the deb for one ROS distro in its image (bionic/melodic, focal/noetic).

## License

The repository carries no open-source license; `package.xml` declares
`Proprietary`.
