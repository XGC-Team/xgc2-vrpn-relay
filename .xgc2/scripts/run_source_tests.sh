#!/usr/bin/env bash
# Build the source tree and run every test: the ROS-free core on its own (it must build and pass
# without ROS or catkin), then the catkin package with its unit tests and the rostest of the node.
# Run it with a ROS distro installed (ROS_DISTRO set or /opt/ros/<distro>/setup.bash given by
# --ros-distro). The rostest starts its own roscore on a private port, never on 11311.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/source-tests}"
ROS_DISTRO="${ROS_DISTRO:-}"
JOBS="${JOBS:-$(nproc)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --jobs)
      JOBS="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${ROS_DISTRO}" || ! -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  echo "set ROS_DISTRO or --ros-distro to an installed ROS distro (melodic or noetic)" >&2
  exit 1
fi

random_free_port() {
  local port
  for _ in $(seq 1 50); do
    port=$((20000 + RANDOM % 20000))
    if python3 - "${port}" <<'PY'
import socket
import sys

with socket.socket() as probe:
    try:
        probe.bind(("127.0.0.1", int(sys.argv[1])))
    except OSError:
        sys.exit(1)
PY
    then
      echo "${port}"
      return 0
    fi
  done
  echo "no free port for the test roscore" >&2
  return 1
}

# The directory is used after cd and in the environment of the tests: make it absolute.
mkdir -p "${WORK_DIR}"
WORK_DIR="$(cd "${WORK_DIR}" && pwd)"
rm -rf "${WORK_DIR}/core-build" "${WORK_DIR}/src" "${WORK_DIR}/build" "${WORK_DIR}/devel" \
  "${WORK_DIR}/install" "${WORK_DIR}/ros_home"
mkdir -p "${WORK_DIR}/ros_home" "${WORK_DIR}/src"

# 1. The core alone: no catkin, and nothing in the build knows where ROS is.
core_build="${WORK_DIR}/core-build"
mkdir -p "${core_build}"
(
  cd "${core_build}"
  cmake "${REPO_ROOT}/xgc2_vrpn_relay/core" \
    -DCMAKE_BUILD_TYPE=Release -DXGC2_VRPN_RELAY_WERROR=ON >/dev/null
  cmake --build . -- -j"${JOBS}"
)

# 2. The catkin package: gtest unit tests of the core and the rostest of the node.
rsync -a --delete --exclude .git --exclude .work --exclude debs --exclude .ci \
  "${REPO_ROOT}/" "${WORK_DIR}/src/xgc2-vrpn-relay/"
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
test_port="$(random_free_port)"
export ROS_MASTER_URI="http://127.0.0.1:${test_port}"
export ROS_IP=127.0.0.1
export ROS_HOME="${WORK_DIR}/ros_home"
cd "${WORK_DIR}"
catkin_make -j"${JOBS}" run_tests_xgc2_vrpn_relay \
  -DCMAKE_BUILD_TYPE=Release \
  -DXGC2_VRPN_RELAY_WERROR=ON \
  -DCATKIN_ENABLE_TESTING=ON
catkin_test_results build/test_results

# mocap.launch hands jump_reacquire_s to the node, with the node's default as its default.
set +u
# shellcheck disable=SC1091
source devel/setup.bash
set -u
launch_params="$(roslaunch --dump-params xgc2_vrpn_relay mocap.launch)"
grep -Eq '^/vrpn_relay/jump_reacquire_s: 1\.0$' <<<"${launch_params}"
launch_params="$(roslaunch --dump-params xgc2_vrpn_relay mocap.launch jump_reacquire_s:=0)"
grep -Eq '^/vrpn_relay/jump_reacquire_s: 0$' <<<"${launch_params}"

echo "Source tests passed"
