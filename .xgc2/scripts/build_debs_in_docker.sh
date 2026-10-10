#!/usr/bin/env bash
# Test and package the relay for one ROS distro inside its XGC2 build image, then install the deb
# there and verify it with dpkg and ROS. The container is offline; extra packages belong in
# xgc2-images.
# shellcheck disable=SC1004,SC2016
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

ROS_DISTRO="${ROS_DISTRO:-noetic}"
DOCKER_IMAGE="${DOCKER_IMAGE:-}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro) ROS_DISTRO="$2"; shift 2 ;;
    --image) DOCKER_IMAGE="$2"; shift 2 ;;
    --work-dir) WORK_DIR="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --skip-install-check) INSTALL_CHECK=false; shift ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

case "${ROS_DISTRO}" in
  melodic)
    DEFAULT_DOCKER_IMAGE="ghcr.io/xgc-team/xgc2-images/xgc2-build-bionic-ros-melodic:1.0.0"
    ;;
  noetic)
    DEFAULT_DOCKER_IMAGE="ghcr.io/xgc-team/xgc2-images/xgc2-build-focal-ros-noetic:1.0.0"
    ;;
  *)
    echo "unsupported ROS distro: ${ROS_DISTRO}" >&2
    exit 1
    ;;
esac
if [[ -z "${DOCKER_IMAGE}" ]]; then
  DOCKER_IMAGE="${DEFAULT_DOCKER_IMAGE}"
fi

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm --network none \
  -e XGC2_BUILD_GID="$(id -g)" \
  -e XGC2_BUILD_UID="$(id -u)" \
  -e ROS_DISTRO="${ROS_DISTRO}" \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -v "${REPO_ROOT}:/workspace/repo:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail
    trap '\''build_status=$?; chown -R "${XGC2_BUILD_UID}:${XGC2_BUILD_GID}" /workspace/work /workspace/out; exit "${build_status}"'\'' EXIT

    /workspace/repo/.xgc2/scripts/build_debs.sh \
      --ros-distro "${ROS_DISTRO}" \
      --work-dir /workspace/work \
      --output-dir /workspace/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      package="ros-${ROS_DISTRO}-xgc2-vrpn-relay"
      dpkg -i /workspace/out/"${package}"_*.deb
      verification="$(dpkg -V "${package}")"
      if [[ -n "${verification}" ]]; then
        echo "dpkg verification failed for ${package}:" >&2
        printf "%s\n" "${verification}" >&2
        exit 1
      fi
      set +u
      source "/opt/ros/${ROS_DISTRO}/setup.bash"
      set -u
      test "$(rospack find xgc2_vrpn_relay)" = "/opt/ros/${ROS_DISTRO}/share/xgc2_vrpn_relay"
      test -x "/opt/ros/${ROS_DISTRO}/lib/xgc2_vrpn_relay/vrpn_relay"
      roslaunch --files xgc2_vrpn_relay mocap.launch >/dev/null
      roslaunch --files xgc2_vrpn_relay vrpn.launch >/dev/null
    fi
  '

echo "Debian package output:"
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "*.deb" -print | sort
