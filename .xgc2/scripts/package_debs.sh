#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

PACKAGE="xgc2-vrpn-relay"
ARCHITECTURE="${ARCHITECTURE:-all}"
OUTPUT_DIR=""
ROS_PREFIXES=(/opt/ros/melodic /opt/ros/noetic)

usage() {
  cat <<EOF
Usage: ${0##*/} --output-dir DIR

Build the ${PACKAGE} Debian package (Architecture: all).
EOF
}

product_version() {
  awk -F': *' '/^version:[[:space:]]*/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"
VERSION="${VERSION:-0.1.0-1}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ -z "${OUTPUT_DIR}" ]]; then
  echo "--output-dir is required" >&2
  usage >&2
  exit 2
fi

if [[ -z "${VERSION}" ]]; then
  echo "package version is missing" >&2
  exit 1
fi

BUILD_DIR="$(mktemp -d)"
cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}/${PACKAGE}_"*.deb

pkg_root="${BUILD_DIR}/${PACKAGE}"
mkdir -p \
  "${pkg_root}/DEBIAN" \
  "${pkg_root}/usr/share/doc/${PACKAGE}"

src="${REPO_ROOT}/xgc2_vrpn_relay"
for prefix in "${ROS_PREFIXES[@]}"; do
  share="${pkg_root}${prefix}/share/xgc2_vrpn_relay"
  lib="${pkg_root}${prefix}/lib/xgc2_vrpn_relay"
  mkdir -p "${share}/launch" "${lib}/xgc2_vrpn_relay"
  install -m 0644 "${src}/package.xml" "${share}/package.xml"
  install -m 0644 "${src}/launch/vrpn.launch" "${share}/launch/vrpn.launch"
  install -m 0644 "${src}/launch/mocap.launch" "${share}/launch/mocap.launch"
  install -m 0755 "${src}/scripts/vrpn_relay" "${lib}/vrpn_relay"
  install -m 0644 \
    "${src}/scripts/xgc2_vrpn_relay/__init__.py" \
    "${src}/scripts/xgc2_vrpn_relay/quality.py" \
    "${src}/scripts/xgc2_vrpn_relay/rate.py" \
    "${lib}/xgc2_vrpn_relay/"
done

cat > "${pkg_root}/usr/share/doc/${PACKAGE}/README" <<EOF
${PACKAGE}

Shared ROS1 VRPN client wrapper and pose/twist/accel/vision_pose relay.
Robots only assemble tracker name and whether vision_pose is on.

Installed ROS package:
  xgc2_vrpn_relay

Prefixes:
  /opt/ros/melodic
  /opt/ros/noetic
EOF
chmod 0644 "${pkg_root}/usr/share/doc/${PACKAGE}/README"
install -m 0644 "${REPO_ROOT}/LICENSE" "${pkg_root}/usr/share/doc/${PACKAGE}/copyright"

installed_size="$(du -sk "${pkg_root}" | awk '{print $1}')"
cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCHITECTURE}
Installed-Size: ${installed_size}
Maintainer: XGC2 <apt@example.com>
Depends: python3
Recommends: ros-melodic-vrpn-client-ros | ros-noetic-vrpn-client-ros, ros-melodic-geometry-msgs | ros-noetic-geometry-msgs, ros-melodic-rospy | ros-noetic-rospy
Description: XGC2 shared VRPN relay
 Reusable ROS1 VRPN client wrapper and pose/twist/accel/vision_pose
 relay. Robots pass tracker and vision_out; they do not copy this node.
EOF
chmod 0644 "${pkg_root}/DEBIAN/control"

find "${pkg_root}" -type d -exec chmod 0755 {} +
chmod 0755 "${pkg_root}/DEBIAN"
for prefix in "${ROS_PREFIXES[@]}"; do
  chmod 0755 "${pkg_root}${prefix}/lib/xgc2_vrpn_relay/vrpn_relay"
done

fakeroot dpkg-deb --build \
  "${pkg_root}" \
  "${OUTPUT_DIR}/${PACKAGE}_${VERSION}_all.deb" >/dev/null

find "${OUTPUT_DIR}" -maxdepth 1 -type f -name '*.deb' -print | sort
