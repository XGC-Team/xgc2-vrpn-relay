#!/usr/bin/env bash
# Package an installed catkin tree as ros-<distro>-xgc2-vrpn-relay for one ROS distro.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="${ROS_DISTRO:-noetic}"
ROS_PACKAGE="xgc2_vrpn_relay"
# Before 0.2.0 the relay was one Architecture: all Python package for every distro.
LEGACY_PACKAGE="xgc2-vrpn-relay"
LEGACY_BOUNDARY="0.2.0"

product_version() {
  # mawk (bionic) does not implement POSIX [[:space:]].
  awk '/^version:/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml"
}

VERSION="${PACKAGE_VERSION:-$(product_version)}"
if [[ -z "${VERSION}" ]]; then
  echo "package version is missing; set PACKAGE_VERSION or .xgc2/product.yml version" >&2
  exit 1
fi

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo "--install-root and --output-dir are required" >&2
  exit 1
fi

case "${ROS_DISTRO}" in
  melodic|noetic) ;;
  *)
    echo "unsupported ROS distro: ${ROS_DISTRO}" >&2
    exit 1
    ;;
esac

ARCH="$(dpkg --print-architecture)"
PACKAGE="ros-${ROS_DISTRO}-xgc2-vrpn-relay"
PREFIX="/opt/ros/${ROS_DISTRO}"
PREFIX_ROOT="${INSTALL_ROOT}${PREFIX}"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

mkdir -p "${OUTPUT_DIR}"
rm -f "${OUTPUT_DIR}/${PACKAGE}_"*.deb

pkg_root="${BUILD_DIR}/${PACKAGE}"
mkdir -p "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${PACKAGE}"

copy_path() {
  local src="$1"
  if [[ ! -e "${src}" ]]; then
    echo "missing installed path: ${src#"${INSTALL_ROOT}"}" >&2
    exit 1
  fi
  mkdir -p "${pkg_root}$(dirname "${src#"${INSTALL_ROOT}"}")"
  cp -a "${src}" "${pkg_root}${src#"${INSTALL_ROOT}"}"
}

# The node, its launch files, and the linkable core: library, headers and CMake config.
copy_path "${PREFIX_ROOT}/share/${ROS_PACKAGE}"
copy_path "${PREFIX_ROOT}/lib/${ROS_PACKAGE}"
copy_path "${PREFIX_ROOT}/lib/libxgc2_vrpn_relay_core.a"
copy_path "${PREFIX_ROOT}/include/${ROS_PACKAGE}"
copy_path "${PREFIX_ROOT}/share/cmake/xgc2_vrpn_relay_core"

if [[ ! -x "${pkg_root}${PREFIX}/lib/${ROS_PACKAGE}/vrpn_relay" ]]; then
  echo "packaged vrpn_relay node is not executable" >&2
  exit 1
fi

cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCH}
Maintainer: XGC2 <lxk36@users.noreply.github.com>
Depends: ros-${ROS_DISTRO}-roscpp, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-roslaunch
Recommends: ros-${ROS_DISTRO}-vrpn-client-ros
Replaces: ${LEGACY_PACKAGE} (<< ${LEGACY_BOUNDARY})
Breaks: ${LEGACY_PACKAGE} (<< ${LEGACY_BOUNDARY})
Description: XGC2 onboard VRPN relay for ROS ${ROS_DISTRO}
 Relays one VRPN rigid body onto local pose, twist and acceleration topics
 after a quality gate, applies a fixed or first-frame XY offset, and can
 publish a rate-limited PX4 vision pose. Also ships the ROS-free core library
 of that logic (libxgc2_vrpn_relay_core, headers and a CMake config).
EOF

cat > "${pkg_root}/usr/share/doc/${PACKAGE}/README" <<EOF
${PACKAGE}

ROS package:
  ${ROS_PACKAGE}

Node:
  ${PREFIX}/lib/${ROS_PACKAGE}/vrpn_relay

Launch files:
  ${PREFIX}/share/${ROS_PACKAGE}/launch/mocap.launch
  ${PREFIX}/share/${ROS_PACKAGE}/launch/vrpn.launch

Core library for other products:
  find_package(xgc2_vrpn_relay_core CONFIG REQUIRED)
  target_link_libraries(<target> PRIVATE xgc2_vrpn_relay_core::core)
EOF

find "${pkg_root}" -type d -exec chmod 0755 {} +
find "${pkg_root}" -type f -exec chmod 0644 {} +
chmod 0755 "${pkg_root}${PREFIX}/lib/${ROS_PACKAGE}/vrpn_relay"

fakeroot dpkg-deb --build "${pkg_root}" "${OUTPUT_DIR}/${PACKAGE}_${VERSION}_${ARCH}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "${PACKAGE}_*.deb" -print | sort
