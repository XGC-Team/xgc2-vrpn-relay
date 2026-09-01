#!/usr/bin/env bash
# Assemble Architecture: all xgc2-vrpn-relay for a single ROS distro.
# Python-only; no compile. Melodic needs a python2 rospy shebang.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
# Do not inherit the host ROS_DISTRO; this vehicle package targets Melodic.
ROS_DISTRO="${XGC2_VRPN_ROS_DISTRO:-melodic}"
OUTPUT_DIR="${1:-${REPO_ROOT}/debs}"
VERSION="${PACKAGE_VERSION:-$(awk '/^version:/ {print $2; exit}' "${REPO_ROOT}/.xgc2/product.yml")}"
PACKAGE="xgc2-vrpn-relay"
PREFIX="/opt/ros/${ROS_DISTRO}"
SRC="${REPO_ROOT}/xgc2_vrpn_relay"

if [[ ! -f "${SRC}/package.xml" ]]; then
  echo "missing ${SRC}/package.xml" >&2
  exit 1
fi

BUILD_DIR="$(mktemp -d)"
cleanup() { rm -rf "${BUILD_DIR}"; }
trap cleanup EXIT

pkg_root="${BUILD_DIR}/${PACKAGE}"
share="${pkg_root}${PREFIX}/share/xgc2_vrpn_relay"
lib="${pkg_root}${PREFIX}/lib/xgc2_vrpn_relay"
mkdir -p "${share}/launch" "${lib}/xgc2_vrpn_relay" \
  "${pkg_root}/DEBIAN" "${pkg_root}/usr/share/doc/${PACKAGE}"

cp -a "${SRC}/package.xml" "${share}/package.xml"
cp -a "${SRC}/launch/"*.launch "${share}/launch/"
cp -a "${SRC}/scripts/xgc2_vrpn_relay/"*.py "${lib}/xgc2_vrpn_relay/"
cp -a "${SRC}/scripts/vrpn_relay" "${lib}/vrpn_relay"
if [[ "${ROS_DISTRO}" == "melodic" ]]; then
  sed -i '1s|^#!.*|#!/usr/bin/env python|' "${lib}/vrpn_relay"
fi
chmod 0755 "${lib}/vrpn_relay"

cat > "${pkg_root}/DEBIAN/control" <<EOF
Package: ${PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: all
Maintainer: XGC2 <apt@example.com>
Depends: ros-${ROS_DISTRO}-vrpn-client-ros, ros-${ROS_DISTRO}-rospy, ros-${ROS_DISTRO}-geometry-msgs, ros-${ROS_DISTRO}-roslaunch
Description: XGC2 VRPN relay and Experiment XYZ localization projection
EOF
printf 'xgc2-vrpn-relay %s for ROS %s; includes Experiment XYZ projection\n' "${VERSION}" "${ROS_DISTRO}" \
  > "${pkg_root}/usr/share/doc/${PACKAGE}/README"

find "${pkg_root}" -type d -exec chmod 0755 {} +
chmod 0755 "${pkg_root}/DEBIAN"
chmod 0644 "${pkg_root}/DEBIAN/control" "${pkg_root}/usr/share/doc/${PACKAGE}/README"

mkdir -p "${OUTPUT_DIR}"
deb_path="${OUTPUT_DIR}/${PACKAGE}_${VERSION}_all.deb"
fakeroot dpkg-deb --build "${pkg_root}" "${deb_path}" >/dev/null
echo "${deb_path}"
