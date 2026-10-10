#!/usr/bin/env bash
# Validate a built ros-<distro>-xgc2-vrpn-relay deb without installing it: package metadata,
# payload, and that another product can find and link the core library from the payload alone.
# The ROS distro is read from the package name. Where /opt/ros/<distro> exists, the node and the
# launch files are also checked against that ROS installation.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
DEB_DIR=""
DEB_PATH=""

usage() {
  cat <<EOF
Usage: ${0##*/} --deb-dir DIR
       ${0##*/} --deb PATH

Validate package metadata and payload without installing the package.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --deb-dir)
      DEB_DIR="$2"
      shift 2
      ;;
    --deb)
      DEB_PATH="$2"
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

if [[ -z "${DEB_DIR}" && -z "${DEB_PATH}" ]]; then
  echo "--deb-dir or --deb is required" >&2
  usage >&2
  exit 2
fi

tmp_dir="$(mktemp -d)"
cleanup() {
  rm -rf "${tmp_dir}"
}
trap cleanup EXIT

find_deb() {
  local match=""
  local path
  if [[ -n "${DEB_PATH}" ]]; then
    printf '%s\n' "${DEB_PATH}"
    return 0
  fi
  shopt -s nullglob
  for path in "${DEB_DIR}"/ros-*-xgc2-vrpn-relay_*.deb; do
    if [[ -n "${match}" ]]; then
      echo "multiple xgc2-vrpn-relay debs in ${DEB_DIR}" >&2
      return 1
    fi
    match="${path}"
  done
  shopt -u nullglob
  if [[ -z "${match}" ]]; then
    echo "missing ros-<distro>-xgc2-vrpn-relay in ${DEB_DIR}" >&2
    return 1
  fi
  printf '%s\n' "${match}"
}

deb="$(find_deb)"
package="$(dpkg-deb --field "${deb}" Package)"
case "${package}" in
  ros-melodic-xgc2-vrpn-relay) ros_distro=melodic ;;
  ros-noetic-xgc2-vrpn-relay) ros_distro=noetic ;;
  *)
    echo "unexpected package ${package}" >&2
    exit 1
    ;;
esac
prefix="/opt/ros/${ros_distro}"

# Metadata. The relay is compiled now: never Architecture: all, no Python.
architecture="$(dpkg-deb --field "${deb}" Architecture)"
if [[ "${architecture}" != "$(dpkg --print-architecture)" ]]; then
  echo "architecture ${architecture} is not the build architecture" >&2
  exit 1
fi
depends="$(dpkg-deb --field "${deb}" Depends)"
for dependency in "ros-${ros_distro}-roscpp" "ros-${ros_distro}-geometry-msgs"; do
  grep -Fq "${dependency}" <<<"${depends}"
done
if grep -Fq "python" <<<"${depends}"; then
  echo "the relay is compiled and must not depend on Python: ${depends}" >&2
  exit 1
fi
dpkg-deb --field "${deb}" Replaces | grep -Fq "xgc2-vrpn-relay (<< 0.2.0)"
dpkg-deb --field "${deb}" Breaks | grep -Fq "xgc2-vrpn-relay (<< 0.2.0)"

root="${tmp_dir}/root"
mkdir -p "${root}"
dpkg-deb --extract "${deb}" "${root}"
installed="${root}${prefix}"

# Payload: node, launch files, package manifest, and the core library with its headers and
# CMake config.
node="${installed}/lib/xgc2_vrpn_relay/vrpn_relay"
test -x "${node}"
readelf -h "${node}" >/dev/null
if [[ "$(head -c 2 "${node}")" == "#!" ]]; then
  echo "vrpn_relay must be a compiled executable, not a script" >&2
  exit 1
fi
readelf -d "${node}" | grep -Fq "libroscpp"
test -f "${installed}/share/xgc2_vrpn_relay/package.xml"
test -f "${installed}/share/xgc2_vrpn_relay/launch/vrpn.launch"
test -f "${installed}/share/xgc2_vrpn_relay/launch/mocap.launch"
grep -q 'refresh_tracker_frequency: 0.0' "${installed}/share/xgc2_vrpn_relay/launch/vrpn.launch"
grep -q 'type="vrpn_relay"' "${installed}/share/xgc2_vrpn_relay/launch/mocap.launch"
grep -q '<license>Proprietary</license>' "${installed}/share/xgc2_vrpn_relay/package.xml"
for header in types quality offset pose_relay vision_cadence; do
  test -f "${installed}/include/xgc2_vrpn_relay/${header}.hpp"
done
test -f "${installed}/lib/libxgc2_vrpn_relay_core.a"
test -f "${installed}/share/cmake/xgc2_vrpn_relay_core/xgc2_vrpn_relay_coreConfig.cmake"
test -f "${installed}/share/cmake/xgc2_vrpn_relay_core/xgc2_vrpn_relay_coreTargets.cmake"
test -f "${installed}/share/cmake/xgc2_vrpn_relay_core/xgc2_vrpn_relay_coreConfigVersion.cmake"

# Nothing of the Python relay is left, and the package ships no service of its own.
if find "${root}" \( -name '*.py' -o -name '*.pyc' -o -path '*/lib/xgc2_vrpn_relay/xgc2_vrpn_relay' \) \
  -print | grep -q .; then
  echo "Python relay files are still packaged" >&2
  exit 1
fi
if find "${root}/lib/systemd/system" -maxdepth 1 -name '*.service' 2>/dev/null | grep -q .; then
  echo "shared relay must not ship a systemd unit" >&2
  exit 1
fi

# Against the ROS installation of the same distro, when this machine has it.
if [[ -f "${prefix}/setup.bash" ]]; then
  ros_libs="${prefix}/lib"
  if LD_LIBRARY_PATH="${ros_libs}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ldd "${node}" |
    grep -Fq 'not found'; then
    echo "vrpn_relay has unresolved dynamic dependencies" >&2
    LD_LIBRARY_PATH="${ros_libs}${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" ldd "${node}" >&2
    exit 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "${prefix}/setup.bash"
  set -u
  export ROS_PACKAGE_PATH="${installed}/share${ROS_PACKAGE_PATH:+:${ROS_PACKAGE_PATH}}"
  test "$(rospack find xgc2_vrpn_relay)" = "${installed}/share/xgc2_vrpn_relay"
  roslaunch --files xgc2_vrpn_relay mocap.launch >/dev/null
  roslaunch --files xgc2_vrpn_relay vrpn.launch >/dev/null
else
  echo "no ${prefix}; skipped the checks against the ROS installation" >&2
fi

# Another product links the core from the payload alone: configure, build and run a program
# that uses it, with nothing but the install prefix to find it.
smoke_build="${tmp_dir}/downstream"
mkdir -p "${smoke_build}"
(
  cd "${smoke_build}"
  cmake "${REPO_ROOT}/xgc2_vrpn_relay/core/test/downstream" \
    -DCMAKE_PREFIX_PATH="${installed}" -DCMAKE_BUILD_TYPE=Release >/dev/null
  cmake --build . >/dev/null
  ./downstream_smoke
)

echo "Package content check passed"
