#!/usr/bin/env bash
# Test and package the relay for one ROS distro in the current environment: run every test, install
# into a staging root, build ros-<distro>-xgc2-vrpn-relay and check the deb. build_debs_in_docker.sh
# runs this inside the XGC2 build image of the distro.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/debs}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
ROS_DISTRO="${ROS_DISTRO:-}"
JOBS="${JOBS:-$(nproc)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --ros-distro)
      ROS_DISTRO="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
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

case "${ROS_DISTRO}" in
  melodic|noetic) ;;
  *)
    echo "set ROS_DISTRO or --ros-distro to melodic or noetic" >&2
    exit 1
    ;;
esac

for package in cmake fakeroot dpkg-dev rsync \
  "ros-${ROS_DISTRO}-roscpp" "ros-${ROS_DISTRO}-geometry-msgs" "ros-${ROS_DISTRO}-rostest" \
  "ros-${ROS_DISTRO}-roslaunch"; do
  if ! dpkg -s "${package}" >/dev/null 2>&1; then
    echo "missing ${package}; use the xgc2-build-<ubuntu>-ros-<distro> image" >&2
    exit 1
  fi
done

# Both are used after cd and by other scripts: make them absolute.
mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"
WORK_DIR="$(cd "${WORK_DIR}" && pwd)"
OUTPUT_DIR="$(cd "${OUTPUT_DIR}" && pwd)"

export ROS_DISTRO
"${SCRIPT_DIR}/run_source_tests.sh" --ros-distro "${ROS_DISTRO}" --jobs "${JOBS}" \
  --work-dir "${WORK_DIR}/tests"

install_ws="${WORK_DIR}/install-ws"
install_root="${WORK_DIR}/install-root"
rm -rf "${install_ws}" "${install_root}"
mkdir -p "${install_ws}/src" "${install_root}"
rsync -a --delete --exclude .git --exclude .work --exclude debs --exclude .ci \
  "${REPO_ROOT}/" "${install_ws}/src/xgc2-vrpn-relay/"
set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO}/setup.bash"
set -u
(
  cd "${install_ws}"
  DESTDIR="${install_root}" catkin_make -j"${JOBS}" install \
    -DCMAKE_INSTALL_PREFIX="/opt/ros/${ROS_DISTRO}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_CXX_FLAGS_RELEASE="-O3 -DNDEBUG" \
    -DCATKIN_ENABLE_TESTING=OFF
)

"${SCRIPT_DIR}/package_debs.sh" --install-root "${install_root}" --output-dir "${OUTPUT_DIR}"

mapfile -t package_debs < <(
  find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "ros-${ROS_DISTRO}-xgc2-vrpn-relay_*.deb" -print | sort
)
if [[ "${#package_debs[@]}" -ne 1 ]]; then
  echo "expected exactly one ros-${ROS_DISTRO}-xgc2-vrpn-relay deb, found ${#package_debs[@]}" >&2
  exit 1
fi
"${SCRIPT_DIR}/check_installed_packages.sh" --deb "${package_debs[0]}"
echo "Built ${package_debs[0]}"
