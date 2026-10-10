#!/usr/bin/env bash
# clang-format, a build with warnings as errors, and clang-tidy over the C++ of this repository.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/cpp-quality}"
JOBS="${JOBS:-$(nproc)}"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --work-dir)
      WORK_DIR="$2"
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

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

for tool in clang-format clang-tidy catkin_make git rsync; do
  if ! command -v "${tool}" >/dev/null; then
    echo "missing required tool: ${tool}" >&2
    exit 1
  fi
done

git config --global --add safe.directory "${REPO_ROOT}" >/dev/null 2>&1 || true

mapfile -t FORMAT_FILES < <(
  cd "${REPO_ROOT}"
  git ls-files '*.cpp' '*.hpp' | sort
)
if [[ "${#FORMAT_FILES[@]}" -eq 0 ]]; then
  echo "no C++ files found" >&2
  exit 1
fi
(
  cd "${REPO_ROOT}"
  clang-format --dry-run --Werror "${FORMAT_FILES[@]}"
)

src_dir="${WORK_DIR}/src/xgc2-vrpn-relay"
build_dir="${WORK_DIR}/build"
rm -rf "${WORK_DIR}/src" "${build_dir}" "${WORK_DIR}/devel"
mkdir -p "${src_dir}"
rsync -a --delete --exclude .git --exclude .work --exclude debs --exclude .ci \
  "${REPO_ROOT}/" "${src_dir}/"

(
  cd "${WORK_DIR}"
  catkin_make -j"${JOBS}" \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DXGC2_VRPN_RELAY_WERROR=ON \
    -DCATKIN_ENABLE_TESTING=ON
  # The test targets are not part of "all"; clang-tidy needs them configured, and built for the
  # generated headers they include.
  catkin_make -j"${JOBS}" tests
)

# The downstream smoke project is built from an installed prefix, not from this tree.
mapfile -t TIDY_FILES < <(
  cd "${REPO_ROOT}"
  git ls-files 'xgc2_vrpn_relay/*.cpp' | grep -v '/downstream/' | sort
)
for file in "${TIDY_FILES[@]}"; do
  clang-tidy --quiet "${src_dir}/${file}" -p "${build_dir}"
done

echo "C++ quality check passed"
