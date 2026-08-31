#!/usr/bin/env bash
set -euo pipefail

PACKAGE="xgc2-vrpn-relay"
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
    if dpkg-deb --field "${DEB_PATH}" Package | grep -Fxq "${PACKAGE}"; then
      printf '%s\n' "${DEB_PATH}"
      return 0
    fi
    echo "missing ${PACKAGE} in ${DEB_PATH}" >&2
    return 1
  fi
  shopt -s nullglob
  for path in "${DEB_DIR}"/*.deb; do
    if dpkg-deb --field "${path}" Package | grep -Fxq "${PACKAGE}"; then
      if [[ -n "${match}" ]]; then
        echo "multiple ${PACKAGE} debs in ${DEB_DIR}" >&2
        return 1
      fi
      match="${path}"
    fi
  done
  shopt -u nullglob
  if [[ -z "${match}" ]]; then
    echo "missing ${PACKAGE} in ${DEB_DIR}" >&2
    return 1
  fi
  printf '%s\n' "${match}"
}

deb="$(find_deb)"
dpkg-deb --field "${deb}" Architecture | grep -Fx all >/dev/null
dpkg-deb --field "${deb}" Depends | grep -F "python3" >/dev/null
mkdir -p "${tmp_dir}/root" "${tmp_dir}/control"
dpkg-deb --extract "${deb}" "${tmp_dir}/root"
dpkg-deb --control "${deb}" "${tmp_dir}/control"

for prefix in /opt/ros/melodic /opt/ros/noetic; do
  test -x "${tmp_dir}/root${prefix}/lib/xgc2_vrpn_relay/experiment_projection"
  test -x "${tmp_dir}/root${prefix}/lib/xgc2_vrpn_relay/vrpn_relay"
  test -f "${tmp_dir}/root${prefix}/lib/xgc2_vrpn_relay/xgc2_vrpn_relay/projection.py"
  test -f "${tmp_dir}/root${prefix}/lib/xgc2_vrpn_relay/xgc2_vrpn_relay/quality.py"
  test -f "${tmp_dir}/root${prefix}/lib/xgc2_vrpn_relay/xgc2_vrpn_relay/rate.py"
  test -f "${tmp_dir}/root${prefix}/share/xgc2_vrpn_relay/package.xml"
  test -f "${tmp_dir}/root${prefix}/share/xgc2_vrpn_relay/launch/vrpn.launch"
  test -f "${tmp_dir}/root${prefix}/share/xgc2_vrpn_relay/launch/mocap.launch"
  test -f "${tmp_dir}/root${prefix}/share/xgc2_vrpn_relay/launch/experiment_projection.launch"
  grep -q 'refresh_tracker_frequency: 0.0' \
    "${tmp_dir}/root${prefix}/share/xgc2_vrpn_relay/launch/vrpn.launch"
  grep -q 'pkg="xgc2_vrpn_relay"' \
    "${tmp_dir}/root${prefix}/share/xgc2_vrpn_relay/launch/mocap.launch"
  launch="${tmp_dir}/root${prefix}/share/xgc2_vrpn_relay/launch/experiment_projection.launch"
  grep -q 'name="source_root"' "${launch}"
  grep -q 'name="mocap_rigid_body"' "${launch}"
  grep -q 'name="robot_namespace"' "${launch}"
  grep -q 'name="publish_vision"' "${launch}"
  if grep -Eq 'name="(pose_in|twist_in|accel_in|pose_out|twist_out|accel_out|vision_out)"' \
      "${launch}"; then
    echo "experiment_projection.launch must not expose explicit topic args" >&2
    exit 1
  fi
done

if find "${tmp_dir}/root/lib/systemd/system" -maxdepth 1 -name '*.service' \
  2>/dev/null | grep -q .; then
  echo "shared relay must not ship a systemd unit" >&2
  exit 1
fi

echo "Package content check passed"
