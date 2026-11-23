#!/usr/bin/env bash
# 按外参和 RTK 轨迹播放点云。先在网页里选择每个话题对应的外参。
# 地图可选。关闭这个终端即停止容器，并卸下容器对 USB 的挂载。
#
#   ./play_rtk_view.sh
#   ./play_rtk_view.sh --mcap <包> --yaml <外参> --map <pcd 或地图目录>
#   ./play_rtk_view.sh --map ""

set -euo pipefail

IMAGE="${ROS_IMAGE:-ros2-jazzy:dev}"
CALIB_DIR="$(cd "$(dirname "$0")" && pwd)"
MCAP="/home/zzl/bag/apolong_calib/lidar_gnss_20261116_160451_0.mcap"
YAML="$CALIB_DIR/sensors_calibration.yaml"
MAP="/home/zzl/bag/apolong_calib/20261113150315_0_voxel0.30.pcd"
CONTAINER="lidar-rtk-view"
PORT="8768"

usage() {
  sed -n '2,7p' "$0" | sed 's/^# \{0,1\}//'
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mcap) MCAP="${2:-}"; shift 2 ;;
    --yaml) YAML="${2:-}"; shift 2 ;;
    --map) MAP="${2-}"; shift 2 ;;
    --image) IMAGE="${2:-}"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "未知参数: $1" >&2; usage ;;
  esac
done

[[ -f "$MCAP" ]] || { echo "找不到 mcap: $MCAP" >&2; exit 1; }
[[ -f "$YAML" ]] || { echo "找不到外参: $YAML" >&2; exit 1; }
if [[ -n "$MAP" && ! -e "$MAP" ]]; then
  echo "找不到地图: $MAP" >&2
  exit 1
fi
MCAP="$(readlink -f "$MCAP")"
YAML="$(readlink -f "$YAML")"
if [[ -n "$MAP" ]]; then
  MAP="$(readlink -f "$MAP")"
fi

CONTROL_PORT="8769"
stop_container() {
  local status=$?
  trap - EXIT INT TERM HUP
  if [[ -n "${HOST_PID:-}" ]]; then
    kill "$HOST_PID" >/dev/null 2>&1 || true
    wait "$HOST_PID" >/dev/null 2>&1 || true
  fi
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  exit "$status"
}
trap stop_container EXIT INT TERM HUP

echo "浏览器: http://127.0.0.1:${PORT}"
echo "数据包: $MCAP"
echo "外参: $YAML"
echo "地图: ${MAP:-不使用}"
echo "网页里可以按当前输入的路径重新挂载。关闭本终端即停止，并卸下容器对 USB 的挂载。"

python3 "$CALIB_DIR/rtk_view_host.py" \
  --image "$IMAGE" \
  --container "$CONTAINER" \
  --port "$PORT" \
  --control-port "$CONTROL_PORT" \
  --calib "$CALIB_DIR" \
  --mcap "$MCAP" \
  --yaml "$YAML" \
  --map "$MAP" \
  --root /home/zzl/bag \
  --root "/home/zzl/SUTPC/点云地图" \
  --root "$CALIB_DIR" \
  --allow /home/zzl \
  --allow /media/zzl &
HOST_PID=$!
wait "$HOST_PID"
