#!/usr/bin/env bash
# 带 RTK 的地图标定。启动后打开 RViz2 和网页。
# 在网页里选择地图和 mcap，再勾选雷达。
#
#   ./play_map_calib.sh
#   ./play_map_calib.sh --mcap <包> --map <pcd 或地图目录> --yaml <初始外参> --output <yaml>
#
# --mcap 和 --map 只作为网页里的默认路径，打开页面后仍可以改。
#
# 北站绝对坐标地图（默认，0.30 m 下采样）：
#   ./play_map_calib.sh \
#     --mcap /home/zzl/bag/apolong_calib/lidar_gnss_20261116_160451_0.mcap \
#     --map /home/zzl/bag/apolong_calib/20261113150315_0_voxel0.30.pcd
#
# 带经纬度起点的横轴墨卡托地图：
#   ./play_map_calib.sh \
#     --mcap /home/zzl/bag/cityloong_calib_data_1/lidar_gnss_20261110_171441_0.mcap \
#     --map /home/zzl/SUTPC/点云地图/map_20261022_morning_2_traffic_signs \
#     --output /home/zzl/SUTPC/产出/apolong/calib/map_calib_cityloong.yaml

set -euo pipefail

IMAGE="${ROS_IMAGE:-ros2-jazzy:dev}"
MCAP="/home/zzl/bag/apolong_calib/lidar_gnss_20261116_160451_0.mcap"
MAP="/home/zzl/bag/apolong_calib/20261113150315_0_voxel0.30.pcd"
CALIB_DIR="$(cd "$(dirname "$0")" && pwd)"
YAML="$CALIB_DIR/sensors_calibration.yaml"
[[ -f "$YAML" ]] || YAML=""
OUTPUT="$CALIB_DIR/map_calib_apolong.yaml"
CONTAINER="lidar-map-calib"
PORT="8766"

usage() {
  sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mcap) MCAP="${2:-}"; shift 2 ;;
    --map) MAP="${2:-}"; shift 2 ;;
    --output) OUTPUT="${2:-}"; shift 2 ;;
    --yaml) YAML="${2-}"; shift 2 ;;
    --image) IMAGE="${2:-}"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "未知参数: $1" >&2; usage ;;
  esac
done

[[ -e "$MCAP" ]] && MCAP="$(readlink -f "$MCAP")" || MCAP=""
[[ -e "$MAP" ]] && MAP="$(readlink -f "$MAP")" || MAP=""
if [[ -n "$YAML" ]]; then
  [[ -f "$YAML" ]] || { echo "找不到初始外参: $YAML" >&2; exit 1; }
  YAML="$(readlink -f "$YAML")"
fi
OUTPUT="$(readlink -f "$OUTPUT" 2>/dev/null || echo "$OUTPUT")"
mkdir -p "$(dirname "$OUTPUT")"

ROOTS=()
add_root() {
  local path="$1"
  [[ -d "$path" ]] || return 0
  local root
  root="$(readlink -f "$path")"
  local existing
  for existing in "${ROOTS[@]+"${ROOTS[@]}"}"; do
    [[ "$root" == "$existing" || "$root" == "$existing"/* ]] && return 0
  done
  local kept=()
  for existing in "${ROOTS[@]+"${ROOTS[@]}"}"; do
    [[ "$existing" == "$root"/* ]] || kept+=("$existing")
  done
  ROOTS=("${kept[@]+"${kept[@]}"}" "$root")
}
add_root /media/zzl/SSD
add_root /home/zzl/bag
add_root "/home/zzl/SUTPC/点云地图"
add_root "/home/zzl/SUTPC/产出"
add_root "$CALIB_DIR"
[[ -n "$YAML" ]] && add_root "$(dirname "$YAML")"
[[ -n "$MCAP" ]] && add_root "$(dirname "$MCAP")"
if [[ -n "$MAP" ]]; then
  if [[ -d "$MAP" ]]; then
    add_root "$MAP"
  else
    add_root "$(dirname "$MAP")"
  fi
fi
DATA_ROOTS=""
if ((${#ROOTS[@]})); then
  DATA_ROOTS="$(IFS=:; echo "${ROOTS[*]}")"
fi

WORKDIR="$(mktemp -d /tmp/lidar-map-calib.XXXXXX)"
HELPER_PORT=8767
helper_pid=""
if ! python3 -c "import socket; socket.create_connection(('127.0.0.1', ${HELPER_PORT}), 0.5).close()" 2>/dev/null; then
  python3 "$CALIB_DIR/copy_calib_clipboard.py" --dir "$(dirname "$OUTPUT")" --port "$HELPER_PORT" &
  helper_pid=$!
fi
trap '[[ -n "$helper_pid" ]] && kill "$helper_pid" 2>/dev/null || true; rm -rf "$WORKDIR"' EXIT

cat > "$WORKDIR/wait_port.py" << EOF
import socket, sys
try:
    socket.create_connection(("127.0.0.1", ${PORT}), 1).close()
except OSError:
    sys.exit(1)
EOF

cat > "$WORKDIR/view.rviz" << 'EOF'
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Plane: XY
      Plane Cell Count: 20
      Cell Size: 10
      Color: 160; 160; 160
      Reference Frame: <Fixed Frame>
    - Class: rviz_default_plugins/PointCloud2
      Name: map
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/map
      Use Fixed Frame: true
      Style: Points
      Size (Pixels): 2
      Color Transformer: FlatColor
      Color: 210; 210; 210
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: rtk
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/rtk
      Use Fixed Frame: true
      Style: Points
      Size (Pixels): 6
      Color Transformer: FlatColor
      Color: 255; 140; 0
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: lidar_0
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/n0/points
      Style: Points
      Size (Pixels): 3
      Color Transformer: FlatColor
      Color: 255; 60; 60
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: lidar_1
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/n1/points
      Style: Points
      Size (Pixels): 3
      Color Transformer: FlatColor
      Color: 60; 200; 80
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: lidar_2
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/n2/points
      Style: Points
      Size (Pixels): 3
      Color Transformer: FlatColor
      Color: 70; 140; 255
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: lidar_3
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/n3/points
      Style: Points
      Size (Pixels): 3
      Color Transformer: FlatColor
      Color: 255; 200; 40
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: lidar_4
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/n4/points
      Style: Points
      Size (Pixels): 3
      Color Transformer: FlatColor
      Color: 80; 220; 220
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: lidar_5
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/n5/points
      Style: Points
      Size (Pixels): 3
      Color Transformer: FlatColor
      Color: 220; 120; 220
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: lidar_6
      Enabled: true
      Topic:
        Depth: 1
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/n6/points
      Style: Points
      Size (Pixels): 3
      Color Transformer: FlatColor
      Color: 255; 120; 60
      Decay Time: 0
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: map
    Frame Rate: 10
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 80
      Name: Current View
      Pitch: 1.1
      Target Frame: <Fixed Frame>
      Yaw: 0.8
Window Geometry:
  Height: 900
  Width: 1400
EOF

DOCKER_ARGS=(
  --rm --name "$CONTAINER"
  --ipc=host
  -p "${PORT}:${PORT}"
  -e "DISPLAY=${DISPLAY:-:0}"
  -e QT_X11_NO_MITSHM=1
  -e "SUGGEST_MCAP=${MCAP}"
  -e "SUGGEST_MAP=${MAP}"
  -e "SUGGEST_YAML=${YAML}"
  -e "DATA_ROOTS=${DATA_ROOTS}"
  -e "OUTPUT=/out/$(basename "$OUTPUT")"
  -e "PORT=${PORT}"
  -e PYTHONPATH=/calib:/opt/calib/lib/python3.12/site-packages
  -e PYTHONDONTWRITEBYTECODE=1
  -v /tmp/.X11-unix:/tmp/.X11-unix
  -v "$WORKDIR/view.rviz:/viz/view.rviz:ro"
  -v "$WORKDIR/wait_port.py:/viz/wait_port.py:ro"
  -v "$CALIB_DIR:/calib"
  -v "$(dirname "$OUTPUT"):/out"
)
for root in "${ROOTS[@]+"${ROOTS[@]}"}"; do
  DOCKER_ARGS+=(-v "$root:$root:ro")
done
if [[ -d /dev/dri ]]; then
  vid_gid="$(getent group video | cut -d: -f3)"
  rnd_gid="$(getent group render | cut -d: -f3)"
  DOCKER_ARGS+=(--device /dev/dri)
  [[ -n "$vid_gid" ]] && DOCKER_ARGS+=(--group-add "$vid_gid")
  [[ -n "$rnd_gid" ]] && DOCKER_ARGS+=(--group-add "$rnd_gid")
else
  DOCKER_ARGS+=(-e LIBGL_ALWAYS_SOFTWARE=1)
fi

xhost +SI:localuser:"${USER}" >/dev/null 2>&1 || true
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

echo "在网页选择地图和 mcap。默认地图: ${MAP:-未设置}"
echo "默认数据包: ${MCAP:-未设置}"
echo "结果: $OUTPUT"
echo "浏览器: http://127.0.0.1:${PORT}"
echo "RViz：白点地图，橙点 RTK，其余颜色是第 1 到第 7 颗被选中的雷达。关闭 RViz 即停止。"

docker run "${DOCKER_ARGS[@]}" "$IMAGE" \
  bash -lc "source /opt/ros/jazzy/setup.bash
python3 -c 'import flask' >/dev/null 2>&1 || pip3 install --break-system-packages -q flask
python3 /calib/web_calib/map_calib.py >/tmp/map_calib.log 2>&1 &
web_pid=\$!
ready=0
for _ in \$(seq 1 80); do
  python3 /viz/wait_port.py && ready=1 && break
  sleep 0.25
done
if [[ \"\$ready\" != 1 ]]; then
  echo \"网页没有起来\" >&2
  cat /tmp/map_calib.log >&2
  kill \$web_pid 2>/dev/null || true
  exit 1
fi
echo '网页已启动。请在页面选择地图和数据包后再读取。关闭 RViz 即停止标定。'
tail -n +1 -f /tmp/map_calib.log &
tail_pid=\$!
(
  while kill -0 \$web_pid 2>/dev/null; do
    sleep 0.5
  done
  pkill -TERM -x rviz2 2>/dev/null || true
) &
watch_pid=\$!
rviz2 -d /viz/view.rviz >/tmp/rviz.log 2>&1
status=\$?
kill \$web_pid \$tail_pid \$watch_pid 2>/dev/null || true
wait \$web_pid \$tail_pid \$watch_pid 2>/dev/null || true
exit \$status"
