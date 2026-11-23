#!/usr/bin/env bash
# 循环回放 160451，按标定文件把四路点云变到 base_link。
# 浏览器 http://127.0.0.1:8765 修改外参。页面或本终端按空格暂停。
#
#   ./play_extrinsic_web.sh
#   ./play_extrinsic_web.sh --rate 0.5

set -euo pipefail

IMAGE="${ROS_IMAGE:-ros2-jazzy:dev}"
MCAP="/media/zzl/SSD/bag/lidar_gnss_20261116_160451/lidar_gnss_20261116_160451_0.mcap"
RATE="1.0"
YAML_NAME="sensors_calibration_manual.yaml"
CONTAINER="lidar-extrinsic-web"
CALIB_DIR="$(cd "$(dirname "$0")" && pwd)"

usage() {
  sed -n '2,5p' "$0" | sed 's/^# \{0,1\}//'
  exit 2
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --mcap) MCAP="${2:-}"; shift 2 ;;
    --rate) RATE="${2:-}"; shift 2 ;;
    --yaml) YAML_NAME="${2:-}"; shift 2 ;;
    --image) IMAGE="${2:-}"; shift 2 ;;
    -h|--help) usage ;;
    *) echo "未知参数: $1" >&2; usage ;;
  esac
done

[[ -f "$MCAP" ]] || { echo "找不到 mcap: $MCAP" >&2; exit 1; }
if [[ "$YAML_NAME" = /* ]]; then
  CALIB_FILE="$YAML_NAME"
else
  CALIB_FILE="$CALIB_DIR/$YAML_NAME"
fi
if [[ -f "$CALIB_FILE" ]]; then
  YAML_NAME="$(basename "$CALIB_FILE")"
  USE_YAML=1
else
  USE_YAML=0
  echo "未提供初始外参（$CALIB_FILE 不存在）。四路从 0 开始，可用 --yaml 指定已有文件。"
fi
MCAP="$(readlink -f "$MCAP")"

WORKDIR="$(mktemp -d /tmp/lidar-extrinsic-web.XXXXXX)"
stop_container() {
  local status=$?
  trap - EXIT INT TERM HUP
  docker rm -f "$CONTAINER" >/dev/null 2>&1 || true
  rm -rf "$WORKDIR"
  exit "$status"
}
trap stop_container EXIT INT TERM HUP

cat > "$WORKDIR/wait_port.py" << 'EOF'
import socket
import sys

try:
    socket.create_connection(("127.0.0.1", 8765), 1).close()
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
      Cell Size: 5
      Color: 160; 160; 160
      Reference Frame: <Fixed Frame>
    - Class: rviz_default_plugins/PointCloud2
      Name: long_left_front
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/long/left_front/points
      Use Fixed Frame: true
      Style: Points
      Size (Pixels): 2
      Color Transformer: FlatColor
      Color: 255; 60; 60
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: long_right_end
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/long/right_end/points
      Use Fixed Frame: true
      Style: Points
      Size (Pixels): 2
      Color Transformer: FlatColor
      Color: 60; 200; 80
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: blind_left_front
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/blind/left_front/points
      Use Fixed Frame: true
      Style: Points
      Size (Pixels): 3
      Color Transformer: FlatColor
      Color: 70; 140; 255
      Decay Time: 0
    - Class: rviz_default_plugins/PointCloud2
      Name: blind_right_end
      Enabled: true
      Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /calib/lidar/blind/right_end/points
      Use Fixed Frame: true
      Style: Points
      Size (Pixels): 3
      Color Transformer: FlatColor
      Color: 255; 200; 40
      Decay Time: 0
  Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: base_link
    Frame Rate: 20
  Name: root
  Tools:
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 40
      Name: Current View
      Pitch: 0.8
      Target Frame: <Fixed Frame>
      Yaw: 0.8
Window Geometry:
  Height: 900
  Width: 1400
EOF

DOCKER_ARGS=(
  --rm --name "$CONTAINER"
  --ipc=host
  -p 8765:8765
  -e "DISPLAY=${DISPLAY:-:0}"
  -e QT_X11_NO_MITSHM=1
  -e CALIB_SAVE=/calib/sensors_calibration_manual.yaml
  -v /tmp/.X11-unix:/tmp/.X11-unix
  -v "$WORKDIR/view.rviz:/viz/view.rviz:ro"
  -v "$WORKDIR/wait_port.py:/viz/wait_port.py:ro"
  -v "$CALIB_DIR:/calib"
  -v "$MCAP:/bag/lidar.mcap:ro"
)
if [[ -t 0 && -t 1 ]]; then
  DOCKER_ARGS+=(-it)
fi
if [[ -d /dev/dri ]]; then
  vid_gid="$(getent group video | cut -d: -f3)"
  rnd_gid="$(getent group render | cut -d: -f3)"
  DOCKER_ARGS+=(--device /dev/dri)
  [[ -n "$vid_gid" ]] && DOCKER_ARGS+=(--group-add "$vid_gid")
  [[ -n "$rnd_gid" ]] && DOCKER_ARGS+=(--group-add "$rnd_gid")
else
  DOCKER_ARGS+=(-e LIBGL_ALWAYS_SOFTWARE=1)
fi

if [[ "$USE_YAML" == 1 ]]; then
  DOCKER_ARGS+=(-e "CALIB_YAML=/calib/$YAML_NAME")
else
  DOCKER_ARGS+=(-e "CALIB_YAML=")
fi
xhost +SI:localuser:"${USER}" >/dev/null 2>&1 || true
docker rm -f "$CONTAINER" >/dev/null 2>&1 || true

echo "回放: $MCAP"
if [[ "$USE_YAML" == 1 ]]; then
  echo "初始外参: $CALIB_FILE"
else
  echo "初始外参: 无，四路为 0"
fi
echo "浏览器: http://127.0.0.1:8765"
echo "红 远距左前  绿 远距右后  蓝 补盲左前  黄 补盲右后"
echo "空格暂停/继续。关闭 RViz，或在此终端按 Ctrl+C，即停止容器并卸下它对 USB 的挂载。保存写入 sensors_calibration_manual.yaml"

docker run --init --sig-proxy=true "${DOCKER_ARGS[@]}" "$IMAGE" \
  bash -lc "source /opt/ros/jazzy/setup.bash
python3 -c 'import flask' >/dev/null 2>&1 || pip3 install --break-system-packages -q flask
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0 --yaw 0 --pitch 0 --roll 0 --frame-id map --child-frame-id base_link >/tmp/tf_base.log 2>&1 &
python3 /calib/web_calib/app.py >/tmp/calib_web.log 2>&1 &
web_pid=\$!
ready=0
for _ in \$(seq 1 50); do
  python3 /viz/wait_port.py && ready=1 && break
  sleep 0.2
done
if [[ \"\$ready\" != 1 ]]; then
  echo \"网页没有起来\" >&2
  cat /tmp/calib_web.log >&2
  kill -TERM 1
  exit 1
fi
rviz2 -d /viz/view.rviz >/tmp/rviz.log 2>&1 &
rviz_pid=\$!
(
  while [[ -d /proc/\$rviz_pid ]]; do
    state=\$(awk '{print \$3}' /proc/\$rviz_pid/stat 2>/dev/null || echo X)
    [[ \"\$state\" == Z || \"\$state\" == X ]] && break
    sleep 0.5
  done
  kill -TERM 1
) >/dev/null 2>&1 &
echo '网页已启动。此终端按空格也可暂停。关闭 RViz 或按 Ctrl+C 即停止。'
exec ros2 bag play /bag/lidar.mcap --loop --rate ${RATE} --read-ahead-queue-size 2000 --topics \\
  /sutpc/sensor_driver/lidar/long/left_front/lidar_points \\
  /sutpc/sensor_driver/lidar/long/right_end/lidar_points \\
  /sutpc/sensor_driver/lidar/blind/left_front/lidar_points \\
  /sutpc/sensor_driver/lidar/blind/right_end/lidar_points"
