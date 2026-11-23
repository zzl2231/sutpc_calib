# sutpc_calib

多激光雷达离线自动标定与 Web 可视化工具集。在 Docker（ROS 2 Jazzy）容器里读取 rosbag2(mcap) 数据包，结合 RTK 轨迹与点云地图完成雷达外参标定，并通过浏览器 + RViz2 实时可视化。

## 功能

| 工具 | 入口脚本 | 界面 | 说明 |
|---|---|---|---|
| 多点云可视化 | `play_rtk_view.sh` | 浏览器 http://127.0.0.1:8768 | 按外参和 RTK 轨迹把多路点云投到地图坐标系播放 |
| 地图标定（自动） | `play_map_calib.sh` | RViz2 + 浏览器 http://127.0.0.1:8766 | 选地图 + mcap，配准起点后逐帧优化，输出外参 yaml |
| 相对外参微调 | `play_extrinsic_web.sh` | RViz2 + 浏览器 http://127.0.0.1:8765 | 循环回放四路点云，网页手动调外参 |

标定算法核心：GICP（[small-gicp](https://github.com/koide3/small_gicp)）+ Scan Context 重定位 + RTK 轨迹约束，实现见 `long_pair_calib.py` 与 `web_calib/map_calib.py`。

## 环境需求

### 宿主机

- Ubuntu 24.04（noble）
- Docker Engine（>= 20.10），用户已在 `docker` 组
- 浏览器（Chrome / Firefox，用于标定与可视化页面）
- X11（RViz2 需要；脚本会自动 `xhost`，无 GPU 时回退软件渲染）

### Docker 镜像 `ros2-jazzy:dev`

代码本身不含 ROS 环境，所有数据处理和标定都在容器内完成。用本仓库 `docker/Dockerfile` 构建：

```bash
docker build -t ros2-jazzy:dev docker/
```

镜像内容（Ubuntu 24.04 基础）：

- ROS 2 Jazzy desktop + ros-dev-tools（colcon、rosbag2）
- `ros-jazzy-rosbag2-storage-mcap`、`ros-jazzy-pcl-ros`、`ros-jazzy-nmea-msgs`、`ros-jazzy-geodesy` 等 ROS 包
- Python 3.12 + `/opt/calib` venv：`small-gicp`、`kiss-icp[all]`、`pyproj`、`flask`、`scipy`、`evo`、`opencv-python-headless`
- PCL / Ceres / OpenCV / GeographicLib 开发库
- 与宿主同 UID 的用户（默认 `zzl`，构建参数 `USERNAME` / `USER_UID` / `USER_GID` 可改）

### 数据需求

- **数据包**：rosbag2 mcap 格式，含多路 `sensor_msgs/PointCloud2` 点云话题与 RTK/GNSS 话题
- **地图**：单文件 `.pcd`（已是 UTM 等投影坐标直接叠加），或含 `pointcloud_map.pcd` 的目录（带 `map_projector_info.yaml` 时按经纬度做横轴墨卡托投影）
- **初始外参**（可选）：`sensors_calibration.yaml` 格式，见下文

## 快速开始

```bash
git clone git@github.com:zzl2231/sutpc_calib.git
cd sutpc_calib

# 1. 构建镜像（只需一次）
docker build -t ros2-jazzy:dev docker/

# 2. 多点云可视化
./play_rtk_view.sh --mcap <数据包.mcap> --yaml <外参.yaml> --map <地图.pcd>

# 3. 地图标定
./play_map_calib.sh --mcap <数据包.mcap> --map <地图.pcd> --output result.yaml
```

## 外参文件格式

参考 `examples/sensors_calibration.yaml`：

```yaml
# 键名：<雷达 frame_id>_base_link，单位 m / rad，RPY 为固定轴 XYZ（x 前、y 左、z 上）
# 父坐标系是 RTK 后轴中心，高度用实测值
hesai_lidar_left_front_base_link:
  x: 0.0
  y: 0.0
  z: 1.5
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

- 参考雷达是 `front_mid_top`，它的安装外参保持不变
- 标定写 yaml 时，基准雷达相对 baseline 的高度固定为 1.0 m，其余雷达相对基准的位置不变；RTK 段仍是测到的高度
- 页面上的角度单位是度（滚轮微调，Shift 更细 / Ctrl 更粗），yaml 里写弧度

## 地图标定流程（play_map_calib.sh）

1. 网页里选择地图和 mcap，初始外参可不填，点「读取」
2. 勾选最多 7 颗雷达，指定一颗基准。填了初始外参时按 `frame_id` 自动预选
3. 「配准起点」：有外参的雷达用 RTK 位姿 × 外参做起点再 GICP；没有外参的雷达由 RTK 给出位置 + 车体姿态，航向做 Scan Context 和整圈扇区搜索，再 small-gicp 配到地图
4. 起点数值可手动修正；「换一帧」只换显示点云，数值保持第一帧
5. 确认每颗雷达起点（「保存起点 / 恢复起点」可临时记录）
6. 基准达到 RTK 条件时按钮是「开始逐帧优化」（基准逐帧配地图、其余每 5 帧配一次，再标定 RTK）；否则是「相对外参标定」（只做雷达间相对外参）

网页显示：白点 = 地图，橙线 = RTK，彩色点 = 各雷达。拖动旋转、滚轮缩放、Shift 或右键平移。

## 多点云可视化流程（play_rtk_view.sh）

1. 网页填写 mcap、外参 yaml；地图可选（`--map ""` 不加载）
2. 路径不在当前挂载里时，点「按当前路径挂载」或直接「读取话题」（容器自动挂载对应目录并重启；白名单由脚本的 `--allow` 控制）
3. 「读取话题」列出点云话题，按 `frame_id` 预选外参，可改、可留空
4. 「坐标变换并播放」：每帧用该时刻 RTK 位姿 × 外参变到地图坐标系；可暂停、拖动进度、改速度

## 端口分配

| 端口 | 用途 |
|---|---|
| 8765 | 相对外参微调网页（play_extrinsic_web） |
| 8766 | 地图标定网页（play_map_calib） |
| 8767 | 标定结果复制到剪贴板辅助服务 |
| 8768 | RTK 可视化网页（play_rtk_view） |
| 8769 | 可视化容器的挂载控制口（rtk_view_host） |

## 目录结构

```
sutpc_calib/
├── play_map_calib.sh        # 入口：地图标定（RViz2 + 8766）
├── play_rtk_view.sh         # 入口：多点云 RTK 可视化（8768）
├── play_extrinsic_web.sh    # 入口：相对外参微调（8765）
├── rtk_view_host.py         # 可视化容器的宿主机守护进程（挂载管理）
├── copy_calib_clipboard.py  # 标定结果复制到剪贴板（8767）
├── long_pair_calib.py       # 标定核心库（被 map_calib.py / 试验脚本 import）
├── web_calib/               # Flask 后端 + three.js 前端
│   ├── app.py               #   相对外参网页服务
│   ├── map_calib.py         #   地图标定服务
│   ├── rtk_view.py          #   RTK 可视化服务
│   ├── templates/           #   三个页面 HTML
│   └── static/              #   JS（含共用 three.js 渲染器 view.js）与 CSS
├── docker/Dockerfile        # ros2-jazzy:dev 镜像构建
├── docs/lidar_commands.md   # 整车多雷达流程文档（录制/连车等，车端脚本不在本仓库）
├── examples/                # 外参 yaml 示例
└── *.py（其余）             # 算法试验脚本，见下
```

## 算法试验脚本

标定流程的单步原型，在容器内运行（需要 `small-gicp` 等依赖）：

```bash
docker run --rm -v "$PWD:/calib" -w /calib \
  -e PYTHONPATH=/calib:/opt/calib/lib/python3.12/site-packages \
  ros2-jazzy:dev bash -lc 'source /opt/ros/jazzy/setup.bash && python3 test_kiss_prior.py --help'
```

| 脚本 | 用途 |
|---|---|
| `test_kiss_prior.py` / `test_ring_prior.py` | KISS-ICP 里程计先验 / 环带先验试验 |
| `match_rtk_prior.py` | RTK 先验匹配 |
| `match_rtk_no_extrinsic.py` / `match_rtk_no_extrinsic_right.py` | 无外参时雷达到地图匹配 |
| `relocalize_scancontext.py` | Scan Context 重定位 |
| `refine_blind_first_frame.py` / `refine_blind_on_rtk.py` | 补盲雷达首帧 / RTK 精化 |
| `rebuild_rtk_pcd.py` | 用 RTK 轨迹重建点云 |

## 注意事项

- 所有脚本都依赖镜像名 `ros2-jazzy:dev`，可用环境变量 `ROS_IMAGE` 覆盖
- 关闭启动脚本的终端即停止容器并卸载挂载；标定结果在关闭前写入 `--output` 指定路径
- 数据路径白名单：可视化容器只接受启动脚本 `--allow` 指定的目录（默认 `/home/zzl`、`/media/zzl`）
- 完整的整车流程（有线连车、硬盘管理、20 秒录制脚本）见 `docs/lidar_commands.md`，那些车端/宿主机脚本不在本仓库
