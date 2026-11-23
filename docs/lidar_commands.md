# 多激光雷达命令

本机有线 `enp46s0` / `192.168.3.211`，车端 `nvidia@192.168.3.50`。仿真车端用下面的脚本指定 IP，只把该地址指到 `enp46s0`。可视化在本机 Docker 镜像 `ros2-jazzy:dev` 里完成，不使用宿主机网络。外参相对原始 `base_link`，单位米和弧度，RPY 为固定轴 XYZ，x 前、y 左、z 上。参考雷达是 `front_mid_top`，它的安装外参保持不变。RTK 天线写在 `gnss_base_link`。

当前准确外参：`/home/zzl/bag/lidar_extrinsic_accurate.yaml`

## 仿真车端有线连接

按车端 IP 配置有线。本机在同一网段使用 `/32` 地址，默认取该网段的 `.211`（车端正好是 `.211` 时用 `.210`）。连接名是 `enp46s0-sim`，只为这个 IP 增加有线路由，不改 Wi-Fi 默认路由。`ssh nvidia@<车端IP>` 绑定有线源地址。未配置密钥时，SSH 会提示输入密码。

```bash
/home/zzl/bag/setup_wired_vehicle.sh 192.168.1.216
```

指定本机地址时：

```bash
/home/zzl/bag/setup_wired_vehicle.sh 192.168.1.216 --local 192.168.1.211
```

## 有线连接检测

```bash
/home/zzl/bag/check_vehicle_link.sh
```

依次检查网卡地址、到车端的路由是否走 `enp46s0`、有线 ping，以及 SSH 后车端看到的来源是否为有线地址。任一项失败即退出。未配置密钥时，SSH 会提示输入车端密码。检查仿真车端时用 `HOST=nvidia@192.168.1.216 BIND_IP=192.168.1.211 /home/zzl/bag/check_vehicle_link.sh`。

## 外置硬盘

约 1TB 的 USB 硬盘优先作为录制目录。同一脚本在三处：

| 位置 | 路径 |
|---|---|
| 本机 | `/home/zzl/bag/disk_manage.sh` |
| 车端 | `/home/nvidia/disk_manage.sh` |
| 硬盘 | `/mnt/usb-1t/disk_manage.sh` |

`status` 查询硬盘和挂载情况，`mount` 自动挂到 `/mnt/usb-1t`，`umount` 同步后卸载并断电，之后可以拔出。

```bash
/home/zzl/bag/disk_manage.sh status
ssh nvidia@192.168.3.50 'bash /home/nvidia/disk_manage.sh status'
ssh nvidia@192.168.3.50 'bash /home/nvidia/disk_manage.sh mount'
```

## 录制

在终端中运行。话题加载完成后屏幕显示「加载完成」，按空格开始，再按空格停止，最长 40 秒。车端有外置硬盘时先自动挂载，数据写入 `/mnt/usb-1t/bag/`；没有硬盘时写入 `/home/nvidia/bag/`。包再拉回 `/home/zzl/bag/`。录制结束后按 `u` 安全卸载并可以拔出硬盘，按其他键保持挂载以便继续录制。

```bash
/home/zzl/bag/record_lidar_gnss_20s.sh
```

## 多点云可视化

在本机 Docker 镜像 `ros2-jazzy:dev` 里按外参和 RTK 轨迹播放多路点云。浏览器打开 http://127.0.0.1:8768 。关闭运行脚本的终端即停止容器；路径在 USB 上时，容器对这块盘的挂载也一起卸掉。

默认数据包是 `/home/zzl/bag/apolong_calib/lidar_gnss_20261116_160451_0.mcap`，默认外参是 `/home/zzl/SUTPC/产出/apolong/calib/sensors_calibration.yaml`，默认地图是 `/home/zzl/bag/apolong_calib/20261113150315_0_voxel0.30.pcd`。`--map ""` 不加载地图。

```bash
/home/zzl/SUTPC/产出/apolong/calib/play_rtk_view.sh

/home/zzl/SUTPC/产出/apolong/calib/play_rtk_view.sh \
  --mcap /home/zzl/bag/lidar_gnss_20s_20261110_150904/lidar_gnss_20s_20261110_150904_0.mcap \
  --yaml /home/zzl/SUTPC/产出/chengjiaolong/calib/lidar_extrinsic_accurate.yaml

/home/zzl/SUTPC/产出/apolong/calib/play_rtk_view.sh --map ""
```

网页上的顺序：

1. 填写 mcap，以及 `sensors_calibration.yaml` 这种外参。键名是 `<雷达 frame_id>_base_link`，单位 m / rad，RPY 为固定轴 XYZ。父坐标系是 RTK 后轴中心，高度用文件里的实测值。
2. 地图可选。带 `map_projector_info.yaml` 和 `pointcloud_map.pcd` 的目录按经纬度做横轴墨卡托。坐标已经是 UTM 等投影坐标的 pcd 直接叠加。
3. 路径不在当前挂载里时，点「按当前路径挂载」，或直接点「读取话题」。容器会挂上这些文件所在的目录并重启。只接受 `/home/zzl` 和 `/media/zzl` 下的路径。
4. 「读取话题」列出点云话题。按 `frame_id` 预选外参名，可以改，留空则不播放。
5. 「坐标变换并播放」。每一帧用该时刻的 RTK 位姿乘上所选外参，变到地图坐标。白点是地图，橙线是 RTK，彩色点是各雷达。可暂停、拖动进度和改速度。拖动旋转，滚轮缩放，Shift 或右键平移。

## 带地图和起点的标定

当前标定是 `/home/zzl/SUTPC/产出/apolong/calib/play_map_calib.sh`。在本机 Docker 镜像 `ros2-jazzy:dev` 里打开 RViz2 和网页 http://127.0.0.1:8766 。关闭 RViz 即停止网页和容器。初始外参可选，格式同 `sensors_calibration.yaml`。不填时位置先验只用这一包的 RTK。填写后，有外参的雷达用该时刻 RTK 位姿乘上外参作为起点，再做 GICP。

默认输入在 `/home/zzl/bag/apolong_calib/`。`--mcap`、`--map` 和 `--yaml` 只预填网页，打开后仍可改。地图是 pcd，或含 `pointcloud_map.pcd` 的目录。结果默认写到 `/home/zzl/SUTPC/产出/apolong/calib/map_calib_apolong.yaml`。

```bash
/home/zzl/SUTPC/产出/apolong/calib/play_map_calib.sh

/home/zzl/SUTPC/产出/apolong/calib/play_map_calib.sh \
  --mcap /home/zzl/bag/apolong_calib/lidar_gnss_20261116_160451_0.mcap \
  --map /home/zzl/bag/apolong_calib/20261113150315_0_voxel0.30.pcd \
  --output /home/zzl/SUTPC/产出/apolong/calib/map_calib_apolong.yaml
```

网页上的顺序：

1. 选择地图和 mcap。初始外参可不填。点「读取」。
2. 勾选最多 7 颗雷达，指定一颗基准。若填了初始外参，每个话题按实际 `frame_id` 和文件里的雷达名预选外参，下拉框可以改成别的名称或「不使用」。改路径后点「匹配外参」再匹配一次。
3. 「配准起点」。有外参的雷达用 RTK 乘上该外参再 GICP。没有外参的雷达仍由 RTK 给出位置和车体姿态，航向做 Scan Context 和整圈扇区搜索，再 small-gicp 配到地图。起点数值是第一帧在地图坐标系下的雷达位姿。
4. 数值可改。页面角度是度，滚轮微调；Shift 更细，Ctrl 更粗。yaml 仍写弧度。
5. 「换一帧」只换显示的点云，换成下一帧与 RTK 同步的扫描。起点数值保持第一帧，点云按 RTK 的相对运动显示。非基准雷达可用「显示 / 不显示」开关点云。
6. 确认每一颗雷达的起点。同一组地图和 mcap 可用「保存起点 / 恢复起点」临时记下这些起点。
7. 基准达到 RTK 条件时，按钮是「开始逐帧优化」：基准雷达逐帧配地图，其余雷达每 5 帧配一次，再用这条轨迹标定 RTK。未达到时，按钮是「相对外参标定」，只做雷达之间的相对外参。

网页白点是地图，橙线是 RTK，彩色点是雷达。拖动旋转，滚轮缩放，Shift 或右键平移。左栏宽度和左下输出高度可拖。标定结果在单独弹窗里显示完整的 x、y、z、roll、pitch、yaw、帧数和离散度，角度为度，点击数值可复制。

写入 yaml 时，基准雷达相对 baseline 的高度固定为 1.0 m，其余雷达相对基准的位置不变。RTK 那一段仍是测到的高度。页面上的起点和配准过程不改这个高度。
