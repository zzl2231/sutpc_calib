#!/usr/bin/env python3
"""远距右后：RTK 先验不乘外参，先搜索航向再 small-gicp。"""

import numpy as np

import match_rtk_no_extrinsic as runner

runner.base.TOPIC = "/sutpc/sensor_driver/lidar/long/right_end/lidar_points"
runner.EXTRINSIC = np.array(
    [-0.2831986909, -0.8307720915, 1.2646607129, -0.0048743213, 0.0077573353, -0.8901263806]
)

if __name__ == "__main__":
    print("lidar=long_right_end", flush=True)
    runner.main()
