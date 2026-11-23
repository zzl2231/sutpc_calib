FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

# 基础工具
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl \
        ca-certificates \
        gnupg2 \
        lsb-release \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 官方 apt 源 + GPG key
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list

# ROS 2 Jazzy 桌面版 + 开发工具链
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-desktop \
        ros-dev-tools \
        build-essential \
        cmake \
        git \
        wget \
        vim \
        nano \
        htop \
        tmux \
        tree \
        iputils-ping \
        net-tools \
        bash-completion \
        python3-pip \
        python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

# 创建与宿主同 UID 的用户(避免挂载目录时的权限问题),并自动 source ROS 2 环境
ARG USERNAME=zzl
ARG USER_UID=1000
ARG USER_GID=1000
# ubuntu:24.04 镜像自带 UID/GID 为 1000 的 ubuntu 用户,重用它而不是新建
RUN set -eux; \
    if getent passwd "$USER_UID" >/dev/null; then \
        old_user=$(getent passwd "$USER_UID" | cut -d: -f1); \
        old_group=$(id -gn "$old_user"); \
        usermod --login "$USERNAME" --home "/home/$USERNAME" --move-home "$old_user"; \
        groupmod --new-name "$USERNAME" "$old_group"; \
    else \
        groupadd --gid "$USER_GID" "$USERNAME"; \
        useradd --uid "$USER_UID" --gid "$USER_GID" -m "$USERNAME"; \
    fi; \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/$USERNAME"; \
    chmod 0440 "/etc/sudoers.d/$USERNAME"; \
    echo "source /opt/ros/jazzy/setup.bash" >> "/home/$USERNAME/.bashrc"; \
    echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> "/home/$USERNAME/.bashrc"; \
    echo "export PATH=\"\$PATH:/opt/calib/bin\"" >> "/home/$USERNAME/.bashrc"; \
    mkdir -p "/home/$USERNAME/ros2_ws/src"; \
    chown -R "$USER_UID":"$USER_GID" "/home/$USERNAME"

# 离线多雷达 + RTK 标定。
# iKalibr 只支持 ROS 1 Noetic / Ubuntu 20.04，不装进本镜像。
# 轨迹用 KISS-ICP，雷达到 RTK 的外参用 OpenCV calibrateHandEye，
# 经纬度转 ENU 用 pyproj / GeographicLib。
RUN apt-get update && apt-get install -y --no-install-recommends \
        libeigen3-dev \
        libceres-dev \
        libpcl-dev \
        libgoogle-glog-dev \
        libgflags-dev \
        libatlas-base-dev \
        libsuitesparse-dev \
        libyaml-cpp-dev \
        libcereal-dev \
        libgeographiclib-dev \
        libopencv-dev \
        libtbb-dev \
        pybind11-dev \
        ninja-build \
        python3-venv \
        python3-numpy \
        python3-scipy \
        python3-matplotlib \
        ros-jazzy-rosbag2-storage-mcap \
        ros-jazzy-pcl-ros \
        ros-jazzy-pcl-conversions \
        ros-jazzy-nmea-msgs \
        ros-jazzy-geographic-msgs \
        ros-jazzy-geodesy \
        ros-jazzy-tf2-tools \
    && python3 -m venv /opt/calib \
    && /opt/calib/bin/pip install --upgrade pip \
    && /opt/calib/bin/pip install --no-cache-dir \
        "kiss-icp[all]" \
        "opencv-python-headless" \
        small-gicp \
        evo \
        pyproj \
    && rm -rf /var/lib/apt/lists/*

# GUI 程序(RViz / rqt)需要的运行时目录
ENV XDG_RUNTIME_DIR=/tmp/runtime-zzl
RUN mkdir -p /tmp/runtime-zzl && chmod 700 /tmp/runtime-zzl \
    && chown "$USER_UID":"$USER_GID" /tmp/runtime-zzl

WORKDIR /home/$USERNAME/ros2_ws
USER $USERNAME

CMD ["bash"]
