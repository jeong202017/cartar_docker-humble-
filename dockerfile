FROM ros:humble

# 기본 패키지 설치
RUN apt update && apt install -y \
    sudo git wget curl lsb-release gnupg2 \
    build-essential cmake g++ \
    python3-colcon-common-extensions \
    python3-vcstool \
    libboost-all-dev \
    libceres-dev \
    liblua5.3-dev \
    libprotobuf-dev protobuf-compiler \
    libcairo2-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libeigen3-dev \
    pkg-config \
    python3-pip \
    ros-humble-tf2-eigen \
    ros-humble-pcl-conversions \
    ros-humble-pcl-msgs \
    ros-humble-nav-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-sensor-msgs \
    ros-humble-tf2-msgs \
    ros-humble-urdf \
    ros-humble-visualization-msgs \
    ros-humble-rosbag2-storage \
    ros-humble-rosbag2-cpp \
    && rm -rf /var/lib/apt/lists/*

# ROS 환경 설정
SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

# 작업공간 생성
WORKDIR /root/cartographer_ws/src

# Cartographer ROS2 설치 (예시)
RUN git clone https://github.com/cartographer-project/cartographer.git --branch master
RUN git clone https://github.com/ros2/cartographer_ros.git --branch ros2

# Install dependencies (ROS2 way)
WORKDIR /root/cartographer_ws
RUN vcs import src < src/cartographer_ros/cartographer_ros.repos

# 빌드
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# 기본 명령어
CMD ["/bin/bash"]
