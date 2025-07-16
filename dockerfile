FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

# 기본 패키지 설치
RUN apt update && apt install -y \
    git wget curl lsb-release gnupg2 sudo \
    python3-colcon-common-extensions python3-vcstool python3-pip \
    build-essential cmake \
    pkg-config \
    libgoogle-glog-dev \
    libprotobuf-dev protobuf-compiler \
    libceres-dev \
    libboost-all-dev \
    libeigen3-dev \
    liblua5.3-dev \
    libcairo2-dev \
    libpcl-dev \
    && rm -rf /var/lib/apt/lists/*

# GPU 런타임 설정 (NVIDIA)
RUN apt update && apt install -y nvidia-cuda-toolkit

# 워크스페이스 생성
WORKDIR /root/cartographer_ws/src

# cartographer_ros 및 cartographer 설치
RUN git clone https://github.com/cartographer-project/cartographer.git
RUN git clone -b ros2 https://github.com/ros2/cartographer_ros.git

# 종속성 설치
WORKDIR /root/cartographer_ws
# RUN vcs import src < src/cartographer_ros/cartographer_ros.repos

# rosdep 초기화 및 설치
RUN rosdep update && rosdep install --from-paths src --ignore-src -r -y

# 빌드
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install

# 엔트리포인트
SHELL ["/bin/bash", "-c"]
CMD ["source /opt/ros/humble/setup.bash && source /root/cartographer_ws/install/setup.bash && bash"]
