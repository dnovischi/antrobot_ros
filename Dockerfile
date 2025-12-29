FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

RUN apt-get update \
 && apt-get upgrade -y \
 && rm -rf /var/lib/apt/lists/*

RUN mkdir -p src
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY . ./src/antrobot_ros
WORKDIR /ros2_ws/src/antrobot_ros
RUN rosdep init 2>/dev/null || true \
 && chmod +x setup_source_dependecies.sh \
 && apt-get update \
 && bash -lc 'source /opt/ros/humble/setup.bash && ./setup_source_dependecies.sh' \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
RUN bash -lc 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "antrobot_ros", "antrobot.launch.py"]