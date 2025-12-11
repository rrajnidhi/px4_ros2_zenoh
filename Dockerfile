# -------------------------------------------------------------------------
# ROS2 Humble desktop-full base with PX4 v1.15.2 + Gazebo Garden + rmw_zenoh
# -------------------------------------------------------------------------

FROM osrf/ros:humble-desktop-full

LABEL maintainer="Nidhi Raj <nidhirajr@gmail.com>"
LABEL description="ROS2 Humble + PX4 v1.15.2 + Gazebo Garden + rmw_zenoh + PX4-ROS2 Bridge"

# ----------------------------
# Environment setup
# ----------------------------
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    ROS_DOMAIN_ID=3 \
    RMW_IMPLEMENTATION=rmw_zenoh_cpp \
    PX4_VERSION=v1.15.2

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# ----------------------------
# Base dependencies (non-ROS)
# ----------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
        tzdata locales lsb-release gnupg2 software-properties-common \
        curl wget git sudo nano vim tmux gdb openjdk-11-jdk ruby-full tree \
        build-essential cmake ninja-build clang lldb python3-dev python3-pip python3-venv \
        python3-setuptools python3-wheel libgtest-dev libeigen3-dev libyaml-dev \
        libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good gstreamer1.0-tools \
        zip qtcreator genromfs exiftool python3-jinja2 python3-empy python3-toml python3-numpy python3-yaml \
        python3-pygments libxml2-utils clang-format clang-tidy lcov pkg-config uuid-dev \
        libopencv-dev libopencv-core-dev libopencv-imgproc-dev libopencv-highgui-dev libopencv-videoio-dev \
        libopencv-imgcodecs-dev libopencv-calib3d-dev libopencv-features2d-dev libopencv-video-dev \
        libopencv-objdetect-dev \
        ros-humble-rqt ros-humble-rqt-common-plugins ros-humble-rqt-image-view \
    && locale-gen en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ----------------------------
# Gazebo Garden
# ----------------------------
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg | tee /usr/share/keyrings/gazebo-archive-keyring.gpg > /dev/null && \
    echo "deb [arch=amd64 signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y --no-install-recommends gz-garden && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# ----------------------------
# ROS-GZ bridge + Zenoh RMW
# ----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-gz \
        ros-humble-ros-gz-bridge \
        ros-humble-ros-gz-image \
        ros-humble-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*

# ----------------------------
# PX4 dependencies and Python packages
# ----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        protobuf-compiler libprotobuf-dev libzmq3-dev python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages system-wide
RUN pip3 install --no-cache-dir \
        future jsonschema lxml toml jinja2 pygments numpy cppy \
        symforce || true \
        pyserial empy pandas pyros-genmsg \
        pyquaternion packaging pyproj ultralytics opencv-python pygame aioconsole mavsdk \
        kconfiglib
RUN python3 -m pip install --upgrade "pip<23.0" setuptools==58.2.0

# Ensure kconfiglib and pyros-genmsg are system-wide and visible
RUN python3 -m pip install --no-cache-dir --upgrade kconfiglib pyros-genmsg

# ----------------------------
# PX4-Autopilot v1.15.2
# ----------------------------
WORKDIR /app
RUN git clone --recursive https://github.com/PX4/PX4-Autopilot.git -b ${PX4_VERSION} /app/PX4-Autopilot

WORKDIR /app/PX4-Autopilot
RUN git submodule update --init --recursive || true

# Build PX4 SITL with Zenoh
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && make px4_sitl_zenoh -j$(nproc)"
RUN mkdir -p /app/PX4-Autopilot/build/px4_sitl_zenoh/px4_zenoh_pico_fs/zenoh

# ----------------------------
# PX4 ROS2 workspace
# ----------------------------
WORKDIR /app/ros2_ws/src
RUN git clone https://github.com/PX4/px4_msgs.git -b release/1.15 && \
    git clone https://github.com/PX4/px4_ros_com.git -b release/1.15


WORKDIR /app/ros2_ws
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"
# ----------------------------
# tmuxinator and Gazebo models/worlds
# ----------------------------
RUN gem install tmuxinator || true
COPY launch_files/single_drone_sitl.sh /app/launch_files/
COPY launch_files/camera_detection.sh /app/launch_files/
COPY launch_files/for_multi_uav_spawn.sh /app/launch_files/
RUN mkdir -p /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY /resources/simulation/models/. /root/.gz/models/
COPY /resources/simulation/models_docker/. /root/.gz/fuel/fuel.ignitionrobotics.org/openrobotics/models/
COPY /resources/simulation/worlds/default_docker.sdf /app/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf

ENV GZ_SIM_RESOURCE_PATH=/app/PX4-Autopilot/Tools/simulation/gz:/root/.gz/models
ENV PX4_GZ_MODEL_PATH=/app/PX4-Autopilot/Tools/simulation/gz/models:/root/.gz/models

# GPU-related groups
RUN groupadd -r render || true && groupadd -r video || true

# ----------------------------
# Entrypoint
# ----------------------------
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]

