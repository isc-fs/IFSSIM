FROM ros:humble-ros-base

# System packages
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-tf2-ros \
    ros-humble-tf2-geometry-msgs \
    ros-humble-sensor-msgs \
    ros-humble-nav-msgs \
    clang-12 clang++-12 libc++-12-dev libc++abi-12-dev \
    ros-humble-foxglove-bridge \
  && rm -rf /var/lib/apt/lists/*

# Python deps — pin numpy<2 (cv_bridge in ros:humble compiled against 1.x)
RUN pip install --no-cache-dir "numpy==1.24.*"
RUN pip install --no-cache-dir \
    "numba>=0.59" \
    opencv-python-headless \
    scipy \
    scikit-learn \
    transforms3d \
    matplotlib \
    icecream \
    typing_extensions
# Re-pin numpy after all installs to prevent any dependency from upgrading it
RUN pip install --no-cache-dir "numpy==1.24.*"


# === GLIM (LiDAR-IMU SLAM) build chain ===
# Built from source, CPU-only (no CUDA — matches real-car compute envelope).
# Uses GLIM's OdometryEstimationCPU module. Heavy layers (GTSAM, gtsam_points)
# go before any pipeline COPY so they cache across pipeline-source changes.
# See docs/glim_integration.md for the full design rationale.

# C++ build deps for GTSAM + gtsam_points + glim
RUN apt-get update && apt-get install -y \
      libboost-all-dev libtbb-dev libgoogle-glog-dev \
      libsuitesparse-dev libeigen3-dev libnanoflann-dev \
      libfmt-dev libspdlog-dev \
      cmake build-essential git \
  && rm -rf /var/lib/apt/lists/*

# GTSAM 4.3a0 (gtsam_points 1.2.0 supports both 4.2a9 and 4.3a0; newer pin)
RUN cd /tmp && git clone --depth 1 --branch 4.3a0 \
      https://github.com/borglab/gtsam.git && \
    cd gtsam && mkdir build && cd build && \
    cmake -DGTSAM_BUILD_PYTHON=OFF \
          -DGTSAM_USE_SYSTEM_EIGEN=ON \
          -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
          -DGTSAM_BUILD_TESTS=OFF \
          -DGTSAM_BUILD_UNSTABLE=ON \
          -DGTSAM_WITH_TBB=ON \
          -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/gtsam

# gtsam_points 1.2.0 (CPU only)
RUN cd /tmp && git clone --depth 1 --branch v1.2.0 \
      https://github.com/koide3/gtsam_points.git && \
    cd gtsam_points && mkdir build && cd build && \
    cmake -DBUILD_DEMO=OFF \
          -DBUILD_WITH_CUDA=OFF \
          -DCMAKE_BUILD_TYPE=Release .. && \
    make -j$(nproc) && make install && ldconfig && \
    rm -rf /tmp/gtsam_points

WORKDIR /dv_pipeline_stack_ws

# Clone GLIM core + ROS 2 wrapper into the colcon workspace src/.
# The existing `colcon build` step below picks them up alongside our packages.
RUN mkdir -p src && cd src && \
    git clone --depth 1 https://github.com/koide3/glim.git && \
    git clone --depth 1 https://github.com/koide3/glim_ros2.git

# Copy bridge packages (fs_msgs + ifssim_bridge)
COPY ros2/src/ src/

# Copy pipeline packages (slam, odometria, path_planning, control)
COPY pipeline/ src/

# Install fsd_path_planning (bundled inside path_planning package)
RUN pip install --no-cache-dir -e src/path_planning/
# Expose fsd_path_planning as a top-level module (it lives inside path_planning/)
RUN ln -s /dv_pipeline_stack_ws/src/path_planning/path_planning/fsd_path_planning \
          /usr/local/lib/python3.10/dist-packages/fsd_path_planning

# Build everything. GLIM's CMakeLists has BUILD_WITH_VIEWER=ON by default
# which pulls in Iridescence (koide3's 3D viewer) — not available in this
# headless container. Disable per-package via colcon's --cmake-args; the
# core SLAM compiles fine without the viewer modules.
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install \
    --cmake-args -DBUILD_WITH_VIEWER=OFF

# Copy launch files, teleop, and entrypoint
COPY docker/dv_pipeline_stack/bridge.launch.py        /dv_pipeline_stack_ws/bridge.launch.py
COPY docker/dv_pipeline_stack/pipeline.launch.py      /dv_pipeline_stack_ws/pipeline.launch.py
COPY docker/dv_pipeline_stack/pipeline_only.launch.py /dv_pipeline_stack_ws/pipeline_only.launch.py
COPY docker/dv_pipeline_stack/teleop_keyboard.py /dv_pipeline_stack_ws/teleop_keyboard.py
COPY docker/dv_pipeline_stack/teleop.sh          /dv_pipeline_stack_ws/teleop.sh
COPY docker/dv_pipeline_stack/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /dv_pipeline_stack_ws/teleop.sh

# UDP sensor ports (UE5 pushes to these)
EXPOSE 41452/udp
EXPOSE 41453/udp
# rosboard web UI
EXPOSE 8888

ENV IFSSIM_HOST=host.docker.internal
ENV IFSSIM_PORT=41451
ENV MISSION_NAME=trackdrive
ENV TRACK_NAME=A
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/entrypoint.sh"]
