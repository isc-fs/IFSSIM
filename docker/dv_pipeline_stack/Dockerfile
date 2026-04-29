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
    ros-humble-rosbag2-storage-mcap \
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

# GTSAM Python bindings — backbone of the cone-graph SLAM (PR A+).
# Pinned to 4.3a0. The previous "4.2.0" pin was a phantom (never
# released; only 4.2a1–4.2a9 and 4.3a0 actually shipped to PyPI), so
# any fresh image build was guaranteed to fail. 4.2a9 was the obvious
# fallback but ships no aarch64 wheel — only manylinux2014_x86_64 —
# which breaks the build on Apple Silicon hosts. 4.3a0 is the only
# remaining version with both x86_64 and aarch64 wheels for cp310.
# factor_graph.py uses the "4.2.x mixed API" (setRelinearizeThreshold
# method + relinearizeSkip property); both still work in 4.3a0 per
# upstream's compatibility shim.
RUN pip install --no-cache-dir "gtsam==4.3a0"

# Re-pin numpy after all installs to prevent any dependency from upgrading it
RUN pip install --no-cache-dir "numpy==1.24.*"


# === Build deps for ifssim_bridge (C++) ===
# cone_slam is pure Python (gtsam wheel, numpy, scipy) so the only
# native build is the bridge. cmake + build-essential + git are enough.
# Earlier revisions of this Dockerfile carried PCL/Eigen/fmt for the
# fast_LIMO C++ build; that toolchain has been dropped along with all
# legacy SLAM approaches (GLIM → FAST_LIO2 → fast_LIMO) which never
# tracked our cone-only LiDAR scenes cleanly. cone_slam (PR A+) is the
# single SLAM path the stack ships with.
RUN apt-get update && apt-get install -y \
      cmake build-essential git \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /dv_pipeline_stack_ws

# Copy bridge packages (fs_msgs + ifssim_bridge)
RUN mkdir -p src
COPY ros2/src/ src/

# Copy pipeline packages (slam, cone_slam, path_planning, control)
COPY pipeline/ src/

# Build everything. With fast_LIMO removed, this is just the bridge
# C++ + a handful of Python packages (--symlink-install means edits
# under src/ are live without re-running colcon).
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

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
