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

WORKDIR /ros_stack_ws

# Copy bridge packages (fs_msgs + ifssim_bridge)
COPY ros2/src/ src/

# Copy pipeline packages (slam, odometria, path_planning, control, coche_urdf)
COPY pipeline/ src/

# Install fsd_path_planning (bundled inside path_planning package)
RUN pip install --no-cache-dir -e src/path_planning/
# Expose fsd_path_planning as a top-level module (it lives inside path_planning/)
RUN ln -s /ros_stack_ws/src/path_planning/path_planning/fsd_path_planning \
          /usr/local/lib/python3.10/dist-packages/fsd_path_planning

# Build everything
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Copy launch files, teleop, and entrypoint
COPY docker/ros_stack/bridge.launch.py   /ros_stack_ws/bridge.launch.py
COPY docker/ros_stack/pipeline.launch.py /ros_stack_ws/pipeline.launch.py
COPY docker/ros_stack/teleop_keyboard.py /ros_stack_ws/teleop_keyboard.py
COPY docker/ros_stack/teleop.sh          /ros_stack_ws/teleop.sh
COPY docker/ros_stack/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /ros_stack_ws/teleop.sh

# UDP sensor ports (UE5 pushes to these)
EXPOSE 41452/udp
EXPOSE 41453/udp

ENV IFSSIM_HOST=host.docker.internal
ENV IFSSIM_PORT=41451
ENV MISSION_NAME=trackdrive
ENV TRACK_NAME=A
ENV ROS_DOMAIN_ID=0

ENTRYPOINT ["/entrypoint.sh"]
