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

# FaSTTUBe Formula Student Driverless path planner (PR #243).
# Installed --no-deps because the package declares numpy>=1.25 while
# cv_bridge in ros:humble is ABI-locked to numpy 1.x — all of papalotis's
# real runtime deps (numba, scipy, scikit-learn, icecream, typing_extensions,
# numpy) are already satisfied by the block above. Pinned to a specific
# commit SHA: the only git tags upstream ships are "summer-23" and
# "color-dependent" (legacy), which don't track v0.4.x.
#
# pip clones the repo shallow, which breaks setuptools-scm's git-tag
# version derivation. setuptools-scm is listed in build-system.requires
# but the project has explicit `name`/`version` in [project] — the
# combination causes setuptools 64+ to auto-invoke setuptools-scm and
# override the static metadata, producing project name "unknown" and
# pip rejecting the install. SETUPTOOLS_SCM_PRETEND_VERSION env var
# alone doesn't fix it. Workaround: clone, sed out the setuptools-scm
# line from build-system.requires so setuptools doesn't auto-invoke it,
# then pip install from the local path.
# Install the FaSTTUBe planner. The upstream pyproject.toml is unhappy
# under our build environment in multiple ways (setuptools-scm in
# build-requires overriding the static [project] version, and `packages
# = ["fsd_path_planning"]` discovering nothing under PEP 621). Rather
# than fight pip's build isolation, clone the repo and write our own
# minimal setup.py that uses find_packages() to pick up the whole
# fsd_path_planning/ tree, then install with the system pip — no
# setuptools-scm, no PEP 621 corner cases.
RUN git clone --depth 1 https://github.com/papalotis/ft-fsd-path-planning.git /tmp/fsd && \
    cd /tmp/fsd && \
    git fetch --depth 1 origin 0283c83125d1ad5d008d80b62f8750dad5f28fb3 && \
    git checkout 0283c83125d1ad5d008d80b62f8750dad5f28fb3 && \
    rm -f pyproject.toml && \
    printf 'from setuptools import find_namespace_packages, setup\nsetup(name="fsd_path_planning", version="0.4.3.2", packages=find_namespace_packages(include=["fsd_path_planning", "fsd_path_planning.*"]), include_package_data=True)\n' > setup.py && \
    pip install --no-cache-dir --no-deps . && \
    rm -rf /tmp/fsd

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


# === Build deps for ifssim_bridge + LIMOncello (C++) ===
# ifssim_bridge needs: cmake, build-essential, git.
#
# LIMOncello (the LiDAR-Inertial SLAM at ros2/src/limoncello) needs:
#   - libeigen3-dev          header-only linear algebra
#   - libtbb-dev             oneTBB; LIMOncello uses
#                            `std::execution::par_unseq` in the
#                            point-cloud processing path
#   - libboost-system-dev    LIMOncello CMakeLists asks for
#                            Boost.system COMPONENTS
#   - ros-humble-pcl-conversions  sensor_msgs ↔ pcl::PointCloud glue
#
# Livox dependencies (livox_interfaces / livox_ros_driver /
# livox_ros_driver2) are NOT installed — our LiDAR is type=2 (Hesai),
# so the Livox CustomMsg code paths (type=4/5/6) are unreachable. The
# submodule points at our fork (isc-fs/LIMOncello, branch ifssim-main)
# where the Livox find_package calls are gated behind the CMake
# option `LIMONCELLO_BUILD_LIVOX` (default OFF). Anyone who wants
# Livox flips the option on and installs the three packages
# separately.
RUN apt-get update && apt-get install -y \
      cmake build-essential git \
      libeigen3-dev libtbb-dev libboost-system-dev \
      ros-humble-pcl-conversions \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /dv_pipeline_stack_ws

# Copy bridge packages (fs_msgs + ifssim_bridge + limoncello submodule).
RUN mkdir -p src
COPY ros2/src/ src/

# Copy pipeline packages (slam, cone_slam, path_planning, control, …)
COPY pipeline/ src/

# Build everything. LIMONCELLO_BUILD_LIVOX stays OFF (the default in
# our fork's CMakeLists.txt) so the build skips the 3 Livox
# find_package calls.
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

# Copy launch files, teleop, entrypoint, and DDS profile.
# fastdds_profile.xml used to be bind-mounted at runtime so we could
# tweak DDS settings without rebuilding the image; #490 moved it into
# the image to eliminate one more bind mount (Docker Desktop's
# virtiofs/9p makes mount overhead non-trivial). To iterate on DDS
# settings now, edit the file and `docker compose build`.
COPY docker/dv_pipeline_stack/fastdds_profile.xml     /dv_pipeline_stack_ws/fastdds_profile.xml
COPY docker/dv_pipeline_stack/bridge.launch.py        /dv_pipeline_stack_ws/bridge.launch.py
COPY docker/dv_pipeline_stack/pipeline.launch.py      /dv_pipeline_stack_ws/pipeline.launch.py
COPY docker/dv_pipeline_stack/pipeline_only.launch.py /dv_pipeline_stack_ws/pipeline_only.launch.py
COPY docker/dv_pipeline_stack/teleop_keyboard.py /dv_pipeline_stack_ws/teleop_keyboard.py
COPY docker/dv_pipeline_stack/teleop.sh          /dv_pipeline_stack_ws/teleop.sh
COPY docker/dv_pipeline_stack/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh /dv_pipeline_stack_ws/teleop.sh

# LIMOncello smoke-test tooling. Currently just the bag rewriter
# that adds the per-point `timestamp` field to pre-#497 bags so they
# can be replayed through LIMOncello. Lives under /tools/limoncello/
# inside the container; ROS Python bindings (rclpy + rosbag2_py +
# sensor_msgs) are already available from the base image.
COPY tools/limoncello /tools/limoncello
RUN chmod +x /tools/limoncello/*.py 2>/dev/null || true

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
