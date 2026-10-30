# Build environment for the robot's .debs: Ubuntu 24.04 + ROS 2 Jazzy + debian packaging tools.
# Native arm64 in CI; `--platform linux/arm64` works anywhere with qemu (slow). Nothing in here is
# specific to a robot, and no secrets are used or baked in.
FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl gnupg lsb-release git \
 && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/ros2.list \
 && apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-base ros-jazzy-ament-cmake ros-jazzy-ament-lint-auto ros-jazzy-ament-lint-common \
      ros-jazzy-rosidl-default-generators \
      python3-bloom python3-rosdep python3-pytest debhelper dh-python fakeroot build-essential cmake lintian \
      libcurl4-openssl-dev \
 && rm -rf /var/lib/apt/lists/* \
 && rosdep init

WORKDIR /src/robot/packaging
