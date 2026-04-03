#!/bin/bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PACKAGE_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)
UBUNTU_VERSION=$(lsb_release -rs 2>/dev/null || true)
ROS_DISTRO_NAME="${ROS_DISTRO:-}"

if [[ "${ROS_DISTRO_NAME}" == "jazzy" || "${UBUNTU_VERSION}" == "24.04" ]]; then
  cat <<'EOF'
This project currently targets ROS 2 Humble on Ubuntu 22.04 with Gazebo Classic.

Your environment looks like Ubuntu 24.04 / ROS 2 Jazzy. On Jazzy, the Gazebo
Classic packages used by this project are not available anymore, including:
  - gazebo_ros_pkgs
  - gazebo_ros
  - gazebo_ros2_control

That means this script cannot solve the dependency error on Jazzy.

Recommended options:
  1. Use Ubuntu 22.04 + ROS 2 Humble to run this package as-is.
  2. Port the package to the new Gazebo stack (gz_sim / ros_gz /
     gz_ros2_control) before building on Jazzy.
EOF
  exit 1
fi

if [[ -n "${ROS_DISTRO_NAME}" && "${ROS_DISTRO_NAME}" != "humble" ]]; then
  echo "ROS_DISTRO is '${ROS_DISTRO_NAME}', but this script only supports 'humble'." >&2
  exit 1
fi

mkdir -p "${HOME}/.gazebo/models/wpr_simulation2"
cp -r "${PACKAGE_ROOT}/models" "${HOME}/.gazebo/models/wpr_simulation2/models"
cp -r "${PACKAGE_ROOT}/meshes" "${HOME}/.gazebo/models/wpr_simulation2/meshes"
cp -r "${PACKAGE_ROOT}/worlds" "${HOME}/.gazebo/models/wpr_simulation2/worlds"

sudo apt update -y
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-slam-toolbox \
  ros-humble-teleop-twist-keyboard \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-gazebo-ros2-control \
  ros-humble-pcl-ros \
  ros-humble-xacro \
  ros-dev-tools \
  python3-pip \
  python3-colcon-common-extensions \
  python3-argcomplete \
  pcl-tools
