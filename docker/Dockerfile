
# Dockerfile for PyRoboSim ROS Humble only build
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base AS pyrobosim_ros
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-o", "pipefail", "-c"]


# Install dependencies
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y \
    apt-utils python3-pip python3-tk \
    libegl1 libgl1-mesa-dev libglu1-mesa-dev '^libxcb.*-dev' libx11-xcb-dev \
    libxi-dev libxkbcommon-dev libxkbcommon-x11-dev libxrender-dev mc


# Create a ROS 2 workspace
RUN mkdir -p /pyrobosim_ws/src/pyrobosim
WORKDIR /pyrobosim_ws/src

# Install dependencies
COPY setup/configure_pddlstream.bash /pyrobosim_ws/src/setup/
RUN setup/configure_pddlstream.bash

# Install PyRoboSim, docs, and testing dependencies
ENV PIP_BREAK_SYSTEM_PACKAGES=1
COPY pyrobosim/setup.py pyrobosim/
COPY docs/python_docs_requirements.txt docs/
COPY test/python_test_requirements.txt test/
RUN pip3 install -e ./pyrobosim && \
    pip3 install -r docs/python_docs_requirements.txt && \
    pip3 install -r test/python_test_requirements.txt && \
    pip3 install flask

# Build the ROS workspace, which includes PyRoboSim
COPY . /pyrobosim_ws/src/
WORKDIR /pyrobosim_ws
RUN . /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

# Remove display warnings
RUN mkdir /tmp/runtime-root
ENV XDG_RUNTIME_DIR "/tmp/runtime-root"
RUN chmod -R 0700 /tmp/runtime-root
ENV NO_AT_BRIDGE 1

# Setup an entrypoint and working folder
COPY setup /pyrobosim_ws/src/setup
COPY docker/entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
RUN echo "source /entrypoint.sh" >> ~/.bashrc
