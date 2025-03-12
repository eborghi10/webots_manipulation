ARG BASE_IMAGE=nvidia/cuda:12.6.0-runtime-ubuntu22.04
FROM ${BASE_IMAGE}
SHELL [ "/bin/bash" , "-c" ]

# Determine Webots version to be used and set default argument
ARG WEBOTS_VERSION=R2025a

# Disable dpkg/gdebi interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies
RUN apt-get update && \
    apt-get install --yes curl wget xvfb locales sudo wget && \
    apt-get upgrade --yes && \
    rm -rf /var/lib/apt/lists/*

# Set the locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Install Webots
RUN mkdir -p /etc/apt/keyrings && \
    cd /etc/apt/keyrings && \
    wget -q https://cyberbotics.com/Cyberbotics.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | tee /etc/apt/sources.list.d/Cyberbotics.list && \
    apt update && \
    apt install -y --no-install-recommends webots
ENV QTWEBENGINE_DISABLE_SANDBOX=1

# Copy the configuration file
COPY Webots-${WEBOTS_VERSION}.conf /root/.config/Cyberbotics/Webots-${WEBOTS_VERSION}.conf

# Set a user name to fix a warning
ENV USER root
ENV ROS_DISTRO=humble

# Add ROS 2 keys
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
RUN apt update && \
    apt install -y ros-${ROS_DISTRO}-ros-base ros-dev-tools ros-${ROS_DISTRO}-webots-ros2

# Create overlay workspace
WORKDIR /root/ros_ws/src
COPY packages.repos .

RUN vcs import < packages.repos; \
    cd ..; \
    rosdep install --from-paths src --ignore-src -r -y; \
    source /opt/ros/${ROS_DISTRO}/setup.bash; \
    colcon build;

# Add sourcing ROS setup.bash to .bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
