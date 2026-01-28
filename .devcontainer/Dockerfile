ARG BASE_IMAGE=nvidia/cuda:12.6.0-devel-ubuntu22.04
FROM ${BASE_IMAGE} AS base

# Determine Webots version to be used and set default argument
ARG WEBOTS_VERSION=R2025a

# Install dependencies
RUN apt-get update && \
    apt-get install --yes \
        curl \
        git \
        locales \
        python3-pip \
        sudo \
        unzip \
        wget \
        xvfb \
        && \
    rm -rf /var/lib/apt/lists/*

# Set the locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Set a user name to fix a warning
ARG USERNAME=root
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
&& useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
# Add sudo support.
&& echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
&& chmod 0440 /etc/sudoers.d/$USERNAME

# Disable dpkg/gdebi interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive

# Install Webots
RUN mkdir -p /etc/apt/keyrings && \
    cd /etc/apt/keyrings && \
    wget -q https://cyberbotics.com/Cyberbotics.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" | tee /etc/apt/sources.list.d/Cyberbotics.list && \
    apt update && \
    apt install -y --no-install-recommends webots
ENV QTWEBENGINE_DISABLE_SANDBOX=1
ENV WEBOTS_HOME=/usr/local/webots
ENV PYTHONPATH=${WEBOTS_HOME}/lib/controller/python:$PYTHONPATH

# Copy the configuration file
COPY .devcontainer/Webots-${WEBOTS_VERSION}.conf /home/$USERNAME/.config/Cyberbotics/Webots-${WEBOTS_VERSION}.conf

# Install Python dependencies for VLM controller
# First install torch with CUDA 12 support, then other packages
RUN pip install --no-cache-dir \
    torch==2.4.0 --index-url https://download.pytorch.org/whl/cu124

RUN pip install --no-cache-dir \
    "transformers>=4.40.0,<5.0.0" \
    "Pillow>=9.0.0" \
    "numpy>=1.21.0" \
    "accelerate>=0.32.0" \
    "timm>=0.9.10,<1.0.0" \
    "torchvision>=0.19.0" \
    "sentencepiece>=0.2.0" \
    "protobuf>=4.0.0" \
    "bitsandbytes>=0.43.0"

# Install flash-attn from pre-built wheel (CUDA 12, PyTorch 2.4, Python 3.10)
RUN pip install --no-cache-dir \
    https://github.com/Dao-AILab/flash-attention/releases/download/v2.8.3/flash_attn-2.8.3+cu12torch2.4cxx11abiFALSE-cp310-cp310-linux_x86_64.whl

RUN mkdir -p /ws && chown $USERNAME:$USERNAME /ws

# Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
ENV USER=$USERNAME
WORKDIR /ws
