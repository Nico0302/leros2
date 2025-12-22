ARG ROS_DISTRO=jazzy

FROM ros:${ROS_DISTRO}-ros-base AS base

ENV ROS_DISTRO=${ROS_DISTRO}

# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
ARG USER_PASSWORD=ros

# Disable dpkg/gdebi interactive dialogs
ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

# Delete existing user if it exists
RUN if getent passwd ${USER_UID}; then \
    userdel -r $(getent passwd ${USER_UID} | cut -d: -f1); \
    fi

# Delete existing group if it exists
RUN if getent group ${USER_GID}; then \
    groupdel $(getent group ${USER_GID} | cut -d: -f1); \
    fi

RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && usermod -aG sudo,dialout,plugdev,video $USERNAME \
    && echo "$USERNAME:$USER_PASSWORD" | chpasswd \
    && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config

# Set up sudo
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# Install essential packages
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    # === Basic utilities === #
    vim \
    wget \
    git \
    unzip \
    iputils-ping \
    net-tools \
    curl \
    ca-certificates \
    # === Build tools === #
    build-essential \
    cmake \
    # === Python tools === #
    pip \
    python3-venv \
    python3-flake8 \
    python3-rosdep \
    python3-setuptools \
    python3-vcstool \
    python3-colcon-common-extensions

FROM base AS workspace

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all

ENV ROS_DISTRO=${ROS_DISTRO}

# Symlink python3 to python
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install Universal Robots ROS packages
RUN apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-ur-client-library \
    ros-${ROS_DISTRO}-ur

USER $USERNAME

# Install Astral UV tool
COPY --from=ghcr.io/astral-sh/uv:latest /uv /uvx /bin/

RUN mkdir -p /home/ros/ros2_ws/src

WORKDIR /home/ros/ros2_ws

# Install dependencies using rosdep
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && rosdep update \
    && rosdep install --ignore-src --from-paths . -y -r

# Build the workspace
COPY ./Makefile ./
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && make build

# Source ROS and workspace setup files in .bashrc
RUN printf "\nsource /opt/ros/$ROS_DISTRO/setup.bash\nsource ~/ros2_ws/install/setup.bash\nsource\n" >> ~/.bashrc