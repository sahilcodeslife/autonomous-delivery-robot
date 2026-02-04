# 1. Start from the base you just created with jetson-containers
FROM dustynv/ros:humble-desktop-l4t-r36.2.0

# 2. Set up your user (matching your host for easy file editing)
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Add sudo and user logic
RUN apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && usermod --append --groups video,dialout,i2c $USERNAME

# 3. Install your specific project needs (SLAM, GZ, etc.)
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-slam-toolbox \
    python3-colcon-common-extensions \
    ros-humble-xacro\
    ros-humble-joint-state-publisher-gui\
    git \
    && rm -rf /var/lib/apt/lists/*

# 4. Set up your workspace
USER $USERNAME
WORKDIR /home/$USERNAME/robot_ws

# ... after step 4 ...
COPY entrypoint.sh /entrypoint.sh
RUN sudo chmod +x /entrypoint.sh

# Use this as the entrypoint
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]