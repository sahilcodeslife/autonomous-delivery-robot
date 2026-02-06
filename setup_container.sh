#!/bin/bash

echo "Setting up ROS2 repository..."

# Add ROS2 GPG key
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update package lists
apt update

echo "Installing ROS2 packages..."

# Install SLAM Toolbox
echo "Installing SLAM Toolbox..."
apt install -y ros-humble-slam-toolbox

# Install ros-gz with force-overwrite to handle conflicts
echo "Installing ros-gz..."
apt install -y -o Dpkg::Options::="--force-overwrite" ros-humble-ros-gz

echo "Installing XACRO and Joint State Publisher GUI..."
apt install -y ros-humble-xacro ros-humble-joint-state-publisher-gui

# Install Joy for gamepad support
echo "Installing Joy for gamepad support..."
apt install -y ros-humble-joy
apt install -y ros-humble-teleop-twist-joy


# Clean up
apt clean
rm -rf /var/lib/apt/lists/*

# Add Tegra environment variables to bashrc for persistence
echo "Setting up Tegra environment variables..."
#echo "export GALLIUM_HUD=1" >> ~/.bashrc
echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra" >> ~/.bashrc

echo "Fixing ROS2 package paths..."
echo "export AMENT_PREFIX_PATH=/opt/ros/humble:\$AMENT_PREFIX_PATH" >> ~/.bashrc

echo "Package installation complete!"
