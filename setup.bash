#!/bin/bash

set -e

# Get the directory where this script is located (assumed workspace root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$SCRIPT_DIR"

# Try to source ROS 2 setup.bash for common distros if not already sourced
if ! command -v ros2 &>/dev/null; then
    for d in /opt/ros/*; do
        if [ -f "$d/setup.bash" ]; then
            source "$d/setup.bash"
            break
        fi
    done
fi

# Now check again
if ! command -v ros2 &>/dev/null; then
    echo "ROS 2 is not installed. Please install ROS 2 before running this script."
    exit 1
fi

# Try to detect ROS_DISTRO if not set
if [ -z "$ROS_DISTRO" ]; then
    # Try to detect from installed ROS 2 distros
    if [ -d /opt/ros ]; then
        # Pick the first directory in /opt/ros
        ROS_DISTRO=$(ls /opt/ros | head -n 1)
        echo "ROS_DISTRO not set. Using detected distro: $ROS_DISTRO"
    fi
fi

# Check again if ROS_DISTRO is set
if [ -z "$ROS_DISTRO" ]; then
    echo "ROS_DISTRO is not set and could not be detected. Please set ROS_DISTRO."
    exit 1
fi

# Source the ROS 2 environment
if [ -f /opt/ros/$ROS_DISTRO/setup.bash ]; then
    source /opt/ros/$ROS_DISTRO/setup.bash
else
    echo "ROS 2 environment not found at /opt/ros/$ROS_DISTRO/setup.bash."
    exit 1
fi

# Install Astra camera dependencies
echo "Installing Astra camera dependencies..."
sudo apt update
sudo apt install -y \
    libgflags-dev \
    ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-publisher \
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
    nlohmann-json3-dev

# Clone, build, and install libuvc in the home directory
echo "Cloning and building libuvc in your home directory..."
cd ~ || exit
if [ ! -d "$HOME/libuvc" ]; then
    git clone https://github.com/libuvc/libuvc.git
fi
cd "$HOME/libuvc" || exit
mkdir -p build && cd build
cmake .. && make -j$(nproc)
sudo make install
sudo ldconfig # Refresh the link library

# Bind the udev rules for Astra camera
echo "Binding udev rules for Astra camera..."
cd "$WORKSPACE_ROOT/src/sensors/ros2_astra_camera/astra_camera/scripts" || {
    echo "Could not find astra_camera/scripts directory!"
    exit 1
}

sudo bash install.sh
sudo udevadm control --reload-rules && sudo udevadm trigger

# Build the workspace
echo "Building the workspace..."
cd "$WORKSPACE_ROOT" || exit
colcon build --symlink-install --event-handlers  console_direct+  --cmake-args  -DCMAKE_BUILD_TYPE=Release
source "$WORKSPACE_ROOT/install/setup.bash"

<<<<<<< HEAD
# download models
echo "Downloading models..."

# yolo: https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt
mkdir -p "$WORKSPACE_ROOT/models"
cd "$WORKSPACE_ROOT/models" || exit

# Download YOLO model
echo "Downloading YOLO model..."
wget -P "$WORKSPACE_ROOT/models" https://github.com/CAIC-AD/YOLOPv2/releases/download/V0.0.1/yolopv2.pt


=======
>>>>>>> 6a1fc5fd6ee3b144053ef6a8483c6a42e3cabed8
echo "Setup complete. All dependencies are installed and udev rules are set."
